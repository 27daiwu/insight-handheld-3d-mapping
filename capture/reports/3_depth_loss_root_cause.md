# 3. Depth "12.65 % loss" — root-cause report (2026-11-26, authorised special study)

## Verdict
* **Where it is NOT:** not the host clock, not the receiving ROS stack, not QoS/queue depth/executor, not rosbag2 writing, not host UDP buffers, not the Windows↔WSL network hop. Evidence below, measured, not inferred from RELIABLE.
* **Where it is:** **upstream of the host NIC, i.e. inside the camera (depth pipeline output / camera DDS writer / camera USB egress).** Everything points to the **depth output rate being ≈ 13.1 Hz** rather than a transport loss: it does not change with bandwidth, subscriber count, stereo fps or time-sync.
* **Not determined:** which camera-internal element (depth engine pacing vs camera DDS writer vs USB egress) — I have no access to the camera's internals (ssh credentials unknown, not guessed) so the "SDK/driver output" layer **cannot be observed directly**. Confidence: high for "camera side", low for the exact element.
* Corroboration from another unit/firmware: the vendor-side bring-up log for fw 2.1.2 on Ubuntu 24.04 measured depth ≈ 12.5–12.7 Hz against 15 Hz documented ([psrobotics/insight9 BRINGUP.md](https://raw.githubusercontent.com/psrobotics/insight9/main/BRINGUP.md)). It blames USB bandwidth; our data does not support a bandwidth explanation (below).

## Layer counts (same 120 s window, 5 independent readers on the depth topic)
`depth_loss/depth_experiment.py`, `depth_consumers.py`, `layer_compare.py`; raw in `runs/`.
| run | readers | frames in common window (119 s) | per-reader count | frames missing in ANY reader vs union | missing vs 15 Hz |
|---|---|---|---|---|---|
| R1 depth only | rclpy RELIABLE keep10 · RELIABLE keep1000 · BEST_EFFORT keep10 · RELIABLE keep10 + MultiThreadedExecutor · **rosbag2 file** | 1580 | 1580 each | **0** | **11.76 %** |
| R2 + all other camera topics recorded (≈ +25 MB/s, 7–8 readers) | same 5 | 1554 | 1554 each | **0** | **13.04 %** |
All readers — including the BEST_EFFORT one, which has no retransmission and would show any transport loss — received **exactly the same frames**, and the rosbag2 written file equals the live receive count. Hence **receive count = rosbag write count**, and no reader-side mechanism (QoS, queue depth 10 vs 1000, executor) loses frames. Loss common to all readers happens before them.

## Host and link counters (before/after each run)
| | R1 | R2 |
|---|---|---|
| WSL NCM iface (`eth0`) rx_packets | +4 134 742 | +6 414 989 |
| Windows NCM adapter ReceivedUnicastPackets | +4 134 736 | +6 414 979 |
| rx_dropped / rx_errors / fifo / over (WSL), discards/errors (Windows) | 0 / 0 / 0 / 0 ; 0 / 0 | same |
| UDP `InErrors = RcvbufErrors` | 27 of 434 795 datagrams | 56 of 733 744 |
(Windows and WSL agree to 10 packets ⇒ nothing is lost between the USB adapter and WSL. 27–56 socket-buffer drops per run are 0.006–0.008 % of datagrams, would hit readers differently, but all readers are identical; they cannot explain ~190 missing frames.) `net.core.rmem_max` is 212992 (read only, unchanged).
Fan-out note: DDS unicasts to each reader, so R1/R2 pushed ≈ 48/74 MB/s through the USB link (≈ 5–7× the 10 MB/s stream) and the depth rate hardly moved (13.24 → 13.05 Hz) ⇒ **not bandwidth-limited**.

## Timestamp structure (340 s bag, `offline_bag_analysis.json`)
* Depth stamps equal left-stereo stamps in 4472 of 4473 frames ⇒ **depth is produced per stereo frame**; stereo itself lost 0 of 10 243 frames.
* Interval between consecutive depth frames in stereo-frame units: 1 → 117 (2.6 %), **2 → 3049 (68 %)**, 3 → 1197 (27 %), 4 → 108 (2.4 %) ⇒ mean 2.29 frames = **13.1 Hz** (15 Hz would be exactly 2). No interval above 4 in 340 s.
* Time distribution: 17–32 missing frames in **every** 10 s bin (no bursts, no 34 s periodicity) ⇒ a steady throughput limit, not an intermittent stall.
* R1/R2 show the same step distribution (1: 1 %, 2: 72 %, 3: 26 %, 4: 1 %).

## Parameters (reports/2_camera_param_experiment_report.md)
stereo 30→20 fps: depth 13.04 → 12.77 Hz (no improvement); time-sync on: 13.10 Hz; depth off: camera CPU 44 % → 21 % and residual gaps in IMU/VIO/stereo disappear (so depth costs camera resources; **no core was seen saturated** in `/api/cpu-monitor`, max core ≈ 57 %, but that sampling is coarse).
Camera writer QoS (Cyclone discovery, `camera_writer_qos.txt`): depth RELIABLE, KEEP_LAST **10** (stereo/colour 5, IMU BEST_EFFORT 1). A writer-queue overflow would depend on reader speed/bandwidth — it did not.

## Hypotheses
| hypothesis | status | evidence |
|---|---|---|
| WSL wall-clock steps create it | **REFUTED** | analysis in header-stamp space; clock had 0 steps during R1/R2 |
| receiver QoS (RELIABLE/BEST_EFFORT), queue depth, executor | **REFUTED** | 5 readers identical |
| rosbag2 write loss | **REFUTED** | bag set = live set |
| host UDP buffer overflow | **REFUTED** as cause | 27–56 drops, readers identical, BE = RELIABLE |
| Windows/WSL NCM hop loses packets | **REFUTED** | counters equal, zero discards |
| USB/link bandwidth saturation | **REFUTED** (for the host side of the link) | rate unchanged with +50 % traffic and stereo −33 % |
| camera depth engine output limit (≈13 Hz, pacing on stereo frames) | **LIKELY (not directly observable)** | constant steady shortfall, per-stereo-frame structure, independent of everything else; same shortfall reported on fw 2.1.2 |
| camera DDS writer/USB egress inside the camera | **POSSIBLE, not separable** | no camera-internal access |
| "15 Hz is the true spec for 2.1.5" | **UNKNOWN** | vendor doc says 15 Hz |

## Consequences
* The 12.65 % is a **device-output shortfall, not recorder loss**; the recorder faithfully stores what arrives. It stays an **open deviation** (not fixed) until the vendor confirms the intended rate/behaviour or a firmware/config remedy is found. Not counted as a clock artefact and not counted as a recorder bug.
* Users of depth must handle irregular intervals (2–4 stereo frames) and ≈ 13.1 Hz.
* Next discriminators (need approval/access): ask vendor; second unit; native Ubuntu (rules out WSL2 only for the host side, already refuted); camera-side logs; try `compatibility-mode`/other colour format (changes camera config).
