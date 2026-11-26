# 2. WSL2 "periodic data hole" root cause report (workstream B) — 2026-11-26

## Summary
* **The ~34 s multi-topic "holes" are NOT packet loss.** They are steps of the **host wall clock (CLOCK_REALTIME)** inside the WSL2 VM. rosbag2 stamps messages with that clock, and my first tooling measured gaps with it (or with sim header stamps taken from it), so every step shows up as a ~2–3 s hole in all topics at once.
* Evidence chain (all measured): (1) no-DDS control — 0 scheduler stalls, 0 of 180 202 UDP loopback packets lost, but 5 wall-clock steps of +1.2…+2.5 s; (2) every step coincides with a `systemd-timesyncd` poll (4/4), step size = timesyncd's reported offset (+2.47 s vs 2473 ms …); (3) on the **real camera bag**, at all 9 holes every topic's *header stamp* advanced by exactly one nominal period (IMU 2.5 ms, stereo 33.3 ms, …) → **0 messages lost in the holes**; (4) simulator re-run with monotonic header stamps: 0 stamp gaps on all image/VIO topics but the same receive-time holes (8, every 34.8 s).
* **Correction of my earlier report:** "sim published 6419 IMU, subscriber received 6002 ⇒ loss in transport" was wrong — the 6419 included ~4 s of start-up/shut-down outside the 60 s probe window (6002 ≈ 60 s × 100 Hz). The 322 s simulator "acceptance failure" was largely this clock artefact. The old `TEST_REPORT.md` conclusion is withdrawn.
* **Not determined:** *why* the VM clock is wrong by ~2.4 s every 32 s (see Root-cause status). No system setting was changed; no native Ubuntu host was available.

## Method (read-only; nothing changed)
Scripts and raw data in `capture/reports/wsl2_rca_data/`: `control_nodds.py` (loop-stall logger + UDP loopback 1 kHz seq test in separate processes + REALTIME−MONOTONIC step logger), `clock_vs_timesyncd.py` (steps vs timesyncd `Packet count`), `linux_minus_windows_ms.txt` (WSL clock vs Windows `UtcNow`), `sim_monostamp_*.json`. Real-camera data: `../camera_data/real5min_*.json`.
Host: WSL2 Ubuntu 22.04, kernel 6.6.87.2, mirrored networking, systemd on, clocksource `hyperv_clocksource_tsc_page`/tsc, `systemd-timesyncd` active (poll 32 s), `/dev/ptp_hyperv` present (no read permission).

## Results
| # | experiment | duration | result |
|---|---|---|---|
| E1 | busy-loop stall logger (no DDS) | 180 s | **0** stalls > 100 ms |
| E2 | UDP loopback 1 kHz, tx/rx separate processes | 180 s | 180 202 received, **0 lost**, 0 gaps > 100 ms, latency p50 0.13 ms / max 2.4 ms |
| E3 | REALTIME vs MONOTONIC steps | 180 s | **5 steps**: +2.03, +1.22, +2.18, +2.47, +2.48 s at t = 16.4, 63.8, 96.1, 128.6, 160.8 s (intervals 47.5, 32.3, 32.5, 32.3 s) |
| E4 | steps vs timesyncd polls | 130 s | steps at 13.16/45.42/77.73/110.15 s (+2.47/+2.49/+2.40/+2.44 s); timesyncd packet count +1 at 13.51/45.5/78.01/110.51 s, reported offset +2.47/+2.49/+2.40/+2.44 s → **4/4 coincide, interval 32.3 s = poll interval** |
| E5 | WSL clock − Windows UtcNow | 24 samples | +2053 ms then falls linearly ≈ −82 ms/s (2020 ms in 24.8 s) to ≈0, then (E3/E4) steps +2.4 s again → sawtooth, no net drift (≈±0.7 s bias, measurement latency ±0.3 s) |
| E6 | REALTIME−MONOTONIC drift between steps | 70 s | 0.00 ms/s (both advance together); `adjtimex` read-only: tick=9167, freq −33.5 ppm. ⇒ between steps the VM's *whole* time base runs ≈8 % slower than Windows' clock; only REALTIME is stepped back up |
| E7 | **real camera bag**, 340 s, 12 topics | – | 9 receive-time holes (t = 3.1, 37.2, 72.1, 106.1, 141.1, 174.9, 225.3, 260.1, 295.1 s; 3.1–4.0 s; median period 34.9 s) on all 12 topics; **header-stamp step at the hole = 1 nominal period on every topic** ⇒ 0 lost; IMU recv−stamp latency jumps −54 ms → +2155 ms across the hole |
| E8 | simulator, **monotonic** header stamps, 310 s | 310 s | stamp gaps 0 on rgb/stereo/vio/info; 8 receive-time holes, period 34.8 s, all classified "recv-only"; (IMU shows 2.4 % stamp gaps = simulator-side python timer overruns, not host) |

## Hypotheses
| hypothesis | status | evidence |
|---|---|---|
| Publisher process stalls | **REFUTED** | E1; real camera is a separate device |
| Transport / UDP loopback loses packets | **REFUTED** (loopback) | E2: 0/180 202 lost, max latency 2.4 ms |
| Fast DDS transport/QoS/queue loses data at the holes | **REFUTED for the real bag** (data present, header stamps contiguous) | E7; DDS arms (QoS/depth/executor/SHM-vs-UDP) were **NOT TESTED** because the symptom they target does not exist |
| Cyclone DDS comparison | **NOT TESTED** (`rmw_cyclonedds_cpp` not installed; install needs sudo) | not needed for this finding |
| rosbag2/recorder stall | **REFUTED as sole cause** | E8 holes appear on the independent probe and on header-stamp-free analysis |
| **Host wall-clock steps (timesyncd) create the holes** | **CONFIRMED** (mechanism + correlation) | E3, E4, E7, E8 |
| timesyncd is the *only* actor / why the clock is 8 % slow between steps | **NOT DETERMINED** | tick=9167 is unexplained; Hyper-V TimeSync (`hv_utils`) or WSL host sync are candidates, not tested |
| Pre-reboot sessions (14:36 real recording) had the same mechanism | **LIKELY, not proven** | same signature (period 33.8–35 s vs 32.3 s now; steps were not logged back then) |
| Native Ubuntu behaves differently | **NOT TESTED** (no native host) | `run_native_control.sh` provided |

## Consequences for the capture tool (done, in scope)
* `inspect` now classifies a multi-topic receive-time hole whose header stamps are contiguous as **"data NOT lost, host receive timestamps jumped"** (WARN) and keeps **FAIL** for holes that also have stamp gaps. Real bag → WARN, 0 data-loss events.
* `preflight` samples REALTIME−MONOTONIC (`--clock-seconds`, default 35 s) and warns about steps; it just warned on the real camera (2 steps in 36 s).
* Rule for data users: **use header stamps for continuity and sync; treat bag receive time on this host as unreliable (±2.5 s sawtooth).** Camera clock is boot-relative, so cross-device sync additionally needs camera time-sync (not configured).

## Remaining real data problems (NOT explained by the clock)
Depth **12.65 %** of frames missing vs 15 Hz (header stamps; RELIABLE QoS ⇒ lost before DDS: camera/USB), IMU 0.12 %, VIO 0.29 % (stamp gaps ≤ 32 ms). Not caused by the clock because they exist in camera-stamp space. Needs native-Ubuntu run and a depth-off/lower-fps comparison (camera setting → your approval).

## Proposed changes needing your approval (not executed)
1. **Diagnostic first:** `sudo systemctl stop systemd-timesyncd` for 5 min while re-running `clock_vs_timesyncd.py`/E5. Expected: steps stop; *risk:* clock then simply runs ~8 % slow (continuous distortion) → that would identify the slow tick as a separate cause. Revert with `start`.
2. If confirmed, durable options: disable timesyncd in WSL and rely on Windows host sync; or chrony in slew-only mode (`makestep 0 -1`); investigate who sets tick=9167 (`.wslconfig`/Hyper-V time sync); `wsl --shutdown` resync.
3. Install `ros-humble-rmw-cyclonedds-cpp` only if a DDS comparison is still wanted after the above (not currently needed).
4. Native Ubuntu 22.04 run of `run_native_control.sh` + a camera recording (the real acceptance run).
