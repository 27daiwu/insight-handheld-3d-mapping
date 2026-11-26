# 1. Camera access report (workstream A) — 2026-11-26

**Status: CONNECTED, CORE TOPICS MEASURED ON REAL DEVICE (WSL2, not native Ubuntu). Acceptance NOT passed:** a real 5-min recording completed and replays, but shows periodic multi-topic receive stalls (see §4) and Depth loss. Everything in this report is real-device data except where marked.

Timeline: at first the camera had no DDS endpoints on the network (ICMP-refused DDS ports; the camera had just booted/was possibly in another mode). After the user re-connected it, DDS appeared on **domain 9**; the user's `ros2 topic list` on the default domain 0 showed only other robots (odin1, unitree) — the camera is on domain 9.

## What was verified on the real device (read-only; no setting was changed, no POST/PUT sent)
| Item | Result | Evidence |
|---|---|---|
| Windows sees camera | YES. VID 3652 PID 0B5C "Deep mirror camera"; interfaces: 2x UVC Camera (MI_00, MI_03), 2x HID, 1x **UsbNcm Host Device #2 (MI_07)**, Sensor HID | `Get-PnpDevice` |
| usbipd | listed as BUSID 1-2, state "Not shared" — **not attached to WSL and not needed**: the camera is a USB-Ethernet (CDC-NCM) device | `usbipd list` |
| Link to WSL | Windows NCM adapter 169.254.10.2/24 is mirrored into WSL as `eth1` (MAC 02:00:00:00:0a:01), camera 169.254.10.1 (MAC 02:00:00:00:0a:02) | `ip addr/neigh` |
| IP reachability | ping 169.254.10.1: 3/3, ~1.1 ms. `looper.local` resolves to 169.254.10.1 | ping, getent |
| Camera web API (HTTP :80, TCP :22 also open) | reachable (must use `curl --noproxy '*'`: shell env has an HTTP proxy that returns 502) | `reports/camera_data/*.json` |
| **Firmware / software version** | **2.1.5 / 2.1.5** (not 2.1.4). Product `insight9` | `/api/ota/device-versions` |
| ROS Domain ID configured on camera | **9** (not 0) | `/api/ros-domain-id` |
| DDS implementation configured on camera | **cyclonedds** | `/api/dds-type` |
| ROS node (camera side) | `insight_full`, namespace `camera`, name `camera` | `/api/camera-config` |
| Colour stream config | mjpg 1088x1920 30fps (options: mjpg 1920x1080/1280x720/640x480, nv12 720x1200@20, 544x960@20/30) | `/api/camera-format(-options)` |
| Stereo stream config | mono8 544x640 30fps, HDR on (sensor sc132gs; 20/30/40/50 fps options) | `/api/stereo-format`, `/api/stereo-config` |
| **Depth** | `deep-flow enabled=true, perception_engine=depth` → **depth is reported ENABLED on this unit** (contradicts "default off"); not changed | `/api/deep-flow` |
| VIO pose covariance | `publish_pose_cov=true` | `/api/sensor-pose-cov` |
| Camera clock | camera `system-time` = **2025-08-26 23:26** vs host 2026-11-26 → **~457 days behind**; `time-sync enabled=false, synced=false`. Header stamps will not be wall-clock until time sync is configured | `/api/system-time`, `/api/time-sync-setting` |
| Compat/calibration mode | compatibilityMode=false, calibrationMode=false | `/api/compatibility-mode`, `/api/mode` |
| Camera uptime | 3 min → 15 min while I worked (booted, stable, temp 40→52 °C) | `/api/system-info` |

## 1. ROS discovery (real)
`ROS_DOMAIN_ID=9`, Fast DDS (Humble default; camera is configured for cyclonedds yet interoperates). RTPS packets from 169.254.10.1 seen. 18 topics: see `camera_data/topics_domain9.txt`. Node `insight_full`.
Publisher QoS: imu BEST_EFFORT; images/depth/VIO RELIABLE; `/tf_static` RELIABLE+TRANSIENT_LOCAL.

## 2. Streams, 60 s passive probe + 320 s recording (`camera_data/probe_60s_run1.json`, `real5min_inspect.json`)
| role | topic | type | nominal | measured (header stamp) | notes |
|---|---|---|---|---|---|
| RGB | `color/image_rect_raw/compressed` | CompressedImage jpeg 1088x1920 | 30 | 30.019 Hz, 0 stamp gaps in 5 min | `color/image_raw/compressed`: publisher exists, **0 msgs** |
| Stereo L/R | `infra1|2/image_rect_raw` | Image mono8 544x640 | 30 | 30.000 Hz, 0 gaps; L/R stamps identical (skew 0 ms, 100 % matched) | `infra*/image_raw`: **0 msgs** |
| Depth | `depth/image_rect_raw` | Image mono16 544x640 | 15 | header 15.0 Hz but **12.65 % frames missing** vs 15 Hz (816 stamp gaps, intervals 66/133 ms; rx 13.1 Hz) | enabled on this unit; USB bandwidth plausible (vendor note), unproven |
| IMU | `imu` | Imu | 400 | 400.0 Hz; 167 est. missing (0.12 %), max gap 32.5 ms | |
| VIO | `vio_100hz` | PoseStamped, frame `world` | 100 (doc: 15) | 100.0 Hz; 98 missing (0.29 %); plus `vio_image` 30 Hz PoseStamped, `vio_image_cov` 30 Hz, `vio_status` 1 Hz String | |
| CameraInfo | `color|infra1|infra2/camera_info` | CameraInfo | 30 | 30 Hz; K calibrated (rgb fx≈769, 1088x1920); size matches image; frame_ids match | 2 publishers each |
| TF | `/tf_static` only | TFMessage | – | 5 edges, rooted at `camera_camera_left`; **no `/tf`**; VIO frame `world` not connected | |
Data volume ≈ 35 MB/s (12 GB / 340 s).

## 3. Timestamps
Header stamps are **seconds since camera boot** (probe: 1339 s) → not wall clock; camera wall clock 457 days behind host; time-sync disabled. Monotonic, no duplicates, no zero stamps on all streams (`vio_status` is a 1 Hz string with duplicates by design). Cross-sensor: stereo L/R identical stamps; RGB vs stereo max 16.7 ms (= half frame, 99.99 % matched within 20 ms). Host–camera alignment impossible until time-sync is configured (not changed).

## 4. Continuity: periodic multi-topic receive stalls
`inspect` (fixed diagnostics): **9 simultaneous holes on all 12 topics, every ~34.9 s (33.8–35.0 s), 3.1–3.3 s each** (t = 3.1, 37.2, 72.1, 106.1, 141.1, 174.9, 225.3 (4.0 s), 260.1, 295.1 s). Not bag-file rotation (files rotate at 61 s). During a hole IMU header stamps stay contiguous (stamp gap 2.5 ms) while receive latency jumps from −54 ms to +2155 ms ⇒ the host receive timestamps jumped (data itself was not delayed or lost) — for IMU/stereo/RGB essentially nothing is lost; Depth (RELIABLE) loses ~12.6 % at the source/USB, IMU/VIO lose 0.1–0.3 %. The same period appears with the simulator, so the camera is not the cause. **Root cause found (report 2): host wall-clock steps (+2.4 s every 32 s, systemd-timesyncd in WSL2)** — header stamps show 0 messages lost in the holes. Only Depth 12.65 %, IMU 0.12 %, VIO 0.29 % are real (camera-stamp-space) losses.
Replay of the real bag (isolated domain 112, 4x, offset 10 s): stamp rates reproduced exactly (imu 400.0, stereo 30.0, vio 100.0, depth 15.0 Hz), exit 0.

## 5. USB / network
usbipd not needed (CDC-NCM). Windows lists VID 3652 PID 0B5C, 2 UVC cams + HID + NCM. Link eth1 169.254.10.2↔.1, ping 1.0 ms, 0 % loss. Camera web API fw 2.1.5, domain 9, dds cyclonedds, uptime/temperature 40→54 °C within 20 min. Raw data: `camera_data/*.json`.
**No read of USB descriptors/bandwidth (needs root/Windows tools) and no native Ubuntu run were possible.**

## 6. Open items
1. Native-Ubuntu repeat (needed for acceptance and to separate WSL2 from host-independent behaviour).
2. Depth loss: compare with depth off (requires changing camera config → needs your approval) or lower stereo fps.
3. Time sync (NTP/`/api/time-sync-setting`) – needs approval.
4. `image_raw` topics empty; confirm whether they publish only on subscription or are unused.
5. Firmware 2.1.5 vs baseline 2.1.4 – decide baseline.
6. Depth was found enabled although specified as default-off: recorded (publisher present), never toggled by tooling.
