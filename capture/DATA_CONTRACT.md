# DATA_CONTRACT — Insight Handheld 3D Mapping (DRAFT v0.2, 2026-11-26)

**Status: DRAFT for review. Not an approved project contract.** Replaces the capture-side provisional v0.1.
Written from REAL data of one device (Insight 9, firmware/software **2.1.5**, ROS domain 9, observed from WSL2 / ROS 2 Humble / Fast DDS).
No native-Ubuntu run exists yet. Location: `capture/DATA_CONTRACT.md` (only `capture/` may be edited here); move to the repo root once approved.

Legend used in every table:
* **[V]** verified: observed on fw 2.1.5 in ≥ 2 independent runs and consistent with other evidence.
* **[O]** observed once / one source; plausible but not cross-checked.
* **[?]** PENDING — not verified. Do **not** build on it; ask the vendor / run the stated check.

---
## 1. Device baseline
| item | value | status |
|---|---|---|
| model / firmware / software | Insight 9, **2.1.5 / 2.1.5** (read from `GET /api/ota/device-versions`, auto-stored in manifest) | [V] |
| baseline history | v2.1.4 profile kept as `config/topics_v2.1.4.yaml` (never verified); v2.1.5 is the project baseline | – |
| ROS domain / DDS | domain **9**; device configured `cyclonedds`; Fast DDS (Humble default) interoperates | [V] |
| ROS node / namespace | `insight_full`, prefix `/camera/camera` (+ `/tf_static` global) | [V] |
| link | USB CDC-NCM, 169.254.10.1 (camera) ↔ 169.254.10.2 (host); no usbipd needed | [V] |
| Depth default | **enabled** on the test unit (contradicts "default off"); tool never toggles it | [O] |

## 2. Session directory (written by `insight-capture record`)
`$INSIGHT_DATA_ROOT/<YYYYmmdd_HHMMSS>_<name>/`: `bag/` (rosbag2 sqlite3, CDR, split at 2 GiB, `metadata.yaml`), `manifest.json`, `plan.json`, `preflight.json`, `session.json` (live state/heartbeat), `diagnostics.json` (after `inspect`), `recorder.log`. A session directory is never reused; `ACTIVE` in the root marks the running session.

## 3. manifest.json (schema_version `0.2-draft`)
`schema_version, session_id, status(recording|complete|stopped_abnormal|recovered|finalize_failed), stop_reason, tool{name,version,git}, host{os,kernel,hostname,python,wsl,ros{…}}, camera{model, firmware, firmware_baseline, profile, profile_verified, prefix, api_snapshot{firmware, software, product, color_format, stereo_format, stereo_hdr, depth_enabled, time_sync_enabled, time_synced, ros_domain_id, dds_type, camera_wall_clock_epoch, temperature_c, uptime}}, time{start_utc,start_wall_ns,end_utc,first_msg_recv_ns,last_msg_recv_ns,duration_s}, depth{enabled,note}, topics[{role,name,type,required,reference_hz,frame_id,count,recv_first/last_ns,stamp_first/last_ns,measured_*_rate_hz,est_missing_msgs,bytes}], bag{…}, disk{…}, preflight{…}, events[]`.
`firmware` is read from the camera's web API (read-only GET) and falls back to `--firmware`. Status [V] for the field set; the field **semantics of `camera.api_snapshot` depend on the vendor API staying stable** [?].

## 4. Time — the most important section
| quantity | definition | status |
|---|---|---|
| **header.stamp** (all device topics) | **camera clock, seconds since camera boot** (e.g. 1339 s, 1719 s …), monotonic, no wall-clock meaning. Streams share one clock: L/R stereo stamps are bit-identical; RGB/IMU/VIO/depth are on the same time base | [V] |
| camera wall clock | `GET /api/system-time` = **2025-08-26** (≈ 457 days behind host); not used by header stamps | [V] |
| camera time-sync toggle | `POST /api/time-sync-setting {enabled:true}` accepted, but over ≥ 110 s `synced` stayed false and stamps/wall clock did not change ⇒ **no observable effect**; sync source/protocol unknown | [V] (no effect) / [?] (how it is meant to work) |
| bag message time (`messages.timestamp`) | host **wall clock (CLOCK_REALTIME) at reception** | [V] |
| host clock reliability | **WSL2: unreliable at times.** 16:07–16:40 the VM wall clock stepped ≈ +2.4 s every 32 s (systemd-timesyncd polls), producing fake multi-topic holes in bag receive time; at 18:08–18:50 (same VM, same camera) no step in > 16 min, rate within ±60 ppm of Windows. Preflight now samples this (`--clock-seconds`) | [V] observed both states / [?] trigger of the bad state |
| **RULE 1** | **Use header.stamp for continuity, loss and sensor synchronisation. Never compute loss/gaps from bag receive time** | – |
| **RULE 2** | Host↔camera alignment is **not available** until camera time-sync works. Offsets between camera stamps and host time must not be assumed constant across camera reboots (boot-relative) | – |
| **RULE 3** | `inspect` classifies multi-topic holes seen only in receive time (header stamps contiguous) as "host clock", never as data loss | – |

## 5. Topics (fw 2.1.5, prefix `/camera/camera`)
QoS = camera publisher QoS read via Cyclone discovery (reports/depth_loss/camera_writer_qos.txt).
| role | topic | type | rate (header) | QoS pub | frame_id | status |
|---|---|---|---|---|---|---|
| rgb | `color/image_rect_raw/compressed` | CompressedImage, jpeg, 1088×1920 (portrait) | 30.0 Hz | RELIABLE, keep 5 | camera_camera_rgb | [V] |
| stereo_left | `infra1/image_rect_raw` | Image mono8 544×640 | 30.0 Hz | RELIABLE, keep 5 | camera_camera_left | [V] |
| stereo_right | `infra2/image_rect_raw` | Image mono8 544×640 | 30.0 Hz | RELIABLE, keep 5 | camera_camera_right | [V] |
| depth | `depth/image_rect_raw` | Image mono16 544×640 | **≈13.1 Hz achieved (15 nominal)** | RELIABLE, keep 10 | camera_camera_depth | **[?] see §8** |
| imu | `imu` | Imu | 400.0 Hz | **BEST_EFFORT, keep 1** | camera_camera_imu_optical | [V] rate / [?] axes |
| vio | `vio_100hz` | PoseStamped | 100.0 Hz, 0.15 % pooled loss | RELIABLE, keep 10 | world | [?] |
| camera_info ×3 | `color|infra1|infra2/camera_info` | CameraInfo | 30 Hz | RELIABLE, keep 5 | as image | [V] |
| depth_info | – | – | **topic does not exist** | – | – | [?] |
| tf_static | `/tf_static` | TFMessage | latched | RELIABLE, TRANSIENT_LOCAL | – | [O] |
| tf | `/tf` | – | **never published** | – | – | [V] absent |
| extras | `vio_image` (30 Hz, follows image rate), `vio_image_cov` (30 Hz PoseWithCovarianceStamped), `vio_status` (String 1 Hz, `TRACKING_STATIC` at rest) | | | | | [O] |
| empty topics | `infra1|2/image_raw`, `color/image_raw/compressed` have a publisher but **0 messages** | | | | | [O] |
Vendor guide v1.2.1 lists `/insight/vio_pose` 15 Hz and RealSense-style names; **`/insight/vio_pose` does not exist on 2.1.5** and VIO is 100 Hz [V].

## 6. Message semantics
| item | content | status |
|---|---|---|
| image encodings | stereo `mono8` step=544; depth `mono16` little-endian step=1088; colour JPEG (SOI ffd8) | [V] |
| CameraInfo (stereo) | 544×640, K=[307.385,0,269.424; 0,307.385,323.476; 0,0,1], plumb_bob with D=0 ⇒ **rectified images**; right camera P[3]=−30.776 ⇒ baseline = 0.10012 m | [V] (numbers) / [?] "rectified" assumption |
| CameraInfo (colour) | 1088×1920, fx=768.863 fy=768.891 cx=539.467 cy=959.494, D=0 | [V] numbers |
| baseline | 0.10012 m: CameraInfo P[3]/fx and tf_static agree | [V] |
| depth values | mono16, median 542 over valid pixels, 0.4 % zeros, max 65535 present. **Unit (mm assumed from vendor doc) and invalid-value convention unconfirmed** | **[?]** |
| depth ↔ stereo | depth stamps equal left-stereo stamps (4472 of 4473) and depth frame is identity from `camera_camera_left` ⇒ depth is in the **left camera optical frame** | [V] stamps / [?] meaning |
| IMU units | |a| = 9.78 m/s² at rest ⇒ accel in **m/s²**; gyro presumably rad/s (mean ≈ 0.002 at rest) | [O] accel / [?] gyro |
| IMU frame/axes | frame `camera_camera_imu_optical` is **identity** from left in tf_static (yet an `imu → left` transform also exists); gravity vector at rest ≈ (−4.3, −8.8, 0.28) | **[?]** axes/sign |
| IMU covariance/orientation | all covariances 0 (= unknown), orientation identity (not estimated) | [V] |
| VIO pose | PoseStamped in frame `world`; first pose ≈ origin with non-identity quaternion (−0.2255, 0.6593, −0.5075, 0.5069); vendor doc says "T_world_imu". `vio_image_cov` diag σ² ≈ 3e-6 (pos) | **[?]** semantics, scale, gravity alignment |
| TF | `/tf_static`: left→right (0.10012,0,0); left→rgb (0.0501,≈0,≈0); imu→left (−0.03898,−0.01482,0.0189); left→depth and left→imu_optical identity. `world` is not connected | [O] values / [?] correctness |
| coordinate convention | ROS REP-103 / optical frame (x right, y down, z forward) for `*_optical` is **assumed, not verified** | **[?]** |

## 7. Quality metrics & thresholds (computed by `insight-capture inspect`)
* loss = missing messages from **header-stamp gaps** (gap > 1.5 × period) relative to the configured/nominal rate; tolerance `max_loss_ratio = 0.1 %` for [V] status.
* systemic hole = ≥ 3 topics with a ≥ 0.5 s hole in receive time; **recv-only** if header stamps advance by ≤ 2 periods across it.
* sync skew: nearest-stamp matching (L/R: 0 ms measured; RGB↔stereo ≤ 16.7 ms).
* Real-data results are in `reports/P1_acceptance_report.md`.

## 8. Known deviations from "nominal" (do not hide)
1. **Depth delivers ≈ 13.1 Hz instead of 15 Hz (11.8–13.1 % missing in every run)**; located camera-side (reports/3_depth_loss_root_cause.md). Consumers must treat depth as irregular (intervals of 2, 3 or 4 left-stereo frames).
2. With depth enabled IMU/VIO/stereo show sporadic extra gaps (0.02–0.3 %); with depth disabled none in 90 s.
3. RGB exists only as JPEG; there is no raw colour topic.
4. VIO frame `world` is not linked to the camera TF tree.

## 9. Open questions (need vendor / experiment)
| # | question | how to close |
|---|---|---|
| 1 | Is depth's intended rate 15 Hz? Is ≈13 Hz a known limit of fw 2.1.5? | vendor; reproduce on a second unit/native host |
| 2 | Depth units and invalid value (0? 65535?) | vendor doc; measure a known target distance |
| 3 | IMU axes, gravity sign, gyro units; why imu_optical is identity | vendor doc; rotate camera and check |
| 4 | VIO pose definition (T_world_imu vs camera), initial orientation, behaviour in motion/loss of tracking | vendor doc; moving test |
| 5 | How does camera time-sync work (NTP/PTP, who is the server)? | vendor doc; then retest with approval |
| 6 | Why was the WSL2 wall clock stepping at 16:07–16:40 but not at 18:08–18:50? | native-Ubuntu control (`run_native_control.sh`), Windows time service review |
| 7 | Same results on native Ubuntu 22.04? | run when a machine is available |
| 8 | `image_raw` topics: unused or published on demand? | vendor |
