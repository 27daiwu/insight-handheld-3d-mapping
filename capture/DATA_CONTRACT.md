# DATA_CONTRACT — Insight Handheld 3D Mapping (DRAFT v0.3, 2026-11-27)

**Status: DRAFT for review. Not an approved project contract.** Supersedes v0.2 (2026-11-26). Same location rule: `capture/DATA_CONTRACT.md`; move to the repo root once approved.
This version puts the **official vendor specification, the fw 2.1.5 measurements, the verification status and the data sources in one document**.
Machine-readable twin: `config/topics_v2.1.5.yaml` → `spec:` (the tables in §5 and §8 are *generated* from it and checked by `tests/test_contract_consistency.py`).

Manifest `schema_version` is still **`0.2-draft`**: this revision changes the documentation of the data, not the manifest fields (§3).

## 0. How to read this document
Status marks (kept from v0.2, unchanged meaning):
* **[V]** verified: observed on fw 2.1.5 in ≥ 2 independent runs and consistent with other evidence.
* **[O]** observed once / one source; plausible but not cross-checked.
* **[?]** PENDING — not verified. Do **not** build on it; ask the vendor / run the stated check.
* **[-]** not applicable.

Source codes (full list with file paths in §5.6): **D0** = vendor guide v1.2.0, **D1** = vendor guide v1.2.1, **M1/M2** = our 5-minute recordings A/B, **TL** = real topic list, **MF** = decoded message fields, **JPEG** = JPEG header decode, **API** = camera web API (read-only GET), **QOS** = camera writer QoS.

**Promotion rule (enforced by a test):** a statement that rests only on vendor documentation (D0/D1) can never be [V] or [O]. Agreement between a guide and a measurement does not raise the guide; the status comes from the measurement and is stated per field.

**Two guide versions exist and they disagree** (v1.2.0 and v1.2.1, both on the vendor site; archived with SHA-256 in `reports/vendor_docs/`). v1.2.0 uses the namespace `/insight/*`; v1.2.1 uses RealSense-style `/camera/camera/*` names. fw 2.1.5 publishes the v1.2.1 style. Where both guides give a different value it is shown for both (§5.3).

## 1. Device baseline
| item | value | status |
|---|---|---|
| model / firmware / software | Insight 9, **2.1.5 / 2.1.5** (read from `GET /api/ota/device-versions`, auto-stored in manifest) | [V] |
| baseline history | v2.1.4 profile kept as `config/topics_v2.1.4.yaml` (never verified); v2.1.5 is the project baseline | – |
| ROS domain / DDS | domain **9**; device configured `cyclonedds`; Fast DDS (Humble default) interoperates | [V] |
| ROS node / namespace | `insight_full`, prefix `/camera/camera` (+ `/tf_static` global) | [V] |
| link | USB CDC-NCM, 169.254.10.1 (camera) ↔ 169.254.10.2 (host); no usbipd needed | [V] |
| Depth default | **enabled** on the tested unit (`/api/deep-flow enabled=true`); the tool never toggles it | [O] |

## 2. Session directory (written by `insight-capture record`)
`$INSIGHT_DATA_ROOT/<YYYYmmdd_HHMMSS>_<name>/`: `bag/` (rosbag2 sqlite3, CDR, split at 2 GiB, `metadata.yaml`), `manifest.json`, `plan.json`, `preflight.json`, `session.json` (live state/heartbeat), `diagnostics.json` (after `inspect`), `recorder.log`. A session directory is never reused; `ACTIVE` in the root marks the running session.

## 3. manifest.json (schema_version `0.2-draft`)
`schema_version, session_id, status(recording|complete|stopped_abnormal|recovered|finalize_failed), stop_reason, tool{name,version,git}, host{os,kernel,hostname,python,wsl,ros{…}}, camera{model, firmware, firmware_baseline, profile, profile_verified, prefix, api_snapshot{firmware, software, product, color_format, stereo_format, stereo_hdr, depth_enabled, time_sync_enabled, time_synced, ros_domain_id, dds_type, camera_wall_clock_epoch, temperature_c, uptime}}, time{start_utc,start_wall_ns,end_utc,first_msg_recv_ns,last_msg_recv_ns,duration_s}, depth{enabled,note}, topics[{role,name,type,required,reference_hz,frame_id,count,recv/stamp first/last ns,measured_stamp_rate_hz (median-period),measured_mean_rate_hz (delivered),est_missing_msgs,bytes}], bag{…}, disk{…}, preflight{…}, events[]`.
`firmware` is read from the camera's web API (GET only) and falls back to `--firmware`. Field set [V]; stability of the vendor API [?].
`measured_mean_rate_hz` (new in tool, additive) is the delivered rate of §5.1; `measured_stamp_rate_hz` is the median-period rate and **must not be used to judge rate or loss** (it reports 15.0 Hz for a depth stream that delivers 13.1 Hz).

## 4. Time — the most important section
| quantity | definition | status |
|---|---|---|
| **header.stamp** (all device topics) | **camera clock, seconds since camera boot** (e.g. 1339 s, 1719 s …), monotonic, no wall-clock meaning. Streams share one clock: L/R stereo stamps are bit-identical; RGB/IMU/VIO/depth are on the same time base | [V] |
| camera wall clock | `GET /api/system-time` = **2025-08-26** (≈ 457 days behind host); not used by header stamps | [V] |
| camera time-sync toggle | `POST /api/time-sync-setting {enabled:true}` accepted, but over ≥ 110 s `synced` stayed false and stamps/wall clock did not change ⇒ **no observable effect**; sync source/protocol unknown | [V] (no effect) / [?] (how it is meant to work) |
| bag message time (`messages.timestamp`) | host **wall clock (CLOCK_REALTIME) at reception** | [V] |
| host clock reliability | **WSL2: unreliable at times.** 16:07–16:40 the VM wall clock stepped ≈ +2.4 s every 32 s (systemd-timesyncd polls), producing fake multi-topic holes in bag receive time; at 18:08–18:50 (same VM, same camera) no step in > 16 min, rate within ±60 ppm of Windows. Preflight samples this (`--clock-seconds`) | [V] observed both states / [?] trigger of the bad state |
| **RULE 1** | **Use header.stamp for continuity, loss and sensor synchronisation. Never compute loss/gaps from bag receive time** | – |
| **RULE 2** | Host↔camera alignment is **not available** until camera time-sync works. Offsets between camera stamps and host time must not be assumed constant across camera reboots (boot-relative) | – |
| **RULE 3** | `inspect` classifies multi-topic holes seen only in receive time (header stamps contiguous) as "host clock", never as data loss | – |

## 5. Unified topic specification (official vs measured)
### 5.1 Definitions used by every number in this document
* **nominal Hz** — the rate the device is configured/claimed to produce. Source stated per field (status column).
* **delivered Hz** — mean rate of what actually arrived, `(n−1)/(last stamp − first stamp)` from **header stamps**. This is the "measured frequency". The *median-period* rate (`1/median interval`) is **not** used: it shows 15.0 Hz for a depth stream that delivers 13.1 Hz.
* **count loss %** — `1 − n/expected`, `expected = span × nominal + 1`, header-stamp space. This is the headline loss. The gap-based estimate (`est_missing_msgs`, stamp gaps > 1.5 × median interval) is a diagnostic only: it **over-counts** jittery stamps (VIO: 0.29/0.38 % gap-based vs 0.056/0.100 % by count) and **under-counts** omitted frames on a coarse grid (depth: 8.5–9.0 % gap-based vs 12.65/12.83 % by count).
* **W x H** — `width x height` as in the ROS message / camera API. The guides print the reverse order (§5.4, DEV-RES-ORDER).
* Run A = 2026-11-26 14:36, 340.6 s; run B = 2026-11-26 18:48, 320.2 s (WSL2 clock stepping during A, healthy during B; header-stamp numbers are unaffected).

### 5.2 Unified table — what fw 2.1.5 actually publishes
All name/type/resolution/encoding/rate values below are **measured**; the per-field status says how well. Items on which the guides and the measurements **agree** (IMU 400 Hz Imu; RGB JPEG CompressedImage 30 Hz; stereo mono8 Image 30 Hz; CameraInfo 30 Hz; message types everywhere) are described once, here, with the measured value — the guide agreeing adds no status.
<!-- BEGIN GENERATED:unified -->
| role | topic (fw 2.1.5, measured) | message type | W x H (measured) | encoding | nominal Hz | delivered Hz run A / run B | count loss % A / B | unit | status per field |
|---|---|---|---|---|---|---|---|---|---|
| `rgb` | `/camera/camera/color/image_rect_raw/compressed` | CompressedImage | 1088x1920 | jpeg | 30 [V] | 30.019 / 30.019 | 0 / 0 | - | topic:V type:V resolution:V encoding:V nominal:V rate:V unit:- |
| `stereo_left` | `/camera/camera/infra1/image_rect_raw` | Image | 544x640 | mono8 | 30 [V] | 30 / 30 | 0 / 0 | - | topic:V type:V resolution:V encoding:V nominal:V rate:V unit:- |
| `stereo_right` | `/camera/camera/infra2/image_rect_raw` | Image | 544x640 | mono8 | 30 [V] | 30 / 30 | 0 / 0 | - | topic:V type:V resolution:V encoding:V nominal:V rate:V unit:- |
| `depth` | `/camera/camera/depth/image_rect_raw` | Image | 544x640 | mono16 | 15 [O] | 13.1 / 13.077 | 12.654 / 12.828 | mm [?] | topic:V type:V resolution:V encoding:V nominal:O rate:V unit:? |
| `imu` | `/camera/camera/imu` | Imu | - | - | 400 [V] | 399.511 / 400 | 0.122 / 0 | accel m/s^2, gyro rad/s [?] | topic:V type:V resolution:- encoding:- nominal:V rate:V unit:? |
| `vio` | `/camera/camera/vio_100hz` | PoseStamped | - | - | 100 [O] | 99.944 / 99.9 | 0.056 / 0.1 | position m, orientation quaternion [?] | topic:V type:V resolution:- encoding:- nominal:O rate:V unit:? |
| `rgb_info` | `/camera/camera/color/camera_info` | CameraInfo | 1088x1920 | - | 30 [V] | 30.019 / 30.019 | 0 / 0 | pixels (K,P) [O] | topic:V type:V resolution:V encoding:- nominal:V rate:V unit:O |
| `stereo_left_info` | `/camera/camera/infra1/camera_info` | CameraInfo | 544x640 | - | 30 [V] | 30 / 30 | 0 / 0 | pixels (K,P) [O] | topic:V type:V resolution:V encoding:- nominal:V rate:V unit:O |
| `stereo_right_info` | `/camera/camera/infra2/camera_info` | CameraInfo | 544x640 | - | 30 [V] | 30 / 30 | 0 / 0 | pixels (K,P); P[0,3] = -fx*baseline (px*m) [O] | topic:V type:V resolution:V encoding:- nominal:V rate:V unit:O |
| `depth_info` | *(no such topic on fw 2.1.5)* | - | - | - | - | - | - | - | topic:O |
| `tf_static` | `/tf_static` | TFMessage | - | - | - | latched, 1 msg/run | - | - | topic:V type:V rate:- unit:O |
| `static_tf` | *(not published on fw 2.1.5)* | - | - | - | - | - | - | - | topic:O |
| `tf` | *(not published on fw 2.1.5)* | - | - | - | - | - | - | - | topic:O |
| `vio_image` (extra) | `/camera/camera/vio_image` | PoseStamped | - | - | 30 | 30 / 30 | - | - | topic:V type:V rate:V |
| `vio_image_cov` (extra) | `/camera/camera/vio_image_cov` | PoseWithCovarianceStamped | - | - | 30 | 30 / 30 | - | - | topic:V type:V rate:V |
| `vio_status` (extra) | `/camera/camera/vio_status` | String | - | - | 1 | - | - | - | topic:V type:V rate:O |
<!-- END GENERATED:unified -->

### 5.3 What the vendor guides say (documentation claims only — no status)
<!-- BEGIN GENERATED:official -->
| role | v1.2.0 topic | v1.2.0 res / enc / Hz / unit | v1.2.1 topic | v1.2.1 Hz / unit | measured topic |
|---|---|---|---|---|---|
| `rgb` | `/insight/camera_rgb/compressed` | 1920x1088 / jpeg / 30 / - | `/camera/camera/color/image_rect_raw/compressed` | 30 / - | `/camera/camera/color/image_rect_raw/compressed` |
| `stereo_left` | `/insight/camera_left_rectified` | 640x544 / mono8 / 30 / - | `/camera/camera/infra1/image_rect_raw` | 30 / - | `/camera/camera/infra1/image_rect_raw` |
| `stereo_right` | `/insight/camera_right_rectified` | 640x544 / mono8 / 30 / - | `/camera/camera/infra2/image_rect_raw` | 30 / - | `/camera/camera/infra2/image_rect_raw` |
| `depth` | `/insight/depth` | 640x544 / mono16 / 15 / mm | `/camera/depth/image_rect_raw` | 15 / mm | `/camera/camera/depth/image_rect_raw` |
| `imu` | `/insight/imu` | - / - / 400 / - | `/camera/camera/imu` | 400 / - | `/camera/camera/imu` |
| `vio` | `/insight/vio_pose` | - / - / 30 / - | `/insight/vio_pose` | 15 / - | `/camera/camera/vio_100hz` |
| `rgb_info` | `/insight/camera_rgb_info` | - / - / 30 / - | `/camera/color/camera_info` | 30 / - | `/camera/camera/color/camera_info` |
| `stereo_left_info` | `/insight/camera_left_info` | - / - / 30 / - | `/camera/infra1/camera_info` | 30 / - | `/camera/camera/infra1/camera_info` |
| `stereo_right_info` | `/insight/camera_right_info` | - / - / 30 / - | `/camera/infra2/camera_info` | 30 / - | `/camera/camera/infra2/camera_info` |
| `depth_info` | *(not listed)* | - | *(not listed)* | - | *(none)* |
| `tf_static` | `/static_tf` | - | `/tf_static` | - | `/tf_static` |
| `static_tf` | `/static_tf` | - | *(not listed)* | - | *(not published)* |
<!-- END GENERATED:official -->

### 5.4 Differences between the guides and fw 2.1.5 (and between the guides)
The three the project must not forget: **Depth** 15 Hz mono16 mm official vs ≈13.1 Hz delivered (12.65–12.83 % short); **VIO** 30 Hz (v1.2.0) / 15 Hz (v1.2.1) official vs 100 Hz (`vio_100hz`) and 30 Hz (`vio_image`) measured; **static TF** `/static_tf` (v1.2.0) vs `/tf_static` (v1.2.1 and measured).
<!-- BEGIN GENERATED:deviations -->
| id | difference | official statement | measured on fw 2.1.5 | status | note |
|---|---|---|---|---|---|
| DEV-DEPTH-RATE | Depth rate: official 15 Hz, delivered ~13.1 Hz | 15 Hz (v1.2.0 and v1.2.1) | delivered mean 13.100 Hz (run A) / 13.077 Hz (run B); count loss 12.654 % / 12.828 % vs 15 Hz; median frame interval 66.7 ms (a 15 Hz grid with frames omitted: intervals of 2,3,4 left-stereo frames) | V | located upstream of the host NIC, camera element undetermined: reports/3_depth_loss_root_cause.md. The gap-based estimate (8.5-9.0 %) UNDER-counts here; use the count-based number. |
| DEV-VIO-RATE | VIO: official /insight/vio_pose 30 Hz (v1.2.0) or 15 Hz (v1.2.1); measured vio_100hz 100 Hz and vio_image 30 Hz | /insight/vio_pose PoseStamped, 30 Hz (v1.2.0) / 15 Hz (v1.2.1) - the two guide versions disagree | /camera/camera/vio_100hz 99.944/99.900 Hz; /camera/camera/vio_image 30.0 Hz; no /insight/* topic exists | V | equivalence of any measured topic with /insight/vio_pose is UNCONFIRMED (?) |
| DEV-VIO-SEMANTICS | VIO pose definition | aligned to T_world_imu (both versions) | frame_id world; first pose near origin with a non-identity quaternion; world not linked in /tf_static | ? | whether the pose is T_world_imu, its scale and gravity alignment are unverified |
| DEV-STATIC-TF-NAME | Static TF topic name | /static_tf (v1.2.0) but /tf_static (v1.2.1) - the guide versions disagree | /tf_static (RELIABLE, TRANSIENT_LOCAL, 1 message, 5 edges); /static_tf absent from the 18-topic list | V | /tf_static is also the standard ROS 2 name; the capture tool records /tf_static only |
| DEV-NAMESPACE | Topic names and namespace | v1.2.0: everything under /insight/* (camera_left_rectified, camera_rgb/compressed, depth, ...); v1.2.1: RealSense-style /camera/camera/* (but depth and camera_info listed as /camera/depth.., /camera/infra1/camera_info, VIO still /insight/vio_pose) | fw 2.1.5 follows the v1.2.1 RealSense style with prefix /camera/camera for every device topic including depth and camera_info; no /insight/* topic | V | the capture discovery matches the measured names (and infra/vio_pose aliases); it does NOT match the v1.2.0 names - see DATA_CONTRACT.md section 9 |
| DEV-RES-ORDER | Resolution notation | strings 640x544 (stereo, depth) and 1920x1088 (RGB) | ROS messages and the camera API: width x height = 544x640 and 1088x1920 (JPEG SOF decoded: 1088 wide, 1920 high); i.e. the guides print height x width | V | interpretation "guide prints H x W" is an inference from the match, not a vendor statement (?) |
| DEV-DEPTH-UNIT | Depth unit | millimetre (both versions) | mono16 little-endian, median valid value 542, 0.4 % zeros, max 65535; never compared with a known distance | ? | the value range does not contradict mm; that is not verification |
| DEV-DEPTH-INFO | No depth CameraInfo | no depth camera_info in either guide | no depth camera_info topic on fw 2.1.5 (depth frame is identity from camera_camera_left in /tf_static) | O | depth intrinsics presumably = left stereo; unconfirmed |
<!-- END GENERATED:deviations -->

### 5.5 Publisher QoS (measured via Cyclone discovery, source QOS, [V] — `reports/depth_loss/camera_writer_qos.txt`)
RGB/stereo/CameraInfo: RELIABLE, KEEP_LAST 5 · depth: RELIABLE, KEEP_LAST 10 · IMU: **BEST_EFFORT, KEEP_LAST 1** · VIO topics: RELIABLE, KEEP_LAST 10 · `/tf_static`, `vio_status`: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 1.
Empty topics (publisher exists, 0 messages in every run, not in the guides): `infra1/image_raw`, `infra2/image_raw`, `color/image_raw/compressed` [O]. `/tf` is never published [O].

### 5.6 Sources
<!-- BEGIN GENERATED:sources -->
| code | kind | what | file |
|---|---|---|---|
| D0 | vendor_document | Looper Quick Start & Developer Guide v1.2.0 (topic namespace /insight/*) | `reports/vendor_docs/looper_insight9_quickstart_v1.2.0.html` |
| D1 | vendor_document | same guide v1.2.1 (RealSense-compatible names /camera/camera/*) | `reports/vendor_docs/looper_insight9_quickstart_v1.2.1.html` |
| M1 | measurement | run A inspect output (counts, stamp rates, diagnostics) | `reports/camera_data/real5min_inspect.json` |
| M2 | measurement | run B inspect output | `reports/camera_data/real5min_v215_inspect.json` |
| TL | measurement | ros2 topic list -t on domain 9 (18 topics) | `reports/camera_data/topics_domain9.txt` |
| MF | measurement | decoded message fields from the recorded bag (CameraInfo, TF, Image encodings, depth values) | `reports/contract_evidence/real_fw215_message_facts.json` |
| JPEG | measurement | JPEG SOF0 decode of RGB frames | `reports/contract_evidence/jpeg_sof_check.json` |
| API | measurement | camera web API GET (stereo-format, camera-format, deep-flow, ...) in reports/camera_data/*.json | `reports/camera_data/stereo-format.json` |
| QOS | measurement | camera DDS writer QoS via Cyclone discovery | `reports/depth_loss/camera_writer_qos.txt` |
| run A | measurement | 2026-11-26 14:36, 340.6 s, WSL2 wall clock was stepping at that time (receive-time artefact only; header-stamp numbers unaffected) | `reports/camera_data/real5min_inspect.json` |
| run B | measurement | 2026-11-26 18:48, 320.2 s, healthy WSL2 clock | `reports/camera_data/real5min_v215_inspect.json` |
<!-- END GENERATED:sources -->

## 6. Message semantics (carried over from v0.2; status marks unchanged)
| item | content | status |
|---|---|---|
| image encodings | stereo `mono8` step=544; depth `mono16` little-endian step=1088; colour JPEG (SOF0, 8-bit, 3 components, 1088 wide × 1920 high) | [V] |
| CameraInfo (stereo) | 544×640, K=[307.385,0,269.424; 0,307.385,323.476; 0,0,1], plumb_bob with D=0 ⇒ **rectified images**; right camera P[3]=−30.776 ⇒ baseline = 0.10012 m | [V] (numbers) / [?] "rectified" assumption |
| CameraInfo (colour) | 1088×1920, fx=768.863 fy=768.891 cx=539.467 cy=959.494, D=0 | [V] numbers |
| baseline | 0.10012 m: CameraInfo P[3]/fx and tf_static agree | [V] |
| depth values | mono16, median 542 over valid pixels, 0.4 % zeros, max 65535 present. **Unit (mm per both guides) and invalid-value convention unconfirmed — documentation only** | **[?]** |
| depth ↔ stereo | depth stamps equal left-stereo stamps (4472 of 4473) and depth frame is identity from `camera_camera_left` ⇒ depth is in the **left camera optical frame** | [V] stamps / [?] meaning |
| IMU units | \|a\| = 9.78 m/s² at rest ⇒ accel in **m/s²**; gyro presumably rad/s (mean ≈ 0.002 at rest). Neither guide states an IMU unit | [O] accel / [?] gyro |
| IMU frame/axes | frame `camera_camera_imu_optical` is **identity** from left in tf_static (yet an `imu → left` transform also exists); gravity vector at rest ≈ (−4.3, −8.8, 0.28) | **[?]** axes/sign |
| IMU covariance/orientation | all covariances 0 (= unknown), orientation identity (not estimated) | [V] |
| VIO pose | PoseStamped in frame `world`; first pose ≈ origin with non-identity quaternion (−0.2255, 0.6593, −0.5075, 0.5069); both guides say "aligned to T_world_imu" (documentation only). `vio_image_cov` diag σ² ≈ 3e-6 (pos) | **[?]** semantics, scale, gravity alignment |
| TF | `/tf_static`: left→right (0.10012,0,0); left→rgb (0.0501,≈0,≈0); imu→left (−0.03898,−0.01482,0.0189); left→depth and left→imu_optical identity. `world` is not connected | [O] values / [?] correctness |
| coordinate convention | ROS REP-103 / optical frame (x right, y down, z forward) for `*_optical` is **assumed, not verified** | **[?]** |

## 7. Quality metrics & thresholds (computed by `insight-capture inspect`)
* rate = **delivered** mean rate; loss = **count loss** vs the nominal rate (§5.1); tolerance `max_loss_ratio = 0.1 %` for [V] status. A stream whose count loss is below tolerance but whose gap-based estimate is above it is reported as *irregular stamp intervals (jitter)*, not as loss.
* systemic hole = ≥ 3 topics with a ≥ 0.5 s hole in receive time; **recv-only** if header stamps advance by ≤ 2 periods across it.
* sync skew: nearest-stamp matching (L/R: 0 ms measured; RGB↔stereo ≤ 16.7 ms).
* Results for the real recordings: `reports/P1_acceptance_report.md`.

## 8. Known deviations from "nominal" (do not hide)
1. **Depth delivers ≈13.1 Hz instead of the documented 15 Hz (12.65 % / 12.83 % short in runs A/B)**; located camera-side (`reports/3_depth_loss_root_cause.md`). Consumers must treat depth as irregular (intervals of 2, 3 or 4 left-stereo frames).
2. With depth enabled IMU/VIO/stereo show sporadic extra gaps (0.0–0.15 % by count in the 90–340 s runs); with depth disabled none in a 90 s test.
3. RGB exists only as JPEG; there is no raw colour topic. Guides and device agree.
4. VIO frame `world` is not linked to the camera TF tree. VIO delivers 100 Hz where the guides say 30 or 15 Hz.
5. The vendor guides' resolution strings are height × width; names differ from the guides (§5.4).

## 9. Open questions (need vendor / experiment)
| # | question | how to close |
|---|---|---|
| 1 | Is depth's intended rate 15 Hz? Is ≈13 Hz a known limit of fw 2.1.5? | vendor; reproduce on a second unit/native host |
| 2 | Depth units (mm per guides) and invalid value (0? 65535?) | measure a known target distance; vendor doc |
| 3 | IMU axes, gravity sign, gyro units; why imu_optical is identity | vendor doc; rotate camera and check |
| 4 | VIO pose definition (T_world_imu vs camera), initial orientation, behaviour in motion/loss of tracking | vendor doc; moving test |
| 5 | Which measured topic, if any, is the guides' `/insight/vio_pose` (30 Hz v1.2.0, 15 Hz v1.2.1)? Candidate: `vio_image` (30 Hz PoseStamped) — **unconfirmed** | vendor; compare poses in motion |
| 6 | How does camera time-sync work (NTP/PTP, who is the server)? | vendor doc; then retest with approval |
| 7 | Why was the WSL2 wall clock stepping at 16:07–16:40 but not at 18:08–18:50? | native-Ubuntu control (`run_native_control.sh`), Windows time service review |
| 8 | Same results on native Ubuntu 22.04? | run when a machine is available |
| 9 | `image_raw` topics: unused or published on demand? | vendor |
| 10 | Would a device with the **v1.2.0 `/insight/*` names** exist (other firmware)? The capture discovery currently resolves `/insight/imu` + `/insight/vio_pose` but **not** the v1.2.0 image/depth/camera_info/`/static_tf` names; no alias was added because those names were never observed (adding unverified aliases would blur the verification policy) | vendor: firmware list; add aliases only after observation |
| 11 | Guide resolution notation (H x W) — inferred from agreement with the device, not stated by the vendor | vendor |
| 12 | The two guide versions disagree (VIO 30 vs 15 Hz; `/static_tf` vs `/tf_static`): which is current? | vendor |

## 10. Change log
* **v0.3 (2026-11-27)**: official v1.2.0/v1.2.1 added next to the fw 2.1.5 measurements; per-field status and source codes; delivered-rate and count-loss definitions (VIO "0.15–0.4 % loss" in earlier notes was the gap-based estimate; by count 0.056/0.100 %); deviations table; generated tables + consistency tests. No status was raised from a document.
* v0.2 (2026-11-26): first version from real 2.1.5 data.
