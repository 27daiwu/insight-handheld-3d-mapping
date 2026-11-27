# 4. Topic config change log — `config/topics_v2.1.4.yaml` (2026-11-26)

**`verified` remains `false`.** Evidence is one real device (fw **2.1.5**, not 2.1.4), a few minutes of passive measurement over
WSL2 (mirrored networking). v2.1.4 was never observed, so the profile name/baseline cannot be called verified.

| # | Change | Reason / evidence | Status |
|---|---|---|---|
| 1 | stereo aliases `infra1`/`infra2` (images + camera_info) | real topics `/camera/camera/infra{1,2}/image_rect_raw`, `.../camera_info` (reports/camera_data/topics_domain9.txt) | **observed on device** |
| 2 | VIO suffix list += `vio_100hz`, `vio_pose`; `any_prefix: true` for vio | real VIO = `/camera/camera/vio_100hz` (PoseStamped, 100 Hz, frame `world`). Vendor guide's `/insight/vio_pose` NOT present on fw 2.1.5 | vio_100hz observed; `vio_pose`/any_prefix from docs only |
| 3 | topic ranking: raw before compressed, **`*rect*` before `*_raw`** | `infra1/image_raw`, `infra2/image_raw`, `color/image_raw/compressed` have a publisher but delivered **0 messages in 60 s**; only the `*_rect_raw` variants carry data | observed (60 s, 1 run) |
| 4 | `/tf` → `required: false`, `/tf_static` → `required: true` | device publishes only `/tf_static` (5 edges); no dynamic `/tf` | observed |
| 5 | `extra_types` += PoseWithCovarianceStamped, std_msgs/String | `vio_image_cov`, `vio_status` (1 Hz string) recorded with `--include-extra` | observed |
| 6 | `thresholds.max_loss_ratio: 0.001` | new diagnostics tolerance | design choice |
| 7 | tests: `test_real_fw215_*` uses the measured topic list; vendor-doc layout test kept (docs ≠ device) | | |

## Real device facts that do NOT fit the baseline/docs (record, don't hide)
* Firmware 2.1.5; ROS domain **9** (not 0); camera DDS configured **cyclonedds**, yet Fast DDS (Humble default) discovered it fine.
* **Depth is enabled** on this unit (`/api/deep-flow enabled=true`; `/camera/camera/depth/image_rect_raw` 15 Hz) — contradicts "default off". Not changed.
* RGB is **only JPEG `CompressedImage`** (no raw colour image). rgb stream 1088x1920 (portrait), stereo mono8 544x640, depth mono16 544x640.
* Rates vs vendor table: IMU 400 Hz ✔, stereo 30 Hz ✔, colour 30 Hz ✔, depth 15 Hz (header) ✔, **VIO 100 Hz (doc: 15 Hz)**, `vio_image` 30 Hz.
* Header stamps are **seconds since camera boot** (~1339 s at probe), not wall clock; camera wall clock is ~457 days behind host and time-sync is disabled. Receive−stamp offset is therefore meaningless until time sync is configured (needs your approval).
* QoS of publishers: IMU BEST_EFFORT; images/depth/vio RELIABLE; tf_static RELIABLE+TRANSIENT_LOCAL. rosbag2 recorder adapts (subscribes with matching QoS).
* `/tf_static` frames: camera_camera_{imu,left,right,rgb,depth,imu_optical}; VIO frame `world` is **not connected** to that tree (no dynamic TF to it).

## Still to verify before `verified: true`
Run on firmware 2.1.4 (or accept 2.1.5 as new baseline, your call); confirm after a camera reboot; decide on `image_raw` topics (empty — only after subscriber?); native-Ubuntu repeat; real 5-min recording without systemic holes (see report 2 and 3).

---
# Addendum 2026-11-26 evening — baseline moved to firmware 2.1.5 (`config/topics_v2.1.5.yaml`)

| change | detail |
|---|---|
| **new file** `config/topics_v2.1.5.yaml` | profile `insight9-v2.1.5`, `firmware_baseline: "2.1.5"`, default profile of the tool (`config.py`). Built from the calibrated v2.1.4 working copy |
| **`config/topics_v2.1.4.yaml` restored to its original content** (+1 comment line "HISTORICAL") | the earlier same-day calibration edits (infra1/infra2 aliases etc.) now live only in 2.1.5; history preserved |
| per-role `verification{verified,evidence,pending}` | **no bulk change.** Policy in the file header: ≥ 2 runs on 2.1.5, name/type/frame_id stable, monotonic stamps, rate = nominal, pooled loss ≤ 0.1 % (header-stamp space) |
| file-level `verified` | **stays `false`** (VIO and Depth unverified; no native-Ubuntu run) |
| `ros_domain_id_observed: 9`, `default_prefix_observed: /camera/camera` | informational |
| `schema_version` 0.1-provisional → `0.2-draft`; manifest gets `camera.api_snapshot` | firmware etc. read via read-only GET (whitelist client `camera_api.py`) |

Per-topic status (one decision each):
| role | verified | reason |
|---|---|---|
| rgb `color/image_rect_raw/compressed` | **true** | 30.019 Hz, 0 stamp gaps in 340 s, 6 runs; (JPEG only, `image_raw` empty) |
| stereo_left / stereo_right (`infra1|2/image_rect_raw`) | **true / true** | 30.000 Hz, L/R stamp-identical, pooled loss 0.02 / 0.03 % |
| imu | **true** | 400.0 Hz, pooled loss ≈ 0.05 %; axes/gyro unit pending (contract §6) |
| rgb_info / stereo_left_info / stereo_right_info | **true ×3** | 30 Hz, size/frame match images, baseline consistent |
| depth | **false** | 11.8–13.1 % shortfall in every run, camera-side (report 3); unit unconfirmed |
| vio (`vio_100hz`) | **false** | pooled loss ≈ 0.15 % > 0.1 %; semantics pending |
| depth_info | **false** | topic does not exist on 2.1.5 |
| `/tf_static` | **false** | extrinsics not validated; `/tf` never published |
Guard tests: `tests/test_profiles.py` (baseline 2.1.5, history kept, depth/vio/depth_info/tf_static must stay unverified until evidence changes).

Discovery changes (unchanged from earlier today): `*rect*` before `*_raw`; sibling prefixes; `any_prefix` roles; `vio_100hz`/`vio_pose` suffixes.

---
# Addendum 2026-11-27 — `spec:` block, DATA_CONTRACT v0.3
| change | detail |
|---|---|
| `topics_v2.1.5.yaml` gains top-level `spec:` | official v1.2.0 / v1.2.1 values next to measured values per role, per-field status (V/O/?/-) and source codes, 8 deviation records (DEV-*), run/source registry. Role matching, `verification` blocks and `verified: false` are **unchanged** |
| `vio` verification evidence rewritten | old text said "pooled loss ~0.15 % … above 0.1 % limit" (gap-based). Now: count loss 0.056/0.100 %; still `verified: false` for different, stated reasons (nominal only from topic name; guides list 30/15 Hz for another topic; semantics pending) |
| comment on `depth.required` | "Depth is OFF by default" replaced by the observed fact (ON on the tested unit) |
| no verification flag changed | no role was promoted; document claims cannot promote (enforced by `contract.validate_spec`, tested) |
| not done on purpose | no alias for the v1.2.0 `/insight/*` names (never observed on a device); no ROS topic or camera setting touched |
Files: `config/topics_v2.1.5.yaml`, `DATA_CONTRACT.md` (v0.3), `insight_capture/contract.py`, `tools/render_contract_tables.py`, `tests/test_contract_consistency.py`, `reports/vendor_docs/*`, `reports/contract_evidence/jpeg_sof_check.json`.
