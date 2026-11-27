# capture — Insight 9 capture node (ROS 2 Humble / Fast DDS / rosbag2)

```
source /opt/ros/humble/setup.bash
./capture/insight-capture preflight  [--prefix /insight9] [--firmware 2.1.4] [--expect imu=200 --expect rgb=30]
./capture/insight-capture record     [--name X] [--duration S] [--include-extra] [--image-mode raw|compressed|both]
./capture/insight-capture stop
./capture/insight-capture inspect    [session|latest] [--repair] [--json]
./capture/insight-capture replay     [session|latest] [-r 1.0] [--roles imu vio] [--clock] [--remap-prefix /x] [--dry-run]
```
Exit codes (preflight/inspect): 0 ok, 1 warnings, 2 failures.

* Topic roles are matched by *suffix + message type*, so any camera prefix works; `config/topics_v2.1.4.yaml`
  is **unverified** (no real device/official list was available) — fix it first when the camera is attached.
* Read-only toward camera/system: only subscribes; never changes camera params, network, drivers. Depth is
  recorded only if it is already publishing.
* Sim (no hardware): `ROS_DOMAIN_ID=77 python3 sim/sim_insight.py [--depth]`.
* Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest capture/tests` (integration tests use a random
  isolated ROS domain).
* WSL2 note: standalone processes did not discover each other without the `ros2 daemon`; preflight starts it.

## Diagnostics & host clock (2026-11-26)
* `inspect` also writes `<session>/diagnostics.json`: loss vs reference rate, interval percentiles, **multi-topic hole detection**; a hole whose
  header stamps are contiguous is reported as "receive time jumped, data NOT lost" (WSL2 timesyncd clock steps), otherwise FAIL.
* `preflight` samples the host wall clock for `--clock-seconds` (default 35). On WSL2 bag *receive* time is unreliable (±2.5 s sawtooth): use header stamps.
* Real camera (fw 2.1.5): ROS domain **9**, prefix `/camera/camera`; run `ROS_DOMAIN_ID=9 ./insight-capture preflight --firmware 2.1.5`.
* Reports: `reports/` (1 camera, 2 WSL2 RCA, 3 tests, 4 topic changelog, 5 GO/NO-GO).

## Update 2026-11-26 evening
* Default profile is now `config/topics_v2.1.5.yaml` (baseline fw 2.1.5; 2.1.4 kept as history; per-role verification, file-level `verified: false`).
* `preflight`/`record` read firmware, depth, stereo format and time-sync state from the camera web API with **GET only** (whitelist, never POST). Disable with `--camera-api off` or `INSIGHT_CAMERA_API=off` (the simulator tests do).
* Camera settings experiments (POST) are NOT part of the tool; they live in `reports/camera_param_experiment/` and need explicit authorisation.
* Contract draft: `DATA_CONTRACT.md` (v0.2). Reports: `reports/` (1 timesyncd, 2 camera params, 3 depth root cause, 4 topic changelog, 5 GO/NO-GO, P1_acceptance_report, 2_wsl2_packet_loss_rca, 1_camera_access_report, 3_capture_tool_test_report).

## Data specification v0.3 (2026-11-27)
* `DATA_CONTRACT.md` **v0.3 (draft)** puts the official vendor guides (v1.2.0 and v1.2.1, archived in `reports/vendor_docs/`), the fw 2.1.5 measurements, per-field verification status ([V]/[O]/[?]) and sources in one document. Manifest `schema_version` stays `0.2-draft`.
* The tables in sections 5.2–5.6 are **generated** from `config/topics_v2.1.5.yaml` (`spec:`): edit the YAML, then `python3 tools/render_contract_tables.py` (`--check` fails if stale). `tests/test_contract_consistency.py` ties document, YAML, archived guides and the raw recordings together.
* Rule: a vendor-document statement is never [V]/[O] by itself. Key differences recorded: **Depth** 15 Hz mono16 mm (official) vs ≈ 13.1 Hz delivered (12.65–12.83 % short, unit unverified); **VIO** 30 Hz (v1.2.0) / 15 Hz (v1.2.1) vs 100 Hz `vio_100hz` (+ 30 Hz `vio_image`); **static TF** `/static_tf` (v1.2.0) vs `/tf_static` (v1.2.1 and device).
* Rate and loss definitions changed in the tool: `inspect`/`preflight` now report the **delivered** mean rate (`13.077 Hz delivered (median-period 14.999 Hz)`) and flag jittery-but-complete streams as jitter, not loss. Manifest topics gain `measured_mean_rate_hz`.
