# Data Contract — Session

Contract version: **1.0.0-draft** (`schema_version` field in manifest).
Maintained on `main` only. Capture and mapping MUST NOT define their own formats.
Machine-readable definition: `schemas/session_manifest.schema.json`.

## 1. Session layout

Each scan produces one independent Session directory:

```
session_<YYYYMMDD>_<HHMMSS>[_<label>]/
├── manifest.json      # metadata, written by capture, validated against schema
├── bag/               # rosbag2 directory (metadata.yaml + MCAP or .db3 files)
├── logs/              # recorder log, ROS/DDS logs, drop/health reports
└── calibration/       # camera intrinsics/extrinsics snapshot, as received
```

`session_id` in the manifest equals the directory name.

## 2. Mandatory recorded data

| Requirement | Detail |
|---|---|
| Original timestamps | Header stamps and bag receive times are stored as-is. No rewriting, resampling, or time-shifting at capture. Any offset estimate goes in the manifest, not the data. |
| CameraInfo | One per camera image stream. |
| TF | `/tf` and `/tf_static`. |
| IMU | `sensor_msgs/Imu` raw stream. |
| VIO | Camera's visual-inertial odometry output (`nav_msgs/Odometry` or equivalent; actual type recorded in manifest). |
| Images | All image streams listed in `config/insight9_topics.yaml` as `required`. |

A Session missing any mandatory stream is `integrity.status = "incomplete"` and
mapping must refuse or warn explicitly.

## 3. Stream records (manifest `streams[]`)

For every recorded topic capture records **measured** values:
- `topic`, `role` (`image`, `depth`, `camera_info`, `imu`, `vio`, `tf`, `tf_static`, `other`), `msg_type`
- `frame_id`
- `message_count`, `measured_rate_hz`, `nominal_rate_hz` (if known)
- images: `width`, `height`, `encoding` (ROS encoding, e.g. `mono8`, `bgr8`, `16UC1`, or compressed format)
- `first_stamp_ns`, `last_stamp_ns` (ROS header stamps)
- `dropped_messages` (estimated, with `drop_estimation_method`)

## 4. Environment record (manifest `environment`)

- `camera`: model, serial, `firmware_version`, `config` (the effective camera configuration as dictionary/snapshot reference)
- `ros`: distro (`humble`), `rmw_implementation`
- `dds`: vendor (`fastdds`), version, QoS profile / XML reference
- `host`: hostname, OS, CPU model, recorder software version/git commit

## 5. Recording and integrity (manifest `recording`, `integrity`)

- `start_time_utc`, `end_time_utc`, `duration_s` (wall-clock) plus ROS time span.
- `storage`: `mcap` or `sqlite3`; compression if any.
- Storage policy: **MCAP preferred**. Use `sqlite3` only when the rosbag2 MCAP plugin
  (`rosbag2_storage_mcap`) is not installed; capture records the reason in `storage.fallback_reason`.
- `integrity`: `status` (`complete` | `incomplete` | `corrupt`), total/expected message counts,
  drop summary, `bag_sha256` of file list (per-file checksums), clock-sync note, warnings list.
- The manifest is written at start (`recording.state = "recording"`) and finalised at stop
  (`"finalized"`); mapping only accepts `finalized` Sessions.

## 6. Calibration folder

Snapshot of calibration as provided by the camera/driver at capture time (CameraInfo dumps,
extrinsics, IMU-camera transform). Files are listed in `manifest.calibration.files`.
Calibration in the bag (CameraInfo/TF) is authoritative; the folder is a convenience copy.

## 7. Change control

- Contract changes: PR to `main` modifying `docs/DATA_CONTRACT.md` + schema together, with version bump
  (semver: patch = clarification, minor = backward-compatible field addition, major = breaking).
- Capture/mapping branches may only read these files.
- Unknown extra fields: forbidden at the top level, allowed under `extensions` only.
