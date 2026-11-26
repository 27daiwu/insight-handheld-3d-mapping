# DATA_CONTRACT (capture side) — PROVISIONAL v0.1

The project-wide `DATA_CONTRACT.md` was **not present** in this repository (only README.md exists), so
this file records what the capture node actually writes. Once the shared contract exists, reconcile
this file with it (schema_version `0.1-provisional` marks the mismatch risk).

## Session layout
`$INSIGHT_DATA_ROOT` (default `~/insight_data/sessions`) / `<YYYYmmdd_HHMMSS>_<name|hostname>/`

| file | content |
|---|---|
| `bag/` | rosbag2, sqlite3 storage, CDR, split at 2 GiB (`bag_N.db3`, `metadata.yaml`) |
| `manifest.json` | dataset description (below) |
| `plan.json` | topics the supervisor was asked to record |
| `preflight.json` | full preflight report taken at record start |
| `session.json` | live state + heartbeat (status, pids, free disk, events) |
| `recorder.log`, `supervisor.out` | rosbag2 / supervisor logs |

A session directory is never reused. `ACTIVE` in the root points to the running session.

## Timestamps
* Bag message time (`messages.timestamp`) = **host receive time** (ns, system clock) — rosbag2 default.
* `header.stamp` inside each message is **untouched camera time**. Nothing is rewritten or re-stamped.
* manifest keeps per topic `recv_first/last_ns` and `stamp_first/last_ns`; the clock offset between the
  two is reported by `inspect` (`recv_minus_stamp_median_s`).

## manifest.json
`schema_version, session_id, status (recording|complete|stopped_abnormal|recovered|finalize_failed),
stop_reason, tool{name,version,git}, host{os,kernel,hostname,python,wsl,ros{...}},
camera{model,firmware,firmware_baseline,profile,profile_verified,prefix},
time{start_utc,start_wall_ns,end_utc,first_msg_recv_ns,last_msg_recv_ns,duration_s},
depth{enabled,note}, topics[{role,name,type,required,reference_hz,frame_id,count,recv/stamp first/last ns,
measured_*_rate_hz,est_missing_msgs,bytes}], bag{dir,storage,max_bag_size,files,size_bytes,metadata_present,
recorder_rc}, disk{path,free_gb_start,free_gb_end,min_free_gb}, preflight{exit_code,warnings}, events[]`

Roles: `rgb stereo_left stereo_right depth imu vio rgb_info stereo_left_info stereo_right_info depth_info tf
tf_static` (+ `extra`). `firmware` is `null` unless supplied with `--firmware` (no ROS interface known).
Depth topics are recorded only when they have a publisher; otherwise `depth.enabled=false` + note.
