# Proposal to `main`: Session manifest extensions (contract 1.0.0-draft → 1.1.0)

**Status: proposal only.** `docs/DATA_CONTRACT.md`, `schemas/` and `config/` are owned by `main`; nothing here was applied.
Author: mapping side (`feat/mapping`). All additions are **optional** fields ⇒ a *minor* bump, 1.0.0 Sessions stay valid.
Until accepted, the mapping side keeps working through name patterns / headers (`mapping/config/topic_map.yaml`) and treats
every declaration below as a *claim to verify*, never as truth (e.g. RGB/Depth alignment is always re-proven from CameraInfo + TF).

## Why (what the mapping side had to guess)

| Guess today | Where | Risk |
|---|---|---|
| which `role: vio` topic is the image-synchronised one and which is the 100 Hz one | regex on topic names (`session.resolve_streams`) | wrong pairing → wrong poses silently |
| which CameraInfo belongs to which image topic | same-namespace / regex | ambiguous with 2 stereo + RGB + depth |
| depth unit (16UC1 mm vs 32FC1 m) and whether depth is registered to RGB | read from msg encoding; alignment re-derived | misscaled map |
| frame ids of cameras, VIO parent/child | read from message headers during extraction (not available before parsing the bag) | cannot pre-validate a Session cheaply |
| whether images are rectified / which distortion model | CameraInfo only | fisheye vs pinhole decided late |
| stamp semantics (exposure start/mid/publish) and VIO↔image latency | unknown | systematic sync offset, indistinguishable from drift |
| calibration folder content | file-name heuristics (`calibfiles.py`) | silent fallback to wrong file |

## Proposed fields (all optional)

`streams[]` items:

| field | type | meaning |
|---|---|---|
| `logical_role` | enum `rgb, depth, left, right, vio_image, vio_100hz, imu, tf, tf_static, other` | unambiguous semantic of the topic (finer than `role`); `vio_image` = pose sample synchronised to the stereo frame, `vio_100hz` = high-rate pose |
| `camera_info_topic` | string | CameraInfo topic belonging to this image stream |
| `depth_unit` | enum `mm, m` | numeric unit of depth pixels (16UC1 → `mm`, 32FC1 → `m`) |
| `registered_to` | string \| null | frame id the depth is *claimed* registered to (mapping re-verifies) |
| `distortion_model` | string | as in CameraInfo (`plumb_bob`, `rational_polynomial`, `equidistant`) |
| `rectified` | bool | image already undistorted/rectified by the driver |
| `odom_frames` | `{parent, child}` | for VIO streams: `Odometry.header.frame_id` / `child_frame_id` |
| `stamp_semantics` | enum `exposure_start, exposure_mid, exposure_end, publish_time, unknown` | what `header.stamp` means |
| `nominal_latency_ms` | number | known processing latency of the stream relative to the physical event, if the SDK documents it |

`environment.camera`:

| field | meaning |
|---|---|
| `extrinsics_source` | `factory`, `user_calibrated`, `unknown` |
| `stereo_baseline_m` | nominal value, to cross-check CameraInfo/TF |

`recording`:

| field | meaning |
|---|---|
| `time_base` | `{source: "camera_clock"\|"host_clock"\|"ptp", offset_ns?: int, offset_stddev_ns?: int}` — relation between camera stamps and the host/bag clock |

`calibration.files[]` — allow objects `{ "path": "...", "kind": "camera_info"|"extrinsics"|"other", "frame_id"?: "..." }` besides plain strings.

### JSON-schema patch sketch (for `streams.items.properties`)
```json
"logical_role": {"enum": ["rgb","depth","left","right","vio_image","vio_100hz","imu","tf","tf_static","other"]},
"camera_info_topic": {"type": "string"},
"depth_unit": {"enum": ["mm","m"]},
"registered_to": {"type": ["string","null"]},
"distortion_model": {"type": "string"},
"rectified": {"type": "boolean"},
"odom_frames": {"type": "object", "additionalProperties": false, "required": ["parent","child"],
                "properties": {"parent": {"type": "string"}, "child": {"type": "string"}}},
"stamp_semantics": {"enum": ["exposure_start","exposure_mid","exposure_end","publish_time","unknown"]},
"nominal_latency_ms": {"type": "number"}
```
Conditional rules suggested: `logical_role ∈ {rgb, left, right, depth}` ⇒ `camera_info_topic` required;
`logical_role = depth` ⇒ `depth_unit` required; `logical_role ∈ {vio_image, vio_100hz}` ⇒ `odom_frames` required.

## Questions that only real Insight 9 data / capture can answer
1. Real topic names, QoS, and whether `/tf` carries the VIO pose (`odom → body`) in addition to the Odometry topics.
2. Which VIO output is synchronised with the stereo image stamp (`vio_image`) and what its latency is.
3. Are stereo images rectified? Is there a depth stream, in which frame and unit? Is RGB present and calibrated to depth?
4. Lens model (pinhole vs fisheye) — the mapping side now supports `plumb_bob`, `rational_polynomial`, `equidistant`.
5. IMU frame, units and gravity sign; extrinsic IMU↔camera (needed before any IMU fusion; **not used** by mapping yet).

## What the mapping side does meanwhile
* `config/topic_map.yaml`: per-run overrides, name patterns, `extrinsics:` and `depth.scale_to_mm` overrides.
* Fallback to `calibration/` files (ROS camera_info YAML / CameraInfo JSON / extrinsics list) only where the bag has nothing;
  conflicts between folder and bag are reported as warnings, bag wins (contract §6).
* Every resolution decision is written to `extracted/extract_summary.json` and `quality_report.json`.
