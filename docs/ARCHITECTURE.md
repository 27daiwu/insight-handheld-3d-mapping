# Architecture

## Overview

```
 Insight 9 ──ROS 2 Humble / Fast DDS──▶ [Capture · Ultra 7 laptop]
                                            │ rosbag2 (MCAP | SQLite3)
                                            ▼
                                   Session directory  (DATA_CONTRACT.md)
                                            │ copy / sync
                                            ▼
                                  [Mapping · RTX 4090 desktop]
                                   RTAB-Map → Open3D → (nvblox later)
```

The **Session** is the only interface between the two halves. Capture produces
Sessions; mapping consumes them. Neither reads the other's code or private formats.

## Components

### Capture (`capture/`, Ultra 7 laptop)
- Subscribes to Insight 9 ROS topics (see `config/insight9_topics.yaml`).
- Records rosbag2 with original ROS timestamps untouched.
- Collects environment facts (firmware, ROS, DDS, camera config) and
  writes `manifest.json`, `logs/`, `calibration/`.
- Computes quality stats (drops, duration, integrity) and finalises the manifest.
- Must stay lightweight: no SLAM/reconstruction on the laptop.

### Mapping (`mapping/`, RTX 4090 desktop)
- Validates a Session against the schema before processing.
- Replays the bag into RTAB-Map (visual-inertial odometry from the camera is
  recorded and can be used as odometry prior).
- Produces point cloud / mesh via Open3D.
- Later: nvblox GPU TSDF/ESDF.
- Writes outputs outside the Session's `bag/` (never mutate raw data).

### Protocol (main)
- `docs/DATA_CONTRACT.md`, `schemas/session_manifest.schema.json`, `config/insight9_topics.yaml`.
- `tools/` validation utilities and `tests/` contract tests.

## Principles
1. Raw data is immutable; original ROS timestamps preserved.
2. One data format, owned by main.
3. Record what was *actually* observed (resolution, rate, encoding), not what was configured.
4. Capture and mapping are independently buildable and testable using Sessions as fixtures.

## Open items (to be resolved during development, not in Phase 0)
- Exact Insight 9 topic names, QoS, frame ids: confirmed by capture-side on real hardware,
  then proposed to main for `config/insight9_topics.yaml`.
- Transport of Sessions between machines (rsync/NAS/USB).
- Whether VIO output is used as RTAB-Map odometry or only as a reference.
