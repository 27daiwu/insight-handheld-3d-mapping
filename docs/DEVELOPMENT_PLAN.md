# Development Plan

## Phase 0 — Initialization (this commit)
Structure, data contract, schema, topic config, docs. No code, no dependencies, no camera access.

## Phase 1 — Parallel development

### Track A: `feat/capture` (only `capture/`)
1. ROS 2 Humble + Fast DDS environment bring-up notes (in `capture/`, not system install scripts run by us).
2. Discover and confirm Insight 9 topics/QoS read-only; report differences to main.
3. rosbag2 recorder wrapper: MCAP preferred, SQLite3 fallback with reason.
4. Manifest writer producing schema-valid output; stats (rate, drops, integrity).
5. Capture UX for handheld use (start/stop, health indicators).
6. Produce sample Sessions into `datasets/` for the mapping team.

### Track B: `feat/mapping` (only `mapping/`)
1. Session loader + manifest validation (uses `schemas/`).
2. Bag replay → RTAB-Map integration (offline).
3. Open3D reconstruction/export (PLY/mesh).
4. Quality report per Session.
5. Later: nvblox GPU stage.
Until real Sessions exist, develop against synthetic/minimal fixtures that conform to the schema.

### Track M: `main` (protocol owner)
- Review protocol-change requests, bump contract versions.
- Maintain `tools/` validation and `tests/` contract tests.
- Merge branches after `tools/check_boundaries.sh` passes.

## Milestones
| Milestone | Exit criterion |
|---|---|
| M0 | Scaffolding committed (done) |
| M1 | Capture produces a schema-valid Session from real Insight 9 |
| M2 | Mapping loads a Session and runs RTAB-Map offline |
| M3 | End-to-end: handheld scan → point cloud/mesh |
| M4 | nvblox integration |

## Rules
- Protocol changes only through main.
- Merges to main require the boundary check and manifest validation to pass.
