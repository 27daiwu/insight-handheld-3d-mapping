# Insight Handheld 3D Mapping

Handheld 3D mapping with the Looper Robotics **Insight 9** camera.
Capture runs on an Ultra 7 laptop; mapping/reconstruction runs on an RTX 4090 desktop.
Both sides communicate **only** through the Session data contract.

## Technology baseline

| Item | Choice |
|---|---|
| Camera | Looper Robotics Insight 9 |
| Integration | ROS Native (preferred) |
| ROS 2 / DDS | Humble / Fast DDS |
| Capture format | rosbag2 (MCAP preferred, SQLite3 fallback) |
| SLAM | RTAB-Map |
| Reconstruction | Open3D |
| Later GPU stage | nvblox |
| Targets | Ultra 7 thin-and-light laptop (capture), RTX 4090 desktop (mapping) |

## Layout

```
docs/       ARCHITECTURE.md, DATA_CONTRACT.md, DEVELOPMENT_PLAN.md
schemas/    session_manifest.schema.json   (the contract, machine-readable)
config/     insight9_topics.yaml           (topic / QoS expectations)
capture/    capture-side code     (branch feat/capture)
mapping/    compute-side code     (branch feat/mapping)
tests/      contract tests (protocol-level, owned by main)
datasets/   local Sessions (git-ignored)
tools/      protocol tooling: manifest validation, boundary check
```

## Branch & ownership rules

| Branch | May modify |
|---|---|
| `feat/capture` | `capture/` only |
| `feat/mapping` | `mapping/` only |
| `main` | everything else: `docs/`, `schemas/`, `config/`, `tests/`, `tools/`, `README.md` |

The data protocol (`docs/DATA_CONTRACT.md`, `schemas/`, `config/`) is maintained on `main`.
Neither side may change it; request changes via an issue/PR against `main`.
Check a branch with `tools/check_boundaries.sh <capture|mapping> [base]`.

## Status

Phase 0: scaffolding, protocol and docs only. No algorithm code, no system dependencies,
no camera connection yet.
