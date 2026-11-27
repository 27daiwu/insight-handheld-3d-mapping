# Phase 1 — Insight 9 stereo-first reconstruction (`insight_mapping.i9`)

Status date: 2026-11-27. Data contract: `capture/DATA_CONTRACT.md` v0.3 (lives on `feat/capture`), device Insight 9, fw 2.1.5.

## HONEST STATUS: no real Insight 9 recording was available on the mapping host
A host-wide search found no real `.mcap/.db3` from the device (capture branch commits only reports, no bag). Everything below was
exercised on a **contract-faithful simulation** (`i9/fixture.py`: real topic names, 544x640 mono8, real K/P/baseline, 13 Hz depth with drops,
PoseStamped VIO, `/tf_static` only, boot-relative header stamps vs wall-clock bag times). **Simulated results prove the code path, not the device.**
Reports carry `data_origin`; real runs must be started with `--origin REAL_INSIGHT9_FW2.1.5`.

## Run
```
source mapping/scripts/env.sh                      # only required for SQLite3 bags
cd mapping
.venv/bin/python -m insight_mapping.i9 scan  <session_or_bag_or.mcap> [--out scan.json]
.venv/bin/python -m insight_mapping.i9 check <bag> --out calib.json              # sync + CameraInfo + calibration verdict
.venv/bin/python -m insight_mapping.i9 run   <bag> --out work/i9/<name> --origin REAL_INSIGHT9_FW2.1.5 [--max-seconds 60] [--voxel 0.02]
.venv/bin/python -m insight_mapping.i9 scale-check <bag> --marker-size 0.10      # independent metric proof (ArUco DICT_4X4_50)
```
Input may be a Session dir, a rosbag2 dir or a bare `.mcap` (read with pure-python `mcap`; installed into `.venv`: `pip install mcap mcap-ros2-support`).

## Stages (all stereo-only; IMU/Depth/VIO are indexed, never fused)
| # | Module | Output |
|---|---|---|
| 1 | `scan.py`, `topics.py`, `reader.py` | topics/types/frame_ids, header-stamp rate & count loss (contract §5.1 definitions), receive-vs-header drift, contract deviations, `scan.json` |
| 2 | `calibcheck.py`, `rig.py` | L/R stamp pairing, CameraInfo K/P/D/R consistency, baseline from P vs TF, epipolar rows (dy), disparity sign/range, depth-stream ratio (hypothesis only), PASS/CONDITIONAL/FAIL |
| 3 | `stereo.py` | parallel SGBM disparity + ORB cache (resumable layout under `<out>/stereo`) |
| 4 | `vo.py` | keyframe stereo VO (ORB, stereo-triangulated 3D, PnP-RANSAC + LM, local map of 3 keyframes, tracking-loss fallback with reason), `trajectory.csv` (T_world_leftoptical, world = first frame) |
| 5 | `dense.py` | disparity→depth with σ_Z gate and edge filter, Open3D TSDF (CUDA), `map.ply/pcd`, `mesh.ply` |
| 6 | `report.py` | `reconstruction_report.md/json`: rates, sync, calibration, VO, density, multi-view depth consistency, plane RMS, grades, limitations |

## Rules kept from the contract
Header stamp (never bag time) for rate/loss/sync; delivered-Hz and count-loss definitions of §5.1; nothing marked [?] is interpreted
(depth unit, VIO semantics, IMU axes); `/tf_static` used only as a cross-check of the CameraInfo baseline; depth CameraInfo absent is expected.

## Simulation result (unit-test baseline, 90 frames, 0.10012 m baseline)
ATE (rigid) 3.8 mm, scale ratio 1.002, path ratio 0.998, 0 lost frames, 124 k map points, depth consistency 0.65 %, wall 5 s. Not evidence for real data.

## Known gaps / next
* Real-data first run: expect to tune SGBM (low-texture, HDR), VO thresholds, `max_sigma_z_rel`.
* No loop closure / pose graph yet (open-loop drift); the existing `slam/` back-end can be reused once real VO is stable.
* RGB texture requires verified RGB↔left extrinsic; depth-stream/VIO fusion after contract items 2–5 of §9 close.
* Scale: only `scale-check` is independent of the vendor calibration.
