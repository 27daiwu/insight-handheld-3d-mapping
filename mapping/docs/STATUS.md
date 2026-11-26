# Mapping status (feat/mapping) — 2026-11-26, after P1.2

Baseline kept: commit `71b895b` (P1: custom back-end, Open3D TSDF, 28 tests). P1.2 builds on it.
No real Insight 9 Session exists yet; everything below is measured on synthetic data and public TUM RGB-D data.

## What P1.2 added
| Item | State |
|---|---|
| RTAB-Map + MCAP plugin | Installed **without root**: `scripts/install_rtabmap_userspace.sh` downloads the apt `.deb`s (RTAB-Map 0.23.x, GTSAM, g2o, `rosbag2_storage_mcap`, deps; 42 packages, 41 MB) and extracts them to `~/.local/opt/ros-extra`; `source scripts/env.sh` activates it. No system change, no driver/CUDA change. |
| RTAB-Map back-end | `rtabmap/backend.py`: replays an extracted Session lock-step through the real `rtabmap_slam` / `rtabmap_odom` ROS 2 nodes. Modes: `rtabmap_vio` (Insight VIO as external odometry, covariance configurable) and `rtabmap_odom` (own `rgbd_odometry` / `stereo_odometry`); sensors rgbd or stereo. Result → common `trajectory_opt.csv`. |
| 3-way comparison | `compare.py` / `insight-mapping compare`: custom vs `rtabmap_vio` vs `rtabmap_odom` (+ raw VIO and each back-end's pre-optimisation odometry): ATE (aligned/anchored), RPE, drift, loop count, wall time, peak RSS, map accuracy/completeness vs a GT-pose map, analytic scene score. Results: `docs/COMPARISON.md`. |
| Config / frames / calibration | `config/topic_map.yaml` overrides (topics, extrinsics, depth scale); `calibfiles.py` (camera_info YAML / CameraInfo JSON / extrinsics from `calibration/`, bag wins on conflict); rectification incl. R/P and **equidistant (fisheye)** model; conflicts reported in `quality_report.json`. |
| Protocol proposal | `docs/PROTOCOL_PROPOSAL.md` — optional manifest fields (1.0.0 → 1.1.0 minor), **not applied**; `main` decides. |
| Texture + GLB | `recon/texture.py`: view-atlas texture, `mesh_textured.obj`(+mtl/png), `mesh.glb` (z-up→y-up); report has `texture` block; all outputs re-opened by independent readers. |
| Cleanup | Removed `rtabmap_bridge.py` (superseded). |

## Tests
`pytest`: **39 passed** (3 min 51 s) with ROS + RTAB-Map env: geometry/contract (baseline), rectification (pinhole, plumb_bob, fisheye), calibration-folder fallback, texture UV/colour check, GLB structure, MCAP round-trip with bit-exact stamps, config overrides, RTAB-Map back-ends on synthetic, GPU≡CPU TSDF, TUM fr1/desk end-to-end. Tests needing ROS/RTAB-Map/TUM data skip when unavailable.

## Measured comparison (ATE aligned RMSE, m; details and caveats in `docs/COMPARISON.md`)
| Data | VIO raw | custom | rtabmap_vio | rtabmap_odom |
|---|---|---|---|---|
| synthetic room, 240 fr (RGB-D) | 0.214 | 0.036 | 0.041 | 0.029 |
| TUM fr1/desk, 307 fr | 0.186 | 0.018 | 0.020 | 0.022–0.05 (noisy; 1 of 4 reruns lost tracking at ~100 fr) |
| TUM fr1/xyz, 399 fr | 0.218 | 0.021 | 0.017 | 0.010 |
Stereo on the synthetic room: `rtabmap_vio` 0.032, `rtabmap_odom` 0.029.
RTAB-Map `rtabmap_vio` replay takes 60–140 s per sequence (lock-step, dominated by replay), custom 2–4 s; RTAB-Map peak RSS ~0.5–0.9 GiB.

Findings:
* RTAB-Map's own-odometry defaults lost tracking on TUM desk (`Vis/MinInliers=20`); defaults were changed to frame-to-map, `MinInliers=12`, `MaxFeatures=1500`, `CorGuessWinSize=40`. These were tuned **on the evaluation sequences** → optimistic. Frame-to-frame and `CorGuessWinSize=40` with 10 inliers diverged on the synthetic room; results vary between reruns (documented).
* For `rtabmap_vio` the GTSAM optimiser beat g2o (0.040 vs 0.058 on synthetic); `RGBD/OptimizeMaxError=0` (accept every loop) destroyed the result (1.3 m); `Optimizer/Robust=true` disabled loop use effectively (0.21 = raw). VIO covariance 1e-3…1e-7 made no measurable difference.
* The VIO-fed and odometry-fed back-ends are indistinguishable on the TUM sequences within run noise; the "VIO" there is ground truth + injected drift, so this says nothing about real Insight VIO quality.
* Texture check (TUM desk): 88 % of triangles textured by 24 views; atlas colour at triangle UVs correlates 0.93 with the vertex colours (MAE 0.067); software preview rendered (`recon/preview.py`) — Open3D's offscreen renderer returned black images on this host, so no GL render was relied on. GLB opens in the in-repo structural verifier only; **not yet checked in an external viewer** (no viewer available here).

## GPU (unchanged from baseline, measured)
TSDF integrate CPU 0.181 s vs CUDA 0.060 s (3.0×, TUM 1 cm voxels, 149 frames); only TSDF integrate/extract on GPU; everything else CPU. nvblox deferred as instructed.

## Known gaps / risks
1. **No real Insight 9 data.** Topic names, VIO frame ids, `vio_image` vs `vio_100hz` semantics, depth unit/encoding, lens model and calibration content are assumptions (`topic_map.yaml`, `PROTOCOL_PROPOSAL.md`).
2. RTAB-Map integration is for offline replay with external VIO via a custom lock-step feeder; stamps are used as given (no latency compensation). Stereo mode expects rectified-or-rectifiable pinhole/fisheye input; RTAB-Map receives *rectified* images with identity R.
3. RTAB-Map keyframe poses are converted to per-frame trajectories by interpolating keyframe corrections (same method as custom) — a modelling choice that adds local error (RPE rotation ≈1.0° vs raw 0.35°).
4. Own-odometry results are noisy and tuned on the test sequences; no held-out sequence passed (the TUM fr1/room download kept truncating and was dropped).
5. IMU is read/checked only; **no IMU fusion** (deferred pending real calibration). nvblox and deep GPU optimisation deferred.
6. Loop closure in custom back-end is ORB+depth-PnP only (O(N²) gated), per-vertex/atlas texture without seam blending or exposure compensation; glTF uses unlit-ish PBR with embedded PNG (24 MB for desk).
7. Sessions with MCAP were exercised with synthetic data only (plugin works through the user-space overlay).
8. `tools/validate_manifest.py` (main) remains unexercised; mapping validates with `jsonschema`.

## Next steps
1. First real Session: `validate` → `quality` → fix topic map / frames; send verified names to `main`; apply accepted protocol fields.
2. Held-out public sequence(s) to validate defaults; try RTAB-Map with real-style stereo (fisheye) once calibration is known.
3. External viewer check of GLB; texture seam blending / exposure compensation.
4. Revisit IMU gravity alignment and nvblox after calibration is confirmed.
