# Mapping status (feat/mapping) — 2026-11-26

## Verified (this host, no real Insight 9 data)
| Item | Result |
|---|---|
| Tests | `pytest`: 28 passed (geometry, contract, end-to-end synthetic, GPU≡CPU, TUM fr1/desk) |
| Synthetic room, 240 frames, **unaligned** RGB/Depth, VIO with injected drift | VIO ATE 0.214 m → optimised 0.036 m; 16 loop-edges accepted; mesh median error to the analytic scene 0.24 m (VIO poses) → 0.029 m (optimised) |
| TUM fr1/desk (307 frames; "VIO" = ground truth + injected smooth drift, no IMU/stereo) | ATE 0.186 m → 0.018 m; 58 loop edges; 146k points, 128k triangles, 12 s end to end; all 6 outputs re-opened OK |
| Frame-convention check | correct convention scores 0.2–1.0° / 1–3 cm; the 3 wrong hypotheses ≥ 7° / ≥ 10 cm; an inverted VIO pose aborts the run (tested) |

**Caveat:** on TUM the "VIO" is synthetic, so this validates the loop-closure/graph/fusion code, not Insight VIO behaviour.

## GPU (P1-E) — measured
* Open3D 0.20.0 pip build has CUDA; real GPU tensor ops and `VoxelBlockGrid` on `CUDA:0` work (not inferred from CUDA presence).
* TSDF integration, CPU vs CUDA, TUM desk 640×480, 1 cm voxel, 149 frames: **0.181 s vs 0.060 s (3.0×)**; mesh extraction 63 ms vs 5 ms; ~840 MiB GPU memory (nvidia-smi delta) at the first, allocating run (~680 MiB on the synthetic scene); later runs reuse the pool; CPU/GPU meshes agree (vertex→surface median 2 mm, p99 14 mm). On the small synthetic images (256×192) the gain is only 1.3×.
* Only TSDF integrate/extract run on GPU. ORB, matching, loop closure, graph optimisation, colour fusion and point-cloud build are CPU — end-to-end speed-up is therefore much smaller than 3×. CPU is already fast on this data (whole pipeline 12 s).
* Legacy `ScalableTSDFVolume` returned empty meshes in our tests (even a single planar depth map) in this wheel; not used. (Not root-caused.) Note: `VoxelBlockGrid.integrate` takes truncation as a *multiple of voxel size*, and mesh extraction requires `weight > threshold`.
* **nvblox:** no apt/pip package found on this host. Not evaluated beyond that. Likely a good fit for fast GPU ESDF/TSDF *if* Insight 9 provides depth + a pose (VIO) topic; needs confirmation of real depth format/rate and Isaac ROS / CUDA toolkit versions (nvcc is not installed; CUDA toolkit/driver were not touched). Decide after real Sessions exist.

## Known gaps / open issues
1. **RTAB-Map not installed** (`ros-humble-rtabmap-ros` is in the configured apt mirror, but system installs need sudo/approval). The in-repo VIO-seeded pose graph is a stand-in; RTAB-Map integration (VIO as external odometry, stereo, loop closure, graph optimisation, own stereo odometry as control) is only specified in `rtabmap_bridge.py`, **untested**.
2. **Stereo-VO control experiment is weak**: on the synthetic scene it drifts ~0.87 m ATE vs ground truth (SGBM depth + ORB-PnP on 320×240 synthetic textures). It is reported, not tuned; it is not a meaningful RTAB-Map comparison yet.
3. **No real Session**: topic names (`config/topic_map.yaml`), VIO frame ids/child frame, depth encoding (16UC1 mm vs 32FC1, compressed), CameraInfo/TF content, `vio_image` vs `vio_100hz` semantics are all assumptions to confirm. Distortion handling covers plumb_bob only; fisheye (likely for a wide stereo rig) is not implemented.
4. Public datasets: only TUM RGB-D (freiburg1 desk, xyz downloaded). No IMU use at all yet (IMU is read and checked, not fused). No real-VIO benchmark.
5. Loop closure is geometric ORB+depth only (no bag-of-words); candidate search is O(N²) in keyframes with spatial gating — fine for ~100s of keyframes, not for long scans.
6. Mesh colour per vertex only; no texture atlas; OBJ has vertex colours as `v x y z r g b` extension. GLB not done.
7. Reconstruction trajectory is in the VIO odom frame (no gravity alignment). Dynamic objects / outliers handled only by TSDF weighting + statistical outlier removal.
8. MCAP bags: reader supports it, but `rosbag2_storage_mcap` is not installed here, so only SQLite3 Sessions were exercised.
9. `tools/validate_manifest.py` (main) was not exercised; the mapping side validates manifests with `jsonschema` (installed in `.venv`) against `schemas/session_manifest.schema.json`. Contract question for main: the schema has no field for stream-level topic semantics (e.g. which VIO is `vio_image` vs `vio_100hz`) or per-camera frame ids → resolved by name patterns for now.

## Next steps
1. Get a real Session from capture; run `validate`/`quality`, fix assumptions in `topic_map.yaml` (propose verified names to `main`).
2. With approval, install `ros-humble-rtabmap-ros` (+ `ros-humble-rosbag2-storage-mcap`), implement the bridge, compare against the in-repo back-end and RTAB-Map stereo odometry.
3. Fisheye/equidistant camera model if Insight 9 needs it; IMU-based gravity alignment.
4. Texture atlas + GLB export; larger-scene scaling (loop candidates via BoW).
5. Decide nvblox after real data; re-run `bench-gpu` on real resolution/scene size.
