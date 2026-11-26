# mapping/ — offline reconstruction (branch `feat/mapping`)

Consumes **Sessions** (`docs/DATA_CONTRACT.md`, owned by `main`) and produces a trajectory, point cloud and mesh.
No camera connection, no capture-side code. Only `mapping/` is modified on this branch.

## Environment (no system packages were installed)
```
source /opt/ros/humble/setup.bash              # rosbag2_py / rclpy for bag reading
cd mapping && virtualenv --system-site-packages -p /usr/bin/python3 .venv   # python3-venv is not installed
.venv/bin/pip install open3d jsonschema pytest "numpy<2"
```
Tested: Ubuntu 22.04, Python 3.10, ROS 2 Humble, Open3D 0.20.0 (pip, CUDA build), RTX 4090, driver 580 (untouched).

## Usage
```
.venv/bin/python -m insight_mapping.cli validate  <session_dir>
.venv/bin/python -m insight_mapping.cli run       <session_dir> --out <out_dir> [--backend tensor-cuda] [--voxel 0.02]
.venv/bin/python -m insight_mapping.cli bench-gpu <session_dir> --out <out_dir> --trajectory <out_dir>/trajectory.csv
.venv/bin/python -m insight_mapping.cli verify    <out_dir>
.venv/bin/python -m pytest -q                    # all tests (~2.5 min, uses synthetic + TUM fr1/desk if present)
```
`--allow-missing imu` downgrades a missing mandatory stream (needed only for public datasets without IMU).

## Pipeline
| Stage | Module | Notes |
|---|---|---|
| Session load/validate | `session.py` | schema from `../schemas/`, requires `finalized`; logical streams (rgb/depth/left/right/vio_image/vio_100hz/imu/…) resolved from `manifest.streams` via `config/topic_map.yaml` (provisional until `main` verifies real topic names) |
| Bag parse / extract | `bagio.py`, `extract.py` | MCAP or SQLite3; original ROS header stamps kept in every CSV/file name; depth → uint16 mm |
| Quality report | `quality.py` | rates, gaps, drops, header-vs-bag time, image↔VIO / RGB↔Depth / L↔R sync, VIO jumps, intrinsics sanity, TF connectivity, stereo baseline (P vs TF), **RGB/Depth alignment verdict** |
| Frame validation | `slam/frames.py` | tests 4 hypotheses of VIO pose / extrinsic direction against vision (PnP); a mismatch aborts the run |
| VIO → camera trajectory | `slam/vio.py` | `T_odom_cam = T_odom_body @ T_body_cam`; `vio_image` paired within 5 ms, else `vio_100hz` interpolation, never extrapolated |
| Loop closure + pose graph | `slam/pipeline.py`, `slam/vo.py`, `slam/posegraph.py` | VIO-seeded keyframe graph, ORB + depth-PnP verified loops, robust SE(3) graph optimisation (in-repo back-end) |
| RTAB-Map | `rtabmap_bridge.py` | **not installed / not run** — availability check + reviewable launch command only |
| Reconstruction | `recon/fusion.py` | Open3D `VoxelBlockGrid` TSDF (CPU or CUDA), mesh, view-weighted vertex colours, coloured point cloud |
| GPU evaluation | `recon/bench.py` | measured CPU vs CUDA, GPU memory, equivalence of meshes |

### RGB/Depth rule
RGB and Depth are never assumed aligned. Geometry is integrated from depth in the **depth camera frame**
(`T_world_depth = T_world_rgb(t_depth) @ T_rgb_depth`, TF extrinsic, pose interpolated to the depth stamp);
colour is attached afterwards by projecting into the RGB image through the calibration with a z-buffer visibility test.
If alignment cannot be proven *and* there is no TF between the frames, the run refuses.

## Output (per reconstruction)
`trajectory.csv` (`stamp_ns,tx,ty,tz,qx,qy,qz,qw`, `T_odom_rgbcam`, optical frame), `map.ply`, `map.pcd`, `mesh.ply`, `mesh.obj`,
`reconstruction_report.json` (time, points, faces, RSS, GPU memory, drift/ATE/RPE if ground truth, loop closures, anomalies,
data-quality summary, output verification). GLB later. Intermediate: `extracted/`, `slam/`, `quality_report.json`.
Mesh colours are per-vertex (no texture atlas yet).
