"""P1-D: Open3D reconstruction from RGB / Depth / optimised trajectory.

Invariant: RGB and Depth are NEVER fed together to an RGB-D TSDF unless proven pixel-aligned. Geometry is always
integrated from depth in the *depth camera's own frame* (pose = T_world_rgbcam @ T_rgb_depth, at the depth stamp);
colour is attached afterwards by projecting geometry into the RGB image through the calibration transform with a
z-buffer visibility test.  (For aligned data this is mathematically the same as RGB-D integration.)
"""
from __future__ import annotations

import json
import resource
import subprocess
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from .. import se3
from ..calib import CameraModel, rgbd_alignment
from ..extract import Extracted
from ..session import Session
from ..slam import pipeline as slam_pipeline
from ..slam import vo


@dataclass
class ReconConfig:
    voxel: float = 0.02
    sdf_trunc: float = 0.08
    depth_max: float = 6.0
    depth_min: float = 0.2
    stride: int = 2                   # use every Nth frame
    pc_pixel_stride: int = 3          # pixel subsampling for the point-cloud map
    map_voxel: float = 0.02
    max_pair_dt_ms: float = 33.0      # rgb<->depth pairing tolerance
    backend: str = "tensor-cpu"       # tensor-cpu | tensor-cuda
    min_weight: float = 2.0           # TSDF observations required (Open3D compares weight > threshold)
    block_count: int = 20000
    color_views: int = 60
    cleanup_min_triangles: int = 200
    trajectory: str = "optimized"     # optimized | vio


def gpu_mem_used_mib():
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout.strip().splitlines()
        return float(out[0])
    except Exception:
        return None


def rss_mib():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def undistort_pair(img, cam: CameraModel, interp):
    if not cam.has_distortion or cam.distortion_model not in ("plumb_bob", "rational_polynomial"):
        return img
    return cv2.undistort(img, cam.K_mat, np.asarray(cam.D, dtype=np.float64), None, cam.K_mat) if interp == cv2.INTER_LINEAR \
        else cv2.remap(img, *cv2.initUndistortRectifyMap(cam.K_mat, np.asarray(cam.D, dtype=np.float64), None, cam.K_mat,
                                                          (cam.width, cam.height), cv2.CV_16SC2), interp)


class FrameSet:
    """Paired RGB/Depth frames with poses. Handles aligned and unaligned data uniformly."""

    def __init__(self, session: Session, ex: Extracted, traj_stamps, traj_T, cfg: ReconConfig):
        self.ex, self.cfg = ex, cfg
        self.use_stereo = not ("rgb" in session.streams and "depth" in session.streams)
        self.anomalies: list = []
        if self.use_stereo:
            if not ("left" in session.streams and "right" in session.streams):
                raise RuntimeError("need rgb+depth or left+right in the Session")
            self.ci, self.cam_c = ex.index("left"), ex.cams["left_info"]
            self.ri = ex.index("right")
            self.di, self.cam_d, self.T_c_d = None, self.cam_c, np.eye(4)
            from ..calib import stereo_baseline
            self.B = stereo_baseline(self.cam_c, ex.cams["right_info"])
            self.aligned = True
            self.anomalies.append("no depth stream: using SGBM stereo depth (lower quality)")
        else:
            self.ci, self.cam_c = ex.index("rgb"), ex.cams["rgb_info"]
            self.di, self.cam_d = ex.index("depth"), ex.cams["depth_info"]
            T = ex.tf.lookup(self.cam_c.frame_id, self.cam_d.frame_id) if ex.tf.can_lookup(self.cam_c.frame_id, self.cam_d.frame_id) else None
            al = rgbd_alignment(self.cam_c, self.cam_d, T)
            self.aligned = al["aligned"]
            self.align_info = al
            if T is None:
                if not self.aligned:
                    raise RuntimeError("RGB and Depth cannot be proven aligned and there is no TF between them: refusing to fuse")
                T = np.eye(4)
            self.T_c_d = T
        self.traj_stamps, self.traj_T = traj_stamps, traj_T
        base = self.ci["stamp_ns"]
        lo, hi = traj_stamps[0], traj_stamps[-1]
        self.items = []
        skipped_pair = skipped_traj = 0
        for k in range(0, len(base), cfg.stride):
            ts = int(base[k])
            if self.use_stereo:
                j = int(np.argmin(np.abs(self.ri["stamp_ns"] - ts)))
                tsd = ts
            else:
                j = int(np.argmin(np.abs(self.di["stamp_ns"] - ts)))
                tsd = int(self.di["stamp_ns"][j])
                if abs(tsd - ts) / 1e6 > cfg.max_pair_dt_ms:
                    skipped_pair += 1
                    continue
            if not (lo <= ts <= hi and lo <= tsd <= hi):
                skipped_traj += 1
                continue
            self.items.append((k, j, ts, tsd))
        if skipped_pair:
            self.anomalies.append(f"{skipped_pair} RGB frames without a depth frame within {cfg.max_pair_dt_ms} ms")
        if skipped_traj:
            self.anomalies.append(f"{skipped_traj} frames outside the trajectory time range")
        self.times = np.array([i[2] for i in self.items])

    def pose_c(self, ts):
        T, *_ = se3.interpolate_poses(self.traj_stamps.astype(np.float64), self.traj_T, np.array([ts], dtype=np.float64))
        return T[0]

    def load(self, n):
        """-> dict(color BGR, depth_m (depth cam), T_w_c at rgb stamp, T_w_d at depth stamp, regd (depth in colour cam))."""
        k, j, ts, tsd = self.items[n]
        ex = self.ex
        color = ex.read(self.ci["file"][k])
        if self.use_stereo:
            left = color if color.ndim == 2 else cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            right = ex.read(self.ri["file"][j], 0)
            depth = vo.depth_from_stereo(left, right, self.cam_c.fx, self.B, self.cfg.depth_max)
            color = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR) if color.ndim == 2 else color
        else:
            depth = ex.read(self.di["file"][j]).astype(np.float32) / 1000.0
            depth = undistort_pair(depth, self.cam_d, cv2.INTER_NEAREST)
        color = undistort_pair(color, self.cam_c, cv2.INTER_LINEAR)
        depth[(depth < self.cfg.depth_min) | (depth > self.cfg.depth_max)] = 0
        T_w_c = self.pose_c(ts)
        T_w_d = self.pose_c(tsd) @ self.T_c_d
        regd = depth if self.aligned else vo.register_depth_to_rgb(depth, self.cam_d, self.cam_c, self.T_c_d)
        return {"color": color, "depth": depth, "T_w_c": T_w_c, "T_w_d": T_w_d, "regd": regd}


def backproject(depth_m, cam: CameraModel, pixel_stride=1):
    v, u = np.mgrid[0:depth_m.shape[0]:pixel_stride, 0:depth_m.shape[1]:pixel_stride]
    z = depth_m[v, u].astype(np.float64)
    m = z > 0
    u, v, z = u[m], v[m], z[m]
    return np.stack([(u - cam.cx) * z / cam.fx, (v - cam.cy) * z / cam.fy, z], 1)


def sample_colors(P_w, T_w_c, cam: CameraModel, color, regd, normals_w=None, tol=0.05):
    """Project world points into the colour camera. Returns (rgb float 0..1 (N,3), weight (N,)); weight 0 = unseen."""
    P_c = (P_w - T_w_c[:3, 3]) @ T_w_c[:3, :3]
    z = P_c[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = P_c[:, 0] * cam.fx / z + cam.cx
        v = P_c[:, 1] * cam.fy / z + cam.cy
    ui, vi = np.round(u).astype(int), np.round(v).astype(int)
    ok = (z > 0.05) & (ui >= 0) & (ui < cam.width) & (vi >= 0) & (vi < cam.height)
    w = np.zeros(len(P_w))
    col = np.zeros((len(P_w), 3))
    idx = np.nonzero(ok)[0]
    zd = regd[vi[idx], ui[idx]]
    vis = (zd > 0) & (np.abs(zd - z[idx]) < tol + 0.03 * z[idx])
    idx = idx[vis]
    col[idx] = color[vi[idx], ui[idx], ::-1] / 255.0   # BGR -> RGB
    ww = 1.0 / np.maximum(z[idx], 0.3)
    if normals_w is not None:
        ray = -P_c[idx] / np.linalg.norm(P_c[idx], axis=1, keepdims=True)
        n_c = normals_w[idx] @ T_w_c[:3, :3]
        cosv = np.abs(np.sum(ray * n_c, 1))
        ww = ww * np.clip(cosv, 0.1, 1.0)
    w[idx] = ww
    return col, w


def _intr(cam: CameraModel):
    return o3d.camera.PinholeCameraIntrinsic(cam.width, cam.height, cam.fx, cam.fy, cam.cx, cam.cy)


class TSDF:
    """TSDF on Open3D t.geometry.VoxelBlockGrid. backend tensor-cpu = CPU reference, tensor-cuda = GPU.

    (The legacy ScalableTSDFVolume returned empty meshes in our synthetic tests with Open3D 0.20.0 pip wheel, even
    for a single planar depth image, while UniformTSDFVolume worked; it is therefore not used.)
    """

    def __init__(self, cfg: ReconConfig, cam_d: CameraModel):
        self.cfg, self.cam = cfg, cam_d
        if cfg.backend not in ("tensor-cpu", "tensor-cuda"):
            raise ValueError(f"unknown backend {cfg.backend}")
        dev = o3d.core.Device("CUDA:0" if cfg.backend == "tensor-cuda" else "CPU:0")
        if cfg.backend == "tensor-cuda" and not o3d.core.cuda.is_available():
            raise RuntimeError("tensor-cuda requested but this Open3D build has no usable CUDA device")
        self.dev = dev
        self.v = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight"), attr_dtypes=(o3d.core.float32, o3d.core.float32), attr_channels=((1), (1)),
            voxel_size=cfg.voxel, block_resolution=16, block_count=cfg.block_count, device=dev)
        self.intr = o3d.core.Tensor(cam_d.K_mat, o3d.core.float64)
        self.mult = cfg.sdf_trunc / cfg.voxel     # NB: Open3D takes truncation as a multiple of voxel size

    def integrate(self, depth_m, T_w_d):
        d = o3d.t.geometry.Image(o3d.core.Tensor(np.round(depth_m * 1000).astype(np.uint16), device=self.dev))
        ext = o3d.core.Tensor(np.linalg.inv(T_w_d), o3d.core.float64)
        dmax = self.cfg.depth_max * 1.5
        coords = self.v.compute_unique_block_coordinates(d, self.intr, ext, 1000.0, dmax, self.mult)
        self.v.integrate(coords, d, self.intr, ext, 1000.0, dmax, self.mult)

    def mesh(self):
        m = self.v.extract_triangle_mesh(weight_threshold=self.cfg.min_weight)
        return m.to_legacy()


def clean_mesh(mesh, min_tri):
    mesh.remove_degenerate_triangles(); mesh.remove_duplicated_vertices(); mesh.remove_unreferenced_vertices()
    cl, n, _ = mesh.cluster_connected_triangles()
    cl, n = np.asarray(cl), np.asarray(n)
    mesh.remove_triangles_by_mask(n[cl] < min_tri)
    mesh.remove_unreferenced_vertices()
    return mesh


def color_mesh(mesh, fs: FrameSet, n_views: int, log=print):
    mesh.compute_vertex_normals()
    V, N = np.asarray(mesh.vertices), np.asarray(mesh.vertex_normals)
    acc, wsum = np.zeros((len(V), 3)), np.zeros(len(V))
    sel = np.linspace(0, len(fs.items) - 1, min(n_views, len(fs.items))).astype(int)
    for n in sel:
        f = fs.load(int(n))
        c, w = sample_colors(V, f["T_w_c"], fs.cam_c, f["color"], f["regd"], N)
        acc += c * w[:, None]; wsum += w
    seen = wsum > 0
    col = np.full((len(V), 3), 0.5)
    col[seen] = acc[seen] / wsum[seen, None]
    mesh.vertex_colors = o3d.utility.Vector3dVector(col)
    return float(seen.mean())


def build_pointcloud(fs: FrameSet, cfg: ReconConfig, log=print):
    chunks_p, chunks_c = [], []
    pcd_all = o3d.geometry.PointCloud()
    for n in range(len(fs.items)):
        f = fs.load(n)
        P_d = backproject(f["depth"], fs.cam_d, cfg.pc_pixel_stride)
        if len(P_d) == 0:
            continue
        P_w = P_d @ f["T_w_d"][:3, :3].T + f["T_w_d"][:3, 3]
        col, w = sample_colors(P_w, f["T_w_c"], fs.cam_c, f["color"], f["regd"])
        keep = w > 0
        p = o3d.geometry.PointCloud()
        p.points = o3d.utility.Vector3dVector(P_w[keep])
        p.colors = o3d.utility.Vector3dVector(col[keep])
        pcd_all += p.voxel_down_sample(cfg.map_voxel)
        if n % 20 == 19:
            pcd_all = pcd_all.voxel_down_sample(cfg.map_voxel)
    pcd = pcd_all.voxel_down_sample(cfg.map_voxel)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=3.0)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=cfg.map_voxel * 3, max_nn=30))
    return pcd


def reconstruct(session: Session, ex: Extracted, traj_csv, out_dir, cfg: ReconConfig | None = None, log=print) -> dict:
    cfg = cfg or ReconConfig()
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    stamps, T = slam_pipeline.read_trajectory_csv(traj_csv)
    fs = FrameSet(session, ex, stamps, T, cfg)
    rep = {"config": asdict(cfg), "frames_used": len(fs.items), "rgbd_aligned": bool(fs.aligned),
           "depth_source": "stereo_sgbm" if fs.use_stereo else "depth_stream", "anomalies": list(fs.anomalies),
           "trajectory_file": str(Path(traj_csv).name)}
    if not fs.aligned:
        rep["rgbd_alignment"] = fs.align_info
        rep["anomalies"].append("RGB/Depth not aligned: depth integrated in its own frame; colour mapped via calibration")
    if not fs.items:
        raise RuntimeError("no usable frames for reconstruction")
    gpu0 = gpu_mem_used_mib()
    timings = {}
    t0 = time.time()
    vol = TSDF(cfg, fs.cam_d)
    for n in range(len(fs.items)):
        f = fs.load(n)
        vol.integrate(f["depth"], f["T_w_d"])
    timings["tsdf_integrate_s"] = time.time() - t0
    t1 = time.time()
    mesh = clean_mesh(vol.mesh(), cfg.cleanup_min_triangles)
    timings["mesh_extract_s"] = time.time() - t1
    gpu1 = gpu_mem_used_mib()
    t2 = time.time()
    rep["mesh_vertex_color_coverage"] = color_mesh(mesh, fs, cfg.color_views, log)
    timings["color_fusion_s"] = time.time() - t2
    t3 = time.time()
    pcd = build_pointcloud(fs, cfg, log)
    timings["pointcloud_s"] = time.time() - t3
    o3d.io.write_point_cloud(str(out / "map.ply"), pcd)
    o3d.io.write_point_cloud(str(out / "map.pcd"), pcd)
    o3d.io.write_triangle_mesh(str(out / "mesh.ply"), mesh, write_vertex_normals=True)
    o3d.io.write_triangle_mesh(str(out / "mesh.obj"), mesh, write_vertex_normals=True)
    rep.update({"timings_s": timings, "points": len(pcd.points), "mesh_vertices": len(mesh.vertices),
                "mesh_triangles": len(mesh.triangles), "peak_rss_mib": rss_mib(),
                "gpu_mem_used_mib": {"before": gpu0, "after_tsdf": gpu1,
                                     "delta_tsdf": None if gpu0 is None or gpu1 is None else gpu1 - gpu0},
                "elapsed_s": time.time() - t0})
    bb = pcd.get_axis_aligned_bounding_box()
    rep["map_extent_m"] = (bb.get_max_bound() - bb.get_min_bound()).tolist()
    log(f"recon: {rep['points']} pts, {rep['mesh_triangles']} tris, {rep['elapsed_s']:.1f}s")
    return rep
