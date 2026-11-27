"""Stage 5 - dense stereo depth -> TSDF fusion -> point cloud / mesh, plus reference-free quality metrics.

Depth comes from SGBM disparity (cached by stereo.py): Z = fx * B / d with B = -P_right[0,3]/fx. Pixels are rejected by
range, a depth-edge test and a per-pixel uncertainty gate sigma_Z = Z^2 * sigma_d / (fx B).
Colour is the left infrared intensity (grey); RGB texture is a later optimisation (RGB<->left extrinsic is only [O]).
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from .. import se3
from ..calib import CameraModel
from ..recon import fusion
from .rig import Rig
from .stereo import StereoCache


@dataclass
class DenseParams:
    stride: int = 3
    z_min: float = 0.3
    z_max: float = 5.0
    sigma_d_px: float = 0.25
    max_sigma_z_rel: float = 0.04      # reject if sigma_Z / Z above this (=> disparity too small)
    edge_rel: float = 0.05             # reject where |dZ|/Z between neighbours exceeds this
    erode_px: int = 2
    voxel: float = 0.02
    sdf_trunc: float = 0.08
    map_voxel: float = 0.02
    pc_pixel_stride: int = 2
    backend: str = "tensor-cuda"
    cleanup_min_triangles: int = 200
    min_weight: float = 2.0


def rig_camera(rig: Rig) -> CameraModel:
    K = rig.K
    return CameraModel("camera_camera_left", rig.left.width, rig.left.height, K.ravel().tolist(), [0.0] * 5, "plumb_bob",
                       [K[0, 0], 0, K[0, 2], 0, 0, K[1, 1], K[1, 2], 0, 0, 0, 1, 0])


def depth_from_disp(disp: np.ndarray, rig: Rig, p: DenseParams) -> np.ndarray:
    with np.errstate(divide="ignore", invalid="ignore"):
        z = np.where(disp >= 1.0, rig.fx * rig.baseline / disp, 0.0).astype(np.float32)
    bad = (z < p.z_min) | (z > p.z_max) | ~np.isfinite(z)
    sig = z * z * p.sigma_d_px / (rig.fx * rig.baseline)
    bad |= sig > p.max_sigma_z_rel * z
    # depth edges (flying pixels)
    zz = np.where(bad, np.nan, z)
    dx = np.abs(zz[:, 1:] - zz[:, :-1]); dy = np.abs(zz[1:, :] - zz[:-1, :])
    edge = np.zeros_like(bad)
    ex = dx > p.edge_rel * np.minimum(zz[:, 1:], zz[:, :-1])
    ey = dy > p.edge_rel * np.minimum(zz[1:, :], zz[:-1, :])
    edge[:, 1:] |= ex; edge[:, :-1] |= ex; edge[1:, :] |= ey; edge[:-1, :] |= ey
    bad |= edge
    if p.erode_px:
        k = np.ones((2 * p.erode_px + 1,) * 2, np.uint8)
        bad = cv2.dilate(bad.astype(np.uint8), k).astype(bool)
    z = z.copy(); z[bad] = 0
    return z


def load_trajectory(path):
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return a[:, 0].astype(np.int64), np.stack([se3.make_T(r[1:4], r[4:8]) for r in a])


def _backproject(z, cam, ps):
    v, u = np.mgrid[0:z.shape[0]:ps, 0:z.shape[1]:ps]
    zz = z[v, u].astype(np.float64)
    m = zz > 0
    u, v, zz = u[m], v[m], zz[m]
    return np.stack([(u - cam.cx) * zz / cam.fx, (v - cam.cy) * zz / cam.fy, zz], 1), (v, u)


def reconstruct(cache: StereoCache, rig: Rig, traj_csv, out_dir, p: DenseParams | None = None, log=print) -> dict:
    p = p or DenseParams()
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    cam = rig_camera(rig)
    ts, Tw = load_trajectory(traj_csv)
    pos = {int(s): T for s, T in zip(ts, Tw)}
    idx = [i for i in range(0, len(cache), p.stride) if int(cache.stamps[i]) in pos]
    if not idx:
        raise RuntimeError("no frames with trajectory poses")
    rc = fusion.ReconConfig(voxel=p.voxel, sdf_trunc=p.sdf_trunc, depth_max=p.z_max, depth_min=p.z_min, backend=p.backend,
                            min_weight=p.min_weight, map_voxel=p.map_voxel, cleanup_min_triangles=p.cleanup_min_triangles)
    t0 = time.time()
    gpu0 = fusion.gpu_mem_used_mib()
    vol = fusion.TSDF(rc, cam)
    pcd_all = o3d.geometry.PointCloud()
    valid_frac = []
    t_depth = t_int = 0.0
    for n, i in enumerate(idx):
        ta = time.time()
        z = depth_from_disp(cache.disp(i), rig, p)
        t_depth += time.time() - ta
        valid_frac.append(float((z > 0).mean()))
        T = pos[int(cache.stamps[i])]
        tb = time.time()
        vol.integrate(z, T)
        t_int += time.time() - tb
        P, (v, u) = _backproject(z, cam, p.pc_pixel_stride)
        if len(P):
            g = cache.left(i)[v, u] if False else None
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(P @ T[:3, :3].T + T[:3, 3])
            gl = cache.left(i)
            vv = np.clip(np.round(P[:, 1] * cam.fy / P[:, 2] + cam.cy).astype(int), 0, gl.shape[0] - 1)
            uu = np.clip(np.round(P[:, 0] * cam.fx / P[:, 2] + cam.cx).astype(int), 0, gl.shape[1] - 1)
            gv = gl[vv, uu].astype(np.float64) / 255.0
            pc.colors = o3d.utility.Vector3dVector(np.repeat(gv[:, None], 3, 1))
            pcd_all += pc.voxel_down_sample(p.map_voxel)
            if n % 25 == 24:
                pcd_all = pcd_all.voxel_down_sample(p.map_voxel)
        if n % 200 == 0 and n:
            log(f"  dense {n}/{len(idx)}")
    t1 = time.time()
    mesh = fusion.clean_mesh(vol.mesh(), p.cleanup_min_triangles)
    t_mesh = time.time() - t1
    gpu1 = fusion.gpu_mem_used_mib()
    # grey vertex colours from the sampled views
    mesh.compute_vertex_normals()
    V, N = np.asarray(mesh.vertices), np.asarray(mesh.vertex_normals)
    acc, wsum = np.zeros((len(V), 3)), np.zeros(len(V))
    sel = np.linspace(0, len(idx) - 1, min(60, len(idx))).astype(int)
    for s in sel:
        i = idx[s]
        z = depth_from_disp(cache.disp(i), rig, p)
        g = cache.left(i)
        color = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        c, w = fusion.sample_colors(V, pos[int(cache.stamps[i])], cam, color, z, N)
        acc += c * w[:, None]; wsum += w
    col = np.full((len(V), 3), 0.5); seen = wsum > 0
    col[seen] = acc[seen] / wsum[seen, None]
    mesh.vertex_colors = o3d.utility.Vector3dVector(col)
    pcd = pcd_all.voxel_down_sample(p.map_voxel)
    n0 = len(pcd.points)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=3.0)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=p.map_voxel * 3, max_nn=30))
    o3d.io.write_point_cloud(str(out / "map.ply"), pcd)
    o3d.io.write_point_cloud(str(out / "map.pcd"), pcd)
    o3d.io.write_triangle_mesh(str(out / "mesh.ply"), mesh, write_vertex_normals=True)
    bb = pcd.get_axis_aligned_bounding_box()
    rep = {"params": asdict(p), "frames_used": len(idx), "points": len(pcd.points), "points_before_outlier_filter": n0,
           "mesh_vertices": len(mesh.vertices), "mesh_triangles": len(mesh.triangles),
           "mesh_vertex_color_coverage": float(seen.mean()), "map_extent_m": (bb.get_max_bound() - bb.get_min_bound()).tolist(),
           "valid_depth_fraction": {"median": float(np.median(valid_frac)), "p5": float(np.percentile(valid_frac, 5))},
           "timings_s": {"depth": t_depth, "tsdf_integrate": t_int, "mesh_extract": t_mesh, "total": time.time() - t0},
           "gpu_mem_used_mib": {"before": gpu0, "after": gpu1}, "colour": "left infrared intensity (grey)"}
    log(f"dense: {rep['points']} pts, {rep['mesh_triangles']} tris, {rep['timings_s']['total']:.1f}s")
    return rep


# ------------------------------------------------------------------ reference-free quality metrics
def depth_consistency(cache: StereoCache, rig: Rig, traj_csv, p: DenseParams | None = None, gap=6, n_pairs=40) -> dict:
    """Warp frame i's depth into frame i+gap with the estimated pose and compare with frame i+gap's own stereo depth.
    Mixes depth noise and pose error; a drifting/incorrect trajectory shows up as a growing relative error."""
    p = p or DenseParams()
    cam = rig_camera(rig)
    ts, Tw = load_trajectory(traj_csv)
    pos = {int(s): T for s, T in zip(ts, Tw)}
    pairs = [(i, i + gap) for i in np.linspace(0, len(cache) - gap - 1, n_pairs).astype(int)]
    rel_err, cover, rows = [], [], []
    for a, b in pairs:
        za = depth_from_disp(cache.disp(a), rig, p); zb = depth_from_disp(cache.disp(b), rig, p)
        Ta, Tb = pos[int(cache.stamps[a])], pos[int(cache.stamps[b])]
        P, (v, u) = _backproject(za, cam, 2)
        if len(P) < 500:
            continue
        Pb = (P @ Ta[:3, :3].T + Ta[:3, 3] - Tb[:3, 3]) @ Tb[:3, :3]
        ok = Pb[:, 2] > 0.1
        ub = np.round(Pb[ok, 0] * cam.fx / Pb[ok, 2] + cam.cx).astype(int)
        vb = np.round(Pb[ok, 1] * cam.fy / Pb[ok, 2] + cam.cy).astype(int)
        inside = (ub >= 0) & (ub < cam.width) & (vb >= 0) & (vb < cam.height)
        zq = zb[vb[inside], ub[inside]]
        zp = Pb[ok][inside, 2]
        m = zq > 0
        if m.sum() < 300:
            continue
        e = np.abs(zp[m] - zq[m]) / zq[m]
        rel_err.append(float(np.median(e))); cover.append(float(m.sum() / len(P)))
        rows.append({"a": int(a), "b": int(b), "median_rel_err": float(np.median(e)), "p90_rel_err": float(np.percentile(e, 90)),
                     "overlap": float(m.sum() / len(P))})
    if not rel_err:
        return {"pairs": 0}
    return {"gap_frames": gap, "pairs": len(rel_err), "median_rel_depth_err": float(np.median(rel_err)),
            "p90_of_pair_medians": float(np.percentile(rel_err, 90)), "median_overlap": float(np.median(cover)), "per_pair": rows}


def map_planarity(pcd_path, n_planes=4, dist=0.02) -> dict:
    """Dominant planes of the fused map (walls/floors): RMS residual to the plane = map noise + residual drift."""
    pc = o3d.io.read_point_cloud(str(pcd_path))
    if len(pc.points) < 2000:
        return {"planes": []}
    planes = []
    rest = pc
    for _ in range(n_planes):
        if len(rest.points) < 1000:
            break
        model, inl = rest.segment_plane(dist, 3, 1500)
        pts = np.asarray(rest.select_by_index(inl).points)
        if len(pts) < 500:
            break
        r = pts @ np.array(model[:3]) + model[3]
        planes.append({"inliers": int(len(pts)), "normal": [float(x) for x in model[:3]], "rms_mm": float(np.sqrt(np.mean(r ** 2)) * 1e3),
                       "fraction_of_map": float(len(pts) / len(pc.points))})
        rest = rest.select_by_index(inl, invert=True)
    return {"inlier_threshold_m": dist, "planes": planes}
