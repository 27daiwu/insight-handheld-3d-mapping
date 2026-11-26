"""Synthetic handheld scan: ray-cast box room (+ sphere) with hashed-colour block texture.

Generates a full Session with RGB, Depth (optionally NOT aligned with RGB), stereo, IMU, VIO (with
injected smooth drift), CameraInfo and TF — exercising every code path without a camera.
Ground truth is returned for evaluation.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .. import se3
from ..calib import CameraModel
from .sessionwriter import SessionBuilder

ROOM_LO = np.array([-2.5, -2.0, -1.25])
ROOM_HI = np.array([2.5, 2.0, 1.25])
SPHERE_C, SPHERE_R = np.array([0.0, 0.0, -0.75]), 0.5     # object in the centre of the circle
R_BODY_OPT = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])  # optical axes expressed in FLU body
CELL = 0.10


def _hash_color(ids: np.ndarray) -> np.ndarray:
    h = (ids[..., 0] * 73856093) ^ (ids[..., 1] * 19349663) ^ (ids[..., 2] * 83492791) ^ (ids[..., 3] * 2654435761)
    h = (h & 0xFFFFFFFF).astype(np.uint32)
    r = (h >> 3) & 0xFF; g = (h >> 11) & 0xFF; b = (h >> 19) & 0xFF
    return np.stack([b, g, r], -1).astype(np.uint8)  # BGR


def render(T_world_cam: np.ndarray, cam: CameraModel, gray=False, noise_mm=0.0, rng=None):
    """-> (bgr uint8 HxWx3 or gray, depth_mm uint16 HxW). Camera frame is optical (+z forward)."""
    H, W = cam.height, cam.width
    u, v = np.meshgrid(np.arange(W) + 0.0, np.arange(H) + 0.0)
    d_cam = np.stack([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, np.ones_like(u)], -1)  # z=1 => t is depth
    R, o = T_world_cam[:3, :3], T_world_cam[:3, 3]
    d = d_cam @ R.T
    with np.errstate(divide="ignore", invalid="ignore"):
        tl = (ROOM_LO - o) / d
        th = (ROOM_HI - o) / d
    t_ax = np.where(d > 0, th, tl)
    t_wall = t_ax.min(-1)
    wall_axis = t_ax.argmin(-1)
    # sphere
    oc = o - SPHERE_C
    a = (d ** 2).sum(-1); b = 2 * (d @ oc); c = oc @ oc - SPHERE_R ** 2
    disc = b * b - 4 * a * c
    ts = np.where(disc > 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
    ts = np.where(ts > 0, ts, np.inf)
    hit_sphere = ts < t_wall
    t = np.where(hit_sphere, ts, t_wall)
    P = o + d * t[..., None]
    cell = np.floor(P / CELL).astype(np.int64)
    ax = np.where(hit_sphere, 3, wall_axis)
    ids = np.concatenate([cell, ax[..., None]], -1)
    col = _hash_color(ids)
    # checker darkening for strong corners
    chk = ((cell.sum(-1)) % 2 == 0)
    col = np.where(chk[..., None], col, (col * 0.55).astype(np.uint8))
    fine = np.concatenate([np.floor(P / (CELL / 3)).astype(np.int64), ax[..., None]], -1)
    col = np.clip(col.astype(np.int16) + (_hash_color(fine).astype(np.int16) - 128) // 3, 0, 255).astype(np.uint8)
    depth_mm = t * 1000.0
    if noise_mm > 0:
        depth_mm = depth_mm + (rng or np.random.default_rng(0)).normal(0, noise_mm, depth_mm.shape)
    depth = np.clip(np.round(depth_mm), 0, 65535).astype(np.uint16)
    if gray:
        col = (0.114 * col[..., 0] + 0.587 * col[..., 1] + 0.299 * col[..., 2]).astype(np.uint8)
    return col, depth


def render_color(T_world_cam, cam: CameraModel, gray=False, ss=3):
    """Anti-aliased colour render (ss x ss supersampling) so frame-to-frame feature matching is realistic."""
    big = cam.scaled(cam.width * ss, cam.height * ss)
    # scaled() maps pixel centres as K*s; keep sub-pixel alignment: shift principal point by (ss-1)/2
    K = list(big.K); K[2] += (ss - 1) / 2.0 - (ss - 1) / 2.0
    col, _ = render(T_world_cam, big, gray=False)
    col = col.reshape(cam.height, ss, cam.width, ss, 3).astype(np.float32).mean((1, 3))
    if gray:
        col = 0.114 * col[..., 0] + 0.587 * col[..., 1] + 0.299 * col[..., 2]
    return np.clip(np.round(col), 0, 255).astype(np.uint8)


def trajectory(n_frames=240, overlap=20, radius=1.3, fps=10.0, t0_ns=1_700_000_000_000_000_000):
    """Circle around the sphere, camera looking tangentially-outward (yaw rotates with the walk), small bob.
    Returns stamps_ns (N,), T_world_body (N,4,4) for FLU body."""
    n_circle = n_frames - overlap
    k = np.arange(n_frames)
    ang = 2 * np.pi * k / n_circle
    pos = np.stack([radius * np.cos(ang), radius * np.sin(ang), 0.15 * np.sin(3 * ang)], 1)
    yaw = ang + np.pi / 2 - 0.8                      # forward ≈ tangent, rotated outward
    pitch = 0.1 * np.sin(2 * ang)
    Ts = np.tile(np.eye(4), (n_frames, 1, 1))
    Ts[:, :3, :3] = Rot.from_euler("zyx", np.stack([yaw, pitch, np.zeros_like(yaw)], 1)).as_matrix()
    Ts[:, :3, 3] = pos
    stamps = t0_ns + (k * 1e9 / fps).astype(np.int64)
    return stamps, Ts


def trajectory_continuous(stamps, Ts, query_ns):
    T, *_ = se3.interpolate_poses(stamps.astype(np.float64), Ts, query_ns.astype(np.float64))
    return T


def drift(T_true, t_s, trans_rate=(0.03, -0.02, 0.005), yaw_rate_deg=0.35):
    """Smooth VIO drift D(t) applied to the true world pose: T_vio = D(t) @ T_true."""
    D = np.tile(np.eye(4), (len(t_s), 1, 1))
    D[:, :3, :3] = Rot.from_euler("z", np.radians(yaw_rate_deg) * t_s).as_matrix()
    D[:, :3, 3] = np.outer(t_s, trans_rate)
    return D @ T_true


def make_cameras(aligned: bool, size=(320, 240)):
    W, H = size
    f = 0.8 * W
    rgb = CameraModel("rgb_optical", W, H, [f, 0, W / 2 - 0.5, 0, f, H / 2 - 0.5, 0, 0, 1], [0.0] * 5)
    rgb.P = [f, 0, W / 2 - 0.5, 0, 0, f, H / 2 - 0.5, 0, 0, 0, 1, 0]
    if aligned:
        dep = CameraModel("rgb_optical", W, H, list(rgb.K), [0.0] * 5, P=list(rgb.P))
    else:
        w2, h2 = 256, 192
        fd = 0.75 * w2
        dep = CameraModel("depth_optical", w2, h2, [fd, 0, w2 / 2 - 0.5, 0, fd, h2 / 2 - 0.5, 0, 0, 1], [0.0] * 5)
        dep.P = [fd, 0, w2 / 2 - 0.5, 0, 0, fd, h2 / 2 - 0.5, 0, 0, 0, 1, 0]
    left = CameraModel("left_optical", W, H, list(rgb.K), [0.0] * 5, P=list(rgb.P))
    right = CameraModel("right_optical", W, H, list(rgb.K), [0.0] * 5, P=list(rgb.P))
    B = 0.06
    right.P = list(rgb.P); right.P[3] = -f * B
    return rgb, dep, left, right, B


def make_session(root, session_id="session_synth", n_frames=240, aligned=False, with_drift=True, seed=0,
                 storage="sqlite3", size=(320, 240), drop_vio_100hz=False, drop_imu=False, vio_pose_inverted=False):
    rng = np.random.default_rng(seed)
    rgb, dep, left, right, B = make_cameras(aligned, size)
    stamps, T_wb = trajectory(n_frames)
    # static extrinsics: T_body_cam = [R_BODY_OPT | offset]
    def Tbc(off, extra_rot=None):
        T = np.eye(4); T[:3, :3] = R_BODY_OPT if extra_rot is None else R_BODY_OPT @ extra_rot
        T[:3, 3] = off; return T
    T_b_rgb = Tbc([0.06, 0.0, 0.03])
    T_b_dep = Tbc([0.06, -0.025, 0.03], Rot.from_euler("xyz", [0.4, -0.3, 0.2], degrees=True).as_matrix()) if not aligned else T_b_rgb
    T_b_left = Tbc([0.06, 0.03, 0.03]); T_b_right = Tbc([0.06, -0.03, 0.03])
    T_b_imu = np.eye(4)
    sb = SessionBuilder(root, session_id, storage)
    t_begin = int(stamps[0]) - 50_000_000
    edges = [("insight_body", "rgb_optical", T_b_rgb), ("insight_body", "left_optical", T_b_left),
             ("insight_body", "right_optical", T_b_right), ("insight_body", "imu", T_b_imu)]
    if not aligned:
        edges.append(("insight_body", "depth_optical", T_b_dep))
    sb.add_tf(edges, t_begin, static=True)
    for tp, cam in (("/insight/rgb/camera_info", rgb), ("/insight/depth/camera_info", dep),
                    ("/insight/left/camera_info", left), ("/insight/right/camera_info", right)):
        for k in range(3):
            sb.add_camera_info(tp, int(stamps[0]) + k * 100_000_000, cam)
    t_rel = (stamps - stamps[0]) / 1e9
    T_vio_img = drift(T_wb, t_rel) if with_drift else T_wb
    if vio_pose_inverted:   # deliberately wrong convention (test of frame validation)
        T_vio_img = se3.inv(T_vio_img)
    for i, ns in enumerate(stamps):
        ns = int(ns)
        T_w_rgb = T_wb[i] @ T_b_rgb
        bgr = render_color(T_w_rgb, rgb)
        sb.add_image("/insight/rgb/image_raw", ns, bgr, "bgr8", rgb.frame_id)
        _, d = render(T_wb[i] @ T_b_dep, dep, noise_mm=2.0, rng=rng)
        sb.add_image("/insight/depth/image_raw", ns + 3_000_000, d, "16UC1", dep.frame_id, role="depth")
        gl = render_color(T_wb[i] @ T_b_left, left, gray=True)
        gr = render_color(T_wb[i] @ T_b_right, right, gray=True)
        sb.add_image("/insight/left/image_raw", ns, gl, "mono8", left.frame_id)
        sb.add_image("/insight/right/image_raw", ns, gr, "mono8", right.frame_id)
        sb.add_odom("/insight/vio_image", ns - 1_000_000, T_vio_img[i], "odom", "insight_body")
        sb.add_tf([("odom", "insight_body", T_vio_img[i])], ns - 1_000_000)
    # vio_100hz
    q100 = np.arange(stamps[0], stamps[-1], 10_000_000, dtype=np.int64)
    Tq = trajectory_continuous(stamps, T_wb, q100)
    T100 = drift(Tq, (q100 - stamps[0]) / 1e9) if with_drift else Tq
    if not drop_vio_100hz:
        for ns, T in zip(q100, T100):
            sb.add_odom("/insight/vio_100hz", int(ns), T, "odom", "insight_body")
    # IMU 200 Hz from true motion
    if not drop_imu:
        qi = np.arange(stamps[0], stamps[-1] - 1, 5_000_000, dtype=np.int64)
        Ti = trajectory_continuous(stamps, T_wb, qi)
        g = np.array([0, 0, 9.81])
        from scipy.interpolate import CubicSpline
        cs = CubicSpline((stamps - stamps[0]) / 1e9, T_wb[:, :3, 3], axis=0)
        acc_w = cs(((qi - stamps[0]) / 1e9), 2)
        for j, ns in enumerate(qi):
            Rb = Ti[j, :3, :3]
            jn = min(j + 1, len(qi) - 1); jp = max(j - 1, 0)
            gyro = Rot.from_matrix(Rb.T @ Ti[jn, :3, :3]).as_rotvec() / 0.005 if jn != j else np.zeros(3)
            sb.add_imu("/insight/imu", int(ns), gyro + rng.normal(0, 1e-3, 3), Rb.T @ (acc_w[j] + g) + rng.normal(0, 1e-2, 3), "imu")
    root_dir = sb.finalize(label="synthetic room loop", camera_config={"synthetic": True},
                           extensions={"synthetic": {"aligned_rgbd": aligned, "drift": with_drift, "baseline_m": B}},
                           missing=(["imu"] if drop_imu else []))
    gt = {"stamps_ns": stamps, "T_world_body": T_wb, "T_body_rgb": T_b_rgb, "T_body_depth": T_b_dep,
          "T_world_rgb": T_wb @ T_b_rgb}
    return root_dir, gt
