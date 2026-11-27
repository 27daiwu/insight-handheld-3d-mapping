"""Contract-faithful *simulated* Insight 9 fw 2.1.5 recording (unit tests / baseline only - never a substitute for real data).

Reproduces what DATA_CONTRACT v0.3 says the device publishes: measured topic names under /camera/camera, 544x640 mono8
stereo with the real CameraInfo numbers (K, P, baseline 0.10012 m), JPEG RGB 1088x1920, depth mono16 at ~13.1 Hz with random drops
and NO depth CameraInfo, IMU, VIO as PoseStamped on `world`, latched /tf_static only (no /tf), header stamps on a boot-relative clock
while bag times are host wall clock. The scene is the synthetic room from `fixtures.synthetic`.
"""
from __future__ import annotations

import json
import shutil
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .. import se3
from ..calib import CameraModel
from ..fixtures import synthetic
from ..fixtures.sessionwriter import SessionBuilder, to_time

P = "/camera/camera"
W, H = 544, 640
K = [307.385, 0.0, 269.424, 0.0, 307.385, 323.476, 0.0, 0.0, 1.0]
B = 0.10012
RGB_K = [768.863, 0.0, 539.467, 0.0, 768.891, 959.494, 0.0, 0.0, 1.0]
BOOT_S = 1339.0          # header stamp of the first frame: seconds since camera boot (contract §4)
WALL_NS = 1_795_000_000_000_000_000   # host wall clock at the first message


def _cam(frame, w, h, Kv, bx=0.0):
    c = CameraModel(frame, w, h, list(Kv), [0.0] * 4 if w == W else [0.0] * 5, "plumb_bob")
    c.P = [Kv[0], 0, Kv[2], -Kv[0] * bx, 0, Kv[4], Kv[5], 0, 0, 0, 1, 0]
    return c


def make_recording(root, name="sim_insight9_fw215", n_frames=120, fps=30.0, circle_frames=None, rgb_every=3, storage="mcap",
                   seed=0, radius=1.3, disp_noise=True):
    rng = np.random.default_rng(seed)
    left, right = _cam("camera_camera_left", W, H, K), _cam("camera_camera_right", W, H, K, B)
    rgbc = _cam("camera_camera_rgb", 1088, 1920, RGB_K)
    stamps0, T_wb = synthetic.trajectory(n_frames=circle_frames or n_frames, overlap=0, radius=radius, fps=fps, t0_ns=0)
    stamps0, T_wb = stamps0[:n_frames], T_wb[:n_frames]
    # camera optical pose = body @ R_BODY_OPT (left camera at body origin); right camera is +x of left in optical frame
    T_wl = np.stack([T @ np.block([[synthetic.R_BODY_OPT, np.zeros((3, 1))], [np.zeros((1, 3)), 1]]) for T in T_wb])
    T_l_r = np.eye(4); T_l_r[0, 3] = B
    T_l_rgb = np.eye(4); T_l_rgb[0, 3] = 0.0501
    root = Path(root) / name
    if root.exists():
        shutil.rmtree(root)
    sb = SessionBuilder(root.parent, name, storage)
    wall = lambda k: WALL_NS + int(k * 1e9 / fps) + int(rng.integers(0, 2_000_000))
    hs = lambda k: int(BOOT_S * 1e9) + int(k * 1e9 / fps)
    sb.add_tf([("camera_camera_left", "camera_camera_right", T_l_r), ("camera_camera_left", "camera_camera_rgb", T_l_rgb),
               ("camera_camera_left", "camera_camera_depth", np.eye(4)), ("camera_camera_left", "camera_camera_imu_optical", np.eye(4))],
              hs(0) - 1000, static=True, bag_ns=WALL_NS - 1000)
    # SessionBuilder uses stamp as bag time too; re-point bag time = wall clock for every write below
    for k in range(n_frames):
        s = hs(k); bag = wall(k)
        sb.add_camera_info(f"{P}/infra1/camera_info", s, left, bag)
        sb.add_camera_info(f"{P}/infra2/camera_info", s, right, bag)
        T = T_wl[k]
        gl = synthetic.render_color(T, left, gray=True, ss=2) if False else _render_gray(T, left)
        gr = _render_gray(T @ T_l_r, right)
        sb.add_image(f"{P}/infra1/image_rect_raw", s, gl, "mono8", "camera_camera_left", bag_ns=bag)
        sb.add_image(f"{P}/infra2/image_rect_raw", s, gr, "mono8", "camera_camera_right", bag_ns=bag)
        if k % rgb_every == 0:
            sb.add_camera_info(f"{P}/color/camera_info", s, rgbc, bag)
            col = _render_bgr(T @ T_l_rgb, rgbc)
            ok, jpg = cv2.imencode(".jpg", col, [cv2.IMWRITE_JPEG_QUALITY, 85])
            sb.add_compressed(f"{P}/color/image_rect_raw/compressed", s, jpg.tobytes(), "jpeg", "camera_camera_rgb", bag_ns=bag)
        if rng.random() > 0.126 and k % 2 == 0:       # ~13 Hz
            _, d = synthetic.render(T, _small(left), noise_mm=2.0, rng=rng) if False else (None, _depth(T, left, rng))
            sb.add_image(f"{P}/depth/image_rect_raw", s, d, "mono16", "camera_camera_depth", role="depth", bag_ns=bag)
        sb.add_pose(f"{P}/vio_100hz", s, T_wl[k], "world", bag_ns=bag)
    for k in range(n_frames * 400 // int(fps)):
        s = int(BOOT_S * 1e9) + int(k * 1e9 / 400)
        sb.add_imu(f"{P}/imu", s, rng.normal(0, 2e-3, 3), np.array([-4.3, -8.8, 0.28]) + rng.normal(0, 1e-2, 3),
                   "camera_camera_imu_optical", bag_ns=WALL_NS + int(k * 1e9 / 400))
    sb.finalize(label="simulated fw2.1.5 (UNIT TEST ONLY)", extensions={"simulated": True})
    # capture-style manifest (schema 0.2-draft) replaces the mapping-fixture one
    (root / "manifest.json").write_text(json.dumps({"schema_version": "0.2-draft", "session_id": name, "status": "complete",
                                                      "camera": {"model": "Insight 9", "firmware": "2.1.5"}, "SIMULATED": True}, indent=1))
    gt = {"stamps_ns": np.array([hs(k) for k in range(n_frames)], np.int64), "T_world_left": T_wl}
    np.savez(root.parent / f"{name}_gt.npz", stamps=gt["stamps_ns"], T=T_wl)
    return root, gt


def _render_gray(T, cam, ss=2):
    big = cam.scaled(cam.width * ss, cam.height * ss)
    col, _ = synthetic.render(T, big)
    col = col.reshape(cam.height, ss, cam.width, ss, 3).astype(np.float32).mean((1, 3))
    g = 0.114 * col[..., 0] + 0.587 * col[..., 1] + 0.299 * col[..., 2]
    g = g + np.random.default_rng(int(T[0, 3] * 1e4) & 0xFFFF).normal(0, 1.0, g.shape)
    return np.clip(np.round(g), 0, 255).astype(np.uint8)


def _render_bgr(T, cam):
    col, _ = synthetic.render(T, cam)
    return col


def _depth(T, cam, rng):
    _, d = synthetic.render(T, cam, noise_mm=2.0, rng=rng)
    return d
