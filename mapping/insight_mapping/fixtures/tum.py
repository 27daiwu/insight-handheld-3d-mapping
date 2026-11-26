"""Convert a TUM RGB-D (freiburg1) sequence into a Session for algorithm validation on public data.

TUM has no IMU / VIO / stereo. The "VIO" stream is ground truth with injected smooth drift (so loop-closure
/ pose-graph code is tested against known truth); IMU is reported missing. The depth is registered to RGB by
the OpenNI driver (documented by TUM), i.e. aligned.  Ground truth stays outside the Session.
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from .. import se3
from ..calib import CameraModel
from .sessionwriter import SessionBuilder
from .synthetic import drift

FR1 = dict(fx=517.3, fy=516.5, cx=318.6, cy=255.3, d=[0.2624, -0.9531, -0.0054, 0.0026, 1.1633])
DEPTH_SCALE = 5000.0


def read_list(p: Path):
    rows = []
    for ln in p.read_text().splitlines():
        if ln and not ln.startswith("#"):
            rows.append(ln.split())
    return rows


def load_groundtruth(seq_dir: Path):
    gt = np.array([[float(x) for x in r] for r in read_list(seq_dir / "groundtruth.txt")])
    T = np.stack([se3.make_T(r[1:4], r[4:8]) for r in gt])
    return gt[:, 0], T   # seconds, T_world_cam (optical)


def make_session(seq_dir, root, session_id="session_tum", max_frames=None, every=1, with_drift=True,
                 drift_kwargs=None, storage="sqlite3"):
    seq = Path(seq_dir)
    rgb_l = read_list(seq / "rgb.txt"); dep_l = read_list(seq / "depth.txt")
    rgb_t = np.array([float(r[0]) for r in rgb_l]); dep_t = np.array([float(r[0]) for r in dep_l])
    sel = np.arange(0, len(rgb_l), every)
    if max_frames:
        sel = sel[:max_frames]
    gt_t, gt_T = load_groundtruth(seq)
    sel = np.array([i for i in sel if gt_t[0] <= rgb_t[i] <= gt_t[-1]])
    rgb_ns = (rgb_t[sel] * 1e9).astype(np.int64)
    cam = CameraModel("rgb_optical", 640, 480, [FR1["fx"], 0, FR1["cx"], 0, FR1["fy"], FR1["cy"], 0, 0, 1], FR1["d"],
                      distortion_model="plumb_bob", P=[FR1["fx"], 0, FR1["cx"], 0, 0, FR1["fy"], FR1["cy"], 0, 0, 0, 1, 0])
    sb = SessionBuilder(root, session_id, storage)
    sb.add_tf([("odom", "world_dummy", np.eye(4))], int(rgb_ns[0]) - 1, static=True)
    for k in range(2):
        sb.add_camera_info("/camera/rgb/camera_info", int(rgb_ns[0]) + k, cam)
        sb.add_camera_info("/camera/depth/camera_info", int(rgb_ns[0]) + k, cam)  # registered to RGB
    T_gt, *_ = se3.interpolate_poses(gt_t * 1e9, gt_T, rgb_ns.astype(np.float64))
    Tv = drift(T_gt, (rgb_ns - rgb_ns[0]) / 1e9, **(drift_kwargs or {})) if with_drift else T_gt
    for n, i in enumerate(sel):
        ns = int(rgb_ns[n])
        sb.add_image("/camera/rgb/image_raw", ns, cv2.imread(str(seq / rgb_l[i][1])), "bgr8", "rgb_optical")
        j = int(np.argmin(np.abs(dep_t - rgb_t[i])))
        d = cv2.imread(str(seq / dep_l[j][1]), cv2.IMREAD_UNCHANGED).astype(np.float64) * (1000.0 / DEPTH_SCALE)
        sb.add_image("/camera/depth/image_raw", int(dep_t[j] * 1e9), np.clip(np.round(d), 0, 65535).astype(np.uint16),
                     "16UC1", "rgb_optical", role="depth")
        sb.add_odom("/camera/vio_image", ns, Tv[n], "odom", "rgb_optical")
    root_dir = sb.finalize(label="TUM " + seq.name + " (drift-injected GT as VIO)",
                           camera_config={"public_dataset": seq.name},
                           extensions={"public_dataset": seq.name, "vio_is": "ground truth + injected drift"},
                           missing=["imu"], warnings=["public dataset: no IMU, no stereo"])
    return root_dir, {"stamps_ns": rgb_ns, "T_world_cam": T_gt}
