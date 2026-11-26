"""VIO -> camera trajectory. Convention everywhere: T_odom_cam maps points in the camera OPTICAL frame
(x right, y down, z forward — the frame of CameraInfo/Image header.frame_id) into the VIO odom frame.

Insight VIO outputs nav_msgs/Odometry: header.frame_id = odom (parent), child_frame_id = body frame.
pose = T_odom_body.  Camera pose = T_odom_body @ T_body_cam with T_body_cam from TF (child->camera frame).
Pairing policy (per requirement): prefer vio_image sample matched to the image stamp; use vio_100hz
interpolation otherwise; never extrapolate.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .. import se3
from ..extract import Extracted


@dataclass
class CamTrajectory:
    stamps_ns: np.ndarray
    T_odom_cam: np.ndarray      # (N,4,4)
    valid: np.ndarray           # (N,) bool
    source: list                # per-sample provenance
    dt_ms: np.ndarray           # distance (ms) to nearest raw VIO sample used

    def sel(self, m):
        return CamTrajectory(self.stamps_ns[m], self.T_odom_cam[m], self.valid[m],
                             [s for s, k in zip(self.source, m) if k], self.dt_ms[m])


def _interp(v, q, max_gap_ns):
    s = v["stamp_ns"]
    ok = (q >= s[0]) & (q <= s[-1])
    T = np.tile(np.eye(4), (len(q), 1, 1))
    hi = np.clip(np.searchsorted(s, q), 1, len(s) - 1)
    gap = s[hi] - s[hi - 1]
    ok &= gap <= max_gap_ns
    if ok.any():
        T[ok], *_ = se3.interpolate_poses(s.astype(np.float64), v["T"], q[ok].astype(np.float64))
    near = np.minimum(np.abs(q - s[hi - 1]), np.abs(s[hi] - q)) / 1e6
    return T, ok, near


def body_poses(ex: Extracted, query_ns: np.ndarray, tol_ms=5.0, prefer="vio_image"):
    vi, v100 = ex.vio("vio_image"), ex.vio("vio_100hz")
    if vi is None and v100 is None:
        raise ValueError("no VIO stream in extracted data")
    q = np.asarray(query_ns, dtype=np.int64)
    n = len(q)
    T = np.tile(np.eye(4), (n, 1, 1)); valid = np.zeros(n, bool); src = [""] * n; dt = np.full(n, np.inf)
    parent = (vi or v100)["parent"]; child = (vi or v100)["child"]
    if vi is not None:
        s = vi["stamp_ns"]
        i = np.clip(np.searchsorted(s, q), 1, len(s) - 1) if len(s) > 1 else np.zeros(n, int)
        cand = np.where(np.abs(q - s[i - 1]) <= np.abs(s[i] - q), i - 1, i)
        d = np.abs(q - s[cand]) / 1e6
        m = d <= tol_ms
        T[m] = vi["T"][cand[m]]; valid |= m; dt[m] = d[m]
        for k in np.nonzero(m)[0]:
            src[k] = "vio_image"
    rest = ~valid
    if rest.any() and v100 is not None:
        Ti, ok, near = _interp(v100, q, 60_000_000)
        m = rest & ok
        T[m] = Ti[m]; valid |= m; dt[m] = near[m]
        for k in np.nonzero(m)[0]:
            src[k] = "vio_100hz_interp"
    rest = ~valid
    if rest.any() and vi is not None:
        Ti, ok, near = _interp(vi, q, 250_000_000)
        m = rest & ok
        T[m] = Ti[m]; valid |= m; dt[m] = near[m]
        for k in np.nonzero(m)[0]:
            src[k] = "vio_image_interp"
    return T, valid, src, dt, parent, child


def camera_trajectory(ex: Extracted, cam_frame: str, query_ns, **kw) -> CamTrajectory:
    T_ob, valid, src, dt, parent, child = body_poses(ex, query_ns, **kw)
    T_bc = ex.tf.lookup(child, cam_frame)
    return CamTrajectory(np.asarray(query_ns, dtype=np.int64), T_ob @ T_bc, valid, src, dt)
