"""Minimal ORB + PnP(RANSAC) visual odometry primitives.

Used for (a) validating VIO frame/transform direction against pure vision, (b) loop-closure verification,
(c) the stereo-odometry *control experiment* (RTAB-Map's own stereo odometry is the intended control when
installed; this is the in-repo stand-in so the experiment can run anywhere).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from .. import se3
from ..calib import CameraModel


def depth_from_stereo(left: np.ndarray, right: np.ndarray, fx: float, baseline: float, max_depth=8.0) -> np.ndarray:
    """Rectified stereo -> depth (metres, float32, 0 invalid) in the LEFT camera frame."""
    sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=96, blockSize=5, P1=8 * 25, P2=32 * 25,
                                 uniquenessRatio=8, speckleWindowSize=60, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disp = sgbm.compute(left, right).astype(np.float32) / 16.0
    with np.errstate(divide="ignore", invalid="ignore"):
        z = fx * baseline / disp
    z[(disp <= 0.5) | (z > max_depth) | ~np.isfinite(z)] = 0
    return z.astype(np.float32)


def register_depth_to_rgb(depth_m: np.ndarray, cam_d: CameraModel, cam_c: CameraModel, T_c_d: np.ndarray) -> np.ndarray:
    """Forward-warp a depth map from the depth camera into the colour camera (z-buffered, metres).
    Needed whenever RGB and Depth are not pixel aligned. Holes are left at 0."""
    v, u = np.nonzero(depth_m > 0)
    z = depth_m[v, u].astype(np.float64)
    X = np.stack([(u - cam_d.cx) * z / cam_d.fx, (v - cam_d.cy) * z / cam_d.fy, z], 0)
    Xc = T_c_d[:3, :3] @ X + T_c_d[:3, 3:4]
    ok = Xc[2] > 1e-3
    Xc = Xc[:, ok]
    uc = np.round(Xc[0] * cam_c.fx / Xc[2] + cam_c.cx).astype(int)
    vc = np.round(Xc[1] * cam_c.fy / Xc[2] + cam_c.cy).astype(int)
    m = (uc >= 0) & (uc < cam_c.width) & (vc >= 0) & (vc < cam_c.height)
    out = np.full((cam_c.height, cam_c.width), np.inf)
    np.minimum.at(out, (vc[m], uc[m]), Xc[2][m])
    out[~np.isfinite(out)] = 0
    out = out.astype(np.float32)
    for _ in range(2):  # close splat holes (resolution/viewpoint differences) with a 3x3 min-filter of valid depths
        inf = np.where(out > 0, out, np.float32(np.inf))
        mn = cv2.erode(inf, np.ones((3, 3), np.uint8))
        out = np.where((out == 0) & np.isfinite(mn), mn, out)
    return out


@dataclass
class Frame:
    kp: np.ndarray            # (N,2) pixel
    des: np.ndarray
    pts3d: np.ndarray         # (N,3) in camera frame, NaN where depth invalid
    size: tuple


_orb = None


def make_frame(gray: np.ndarray, depth_m: np.ndarray, cam: CameraModel, n_features=1500, max_depth=8.0) -> Frame:
    global _orb
    if _orb is None or _orb.getMaxFeatures() != n_features:
        _orb = cv2.ORB_create(nfeatures=n_features, scaleFactor=1.2, nlevels=8, fastThreshold=10)
    kps, des = _orb.detectAndCompute(gray, None)
    if des is None or len(kps) == 0:
        return Frame(np.zeros((0, 2)), np.zeros((0, 32), np.uint8), np.zeros((0, 3)), gray.shape[::-1])
    xy = np.array([k.pt for k in kps], dtype=np.float64)
    ui = np.clip(np.round(xy[:, 0]).astype(int), 0, depth_m.shape[1] - 1)
    vi = np.clip(np.round(xy[:, 1]).astype(int), 0, depth_m.shape[0] - 1)
    z = depth_m[vi, ui].astype(np.float64)
    z[(z <= 0) | (z > max_depth)] = np.nan
    P = np.stack([(xy[:, 0] - cam.cx) * z / cam.fx, (xy[:, 1] - cam.cy) * z / cam.fy, z], 1)
    return Frame(xy, des, P, gray.shape[::-1])


_bf = cv2.BFMatcher(cv2.NORM_HAMMING)


def match(a: Frame, b: Frame, ratio=0.8):
    if len(a.des) < 8 or len(b.des) < 8:
        return np.zeros(0, int), np.zeros(0, int)
    m = _bf.knnMatch(a.des, b.des, k=2)
    ia, ib = [], []
    for pair in m:
        if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance:
            ia.append(pair[0].queryIdx); ib.append(pair[0].trainIdx)
    return np.array(ia, int), np.array(ib, int)


def relative_pose(a: Frame, b: Frame, cam: CameraModel, min_inliers=25, reproj_px=3.0, ratio=0.8):
    """Estimate T_a_b (maps points in b's camera frame into a's) using a's 3D points and b's pixels.
    Returns (T, n_inliers, n_matches) or (None, n_inliers, n_matches)."""
    ia, ib = match(a, b, ratio)
    good = np.isfinite(a.pts3d[ia]).all(1) if len(ia) else np.zeros(0, bool)
    ia, ib = ia[good], ib[good]
    if len(ia) < min_inliers:
        return None, 0, len(ia)
    ok, rvec, tvec, inl = cv2.solvePnPRansac(a.pts3d[ia].astype(np.float64), b.kp[ib].astype(np.float64),
                                              cam.K_mat, None, iterationsCount=300, reprojectionError=reproj_px,
                                              confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inl is None or len(inl) < min_inliers:
        return None, 0 if inl is None else len(inl), len(ia)
    inl = inl[:, 0]
    ok, rvec, tvec = cv2.solvePnP(a.pts3d[ia][inl].astype(np.float64), b.kp[ib][inl].astype(np.float64), cam.K_mat, None,
                                  rvec, tvec, True, cv2.SOLVEPNP_ITERATIVE)
    T_b_a = np.eye(4)
    T_b_a[:3, :3] = cv2.Rodrigues(rvec)[0]
    T_b_a[:3, 3] = tvec[:, 0]
    return se3.inv(T_b_a), len(inl), len(ia)
