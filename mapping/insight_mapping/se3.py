"""Small SE(3) helpers. Conventions: T_a_b is a 4x4 matrix mapping points in frame b to frame a
(ROS TransformStamped with header.frame_id=a, child_frame_id=b). Quaternions are (x, y, z, w)."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def make_T(t, q) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def T_to_tq(T: np.ndarray):
    return T[:3, 3].copy(), Rotation.from_matrix(T[:3, :3]).as_quat()


def inv(T: np.ndarray) -> np.ndarray:
    R, t = T[..., :3, :3], T[..., :3, 3]
    out = np.zeros_like(T)
    out[..., :3, :3] = np.swapaxes(R, -1, -2)
    out[..., :3, 3] = -np.einsum("...ij,...j->...i", np.swapaxes(R, -1, -2), t)
    out[..., 3, 3] = 1.0
    return out


def rot_angle(R: np.ndarray) -> float:
    """Rotation angle in radians of a 3x3 rotation matrix."""
    return float(np.linalg.norm(Rotation.from_matrix(R).as_rotvec()))


def relative(Ta: np.ndarray, Tb: np.ndarray) -> np.ndarray:
    """T_a^-1 * T_b (motion from a to b expressed in a)."""
    return inv(Ta) @ Tb


def interpolate_poses(stamps: np.ndarray, poses: np.ndarray, query: np.ndarray):
    """Linear/slerp interpolation. stamps strictly increasing (N,), poses (N,4,4).
    Returns (T (M,4,4), idx_lo (M,), idx_hi (M,), alpha(M,)); queries outside [first,last] are clamped
    (caller must mask them with `in_range`)."""
    stamps = np.asarray(stamps, dtype=np.float64)
    q = np.asarray(query, dtype=np.float64)
    hi = np.clip(np.searchsorted(stamps, q, side="left"), 1, len(stamps) - 1)
    lo = hi - 1
    dt = stamps[hi] - stamps[lo]
    alpha = np.clip(np.where(dt > 0, (q - stamps[lo]) / np.where(dt > 0, dt, 1), 0.0), 0.0, 1.0)
    out = np.tile(np.eye(4), (len(q), 1, 1))
    out[:, :3, 3] = (1 - alpha)[:, None] * poses[lo, :3, 3] + alpha[:, None] * poses[hi, :3, 3]
    for i in range(len(q)):  # per-sample slerp; N is images, fine
        rots = Rotation.from_matrix(np.stack([poses[lo[i], :3, :3], poses[hi[i], :3, :3]]))
        out[i, :3, :3] = Slerp([0.0, 1.0], rots)([alpha[i]]).as_matrix()[0]
    return out, lo, hi, alpha


def umeyama_align(src: np.ndarray, dst: np.ndarray, with_scale: bool = False) -> np.ndarray:
    """Rigid (optionally similarity) transform T with dst ≈ T @ src (points Nx3)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(xd.T @ xs / len(src))
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = (S * np.diag(D)).sum() / xs.var(0).sum() if with_scale else 1.0
    T = np.eye(4)
    T[:3, :3] = s * R
    T[:3, 3] = mu_d - s * R @ mu_s
    return T
