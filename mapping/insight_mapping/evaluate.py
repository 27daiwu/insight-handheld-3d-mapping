"""Trajectory evaluation against ground truth: ATE (after rigid alignment) and RPE."""
from __future__ import annotations

import numpy as np

from . import se3


def ate(T_est: np.ndarray, T_gt: np.ndarray, align=True) -> dict:
    p, q = T_est[:, :3, 3], T_gt[:, :3, 3]
    if align:
        A = se3.umeyama_align(p, q)
        p = p @ A[:3, :3].T + A[:3, 3]
    e = np.linalg.norm(p - q, axis=1)
    return {"rmse_m": float(np.sqrt((e ** 2).mean())), "mean_m": float(e.mean()), "max_m": float(e.max()),
            "final_m": float(e[-1]), "aligned": bool(align)}


def rpe(T_est, T_gt, delta=10) -> dict:
    n = len(T_est)
    if n <= delta:
        return {}
    a = se3.inv(T_gt[:-delta]) @ T_gt[delta:]
    b = se3.inv(T_est[:-delta]) @ T_est[delta:]
    E = se3.inv(a) @ b
    tr = np.linalg.norm(E[:, :3, 3], axis=1)
    ro = np.array([np.degrees(se3.rot_angle(e[:3, :3])) for e in E])
    return {"delta_frames": delta, "trans_rmse_m": float(np.sqrt((tr ** 2).mean())), "rot_rmse_deg": float(np.sqrt((ro ** 2).mean()))}


def path_length(T) -> float:
    return float(np.linalg.norm(np.diff(T[:, :3, 3], axis=0), axis=1).sum())
