"""SE(3) pose-graph optimisation (scipy, sparse LM-style TRF, robust loss on loop edges).

Edge convention: measurement Z_ij ≈ inv(T_w_i) @ T_w_j  (pose of j expressed in i). Residual = log(inv(Z) inv(Ti) Tj).
Used as the in-repo back-end (RTAB-Map's graph optimiser — g2o/GTSAM — is the production target, see rtabmap_bridge).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation as Rot

from .. import se3


@dataclass
class Edge:
    i: int
    j: int
    Z: np.ndarray
    sigma_t: float = 0.02     # m
    sigma_r: float = 0.02     # rad
    loop: bool = False


def _pack(T):
    return np.concatenate([Rot.from_matrix(T[:, :3, :3]).as_rotvec(), T[:, :3, 3]], 1).ravel()


def _unpack(x):
    x = x.reshape(-1, 6)
    T = np.tile(np.eye(4), (len(x), 1, 1))
    T[:, :3, :3] = Rot.from_rotvec(x[:, :3]).as_matrix()
    T[:, :3, 3] = x[:, 3:]
    return T


def edge_residuals(T, edges):
    I = np.array([e.i for e in edges]); J = np.array([e.j for e in edges])
    Z = np.stack([e.Z for e in edges])
    rel = se3.inv(T[I]) @ T[J]
    E = se3.inv(Z) @ rel
    r = Rot.from_matrix(E[:, :3, :3]).as_rotvec() / np.array([e.sigma_r for e in edges])[:, None]
    t = E[:, :3, 3] / np.array([e.sigma_t for e in edges])[:, None]
    return np.concatenate([r, t], 1)


def optimize(T0: np.ndarray, edges: list, loss="soft_l1", max_nfev=60):
    n = len(T0)
    x0 = _pack(T0)
    m = len(edges)

    def fun(x):
        T = _unpack(x)
        r = edge_residuals(T, edges).ravel()
        prior = (_pack(T[:1]) - _pack(T0[:1])) / 1e-4
        return np.concatenate([r, prior])

    S = lil_matrix((6 * m + 6, 6 * n), dtype=int)
    for k, e in enumerate(edges):
        S[6 * k:6 * k + 6, 6 * e.i:6 * e.i + 6] = 1
        S[6 * k:6 * k + 6, 6 * e.j:6 * e.j + 6] = 1
    S[6 * m:, 0:6] = 1
    c0 = float(np.sum(fun(x0) ** 2))
    res = least_squares(fun, x0, jac_sparsity=S, loss=loss, f_scale=3.0, max_nfev=max_nfev, x_scale="jac", method="trf")
    T = _unpack(res.x)
    return T, {"cost_before": c0, "cost_after": float(np.sum(fun(res.x) ** 2)), "nfev": int(res.nfev), "status": int(res.status)}
