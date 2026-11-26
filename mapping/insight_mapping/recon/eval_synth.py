"""Reconstruction accuracy against the analytic synthetic scene (room box + sphere)."""
from __future__ import annotations

import numpy as np

from ..fixtures.synthetic import ROOM_HI, ROOM_LO, SPHERE_C, SPHERE_R


def surface_distance(P: np.ndarray) -> np.ndarray:
    d_box = np.minimum(P - ROOM_LO, ROOM_HI - P).min(1)      # >0 inside the room
    d_box = np.abs(d_box)
    d_sph = np.abs(np.linalg.norm(P - SPHERE_C, axis=1) - SPHERE_R)
    return np.minimum(d_box, d_sph)


def evaluate_points(P: np.ndarray) -> dict:
    d = surface_distance(P)
    return {"n": int(len(P)), "median_m": float(np.median(d)), "mean_m": float(d.mean()),
            "p95_m": float(np.percentile(d, 95)), "frac_within_3cm": float((d < 0.03).mean())}


def evaluate_mesh(mesh, n=200000) -> dict:
    pc = mesh.sample_points_uniformly(min(n, max(len(mesh.triangles), 10)))
    return evaluate_points(np.asarray(pc.points))
