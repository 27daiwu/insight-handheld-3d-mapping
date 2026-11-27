"""Stage 4 - stereo visual odometry (keyframe-based, ORB + stereo-triangulated 3D points + PnP RANSAC).

World frame = the first left optical frame (x right, y down, z forward) - a *relative* frame; no VIO/IMU/gravity.
Pose convention: T_world_cam, trajectory CSV `stamp_ns,tx,ty,tz,qx,qy,qz,qw` for the LEFT camera optical frame.
"""
from __future__ import annotations

import csv
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import cv2
import numpy as np

from .. import se3
from .rig import Rig
from .stereo import StereoCache


@dataclass
class VOParams:
    z_min: float = 0.3
    z_max: float = 6.0
    ratio: float = 0.75
    ransac_reproj_px: float = 2.0
    ransac_iters: int = 400
    min_inliers: int = 25
    kf_min_inliers: int = 60          # new keyframe when tracked inliers drop below this
    kf_inlier_ratio: float = 0.45     # ... or below this fraction of the keyframe's valid points
    kf_dist_m: float = 0.25
    kf_rot_deg: float = 12.0
    max_speed_mps: float = 4.0
    max_rot_dps: float = 400.0
    local_map_kfs: int = 3            # extra recent keyframes whose points are also matched (frame-to-local-map)


class _Frame:
    __slots__ = ("kp", "des", "P", "T", "idx", "n_valid")

    def __init__(self, kp, des, disp, rig: Rig, p: VOParams, idx):
        d = disp.astype(np.float64)
        with np.errstate(divide="ignore", invalid="ignore"):
            z = rig.fx * rig.baseline / d
        ok = (d >= 1.0) & (z >= p.z_min) & (z <= p.z_max)
        z = np.where(ok, z, np.nan)
        self.kp, self.des, self.idx = kp, des, idx
        self.P = np.stack([(kp[:, 0] - rig.cx) * z / rig.fx, (kp[:, 1] - rig.cy) * z / rig.fy, z], 1)
        self.n_valid = int(ok.sum())
        self.T = None


_bf = cv2.BFMatcher(cv2.NORM_HAMMING)


def _match(a_des, b_des, ratio):
    if len(a_des) < 8 or len(b_des) < 8:
        return np.zeros(0, int), np.zeros(0, int)
    m = _bf.knnMatch(a_des, b_des, k=2)
    ia, ib = [], []
    for pr in m:
        if len(pr) == 2 and pr[0].distance < ratio * pr[1].distance:
            ia.append(pr[0].queryIdx); ib.append(pr[0].trainIdx)
    return np.array(ia, int), np.array(ib, int)


def _pnp(P, uv, K, p: VOParams):
    if len(P) < p.min_inliers:
        return None
    ok, rvec, tvec, inl = cv2.solvePnPRansac(P, uv, K, None, iterationsCount=p.ransac_iters, reprojectionError=p.ransac_reproj_px,
                                             confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inl is None or len(inl) < p.min_inliers:
        return None
    inl = inl[:, 0]
    rvec, tvec = cv2.solvePnPRefineLM(P[inl], uv[inl], K, None, rvec, tvec)
    # re-evaluate inliers with the refined pose
    pr, _ = cv2.projectPoints(P, rvec, tvec, K, None)
    err = np.linalg.norm(pr[:, 0] - uv, axis=1)
    inl = np.nonzero(err < p.ransac_reproj_px)[0]
    if len(inl) < p.min_inliers:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(P[inl], uv[inl], K, None, rvec, tvec)
    T_c_k = np.eye(4); T_c_k[:3, :3] = cv2.Rodrigues(rvec)[0]; T_c_k[:3, 3] = tvec[:, 0]
    pr, _ = cv2.projectPoints(P[inl], rvec, tvec, K, None)
    rms = float(np.sqrt(np.mean(np.sum((pr[:, 0] - uv[inl]) ** 2, 1))))
    return T_c_k, len(inl), rms


def run_vo(cache: StereoCache, rig: Rig, p: VOParams | None = None, out_dir=None, log=print) -> dict:
    p = p or VOParams()
    K = rig.K
    n = len(cache)
    t0 = time.time()
    poses = np.tile(np.eye(4), (n, 1, 1))
    status = []
    kfs: list[_Frame] = []
    prev: _Frame | None = None
    vel = np.eye(4)                          # last relative motion (constant-velocity fallback)
    n_lost = 0
    lost_segments = []
    kf_ids = []
    rows = []
    for i in range(n):
        kp, des, disp = cache.feat(i)
        fr = _Frame(kp, des, disp, rig, p, i)
        if prev is None:
            fr.T = np.eye(4); kfs.append(fr); poses[i] = fr.T; prev = fr; kf_ids.append(i)
            rows.append((cache.stamps[i], len(kp), 0, 0, 0.0, "init")); status.append("init"); continue
        dt = (cache.stamps[i] - cache.stamps[i - 1]) / 1e9
        # gather correspondences: keyframe + a few earlier keyframes (frame-to-local-map)
        refs = kfs[-p.local_map_kfs:][::-1]
        P_all, uv_all, src_id = [], [], []
        for r_id, r in enumerate(refs):
            ia, ib = _match(r.des, des, p.ratio)
            if len(ia) == 0:
                continue
            good = np.isfinite(r.P[ia]).all(1)
            ia, ib = ia[good], ib[good]
            # bring the keyframe's 3D points into the last-keyframe-relative frame via world pose
            Xw = r.P[ia] @ r.T[:3, :3].T + r.T[:3, 3]
            P_all.append(Xw); uv_all.append(kp[ib].astype(np.float64)); src_id.append(np.full(len(ia), r_id))
        res = None
        n_match = 0
        why = "no_matches"

        def _try(Xw, uv):
            r_ = _pnp(Xw, uv, K, p)
            if r_ is None:
                return None, "pnp_failed"
            T_c_w, ninl, rms = r_
            T_w_c = se3.inv(T_c_w)
            rel = se3.relative(prev.T, T_w_c)
            sp = np.linalg.norm(rel[:3, 3]) / max(dt, 1e-3)
            rt = np.degrees(se3.rot_angle(rel[:3, :3])) / max(dt, 1e-3)
            if sp > p.max_speed_mps or rt > p.max_rot_dps:
                return None, f"motion_gate({sp:.1f}m/s,{rt:.0f}dps)"
            return (T_w_c, ninl, rms), "ok"

        if P_all:
            n_match = sum(len(x) for x in P_all)
            res, why = _try(np.concatenate(P_all), np.concatenate(uv_all))
            if res is None:                       # retry per reference (a stale keyframe may poison the joint fit)
                for Xw_, uv_ in zip(P_all, uv_all):
                    res, why2 = _try(Xw_, uv_)
                    if res is not None:
                        why = "ok_single_ref"; break
        if res is None:                                          # tracking failure: constant-velocity guess, restart keyframe
            n_lost += 1
            T_w_c = prev.T @ vel
            fr.T = T_w_c; poses[i] = T_w_c
            kfs.append(fr); kf_ids.append(i)
            lost_segments.append((i, why))
            rows.append((cache.stamps[i], n_match, 0, 0.0, 0.0, "lost:" + why)); status.append("lost")
            prev = fr
            continue
        T_w_c, ninl, rms = res
        vel = se3.relative(prev.T, T_w_c)
        fr.T = T_w_c; poses[i] = T_w_c
        kf = kfs[-1]
        rel_kf = se3.relative(kf.T, T_w_c)
        need_kf = (ninl < p.kf_min_inliers or ninl < p.kf_inlier_ratio * max(kf.n_valid, 1)
                   or np.linalg.norm(rel_kf[:3, 3]) > p.kf_dist_m or np.degrees(se3.rot_angle(rel_kf[:3, :3])) > p.kf_rot_deg)
        if need_kf and fr.n_valid >= p.min_inliers:
            kfs.append(fr); kf_ids.append(i)
        rows.append((cache.stamps[i], n_match, ninl, rms, np.linalg.norm(rel_kf[:3, 3]), "kf" if need_kf else "ok"))
        status.append("kf" if need_kf else "ok")
        prev = fr
        if len(kfs) > 40:
            kfs = kfs[-40:]
        if i % 500 == 0 and i:
            log(f"  vo {i}/{n}  lost={n_lost}  {i / (time.time() - t0):.1f} fps")
    # --- products
    steps = np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1)
    inl = np.array([r[2] for r in rows if r[5] in ("ok", "kf")]) if rows else np.zeros(0)
    rms = np.array([r[3] for r in rows if r[5] in ("ok", "kf")]) if rows else np.zeros(0)
    rep = {"frames": int(n), "keyframes": len(kf_ids), "lost_frames": n_lost, "lost_fraction": n_lost / max(n, 1),
           "lost_at": [{"stamp_ns": int(cache.stamps[i]), "reason": w} for i, w in lost_segments[:50]],
           "path_length_m": float(steps.sum()), "end_to_start_m": float(np.linalg.norm(poses[-1, :3, 3] - poses[0, :3, 3])),
           "inliers": {"median": float(np.median(inl)) if len(inl) else 0, "p5": float(np.percentile(inl, 5)) if len(inl) else 0},
           "reproj_rms_px": {"median": float(np.median(rms)) if len(rms) else 0, "p95": float(np.percentile(rms, 95)) if len(rms) else 0},
           "max_step_m": float(steps.max()) if len(steps) else 0, "elapsed_s": time.time() - t0,
           "fps": n / max(time.time() - t0, 1e-9), "params": asdict(p),
           "frame": "world = first left-camera optical frame (relative, gravity-unaligned)"}
    if out_dir:
        out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
        with open(out / "trajectory.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["stamp_ns", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
            for s, T in zip(cache.stamps, poses):
                t, q = se3.T_to_tq(T); w.writerow([int(s), *map(float, t), *map(float, q)])
        with open(out / "vo_log.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["stamp_ns", "matches", "inliers", "reproj_rms_px", "dist_to_kf_m", "status"]); w.writerows(rows)
    return {"report": rep, "poses": poses, "status": status}
