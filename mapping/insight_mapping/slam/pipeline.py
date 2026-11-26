"""P1-C SLAM stage: VIO-seeded keyframe graph + visual loop closure + pose-graph optimisation.

Odometry source: Insight VIO (vio_image paired to image stamps, vio_100hz interpolation).
Loop closure: ORB + depth-PnP geometric verification. Optimiser: se3 pose graph (posegraph.py).
This is the in-repo back-end that runs everywhere; RTAB-Map (rtabmap/backend.py) is the second back-end; both write `trajectory_opt.csv` (same CSV contract) for the same fusion stage.
"""
from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path

import numpy as np

from .. import evaluate, se3
from ..extract import Extracted
from ..session import Session
from . import frames, posegraph, vio, vo


@dataclass
class SlamConfig:
    kf_dist_m: float = 0.15
    kf_rot_deg: float = 10.0
    loop_min_gap_kf: int = 6          # keyframes between i and j to count as revisit
    loop_max_dist_m: float = 2.5      # VIO position gate (generous: VIO drifts)
    loop_max_view_deg: float = 75.0
    loop_min_inliers: int = 30
    loop_max_rot_dev_deg: float = 30.0
    loop_max_trans_dev_m: float = 1.2
    loops_per_kf: int = 2
    use_stereo_control: bool = True
    validate_frames: bool = True


def trajectory_to_csv(path, stamps_ns, T_world_cam, valid=None):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp_ns", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
        for k, (s, T) in enumerate(zip(stamps_ns, T_world_cam)):
            if valid is not None and not valid[k]:
                continue
            t, q = se3.T_to_tq(T)
            w.writerow([int(s), *map(float, t), *map(float, q)])


def read_trajectory_csv(path):
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    T = np.stack([se3.make_T(r[1:4], r[4:8]) for r in a])
    return a[:, 0].astype(np.int64), T


def select_keyframes(T, valid, cfg: SlamConfig):
    kf = []
    for i in np.nonzero(valid)[0]:
        if not kf:
            kf.append(i); continue
        rel = se3.relative(T[kf[-1]], T[i])
        if np.linalg.norm(rel[:3, 3]) > cfg.kf_dist_m or np.degrees(se3.rot_angle(rel[:3, :3])) > cfg.kf_rot_deg:
            kf.append(i)
    return np.array(kf, dtype=int)


def apply_corrections(stamps, T_vio, kf_idx, T_kf_opt):
    """World correction C_k = T_opt_k inv(T_vio_k) interpolated (slerp/lerp) in time between keyframes."""
    C = T_kf_opt @ se3.inv(T_vio[kf_idx])
    if len(kf_idx) == 1:
        return C[0] @ T_vio
    Ci, *_ = se3.interpolate_poses(stamps[kf_idx].astype(np.float64), C, stamps.astype(np.float64))
    return Ci @ T_vio


def run(session: Session, ex: Extracted, out_dir, cfg: SlamConfig | None = None, gt_T=None, log=print) -> dict:
    """gt_T: optional ground-truth T_world_cam aligned index-wise with the base image stream (evaluation only)."""
    cfg = cfg or SlamConfig()
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    t_start = time.time()
    key = "rgb" if "rgb" in session.streams else "left"
    idx = ex.index(key)
    cam = ex.cams[f"{key}_info"]
    stamps = idx["stamp_ns"]
    rep: dict = {"base_stream": key, "frames": int(len(stamps)), "config": asdict(cfg)}

    traj = vio.camera_trajectory(ex, cam.frame_id, stamps)
    rep["vio_pairing"] = {s: int(sum(1 for x in traj.source if x == s)) for s in set(traj.source)}
    rep["vio_pairing"]["unpaired"] = int((~traj.valid).sum())
    if "vio_image" in rep["vio_pairing"]:
        rep["vio_pairing"]["dt_ms_max_vio_image"] = float(traj.dt_ms[[s == "vio_image" for s in traj.source]].max())
    if not traj.valid.any():
        raise RuntimeError("no image could be paired with a VIO pose")

    if cfg.validate_frames:
        rep["frame_validation"] = frames.validate_directions(ex, session, log=log)
        if rep["frame_validation"].get("status") == "MISMATCH":
            raise RuntimeError("VIO frame convention mismatch: " + rep["frame_validation"]["message"])

    T_vio = traj.T_odom_cam
    kf = select_keyframes(T_vio, traj.valid, cfg)
    rep["keyframes"] = int(len(kf))
    log(f"keyframes: {len(kf)} of {len(stamps)}")

    frs = []
    for i in kf:
        g, d, c, _, _ = frames.load_vo_inputs(ex, session, int(i))
        frs.append(vo.make_frame(g, d, c))
    cam_vo = frames.load_vo_inputs(ex, session, int(kf[0]))[2]

    edges = []
    for a in range(len(kf) - 1):
        Z = se3.relative(T_vio[kf[a]], T_vio[kf[a + 1]])
        edges.append(posegraph.Edge(a, a + 1, Z, 0.02, np.radians(1.0)))
    n_odom = len(edges)

    # --- loop closure
    loops = []
    pos = T_vio[kf][:, :3, 3]
    for j in range(len(kf)):
        cands = []
        for i in range(0, j - cfg.loop_min_gap_kf + 1):
            if np.linalg.norm(pos[i] - pos[j]) > cfg.loop_max_dist_m:
                continue
            view = np.degrees(se3.rot_angle(se3.relative(T_vio[kf[i]], T_vio[kf[j]])[:3, :3]))
            if view > cfg.loop_max_view_deg:
                continue
            ia, ib = vo.match(frs[i], frs[j])
            cands.append((len(ia), i))
        cands.sort(reverse=True)
        for _, i in cands[:cfg.loops_per_kf + 2]:
            Z, ninl, nm = vo.relative_pose(frs[i], frs[j], cam_vo, min_inliers=cfg.loop_min_inliers)
            if Z is None:
                continue
            dev = se3.inv(se3.relative(T_vio[kf[i]], T_vio[kf[j]])) @ Z
            if np.degrees(se3.rot_angle(dev[:3, :3])) > cfg.loop_max_rot_dev_deg or np.linalg.norm(dev[:3, 3]) > cfg.loop_max_trans_dev_m:
                continue
            loops.append({"i": int(i), "j": int(j), "frame_i": int(kf[i]), "frame_j": int(kf[j]), "inliers": int(ninl),
                          "dev_trans_m": float(np.linalg.norm(dev[:3, 3])), "dev_rot_deg": float(np.degrees(se3.rot_angle(dev[:3, :3])))})
            edges.append(posegraph.Edge(i, j, Z, 0.05, np.radians(2.0), loop=True))
            if sum(1 for l in loops if l["j"] == j) >= cfg.loops_per_kf:
                break
    rep["loop_closure"] = {"accepted": len(loops), "edges": loops[:50]}
    log(f"loop closures accepted: {len(loops)}")

    T_kf, info = posegraph.optimize(T_vio[kf], edges)
    rep["optimization"] = info
    T_opt = apply_corrections(stamps, T_vio, kf, T_kf)
    rep["optimization"]["max_keyframe_correction_m"] = float(np.linalg.norm(T_kf[:, :3, 3] - T_vio[kf][:, :3, 3], axis=1).max())

    trajectory_to_csv(out / "trajectory_vio.csv", stamps, T_vio, traj.valid)
    trajectory_to_csv(out / "trajectory_opt.csv", stamps, T_opt, traj.valid)
    np.save(out / "kf_indices.npy", kf)
    v = traj.valid
    rep["path_length_m"] = evaluate.path_length(T_vio[v])
    rep["drift"] = {"vio_end_to_start_m": float(np.linalg.norm(T_vio[v][-1, :3, 3] - T_vio[v][0, :3, 3])),
                    "opt_end_to_start_m": float(np.linalg.norm(T_opt[v][-1, :3, 3] - T_opt[v][0, :3, 3])),
                    "note": "end-to-start gap is only a drift measure when the scan returns to its start"}
    if gt_T is not None:
        rep["vs_ground_truth"] = {"vio": {"ate": evaluate.ate(T_vio[v], gt_T[v]), "rpe": evaluate.rpe(T_vio[v], gt_T[v])},
                                  "optimized": {"ate": evaluate.ate(T_opt[v], gt_T[v]), "rpe": evaluate.rpe(T_opt[v], gt_T[v])}}

    if cfg.use_stereo_control and "left" in session.streams and "right" in session.streams:
        rep["stereo_vo_control"] = stereo_vo_control(session, ex, out, T_vio, traj, gt_T, log)
    rep["elapsed_s"] = time.time() - t_start
    (out / "slam_report.json").write_text(json.dumps(rep, indent=1))
    return rep


def stereo_vo_control(session, ex, out, T_vio, traj, gt_T, log):
    """Control experiment: pure stereo visual odometry (frame-to-keyframe ORB+PnP, SGBM depth), no VIO input."""
    from ..calib import stereo_baseline
    li, ri = ex.index("left"), ex.index("right")
    cam, rc = ex.cams["left_info"], ex.cams["right_info"]
    B = stereo_baseline(cam, rc)
    if B is None:
        return {"status": "skipped", "reason": "no stereo baseline in CameraInfo.P"}
    n = len(li["stamp_ns"])
    T = np.tile(np.eye(4), (n, 1, 1))
    ok = np.zeros(n, bool); ok[0] = True
    t0 = time.time()
    left = ex.read(li["file"][0], 0); right = ex.read(ri["file"][0], 0)
    kf_frame = vo.make_frame(left, vo.depth_from_stereo(left, right, cam.fx, B), cam)
    T_kf = np.eye(4); lost = 0
    for k in range(1, n):
        left = ex.read(li["file"][k], 0)
        F = vo.make_frame(left, np.zeros_like(left, np.float32), cam)
        Z, ninl, _ = vo.relative_pose(kf_frame, F, cam, min_inliers=20)
        if Z is None:
            lost += 1
            T[k] = T[k - 1]
            continue
        T[k] = T_kf @ Z
        ok[k] = True
        if ninl < 80 or np.linalg.norm(Z[:3, 3]) > 0.4:   # re-key
            right = ex.read(ri["file"][k], 0)
            kf_frame = vo.make_frame(left, vo.depth_from_stereo(left, right, cam.fx, B), cam)
            T_kf = T[k]
    res = {"status": "ok", "tracking_lost_frames": int(lost), "elapsed_s": time.time() - t0, "frames": int(n)}
    # express in the VIO/odom frame for comparison: align by first-pose (cam0 -> odom)
    T_odom = T_vio[0] @ T  # first valid VIO pose as origin
    trajectory_to_csv(out / "trajectory_stereo_vo.csv", li["stamp_ns"], T_odom, ok)
    v = ok & traj.valid
    if v.sum() > 3:
        res["ate_vs_vio_m"] = evaluate.ate(T_odom[v], T_vio[v])
        if gt_T is not None:
            res["ate_vs_gt"] = evaluate.ate(T_odom[v], gt_T[v])
    log(f"stereo VO control: lost={lost}")
    return res
