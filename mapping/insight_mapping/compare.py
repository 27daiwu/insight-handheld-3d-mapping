"""Three-way back-end comparison with unified trajectory / map / performance metrics.

Rows: vio_raw (reference, no loop closure), custom, rtabmap_vio, rtabmap_odom.
Each SLAM back-end runs in its own subprocess (`insight-mapping backend ...`) so wall time and peak RSS are per back-end
(os.wait4 rusage, includes children such as the rtabmap nodes). The reconstruction stage is identical for all rows
(recon.fusion) and runs in-process on the trajectory expressed in the ground-truth frame (rigid Umeyama alignment).
Map metrics: accuracy / completeness against the map reconstructed from ground-truth poses (same pipeline, same frames),
plus the analytic scene for synthetic data.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import open3d as o3d

from . import evaluate, se3
from .extract import Extracted
from .recon import fusion
from .session import Session
from .slam import pipeline as slam


def load_gt(path):
    g = np.load(path)
    return g["stamps"].astype(np.int64), g["T"]


def gt_at(gt, stamps_ns):
    s, T = gt
    ok = (stamps_ns >= s[0]) & (stamps_ns <= s[-1])
    out = np.tile(np.eye(4), (len(stamps_ns), 1, 1))
    if ok.any():
        out[ok], *_ = se3.interpolate_poses(s.astype(np.float64), T, stamps_ns[ok].astype(np.float64))
    return out, ok


def run_subprocess(argv, cwd=None):
    t = time.time()
    p = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = p.stdout.read()
    _, status, ru = os.wait4(p.pid, 0)
    p.returncode = os.waitstatus_to_exitcode(status)
    return {"returncode": p.returncode, "wall_s": time.time() - t, "peak_rss_mib": ru.ru_maxrss / 1024.0,
            "cpu_user_s": ru.ru_utime, "cpu_sys_s": ru.ru_stime, "log_tail": out.decode(errors="replace")[-1500:]}


def traj_metrics(csv, gt, n_frames_total) -> dict:
    st, T = slam.read_trajectory_csv(csv)
    G, ok = gt_at(gt, st)
    st, T, G = st[ok], T[ok], G[ok]
    if len(st) < 5:
        return {"frames": int(len(st)), "error": "too few frames inside GT range"}
    m = {"frames": int(len(st)), "coverage": float(len(st) / n_frames_total), "path_length_m": evaluate.path_length(T),
         "ate_aligned": evaluate.ate(T, G, True), "ate_anchored": evaluate.ate(T, G, False), "rpe": evaluate.rpe(T, G, 10)}
    # drift: error growth between first and last quarter after alignment
    A = se3.umeyama_align(T[:, :3, 3], G[:, :3, 3])
    e = np.linalg.norm(T[:, :3, 3] @ A[:3, :3].T + A[:3, 3] - G[:, :3, 3], axis=1)
    q = max(len(e) // 4, 1)
    m["drift_m_per_m"] = float((e[-q:].mean() - e[:q].mean()) / max(evaluate.path_length(G), 1e-6))
    return m, A


def write_gt_frame_csv(csv_in, A, csv_out):
    st, T = slam.read_trajectory_csv(csv_in)
    slam.trajectory_to_csv(csv_out, st, A @ T)


def map_metrics(est_ply, ref_ply, analytic=False) -> dict:
    est, ref = o3d.io.read_point_cloud(str(est_ply)), o3d.io.read_point_cloud(str(ref_ply))
    d_er = np.asarray(est.compute_point_cloud_distance(ref))      # accuracy-like (est -> ref)
    d_re = np.asarray(ref.compute_point_cloud_distance(est))      # completeness-like (ref -> est)
    m = {"accuracy_median_m": float(np.median(d_er)), "accuracy_p95_m": float(np.percentile(d_er, 95)),
         "completeness_median_m": float(np.median(d_re)), "completeness_p95_m": float(np.percentile(d_re, 95)),
         "frac_est_within_3cm": float((d_er < 0.03).mean()), "frac_ref_within_3cm": float((d_re < 0.03).mean()),
         "chamfer_m": float(0.5 * (d_er.mean() + d_re.mean()))}
    if analytic:
        from .recon import eval_synth
        m["analytic_median_m"] = eval_synth.evaluate_points(np.asarray(est.points))["median_m"]
        m["analytic_frac_within_3cm"] = eval_synth.evaluate_points(np.asarray(est.points))["frac_within_3cm"]
    return m


def compare(session_dir, work_dir, out_dir, gt_path, backends=("custom", "rtabmap_vio", "rtabmap_odom"), sensor="rgbd",
            recon_cfg: fusion.ReconConfig | None = None, analytic=False, python=None, log=print) -> dict:
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    session = Session.open(session_dir)
    ex = Extracted(work_dir)
    gt = load_gt(gt_path)
    key = "rgb" if "rgb" in session.streams else "left"
    n_total = len(ex.index(key)["stamp_ns"])
    py = python or sys.executable
    recon_cfg = recon_cfg or fusion.ReconConfig()
    rows = {}
    tracks = {}            # row -> csv
    for b in backends:
        bo = out / b
        log(f"== backend {b}")
        pr = run_subprocess([py, "-m", "insight_mapping.cli", "backend", b, str(session_dir), "--work", str(work_dir),
                             "--out", str(bo), "--sensor", sensor])
        row = {"process": {k: pr[k] for k in ("returncode", "wall_s", "peak_rss_mib", "cpu_user_s", "cpu_sys_s")}}
        if pr["returncode"] != 0 or not (bo / "trajectory_opt.csv").exists():
            row["error"] = pr["log_tail"]
            rows[b] = row
            log(f"   FAILED: {pr['log_tail'][-300:]}")
            continue
        rep = json.loads((bo / "backend_report.json").read_text())
        row["loop_closures"] = rep.get("loops")
        row["backend_wall_s"] = rep.get("wall_s")
        for k in ("own_odometry", "feed", "db", "keyframes"):
            if k in rep:
                row[k] = rep[k]
        rows[b] = row
        tracks[b] = bo / "trajectory_opt.csv"
        if b == "custom":
            tracks["vio_raw"] = bo / "trajectory_vio.csv"
            rows["vio_raw"] = {"note": "Insight VIO / odometry as recorded, no loop closure"}
        if b == "rtabmap_vio" or b == "rtabmap_odom":
            tracks[b + "_odom_only"] = bo / "trajectory_odom.csv"
            rows[b + "_odom_only"] = {"note": "odometry used by this back-end before RTAB-Map optimisation"}
    # trajectory metrics + gt-frame trajectories
    gtT, gok = gt_at(gt, ex.index(key)["stamp_ns"])
    gt_csv = out / "gt_trajectory.csv"
    slam.trajectory_to_csv(gt_csv, ex.index(key)["stamp_ns"][gok], gtT[gok])
    A_of = {}
    for name, csv in tracks.items():
        res = traj_metrics(csv, gt, n_total)
        if isinstance(res, tuple):
            rows[name]["trajectory"], A_of[name] = res
            write_gt_frame_csv(csv, A_of[name], out / f"{name}_gtframe.csv")
        else:
            rows[name]["trajectory"] = res
    # maps
    cfgm = recon_cfg
    ref_dir = out / "map_reference_gt"
    rrep = fusion.reconstruct(session, ex, gt_csv, ref_dir, cfgm, log=lambda *_: None)
    rows["_reference"] = {"note": "map reconstructed from ground-truth poses", "points": rrep["points"], "triangles": rrep["mesh_triangles"]}
    for name in list(A_of):
        d = out / f"map_{name}"
        try:
            r = fusion.reconstruct(session, ex, out / f"{name}_gtframe.csv", d, cfgm, log=lambda *_: None)
            rows[name]["map"] = {"points": r["points"], "triangles": r["mesh_triangles"], "recon_s": r["elapsed_s"],
                                 "peak_rss_mib_inprocess": r["peak_rss_mib"], **map_metrics(d / "map.ply", ref_dir / "map.ply", analytic)}
        except Exception as e:
            rows[name]["map_error"] = str(e)
    report = {"session": str(session_dir), "sensor": sensor, "frames": n_total, "recon_config": cfgm.__dict__, "rows": rows}
    (out / "comparison.json").write_text(json.dumps(report, indent=1, default=float))
    (out / "comparison.md").write_text(to_markdown(report))
    return report


def to_markdown(rep) -> str:
    L = [f"### Comparison — {Path(rep['session']).name} (sensor={rep['sensor']}, {rep['frames']} frames)", "",
         "| row | frames | ATE aligned RMSE (m) | ATE anchored RMSE (m) | RPE10 trans (m) / rot (deg) | loops | SLAM wall (s) | peak RSS (MiB) | map accuracy med/p95 (m) | map completeness med/p95 (m) | frac≤3cm (est/ref) |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, r in rep["rows"].items():
        if name.startswith("_"):
            continue
        t, p, m = r.get("trajectory"), r.get("process", {}), r.get("map")
        if "error" in r:
            L.append(f"| {name} | FAILED | | | | | | | | | |"); continue
        if not t or "ate_aligned" not in t:
            L.append(f"| {name} | — | | | | | | | | | |"); continue
        cells = [name, str(t["frames"]), f"{t['ate_aligned']['rmse_m']:.4f}", f"{t['ate_anchored']['rmse_m']:.4f}",
                 f"{t['rpe']['trans_rmse_m']:.4f} / {t['rpe']['rot_rmse_deg']:.2f}", str(r.get("loop_closures", "")),
                 f"{p['wall_s']:.1f}" if p else "", f"{p['peak_rss_mib']:.0f}" if p else "",
                 f"{m['accuracy_median_m']:.4f} / {m['accuracy_p95_m']:.3f}" if m else "",
                 f"{m['completeness_median_m']:.4f} / {m['completeness_p95_m']:.3f}" if m else "",
                 f"{m['frac_est_within_3cm']:.2f} / {m['frac_ref_within_3cm']:.2f}" if m else ""]
        L.append("| " + " | ".join(cells) + " |")
    L += ["", "ATE *aligned* = rigid (Umeyama) alignment; *anchored* = no alignment (frames coincide by construction of the Session/VIO anchor).",
          "Peak RSS = max resident set over the back-end process tree (os.wait4 rusage). `*_odom_only` rows = the odometry fed to RTAB-Map."]
    return "\n".join(L) + "\n"
