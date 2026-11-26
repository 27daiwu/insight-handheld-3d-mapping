"""End-to-end offline reconstruction of one Session -> outputs + reconstruction_report.json (P1-F)."""
from __future__ import annotations

import json
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import open3d as o3d

from . import __version__, extract as extract_mod, quality
from .extract import Extracted
from .recon import fusion
from .session import Session
from .slam import pipeline as slam

OUTPUT_FILES = ["trajectory.csv", "map.ply", "map.pcd", "mesh.ply", "mesh.obj", "reconstruction_report.json"]


def verify_outputs(out_dir) -> dict:
    """Open every output with an independent reader and sanity-check contents."""
    out = Path(out_dir)
    res = {}
    for f in OUTPUT_FILES:
        p = out / f
        if not p.exists():
            res[f] = {"ok": False, "error": "missing"}
            continue
        try:
            if f == "trajectory.csv":
                a = np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2)
                q = np.linalg.norm(a[:, 4:8], axis=1)
                res[f] = {"ok": bool(a.shape[1] == 8 and len(a) > 0 and np.allclose(q, 1, atol=1e-3) and np.isfinite(a).all()), "rows": int(len(a))}
            elif f.startswith("map."):
                pc = o3d.io.read_point_cloud(str(p))
                pts = np.asarray(pc.points)
                res[f] = {"ok": bool(len(pts) > 0 and np.isfinite(pts).all() and pc.has_colors()), "points": int(len(pts))}
            elif f.startswith("mesh."):
                m = o3d.io.read_triangle_mesh(str(p))
                V, T = np.asarray(m.vertices), np.asarray(m.triangles)
                res[f] = {"ok": bool(len(V) > 0 and len(T) > 0 and T.max() < len(V) and np.isfinite(V).all()),
                          "vertices": int(len(V)), "triangles": int(len(T)), "has_colors": bool(m.has_vertex_colors())}
            else:
                json.loads(p.read_text()); res[f] = {"ok": True}
        except Exception as e:
            res[f] = {"ok": False, "error": str(e)[:200]}
    res["all_ok"] = all(v["ok"] for v in res.values() if isinstance(v, dict))
    return res


def run_session(session_dir, out_dir, slam_cfg: slam.SlamConfig | None = None, recon_cfg: fusion.ReconConfig | None = None,
                gt_T=None, topic_map=None, max_frames=None, every=1, allow_missing=(), log=print) -> dict:
    t0 = time.time()
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    session = Session.open(session_dir, topic_map=topic_map)
    timings = {}
    t = time.time()
    extract_mod.extract(session, out, max_frames=max_frames, every=every, log=log)
    timings["extract_s"] = time.time() - t
    ex = Extracted(out)
    t = time.time()
    q = quality.check_session(session, out, log=log, allow_missing=allow_missing)
    timings["quality_s"] = time.time() - t
    if not q["summary"]["usable_for_reconstruction"]:
        rep = {"status": "refused", "reason": "data quality errors", "findings": q["findings"]}
        (out / "reconstruction_report.json").write_text(json.dumps(rep, indent=1))
        raise RuntimeError("Session not usable: " + "; ".join(f["message"] for f in q["findings"] if f["severity"] == "error"))
    t = time.time()
    srep = slam.run(session, ex, out / "slam", slam_cfg, gt_T=gt_T, log=log)
    timings["slam_s"] = time.time() - t
    recon_cfg = recon_cfg or fusion.ReconConfig()
    traj_csv = out / "slam" / ("trajectory_opt.csv" if recon_cfg.trajectory == "optimized" else "trajectory_vio.csv")
    t = time.time()
    rrep = fusion.reconstruct(session, ex, traj_csv, out, recon_cfg, log=log)
    timings["recon_s"] = time.time() - t
    (out / "trajectory.csv").write_text(traj_csv.read_text())

    anomalies = [f["message"] for f in q["findings"] if f["severity"] in ("warn", "error")] + rrep["anomalies"]
    rep = {
        "tool_version": __version__, "session_id": session.session_id, "status": "ok",
        "trajectory": {"file": "trajectory.csv", "source": recon_cfg.trajectory, "frame": "T_odom_rgbcam (optical frame), CSV stamp_ns,tx,ty,tz,qx,qy,qz,qw",
                       "path_length_m": srep["path_length_m"], "drift": srep["drift"],
                       "vs_ground_truth": srep.get("vs_ground_truth"), "vio_pairing": srep["vio_pairing"],
                       "frame_validation": srep.get("frame_validation", {}).get("status"),
                       "stereo_vo_control": srep.get("stereo_vo_control")},
        "loop_closure": {"keyframes": srep["keyframes"], "accepted": srep["loop_closure"]["accepted"],
                         "optimization": srep["optimization"]},
        "reconstruction": {k: rrep[k] for k in ("frames_used", "rgbd_aligned", "depth_source", "points", "mesh_vertices",
                                                  "mesh_triangles", "mesh_vertex_color_coverage", "map_extent_m", "config")},
        "performance": {"reconstruction_elapsed_s": rrep["elapsed_s"], "reconstruction_timings_s": rrep["timings_s"],
                        "stage_timings_s": timings, "total_s": time.time() - t0,
                        "peak_rss_mib": rrep["peak_rss_mib"], "gpu_mem_used_mib": rrep["gpu_mem_used_mib"],
                        "backend": recon_cfg.backend},
        "data_quality": {"summary": q["summary"], "time_sync": q["time_sync"], "calibration": q["calibration"]},
        "anomalies": anomalies,
    }
    (out / "reconstruction_report.json").write_text(json.dumps(rep, indent=1, default=float))
    rep["outputs_verified"] = verify_outputs(out)
    (out / "reconstruction_report.json").write_text(json.dumps(rep, indent=1, default=float))
    return rep
