"""Uniform entry point for the SLAM back-ends compared in `compare.py`.

  custom         in-repo VIO-seeded keyframe graph (ORB+depth-PnP loops, SE(3) pose-graph)         [slam/pipeline.py]
  rtabmap_vio    RTAB-Map with Insight VIO as external odometry (loop closure + graph optimisation)
  rtabmap_odom   RTAB-Map with its own visual odometry (rgbd_odometry / stereo_odometry), VIO only anchors the frame
Every back-end writes <out>/trajectory_opt.csv (per-frame T_world_rgbcam, contract of fusion) + <out>/backend_report.json.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

from .extract import Extracted
from .session import Session

BACKENDS = ["custom", "rtabmap_vio", "rtabmap_odom"]


def run_backend(name: str, session_dir, work_dir, out_dir, sensor: str = "rgbd", stride: int = 1, max_frames=None,
                topic_map=None, rtabmap_overrides=None, log=print) -> dict:
    session = Session.open(session_dir, topic_map=topic_map)
    ex = Extracted(work_dir)
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    if name == "custom":
        from .slam import pipeline
        rep = pipeline.run(session, ex, out, pipeline.SlamConfig(use_stereo_control=False), log=log)
        rep["backend"] = "custom"
        rep["loops"] = rep["loop_closure"]["accepted"]
    elif name in ("rtabmap_vio", "rtabmap_odom"):
        from .rtabmap import backend as rb
        cfg = rb.RtabmapConfig(odom="vio" if name == "rtabmap_vio" else "own", sensor=sensor, stride=stride, max_frames=max_frames)
        for k, v in (rtabmap_overrides or {}).items():
            if k.startswith("rtabmap:"):
                cfg.rtabmap_params[k[8:]] = v
            elif k.startswith("odom:"):
                cfg.odom_params[k[5:]] = v
            else:
                setattr(cfg, k, type(getattr(cfg, k))(v) if getattr(cfg, k) is not None else float(v))
        rep = rb.run(session, ex, out, cfg, log=log)
        rep["loops"] = rep["db"]["loop_closures"]
    else:
        raise ValueError(f"unknown backend {name}; choose from {BACKENDS}")
    rep["wall_s"] = time.time() - t0
    (out / "backend_report.json").write_text(json.dumps(rep, indent=1, default=float))
    return rep
