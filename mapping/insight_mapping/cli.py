"""insight-mapping CLI. Needs `source /opt/ros/humble/setup.bash` (rosbag2)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import quality
from .extract import extract, Extracted
from .session import Session


def main(argv=None):
    ap = argparse.ArgumentParser(prog="insight-mapping")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("validate", "extract", "quality", "run"):
        p = sub.add_parser(name)
        p.add_argument("session")
        p.add_argument("--topic-map")
        if name != "validate":
            p.add_argument("--out", required=True)
        if name in ("quality", "run"):
            p.add_argument("--allow-missing", nargs="*", default=[], help="downgrade missing mandatory streams (e.g. imu for public datasets)")
        if name in ("extract", "run"):
            p.add_argument("--every", type=int, default=1)
            p.add_argument("--max-frames", type=int)
        if name == "run":
            p.add_argument("--backend", default="tensor-cpu", choices=["tensor-cpu", "tensor-cuda"])
            p.add_argument("--voxel", type=float, default=0.02)
            p.add_argument("--stride", type=int, default=2)
            p.add_argument("--trajectory", default="optimized", choices=["optimized", "vio"])
    b = sub.add_parser("bench-gpu"); b.add_argument("session"); b.add_argument("--out", required=True)
    b.add_argument("--trajectory", required=True); b.add_argument("--voxel", type=float, default=0.02)
    v = sub.add_parser("verify"); v.add_argument("out")
    a = ap.parse_args(argv)

    if a.cmd == "validate":
        s = Session.open(a.session, a.topic_map)
        print(json.dumps({"streams": s.streams, "problems": s.problems, "notes": s.stream_notes,
                          "missing_mandatory": s.missing_mandatory()}, indent=1))
        return 1 if any(sev == "error" for sev, _ in s.problems) else 0
    if a.cmd == "extract":
        extract(Session.open(a.session, a.topic_map), a.out, a.max_frames, a.every); return 0
    if a.cmd == "quality":
        r = quality.check_session(Session.open(a.session, a.topic_map), a.out, allow_missing=a.allow_missing)
        return 0 if r["summary"]["usable_for_reconstruction"] else 1
    if a.cmd == "run":
        from .recon.fusion import ReconConfig
        from .run import run_session
        rc = ReconConfig(voxel=a.voxel, sdf_trunc=4 * a.voxel, map_voxel=a.voxel, stride=a.stride, backend=a.backend, trajectory=a.trajectory)
        rep = run_session(a.session, a.out, recon_cfg=rc, topic_map=a.topic_map, max_frames=a.max_frames, every=a.every, allow_missing=a.allow_missing)
        print(json.dumps({k: rep[k] for k in ("status", "reconstruction", "performance", "outputs_verified")}, indent=1, default=float))
        return 0 if rep["outputs_verified"]["all_ok"] else 1
    if a.cmd == "bench-gpu":
        from .recon.bench import bench
        print(json.dumps(bench(a.session, a.out, a.trajectory, a.voxel), indent=1)); return 0
    if a.cmd == "verify":
        from .run import verify_outputs
        r = verify_outputs(a.out); print(json.dumps(r, indent=1)); return 0 if r["all_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
