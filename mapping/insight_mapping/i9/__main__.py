"""python -m insight_mapping.i9 <cmd>   (needs the mapping venv; source scripts/env.sh only for SQLite3 bags)"""
from __future__ import annotations

import argparse
import json
import sys


def main(argv=None):
    ap = argparse.ArgumentParser(prog="insight_mapping.i9", description="Insight 9 stereo-first mapping")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan", help="import a bag: topics, types, stamps, contract comparison")
    s.add_argument("bag"); s.add_argument("--out"); s.add_argument("--topic", action="append", default=[], metavar="ROLE=TOPIC")
    c = sub.add_parser("check", help="stereo sync + CameraInfo + calibration verdict")
    c.add_argument("bag"); c.add_argument("--out"); c.add_argument("--pairs", type=int, default=24); c.add_argument("--topic", action="append", default=[])
    r = sub.add_parser("run", help="end-to-end: scan, check, stereo VO, dense map, report")
    r.add_argument("bag"); r.add_argument("--out", required=True)
    r.add_argument("--origin", default="UNSPECIFIED", help="e.g. REAL_INSIGHT9_FW2.1.5 | SIMULATED_UNIT_TEST")
    r.add_argument("--stride", type=int, default=1, help="use every Nth stereo pair for VO")
    r.add_argument("--dense-stride", type=int, default=3, help="every Nth VO frame is fused")
    r.add_argument("--max-seconds", type=float, default=0.0); r.add_argument("--max-frames", type=int, default=0)
    r.add_argument("--workers", type=int, default=0); r.add_argument("--voxel", type=float, default=0.02)
    r.add_argument("--tsdf-backend", default="tensor-cuda", choices=["tensor-cpu", "tensor-cuda"])
    r.add_argument("--gt-npz", help="simulated ground truth (tests only)"); r.add_argument("--skip-dense", action="store_true")
    r.add_argument("--topic", action="append", default=[])
    k = sub.add_parser("scale-check", help="independent scale check with an ArUco marker of known size")
    k.add_argument("bag"); k.add_argument("--marker-size", type=float, required=True, help="black square edge in metres")
    k.add_argument("--out"); k.add_argument("--topic", action="append", default=[])
    a = ap.parse_args(argv)
    ov = dict(kv.split("=", 1) for kv in getattr(a, "topic", []))
    if a.cmd == "scan":
        from . import scan
        rep = scan.scan(a.bag, ov)
        if a.out:
            open(a.out, "w").write(json.dumps(rep, indent=1, default=float))
        else:
            print(json.dumps({k: rep[k] for k in ("storage", "roles_resolved", "usable_for_stereo")}, indent=1))
        return 0 if rep["usable_for_stereo"] else 1
    if a.cmd == "check":
        from . import scan, calibcheck
        from .reader import BagSource
        from .rig import load_rig
        sc = scan.scan(a.bag, ov, log=lambda *_: None)
        src = BagSource(a.bag)
        res = calibcheck.run_calibcheck(src, sc["roles_resolved"], load_rig(src, sc["roles_resolved"]), n_pairs=a.pairs)
        if a.out:
            open(a.out, "w").write(json.dumps(res, indent=1, default=float))
        return 0 if res["verdict"]["calibration_ok_for_metric_reconstruction"] != "FAIL" else 1
    if a.cmd == "run":
        from . import pipeline, dense
        dp = dense.DenseParams(voxel=a.voxel, map_voxel=a.voxel, sdf_trunc=4 * a.voxel, backend=a.tsdf_backend)
        rep = pipeline.run_all(a.bag, a.out, a.origin, ov, a.stride, a.dense_stride, a.max_seconds, a.max_frames, a.workers,
                               dense_params=dp, gt_npz=a.gt_npz, skip_dense=a.skip_dense)
        print(json.dumps({"status": rep["status"], "grades": rep.get("grades"), "out": a.out}, indent=1))
        return 0
    if a.cmd == "scale-check":
        from . import scale_check
        rep = scale_check.scale_check(a.bag, a.marker_size, overrides=ov)
        print(json.dumps(rep, indent=1))
        if a.out:
            open(a.out, "w").write(json.dumps(rep, indent=1))
        return 0 if rep["verdict"].startswith("SCALE_") and "WRONG" not in rep["verdict"] else 1
    return 2


if __name__ == "__main__":
    sys.exit(main())
