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
            p.add_argument("--tsdf-backend", default="tensor-cpu", choices=["tensor-cpu", "tensor-cuda"])
            p.add_argument("--voxel", type=float, default=0.02)
            p.add_argument("--stride", type=int, default=2)
            p.add_argument("--trajectory", default="optimized", choices=["optimized", "vio"])
            p.add_argument("--backend", dest="slam_backend", default="custom", choices=["custom", "rtabmap_vio", "rtabmap_odom"])
            p.add_argument("--sensor", default="rgbd", choices=["rgbd", "stereo"])
            p.add_argument("--no-texture", action="store_true")
    b = sub.add_parser("bench-gpu"); b.add_argument("session"); b.add_argument("--out", required=True)
    b.add_argument("--trajectory", required=True); b.add_argument("--voxel", type=float, default=0.02)
    v = sub.add_parser("verify"); v.add_argument("out")
    be = sub.add_parser("backend", help="run one SLAM back-end on an extracted Session")
    be.add_argument("name", choices=["custom", "rtabmap_vio", "rtabmap_odom"]); be.add_argument("session")
    be.add_argument("--work", required=True); be.add_argument("--out", required=True)
    be.add_argument("--sensor", default="rgbd", choices=["rgbd", "stereo"]); be.add_argument("--stride", type=int, default=1)
    be.add_argument("--max-frames", type=int); be.add_argument("--topic-map")
    be.add_argument("--set", action="append", default=[], metavar="K=V",
                    help="RtabmapConfig field (odom_cov_lin=1e-5), 'rtabmap:Group/Param=v' or 'odom:Group/Param=v'")
    cp = sub.add_parser("compare", help="three-way back-end comparison (needs ground truth npz: stamps, T)")
    cp.add_argument("session"); cp.add_argument("--work", required=True); cp.add_argument("--out", required=True)
    cp.add_argument("--gt", required=True); cp.add_argument("--sensor", default="rgbd", choices=["rgbd", "stereo"])
    cp.add_argument("--backends", nargs="+", default=["custom", "rtabmap_vio", "rtabmap_odom"])
    cp.add_argument("--analytic", action="store_true", help="synthetic scene: also score against the analytic room")
    cp.add_argument("--voxel", type=float, default=0.02)
    mf = sub.add_parser("make-fixture", help="create a test Session (+ ground-truth npz) from synthetic or TUM data")
    mf.add_argument("kind", choices=["synthetic", "tum"]); mf.add_argument("--root", required=True)
    mf.add_argument("--id", required=True); mf.add_argument("--tum-dir"); mf.add_argument("--frames", type=int, default=240)
    mf.add_argument("--every", type=int, default=2); mf.add_argument("--aligned", action="store_true")
    mf.add_argument("--storage", default="sqlite3", choices=["sqlite3", "mcap"])
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
        rc = ReconConfig(voxel=a.voxel, sdf_trunc=4 * a.voxel, map_voxel=a.voxel, stride=a.stride, backend=a.tsdf_backend, trajectory=a.trajectory)
        rep = run_session(a.session, a.out, recon_cfg=rc, topic_map=a.topic_map, max_frames=a.max_frames, every=a.every, allow_missing=a.allow_missing,
                          backend=a.slam_backend, sensor=a.sensor, texture=not a.no_texture)
        print(json.dumps({k: rep[k] for k in ("status", "reconstruction", "performance", "outputs_verified")}, indent=1, default=float))
        return 0 if rep["outputs_verified"]["all_ok"] else 1
    if a.cmd == "bench-gpu":
        from .recon.bench import bench
        print(json.dumps(bench(a.session, a.out, a.trajectory, a.voxel), indent=1)); return 0
    if a.cmd == "backend":
        from .backends import run_backend
        rep = run_backend(a.name, a.session, a.work, a.out, a.sensor, a.stride, a.max_frames, a.topic_map,
                          dict(kv.split("=", 1) for kv in a.set))
        print(json.dumps({"backend": a.name, "wall_s": rep["wall_s"], "loops": rep.get("loops")})); return 0
    if a.cmd == "compare":
        from .compare import compare
        from .recon.fusion import ReconConfig
        rep = compare(a.session, a.work, a.out, a.gt, a.backends, a.sensor, ReconConfig(voxel=a.voxel, sdf_trunc=4 * a.voxel, map_voxel=a.voxel),
                      analytic=a.analytic)
        print((Path(a.out) / "comparison.md").read_text()); return 0
    if a.cmd == "make-fixture":
        import numpy as np
        if a.kind == "synthetic":
            from .fixtures import synthetic
            root, gt = synthetic.make_session(a.root, a.id, n_frames=a.frames, aligned=a.aligned, storage=a.storage)
            np.savez(Path(a.root) / f"{a.id}_gt.npz", stamps=gt["stamps_ns"], T=gt["T_world_rgb"])
        else:
            from .fixtures import tum
            root, gt = tum.make_session(a.tum_dir, a.root, a.id, every=a.every, storage=a.storage)
            np.savez(Path(a.root) / f"{a.id}_gt.npz", stamps=gt["stamps_ns"], T=gt["T_world_cam"])
        print(root); return 0
    if a.cmd == "verify":
        from .run import verify_outputs
        r = verify_outputs(a.out); print(json.dumps(r, indent=1)); return 0 if r["all_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
