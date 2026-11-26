"""P1-E: honest CPU vs GPU measurement of the TSDF stage (Open3D VoxelBlockGrid)."""
from __future__ import annotations

import gc
import json
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import open3d as o3d

from ..extract import Extracted
from ..session import Session
from ..slam import pipeline as slam
from . import fusion


def cuda_status() -> dict:
    s = {"open3d": o3d.__version__, "cuda_available": bool(o3d.core.cuda.is_available()),
         "devices": int(o3d.core.cuda.device_count()), "built_with_cuda": bool(getattr(o3d, "_build_config", {}).get("BUILD_CUDA_MODULE", None))}
    if s["cuda_available"]:
        try:
            a = o3d.core.Tensor(np.ones((512, 512), np.float32), device=o3d.core.Device("CUDA:0"))
            s["tensor_op_on_gpu_ok"] = bool(float((a @ a).cpu().numpy()[0, 0]) == 512.0)
        except Exception as e:
            s["tensor_op_on_gpu_ok"] = False; s["error"] = str(e)[:200]
    return s


def bench(session_dir, work_dir, traj_csv, voxel=0.02, stride=2, repeats=2) -> dict:
    session = Session.open(session_dir)
    ex = Extracted(work_dir)
    stamps, T = slam.read_trajectory_csv(traj_csv)
    res = {"cuda": cuda_status(), "runs": {}}
    base = fusion.ReconConfig(voxel=voxel, sdf_trunc=4 * voxel, stride=stride)
    for backend in ("tensor-cpu", "tensor-cuda"):
        if backend == "tensor-cuda" and not res["cuda"]["cuda_available"]:
            res["runs"][backend] = {"skipped": "no CUDA device in this Open3D build"}; continue
        cfg = replace(base, backend=backend)
        fs = fusion.FrameSet(session, ex, stamps, T, cfg)
        frames = [fs.load(n) for n in range(len(fs.items))]    # exclude image I/O from the timing
        runs = []
        for r in range(repeats + 1):                            # first run = warm-up (CUDA init/kernels)
            gc.collect()
            g0 = fusion.gpu_mem_used_mib()
            vol = fusion.TSDF(cfg, fs.cam_d)
            t = time.time()
            for f in frames:
                vol.integrate(f["depth"], f["T_w_d"])
            if backend == "tensor-cuda":
                o3d.core.cuda.synchronize()
            ti = time.time() - t
            t = time.time()
            m = vol.mesh()
            te = time.time() - t
            g1 = fusion.gpu_mem_used_mib()
            runs.append({"integrate_s": ti, "extract_s": te, "triangles": len(m.triangles), "vertices": len(m.vertices),
                         "gpu_mem_delta_mib": None if g0 is None or g1 is None else g1 - g0,
                         "frames": len(frames), "ms_per_frame": 1000 * ti / len(frames)})
            if r == 0:
                runs[-1]["warmup"] = True
            if r == repeats:
                last = m
            del vol
        meas = [x for x in runs if not x.get("warmup")]
        res["runs"][backend] = {"warmup": runs[0], "measured": meas,
                                "integrate_s_mean": float(np.mean([x["integrate_s"] for x in meas])),
                                "extract_s_mean": float(np.mean([x["extract_s"] for x in meas]))}
        res["runs"][backend]["_mesh"] = last
    if "_mesh" in res["runs"].get("tensor-cuda", {}):
        a, b = res["runs"]["tensor-cpu"]["_mesh"], res["runs"]["tensor-cuda"]["_mesh"]
        pa = o3d.geometry.PointCloud(a.vertices)
        pb = b.sample_points_uniformly(max(200000, 3 * len(b.vertices)))
        d = np.asarray(pa.compute_point_cloud_distance(pb))
        res["cpu_vertices_to_gpu_surface_m"] = {"median": float(np.median(d)), "p99": float(np.percentile(d, 99))}
        res["speedup_integrate"] = res["runs"]["tensor-cpu"]["integrate_s_mean"] / res["runs"]["tensor-cuda"]["integrate_s_mean"]
    for v in res["runs"].values():
        v.pop("_mesh", None)
    res["note"] = ("GPU speedup is only claimed from the measured `speedup_integrate` of this run; "
                   "`cuda_available` alone does not imply the pipeline uses the GPU (only TSDF integration/extraction "
                   "run through t.geometry.VoxelBlockGrid on CUDA:0; feature extraction, loop closure, pose graph, "
                   "point-cloud build, colour fusion are CPU).")
    (Path(work_dir) / "gpu_benchmark.json").write_text(json.dumps(res, indent=1))
    return res
