"""End-to-end driver: bag -> import/scan -> calibration check -> stereo cache -> VO -> dense map -> report."""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from . import calibcheck, dense, report, scan as scanmod, stereo, vo
from .reader import BagSource
from .rig import load_rig

LIMITATIONS = [
    "Absolute scale rests on the vendor baseline (CameraInfo P / TF); no known-length object was measured. Use `i9 scale-check` on a recording with a fiducial of known size.",
    "No loop closure / global optimisation yet: drift accumulates (open-loop VO). Drift can only be measured against a revisit or a reference.",
    "IMU, Depth and VIO are indexed but deliberately unused (semantics [?] in the contract). VIO is compared only as a non-authoritative reference when present.",
    "Colour is infrared grey; RGB texture needs the verified RGB<->left extrinsic ([O] in the contract).",
    "Quality grades use first-pass thresholds set without real Insight 9 data.",
]


def vio_reference(src: BagSource, roles: dict, cache_stamps, poses) -> dict | None:
    """Non-authoritative comparison with the camera's VIO (semantics unverified): only translation-path length ratio and a
    similarity alignment RMS are reported; nothing from VIO feeds the map."""
    if "vio" not in roles:
        return None
    from ..bagio import fast_header
    ts, P = [], []
    for raw, m in src.iter_msgs([roles["vio"]]):
        ts.append(fast_header(raw.data)[0]); p = m.pose.position; P.append([p.x, p.y, p.z])
    if len(ts) < 10:
        return None
    ts, P = np.array(ts, np.int64), np.array(P)
    q = cache_stamps
    ok = (q >= ts[0]) & (q <= ts[-1])
    if ok.sum() < 10:
        return {"note": "no time overlap"}
    Pv = np.stack([np.interp(q[ok], ts, P[:, k]) for k in range(3)], 1)
    Ps = poses[ok, :3, 3]
    from .. import se3
    Tm = se3.umeyama_align(Ps, Pv, with_scale=True)
    s = float(np.cbrt(abs(np.linalg.det(Tm[:3, :3]))))
    A = (Ps @ Tm[:3, :3].T + Tm[:3, 3]) - Pv
    ls, lv = np.linalg.norm(np.diff(Ps, axis=0), axis=1).sum(), np.linalg.norm(np.diff(Pv, axis=0), axis=1).sum()
    return {"name": "camera vio (semantics UNVERIFIED, not ground truth)",
            "metrics": {"sim3_scale_stereo_to_vio": s, "path_length_stereo_m": float(ls), "path_length_vio_m": float(lv),
                        "sim3_aligned_rmse_m": float(np.sqrt((A ** 2).sum(1).mean())), "n": int(ok.sum())}}


def run_all(path, out_dir, data_origin="UNSPECIFIED", overrides=None, stride=1, dense_stride=3, max_seconds=0.0, max_frames=0,
            workers=0, vo_params=None, dense_params=None, stereo_params=None, gt_npz=None, skip_dense=False, log=print) -> dict:
    t0 = time.time()
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    rep: dict = {"input": str(path), "data_origin": data_origin}
    log("== 1/6 import & scan")
    sc = scanmod.scan(path, overrides, log=log)
    rep["scan"] = sc
    (out / "scan.json").write_text(json.dumps(sc, indent=1, default=float))
    if not sc["usable_for_stereo"]:
        rep["status"] = "refused"
        (out / "reconstruction_report.json").write_text(json.dumps(rep, indent=1, default=float))
        raise RuntimeError("bag unusable for stereo: " + "; ".join(f["message"] for f in sc["findings"] if f["severity"] == "error"))
    src = BagSource(path)
    roles = sc["roles_resolved"]
    log("== 2/6 stereo sync / CameraInfo / calibration")
    rig = load_rig(src, roles)
    cal = calibcheck.run_calibcheck(src, roles, rig, log=log)
    rep["calibration"] = cal
    (out / "calibration_check.json").write_text(json.dumps(cal, indent=1, default=float))
    if cal["verdict"]["calibration_ok_for_metric_reconstruction"] == "FAIL":
        rep["status"] = "refused_calibration"
        report.write(out, {**rep, "dense": None, "limitations": LIMITATIONS})
        raise RuntimeError("stereo calibration FAIL: " + "; ".join(cal["verdict"]["errors"]))
    log("== 3/6 per-frame stereo (disparity + features)")
    sp = stereo_params or stereo.StereoParams()
    sp.stride, sp.max_seconds, sp.max_frames = stride, max_seconds, max_frames
    rep["stereo"] = stereo.build_cache(src, roles, out, sp, workers=workers, log=log)
    cache = stereo.StereoCache(out)
    log("== 4/6 stereo visual odometry")
    res = vo.run_vo(cache, rig, vo_params, out_dir=out, log=log)
    rep["vo"] = res["report"]
    if gt_npz:
        g = np.load(gt_npz)
        from .. import evaluate
        m = {k: v for k, v in _ate(cache.stamps, res["poses"], g["stamps"], g["T"]).items()}
        rep["vs_reference"] = {"name": "simulated ground truth (unit-test only)", "metrics": m}
    else:
        r = vio_reference(src, roles, cache.stamps, res["poses"])
        if r:
            rep["vs_reference"] = r
    if not skip_dense:
        log("== 5/6 dense reconstruction")
        dp = dense_params or dense.DenseParams()
        dp.stride = dense_stride
        rep["dense"] = dense.reconstruct(cache, rig, out / "trajectory.csv", out, dp, log=log)
        rep["depth_consistency"] = dense.depth_consistency(cache, rig, out / "trajectory.csv", dp)
        rep["planarity"] = dense.map_planarity(out / "map.ply")
    log("== 6/6 report")
    rep["limitations"] = LIMITATIONS
    rep["status"] = "ok"
    rep["total_s"] = time.time() - t0
    report.write(out, rep)
    return rep


def _ate(stamps, poses, gt_stamps, gt_T):
    from .. import se3
    common, ia, ib = np.intersect1d(stamps, gt_stamps, return_indices=True)
    A, B = poses[ia][:, :3, 3], gt_T[ib][:, :3, 3]
    Tm = se3.umeyama_align(A, B)           # rigid alignment only: scale error is *not* absorbed
    e = (A @ Tm[:3, :3].T + Tm[:3, 3]) - B
    Ts = se3.umeyama_align(A, B, with_scale=True)
    s = float(np.cbrt(abs(np.linalg.det(Ts[:3, :3]))))
    pl_a, pl_b = np.linalg.norm(np.diff(A, axis=0), axis=1).sum(), np.linalg.norm(np.diff(B, axis=0), axis=1).sum()
    return {"ate_rigid_rmse_m": float(np.sqrt((e ** 2).sum(1).mean())), "ate_max_m": float(np.linalg.norm(e, axis=1).max()),
            "scale_ratio_est_over_gt": s, "path_length_ratio": float(pl_a / pl_b), "n": int(len(ia))}
