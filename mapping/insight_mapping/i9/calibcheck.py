"""Stage 2 - stereo synchronisation, CameraInfo and calibration verification (is metric reconstruction justified?).

Evidence classes (kept apart in the report):
  A. internal consistency of the vendor numbers (CameraInfo K/P/D/R, TF baseline, size, frame ids)
  B. data-driven geometry checks on real image pairs (epipolar rows, disparity sign/range, rotation residual)
  C. hypotheses that need an external reference (Depth stream unit, VIO scale): reported as ratios, never as a verdict
A *physical* scale check (known-length object) is the only independent proof of the metric scale; none of A-C is.
"""
from __future__ import annotations

import cv2
import numpy as np

from ..bagio import image_to_numpy, stamp_to_ns
from .reader import BagSource
from .rig import Rig


def sync_check(src: BagSource, roles: dict) -> dict:
    """Pair left/right by header stamp over the whole bag (header-only pass)."""
    from ..bagio import fast_header
    stl, str_ = [], []
    for raw in src.iter_raw([roles["stereo_left"], roles["stereo_right"]]):
        h = fast_header(raw.data)
        (stl if raw.topic == roles["stereo_left"] else str_).append(h[0])
    L, R = np.array(stl, np.int64), np.array(str_, np.int64)
    sr = np.sort(R)
    i = np.clip(np.searchsorted(sr, L), 1, max(len(sr) - 1, 1))
    d = np.minimum(np.abs(L - sr[i - 1]), np.abs(sr[min(i.max(), len(sr) - 1)] - L)) if len(sr) > 1 else np.zeros(len(L))
    d = np.minimum(np.abs(L - sr[i - 1]), np.abs(L - sr[np.minimum(i, len(sr) - 1)]))
    return {"left": int(len(L)), "right": int(len(R)), "exact_stamp_pairs": int(np.intersect1d(L, R).size),
            "max_skew_ms": float(d.max() / 1e6) if len(d) else None, "median_skew_ms": float(np.median(d) / 1e6) if len(d) else None,
            "unpaired_left": int((d > 5e6).sum()), "pairing_rule": "exact header stamp (contract: L/R stamps bit-identical)"}


def sample_pairs(src: BagSource, roles: dict, n_pairs: int, stamps_hint=None):
    """Decode ~n_pairs evenly spread (L,R) image pairs (one pass)."""
    tl, tr = roles["stereo_left"], roles["stereo_right"]
    total = None
    for c in src.connections():
        if c.topic == tl:
            total = c.count
    step = max(1, (total or 0) // max(n_pairs, 1)) if total else 1
    pend: dict = {}
    k = 0
    out = []
    for raw in src.iter_raw([tl, tr]):
        h_ns = None
        from ..bagio import fast_header
        h_ns = fast_header(raw.data)[0]
        if raw.topic == tl:
            if k % step == 0:
                pend[h_ns] = [raw, None]
            k += 1
        elif h_ns in pend:
            pend[h_ns][1] = raw
            a, b = pend.pop(h_ns)
            out.append((h_ns, image_to_numpy(src.decode(a)), image_to_numpy(src.decode(b))))
            if len(out) >= n_pairs:
                break
    return out


def camerainfo_checks(rig: Rig, size_wh_images=None) -> tuple[dict, list]:
    L, R = rig.left, rig.right
    F: list = []
    rep: dict = {"size": [L.width, L.height], "left": {"K": L.K, "P": L.P, "D": L.D, "R": L.R, "model": L.distortion_model, "frame_id": L.frame_id},
                 "right": {"K": R.K, "P": R.P, "D": R.D, "R": R.R, "model": R.distortion_model, "frame_id": R.frame_id},
                 "variants_over_recording": rig.info_variants}
    if any(v > 1 for v in rig.info_variants.values()):
        F.append(("error", f"CameraInfo content changes during the recording ({rig.info_variants}); per-frame intrinsics are not supported"))
    if (L.width, L.height) != (R.width, R.height):
        F.append(("error", "left/right CameraInfo sizes differ"))
    if size_wh_images and tuple(size_wh_images) != (L.width, L.height):
        F.append(("error", f"CameraInfo size {L.width}x{L.height} != image size {size_wh_images}"))
    if L.fx <= 0 or L.fy <= 0:
        F.append(("error", "left CameraInfo has non-positive focal length"))
    Kd = np.abs(np.array(L.K) - np.array(R.K)).max()
    rep["left_right_K_max_diff_px"] = float(Kd)
    if Kd > 1e-3:
        F.append(("error", f"left/right K differ by {Kd:.3f}: images are not rectified to a common model"))
    pd = np.abs(np.array([L.P[0] - L.K[0], L.P[2] - L.K[2], L.P[5] - L.K[5], L.P[6] - L.K[5 + 0]]))
    rep["P_vs_K_left_max_diff"] = float(np.abs(np.array(L.P)[[0, 2, 5, 6]] - np.array(L.K)[[0, 2, 4, 5]]).max())
    rep["distortion_nonzero"] = bool(L.has_distortion or R.has_distortion)
    rep["R_is_identity"] = bool(np.allclose(L.R, np.eye(3).ravel()) and np.allclose(R.R, np.eye(3).ravel()))
    if rep["distortion_nonzero"]:
        F.append(("warn", "CameraInfo D != 0: images may not be rectified; phase 1 would need to rectify them (done in i9.stereo_vo only for plumb_bob)"))
    if abs(L.cx - L.width / 2) > 0.1 * L.width or abs(L.cy - L.height / 2) > 0.1 * L.height:
        F.append(("warn", "principal point > 10% from image centre"))
    # baselines
    rep["baseline_from_P_m"] = rig.baseline_P
    rep["baseline_from_tf_m"] = rig.baseline_tf
    if rig.baseline_P is None:
        F.append(("error", "right P[0,3] == 0: no baseline in CameraInfo, metric stereo impossible"))
    elif rig.baseline_P <= 0:
        F.append(("error", f"baseline from P is {rig.baseline_P:.4f} m (<=0): right camera on the wrong side or sign convention differs"))
    if rig.baseline_P and rig.baseline_tf:
        dd = abs(rig.baseline_P - rig.baseline_tf)
        rep["baseline_P_vs_tf_diff_mm"] = dd * 1e3
        if dd > 0.002:
            F.append(("error", f"baseline mismatch CameraInfo {rig.baseline_P*1e3:.2f} mm vs TF {rig.baseline_tf*1e3:.2f} mm"))
    elif rig.baseline_tf is None:
        F.append(("warn", "no TF between left and right frames in /tf_static: baseline has a single source"))
    if rig.T_left_right is not None:
        t = rig.T_left_right[:3, 3]
        rot = float(np.degrees(np.linalg.norm(__import__("scipy.spatial.transform", fromlist=["Rotation"]).Rotation.from_matrix(rig.T_left_right[:3, :3]).as_rotvec())))
        rep["tf_left_right"] = {"t_m": t.tolist(), "rot_deg": rot}
        if rot > 0.2:
            F.append(("warn", f"TF left->right has {rot:.2f} deg rotation but CameraInfo claims rectified (R=I)"))
        if abs(t[1]) > 2e-3 or abs(t[2]) > 2e-3:
            F.append(("warn", f"TF left->right translation not along x ({t}): inconsistent with horizontal rectified stereo"))
        if rig.baseline_P and t[0] * rig.baseline_P < 0:
            F.append(("error", "TF and CameraInfo disagree on which side the right camera is"))
    return rep, F


def epipolar_check(pairs, rig: Rig, max_disp: float = 160.0) -> tuple[dict, list]:
    """Evidence B: do real pairs behave like a rectified stereo pair with positive disparity?"""
    F: list = []
    orb = cv2.ORB_create(2000, fastThreshold=10)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    dy, dx, xs, ys, per = [], [], [], [], []
    Fres, ninl = [], []
    for stamp, l, r in pairs:
        kl, dl = orb.detectAndCompute(l, None)
        kr, dr = orb.detectAndCompute(r, None)
        if dl is None or dr is None or len(kl) < 30 or len(kr) < 30:
            per.append({"stamp": int(stamp), "matches": 0}); continue
        m = bf.match(dl, dr)
        pl = np.array([kl[x.queryIdx].pt for x in m]); pr = np.array([kr[x.trainIdx].pt for x in m])
        # coarse filter by the rectified-geometry hypothesis is deliberately NOT applied: use a loose gate and measure
        d = pl[:, 0] - pr[:, 0]; v = pl[:, 1] - pr[:, 1]
        keep = (np.abs(v) < 8) & (np.abs(d) < max_disp + 50)
        if keep.sum() < 20:
            per.append({"stamp": int(stamp), "matches": int(len(m)), "kept": int(keep.sum())}); continue
        pl, pr, d, v = pl[keep], pr[keep], d[keep], v[keep]
        # robust: only matches whose vertical offset is consistent are 'inliers' of the hypothesis; report fraction too
        inl = np.abs(v - np.median(v)) < 1.5
        dy.append(v[inl]); dx.append(d[inl]); xs.append(pl[inl, 0]); ys.append(pl[inl, 1])
        Fm, mask = cv2.findFundamentalMat(pl, pr, cv2.FM_RANSAC, 1.0, 0.999)
        e = None
        if Fm is not None and Fm.shape == (3, 3):
            Fn = Fm / (np.linalg.norm(Fm))
            ideal = np.array([[0, 0, 0], [0, 0, -1], [0, 1, 0]]) / np.sqrt(2)
            e = float(min(np.linalg.norm(Fn - ideal), np.linalg.norm(Fn + ideal)))
            Fres.append(e); ninl.append(int(mask.sum()))
        per.append({"stamp": int(stamp), "matches": int(len(m)), "kept": int(keep.sum()), "vertical_consistent": int(inl.sum()),
                    "median_dy_px": float(np.median(v)), "F_distance_to_rectified_form": e})
    if not dy:
        F.append(("error", "no usable stereo matches in sampled pairs (texture/exposure?)"))
        return {"pairs_sampled": len(pairs), "per_pair": per}, F
    dy, dx, xs, ys = map(np.concatenate, (dy, dx, xs, ys))
    # vertical-offset model dy = a + b*x + c*y : constant (a) = vertical misalignment, b/c = relative rotation/tilt
    A = np.stack([np.ones_like(xs), xs - rig.cx, ys - rig.cy], 1)
    coef, *_ = np.linalg.lstsq(A, dy, rcond=None)
    resid = dy - A @ coef
    rep = {
        "pairs_sampled": len(pairs), "matches_used": int(len(dy)),
        "dy_px": {"median": float(np.median(dy)), "mad_sigma": float(1.4826 * np.median(np.abs(dy - np.median(dy)))),
                  "p95_abs": float(np.percentile(np.abs(dy), 95))},
        "dy_model_px": {"offset": float(coef[0]), "per_px_x": float(coef[1]), "per_px_y": float(coef[2]),
                        "residual_std": float(resid.std()),
                        "edge_drift_x_px": float(coef[1] * rig.left.width), "edge_drift_y_px": float(coef[2] * rig.left.height)},
        "disparity_px": {"min": float(dx.min()), "p5": float(np.percentile(dx, 5)), "median": float(np.median(dx)),
                         "p95": float(np.percentile(dx, 95)), "max": float(dx.max()),
                         "positive_fraction": float((dx > 0).mean())},
        "F_distance_to_rectified_form_median": float(np.median(Fres)) if Fres else None,
        "per_pair": per,
    }
    if abs(rep["dy_px"]["median"]) > 0.7 or rep["dy_px"]["mad_sigma"] > 0.7:
        F.append(("error" if abs(rep["dy_px"]["median"]) > 2 else "warn",
                  f"rows not aligned: dy median {rep['dy_px']['median']:.2f} px, sigma {rep['dy_px']['mad_sigma']:.2f} px: "
                  "images are not exactly rectified; semi-global matching quality will suffer"))
    if max(abs(rep["dy_model_px"]["edge_drift_x_px"]), abs(rep["dy_model_px"]["edge_drift_y_px"])) > 1.0:
        F.append(("warn", f"vertical offset varies across the image by >1 px (x:{rep['dy_model_px']['edge_drift_x_px']:.2f}, y:{rep['dy_model_px']['edge_drift_y_px']:.2f}): residual rotation between L/R"))
    if rep["disparity_px"]["positive_fraction"] < 0.98:
        F.append(("error", f"only {rep['disparity_px']['positive_fraction']:.1%} of matches have positive disparity: left/right swapped or not rectified"))
    return rep, F


def depth_uncertainty_table(fx: float, baseline: float, sigma_d: float = 0.25) -> dict:
    """Theoretical stereo depth quantisation/noise: sigma_Z = Z^2 sigma_d / (fx B)."""
    out = {}
    for z in (0.5, 1, 2, 3, 5, 8):
        out[str(z)] = {"disparity_px": fx * baseline / z, "sigma_z_mm_at_0.25px": z * z * sigma_d / (fx * baseline) * 1e3}
    out["min_range_for_96px_search_m"] = fx * baseline / 96.0
    out["max_range_for_1px_disparity_m"] = fx * baseline / 1.0
    return out


def stereo_depth_sgbm(l, r, rig: Rig, num_disp=128, block=5):
    sg = cv2.StereoSGBM_create(0, num_disp, block, 8 * block * block, 32 * block * block, uniquenessRatio=8,
                               speckleWindowSize=60, speckleRange=2, disp12MaxDiff=1, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    d = sg.compute(l, r).astype(np.float32) / 16.0
    with np.errstate(divide="ignore", invalid="ignore"):
        z = rig.fx * rig.baseline / d
    z[(d <= 1.0) | ~np.isfinite(z)] = 0
    return z


def depth_stream_ratio(src: BagSource, roles: dict, rig: Rig, n_frames: int = 20) -> dict:
    """Evidence C: stereo depth (this baseline) / depth-stream value on the same stamp. median ratio ~1000 would be
    consistent with mm; it also *jointly* tests baseline and unit, so a deviation cannot be attributed to either alone."""
    if "depth" not in roles:
        return {"available": False}
    tl, tr, td = roles["stereo_left"], roles["stereo_right"], roles["depth"]
    total = next((c.count for c in src.connections() if c.topic == td), None) or 0
    step = max(1, total // n_frames)
    from ..bagio import fast_header
    want: dict = {}
    k = 0
    for raw in src.iter_raw([td]):
        if k % step == 0 and len(want) < n_frames:
            want[fast_header(raw.data)[0]] = {"depth": raw}
        k += 1
    for raw in src.iter_raw([tl, tr]):
        s = fast_header(raw.data)[0]
        if s in want:
            want[s]["L" if raw.topic == tl else "R"] = raw
    ratios, rows = [], []
    for s, w in want.items():
        if not all(k in w for k in ("depth", "L", "R")):
            continue
        d = image_to_numpy(src.decode(w["depth"])).astype(np.float64)
        z = stereo_depth_sgbm(image_to_numpy(src.decode(w["L"])), image_to_numpy(src.decode(w["R"])), rig)
        m = (d > 0) & (d < 60000) & (z > 0.4) & (z < 6)
        if m.sum() > 2000:
            ratios.append(np.median(d[m] / z[m])); rows.append(int(m.sum()))
    if not ratios:
        return {"available": True, "frames_compared": 0}
    r = np.array(ratios)
    return {"available": True, "frames_compared": int(len(r)), "median_ratio_depth_over_stereo_m": float(np.median(r)),
            "min": float(r.min()), "max": float(r.max()),
            "reading": "ratio~1000 => stereo(m) and depth stream(mm) agree (joint test of baseline AND unit); ratio~1 => depth stream in m; anything else => one of baseline/unit/depth calibration is off",
            "status": "HYPOTHESIS EVIDENCE ONLY - depth unit is [?] in the contract"}


def verdict(findings: list, epi: dict, rig: Rig) -> dict:
    errs = [m for s, m in findings if s == "error"]
    warns = [m for s, m in findings if s == "warn"]
    level = "FAIL" if errs else ("CONDITIONAL" if warns else "PASS")
    caveats = [
        "Baseline (CameraInfo P and TF) comes from one vendor calibration: P/TF agreement is a consistency check, not independent proof of absolute scale.",
        "Absolute scale is only proven by measuring a known-length object in a stereo recording (see `scale-check`); until then maps are 'metric by vendor calibration'.",
        "Rectification (D=0, R=I) is assumed by CameraInfo; evidence B tests it on the data.",
    ]
    return {"calibration_ok_for_metric_reconstruction": level, "errors": errs, "warnings": warns,
            "caveats": caveats, "uses_only": ["stereo images", "stereo CameraInfo", "/tf_static (cross-check only)"]}


def run_calibcheck(src: BagSource, roles: dict, rig: Rig, n_pairs=24, with_depth_ratio=True, log=print) -> dict:
    F: list = []
    ci, f1 = camerainfo_checks(rig)
    F += f1
    sync = sync_check(src, roles)
    if sync["exact_stamp_pairs"] < 0.99 * min(sync["left"], sync["right"]):
        F.append(("warn" if sync["max_skew_ms"] is not None and sync["max_skew_ms"] < 5 else "error",
                  f"only {sync['exact_stamp_pairs']} of {min(sync['left'], sync['right'])} stereo frames have identical stamps (max skew {sync['max_skew_ms']:.2f} ms)"))
    pairs = sample_pairs(src, roles, n_pairs)
    if pairs:
        im = pairs[0][1]
        if (im.shape[1], im.shape[0]) != (rig.left.width, rig.left.height):
            F.append(("error", f"image {im.shape[1]}x{im.shape[0]} != CameraInfo {rig.left.width}x{rig.left.height}"))
    epi, f2 = epipolar_check(pairs, rig) if pairs else ({}, [("error", "no stereo pairs decoded")])
    F += f2
    res = {"sync": sync, "camera_info": ci, "epipolar": epi,
           "depth_uncertainty_theory": depth_uncertainty_table(rig.fx, rig.baseline) if rig.baseline_P else None}
    if with_depth_ratio:
        try:
            res["depth_stream_cross_check"] = depth_stream_ratio(src, roles, rig)
        except Exception as e:
            res["depth_stream_cross_check"] = {"available": False, "error": str(e)}
    res["findings"] = [{"severity": s, "message": m} for s, m in F]
    res["verdict"] = verdict(F, epi, rig)
    for s, m in F:
        log(f"[{s.upper()}] {m}")
    log(f"stereo calibration verdict: {res['verdict']['calibration_ok_for_metric_reconstruction']}")
    return res
