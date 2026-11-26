"""P1-B: data quality report — rates, gaps, time sync, VIO sanity, frames, intrinsics/extrinsics."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import calib, se3
from .extract import Extracted
from .session import Session

SYNC_WARN_MS = 15.0     # image <-> pose / rgb <-> depth
SYNC_ERR_MS = 40.0


def stream_stats(stamps_ns: np.ndarray, bag_ns: np.ndarray | None = None) -> dict:
    s = np.asarray(stamps_ns, dtype=np.int64)
    if len(s) < 2:
        return {"count": int(len(s))}
    dt = np.diff(s) / 1e9
    med = float(np.median(dt))
    out = {"count": int(len(s)), "duration_s": float((s[-1] - s[0]) / 1e9),
           "rate_hz": float(1 / med) if med > 0 else 0.0,
           "dt_ms": {"median": med * 1e3, "p99": float(np.percentile(dt, 99) * 1e3), "max": float(dt.max() * 1e3)},
           "non_monotonic": int((dt < 0).sum()), "duplicates": int((dt == 0).sum()),
           "gaps": int((dt > 2.5 * med).sum()),
           "estimated_dropped": int(np.maximum(np.round(dt[dt > 1.5 * med] / med) - 1, 0).sum())}
    if bag_ns is not None and len(bag_ns) == len(s):
        off = (np.asarray(bag_ns, dtype=np.int64) - s) / 1e6
        out["header_vs_bag_ms"] = {"median": float(np.median(off)), "std": float(off.std())}
    return out


def nearest_dt_ms(a_ns: np.ndarray, b_ns: np.ndarray) -> np.ndarray:
    """|a_i - nearest b| in ms."""
    b = np.sort(b_ns)
    i = np.clip(np.searchsorted(b, a_ns), 1, len(b) - 1)
    d = np.minimum(np.abs(a_ns - b[i - 1]), np.abs(b[i] - a_ns))
    return d / 1e6


def _sync_entry(name, dts, findings, hint=""):
    e = {"median_ms": float(np.median(dts)), "p95_ms": float(np.percentile(dts, 95)), "max_ms": float(dts.max())}
    if e["p95_ms"] > SYNC_ERR_MS:
        findings.append(("error", f"{name}: p95 time offset {e['p95_ms']:.1f} ms > {SYNC_ERR_MS} ms {hint}"))
    elif e["p95_ms"] > SYNC_WARN_MS:
        findings.append(("warn", f"{name}: p95 time offset {e['p95_ms']:.1f} ms > {SYNC_WARN_MS} ms {hint}"))
    return e


def vio_checks(v, name, findings, max_speed=5.0) -> dict:
    T = v["T"]
    out = stream_stats(v["stamp_ns"], v["bag_ns"])
    if len(T) < 2:
        return out
    step = np.linalg.norm(np.diff(T[:, :3, 3], axis=0), axis=1)
    dt = np.maximum(np.diff(v["stamp_ns"]) / 1e9, 1e-6)
    sp = step / dt
    rot = np.array([np.degrees(se3.rot_angle(se3.relative(T[i], T[i + 1])[:3, :3])) for i in range(len(T) - 1)])
    out.update({"path_length_m": float(step.sum()), "max_speed_mps": float(sp.max()),
                "max_ang_rate_dps": float((rot / dt).max()), "jumps": int((sp > max_speed).sum()),
                "parent_frame": v["parent"], "child_frame": v["child"]})
    if out["jumps"]:
        findings.append(("error", f"{name}: {out['jumps']} pose jumps with speed > {max_speed} m/s (max {sp.max():.1f})"))
    if not np.allclose(np.linalg.det(T[:, :3, :3]), 1, atol=1e-3):
        findings.append(("error", f"{name}: non-rotation matrices"))
    return out


def check_session(session: Session, work_dir, log=print, allow_missing=()) -> dict:
    ex = Extracted(work_dir)
    F: list = [tuple(p) for p in session.problems]
    for n in session.stream_notes:
        F.append(("warn", n))
    miss = session.missing_mandatory()
    allowed = [m for m in miss if m in allow_missing]
    miss = [m for m in miss if m not in allow_missing]
    if allowed:
        F.append(("warn", f"mandatory streams missing but explicitly allowed (--allow-missing): {allowed}"))
    if miss:
        F.append(("error" if any(m in ("vio", "imu", "image(rgb|left)") for m in miss) else "warn",
                  f"mandatory streams missing (contract §2): {miss}"))
    rep: dict = {"session_id": session.session_id, "streams_resolved": session.streams, "streams": {}}

    idx = {k: ex.index(k) for k in ("rgb", "depth", "left", "right")}
    for k, v in idx.items():
        if v is not None:
            rep["streams"][k] = {**stream_stats(v["stamp_ns"], v["bag_ns"]), **ex.summary["image_meta"].get(k, {})}
            mm = session.stream_meta(k)
            if mm and mm.get("message_count") and abs(mm["message_count"] - ex.summary["messages_read"].get(k, 0)) > 0:
                F.append(("warn", f"{k}: manifest message_count {mm['message_count']} != bag {ex.summary['messages_read'].get(k)}"))
    vios = {k: ex.vio(k) for k in ("vio_image", "vio_100hz")}
    for k, v in vios.items():
        if v is not None:
            rep["streams"][k] = vio_checks(v, k, F)
    imu = ex.imu()
    if imu is not None:
        st = stream_stats(imu["stamp_ns"], imu["bag_ns"])
        st["acc_norm_mean"] = float(np.linalg.norm(imu["acc"], axis=1).mean())
        rep["streams"]["imu"] = st
        if not (7.0 < st["acc_norm_mean"] < 12.5):
            F.append(("warn", f"imu: mean |acc| {st['acc_norm_mean']:.2f} m/s^2, expected ~9.8 (units/gravity?)"))

    # --- time sync
    sync = {}
    base = "rgb" if idx["rgb"] else ("left" if idx["left"] else None)
    ref_vio = vios["vio_image"] or vios["vio_100hz"]
    if base and ref_vio is not None:
        t = idx[base]["stamp_ns"]
        sync[f"{base}->vio_image"] = _sync_entry(f"{base}->vio_image", nearest_dt_ms(t, ref_vio["stamp_ns"]), F)
        slack = 20_000_000  # ns: 20 ms at the ends is tolerated (extrapolation by clamping)
        cover = ((t >= ref_vio["stamp_ns"][0] - slack) & (t <= ref_vio["stamp_ns"][-1] + slack)).mean()
        sync["image_in_vio_time_range_fraction"] = float(cover)
        if cover < 0.99:
            F.append(("warn", f"only {cover:.1%} of {base} stamps lie inside the VIO time range"))
        if vios["vio_100hz"] is not None and vios["vio_image"] is not None:
            sync[f"{base}->vio_100hz"] = _sync_entry(f"{base}->vio_100hz", nearest_dt_ms(t, vios["vio_100hz"]["stamp_ns"]), F)
    if idx["rgb"] and idx["depth"]:
        sync["rgb->depth"] = _sync_entry("rgb->depth", nearest_dt_ms(idx["rgb"]["stamp_ns"], idx["depth"]["stamp_ns"]), F,
                                          "(RGB/Depth must be paired by stamp)")
    if idx["left"] and idx["right"]:
        sync["left->right"] = _sync_entry("left->right", nearest_dt_ms(idx["left"]["stamp_ns"], idx["right"]["stamp_ns"]), F)
    if imu is not None and base:
        sync[f"{base}->imu"] = _sync_entry(f"{base}->imu", nearest_dt_ms(idx[base]["stamp_ns"], imu["stamp_ns"]), F)
    rep["time_sync"] = sync

    # --- vio_image vs vio_100hz consistency (translation of interpolated 100Hz at vio_image stamps)
    if vios["vio_100hz"] is not None and vios["vio_image"] is not None:
        a, b = vios["vio_image"], vios["vio_100hz"]
        m = (a["stamp_ns"] >= b["stamp_ns"][0]) & (a["stamp_ns"] <= b["stamp_ns"][-1])
        if m.sum() > 2:
            Ti, *_ = se3.interpolate_poses(b["stamp_ns"].astype(float), b["T"], a["stamp_ns"][m].astype(float))
            err = np.linalg.norm(Ti[:, :3, 3] - a["T"][m][:, :3, 3], axis=1)
            rep["vio_100hz_vs_image"] = {"trans_err_median_m": float(np.median(err)), "trans_err_max_m": float(err.max())}
            if err.max() > 0.05:
                F.append(("warn", f"vio_100hz interpolated vs vio_image differ up to {err.max()*100:.1f} cm "
                                  "(different frames/origins, or latency)"))

    # --- frames / TF / calibration
    cal: dict = {"cameras": {}}
    for k, cam in ex.cams.items():
        base_k = k[:-5]
        wh = (ex.summary["image_meta"][base_k]["width"], ex.summary["image_meta"][base_k]["height"]) \
            if base_k in ex.summary["image_meta"] else None
        cal["cameras"][k] = {"frame_id": cam.frame_id, "size": [cam.width, cam.height], "fx": cam.fx, "fy": cam.fy,
                             "cx": cam.cx, "cy": cam.cy, "distortion_model": cam.distortion_model,
                             "has_distortion": cam.has_distortion}
        for sev, msg in calib.check_camera(cam, wh, k):
            F.append((sev, msg))
        im = ex.summary["image_meta"].get(base_k, {})
        if im.get("frame_id") and cam.frame_id and im["frame_id"] != cam.frame_id:
            F.append(("warn", f"{k}: CameraInfo frame_id '{cam.frame_id}' != image frame_id '{im['frame_id']}'"))
        if cam.frame_id and not ex.tf.can_lookup(cam.frame_id, cam.frame_id) :
            pass
    cal["tf_frames"] = sorted(ex.tf.frames())
    if ref_vio is not None:
        for k, cam in ex.cams.items():
            fid = cam.frame_id
            if fid and not ex.tf.can_lookup(ref_vio["child"], fid):
                F.append(("error", f"no TF path from VIO child frame '{ref_vio['child']}' to {k} frame '{fid}': cannot place cameras"))
    rgb_c, dep_c = ex.cams.get("rgb_info"), ex.cams.get("depth_info")
    if rgb_c and dep_c:
        T = None
        try:
            T = ex.tf.lookup(rgb_c.frame_id, dep_c.frame_id) if ex.tf.can_lookup(rgb_c.frame_id, dep_c.frame_id) else None
        except KeyError:
            pass
        al = calib.rgbd_alignment(rgb_c, dep_c, T)
        cal["rgbd_alignment"] = {**al, "T_rgb_depth": None if T is None else T.tolist()}
        if not al["aligned"]:
            F.append(("warn", "RGB/Depth are NOT pixel-aligned: " + "; ".join(al["reasons"]) +
                      " -> depth is processed in its own frame, colour is mapped via calibration"))
    if "left_info" in ex.cams and "right_info" in ex.cams:
        bl = calib.stereo_baseline(ex.cams["left_info"], ex.cams["right_info"])
        Tlr = None
        try:
            Tlr = ex.tf.lookup(ex.cams["left_info"].frame_id, ex.cams["right_info"].frame_id)
        except KeyError:
            pass
        cal["stereo"] = {"baseline_from_P_m": bl, "baseline_from_tf_m": None if Tlr is None else float(np.linalg.norm(Tlr[:3, 3]))}
        if bl and Tlr is not None and abs(bl - cal["stereo"]["baseline_from_tf_m"]) > 0.005:
            F.append(("error", f"stereo baseline mismatch: CameraInfo {bl*1000:.1f} mm vs TF {cal['stereo']['baseline_from_tf_m']*1000:.1f} mm"))
        if Tlr is None:
            F.append(("warn", "no TF between left and right cameras"))
    rep["calibration"] = cal
    rep["findings"] = [{"severity": s, "message": m} for s, m in F]
    rep["summary"] = {"errors": sum(s == "error" for s, _ in F), "warnings": sum(s == "warn" for s, _ in F),
                      "usable_for_reconstruction": not any(s == "error" for s, _ in F)}
    (Path(work_dir) / "quality_report.json").write_text(json.dumps(rep, indent=1))
    for s, m in F:
        log(f"[{s.upper()}] {m}")
    log(f"quality: {rep['summary']}")
    return rep
