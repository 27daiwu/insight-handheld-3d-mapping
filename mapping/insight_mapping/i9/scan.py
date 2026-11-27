"""Stage 1 - import: enumerate topics / types / header timestamps of a bag and compare with the contract.

Rules from the contract that are enforced here:
  * RULE 1: rate, loss and gaps are computed from header.stamp only; bag receive time is reported separately.
  * rates are *delivered* Hz = (n-1)/span; loss is count-based vs the nominal rate.
  * nothing marked [?] in the contract is interpreted - only recorded.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from ..bagio import fast_header
from . import topics as T
from .reader import BagSource

HEADER_FIRST = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/CameraInfo",
                "sensor_msgs/msg/Imu", "geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/PoseWithCovarianceStamped",
                "nav_msgs/msg/Odometry")


def stamp_stats(stamps_ns: np.ndarray, nominal_hz) -> dict:
    s = np.asarray(stamps_ns, dtype=np.int64)
    n = len(s)
    out = {"count": int(n)}
    if n < 2:
        return out
    dt = np.diff(s) / 1e9
    span = float((s[-1] - s[0]) / 1e9)
    med = float(np.median(dt))
    out.update({
        "first_stamp_s": float(s[0] / 1e9), "last_stamp_s": float(s[-1] / 1e9), "span_s": span,
        "delivered_hz": float((n - 1) / span) if span > 0 else None,
        "median_period_hz": float(1 / med) if med > 0 else None,
        "dt_ms": {"median": med * 1e3, "p99": float(np.percentile(dt, 99) * 1e3), "max": float(dt.max() * 1e3)},
        "non_monotonic": int((dt < 0).sum()), "duplicates": int((dt == 0).sum()),
        "gap_events_gt1.5x": int((dt > 1.5 * med).sum()),
    })
    if nominal_hz:
        expected = span * nominal_hz + 1
        out["nominal_hz"] = nominal_hz
        out["count_loss_pct"] = float(max(0.0, 100 * (1 - n / expected)))
    return out


def clock_relation(stamps_ns, log_ns) -> dict:
    """Bag receive time vs header stamp. header = camera boot clock, receive = host wall clock: only the *drift*
    and *jitter* of (receive - header) are meaningful, never the absolute offset."""
    s = np.asarray(stamps_ns, dtype=np.float64) / 1e9
    l = np.asarray(log_ns, dtype=np.float64) / 1e9
    if len(s) < 3:
        return {}
    off = l - s
    k = np.polyfit(s - s[0], off - off[0], 1)
    res = (off - off[0]) - np.polyval(k, s - s[0])
    return {"offset_s_first": float(off[0]), "drift_ppm": float(k[0] * 1e6), "residual_std_ms": float(res.std() * 1e3),
            "residual_max_ms": float(np.abs(res).max() * 1e3)}


def _first_msg_facts(src: BagSource, topic: str, msg_type: str) -> dict:
    for raw, msg in src.iter_msgs([topic]):
        f: dict = {}
        if msg_type.endswith("/Image"):
            f.update(width=int(msg.width), height=int(msg.height), encoding=msg.encoding, step=int(msg.step),
                     bigendian=int(msg.is_bigendian))
            if msg.encoding == "mono16":
                a = np.frombuffer(bytes(msg.data), dtype="<u2").reshape(msg.height, msg.step // 2)[:, :msg.width]
                nz = a[a > 0]
                f["value_stats"] = {"zero_fraction": float((a == 0).mean()), "max": int(a.max()),
                                    "median_nonzero": float(np.median(nz)) if nz.size else None,
                                    "unit": "UNVERIFIED (contract §5.4 DEV-DEPTH-UNIT)"}
        elif msg_type.endswith("CompressedImage"):
            import cv2
            im = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_UNCHANGED)
            f.update(format=msg.format, width=None if im is None else int(im.shape[1]), height=None if im is None else int(im.shape[0]))
        elif msg_type.endswith("CameraInfo"):
            f.update(width=int(msg.width), height=int(msg.height), distortion_model=msg.distortion_model,
                     D=list(map(float, msg.d)), K=list(map(float, msg.k)), R=list(map(float, msg.r)), P=list(map(float, msg.p)))
        return f
    return {}


def scan(path, overrides=None, log=print, sample_facts=True) -> dict:
    t0 = time.time()
    src = BagSource(path)
    conns = src.connections()
    types = {c.topic: c.msg_type for c in conns}
    roles, notes = T.resolve(types, overrides)
    role_of = {v: k for k, v in roles.items()}
    stamps: dict = {t: [] for t in types}
    logs: dict = {t: [] for t in types}
    frames: dict = {t: set() for t in types}
    wanted = [t for t, ty in types.items() if ty in HEADER_FIRST]
    for raw in src.iter_raw(None):
        t = raw.topic
        logs[t].append(raw.log_ns)
        if raw.msg_type in HEADER_FIRST:
            h = fast_header(raw.data)
            if h is None:
                notes.append(f"{t}: header not parseable by fast path (big-endian CDR?)")
                continue
            stamps[t].append(h[0])
            frames[t].add(h[1])
    rep: dict = {"input": str(src.path), "storage": src.storage, "files": [str(f) for f in src.files],
                 "topic_count": len(types), "roles_resolved": roles, "resolution_notes": notes, "topics": {}}
    findings: list = []
    for c in conns:
        role = role_of.get(c.topic)
        spec = T.BY_ROLE.get(role) if role else None
        e: dict = {"role": role, "type": c.msg_type, "container_count": c.count, "bag_messages": len(logs[c.topic]),
                   "frame_ids": sorted(frames[c.topic]), "used_in_phase1": bool(spec and spec.used_in_phase1)}
        if stamps[c.topic]:
            st = np.array(stamps[c.topic], dtype=np.int64)
            e.update(stamp_stats(np.sort(st) if (np.diff(st) < 0).any() else st, spec.nominal_hz if spec else None))
            if (np.diff(st) < 0).any():
                e["stamp_stats_note"] = "stamps were not monotonic in receive order; stats use sorted stamps"
            e["receive_vs_header"] = clock_relation(st, logs[c.topic][: len(st)])
        else:
            e["count"] = len(logs[c.topic])
        if sample_facts and spec and c.msg_type in HEADER_FIRST:
            e["first_message"] = _first_msg_facts(src, c.topic, c.msg_type)
        if spec:
            if spec.msg_type != c.msg_type:
                findings.append(("error", f"{role}: type {c.msg_type} != contract {spec.msg_type}"))
            fm = e.get("first_message", {})
            if spec.size_wh and fm.get("width") and (fm["width"], fm["height"]) != spec.size_wh:
                findings.append(("error", f"{role}: size {fm['width']}x{fm['height']} != contract {spec.size_wh[0]}x{spec.size_wh[1]}"))
            if spec.encoding and spec.encoding not in ("jpeg",) and fm.get("encoding") and fm["encoding"] != spec.encoding:
                findings.append(("error", f"{role}: encoding {fm['encoding']} != contract {spec.encoding}"))
            if e.get("non_monotonic"):
                findings.append(("warn", f"{role}: {e['non_monotonic']} non-monotonic header stamps"))
            if spec.nominal_hz and e.get("delivered_hz") and abs(e["delivered_hz"] / spec.nominal_hz - 1) > 0.02:
                findings.append(("info" if role == "depth" else "warn",
                                 f"{role}: delivered {e['delivered_hz']:.2f} Hz vs nominal {spec.nominal_hz} Hz, count loss {e.get('count_loss_pct', 0):.2f} %"))
        rep["topics"][c.topic] = e
    for s in T.SPECS:
        if s.role not in roles and s.used_in_phase1:
            findings.append(("error", f"required stereo input '{s.role}' ({s.topic}) not found in bag"))
        elif s.role not in roles and s.role in ("imu", "vio", "depth", "rgb", "rgb_info"):
            findings.append(("warn", f"contract stream '{s.role}' ({s.topic}) not in bag (kept for later fusion, not needed for phase 1)"))
    extra = [t for t in types if t not in role_of]
    if extra:
        findings.append(("info", f"topics not in the contract spec: {extra}"))
    mf = Path(src.info.get("session_dir", "")) / "manifest.json" if src.info.get("session_dir") else None
    if mf and mf.exists():
        try:
            m = json.loads(mf.read_text())
            rep["manifest"] = {"schema_version": m.get("schema_version"), "session_id": m.get("session_id"),
                               "status": m.get("status") or m.get("recording", {}).get("state"),
                               "firmware": (m.get("camera") or {}).get("firmware")}
        except Exception as ex:
            findings.append(("warn", f"manifest.json unreadable: {ex}"))
    rep["findings"] = [{"severity": s, "message": m} for s, m in findings]
    rep["elapsed_s"] = time.time() - t0
    rep["usable_for_stereo"] = not any(s == "error" for s, _ in findings)
    for s, m in findings:
        log(f"[{s.upper()}] {m}")
    return rep
