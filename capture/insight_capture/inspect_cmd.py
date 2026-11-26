"""inspect: dataset checks (works after normal stop and after a crash); manifest finalisation."""
from __future__ import annotations

import json
import sys
from pathlib import Path

from . import bagdb, sysinfo
from .config import data_root, load_profile
from .preflight import Report, _connected
from .session import (BAG, LOG, MANIFEST, PLAN, STATE, bag_size, pid_alive, read_json, resolve_session,
                      utc_now, write_json)
from .stats import judge

NS = 1e9


def _summarize_topic(plan_t: dict, st: dict | None) -> dict:
    st = st or {}
    return {**plan_t, "count": st.get("count", 0), "recv_first_ns": st.get("recv_first_ns"),
            "recv_last_ns": st.get("recv_last_ns"), "stamp_first_ns": st.get("stamp_first_ns"),
            "stamp_last_ns": st.get("stamp_last_ns"), "measured_stamp_rate_hz": st.get("stamp_rate_hz"),
            "measured_recv_rate_hz": st.get("recv_rate_hz"), "est_missing_msgs": st.get("est_missing_msgs"),
            "bytes": st.get("bytes", 0)}


def finalize_manifest(sdir: Path, status: str, reason: str | None, rc: int | None, events: list,
                      scan: dict | None = None) -> dict:
    m = read_json(sdir / MANIFEST, {})
    bag = sdir / BAG
    scan = scan or bagdb.scan(bag)
    stats = scan["topics"]
    recs = [s["recv_first_ns"] for s in stats.values() if s.get("recv_first_ns")]
    rece = [s["recv_last_ns"] for s in stats.values() if s.get("recv_last_ns")]
    m["topics"] = [_summarize_topic(t, stats.get(t["name"])) for t in m.get("topics", [])]
    known = {t["name"] for t in m["topics"]}
    for n, s in stats.items():   # recorded but not planned (should not happen) - keep visible
        if n not in known:
            m["topics"].append(_summarize_topic({"role": "unplanned", "name": n, "type": s["type"]}, s))
    m["status"], m["stop_reason"] = status, reason
    m["time"]["end_utc"] = utc_now()
    if recs and rece:
        m["time"]["first_msg_recv_ns"], m["time"]["last_msg_recv_ns"] = min(recs), max(rece)
        m["time"]["duration_s"] = round((max(rece) - min(recs)) / NS, 3)
    m["bag"].update(files=scan["files"], size_bytes=bag_size(bag),
                    metadata_present=(bag / "metadata.yaml").exists(), recorder_rc=rc)
    m["disk"]["free_gb_end"] = sysinfo.disk_info(sdir)["free_gb"]
    m["events"] = list(m.get("events", [])) + list(events)
    if scan["errors"]:
        m["events"].append({"t": utc_now(), "level": "error", "msg": "; ".join(scan["errors"])})
    # a depth topic that was promised but is empty
    if m.get("depth", {}).get("enabled") and not any(t["role"] == "depth" and t["count"] for t in m["topics"]):
        m["depth"]["enabled"] = False
        m["depth"]["note"] = "depth topic had no recorded messages"
    write_json(sdir / MANIFEST, m)
    return m


def session_state(sdir: Path) -> tuple[str, str]:
    """-> (state, explanation). state in complete|recording|interrupted|abnormal|unknown."""
    st = read_json(sdir / STATE, {})
    m = read_json(sdir / MANIFEST, {})
    status = st.get("status") or m.get("status")
    if status == "complete":
        return "complete", "finalized normally"
    if status == "recovered":
        return "recovered", "finalized by `inspect --repair` after an unclean exit"
    if status in ("stopped_abnormal", "finalize_failed"):
        return "abnormal", f"stopped abnormally: {st.get('stop_reason') or m.get('stop_reason')}"
    if status in ("recording", "starting", "finalizing"):
        if pid_alive(st.get("supervisor_pid")):
            return "recording", "recorder still running"
        return "interrupted", "supervisor is gone but session was never finalized (crash / kill / power loss)"
    return "unknown", "no session state (bare bag directory?)"


def inspect(args) -> int:
    root = data_root(args.root)
    sdir = resolve_session(root, args.session)
    if sdir is None:
        print("no session found", file=sys.stderr)
        return 2
    if not (sdir / BAG).exists() and list(sdir.glob("*.db3")):
        bag, sdir_is_bag = sdir, True
    else:
        bag, sdir_is_bag = sdir / BAG, False
    prof = load_profile(args.profile)
    th = prof["thresholds"]
    rep = Report()
    m = read_json(sdir / MANIFEST, {}) if not sdir_is_bag else {}
    state, why = ("unknown", "bare bag directory") if sdir_is_bag else session_state(sdir)
    rep.add({"complete": "PASS", "recording": "INFO"}.get(state, "WARN"), "state", f"Session {sdir.name}: {state}", why)

    if not bag.exists() or not bagdb.db_files(bag):
        rep.add("FAIL", "bag", "No rosbag2 db3 files found", str(bag))
        return _out(args, rep)

    has_meta = (bag / "metadata.yaml").exists()
    rep.add("PASS" if has_meta else "WARN", "metadata", "metadata.yaml " + ("present" if has_meta else "MISSING"),
            "" if has_meta else "Bag was not closed cleanly. Data is readable; `inspect --repair` will "
                                "run `ros2 bag reindex` (writes metadata.yaml only; db3 untouched).")
    nominal = {t["name"]: t.get("reference_hz") for t in m.get("topics", []) if t.get("reference_hz")}
    scan = bagdb.scan(bag, th["gap_factor"], nominal=None)
    if scan["errors"]:
        rep.add("FAIL", "db_errors", "sqlite errors while reading bag", "\n".join(scan["errors"]))
    stats = scan["topics"]
    rep.data["topics"] = stats
    rep.data["manifest"] = m

    if not m:
        rep.add("WARN", "manifest", "manifest.json missing")
    else:
        for k in ("session_id", "camera", "time", "topics", "bag", "depth", "host", "schema_version"):
            if k not in m:
                rep.add("FAIL", "manifest", f"manifest missing field '{k}'")
        if m.get("camera", {}).get("firmware") is None:
            rep.add("WARN", "manifest_fw", "manifest: firmware version unknown")

    planned = {t["name"]: t for t in m.get("topics", [])} if m else {}
    for name, t in planned.items():
        role = t.get("role")
        st = stats.get(name)
        if st is None or st["count"] == 0:
            lvl = "FAIL" if t.get("required") else "WARN"
            rep.add(lvl, f"data_{role}", f"{role} {name}: NO data in bag")
            continue
        fnd = judge(st, t.get("reference_hz"), th)
        hz = st.get("stamp_rate_hz") or st.get("recv_rate_hz")
        ref = t.get("reference_hz")
        summ = (f"{role} {name}: {st['count']} msgs, {hz} Hz"
                + (f" (ref {ref:.1f})" if ref else "")
                + f", est. missing {st.get('est_missing_msgs', 0)}, {st['bytes'] / 1e6:.1f} MB")
        rep.add("PASS" if not fnd else "WARN", f"data_{role}", summ, "\n".join(x for _, x in fnd))
        if state == "complete" and m.get("topics"):
            mc = t.get("count")
            if mc is not None and mc != st["count"]:
                rep.add("FAIL", f"count_{role}", f"{role}: manifest says {mc} msgs, bag has {st['count']}")
    for name, st in stats.items():
        if name not in planned and planned:
            rep.add("WARN", "unplanned", f"topic in bag but not in manifest: {name}")
    if not m:   # no manifest: still report each topic
        for name, st in stats.items():
            fnd = judge(st, None, th)
            rep.add("PASS" if not fnd else "WARN", "data", f"{name}: {st['count']} msgs, "
                    f"{st.get('stamp_rate_hz') or st.get('recv_rate_hz')} Hz", "\n".join(x for _, x in fnd))

    # cross-topic time alignment
    firsts = {n: s["recv_first_ns"] for n, s in stats.items() if s.get("recv_first_ns")}
    lasts = {n: s["recv_last_ns"] for n, s in stats.items() if s.get("recv_last_ns")}
    core = [n for n, t in planned.items() if t.get("required") and n in firsts and t["role"] != "tf_static"]
    if len(core) > 1:
        sf = (max(firsts[n] for n in core) - min(firsts[n] for n in core)) / NS
        sl = (max(lasts[n] for n in core) - min(lasts[n] for n in core)) / NS
        rep.add("PASS" if max(sf, sl) < 2.0 else "WARN", "alignment",
                f"core topics start within {sf:.2f}s and end within {sl:.2f}s of each other")
    if firsts:
        dur = (max(lasts.values()) - min(firsts.values())) / NS
        rep.data["duration_s"] = round(dur, 3)
        rep.add("INFO", "duration", f"Recorded span {dur:.1f}s, {bag_size(bag) / 1e6:.1f} MB on disk")

    # depth consistency
    if m:
        dep = [t for t in m.get("topics", []) if t.get("role") == "depth"]
        if not m.get("depth", {}).get("enabled") and not dep:
            rep.add("WARN", "depth", "Depth not recorded (camera depth was not enabled)")
        elif dep and stats.get(dep[0]["name"], {}).get("count"):
            rep.add("PASS", "depth", "Depth recorded")

    # TF
    tfn = [n for n in ("/tf", "/tf_static") if n in stats]
    if tfn and not args.no_tf:
        try:
            edges = bagdb.tf_frames(bag, tfn)
            frames = [t.get("frame_id") for t in m.get("topics", []) if t.get("frame_id")
                      and t["role"] not in ("tf", "tf_static")]
            ok, miss = _connected(edges, frames)
            rep.add("PASS" if edges and ok else "WARN", "tf", f"TF: {len(edges)} edges in bag",
                    "" if ok else f"data frames not connected: {miss}")
        except Exception as e:
            rep.add("WARN", "tf", "TF check failed", repr(e))

    if has_meta:
        rc, out = sysinfo.run(["ros2", "bag", "info", str(bag)], 30)
        rep.add("PASS" if rc == 0 else "FAIL", "ros2_bag_info", "ros2 bag info " + ("ok" if rc == 0 else "failed"),
                "" if rc == 0 else out[-300:])

    if args.repair:
        _repair(sdir, bag, state, rep, sdir_is_bag)
    return _out(args, rep)


def _repair(sdir: Path, bag: Path, state: str, rep: Report, bare: bool):
    if not (bag / "metadata.yaml").exists():
        rc, out = sysinfo.run(["ros2", "bag", "reindex", "-s", "sqlite3", str(bag)], 120)
        rep.add("PASS" if rc == 0 else "FAIL", "repair_reindex", "ros2 bag reindex " + ("ok" if rc == 0 else "FAILED"),
                out[-300:] if rc else "metadata.yaml regenerated")
    if not bare and state in ("interrupted",) and (bag / "metadata.yaml").exists():
        finalize_manifest(sdir, "recovered", "crash_recovered", None,
                          [{"t": utc_now(), "level": "warn", "msg": "recovered by inspect --repair after unclean exit"}])
        st = read_json(sdir / STATE, {})
        st.update(status="recovered")
        write_json(sdir / STATE, st)
        rep.add("PASS", "repair_manifest", "manifest.json finalized with status 'recovered'")


def _out(args, rep: Report) -> int:
    if args.json:
        print(json.dumps(rep.to_dict(), indent=2, default=str))
    else:
        print(rep.render())
    return rep.exit_code
