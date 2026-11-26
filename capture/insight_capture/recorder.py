"""record / stop: one independent session per recording, supervised rosbag2 recorder."""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from . import SCHEMA_VERSION, __version__, sysinfo, bagdb
from .config import data_root, load_profile
from .discovery import Discovery
from .preflight import run_preflight, parse_expect
from .session import (BAG, LOG, MANIFEST, PLAN, STATE, active_marker, active_session, bag_size,
                      new_session, pid_alive, read_json, resolve_session, utc_now, write_json)

MAX_BAG_SIZE = 2 * 1024**3
CACHE = 256 * 1024**2


def topic_list(disc: Discovery, image_mode: str, include_extra: bool) -> list[dict]:
    """Flatten the chosen topics into [{role,name,type,required}] (depth only if publishing)."""
    prof_roles = {}
    out, seen = [], set()
    for role, ms in disc.chosen(image_mode).items():
        for m in ms:
            if role == "depth" or role == "depth_info":
                if not disc.depth_publishing:
                    continue
            if m.topic in seen:
                continue
            seen.add(m.topic)
            out.append({"role": role, "name": m.topic, "type": m.type, "pub_count": m.pub_count})
    if include_extra:
        for m in disc.extras:
            if m.topic not in seen:
                seen.add(m.topic)
                out.append({"role": "extra", "name": m.topic, "type": m.type, "pub_count": m.pub_count})
    return out


def _snap(hz):
    """Round a measured rate to an integer when within 2% (typical configured rates)."""
    if not hz:
        return None
    r = round(hz)
    return float(r) if r and abs(hz - r) / r < 0.02 else round(hz, 2)


def start(args) -> int:
    root = data_root(args.root)
    prof = load_profile(args.profile)
    a = active_session(root)
    if a:
        print(f"ERROR: a recording is already active: {a}. Run `stop` first.", file=sys.stderr)
        return 2
    expect = parse_expect(getattr(args, "expect", None))
    min_free = args.min_free_gb if args.min_free_gb is not None else prof["thresholds"]["min_free_gb"]

    if args.skip_preflight:
        print("preflight skipped (--skip-preflight); topic discovery only")
    rep, disc, ctx = run_preflight(args.profile, args.prefix, 0.0 if args.skip_preflight else args.probe_seconds,
                                   args.firmware, args.root, min_free, expect, image_mode=args.image_mode,
                                   camera_api=getattr(args, "camera_api", None))
    print(rep.render())
    if disc is None or not disc.found or disc.missing_required:
        if not args.allow_missing or disc is None or not disc.found:
            print("\nREFUSING to record: camera/required topics missing "
                  "(use --allow-missing to record what exists).", file=sys.stderr)
            return 2
    if any(c["level"] == "FAIL" for c in rep.checks) and not args.force:
        if not (args.allow_missing and all(
                c["id"].startswith(("missing_", "rate_", "topic_")) for c in rep.checks if c["level"] == "FAIL")):
            print("\nREFUSING to record: preflight FAIL (use --force to override).", file=sys.stderr)
            return 2

    topics = topic_list(disc, args.image_mode, args.include_extra)
    sid, sdir = new_session(root, args.name)
    probe = ctx.get("probe", {})
    for t in topics:
        t["reference_hz"] = expect.get(t["role"]) or _snap((probe.get(t["name"]) or {}).get("stamp_rate_hz"))
        if t["role"] in ("tf", "tf_static"):
            t["reference_hz"] = expect.get(t["role"])   # TF has no header stamp / irregular rate
        t["required"] = bool(prof["roles"].get(t["role"], {}).get("required")
                             or prof.get("global_topics", {}).get(t["role"], {}).get("required"))
        t["frame_id"] = ((probe.get(t["name"]) or {}).get("first") or {}).get("frame_id")
    depth_enabled = disc.depth_publishing
    plan = {
        "session_id": sid, "topics": topics, "min_free_gb": min_free, "duration_s": args.duration,
        "max_bag_size": MAX_BAG_SIZE, "cache": CACHE, "ros2": "ros2",
    }
    write_json(sdir / PLAN, plan)
    manifest = {
        "contract": "DATA_CONTRACT (provisional, see capture/DATA_CONTRACT.md)",
        "schema_version": SCHEMA_VERSION,
        "session_id": sid,
        "status": "recording",
        "stop_reason": None,
        "tool": {"name": "insight-capture", "version": __version__, "git": sysinfo.git_commit()},
        "host": rep.data["system"]["os"] | {"ros": rep.data["system"]["ros"]},
        "camera": {"model": "Insight 9", "firmware": ctx.get("firmware"),
                   "firmware_baseline": prof.get("firmware_baseline"), "profile": prof["profile"],
                   "profile_verified": prof.get("verified"), "prefix": disc.prefix,
                   "api_snapshot": (ctx.get("camera_api") or {}).get("summary")},
        "time": {"start_utc": utc_now(), "start_wall_ns": time.time_ns(), "end_utc": None, "duration_s": None,
                 "note": "bag timestamps (t_recv) are host receive times; header.stamp are untouched "
                         "camera stamps. Both are preserved."},
        "depth": {"enabled": depth_enabled,
                  "note": None if depth_enabled else "Depth not enabled on the camera (default). "
                                                      "Not recorded; camera config was not modified."},
        "topics": topics,
        "bag": {"dir": BAG, "storage": "sqlite3", "max_bag_size": MAX_BAG_SIZE},
        "disk": {"path": ctx["disk"]["path"], "free_gb_start": ctx["disk"]["free_gb"], "min_free_gb": min_free},
        "preflight": {"exit_code": rep.exit_code,
                      "warnings": [c["title"] for c in rep.checks if c["level"] in ("WARN", "FAIL")]},
        "events": [],
    }
    write_json(sdir / MANIFEST, manifest)
    write_json(sdir / "preflight.json", rep.to_dict())
    write_json(sdir / STATE, {"status": "starting", "supervisor_pid": None})
    active_marker(root).write_text(str(sdir))

    if not depth_enabled:
        print("\nNOTE: Depth is NOT enabled on the camera; it will not be recorded.")
    if args.foreground:
        rc = supervise(sdir)
        return rc
    log = open(sdir / "supervisor.out", "ab")
    p = subprocess.Popen([sys.executable, "-m", "insight_capture", "_supervise", str(sdir)],
                         stdout=log, stderr=log, stdin=subprocess.DEVNULL, start_new_session=True,
                         cwd=str(Path(__file__).resolve().parent.parent))
    for _ in range(100):  # wait until supervisor reports 'recording'
        st = read_json(sdir / STATE, {})
        if st.get("status") == "recording" or p.poll() is not None:
            break
        time.sleep(0.1)
    st = read_json(sdir / STATE, {})
    if st.get("status") != "recording":
        print(f"ERROR: recorder failed to start: see {sdir / LOG}", file=sys.stderr)
        return 2
    print(f"\nrecording started: {sdir}\n  supervisor pid {p.pid}; stop with: insight-capture stop")
    return 0


# --------------------------------------------------------------------------- supervisor
class _Flag:
    stop = False
    reason = None


def _event(sdir: Path, state: dict, level: str, msg: str):
    ev = {"t": utc_now(), "level": level, "msg": msg}
    state.setdefault("events", []).append(ev)
    with open(sdir / LOG, "a") as f:
        f.write(f"{ev['t']} [{level}] {msg}\n")


def supervise(sdir: Path) -> int:
    sdir = Path(sdir)
    plan = read_json(sdir / PLAN)
    state = {"status": "recording", "supervisor_pid": os.getpid(), "child_pid": None,
             "started_utc": utc_now(), "events": []}
    bag = sdir / BAG
    cmd = [plan["ros2"], "bag", "record", "-o", str(bag), "-s", "sqlite3",
           "-b", str(plan["max_bag_size"]), "--max-cache-size", str(plan["cache"]),
           "--include-unpublished-topics"] + [t["name"] for t in plan["topics"]]
    flag = _Flag()

    def on_sig(signum, _f):
        flag.stop = True
        flag.reason = flag.reason or ("user_stop" if signum in (signal.SIGINT, signal.SIGTERM) else "signal")
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    logf = open(sdir / LOG, "a")
    child = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    state["child_pid"] = child.pid
    _event(sdir, state, "info", "started: " + " ".join(cmd))
    write_json(sdir / STATE, state)

    t0 = time.monotonic()
    min_free = plan["min_free_gb"] * 1e9
    last_size, last_growth = 0, time.monotonic()
    warned_disk = stalled = False
    reason, rc = None, None
    while True:
        rc = child.poll()
        if rc is not None:
            reason = reason or ("completed" if flag.stop else f"recorder_exited_rc{rc}")
            break
        if flag.stop:
            reason = flag.reason
            break
        el = time.monotonic() - t0
        if plan.get("duration_s") and el >= plan["duration_s"]:
            reason = "duration_reached"
            break
        free = sysinfo.disk_info(sdir)["free_bytes"]
        if free < min_free:
            _event(sdir, state, "error", f"free disk {free / 1e9:.1f} GB below minimum; stopping")
            reason = "low_disk"
            break
        if free < 2 * min_free and not warned_disk:
            warned_disk = True
            _event(sdir, state, "warn", f"free disk {free / 1e9:.1f} GB approaching minimum")
        size = bag_size(bag)
        if size > last_size:
            last_size, last_growth = size, time.monotonic()
            if stalled:
                stalled = False
                _event(sdir, state, "info", "bag growth resumed")
        elif el > 10 and time.monotonic() - last_growth > 15 and not stalled:
            stalled = True
            _event(sdir, state, "warn", "bag size has not grown for 15s (publishers stalled?)")
        state.update(heartbeat_utc=utc_now(), bag_bytes=size, free_gb=round(free / 1e9, 2),
                     elapsed_s=round(el, 1))
        write_json(sdir / STATE, state)
        time.sleep(1.0)

    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=60)
        except subprocess.TimeoutExpired:
            _event(sdir, state, "error", "recorder ignored SIGINT for 60s; killing")
            child.kill(); child.wait()
            reason = (reason or "") + "+killed"
    rc = child.returncode
    state["status"] = "finalizing"
    write_json(sdir / STATE, state)
    _finalize(sdir, state, reason, rc)
    try:
        am = active_marker(sdir.parent)
        if am.exists() and am.read_text().strip() == str(sdir):
            am.unlink()
    except OSError:
        pass
    return 0 if reason in ("user_stop", "duration_reached", "completed") else 1


def _finalize(sdir: Path, state: dict, reason: str, rc: int | None):
    from .inspect_cmd import finalize_manifest
    ok = reason in ("user_stop", "duration_reached", "completed")
    status = "complete" if ok else "stopped_abnormal"
    try:
        finalize_manifest(sdir, status, reason, rc, state.get("events", []))
    except Exception as e:  # keep the state file honest even if manifest finalize fails
        _event(sdir, state, "error", f"manifest finalize failed: {e!r}")
        status = "finalize_failed"
    state.update(status=status, stop_reason=reason, ended_utc=utc_now(), child_rc=rc)
    write_json(sdir / STATE, state)


# --------------------------------------------------------------------------- stop
def stop(args) -> int:
    root = data_root(args.root)
    sdir = resolve_session(root, args.session)
    if sdir is None:
        print("no session found", file=sys.stderr)
        return 2
    st = read_json(sdir / STATE, {})
    pid = st.get("supervisor_pid")
    if st.get("status") in ("complete", "stopped_abnormal", "finalize_failed"):
        print(f"session {sdir.name} already ended: {st['status']} ({st.get('stop_reason')})")
        return 0
    if not pid_alive(pid):
        print(f"session {sdir.name}: supervisor (pid {pid}) is not running but session was never finalized "
              f"(crash/kill). Run: insight-capture inspect {sdir} --repair", file=sys.stderr)
        return 3
    os.kill(pid, signal.SIGTERM)
    deadline = time.time() + args.timeout
    while time.time() < deadline and pid_alive(pid):
        time.sleep(0.2)
    if pid_alive(pid):
        print(f"supervisor still running after {args.timeout}s", file=sys.stderr)
        return 1
    st = read_json(sdir / STATE, {})
    print(f"stopped: {sdir}\n  status={st.get('status')} reason={st.get('stop_reason')}")
    return 0 if st.get("status") == "complete" else 1
