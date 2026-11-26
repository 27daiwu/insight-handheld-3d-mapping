#!/usr/bin/env python3
"""Authorised camera-parameter experiment. One variable per scenario, always restored from the saved backup.
Config calls: camera_cfg.py (vendor web-UI HTTP API, bodies copied from the UI source). Measurement: passive rclpy subscribers on ROS domain 9.
Scenarios: S0 baseline | S1 depth off | S2 stereo 30->20 fps (UI forces hdr=false for non-30 fps) | S3 camera time-sync on."""
import json, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, HERE.rsplit("/reports/", 1)[0])
import camera_cfg as C
from insight_capture.rosgraph import Graph
from insight_capture import diagnostics as D
BACKUP = sys.argv[1]; OUT = sys.argv[2]; MEAS = float(sys.argv[3]) if len(sys.argv) > 3 else 90
P = "/camera/camera"
TOP = {f"{P}/depth/image_rect_raw": "sensor_msgs/msg/Image", f"{P}/infra1/image_rect_raw": "sensor_msgs/msg/Image",
       f"{P}/infra2/image_rect_raw": "sensor_msgs/msg/Image", f"{P}/color/image_rect_raw/compressed": "sensor_msgs/msg/CompressedImage",
       f"{P}/imu": "sensor_msgs/msg/Imu", f"{P}/vio_100hz": "geometry_msgs/msg/PoseStamped", f"{P}/vio_image": "geometry_msgs/msg/PoseStamped",
       f"{P}/infra1/camera_info": "sensor_msgs/msg/CameraInfo", f"{P}/color/camera_info": "sensor_msgs/msg/CameraInfo"}
orig = json.load(open(BACKUP)); ocore = C.core(orig)
res = {"backup": BACKUP, "original_core": ocore, "log": [], "scenarios": {}}
def log(m): res["log"].append({"t": time.time(), "m": m}); print(m, flush=True); json.dump(res, open(OUT, "w"), indent=1, default=str)
def cam_state():
    s = {}
    for p in ("cpu-monitor", "system-info", "system-time"):
        try: s[p] = C.req("/api/" + p)
        except Exception as e: s[p] = str(e)
    return s
def measure(name, secs, nominal):
    t0 = time.time()
    with Graph("cfg_meas") as g:
        g.snapshot(3.0); raw = g.probe(TOP, secs)
    out = {}
    for t, r in raw.items():
        c = r["count"]
        o = {"count": c, "type": r["type"]}
        if c > 3:
            span = (r["stamp_last_ns"] - r["stamp_first_ns"]) / 1e9 if r.get("stamp_last_ns") else None
            o.update(mean_hz=round((c - 1) / span, 3) if span else None, stamp_first_s=round(r["stamp_first_ns"] / 1e9, 1), recv_rate_hz=r.get("recv_rate_hz"),
                     gap_events=r.get("gap_events"), est_missing=r.get("est_missing_msgs"), max_gap_ms=r.get("max_gap_ms"), MBps=round(r["bytes_per_s"] / 1e6, 2))
            nom = nominal.get(t.split(P + "/")[-1])
            if nom and span: o["missing_vs_nominal_pct"] = round(100 * (1 - c / (span * nom + 1)), 2)
            o["first_frame_id"] = (r.get("first") or {}).get("frame_id")
        out[t.split(P + "/")[-1]] = o
    return {"name": name, "secs": secs, "meas_started": t0, "topics": out, "cam": cam_state(), "cfg": C.core(C.snapshot())}
def wait_topics(need, timeout=90):
    end = time.time() + timeout
    while time.time() < end:
        try:
            with Graph("cfg_wait") as g:
                names = {t.name for t in g.snapshot(3.0)}
            if all(f"{P}/{n}" in names for n in need): return True
        except Exception: pass
        time.sleep(3)
    return False
def apply_and_check(fn, key, want, settle=12):
    r = fn(); log(f"APPLY -> {r}"); time.sleep(settle)
    got = C.core(C.snapshot()).get(key); log(f"config readback {key}={got} (wanted {want})"); return got == want
NOM = {"depth/image_rect_raw": 15, "infra1/image_rect_raw": 30, "infra2/image_rect_raw": 30, "color/image_rect_raw/compressed": 30, "imu": 400, "vio_100hz": 100, "vio_image": 30, "infra1/camera_info": 30, "color/camera_info": 30}
try:
    os.environ["ROS_DOMAIN_ID"] = "9"
    wait_topics(["imu"]); log("S0 baseline"); res["scenarios"]["S0_baseline"] = measure("S0_baseline", MEAS, NOM)
    # S1 depth off
    log("S1 depth off"); ok = apply_and_check(lambda: C.req("/api/deep-flow", {"enabled": False}), "deep-flow", False, 15)
    res["scenarios"]["S1_depth_off"] = measure("S1_depth_off", MEAS, NOM); res["scenarios"]["S1_depth_off"]["config_applied"] = ok
    log("S1 restore depth on"); ok = apply_and_check(lambda: C.req("/api/deep-flow", {"enabled": bool(ocore["deep-flow"])}), "deep-flow", ocore["deep-flow"], 15)
    wait_topics(["depth/image_rect_raw"]); res["scenarios"]["S1r_after_restore"] = measure("S1r_after_restore", 45, NOM); res["scenarios"]["S1r_after_restore"]["config_restored"] = ok
    # S2 stereo 20 fps
    fmt20 = "mono8 544x640 20fps"; log("S2 stereo 20fps hdr=false")
    ok = apply_and_check(lambda: C.req("/api/stereo-format", {"format": fmt20, "hdr": False}, 90), "stereo-format", fmt20, 20)
    wait_topics(["infra1/image_rect_raw", "depth/image_rect_raw"]); nom2 = dict(NOM, **{"infra1/image_rect_raw": 20, "infra2/image_rect_raw": 20, "infra1/camera_info": 20})
    res["scenarios"]["S2_stereo20"] = measure("S2_stereo20", MEAS, nom2); res["scenarios"]["S2_stereo20"]["config_applied"] = ok
    log("S2 restore"); ok = apply_and_check(lambda: C.req("/api/stereo-format", {"format": ocore["stereo-format"], "hdr": bool(ocore["stereo-hdr"])}, 90), "stereo-format", ocore["stereo-format"], 20)
    wait_topics(["infra1/image_rect_raw", "depth/image_rect_raw"]); res["scenarios"]["S2r_after_restore"] = measure("S2r_after_restore", 45, NOM); res["scenarios"]["S2r_after_restore"]["config_restored"] = ok
    # S3 time sync
    log("S3 time-sync on"); ok = apply_and_check(lambda: C.req("/api/time-sync-setting", {"enabled": True}), "time-sync", True, 20)
    res["scenarios"]["S3_timesync_on_t+20s"] = {"api": C.req("/api/time-sync-setting"), "cam": cam_state()}
    res["scenarios"]["S3_timesync_on"] = measure("S3_timesync_on", MEAS, NOM); res["scenarios"]["S3_timesync_on"]["config_applied"] = ok
    res["scenarios"]["S3_timesync_on"]["host_epoch_end"] = time.time()
finally:
    log("RESTORE all from backup")
    try:
        for fn in (lambda: C.req("/api/time-sync-setting", {"enabled": bool(ocore["time-sync"])}),
                   lambda: C.req("/api/deep-flow", {"enabled": bool(ocore["deep-flow"])}),
                   lambda: C.req("/api/stereo-format", {"format": ocore["stereo-format"], "hdr": bool(ocore["stereo-hdr"])}, 90)):
            try: log(f"restore call -> {fn()}")
            except Exception as e: log(f"restore call failed: {e}")
        time.sleep(15); now = C.core(C.snapshot()); res["final_core"] = now; res["restored_ok"] = (now == ocore); log(f"final {now} restored_ok={now == ocore}")
    finally:
        json.dump(res, open(OUT, "w"), indent=1, default=str)
