#!/usr/bin/env python3
"""Backup / apply / restore Insight 9 settings through the vendor web UI's own HTTP API (found in the UI JS bundle).
No guessed commands: every POST body below was copied from the UI source (pages-settings-camera / looper-control / time-sync).
  backup  -> GET all config endpoints -> JSON            restore FILE -> POST the saved values back, then re-GET and diff
  set depth on|off | stereo "<format>" <hdr:true|false> | timesync on|off | colour "<format>"
"""
import json, sys, time, urllib.request
B = "http://169.254.10.1"
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))      # bypass shell proxy
def req(path, body=None, t=20):
    r = urllib.request.Request(B + path, data=None if body is None else json.dumps(body).encode(),
                               headers={"Content-Type": "application/json"}, method="GET" if body is None else "POST")
    with opener.open(r, timeout=t) as f: return json.loads(f.read().decode() or "null")
GET = ["/api/camera-format", "/api/camera-format-options", "/api/stereo-format", "/api/stereo-config", "/api/deep-flow",
       "/api/time-sync-setting", "/api/sensor-pose-cov", "/api/dds-type", "/api/ros-domain-id", "/api/ip-config", "/api/mode",
       "/api/compatibility-mode", "/api/camera-config", "/api/insight-status", "/api/ota/device-versions", "/api/system-time", "/api/system-info"]
def snapshot():
    out = {"taken_epoch": time.time()}
    for p in GET:
        try: out[p] = req(p)
        except Exception as e: out[p] = {"ERROR": str(e)}
    return out
def core(s):   # values that the experiments touch
    g = lambda p, *k: (lambda d: d if not k else (d.get(k[0]) if isinstance(d, dict) else None))((s.get(p) or {}).get("data"))
    return {"camera-format": g("/api/camera-format", "format"), "stereo-format": g("/api/stereo-format", "format"),
            "stereo-hdr": g("/api/stereo-format", "hdr"), "deep-flow": g("/api/deep-flow", "enabled"),
            "time-sync": g("/api/time-sync-setting", "enabled"), "pose-cov": g("/api/sensor-pose-cov", "publish_pose_cov"),
            "ros-domain": g("/api/ros-domain-id", "rosDomainId"), "dds": g("/api/dds-type", "ddsType")}
if __name__ == "__main__":
    c = sys.argv[1]
    if c == "backup":
        s = snapshot(); json.dump(s, open(sys.argv[2], "w"), indent=1); print(json.dumps(core(s)))
    elif c == "show": print(json.dumps(core(snapshot())))
    elif c == "set":
        w = sys.argv[2]
        if w == "depth": print(req("/api/deep-flow", {"enabled": sys.argv[3] == "on"}))
        elif w == "stereo": print(req("/api/stereo-format", {"format": sys.argv[3], "hdr": sys.argv[4].lower() == "true"}, 60))
        elif w == "timesync": print(req("/api/time-sync-setting", {"enabled": sys.argv[3] == "on"}))
        elif w == "colour": print(req("/api/camera-format", {"format": sys.argv[3]}, 60))
    elif c == "restore":
        s = json.load(open(sys.argv[2])); k = core(s)
        print("restoring", k)
        print(req("/api/deep-flow", {"enabled": bool(k["deep-flow"])}))
        print(req("/api/stereo-format", {"format": k["stereo-format"], "hdr": bool(k["stereo-hdr"])}, 60))
        print(req("/api/time-sync-setting", {"enabled": bool(k["time-sync"])}))
        print(req("/api/camera-format", {"format": k["camera-format"]}, 60))
        time.sleep(5); now = core(snapshot()); print("after:", now, "MATCH" if now == k else "MISMATCH")
