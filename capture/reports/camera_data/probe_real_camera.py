"""Passive 60 s measurement of every topic of the real camera (subscribe only). Usage: ROS_DOMAIN_ID=9 python3 probe_real_camera.py SECONDS OUT.json"""
import json, sys, time
sys.path.insert(0, __file__.rsplit("/reports/", 1)[0])
from insight_capture.rosgraph import Graph
from insight_capture import diagnostics as D
secs = float(sys.argv[1]); out = sys.argv[2]
with Graph("real_cam_probe") as g:
    infos = g.snapshot(3.0)
    topics = {t.name: t.type for t in infos if t.name.startswith("/camera") or t.name in ("/tf", "/tf_static")}
    topics = {n: t for n, t in topics.items() if t != "std_msgs/msg/String"} | {n: t for n, t in topics.items() if t == "std_msgs/msg/String"}
    res = g.probe(topics, secs)
rep = {}
for n, r in res.items():
    rep[n] = {k: v for k, v in r.items() if k != "edges"}
    rep[n]["edges"] = r["edges"]
json.dump({"host_time_ns": time.time_ns(), "seconds": secs, "topics": rep}, open(out, "w"), indent=1, default=str)
for n, r in sorted(rep.items()):
    print(f"{n:55s} n={r['count']:6d} stamp_hz={r.get('stamp_rate_hz')} recv_hz={r.get('recv_rate_hz')} miss~{r.get('est_missing_msgs')} maxgap_ms={r.get('max_gap_ms')} MB/s={r['bytes_per_s']/1e6:.2f} frame={(r.get('first') or {}).get('frame_id')}")
