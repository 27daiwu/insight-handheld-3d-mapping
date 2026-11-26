#!/usr/bin/env python3
"""One independent rclpy raw subscriber on a topic. Usage: depth_consumers.py TOPIC TYPE QOS(rel|be) DEPTH SECONDS OUT.json [executor=single|multi]
Logs header stamp (first 12 CDR bytes) and receive monotonic time per message."""
import json, sys, time
sys.path.insert(0, __file__.rsplit("/reports/", 1)[0])
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from insight_capture.stats import parse_header_stamp
topic, typ, q, depth, secs, out = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4]), float(sys.argv[5]), sys.argv[6]
ex_kind = sys.argv[7] if len(sys.argv) > 7 else "single"
rclpy.init(); n = rclpy.create_node("depth_cons_%d" % int(time.time() * 1000 % 100000), enable_rosout=False)
qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth, reliability=ReliabilityPolicy.RELIABLE if q == "rel" else ReliabilityPolicy.BEST_EFFORT)
st, rx = [], []
def cb(d): rx.append(time.monotonic_ns()); st.append(parse_header_stamp(d))
n.create_subscription(get_message(typ), topic, cb, qos, raw=True)
ex = MultiThreadedExecutor(2) if ex_kind == "multi" else SingleThreadedExecutor(); ex.add_node(n)
t0 = time.monotonic(); end = t0 + secs
while time.monotonic() < end: ex.spin_once(timeout_sec=0.05)
json.dump({"topic": topic, "qos": q, "depth": depth, "executor": ex_kind, "secs": secs, "stamps": st, "recv_mono_ns": rx}, open(out, "w"))
