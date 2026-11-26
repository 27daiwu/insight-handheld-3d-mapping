#!/usr/bin/env python3
"""Depth loss experiment. Runs several INDEPENDENT consumers on the depth topic in the same window, optionally together
with a rosbag2 recorder (depth only) and optionally with load (all other camera topics recorded by a second bag).
Collects host-side counters before/after: /proc/net/snmp Udp, NCM interface stats, Windows adapter stats, camera cpu/temp.
Usage: depth_experiment.py LABEL SECONDS [--load]"""
import glob, json, os, re, subprocess, sys, time, urllib.request
label, secs = sys.argv[1], float(sys.argv[2]); load = "--load" in sys.argv
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = f"{HERE}/runs/{label}"; os.makedirs(OUT, exist_ok=True)
D = "/camera/camera/depth/image_rect_raw"; IMG = "sensor_msgs/msg/Image"
def iface():
    for l in subprocess.run(["ip", "-br", "addr"], capture_output=True, text=True).stdout.splitlines():
        if "169.254.10.2" in l: return l.split()[0]
IF = iface()
def udp():
    h, v = [l.split() for l in open("/proc/net/snmp") if l.startswith("Udp:")]; return dict(zip(h[1:], map(int, v[1:])))
def ifs(): return {f: int(open(f"/sys/class/net/{IF}/statistics/{f}").read()) for f in ("rx_packets", "rx_bytes", "rx_dropped", "rx_errors", "rx_fifo_errors", "rx_over_errors")}
def win():
    r = subprocess.run(["powershell.exe", "-NoProfile", "-Command", "Get-NetAdapterStatistics | ForEach-Object { '{0}|{1}|{2}|{3}|{4}' -f $_.Name,$_.ReceivedBytes,$_.ReceivedDiscardedPackets,$_.ReceivedPacketErrors,$_.ReceivedUnicastPackets }"], capture_output=True, timeout=40)
    return r.stdout.decode('gbk', errors='replace').strip().splitlines()
def cam():
    o = {}
    for p in ("cpu-monitor", "system-info"):
        try:
            op = urllib.request.build_opener(urllib.request.ProxyHandler({})); o[p] = json.loads(op.open(f"http://169.254.10.1/api/{p}", timeout=5).read().decode())
        except Exception as e: o[p] = str(e)
    return o
snap = lambda: {"t": time.time(), "udp": udp(), "if": ifs(), "win": win(), "cam": cam()}
specs = [("rel10", "rel", 10, "single"), ("rel1000", "rel", 1000, "single"), ("be10", "be", 10, "single"), ("rel10_multi", "rel", 10, "multi")]
procs = []; bagp = []; before = snap(); cam_samples = []
for name, q, dep, ex in specs:
    procs.append(subprocess.Popen([sys.executable, f"{HERE}/depth_consumers.py", D, IMG, q, str(dep), str(secs), f"{OUT}/{name}.json", ex]))
bagp.append(subprocess.Popen(["ros2", "bag", "record", "-o", f"{OUT}/bag_depth", "-s", "sqlite3", D], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
if load:
    others = ["/camera/camera/infra1/image_rect_raw", "/camera/camera/infra2/image_rect_raw", "/camera/camera/color/image_rect_raw/compressed", "/camera/camera/imu", "/camera/camera/vio_100hz", "/camera/camera/color/camera_info", "/camera/camera/infra1/camera_info", "/camera/camera/infra2/camera_info"]
    bagp.append(subprocess.Popen(["ros2", "bag", "record", "-o", f"{OUT}/bag_load", "-s", "sqlite3"] + others, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
t_end = time.monotonic() + secs + 3
while time.monotonic() < t_end:
    time.sleep(15); cam_samples.append({"t": time.time(), **cam()})
for p in procs: p.wait(timeout=60)
for b in bagp: b.send_signal(2)
for b in bagp: b.wait(timeout=90)
after = snap()
json.dump({"label": label, "secs": secs, "load": load, "iface": IF, "before": before, "after": after, "cam_samples": cam_samples}, open(f"{OUT}/counters.json", "w"), indent=1)
print("done", label)
