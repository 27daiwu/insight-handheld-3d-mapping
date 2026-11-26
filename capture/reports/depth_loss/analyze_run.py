#!/usr/bin/env python3
"""Per-run summary: for every consumer + the rosbag: count, mean rate, missing vs 15 Hz (stereo-frame steps), gap histogram,
agreement between consumers (are the SAME stamps missing everywhere?), host counters delta."""
import glob, json, os, sqlite3, statistics as st, sys, collections
sys.path.insert(0, __file__.rsplit("/reports/", 1)[0])
from insight_capture.stats import parse_header_stamp
run = sys.argv[1]; R = f"{os.path.dirname(os.path.abspath(__file__))}/runs/{run}"
C = {}
for f in sorted(glob.glob(R + "/*.json")):
    n = os.path.basename(f)[:-5]
    if n == "counters": continue
    C[n] = json.load(open(f))["stamps"]
for db in glob.glob(R + "/bag_depth/*.db3"):
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    C["rosbag_depth"] = [parse_header_stamp(h) for (h,) in con.execute("SELECT substr(data,1,12) FROM messages ORDER BY id")]
for db in glob.glob(R + "/bag_load/*.db3"):
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    names = dict(con.execute("SELECT id,name FROM topics"))
    cnt = dict(con.execute("SELECT topic_id,count(*) FROM messages GROUP BY topic_id"))
    print("load bag counts:", {names[k].split('camera/camera/')[-1]: v for k, v in cnt.items()})
def summ(stamps):
    s = [x for x in stamps if x]; 
    if len(s) < 3: return {"n": len(s)}
    d = [b - a for a, b in zip(s, s[1:])]; span = (s[-1] - s[0]) / 1e9
    u = collections.Counter(round(x / 33.333e6) for x in d)
    return {"n": len(s), "span_s": round(span, 1), "mean_hz": round((len(s) - 1) / span, 3), "missing_vs_15hz_pct": round(100 * (1 - len(s) / (span * 15 + 1)), 2),
            "step_units": dict(sorted(u.items())), "nonmono": sum(1 for x in d if x <= 0)}
out = {k: summ(v) for k, v in C.items()}
for k, v in out.items(): print(f"{k:14s}", v)
sets = {k: set(x for x in v if x) for k, v in C.items()}
ref = max(sets, key=lambda k: len(sets[k]))
print("agreement vs largest set (%s, %d):" % (ref, len(sets[ref])), {k: {"only_in_this": len(sets[k] - sets[ref]), "missing_vs_ref": len(sets[ref] - sets[k])} for k in sets if k != ref})
cj = json.load(open(R + "/counters.json")); b, a = cj["before"], cj["after"]
dif = lambda k: {x: a[k][x] - b[k][x] for x in a[k]}
print("udp delta:", {k: v for k, v in dif("udp").items() if v}); print("iface", cj["iface"], dif("if")); print("windows adapters before/after:", b["win"], a["win"])
cs = [(round(x["cpu-monitor"].get("average", -1), 1) if isinstance(x["cpu-monitor"], dict) else None, x["system-info"].get("temperature") if isinstance(x["system-info"], dict) else None) for x in cj["cam_samples"]]
print("camera (cpu% avg, temp C) samples:", cs)
json.dump({"summary": out, "udp_delta": dif("udp"), "if_delta": dif("if"), "cam": cs}, open(R + "/summary.json", "w"), indent=1)
