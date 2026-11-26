import glob, json, os, sqlite3, sys
sys.path.insert(0, __file__.rsplit("/reports/", 1)[0])
from insight_capture.stats import parse_header_stamp
H = os.path.dirname(os.path.abspath(__file__)); out = {}
for run in ("R1_depth_only_4consumers", "R2_depth_plus_full_load"):
    R = f"{H}/runs/{run}"; C = {os.path.basename(f)[:-5]: [x for x in json.load(open(f))["stamps"] if x] for f in glob.glob(R + "/[brm]*.json") if "counters" not in f}
    C = {k: v for k, v in C.items() if k in ("rel10", "rel1000", "be10", "rel10_multi")}
    con = sqlite3.connect("file:" + glob.glob(R + "/bag_depth/*.db3")[0] + "?mode=ro", uri=True)
    bag = [parse_header_stamp(h) for (h,) in con.execute("SELECT substr(data,1,12) FROM messages ORDER BY id")]
    lo = max(min(v) for v in list(C.values()) + [bag]); hi = min(max(v) for v in list(C.values()) + [bag])
    win = {k: set(x for x in v if lo <= x <= hi) for k, v in list(C.items()) + [("rosbag", bag)]}
    union = set().union(*win.values()); inter = set.intersection(*win.values())
    r = {"window_s": round((hi - lo) / 1e9, 1), "union": len(union), "intersection": len(inter),
         "per_reader": {k: {"count": len(v), "missing_vs_union": len(union - v)} for k, v in win.items()}}
    exp15 = (hi - lo) / 1e9 * 15 + 1; r["expected_at_15Hz"] = round(exp15); r["union_missing_vs_15Hz_pct"] = round(100 * (1 - len(union) / exp15), 2)
    out[run] = r; print(run, json.dumps(r))
json.dump(out, open(H + "/layer_compare.json", "w"), indent=1)
