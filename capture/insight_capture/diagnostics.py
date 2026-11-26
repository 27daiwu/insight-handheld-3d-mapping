"""Continuity / timestamp / loss / multi-topic synchronisation diagnostics.

Pure python on (stamp_ns, recv_ns) series, so it works on a rosbag2 file, a live probe or synthetic data.
It only *describes* the data; it never concludes that a camera is healthy from simulator input.
"""
from __future__ import annotations

import statistics

NS = 1_000_000_000


def gaps(times_ns: list[int], ref_period_ns: float | None = None, gap_factor: float = 1.5,
         min_gap_ns: int = 0) -> list[dict]:
    """Holes in a monotonic time series. Each: {start_ns, end_ns, dur_ms, missing}."""
    ts = [t for t in times_ns if t is not None]
    if len(ts) < 3:
        return []
    d = [b - a for a, b in zip(ts, ts[1:])]
    pos = [x for x in d if x > 0]
    if not pos:
        return []
    ref = ref_period_ns or statistics.median(pos)
    thr = max(ref * gap_factor, min_gap_ns)
    out = []
    for i, (a, b, x) in enumerate(zip(ts, ts[1:], d), 1):
        if x > thr:
            out.append({"start_ns": a, "end_ns": b, "dur_ms": round(x / 1e6, 3),
                        "missing": max(round(x / ref) - 1, 0), "idx": i})
    return out


def continuity(stamps_ns, recv_ns=None, nominal_hz=None, gap_factor=1.5) -> dict:
    """Per-topic continuity summary using header stamps (falls back to receive time).

    loss ratios: `loss_vs_median` is relative to the stream's own median period (always available);
    `loss_vs_reference` is relative to a configured/expected rate (the one that matters for acceptance).
    """
    use = [s for s in stamps_ns if s is not None]
    basis = "header_stamp"
    if len(use) < 3 and recv_ns:
        use, basis = list(recv_ns), "recv_time"
    out: dict = {"basis": basis, "count": len(use)}
    if len(use) < 3:
        return out
    diffs = [b - a for a, b in zip(use, use[1:])]
    pos = [x for x in diffs if x > 0]
    med = statistics.median(pos) if pos else 0
    ref = (NS / nominal_hz) if nominal_hz else med
    g = gaps(use, ref, gap_factor)
    span = (use[-1] - use[0])
    expected_ref = round(span / ref) + 1 if ref else None
    missing = sum(x["missing"] for x in g)
    out.update(
        span_s=round(span / NS, 3),
        median_period_ms=round(med / 1e6, 4),
        gap_events=len(g), est_missing=missing,
        loss_vs_median=round(missing / (len(use) + missing), 5),
        gaps=g,
    )
    if expected_ref:
        out["expected_count"] = expected_ref
        out["loss_vs_reference"] = round(max(expected_ref - len(use), 0) / expected_ref, 5)
    if pos:
        q = sorted(pos)
        out["interval_ms"] = {"min": round(q[0] / 1e6, 3), "p50": round(q[len(q) // 2] / 1e6, 3),
                              "p99": round(q[int(len(q) * 0.99)] / 1e6, 3), "max": round(q[-1] / 1e6, 3)}
        out["jitter_ms"] = round(statistics.pstdev(pos) / 1e6, 4) if len(pos) > 2 else 0.0
    out["non_monotonic"] = sum(1 for x in diffs if x < 0)
    out["duplicates"] = sum(1 for x in diffs if x == 0)
    # stamp-vs-receive: distinguishes "never arrived" (recv gap) from "arrived late" (stamp gap, no recv gap)
    if recv_ns and len(recv_ns) == len(stamps_ns) and basis == "header_stamp":
        pairs = [(s, r) for s, r in zip(stamps_ns, recv_ns) if s is not None]
        lat = [r - s for s, r in pairs]
        out["recv_minus_stamp_s"] = {"median": round(statistics.median(lat) / NS, 4),
                                     "spread_ms": round((max(lat) - min(lat)) / 1e6, 3)}
    return out


def systemic_gaps(per_topic: dict[str, dict], min_topics: int = 3, tol_s: float = 0.5,
                  min_gap_s: float = 0.5) -> dict:
    """Find holes that hit several topics at (nearly) the same wall time.

    Many independent sensors losing data in the same window points at the shared path
    (transport / host / recorder) and not at one sensor. Uses receive time so it is comparable
    across topics whose header clocks differ. per_topic[name] needs 'gaps_recv' = [{start_ns,end_ns}].
    """
    events = []
    for name, d in per_topic.items():
        for gp in d.get("gaps_recv", []):
            if (gp["end_ns"] - gp["start_ns"]) < min_gap_s * NS:
                continue            # ms-scale jitter is per-topic noise; chaining it would fabricate long holes
            events.append((gp["start_ns"], gp["end_ns"], name, gp.get("recv_only")))   # None = not evaluable (no header stamp)
    events.sort()
    clusters = []
    for s, e, n, ro in events:
        for c in clusters:
            if s <= c["end"] + tol_s * NS and e >= c["start"] - tol_s * NS and (max(e, c["end"]) - min(s, c["start"])) < 10 * NS:
                c["start"], c["end"] = min(c["start"], s), max(c["end"], e)
                c["topics"].add(n)
                c["ro"].append(ro)
                break
        else:
            clusters.append({"start": s, "end": e, "topics": {n}, "ro": [ro]})
    sys_ev = [c for c in clusters if len(c["topics"]) >= min_topics]
    out = {"clusters_total": len(clusters), "systemic_events": len(sys_ev), "min_topics": min_topics,
           "events": [{"start_ns": c["start"], "dur_s": round((c["end"] - c["start"]) / NS, 3),
                       "n_topics": len(c["topics"]), "topics": sorted(c["topics"]),
                       "recv_only": bool([x for x in c["ro"] if x is not None]) and all(x for x in c["ro"] if x is not None)} for c in sys_ev]}
    out["recv_only_events"] = sum(1 for e in out["events"] if e["recv_only"])
    out["data_loss_events"] = out["systemic_events"] - out["recv_only_events"]
    if len(sys_ev) >= 3:
        starts = [c["start"] for c in sys_ev]
        per = [(b - a) / NS for a, b in zip(starts, starts[1:])]
        out["period_s"] = {"median": round(statistics.median(per), 2), "min": round(min(per), 2),
                           "max": round(max(per), 2)}
        out["periodic"] = (max(per) - min(per)) < 0.2 * statistics.median(per)
    elif sys_ev:
        out["periodic"] = None
    else:
        out["periodic"] = False
    return out


def pair_skew(stamps_a: list[int], stamps_b: list[int], max_dt_ms: float = 20.0) -> dict:
    """Nearest-stamp matching between two streams (e.g. left/right, rgb/stereo)."""
    a = [x for x in stamps_a if x is not None]
    b = [x for x in stamps_b if x is not None]
    if not a or not b:
        return {"matched": 0}
    import bisect
    d = []
    for t in a:
        i = bisect.bisect_left(b, t)
        c = [b[j] - t for j in (i - 1, i) if 0 <= j < len(b)]
        d.append(min(c, key=abs))
    ok = [x for x in d if abs(x) <= max_dt_ms * 1e6]
    res = {"matched": len(ok), "unmatched": len(d) - len(ok), "match_ratio": round(len(ok) / len(d), 4)}
    if ok:
        res.update(mean_ms=round(statistics.mean(ok) / 1e6, 4), max_abs_ms=round(max(abs(x) for x in ok) / 1e6, 4))
    return res


def diagnose(series: dict[str, dict], nominal: dict[str, float] | None = None, sync_pairs=None,
             gap_factor: float = 1.5) -> dict:
    """series[name] = {"stamps": [...], "recv": [...], "role": str}. Returns a JSON-able report."""
    nominal = nominal or {}
    per, rep = {}, {"topics": {}}
    for name, s in series.items():
        c = continuity(s["stamps"], s["recv"], nominal.get(name), gap_factor)
        rg = gaps(s["recv"], (NS / nominal[name]) if nominal.get(name) else None, gap_factor) if s.get("recv") else []
        st_ok = [x for x in s["stamps"] if x is not None]
        if rg and len(st_ok) == len(s["stamps"]) and len(st_ok) > 3:
            med = statistics.median([b - a for a, b in zip(st_ok, st_ok[1:]) if b > a] or [0])
            for gp in rg:
                i = gp["idx"]
                dt = st_ok[i] - st_ok[i - 1]
                gp["stamp_dt_ms"] = round(dt / 1e6, 3)
                # header stamps contiguous (<= 1 missing message) => data present, only the arrival time jumped.
                # med == 0 (e.g. 1 Hz String with constant stamps) is not evaluable -> None.
                gp["recv_only"] = (dt <= 2.05 * med) if med else None
        c["recv_only_gap_events"] = sum(1 for gp in rg if gp.get("recv_only"))
        c["gaps_recv"] = rg
        c["recv_gap_events"] = len(rg)
        per[name] = c
        rep["topics"][name] = {k: v for k, v in c.items() if k not in ("gaps", "gaps_recv")} | {
            "gaps_head": c.get("gaps", [])[:10], "recv_gaps_head": rg[:10]}
    rep["systemic"] = systemic_gaps({n: {"gaps_recv": c["gaps_recv"]} for n, c in per.items()})
    sk = {}
    for a, b in sync_pairs or []:
        if a in series and b in series:
            sk[f"{a} ~ {b}"] = pair_skew(series[a]["stamps"], series[b]["stamps"])
    rep["pair_skew"] = sk
    return rep


def verdicts(rep: dict, th: dict) -> list[tuple[str, str]]:
    """(level, message). Strict: any loss vs reference beyond tolerance is WARN, systemic holes are FAIL."""
    out = []
    tol = th.get("max_loss_ratio", 0.001)
    for n, t in rep["topics"].items():
        lr = t.get("loss_vs_reference")
        if t.get("basis") == "recv_time":
            continue            # no header stamp: arrival-time based loss would just echo host clock artefacts
        if lr is not None and lr > tol:
            out.append(("WARN", f"{n}: loss {lr * 100:.2f}% vs reference rate ({t['gap_events']} gaps)"))
        elif t.get("loss_vs_median", 0) > tol:
            out.append(("WARN", f"{n}: loss {t['loss_vs_median'] * 100:.2f}% vs own median period"))
        if t.get("non_monotonic"):
            out.append(("WARN", f"{n}: {t['non_monotonic']} non-monotonic stamps"))
    s = rep["systemic"]
    if s["systemic_events"] and s.get("data_loss_events", 1) == 0:
        out.append(("WARN", f"{s['systemic_events']} multi-topic RECEIVE-TIME holes with contiguous header stamps: "
                            "data NOT lost, host receive timestamps jumped (host clock steps / VM time sync / stalled "
                            "recorder + catch-up). Receive-time based analyses on this bag are unreliable"))
    elif s["systemic_events"]:
        extra = f", periodic ~{s['period_s']['median']}s" if s.get("periodic") else ""
        out.append(("FAIL", f"{s['systemic_events']} multi-topic holes hit >= {s['min_topics']} topics at once{extra}"
                            " (shared path: transport/host/recorder, not one sensor)"))
    for k, v in rep["pair_skew"].items():
        if v.get("matched") and v.get("match_ratio", 1) < 0.99:
            out.append(("WARN", f"sync {k}: only {v['match_ratio'] * 100:.1f}% of frames matched within 20 ms"))
    return out
