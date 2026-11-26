#!/usr/bin/env python3
"""No-DDS control: (1) busy-loop scheduler stall logger, (2) UDP loopback 1 kHz seq sender/receiver (separate processes),
(3) CLOCK_MONOTONIC vs REALTIME jump logger. Usage: control_nodds.py SECONDS OUT.json
Reports every stall > 100 ms (loop), every receive gap > 100 ms and lost seq (udp), every realtime-vs-monotonic step > 50 ms."""
import json, multiprocessing as mp, os, socket, sys, time
SECS = float(sys.argv[1]); OUT = sys.argv[2]; PORT = 47123
M = time.monotonic_ns

def stall_loop(q, secs):
    end = M() + int(secs * 1e9); last = M(); ev = []
    while M() < end:
        n = M()
        if n - last > 100e6: ev.append((last, n))
        last = n
    q.put(("loop", ev))

def clock_jump(q, secs):
    end = M() + int(secs * 1e9); m0, r0 = M(), time.time_ns(); ev = []; pm, pr = m0, r0
    while M() < end:
        time.sleep(0.01); m, r = M(), time.time_ns()
        step = (r - pr) - (m - pm)
        if abs(step) > 50e6: ev.append((m, step))
        pm, pr = m, r
    q.put(("clock", ev))

def udp_tx(secs):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); i = 0
    t = M(); end = t + int(secs * 1e9) + int(1e9); per = 1_000_000
    gaps = []
    while M() < end:
        s.sendto(i.to_bytes(8, "little") + M().to_bytes(8, "little"), ("127.0.0.1", PORT)); i += 1
        t += per
        d = t - M()
        if d > 0: time.sleep(d / 1e9)
    return

def udp_rx(q, secs):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    s.bind(("127.0.0.1", PORT)); s.settimeout(0.5)
    seqs = []; gaps = []; last = None; end = M() + int((secs + 0.5) * 1e9); t0 = M()
    while M() < end:
        try: d = s.recv(64)
        except socket.timeout: continue
        n = M(); seq = int.from_bytes(d[:8], "little"); sent = int.from_bytes(d[8:16], "little")
        if last is not None and n - last > 100e6: gaps.append((last, n))
        last = n; seqs.append((seq, n - sent))
    lost = (seqs[-1][0] - seqs[0][0] + 1 - len(seqs)) if seqs else None
    lat = sorted(x[1] for x in seqs)
    q.put(("udp", {"gaps": gaps, "received": len(seqs), "lost": lost, "lat_ms_p50": lat[len(lat)//2]/1e6 if lat else None,
                   "lat_ms_max": lat[-1]/1e6 if lat else None}))

if __name__ == "__main__":
    q = mp.Queue(); t0 = M()
    procs = [mp.Process(target=stall_loop, args=(q, SECS)), mp.Process(target=clock_jump, args=(q, SECS)),
             mp.Process(target=udp_rx, args=(q, SECS)), mp.Process(target=udp_tx, args=(SECS,))]
    for p in procs[:3]: p.start()
    time.sleep(0.3); procs[3].start()
    res = {}
    for _ in range(3):
        k, v = q.get(timeout=SECS + 30); res[k] = v
    for p in procs: p.join()
    rel = lambda x: round((x - t0) / 1e9, 2)
    out = {"seconds": SECS, "start_realtime": time.time(), "uptime_s": float(open("/proc/uptime").read().split()[0]),
           "loop_stalls": [(rel(a), round((b - a) / 1e6)) for a, b in res["loop"]],
           "clock_steps_ms": [(rel(m), round(s / 1e6)) for m, s in res["clock"]],
           "udp": {**{k: v for k, v in res["udp"].items() if k != "gaps"},
                   "gaps": [(rel(a), round((b - a) / 1e6)) for a, b in res["udp"]["gaps"]]}}
    json.dump(out, open(OUT, "w"), indent=1)
    print(json.dumps({k: (v if k in ("seconds",) else (len(v) if isinstance(v, list) else {kk: (len(vv) if isinstance(vv, list) else vv) for kk, vv in v.items()})) for k, v in out.items()}))
