#!/usr/bin/env python3
"""Authorised experiment (2026-11-26): temporarily stop systemd-timesyncd (NOT disable/mask, no config edits), observe clocks.
Phases: A baseline (active) -> B stopped -> C restored. Always restores the original state (finally + signals).
Samples: CLOCK_REALTIME-CLOCK_MONOTONIC steps (10 ms), WSL-vs-Windows offset (bracketed powershell UtcNow every ~1.5 s),
timesyncd status, adjtimex(read-only). Output JSON."""
import atexit, ctypes, ctypes.util, json, re, signal, subprocess, sys, threading, time
A, B, C = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]); OUT = sys.argv[4]
ROOT = ["wsl.exe", "-d", "Ubuntu-22.04", "-u", "root", "--", "systemctl"]
def sh(cmd, t=30): return subprocess.run(cmd, capture_output=True, text=True, timeout=t)
def state(): return {"journal_lines_since_stop": None, "active": sh(["systemctl", "is-active", "systemd-timesyncd"]).stdout.strip(),
                     "enabled": sh(["systemctl", "is-enabled", "systemd-timesyncd"]).stdout.strip()}
class TX(ctypes.Structure): _fields_=[('modes',ctypes.c_uint),('offset',ctypes.c_long),('freq',ctypes.c_long),('maxerror',ctypes.c_long),('esterror',ctypes.c_long),('status',ctypes.c_int),('constant',ctypes.c_long),('precision',ctypes.c_long),('tolerance',ctypes.c_long),('time_s',ctypes.c_long),('time_us',ctypes.c_long),('tick',ctypes.c_long),('ppsfreq',ctypes.c_long),('jitter',ctypes.c_long),('shift',ctypes.c_int),('stabil',ctypes.c_long),('jitcnt',ctypes.c_long),('calcnt',ctypes.c_long),('errcnt',ctypes.c_long),('stbcnt',ctypes.c_long),('tai',ctypes.c_int),('pad',ctypes.c_int*11)]
libc = ctypes.CDLL(ctypes.util.find_library('c'), use_errno=True)
def adj():
    t = TX(); t.modes = 0; libc.adjtimex(ctypes.byref(t)); return {"tick": t.tick, "freq_ppm": round(t.freq/65536, 2), "status": hex(t.status)}
def tsync():
    o = sh(["timedatectl", "timesync-status"]).stdout
    g = lambda k: (re.search(k + r":\s*(.+)", o) or [None, None])[1]
    return {"offset": g("Offset"), "packets": g("Packet count"), "poll": g("Poll interval")}
orig = state(); res = {"original_state": orig, "start_epoch": time.time(), "phases": [], "events": [], "steps": [], "samples": []}
stopped = {"v": False}
def restore():
    if stopped["v"]:
        r = subprocess.run(ROOT + ["start", "systemd-timesyncd"], capture_output=True, text=True, timeout=60)
        res["events"].append({"t": time.time(), "ev": "RESTORE start", "rc": r.returncode, "state": state()}); stopped["v"] = False
    res["final_state"] = state(); res["end_epoch"] = time.time(); json.dump(res, open(OUT, "w"), indent=1)
atexit.register(restore)
for s in (signal.SIGINT, signal.SIGTERM): signal.signal(s, lambda *a: (restore(), sys.exit(1)))
phase = {"name": "A_baseline_active"}; T0 = time.monotonic(); stop_flag = False
def stepper():
    pm, pr = time.monotonic_ns(), time.time_ns()
    while not stop_flag:
        time.sleep(0.01); m, r = time.monotonic_ns(), time.time_ns(); d = (r-pr)-(m-pm)
        if abs(d) > 50e6: res["steps"].append({"t": round(time.monotonic()-T0, 2), "phase": phase["name"], "ms": round(d/1e6)})
        pm, pr = m, r
threading.Thread(target=stepper, daemon=True).start()
def sample():
    l0, m0 = time.time_ns(), time.monotonic_ns()
    w = int(sh(["powershell.exe", "-NoProfile", "-Command", "[DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()"], 40).stdout.strip())
    l1, m1 = time.time_ns(), time.monotonic_ns()
    res["samples"].append({"t": round((m0+m1)/2e9 - T0, 2), "phase": phase["name"], "rt_minus_win_ms": round(((l0+l1)/2 - w*1e6)/1e6, 1), "latency_ms": round((l1-l0)/1e6),
                           "mono_ns": (m0+m1)//2, "win_ms": w})
def run_phase(name, dur, td=True):
    phase["name"] = name; t_end = time.monotonic() + dur
    res["phases"].append({"name": name, "t_start": round(time.monotonic()-T0, 2), "adjtimex": adj(), "timesyncd": (tsync() if td else "NOT QUERIED (timedatectl would D-Bus-activate timesyncd)"), "state": state(), "active_enter_mono": sh(["systemctl","show","systemd-timesyncd","-p","ActiveEnterTimestampMonotonic"]).stdout.strip()})
    while time.monotonic() < t_end: sample(); time.sleep(0.6)
    res["phases"][-1].update(t_end=round(time.monotonic()-T0, 2), adjtimex_end=adj(), timesyncd_end=(tsync() if td else "NOT QUERIED"), state_end=state(), active_enter_mono_end=sh(["systemctl","show","systemd-timesyncd","-p","ActiveEnterTimestampMonotonic"]).stdout.strip())
try:
    run_phase("A_baseline_active", A)
    r = subprocess.run(ROOT + ["stop", "systemd-timesyncd"], capture_output=True, text=True, timeout=60); stopped["v"] = True
    res["events"].append({"t": time.time(), "tmono": round(time.monotonic()-T0, 2), "ev": "STOP systemd-timesyncd", "rc": r.returncode, "state": state()})
    run_phase("B_timesyncd_stopped", B, td=False)
    restore(); res["events"][-1]["tmono"] = round(time.monotonic()-T0, 2)
    run_phase("C_restored", C)
finally:
    stop_flag = True; restore()
print("done; final state", res["final_state"], "original", orig)
