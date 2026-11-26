#!/usr/bin/env python3
"""Read-only: log REALTIME-vs-MONOTONIC steps (>50 ms) and the moments systemd-timesyncd's 'Packet count' increments."""
import subprocess, sys, time, re, json
secs = float(sys.argv[1]); out = sys.argv[2]
def pc():
    o = subprocess.run(["timedatectl", "timesync-status"], capture_output=True, text=True).stdout
    m = re.search(r"Packet count:\s*(\d+)", o); off = re.search(r"Offset:\s*([+-][\d.]+)(ms|s|us)", o)
    return (int(m.group(1)) if m else None), (off.group(0) if off else None)
t0 = time.monotonic(); pm, pr = time.monotonic_ns(), time.time_ns(); steps = []; polls = []; lastpc, _ = pc(); nextpoll = 0
while time.monotonic() - t0 < secs:
    time.sleep(0.01); m, r = time.monotonic_ns(), time.time_ns()
    d = (r - pr) - (m - pm); pm, pr = m, r
    if abs(d) > 50e6: steps.append((round(time.monotonic() - t0, 2), round(d / 1e6)))
    if time.monotonic() - t0 >= nextpoll:
        nextpoll += 0.5; c, o = pc()
        if c != lastpc: polls.append((round(time.monotonic() - t0, 2), c, o)); lastpc = c
json.dump({"steps_ms": steps, "timesyncd_packet_events": polls}, open(out, "w"), indent=1)
print("steps (t_s, ms):", steps); print("timesyncd packet-count changes (t_s, count, offset):", polls)
