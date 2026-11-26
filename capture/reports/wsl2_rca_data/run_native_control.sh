#!/usr/bin/env bash
# Native-Ubuntu control for the WSL2 clock-step finding. Read-only: no sysctl, no network, no clock changes.
# Usage: bash run_native_control.sh [SECONDS=180]   -> writes native_control_<host>.json next to this script
set -e
D="$(cd "$(dirname "$0")" && pwd)"; S="${1:-180}"; H="$(hostname)"
echo "host=$H kernel=$(uname -r) wsl=$(grep -ci microsoft /proc/version) timesyncd=$(systemctl is-active systemd-timesyncd || true)"
python3 "$D/control_nodds.py" "$S" "$D/native_control_${H}_nodds.json" >/dev/null     # loop stalls, UDP loopback loss, REALTIME-vs-MONOTONIC steps
python3 "$D/clock_vs_timesyncd.py" "$S" "$D/native_control_${H}_clock.json"           # steps vs timesyncd polls
python3 - "$D/native_control_${H}_nodds.json" <<'PY'
import json,sys; d=json.load(open(sys.argv[1]))
print("loop stalls >100ms:",len(d["loop_stalls"]),"| UDP lost:",d["udp"]["lost"],"gaps:",len(d["udp"]["gaps"]),"| wall-clock steps >50ms:",len(d["clock_steps_ms"]),d["clock_steps_ms"])
print("EXPECTED on healthy native host: 0 / 0 / 0 / 0.  Any steps repeating every ~32 s reproduce the WSL2 finding.")
PY
