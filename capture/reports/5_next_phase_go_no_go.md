# 5. Next-phase GO / NO-GO — 2026-11-26

## Verdict
* **GO** — continue capture-tool development and a first real data collection campaign **on WSL2, using header stamps** (tool works on the real camera: record/inspect/replay/manifest verified over 340 s).
* **NO-GO for declaring P1-C "accepted"** and for starting any data-consuming phase that needs (a) host↔camera time alignment, (b) gap-free Depth, or (c) validated native-Ubuntu operation. `topics_v2.1.4.yaml` stays `verified: false`; `DATA_CONTRACT.md` stays provisional.

## Why
| open blocker | state | to close |
|---|---|---|
| Host clock steps in WSL2 corrupt bag receive time (±2.5 s) | root cause **mechanism confirmed**, origin of slow tick not determined | approve diagnostic (stop timesyncd 5 min) or run on native Ubuntu |
| Depth loses 12.65 % frames; IMU 0.12 %, VIO 0.29 % | unexplained, real (camera stamps) | native run; depth-off / lower stereo fps comparison (camera setting → approval) |
| Camera clock boot-relative, wall clock 457 days off, time-sync off | known | enable camera time-sync/NTP (approval); decide hardware-sync strategy |
| Firmware 2.1.5 ≠ baseline 2.1.4; depth found ON (contradicts default-off) | known | decide new baseline; confirm default |
| No native Ubuntu result | not available to me | run `reports/wsl2_rca_data/run_native_control.sh` + a 5-min recording there |
| `image_raw` topics empty; VIO `world` frame not in TF | open | ask vendor / check on subscription |
| Official v2.1.4 topic list, project DATA_CONTRACT.md | absent | provide |

## Decisions needed from you
1. Allow the 5-minute `systemd-timesyncd` stop experiment (reversible) — or skip and go native.
2. Allow changing camera settings for the Depth/fps comparison and time-sync? (not touched so far)
3. Baseline firmware: 2.1.5?
4. Provide DATA_CONTRACT.md and a native Ubuntu 22.04 machine/time slot.

## Not done / scope notes
Native Ubuntu runs; Cyclone DDS arm (not needed after RCA); DDS QoS/executor arms (symptom was not a DDS effect); a first subagent for B died without output (tooling failure) and I did B myself. Subagent model availability: only `starflow-claude/mcs-7`; no DeepSeek model is listed, so none was used.
