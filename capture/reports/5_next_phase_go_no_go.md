# 5. Next-phase GO / NO-GO — updated 2026-11-26 19:05 (supersedes the morning version)

## Verdict
* **GO** for continuing capture-side development and for **collecting data on WSL2 using header stamps**: two real 5-minute recordings (340 s, 320 s) record/inspect/replay cleanly; in a healthy-clock state there are **0 receive-time holes and 0 loss on RGB, stereo, camera_info, IMU** (header-stamp space); firmware, depth/stereo/time-sync state are now stored automatically (read-only camera API).
* **NO-GO for declaring P1 passed**, and for consuming **Depth** or **VIO** as if they were gap-free/semantically known: Depth ≈ 13.1 Hz (11.8–13.1 % missing in every run, camera-side), VIO 0.15–0.4 % loss, VIO/IMU/depth semantics unverified (see `capture/DATA_CONTRACT.md` §9). No native-Ubuntu result.
* `topics_v2.1.5.yaml`: 6 of 10 roles verified individually; file-level `verified: false`. `DATA_CONTRACT.md`: draft, not approved.

## Resolved since this morning
| item | result |
|---|---|
| WSL2 data "holes" | host wall-clock steps, not loss; currently dormant (0 steps in > 16 min incl. 5-min timesyncd stop); trigger of the afternoon state unknown |
| Firmware baseline | 2.1.5 adopted, 2.1.4 profile kept |
| Depth loss | located upstream of the host NIC (5 readers + rosbag identical, counters clean); camera-internal element undetermined |
| Camera parameters | depth-off / stereo 20 fps / time-sync tested one by one and restored (verified against backup) |

## Remaining blockers
| blocker | needed to close |
|---|---|
| Depth shortfall (13 vs 15 Hz) | vendor statement or fix; ideally a second unit and a native host |
| Camera time-sync does nothing visible | vendor documentation of the mechanism (source/protocol); then re-test (needs approval) |
| VIO loss and pose semantics; IMU axes; depth units | vendor docs + a moving test with known motion |
| Native Ubuntu control | a machine: run `capture/reports/wsl2_rca_data/run_native_control.sh`, then a 5-min recording |
| Formal DATA_CONTRACT | review/approval of the draft |

## Suggested next steps (not started)
1. Collect first real mapping-quality sessions on WSL2 with depth **off** if depth continuity is not needed (S1: all other streams had 0 gaps) — your call, it changes a camera setting.
2. Send vendor the depth evidence (`reports/3_depth_loss_root_cause.md`) and time-sync question.
3. Native-Ubuntu repeat when available.
