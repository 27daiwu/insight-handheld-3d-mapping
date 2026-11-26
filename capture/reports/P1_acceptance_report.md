# P1 acceptance report (re-assessment) — 2026-11-26 evening
Supersedes the acceptance matrix in `3_capture_tool_test_report.md`. Platform: **WSL2 only** (no native Ubuntu result). Device: Insight 9 fw 2.1.5. All numbers are from the REAL camera unless stated; simulator results are not used.

## Overall verdict
**P1-C is NOT fully passed.** Core capture works and is repeatable; two items remain open: (1) **Depth delivers ≈ 13.1 Hz instead of 15 Hz (11.8–13.1 % missing in every run) — located camera-side but not fixed or fully explained**; (2) **no native-Ubuntu run**. VIO shows a small unexplained loss (0.15–0.4 %). Per instruction, P1 is not declared passed while the Depth cause is not resolved.

## Two real 5-minute recordings, clock artefacts and sensor loss counted separately
| | Run A 14:36 | Run B 18:48 |
|---|---|---|
| duration / size / files | 340.6 s / 12.0 GB / 6 | 320.2 s / 11.0 GB / 6 |
| host clock state (preflight/experiments) | **stepping** (+2.4 s / 32 s; seen 16:07–16:40; during this recording inferred from signature) | **healthy** (0 steps in > 16 min, ±60 ppm) |
| **receive-time holes (clock artefact — NOT data loss)** | **9** holes on all topics, 3.1–4.0 s, median period 34.9 s, header stamps contiguous at every hole ⇒ 0 messages lost in them | **0** |
| **sensor loss, header-stamp space (real)** | | |
| RGB 30 Hz / camera_info ×3 | 0 / 0 | 0 / 0 |
| stereo L/R 30 Hz | 0 / 0 (skew 0 ms) | 0 / 0 (skew 0 ms) |
| IMU 400 Hz | 0.12 % (167 msgs) | **0** (128 081 = 400.0 × 320.2 s) |
| VIO 100 Hz | 0.29 % (98) | 0.38 % (123) |
| **Depth 15 Hz nominal** | **12.65 %** (4473 frames, 13.10 Hz) | **12.83 %** (4186 frames, 13.07 Hz) |
| manifest | fw given by hand (2.1.5) | fw **auto-read from camera API**, API snapshot stored, `complete/duration_reached` |
Replay of both bags: exit 0, stamp rates reproduced (A: imu 400.0, stereo 30.0, vio 100.0, depth 15.0 header-rate; B: see below).

## P1-C criteria
| criterion | status | note |
|---|---|---|
| camera connected normally | **PASS** (WSL2) | USB-NCM link, API + DDS on domain 9; native Ubuntu not tried |
| core topics publish | **PASS** | rgb(JPEG), stereo, imu, vio, camera_info×3, `/tf_static`; depth publishes; `/tf` absent (never published) |
| rates match configuration | **PASS** rgb/stereo/imu/vio/info; **FAIL depth** (13.1 vs 15 Hz) | |
| ≥ 5 min recording | **PASS** ×2 | |
| rosbag2 replays | **PASS** | |
| manifest complete | **PASS** | firmware now read-only from the camera API |
| no obvious timestamp anomaly | **PASS in header-stamp space**; receive-time anomalies exist only while the WSL2 clock steps (classified, tool warns) | camera stamps are boot-relative, camera wall clock 457 days off, camera time-sync toggle has no effect ⇒ **host↔camera alignment not available** |
| no unexplained data loss | **NOT MET** | Depth 12–13 % camera-side, root element unknown (report 3); VIO 0.15–0.4 % unexplained, appears with depth on, absent with depth off in a 90 s test |
| native Ubuntu | **NOT DONE** | `wsl2_rca_data/run_native_control.sh` waiting |

## Status of the requested investigations
| item | result |
|---|---|
| 1 timesyncd 5-min stop | done, restored; clock healthy now, hypothesis not re-testable by intervention (report 1) |
| 2 camera parameters (depth off / stereo 20 fps / time-sync) | done one-by-one, restored and verified equal to backup (report 2) |
| 3 baseline 2.1.5 | `topics_v2.1.5.yaml` added, 2.1.4 kept; per-role verification: 6 of 10 roles verified, file-level `verified: false` |
| 4 Depth loss | located camera-side with 5-reader/3-layer evidence; exact camera element undetermined (report 3) |
| 5 DATA_CONTRACT | draft v0.2 with [V]/[O]/[?] tags (`capture/DATA_CONTRACT.md`) |
| 6 native Ubuntu | not available, not blocking |

## Tool/test status
`pytest capture/tests`: see `3_capture_tool_test_report.md` (updated counts). New this round: read-only camera API client with whitelist (never POSTs), preflight reads firmware/depth/stereo/time-sync, clock-domain warning aggregated once, headerless types not stamp-parsed (fixed a false "duplicate stamps" warning on `vio_status`), profile governance tests.

## Open items (owner)
1. Depth ≈ 13 Hz vs 15 Hz — vendor confirmation / second unit / native run (you/vendor).
2. VIO 0.15–0.4 % loss and semantics; IMU axes; depth units (contract §9).
3. Camera time-sync mechanism (vendor) — then re-test with approval.
4. WSL2 clock-step trigger unknown; native Ubuntu control.
5. Formal DATA_CONTRACT approval; promote the draft to repo root.
