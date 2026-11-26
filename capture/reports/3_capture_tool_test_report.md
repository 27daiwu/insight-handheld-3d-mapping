# 3. Capture tool test report — 2026-11-26

## Automated: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest capture/tests` → **42 passed, run twice (134 s each)**
The original **26 tests are unchanged and pass**; 16 added. (Tests run on synthetic data / the simulator on an isolated random ROS domain; they are **software tests, not camera acceptance**.)
| file | tests | covers |
|---|---|---|
| test_discovery.py | 8 → 12 | any prefix, foreign device rejected, depth on/off, raw/compressed, extras, **vendor-doc layout (docs ≠ device, labelled)**, **real fw2.1.5 topic list** (prefers `*_rect_raw`, /tf optional) |
| test_stats.py | 7 | drops, non-monotonic, duplicates, clock offset, rate vs reference |
| test_inspect_offline.py | 6 | manifest finalise, crash detection, count mismatch, missing required topic |
| test_diagnostics.py (new) | 13 | gap/loss vs median **and vs reference rate**, uniformly slow stream, single-topic vs systemic holes, periodic hole (34.5 s pattern), **host-clock-step ⇒ "recv-only, data not lost"**, real loss still FAIL, un-stamped /tf, tiny gaps not chained into fake holes (regression from a real-data bug), L/R & RGB sync skew, bag-level inspect |
| test_integration.py | 5 | no-camera refusal, record→stop→inspect→replay (message count checked), depth only when publishing, SIGKILL→inspect→repair→replay, low-disk refusal |

New tool features: `diagnostics.py` (continuity, interval p50/p99/max, jitter, loss vs median and vs reference, systemic multi-topic holes with periodicity, recv-only vs stamp-gap classification, pair skew), `inspect` writes `diagnostics.json`, `preflight --clock-seconds` host-clock check, discovery rank (`*rect*` first) + sibling-prefix/any_prefix roles, `sim --stamp-clock monotonic`.
Test-hardening found while working: DDS discovery on a fresh domain lags on this WSL2 host (preflight re-poll 4×3 s; tests start the daemon first). One test failed 3× before that fix (order-dependent), 0× after.

## Real-device results (WSL2, Insight 9 fw 2.1.5, domain 9; NOT simulator)
| item | result |
|---|---|
| Preflight (36 s probe) | camera found, core topics PASS, depth publishing, **WARN host wall clock stepped 2×**, WARN TF `world` frame unattached, WARN depth rate |
| Real recording | 340.6 s, 12 topics, 12 GB, 6 bag files, finalised `complete/duration_reached`, manifest firmware 2.1.5, depth enabled=true |
| Rates (header stamps) | RGB 30.019, stereo 30.000/30.000 (skew 0 ms), IMU 400.0, VIO 100.0, depth 15.0 Hz; CameraInfo 30 Hz |
| Loss (header stamps) | RGB/stereo/info **0**; IMU 0.12 %; VIO 0.29 %; **Depth 12.65 %** |
| Systemic receive-time holes | 9, **all recv-only; 0 data-loss events** (cause: host clock steps, report 2) |
| Replay (isolated domain 112, 4×, offset 10 s, imu+stereo+depth+vio) | exit 0; stamp rates reproduced exactly |
| Timestamps | monotonic, no duplicates/zeros; **camera stamps boot-relative (~1339 s), camera wall clock ~457 days off, time-sync disabled** |

## Acceptance matrix P1-C (honest)
| criterion | status |
|---|---|
| camera connected | **YES** (WSL2; native Ubuntu not tried) |
| core topics publish | **YES** (rgb is JPEG only; `/tf` absent, `/tf_static` present) |
| frequency matches config | **YES** (all header-stamp rates = nominal) |
| ≥ 5 min recording | **YES** (340 s real) |
| rosbag2 replays | **YES** (partial-topic replay verified; counts/rates match) |
| manifest complete | **YES** for recorded fields; firmware is user-supplied (no ROS API) |
| no obvious timestamp anomaly | **PARTIAL**: header stamps clean; receive-time anomalies explained (host clock) but unfixed |
| no unexplained data loss | **NO**: Depth 12.65 %, IMU 0.12 %, VIO 0.29 % unexplained |
| native Ubuntu run | **NOT DONE** |
Simulator results are not used in this matrix.
