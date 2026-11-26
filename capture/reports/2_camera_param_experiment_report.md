# 2. Camera parameter control experiment — 2026-11-26 18:30–18:50 (authorised)

## Method and safety
* **Official configuration path:** the camera's own web UI (http://169.254.10.1, the vendor dashboard) talks to a REST API. I did **not** guess commands; every request body below was copied from the UI's JavaScript bundle (`pages-settings-camera`, `pages-looper-control`, `pages-settings-time-sync`; also matches the vendor bring-up note for fw 2.1.2 `POST /api/deep-flow {"enabled":…}`):
  | variable | request |
  |---|---|
  | Depth | `POST /api/deep-flow {"enabled":false}` (no reboot; UI text "Deep Flow updated") |
  | stereo fps | `POST /api/stereo-format {"format":"mono8 544x640 20fps","hdr":false}` (UI forces `hdr=false` for non-30 fps ⇒ **fps and HDR change together, cannot be separated**; response "Software will restart…") |
  | time sync | `POST /api/time-sync-setting {"enabled":true}` |
* **Rollback:** the same endpoints with the original values from the backup. The vendor UI offers no other rollback (no factory-restore was touched; `/api/reboot`, `/api/insight-stop` etc. were never called).
* **Backup before any change:** `camera_param_experiment/backup_original_20261126_181847.json` = GET of 17 config endpoints (API-visible configuration; internal files are not accessible — ssh credentials unknown, not guessed).
  Original: colour `mjpg 1088x1920 30fps`; stereo `mono8 544x640 30fps` HDR **on**; Depth **on**; time-sync **off**; pose-cov on; domain 9; dds cyclonedds.
* One variable per scenario, applied → read back → measured → restored → read back → re-measured. A `finally` block restores everything from the backup on any failure.
* **Final state check: `restored_ok = True` — all values equal the backup** (`camera_param_experiment.json`, `final_core`). Camera software restarted twice (stereo apply/restore) and recovered with the same topics.
* Measurement: 90 s passive rclpy subscribers (BEST_EFFORT depth 200) on all 9 data topics; rates from header stamps (n−1)/(last−first); loss vs nominal; host WSL clock was healthy (report 1).

## Results (header-stamp space)
| scenario | depth Hz (miss vs 15) | stereo L/R Hz (miss) | colour Hz | IMU Hz (miss) | VIO 100 Hz (miss) | camera CPU / temp |
|---|---|---|---|---|---|---|
| S0 baseline (30 fps, HDR, depth on) | **13.04 (13.1 %)** | 29.97/29.96 (0.11/0.15 %) | 30.00 | 399.92 (0.02 %) | 99.93 (0.07 %) | 44 % / 59 °C |
| S1 **depth off** | – (no messages) | **30.00/30.00 (0 %, 0 gaps)** | 30.02 | **400.00 (0 %, 0 gaps)** | **99.99 (0.01 %)** | **21 %** / 56 °C |
| S1r depth restored (45 s) | 13.08 (12.8 %) | 30.00/30.00 (0) | 30.02 | 400.00 (0) | 99.93 (0.07 %, 23 gaps) | 35 % / 58 °C |
| S2 **stereo 20 fps, hdr off** | **12.77 (14.9 %)** | **20.00/19.99 (0/0.06 %)** | 30.01 | 400.00 (0) | 99.98 (0.02 %) | 24 % / 58 °C |
| S2r stereo restored 30 fps+HDR | 13.17 (12.2 %) | 30.00/30.00 (0) | 30.02 | 400.00 (0) | 99.89 (0.11 %) | 37 % / 58 °C |
| S3 **time-sync on** (90 s after +20 s settle) | 13.10 (12.6 %) | 30.00/30.00 (0) | 30.02 | 400.00 (0) | 99.92 (0.08 %) | 35 % / 59 °C |
`vio_image` follows the stereo rate (20 Hz in S2) — expected, not a loss.

## Findings
1. **Depth rate is insensitive to the stereo frame rate (30→20 fps: 13.04→12.77 Hz), to time-sync, to subscriber QoS/count and to bandwidth** (reports/3_depth_loss_root_cause.md). The vendor's hint "lower the stereo fps if depth rate matters" (bring-up note, fw 2.1.2) did **not** help on 2.1.5.
2. **Turning depth off removes all residual gaps in the other streams** (IMU 0 gaps vs 2, stereo 0 vs 3, VIO 3 vs 19 in 90 s) and halves camera CPU (44 %→21 %). Depth therefore costs resources in the camera (or USB egress) and slightly disturbs IMU/VIO/stereo continuity. This is an association from 3 interleaved measurements, ~90 s each, not a statistical proof.
3. **Time sync toggle had no observable effect**: `time-sync-setting` reports `enabled=true, synced=false` throughout; header stamps still boot-relative (stamp0 ≈ 2222 s); camera wall clock still 2025-08-26. The sync source/protocol is unknown (I did not guess an NTP/PTP server setup). Not a usable path to host-aligned stamps yet.
4. 20 fps stereo is clean (0.06 % loss) and cuts the stereo data volume by 1/3; colour unaffected by all changes.
5. HDR was not isolated (tied to 30 fps by the UI); no claim about HDR's effect.

## Not done
No change of colour format; no VIO/pose-cov change; no repeated trials (each cell is one 45–90 s run); no native-Ubuntu repeat.
Files: `camera_param_experiment/` (`camera_cfg.py` helper, `camera_param_experiment.py`, `.json` raw results, `backup_original_*.json`, `exp.log`).
