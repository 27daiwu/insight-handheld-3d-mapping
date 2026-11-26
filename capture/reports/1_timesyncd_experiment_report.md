# 1. timesyncd control experiment report — 2026-11-26 18:08–18:26 (authorised)

## Authorisation and limits
Allowed: temporarily stop `systemd-timesyncd` for 5 min, record clocks, restore. **Not allowed / not done:** disable, mask, edit any config. Original state `active/enabled`; final state `active/enabled` (verified in the script and in the journal).
Mechanism: `sudo` needs a password and plain `systemctl stop` is refused by polkit, so only the two commands `systemctl stop|start systemd-timesyncd` were run through `wsl.exe -u root` (the Windows user's own root channel). Nothing else ran as root.

## Run 1 — INVALID for the stopped phase (my mistake, kept for the record)
`run1_INVALID_B_reactivated.json`. After `stop`, my script called `timedatectl timesync-status`; that talks to the D-Bus service `org.freedesktop.timesync1`, which **re-activates timesyncd** (journal 18:10:19.393 "Activating via systemd … requested by uid=1000", `Stopped` and `Starting` within 0.1 s). So phase B was not stopped. Lesson: never query `timedatectl` while the service is intentionally stopped; use `systemctl is-active`/`show`.

## Run 2 — valid (`timesyncd_experiment.json`, `summary.json`)
Phases: A baseline 90 s (active) → **B stopped 300 s (journal: Stopped 18:19:57, Started 18:24:57; `ActiveEnterTimestampMonotonic` unchanged during B, `is-active=inactive` at start and end of B)** → C restored 80 s. Sampling: REALTIME−MONOTONIC every 10 ms (steps > 50 ms), WSL−Windows `UtcNow` every ~0.6 s (bracketed by two local reads; median round-trip 193 ms), `adjtimex` read-only.
| phase | wall-clock steps > 50 ms | REALTIME − Windows (mean / sd) | rate vs Windows | adjtimex |
|---|---|---|---|---|
| A active (90 s) | **0** | −73.6 ms / 7.1 ms | −54 ppm (short window, noise-limited) | tick 9999, freq +76…+99 ppm |
| **B stopped (300 s)** | **0** | −73.2 ms / 4.1 ms | **+5.9 ppm** (≈ 1.8 ms drift in 5 min) | tick 9999, freq +99 ppm |
| C restored (80 s) | **0** | −72.1 ms / 2.8 ms | −15 ppm | tick 9999→10000 |
(the constant ≈ −73 ms offset is the measurement bias of the PowerShell reading). MONOTONIC and REALTIME stayed locked together in all phases.

## What this shows — and does not
1. **In this session the VM clock is healthy**: no wall-clock step in 8 min (plus no step in the 100 s + 480 s of run 1 and none during the 5-min camera recording started at 18:48), rate error ≤ ~60 ppm. The **≈ +2.4 s / 32 s sawtooth and the ≈ 8–9 % rate error (tick 9167)** measured at 16:07–16:40 are **not present** now. timesyncd status now: offset −44 µs, poll interval 17 min (then: 32 s, offset +2.4 s, packet count rising each step).
2. Stopping timesyncd for 5 min changes nothing measurable now ⇒ **this experiment cannot confirm or refute the earlier "timesyncd is the stepper" hypothesis by intervention**. The earlier evidence remains correlational: 4/4 steps coincided with timesyncd polls and their size equalled timesyncd's reported offset (reports/2_wsl2_packet_loss_rca.md, E3–E4).
3. Why the bad state existed at 16:07–16:40 (and probably 12:5x–14:4x) but not at 18:08+ is **undetermined**. Candidates (untested): the VM clock was far from NTP right after the 15:52 restart/resume and timesyncd was converging with large corrections; Windows host sleep/resume or time-service events; network path to NTP (the journal at 16:00 shows repeated NTP timeouts, even `169.254.10.1:123` via DNS for ntp.ubuntu.com). A reproduction would need to perturb the clock (not authorised).
4. Practical consequence stands: bag receive time on WSL2 can be wrong by seconds; header stamps are the reference. `preflight --clock-seconds` detects the bad state at record time (it reported 2 steps at 16:35 and none now).
5. "WSL2 clock rate deviation" (task statement): ≈ −9 % earlier (tick 9167, measured vs Windows over 19 s), **≈ 0 ± 60 ppm now**; cause of the earlier state unknown.

## Files
`timesyncd_experiment/timesyncd_experiment.py` (script, with restore-on-any-exit), `.json` (raw samples), `summary.json`, `run1_INVALID_B_reactivated.json`, `run1_script.py.txt`, `exp.log`.
