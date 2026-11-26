# capture — Insight 9 capture node (ROS 2 Humble / Fast DDS / rosbag2)

```
source /opt/ros/humble/setup.bash
./capture/insight-capture preflight  [--prefix /insight9] [--firmware 2.1.4] [--expect imu=200 --expect rgb=30]
./capture/insight-capture record     [--name X] [--duration S] [--include-extra] [--image-mode raw|compressed|both]
./capture/insight-capture stop
./capture/insight-capture inspect    [session|latest] [--repair] [--json]
./capture/insight-capture replay     [session|latest] [-r 1.0] [--roles imu vio] [--clock] [--remap-prefix /x] [--dry-run]
```
Exit codes (preflight/inspect): 0 ok, 1 warnings, 2 failures.

* Topic roles are matched by *suffix + message type*, so any camera prefix works; `config/topics_v2.1.4.yaml`
  is **unverified** (no real device/official list was available) — fix it first when the camera is attached.
* Read-only toward camera/system: only subscribes; never changes camera params, network, drivers. Depth is
  recorded only if it is already publishing.
* Sim (no hardware): `ROS_DOMAIN_ID=77 python3 sim/sim_insight.py [--depth]`.
* Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest capture/tests` (integration tests use a random
  isolated ROS domain).
* WSL2 note: standalone processes did not discover each other without the `ros2 daemon`; preflight starts it.

## Diagnostics & host clock (2026-11-26)
* `inspect` also writes `<session>/diagnostics.json`: loss vs reference rate, interval percentiles, **multi-topic hole detection**; a hole whose
  header stamps are contiguous is reported as "receive time jumped, data NOT lost" (WSL2 timesyncd clock steps), otherwise FAIL.
* `preflight` samples the host wall clock for `--clock-seconds` (default 35). On WSL2 bag *receive* time is unreliable (±2.5 s sawtooth): use header stamps.
* Real camera (fw 2.1.5): ROS domain **9**, prefix `/camera/camera`; run `ROS_DOMAIN_ID=9 ./insight-capture preflight --firmware 2.1.5`.
* Reports: `reports/` (1 camera, 2 WSL2 RCA, 3 tests, 4 topic changelog, 5 GO/NO-GO).
