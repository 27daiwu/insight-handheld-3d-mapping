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
