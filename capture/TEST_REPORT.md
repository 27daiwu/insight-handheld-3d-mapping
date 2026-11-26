# Capture test report (2026-11-26, host: WSL2 Ubuntu 22.04, ROS 2 Humble, Fast DDS, NO camera attached)

## Automated: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest capture/tests` -> 26 passed
unit: discovery (arbitrary prefix, foreign device rejected, depth on/off, raw/compressed), stats (drops,
non-monotonic, duplicates, clock offset, low rate), inspect/manifest on synthetic rosbag2 sqlite.
integration (simulator, isolated random domain): no-camera refusal, record->stop->inspect->replay (replayed IMU
count verified), depth recorded only when enabled, SIGKILL crash -> inspect -> `--repair` -> replay, low-disk refusal.

## NOT passed: 5-minute acceptance recording (simulator, 322 s, 939 MB, rgb/stereo/imu/vio/info/tf)
`inspect` reported ~2.2-2.5 s holes in ALL topics every ~34.5 s (~520 missing frames/topic, ~2090 IMU samples).
Isolation experiments (simulator alone, passive probe, no recorder):
* simulator's own timers did not stall (no stall > 0.5 s logged);
* simulator published 6419 IMU msgs in ~65 s, subscriber received 6002 -> loss is between publisher and subscriber;
* reproduced with default Fast DDS transport and with UDP-only profile, and with ROS_LOCALHOST_ONLY=1.
Root cause NOT identified (suspect WSL2 loopback/DDS transport). Not fixed: candidate mitigations (sysctl socket
buffers, Fast DDS profile) change system config and need your approval. The result says nothing about the real camera.
Also: fresh-domain discovery on this host only works when the `ros2 daemon` for that domain is running (preflight starts it).
