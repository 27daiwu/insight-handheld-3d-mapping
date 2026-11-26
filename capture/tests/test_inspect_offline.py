"""inspect / manifest on synthetic rosbag2 sqlite files (no ROS runtime needed except `ros2 bag info`)."""
import json
import types

from conftest import NS, make_bag, series
from insight_capture import bagdb
from insight_capture.inspect_cmd import finalize_manifest, inspect, session_state
from insight_capture.session import write_json


def args(root, session, **kw):
    d = dict(root=str(root), session=session, profile=None, json=True, repair=False, no_tf=True)
    d.update(kw)
    return types.SimpleNamespace(**d)


def make_session(root, status, skip=(), with_depth=False, pid=None):
    sd = root / "s1"
    topics = {"/c/rgb/image_raw": ("sensor_msgs/msg/Image", series(30, 300, skip=skip)),
              "/c/imu": ("sensor_msgs/msg/Imu", series(200, 2000))}
    plan = [{"role": "rgb", "name": "/c/rgb/image_raw", "type": "sensor_msgs/msg/Image", "required": True,
             "reference_hz": 30.0},
            {"role": "imu", "name": "/c/imu", "type": "sensor_msgs/msg/Imu", "required": True, "reference_hz": 200.0}]
    if with_depth:
        plan.append({"role": "depth", "name": "/c/depth/image_raw", "type": "sensor_msgs/msg/Image", "required": False})
    make_bag(sd / "bag", topics)
    man = {"schema_version": "x", "session_id": "s1", "status": "recording", "camera": {"firmware": "2.1.4"},
           "time": {}, "topics": plan, "bag": {}, "depth": {"enabled": with_depth}, "host": {}, "disk": {}}
    write_json(sd / "manifest.json", man)
    write_json(sd / "session.json", {"status": status, "supervisor_pid": pid})
    return sd


def test_finalize_fills_manifest(tmp_path):
    sd = make_session(tmp_path, "recording")
    m = finalize_manifest(sd, "complete", "user_stop", 0, [])
    by = {t["role"]: t for t in m["topics"]}
    assert by["rgb"]["count"] == 300 and by["imu"]["count"] == 2000
    assert by["rgb"]["recv_first_ns"] and by["rgb"]["stamp_first_ns"]       # raw timestamps preserved
    assert m["time"]["duration_s"] > 9 and m["bag"]["size_bytes"] > 0 and m["bag"]["files"]


def test_interrupted_detected(tmp_path):
    sd = make_session(tmp_path, "recording", pid=2**22 + 12345)   # dead pid
    assert session_state(sd)[0] == "interrupted"


def test_inspect_clean_session(tmp_path, capsys):
    sd = make_session(tmp_path, "complete")
    finalize_manifest(sd, "complete", "user_stop", 0, [])
    rc = inspect(args(tmp_path, "s1"))
    out = json.loads(capsys.readouterr().out)
    lv = {c["id"]: c["level"] for c in out["checks"]}
    assert lv["data_rgb"] == "PASS" and lv["data_imu"] == "PASS"
    assert lv["depth"] == "WARN"                      # depth off is reported, not silent
    assert not any(c["level"] == "FAIL" for c in out["checks"]) and rc == 1


def test_inspect_reports_drops_and_count_mismatch(tmp_path, capsys):
    sd = make_session(tmp_path, "complete", skip={10, 11, 12})
    m = finalize_manifest(sd, "complete", "user_stop", 0, [])
    m["topics"][0]["count"] += 5                        # corrupt the manifest
    write_json(sd / "manifest.json", m)
    rc = inspect(args(tmp_path, "s1"))
    out = json.loads(capsys.readouterr().out)
    d = {c["id"]: c for c in out["checks"]}
    assert d["data_rgb"]["level"] == "WARN" and "3 msgs missing" in d["data_rgb"]["detail"]
    assert d["count_rgb"]["level"] == "FAIL" and rc == 2


def test_missing_required_topic_fails(tmp_path, capsys):
    sd = make_session(tmp_path, "complete")
    m = finalize_manifest(sd, "complete", "user_stop", 0, [])
    m["topics"].append({"role": "vio", "name": "/c/vio", "type": "nav_msgs/msg/Odometry", "required": True})
    write_json(sd / "manifest.json", m)
    assert inspect(args(tmp_path, "s1")) == 2


def test_truncated_db_is_still_readable(tmp_path):
    """Simulates a crash: db has data, no metadata.yaml, no manifest finalisation."""
    sd = make_session(tmp_path, "recording", pid=2**22 + 99)
    sc = bagdb.scan(sd / "bag")
    assert sc["topics"]["/c/imu"]["count"] == 2000 and not sc["errors"]
