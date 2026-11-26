import copy
import json

import pytest

from insight_mapping import session as sess

BASE = {
    "schema_version": "1.0.0-draft", "session_id": "session_t",
    "recording": {"state": "finalized", "start_time_utc": "2026-01-01T00:00:00Z", "end_time_utc": "2026-01-01T00:01:00Z",
                  "duration_s": 60, "ros_start_stamp_ns": 1, "ros_end_stamp_ns": 2},
    "environment": {"camera": {"model": "Insight 9", "firmware_version": "x", "config": {}},
                    "ros": {"distro": "humble", "rmw_implementation": "rmw_fastrtps_cpp"}, "dds": {"vendor": "fastdds"},
                    "host": {"hostname": "h", "os": "o", "recorder_version": "v"}},
    "storage": {"format": "mcap", "bag_path": "bag"},
    "streams": [{"topic": "/a/image", "role": "image", "msg_type": "sensor_msgs/msg/Image", "message_count": 1,
                 "measured_rate_hz": 10.0, "width": 2, "height": 2, "encoding": "mono8"}],
    "calibration": {"files": []},
    "integrity": {"status": "complete", "total_messages": 1, "dropped_messages": 0},
}


def test_valid_manifest_passes():
    assert sess.validate_manifest(BASE) == []


def test_sqlite_requires_fallback_reason():
    m = copy.deepcopy(BASE); m["storage"]["format"] = "sqlite3"
    assert sess.validate_manifest(m)
    m["storage"]["fallback_reason"] = "rosbag2_storage_mcap not installed"
    assert sess.validate_manifest(m) == []


def test_image_stream_requires_resolution_and_encoding():
    m = copy.deepcopy(BASE); del m["streams"][0]["encoding"]
    assert sess.validate_manifest(m)


def test_unknown_top_level_field_rejected():
    m = copy.deepcopy(BASE); m["my_private_format"] = {}
    assert sess.validate_manifest(m)


def test_unfinalized_session_is_flagged(tmp_path):
    m = copy.deepcopy(BASE); m["recording"] = {"state": "recording", "start_time_utc": "2026-01-01T00:00:00Z"}
    d = tmp_path / "session_t"; (d / "bag").mkdir(parents=True); (d / "logs").mkdir(); (d / "calibration").mkdir()
    (d / "manifest.json").write_text(json.dumps(m))
    s = sess.Session.open(d)
    assert any("finalized" in msg for sev, msg in s.problems if sev == "error")
    with pytest.raises(sess.SessionError):
        sess.Session.open(d, strict=True)


def test_missing_manifest(tmp_path):
    with pytest.raises(sess.SessionError):
        sess.Session.open(tmp_path)
