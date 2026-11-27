"""Test/fixture helper: write a contract-conformant Session (manifest.json + rosbag2 + dirs).

This is a *mapping-side test fixture generator* so the pipeline can be exercised without a camera.
It implements the layout of docs/DATA_CONTRACT.md but is NOT the capture implementation.
"""
from __future__ import annotations

import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .. import se3


def _ros():
    import rosbag2_py
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from tf2_msgs.msg import TFMessage
    return locals()


def to_time(ns: int):
    from builtin_interfaces.msg import Time
    return Time(sec=int(ns // 10**9), nanosec=int(ns % 10**9))


class SessionBuilder:
    def __init__(self, root, session_id, storage="sqlite3"):
        self.root = Path(root) / session_id
        self.session_id = session_id
        if self.root.exists():
            shutil.rmtree(self.root)
        for d in ("logs", "calibration"):
            (self.root / d).mkdir(parents=True)
        self.R = _ros()
        rb = self.R["rosbag2_py"]
        self.w = rb.SequentialWriter()
        self.w.open(rb.StorageOptions(uri=str(self.root / "bag"), storage_id=storage), rb.ConverterOptions("", ""))
        self.storage = storage
        self.topics: dict[str, dict] = {}
        self.stamps: dict[str, list] = {}

    def _topic(self, name, typ, role, frame_id="", extra=None):
        if name not in self.topics:
            rb = self.R["rosbag2_py"]
            self.w.create_topic(rb.TopicMetadata(name=name, type=typ, serialization_format="cdr"))
            self.topics[name] = {"topic": name, "role": role, "msg_type": typ, "frame_id": frame_id, **(extra or {})}
            self.stamps[name] = []

    def _write(self, topic, msg, stamp_ns, bag_ns=None):
        self.w.write(topic, self.R["serialize_message"](msg), int(stamp_ns if bag_ns is None else bag_ns))
        self.stamps[topic].append(int(stamp_ns))

    def add_image(self, topic, stamp_ns, arr, encoding, frame_id, role="image", bag_ns=None):
        Image = self.R["Image"]
        self._topic(topic, "sensor_msgs/msg/Image", role, frame_id,
                    {"width": int(arr.shape[1]), "height": int(arr.shape[0]), "encoding": encoding})
        m = Image()
        m.header.stamp = to_time(stamp_ns); m.header.frame_id = frame_id
        m.height, m.width, m.encoding = int(arr.shape[0]), int(arr.shape[1]), encoding
        m.is_bigendian = 0
        m.step = int(arr.strides[0])
        m.data = np.ascontiguousarray(arr).tobytes()
        self._write(topic, m, stamp_ns, bag_ns)

    def add_compressed(self, topic, stamp_ns, data: bytes, fmt, frame_id, role="image", bag_ns=None):
        CompressedImage = __import__("sensor_msgs.msg", fromlist=["CompressedImage"]).CompressedImage
        self._topic(topic, "sensor_msgs/msg/CompressedImage", role, frame_id)
        m = CompressedImage()
        m.header.stamp = to_time(stamp_ns); m.header.frame_id = frame_id
        m.format = fmt; m.data = list(data) if False else data
        self._write(topic, m, stamp_ns, bag_ns)

    def add_pose(self, topic, stamp_ns, T, frame_id, role="vio", bag_ns=None):
        PoseStamped = __import__("geometry_msgs.msg", fromlist=["PoseStamped"]).PoseStamped
        self._topic(topic, "geometry_msgs/msg/PoseStamped", role, frame_id)
        m = PoseStamped()
        m.header.stamp = to_time(stamp_ns); m.header.frame_id = frame_id
        t, q = se3.T_to_tq(T)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, t)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)
        self._write(topic, m, stamp_ns, bag_ns)

    def add_camera_info(self, topic, stamp_ns, cam, bag_ns=None):
        CameraInfo = self.R["CameraInfo"]
        self._topic(topic, "sensor_msgs/msg/CameraInfo", "camera_info", cam.frame_id)
        m = CameraInfo()
        m.header.stamp = to_time(stamp_ns); m.header.frame_id = cam.frame_id
        m.width, m.height = cam.width, cam.height
        m.distortion_model = cam.distortion_model
        m.d = list(map(float, cam.D)); m.k = list(map(float, cam.K)); m.r = list(map(float, cam.R)); m.p = list(map(float, cam.P))
        self._write(topic, m, stamp_ns, bag_ns)

    def add_odom(self, topic, stamp_ns, T, parent, child, role="vio", v=None, bag_ns=None):
        Odometry = self.R["Odometry"]
        self._topic(topic, "nav_msgs/msg/Odometry", role, parent)
        m = Odometry()
        m.header.stamp = to_time(stamp_ns); m.header.frame_id = parent; m.child_frame_id = child
        t, q = se3.T_to_tq(T)
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, t)
        m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = map(float, q)
        if v is not None:
            m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = map(float, v)
        self._write(topic, m, stamp_ns, bag_ns)

    def add_imu(self, topic, stamp_ns, gyro, acc, frame_id, bag_ns=None):
        Imu = self.R["Imu"]
        self._topic(topic, "sensor_msgs/msg/Imu", "imu", frame_id)
        m = Imu()
        m.header.stamp = to_time(stamp_ns); m.header.frame_id = frame_id
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, gyro)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, acc)
        m.orientation.w = 1.0
        self._write(topic, m, stamp_ns, bag_ns)

    def add_tf(self, edges, stamp_ns, static=False, bag_ns=None):
        """edges: list of (parent, child, T_parent_child)."""
        TFMessage, TS = self.R["TFMessage"], self.R["TransformStamped"]
        name = "/tf_static" if static else "/tf"
        self._topic(name, "tf2_msgs/msg/TFMessage", "tf_static" if static else "tf")
        msg = TFMessage()
        for p, c, T in edges:
            ts = TS()
            ts.header.stamp = to_time(stamp_ns); ts.header.frame_id = p; ts.child_frame_id = c
            t, q = se3.T_to_tq(T)
            ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = map(float, t)
            ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = map(float, q)
            msg.transforms.append(ts)
        self._write(name, msg, stamp_ns, bag_ns)

    def finalize(self, label="", camera_config=None, extensions=None, integrity_status=None,
                 missing=(), warnings=(), nominal_rates=None):
        del self.w  # flush/close
        streams = []
        total = 0
        for name, t in self.topics.items():
            s = np.array(sorted(self.stamps[name]), dtype=np.int64)
            rate = float((len(s) - 1) / ((s[-1] - s[0]) / 1e9)) if len(s) > 1 and s[-1] > s[0] else 0.0
            rec = dict(t, message_count=len(s), measured_rate_hz=rate,
                       first_stamp_ns=int(s[0]), last_stamp_ns=int(s[-1]), dropped_messages=0,
                       drop_estimation_method="synthetic: none dropped")
            if nominal_rates and name in nominal_rates:
                rec["nominal_rate_hz"] = nominal_rates[name]
            streams.append(rec)
            total += len(s)
        starts = [r["first_stamp_ns"] for r in streams]; ends = [r["last_stamp_ns"] for r in streams]
        files = {}
        for p in sorted((self.root / "bag").rglob("*")):
            if p.is_file():
                files[str(p.relative_to(self.root))] = hashlib.sha256(p.read_bytes()).hexdigest()
        t0 = datetime.now(timezone.utc)
        status = integrity_status or ("incomplete" if missing else "complete")
        manifest = {
            "schema_version": "1.0.0-draft", "session_id": self.session_id,
            **({"label": label} if label else {}),
            "recording": {"state": "finalized", "start_time_utc": t0.isoformat().replace("+00:00", "Z"),
                          "end_time_utc": t0.isoformat().replace("+00:00", "Z"),
                          "duration_s": (max(ends) - min(starts)) / 1e9,
                          "ros_start_stamp_ns": min(starts), "ros_end_stamp_ns": max(ends)},
            "environment": {
                "camera": {"model": "Insight 9", "firmware_version": "n/a (synthetic/public fixture)",
                           "config": camera_config or {}},
                "ros": {"distro": "humble", "rmw_implementation": "rmw_fastrtps_cpp"},
                "dds": {"vendor": "fastdds"},
                "host": {"hostname": "fixture", "os": "n/a", "recorder_version": "insight_mapping.fixtures"}},
            "storage": {"format": self.storage, "bag_path": "bag",
                        **({"fallback_reason": "rosbag2_storage_mcap not installed on this host"} if self.storage == "sqlite3" else {})},
            "streams": streams,
            "calibration": {"files": []},
            "integrity": {"status": status, "total_messages": total, "dropped_messages": 0,
                          "missing_required_streams": list(missing), "file_checksums": files, "warnings": list(warnings)},
            "extensions": {"fixture": True, **(extensions or {})},
        }
        (self.root / "manifest.json").write_text(json.dumps(manifest, indent=1))
        return self.root
