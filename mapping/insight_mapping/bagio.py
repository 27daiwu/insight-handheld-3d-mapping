"""rosbag2 reading (MCAP or SQLite3) and ROS message -> numpy conversion.

Needs a sourced ROS 2 Humble environment (rosbag2_py, rclpy). Only the mapping side reads bags.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Optional

import numpy as np
import yaml


class BagError(RuntimeError):
    pass


def _import_ros():
    try:
        import rosbag2_py  # noqa
        from rclpy.serialization import deserialize_message  # noqa
        from rosidl_runtime_py.utilities import get_message  # noqa
        return rosbag2_py, deserialize_message, get_message
    except ImportError as e:  # pragma: no cover
        raise BagError("ROS 2 Humble python env not available; run `source /opt/ros/humble/setup.bash` "
                       f"before using the mapping tools ({e})") from e


def read_bag_metadata(bag_dir: Path) -> dict:
    meta = bag_dir / "metadata.yaml"
    if not meta.exists():
        raise BagError(f"{meta} missing: not a rosbag2 directory")
    return yaml.safe_load(meta.read_text())["rosbag2_bagfile_information"]


def storage_plugin_available(storage_id: str) -> bool:
    if storage_id == "sqlite3":
        return True
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix("rosbag2_storage_mcap" if storage_id == "mcap" else f"rosbag2_storage_{storage_id}")
        return True
    except Exception:
        return False


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def fast_header(raw: bytes):
    """Parse (stamp_ns, frame_id) straight from CDR bytes of a message whose first field is std_msgs/Header.
    Avoids deserialising large images when only indexing. Returns None for big-endian/odd payloads."""
    if len(raw) < 16 or raw[1] != 1:  # little-endian CDR only
        return None
    sec, nsec, n = struct.unpack_from("<iII", raw, 4)
    if n > 256 or 16 + n > len(raw):
        return None
    return sec * 1_000_000_000 + nsec, raw[16:16 + max(n - 1, 0)].decode("ascii", "replace")


@dataclass
class RawMsg:
    topic: str
    bag_ns: int
    data: bytes
    msg_type: str


class BagReader:
    def __init__(self, bag_dir):
        self.bag_dir = Path(bag_dir)
        self.meta = read_bag_metadata(self.bag_dir)
        self.storage_id = self.meta.get("storage_identifier", "sqlite3")
        if not storage_plugin_available(self.storage_id):
            raise BagError(f"bag uses storage '{self.storage_id}' but the rosbag2 plugin is not installed "
                           "(MCAP: apt install ros-humble-rosbag2-storage-mcap)")
        self._ros = _import_ros()
        self._types: dict[str, str] = {}
        self._classes: dict[str, type] = {}

    def _open(self, topics: Optional[Iterable[str]]):
        rb2 = self._ros[0]
        r = rb2.SequentialReader()
        r.open(rb2.StorageOptions(uri=str(self.bag_dir), storage_id=self.storage_id),
               rb2.ConverterOptions("", ""))
        self._types = {t.name: t.type for t in r.get_all_topics_and_types()}
        if topics is not None:
            r.set_filter(rb2.StorageFilter(topics=list(topics)))
        return r

    def topics(self) -> dict:
        self._open(None)
        return dict(self._types)

    def message_counts(self) -> dict:
        return {t["topic_metadata"]["name"]: int(t["message_count"])
                for t in self.meta.get("topics_with_message_count", [])}

    def iter_raw(self, topics: Optional[Iterable[str]] = None) -> Iterator[RawMsg]:
        r = self._open(topics)
        while r.has_next():
            topic, data, ns = r.read_next()
            yield RawMsg(topic, int(ns), data, self._types[topic])

    def deserialize(self, raw: RawMsg):
        cls = self._classes.get(raw.msg_type)
        if cls is None:
            cls = self._classes[raw.msg_type] = self._ros[2](raw.msg_type)
        return self._ros[1](raw.data, cls)

    def iter_msgs(self, topics: Optional[Iterable[str]] = None):
        for raw in self.iter_raw(topics):
            yield raw, self.deserialize(raw)


# ---------------------------------------------------------------- message converters
_DTYPES = {"mono8": (np.uint8, 1), "8UC1": (np.uint8, 1), "bgr8": (np.uint8, 3), "rgb8": (np.uint8, 3),
           "bgra8": (np.uint8, 4), "rgba8": (np.uint8, 4), "mono16": (np.uint16, 1), "16UC1": (np.uint16, 1),
           "32FC1": (np.float32, 1), "8UC3": (np.uint8, 3)}


def image_to_numpy(msg) -> np.ndarray:
    """sensor_msgs/Image -> ndarray without changing channel order (use `to_bgr`/`to_gray` explicitly)."""
    enc = msg.encoding
    if enc not in _DTYPES:
        raise BagError(f"unsupported image encoding '{enc}'")
    dt, ch = _DTYPES[enc]
    itemsize = np.dtype(dt).itemsize
    row = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    arr = row[:, : msg.width * ch * itemsize].copy().view(dt)
    if msg.is_bigendian and itemsize > 1:
        arr = arr.byteswap()
    return arr.reshape(msg.height, msg.width, ch)[..., 0] if ch == 1 else arr.reshape(msg.height, msg.width, ch)


def compressed_to_numpy(msg) -> np.ndarray:
    import cv2
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if "compressedDepth" in msg.format:
        data = data[12:]  # image_transport compressed_depth config header
    img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise BagError(f"cannot decode CompressedImage format '{msg.format}'")
    return img


def to_bgr(arr: np.ndarray, encoding: str) -> np.ndarray:
    import cv2
    if arr.ndim == 2:
        return cv2.cvtColor(arr if arr.dtype == np.uint8 else (arr >> 8).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    if encoding in ("rgb8",):
        return arr[..., ::-1].copy()
    if encoding in ("rgba8",):
        return arr[..., 2::-1].copy()
    if encoding in ("bgra8",):
        return arr[..., :3].copy()
    return arr


def to_gray(arr: np.ndarray, encoding: str) -> np.ndarray:
    import cv2
    if arr.ndim == 2:
        return arr if arr.dtype == np.uint8 else (arr >> 8).astype(np.uint8)
    return cv2.cvtColor(to_bgr(arr, encoding), cv2.COLOR_BGR2GRAY)


def depth_to_mm_u16(arr: np.ndarray, encoding: str) -> np.ndarray:
    """Depth -> uint16 millimetres (0 = invalid). 16UC1 is assumed mm (REP-118); 32FC1 metres."""
    if arr.dtype == np.uint16:
        return arr
    d = np.nan_to_num(arr.astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
    d[(d < 0) | (d > 65535)] = 0
    return np.round(d).astype(np.uint16)


def odometry_to_row(msg):
    """-> (stamp_ns, parent_frame, child_frame, t[3], q[4 xyzw], twist_lin[3], twist_ang[3], cov_diag[6])."""
    p, o = msg.pose.pose.position, msg.pose.pose.orientation
    q = np.array([o.x, o.y, o.z, o.w], dtype=float)
    n = np.linalg.norm(q)
    q = q / n if n > 0 else np.array([0, 0, 0, 1.0])
    tw = msg.twist.twist
    cov = np.array(msg.pose.covariance, dtype=float).reshape(6, 6)
    return (stamp_to_ns(msg.header.stamp), msg.header.frame_id, msg.child_frame_id,
            np.array([p.x, p.y, p.z]), q, np.array([tw.linear.x, tw.linear.y, tw.linear.z]),
            np.array([tw.angular.x, tw.angular.y, tw.angular.z]), np.diag(cov))


def imu_to_row(msg):
    return (stamp_to_ns(msg.header.stamp), msg.header.frame_id,
            np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]),
            np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]),
            np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
