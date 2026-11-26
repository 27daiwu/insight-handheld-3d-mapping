"""rclpy helpers: graph snapshot and passive topic probing (subscribe only)."""
from __future__ import annotations

import time

from .discovery import TopicInfo
from .stats import analyze, parse_frame_id, parse_header_stamp


def _rclpy():
    import rclpy
    return rclpy


def _qos_str(p) -> str:
    try:
        return f"{p.reliability.name.lower()}/{p.durability.name.lower()}"
    except Exception:
        return ""


class Graph:
    """Context manager owning an rclpy context + node. Never publishes."""

    def __init__(self, name: str = "insight_capture_probe"):
        self.name = name

    def __enter__(self):
        rclpy = _rclpy()
        import rclpy.context
        self.ctx = rclpy.context.Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node(self.name, context=self.ctx, enable_rosout=False,
                                      start_parameter_services=False)
        from rclpy.executors import SingleThreadedExecutor
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        return self

    def __exit__(self, *a):
        try:
            self.ex.shutdown()
            self.node.destroy_node()
        finally:
            self.ctx.try_shutdown()

    def spin(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.05)

    def snapshot(self, settle: float = 2.0) -> list[TopicInfo]:
        self.spin(settle)
        out = []
        for name, types in self.node.get_topic_names_and_types():
            try:
                pubs = self.node.get_publishers_info_by_topic(name)
            except Exception:
                pubs = []
            qos = ",".join(sorted({_qos_str(p.qos_profile) for p in pubs})) if pubs else ""
            for t in types:
                out.append(TopicInfo(name, t, len(pubs), qos))
        return out

    def node_names(self) -> list[str]:
        return [f"{ns.rstrip('/')}/{n}" for n, ns in self.node.get_node_names_and_namespaces()]

    def probe(self, topics: dict[str, str], seconds: float, gap_factor: float = 1.5,
              nominal: dict[str, float] | None = None) -> dict:
        """Subscribe to {topic: type} for `seconds`; return per-topic stats + first-message info."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import deserialize_message

        rec: dict[str, dict] = {}
        subs = []
        for topic, typ in topics.items():
            static = topic.endswith("tf_static")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=200 if not static else 100,
                reliability=ReliabilityPolicy.RELIABLE if static else ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if static else DurabilityPolicy.VOLATILE)
            r = {"type": typ, "stamps": [], "recv": [], "bytes": 0, "first": None, "edges": set()}
            rec[topic] = r
            cls = get_message(typ)

            def cb(data, r=r, cls=cls, typ=typ):
                now = time.time_ns()
                r["recv"].append(now)
                r["bytes"] += len(data)
                if typ == "tf2_msgs/msg/TFMessage":
                    r["stamps"].append(None)
                    for t in deserialize_message(data, cls).transforms:
                        r["edges"].add((t.header.frame_id.strip("/"), t.child_frame_id.strip("/")))
                    return
                r["stamps"].append(parse_header_stamp(data))
                if r["first"] is None:
                    r["first"] = _first_info(data, cls, typ, deserialize_message)

            subs.append(self.node.create_subscription(cls, topic, cb, qos, raw=True))
        self.spin(seconds)
        for s in subs:
            self.node.destroy_subscription(s)
        result = {}
        for topic, r in rec.items():
            st = analyze(r["stamps"], r["recv"], gap_factor, (nominal or {}).get(topic))
            st["bytes"] = r["bytes"]
            span = st.get("recv_span_s") or seconds
            st["bytes_per_s"] = int(r["bytes"] / span) if span else 0
            st["type"] = r["type"]
            st["first"] = r["first"]
            st["edges"] = sorted(r["edges"])
            result[topic] = st
        return result


def _first_info(data, cls, typ, deserialize_message) -> dict:
    info: dict = {"frame_id": parse_frame_id(data)}
    try:
        if typ == "sensor_msgs/msg/CameraInfo":
            m = deserialize_message(data, cls)
            info.update(width=m.width, height=m.height, k=list(m.k), distortion_model=m.distortion_model,
                        calibrated=any(abs(x) > 0 for x in m.k))
        elif typ == "sensor_msgs/msg/Image":
            m = deserialize_message(data, cls)
            info.update(width=m.width, height=m.height, encoding=m.encoding)
        elif typ == "sensor_msgs/msg/CompressedImage":
            m = deserialize_message(data, cls)
            info.update(format=m.format)
        elif typ in ("nav_msgs/msg/Odometry",):
            m = deserialize_message(data, cls)
            info.update(child_frame_id=m.child_frame_id)
    except Exception as e:  # informational only
        info["decode_error"] = str(e)
    return info
