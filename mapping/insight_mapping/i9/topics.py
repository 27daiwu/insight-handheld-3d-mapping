"""fw 2.1.5 topic specification, copied from capture/DATA_CONTRACT.md v0.3 §5.2 (measured names, NOT the v1.2.0 /insight/* names).

Only facts the contract marks as measured are encoded as expectations. Fields the contract marks [?] (depth unit, VIO
semantics, IMU axes, rectified-ness) are *recorded and reported*, never assumed.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

P = "/camera/camera"


@dataclass(frozen=True)
class Spec:
    role: str
    topic: str
    msg_type: str
    nominal_hz: Optional[float]
    size_wh: Optional[tuple] = None      # (width, height) as in the ROS message
    encoding: Optional[str] = None
    used_in_phase1: bool = False         # geometric input of the stereo pipeline
    note: str = ""


SPECS = [
    Spec("stereo_left", f"{P}/infra1/image_rect_raw", "sensor_msgs/msg/Image", 30, (544, 640), "mono8", True),
    Spec("stereo_right", f"{P}/infra2/image_rect_raw", "sensor_msgs/msg/Image", 30, (544, 640), "mono8", True),
    Spec("stereo_left_info", f"{P}/infra1/camera_info", "sensor_msgs/msg/CameraInfo", 30, (544, 640), None, True),
    Spec("stereo_right_info", f"{P}/infra2/camera_info", "sensor_msgs/msg/CameraInfo", 30, (544, 640), None, True),
    Spec("tf_static", "/tf_static", "tf2_msgs/msg/TFMessage", None, None, None, True),
    Spec("rgb", f"{P}/color/image_rect_raw/compressed", "sensor_msgs/msg/CompressedImage", 30, (1088, 1920), "jpeg"),
    Spec("rgb_info", f"{P}/color/camera_info", "sensor_msgs/msg/CameraInfo", 30, (1088, 1920), None),
    Spec("depth", f"{P}/depth/image_rect_raw", "sensor_msgs/msg/Image", 15, (544, 640), "mono16",
         note="contract: 15 Hz nominal [O], ~13.1 Hz delivered, unit [?]"),
    Spec("imu", f"{P}/imu", "sensor_msgs/msg/Imu", 400, note="axes / gyro unit [?]"),
    Spec("vio", f"{P}/vio_100hz", "geometry_msgs/msg/PoseStamped", 100, note="pose semantics [?]"),
    Spec("vio_image", f"{P}/vio_image", "geometry_msgs/msg/PoseStamped", 30),
    Spec("vio_image_cov", f"{P}/vio_image_cov", "geometry_msgs/msg/PoseWithCovarianceStamped", 30),
    Spec("vio_status", f"{P}/vio_status", "std_msgs/msg/String", 1),
]
BY_ROLE = {s.role: s for s in SPECS}

# Fallback matching when the exact contract name is absent (reported as a deviation, never silent).
_ALIASES = {
    "stereo_left": ("infra1/image_rect_raw", "left/image_rect", "left_rectified"),
    "stereo_right": ("infra2/image_rect_raw", "right/image_rect", "right_rectified"),
    "stereo_left_info": ("infra1/camera_info", "left/camera_info", "left_info"),
    "stereo_right_info": ("infra2/camera_info", "right/camera_info", "right_info"),
}


def resolve(topics: dict, overrides: Optional[dict] = None) -> tuple[dict, list]:
    """topics: {name: msg_type}. Returns ({role: topic}, notes). Exact contract name first, then overrides, then aliases."""
    out: dict = {}
    notes: list = []
    overrides = overrides or {}
    for s in SPECS:
        if s.role in overrides:
            out[s.role] = overrides[s.role]
            notes.append(f"{s.role}: topic override {overrides[s.role]}")
        elif s.topic in topics:
            out[s.role] = s.topic
        else:
            for al in _ALIASES.get(s.role, ()):
                c = [t for t in topics if t.endswith(al)]
                if len(c) == 1:
                    out[s.role] = c[0]
                    notes.append(f"{s.role}: contract topic {s.topic} absent, matched alias {c[0]} (NOT verified by the contract)")
                    break
    if "tf_static" not in out and "/static_tf" in topics:
        notes.append("only /static_tf present (v1.2.0 guide name, never observed on fw 2.1.5) - not used automatically; pass --topic tf_static=/static_tf")
    return out, notes
