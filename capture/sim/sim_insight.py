#!/usr/bin/env python3
"""Simulated Insight 9 ROS 2 publisher for developing/testing capture without hardware.

NOT a driver: it only fabricates standard sensor_msgs/nav_msgs/tf2 topics under a configurable
prefix with the naming assumed by config/topics_v2.1.4.yaml. Run it on an ISOLATED domain:

    ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 python3 sim/sim_insight.py --prefix /insight9 [--depth]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class Sim(Node):
    def __init__(self, a):
        super().__init__("sim_insight9")
        self.a = a
        p = a.prefix.rstrip("/")
        sensor = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.t0 = time.monotonic()
        w, h = a.width, a.height
        self.dropped = 0
        self.n = {}
        cams = [("rgb", "rgb8", 3, a.rgb_hz), ("left", "mono8", 1, a.stereo_hz), ("right", "mono8", 1, a.stereo_hz)]
        if a.depth:
            cams.append(("depth", "16UC1", 2, a.depth_hz))
        for name, enc, bpp, hz in cams:
            base = f"{p}/{name}"
            pub = self.create_publisher(Image, base + "/image_raw", sensor)
            ipub = self.create_publisher(CameraInfo, base + "/camera_info", sensor)
            payload = bytes((i * 7) % 251 for i in range(w * h * bpp))
            self.create_timer(1.0 / hz, lambda n=name, e=enc, b=bpp, pu=pub, ip=ipub, pl=payload:
                              self.cam(n, e, b, pu, ip, pl))
        self.imu = self.create_publisher(Imu, p + "/imu", sensor)
        self.create_timer(1.0 / a.imu_hz, self.imu_cb)
        self.vio = self.create_publisher(Odometry, p + "/vio", sensor)
        self.create_timer(1.0 / a.vio_hz, self.vio_cb)
        self.tf = self.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
        self.tfs = self.create_publisher(TFMessage, "/tf_static", latched)
        self.create_timer(1.0 / 50, self.tf_cb)
        self.tfs.publish(TFMessage(transforms=[
            self._tf("camera_link", f) for f in ("rgb_optical", "left_optical", "right_optical", "imu_link")] +
            ([self._tf("camera_link", "depth_optical")] if a.depth else [])))

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _tf(self, parent, child):
        t = TransformStamped()
        t.header.stamp = self._stamp(); t.header.frame_id = parent; t.child_frame_id = child
        t.transform.rotation.w = 1.0
        return t

    def _skip(self, key):
        a = self.a
        self.n[key] = self.n.get(key, 0) + 1
        el = time.monotonic() - self.t0
        if a.stall_after is not None and a.stall_after <= el < a.stall_after + a.stall_for and key.startswith("rgb"):
            return True
        return bool(a.drop_every and key == "rgb" and self.n[key] % a.drop_every == 0)

    def cam(self, name, enc, bpp, pub, ipub, payload):
        key = "rgb" if name == "rgb" else name
        if self._skip(key):
            self.dropped += 1
            return
        stamp = self._stamp()
        fid = f"{name}_optical"
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, fid
        m.height, m.width, m.encoding, m.step, m.data = self.a.height, self.a.width, enc, self.a.width * bpp, payload
        pub.publish(m)
        ci = CameraInfo()
        ci.header.stamp, ci.header.frame_id = stamp, fid
        ci.height, ci.width = self.a.height, self.a.width
        f = float(self.a.width)
        ci.k = [f, 0.0, self.a.width / 2, 0.0, f, self.a.height / 2, 0.0, 0.0, 1.0]
        ci.distortion_model = "plumb_bob"; ci.d = [0.0] * 5
        ipub.publish(ci)

    def imu_cb(self):
        now = time.monotonic()
        last = getattr(self, '_last_imu', None)
        self._last_imu = now
        if last is not None and now - last > 0.5:
            self.get_logger().warn(f'SIM-SIDE STALL: imu timer gap {now - last:.2f}s')
        m = Imu()
        m.header.stamp, m.header.frame_id = self._stamp(), "imu_link"
        t = time.monotonic() - self.t0
        m.angular_velocity.z = 0.1 * math.sin(t)
        m.linear_acceleration.z = 9.81
        m.orientation.w = 1.0
        self.imu.publish(m)
        self.imu_pub_count = getattr(self, 'imu_pub_count', 0) + 1

    def vio_cb(self):
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = self._stamp(), "odom", "camera_link"
        t = time.monotonic() - self.t0
        m.pose.pose.position.x = 0.1 * t
        m.pose.pose.orientation.w = 1.0
        self.vio.publish(m)

    def tf_cb(self):
        self.tf.publish(TFMessage(transforms=[self._tf("odom", "camera_link")]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", default="/insight9")
    ap.add_argument("--depth", action="store_true", help="publish depth (default OFF like the real camera)")
    ap.add_argument("--rgb-hz", type=float, default=30)
    ap.add_argument("--stereo-hz", type=float, default=30)
    ap.add_argument("--depth-hz", type=float, default=15)
    ap.add_argument("--imu-hz", type=float, default=200)
    ap.add_argument("--vio-hz", type=float, default=30)
    ap.add_argument("--width", type=int, default=160)
    ap.add_argument("--height", type=int, default=120)
    ap.add_argument("--drop-every", type=int, default=0, help="skip every Nth rgb frame (fault injection)")
    ap.add_argument("--stall-after", type=float, help="stop publishing rgb after S seconds (fault injection)")
    ap.add_argument("--stall-for", type=float, default=0)
    a = ap.parse_args()
    rclpy.init()
    n = Sim(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        print(f'SIM published imu={getattr(n, "imu_pub_count", 0)}', flush=True)
        try:
            n.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
