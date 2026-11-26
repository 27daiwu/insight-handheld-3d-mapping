"""RTAB-Map offline reconstruction back-end.

Modes
  odom="vio"  : Insight VIO is the external odometry (nav_msgs/Odometry stamped with the image stamp, pose chosen with the
                same vio_image/vio_100hz policy as the in-repo back-end); rtabmap does loop closure + graph optimisation.
  odom="own"  : RTAB-Map's own visual odometry (rgbd_odometry or stereo_odometry); VIO is used only to anchor the world frame.
Sensors: sensor="rgbd" (RGB + Depth; depth is registered into the RGB camera first when not pixel aligned) or "stereo".

Replay is lock-step and deterministic: a feeder node publishes one synchronised frame, then waits for /rtabmap/info of that
frame (or a timeout) before publishing the next, so no frame is dropped by the pipeline's own rate limits.
The result is converted to the common `trajectory.csv` contract (per-frame T_world_rgbcam): keyframe poses optimised by
RTAB-Map (rtabmap-export --poses, robot frame) are turned into smooth corrections of the per-frame odometry,
exactly as done for the in-repo back-end, so every back-end feeds the same fusion code.
"""
from __future__ import annotations

import json
import os
import resource
import signal
import sqlite3
import subprocess
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path

import cv2
import numpy as np

from .. import se3
from ..calib import rectified_model, rectify
from ..extract import Extracted
from ..session import Session
from ..slam import pipeline as slam_pipeline, vio, vo
from ..slam.pipeline import trajectory_to_csv


@dataclass
class RtabmapConfig:
    odom: str = "vio"                 # vio | own
    sensor: str = "rgbd"              # rgbd | stereo
    stride: int = 1
    max_frames: int | None = None
    odom_cov_lin: float = 1e-5        # (m^2) external-odometry covariance, controls how much loops may bend the odometry
    odom_cov_rot: float = 1e-5        # (rad^2)
    optimizer: int = 2                # Optimizer/Strategy: 2=GTSAM (best in sweeps: ATE 0.040 vs 0.058 g2o), 1=g2o, 0=TORO
    max_pair_dt_ms: float = 33.0
    first_frame_timeout_s: float = 30.0
    frame_timeout_s: float = 6.0
    max_consecutive_timeouts: int = 12   # abort feeding (odometry lost for good) instead of waiting out the rest
    rtabmap_params: dict = field(default_factory=dict)   # extra "Group/Param": value passed to rtabmap
    # own-odometry defaults (frame-to-map): chosen on BOTH TUM fr1/desk (stock MinInliers=20 lost tracking after ~100 frames) and the synthetic room (frame-to-frame diverged there)
    odom_params: dict = field(default_factory=lambda: {"Odom/Strategy": 0, "Vis/MaxFeatures": 1500, "Vis/MinInliers": 12,
                                                       "Vis/CorGuessWinSize": 40})
    keep_db: bool = True


def availability() -> dict:
    import shutil
    out = {"rtabmap_slam": None, "mcap_plugin": False, "rtabmap_export": shutil.which("rtabmap-export")}
    try:
        r = subprocess.run(["ros2", "pkg", "prefix", "rtabmap_slam"], capture_output=True, text=True, timeout=30)
        out["rtabmap_slam"] = r.stdout.strip() if r.returncode == 0 else None
        r = subprocess.run(["ros2", "pkg", "prefix", "rosbag2_storage_mcap"], capture_output=True, text=True, timeout=30)
        out["mcap_plugin"] = r.returncode == 0
        r = subprocess.run(["rtabmap-info", "--version"], capture_output=True, text=True, timeout=30)
        out["rtabmap_version"] = (r.stdout + r.stderr).strip().splitlines()[0] if (r.stdout + r.stderr).strip() else None
    except Exception:
        pass
    out["available"] = bool(out["rtabmap_slam"] and out["rtabmap_export"])
    return out


def _time(ns):
    from builtin_interfaces.msg import Time
    return Time(sec=int(ns // 10**9), nanosec=int(ns % 10**9))


def _img_msg(arr, enc, stamp_ns, frame_id):
    from sensor_msgs.msg import Image
    m = Image()
    m.header.stamp = _time(stamp_ns); m.header.frame_id = frame_id
    m.height, m.width, m.encoding, m.is_bigendian = int(arr.shape[0]), int(arr.shape[1]), enc, 0
    m.step = int(arr.strides[0])
    m.data = np.ascontiguousarray(arr).tobytes()
    return m


def _info_msg(cam, stamp_ns, P_tx=None):
    """CameraInfo of a RECTIFIED image (rectified_model): D empty, R identity."""
    from sensor_msgs.msg import CameraInfo
    m = CameraInfo()
    m.header.stamp = _time(stamp_ns); m.header.frame_id = cam.frame_id
    m.width, m.height = cam.width, cam.height
    m.distortion_model = "plumb_bob"; m.d = [0.0] * 5
    m.k = list(map(float, cam.K)); m.r = list(map(float, np.eye(3).ravel())); m.p = list(map(float, cam.P))
    return m


def _odom_msg(stamp_ns, T, parent, child, cfg):
    from nav_msgs.msg import Odometry
    m = Odometry()
    m.header.stamp = _time(stamp_ns); m.header.frame_id = parent; m.child_frame_id = child
    t, q = se3.T_to_tq(T)
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, t)
    m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = map(float, q)
    cov = np.zeros(36)
    for i in range(3):
        cov[i * 7] = cfg.odom_cov_lin
        cov[(i + 3) * 7] = cfg.odom_cov_rot
    m.pose.covariance = cov.tolist()
    m.twist.covariance = (np.eye(6).ravel() * 1e-3).tolist()
    return m


class Replay:
    def __init__(self, session: Session, ex: Extracted, cfg: RtabmapConfig):
        self.s, self.ex, self.cfg = session, ex, cfg
        st = session.streams
        if cfg.sensor == "rgbd":
            if not ("rgb" in st and "depth" in st):
                raise RuntimeError("sensor=rgbd needs rgb+depth streams")
            self.base, self.cam_raw = ex.index("rgb"), ex.cams["rgb_info"]
            self.dep, self.dcam_raw = ex.index("depth"), ex.cams["depth_info"]
        else:
            if not ("left" in st and "right" in st):
                raise RuntimeError("sensor=stereo needs left+right streams")
            self.base, self.cam_raw = ex.index("left"), ex.cams["left_info"]
            self.dep, self.dcam_raw = ex.index("right"), ex.cams["right_info"]
        self.cam = rectified_model(self.cam_raw)
        self.dcam = rectified_model(self.dcam_raw)
        vi = ex.vio("vio_image") or ex.vio("vio_100hz")
        self.vio_child = vi["child"] if vi else self.cam.frame_id
        self.vio_parent = vi["parent"] if vi else "odom"
        # base frame: the VIO body frame if cameras are reachable from it, else the camera frame itself
        if vi and ex.tf.can_lookup(self.vio_child, self.cam.frame_id):
            self.base_frame = self.vio_child
        else:
            self.base_frame = self.cam.frame_id
        self.T_bc = ex.tf.lookup(self.base_frame, self.cam.frame_id) if self.base_frame != self.cam.frame_id else np.eye(4)
        n = len(self.base["stamp_ns"])
        sel = list(range(0, n, cfg.stride))
        if cfg.max_frames:
            sel = sel[:cfg.max_frames]
        stamps = self.base["stamp_ns"][sel]
        self.has_vio = vi is not None
        if self.has_vio:
            self.T_ob, self.vio_valid, self.vio_src, *_ = vio.body_poses(ex, stamps)
        else:
            self.T_ob, self.vio_valid = np.tile(np.eye(4), (len(stamps), 1, 1)), np.ones(len(stamps), bool)
            self.vio_src = ["none"] * len(stamps)
        if cfg.odom == "vio" and not self.has_vio:
            raise RuntimeError("odom=vio requested but the Session has no VIO stream")
        keep = self.vio_valid if (cfg.odom == "vio") else np.ones(len(stamps), bool)
        self.idx = [sel[i] for i in range(len(sel)) if keep[i]]
        self.frame_pos = [i for i in range(len(sel)) if keep[i]]       # position in stamps/T_ob arrays
        self.stamps = stamps
        self.dep_j = []
        for k in self.idx:
            j = int(np.argmin(np.abs(self.dep["stamp_ns"] - self.base["stamp_ns"][k])))
            self.dep_j.append(j)
        self.aligned = None
        if cfg.sensor == "rgbd":
            from ..calib import rgbd_alignment
            T = ex.tf.lookup(self.cam_raw.frame_id, self.dcam_raw.frame_id) if ex.tf.can_lookup(self.cam_raw.frame_id, self.dcam_raw.frame_id) else None
            al = rgbd_alignment(self.cam_raw, self.dcam_raw, T)
            self.aligned = al["aligned"] and np.allclose(self.cam.K, self.dcam.K, atol=1.0)
            if T is None and not self.aligned:
                raise RuntimeError("RGB/Depth alignment unproven and no TF: refusing (same rule as fusion)")
            self.T_c_d = T if T is not None else np.eye(4)

    def frame(self, n):
        """-> (stamp_ns, dict of messages by topic key). Depth is re-stamped to the RGB stamp (registered into the RGB frame)."""
        k, j = self.idx[n], self.dep_j[n]
        ts = int(self.base["stamp_ns"][k])
        ex = self.ex
        out = {}
        if self.cfg.sensor == "rgbd":
            rgb = rectify(ex.read(self.base["file"][k]), self.cam_raw, cv2.INTER_LINEAR)
            if rgb.ndim == 2:
                rgb = cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)
            d = rectify(ex.read(self.dep["file"][j]), self.dcam_raw, cv2.INTER_NEAREST).astype(np.float32) / 1000.0
            if not self.aligned:
                d = vo.register_depth_to_rgb(d, self.dcam, self.cam, self.T_c_d)
            out["rgb"] = _img_msg(rgb, "bgr8", ts, self.cam.frame_id)
            out["depth"] = _img_msg(np.round(d * 1000).astype(np.uint16), "16UC1", ts, self.cam.frame_id)
            out["info"] = _info_msg(self.cam, ts)
        else:
            left = rectify(ex.read(self.base["file"][k], 0), self.cam_raw, cv2.INTER_LINEAR)
            right = rectify(ex.read(self.dep["file"][j], 0), self.dcam_raw, cv2.INTER_LINEAR)
            out["left"] = _img_msg(left, "mono8", ts, self.cam.frame_id)
            out["right"] = _img_msg(right, "mono8", ts, self.dcam.frame_id)
            out["linfo"] = _info_msg(self.cam, ts)
            out["rinfo"] = _info_msg(self.dcam, ts)
        return ts, out


def _launch_args(rp: Replay, cfg: RtabmapConfig, db: Path, odom_topic: str) -> list:
    params = {"Rtabmap/DetectionRate": "0", "Optimizer/Strategy": str(cfg.optimizer), "RGBD/OptimizeFromGraphEnd": "false",
              "Mem/IncrementalMemory": "true", "Mem/STMSize": "10"}
    params.update({k: str(v) for k, v in cfg.rtabmap_params.items()})
    rt_args = "--delete_db_on_start " + " ".join(f"--{k} {v}" for k, v in params.items())
    a = ["ros2", "launch", "rtabmap_launch", "rtabmap.launch.py", "rtabmap_viz:=false", "rviz:=false",
         f"frame_id:={rp.base_frame}", f"database_path:={db}", "approx_sync:=false", "qos:=1", "queue_size:=30",
         "topic_queue_size:=30", "wait_for_transform:=0.2", "odom_sensor_sync:=false", f"rtabmap_args:={rt_args}", f"odom_topic:={odom_topic}",
         "use_sim_time:=false", f"publish_tf_odom:={'true' if cfg.odom == 'own' else 'false'}"]
    if cfg.sensor == "rgbd":
        a += ["rgb_topic:=/feed/rgb/image", "depth_topic:=/feed/depth/image", "camera_info_topic:=/feed/rgb/camera_info"]
    else:
        a += ["stereo:=true", "left_image_topic:=/feed/left/image", "right_image_topic:=/feed/right/image",
              "left_camera_info_topic:=/feed/left/camera_info", "right_camera_info_topic:=/feed/right/camera_info"]
    if cfg.odom == "own":
        a += ["visual_odometry:=true", "vo_frame_id:=odom"]
        if cfg.odom_params:
            a += [f"odom_args:=" + " ".join(f"--{k} {v}" for k, v in cfg.odom_params.items())]
    else:
        a += ["visual_odometry:=false"]
    return a


def _read_poses(path: Path):
    rows = [ln.split() for ln in path.read_text().splitlines() if ln.strip() and not ln.startswith("#")]
    a = np.array([[float(x) for x in r[:8]] for r in rows])
    return a[:, 0], np.stack([se3.make_T(r[1:4], r[4:8]) for r in a])


def db_stats(db: Path) -> dict:
    c = sqlite3.connect(str(db))
    try:
        nodes = c.execute("select count(*) from Node").fetchone()[0]
        links = dict(c.execute("select type, count(*) from Link group by type").fetchall())
    finally:
        c.close()
    names = {0: "neighbor", 1: "global_closure", 2: "local_space_closure", 3: "local_time_closure", 4: "user_closure",
             5: "virtual_closure", 6: "neighbor_merged", 7: "poseprior", 8: "landmark", 9: "gravity"}
    return {"nodes": int(nodes), "links": {names.get(k, str(k)): int(v) for k, v in links.items()},
            "loop_closures": int(links.get(1, 0) + links.get(2, 0)), "global_closures": int(links.get(1, 0)),
            "local_space_closures": int(links.get(2, 0))}


def run(session: Session, ex: Extracted, out_dir, cfg: RtabmapConfig | None = None, gt_T=None, log=print) -> dict:
    cfg = cfg or RtabmapConfig()
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from rtabmap_msgs.msg import Info

    av = availability()
    if not av["available"]:
        raise RuntimeError(f"RTAB-Map not available: {av}. Run scripts/install_rtabmap_userspace.sh and `source scripts/env.sh`")
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    db = out / "rtabmap.db"
    rp = Replay(session, ex, cfg)
    topics = {"odom": "/vo/odom" if cfg.odom == "own" else "/vio/odom"}
    t_all = time.time()
    rep: dict = {"backend": f"rtabmap_{'vio' if cfg.odom == 'vio' else 'odom'}_{cfg.sensor}", "config": asdict(cfg),
                 "frames_fed": len(rp.idx), "base_frame": rp.base_frame, "rgbd_aligned": rp.aligned, "rtabmap": av}

    rclpy.init()
    node = Node("insight_feeder")
    rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    pubs = {}
    if cfg.sensor == "rgbd":
        pubs = {"rgb": node.create_publisher(Image, "/feed/rgb/image", rel), "depth": node.create_publisher(Image, "/feed/depth/image", rel),
                "info": node.create_publisher(CameraInfo, "/feed/rgb/camera_info", rel)}
    else:
        pubs = {"left": node.create_publisher(Image, "/feed/left/image", rel), "right": node.create_publisher(Image, "/feed/right/image", rel),
                "linfo": node.create_publisher(CameraInfo, "/feed/left/camera_info", rel),
                "rinfo": node.create_publisher(CameraInfo, "/feed/right/camera_info", rel)}
    odom_pub = node.create_publisher(Odometry, topics["odom"], rel) if cfg.odom == "vio" else None
    info_seen = {"stamp": 0, "n": 0}
    recorded = {}      # stamp_ns -> (T_odom_body, valid) from own odometry

    def on_info(m):
        info_seen["stamp"] = max(info_seen["stamp"], m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
        info_seen["n"] += 1

    def on_odom(m):
        st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        p, o = m.pose.pose.position, m.pose.pose.orientation
        q = np.array([o.x, o.y, o.z, o.w]); nq = np.linalg.norm(q)
        lost = (nq < 0.5) or (m.pose.covariance[0] > 1e3)
        if nq > 0:
            recorded[st] = (se3.make_T([p.x, p.y, p.z], q / nq), not lost)

    node.create_subscription(Info, "/rtabmap/info", on_info, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
    if cfg.odom == "own":
        node.create_subscription(Odometry, topics["odom"], on_odom, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        node.create_subscription(Odometry, topics["odom"], on_odom, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)) if False else None

    # static TF: every static edge of the Session (+ nothing dynamic: external odometry travels on the topic)
    tfb = StaticTransformBroadcaster(node)
    msgs = []
    for (p, c), T in ex.tf.static.items():
        ts = TransformStamped(); ts.header.frame_id = p; ts.child_frame_id = c
        t, q = se3.T_to_tq(T)
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = map(float, t)
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = map(float, q)
        msgs.append(ts)
    if msgs:
        tfb.sendTransform(msgs)

    cmd = _launch_args(rp, cfg, db, topics["odom"])
    logf = open(out / "rtabmap_launch.log", "w")
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
    rep["launch_cmd"] = " ".join(cmd)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        t0 = time.time()
        need = list(pubs.values()) + ([odom_pub] if odom_pub else [])
        while time.time() - t0 < 60 and not all(p.get_subscription_count() >= 1 for p in need):
            spin(0.2)
            if proc.poll() is not None:
                raise RuntimeError("rtabmap launch exited early, see rtabmap_launch.log")
        if not all(p.get_subscription_count() >= 1 for p in need):
            raise RuntimeError("rtabmap did not subscribe to the feed topics within 60 s (see rtabmap_launch.log)")
        spin(1.0)
        lat, timeouts, fed_stamps, consec, aborted = [], 0, [], 0, False
        t_feed = time.time()
        for n in range(len(rp.idx)):
            ts, msgs_ = rp.frame(n)
            if odom_pub is not None:
                odom_pub.publish(_odom_msg(ts, rp.T_ob[rp.frame_pos[n]], rp.vio_parent, rp.vio_child, cfg))
            t1 = time.time()
            for k, m in msgs_.items():
                pubs[k].publish(m)
            tmo = cfg.first_frame_timeout_s if n == 0 else cfg.frame_timeout_s
            while info_seen["stamp"] < ts - 1_000_000 and time.time() - t1 < tmo:
                rclpy.spin_once(node, timeout_sec=0.01)
            if info_seen["stamp"] < ts - 1_000_000:
                timeouts += 1; consec += 1
            else:
                lat.append(time.time() - t1); consec = 0
            fed_stamps.append(ts)
            if consec >= cfg.max_consecutive_timeouts:
                aborted = True
                log(f"rtabmap: {consec} consecutive frames without output at frame {n}: aborting feed")
                break
            if n % 50 == 0:
                log(f"rtabmap feed {n}/{len(rp.idx)}  infos={info_seen['n']} timeouts={timeouts}")
        spin(1.5)
        rep["feed"] = {"aborted_after_lost_tracking": aborted, "frames_fed": len(fed_stamps), "wall_s": time.time() - t_feed, "info_messages": info_seen["n"], "timeouts": timeouts,
                       "latency_ms_median": float(np.median(lat) * 1e3) if lat else None,
                       "latency_ms_p95": float(np.percentile(lat, 95) * 1e3) if lat else None}
    finally:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=120)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass
        logf.close()
        node.destroy_node()
        rclpy.shutdown()
    rep["peak_rss_children_mib"] = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / 1024.0

    if not db.exists():
        raise RuntimeError("rtabmap produced no database (see rtabmap_launch.log)")
    rep["db"] = db_stats(db)
    pose_dir = out / "export"
    pose_dir.mkdir(exist_ok=True)
    r = subprocess.run(["rtabmap-export", "--poses", "--poses_format", "11", "--opt", "0", "--output_dir", str(pose_dir),
                        "--output", "rtabmap", str(db)], capture_output=True, text=True, timeout=300)
    pf = sorted(pose_dir.glob("*poses*.txt"))
    if not pf:
        raise RuntimeError("rtabmap-export produced no poses:\n" + r.stdout[-800:] + r.stderr[-800:])
    kst, kT = _read_poses(pf[0])
    rep["keyframes"] = int(len(kst))

    # ---- convert to the common per-frame trajectory contract
    fed = np.array(fed_stamps, dtype=np.int64)
    pos = np.array(rp.frame_pos)
    if cfg.odom == "vio":
        T_odom_body = rp.T_ob[pos]; valid = np.ones(len(fed), bool)
        anchor = np.eye(4)
    else:
        T_odom_body = np.tile(np.eye(4), (len(fed), 1, 1)); valid = np.zeros(len(fed), bool)
        for i, s in enumerate(fed):
            if int(s) in recorded and recorded[int(s)][1]:
                T_odom_body[i] = recorded[int(s)][0]; valid[i] = True
        rep["own_odometry"] = {"tracked": int(valid.sum()), "lost": int((~valid).sum()), "frames": int(len(fed))}
        if not valid.any():
            raise RuntimeError("RTAB-Map odometry never produced a pose")
        first = int(np.nonzero(valid)[0][0])
        # anchor own-odometry frame into the VIO world frame using the first tracked frame
        anchor = rp.T_ob[pos[first]] @ se3.inv(T_odom_body[first]) if rp.has_vio else np.eye(4)
    # keyframes -> frame index (nearest stamp, 2 ms)
    kns = np.round(kst * 1e9).astype(np.int64)
    k_idx = []
    for s in kns:
        i = int(np.argmin(np.abs(fed - s)))
        if abs(int(fed[i]) - int(s)) < 2_000_000 and valid[i]:
            k_idx.append(i)
    k_idx = np.array(sorted(set(k_idx)), dtype=int)
    rep["keyframes_matched"] = int(len(k_idx))
    if len(k_idx) < 1:
        raise RuntimeError("no RTAB-Map keyframe matched a fed frame")
    # robot-frame optimised pose per matched keyframe
    kmap = {int(fed[int(np.argmin(np.abs(fed - s)))]): T for s, T in zip(kns, kT)}
    T_opt_kf = np.stack([kmap[int(fed[i])] for i in k_idx])
    T_odom_kf = T_odom_body[k_idx]
    T_corr_opt = slam_pipeline.apply_corrections(fed[valid].astype(np.int64), T_odom_body[valid], np.searchsorted(np.nonzero(valid)[0], k_idx), T_opt_kf)
    T_body_final = np.tile(np.eye(4), (len(fed), 1, 1)); T_body_final[valid] = T_corr_opt
    T_cam = anchor @ T_body_final @ rp.T_bc
    T_cam_odom = anchor @ T_odom_body @ rp.T_bc
    trajectory_to_csv(out / "trajectory_opt.csv", fed, T_cam, valid)
    trajectory_to_csv(out / "trajectory_odom.csv", fed, T_cam_odom, valid)
    rep["trajectory_frames"] = int(valid.sum())
    rep["max_keyframe_correction_m"] = float(np.linalg.norm(T_opt_kf[:, :3, 3] - T_odom_kf[:, :3, 3], axis=1).max())
    rep["elapsed_s"] = time.time() - t_all
    (out / "rtabmap_report.json").write_text(json.dumps(rep, indent=1, default=float))
    if not cfg.keep_db:
        db.unlink(missing_ok=True)
    return rep
