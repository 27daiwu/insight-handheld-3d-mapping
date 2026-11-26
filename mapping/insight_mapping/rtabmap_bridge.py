"""RTAB-Map integration point (production back-end for P1-C).

STATUS: RTAB-Map is NOT installed on this host (apt package ros-humble-rtabmap-ros exists in the configured mirror but
installing system packages was out of scope / needs sudo). Nothing here has been run against RTAB-Map. What exists:
 * `availability()`            - detect rtabmap binaries / ROS packages.
 * `replay_command()`          - the exact `ros2` command + parameters we intend to run (VIO as external odometry,
                                  stereo images, vio_image preferred), for review.
 * contract: a RTAB-Map result must be converted to `trajectory_opt.csv` (stamp_ns,tx..qw; T_odom_rgbcam)
   and then `fusion.reconstruct` is back-end agnostic.
Control experiment (RTAB-Map's own stereo odometry): run with `odom_frame_id:=` empty and `stereo_odometry` node; the
in-repo stand-in is slam.pipeline.stereo_vo_control (ORB+PnP, SGBM depth).
"""
from __future__ import annotations

import shutil
import subprocess


def availability() -> dict:
    out = {"rtabmap_binary": shutil.which("rtabmap"), "rtabmap_ros": False}
    try:
        r = subprocess.run(["ros2", "pkg", "prefix", "rtabmap_ros"], capture_output=True, text=True, timeout=20)
        out["rtabmap_ros"] = r.returncode == 0
    except Exception:
        pass
    out["available"] = bool(out["rtabmap_binary"] or out["rtabmap_ros"])
    return out


def replay_command(bag_dir, left, right, left_info, right_info, vio_topic, frame_id="insight_body", use_vio=True) -> list:
    """Reviewable (untested) launch: rtabmap_ros stereo_odometry-less SLAM using external VIO odometry."""
    params = ["frame_id:=" + frame_id, "stereo:=true", f"left_image_topic:={left}", f"right_image_topic:={right}",
              f"left_camera_info_topic:={left_info}", f"right_camera_info_topic:={right_info}",
              "approx_sync:=true", "approx_sync_max_interval:=0.02", "use_sim_time:=true",
              "rtabmap_args:=--delete_db_on_start --Optimizer/Strategy 1 --RGBD/OptimizeFromGraphEnd false"]
    if use_vio:
        params += [f"odom_topic:={vio_topic}", "visual_odometry:=false"]
    else:
        params += ["visual_odometry:=true"]
    return ["ros2", "launch", "rtabmap_launch", "rtabmap.launch.py", *params, "&&", "ros2", "bag", "play", str(bag_dir), "--clock"]
