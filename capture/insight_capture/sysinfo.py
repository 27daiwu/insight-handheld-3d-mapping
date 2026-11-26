"""Host / ROS / USB / network / disk information. Read-only, every external call has a timeout."""
from __future__ import annotations

import os
import platform
import shutil
import socket
import subprocess
from pathlib import Path


def run(cmd: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout + p.stderr).strip()
    except FileNotFoundError:
        return 127, f"{cmd[0]}: not found"
    except subprocess.TimeoutExpired:
        return 124, f"{cmd[0]}: timed out after {timeout}s"


def os_info() -> dict:
    d = {"hostname": socket.gethostname(), "kernel": platform.release(), "python": platform.python_version()}
    try:
        for line in Path("/etc/os-release").read_text().splitlines():
            if line.startswith("PRETTY_NAME="):
                d["os"] = line.split("=", 1)[1].strip('"')
    except OSError:
        pass
    d["wsl"] = "microsoft" in platform.release().lower() or "WSL_DISTRO_NAME" in os.environ
    return d


def ros_info() -> dict:
    rmw_env = os.environ.get("RMW_IMPLEMENTATION")
    return {
        "distro": os.environ.get("ROS_DISTRO"),
        "domain_id": os.environ.get("ROS_DOMAIN_ID", "0 (default)"),
        "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "0"),
        "rmw_env": rmw_env,
        # Humble's default RMW is Fast DDS when RMW_IMPLEMENTATION is unset.
        "rmw_effective": rmw_env or "rmw_fastrtps_cpp (Humble default)",
        "fastdds_profile": os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"),
        "ros2_cli": shutil.which("ros2"),
        "ament_prefix": os.environ.get("AMENT_PREFIX_PATH", "").split(":")[0],
    }


def rosbag2_info() -> dict:
    rc, out = run(["ros2", "bag", "list", "storage"], 20)
    return {"ok": rc == 0, "storage_plugins": out.splitlines() if rc == 0 else [], "error": None if rc == 0 else out}


def usb_info() -> dict:
    d: dict = {"lsusb": None, "devices": [], "video_nodes": sorted(str(p) for p in Path("/dev").glob("video*")),
               "usb_bus_present": Path("/dev/bus/usb").exists()}
    rc, out = run(["lsusb"], 5)
    if rc == 0:
        d["devices"] = out.splitlines()
    else:
        d["lsusb"] = out
    return d


def net_info() -> dict:
    rc, out = run(["ip", "-br", "addr"], 5)
    ifs = []
    if rc == 0:
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 2:
                ifs.append({"name": parts[0], "state": parts[1], "addrs": parts[2:]})
    return {"interfaces": ifs, "error": None if rc == 0 else out}


def disk_info(path: str | os.PathLike) -> dict:
    p = Path(path)
    while not p.exists() and p != p.parent:
        p = p.parent
    u = shutil.disk_usage(p)
    return {"path": str(p), "total_gb": round(u.total / 1e9, 2), "free_gb": round(u.free / 1e9, 2),
            "free_bytes": u.free}


def git_commit() -> str | None:
    rc, out = run(["git", "-C", str(Path(__file__).resolve().parent), "rev-parse", "--short", "HEAD"], 3)
    return out if rc == 0 else None


def ensure_ros2_daemon() -> tuple[bool, str]:
    """Start the per-domain `ros2 daemon` if needed (idempotent, ROS-level only).

    On the dev host (WSL2) standalone rclpy processes did NOT discover each other via Fast DDS
    multicast; discovery only worked while the ros2 daemon of that domain was running. Starting it
    does not touch system network/driver/camera settings."""
    rc, out = run(["ros2", "daemon", "start"], 20)
    return rc == 0, out.splitlines()[-1] if out else ""
