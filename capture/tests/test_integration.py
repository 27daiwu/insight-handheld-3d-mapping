"""End-to-end with the simulator on an ISOLATED ROS domain (never touches the shared lab domain)."""
import json
import os
import random
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

CAP = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.ros

if not shutil.which("ros2"):
    pytest.skip("ROS 2 not sourced", allow_module_level=True)


def cli(env, *a, timeout=180):
    return subprocess.run([sys.executable, "-m", "insight_capture", *a], cwd=CAP, env=env,
                          capture_output=True, text=True, timeout=timeout)


@pytest.fixture(scope="module")
def env(tmp_path_factory):
    e = dict(os.environ)
    e["ROS_DOMAIN_ID"] = str(random.randint(120, 200))
    e["INSIGHT_CAMERA_API"] = "off"                       # never touch a real camera from the simulator tests
    e["INSIGHT_DATA_ROOT"] = str(tmp_path_factory.mktemp("sessions"))
    e["PYTHONPATH"] = str(CAP) + ":" + e.get("PYTHONPATH", "")
    return e


def start_sim(env, *extra):
    subprocess.run(["ros2", "daemon", "start"], env=env, capture_output=True, timeout=30)   # discovery on this host needs the per-domain daemon
    p = subprocess.Popen([sys.executable, "sim/sim_insight.py", "--imu-hz", "100", *extra], cwd=CAP, env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.5)
    return p


def kill(p):
    p.send_signal(signal.SIGINT)
    try:
        p.wait(10)
    except subprocess.TimeoutExpired:
        p.kill()


def manifest(env, sid):
    return json.loads((Path(env["INSIGHT_DATA_ROOT"]) / sid / "manifest.json").read_text())


def session_dirs(env):
    return sorted(d for d in Path(env["INSIGHT_DATA_ROOT"]).iterdir() if d.is_dir())


def test_no_camera_reports_and_refuses(env):
    r = cli(env, "preflight", "--probe-seconds", "1")
    assert r.returncode == 2 and "No Insight camera topics discovered" in r.stdout
    r = cli(env, "record", "--duration", "2")
    assert r.returncode == 2 and "REFUSING" in r.stderr
    assert not any(d.is_dir() for d in Path(env["INSIGHT_DATA_ROOT"]).glob("*") if (d / "manifest.json").exists())


def test_record_stop_inspect_replay_depth_off(env):
    sim = start_sim(env)
    try:
        r = cli(env, "preflight", "--probe-seconds", "3", "--firmware", "2.1.4")
        assert r.returncode == 1, r.stdout          # warnings only (depth off / unverified profile)
        assert "Depth NOT enabled" in r.stdout
        r = cli(env, "record", "--name", "e2e", "--firmware", "2.1.4", "--probe-seconds", "2")
        assert r.returncode == 0, r.stdout + r.stderr
        time.sleep(8)
        r = cli(env, "stop")
        assert r.returncode == 0 and "status=complete" in r.stdout
        sd = session_dirs(env)[-1]
        m = manifest(env, sd.name)
        assert m["status"] == "complete" and m["depth"]["enabled"] is False
        roles = {t["role"]: t for t in m["topics"]}
        assert {"rgb", "stereo_left", "stereo_right", "imu", "vio", "rgb_info", "stereo_left_info",
                "stereo_right_info", "tf", "tf_static"} <= set(roles)
        assert "depth" not in roles
        assert all(roles[k]["count"] > 0 for k in roles)
        assert m["camera"]["prefix"] == "/insight9" and m["camera"]["firmware"] == "2.1.4"
        assert (sd / "bag" / "metadata.yaml").exists()
        r = cli(env, "inspect", "latest", "--json")
        rep = json.loads(r.stdout)
        assert not [c for c in rep["checks"] if c["level"] == "FAIL"], r.stdout
        # replay: count imu messages republished
        imu_n = roles["imu"]["count"]
        sys.path.insert(0, str(CAP))
        os.environ.update(ROS_DOMAIN_ID=env["ROS_DOMAIN_ID"])
        from insight_capture.rosgraph import Graph
        kill(sim)                                    # make sure only replayed data is seen
        sim = None
        with Graph("replay_counter") as g:
            g.spin(1.0)
            rp = subprocess.Popen([sys.executable, "-m", "insight_capture", "replay", "latest", "--roles", "imu",
                                   "--rate", "4"], cwd=CAP, env=env, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
            res = g.probe({"/insight9/imu": "sensor_msgs/msg/Imu"}, 8.0)
        rp.wait(30)
        got = res["/insight9/imu"]["count"]
        assert rp.returncode == 0 and imu_n * 0.95 <= got <= imu_n, (got, imu_n)
    finally:
        if sim:
            kill(sim)


def test_depth_enabled_is_recorded(env):
    sim = start_sim(env, "--depth")
    try:
        r = cli(env, "record", "--name", "depth", "--duration", "5", "--foreground", "--probe-seconds", "2")
        assert r.returncode == 0, r.stdout + r.stderr
        sd = session_dirs(env)[-1]
        m = manifest(env, sd.name)
        assert m["depth"]["enabled"] is True
        assert any(t["role"] == "depth" and t["count"] > 0 for t in m["topics"])
    finally:
        kill(sim)


def test_crash_then_inspect_repair_replay(env):
    sim = start_sim(env)
    try:
        r = cli(env, "record", "--name", "crash", "--probe-seconds", "2")
        assert r.returncode == 0, r.stdout + r.stderr
        time.sleep(6)
        sd = session_dirs(env)[-1]
        st = json.loads((sd / "session.json").read_text())
        os.kill(st["supervisor_pid"], signal.SIGKILL)
        os.kill(st["child_pid"], signal.SIGKILL)
        time.sleep(1)
        assert cli(env, "stop").returncode == 3                       # tells the user it crashed
        r = cli(env, "inspect", sd.name, "--json")
        rep = json.loads(r.stdout)
        ids = {c["id"]: c for c in rep["checks"]}
        assert ids["state"]["title"].endswith("interrupted") and ids["metadata"]["level"] == "WARN"
        assert ids["data_imu"]["level"] in ("PASS", "WARN")            # data readable without metadata
        assert cli(env, "replay", sd.name, "--dry-run").returncode == 3
        r = cli(env, "inspect", sd.name, "--repair", "--json")
        assert (sd / "bag" / "metadata.yaml").exists()
        assert manifest(env, sd.name)["status"] == "recovered"
        assert cli(env, "replay", sd.name, "--dry-run").returncode == 0
    finally:
        kill(sim)


def test_low_disk_refused(env):
    sim = start_sim(env)
    try:
        r = cli(env, "record", "--min-free-gb", "99999999", "--probe-seconds", "1")
        assert r.returncode == 2 and "REFUSING" in r.stderr
    finally:
        kill(sim)
