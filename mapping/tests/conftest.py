import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
TUM_DIR = ROOT / ".cache" / "public" / "rgbd_dataset_freiburg1_desk"

try:
    import rosbag2_py  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


def _rtabmap_ok():
    try:
        from insight_mapping.rtabmap import backend
        return backend.availability()["available"]
    except Exception:
        return False


HAVE_RTABMAP = HAVE_ROS and _rtabmap_ok()


def pytest_collection_modifyitems(config, items):
    for it in items:
        if "ros" in it.keywords and not HAVE_ROS:
            it.add_marker(pytest.skip("needs `source /opt/ros/humble/setup.bash`"))
        if "rtabmap" in it.keywords and not HAVE_RTABMAP:
            it.add_marker(pytest.skip("RTAB-Map not available (source mapping/scripts/env.sh)"))
        if "tum" in it.keywords and not TUM_DIR.exists():
            it.add_marker(pytest.skip("TUM fr1/desk not downloaded (mapping/.cache/public)"))


@pytest.fixture(scope="session")
def synth(tmp_path_factory):
    """240-frame synthetic Session (RGB/Depth NOT aligned, VIO with injected drift) + its pipeline run."""
    from insight_mapping.fixtures import synthetic
    from insight_mapping.recon.fusion import ReconConfig
    from insight_mapping.run import run_session
    root = tmp_path_factory.mktemp("synth")
    sdir, gt = synthetic.make_session(root, "session_synth", n_frames=240, aligned=False)
    out = root / "out"
    rep = run_session(sdir, out, gt_T=gt["T_world_rgb"], recon_cfg=ReconConfig(), log=lambda *_: None)
    return {"dir": sdir, "gt": gt, "out": out, "rep": rep}
