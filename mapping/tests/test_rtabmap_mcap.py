"""RTAB-Map back-end, MCAP storage, config overrides and texture integration (ROS env required)."""
import json

import numpy as np
import pytest

from insight_mapping import compare, extract
from insight_mapping.fixtures import synthetic
from insight_mapping.session import Session
from insight_mapping.slam import pipeline

pytestmark = pytest.mark.ros


def test_mcap_session_roundtrip(tmp_path):
    from insight_mapping import bagio
    if not bagio.storage_plugin_available("mcap"):
        pytest.skip("rosbag2_storage_mcap not installed")
    sdir, gt = synthetic.make_session(tmp_path, "session_mcap", n_frames=12, storage="mcap")
    s = Session.open(sdir)
    assert s.problems == [] and s.manifest["storage"]["format"] == "mcap" and "fallback_reason" not in s.manifest["storage"]
    extract.extract(s, tmp_path / "w", log=lambda *_: None)
    ex = extract.Extracted(tmp_path / "w")
    assert np.array_equal(ex.index("rgb")["stamp_ns"], gt["stamps_ns"])            # stamps preserved through MCAP


def test_topic_map_overrides_extrinsic_and_depth_scale(tmp_path):
    sdir, _ = synthetic.make_session(tmp_path, "session_ov", n_frames=6)
    tm = tmp_path / "tm.yaml"
    tm.write_text("overrides: {}\npatterns: " + json.dumps(__import__("yaml").safe_load(open("config/topic_map.yaml"))["patterns"]) +
                  "\nextrinsics:\n  - {parent: insight_body, child: depth_optical, translation: [0.0, 0.0, 0.0], quaternion: [0, 0, 0, 1]}\n"
                  "depth: {scale_to_mm: 2.0}\n")
    s = Session.open(sdir, topic_map=tm)
    extract.extract(s, tmp_path / "w", log=lambda *_: None)
    ex = extract.Extracted(tmp_path / "w")
    assert np.allclose(ex.tf.lookup("insight_body", "depth_optical")[:3, 3], 0)      # override beat the bag
    assert any("extrinsic" in o for o in ex.summary["overrides_applied"])
    s0 = Session.open(sdir)
    extract.extract(s0, tmp_path / "w0", log=lambda *_: None)
    d0, d1 = extract.Extracted(tmp_path / "w0"), ex
    a = d0.read(d0.index("depth")["file"][0]).astype(int); b = d1.read(d1.index("depth")["file"][0]).astype(int)
    assert np.abs(b - 2 * a).max() <= 1


@pytest.mark.rtabmap
@pytest.mark.slow
def test_rtabmap_backends_on_synthetic(synth, tmp_path):
    work = synth["out"]
    gt = (synth["gt"]["stamps_ns"], synth["gt"]["T_world_rgb"])
    from insight_mapping.backends import run_backend
    res = {}
    for b in ("rtabmap_vio", "rtabmap_odom"):
        rep = run_backend(b, synth["dir"], work, tmp_path / b, log=lambda *_: None)
        assert rep["loops"] > 0
        m, _ = compare.traj_metrics(tmp_path / b / "trajectory_opt.csv", gt, 240)
        res[b] = m
    vio = pipeline.read_trajectory_csv(work / "slam" / "trajectory_vio.csv")
    raw = compare.traj_metrics(work / "slam" / "trajectory_vio.csv", gt, 240)[0]
    assert res["rtabmap_vio"]["ate_aligned"]["rmse_m"] < 0.5 * raw["ate_aligned"]["rmse_m"]      # loop closure fixes VIO drift
    assert res["rtabmap_odom"]["ate_aligned"]["rmse_m"] < 0.10                                   # own odometry works
    assert res["rtabmap_odom"]["frames"] >= 230


def test_run_session_texture_outputs(synth):
    t = synth["rep"]["texture"]
    assert t["textured_fraction"] > 0.5
    assert synth["rep"]["outputs_verified"]["mesh.glb"]["ok"] and synth["rep"]["outputs_verified"]["mesh_textured.obj"]["ok"]
