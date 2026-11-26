"""End-to-end tests on the synthetic Session (needs ROS 2 Humble python env for rosbag2)."""
import numpy as np
import open3d as o3d
import pytest

from insight_mapping import extract, run as run_mod
from insight_mapping.fixtures import synthetic
from insight_mapping.recon import eval_synth, fusion
from insight_mapping.session import Session
from insight_mapping.slam import frames, pipeline

pytestmark = pytest.mark.ros


def test_session_resolves_streams_and_contract(synth):
    s = Session.open(synth["dir"])
    assert s.problems == []
    for k in ("rgb", "depth", "left", "right", "vio_image", "vio_100hz", "imu", "rgb_info", "depth_info"):
        assert k in s.streams
    assert s.missing_mandatory() == []


def test_original_timestamps_preserved(synth):
    ex = extract.Extracted(synth["out"])
    assert np.array_equal(ex.index("rgb")["stamp_ns"], synth["gt"]["stamps_ns"])      # bit-exact ROS stamps


def test_quality_report(synth):
    rep = synth["rep"]["data_quality"]
    assert rep["summary"]["errors"] == 0
    assert rep["time_sync"]["rgb->vio_image"]["max_ms"] <= 1.5
    assert rep["time_sync"]["rgb->depth"]["median_ms"] == pytest.approx(3.0)
    assert rep["calibration"]["rgbd_alignment"]["aligned"] is False                   # never assumed aligned
    assert rep["calibration"]["stereo"]["baseline_from_P_m"] == pytest.approx(0.06)
    assert rep["calibration"]["stereo"]["baseline_from_tf_m"] == pytest.approx(0.06)


def test_vio_uses_vio_image_for_pairing(synth):
    p = synth["rep"]["trajectory"]["vio_pairing"]
    assert p["vio_image"] == 240 and p["unpaired"] == 0


def test_frame_convention_validated(synth):
    assert synth["rep"]["trajectory"]["frame_validation"] == "ok"


def test_wrong_vio_convention_is_detected(tmp_path):
    sdir, _ = synthetic.make_session(tmp_path, "session_bad", n_frames=60, vio_pose_inverted=True)
    s = Session.open(sdir)
    extract.extract(s, tmp_path / "w", log=lambda *_: None)
    r = frames.validate_directions(extract.Extracted(tmp_path / "w"), s, log=lambda *_: None)
    assert r["status"] == "MISMATCH"
    with pytest.raises(RuntimeError, match="convention"):
        pipeline.run(s, extract.Extracted(tmp_path / "w"), tmp_path / "slam", log=lambda *_: None)


def test_loop_closure_reduces_drift(synth):
    rep = synth["rep"]
    assert rep["loop_closure"]["accepted"] > 0
    gt = rep["trajectory"]["vs_ground_truth"]
    assert gt["optimized"]["ate"]["rmse_m"] < 0.5 * gt["vio"]["ate"]["rmse_m"]
    assert gt["optimized"]["ate"]["rmse_m"] < 0.08


def test_stereo_control_experiment_present(synth):
    c = synth["rep"]["trajectory"]["stereo_vo_control"]
    assert c["status"] == "ok" and "ate_vs_gt" in c           # reported as-is; accuracy is NOT asserted (see docs)


def test_unaligned_rgbd_handled_and_reported(synth):
    r = synth["rep"]["reconstruction"]
    assert r["rgbd_aligned"] is False
    assert any("not aligned" in a for a in synth["rep"]["anomalies"])


def test_reconstruction_accuracy_vs_analytic_scene(synth):
    m = o3d.io.read_triangle_mesh(str(synth["out"] / "mesh.ply"))
    e = eval_synth.evaluate_mesh(m)
    assert e["median_m"] < 0.05 and e["frac_within_3cm"] > 0.4
    assert synth["rep"]["reconstruction"]["mesh_vertex_color_coverage"] > 0.9


def test_outputs_open_and_report_complete(synth):
    v = run_mod.verify_outputs(synth["out"])
    assert v["all_ok"], v
    rep = synth["rep"]
    assert rep["performance"]["reconstruction_elapsed_s"] > 0 and rep["performance"]["peak_rss_mib"] > 0
    assert rep["reconstruction"]["points"] > 1000 and rep["reconstruction"]["mesh_triangles"] > 1000
    for k in ("drift", "loop_closure", "anomalies", "data_quality"):
        assert k in rep or k in rep["trajectory"]


def test_refuses_unalignable_rgbd_without_tf(synth, tmp_path):
    ex = extract.Extracted(synth["out"])
    ex.tf.static.pop(("insight_body", "depth_optical"))        # remove the only link to the depth frame
    s = Session.open(synth["dir"])
    st, T = pipeline.read_trajectory_csv(synth["out"] / "slam" / "trajectory_opt.csv")
    with pytest.raises(RuntimeError, match="refusing to fuse"):
        fusion.FrameSet(s, ex, st, T, fusion.ReconConfig())


def test_ignoring_extrinsic_would_be_wrong(synth):
    """Guard for the guard: with the calibration transform dropped the map gets measurably worse."""
    ex = extract.Extracted(synth["out"])
    s = Session.open(synth["dir"])
    st, T = synth["gt"]["stamps_ns"], synth["gt"]["T_world_rgb"]   # ground-truth poses isolate the extrinsic effect
    fs = fusion.FrameSet(s, ex, st, T, fusion.ReconConfig(stride=6))
    good = []
    bad = []
    for n in range(len(fs.items)):
        f = fs.load(n)
        P = fusion.backproject(f["depth"], fs.cam_d, 4)
        good.append(eval_synth.surface_distance(P @ f["T_w_d"][:3, :3].T + f["T_w_d"][:3, 3]))
        Tw = fs.pose_c(fs.items[n][3])                          # pretend depth frame == rgb frame
        bad.append(eval_synth.surface_distance(P @ Tw[:3, :3].T + Tw[:3, 3]))
    assert np.median(np.concatenate(good)) < 0.006                  # depth noise only
    assert np.median(np.concatenate(bad)) > 3 * np.median(np.concatenate(good))


@pytest.mark.skipif(not o3d.core.cuda.is_available(), reason="no CUDA in Open3D")
def test_gpu_tsdf_matches_cpu(synth):
    from dataclasses import replace
    ex = extract.Extracted(synth["out"])
    s = Session.open(synth["dir"])
    st, T = pipeline.read_trajectory_csv(synth["out"] / "slam" / "trajectory_opt.csv")
    meshes = {}
    for b in ("tensor-cpu", "tensor-cuda"):
        cfg = fusion.ReconConfig(stride=4, backend=b)
        fs = fusion.FrameSet(s, ex, st, T, cfg)
        v = fusion.TSDF(cfg, fs.cam_d)
        for n in range(len(fs.items)):
            f = fs.load(n); v.integrate(f["depth"], f["T_w_d"])
        meshes[b] = v.mesh()
    a, b = meshes["tensor-cpu"], meshes["tensor-cuda"]
    assert abs(len(a.triangles) - len(b.triangles)) < 0.01 * len(a.triangles)
    d = np.asarray(o3d.geometry.PointCloud(a.vertices).compute_point_cloud_distance(b.sample_points_uniformly(300000)))
    assert np.percentile(d, 99) < 0.02


@pytest.mark.tum
@pytest.mark.slow
def test_public_dataset_tum_desk(tmp_path):
    from insight_mapping.fixtures import tum
    from conftest import TUM_DIR
    sdir, gt = tum.make_session(TUM_DIR, tmp_path, "session_tum", every=4)
    rep = run_mod.run_session(sdir, tmp_path / "out", gt_T=gt["T_world_cam"], allow_missing=("imu",), log=lambda *_: None)
    g = rep["trajectory"]["vs_ground_truth"]
    assert g["optimized"]["ate"]["rmse_m"] < 0.5 * g["vio"]["ate"]["rmse_m"]
    assert rep["outputs_verified"]["all_ok"]
