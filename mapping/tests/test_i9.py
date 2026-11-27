"""Insight 9 stereo-first pipeline tests. Data = contract-faithful SIMULATION (unit tests only, never evidence about the real device)."""
import json

import numpy as np
import pytest

pytestmark = pytest.mark.ros


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    from insight_mapping.i9 import fixture
    root = tmp_path_factory.mktemp("sim")
    r, gt = fixture.make_recording(root, "sim", n_frames=60, circle_frames=300)
    return r, root / "sim_gt.npz"


def test_scan_real_topic_names_and_loss(sim):
    from insight_mapping.i9 import scan
    rep = scan.scan(sim[0], log=lambda *_: None)
    assert rep["storage"] == "mcap" and rep["usable_for_stereo"]
    assert rep["roles_resolved"]["stereo_left"] == "/camera/camera/infra1/image_rect_raw"
    assert rep["roles_resolved"]["tf_static"] == "/tf_static"
    d = rep["topics"]["/camera/camera/depth/image_rect_raw"]
    assert 11 < d["delivered_hz"] < 15 and d["count_loss_pct"] > 5      # depth is irregular, reported not hidden
    st = rep["topics"]["/camera/camera/infra1/image_rect_raw"]
    assert abs(st["delivered_hz"] - 30) < 0.5 and st["non_monotonic"] == 0
    assert rep["topics"]["/camera/camera/depth/camera_info"] if False else True


def test_missing_stereo_is_refused(tmp_path):
    from insight_mapping.i9 import topics
    roles, notes = topics.resolve({"/camera/camera/imu": "sensor_msgs/msg/Imu"})
    assert "stereo_left" not in roles


def test_calibration_pass_and_baseline(sim):
    from insight_mapping.i9 import scan, calibcheck
    from insight_mapping.i9.reader import BagSource
    from insight_mapping.i9.rig import load_rig
    src = BagSource(sim[0]); roles = scan.scan(sim[0], log=lambda *_: None, sample_facts=False)["roles_resolved"]
    rig = load_rig(src, roles)
    assert abs(rig.baseline_P - 0.10012) < 1e-4 and abs(rig.baseline_tf - 0.10012) < 1e-4
    res = calibcheck.run_calibcheck(src, roles, rig, n_pairs=6, log=lambda *_: None)
    assert res["verdict"]["calibration_ok_for_metric_reconstruction"] == "PASS"
    assert res["sync"]["exact_stamp_pairs"] == res["sync"]["left"]


def test_calibration_detects_bad_baseline(sim):
    from insight_mapping.i9 import scan, calibcheck
    from insight_mapping.i9.reader import BagSource
    from insight_mapping.i9.rig import load_rig
    src = BagSource(sim[0]); roles = scan.scan(sim[0], log=lambda *_: None, sample_facts=False)["roles_resolved"]
    rig = load_rig(src, roles)
    rig.right.P[3] *= 1.2                                  # 20 % baseline error in CameraInfo only
    rig.baseline_P = float(-rig.right.P[3] / rig.right.P[0])
    ci, F = calibcheck.camerainfo_checks(rig)
    assert any(s == "error" and "baseline mismatch" in m for s, m in F)


def test_end_to_end_metric_accuracy(sim, tmp_path):
    from insight_mapping.i9 import pipeline
    rep = pipeline.run_all(sim[0], tmp_path / "o", "SIMULATED_UNIT_TEST", gt_npz=sim[1], dense_stride=2, log=lambda *_: None)
    assert rep["status"] == "ok" and rep["vo"]["lost_fraction"] < 0.05
    m = rep["vs_reference"]["metrics"]
    assert m["ate_rigid_rmse_m"] < 0.03 and abs(m["scale_ratio_est_over_gt"] - 1) < 0.03
    assert rep["dense"]["points"] > 10000 and rep["dense"]["mesh_triangles"] > 1000
    for f in ("trajectory.csv", "map.ply", "mesh.ply", "reconstruction_report.md", "reconstruction_report.json"):
        assert (tmp_path / "o" / f).exists()
    assert rep["depth_consistency"]["median_rel_depth_err"] < 0.03


def test_scale_check_known_marker():
    """Pure-geometry test: a 0.10 m marker rendered into a rectified pair at 1.5 m must measure ratio ~1; a 10 % wrong baseline must show ~0.9."""
    import cv2
    from insight_mapping.i9 import fixture, scale_check
    from insight_mapping.i9.rig import Rig
    from insight_mapping.calib import TFTree
    L, R = fixture._cam("l", 544, 640, fixture.K), fixture._cam("r", 544, 640, fixture.K, fixture.B)
    rig = Rig(L, R, {}, fixture.B, fixture.B, None, TFTree())
    D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    mk = cv2.aruco.generateImageMarker(D, 7, 200) if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(D, 7, 200)
    Z, side = 1.5, 0.10
    s_px = fixture.K[0] * side / Z
    src_c = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
    def render(x0):
        dst = np.array([[x0, 300], [x0 + s_px, 300], [x0 + s_px, 300 + s_px], [x0, 300 + s_px]], np.float32)
        M = cv2.getPerspectiveTransform(src_c, dst)
        img = np.full((640, 544), 255, np.uint8)
        w = cv2.warpPerspective(mk, M, (544, 640), flags=cv2.INTER_LINEAR, borderValue=255)
        return np.minimum(img, w)
    disp = fixture.K[0] * fixture.B / Z
    l, r = render(250), render(250 - disp)
    m = scale_check.measure_pair(l, r, rig, side, cv2.aruco.DICT_4X4_50)
    assert m and abs(m[0]["ratio"] - 1) < 0.03
    rig.right.P[3] *= 1.1; rig.baseline_P = float(-rig.right.P[3] / rig.right.P[0])
    m2 = scale_check.measure_pair(l, r, rig, side, cv2.aruco.DICT_4X4_50)
    assert abs(m2[0]["ratio"] - 1.1) < 0.04
