import json
import struct

import cv2
import numpy as np
import pytest
import yaml

from insight_mapping import calib, calibfiles, se3
from insight_mapping.recon import texture


def test_rectify_is_noop_for_ideal_pinhole():
    cam = calib.CameraModel("c", 64, 48, [50, 0, 32, 0, 50, 24, 0, 0, 1], [0.0] * 5)
    img = np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8)
    assert calib.rectify(img, cam) is img


@pytest.mark.parametrize("model,D", [("plumb_bob", [-0.2, 0.05, 0.001, -0.001, 0.0]), ("equidistant", [0.05, -0.01, 0.002, 0.0, 0.0])])
def test_rectify_removes_distortion(model, D):
    """Project a plane of points with the distortion model; after rectify() they must lie on the pinhole projection."""
    W, H, f = 320, 240, 200.0
    cam = calib.CameraModel("c", W, H, [f, 0, W / 2, 0, f, H / 2, 0, 0, 1], D, model)
    pts = np.stack(np.meshgrid(np.linspace(-0.6, 0.6, 9), np.linspace(-0.4, 0.4, 7)), -1).reshape(-1, 2)
    P = np.concatenate([pts, np.ones((len(pts), 1))], 1)
    K = cam.K_mat
    if model == "equidistant":
        d, _ = cv2.fisheye.projectPoints(P[None].astype(np.float64), np.zeros(3), np.zeros(3), K, np.asarray(D[:4], float))
    else:
        d, _ = cv2.projectPoints(P.astype(np.float64), np.zeros(3), np.zeros(3), K, np.asarray(D, float))
    d = d.reshape(-1, 2)
    img = np.zeros((H, W), np.uint8)
    for u, v in d:
        cv2.circle(img, (int(round(u)), int(round(v))), 2, 255, -1)
    rect = calib.rectify(img, cam, cv2.INTER_LINEAR)
    ideal = (K @ P.T).T; ideal = ideal[:, :2] / ideal[:, 2:]
    for u, v in ideal:
        patch = rect[int(round(v)) - 3:int(round(v)) + 4, int(round(u)) - 3:int(round(u)) + 4]
        assert patch.max() > 100, f"no dot near rectified location {u:.1f},{v:.1f}"


def test_rectified_model_uses_P():
    cam = calib.CameraModel("c", 64, 48, [50, 0, 32, 0, 50, 24, 0, 0, 1], [0.1, 0, 0, 0, 0],
                            P=[60, 0, 31, -3.0, 0, 60, 23, 0, 0, 0, 1, 0])
    r = calib.rectified_model(cam)
    assert (r.fx, r.cx, r.cy) == (60, 31, 23) and not r.has_distortion


def test_calibration_folder_fallback_and_conflict(tmp_path):
    cal = tmp_path / "calibration"; cal.mkdir()
    (cal / "rgb_camera.yaml").write_text(yaml.safe_dump({
        "image_width": 64, "image_height": 48, "camera_name": "rgb_optical",
        "camera_matrix": {"rows": 3, "cols": 3, "data": [50, 0, 32, 0, 50, 24, 0, 0, 1]},
        "distortion_model": "plumb_bob", "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0] * 5}}))
    (cal / "left.json").write_text(json.dumps({"width": 64, "height": 48, "frame_id": "left_optical", "k": [40, 0, 32, 0, 40, 24, 0, 0, 1],
                                               "d": [], "p": [40, 0, 32, 0, 0, 40, 24, 0, 0, 0, 1, 0]}))
    (cal / "extr.yaml").write_text(yaml.safe_dump({"extrinsics": [
        {"parent": "body", "child": "rgb_optical", "translation": [0.1, 0, 0], "quaternion": [0, 0, 0, 1]}]}))
    (cal / "notes.txt").write_text("hello")
    tree = calib.TFTree(); tree.finalize()
    bag_left = calib.CameraModel("left_optical", 64, 48, [45, 0, 32, 0, 45, 24, 0, 0, 1], [])   # differs from the folder
    cams = {"left_info": bag_left}
    meta = {"rgb": {"frame_id": "rgb_optical"}, "left": {"frame_id": "left_optical"}}
    rep = calibfiles.apply_fallback(tmp_path, None, cams, tree, meta)
    assert "rgb_info" in cams and cams["rgb_info"].fx == 50                       # filled from folder
    assert cams["left_info"].fx == 45                                             # bag wins
    assert any("left" in m and "differs" in m for m in rep["mismatch"])
    assert np.allclose(tree.lookup("body", "rgb_optical")[:3, 3], [0.1, 0, 0])    # extrinsic filled
    assert "notes.txt" in rep["unrecognised"]


@pytest.fixture(scope="module")
def tiny_atlas():
    """A single plane mesh textured from one synthetic view."""
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh.create_box(1, 1, 0.01)
    V = np.asarray(mesh.vertices); V += [-0.5, -0.5, 2.0]; mesh.vertices = o3d.utility.Vector3dVector(V)
    cam = calib.CameraModel("c", 64, 48, [60, 0, 32, 0, 60, 24, 0, 0, 1], [0] * 5)
    color = np.zeros((48, 64, 3), np.uint8); color[..., 2] = 255          # pure red (BGR)
    regd = np.full((48, 64), 2.0, np.float32)
    f = {"T_w_c": np.eye(4), "regd": regd, "color": color}

    class FS:
        items = [0]; cam_c = cam
        def load(self, n): return f
    return texture.build_atlas(mesh, FS(), texture.TextureConfig(views=2, candidates=1, depth_tol_abs=0.1)), mesh


def test_texture_atlas_colours_and_uv(tiny_atlas):
    atlas, mesh = tiny_atlas
    assert atlas["stats"]["textured_fraction"] > 0.3
    uv = atlas["uv_top"]
    assert uv.min() >= 0 and uv.max() <= 1
    img = atlas["atlas_bgr"]
    H, W = img.shape[:2]
    px = img[np.clip((uv[:, 1] * H).astype(int), 0, H - 1), np.clip((uv[:, 0] * W).astype(int), 0, W - 1)]
    textured = (px[:, 2] > 200) & (px[:, 0] < 50)
    assert textured.mean() > 0.3                                              # UVs land on the red tile


def test_glb_roundtrip(tiny_atlas, tmp_path):
    atlas, _ = tiny_atlas
    p = tmp_path / "m.glb"
    info = texture.write_glb(p, atlas)
    v = texture.verify_glb(p)
    assert v["ok"] and v["triangles"] == info["triangles"] == len(atlas["faces"])
    b = p.read_bytes()
    assert struct.unpack_from("<I", b, 0)[0] == 0x46546C67 and len(b) == info["bytes"]
    g = json.loads(b[20:20 + struct.unpack_from("<I", b, 12)[0]])
    assert g["asset"]["version"] == "2.0" and g["extras"]["axis_conversion"].startswith("z-up")
    # axis conversion: a +z (up) point must become +y in glTF
    assert np.allclose(texture.zup_to_yup(np.array([[0, 0, 1.0]])), [[0, 1, 0]])
