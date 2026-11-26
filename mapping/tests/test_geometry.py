import numpy as np
from scipy.spatial.transform import Rotation as Rot

from insight_mapping import calib, evaluate, se3
from insight_mapping.slam import posegraph


def rand_T(rng):
    T = np.eye(4)
    T[:3, :3] = Rot.random(random_state=int(rng.integers(1 << 30))).as_matrix()
    T[:3, 3] = rng.normal(size=3)
    return T


def test_inverse_and_relative():
    rng = np.random.default_rng(0)
    A, B = rand_T(rng), rand_T(rng)
    assert np.allclose(se3.inv(A) @ A, np.eye(4))
    assert np.allclose(A @ se3.relative(A, B), B)


def test_pose_interpolation_exact_on_samples_and_midpoint():
    rng = np.random.default_rng(1)
    Ts = np.stack([rand_T(rng) for _ in range(5)])
    st = np.arange(5, dtype=float)
    T, *_ = se3.interpolate_poses(st, Ts, st)
    assert np.allclose(T, Ts, atol=1e-9)
    mid, *_ = se3.interpolate_poses(st, Ts, np.array([1.5]))
    assert np.allclose(mid[0, :3, 3], 0.5 * (Ts[1, :3, 3] + Ts[2, :3, 3]))


def test_umeyama_recovers_rigid():
    rng = np.random.default_rng(2)
    P = rng.normal(size=(50, 3))
    A = rand_T(rng)
    Q = P @ A[:3, :3].T + A[:3, 3]
    assert np.allclose(se3.umeyama_align(P, Q), A, atol=1e-9)
    assert evaluate.ate(np.tile(np.eye(4), (50, 1, 1)) * 0 + np.stack([se3.make_T(p, [0, 0, 0, 1]) for p in Q]),
                        np.stack([se3.make_T(p, [0, 0, 0, 1]) for p in P]))["rmse_m"] < 1e-9


def test_tf_lookup_direction_and_chain():
    rng = np.random.default_rng(3)
    A, B = rand_T(rng), rand_T(rng)
    tf = calib.TFTree()
    tf.add_static("odom", "body", A)
    tf.add_static("body", "cam", B)
    tf.finalize()
    assert np.allclose(tf.lookup("odom", "cam"), A @ B)
    assert np.allclose(tf.lookup("cam", "odom"), se3.inv(A @ B))      # reverse path
    assert np.allclose(tf.lookup("body", "cam"), B)
    assert not tf.can_lookup("odom", "nowhere")


def test_rgbd_alignment_never_assumed():
    f = 300.0
    cam = calib.CameraModel("c", 320, 240, [f, 0, 160, 0, f, 120, 0, 0, 1], [0] * 5)
    assert calib.rgbd_alignment(cam, cam, np.eye(4))["aligned"]
    assert not calib.rgbd_alignment(cam, cam, None)["aligned"]                       # unknown extrinsic
    T = np.eye(4); T[0, 3] = 0.025
    assert not calib.rgbd_alignment(cam, cam, T)["aligned"]                          # 25 mm offset
    other = calib.CameraModel("d", 256, 192, [240, 0, 128, 0, 240, 96, 0, 0, 1], [0] * 5)
    assert not calib.rgbd_alignment(cam, other, np.eye(4))["aligned"]                # size/intrinsics differ


def test_camera_checks_flag_uncalibrated():
    zero = calib.CameraModel("c", 640, 480, [0.0] * 9, [])
    assert any(s == "error" for s, _ in calib.check_camera(zero, (640, 480), "x"))
    ok = calib.CameraModel("c", 640, 480, [500, 0, 320, 0, 500, 240, 0, 0, 1], [])
    assert not calib.check_camera(ok, (640, 480), "x")
    assert any(s == "error" for s, _ in calib.check_camera(ok, (320, 240), "x"))      # size mismatch


def test_pose_graph_closes_loop():
    n = 40
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    true = np.stack([se3.make_T([np.cos(a), np.sin(a), 0], Rot.from_euler("z", a + np.pi / 2).as_quat()) for a in ang])
    drift = np.stack([se3.make_T([0.01 * k, 0, 0], Rot.from_euler("z", 0.01 * k).as_quat()) for k in range(n)])
    meas = drift @ true
    edges = [posegraph.Edge(i, i + 1, se3.relative(meas[i], meas[i + 1]), 0.02, 0.02) for i in range(n - 1)]
    edges.append(posegraph.Edge(0, n - 1, se3.relative(true[0], true[n - 1]), 0.02, 0.02, loop=True))
    opt, info = posegraph.optimize(meas, edges)
    def closure_err(T):
        E = se3.inv(se3.relative(true[0], true[n - 1])) @ se3.relative(T[0], T[n - 1])
        return np.linalg.norm(E[:3, 3])
    assert closure_err(opt) < 0.1 * closure_err(meas)                 # the loop constraint is honoured
    assert evaluate.ate(opt, true)["rmse_m"] < 0.7 * evaluate.ate(meas, true)["rmse_m"]   # error spread along the chain
    assert info["cost_after"] < info["cost_before"]
