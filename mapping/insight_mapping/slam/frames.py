"""Coordinate-frame / transform-direction validation of VIO against pure vision.

Compares the relative camera motion predicted by VIO under several *hypotheses* (wrong direction of the pose,
wrong direction of the extrinsic, missing extrinsic) with the motion measured by RGB-D (or stereo) PnP.
The hypothesis with the smallest error is reported; anything other than "expected" is an error to fix BEFORE
feeding VIO to RTAB-Map.
"""
from __future__ import annotations

import numpy as np

from .. import se3
from ..calib import CameraModel
from ..extract import Extracted
from . import vio, vo

HYPOTHESES = ["expected: T_odom_body @ T_body_cam",
              "pose inverted: inv(T_odom_body) @ T_body_cam",
              "extrinsic inverted: T_odom_body @ inv(T_body_cam)",
              "extrinsic ignored: T_odom_body"]


def load_vo_inputs(ex: Extracted, session, i_rgb: int):
    """Returns (gray, depth_m registered into the colour/left camera, camera model, camera frame id) or None."""
    s = session.streams
    if "rgb" in s and "depth" in s:
        idx, didx = ex.index("rgb"), ex.index("depth")
        cam, dcam = ex.cams["rgb_info"], ex.cams["depth_info"]
        j = int(np.argmin(np.abs(didx["stamp_ns"] - idx["stamp_ns"][i_rgb])))
        gray = vo_gray(ex.read(idx["file"][i_rgb]))
        depth = ex.read(didx["file"][j]).astype(np.float32) / 1000.0
        if (cam.frame_id, cam.width, cam.height) != (dcam.frame_id, dcam.width, dcam.height) or \
                np.abs(np.array(cam.K) - np.array(dcam.K)).max() > 1.0:
            T = ex.tf.lookup(cam.frame_id, dcam.frame_id)
            depth = vo.register_depth_to_rgb(depth, dcam, cam, T)
        return gray, depth, cam, cam.frame_id, idx["stamp_ns"][i_rgb]
    if "left" in s and "right" in s:
        li, ri = ex.index("left"), ex.index("right")
        cam, rcam = ex.cams["left_info"], ex.cams["right_info"]
        from ..calib import stereo_baseline
        B = stereo_baseline(cam, rcam)
        j = int(np.argmin(np.abs(ri["stamp_ns"] - li["stamp_ns"][i_rgb])))
        left, right = ex.read(li["file"][i_rgb], 0), ex.read(ri["file"][j], 0)
        return left, vo.depth_from_stereo(left, right, cam.fx, B), cam, cam.frame_id, li["stamp_ns"][i_rgb]
    return None


def vo_gray(img):
    import cv2
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def _hyp_pose(T_ob, T_bc, h):
    if h == 0: return T_ob @ T_bc
    if h == 1: return se3.inv(T_ob) @ T_bc
    if h == 2: return T_ob @ se3.inv(T_bc)
    return T_ob


def validate_directions(ex: Extracted, session, n_pairs=25, gap=3, min_step_m=0.01, log=print) -> dict:
    key = "rgb" if "rgb" in session.streams else "left"
    idx = ex.index(key)
    n = len(idx["stamp_ns"])
    first = vo.make_frame  # noqa
    sel = np.linspace(0, n - gap - 1, min(n_pairs, max(n - gap, 1))).astype(int)
    vi = ex.vio("vio_image") or ex.vio("vio_100hz")
    child = vi["child"]
    rows = []
    for i in sel:
        a, b = load_vo_inputs(ex, session, i), load_vo_inputs(ex, session, i + gap)
        if a is None or b is None:
            break
        cam = a[2]
        Fa, Fb = vo.make_frame(a[0], a[1], cam), vo.make_frame(b[0], b[1], cam)
        T_ab, ninl, _ = vo.relative_pose(Fa, Fb, cam)
        if T_ab is None:
            continue
        T_ob, valid, *_ = vio.body_poses(ex, np.array([a[4], b[4]]))
        if not valid.all():
            continue
        T_bc = ex.tf.lookup(child, cam.frame_id)
        preds = []
        for h in range(len(HYPOTHESES)):
            Pa, Pb = _hyp_pose(T_ob[0], T_bc, h), _hyp_pose(T_ob[1], T_bc, h)
            rel = se3.relative(Pa, Pb)
            E = se3.inv(rel) @ T_ab
            preds.append((np.degrees(se3.rot_angle(E[:3, :3])), np.linalg.norm(E[:3, 3]), np.linalg.norm(rel[:3, 3])))
        rows.append({"i": int(i), "inliers": int(ninl), "vo_step_m": float(np.linalg.norm(T_ab[:3, 3])),
                     "vo_rot_deg": float(np.degrees(se3.rot_angle(T_ab[:3, :3]))), "err": preds})
    if len(rows) < 3:
        return {"status": "insufficient_data", "pairs": len(rows),
                "message": "fewer than 3 usable image pairs; cannot validate frame conventions"}
    moving = [r for r in rows if r["vo_step_m"] > min_step_m or r["vo_rot_deg"] > 1.0] or rows
    med = np.array([[np.median([r["err"][h][k] for r in moving]) for k in range(2)] for h in range(len(HYPOTHESES))])
    score = med[:, 0] / 5.0 + med[:, 1] / 0.05      # 5 deg ~ 5 cm
    best = int(np.argmin(score))
    res = {"status": "ok" if best == 0 and med[0, 0] < 5.0 and med[0, 1] < 0.08 else "MISMATCH",
           "pairs_used": len(moving), "best_hypothesis": HYPOTHESES[best],
           "hypotheses": [{"name": HYPOTHESES[h], "median_rot_err_deg": float(med[h, 0]),
                           "median_trans_err_m": float(med[h, 1])} for h in range(len(HYPOTHESES))],
           "median_vo_step_m": float(np.median([r["vo_step_m"] for r in moving]))}
    if res["status"] != "ok":
        res["message"] = (f"VIO relative motion best matches '{HYPOTHESES[best]}' (err {med[best,0]:.1f} deg / "
                          f"{med[best,1]*100:.1f} cm), not the expected convention. Fix before RTAB-Map.")
    log(f"frame validation: {res['status']} best={res['best_hypothesis']}  "
        f"(expected: {med[0,0]:.2f} deg, {med[0,1]*100:.1f} cm)")
    return res
