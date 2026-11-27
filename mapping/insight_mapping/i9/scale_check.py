"""Independent metric-scale check: triangulate an ArUco marker of KNOWN physical size from the stereo pair.

This is the only check in the pipeline that is not derived from the vendor calibration. Record the marker (printed at exactly
`--marker-size` metres, flat, ~0.5-3 m from the camera) for a few seconds; run `i9 scale-check <bag> --marker-size 0.10`.
ratio = measured edge length / true edge length; ~1.00 confirms CameraInfo baseline + focal length, otherwise the map scale is off by that factor.
"""
from __future__ import annotations

import cv2
import numpy as np

from ..bagio import fast_header, image_to_numpy
from . import scan as scanmod
from .reader import BagSource
from .rig import load_rig


def measure_pair(l, r, rig, marker_m, dictionary):
    d_ = cv2.aruco.getPredefinedDictionary(dictionary)
    if hasattr(cv2.aruco, "ArucoDetector"):                       # OpenCV >= 4.7
        det = cv2.aruco.ArucoDetector(d_, cv2.aruco.DetectorParameters())
        (cl, il, _), (cr, ir, _) = det.detectMarkers(l), det.detectMarkers(r)
    else:                                                          # OpenCV 4.5 (this host)
        prm = cv2.aruco.DetectorParameters_create()
        cl, il, _ = cv2.aruco.detectMarkers(l, d_, parameters=prm)
        cr, ir, _ = cv2.aruco.detectMarkers(r, d_, parameters=prm)
    out = []
    if il is None or ir is None:
        return out
    mr = {int(i): c for i, c in zip(ir.ravel(), cr)}
    for i, c in zip(il.ravel(), cl):
        if int(i) not in mr:
            continue
        a, b = c.reshape(4, 2), mr[int(i)].reshape(4, 2)
        d = a[:, 0] - b[:, 0]
        if (d <= 0.5).any() or np.abs(a[:, 1] - b[:, 1]).max() > 1.5:
            continue
        Z = rig.fx * rig.baseline / d
        P = np.stack([(a[:, 0] - rig.cx) * Z / rig.fx, (a[:, 1] - rig.cy) * Z / rig.fy, Z], 1)
        edges = np.linalg.norm(P - np.roll(P, -1, 0), axis=1)
        out.append({"id": int(i), "range_m": float(Z.mean()), "edge_m": edges.tolist(), "mean_edge_m": float(edges.mean()),
                    "ratio": float(edges.mean() / marker_m), "disparity_px": float(d.mean())})
    return out


def scale_check(path, marker_m, dictionary=None, overrides=None, max_pairs=300, log=print) -> dict:
    dictionary = cv2.aruco.DICT_4X4_50 if dictionary is None else dictionary
    sc = scanmod.scan(path, overrides, log=lambda *_: None, sample_facts=False)
    src = BagSource(path); roles = sc["roles_resolved"]; rig = load_rig(src, roles)
    tl, tr = roles["stereo_left"], roles["stereo_right"]
    pend, res, n = {}, [], 0
    for raw in src.iter_raw([tl, tr]):
        s = fast_header(raw.data)[0]
        if s in pend:
            a, b = (raw, pend.pop(s)) if raw.topic == tl else (pend.pop(s), raw)
            res += [{**m, "stamp_ns": int(s)} for m in measure_pair(image_to_numpy(src.decode(a)), image_to_numpy(src.decode(b)), rig, marker_m, dictionary)]
            n += 1
            if n >= max_pairs:
                break
        else:
            pend[s] = raw
    if not res:
        return {"pairs_examined": n, "detections": 0, "verdict": "NO_MARKER_FOUND"}
    r = np.array([m["ratio"] for m in res])
    med = float(np.median(r))
    rep = {"pairs_examined": n, "detections": len(res), "marker_size_m": marker_m, "median_ratio": med,
           "p10": float(np.percentile(r, 10)), "p90": float(np.percentile(r, 90)),
           "range_m": {"min": min(m["range_m"] for m in res), "max": max(m["range_m"] for m in res)},
           "verdict": "SCALE_CONFIRMED_(+-2%)" if abs(med - 1) < 0.02 else ("SCALE_PLAUSIBLE_(+-5%)" if abs(med - 1) < 0.05 else "SCALE_WRONG"),
           "note": "ratio 1.00 verifies CameraInfo focal length x baseline at the measured range only; the marker size you passed must be the printed black-square edge."}
    log(f"scale-check: median ratio {med:.4f} over {len(res)} detections")
    return rep
