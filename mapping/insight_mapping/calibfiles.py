"""Calibration-folder parsing (`calibration/` of a Session) and the *fallback* policy.

DATA_CONTRACT §6: calibration in the bag (CameraInfo/TF) is authoritative, the folder is a convenience copy. We therefore
use the folder only (a) to fill what the bag lacks and (b) to cross-check what the bag has.

Accepted camera files : ROS camera_info YAML (camera_calibration_parsers / camera_info_manager) and JSON dumps of
                        sensor_msgs/CameraInfo (keys width,height,k,d,r,p,distortion_model[,frame_id]).
Accepted extrinsics   : YAML/JSON  {extrinsics: [{parent, child, translation:[x,y,z], quaternion:[x,y,z,w]} | {parent, child, T:[[4x4]]}]}
Both are optional; unknown files are listed in the report, never guessed.
"""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from . import se3
from .calib import CameraModel, TFTree


def _load(path: Path):
    txt = path.read_text()
    return json.loads(txt) if path.suffix.lower() == ".json" else yaml.safe_load(txt)


def parse_camera_file(path: Path) -> Optional[CameraModel]:
    try:
        d = _load(path)
    except Exception:
        return None
    if not isinstance(d, dict):
        return None
    try:
        if "camera_matrix" in d:                                     # camera_info_manager YAML
            g = lambda k, n: [float(x) for x in d.get(k, {}).get("data", [0.0] * n)]
            return CameraModel(frame_id=str(d.get("frame_id", d.get("camera_name", ""))), width=int(d["image_width"]),
                               height=int(d["image_height"]), K=g("camera_matrix", 9), D=g("distortion_coefficients", 5),
                               distortion_model=d.get("distortion_model", "plumb_bob"), P=g("projection_matrix", 12),
                               R=g("rectification_matrix", 9) if "rectification_matrix" in d else list(np.eye(3).ravel()))
        if "k" in d and "width" in d:                                # CameraInfo message dump
            return CameraModel(frame_id=str(d.get("frame_id", d.get("header", {}).get("frame_id", ""))), width=int(d["width"]),
                               height=int(d["height"]), K=[float(x) for x in d["k"]], D=[float(x) for x in d.get("d", [])],
                               distortion_model=d.get("distortion_model", "plumb_bob"),
                               P=[float(x) for x in d.get("p", [0.0] * 12)],
                               R=[float(x) for x in d.get("r", np.eye(3).ravel())])
    except (KeyError, ValueError, TypeError):
        return None
    return None


def parse_extrinsics_file(path: Path) -> list:
    try:
        d = _load(path)
    except Exception:
        return []
    out = []
    for e in (d or {}).get("extrinsics", []) if isinstance(d, dict) else []:
        try:
            if "T" in e:
                T = np.asarray(e["T"], dtype=float).reshape(4, 4)
            else:
                q = np.asarray(e["quaternion"], dtype=float)
                T = se3.make_T(e["translation"], q / np.linalg.norm(q))
            out.append((str(e["parent"]).lstrip("/"), str(e["child"]).lstrip("/"), T))
        except (KeyError, ValueError, TypeError):
            continue
    return out


def scan_folder(session_dir: Path, listed=None) -> dict:
    cal = Path(session_dir) / "calibration"
    files = [cal / f for f in (listed or [])] if listed else sorted(p for p in cal.glob("*") if p.is_file()) if cal.is_dir() else []
    cams, ext, unknown = {}, [], []
    for f in files:
        if not f.exists():
            unknown.append(f"{f.name} (listed but missing)"); continue
        if f.suffix.lower() not in (".yaml", ".yml", ".json"):
            unknown.append(f.name); continue
        c = parse_camera_file(f)
        if c is not None:
            cams[f.stem] = c
            continue
        e = parse_extrinsics_file(f)
        if e:
            ext += e
        else:
            unknown.append(f.name)
    return {"cameras": cams, "extrinsics": ext, "unrecognised": unknown}


def match_camera(cams: dict, key: str, frame_id: str) -> Optional[CameraModel]:
    """Find the folder camera for logical stream `key` (rgb/depth/left/right): by frame_id first, then by file name."""
    for name, c in cams.items():
        if frame_id and c.frame_id == frame_id:
            return c
    rx = {"rgb": "rgb|color", "depth": "depth", "left": "left", "right": "right"}[key]
    hits = [c for n, c in cams.items() if re.search(rx, n, re.I)]
    return hits[0] if len(hits) == 1 else None


def apply_fallback(session_dir: Path, listed, cams: dict, tree: TFTree, image_meta: dict) -> dict:
    """Fill missing CameraInfo / TF edges from the calibration folder, cross-check the rest. Returns a report."""
    sc = scan_folder(session_dir, listed)
    rep = {"used": [], "mismatch": [], "unrecognised": sc["unrecognised"], "folder_cameras": sorted(sc["cameras"])}
    for key in ("rgb", "depth", "left", "right"):
        if key not in image_meta:
            continue
        fid = image_meta[key].get("frame_id", "")
        folder = match_camera(sc["cameras"], key, fid)
        bag = cams.get(f"{key}_info")
        if folder is None:
            continue
        if bag is None:
            cams[f"{key}_info"] = folder
            rep["used"].append(f"{key}_info from calibration/ (bag had no CameraInfo)")
        else:
            dk = max(abs(a - b) for a, b in zip(bag.K, folder.K))
            if dk > 0.5 or (bag.width, bag.height) != (folder.width, folder.height):
                rep["mismatch"].append(f"{key}: calibration/ differs from bag CameraInfo (max |dK|={dk:.2f} px / size) — bag is used")
    frames_known = tree.frames()
    for p, c, T in sc["extrinsics"]:
        if (p, c) in tree.static or not tree.can_lookup(p, c):
            if not tree.can_lookup(p, c):
                tree.add_static(p, c, T)
                rep["used"].append(f"extrinsic {p}->{c} from calibration/ (no TF path in bag)")
        else:
            T_bag = tree.lookup(p, c)
            if np.linalg.norm(T_bag[:3, 3] - T[:3, 3]) > 2e-3 or np.degrees(se3.rot_angle(T_bag[:3, :3].T @ T[:3, :3])) > 0.3:
                rep["mismatch"].append(f"extrinsic {p}->{c}: calibration/ differs from bag TF — bag is used")
    tree.finalize()
    return rep
