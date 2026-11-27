"""Stereo rig description built from CameraInfo + /tf_static (the only geometric inputs of phase 1)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .. import se3
from ..calib import CameraModel, TFTree
from .reader import BagSource


@dataclass
class Rig:
    left: CameraModel
    right: CameraModel
    topics: dict
    baseline_P: Optional[float]                 # -P_right[0,3]/P_right[0,0]  (m)
    baseline_tf: Optional[float]                # |t| of left->right static transform (m)
    T_left_right: Optional[np.ndarray]
    tf: TFTree
    info_variants: dict = field(default_factory=dict)   # CameraInfo content changes during the recording
    other_cams: dict = field(default_factory=dict)

    @property
    def fx(self): return self.left.P[0] if self.left.P[0] > 0 else self.left.fx
    @property
    def fy(self): return self.left.P[5] if self.left.P[5] > 0 else self.left.fy
    @property
    def cx(self): return self.left.P[2] if self.left.P[0] > 0 else self.left.cx
    @property
    def cy(self): return self.left.P[6] if self.left.P[0] > 0 else self.left.cy

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    @property
    def baseline(self) -> float:
        """Baseline used for metric depth: CameraInfo P (the quantity stereo algorithms consume)."""
        if self.baseline_P is None:
            raise ValueError("right CameraInfo has P[0,3]=0: baseline unknown")
        return self.baseline_P


def _tf_from_msg(msg, tree: TFTree):
    for t in msg.transforms:
        q = t.transform.rotation
        qa = np.array([q.x, q.y, q.z, q.w], float)
        n = np.linalg.norm(qa)
        qa = qa / n if n > 0 else np.array([0, 0, 0, 1.0])
        T = se3.make_T([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z], qa)
        tree.add_static(t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/"), T)


def load_rig(src: BagSource, roles: dict, info_sample_every: int = 30) -> Rig:
    need = [roles[k] for k in ("stereo_left_info", "stereo_right_info") if k in roles]
    if len(need) < 2:
        raise ValueError("stereo CameraInfo topics missing")
    topics = list(need) + ([roles["tf_static"]] if "tf_static" in roles else [])
    cams: dict = {}
    variants: dict = {need[0]: set(), need[1]: set()}
    n = {need[0]: 0, need[1]: 0}
    tree = TFTree()
    for raw, msg in src.iter_msgs(topics):
        if raw.topic in n:
            n[raw.topic] += 1
            cam = CameraModel.from_msg(msg)
            cams.setdefault(raw.topic, cam)
            if (n[raw.topic] - 1) % info_sample_every == 0:
                variants[raw.topic].add((tuple(np.round(cam.K, 6)), tuple(np.round(cam.P, 6)), tuple(np.round(cam.D, 8)), tuple(np.round(cam.R, 8)), cam.width, cam.height))
        else:
            _tf_from_msg(msg, tree)
    tree.finalize()
    L, R = cams[need[0]], cams[need[1]]
    bP = None
    if abs(R.P[0]) > 1e-9 and abs(R.P[3]) > 1e-9:
        bP = float(-R.P[3] / R.P[0])
    Tlr, btf = None, None
    if "tf_static" in roles and tree.can_lookup(L.frame_id, R.frame_id):
        Tlr = tree.lookup(L.frame_id, R.frame_id)
        btf = float(np.linalg.norm(Tlr[:3, 3]))
    return Rig(L, R, {"left_info": need[0], "right_info": need[1]}, bP, btf, Tlr, tree,
               {k: len(v) for k, v in variants.items()})
