"""Camera models, TF tree and calibration consistency checks."""
from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np

from . import se3


@dataclass
class CameraModel:
    frame_id: str
    width: int
    height: int
    K: list  # 3x3 row-major flat (9)
    D: list
    distortion_model: str = "plumb_bob"
    P: list = field(default_factory=lambda: [0.0] * 12)
    R: list = field(default_factory=lambda: list(np.eye(3).ravel()))

    @property
    def K_mat(self) -> np.ndarray:
        return np.asarray(self.K, dtype=np.float64).reshape(3, 3)

    @property
    def P_mat(self) -> np.ndarray:
        return np.asarray(self.P, dtype=np.float64).reshape(3, 4)

    @property
    def fx(self): return self.K[0]
    @property
    def fy(self): return self.K[4]
    @property
    def cx(self): return self.K[2]
    @property
    def cy(self): return self.K[5]

    @property
    def has_distortion(self) -> bool:
        return bool(len(self.D) and np.any(np.abs(np.asarray(self.D, dtype=float)) > 1e-9))

    def to_dict(self) -> dict:
        return {k: (list(map(float, v)) if isinstance(v, (list, np.ndarray)) else v)
                for k, v in asdict(self).items()}

    @staticmethod
    def from_dict(d: dict) -> "CameraModel":
        return CameraModel(**d)

    @staticmethod
    def from_msg(msg) -> "CameraModel":
        return CameraModel(frame_id=msg.header.frame_id, width=int(msg.width), height=int(msg.height),
                           K=[float(x) for x in msg.k], D=[float(x) for x in msg.d],
                           distortion_model=msg.distortion_model, P=[float(x) for x in msg.p],
                           R=[float(x) for x in msg.r])

    def scaled(self, width: int, height: int) -> "CameraModel":
        sx, sy = width / self.width, height / self.height
        K = list(self.K); K[0] *= sx; K[2] *= sx; K[4] *= sy; K[5] *= sy
        P = list(self.P); P[0] *= sx; P[2] *= sx; P[3] *= sx; P[5] *= sy; P[6] *= sy
        return CameraModel(self.frame_id, width, height, K, self.D, self.distortion_model, P, self.R)


class TFTree:
    """Transform graph from /tf and /tf_static. Edges are stored as T_parent_child."""

    def __init__(self):
        self.static: dict[tuple[str, str], np.ndarray] = {}
        self.dynamic: dict[tuple[str, str], list] = defaultdict(list)  # (stamps, T) lists

    def add_static(self, parent: str, child: str, T: np.ndarray):
        self.static[(parent, child)] = T

    def add_dynamic(self, parent: str, child: str, stamp_ns: int, T: np.ndarray):
        self.dynamic[(parent, child)].append((stamp_ns, T))

    def finalize(self):
        self._dyn = {}
        for k, v in self.dynamic.items():
            v.sort(key=lambda x: x[0])
            self._dyn[k] = (np.array([s for s, _ in v], dtype=np.float64), np.stack([t for _, t in v]))

    def frames(self) -> set:
        s = set()
        for p, c in list(self.static) + list(self.dynamic):
            s.update((p, c))
        return s

    def _edge(self, key, stamp_ns):
        if key in self.static:
            return self.static[key]
        stamps, Ts = self._dyn[key]
        if stamp_ns is None or len(stamps) == 1:
            return Ts[-1] if stamp_ns is None else Ts[0]
        T, *_ = se3.interpolate_poses(stamps, Ts, np.array([stamp_ns], dtype=np.float64))
        return T[0]

    def can_lookup(self, target: str, source: str) -> bool:
        try:
            self._path(target, source)
            return True
        except KeyError:
            return False

    def _path(self, target, source):
        if target == source:
            return []
        adj = defaultdict(list)
        for (p, c) in list(self.static) + list(self.dynamic):
            adj[p].append((c, (p, c), False))   # p -> c : have T_p_c
            adj[c].append((p, (p, c), True))
        prev = {target: None}
        dq = deque([target])
        while dq:
            n = dq.popleft()
            if n == source:
                break
            for m, key, inverted in adj[n]:
                if m not in prev:
                    prev[m] = (n, key, inverted)
                    dq.append(m)
        if source not in prev:
            raise KeyError(f"no TF path {target} <- {source}")
        path, n = [], source
        while prev[n] is not None:
            p, key, inverted = prev[n]
            path.append((key, inverted))
            n = p
        return path[::-1]  # from target towards source

    def lookup(self, target: str, source: str, stamp_ns: Optional[int] = None) -> np.ndarray:
        """T_target_source: maps points in `source` into `target`."""
        T = np.eye(4)
        for key, inverted in self._path(target, source):
            E = self._edge(key, stamp_ns)
            T = T @ (se3.inv(E) if inverted else E)
        return T

    def is_static_path(self, target: str, source: str) -> bool:
        return all(k in self.static for k, _ in self._path(target, source))

    def to_dict(self) -> dict:
        return {"static": [[p, c, T.tolist()] for (p, c), T in self.static.items()],
                "dynamic": [[p, c, [[s, T.tolist()] for s, T in v]] for (p, c), v in self.dynamic.items()]}

    @staticmethod
    def from_dict(d: dict) -> "TFTree":
        t = TFTree()
        for p, c, T in d["static"]:
            t.add_static(p, c, np.array(T))
        for p, c, lst in d["dynamic"]:
            for s, T in lst:
                t.add_dynamic(p, c, int(s), np.array(T))
        t.finalize()
        return t


def stereo_baseline(left: CameraModel, right: CameraModel) -> Optional[float]:
    """Baseline from rectified projection matrices (P[0,3] = -fx*B for the right camera)."""
    pr = right.P_mat
    if abs(pr[0, 0]) < 1e-9 or abs(pr[0, 3]) < 1e-9:
        return None
    return float(-pr[0, 3] / pr[0, 0])


def check_camera(cam: CameraModel, image_wh: Optional[tuple], name: str) -> list:
    """Return list of (severity, message) findings for one camera's intrinsics."""
    out = []
    K = cam.K_mat
    if cam.width <= 0 or cam.height <= 0 or K[0, 0] <= 0 or K[1, 1] <= 0:
        out.append(("error", f"{name}: invalid intrinsics (fx={K[0,0]}, fy={K[1,1]}, {cam.width}x{cam.height}); "
                             "CameraInfo is likely uncalibrated/zeros"))
        return out
    if not (0 < K[0, 2] < cam.width and 0 < K[1, 2] < cam.height):
        out.append(("error", f"{name}: principal point ({K[0,2]:.1f},{K[1,2]:.1f}) outside {cam.width}x{cam.height}"))
    elif abs(K[0, 2] - cam.width / 2) > 0.25 * cam.width or abs(K[1, 2] - cam.height / 2) > 0.25 * cam.height:
        out.append(("warn", f"{name}: principal point far from image centre"))
    if abs(K[0, 0] / K[1, 1] - 1) > 0.05:
        out.append(("warn", f"{name}: fx/fy = {K[0,0]/K[1,1]:.3f} (non-square pixels?)"))
    if image_wh and (cam.width, cam.height) != tuple(image_wh):
        out.append(("error", f"{name}: CameraInfo size {cam.width}x{cam.height} != image size {image_wh[0]}x{image_wh[1]}"))
    return out


def rgbd_alignment(rgb: CameraModel, depth: CameraModel, T_rgb_depth: Optional[np.ndarray],
                   same_stamp_fraction: float = 1.0, tol_t=2e-3, tol_rot=0.2, tol_px=1.0) -> dict:
    """Decide whether depth can be treated as pixel-aligned with RGB. NEVER assume it.
    Aligned only if frame ids, size, intrinsics and the extrinsic (identity) all agree."""
    reasons = []
    if T_rgb_depth is None:
        reasons.append("no TF between rgb and depth frames: alignment cannot be verified")
    else:
        t = np.linalg.norm(T_rgb_depth[:3, 3])
        ang = np.degrees(se3.rot_angle(T_rgb_depth[:3, :3]))
        if t > tol_t:
            reasons.append(f"extrinsic translation {t*1000:.1f} mm")
        if ang > tol_rot:
            reasons.append(f"extrinsic rotation {ang:.2f} deg")
    if (rgb.width, rgb.height) != (depth.width, depth.height):
        reasons.append(f"size differs {rgb.width}x{rgb.height} vs {depth.width}x{depth.height}")
    d = np.abs(np.array([rgb.fx - depth.fx, rgb.fy - depth.fy, rgb.cx - depth.cx, rgb.cy - depth.cy]))
    if d.max() > tol_px:
        reasons.append("intrinsics differ (fx/fy/cx/cy)")
    return {"aligned": not reasons, "reasons": reasons}
