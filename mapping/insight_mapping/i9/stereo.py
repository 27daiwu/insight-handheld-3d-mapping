"""Stage 3 - per-frame stereo processing, cached on disk (resumable, parallel).

For each synchronised (left,right) pair: SGBM disparity (uint16 = 16*px, 0 invalid), ORB features on the left image
with disparity sampled at keypoints. Cache layout under <work>/stereo/:
  index.csv                 stamp_ns,bag_ns
  <stamp>.left.png (uint8)  <stamp>.disp.png (uint16, disparity*16)  <stamp>.feat.npz (kp, des, disp)
"""
from __future__ import annotations

import csv
import multiprocessing as mp
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import cv2
import numpy as np

from ..bagio import fast_header, image_to_numpy
from .reader import BagSource


@dataclass
class StereoParams:
    num_disp: int = 128
    block: int = 5
    uniqueness: int = 8
    speckle_win: int = 60
    n_features: int = 1500
    fast_threshold: int = 10
    stride: int = 1
    max_frames: int = 0
    max_seconds: float = 0.0       # limit by header-stamp span from the first frame


def make_sgbm(p: StereoParams):
    b = p.block
    return cv2.StereoSGBM_create(0, p.num_disp, b, 8 * b * b, 32 * b * b, uniquenessRatio=p.uniqueness,
                                 speckleWindowSize=p.speckle_win, speckleRange=2, disp12MaxDiff=1, preFilterCap=31,
                                 mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)


def disparity(left: np.ndarray, right: np.ndarray, p: StereoParams) -> np.ndarray:
    d = make_sgbm(p).compute(left, right)            # int16, 16*disp, negative invalid
    d[d < 16] = 0                                    # disparity < 1 px carries no depth
    return d.astype(np.uint16)


def _work(args):
    stamp, l, r, p, out = args
    cv2.setNumThreads(1)
    d = disparity(l, r, p)
    orb = cv2.ORB_create(p.n_features, scaleFactor=1.2, nlevels=8, fastThreshold=p.fast_threshold)
    kps, des = orb.detectAndCompute(l, None)
    if des is None:
        kp = np.zeros((0, 2), np.float32); des = np.zeros((0, 32), np.uint8)
    else:
        kp = np.array([k.pt for k in kps], np.float32)
    ui = np.clip(np.round(kp[:, 0]).astype(int), 0, d.shape[1] - 1)
    vi = np.clip(np.round(kp[:, 1]).astype(int), 0, d.shape[0] - 1)
    dk = d[vi, ui].astype(np.float32) / 16.0
    cv2.imwrite(str(out / f"{stamp}.left.png"), l)
    cv2.imwrite(str(out / f"{stamp}.disp.png"), d)
    np.savez(out / f"{stamp}.feat.npz", kp=kp, des=des, disp=dk)
    return stamp, int(len(kp)), float((d > 0).mean())


def iter_pairs(src: BagSource, roles: dict, stats: dict):
    """Yield (stamp_ns, bag_ns, left, right) pairing by identical header stamp; bounded buffer."""
    tl, tr = roles["stereo_left"], roles["stereo_right"]
    pend_l: dict = {}; pend_r: dict = {}
    for raw in src.iter_raw([tl, tr]):
        s = fast_header(raw.data)[0]
        mine, other = (pend_l, pend_r) if raw.topic == tl else (pend_r, pend_l)
        if s in other:
            o = other.pop(s)
            a, b = (raw, o) if raw.topic == tl else (o, raw)
            stats["pairs"] += 1
            yield s, a.log_ns, image_to_numpy(src.decode(a)), image_to_numpy(src.decode(b))
        else:
            mine[s] = raw
        for buf in (pend_l, pend_r):               # drop orphans older than 0.5 s
            if len(buf) > 15:
                for k in sorted(buf)[:-15]:
                    buf.pop(k); stats["orphans"] += 1


def build_cache(src: BagSource, roles: dict, work, p: StereoParams, workers: int = 0, log=print) -> dict:
    out = Path(work) / "stereo"
    out.mkdir(parents=True, exist_ok=True)
    workers = workers or max(1, min(24, (mp.cpu_count() or 4) - 2))
    stats = {"pairs": 0, "orphans": 0}
    t0 = time.time()
    rows, res = [], []
    first = None
    pool = mp.get_context("fork").Pool(workers)
    try:
        def gen():
            nonlocal first
            k = 0
            for s, bag, l, r in iter_pairs(src, roles, stats):
                if first is None:
                    first = s
                if p.max_seconds and (s - first) / 1e9 > p.max_seconds:
                    return
                if k % p.stride == 0:
                    rows.append((s, bag))
                    yield (s, l, r, p, out)
                k += 1
                if p.max_frames and len(rows) >= p.max_frames:
                    return
        for rr in pool.imap(_work, gen(), chunksize=2):
            res.append(rr)
            if len(res) % 200 == 0:
                log(f"  stereo cache: {len(res)} frames, {len(res) / (time.time() - t0):.1f} fps")
    finally:
        pool.close(); pool.join()
    rows.sort()
    with open(out / "index.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["stamp_ns", "bag_ns"]); w.writerows(rows)
    nf = np.array([r[1] for r in res]); vf = np.array([r[2] for r in res])
    rep = {"frames": len(rows), "pairs_seen": stats["pairs"], "orphan_frames": stats["orphans"], "workers": workers,
           "params": asdict(p), "elapsed_s": time.time() - t0,
           "features_per_frame": {"median": float(np.median(nf)) if len(nf) else 0, "min": int(nf.min()) if len(nf) else 0},
           "valid_disparity_fraction": {"median": float(np.median(vf)) if len(vf) else 0, "p5": float(np.percentile(vf, 5)) if len(vf) else 0}}
    log(f"stereo cache: {rep['frames']} frames in {rep['elapsed_s']:.1f}s -> {out}")
    return rep


class StereoCache:
    def __init__(self, work):
        self.dir = Path(work) / "stereo"
        a = np.loadtxt(self.dir / "index.csv", delimiter=",", skiprows=1, dtype=np.int64, ndmin=2)
        self.stamps, self.bag = a[:, 0], a[:, 1]

    def __len__(self): return len(self.stamps)

    def feat(self, i):
        z = np.load(self.dir / f"{self.stamps[i]}.feat.npz")
        return z["kp"], z["des"], z["disp"]

    def left(self, i): return cv2.imread(str(self.dir / f"{self.stamps[i]}.left.png"), cv2.IMREAD_UNCHANGED)

    def disp(self, i): return cv2.imread(str(self.dir / f"{self.stamps[i]}.disp.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) / 16.0
