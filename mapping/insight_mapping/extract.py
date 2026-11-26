"""P1-B: parse a Session's rosbag2 and extract RGB / stereo / depth / IMU / VIO / CameraInfo / TF.

Output layout (under <out>/extracted/), all stamps are ORIGINAL ROS header stamps in ns:
  rgb/ depth/ left/ right/ *.png   (depth = uint16 millimetres, 0 = invalid)
  <stream>_index.csv               stamp_ns,bag_ns,file,seq
  vio_image.csv, vio_100hz.csv     stamp_ns,bag_ns,tx,ty,tz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,parent,child
  imu.csv                          stamp_ns,bag_ns,gx,gy,gz,ax,ay,az
  camera_info.json  tf.json  extract_summary.json
"""
from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from . import bagio, se3
from .calib import CameraModel, TFTree
from .session import Session

IMAGE_STREAMS = ["rgb", "depth", "left", "right"]


def _tf_msgs(msg, tree: TFTree, static: bool):
    for t in msg.transforms:
        q = t.transform.rotation
        qa = np.array([q.x, q.y, q.z, q.w])
        qa = qa / np.linalg.norm(qa) if np.linalg.norm(qa) > 0 else np.array([0, 0, 0, 1.0])
        T = se3.make_T([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z], qa)
        parent, child = t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/")
        if static:
            tree.add_static(parent, child, T)
        else:
            tree.add_dynamic(parent, child, bagio.stamp_to_ns(t.header.stamp), T)


def extract(session: Session, out_dir, max_frames: int | None = None, every: int = 1, log=print) -> dict:
    out = Path(out_dir) / "extracted"
    out.mkdir(parents=True, exist_ok=True)
    reader = bagio.BagReader(session.bag_dir)
    topic_of = session.streams
    want = {v: k for k, v in topic_of.items()}
    topics = list(want)
    for k in IMAGE_STREAMS:
        (out / k).mkdir(exist_ok=True)

    idx = defaultdict(list)
    vio = defaultdict(list)
    imu_rows = []
    cams: dict[str, CameraModel] = {}
    tree = TFTree()
    img_meta: dict[str, dict] = {}
    counters = defaultdict(int)
    notes = []

    for raw, msg in reader.iter_msgs(topics):
        k = want[raw.topic]
        counters[k] += 1
        if k in IMAGE_STREAMS:
            if (counters[k] - 1) % every or (max_frames and len(idx[k]) >= max_frames):
                continue
            stamp = bagio.stamp_to_ns(msg.header.stamp)
            is_comp = raw.msg_type.endswith("CompressedImage")
            arr = bagio.compressed_to_numpy(msg) if is_comp else bagio.image_to_numpy(msg)
            enc = ("compressed:" + msg.format) if is_comp else msg.encoding
            if k == "depth":
                if is_comp:
                    arr = arr.astype(np.uint16) if arr.dtype != np.uint16 else arr
                else:
                    arr = bagio.depth_to_mm_u16(arr, msg.encoding)
                    if msg.encoding == "32FC1":
                        notes.append("depth 32FC1 metres converted to uint16 mm")
            elif k in ("left", "right"):
                arr = bagio.to_gray(arr, enc) if not is_comp else (arr if arr.ndim == 2 else cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY))
            else:
                arr = bagio.to_bgr(arr, enc) if not is_comp else arr
            fn = f"{k}/{stamp}.png"
            cv2.imwrite(str(out / fn), arr)
            idx[k].append((stamp, raw.bag_ns, fn, counters[k] - 1))
            img_meta.setdefault(k, {"width": int(arr.shape[1]), "height": int(arr.shape[0]), "encoding": enc,
                                    "frame_id": msg.header.frame_id})
        elif k.endswith("_info"):
            cams.setdefault(k, CameraModel.from_msg(msg))
        elif k in ("vio_image", "vio_100hz"):
            vio[k].append((raw.bag_ns, *bagio.odometry_to_row(msg)))
        elif k == "imu":
            imu_rows.append((raw.bag_ns, *bagio.imu_to_row(msg)))
        elif k == "tf":
            _tf_msgs(msg, tree, False)
        elif k == "tf_static":
            _tf_msgs(msg, tree, True)
    tree.finalize()

    for k, rows in idx.items():
        with open(out / f"{k}_index.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["stamp_ns", "bag_ns", "file", "seq"]); w.writerows(rows)
    for k, rows in vio.items():
        with open(out / f"{k}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stamp_ns", "bag_ns", "tx", "ty", "tz", "qx", "qy", "qz", "qw",
                        "vx", "vy", "vz", "wx", "wy", "wz", "parent", "child"])
            for bag_ns, st, par, ch, t, q, vl, va, _ in rows:
                w.writerow([st, bag_ns, *t, *q, *vl, *va, par, ch])
    if imu_rows:
        with open(out / "imu.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["stamp_ns", "bag_ns", "gx", "gy", "gz", "ax", "ay", "az", "frame_id"])
            for bag_ns, st, fid, g, a, _ in imu_rows:
                w.writerow([st, bag_ns, *g, *a, fid])
    (out / "camera_info.json").write_text(json.dumps({k: v.to_dict() for k, v in cams.items()}, indent=1))
    (out / "tf.json").write_text(json.dumps(tree.to_dict()))
    summary = {"messages_read": dict(counters), "images_extracted": {k: len(v) for k, v in idx.items()},
               "image_meta": img_meta, "streams": topic_of, "notes": sorted(set(notes))}
    (out / "extract_summary.json").write_text(json.dumps(summary, indent=1))
    log(f"extracted -> {out}  {summary['images_extracted']}")
    return summary


def _load_csv(path) -> dict:
    with open(path, newline="") as f:
        r = csv.reader(f)
        cols = next(r)
        rows = list(r)
    out = {}
    for j, c in enumerate(cols):
        vals = [row[j] for row in rows]
        try:
            out[c] = np.array(vals, dtype=np.float64) if c not in ("stamp_ns", "bag_ns", "seq") else np.array(vals, dtype=np.int64)
        except ValueError:
            out[c] = np.array(vals, dtype=object)
    return out


class Extracted:
    """Lazy accessor for an `extracted/` directory."""

    def __init__(self, work_dir):
        self.dir = Path(work_dir) / "extracted"
        if not self.dir.is_dir():
            raise FileNotFoundError(f"{self.dir} missing; run `insight-mapping extract` first")
        self.summary = json.loads((self.dir / "extract_summary.json").read_text())
        self.cams = {k: CameraModel.from_dict(v) for k, v in json.loads((self.dir / "camera_info.json").read_text()).items()}
        self.tf = TFTree.from_dict(json.loads((self.dir / "tf.json").read_text()))

    def index(self, k):
        p = self.dir / f"{k}_index.csv"
        if not p.exists():
            return None
        a = _load_csv(p)
        return {"stamp_ns": a["stamp_ns"], "bag_ns": a["bag_ns"], "file": [str(x) for x in a["file"]]}

    def vio(self, k):
        p = self.dir / f"{k}.csv"
        if not p.exists():
            return None
        a = _load_csv(p)
        T = np.stack([se3.make_T(t, q) for t, q in zip(np.stack([a["tx"], a["ty"], a["tz"]], 1),
                                                       np.stack([a["qx"], a["qy"], a["qz"], a["qw"]], 1))])
        return {"stamp_ns": a["stamp_ns"], "bag_ns": a["bag_ns"], "T": T,
                "v": np.stack([a["vx"], a["vy"], a["vz"]], 1), "parent": str(a["parent"][0]), "child": str(a["child"][0])}

    def imu(self):
        p = self.dir / "imu.csv"
        if not p.exists():
            return None
        a = _load_csv(p)
        return {"stamp_ns": a["stamp_ns"], "bag_ns": a["bag_ns"],
                "gyro": np.stack([a["gx"], a["gy"], a["gz"]], 1), "acc": np.stack([a["ax"], a["ay"], a["az"]], 1)}

    def read(self, file: str, flags=cv2.IMREAD_UNCHANGED):
        img = cv2.imread(str(self.dir / file), flags)
        if img is None:
            raise FileNotFoundError(self.dir / file)
        return img
