"""Texture mapping (view-atlas) and OBJ / GLB export.

Method: pick up to N well-spread, high-coverage RGB views (greedy on visible-triangle score), assign every triangle to its best
view (front-facing, z-buffer visible against the depth registered into that RGB camera, all 3 corners inside the image) and
use the undistorted RGB image itself as an atlas tile (UV = projected pixel). Mesh vertices are unwelded per triangle so each
triangle carries its own UVs. Triangles seen by no selected view fall onto a neutral tile (reported as `uncovered_fraction`).
RGB/Depth are never assumed aligned: visibility uses `FrameSet.load()['regd']` (depth warped into the colour camera through
the calibration extrinsic when not aligned) and the projection uses the RGB camera pose at the RGB stamp.
"""
from __future__ import annotations

import hashlib
import json
import struct
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .fusion import FrameSet


@dataclass
class TextureConfig:
    views: int = 24
    candidates: int = 80
    min_cos: float = 0.15
    depth_tol_abs: float = 0.04
    depth_tol_rel: float = 0.03
    max_tile_px: int = 960          # tiles are downscaled so the atlas stays <= ~8k px on a side


def _score_view(V, F, N_f, T_w_c, cam, regd, cfg):
    """-> (score per triangle (0 = not usable), uv (T,3,2) pixel coords)."""
    P_c = (V - T_w_c[:3, 3]) @ T_w_c[:3, :3]
    z = P_c[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = P_c[:, 0] * cam.fx / z + cam.cx
        v = P_c[:, 1] * cam.fy / z + cam.cy
    ok = (z > 0.1) & (u >= 1) & (u <= cam.width - 2) & (v >= 1) & (v <= cam.height - 2)
    ui, vi = np.clip(np.round(u).astype(int), 0, cam.width - 1), np.clip(np.round(v).astype(int), 0, cam.height - 1)
    zd = regd[vi, ui]
    vis = ok & (zd > 0) & (np.abs(zd - z) < cfg.depth_tol_abs + cfg.depth_tol_rel * z)
    tri_vis = vis[F].all(1)
    cen = V[F].mean(1)
    ray = T_w_c[:3, 3] - cen
    dist = np.linalg.norm(ray, axis=1)
    cosv = np.einsum("ij,ij->i", N_f, ray) / np.maximum(dist, 1e-9)
    cosv = np.abs(cosv)                                  # normals of a TSDF mesh are consistently oriented; abs() keeps this orientation-agnostic
    score = np.where(tri_vis & (cosv > cfg.min_cos), cosv / np.maximum(dist, 0.3), 0.0)
    uv = np.stack([u, v], 1)[F]
    return score, uv


def build_atlas(mesh, fs: FrameSet, cfg: TextureConfig | None = None, log=print) -> dict:
    cfg = cfg or TextureConfig()
    mesh.compute_vertex_normals(); mesh.compute_triangle_normals()
    V, F = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    N_f = np.asarray(mesh.triangle_normals)
    cand = np.unique(np.linspace(0, len(fs.items) - 1, min(cfg.candidates, len(fs.items))).astype(int))
    scores, uvs, colors = [], [], []
    for n in cand:
        f = fs.load(int(n))
        s, uv = _score_view(V, F, N_f, f["T_w_c"], fs.cam_c, f["regd"], cfg)
        scores.append(s.astype(np.float32)); uvs.append(uv.astype(np.float32)); colors.append(f["color"])
    S = np.stack(scores)                                  # (C, T)
    chosen, covered = [], np.zeros(len(F), np.float32)
    for _ in range(min(cfg.views, len(cand))):
        gain = np.maximum(S - covered[None], 0).sum(1)
        gain[chosen] = -1
        c = int(np.argmax(gain))
        if gain[c] <= 0:
            break
        chosen.append(c)
        covered = np.maximum(covered, S[c])
    if not chosen:
        raise RuntimeError("no view sees any triangle: cannot texture")
    best_local = np.argmax(S[chosen], axis=0)             # index in chosen
    best_score = S[chosen][best_local, np.arange(len(F))]
    tex_tri = best_score > 0
    # atlas layout
    H0, W0 = colors[0].shape[:2]
    scale = min(1.0, cfg.max_tile_px / max(H0, W0))
    tw, th = int(round(W0 * scale)), int(round(H0 * scale))
    k = len(chosen) + 1                                   # + neutral tile
    cols = int(np.ceil(np.sqrt(k))); rows = int(np.ceil(k / cols))
    atlas = np.full((rows * th, cols * tw, 3), 127, np.uint8)
    tiles = []
    for i, c in enumerate(chosen):
        img = colors[c]
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img = cv2.resize(img, (tw, th), interpolation=cv2.INTER_AREA) if scale != 1.0 else img
        r, cc = divmod(i, cols)
        atlas[r * th:(r + 1) * th, cc * tw:(cc + 1) * tw] = img
        tiles.append((cc * tw, r * th))
    nr, nc = divmod(len(chosen), cols)
    neutral = (nc * tw + tw // 2, nr * th + th // 2)
    Aw, Ah = atlas.shape[1], atlas.shape[0]
    # per-triangle UVs (pixels in atlas)
    uvp = np.zeros((len(F), 3, 2), np.float32)
    uvp[:] = np.array(neutral, np.float32)
    for i, c in enumerate(chosen):
        m = tex_tri & (best_local == i)
        if m.any():
            p = uvs[c][m]                                  # (m,3,2) in source pixels
            p = p * scale + np.array(tiles[i], np.float32)
            uvp[m] = p
    Vu = V[F].reshape(-1, 3)
    Nu = np.asarray(mesh.vertex_normals)[F].reshape(-1, 3)
    Fu = np.arange(len(Vu), dtype=np.uint32).reshape(-1, 3)
    UV = uvp.reshape(-1, 2)
    # pixel centre convention: pixel (i,j) centre at (i+0.5)/W
    UV_top = np.stack([(UV[:, 0] + 0.5) / Aw, (UV[:, 1] + 0.5) / Ah], 1).astype(np.float32)   # glTF: origin top-left
    return {"vertices": Vu.astype(np.float32), "normals": Nu.astype(np.float32), "faces": Fu, "uv_top": UV_top,
            "atlas_bgr": atlas, "views_used": [int(cand[c]) for c in chosen],
            "stats": {"triangles": int(len(F)), "textured_fraction": float(tex_tri.mean()), "uncovered_fraction": float(1 - tex_tri.mean()),
                      "views": len(chosen), "atlas_size": [int(Aw), int(Ah)], "tile_size": [tw, th]}}


def write_obj(path: Path, atlas: dict, name="mesh_textured"):
    path = Path(path)
    png = path.with_suffix(".png"); mtl = path.with_suffix(".mtl")
    cv2.imwrite(str(png), atlas["atlas_bgr"])
    mtl.write_text(f"newmtl mat0\nKa 1 1 1\nKd 1 1 1\nmap_Kd {png.name}\n")
    uv = atlas["uv_top"].copy(); uv[:, 1] = 1.0 - uv[:, 1]              # OBJ: origin bottom-left
    with open(path, "w") as f:
        f.write(f"mtllib {mtl.name}\nusemtl mat0\n")
        np.savetxt(f, atlas["vertices"], fmt="v %.6f %.6f %.6f")
        np.savetxt(f, uv, fmt="vt %.6f %.6f")
        np.savetxt(f, atlas["normals"], fmt="vn %.5f %.5f %.5f")
        idx = atlas["faces"].astype(np.int64) + 1
        np.savetxt(f, np.stack([idx[:, 0], idx[:, 0], idx[:, 0], idx[:, 1], idx[:, 1], idx[:, 1], idx[:, 2], idx[:, 2], idx[:, 2]], 1),
                   fmt="f %d/%d/%d %d/%d/%d %d/%d/%d")
    return [path, mtl, png]


def zup_to_yup(a: np.ndarray) -> np.ndarray:
    """(x, y, z) world (z up, REP-103) -> glTF (x, z, -y) (y up, right-handed)."""
    return np.stack([a[:, 0], a[:, 2], -a[:, 1]], 1)


def write_glb(path: Path, atlas: dict, z_up_to_y_up=True, generator="insight_mapping") -> dict:
    """Minimal glTF 2.0 binary: one mesh, POSITION/NORMAL/TEXCOORD_0, uint32 indices, embedded PNG baseColorTexture."""
    P, Nn, UV, F = atlas["vertices"], atlas["normals"], atlas["uv_top"], atlas["faces"].astype(np.uint32)
    if z_up_to_y_up:
        P, Nn = zup_to_yup(P), zup_to_yup(Nn)
    ok, png = cv2.imencode(".png", atlas["atlas_bgr"])
    png = png.tobytes()
    blobs = [("idx", F.tobytes()), ("pos", P.astype("<f4").tobytes()), ("nor", Nn.astype("<f4").tobytes()),
             ("uv", UV.astype("<f4").tobytes()), ("png", png)]
    off, views, o = {}, [], 0
    for name, b in blobs:
        pad = (-len(b)) % 4
        views.append({"buffer": 0, "byteOffset": o, "byteLength": len(b), **({"target": 34963} if name == "idx" else {"target": 34962} if name != "png" else {})})
        off[name] = len(views) - 1
        o += len(b) + pad
    binblob = b"".join(b + b"\0" * ((-len(b)) % 4) for _, b in blobs)
    n_v, n_i = len(P), F.size
    gltf = {
        "asset": {"version": "2.0", "generator": generator},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 1, "NORMAL": 2, "TEXCOORD_0": 3}, "indices": 0, "material": 0, "mode": 4}]}],
        "materials": [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}, "metallicFactor": 0.0, "roughnessFactor": 1.0},
                       "doubleSided": True}],
        "textures": [{"source": 0, "sampler": 0}], "samplers": [{"magFilter": 9729, "minFilter": 9729, "wrapS": 33071, "wrapT": 33071}],
        "images": [{"bufferView": off["png"], "mimeType": "image/png"}],
        "buffers": [{"byteLength": len(binblob)}], "bufferViews": views,
        "accessors": [
            {"bufferView": off["idx"], "componentType": 5125, "count": int(n_i), "type": "SCALAR", "max": [int(F.max())], "min": [int(F.min())]},
            {"bufferView": off["pos"], "componentType": 5126, "count": int(n_v), "type": "VEC3", "max": P.max(0).astype(float).tolist(), "min": P.min(0).astype(float).tolist()},
            {"bufferView": off["nor"], "componentType": 5126, "count": int(n_v), "type": "VEC3"},
            {"bufferView": off["uv"], "componentType": 5126, "count": int(n_v), "type": "VEC2"}],
        "extras": {"axis_conversion": "z-up -> y-up (x, z, -y)" if z_up_to_y_up else "none", "units": "metre"},
    }
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((-len(js)) % 4)
    total = 12 + 8 + len(js) + 8 + len(binblob)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A)); f.write(js)
        f.write(struct.pack("<II", len(binblob), 0x004E4942)); f.write(binblob)
    return {"vertices": int(n_v), "triangles": int(n_i // 3), "bytes": int(total), "png_bytes": len(png)}


def verify_glb(path: Path) -> dict:
    """Independent structural check of a GLB (header, chunk lengths, accessor ranges, indices, UV range, PNG decode)."""
    b = Path(path).read_bytes()
    magic, ver, length = struct.unpack_from("<III", b, 0)
    res = {"ok": False}
    if magic != 0x46546C67 or ver != 2 or length != len(b):
        res["error"] = "bad GLB header/length"; return res
    jl, jt = struct.unpack_from("<II", b, 12)
    g = json.loads(b[20:20 + jl])
    bo = 20 + jl
    bl, bt = struct.unpack_from("<II", b, bo)
    binb = b[bo + 8: bo + 8 + bl]
    if jt != 0x4E4F534A or bt != 0x004E4942:
        res["error"] = "bad chunk types"; return res

    def acc(i, dtype, ncomp):
        a = g["accessors"][i]; v = g["bufferViews"][a["bufferView"]]
        arr = np.frombuffer(binb, dtype=dtype, count=a["count"] * ncomp, offset=v["byteOffset"])
        return arr.reshape(-1, ncomp) if ncomp > 1 else arr
    pr = g["meshes"][0]["primitives"][0]
    idx = acc(pr["indices"], "<u4", 1); pos = acc(pr["attributes"]["POSITION"], "<f4", 3)
    uv = acc(pr["attributes"]["TEXCOORD_0"], "<f4", 2)
    v = g["bufferViews"][g["images"][0]["bufferView"]]
    img = cv2.imdecode(np.frombuffer(binb[v["byteOffset"]:v["byteOffset"] + v["byteLength"]], np.uint8), cv2.IMREAD_COLOR)
    res.update({"vertices": int(len(pos)), "triangles": int(len(idx) // 3), "texture_size": None if img is None else list(img.shape[:2][::-1]),
                "indices_in_range": bool(idx.max() < len(pos) and len(idx) % 3 == 0), "finite": bool(np.isfinite(pos).all()),
                "uv_in_0_1": bool(uv.min() >= -1e-4 and uv.max() <= 1 + 1e-4)})
    res["ok"] = bool(img is not None and res["indices_in_range"] and res["finite"] and res["uv_in_0_1"])
    return res
