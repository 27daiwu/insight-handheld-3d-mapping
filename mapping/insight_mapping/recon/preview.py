"""Software preview of a textured OBJ (no GPU/GL needed): triangles are splatted at their centroids with the colour the
atlas gives at the centroid UV (z-buffered). Used for visual sanity checks and tests, not as a renderer.
(Open3D's OffscreenRenderer returned all-black images on this headless host, so it is not relied upon.)"""
from __future__ import annotations

import cv2
import numpy as np
import open3d as o3d


def atlas_colors(obj_path, png_path):
    m = o3d.io.read_triangle_mesh(str(obj_path), enable_post_processing=False)
    atlas = cv2.imread(str(png_path))[:, :, ::-1]
    H, W = atlas.shape[:2]
    uv = np.asarray(m.triangle_uvs).reshape(-1, 3, 2).mean(1)               # o3d: origin bottom-left
    x = np.clip((uv[:, 0] * W).astype(int), 0, W - 1); y = np.clip(((1 - uv[:, 1]) * H).astype(int), 0, H - 1)
    V, F = np.asarray(m.vertices), np.asarray(m.triangles)
    return V[F].mean(1), atlas[y, x]


def render(cen, col, eye, target, up=(0, 0, 1), size=(800, 600), fov_deg=60.0, splat=2):
    f = (target - eye); f = f / np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r); u = np.cross(r, f)
    P = cen - eye
    x, y, z = P @ r, -(P @ u), P @ f
    m = z > 0.1
    fx = size[0] / (2 * np.tan(np.radians(fov_deg) / 2))
    px = (fx * x[m] / z[m] + size[0] / 2).astype(int); py = (fx * y[m] / z[m] + size[1] / 2).astype(int)
    zz, cc = z[m], col[m]
    img = np.zeros((size[1], size[0], 3), np.uint8)
    for i in np.argsort(-zz):
        if 0 <= px[i] < size[0] and 0 <= py[i] < size[1]:
            img[max(py[i] - splat, 0):py[i] + splat + 1, max(px[i] - splat, 0):px[i] + splat + 1] = cc[i]
    return img
