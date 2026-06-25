"""
Headless orthographic 8-view renderer — lays an STL out like the reference sheet
(isometric, the three side views, top/back/bottom, reverse isometric).

No GPU/GL needed: trimesh loads the mesh, the triangles are projected
orthographically, depth-sorted (painter's algorithm) and flat-shaded with
matplotlib's PolyCollection. Good enough to compare silhouettes & proportions
against a reference image while iterating on the model.

World axes (must match curl_stand.py):
    X = depth (+X front / lip side),  Y = width,  Z = up.

Requires: trimesh, numpy, matplotlib
Usage:    python render_views.py [model.stl] [out.png]
"""
import sys
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

CLAY = np.array([0.80, 0.34, 0.21])   # reference-ish clay/leather colour
BG = (0.62, 0.62, 0.64)

# title, camera forward (cam -> object), up hint
VIEWS = [
    ("ISOMETRIC PERSPECTIVE", (-1, -1, -0.85), (0, 0, 1)),
    ("LEFT SIDE VIEW",        (0, 1, 0),       (0, 0, 1)),
    ("FRONT VIEW",            (-1, 0, 0),      (0, 0, 1)),
    ("RIGHT SIDE VIEW",       (0, -1, 0),      (0, 0, 1)),
    ("TOP VIEW",              (0, 0, -1),      (1, 0, 0)),
    ("BACK VIEW",             (1, 0, 0),       (0, 0, 1)),
    ("BOTTOM VIEW",           (0, 0, 1),       (-1, 0, 0)),
    ("REVERSE ISOMETRIC",     (1, 1, -0.85),   (0, 0, 1)),
]
GRID = {
    "ISOMETRIC PERSPECTIVE": (0, 1),
    "LEFT SIDE VIEW": (1, 0), "FRONT VIEW": (1, 1), "RIGHT SIDE VIEW": (1, 2),
    "TOP VIEW": (2, 0), "BACK VIEW": (2, 1), "BOTTOM VIEW": (2, 2),
    "REVERSE ISOMETRIC": (3, 1),
}


def look_R(forward, up):
    f = np.array(forward, float); f /= np.linalg.norm(f)
    zc = -f
    up = np.array(up, float)
    xc = np.cross(up, zc)
    if np.linalg.norm(xc) < 1e-9:
        xc = np.cross(np.array([1.0, 0, 0]), zc)
    xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    return np.vstack([xc, yc, zc])


def render_view(ax, V, F, N, forward, up):
    R = look_R(forward, up)
    Pc, Nc = V @ R.T, N @ R.T
    light = np.array([-0.3, -0.4, 1.0]); light /= np.linalg.norm(light)
    shade = np.clip(Nc @ light, 0, 1) * 0.75 + 0.25
    tri = Pc[F]
    order = np.argsort(tri[:, :, 2].mean(axis=1))          # far first
    polys = tri[order][:, :, :2]
    colors = np.clip(CLAY[None, :] * shade[order][:, None], 0, 1)
    ax.add_collection(PolyCollection(polys, facecolors=colors,
                                     edgecolors='none', antialiased=True))
    u, v = Pc[:, 0], Pc[:, 1]
    half = max(np.ptp(u), np.ptp(v)) / 2 * 1.12
    cu, cv = (u.min() + u.max()) / 2, (v.min() + v.max()) / 2
    ax.set_xlim(cu - half, cu + half); ax.set_ylim(cv - half, cv + half)
    ax.set_aspect('equal'); ax.axis('off')


def main(stl_path, out_path):
    m = trimesh.load(stl_path, force='mesh')
    V = np.asarray(m.vertices) - np.asarray(m.vertices).mean(axis=0)
    F, N = np.asarray(m.faces), np.asarray(m.face_normals)

    fig = plt.figure(figsize=(11, 14)); fig.patch.set_facecolor(BG)
    gs = fig.add_gridspec(4, 3, hspace=0.18, wspace=0.06)
    for title, fwd, up in VIEWS:
        rrow, ccol = GRID[title]
        ax = fig.add_subplot(gs[rrow, ccol]); ax.set_facecolor(BG)
        render_view(ax, V, F, N, fwd, up)
        ax.set_title(title, fontsize=11, color='black', fontfamily='monospace')
    fig.suptitle("CODE-CAD RECONSTRUCTION", fontsize=15,
                 fontfamily='monospace', y=0.96)
    fig.savefig(out_path, dpi=110, facecolor=BG, bbox_inches='tight')
    print("wrote", out_path)


if __name__ == "__main__":
    stl = sys.argv[1] if len(sys.argv) > 1 else "curl_stand.stl"
    out = sys.argv[2] if len(sys.argv) > 2 else "comparison.png"
    main(stl, out)
