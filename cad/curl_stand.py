"""
Curl Stand — parametric Code-CAD reconstruction of the reference render.

The object is a single ribbon of constant thickness: it lies flat on the ground
(the base), rises at the back into a big round loop that curls forward, and the
lip tucks back in over the base. The side profile (the curl) is the only "design"
input — it is swept across the width and the side edges are filleted for the soft,
leather-like look. The central seam is a shallow groove on the outer surface.

Pipeline (no API calls, fully deterministic, re-runnable):
    centerline spline  ->  offset to a constant-thickness band (Face, XZ plane)
                       ->  extrude across the width (Y)
                       ->  fillet the side edges
                       ->  subtract a central seam groove
                       ->  export STL

Everything is parametric: tweak the millimetre constants or the CTRL points and
re-run.  Render an 8-view comparison sheet with `render_views.py`.

Coordinate frame:  X = depth (+X front / lip side),  Y = width,  Z = up.

Requires: build123d, numpy   (pip install build123d numpy)
Usage:    python curl_stand.py [output.stl]
"""
import os
import sys
import numpy as np
from build123d import (
    BuildPart, BuildSketch, Plane, Wire, Face, Spline, ShapeList,
    add, extrude, export_stl,
)

# ----------------------------- parameters (mm) -----------------------------
W = 46.0              # overall width (sweep length along Y)
THICKNESS = 4.0       # ribbon thickness
EDGE_ROUND = 1.5      # side-edge fillet radius (soft "leather" edges)

GROOVE = True         # central longitudinal seam on the outer surface
GROOVE_W = 2.2        # seam width (along Y)
GROOVE_DEPTH = 0.45   # how deep the seam bites into the surface

# Side-profile centerline control points (X = depth, Z = up).
# Flat base on the ground, then a big round loop curling forward, lip tucking in.
t = THICKNESS
CTRL = [
    (78, t / 2),  # front tip of the flat base (on the ground)
    (52, t / 2),  # base, front third
    (30, t / 2),  # base, under the loop's front
    (8,  t / 2),  # base, back edge
    (3,  12),     # back wall rising
    (2,  28),     # back of the loop
    (10, 42),     # top-back
    (28, 46),     # crest (round top)
    (46, 40),     # top-front, rolling down
    (54, 26),     # front of the loop coming down
    (52, 14),     # lip curling back in
    (42, 9),      # lip tip (hovers above the base)
]


# ------------------------------- 2D helpers --------------------------------
def catmull_rom(pts, samples_per_seg=24):
    """Smooth interpolating spline through the control points (for the centerline)."""
    p = np.asarray(pts, float)
    ext = np.vstack([p[0] + (p[0] - p[1]), p, p[-1] + (p[-1] - p[-2])])
    out = []
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        for s in np.linspace(0, 1, samples_per_seg, endpoint=False):
            s2, s3 = s * s, s * s * s
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * s
                              + (2 * p0 - 5 * p1 + 4 * p2 - p3) * s2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * s3))
    out.append(p[-1])
    return np.array(out)


def offset_band(centerline, half):
    """Parallel curves at +/- `half` along the centerline's left/right normals.
    Returns (left, right); `right` is the outer side of the curl."""
    d = np.gradient(centerline, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    n = np.column_stack([-d[:, 1], d[:, 0]])
    return centerline + n * half, centerline - n * half


def cap(a, b):
    """Flat (straight) end-cap from a to b. Flat caps keep the filleted B-rep's STL
    tessellation watertight; semicircular caps make OCCT emit degenerate corner
    patches at the tips, which leave micro-gaps in the mesh. The side-edge fillet
    still rounds the tip edges, so the ends read as softly rounded."""
    return np.array([np.asarray(a, float), np.asarray(b, float)])


def to_tuples(arr):
    return [(float(p[0]), float(p[1])) for p in arr]


def resample(P, n):
    idx = np.unique(np.linspace(0, len(P) - 1, n).round().astype(int))
    return P[idx]


# ------------------------------- 3D builders -------------------------------
def ribbon_face(centerline, half, n=60):
    """Smooth constant-thickness band as a Face in the XZ plane: two B-splines
    (left/right) joined by rounded arc end-caps."""
    lft, rgt = offset_band(centerline, half)
    lft, rgt = resample(lft, n), resample(rgt, n)
    ce, cs = cap(lft[-1], rgt[-1]), cap(rgt[0], lft[0])
    return Face(Wire([Spline(*to_tuples(lft)),
                      Spline(*to_tuples(ce)),
                      Spline(*to_tuples(rgt[::-1])),
                      Spline(*to_tuples(cs))]))


def side_faces(part):
    """The two flat width-end faces (at y = +/- W/2), selected by location."""
    return ShapeList([f for f in part.faces()
                      if abs(abs(f.center().Y) - W / 2) < 0.3])


def build():
    centerline = catmull_rom(CTRL)

    # main solid: extrude the smooth ribbon profile across the width
    with BuildPart() as part:
        with BuildSketch(Plane.XZ):
            add(ribbon_face(centerline, THICKNESS / 2))
        extrude(amount=W / 2, both=True)
    solid = part.part

    # soft rounded side edges; fall back to a smaller radius if OCCT refuses
    final, used_r = solid, 0.0
    for rr in [EDGE_ROUND, 1.2, 1.0, 0.8, 0.6, 0.4]:
        try:
            final = solid.fillet(rr, side_faces(solid).edges())
            used_r = rr
            break
        except Exception:
            continue

    # central seam: a shallow channel hugging the outer surface along the profile
    if GROOVE:
        _, c_in = offset_band(centerline, THICKNESS / 2 - GROOVE_DEPTH)
        _, c_out = offset_band(centerline, THICKNESS / 2 + 1.0)
        c_in, c_out = resample(c_in, 60), resample(c_out, 60)
        gce, gcs = cap(c_in[-1], c_out[-1]), cap(c_out[0], c_in[0])
        with BuildPart() as gpart:
            with BuildSketch(Plane.XZ):
                add(Face(Wire([Spline(*to_tuples(c_in)),
                               Spline(*to_tuples(gce)),
                               Spline(*to_tuples(c_out[::-1])),
                               Spline(*to_tuples(gcs))])))
            extrude(amount=GROOVE_W / 2, both=True)
        final = final - gpart.part

    return final, used_r


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "curl_stand.stl")
    part, used_r = build()
    export_stl(part, out, tolerance=0.02, angular_tolerance=0.1)
    size = part.bounding_box().size
    print(f"wrote {out}")
    print(f"  edge fillet : {used_r} mm")
    print(f"  bbox X,Y,Z  : {tuple(round(v, 1) for v in size)} mm")
    # Report mesh stats from the exported STL (accurate material volume + a printability
    # check). Optional: only if trimesh is installed.
    try:
        import trimesh
        m = trimesh.load(out, force="mesh")
        print(f"  material    : {round(m.volume / 1000, 1)} cm^3")
        print(f"  watertight  : {m.is_watertight}  (euler={m.euler_number}, "
              f"faces={len(m.faces)})")
    except Exception:
        pass
