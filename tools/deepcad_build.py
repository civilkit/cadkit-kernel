#!/usr/bin/env python3.12
"""Translate simple DeepCAD sketch-and-extrude sequences to our wire and
build the same solid with OCP for a volume reference.

Only single-sketch / single-extrude models with axis-aligned sketch planes,
Line3D/Circle3D/Arc3D curves, OneSide extents, no taper, and NewBody/Join/Cut
are translated. Everything else is reported as `skip` with a reason.

    python3.12 tools/deepcad_build.py --sample ~/.cache/b-rep-corpora/deepcad/sample \
        --out tools/deepcad_cases.json
"""
import argparse
import glob
import json
import math
import os

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.GC import GC_MakeCircle
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Dir, gp_Pnt, gp_Vec

TAU = 2 * math.pi


def vec(d):
    return (float(d.get("x", 0)), float(d.get("y", 0)), float(d.get("z", 0)))


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 0 else (0, 0, 0)


def world(t, p):
    o = vec(t.get("origin", {}))
    x = vec(t.get("x_axis", {}))
    y = vec(t.get("y_axis", {}))
    z = vec(t.get("z_axis", {}))
    return add(o, add(add(mul(x, p[0]), mul(y, p[1])), mul(z, p[2])))


def axis(v):
    for i in range(3):
        if abs(v[i]) > 0.999:
            return i
    return -1


def std_frame(normal):
    """(name, x, y, z, sign) for a positive/negative world-axis normal."""
    ax = axis(normal)
    if ax < 0:
        return None
    sign = 1 if normal[ax] > 0 else -1
    if ax == 2:
        return ("front", (1, 0, 0), (0, 1, 0), (0, 0, 1), sign)
    if ax == 1:
        return ("top", (1, 0, 0), (0, 0, -1), (0, 1, 0), sign)
    return ("right", (0, 0, -1), (0, 1, 0), (1, 0, 0), sign)


def curve_points(c, t):
    """World-space endpoints/centre for a DeepCAD curve."""
    if c["type"] == "Line3D":
        return world(t, vec(c["start_point"])), world(t, vec(c["end_point"]))
    if c["type"] == "Circle3D":
        return world(t, vec(c["center_point"])), None
    if c["type"] == "Arc3D":
        return world(t, vec(c["start_point"])), world(t, vec(c["end_point"]))
    return None, None


def ocp_edge(c, t):
    typ = c["type"]
    if typ == "Line3D":
        p1, p2 = curve_points(c, t)
        if norm((p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])) < 1e-12:
            return None
        return BRepBuilderAPI_MakeEdge(gp_Pnt(*p1), gp_Pnt(*p2)).Edge()
    if typ == "Circle3D":
        ctr = world(t, vec(c["center_point"]))
        n = unit(vec(c.get("normal", {"x": 0, "y": 0, "z": 1})))
        circ = GC_MakeCircle(gp_Pnt(*ctr), gp_Dir(*n), float(c["radius"])).Value()
        return BRepBuilderAPI_MakeEdge(circ).Edge()
    if typ == "Arc3D":
        ctr = world(t, vec(c["center_point"]))
        n = unit(vec(c.get("normal", {"x": 0, "y": 0, "z": 1})))
        circ = GC_MakeCircle(gp_Pnt(*ctr), gp_Dir(*n), float(c["radius"])).Value()
        a0 = float(c["start_angle"])
        a1 = float(c["end_angle"])
        if a1 <= a0:
            a1 += TAU
        return BRepBuilderAPI_MakeEdge(circ, a0, a1).Edge()
    return None


def loop_points(c, t):
    if c["type"] == "Line3D":
        return [world(t, vec(c["start_point"])), world(t, vec(c["end_point"]))]
    if c["type"] == "Circle3D":
        ctr = world(t, vec(c["center_point"]))
        n = unit(vec(c.get("normal", {"x": 0, "y": 0, "z": 1})))
        x = unit(cross(n, (0, 0, 1))) if abs(n[2]) < 0.9 else (1, 0, 0)
        y = unit(cross(n, x))
        r = float(c["radius"])
        return [add(ctr, add(mul(x, r * math.cos(TAU * k / 16)), mul(y, r * math.sin(TAU * k / 16)))) for k in range(17)]
    if c["type"] == "Arc3D":
        ctr = world(t, vec(c["center_point"]))
        n = unit(vec(c.get("normal", {"x": 0, "y": 0, "z": 1})))
        x = unit(cross(n, (0, 0, 1))) if abs(n[2]) < 0.9 else (1, 0, 0)
        y = unit(cross(n, x))
        r = float(c["radius"])
        a0 = float(c["start_angle"])
        a1 = float(c["end_angle"])
        if a1 <= a0:
            a1 += TAU
        return [add(ctr, add(mul(x, r * math.cos(a0 + (a1 - a0) * k / 16)), mul(y, r * math.sin(a0 + (a1 - a0) * k / 16)))) for k in range(17)]
    return []


def loop_area(loop, t):
    pts = []
    for c in loop["profile_curves"]:
        pts.extend(loop_points(c, t))
    if len(pts) < 3:
        return 0.0
    # project onto the sketch plane normal
    nx = unit(vec(t.get("z_axis", {"x": 0, "y": 0, "z": 1})))
    ox = vec(t.get("origin", {}))
    x = unit(vec(t.get("x_axis", {"x": 1, "y": 0, "z": 0})))
    y = unit(vec(t.get("y_axis", {"x": 0, "y": 1, "z": 0})))
    a = 0.0
    for i in range(len(pts) - 1):
        p = (dot((pts[i][0] - ox[0], pts[i][1] - ox[1], pts[i][2] - ox[2]), x),
             dot((pts[i][0] - ox[0], pts[i][1] - ox[1], pts[i][2] - ox[2]), y))
        q = (dot((pts[i + 1][0] - ox[0], pts[i + 1][1] - ox[1], pts[i + 1][2] - ox[2]), x),
             dot((pts[i + 1][0] - ox[0], pts[i + 1][1] - ox[1], pts[i + 1][2] - ox[2]), y))
        a += p[0] * q[1] - q[0] * p[1]
    return a / 2.0


def make_face(loops, t):
    wires = []
    for loop in loops:
        mk = BRepBuilderAPI_MakeWire()
        for c in loop["profile_curves"]:
            e = ocp_edge(c, t)
            if e is None:
                return None, f"bad edge {c['type']}"
            mk.Add(e)
        if not mk.IsDone():
            return None, "wire not closed"
        wires.append(mk.Wire())
    if not wires:
        return None, "no wires"
    faces = []
    for w in wires:
        mk = BRepBuilderAPI_MakeFace(w)
        if not mk.IsDone():
            return None, "face not made"
        faces.append(mk.Face())
    areas = []
    for f in faces:
        gp = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, gp); areas.append(abs(gp.Mass()))
    oi = max(range(len(faces)), key=lambda i: areas[i])
    outer = faces[oi]
    for i, f in enumerate(faces):
        if i != oi:
            op = BRepAlgoAPI_Cut(outer, f); op.Build()
            if not op.IsDone():
                return None, "hole cut failed"
            outer = op.Shape()
    return outer, None


def build_occt(sk, ex):
    t = sk.get("transform", {})
    prof = ex["profiles"][0]["profile"]
    loops = sk["profiles"][prof]["loops"]
    face, err = make_face(loops, t)
    if face is None:
        return None, err
    d = float(ex["extent_one"]["distance"]["value"])
    n = unit(vec(t.get("z_axis", {"x": 0, "y": 0, "z": 1})))
    solid = BRepPrimAPI_MakePrism(face, gp_Vec(*(mul(n, d)))).Shape()
    return solid, None


def part_features(sk, ex):
    t = sk.get("transform", {})
    z = unit(vec(t.get("z_axis", {"x": 0, "y": 0, "z": 1})))
    ax = axis(z)
    if ax < 0:
        return None, "sketch normal is not axis-aligned"
    sign = 1 if z[ax] > 0 else -1
    if ax == 2:
        name, sx, sy, sz = "front", (1, 0, 0), (0, 1, 0), (0, 0, 1)
    elif ax == 1:
        name, sx, sy, sz = "top", (1, 0, 0), (0, 0, -1), (0, 1, 0)
    else:
        name, sx, sy, sz = "right", (0, 0, -1), (0, 1, 0), (1, 0, 0)
    flip = sign < 0
    if flip:
        sx = (-sx[0], -sx[1], -sx[2])
        sz = (-sz[0], -sz[1], -sz[2])
    origin = vec(t.get("origin", {}))
    offset = dot(origin, sz)
    feats = []
    plane = {"kind": "std", "id": name}
    if flip or abs(offset) > 1e-12:
        pid = "P1"
        feats.append({"type": "plane", "id": pid, "mode": "offset", "base": {"kind": "std", "id": name}, "offset": offset, "flip": flip, "visible": False})
        plane = {"kind": "feature", "id": pid}
    # profile to 2D
    prof = ex["profiles"][0]["profile"]
    loops = sk["profiles"][prof]["loops"]
    points = []
    entities = []
    pmap = {}
    def pid(p):
        key = (round(p[0], 12), round(p[1], 12))
        if key not in pmap:
            i = len(pmap) + 1
            pmap[key] = f"p{i}"
            points.append({"id": pmap[key], "x": p[0], "y": p[1]})
        return pmap[key]
    def to2(pw):
        q = (pw[0] - offset * sz[0], pw[1] - offset * sz[1], pw[2] - offset * sz[2])
        return (dot(q, sx), dot(q, sy))
    n = 0
    for loop in loops:
        for c in loop["profile_curves"]:
            n += 1
            if c["type"] == "Line3D":
                p1 = to2(world(t, vec(c["start_point"])))
                p2 = to2(world(t, vec(c["end_point"])))
                if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 1e-12:
                    continue
                entities.append({"type": "line", "id": f"e{n}", "p1": pid(p1), "p2": pid(p2)})
            elif c["type"] == "Circle3D":
                ctr = to2(world(t, vec(c["center_point"])))
                entities.append({"type": "circle", "id": f"e{n}", "c": pid(ctr), "r": float(c["radius"])})
            elif c["type"] == "Arc3D":
                ctr = to2(world(t, vec(c["center_point"])))
                p1 = to2(world(t, vec(c["start_point"])))
                p2 = to2(world(t, vec(c["end_point"])))
                entities.append({"type": "arc", "id": f"e{n}", "c": pid(ctr), "p1": pid(p1), "p2": pid(p2)})
            else:
                return None, f"curve {c['type']}"
    if not entities:
        return None, "empty sketch"
    feats.append({"type": "sketch", "id": "S1", "plane": plane, "points": points, "entities": entities, "constraints": []})
    op = ex.get("operation")
    if op == "CutFeatureOperation":
        eop, flip = "cut", True
    elif op in ("JoinFeatureOperation", "NewBodyFeatureOperation"):
        eop, flip = "add", False
    else:
        return None, f"operation {op}"
    depth = float(ex["extent_one"]["distance"]["value"])
    feats.append({"type": "extrude", "id": "F1", "op": eop, "sketch": "S1", "end": "blind", "depth": depth, "flip": flip})
    return {"format": "ck-part", "version": 1, "features": feats}, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sample", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=200)
    args = ap.parse_args()
    files = sorted(glob.glob(os.path.join(args.sample, "**", "*.json"), recursive=True))[: args.limit]
    cases, skips = {}, {}
    for f in files:
        j = json.load(open(f))
        es = j.get("entities", {})
        sketches = [e for e in es.values() if e.get("type") == "Sketch"]
        extrudes = [e for e in es.values() if e.get("type") == "ExtrudeFeature"]
        name = os.path.basename(f)
        if len(sketches) != 1 or len(extrudes) != 1:
            skips["not single sketch+extrude"] = skips.get("not single sketch+extrude", 0) + 1
            continue
        ex = extrudes[0]
        if len(ex.get("profiles", [])) != 1:
            skips["multi-profile"] = skips.get("multi-profile", 0) + 1
            continue
        if ex.get("extent_type") != "OneSideFeatureExtentType":
            skips["not OneSide"] = skips.get("not OneSide", 0) + 1
            continue
        if ex.get("operation") not in ("NewBodyFeatureOperation", "JoinFeatureOperation", "CutFeatureOperation"):
            skips["operation"] = skips.get("operation", 0) + 1
            continue
        if abs(float(ex.get("extent_one", {}).get("taper_angle", {}).get("value", 0) or 0)) > 1e-9:
            skips["taper"] = skips.get("taper", 0) + 1
            continue
        sk = sketches[0]
        try:
            solid, err = build_occt(sk, ex)
            if solid is None:
                skips[f"occt: {err}"] = skips.get(f"occt: {err}", 0) + 1
                continue
            p = GProp_GProps(); BRepGProp.VolumeProperties_s(solid, p); vol = p.Mass()
            a = GProp_GProps(); BRepGProp.SurfaceProperties_s(solid, a); area = a.Mass()
            part, err = part_features(sk, ex)
            if part is None:
                skips[f"wire: {err}"] = skips.get(f"wire: {err}", 0) + 1
                continue
            cases[name] = {"part": part, "occt": {"volume": vol, "area": area}}
        except Exception as e:
            skips[f"exception: {type(e).__name__}"] = skips.get(f"exception: {type(e).__name__}", 0) + 1
    json.dump(cases, open(args.out, "w"), indent=1)
    print(json.dumps({"translated": len(cases), "skipped": len(files) - len(cases), "reasons": skips}, indent=1))


if __name__ == "__main__":
    main()
