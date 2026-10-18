#!/usr/bin/env python3.12
"""Translate DeepCAD sketch/extrude sequences to our wire and build the same
solid with OCP for a volume reference.

Supported now: multi-feature sequences, multi-profile extrudes and
OneSide / TwoSides / Symmetric extents, with axis-aligned sketch planes,
Line3D/Circle3D/Arc3D curves, no taper, and NewBody/Join/Cut operations.
Anything else is `skip` with a reason.

    python3.12 tools/deepcad_build.py --sample DIR --out cases.json --limit 1000
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
from OCP.GC import GC_MakeArcOfCircle, GC_MakeCircle
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Vec
from OCP.ShapeExtend import ShapeExtend_WireData
from OCP.ShapeFix import ShapeFix_Wire
from OCP.ShapeFix import ShapeFix_Shape

TAU = 2 * math.pi


def vec(d):
    return (float(d.get("x", 0)), float(d.get("y", 0)), float(d.get("z", 0)))


def addp(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a): return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 0 else (0, 0, 0)


def world(t, p):
    o = vec(t.get("origin", {}))
    x = vec(t.get("x_axis", {}))
    y = vec(t.get("y_axis", {}))
    z = vec(t.get("z_axis", {}))
    return addp(o, addp(addp(mul(x, p[0]), mul(y, p[1])), mul(z, p[2])))


def rot_vec(t, v):
    x = vec(t.get("x_axis", {"x": 1, "y": 0, "z": 0}))
    y = vec(t.get("y_axis", {"x": 0, "y": 1, "z": 0}))
    z = vec(t.get("z_axis", {"x": 0, "y": 0, "z": 1}))
    return addp(addp(mul(x, v[0]), mul(y, v[1])), mul(z, v[2]))


def axis(v):
    for i in range(3):
        if abs(v[i]) > 0.999:
            return i
    return -1


def std_frame(normal):
    ax = axis(normal)
    if ax < 0:
        return None
    sign = 1 if normal[ax] > 0 else -1
    if ax == 2:
        return ("front", (1, 0, 0), (0, 1, 0), (0, 0, 1), sign)
    if ax == 1:
        return ("top", (1, 0, 0), (0, 0, -1), (0, 1, 0), sign)
    return ("right", (0, 0, -1), (0, 1, 0), (1, 0, 0), sign)


def ocp_edge(c, t):
    typ = c["type"]
    if typ == "Line3D":
        p1 = world(t, vec(c["start_point"]))
        p2 = world(t, vec(c["end_point"]))
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
        a0, a1 = float(c["start_angle"]), float(c["end_angle"])
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
        return [addp(ctr, addp(mul(x, r * math.cos(TAU * k / 16)), mul(y, r * math.sin(TAU * k / 16)))) for k in range(17)]
    if c["type"] == "Arc3D":
        ctr = world(t, vec(c["center_point"]))
        n = unit(vec(c.get("normal", {"x": 0, "y": 0, "z": 1})))
        x = unit(cross(n, (0, 0, 1))) if abs(n[2]) < 0.9 else (1, 0, 0)
        y = unit(cross(n, x))
        r = float(c["radius"])
        a0, a1 = float(c["start_angle"]), float(c["end_angle"])
        if a1 <= a0:
            a1 += TAU
        return [addp(ctr, addp(mul(x, r * math.cos(a0 + (a1 - a0) * k / 16)), mul(y, r * math.sin(a0 + (a1 - a0) * k / 16)))) for k in range(17)]
    return []


def loop_tol(loops, t):
    pts = []
    for loop in loops:
        for c in loop["profile_curves"]:
            if c["type"] == "Line3D":
                pts += [world(t, vec(c["start_point"])), world(t, vec(c["end_point"]))]
            elif c["type"] in ("Circle3D", "Arc3D"):
                pts.append(world(t, vec(c["center_point"])))
    if not pts:
        return 1e-9
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    diag = math.sqrt(sum((hi[i] - lo[i]) ** 2 for i in range(3)))
    return max(1e-3 * diag, 1e-9)


def conditioned_curves(loop, t):
    """Snap near-coincident endpoints and rebuild arc endpoints on the mean
    radius. Returns curve dicts in world coordinates with shared endpoints."""
    raw = []
    for c in loop["profile_curves"]:
        if c["type"] == "Line3D":
            raw.append({"type": "line", "a": world(t, vec(c["start_point"])), "b": world(t, vec(c["end_point"]))})
        elif c["type"] == "Circle3D":
            raw.append({"type": "circle", "c": world(t, vec(c["center_point"])), "r": abs(float(c["radius"])), "n": unit(vec(c.get("normal", {"x": 0, "y": 0, "z": 1})))})
        elif c["type"] == "Arc3D":
            ctr = world(t, vec(c["center_point"]))
            n = unit(rot_vec(t, vec(c.get("normal", {"x": 0, "y": 0, "z": 1}))))
            ref = rot_vec(t, vec(c.get("reference_vector", {"x": 1, "y": 0, "z": 0})))
            ref = unit(addp(ref, mul(n, -dot(ref, n))))
            if norm(ref) < 1e-9:
                ref = unit(cross(n, (0, 0, 1)))
            # reference vectors that are within ~0.03 rad of a world axis are
            # data noise on a normalised model; snap them to the axis
            for ax in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                d = dot(ref, ax)
                if abs(d) > 0.9995:
                    ref = mul(ax, 1.0 if d > 0 else -1.0)
                    break
            yv = unit(cross(n, ref))
            a0, a1 = float(c["start_angle"]), float(c["end_angle"])
            if a1 <= a0:
                a1 += TAU
            S = world(t, vec(c["start_point"])); E = world(t, vec(c["end_point"]))
            r = 0.5 * (norm((S[0] - ctr[0], S[1] - ctr[1], S[2] - ctr[2])) + norm((E[0] - ctr[0], E[1] - ctr[1], E[2] - ctr[2])))
            A = addp(ctr, addp(mul(ref, r * math.cos(a0)), mul(yv, r * math.sin(a0))))
            B = addp(ctr, addp(mul(ref, r * math.cos(a1)), mul(yv, r * math.sin(a1))))
            raw.append({"type": "arc", "c": ctr, "r": r, "a": A, "b": B, "a0": a0, "a1": a1, "ref": ref, "n": n, "yv": yv})
    pts = [p for e in raw for p in ([e["a"], e["b"]] if e["type"] in ("line", "arc") else [])]
    if not pts:
        return raw
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    diag = math.sqrt(sum((hi[i] - lo[i]) ** 2 for i in range(3)))
    tol = max(1e-3 * diag, 1e-9)
    canon = []
    def snap(p):
        for q in canon:
            if norm((p[0] - q[0], p[1] - q[1], p[2] - q[2])) <= tol:
                return q
        canon.append(p)
        return p
    for e in raw:
        if e["type"] in ("line", "arc"):
            e["a"] = snap(e["a"]); e["b"] = snap(e["b"])
    return raw


def ocp_edge_cond(e):
    if e["type"] == "line":
        return BRepBuilderAPI_MakeEdge(gp_Pnt(*e["a"]), gp_Pnt(*e["b"])).Edge()
    if e["type"] == "circle":
        circ = GC_MakeCircle(gp_Pnt(*e["c"]), gp_Dir(*e["n"]), e["r"]).Value()
        return BRepBuilderAPI_MakeEdge(circ).Edge()
    if e["type"] == "arc":
        am = 0.5 * (e["a0"] + e["a1"])
        mid = addp(e["c"], addp(mul(e["ref"], e["r"] * math.cos(am)), mul(e["yv"], e["r"] * math.sin(am))))
        try:
            return BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(gp_Pnt(*e["a"]), gp_Pnt(*e["b"]), gp_Pnt(*mid)).Value()).Edge()
        except Exception:
            circ = GC_MakeCircle(gp_Pnt(*e["c"]), gp_Dir(*e["n"]), e["r"]).Value()
            return BRepBuilderAPI_MakeEdge(circ, e["a0"], e["a1"]).Edge()
    return None


def make_faces(loops, t):
    wires = []
    tol = loop_tol(loops, t)
    for loop in loops:
        edges = []
        for c in conditioned_curves(loop, t):
            e = ocp_edge_cond(c)
            if e is None:
                return None, f"bad edge {c['type']}"
            edges.append(e)
        mk = BRepBuilderAPI_MakeWire()
        for e in edges:
            mk.Add(e)
        if mk.IsDone():
            wires.append(mk.Wire())
            continue
        # relative-tolerance fallback for small gaps in normalised DeepCAD data
        wd = ShapeExtend_WireData()
        for e in edges:
            wd.Add(e)
        fix = ShapeFix_Wire()
        fix.Load(wd)
        fix.SetPrecision(tol)
        fix.FixReorder()
        fix.FixConnected()
        fix.FixClosed()
        w = fix.Wire()
        if w.IsNull():
            return None, "wire not closed"
        wires.append(w)
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
    order = sorted(range(len(faces)), key=lambda i: -areas[i])
    outers = []
    for i in order:
        f = faces[i]
        placed = False
        for j, outer in enumerate(outers):
            op = BRepAlgoAPI_Cut(outer, f); op.Build()
            if op.IsDone() and op.Shape().IsNull() is False:
                # keep it as a separate outer if the cut is empty/unchanged is hard to test;
                # use the bounding boxes: a hole must be inside the outer face's box.
                from OCP.Bnd import Bnd_Box
                from OCP.BRepBndLib import BRepBndLib
                ob, fb = Bnd_Box(), Bnd_Box()
                BRepBndLib.Add_s(outer, ob); BRepBndLib.Add_s(f, fb)
                ox = ob.CornerMin(), ob.CornerMax(); fx = fb.CornerMin(), fb.CornerMax()
                inside = all(ox[0].Coord(i) - 1e-9 <= fx[0].Coord(i) and fx[1].Coord(i) <= ox[1].Coord(i) + 1e-9 for i in range(1, 4))
                if inside:
                    op = BRepAlgoAPI_Cut(outer, f); op.Build()
                    if op.IsDone():
                        outers[j] = op.Shape(); placed = True
                        break
        if not placed:
            outers.append(f)
    return outers, None


def prism(face, t, depth, sign):
    n = unit(vec(t.get("z_axis", {"x": 0, "y": 0, "z": 1})))
    return BRepPrimAPI_MakePrism(face, gp_Vec(*mul(n, depth * sign))).Shape()


def combine(cur, solid, op):
    if cur is None:
        return solid
    mk = BRepAlgoAPI_Fuse if op in ("NewBodyFeatureOperation", "JoinFeatureOperation") else BRepAlgoAPI_Cut
    b = mk(cur, solid); b.Build()
    return b.Shape() if b.IsDone() else None


def build_occt(j):
    es = j["entities"]
    cur = None
    for step in j.get("sequence", []):
        if step.get("type") != "ExtrudeFeature":
            continue
        ex = es.get(step.get("entity"))
        if not ex:
            continue
        if not ex.get("profiles"):
            return None, "no profiles"
        sk = es.get(ex["profiles"][0]["sketch"])
        if not sk:
            return None, "missing sketch"
        t = sk.get("transform", {})
        d1 = float(ex["extent_one"]["distance"]["value"])
        d2 = float(ex.get("extent_two", {}).get("distance", {}).get("value", 0) or 0)
        et = ex.get("extent_type")
        if et == "OneSideFeatureExtentType":
            dirs = [(abs(d1), 1 if d1 >= 0 else -1)]
        elif et == "TwoSidesFeatureExtentType":
            dirs = [(abs(d1), 1 if d1 >= 0 else -1), (abs(d2), 1 if d2 >= 0 else -1)]
        else:
            dirs = [(abs(d1), 1), (abs(d1), -1)]
        for p in ex.get("profiles", []):
            sk = es.get(p["sketch"])
            t = sk.get("transform", {})
            prof = p["profile"]
            if prof not in sk.get("profiles", {}):
                return None, f"missing profile {prof}"
            faces, err = make_faces(sk["profiles"][prof]["loops"], t)
            if faces is None:
                return None, err
            for face in faces:
                for depth, sign in dirs:
                    solid = prism(face, t, depth, sign)
                    cur = combine(cur, solid, ex.get("operation"))
                    if cur is None:
                        return None, "boolean failed"
    return cur, None


def loop_is_cw(loop, t):
    pts = []
    for c in loop["profile_curves"]:
        if c["type"] == "Line3D":
            pts.append(world(t, vec(c["start_point"])))
        elif c["type"] == "Arc3D":
            pts.append(world(t, vec(c["start_point"])))
        else:
            return None
    if len(pts) < 3:
        return None
    a = 0.0
    for i in range(len(pts)):
        p, q = pts[i], pts[(i + 1) % len(pts)]
        a += p[0] * q[1] - q[0] * p[1]
    return a < 0


def emit_sketch(sk, profile, ctx):
    key = (id(sk), profile)
    if key in ctx["sketch_ids"]:
        return ctx["sketch_ids"][key]
    t = sk.get("transform", {})
    z = unit(vec(t.get("z_axis", {"x": 0, "y": 0, "z": 1})))
    fr = std_frame(z)
    if fr is None:
        return None, "sketch normal not axis-aligned"
    name, sx, sy, sz, sign = fr
    flip = sign < 0
    if flip:
        sx = (-sx[0], -sx[1], -sx[2]); sz = (-sz[0], -sz[1], -sz[2])
    origin = vec(t.get("origin", {}))
    offset = dot(origin, sz)
    plane = {"kind": "std", "id": name}
    if flip or abs(offset) > 1e-12:
        pid = f"P{ctx['nextP']}"; ctx["nextP"] += 1
        ctx["feats"].append({"type": "plane", "id": pid, "mode": "offset", "base": {"kind": "std", "id": name}, "offset": offset, "flip": flip, "visible": False})
        plane = {"kind": "feature", "id": pid}
    sid = f"S{ctx['nextS']}"; ctx["nextS"] += 1
    points, entities, pmap = [], [], {}

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
    for loop in sk["profiles"][profile]["loops"]:
        for c in conditioned_curves(loop, t):
            n += 1
            if c["type"] == "line":
                p1 = to2(c["a"]); p2 = to2(c["b"])
                if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 1e-12:
                    continue
                entities.append({"type": "line", "id": f"e{n}", "p1": pid(p1), "p2": pid(p2)})
            elif c["type"] == "circle":
                ctr = to2(c["c"])
                entities.append({"type": "circle", "id": f"e{n}", "c": pid(ctr), "r": c["r"]})
            elif c["type"] == "arc":
                ctr = to2(c["c"])
                p1 = to2(c["a"]); p2 = to2(c["b"])
                entities.append({"type": "arc", "id": f"e{n}", "c": pid(ctr), "p1": pid(p1), "p2": pid(p2)})
            else:
                return None, f"curve {c['type']}"
    if not entities:
        return None, "empty sketch"
    ctx["feats"].append({"type": "sketch", "id": sid, "plane": plane, "points": points, "entities": entities, "constraints": []})
    ctx["sketch_ids"][key] = sid
    return sid, None


def emit_extrude(ctx, sid, op, depth, sign):
    flip = (op == "CutFeatureOperation") == (sign > 0)
    fid = f"F{ctx['nextF']}"; ctx["nextF"] += 1
    eop = "cut" if op == "CutFeatureOperation" else "add"
    ctx["feats"].append({"type": "extrude", "id": fid, "op": eop, "sketch": sid, "end": "blind", "depth": depth, "flip": flip})


def build_part(j):
    es = j["entities"]
    ctx = {"feats": [], "nextS": 1, "nextF": 1, "nextP": 1, "sketch_ids": {}}
    steps = [s for s in j.get("sequence", []) if s.get("type") == "ExtrudeFeature"]
    # first emit every sketch referenced by any extrude, so no extrude can name
    # a sketch feature that appears later in the tree
    for step in steps:
        ex = es.get(step.get("entity"))
        if not ex:
            continue
        if not ex.get("profiles"):
            return None, "no profiles"
        for p in ex["profiles"]:
            sk = es.get(p.get("sketch"))
            if not sk:
                return None, "missing sketch"
            if p.get("profile") not in sk.get("profiles", {}):
                return None, f"missing profile {p.get('profile')}"
            sid, err = emit_sketch(sk, p["profile"], ctx)
            if sid is None:
                return None, err
    for step in steps:
        ex = es.get(step.get("entity"))
        if not ex:
            continue
        d1 = float(ex["extent_one"]["distance"]["value"])
        d2 = float(ex.get("extent_two", {}).get("distance", {}).get("value", 0) or 0)
        et = ex.get("extent_type")
        if et == "OneSideFeatureExtentType":
            dirs = [(abs(d1), 1 if d1 >= 0 else -1)]
        elif et == "TwoSidesFeatureExtentType":
            dirs = [(abs(d1), 1 if d1 >= 0 else -1), (abs(d2), 1 if d2 >= 0 else -1)]
        else:
            dirs = [(abs(d1), 1), (abs(d1), -1)]
        for p in ex["profiles"]:
            sk = es.get(p["sketch"])
            sid = ctx["sketch_ids"][(id(sk), p["profile"])]
            for depth, sign in dirs:
                emit_extrude(ctx, sid, ex.get("operation"), depth, sign)
    if not ctx["feats"]:
        return None, "no features"
    return {"format": "ck-part", "version": 1, "features": ctx["feats"]}, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sample", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=1000)
    args = ap.parse_args()
    files = sorted(glob.glob(os.path.join(args.sample, "**", "*.json"), recursive=True))[: args.limit]
    cases, skips = {}, {}
    for f in files:
        j = json.load(open(f))
        es = j.get("entities", {})
        extrudes = [e for e in es.values() if e.get("type") == "ExtrudeFeature"]
        if not extrudes:
            skips["no extrude"] = skips.get("no extrude", 0) + 1
            continue
        bad = None
        for ex in extrudes:
            if ex.get("operation") not in ("NewBodyFeatureOperation", "JoinFeatureOperation", "CutFeatureOperation"):
                bad = "operation"; break
            if ex.get("extent_type") not in ("OneSideFeatureExtentType", "TwoSidesFeatureExtentType", "SymmetricFeatureExtentType"):
                bad = "extent"; break
            if abs(float(ex.get("extent_one", {}).get("taper_angle", {}).get("value", 0) or 0)) > 1e-9:
                bad = "taper"; break
            if abs(float(ex.get("extent_two", {}).get("taper_angle", {}).get("value", 0) or 0)) > 1e-9:
                bad = "taper"; break
            for p in ex.get("profiles", []):
                sk = es.get(p["sketch"])
                if not sk or p["profile"] not in sk.get("profiles", {}):
                    bad = "missing profile"; break
                for loop in sk["profiles"][p["profile"]]["loops"]:
                    for c in loop.get("profile_curves", []):
                        if c["type"] not in ("Line3D", "Circle3D", "Arc3D"):
                            bad = f"curve {c['type']}"; break
                    if bad: break
                if bad: break
            if bad: break
        if bad:
            skips[bad] = skips.get(bad, 0) + 1
            continue
        try:
            part, err = build_part(j)
            if part is None:
                skips[f"wire: {err}"] = skips.get(f"wire: {err}", 0) + 1
                continue
            vol, area, reference, oerr = 0.0, 0.0, "unusable", None
            try:
                solid, err = build_occt(j)
                if solid is None or solid.IsNull():
                    oerr = err
                else:
                    p = GProp_GProps(); BRepGProp.VolumeProperties_s(solid, p); vol = p.Mass()
                    reference = "ok"
                    if vol < 0:
                        vol = abs(vol); reference = "reversed"
                    if not (vol > 0) or not math.isfinite(vol):
                        try:
                            fix = ShapeFix_Shape(solid); fix.Perform(); fixed = fix.Shape()
                            p = GProp_GProps(); BRepGProp.VolumeProperties_s(fixed, p); vol = abs(p.Mass())
                            solid = fixed
                            reference = "fixed" if vol > 0 else "unusable"
                        except Exception:
                            reference = "unusable"
                    a = GProp_GProps(); BRepGProp.SurfaceProperties_s(solid, a); area = a.Mass()
            except Exception as e:
                oerr = type(e).__name__
            occt = {"volume": vol, "area": area, "reference": reference}
            if oerr:
                occt["error"] = oerr
            cases[os.path.basename(f)] = {"part": part, "occt": occt}
        except Exception as e:
            skips[f"exception: {type(e).__name__}"] = skips.get(f"exception: {type(e).__name__}", 0) + 1
    json.dump(cases, open(args.out, "w"), indent=1)
    print(json.dumps({"translated": len(cases), "skipped": len(files) - len(cases), "reasons": skips}, indent=1))


if __name__ == "__main__":
    main()
