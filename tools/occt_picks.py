#!/usr/bin/env python3.12
"""Edge and face picks for OCCT test scripts.

DRAW's `explode` numbers sub-shapes in TopExp_Explorer order, skipping
duplicates; `nexplode` sorts faces/edges by a centre-of-mass key
(DBRep.cxx).  This helper rebuilds the same base with OCP, applies any
earlier blends, and returns midpoint + direction for edges and midpoint +
outward normal for faces.  The JS driver resolves the kernel entity by
geometry, not by OCCT's internal index.

    python3.12 tools/occt_picks.py --case tests/blend/simple/A2
    python3.12 tools/occt_picks.py --case tests/draft/angle/A1

Output: {"ok": true, "stages": [{"picks": [...]}], "face_picks": [...]}
Anything outside the primitive/boolean/transform/blend/draft subset is
{"ok": false, "why": ...}, so the case stays `unsupported`.
"""
import argparse
import json
import math
import re

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Common
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCone,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSphere,
)
from OCP.BRep import BRep_Tool
from OCP.GProp import GProp_GProps
from OCP.GeomLProp import GeomLProp_SLProps
from OCP.ShapeAnalysis import ShapeAnalysis_Surface
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec


def edges_of(shape):
    """TopExp_Explorer order, duplicates skipped (DRAW `explode`)."""
    m = TopTools_IndexedMapOfShape()
    ex = TopExp_Explorer(shape, TopAbs_EDGE)
    out = []
    while ex.More():
        s = ex.Current()
        if not m.Contains(s):
            m.Add(s)
            out.append(TopoDS.Edge_s(s))
        ex.Next()
    return out


def faces_of(shape, mode):
    """`explode` order, or `nexplode` (sort by the DBRep centre key)."""
    m = TopTools_IndexedMapOfShape()
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    raw = []
    while ex.More():
        s = ex.Current()
        if not m.Contains(s):
            m.Add(s)
            raw.append(TopoDS.Face_s(s))
        ex.Next()
    if mode != "nexplode":
        return raw
    keyed = []
    for i, f in enumerate(raw):
        p = GProp_GProps()
        BRepGProp.LinearProperties_s(f, p)
        c = p.CentreOfMass()
        keyed.append((c.X() * 999 + c.Y() * 99 + c.Z() * 0.9, i, f))
    keyed.sort(key=lambda x: x[0])
    return [x[2] for x in keyed]


def sample_edge(e):
    try:
        c = BRepAdaptor_Curve(e)
        a, b = c.FirstParameter(), c.LastParameter()
        if not (math.isfinite(a) and math.isfinite(b)) or b <= a:
            return []
        return [c.Value(a + (b - a) * f) for f in (0.15, 0.35, 0.5, 0.65, 0.85)]
    except Exception:
        return []


def edge_mid_dir(e):
    c = BRepAdaptor_Curve(e)
    a, b = c.FirstParameter(), c.LastParameter()
    m = a + (b - a) * 0.5
    p = c.Value(m)
    d = c.DN(m, 1)
    n = math.sqrt(d.X() ** 2 + d.Y() ** 2 + d.Z() ** 2) or 1.0
    return p, (d.X() / n, d.Y() / n, d.Z() / n)


def face_mid_normal(f):
    p = GProp_GProps()
    BRepGProp.SurfaceProperties_s(f, p)
    c = p.CentreOfMass()
    surf = BRep_Tool.Surface_s(f)
    sas = ShapeAnalysis_Surface(surf)
    uv = sas.ValueOfUV(c, 1e-6)
    pr = GeomLProp_SLProps(surf, uv.X(), uv.Y(), 1, 1e-6)
    n = pr.Normal()
    s = -1.0 if f.Orientation() == TopAbs_REVERSED else 1.0
    return c, (s * n.X(), s * n.Y(), s * n.Z())


def dist(p, q):
    return math.sqrt((p.X() - q.X()) ** 2 + (p.Y() - q.Y()) ** 2 + (p.Z() - q.Z()) ** 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", required=True)
    args = ap.parse_args()

    lines = []
    for raw in open(args.case, errors="ignore"):
        s = raw.strip()
        if not s or s.startswith("#") or s.startswith("##"):
            continue
        if re.match(r"^(puts|set|if|foreach|for|global|proc|return|unset|lappend|lset|incr|cpulimit|protect|pload)\b", s):
            continue
        if s in ("}", "{"):
            continue
        lines.append(s)

    shapes = {}
    stages = []
    face_picks = []
    face_mode = {}
    unsupported = None

    def num(x):
        return float(eval(x, {"__builtins__": {}}, {"pi": math.pi, "PI": math.pi}))

    def fail(why):
        nonlocal unsupported
        unsupported = why

    for line in lines:
        w = line.split()
        c = w[0]
        try:
            if c == "box":
                if len(w) == 5:
                    s = BRepPrimAPI_MakeBox(num(w[2]), num(w[3]), num(w[4])).Shape()
                elif len(w) == 8:
                    s = BRepPrimAPI_MakeBox(gp_Pnt(num(w[2]), num(w[3]), num(w[4])), num(w[5]), num(w[6]), num(w[7])).Shape()
                else:
                    raise ValueError("box args")
                shapes[w[1]] = s
            elif c == "pcylinder":
                if len(w) == 4:
                    shapes[w[1]] = BRepPrimAPI_MakeCylinder(num(w[2]), num(w[3])).Shape()
                elif len(w) == 5:
                    shapes[w[1]] = BRepPrimAPI_MakeCylinder(num(w[2]), num(w[3]), math.radians(num(w[4]))).Shape()
                else:
                    raise ValueError("pcylinder args")
            elif c == "pcone":
                if len(w) == 5:
                    shapes[w[1]] = BRepPrimAPI_MakeCone(num(w[2]), num(w[3]), num(w[4])).Shape()
                elif len(w) == 6:
                    shapes[w[1]] = BRepPrimAPI_MakeCone(num(w[2]), num(w[3]), num(w[4]), math.radians(num(w[5]))).Shape()
                else:
                    raise ValueError("pcone args")
            elif c == "psphere":
                if len(w) != 3:
                    raise ValueError("partial sphere")
                shapes[w[1]] = BRepPrimAPI_MakeSphere(num(w[2])).Shape()
            elif c in ("bfuse", "bcut", "bcommon"):
                a, b, r = shapes[w[2]], shapes[w[3]], w[1]
                mk = BRepAlgoAPI_Fuse if c == "bfuse" else BRepAlgoAPI_Cut if c == "bcut" else BRepAlgoAPI_Common
                op = mk(a, b)
                op.Build()
                if not op.IsDone():
                    raise ValueError(f"{c} failed in OCCT")
                shapes[r] = op.Shape()
            elif c in ("trotate", "ttranslate", "tscale", "copy"):
                if c == "copy":
                    shapes[w[2]] = shapes[w[1]]
                    continue
                if c == "tscale":
                    raise ValueError("tscale")
                name = w[1]
                vals = [num(x) for x in w[2:]]
                t = gp_Trsf()
                if c == "trotate":
                    if len(vals) == 4:
                        t.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(vals[0], vals[1], vals[2])), math.radians(vals[3]))
                    elif len(vals) == 7:
                        t.SetRotation(gp_Ax1(gp_Pnt(*vals[:3]), gp_Dir(*vals[3:6])), math.radians(vals[6]))
                    else:
                        raise ValueError("trotate args")
                else:
                    if len(vals) == 3:
                        t.SetTranslation(gp_Vec(*vals))
                    else:
                        raise ValueError("ttranslate args")
                shapes[name] = BRepBuilderAPI_Transform(shapes[name], t, True).Shape()
            elif c in ("explode", "nexplode"):
                if len(w) >= 3 and w[2][0] in "fF":
                    face_mode[w[1]] = c
            elif c == "blend":
                # blend result base r edge [r edge ...]
                result, base = w[1], w[2]
                rest = w[3:]
                if len(rest) % 2:
                    raise ValueError("blend args")
                final = edges_of(shapes[base])
                final_samples = [sample_edge(e) for e in final]
                picks = []
                selected = []
                for i in range(0, len(rest), 2):
                    radius = num(rest[i])
                    ename = rest[i + 1]
                    m = re.match(r"^(.*)_(\d+)$", ename)
                    if not m or m.group(1) not in shapes:
                        raise ValueError(f"edge {ename} not built")
                    src = edges_of(shapes[m.group(1)])
                    ix = int(m.group(2)) - 1
                    if ix < 0 or ix >= len(src):
                        raise ValueError(f"edge {ename} out of range")
                    samples = sample_edge(src[ix])
                    if not samples:
                        raise ValueError(f"edge {ename} has no samples")
                    best = None
                    for p in samples:
                        for fi, fs in enumerate(final_samples):
                            for q in fs:
                                d = dist(p, q)
                                if best is None or d < best[0]:
                                    best = (d, fi)
                    f_e = final[best[1]]
                    mp, md = edge_mid_dir(f_e)
                    src_mp, src_md = edge_mid_dir(src[ix])
                    picks.append({
                        "radius": radius,
                        "point": [mp.X(), mp.Y(), mp.Z()],
                        "dir": list(md),
                        "edge": ename,
                        "occt_point": [src_mp.X(), src_mp.Y(), src_mp.Z()],
                        "occt_dir": list(src_md),
                        "dist": best[0],
                    })
                    selected.append((radius, f_e))
                stages.append({"picks": picks})
                mk = BRepFilletAPI_MakeFillet(shapes[base])
                for radius, f_e in selected:
                    mk.Add(radius, f_e)
                mk.Build()
                if not mk.IsDone():
                    raise ValueError("OCCT fillet failed")
                shapes[result] = mk.Shape()
            elif c == "depouille":
                # depouille result shape dx dy dz face angle x y z nx ny nz
                if len(w) < 14:
                    raise ValueError("depouille args")
                shape, ename = w[2], w[6]
                m = re.match(r"^(.*)_(\d+)$", ename)
                if not m or m.group(1) not in shapes:
                    raise ValueError(f"face {ename} not built")
                faces = faces_of(shapes[m.group(1)], face_mode.get(m.group(1), "nexplode"))
                ix = int(m.group(2)) - 1
                if ix < 0 or ix >= len(faces):
                    raise ValueError(f"face {ename} out of range")
                p, n = face_mid_normal(faces[ix])
                face_picks.append({
                    "face": ename,
                    "point": [p.X(), p.Y(), p.Z()],
                    "normal": list(n),
                    "angle": num(w[7]),
                    "pull": [num(w[3]), num(w[4]), num(w[5])],
                    "neutral_point": [num(w[8]), num(w[9]), num(w[10])],
                    "neutral_normal": [num(w[11]), num(w[12]), num(w[13])],
                })
                shapes[w[1]] = shapes[shape]
            elif c in ("checkprops", "checkshape", "checknbshapes", "checkview", "dset", "lset", "renamevar", "donly", "nurbsconvert"):
                pass
            else:
                raise ValueError(f"untranslated command {c}")
        except Exception as e:
            fail(f"{c}: {e}")
            break

    if unsupported:
        print(json.dumps({"ok": False, "why": unsupported}))
        return
    print(json.dumps({"ok": True, "stages": stages, "face_picks": face_picks}))


if __name__ == "__main__":
    main()
