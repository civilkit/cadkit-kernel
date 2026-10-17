#!/usr/bin/env python3.12
"""Edge picks for OCCT blend scripts.

The DRAW `explode` command numbers the sub-shapes of a shape in
TopExp_Explorer order, skipping duplicates (DBRep.cxx).  This helper builds
the same base shape with OCP and returns, for a requested `shape_N`, a point
on the nearest edge of the final body.

    python3.12 tools/occt_picks.py --case tests/blend/simple/A2

It only understands the primitive/boolean/transform subset the corpus uses;
anything else is reported as `{"ok": false, "why": ...}` so the JS driver can
mark the case unsupported instead of guessing.
"""
import argparse
import json
import math
import re
import sys

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Common
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCone,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSphere,
)
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec


def edges_of(shape):
    """The DRAW explode order: TopExp_Explorer, duplicates skipped."""
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
    """Midpoint and unit tangent of an edge (the disambiguator at a corner)."""
    c = BRepAdaptor_Curve(e)
    a, b = c.FirstParameter(), c.LastParameter()
    m = a + (b - a) * 0.5
    p = c.Value(m)
    d = c.DN(m, 1)
    n = math.sqrt(d.X() ** 2 + d.Y() ** 2 + d.Z() ** 2) or 1.0
    return p, (d.X() / n, d.Y() / n, d.Z() / n)


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
    blends = []  # (result, base, [(radius, edgename)])
    unsupported = None

    def num(x):
        return float(eval(x, {"__builtins__": {}}, {"pi": math.pi, "PI": math.pi}))

    def setshape(name, s):
        shapes[name] = s

    for line in lines:
        w = line.split()
        c = w[0]
        try:
            if c == "box":
                if len(w) == 5:
                    s = BRepPrimAPI_MakeBox(num(w[1 + 1]), num(w[2 + 1]), num(w[3 + 1])).Shape()
                elif len(w) == 8:
                    s = BRepPrimAPI_MakeBox(
                        gp_Pnt(num(w[2]), num(w[3]), num(w[4])),
                        num(w[5]), num(w[6]), num(w[7]),
                    ).Shape()
                else:
                    raise ValueError("box args")
                setshape(w[1], s)
            elif c == "pcylinder":
                if len(w) == 4:
                    setshape(w[1], BRepPrimAPI_MakeCylinder(num(w[2]), num(w[3])).Shape())
                elif len(w) == 5:
                    setshape(w[1], BRepPrimAPI_MakeCylinder(num(w[2]), num(w[3]), math.radians(num(w[4]))).Shape())
                else:
                    raise ValueError("pcylinder args")
            elif c == "pcone":
                if len(w) == 5:
                    setshape(w[1], BRepPrimAPI_MakeCone(num(w[2]), num(w[3]), num(w[4])).Shape())
                elif len(w) == 6:
                    setshape(w[1], BRepPrimAPI_MakeCone(num(w[2]), num(w[3]), num(w[4]), math.radians(num(w[5]))).Shape())
                else:
                    raise ValueError("pcone args")
            elif c == "psphere":
                if len(w) != 3:
                    raise ValueError("partial sphere")
                setshape(w[1], BRepPrimAPI_MakeSphere(num(w[2])).Shape())
            elif c in ("bfuse", "bcut", "bcommon"):
                a, b, r = shapes[w[2]], shapes[w[3]], w[1]
                mk = BRepAlgoAPI_Fuse if c == "bfuse" else BRepAlgoAPI_Cut if c == "bcut" else BRepAlgoAPI_Common
                op = mk(a, b)
                op.Build()
                if not op.IsDone():
                    raise ValueError(f"{c} failed in OCCT")
                setshape(r, op.Shape())
            elif c in ("trotate", "ttranslate", "tscale", "copy"):
                if c == "copy":
                    setshape(w[2], shapes[w[1]])
                    continue
                if c == "tscale":
                    raise ValueError("tscale")
                name = w[1]
                vals = [num(x) for x in w[2:]]
                t = gp_Trsf()
                if c == "trotate":
                    if len(vals) == 4:
                        ax = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(vals[0], vals[1], vals[2]))
                        t.SetRotation(ax, math.radians(vals[3]))
                    elif len(vals) == 7:
                        ax = gp_Ax1(gp_Pnt(*vals[:3]), gp_Dir(*vals[3:6]))
                        t.SetRotation(ax, math.radians(vals[6]))
                    else:
                        raise ValueError("trotate args")
                else:
                    if len(vals) == 3:
                        t.SetTranslation(gp_Vec(*vals))
                    elif len(vals) == 6:
                        # ttranslate dest src ... (copy form): dest is w[1], src w[2]
                        raise ValueError("ttranslate copy form")
                    else:
                        raise ValueError("ttranslate args")
                shapes[name] = BRepBuilderAPI_Transform(shapes[name], t, True).Shape()
            elif c in ("explode", "nexplode"):
                pass
            elif c == "blend":
                # blend result base r edge [r edge ...]
                result, base = w[1], w[2]
                rest = w[3:]
                if len(rest) % 2:
                    raise ValueError("blend args")
                picks = []
                for i in range(0, len(rest), 2):
                    picks.append((num(rest[i]), rest[i + 1]))
                blends.append((result, base, picks))
            elif c == "checkprops":
                pass
            elif c in ("checkshape", "checknbshapes", "checkview", "dset", "lset", "renamevar", "donly", "nurbsconvert"):
                pass
            else:
                raise ValueError(f"untranslated command {c}")
        except Exception as e:
            unsupported = f"{c}: {e}"
            break

    if unsupported:
        print(json.dumps({"ok": False, "why": unsupported}))
        return

    picks_out = []
    for result, base, picks in blends:
        if base not in shapes:
            print(json.dumps({"ok": False, "why": f"base {base} not built"}))
            return
        final = edges_of(shapes[base])
        if not final:
            print(json.dumps({"ok": False, "why": f"base {base} has no edges"}))
            return
        final_samples = [sample_edge(e) for e in final]
        for radius, ename in picks:
            m = re.match(r"^(.*)_(\d+)$", ename)
            if not m or m.group(1) not in shapes:
                print(json.dumps({"ok": False, "why": f"edge {ename} not built"}))
                return
            src = edges_of(shapes[m.group(1)])
            ix = int(m.group(2)) - 1
            if ix < 0 or ix >= len(src):
                print(json.dumps({"ok": False, "why": f"edge {ename} out of range"}))
                return
            samples = sample_edge(src[ix])
            if not samples:
                print(json.dumps({"ok": False, "why": f"edge {ename} has no samples"}))
                return
            # nearest final edge to any sample of the requested edge
            best = None
            for p in samples:
                for fi, f_s in enumerate(final_samples):
                    for q in f_s:
                        d = dist(p, q)
                        if best is None or d < best[0]:
                            best = (d, fi)
            f_e = final[best[1]]
            mp, md = edge_mid_dir(f_e)
            src_mp, src_md = edge_mid_dir(src[ix])
            picks_out.append({
                "radius": radius,
                "point": [mp.X(), mp.Y(), mp.Z()],
                "dir": list(md),
                "edge": ename,
                "occt_point": [src_mp.X(), src_mp.Y(), src_mp.Z()],
                "occt_dir": list(src_md),
                "dist": best[0],
            })

    print(json.dumps({"ok": True, "picks": picks_out, "base": blends[-1][1] if blends else None}))


if __name__ == "__main__":
    main()
