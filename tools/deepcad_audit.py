#!/usr/bin/env python3.12
"""Independent analytic area/volume audit for simple DeepCAD cases.

For cases with one NewBody extrude and Line3D/Circle3D/Arc3D profile loops,
compute the profile area from dense polygonal sampling (outer minus holes) and
compare it with our kernel volume and the OCP volume. This decides whether a
`wrong` result is a kernel bug or a bad OCP reference.

    python3.12 tools/deepcad_audit.py --cases cases.json --rows rows.json --out audit.json
"""
import argparse
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(__file__))
import deepcad_build as db

SAMPLES = 400


def sample_curve(c, t):
    typ = c["type"]
    if typ == "Line3D":
        p1 = db.world(t, db.vec(c["start_point"]))
        p2 = db.world(t, db.vec(c["end_point"]))
        return [p1, p2]
    if typ in ("Circle3D", "Arc3D"):
        ctr = db.world(t, db.vec(c["center_point"]))
        n = db.unit(db.vec(c.get("normal", {"x": 0, "y": 0, "z": 1})))
        x = db.unit(db.cross(n, (0, 0, 1))) if abs(n[2]) < 0.9 else (1, 0, 0)
        y = db.unit(db.cross(n, x))
        r = float(c["radius"])
        if typ == "Circle3D":
            a0, a1 = 0.0, db.TAU
        else:
            a0, a1 = float(c["start_angle"]), float(c["end_angle"])
            if a1 <= a0:
                a1 += db.TAU
        return [db.addp(ctr, db.addp(db.mul(x, r * math.cos(a0 + (a1 - a0) * k / SAMPLES)), db.mul(y, r * math.sin(a0 + (a1 - a0) * k / SAMPLES)))) for k in range(SAMPLES + 1)]
    return []


def loop_poly(c, t):
    pts = []
    for curve in c["profile_curves"]:
        pts.extend(sample_curve(curve, t))
    # drop consecutive duplicates
    out = []
    for p in pts:
        if not out or db.norm((p[0] - out[-1][0], p[1] - out[-1][1], p[2] - out[-1][2])) > 1e-15:
            out.append(p)
    return out


def project(poly, t):
    x = db.unit(db.vec(t.get("x_axis", {"x": 1, "y": 0, "z": 0})))
    y = db.unit(db.vec(t.get("y_axis", {"x": 0, "y": 1, "z": 0})))
    o = db.vec(t.get("origin", {}))
    return [(db.dot((p[0] - o[0], p[1] - o[1], p[2] - o[2]), x), db.dot((p[0] - o[0], p[1] - o[1], p[2] - o[2]), y)) for p in poly]


def area(poly2):
    a = 0.0
    for i in range(len(poly2) - 1):
        a += poly2[i][0] * poly2[i + 1][1] - poly2[i + 1][0] * poly2[i][1]
    return a / 2.0


def inside(pt, poly):
    x, y = pt
    hit = False
    for i in range(len(poly) - 1):
        x1, y1 = poly[i]; x2, y2 = poly[i + 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    return hit


def profile_area(loops, t):
    polys = [project(loop_poly(l, t), t) for l in loops]
    areas = [abs(area(p)) for p in polys]
    order = sorted(range(len(polys)), key=lambda i: -areas[i])
    outers = []
    for i in order:
        p = polys[i]
        ins = next((j for j, o in enumerate(outers) if inside(p[0], polys[o])), None)
        if ins is None:
            outers.append(i)
        else:
            # hole: subtract from the outer by appending its polygon
            polys[outers[ins]] = polys[outers[ins]] + p[::-1]
    return sum(abs(area(polys[i])) for i in outers)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", required=True)
    ap.add_argument("--rows", required=True)
    ap.add_argument("--sample", required=True)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    cases = json.load(open(args.cases))
    rows = {r["name"]: r for r in json.load(open(args.rows))}
    paths = {}
    for root, _, files in os.walk(args.sample):
        for f in files:
            if f.endswith(".json"):
                paths[f] = os.path.join(root, f)
    out = []
    counts = {}
    for name, c in cases.items():
        row = rows.get(name)
        if not row or row["cls"] != "wrong":
            continue
        f = paths.get(name)
        if not f:
            continue
        j = json.load(open(f))
        es = j["entities"]
        exs = [e for e in es.values() if e.get("type") == "ExtrudeFeature"]
        if len(exs) != 1 or exs[0].get("operation") != "NewBodyFeatureOperation":
            continue
        ex = exs[0]
        d1 = float(ex["extent_one"]["distance"]["value"])
        d2 = float(ex.get("extent_two", {}).get("distance", {}).get("value", 0) or 0)
        et = ex.get("extent_type")
        if et == "OneSideFeatureExtentType":
            depth = abs(d1)
        elif et == "TwoSidesFeatureExtentType":
            depth = abs(d1) + abs(d2)
        elif et == "SymmetricFeatureExtentType":
            depth = 2 * abs(d1)
        else:
            continue
        try:
            total = 0.0
            for p in ex.get("profiles", []):
                sk = es[p["sketch"]]
                total += profile_area(sk["profiles"][p["profile"]]["loops"], sk.get("transform", {}))
            analytic = total * depth * args.scale ** 3
        except Exception:
            continue
        m = re.search(r"vol ([0-9.eE+\-]+) occt ([0-9.eE+\-]+)", row["detail"])
        if not m:
            continue
        ours, occt = float(m.group(1)), float(m.group(2))
        def near(a, b):
            return abs(a - b) / max(abs(b), 1e-15) <= 1e-4 if b else a == 0
        if near(ours, analytic) and not near(occt, analytic):
            kind = "ours-right"
        elif near(occt, analytic) and not near(ours, analytic):
            kind = "occt-right"
        elif near(ours, analytic) and near(occt, analytic):
            kind = "both-agree"
        else:
            kind = "neither"
        counts[kind] = counts.get(kind, 0) + 1
        out.append({"name": name, "kind": kind, "analytic": analytic, "ours": ours, "occt": occt,
                    "ours_rel": abs(ours - analytic) / max(abs(analytic), 1e-15),
                    "occt_rel": abs(occt - analytic) / max(abs(analytic), 1e-15)})
    json.dump(out, open(args.out, "w"), indent=1)
    print(json.dumps({"audited": len(out), "counts": counts}, indent=1))
    for x in out:
        print(f"{x['kind']:12s} {x['name']} analytic {x['analytic']:.6e} ours {x['ours']:.6e} ({x['ours_rel']:.1e}) occt {x['occt']:.6e} ({x['occt_rel']:.1e})")


if __name__ == "__main__":
    main()
