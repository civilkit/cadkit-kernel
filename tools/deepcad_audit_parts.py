#!/usr/bin/env python3.12
"""Independent analytic volume for simple translated DeepCAD parts.

Reads the translated part features (not OCP, not the DeepCAD source), samples
the sketch loops densely, computes outer-minus-holes area, and multiplies by
the total extrude depth. This decides whether a `wrong` result is our kernel
or a bad OCP reference.

    python3.12 tools/deepcad_audit_parts.py --cases cases.json --rows rows.json --out audit.json
"""
import argparse
import json
import math
import re

TAU = 2 * math.pi
N = 1000


def sample_entity(e, pts):
    if e["type"] == "line":
        a = pts[e["p1"]]; b = pts[e["p2"]]
        return [(a[0], a[1]), (b[0], b[1])]
    if e["type"] == "circle":
        c = pts[e["c"]]; r = e["r"]
        return [(c[0] + r * math.cos(TAU * k / N), c[1] + r * math.sin(TAU * k / N)) for k in range(N + 1)]
    if e["type"] == "arc":
        c = pts[e["c"]]; a = pts[e["p1"]]; b = pts[e["p2"]]
        a0 = math.atan2(a[1] - c[1], a[0] - c[0])
        a1 = math.atan2(b[1] - c[1], b[0] - c[0])
        while a1 <= a0:
            a1 += TAU
        r = math.hypot(a[0] - c[0], a[1] - c[1])
        return [(c[0] + r * math.cos(a0 + (a1 - a0) * k / N), c[1] + r * math.sin(a0 + (a1 - a0) * k / N)) for k in range(N + 1)]
    return None


def area(poly):
    return 0.5 * sum(poly[i][0] * poly[i + 1][1] - poly[i + 1][0] * poly[i][1] for i in range(len(poly) - 1))


def inside(pt, poly):
    x, y = pt; hit = False
    for i in range(len(poly) - 1):
        x1, y1 = poly[i]; x2, y2 = poly[i + 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    return hit


def sketch_area(f):
    pts = {p["id"]: (p["x"], p["y"]) for p in f["points"]}
    ents = f["entities"]
    # standalone circles
    loops = []
    for e in ents:
        if e["type"] == "circle":
            loops.append(sample_entity(e, pts))
    segs = [e for e in ents if e["type"] in ("line", "arc")]
    # chain segments by endpoint id
    adj = {}
    for i, e in enumerate(segs):
        adj.setdefault(e["p1"], []).append(i)
        adj.setdefault(e["p2"], []).append(i)
    used = set()
    for i0 in range(len(segs)):
        if i0 in used:
            continue
        used.add(i0)
        e = segs[i0]
        start, cur = e["p1"], e
        chain = [(i0, e["p1"] == start)]
        end = e["p2"] if e["p1"] == start else e["p1"]
        guard = 0
        while end != start and guard < len(segs) + 1:
            guard += 1
            nxt = [j for j in adj.get(end, []) if j not in used]
            if not nxt:
                break
            j = nxt[0]; used.add(j); ej = segs[j]
            fwd = ej["p1"] == end
            chain.append((j, fwd))
            end = ej["p2"] if fwd else ej["p1"]
        if end != start:
            return None, "open loop"
        poly = []
        for j, fwd in chain:
            p = sample_entity(segs[j], pts)
            if not fwd:
                p = list(reversed(p))
            if poly and poly[-1] == p[0]:
                poly.extend(p[1:])
            else:
                poly.extend(p)
        if poly[0] != poly[-1]:
            poly.append(poly[0])
        loops.append(poly)
    if not loops:
        return None, "no loops"
    areas = [abs(area(p)) for p in loops]
    order = sorted(range(len(loops)), key=lambda i: -areas[i])
    outers = []
    for i in order:
        p = loops[i]
        if not p or len(p) < 2:
            continue
        ins = next((j for j, o in enumerate(outers) if inside(p[0], loops[o])), None)
        if ins is None:
            outers.append(i)
        else:
            loops[outers[ins]] = loops[outers[ins]] + list(reversed(p))
    return abs(sum(area(loops[i]) for i in outers)), None


def analytic(case):
    feats = case["part"]["features"]
    sketches = {f["id"]: f for f in feats if f["type"] == "sketch"}
    ex = [f for f in feats if f["type"] == "extrude"]
    if not ex:
        return None, "no extrude"
    if any(f["type"] not in ("sketch", "plane", "extrude") for f in feats):
        return None, "other features"
    if any(f.get("op") != "add" for f in ex):
        return None, "cut/join"
    if len({f["sketch"] for f in ex}) != 1:
        return None, "multiple sketches"
    sid = ex[0]["sketch"]
    if sid not in sketches:
        return None, "missing sketch"
    a, err = sketch_area(sketches[sid])
    if a is None:
        return None, err
    depth = sum(abs(f.get("depth", 0.0)) for f in ex)
    return a * depth, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", required=True)
    ap.add_argument("--rows", required=True)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    cases = json.load(open(args.cases))
    rows = {r["name"]: r for r in json.load(open(args.rows))}
    out, counts = [], {}
    for name, c in cases.items():
        r = rows.get(name)
        if not r or r["cls"] != "wrong":
            continue
        a, err = analytic(c)
        if a is None:
            out.append({"name": name, "kind": "unresolved", "why": err})
            counts["unresolved"] = counts.get("unresolved", 0) + 1
            continue
        a *= args.scale ** 3
        m = re.search(r"vol ([0-9.eE+\-]+) occt ([0-9.eE+\-]+)", r["detail"])
        if not m:
            continue
        ours, occt = float(m.group(1)), float(m.group(2))
        def near(x, y):
            return abs(x - y) / max(abs(y), 1e-15) <= 1e-4
        if near(ours, a) and not near(occt, a):
            kind = "ours-right"
        elif near(occt, a) and not near(ours, a):
            kind = "occt-right"
        elif near(ours, a) and near(occt, a):
            kind = "both-agree"
        else:
            kind = "neither"
        counts[kind] = counts.get(kind, 0) + 1
        out.append({"name": name, "kind": kind, "analytic": a, "ours": ours, "occt": occt,
                    "ours_rel": abs(ours - a) / max(abs(a), 1e-15),
                    "occt_rel": abs(occt - a) / max(abs(a), 1e-15)})
    json.dump(out, open(args.out, "w"), indent=1)
    print(json.dumps({"audited": len(out), "counts": counts}, indent=1))
    for x in out:
        if "analytic" in x:
            print(f"{x['kind']:14s} {x['name']} analytic {x['analytic']:.6e} ours {x['ours']:.6e} ({x['ours_rel']:.1e}) occt {x['occt']:.6e} ({x['occt_rel']:.1e})")


if __name__ == "__main__":
    main()
