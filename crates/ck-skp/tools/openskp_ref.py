"""Reference dump of OpenSKP's own Python output, for the parity check of ck-skp.

    python3 -I crates/ck-skp/tools/openskp_ref.py <openskp-clone-dir> <file-or-dir>... > ref.jsonl

Reads the .skp files read-only, imports OpenSKP's Python package straight from the clone (no install, no bytecode
written), and prints one JSON line per file with the same fields as `cargo run -p ck-skp --example skp_report`
(entity totals, baked triangle count, area, bounding box in mm / Z up).

OpenSKP imports two optional-looking third-party modules that may not be installed here: `defusedxml` (replaced by the
standard library parser: the inputs are the project's own test fixtures) and `mapbox_earcut` (replaced by the small
ear-clipping shim below, which triangulates polygons with holes correctly, so areas can be compared; the triangle
*count* of OpenSKP with a real earcut can differ by degenerate triangles). The shapely-based merge of overlapping holes is
turned off.

This file is part of ck-skp and is MIT-licensed like the rest of the crate; it contains no OpenSKP code.
"""
import sys, os, json, types, math
sys.dont_write_bytecode = True
root = sys.argv[1]
sys.path.insert(0, os.path.join(root, "packages", "python", "src"))

import xml.etree.ElementTree as _ET
d = types.ModuleType("defusedxml"); de = types.ModuleType("defusedxml.ElementTree"); dc = types.ModuleType("defusedxml.common")
for k in dir(_ET):
    if not k.startswith("__"):
        setattr(de, k, getattr(_ET, k))
class _DefusedXmlException(Exception): pass
dc.DefusedXmlException = _DefusedXmlException
d.ElementTree = de; d.common = dc
sys.modules.update({"defusedxml": d, "defusedxml.ElementTree": de, "defusedxml.common": dc})

import numpy as np

def _area(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

def _seg_cross(a, b, c, d_):
    o1, o2, o3, o4 = _area(a, b, c), _area(a, b, d_), _area(c, d_, a), _area(c, d_, b)
    return ((o1 > 0 and o2 < 0) or (o1 < 0 and o2 > 0)) and ((o3 > 0 and o4 < 0) or (o3 < 0 and o4 > 0))

def _ring_area(r):
    return sum(r[i][0]*r[(i+1) % len(r)][1] - r[(i+1) % len(r)][0]*r[i][1] for i in range(len(r))) / 2

def _earcut(coords, ends):
    pts = [tuple(p) for p in coords]
    rings, s = [], 0
    for e in ends:
        rings.append(list(range(s, int(e)))); s = int(e)
    outer = rings[0]
    if _ring_area([pts[i] for i in outer]) < 0: outer = outer[::-1]
    holes = []
    for h in rings[1:]:
        if len(h) < 3: continue
        if _ring_area([pts[i] for i in h]) > 0: h = h[::-1]
        holes.append(h)
    holes.sort(key=lambda h: -max(pts[i][0] for i in h))
    poly = list(outer)
    for h in holes:
        hm = max(range(len(h)), key=lambda k: pts[h[k]][0]); m = pts[h[hm]]
        cand = sorted(range(len(poly)), key=lambda k: (pts[poly[k]][0]-m[0])**2 + (pts[poly[k]][1]-m[1])**2)
        for ci in cand:
            v = pts[poly[ci]]; ok = True
            for ring in (poly, h):
                for k in range(len(ring)):
                    a, b = pts[ring[k]], pts[ring[(k+1) % len(ring)]]
                    if a == v or b == v or a == m or b == m: continue
                    if _seg_cross(m, v, a, b): ok = False; break
                if not ok: break
            if ok:
                poly = poly[:ci+1] + [h[(hm+k) % len(h)] for k in range(len(h)+1)] + [poly[ci]] + poly[ci+1:]
                break
    tris = []; n = list(poly)
    guard = 0
    while len(n) > 3 and guard < 10 * len(poly) ** 2 + 100:
        guard += 1; clipped = False
        for i in range(len(n)):
            a, b, c = pts[n[i-1]], pts[n[i]], pts[n[(i+1) % len(n)]]
            ar = _area(a, b, c)
            if abs(ar) < 1e-12: n.pop(i); clipped = True; break
            if ar < 0: continue
            if any(pts[q] not in (a, b, c) and _area(a, b, pts[q]) >= 0 and _area(b, c, pts[q]) >= 0 and _area(c, a, pts[q]) >= 0 for q in n): continue
            tris.append((n[i-1], n[i], n[(i+1) % len(n)])); n.pop(i); clipped = True; break
        if not clipped: break
    if len(n) == 3 and abs(_area(*[pts[q] for q in n])) > 1e-12: tris.append(tuple(n))
    return np.array([i for t in tris for i in t], dtype=np.uint32)

me = types.ModuleType("mapbox_earcut"); me.triangulate_float64 = lambda c, e: _earcut(np.asarray(c), np.asarray(e)); sys.modules["mapbox_earcut"] = me

from openskp import _core, scene as _scene
_core.mapbox_earcut = me
_core._merge_overlapping_hole_loops = lambda loops, *a, **k: loops

def collect(p, out):
    if os.path.isdir(p):
        for n in sorted(os.listdir(p)): collect(os.path.join(p, n), out)
    elif p.lower().endswith(".skp"):
        out.append(p)

files = []
for a in sys.argv[2:]: collect(a, files)
for f in files:
    rec = {"file": f}
    try:
        parsed = _core.full_parse(f)
        defs = parsed["defs_dict"]
        tot = [0, 0, 0, 0]
        for k, dd in defs.items():
            b = dd["builder"]
            tot[0] += len(b.vertices); tot[1] += len(b.edges); tot[2] += len(b.faces); tot[3] += len(b.instances)
        rec.update(ok=True, version=parsed["version"], defs=len(defs) - 1, vertices=tot[0], edges=tot[1], faces=tot[2], instances=tot[3],
                   materials=len(parsed["materials"]), layers=len(parsed["layer_colors"]))
        try:
            sc = _scene.build_scene(parsed)
            tris = 0; area = 0.0; lo = [1e300]*3; hi = [-1e300]*3
            for p in sc.glb_primitives:
                pos = np.frombuffer(p.positions, dtype=np.float32).reshape(-1, 3).astype(np.float64)
                idx = np.frombuffer(p.indices, dtype=np.uint32).reshape(-1, 3)
                tris += len(idx)
                if len(idx):
                    a, b, c = pos[idx[:, 0]], pos[idx[:, 1]], pos[idx[:, 2]]
                    area += float(np.linalg.norm(np.cross(b - a, c - a), axis=1).sum() / 2)
                if len(pos):
                    z = np.stack([pos[:, 0], -pos[:, 2], pos[:, 1]], 1) * 1000.0  # Y-up m -> Z-up mm
                    lo = [min(lo[i], float(z[:, i].min())) for i in range(3)]; hi = [max(hi[i], float(z[:, i].max())) for i in range(3)]
            rec.update(prims=len(sc.glb_primitives), tris=tris, area_mm2=round(area * 1e6, 3), gltf_materials=len(sc.gltf_materials), textures=len(sc.textures),
                       bbox_mm=None if tris == 0 and lo[0] > 1e299 else [round(x, 3) for x in lo + hi])
        except Exception as e:
            rec["bake_error"] = "%s: %s" % (type(e).__name__, e)
    except Exception as e:
        rec.update(ok=False, error="%s: %s" % (type(e).__name__, e))
    print(json.dumps(rec))
