#!/usr/bin/env python3.12
"""OCCT reference volumes/areas for tools/shell_sweep.mjs."""
import json, math, sys
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeCone
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeThickSolid
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.TopTools import TopTools_ListOfShape
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.gp import gp_Trsf, gp_Vec, gp_Pnt
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.GeomAbs import GeomAbs_Plane


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def edges(shape):
    out, seen = [], []
    ex = TopExp_Explorer(shape, TopAbs_EDGE)
    while ex.More():
        e = TopoDS.Edge_s(ex.Current())
        if not any(e.IsSame(s) for s in seen):
            seen.append(e); out.append(e)
        ex.Next()
    return out


def faces(shape):
    out = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        out.append(TopoDS.Face_s(ex.Current())); ex.Next()
    return out


def center(f):
    p = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, p)
    c = p.CentreOfMass(); return (c.X(), c.Y(), c.Z())


def base(spec):
    k = spec['kind']
    if k == 'box':
        return BRepPrimAPI_MakeBox(spec['w'], spec['d'], spec['h']).Shape()
    if k == 'cyl':
        return BRepPrimAPI_MakeCylinder(spec['r'], spec['h']).Shape()
    if k == 'tube':
        o = BRepPrimAPI_MakeCylinder(spec['ro'], spec['h']).Shape()
        i = BRepPrimAPI_MakeCylinder(spec['ri'], spec['h'] + 2).Shape()
        t = BRepAlgoAPI_Cut(o, i); t.Build(); return t.Shape()
    if k == 'cone':
        # extrude draft shrinks the top; use the matching frustum
        rt = spec['r'] - spec['h'] * math.tan(math.radians(spec['draft']))
        return BRepPrimAPI_MakeCone(max(rt, 0.01), spec['r'], spec['h']).Shape()
    if k == 'l':
        w, d, h, t = spec['w'], spec['d'], spec['h'], spec['t']
        a = BRepPrimAPI_MakeBox(w, d, t).Shape()
        b = BRepPrimAPI_MakeBox(t, d, h).Shape()
        tr = gp_Trsf(); tr.SetTranslation(gp_Vec(0, 0, t))
        b = BRepBuilderAPI_Transform(b, tr, True).Shape()
        f = BRepAlgoAPI_Fuse(a, b); f.Build(); return f.Shape()
    if k == 'filletbox':
        s = BRepPrimAPI_MakeBox(spec['w'], spec['d'], spec['h']).Shape()
        v = BRepBuilderAPI_MakeVertex(gp_Pnt(spec['w'], 0, spec['h'] / 2)).Vertex()
        best = None
        for e in edges(s):
            c = BRepAdaptor_Curve(e)
            if abs(c.Value(c.FirstParameter()).Z() - c.Value(c.LastParameter()).Z()) < 1e-9:
                continue
            d = BRepExtrema_DistShapeShape(v, e); d.Perform()
            if d.IsDone() and (best is None or d.Value() < best[0]):
                best = (d.Value(), e)
        mk = BRepFilletAPI_MakeFillet(s); mk.Add(spec['r'], best[1]); mk.Build()
        if not mk.IsDone():
            raise RuntimeError('fillet build failed')
        return mk.Shape()
    raise RuntimeError('unknown base')


def pick(shape, what):
    fs = faces(shape)
    if what == 'top':
        return [max(fs, key=lambda f: center(f)[2])]
    if what == 'bottom':
        return [min(fs, key=lambda f: center(f)[2])]
    if what == 'side_xmin':
        return [min(fs, key=lambda f: center(f)[0])]
    if what == 'side_ymin':
        return [min(fs, key=lambda f: center(f)[1])]
    raise RuntimeError('unknown face ' + what)


def shell(c):
    s = base(c['base']); op = c['op']
    L = TopTools_ListOfShape()
    for w in op.get('remove', []):
        for f in pick(s, w):
            L.Append(f)
    m = BRepOffsetAPI_MakeThickSolid()
    off = op['thickness'] if op.get('outward') else -op['thickness']
    m.MakeThickSolidByJoin(s, L, off, 1e-6)
    if not m.IsDone():
        return {'error': 'thick solid failed'}
    out = m.Shape(); p = GProp_GProps(); BRepGProp.VolumeProperties_s(out, p)
    return {'volume': abs(p.Mass())}


def area(c):
    b = c['base']
    if b['kind'] == 'sheet_planar':
        return {'area': b['w'] * b['d']}
    if b['kind'] == 'sheet_cyl':
        return {'area': 2 * math.pi * b['r'] * b['h']}
    if b['kind'] == 'sheet_cone':
        r1, r2, h = b['r1'], b['r2'], b['h']
        return {'area': math.pi * (r1 + r2) * math.hypot(h, r1 - r2)}
    raise RuntimeError('unknown sheet')


def main():
    cases = json.load(open(arg('--cases')))
    out = {}
    for name, c in cases.items():
        try:
            out[name] = area(c) if c['ref']['kind'] == 'thicken' else shell(c)
        except Exception as e:
            out[name] = {'error': f'{type(e).__name__}: {e}'}
    json.dump(out, open(arg('--out'), 'w'), indent=1)


if __name__ == '__main__':
    main()
