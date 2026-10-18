#!/usr/bin/env python3.12
"""OCCT reference volumes for tools/fillet_sweep.mjs.

Cases are parametric base solids + a blend op + pick points. The base is
rebuilt with OCP primitives; the edge nearest each pick is selected, the blend
is applied with BRepFilletAPI, and the volume is returned (absolute value for
reversed solids).
"""
import json, math, sys
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeEdge
from OCP.GC import GC_MakeArcOfCircle
from OCP.gp import gp_Vec
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet, BRepFilletAPI_MakeChamfer
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCP.TopoDS import TopoDS
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRep import BRep_Tool
from OCP.gp import gp_Pnt, gp_Dir
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Cone
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCP.GeomLProp import GeomLProp_SLProps


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def edges_of(shape):
    seen, out = [], []
    ex = TopExp_Explorer(shape, TopAbs_EDGE)
    while ex.More():
        e = TopoDS.Edge_s(ex.Current())
        if not any(e.IsSame(s) for s in seen):
            seen.append(e); out.append(e)
        ex.Next()
    return out


def faces_of_edge(shape, e):
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, m)
    return [TopoDS.Face_s(f) for f in m.FindFromKey(e)]


def edge_mid(e):
    c = BRepAdaptor_Curve(e); return c.Value(0.5 * (c.FirstParameter() + c.LastParameter()))


def nearest_edge(shape, at):
    v = BRepBuilderAPI_MakeVertex(gp_Pnt(*at)).Vertex(); best = None
    for e in edges_of(shape):
        fs = faces_of_edge(shape, e)
        if len(fs) < 2 or all(fs[0].IsSame(x) for x in fs):
            continue
        d = BRepExtrema_DistShapeShape(v, e); d.Perform()
        if d.IsDone() and (best is None or d.Value() < best[0]):
            best = (d.Value(), e)
    if best is None or best[0] > 1e-2:
        raise RuntimeError(f'no edge near {at}')
    return best[1]


def face_normal(f, pt):
    surf = BRep_Tool.Surface_s(f)
    pr = GeomAPI_ProjectPointOnSurf(gp_Pnt(*pt), surf)
    u, v = pr.LowerDistanceParameters()
    p = GeomLProp_SLProps(surf, u, v, 1, 1e-9); n = p.Normal()
    if f.Orientation() == TopAbs_REVERSED:
        n = n.Reversed()
    return (n.X(), n.Y(), n.Z()), BRepAdaptor_Surface(f).GetType()


def face_by_normal(shape, e, want, mid):
    if want in ('cyl', 'cone'):
        for f in faces_of_edge(shape, e):
            if face_normal(f, mid)[1] == (GeomAbs_Cylinder if want == 'cyl' else GeomAbs_Cone):
                return f
    best = None
    for f in faces_of_edge(shape, e):
        n, _ = face_normal(f, mid)
        d = n[0] * want[0] + n[1] * want[1] + n[2] * want[2]
        if best is None or d > best[0]:
            best = (d, f)
    if best is None or best[0] < 0.99:
        raise RuntimeError(f'no face with normal {want}')
    return best[1]


def base(spec):
    k = spec['kind']
    if k == 'box':
        w, d, h = spec['w'], spec['d'], spec['h']
        return BRepPrimAPI_MakeBox(w, d, h).Shape()
    if k == 'cyl':
        return BRepPrimAPI_MakeCylinder(spec['r'], spec['h']).Shape()
    if k == 'tube':
        o = BRepPrimAPI_MakeCylinder(spec['ro'], spec['h']).Shape()
        i = BRepPrimAPI_MakeCylinder(spec['ri'], spec['h'] + 2).Shape()
        t = BRepAlgoAPI_Cut(o, i); t.Build(); return t.Shape()
    if k == 'boss':
        p = BRepPrimAPI_MakeBox(spec['w'], spec['d'], spec['pt']).Shape()
        b = BRepPrimAPI_MakeCylinder(spec['br'], spec['bh']).Shape()
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        from OCP.gp import gp_Trsf
        tr = gp_Trsf(); tr.SetTranslation(gp_Vec(spec['bx'], spec['by'], spec['pt']))
        b = BRepBuilderAPI_Transform(b, tr, True).Shape()
        f = BRepAlgoAPI_Fuse(p, b); f.Build(); return f.Shape()
    if k == 'hole':
        p = BRepPrimAPI_MakeBox(spec['w'], spec['d'], spec['h']).Shape()
        c = BRepPrimAPI_MakeCylinder(spec['r'], spec['h'] + 2).Shape()
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        from OCP.gp import gp_Trsf
        tr = gp_Trsf(); tr.SetTranslation(gp_Vec(spec['cx'], spec['cy'], -1))
        c = BRepBuilderAPI_Transform(c, tr, True).Shape()
        f = BRepAlgoAPI_Cut(p, c); f.Build(); return f.Shape()
    if k == 'rrect':
        w, d, cr, h = spec['w'], spec['d'], spec['cr'], spec['h']
        P = [(cr,0),(w-cr,0),(w,cr),(w,d-cr),(w-cr,d),(cr,d),(0,d-cr),(0,cr)]
        C = [(w-cr,cr),(w-cr,d-cr),(cr,d-cr),(cr,cr)]
        mk = BRepBuilderAPI_MakeWire()
        def arc(a,b,c):
            import math as _m
            u=(a[0]-c[0],a[1]-c[1]); v=(b[0]-c[0],b[1]-c[1])
            lu=_m.hypot(*u) or 1.0; lv=_m.hypot(*v) or 1.0
            d=(u[0]/lu+v[0]/lv,u[1]/lu+v[1]/lv); ld=_m.hypot(*d) or 1.0
            r=lu
            mid=(c[0]+d[0]/ld*r,c[1]+d[1]/ld*r)
            return BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(gp_Pnt(a[0],a[1],0),gp_Pnt(mid[0],mid[1],0),gp_Pnt(b[0],b[1],0)).Value()).Edge()
        mk.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(P[0][0],P[0][1],0),gp_Pnt(P[1][0],P[1][1],0)).Edge())
        mk.Add(arc(P[1],P[2],C[0]))
        mk.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(P[2][0],P[2][1],0),gp_Pnt(P[3][0],P[3][1],0)).Edge())
        mk.Add(arc(P[3],P[4],C[1]))
        mk.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(P[4][0],P[4][1],0),gp_Pnt(P[5][0],P[5][1],0)).Edge())
        mk.Add(arc(P[5],P[6],C[2]))
        mk.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(P[6][0],P[6][1],0),gp_Pnt(P[7][0],P[7][1],0)).Edge())
        mk.Add(arc(P[7],P[0],C[3]))
        face = BRepBuilderAPI_MakeFace(mk.Wire()).Face()
        return BRepPrimAPI_MakePrism(face, gp_Vec(0,0,h)).Shape()
    raise RuntimeError('unknown base')


def apply(c):
    s = base(c['base']); op = c['op']
    if op['kind'] == 'fillet':
        mk = BRepFilletAPI_MakeFillet(s)
        if op.get('propagate'):
            top = c['base']['h']
            for e in edges_of(s):
                if abs(edge_mid(e).Z() - top) < 1e-6:
                    mk.Add(op['radius'], e)
        else:
            for p in c['picks']:
                e = nearest_edge(s, p['at'])
                if 'radius' in p:
                    mk.Add(p['radius'], e)
                else:
                    mk.Add(op['radius'], e)
    elif op['kind'] == 'chamfer':
        mk = BRepFilletAPI_MakeChamfer(s)
        for p in c['picks']:
            e = nearest_edge(s, p['at'])
            if op.get('type') == 'two':
                f = face_by_normal(s, e, op['face'], p['at'])
                mk.Add(op['d1'], op['d2'], e, f)
            elif op.get('type') == 'angle':
                f = face_by_normal(s, e, op['face'], p['at'])
                mk.AddDA(op['d'], math.radians(op['a']), e, f)
            else:
                mk.Add(op['d'], e)
    elif op['kind'] == 'varfillet':
        mk = BRepFilletAPI_MakeFillet(s)
        for p in c['picks']:
            mk.Add(op['r1'], op['r2'], nearest_edge(s, p['at']))
    else:
        raise RuntimeError('unknown op')
    mk.Build()
    if not mk.IsDone():
        return {'error': 'Build failed'}
    try:
        out = mk.Shape()
    except Exception:
        return {'error': 'Shape failed'}
    p = GProp_GProps(); BRepGProp.VolumeProperties_s(out, p)
    return {'volume': abs(p.Mass())}


def main():
    cases = json.load(open(arg('--cases')))
    out = {}
    for name, c in cases.items():
        try:
            out[name] = apply(c)
        except Exception as e:
            out[name] = {'error': str(e).split('\n')[0][:120]}
    json.dump(out, open(arg('--out'), 'w'), indent=1)
    ok = sum(1 for v in out.values() if 'volume' in v)
    print(f'fillet_sweep_ref: {ok}/{len(out)} volumes')


if __name__ == '__main__':
    main()
