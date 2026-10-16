"""OCCT reference volumes for tools/blend_corpus.mjs (python3.12 + cadquery-ocp).

usage: python3.12 tools/blend_reference.py DIR [--out tools/blend_reference.json] [--only substr]
DIR comes from `node tools/blend_corpus.mjs --emit DIR`: <base>.step (the kernel's base bodies) and cases.json.
Each case is replayed with BRepFilletAPI_MakeFillet / MakeChamfer on the edges nearest the pick points.
  chamfer2 : MakeChamfer.Add(d1, d2, E, F)   d1 is measured on the face F (the face whose outward normal is op.face, or the cylinder)
  chamferA : MakeChamfer.AddDA(d, angle, E, F)   d on F, angle between the chamfer and F
  fillet   : MakeFillet.Add(r, E)   (per-pick `r` when given)
  varfillet: MakeFillet.Add(r1, r2, E)  r1 at the end of E nearer the pick (the first vertex of E when the pick is nearer to it)
Writes {case: {volume, valid, error?}}.
"""
import json, math, os, sys

from OCP.STEPControl import STEPControl_Reader
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet, BRepFilletAPI_MakeChamfer
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Cone, GeomAbs_Plane
from OCP.gp import gp_Pnt, gp_Vec
from OCP.GeomLProp import GeomLProp_SLProps
from OCP.BRepTools import BRepTools


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def read_step(path):
    r = STEPControl_Reader()
    if r.ReadFile(path) != 1:
        raise RuntimeError('STEP read failed: ' + path)
    r.TransferRoots()
    return r.OneShape()


def volume(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    return p.Mass()


def edges_of(shape):
    seen, out = [], []
    ex = TopExp_Explorer(shape, TopAbs_EDGE)
    while ex.More():
        e = TopoDS.Edge_s(ex.Current())
        if not any(e.IsSame(s) for s in seen):
            seen.append(e)
            out.append(e)
        ex.Next()
    return out


def nearest_edge(shape, at):
    v = BRepBuilderAPI_MakeVertex(gp_Pnt(*at)).Vertex()
    best = None
    for e in edges_of(shape):
        fs = faces_of_edge(shape, e)
        if len(fs) < 2 or all(fs[0].IsSame(x) for x in fs):
            continue  # a seam: both sides are one face
        d = BRepExtrema_DistShapeShape(v, e)
        d.Perform()
        if d.IsDone() and (best is None or d.Value() < best[0]):
            best = (d.Value(), e)
    if best is None or best[0] > 1e-3:
        raise RuntimeError(f'no edge near {at}')
    return best[1]


def faces_of_edge(shape, e):
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, m)
    out = []
    for f in m.FindFromKey(e):
        out.append(TopoDS.Face_s(f))
    return out


def face_normal_at(f, pt):
    ad = BRepAdaptor_Surface(f)
    # closest parameter via projection on the underlying surface
    from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
    surf = BRep_Tool.Surface_s(f)
    pr = GeomAPI_ProjectPointOnSurf(gp_Pnt(*pt), surf)
    u, v = pr.LowerDistanceParameters()
    pr2 = GeomLProp_SLProps(surf, u, v, 1, 1e-9)
    n = pr2.Normal()
    if f.Orientation() == TopAbs_REVERSED:
        n = n.Reversed()
    return (n.X(), n.Y(), n.Z()), ad.GetType()


def pick_face(shape, e, want, mid):
    best = None
    for f in faces_of_edge(shape, e):
        n, ty = face_normal_at(f, mid)
        if want in ('cyl', 'cone'):
            if ty == (GeomAbs_Cylinder if want == 'cyl' else GeomAbs_Cone):
                return f
            continue
        d = n[0] * want[0] + n[1] * want[1] + n[2] * want[2]
        if best is None or d > best[0]:
            best = (d, f)
    if best is None or best[0] < 0.99:
        raise RuntimeError(f'no face with normal {want}')
    return best[1]


def edge_mid(e):
    c = BRepAdaptor_Curve(e)
    p = c.Value(0.5 * (c.FirstParameter() + c.LastParameter()))
    return (p.X(), p.Y(), p.Z())


def near_first(e, at):
    f = TopExp.FirstVertex_s(e, True)
    l = TopExp.LastVertex_s(e, True)
    pf, pl = BRep_Tool.Pnt_s(f), BRep_Tool.Pnt_s(l)
    return pf.Distance(gp_Pnt(*at)) <= pl.Distance(gp_Pnt(*at))


def run_case(c, shapes):
    out = run_step(c, shapes[c['base']], c['picks'])
    if c.get('step2'):
        out = run_step(c, out, c['step2'], shape_given=True)
    return {'volume': volume(out), 'valid': bool(BRepCheck_Analyzer(out).IsValid())}


def run_step(c, shape, picks, shape_given=False):
    edges = [(nearest_edge(shape, p['at']), p) for p in picks]
    op = c['op']
    k = op['k']
    if k in ('chamfer', 'chamfer2', 'chamferA'):
        mk = BRepFilletAPI_MakeChamfer(shape)
        for e, p in edges:
            if k == 'chamfer':
                mk.Add(op['d'], e)
            else:
                f = pick_face(shape, e, op['face'], edge_mid(e))
                if k == 'chamfer2':
                    mk.Add(op['d1'], op['d2'], e, f)
                else:
                    mk.AddDA(op['d'], math.radians(op['a']), e, f)
    else:
        mk = BRepFilletAPI_MakeFillet(shape)
        for e, p in edges:
            if k == 'fillet':
                mk.Add(p.get('r') or op['r'], e)
            else:
                a, b = p.get('radii') or op['radii']
                if near_first(e, p['at']):
                    mk.Add(a, b, e)
                else:
                    mk.Add(b, a, e)
    mk.Build()
    if not mk.IsDone():
        raise RuntimeError('Build failed')
    return mk.Shape()


# OCCT's chamfer builder fails on these plane / cone edges; the volumes are the Pappus integral of the
# removed triangle (revolved about the axis) taken from the hollow frustum (outer r 15 -> 10, bore 8, height 20)
ANALYTIC = {
    'cone_ch2_top_rim': 5882.390749073987,
    'cone_ch2_top_rim_cone_first': 5882.390749073987,
    'cone_chA_bottom_rim': 5729.224758826028,
}


def main():
    d = sys.argv[1]
    only = arg('--only', '')
    out_path = arg('--out', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'blend_reference.json'))
    cases = json.load(open(os.path.join(d, 'cases.json')))
    shapes = {}
    for c in cases:
        if c['base'] not in shapes:
            shapes[c['base']] = read_step(os.path.join(d, c['base'] + '.step'))
    ref = json.load(open(out_path)) if os.path.exists(out_path) else {}
    ref['_base'] = {b: volume(s) for b, s in shapes.items()}
    for c in cases:
        if only and only not in c['name']:
            continue
        try:
            ref[c['name']] = run_case(c, shapes)
        except Exception as ex:  # noqa: BLE001
            ref[c['name']] = {'volume': None, 'error': str(ex)[:120]}
            if c['name'] in ANALYTIC:
                ref[c['name']] = {'volume': ANALYTIC[c['name']], 'valid': True, 'source': 'analytic (OCCT: ' + str(ex)[:40] + ')'}
        print(c['name'].ljust(34), ref[c['name']])
    json.dump(ref, open(out_path, 'w'), indent=1, sort_keys=True)


main()
