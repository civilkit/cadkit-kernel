#!/usr/bin/env python3.12
"""OCCT mass properties for tools/massprops_sweep.mjs."""
import json, math, sys, os
sys.path.insert(0, os.path.dirname(__file__))
import shape_sweep_ref as S
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeTorus
from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
from OCP.gp import gp_Pnt
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def shape(s):
    k = s['kind']
    if k == 'box':
        return BRepPrimAPI_MakeBox(gp_Pnt(-s['w']/2, -s['d']/2, 0), gp_Pnt(s['w']/2, s['d']/2, s['h'])).Shape()
    if k == 'cyl':
        return BRepPrimAPI_MakeCylinder(s['r'], s['h']).Shape()
    if k == 'cone':
        rt = max(s['r'] - s['h'] * math.tan(math.radians(s['draft'])), 0.01)
        return BRepPrimAPI_MakeCone(s['r'], rt, s['h']).Shape()
    if k == 'sphere':
        return BRepPrimAPI_MakeSphere(s['r']).Shape()
    if k == 'torus':
        return BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), s['R'], s['r']).Shape()
    if k == 'tube':
        o = BRepPrimAPI_MakeCylinder(s['ro'], s['h']).Shape()
        i = BRepPrimAPI_MakeCylinder(s['ri'], s['h'] + 2).Shape()
        c = BRepAlgoAPI_Cut(o, i); c.Build(); return c.Shape()
    if k == 'loft':
        return S.loft(s['sketches'])
    if k == 'revolve':
        return S.revolve(s)
    if k == 'sweep_line':
        return S.sweep_line(s)
    if k == 'sweep_arc':
        return S.sweep_arc(s)
    if k == 'sweep_L':
        return S.sweep_L(s)
    if k == 'sweep_L':
        return S.sweep_L(s)
    raise RuntimeError('unknown ' + k)


def props(sh):
    p = GProp_GProps(); BRepGProp.VolumeProperties_s(sh, p)
    a = GProp_GProps(); BRepGProp.SurfaceProperties_s(sh, a)
    c = p.CentreOfMass()
    m = p.MatrixOfInertia()
    return {'volume': abs(p.Mass()), 'area': abs(a.Mass()),
            'com': [c.X(), c.Y(), c.Z()],
            'inertia': [[m.Value(i, j) for j in range(1, 4)] for i in range(1, 4)]}


def main():
    cases = json.load(open(arg('--cases')))
    out = {}
    for name, spec in cases.items():
        try:
            out[name] = props(shape(spec))
        except Exception as e:
            out[name] = {'error': f'{type(e).__name__}: {e}'}
    json.dump(out, open(arg('--out'), 'w'), indent=1)


if __name__ == '__main__':
    main()
