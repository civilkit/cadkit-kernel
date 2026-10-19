#!/usr/bin/env python3.12
"""OCCT reference volumes for tools/boolean_sweep.mjs."""
import json, math, sys
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeTorus
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf, gp_Vec, gp_Ax1, gp_Pnt, gp_Dir
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def shape(s):
    k = s['k']
    if k == 'box':
        return BRepPrimAPI_MakeBox(s['w'], s['d'], s['h']).Shape()
    if k == 'cyl':
        return BRepPrimAPI_MakeCylinder(s['r'], s['h']).Shape()
    if k == 'sphere':
        return BRepPrimAPI_MakeSphere(s['r']).Shape()
    if k == 'cone':
        # the kernel's extruded circle keeps r1 at the sketch (z = 0) and the draft shrinks
        # the far end; OCCT's MakeCone takes (base radius, top radius, height)
        rt = max(s['r1'] - s['h'] * math.tan(math.radians(s.get('draft', 0))), 0.01)
        return BRepPrimAPI_MakeCone(s['r1'], rt, s['h']).Shape()
    if k == 'torus':
        # the kernel's front-plane sketch revolves about the sketch's y axis (world Y);
        # OCCT's torus axis is Z, so rotate it into the same orientation (the solid is
        # symmetric through its plane, so either 90-degree sign is the same shape)
        t = BRepPrimAPI_MakeTorus(s['R'], s['r']).Shape()
        tr = gp_Trsf()
        tr.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), math.pi / 2)
        return BRepBuilderAPI_Transform(t, tr, True).Shape()
    raise RuntimeError('unknown ' + k)


def moved(sh, mv):
    tr = gp_Trsf()
    if mv.get('rotate'):
        r = mv['rotate']
        tr.SetRotation(gp_Ax1(gp_Pnt(*r['origin']), gp_Dir(*r['axis'])), math.radians(r['angle']))
    if mv.get('translate'):
        tr.SetTranslationPart(gp_Vec(*mv['translate']))
    return BRepBuilderAPI_Transform(sh, tr, True).Shape()


def op(a, b, name):
    m = {'union': BRepAlgoAPI_Fuse, 'cut': BRepAlgoAPI_Cut, 'intersect': BRepAlgoAPI_Common}[name]
    x = m(a, b); x.Build()
    if not x.IsDone():
        raise RuntimeError('boolean failed')
    return x.Shape()


def main():
    cases = json.load(open(arg('--cases')))
    out = {}
    for name, c in cases.items():
        try:
            a = shape(c['A'])
            b = moved(shape(c['B']), c['move'])
            x = op(a, b, c['op'])
            if c.get('C'):
                cc = moved(shape(c['C']), c.get('Cmove') or {'translate': [0, 0, 0]})
                x = op(x, cc, c['op2'])
            p = GProp_GProps(); BRepGProp.VolumeProperties_s(x, p)
            out[name] = {'volume': abs(p.Mass())}
        except Exception as e:
            out[name] = {'error': f'{type(e).__name__}: {e}'}
    json.dump(out, open(arg('--out'), 'w'), indent=1)


if __name__ == '__main__':
    main()
