#!/usr/bin/env python3.12
"""OCCT/analytic volumes for tools/feature_sweep.mjs."""
import json, math, sys
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeCone
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf, gp_Vec
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def vol(sh):
    p = GProp_GProps(); BRepGProp.VolumeProperties_s(sh, p); return abs(p.Mass())


def cut(sh, tool):
    c = BRepAlgoAPI_Cut(sh, tool); c.Build()
    if not c.IsDone():
        raise RuntimeError('cut failed')
    return c.Shape()


def cyl(x, y, r, z0, z1):
    c = BRepPrimAPI_MakeCylinder(r, z1 - z0).Shape()
    tr = gp_Trsf(); tr.SetTranslation(gp_Vec(x, y, z0))
    return BRepBuilderAPI_Transform(c, tr, True).Shape()


def cone(x, y, r0, r1, z0, z1):
    c = BRepPrimAPI_MakeCone(r0, r1, z1 - z0).Shape()
    tr = gp_Trsf(); tr.SetTranslation(gp_Vec(x, y, z0))
    return BRepBuilderAPI_Transform(c, tr, True).Shape()


def ref(s):
    k = s['kind']
    if k == 'holes':
        sh = BRepPrimAPI_MakeBox(s['w'], s['d'], s['h']).Shape()
        for h in s['holes']:
            if h.get('depth') is None: sh = cut(sh, cyl(h['x'], h['y'], h['r'], -1, s['h'] + 1))
            else: sh = cut(sh, cyl(h['x'], h['y'], h['r'], s['h'] - h['depth'], s['h'] + 1))
        return vol(sh)
    if k == 'countersink':
        sh = BRepPrimAPI_MakeBox(s['w'], s['d'], s['h']).Shape()
        r1 = max(s['r'] - s['depth'] * math.tan(math.radians(s['angle'])), 0.01)
        sh = cut(sh, cone(s['x'], s['y'], r1, s['r'], s['h'] - s['depth'], s['h'] + 0.01))
        return vol(sh)
    if k == 'lpattern':
        sh = BRepPrimAPI_MakeBox(s['w'], s['d'], s['h']).Shape()
        for i in range(s['count']):
            x = s['x'] + i * s['spacing']
            sh = cut(sh, cyl(x, s['y'], s['r'], -1, s['h'] + 1))
        return vol(sh)
    if k == 'cpattern':
        sh = BRepPrimAPI_MakeBox(s['w'], s['d'], s['h']).Shape()
        for i in range(s['count']):
            a = 2 * math.pi * i / s['count']
            x = s['cx'] * math.cos(a) - s['cy'] * math.sin(a)
            y = s['cx'] * math.sin(a) + s['cy'] * math.cos(a)
            sh = cut(sh, cyl(x, y, s['r'], -1, s['h'] + 1))
        return vol(sh)
    if k == 'mirror':
        sh = BRepPrimAPI_MakeBox(s['w'], s['d'], s['h']).Shape()
        sh = cut(sh, cyl(s['x'], s['y'], s['r'], -1, s['h'] + 1))
        sh = cut(sh, cyl(2 * s['mirrorX'] - s['x'], s['y'], s['r'], -1, s['h'] + 1))
        return vol(sh)
    if k == 'draft':
        w, d, h, a = s['w'], s['d'], s['h'], math.tan(math.radians(s['angle']))
        return w * d * h - (w + d) * h * h * a + (4 / 3) * h ** 3 * a * a
    raise RuntimeError('unknown ' + k)


def main():
    cases = json.load(open(arg('--cases')))
    out = {}
    for name, c in cases.items():
        try: out[name] = {'volume': ref(c)}
        except Exception as e: out[name] = {'error': f'{type(e).__name__}: {e}'}
    json.dump(out, open(arg('--out'), 'w'), indent=1)


if __name__ == '__main__':
    main()
