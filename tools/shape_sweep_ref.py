#!/usr/bin/env python3.12
"""OCCT reference volumes for tools/shape_sweep.mjs."""
import json, math, sys
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeVertex, BRepBuilderAPI_TransitionMode
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections, BRepOffsetAPI_MakePipeShell
from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
from OCP.GC import GC_MakeArcOfCircle
from OCP.Geom import Geom_Circle, Geom_Ellipse
from OCP.gp import gp_Pnt, gp_Dir, gp_Ax2, gp_Ax1, gp_Vec
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def circle_wire(r, z, cx=0, cy=0):
    e = BRepBuilderAPI_MakeEdge(Geom_Circle(gp_Ax2(gp_Pnt(cx, cy, z), gp_Dir(0, 0, 1)), r)).Edge()
    w = BRepBuilderAPI_MakeWire(); w.Add(e); return w.Wire()


def ellipse_wire(a, b, z):
    e = BRepBuilderAPI_MakeEdge(Geom_Ellipse(gp_Ax2(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)), a, b)).Edge()
    w = BRepBuilderAPI_MakeWire(); w.Add(e); return w.Wire()


def rect_points(w, d, z, rot=0):
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    return [(x * c - y * s, x * s + y * c, z) for x, y in [(-w/2, -d/2), (w/2, -d/2), (w/2, d/2), (-w/2, d/2)]]


def polygon_wire(pts):
    w = BRepBuilderAPI_MakeWire()
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        w.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge())
    return w.Wire()


def open_polyline(pts):
    w = BRepBuilderAPI_MakeWire()
    for i in range(len(pts) - 1):
        w.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*pts[i]), gp_Pnt(*pts[i + 1])).Edge())
    return w.Wire()


def face_of(wire):
    return BRepBuilderAPI_MakeFace(wire).Face()


def volume(shape):
    p = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, p)
    return abs(p.Mass())


def loft(specs):
    ts = BRepOffsetAPI_ThruSections(True)
    ts.CheckCompatibility(False)
    for s in specs:
        if s['kind'] == 'circle':
            ts.AddWire(circle_wire(s['r'], s['z']))
        elif s['kind'] == 'ellipse':
            ts.AddWire(ellipse_wire(s['a'], s['b'], s['z']))
        elif s['kind'] == 'rect':
            ts.AddWire(polygon_wire(rect_points(s['w'], s['d'], s['z'], s.get('rot', 0))))
        elif s['kind'] == 'point':
            ts.AddVertex(BRepBuilderAPI_MakeVertex(gp_Pnt(0, 0, s['z'])).Vertex())
    ts.Build()
    if not ts.IsDone():
        raise RuntimeError('loft failed')
    return ts.Shape()


def revolve(r):
    pts = r['profile']
    wire = polygon_wire([(x, y, 0) for x, y in pts])
    face = face_of(wire)
    axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
    m = BRepPrimAPI_MakeRevol(face, axis, math.radians(r['angle']))
    if not m.IsDone():
        raise RuntimeError('revolve failed')
    return m.Shape()


def pipe(path, prof_wire):
    m = BRepOffsetAPI_MakePipeShell(path)
    m.Add(prof_wire, False, False)
    m.Build()
    if not m.IsDone():
        raise RuntimeError('pipe failed')
    m.MakeSolid()
    return m.Shape()


def sweep_line(r):
    path = open_polyline([(0, 0, 0), (0, r['len'], 0)])
    prof = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), r['r'])).Edge()).Wire()
    return pipe(path, prof)


def sweep_arc(r):
    R, a1 = r['R'], math.radians(r['a1'])
    b = (-R + R * math.cos(a1), R * math.sin(a1), 0)
    mid = (-R + R * math.cos(a1 / 2), R * math.sin(a1 / 2), 0)
    e = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(gp_Pnt(0, 0, 0), gp_Pnt(*mid), gp_Pnt(*b)).Value()).Edge()
    w = BRepBuilderAPI_MakeWire(); w.Add(e); path = w.Wire()
    prof = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), r['r'])).Edge()).Wire()
    return pipe(path, prof)


def sweep_L(r):
    path = open_polyline([(0, 0, 0), (0, r['l1'], 0), (r['l2'], r['l1'], 0)])
    pts = [(x, 0, -y) for x, y in [(-r['rw']/2, -r['rd']/2), (r['rw']/2, -r['rd']/2), (r['rw']/2, r['rd']/2), (-r['rw']/2, r['rd']/2)]]
    m = BRepOffsetAPI_MakePipeShell(path)
    m.SetTransitionMode(BRepBuilderAPI_TransitionMode.BRepBuilderAPI_RightCorner)
    m.Add(polygon_wire(pts), False, False)
    m.Build()
    if not m.IsDone():
        raise RuntimeError('pipe failed')
    m.MakeSolid()
    return m.Shape()


def main():
    cases = json.load(open(arg('--cases')))
    out = {}
    for name, c in cases.items():
        try:
            r = c['ref']
            k = r['kind']
            if k == 'loft':
                out[name] = {'volume': volume(loft(r['sketches']))}
            elif k == 'loft_point':
                out[name] = {'volume': volume(loft([{'kind': 'circle', 'r': r['r'], 'z': 0}, {'kind': 'point', 'z': r['z']}]))}
            elif k == 'revolve':
                out[name] = {'volume': volume(revolve(r))}
            elif k == 'sweep_line':
                out[name] = {'volume': volume(sweep_line(r))}
            elif k == 'sweep_arc':
                out[name] = {'volume': volume(sweep_arc(r))}
            elif k == 'sweep_L':
                out[name] = {'volume': volume(sweep_L(r))}
            else:
                raise RuntimeError('unknown ' + k)
        except Exception as e:
            out[name] = {'error': f'{type(e).__name__}: {e}'}
    json.dump(out, open(arg('--out'), 'w'), indent=1)


if __name__ == '__main__':
    main()
