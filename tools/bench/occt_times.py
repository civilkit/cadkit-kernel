"""Time the BenchCAD CadQuery programs (OCCT through the OCP bindings).

usage: python3.12 occt_times.py <codedir> <out.json> [--runs 5] [--only id,id] [--tess id,id] [--tol 4e-4] [--noclean]

<codedir> holds <id>.py (fetch with fetch_benchcad_code.py). Each program is exec'd as is (import
cadquery, build `result`, show_object stub); the timed span is the whole exec = building the solid.
1 warmup, then N timed runs. Reports min / median ms, volume, validity (BRepCheck), face count, and,
for the ids named in --tess, the time of BRepMesh at the kernel's display tolerance (linear 4e-4 x bbox diagonal,
0.3 rad angular) plus the triangle count.
Volume is in the dataset frame (Z-up, same scalar as the Y-up frame we compare against).
"""
import json, os, statistics, sys, time, traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from occt_shim import cq  # noqa: E402


def arg(flag, default=None):
    if flag in sys.argv:
        return sys.argv[sys.argv.index(flag) + 1]
    return default


codedir, out = sys.argv[1], sys.argv[2]
runs = int(arg('--runs', 5))
only = arg('--only')
noclean = '--noclean' in sys.argv  # monkeypatch CadQuery's clean() (UnifySameDomain after every boolean): closer to raw OCCT calls
tess_ids = set((arg('--tess') or '').split(','))
rel_tol = float(arg('--tol', 4e-4))  # chord = rel_tol * bbox diagonal, as the kernel's display mesh (angle 0.3)
ids = sorted(f[:-3] for f in os.listdir(codedir) if f.endswith('.py'))
if only:
    ids = [i for i in ids if i in only.split(',')]


if noclean:
    cq.Workplane.clean = lambda self: self
    def _walk(c):
        for sub in c.__subclasses__():
            yield sub
            yield from _walk(sub)
    for _c in [cq.Shape, *_walk(cq.Shape)]:
        if 'clean' in vars(_c):
            _c.clean = lambda self: self


def build(src):
    box = {}
    ns = {'show_object': lambda o, *a, **k: box.setdefault('r', o), 'cq': cq}
    exec(compile(src, 'prog', 'exec'), ns)
    r = box.get('r', ns.get('result'))
    return r


def shape_of(r):
    if hasattr(r, 'val'):
        vs = r.vals()
        if len(vs) == 1:
            return vs[0]
        return cq.Compound.makeCompound([v for v in vs if hasattr(v, 'wrapped')])
    return r


res = {}
for i in ids:
    src = open(f'{codedir}/{i}.py').read()
    row = {'id': i}
    try:
        t0 = time.perf_counter()
        r = build(src)
        row['cold_ms'] = (time.perf_counter() - t0) * 1e3
        ts = []
        for _ in range(runs):
            t0 = time.perf_counter()
            r = build(src)
            ts.append((time.perf_counter() - t0) * 1e3)
        row['ms'] = statistics.median(ts)
        row['min_ms'] = min(ts)
        row['runs_ms'] = ts
        s = shape_of(r)
        row['volume'] = s.Volume()
        row['area'] = s.Area()
        row['valid'] = bool(s.isValid())
        row['faces'] = len(s.Faces())
        row['solids'] = len(s.Solids())
        if i in tess_ids:
            from OCP.BRepMesh import BRepMesh_IncrementalMesh
            from OCP.BRepTools import BRepTools
            tol = max(rel_tol * s.BoundingBox().DiagonalLength, 1e-3)
            tt = []
            for k in range(3):
                # fresh copy each time: BRepMesh caches triangulations on the shape
                c = s.copy()
                BRepTools.Clean_s(c.wrapped)
                t0 = time.perf_counter()
                BRepMesh_IncrementalMesh(c.wrapped, tol, False, 0.3, False)
                tt.append((time.perf_counter() - t0) * 1e3)
            vs, tris = c.tessellate(tol, 0.3)
            row['tess_ms'] = statistics.median(tt)
            row['tess_min_ms'] = min(tt)
            row['tess_tris'] = len(tris)
            row['tess_tol'] = tol
    except Exception as e:  # noqa: BLE001
        row['error'] = f'{type(e).__name__}: {e}'[:200]
        traceback.print_exc(limit=1)
    res[i] = row
    print(i, row.get('ms'), row.get('volume'), row.get('error', ''), flush=True)
json.dump(res, open(out, 'w'), indent=1)
