"""Cross-check of the `distance` request against OCCT (python3.12 with OCP).

usage: python3.12 tools/measure_occt_check.py [--wasm path] [--samples dir] [--pairs 20] [--tol 1e-6] [--parts a,b,c]

For each sample part: tools/measure_occt_dump.mjs regenerates it, exports STEP and measures random face pairs with
our `distance`; here the STEP is read with OCP, every face id is matched to the OCCT face holding an interior point
of it (BRepExtrema point-to-face, must be 0), and BRepExtrema_DistShapeShape(faceA, faceB) is compared with ours.
Exit 1 when any pair differs by more than --tol (absolute, or relative for distances above 1).
"""
import json, os, subprocess, sys, tempfile

from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.gp import gp_Pnt

HERE = os.path.dirname(os.path.abspath(__file__))


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def faces_of(path):
    r = STEPControl_Reader()
    if r.ReadFile(path) != 1:
        raise RuntimeError('STEP read failed')
    r.TransferRoots()
    sh = r.OneShape()
    ex = TopExp_Explorer(sh, TopAbs_FACE)
    out = []
    while ex.More():
        out.append(TopoDS.Face_s(ex.Current()))
        ex.Next()
    return out


def dist(a, b):
    d = BRepExtrema_DistShapeShape(a, b)
    d.Perform()
    return d.Value() if d.IsDone() else None


def find_face(faces, p):
    v = BRepBuilderAPI_MakeVertex(gp_Pnt(*p)).Vertex()
    best = (1e30, None)
    for i, f in enumerate(faces):
        d = dist(v, f)
        if d is not None and d < best[0]:
            best = (d, i)
    return best


def main():
    wasm = arg('--wasm', os.path.join(HERE, '..', 'target', 'wasm32-unknown-unknown', 'release', 'ck_wasm.wasm'))
    samples = arg('--samples', '/path/to/work/kit/apps/cad/public/samples')
    n = arg('--pairs', '20')
    tol = float(arg('--tol', '1e-6'))
    parts = arg('--parts', 'mounting-plate,parametric-bracket,stepper-bracket,bench-pipe_elbow,large-sample').split(',')
    out = tempfile.mkdtemp(prefix='measure_occt_')
    worst, total, bad = 0.0, 0, []
    for name in parts:
        f = os.path.join(samples, name + '.ckpart')
        if not os.path.exists(f):
            print(f'skip {name}: no file')
            continue
        r = subprocess.run(['node', os.path.join(HERE, 'measure_occt_dump.mjs'), f, out, '--pairs', n, '--wasm', wasm], capture_output=True, text=True)
        if r.returncode:
            print(f'skip {name}: {r.stderr.strip()[-200:]}')
            continue
        print(r.stdout.strip())
        faces = faces_of(os.path.join(out, name + '.step'))
        pairs = json.load(open(os.path.join(out, name + '.pairs.json')))
        cache = {}

        def occt_face(fid, p):
            if fid not in cache:
                d, i = find_face(faces, p)
                cache[fid] = i if d < 1e-5 else None
            return cache[fid]

        for pr in pairs:
            ia, ib = occt_face(pr['a'], pr['ptA']), occt_face(pr['b'], pr['ptB'])
            if ia is None or ib is None:
                continue
            want = dist(faces[ia], faces[ib])
            got = pr['ours']['distance']
            err = abs(got - want) / max(1.0, abs(want))
            total += 1
            worst = max(worst, err)
            if err > tol:
                bad.append((name, pr['a'], pr['b'], got, want))
    print(f'{total} face pairs compared, worst error {worst:.3e}')
    for b in bad:
        print('MISMATCH %s %s %s ours %.9g occt %.9g' % b)
    sys.exit(1 if bad else 0)


main()
