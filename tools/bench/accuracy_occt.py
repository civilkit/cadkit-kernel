"""OCCT as an independent judge for tools/bench/accuracy.mjs (python3.12 + cadquery-ocp / OCP).

usage: python3.12 accuracy_occt.py <stage> --dir DIR [--only id,id] [--procs 3]
  stage: step    GProp / BRepCheck / counts of every STEP the kernels exported (DIR/step/{ours,orig}/*.step)
                 and of the BenchCAD ground-truth STEPs (tools/bench/benchcad/gt/*.step)
         mesh    mesh-divergence volume with decreasing BRepMesh deflection + Richardson extrapolation
         mc      Monte Carlo volume with BRepClass3d_SolidClassifier (binomial error bars)
                 vs the kernels' volume after it (DIR/prefix_step/*.step, DIR/prefix.json)
         all     the four above
Writes/merges DIR/occt.json.  Independent of both kernels: only the exported STEP files are read.
Volumes are in mm^3; the STEP of the kernels is Y-up, the ground truth STEP is Z-up (scalars only compared).
"""
import json, math, os, random, sys, time, traceback
from multiprocessing import Pool

from OCP.STEPControl import STEPControl_Reader
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_IN, TopAbs_REVERSED
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt
from OCP.BRepClass3d import BRepClass3d_SolidClassifier

HERE = os.path.dirname(os.path.abspath(__file__))
GT_DIR = os.path.join(HERE, 'benchcad', 'gt')


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def read_step(path):
    r = STEPControl_Reader()
    if r.ReadFile(path) != 1:
        raise RuntimeError('STEP read failed')
    r.TransferRoots()
    return r.OneShape()


def count(shape, kind):
    ex = TopExp_Explorer(shape, kind)
    n = 0
    while ex.More():
        n += 1
        ex.Next()
    return n


def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b)
    x0, y0, z0, x1, y1, z1 = b.Get()
    return [x0, y0, z0, x1, y1, z1]


def metrics(shape, eps=True):
    out = {}
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g, False, False, False)
    out['volume'] = g.Mass()
    c = g.CentreOfMass()
    out['com'] = [c.X(), c.Y(), c.Z()]
    g2 = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, g2, False, False)
    out['area'] = g2.Mass()
    if eps:  # OCCT's adaptive integrator with its own error estimate (relative eps 1e-7)
        g3 = GProp_GProps()
        e = BRepGProp.VolumeProperties_s(shape, g3, 1e-7, False, False)
        out['volume_adaptive'] = g3.Mass()
        out['volume_adaptive_err'] = e
        g4 = GProp_GProps()
        e4 = BRepGProp.SurfaceProperties_s(shape, g4, 1e-7, False)
        out['area_adaptive'] = g4.Mass()
        out['area_adaptive_err'] = e4
    out['solids'] = count(shape, TopAbs_SOLID)
    out['shells'] = count(shape, TopAbs_SHELL)
    out['faces'] = count(shape, TopAbs_FACE)
    t = time.time()
    out['valid'] = bool(BRepCheck_Analyzer(shape).IsValid())
    out['check_s'] = time.time() - t
    out['bbox'] = bbox(shape)
    return out


def step_job(job):
    key, path = job
    t = time.time()
    try:
        s = read_step(path)
        m = metrics(s)
        m['secs'] = time.time() - t
        return key, m
    except Exception as e:  # noqa: BLE001
        return key, {'error': f'{type(e).__name__}: {e}'[:200]}


def solids_of(shape):
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    out = []
    while ex.More():
        out.append(TopoDS.Solid_s(ex.Current()))
        ex.Next()
    return out


def mesh_volume(shape, lin, ang=0.3):
    BRepTools.Clean_s(shape)
    BRepMesh_IncrementalMesh(shape, lin, False, ang, True)
    vol = 0.0
    area = 0.0
    tris = 0
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            tr = loc.Transformation()
            rev = f.Orientation() == TopAbs_REVERSED
            nodes = []
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(tr)
                nodes.append((p.X(), p.Y(), p.Z()))
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if rev:
                    b, c = c, b
                p0, p1, p2 = nodes[a - 1], nodes[b - 1], nodes[c - 1]
                vol += (p0[0] * (p1[1] * p2[2] - p1[2] * p2[1]) - p0[1] * (p1[0] * p2[2] - p1[2] * p2[0]) + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0])) / 6.0
                ux, uy, uz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
                vx, vy, vz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
                area += 0.5 * math.sqrt((uy * vz - uz * vy) ** 2 + (uz * vx - ux * vz) ** 2 + (ux * vy - uy * vx) ** 2)
                tris += 1
        ex.Next()
    return vol, area, tris


def mesh_job(job):
    key, path, rels, budget = job
    try:
        s = read_step(path)
        diag = math.dist(bbox(s)[:3], bbox(s)[3:])
        rows = []
        t0 = time.time()
        for rel in rels:  # coarse -> fine
            lin = rel * diag
            v, a, n = mesh_volume(s, lin, max(0.3 * math.sqrt(rel / 4e-3), 0.02))  # angular deflection shrinks with sqrt(linear): sag ~ R theta^2 / 8
            rows.append({'rel': rel, 'lin': lin, 'volume': v, 'area': a, 'tris': n, 't': time.time() - t0})
            if time.time() - t0 > budget:
                break
        # Richardson: deficit is ~linear in the deflection (inscribed polyhedron): V0 ~ 2 V(d/2) - V(d)
        ext = []
        for i in range(1, len(rows)):
            r = rows[i - 1]['lin'] / rows[i]['lin']
            ext.append((r * rows[i]['volume'] - rows[i - 1]['volume']) / (r - 1))
        return key, {'rows': rows, 'richardson': ext}
    except Exception as e:  # noqa: BLE001
        return key, {'error': f'{type(e).__name__}: {e}'[:200]}


def mc_job(job):
    key, path, budget, seed = job
    try:
        s = read_step(path)
        sol = solids_of(s)
        if not sol:
            return key, {'error': 'no solid'}
        sh = sol[0] if len(sol) == 1 else s
        x0, y0, z0, x1, y1, z1 = bbox(s)
        vb = (x1 - x0) * (y1 - y0) * (z1 - z0)
        cl = BRepClass3d_SolidClassifier(sh)
        rnd = random.Random(seed)
        n = hit = 0
        t0 = time.time()
        while time.time() - t0 < budget:
            p = gp_Pnt(rnd.uniform(x0, x1), rnd.uniform(y0, y1), rnd.uniform(z0, z1))
            cl.Perform(p, 1e-7)
            n += 1
            if cl.State() == TopAbs_IN:
                hit += 1
        pr = hit / max(n, 1)
        return key, {'n': n, 'hit': hit, 'volume': pr * vb, 'sigma': vb * math.sqrt(max(pr * (1 - pr), 1e-12) / max(n, 1)), 'bbox_volume': vb, 'solids': len(sol)}
    except Exception as e:  # noqa: BLE001
        return key, {'error': f'{type(e).__name__}: {e}'[:200]}


def load(dirp):
    f = os.path.join(dirp, 'occt.json')
    return json.load(open(f)) if os.path.exists(f) else {}


def save(dirp, res, *keys):
    """merge-save: stages may run concurrently, so only the stage's own top-level keys are replaced on disk."""
    cur = load(dirp)
    for k in keys:
        cur[k] = res[k]
    tmp = os.path.join(dirp, 'occt.json') + f'.{os.getpid()}.tmp'
    json.dump(cur, open(tmp, 'w'), indent=1)
    os.replace(tmp, os.path.join(dirp, 'occt.json'))


def stage_step(d, only, procs):
    res = load(d)
    res.setdefault('step', {})
    res.setdefault('gt', {})
    jobs = []
    for eng in ('ours', 'orig'):
        p = os.path.join(d, 'step', eng)
        if os.path.isdir(p):
            for f in sorted(os.listdir(p)):
                if f.endswith('.step') and (not only or f[:-5] in only):
                    jobs.append((f'{eng}/{f[:-5]}', os.path.join(p, f)))
    for f in sorted(os.listdir(GT_DIR)):
        if f.endswith('.step') and (not only or f[:-5] in only):
            jobs.append((f'gt/{f[:-5]}', os.path.join(GT_DIR, f)))
    with Pool(procs) as pool:
        for key, m in pool.imap_unordered(step_job, jobs):
            kind, name = key.split('/', 1)
            (res['gt'] if kind == 'gt' else res['step'])[key if kind != 'gt' else name] = m
            print('step', key, m.get('volume'), m.get('valid'), m.get('error', ''), flush=True)
            save(d, res, 'step', 'gt')


MESH_RELS = [4e-3, 2e-3, 1e-3, 5e-4, 2.5e-4, 1.25e-4, 6e-5, 3e-5, 1.5e-5]
MESH_PARTS_DEFAULT = ['round_flange_000341', 'pulley_000030', 'spur_gear_000098', 'ball_knob_007575', 'torus_link_000095', 'dome_cap_000212', 'bolt_000008', 'enclosure_000042']


def stage_mesh(d, only, procs):
    res = load(d)
    res.setdefault('mesh', {})
    ids = only or MESH_PARTS_DEFAULT
    jobs = []
    for i in ids:
        for eng in ('ours', 'orig'):
            p = os.path.join(d, 'step', eng, f'{i}.step')
            if os.path.exists(p) and not (eng == 'orig' and i.startswith('hair')):  # the original writes OFFSET_SURFACE faces OCCT cannot read
                jobs.append((f'{eng}/{i}', p, MESH_RELS, 240))
    p = os.path.join(GT_DIR, 'x')
    for i in ids:
        g = os.path.join(GT_DIR, f'{i}.step')
        if os.path.exists(g):
            jobs.append((f'gt/{i}', g, MESH_RELS, 240))
    with Pool(procs) as pool:
        for key, m in pool.imap_unordered(mesh_job, jobs):
            res['mesh'][key] = m
            print('mesh', key, m.get('richardson', [None])[-1:], m.get('error', ''), flush=True)
            save(d, res, 'mesh')


MC_PARTS_DEFAULT = ['round_flange_000341', 'pulley_000030', 'ball_knob_007575', 'dome_cap_000212', 'torus_link_000095', 'bolt_000008']


def stage_mc(d, only, procs):
    res = load(d)
    res.setdefault('mc', {})
    ids = only or MC_PARTS_DEFAULT
    jobs = []
    for i in ids:
        for eng in ('ours',):
            p = os.path.join(d, 'step', eng, f'{i}.step')
            if os.path.exists(p):
                jobs.append((f'{eng}/{i}', p, float(arg('--mc-budget', 90)), 12345))
    with Pool(procs) as pool:
        for key, m in pool.imap_unordered(mc_job, jobs):
            res['mc'][key] = m
            print('mc', key, m, flush=True)
            save(d, res, 'mc')


def eps_job(job):
    key, path = job
    try:
        s = read_step(path)
        rows = []
        g = GProp_GProps()
        BRepGProp.VolumeProperties_s(s, g, False, False, False)
        rows.append({'eps': 'default (fixed Gauss)', 'volume': g.Mass()})
        for eps in (1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9):
            g = GProp_GProps()
            t = time.time()
            e = BRepGProp.VolumeProperties_s(s, g, eps, False, False)
            rows.append({'eps': eps, 'volume': g.Mass(), 'err_estimate': e, 'secs': time.time() - t})
        return key, rows
    except Exception as e:  # noqa: BLE001
        return key, {'error': f'{type(e).__name__}: {e}'[:200]}


def stage_eps(d, only, procs):
    """How much OCCT's own volume of ONE fixed STEP moves with its integrator setting (the yardstick's own error bar)."""
    res = {'eps': {}}
    ids = only or ['eyebolt_000265', 'worm_screw_000021', 'coil_spring_000026', 'propeller_000192', 'tapered_boss_000256']
    jobs = [(f'ours/{i}', os.path.join(d, 'step', 'ours', f'{i}.step')) for i in ids if os.path.exists(os.path.join(d, 'step', 'ours', f'{i}.step'))]
    jobs += [(f'gt/{i}', os.path.join(GT_DIR, f'{i}.step')) for i in ids if os.path.exists(os.path.join(GT_DIR, f'{i}.step'))]
    with Pool(procs) as pool:
        for key, rows in pool.imap_unordered(eps_job, jobs):
            res['eps'][key] = rows
            print('eps', key, flush=True)
            save(d, res, 'eps')


def stage_prefixvol(d, only, procs):
    import accuracy_prefix_vol
    save(d, {'prefixvol': accuracy_prefix_vol.run(d, procs)}, 'prefixvol')


def stage_replay(d, only, procs):
    try:
        import accuracy_replay
    except ImportError:
        sys.path.insert(0, HERE)
        import accuracy_replay
    res = {'replay': accuracy_replay.run(d)}
    save(d, res, 'replay')


if __name__ == '__main__':
    stage = sys.argv[1]
    d = arg('--dir')
    only = arg('--only')
    only = only.split(',') if only else None
    procs = int(arg('--procs', 3))
    sys.path.insert(0, HERE)
    stages = ['step', 'eps', 'prefixvol', 'mesh', 'mc', 'replay'] if stage == 'all' else [stage]
    for s in stages:
        try:
            {'step': stage_step, 'eps': stage_eps, 'prefixvol': stage_prefixvol, 'mesh': stage_mesh, 'mc': stage_mc, 'replay': stage_replay}[s](d, only, procs)
        except Exception:  # noqa: BLE001
            traceback.print_exc()
