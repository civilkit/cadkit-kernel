"""OCCT volume (adaptive, eps 1e-7) of every prefix STEP of the hair dryer (our kernel's body after each feature),
next to the volume both kernels report for that prefix. Called by accuracy_occt.py stage `prefixvol`."""
import json, os, sys, time
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import accuracy_occt as A  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402


def job(a):
    fid, path = a
    try:
        s = A.read_step(path)
        g = GProp_GProps()
        e = BRepGProp.VolumeProperties_s(s, g, 1e-7, False, False)
        g2 = GProp_GProps()
        BRepGProp.VolumeProperties_s(s, g2, False, False, False)
        ga = GProp_GProps()
        BRepGProp.SurfaceProperties_s(s, ga, 1e-7, False)
        return fid, {'volume_adaptive': g.Mass(), 'err': e, 'volume_gauss': g2.Mass(), 'area_adaptive': ga.Mass(), 'solids': A.count(s, A.TopAbs_SOLID),
                     'shells': A.count(s, A.TopAbs_SHELL), 'faces': A.count(s, A.TopAbs_FACE), 'valid': bool(A.BRepCheck_Analyzer(s).IsValid())}
    except Exception as ex:  # noqa: BLE001
        return fid, {'error': f'{type(ex).__name__}: {ex}'[:160]}


def run(d, procs=3):
    sp = os.path.join(d, 'prefix_step')
    jobs = [(f[:-5], os.path.join(sp, f)) for f in sorted(os.listdir(sp)) if f.endswith('.step')]
    out = {}
    with Pool(procs) as pool:
        for fid, r in pool.imap_unordered(job, jobs):
            out[fid] = r
            print('prefixvol', fid, r.get('volume_adaptive'), flush=True)
    return out
