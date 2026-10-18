#!/usr/bin/env python3.12
"""OCP read-back for tools/step_roundtrip_sweep.mjs."""
import json, re, sys, collections, os
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


def types_of(text):
    c = collections.Counter(re.findall(r'^#\d+\s*=\s*([A-Z0-9_]+)', text, re.M))
    return dict(c.most_common(8))


def read(file):
    text = open(file, 'r', errors='replace').read()
    r = STEPControl_Reader(); status = r.ReadFile(file)
    if status != IFSelect_RetDone:
        return {'error': f'read status {status}', 'types': types_of(text)}
    r.TransferRoots()
    sh = r.OneShape()
    valid = BRepCheck_Analyzer(sh).IsValid()
    p = GProp_GProps(); BRepGProp.VolumePropertiesGK_s(sh, p, 1e-9, True)
    a = GProp_GProps(); BRepGProp.SurfaceProperties_s(sh, a)
    return {'valid': valid, 'volume': abs(p.Mass()), 'area': abs(a.Mass()), 'types': types_of(text)}


def main():
    cpath = arg('--cases')
    cases = json.load(open(cpath)) if os.path.exists(cpath) else json.loads(cpath)
    out = {}
    for c in cases:
        try:
            out[c['id']] = read(c['file'])
        except Exception as e:
            out[c['id']] = {'error': f'{type(e).__name__}: {e}'}
    json.dump(out, open(arg('--out'), 'w'), indent=1)


if __name__ == '__main__':
    main()
