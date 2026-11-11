#!/usr/bin/env python3
"""Load an MJCF model with MuJoCo and print what it compiled to, as JSON (used by packages/mjcf tests).

    python tools/mjcf_check.py MODEL.xml [--qpos '{"joint": value, ...}'] [--steps N]

Output: {"bodies": {name: {"pos", "quat", "mass", "inertia"}}, "joints": {name: {"type", "axis", "anchor", "range", "limited"}},
"nq", "nv", "qpos": {joint: value}} with world poses after mj_forward at qpos0 (or at the given joint values). --steps
runs that many simulation steps first and reports the final joint values. Needs the `mujoco` python package.
"""
import argparse, json, sys

import mujoco
import numpy as np

TYPES = {0: 'free', 1: 'ball', 2: 'slide', 3: 'hinge'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('--qpos', default='{}')
    ap.add_argument('--steps', type=int, default=0)
    a = ap.parse_args()
    m = mujoco.MjModel.from_xml_path(a.model)
    d = mujoco.MjData(m)
    for name, v in json.loads(a.qpos).items():
        j = m.joint(name)
        adr = m.jnt_qposadr[j.id]
        vals = v if isinstance(v, list) else [v]
        d.qpos[adr:adr + len(vals)] = vals
    for _ in range(a.steps):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    bodies = {}
    for i in range(1, m.nbody):
        b = m.body(i)
        bodies[b.name] = dict(pos=d.xpos[i].tolist(), quat=d.xquat[i].tolist(), mass=float(m.body_mass[i]),
                              inertia=m.body_inertia[i].tolist(), ipos=m.body_ipos[i].tolist(), parent=m.body(m.body_parentid[i]).name)
    joints, qpos = {}, {}
    for i in range(m.njnt):
        j = m.joint(i)
        t = TYPES[int(m.jnt_type[i])]
        joints[j.name] = dict(type=t, axis=d.xaxis[i].tolist(), anchor=d.xanchor[i].tolist(), range=m.jnt_range[i].tolist(),
                              limited=bool(m.jnt_limited[i]), body=m.body(m.jnt_bodyid[i]).name)
        adr = m.jnt_qposadr[i]
        n = {'free': 7, 'ball': 4}.get(t, 1)
        qpos[j.name] = d.qpos[adr:adr + n].tolist() if n > 1 else float(d.qpos[adr])
    json.dump(dict(bodies=bodies, joints=joints, qpos=qpos, nq=int(m.nq), nv=int(m.nv), time=float(d.time)), sys.stdout)


if __name__ == '__main__':
    main()
