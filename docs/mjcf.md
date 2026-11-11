# MuJoCo interop (MJCF export, trajectory import)

`packages/mjcf` turns an assembly (a part document with `insertPart` components and `mate` features) into a MuJoCo MJCF model,
and reads a joint log from a MuJoCo run back as a motion study on the same components. It uses only the kernel's public requests
(`regen` with `opts.dofReport`, `massProps`, `measure`, `describe`, `angles`); nothing in the kernel changed. Plan:
`docs/superpowers/plans/2026-11-08-mjcf-interop.md`.

## Usage

```ts
import { createWorkerKernel } from '../kernel-client/src/worker.js';
import { exportMjcf, writeMjcf, readTrajectory, motionStudy, withMotionStudy, posesAt } from '../mjcf/src/index.js';

const k = await createWorkerKernel();
const r = await exportMjcf(k, assemblyDoc, {
  collision: 'solids',                       // 'hull' (default) | 'solids' | 'primitive' | 'none'
  collisionByPart: { Ball: 'primitive' },    // per component or part name
  visualOnly: ['Cover'], static: ['Bench'],  // no collision geom / welded to the world
  up: 'z',                                   // 'y' turns a y-up CAD model so +y is MuJoCo's up
  sim: { option: { timestep: 0.001 }, actuators: [{ type: 'position', name: 'arm', joint: 'Arm_pivot', kp: 5 }] },
});
await writeMjcf('out', r);                   // out/<model>.xml, out/meshes/*.stl, out/<model>.meta.json
// later, after a run that logged time + qpos per joint:
const { study, unknown } = motionStudy(r.meta, readTrajectory(csvText), { name: 'run 7' });
const doc2 = withMotionStudy(assemblyDoc, study);   // stored as doc.motion.studies (ignored by the kernel)
const poses = posesAt(r.plan, study, 0.25);         // component id -> world placement (mm) at t = 0.25 s
```

Command line (after `cd packages && npm run build`):

```
node tools/mjcf.mjs export assembly.ckpart out --sim sim.json [--options opts.json] [--wasm kernel.wasm]
node tools/mjcf.mjs motion assembly.ckpart run.csv assembly-run.ckpart --meta out/<model>.meta.json [--name N] [--units m|mm]
python tools/mjcf_check.py out/<model>.xml [--qpos '{"joint": v}'] [--steps N]   # what MuJoCo compiled, as JSON
```

## Mapping

| CAD | MJCF |
|---|---|
| component, solved placement `components[].place` | `<body>`, `pos`/`quat` relative to the parent body; metres |
| `fixed` component, `static` option | body without a joint (welded) |
| component without mates | `<freejoint>` (`unmated: 'fixed'` welds it) |
| mates between a component and its parent | the free directions left by the mates: `slide` per free translation, `hinge` per free rotation (on the axis of a hinge / concentric / screw mate when one is parallel), `ball` for three rotations (at a sphere mate's centre) |
| `hinge` with axial stop | hinge; without stop, or `concentric`: hinge + slide (cylindrical) |
| `limits` (`of: angle` / distance) | joint `range` relative to the exported pose: the CAD angle is unsigned (0..180), so `0..110` at 0 becomes `-110..110` deg |
| `gear`, `rackPinion`, `screw` | `<equality><joint polycoef>` between the joints, same sign convention as the kernel's solver |
| part (standalone regen, its own frame) | one STL in metres; visual geom (class `visual`, group 2) + collision geoms (class `collision`, group 3) |
| `massProps` of the part alone x density | explicit `<inertial pos mass fullinertia>` on every body; `inertiafromgeom="false"` |
| names | body = component name, joint = mate name (`_slide` / `_hinge` when one mate gives two), mesh = part name; sanitised, unique; the map is in `<model>.meta.json` |

The exported pose is MuJoCo's `qpos0`, so every joint reads 0 at the CAD pose and `mj_resetData` returns to it.

Tree: fixed components are roots; the others hang off the component they are mated to (breadth first). A mate that closes a loop is
reported (MuJoCo does not enforce it; add an `<equality>` in `sim.raw`). For components whose parent is fixed the joint counts are
checked against the kernel's DOF report; a mismatch is a warning, and `options.joints` states joints by hand
(`{type: 'hinge' | 'slide' | 'ball' | 'free' | 'weld', axis, pos, range, name, parent}`, world frame, mm).

Densities: the part's `material` in the same table as the GUI (`DENSITIES`), `options.densities` over it, `defaultDensity` (1 g/cm3,
with a warning) for unknown names.

## Collision

MuJoCo collides a mesh as its convex hull.

- `hull`: the visual mesh is the collision mesh (one hull per part).
- `solids`: one mesh per solid (lump) of the part; `hints` (and `meta.hints`) lists every solid with `convex` from the B-rep (no concave
  edge, no concave face); non-convex ones are the input for an external CoACD / V-HACD run, whose pieces replace that geom.
- `primitive`: box / cylinder / sphere / capsule when the part is one solid whose face kinds and exact volume and principal moments match
  the primitive (1e-4); otherwise the hull, with a warning.

No decomposition runs inside the exporter: the kernel has no convex decomposition and the package takes no dependencies.

## Sim-only block

`sim` (or a JSON side file read with `readSimFile`) holds what the CAD does not: `option`, `compiler`, `contact.default` /
`contact.bodies[name]` (friction, solref, solimp, condim, margin, gap, priority, contype, conaffinity), `contact.exclude`,
`joints[name | '*']` (damping, armature, frictionloss, stiffness...), `actuators`, `sensors`, raw MJCF for `worldbody` (balls, floors,
lights), `asset` and the end of the model (`raw`), and `include` files. Names refer to the exported names, so the sim code can attach
bodies and actuators without changing the export.

## Trajectory import

`readTrajectory` reads CSV (`time,<joint>,...`, a `qpos.` prefix is dropped) or JSON (`{time, qpos: {joint: []}}`,
`{time, joints: [], qpos: [[]]}`, or rows `[{time, qpos: {}}]`). `motionStudy` converts rad to degrees and m to mm (`units: 'mm'` for a
log already in mm), keys tracks by joint name with the component and mate, and lists columns it cannot place (a mate id or name works
as a column when it gives exactly one joint). `posesAt` composes the joints exactly as MuJoCo does (checked against MuJoCo body poses in
`mjcf.kernel.test.ts`).

## Tests

`packages/mjcf/src/mjcf.test.ts` (no kernel) and `mjcf.kernel.test.ts` (kernel wasm; the MuJoCo part runs when `MUJOCO_PYTHON` names a
python with the `mujoco` package, e.g. a venv). The fixture (`fixtures.ts`) is a fixed base with a rod, a carriage sliding on the rod with
travel limits, an arm hinged on the carriage with swing limits, and a free ball.

## Not done

- MJCF import (bodies / joints / primitive geoms -> assembly): not implemented. It would need the inverse of the joint mapping (a joint
  is not a mate) and mesh import; `parseXml` is there for whoever does it.
- Showing a motion study in the GUI (kit repo): the study is stored in the document and `posesAt` gives the placements; the kit has no
  timeline yet.
- Copies of components made by component patterns are not exported (warning).
- Tendons, flex bodies, and contact pairs beyond `exclude` go through `sim.raw`.
