# Decision memo: kinematic joints for the Woodwork plugin

Status: proposal for the product owner. Docs only; nothing here changes behaviour. Date: 2026-11-20.

## The question

The Woodwork plugin (third party, run by the kit's plugin host) cannot wire its joint and placement commands: doors and drawers do not swing, and `Place` makes no link between cabinets. The assumption was that the `ck-part` document has no kinematic joints. Reading the code, that is only half true, and the half that is false changes the options.

## What the format has today

Read from `packages/part-model/src/{index,features}.ts`, `crates/ck-model/src/features/{mate,mate_kinds}.rs`, `docs/adr/0004-multiple-bodies-in-one-part.md`, `docs/mjcf.md`, and the kit's plugin host (`kit/packages/plugin-wasm/src/interpreter.ts`, `kit/docs/plugins.md`).

- A part is a flat list of features regenerating to one B-rep that may hold several lumps ("bodies"). Bodies can be named (ADR 0004 is "Proposed"; `newBody` and `body` exist in the types, `{body: "Name"}` is a selector). `moveBody` rotates and translates lumps, with expression-valued angle and translation, optionally as a copy.
- **There is already an assembly layer.** `insertPart` embeds a whole nested part document as a component (placed by `place` or by `dx/dy/dz` + axis + angle, optionally `fixed`). `mate` features constrain two components through face references. Kinds today: coincident, concentric, distance, angle, parallel, perpendicular, tangent, lock, symmetric, width, **hinge** (two cylinder axes made concentric plus an optional axial stop), **slot**, screw, gear, rackPinion, and **limits** (min / max of a distance or angle). The kernel solves the mates one mover at a time (Gauss-Seidel) and reports degrees of freedom per component (`regen` with `opts.dofReport`; `components[] = {id, dof, place}`). `packages/mjcf` already turns such an assembly into MuJoCo joints (hinge, slide, ball, free) and stores motion studies in the document (`doc.motion`, ignored by the kernel).
- What it does not have: a joint as an object. There is no joint value (a door at 35 degrees is expressed by an `angle` mate between two faces, not a stored "joint position"), no joint frame (a mate is face-to-face: a hinge needs two cylindrical faces, a slide needs a pin and a planar wall), no joint between two bodies inside one part, and no placement link ("cabinet B sits to the right of cabinet A" is lost once `Place` has moved the bodies; `moveBody` is a one-off move, not a relation).

Why Woodwork cannot use it: the plugin host runs SolidCraft-style commands on bodies in one part (`solid.box`, `component.create`). Its components are just a recorded feature range; `component.*`, `occurrence.*` and `joint.*` are inert stubs. `joint.create` stores the request in memory ("kept for a later assembly, nothing moves yet") and `occurrence.move` is implemented as a `solid.move` of the bodies made by that component, which is the whole of `Place`. The cabinet is not a nested part and has no faces named for a mate. In short: the kernel has assemblies, but the plugin's data model (bodies and axes in one part, joints defined by an origin and axis) does not map onto them.

## Joint kinds Woodwork needs

1. **Revolute (hinge)**: door on a side or top edge, flap, lid. Axis (origin and direction), limits (for example 0 to 110 degrees), current angle.
2. **Prismatic (slide)**: drawer runner, sliding door, pull-out. Direction, limits, current travel.
3. **Fixed / placement link**: cabinet to cabinet, panel to carcass. Frame of B relative to frame of A, with an optional offset and gap. Moves A, moves B. This is the `Place` problem and is by far the most common.
4. **Rigid mate with offset**: a special case of 3 with a stated gap (reveal between doors, shelf pin spacing).

Not needed for v1: gears, screws, ball joints, loops. Driven animation is nice, not required.

## How mainstream tools model it (as far as I know; guesses marked)

- **SolidWorks mates**: constraints between faces, edges, planes or points of two components in an assembly (coincident, concentric, distance, angle, and advanced mates such as hinge, slot, width, limit). A mate does not store a position; the solver finds one that satisfies all the mates, and leftover degrees of freedom are what the user drags (a door can swing because only a concentric and an axial mate are applied). Limits are limit mates. I am confident of this. The mate set is close to what our kernel already has.
- **Onshape mates and mate connectors**: each component has mate connectors (a local coordinate system placed on geometry); a mate (fastened, revolute, slider, planar, cylindrical, pin slot, parallel, ball) connects two connectors and the type fixes what is free. Each mate carries limits and a current value. I am fairly confident; the key idea is joints defined by frames, not faces. This is the closest match to what Woodwork sends (origin and axis).
- **Fusion joints**: a joint is created between two components using a joint origin on each (a frame snapped to geometry) with a motion type (rigid, revolute, slider, cylindrical, pin-slot, planar, ball), limits, and a drive value; "as-built joints" are joints on components already in position. I know this reasonably well; the exact internal storage format I do not know. Guess: the document stores the joint type, the two origins and the current value, and regenerates positions from them.

The common idea, and the one the SolidCraft command set Woodwork was written against follows (`joint.create`, `joint.as_built`, `joint.limits`): a joint is a record with a type, two frames and limits.

## Design options for this format

**Option A: joints as features referencing two bodies and two frames.** A new feature `joint` with `kind: revolute | slider | fixed`, `a` and `b` as body names (`{body: "Door"}`), a frame on each (origin, axis and x direction, as an expression), `value` (angle or travel, expression, parameter-drivable), optional `min` / `max`. Regeneration applies the transform of `b` relative to `a` as a rigid move (the kernel already has rotate-about-axis and translate in `moveBody`). No solver: a tree of joints, evaluated in order.
- Kernel work: small to moderate. New feature type and `known_fields` entry, evaluation reusing the move code, errors for cycles and unknown bodies. About 3 to 5 days.
- GUI work: a joint list in the tree, a value slider per joint (writes the expression, one undo step per drag), a frame gizmo. Moderate, 3 to 5 days; a read-only version is almost free (the pose already shows).
- Compatibility and versioning: a new feature type means an older kernel answers `unknownFeature` (regen.rs). Either bump `version` (the loader already tolerates a higher one) or accept that old readers refuse such files. Part-model types, the skill registry and `known_fields.rs` must be updated together.
- Undo / redo: free, a joint is a feature and its value an expression.
- Regen cost: one extra rigid move per joint; no change to the face-naming scheme. The moved body keeps its names (as `moveBody` does today).
- Weakness: parallel to the existing mate system, so there will be two ways to say "hinge". Must be said clearly in docs, or the joint feature should later lower onto the mate solver.

**Option B: use the existing assembly layer** (`insertPart` plus `mate`), and teach the plugin host to emit it.
- Kernel work: little. Mates are face-referenced, so the host must either name real faces (a hinge needs two cylindrical faces, a placement link needs planar faces) or the kernel gains a frame-referenced mate. Doors would need hinge pin cylinders modelled in the part. The plugin's `joint.create` (origin and axis) has nothing to snap to. Likely needs a new "frame or axis" reference kind in mates: 4 to 6 days.
- Host work: each Woodwork component becomes a nested part document; this is a rewrite of the interpreter's component stubs, plus a naming plan for faces. Moderate to large, 5 to 8 days, and the current "component is a feature range" shortcut (and `pluginRuns` parameter editing, which rebuilds a folder in place) would have to be revisited.
- Compatibility: no new feature types. Existing `mate` and `insertPart` are in files already.
- Undo / redo: as for any feature.
- Regen cost: highest. Each component is a nested regen (cached per distinct part), then the mate solver runs; the DOF report is a numerical Jacobian per component. Fine for tens of components, a concern for a kitchen of hundreds of panels.
- Gain: one concept, and MJCF export already works. Loss: more work in the host, and nested-part editing in the GUI is a heavier workflow than a cabinet-as-bodies.

**Option C: plugin-private metadata in the document, no kernel semantics.** Store `joints` and `placements` as a kernel-ignored block next to `pluginRuns` (the way `doc.motion` already is). The host records the request; a plugin or the GUI applies poses itself by emitting `moveBody` features.
- Kernel work: none. GUI work: small (display and a slider that rewrites a `moveBody` angle). Compatibility: best; old kernels ignore the block.
- Weaknesses: no semantics. Other tools and the agent cannot ask the kernel "what moves"; nothing validates limits; MJCF export cannot see it; two plugins would invent two formats. Fine as a first step, a trap as the end state.

| | Kernel | GUI | Old files / versions | Undo | Regen | Notes |
|---|---|---|---|---|---|---|
| A joint feature | 3-5 d | 3-5 d | new feature type, bump | free | one move per joint | two ways to say hinge |
| B assembly layer | 4-6 d (frame mates) | 5-8 d host | none new | free | heaviest | one concept, big host change |
| C plugin metadata | 0 | 1-2 d | best | partial | none | no semantics |

(Estimates from reading the code, no prototype.)

## Minimal viable proposal

Do A in its smallest form, and let C carry the plugin-side bookkeeping.

1. Part model: `joint` feature `{type: 'joint', kind: 'revolute' | 'slider' | 'fixed', a: BodyRef, b: BodyRef, origin: [Expr x3], axis: [Expr x3], value?: Expr, min?: Expr, max?: Expr}`, joint frame given in the frame of `a`. `fixed` ignores axis and value and takes an optional `offset`.
2. Kernel: evaluate in feature order as "move `b` by the joint transform", reusing the `moveBody` path; refuse a value outside `[min, max]` with a structured error naming the joint (consistent with how `mate` limits report); cycles refused. A joint on a body already moved by another joint composes.
3. Host: `joint.create` writes a `joint` feature; `joint.limits` edits min / max; `joint.edit` edits value; `joint.list` reads them back; `occurrence.move` that is a Place also writes a `fixed` joint between the two cabinets (so the link survives re-running the plugin and moving cabinet A moves cabinet B). `component.*` remains a folder of bodies.
4. GUI (second step): a value field per joint; slider later.
5. Do not touch the mate solver. Add one line to the docs saying mates are for assemblies of parts and joints are for bodies inside a part, and that a joint may later be lowered to a mate.

Open technical point to verify before building: whether a body moved by one feature keeps being addressable by `{body}` and by face name through later features and through plugin Update (the folder rebuild in place described in `kit/docs/plugins.md`). `moveBody` says face names are unchanged, which is encouraging, but I have not run it.

## Recommendation

Option A as the core, with C used only for host-side bookkeeping that has no kernel meaning. Reasons: it is the smallest thing that makes doors swing and cabinets stay attached; it keeps the plugin's own data model (bodies, origin and axis); it needs no solver; undo, parameters and persistent names come for free; and it avoids the heavy rewrite option B implies. Option B is the right long-term home for true assemblies of separate parts and should stay as that; do not stretch it to cover cabinets. Option C alone is quick but leaves the doors unable to move in any consumer except the plugin.

## Product questions to answer first

1. Is a Woodwork cabinet one part (bodies in one document) or an assembly of parts? Everything above depends on this. Do users expect to open a door panel as its own part?
2. What must move: door and drawer swing only, or whole-run placement (move a cabinet and its neighbours follow)? Is the `Place` link required to survive editing and re-running the plugin?
3. Is animation or driving needed (a slider, a timeline), or only a posed state saved in the file?
4. Should limits be enforced (the door cannot go past 110 degrees) or advisory?
5. Do joints need to survive in files opened by older builds? If yes: bump the version and accept a refusal, or keep the block kernel-ignored (option C)?
6. Collision: should doors stop at a neighbour or just pass through? (The kernel has no collision in a pose today.)
7. Who owns the format: do we offer joints to any plugin (public contract, versioned, documented in the skill registry), or is this Woodwork-only? A public contract changes how careful the design must be.
8. Scale: how many components in a typical job (a kitchen can be hundreds of panels)? This decides whether option B's regen cost is acceptable.
9. Are MJCF export or simulation of cabinets ever wanted? If not, one of the arguments for reusing the mate layer disappears.
