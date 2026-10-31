# ck-model

The document model and feature program: parses .ckpart JSON, resolves
references, evaluates expressions, runs the regen program by calling into
ck-topo / ck-ops. Depends on ck-core, ck-geom, ck-topo, ck-ops, ck-json.

## Modules

- `lib.rs` — Part document (format "ck-part" v1), Plane, Ref, Expr
- `sketch/` — 2D sketch: mod (points/entities/constraints), solver.rs
  (the constraint SOLVER — this is a core algorithm), offset.rs, profile.rs
  (profile/loop detection), smooth.rs, spline.rs
- `sketch3/` — 3D sketches + solver
- `features/` — one module per feature family; payload field tables in
  features/README.md; ids look like "F4", entity refs "e1@F1.1"
- `expr.rs` — expression evaluator for dimension values ("20", "d/2")
- `refs.rs` — reference resolution ("F3.e2" -> geometry)
- `regen.rs` — the regen driver: ordering, upTo, rollback, caching
  ("features N reused from the cache" string), error collection
- `interference.rs` — assembly interference checking

## Interface (wire)

- IN: .ckpart object inside `{type:"regen",tag:"main",part:...}`.
- OUT: see root README — errors[], components[] (assembly placement solver:
  dof + placement frames), body{faces,edges,mesh}, sketches, planes, curves.
- Feature ids are minted as `"F" + part.seq` (function `dn` in app.pretty.js).

## Rebuild notes

- Constraint solver correctness is pinned by the assembly golden:
  component placements (dof 6 free / 0 fixed / 1 sliding) came out of the
  mate solving, not the features.
- `Ot` (front/top/right frames) in app.pretty.js line 1 = std planes.

## Status (document/sketch layer)

- `Part::from_json` / `Part::parse` — lenient like the kernel; the only part
  error is `"Format"` (reply `invalid part: Format`). Every sample feature
  parses to a typed `FeatureKind`; raw DOM kept in `Feature::raw`.
- `expr` — the client `al()` grammar (units, deg trig, `^`, pi), `=expr`
  numeric fields, variable resolution (`badName`/`badExpression`/`cyclicVariables`).
- `sketch::solver` — bit-exact port of the kernel solver (verified on 22
  oracle goldens in the recorded reference data (not published)). Needs a correctly
  rounded hypot (`solver::cr_hypot`) and the kernel's LU details.
- `sketch::profile` — planar arrangement -> regions/holes/open (matches the
  oracle structurally, coordinates within 1e-7).
- `sketch::offset` — `sketchOffset` message (first pass).
- `regen` — driver: upTo/rollback/suppressed, sketches (expr -> re-solve),
  plane features, id-sorted output maps, error wire `[id,{code,params}]`.

## Status (geometry layer, first pass)

- `features::Brep` = the B-rep `FeatureExec`: `extrude` (blind / flip /
  mid / through = 1000 / throughBoth = ±1000 on an empty body, depth <= 0
  -> 1e-6, draft / draftOut one-sided) and `revolve` (axis = a sketch line
  entity by id, degrees, flip, mid, >= 360 full) onto an EMPTY body via
  ck-topo. Other geometry feature types -> `TODO(rebuild): <type> needs
  ck-ops`.
  Error codes/precedence pinned: badExpression -> missingAxis (revolve) ->
  noClosedProfile -> unknownEnd{end} / invalidValue (angle 0) ->
  missingReference (upToFace/offsetFromFace/upToPoint) / nothingToCut.
- `extrude` end conditions `upToFace` / `offsetFromFace` / `upToPoint` (`endRef`, `offset`): `extrude.rs` `plan_end` +
  `Brep::extrude_to_reference` (slanted ends trimmed by a boolean); unreachable ends fall back to blind with an
  `endConditionFallback` warning (docs/spec-agent-interface.md, "End references").
- `extrude` `end:"toObject"` (`endRef`: face, plane, point, vertex, body; `endOffset`) and `start:"fromObject"`
  (`startRef`, `startOffset`): `extrude_object.rs` + `Brep::extrude_to_object` / `trim_to_body`. Strict: a lost or
  unreachable reference is a refusal with a `hint`, never a fallback (docs/spec-agent-interface.md, "To object / From object").
- `refs::face_frame`: face sketch planes / plane bases (rule in refs.rs).
- `features::body_wire`: wire `box/faces/edges` (edge `pts` = ck-topo
  `sample_edge` stand-in; box = exact bounds ∪ f32-rounded, GUESS for
  curved bodies whose oracle box comes from the mesh).
- regen trace spans: `"<id> <type>"`, `" → N faces"` once a body exists,
  `"face names"` after each successful feature with a body, failures
  `"<id> <type>: <code>[ <params>]"` + child `"<id> failed: ..."`.
- Tests: recorded reference data (not published)
  (the reference-capture tool (not published), parts in `tools/brep_cases.mjs`):
  20/24 regen replies byte-identical minus mesh/pts (draft, draftOut ~1e-10,
  rev90mid / cone last ulp); massProps within 1e-9 (never byte-exact: the
  oracle's integration rounds differently; degenerate principal axes are
  arbitrary). `node tools/diff.mjs --cases brep` diffs the follow-up
  requests byte-for-byte.

## Status (ck-ops wiring: Gate A = bracket.ckpart)

- `Brep::combine`: cut / add onto an existing body -> `ck_ops::boolean`
  (tol `features::TOL` = 1e-6, GUESS). Trace child span
  `boolean Unite|Subtract (m × n faces) → k faces` + stage leaves
  locate/imprint/apply/classify/assemble/simplify/check (`Brep::spans`,
  drained by the server's exec wrapper via `Brep::flush_spans`).
  Extrude/revolve cut that removes nothing -> `cutNoEffect`, body
  unchanged; the boolean span is still emitted.
- extrude `op:"cut"` runs against the sketch normal unless `flip`; revolve
  cuts do NOT flip (probe). `through`/`throughBoth` with a body: length =
  |sketch origin − body box centre| + box diagonal + 10 (GUESS fitted to one
  probe to 3e-6; long enough for every cut). `upToNext` with a body
  (`Brep::up_to_next`, probes): a `through` tool; add -> `boolean Subtract
  (tool × body)`, cut -> `boolean Intersect (tool × body)`; the pieces
  touching the sketch plane are the real tool (split `~k` suffixes dropped),
  then the usual Unite / Subtract. Cut piece not at the sketch ->
  `upToNextNone`; add that never meets the body -> `upToNextOpen`.
- `Brep::tools`: every extrude/revolve keeps its tool body + `Recipe`
  (profile + sweep params). mirror / lpattern / cpattern seed on FEATURES ->
  their tool bodies (`features/pattern.rs`): copies named `@<feature>.<k>`,
  one boolean per seed (patterns append all instances of a seed into one
  tool), a patterned/mirrored cut that misses is silent; `noSeeds` /
  `missingReference`; count <= 1 -> no-op. Mirror copies are re-swept from
  the reflected profile (ck-topo transforms are rigid-only), caps renamed
  back so `C0@F10.1` is the image of C0. `mirror body:true`: the whole body
  reflected exactly (`pattern::reflect_body`: right-handed frames, v -> −v
  pcurves on analytic curved faces, senses / loops flipped), renamed
  `@<feature>.1`, united — or appended with no boolean span when the copy's
  box is clear of the body (oracle; GUESS: box test).
- chamfer -> `ck_ops::blend::build::chamfer` (`equal`, also the default);
  span `"F3: Chamfer(2.0, 2.0) of 1 edge"`; `distances` (alias `distanceDistance`) and
  `angle` (alias `angleDistance`) take `distance` on the first face of the pick (or on `face`; `flip` swaps),
  `distance2` / `angle` on the other; unknown types fail `unsupported {what:"chamfer <type>"}`
  (oracle, before anything else); `distance <= 0` -> `invalidValue` (no
  span); `chamferType:"vertex"` -> `ck_ops::blend::vertex::chamfer_vertex`
  (no span; `at` or the first edge's pick point, `distance`/`distance2`/
  `distance3`).
- fillet -> `ck_ops::blend::build::fillet` (`features/blends.rs`); span
  `"F11: Fillet(5.0, Circular) of 4 edges"`; the selected pair is reordered
  so the face whose loop runs the edge forward comes first (oracle touched
  face order, independent of selection order).
- bracket.ckpart, every prefix upTo 1..11: faces (ids/order/normals/lumps),
  box, errors, trace labels identical to the oracle, massProps within
  3e-14; wire EDGE lists have the same face-pair set but differ in order
  and closed-circle start points (ck-ops, see crates/ck-ops README
  findings). Pinned by recorded reference data (not published)
  (`--ignored`, release: ~35 s — the ck-ops classifier integrates winding
  numbers) + 12 ck-ops mini-parts in `tools/brep_cases.mjs`.
- `tools/regencmp.mjs` (`--prefixes --mass --trace --show`) diffs regen
  replies vs the oracle per body key; `examples/regen_part.rs` times each
  feature natively.

## Status (Gate B: assemblies — `features/mate.rs`, `hole.rs`, `sweep.rs`)

- `insertPart` (pinned by `tools/asm/asmcases.mjs`, 22/22 byte-identical
  minus mesh/pts incl. trace): nested regen (nested `rollback` ignored),
  faces renamed `@<insert>.1`, placement `place` / `dx,dy,dz` + `axis` +
  `angle`, components appended as their own lumps (never united),
  `components[] {id, dof, place}` sorted by id, `insertPartErrors {detail:
  "<first error id>: <code>"}`, empty parts silent — no error and no
  component, whether the inserted part has no features or builds no solid,
  `"<n> inserted parts"` pre-pass span for >= 2 distinct parts (identical
  parts regenerate once). Component `cpattern` (`seeds:[]`, `bodies:[{face}]`):
  copies in component order, `@F12.k` after the component suffix, not in
  `components[]`.
- `Xf`: the kernel's rigid transform with its decompiled arithmetic (WAT
  f3910 rotation, f3920 compose, f3928 frame (axes re-normalised), f3499 /
  f3800 / f3762 body transform, radii × cbrt|det|). ck-core's `Xform`
  associates differently and is NOT bit-compatible.
- Wire planar normals / circle normals of assembly faces print
  `normalize(x × y)` of the frame (component faces only: ck-topo's own
  frames already print like the oracle).
- `mate` solver = port of f1375 (Gauss-Seidel over components, the new
  mate's mover first, until a pass moves nothing, ≤ 8n moves) + f1351
  (damped Gauss-Newton: J^T J with 1e-4 / 1e-10 diagonal damping of the
  largest diagonal, rotation columns / max(scale, 1), step ≤ 0.5 rad, one
  0.001 rad kick when stalled, 200 iterations, converged at pos ≤ 1e-7 /
  ang ≤ 1e-9) + f1352 kinds 0 (direction), 2 (point on plane), 4 (point on
  line) with their analytic rows. Mover = a's component unless a is fixed.
  Records: coincident / distance = normals opposed (flip: aligned) + point
  on plane at `distance`; parallel = direction only; concentric = axes
  aligned by the sign of their dot + point on axis; directions =
  normalize(x × y) of the face frame (planes) / normalize(frame z)
  (cylinders), points = frame origins. Centre / scale = component box from
  edge boxes padded by 1e-6 (arc box = f3769). `tools/asm/matecases.mjs`:
  16/17 byte-identical (cylinder–plane coincident = kinds 1/2, not decoded
  -> `mateGeometry`). `dof` GUESS = 6 − rank(mate rows) (matches all
  probes, stepper and flange).
- `hole` (`tools/asm/holecases.mjs`): iso table, simple / cbore, fits,
  blind / through / tip, `S.h0..h4` (+ `~k` per point), holeInvalid /
  holeNoPoints; tools combined with the box rule (append if clear, else
  `boolean Unite`) — same rule now used for pattern instances, and
  patterns keep their combined tool as a seed for later patterns.
- `helix` -> `curves[]` byte-identical (`tools/asm/helixcases.mjs`).
- REBUILD-ONLY: nested errors from unrebuilt ops (`TODO(rebuild)…`,
  `blendUnsupported`) don't abort an insert (`mate::is_rebuild_gap`).

Flange (oracle-flange.json): components dof 6/0/1/0 all match; places Δ
F1 0, F2 1.7e-14, F6 2.2e-12, F9 2.9e-10 (the residue: the oracle's x/z
component box extents are NOT padded for the rotated half — its face box
is the surface box over the mesh uv bounds ∪ edge boxes, and some circle
extremes evidently come from the unpadded surface part; plus the missing
chamfer / fillet / thread faces of bolt and nut).

stepper (TODO): components F1 0, F13 1.2e-23 (exact), F10 1.4e-8; F2 / F6 /
F22 / F26 / F30 off by 1..31 mm — the stator / rear-cap mate records differ
(clocking concentric on split faces `~k`, cbore `h3` refs; `find_ref` picks
the first split piece, GUESS); F16 shaft / F19 rotor fail nested
`booleanFailed` (ck-ops). Not yet decoded: mate kinds 1/3/5..9 (tangent,
angle, hinge, gear, ...), f1378 dof (rank GUESS), face surface box f3805.
