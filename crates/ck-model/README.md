# ck-model

The document model and feature program: parses .ckpart JSON, resolves
references, evaluates expressions, runs the regen program by calling into
ck-topo. Depends on ck-core, ck-geom, ck-topo, ck-json; ck-ops plugs in
later through `features::Brep::boolean` (and per-feature code).

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
  oracle goldens in `tests/golden/sketch_solve.json`). Needs a correctly
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
  ck-topo. `op:"cut"` / add onto an existing body -> `Brep::boolean`
  (None -> `TODO(rebuild): boolean ... needs ck-ops` error code); other
  geometry feature types -> `TODO(rebuild): <type> needs ck-ops`.
  Error codes/precedence pinned: badExpression -> missingAxis (revolve) ->
  noClosedProfile -> unknownEnd{end} / invalidValue (angle 0) ->
  missingReference (upToFace/offsetFromFace/upToPoint) / nothingToCut.
- `refs::face_frame`: face sketch planes / plane bases (rule in refs.rs).
- `features::body_wire`: wire `box/faces/edges` (edge `pts` = ck-topo
  `sample_edge` stand-in; box = exact bounds ∪ f32-rounded, GUESS for
  curved bodies whose oracle box comes from the mesh).
- regen trace spans: `"<id> <type>"`, `" → N faces"` once a body exists,
  `"face names"` after each successful feature with a body, failures
  `"<id> <type>: <code>[ <params>]"` + child `"<id> failed: ..."`.
- Tests: `tests/brep_golden.rs` vs `tests/golden/brep.json`
  (`node tools/brep_golden.mjs`, parts in `tools/brep_cases.mjs`):
  20/24 regen replies byte-identical minus mesh/pts (draft, draftOut ~1e-10,
  rev90mid / cone last ulp); massProps within 1e-9 (never byte-exact: the
  oracle's integration rounds differently; degenerate principal axes are
  arbitrary). `node tools/diff.mjs --cases brep` diffs the follow-up
  requests byte-for-byte.
