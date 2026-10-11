# ck-model

The document model and feature program: parses .ckpart JSON, resolves
references, evaluates expressions, runs the regen program by calling into
ck-ops. Depends on ck-json, ck-ops, ck-geom, ck-topo, ck-core.

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
