# ck-ops

Modeling operations: the feature engine. Depends on ck-topo, ck-geom, ck-core.
This is the largest crate (58 modules) and the bulk of the kernel.

## Module map (pinned by rs_paths.txt)

- `boolean/` — apply, assemble, classify, imprint, mod: solid booleans
  (union/cut/intersect) with imprinting and classification
- `blend/` — fillets/chamfers, 17 files: build, cap, chain, corner, g2,
  inplace, interp, march, overlap, rolling, sheet, split, spring, stripe,
  surface, vertex. Error strings: "the chain through X runs into itself",
  "edge between X shrinks to nothing", "both must move by the same distance"
- `loft/` — centre.rs, guides.rs, smooth.rs, stations.rs + loft.rs
- `sweep.rs`, `helix.rs`, `offset.rs`, `shell.rs`, `draft.rs`, `pipecut.rs`
- `surfacing/` — boundary, extend (+reach/shape), fair (+meow.rs!),
  fill, knit, sheet, thicken, trim, rebuild/ (deviation, edits, net,
  replace, mod)
- `intersect/` — cc (curve-curve), cs (curve-surface), ss (surface-surface)
- `bvh.rs` — bounding volume hierarchy for face/edge queries
- `contain.rs`, `continuity.rs`, `curve_edit.rs`, `guide.rs`, `simplify.rs`

## Interface

Each op takes resolved inputs (bodies, faces, edges, params) and returns a
mutated body graph + errors. Feature-level validation and error strings live
in ck-model/src/features; the geometric work lives here.

## Rebuild notes

- Error strings in kernel.strings pin the exact validation messages per op —
  grep them when implementing an op's failure paths.
- The golden part exercises: extrude, revolve, fillet, hole, helix, mate,
  pattern (cpattern), insertPart.
