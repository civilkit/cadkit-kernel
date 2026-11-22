# D2: ck-model depends on ck-io, ck-heal and ck-sldprt

Status: proposal, nothing implemented.

## What ck-model takes from each

All uses are in four files, 790 lines of ck-model's 45 000 (the other three crates are 6 200, 2 200 and 6 700 lines).

| Crate | Used for | Where |
|---|---|---|
| ck-io | `step::read_step` and the `Imported` type that `sort_step` takes | `features/import_step.rs` |
| ck-heal | `heal_with`, `HealOpts`; `honest::{Outcome, Part, Reason, Sorted, PartIn, SURFACE_MODEL}` (the verdict on each imported part); `repair::LeftOut`; `tolerant_only` | `import_step.rs`, `import_body.rs`, `repair_part.rs`, `built_cache.rs` |
| ck-sldprt | `import_sldprt`, `build::PartRead` | `import_body.rs` |

ck-heal itself does not depend on ck-io or ck-sldprt (those are its dev-dependencies, as is ck-model), so there is no cycle
and nothing else in ck-model pulls them in. `ck-wasm` already depends on all three directly.

Tests in `crates/ck-model/tests` that touch these modules: ten files (`import_step.rs`, `import_body.rs`, `mesh_solid*.rs`,
`built_cache.rs`, `sldprt_surface_faces.rs`, ...).

## Can it move

- Behind a trait: the seam would be an `ImportHost` trait in ck-model (`read_step`, `read_sldprt`, `heal`). It does not hold
  the weight, because the result types are ck-heal's (`honest::Part`, `Outcome`) and flow into the feature's reply and into
  `built_cache` (which also serialises them). A trait over those types either re-exports them from ck-model (so ck-model
  still depends on ck-heal) or gets duplicated.
- Up into ck-wasm: feasible and mechanical. Move the four files to ck-wasm (or a new `ck-import` crate that ck-wasm and
  ck-native depend on), and register the three feature types (`importStep`, `importBody`, `repairPart`) through a small
  executor registry in `Brep::exec` instead of the `if f.type_name == "importStep"` arms at `features/mod.rs` ~1775.
  The ten test files move with them.

## What it buys and costs

- Buys: ck-model compiles and tests without three importer crates; the model crate's contract becomes "features and regen
  only". No effect on wasm size or speed, and little on build time (the importers build in parallel with ck-ops).
- Costs: a registry in the feature executor (a new indirection on the hot path of dispatch, though not per face), the test
  move, and the regen cache and `builtExport` plumbing that currently lives next to these features.

## Recommendation

Do not move it now: the benefit is architectural tidiness only. Cheap preparation worth doing: keep the four files the only
importers of those crates (add a lint or a grep test), so the move stays a half-day job. Do the move to a `ck-import` crate
when a second consumer of ck-model that does not want importers appears (for example a headless validator), not before.
