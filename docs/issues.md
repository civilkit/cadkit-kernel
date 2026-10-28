# Open issues and task list

Found while writing and reviewing the documentation (Oct 2026): every claim in the two books was checked against the
code, and the examples were run. Status: **fixed** (in this change), **open** (a task, with a rough size), or
**decision** (the owner chooses before anyone codes).

Sizes: S = under an hour, M = a few hours, L = a day or more.

## Kernel

### Fixed in this change

| # | Issue | Fix | Where |
|---|-------|-----|-------|
| K1 | A regeneration that failed because a time limit ran out (`regenBudgetExceeded`, `featureBudgetExceeded`) was stored in the cache and replayed on every later regeneration of the same part, even after the limit was raised or lifted. | The recorder stops storing states once a budget error is in the result. | `ck-wasm/src/regen_cache.rs`; test `a_time_limit_failure_is_not_replayed` |
| K2 | `opts.cacheCap: 0` returned a null body with no error. | A cap of 0 turns the cache off for that request. | `ck-wasm/src/engine.rs`; test `cache_cap_zero_still_returns_the_body` |
| K3 | `trimSurface` could panic (`expect("piece loop chains")`) and kill the wasm instance. | A loop that does not chain now returns the feature error `trimPieceLoop`. | `ck-ops/src/surfacing/trim.rs` |
| K4 | The `healed` notice was written with the same key as the feature's own message and could replace it. | `healed` is not recorded when the feature already has a message; the repair itself still applies. | `ck-wasm/src/engine.rs` |
| K5 | Native service: `--port 65536` wrapped to 0 (a random port). | Values above 65535 are refused with a message. | `ck-native/src/main.rs` |
| K6 | Native service: no limit on wrong pairing codes. | After 5 wrong codes the pending code is cancelled (run `ck-native --pair` again). Counted per config directory. | `ck-native/src/store.rs`; test `pairing_code_is_cancelled_after_repeated_wrong_guesses` |
| K7 | Native service: the 65th simultaneous connection was dropped silently. | It gets `503 Service Unavailable` with `Retry-After: 1`. | `ck-native/src/lib.rs`; test `connections_over_the_limit_get_a_503` |

### Open

| # | Issue | Size | Notes |
|---|-------|------|-------|
| K8 | Misspelled feature fields are ignored (a typo in `depth` gives a 1e-6 slab with no message). | M | Add an `unknownField` warning per feature type from the list of known keys. Strict rejection would break older files, so warn only. Touches every feature parser; needs the key list per type. |
| K9 | `massProps` returns null for `com` and `principal` on a body with no volume (surfaces only). | S | Decide the answer first (zero volume with area only, or an error). The reference comparison in the gate pins the current shape. |
| K10 | `importSat` reports errors in a different shape from the other importers. | S | The shape is pinned by the gate against the reference build; change both together. |
| K11 | Several features always fail with `TODO(rebuild)`: `dome`, `splitLine`, `fillpattern`, `curvepattern`, `surfaceOffset`, `cutWithSurface`, `spattern`, and a few option combinations (twisted sweeps along curved or multi-piece paths, mitre joints under 166 degrees). | L each | The error text names an internal project word; give it a stable code (`notImplemented`) and keep the text. The gate compares codes, so change it with the reference. |
| K13 | Validator hairline forgiveness (`tessellation.rs`) can hide a missing face that is thinner than the tolerance. | M | Needs a measured threshold from the corpus; do not tighten without running the corpus. |
| K14 | `GRID_SEEDS` and `ORIG_STEP` are process-wide atomics, so native-service sessions share them. | M | Move into the session options; only matters for concurrent debugging requests. |
| K15 | Three crash-fuzz cases hang (`case_34`, `case_644`, `case_2668`), and the fillet sweep has an `rrect_loop` hang risk. | L | The time limits stop them in production; the root causes are in the boolean and fillet loops. |
| K16 | `eyebolt_torus_fragments_tessellate` fails (also before the rename). | M | Marked `#[ignore]` in the published tree. Torus fragment tessellation. |
| K17 | The marine propeller default fails `TESSELLATION_OPEN`; `coneOnPlate` (2 tests) and the dash timing test fail. | M | All also failed before the rename. |
| K18 | The regression gate needs a second kernel build to compare against, which is not shipped. | M | Ship the comparison build as a release artefact, or let the gate run without it (numbers only). |
| K19 | `regen.brep.mesh 72/319 (-1)` soft warning in the gate. | S | Known; one case moved. Re-baseline when understood. |
| K20 | `drawing`: a thin shaft (a 2 mm revolve or extruded circle) contributes almost no lines in side views; circular edges seen edge-on (chamfers, bosses) come out as broken dashes; silhouette pieces are missing at a can end. Reported with repro parts (11-13 bodies). | M | The repro parts are outside the repositories; copy small versions into the test fixtures before fixing. |
| K21 | `drawing`: a section view requested through the script layer did not appear; a hole callout can land on a dimension text; callouts exist only for real hole features, not for a sketch circle plus a cut; the PDF font prints the depth glyph as `?`. | M | App side as much as kernel; find which layer drops the section. |

## Application and tools

| # | Issue | Status |
|---|-------|--------|
| A1 | Helix "Defined by" listed the wrong values; two checkboxes showed raw field names; the mate editor printed a degree escape; the file-drop message left out `.solidcraft`; the MCP `snapshot` tool put 48,000 characters of pixel data in its text. | **fixed** (v0.4.0 and later) |
| A2 | The Slot flick test is flaky; `threadData` fails only in full-suite order. | open, S each |
| A3 | Navigation presets are labelled with other programs' names ("SolidWorks style", "Onshape, Fusion, Blender style", "Plasticity-style"). | **decision**: rename to neutral labels (a locale change) or keep. The guide quotes the labels as shown. |
| A4 | BOSL2 attribution line and the library name. | **decision** |
| A5 | MCP: the live link between the MCP server and the open browser app (the GUI bridge) and the in-app chat are design only. | **decision**: build now or later. The guide says so. |
| A6 | Settings screen for the local kernel service has no screenshot (it would show a real token); no shareable SketchUp sample for an import screenshot. | open, S |
| A7 | Imported SolidCraft models show every sketch where it was drawn while the bodies were moved away by `moveBody` (Move, Mirror and Pattern import as `moveBody`; the kernel moves bodies only). The sketches look misaligned with the parts. | open: hide such sketches by default (S), then draw a sketch at the pose of its body (M) |
| A8 | MCP gaps from a multi-body modelling trial (17 items): no `llms.txt` and a five-line `ck://docs/guide`; `list_methods` and the docs omit methods `run_script` accepts (appearance, bodyName, group, handle, sweep, workplane, arc3, rrect, text); std-plane sketch axes undocumented; a cut hits every existing body; touching solids fuse with no `newBody` in the script API; no per-body list tool; appearance on a pattern misses the seed body; unhelpful `profileCrossesAxis` hint; `snapshot{sheet:true}` and `annotate:true` crash on a boolean although the schema allows it; no drawing tool; the dev server default `CK_REBUILD=../kernel` does not exist here. | open: the `snapshot` boolean crash and the docs gaps are S each |

## Repository and publishing

| # | Issue | Status |
|---|-------|--------|
| R1 | The old README wording about earlier file names is one commit back in the published history of both private GitHub repositories. Scrubbing it needs a force-push. | **decision** (a force-push needs your approval) |
| R2 | `docs/calc/` is free; no calculator documentation yet. | open |
| R3 | Throwaway folders: `_export/hist`, `_export/old`, extra worktrees, local `public-cleanup` branches. | **decision**: delete when you say so |
