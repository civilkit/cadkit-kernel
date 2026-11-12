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
| K9 | `massProps` returned null for `com` and `principal` on a body with no volume (surfaces only). Done: zero volume, area, area centroid, `kind: "surface"`. | S | Decide the answer first (zero volume with area only, or an error). The reference comparison in the gate pins the current shape. |
| K10 | `importSat` reports errors in a different shape from the other importers (done: now `["import", {code, params}]`). | S | The shape is pinned by the gate against the reference build; change both together. |
| K11 | Several features always fail with the stable code `notImplemented` (was free text starting `TODO(rebuild)`; done): `dome`, `splitLine`, `fillpattern`, `surfaceOffset`, `cutWithSurface` (`curvepattern` and `spattern` are built now), and a few option combinations (twisted sweeps along curved or multi-piece paths, mitre joints under 166 degrees). | L each | The error text names an internal project word; give it a stable code (`notImplemented`) and keep the text. The gate compares codes, so change it with the reference. |
| K13 | Validator hairline forgiveness (`tessellation.rs`) can hide a missing face that is thinner than the tolerance. | M | Needs a measured threshold from the corpus; do not tighten without running the corpus. |
| K14 | `GRID_SEEDS` and `ORIG_STEP` were process-wide atomics, so native-service sessions shared them (done: now per-thread, set per regen). | M | Move into the session options; only matters for concurrent debugging requests. |
| K15 | Crash fuzz: the three hangs of the old baseline (`case_34`, `case_644`, `case_2668`) are gone (structured errors under the time budgets, at most 0.35 s CPU); the baseline was re-recorded (trap 0, hang 0, invalid-result 9). The fillet sweep `rrect_loop` hang risk is untested by the fuzz. | L | The nine remaining `invalid-result` cases are listed in `docs/crash-fuzz.md` (not crashes). The root causes of the former hangs in the boolean and fillet loops are not fixed; the time limits stop them. |
| K16 | `eyebolt_torus_fragments_tessellate` fails (also before the rename). | M | Marked `#[ignore]` in the published tree. Torus fragment tessellation. |
| K17 | The marine propeller default fails `TESSELLATION_OPEN`; `coneOnPlate` (2 tests) and the dash timing test fail. | M | All also failed before the rename. |
| K18 | The regression gate needs a second kernel build to compare against, which is not shipped. | M | Ship the comparison build as a release artefact, or let the gate run without it (numbers only). |
| K19 | Soft warnings in the gate: `regen.brep.json 83/319 (-3)`, `regen.brep.mesh 70/319 (-3)`, `regen.brep.rest 240/319 (-4)` against the old baseline 86 / 73 / 244; 13 cases listed as `MESH DIFF`. | S | **Explained, no regression.** Four cases moved, each by a deliberate change, none in the geometry. `draftBig` (refusal now carries `maxSize`, commit 1025b6ed) and `filletTooBig` (the refusal names the largest radius that builds, 3a1bf4e9) differ from the original kernel only by the added hint and `maxSize` fields in the error; `cpatternBoss` (patterns, ed74ce61) now returns the warning `patternCopiesApart` where the original is silent (it was already a mesh-only match; it now fails the JSON match and `rest`). `zeroDepth` (a 1e-6 thick body) has the same JSON but its vertex buffer order differs: the tessellator's vertex snap (de97069e) merges vertices a few ulps apart and re-orders them. The other 12 `MESH DIFF` cases were already in the baseline. **Baseline re-recorded 2026-11-13** in `tools/scoreboard.json` to json 83, mesh 71, rest 240 (from 86 / 73 / 244; confirmed by a gate on the box on this tree: the other metrics are unchanged), because the four moves are deliberate and the gate printed three soft warnings on every run. Re-checked on `fuzz-fix` (kernel main be8f8987 plus the crash-fuzz fixes): json 83, mesh 71 (-2 against the 73 baseline), rest 240; no new case moved by the fuzz fixes. Two `ck-model` tests fail on main as well and are not from those fixes: `blend_regression_table` (five `daisy-ring-*-T` cases now pass and want `pass` in `blend_expect.json`) and `combine_feature::tolerance_snaps_a_wall_1e3_off_and_warns`. |
| K20 | `drawing`: a thin shaft (a 2 mm revolve or extruded circle) contributes almost no lines in side views; circular edges seen edge-on (chamfers, bosses) come out as broken dashes; silhouette pieces are missing at a can end. Reported with repro parts (11-13 bodies). | M | The repro parts are outside the repositories; copy small versions into the test fixtures before fixing. |
| K21 | `drawing`: a section view requested through the script layer did not appear; a hole callout can land on a dimension text; callouts exist only for real hole features, not for a sketch circle plus a cut; the PDF font prints the depth glyph as `?`. | M | App side as much as kernel; find which layer drops the section. |
| K22 | `extrude` with depth 0 (or an expression that gives 0, or no depth) builds a 1e-6 mm sliver (volume ~3e-4 mm3 on a 10 mm disc) and a later union with it can fail with `booleanFailed` / `openEdge` (civilkit/cadkit-kernel #49). | S | The reference clamps and builds the sliver (corpus case `zeroDepth`), so a refusal would break the gate. Done: warning `extrudeZeroDepth {depth, built, hint}` on blind and mid extrudes. Turning it into an error needs the reference decision first. |
| K23 | A plain extrude cut on a folded sheet that the panel tree refuses runs as an ordinary cut of the solid, and the part silently stops being an unfoldable sheet. | S | Done: warning `sheetCutFallback {detail, hint, reason, reasonDetail}` on that feature; it is left out of the comparison with the reference like `extrudeZeroDepth`. |

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
