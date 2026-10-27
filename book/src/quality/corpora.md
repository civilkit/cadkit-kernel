# Corpora and sweeps

Beyond the unit tests, the kernel is measured against sets of parts with known answers. A corpus is a fixed list of cases, each with an expected result. A sweep generates seeded random cases from templates and compares each with an independently computed value. This chapter lists them, says what each measures and how to read the output, and gives the committed results. All scripts are in `tools/` and run on Node against the built module (see [testing](testing.md)).

## Reading a result

Most scripts print one line per case and a summary of classes. The class names are shared:

| Class | Meaning |
|---|---|
| `ok` | No feature error, `validate` clean where checked, and the number agrees with the reference within the tolerance of that script |
| `okNoEffect`, `okEmpty` | The operands do not overlap, so a cut changed nothing or an intersection was empty; correct, not a failure |
| `wrongVolume`, `wrongArea` | The kernel built something valid-looking whose volume or area is off the reference |
| `invalid` | The result has `validate` issues (`EDGE_OFF_SURFACE`, `TESSELLATION_OPEN`, ...) |
| `booleanFailed`, `blendFailed`, `blendOverflow`, `shellFailed`, ... | A structured feature error; see [errors](../protocol/errors.md) |
| `refused` | An expected refusal (for example a shell wall thicker than the part allows) |
| `kernel-null`, `trap`, `hang` | No reply, a WebAssembly trap, or a timeout |
| `ref-fails` | The reference computation could not build the case; the case says nothing about the kernel |
| `known` | A failure with a documented cause, which does not fail the run |

Sweeps and fuzzers take `--samples N --seed S`, write detail with `--json out.json`, store a class-only baseline with `--update-baseline file` and compare with `--check file`. A check exits non-zero only when a case that passed in the baseline now fails; new passes are printed but are not errors. A baseline stores case ids and classes, plus the module hash, seed and sample count.

## Reference values

- **Closed forms** derived from the parameters: prisms, revolved profiles, spheres, tori, grids of unit cells.
- **An independent geometry library**, called through Python bindings, for volume, area, centre of mass and inertia of the same shape built with the same parameters. The scripts `tools/*_ref.py` are needed only to refresh or extend a baseline; running a baseline `--check` needs only Node.
- **Ground truth shipped with a dataset** (BenchCAD, brepjs examples): volume, area, centre of mass, bounding box and a STEP file per part.

## Corpora

| Corpus | Script | Cases | What it measures |
|---|---|---|---|
| Generated parts | `tools/brep_cases.mjs` (feeds the gate) | 319 | Small documents across the feature families (extrudes with each end condition, revolves, holes, patterns, blends, lofts, sweeps) and prefixes of the larger sample parts; 59 have closed-form volume, area, centre and box |
| Protocol cases | `tools/cases.mjs` (feeds the gate) | 133 cases, 280 request bodies | Envelope handling, malformed bodies and each request type |
| Boolean coincidence | `tools/boolean_corpus.mjs` | 147 | Extrude and revolve add and cut, `moveBody` and the body-level `combine` with coincident, tangent, near-coincident, contained, coaxial and sliver geometry. Each has an analytic volume and, where defined, a lump count. `--fuzz N` runs random add and cut sequences on an integer grid whose exact volume is a count of unit cells. `--tol` and `--auto` try the fuzzy tolerance |
| Blends | `tools/blend_corpus.mjs` | 150 | Fillets and chamfers (constant and variable radius, unequal radii, corners, rounded outlines, overhang corners) against reference volumes in `tools/blend_reference.json`. Bands: 1e-5 for planar bodies, 1e-4 for curved ones, wider for variable and unequal radii at corners |
| Lettering | `tools/text_corpus.mjs` | 126 | Engraving and embossing, every glyph with holes, planes, arcs, fonts; the volume change must equal the glyph area times the depth to 1 percent. `--fuzz N --seed S` runs random strings |
| OpenCASCADE test scripts | `tools/occt_corpus.mjs` | 1664 | Primitives, booleans, blends and drafts translated from the OpenCASCADE test suite, compared with its own `checkprops` values |
| BenchCAD-derived parts | `tools/bench/run.mjs` | 50 | Parts translated by hand from a public benchmark: regenerate, compare volume and area (1e-3), centre and box, `validate`, and import the dataset's STEP |
| brepjs-derived parts | `tools/bench/run.mjs --set brepjs` | 25 | Mechanical examples translated by hand; the reference STEP files go through `importStep` |
| Crash cases | `tools/cases/crash/` | 24 | Minimal documents that once trapped or hung; replayed by `robust_cases.rs` |
| STEP imports | `tools/step_nist_check.mjs`, `tools/import_sample_check.mjs`, `tools/heal_check.mjs` | varies | NIST test-case STEP files with reference volumes, and samples of 200 files from a public research set |
| SketchUp, SolidWorks | `tools/skp_check.mjs`, `tools/sldprt_check.mjs` | 17 and 4 | Sample files per container version: imported, faces, volume, validity, time |
| Recognition | `tools/recog_check.mjs` | varies | Regenerate, export, re-import and recognise sample parts |

### Boolean coincidence corpus

Run in this chapter: 145 of 147 cases ok. The two failures are `booleanFailed` on near-coincident cylinders 1e-3 apart: two stacked cylinders of radius 5 and 5.001, and a plug of radius 5.001 fused into a hole of radius 5. The rim circles are too far apart to weld, so the intersection curves cannot close. With `--tol 1e-3` or `--auto` all 147 pass, because the tool wall is first snapped onto the host wall and the case becomes the exactly coincident one. The grid fuzz (`--fuzz 300 --seed 7`: four boxes per sequence on a 5-unit grid) passes 299 of 300.

### Blend corpus

Run in this chapter: 142 of 150 ok, 0 failed, 6 known, 2 without a reference volume (the external builder fails on them). A pick is a point on an edge; both the kernel and the reference take the edge of the base body nearest to it. The base bodies are built by the kernel and written with `step`, so both start from the same geometry. The known cases and the open items (three-edge corners with unequal radii, which are valid but not tangent-continuous and carry a `cornerNotTangent` warning, and some sector fillets) are in the corpus notes.

### OpenCASCADE test scripts

`tools/occt_corpus.mjs` translates test scripts from the OpenCASCADE repository into feature trees. The test data files of that suite are distributed separately and are not used; the data-driven cases are counted as `needs-data` and left out. OpenCASCADE source and test data are not part of this repository (OpenCASCADE is LGPL-2.1 with an exception). The runner reads a checkout fetched by `tools/occt_fetch.sh`; the committed `tools/occt_picks.json` holds derived edge midpoints and directions only. Edge and face picks are matched by position, not by index.

Committed baseline of the 1664 selected cases (`tools/occt_baseline.json`): 1570 ok, 29 invalid, 19 `booleanFailed`, 17 `wrongArea`, 8 unsupported (shapes the translator cannot express), 7 `blendUnsupported`, 6 `booleanVolumeMismatch`, 3 `blendFailed`, and five single cases: `combineEmpty`, `missingEdge`, `draftUnsupported`, `invalidValue` and `missingReference`. The failures cluster in sphere against box corners, perpendicular cylinder intersections, cone and cylinder seams, and fillets on partial cylinder and cone sectors.

### Sets that ship and their licences

| Set | Source | Licence | In the repository |
|---|---|---|---|
| BenchCAD-derived parts | `BenchCAD/BenchCAD`, configuration `code_gen` | CC-BY-4.0; attribute the BenchCAD authors | Ground-truth STEP files and our own translations of 50 parts (no source code copied) |
| brepjs-derived parts | The mechanical examples of the brepjs playground | Apache-2.0; attribute the brepjs project | Our translations of 25 examples (parameters paraphrased) and ground truth from running the examples |
| NIST PMI test files | NIST MBE PMI validation project | US Government work, public domain | One file (`nist_ftc_11.step`) and computed reference volumes of the others |
| OpenCASCADE tests | OpenCASCADE Technology | LGPL-2.1 with exception | Nothing vendored; derived pick coordinates only |
| Public research sets of STEP files | Published by CAD vendors and universities | Non-commercial research terms | Nothing: fixed sample ids and class-only baselines only |

`THIRD_PARTY.md` has the exact entries. Research-only data must not be redistributed or used as a release gate for a product, which is why the repository keeps sample ids and baselines, not files.

## Sweeps

Each sweep is seeded, generates cases from templates, runs them through the module, and compares with an independently computed value. The results below are the committed class-only baselines (seed 7 unless noted); they are snapshots of one build, so re-run `--check` for the current state.

| Sweep | Script | Cases | Measures | Committed result |
|---|---|---|---|---|
| Fillet and chamfer | `tools/fillet_sweep.mjs` | 640 (16 templates x 40) | Volume of box, cylinder, tube, boss, hole-rim, variable-radius and rounded-outline fillets and the three chamfer forms | 575 ok, 32 `blendOverflow`, 28 `wrongVolume`, 3 `blendFailed`, 2 `kernel-null`. Seed 11: 591 ok, 30 `wrongVolume`, 16 `blendOverflow`, 3 `kernel-null` |
| Shell and thicken | `tools/shell_sweep.mjs` | 480 | Inward and outward shells of boxes, tubes, cones and an L body; thicken of planar, cylindrical and conical sheets | 324 ok, 97 `wrongVolume`, 42 `refused`, 15 `shellFailed`, 2 `draftTooLarge` |
| Boolean | `tools/boolean_sweep.mjs` | 320 | Primitive pairs (box, cylinder, cone, sphere, torus) with overlap, coincident, tangent, contained, sliver, angled and chain relations | 146 ok, 80 `okNoEffect`, 65 `okEmpty`, 22 `wrongVolume`, 5 `invalid`, 2 `booleanFailed`. The 175 cases that are neither `okNoEffect` nor `okEmpty` contain all 29 failures |
| Loft, revolve, sweep | `tools/shape_sweep.mjs` | 240 | Volume of two- and three-section lofts, point-ended lofts, revolves, line, arc and mitred sweeps | 223 ok, 16 `wrongVolume`, 1 `ref-fails` |
| Mass properties | `tools/massprops_sweep.mjs` | 240 | Volume, area, centre of mass and inertia tensor, 1e-6 relative | 220 ok, 20 `wrongVolume`. All 20 are three-section circle lofts, which differ from a thru-sections reference surface by up to 6.7 percent in volume, 3.7 percent in area, 1.6 percent in centre of mass and 10 percent in inertia. Every other template agrees to 2e-14 or better |
| STEP round trip | `tools/step_roundtrip_sweep.mjs` | 120 | Export, read by the reference library and re-import: volume and area to 1e-6, a validity check on its body, `validate` on ours | 100 ok, 20 `ocp-wrongArea` (lofts: the reference library's area of the written file is 5e-4 to 8e-4 low; our re-import reproduces both numbers exactly) |
| Tessellation | `tools/tess_sweep.mjs` | 180 | Volume and area of the display mesh against `massProps`; fails above 0.1 percent | 96 ok, 84 `meshVolume`; the worst are torus (0.8 percent) and cylinder (0.4 percent), caused by chord height at the default tolerance. Mesh area stays under 0.1 percent everywhere |
| Holes, patterns, mirror, draft | `tools/feature_sweep.mjs` | 120 | Through, blind, counterbore and countersink holes; linear and circular patterns; mirror; draft | 120 ok |
| Crash fuzz | `tools/crash_fuzz.mjs` | 5000 | Schema-valid random documents of 2 to 8 features with degenerate parameters; see [robustness](robustness.md) | 668 ok, 4290 structured errors, 24 invalid results, 13 surface-only and 2 degenerate documents, 0 traps, and 3 hangs (an open defect) |
| Research STEP samples | `tools/import_sample_check.mjs` | 200 per sample | `importStep` and `validate` on files kept outside the repository | Seed 1234: 185 ok, 15 invalid (`TESSELLATION_OPEN` 13, `EDGE_OFF_SURFACE` 7, `LOOP_SELF_INTERSECT` 1, `EULER` 1; some files carry two codes). Other seeds: 4242 gives 188 ok, 5678 gives 185, 9001 gives 187. One file is a known slow import (over 90 s) |

How to read these:

- A `wrongVolume` group is not random: each sweep's notes name the template and parameter range (for example outward shells with the top removed, or three-section lofts). Use the failing parameter range to decide whether a case is in a known gap before suspecting a regression.
- `blendOverflow` in the fillet sweep starts when the radius approaches half the shortest side of a box, or half the height of a cylinder; the reference library succeeds on those, so they are genuine large-radius gaps.
- `refused` in the shell sweep is the expected `shellTooThick` answer for walls that cannot fit.
- The tessellation sweep measures the display mesh, not the exact B-rep; exact volumes come from `massProps`.
- The shell sweep is run one template at a time (`--only box_top`); a single process that walks every template can stop responding.

## One-command run

`tools/verify_wasm.sh <module> [--quick|--accept]` runs the sweeps and corpora above against one module and prints a single page of counts per class, new passes and regressions with case ids. Each template runs in its own process with a hard timeout, so a hang is reported as `ERROR: <template>: hang` and does not block the run. The exit status is non-zero only on a regression, meaning a case that passed in its baseline and now fails. `--quick` uses one seed and four templates per sweep and takes under two minutes. `--accept` does a full run and rewrites every baseline.

## See also

- [Testing](testing.md)
- [Robustness under edits](robustness.md)
- [Known limits](limits.md)
- [STEP](../io/step.md)
- [Feature recognition](../io/recognition.md)
