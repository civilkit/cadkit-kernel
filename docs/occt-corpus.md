# OCCT-derived corpus

`tools/occt_corpus.mjs` translates OCCT `tests/` scripts into the kernel's
feature tree and checks the result against OCCT's own `checkprops` values.
It is a frontier corpus: OCCT passes its own checks, so an `ok` here is
agreement with an independent kernel and a failure is a real gap.

## Data files (step 1 decision)

The 4734 scripts behind `CSF_TestDataPath` cannot be used from this
repository. Findings:

* The OCCT contribution guide (`dox/contribution/tests/tests.md`) points at
  `CSF_TestDataPath` and says the data is distributed separately; it gives no
  public URL.
* The dedicated OCCT page is the forum archive
  `https://occt3d.com/dev/content/test-data-files/`. Its answer is that the
  data files were "not yet ready for publication, waiting for analysis and
  validation by our legal department", and later replies say reference data
  "might be confidential".
* There is no `occt-test-data` repository under `Open-Cascade-SAS` on GitHub
  (404), and the OCCT GitHub releases carry source and binary archives only,
  no test-data asset.
* No public total size or licence/terms are published. Third-party mirrors
  exist but are partial and unlicensed.

Decision: **no data files are downloaded or committed**. The data-driven
cases stay out of the corpus and are counted `needs-data` (7739 scripts in
the fetched groups, 4734 of them data-driven). Step 3 (import-then-operate
cases) is therefore skipped.

## Selection and coverage

1664 cases are selected:

| group | selected | ok | unsupported | failing |
|---|---|---|---|---|
| boolean/bopcut_simple | 367 | 355 | 0 | 12 |
| boolean/bopcommon_simple | 367 | 358 | 0 | 9 |
| boolean/bopfuse_simple | 367 | 355 | 0 | 12 |
| boolean/boptuc_simple | 367 | 360 | 0 | 7 |
| boolean/bcut_simple | 65 | 54 | 2 | 9 |
| boolean/bfuse_simple | 38 | 30 | 1 | 7 |
| boolean/bcommon_simple | 38 | 30 | 1 | 7 |
| boolean/bfuse_complex | 2 | 1 | 0 | 1 |
| blend/simple | 32 | 18 | 0 | 14 |
| draft/angle | 10 | 5 | 2 | 3 |
| bugs/* (primitive/boolean) | 11 | 4 | 2 | 5 |
| **all** | **1664** | **1570** | **8** | **86** |

Full run: about 55 s with one in-process wasm instance.

### Translation subset

* `box`, `pcylinder`, `pcone`, full `psphere` (negative box extents
  normalised; cone built about world +Z via the `top` plane).
* `trotate`, `ttranslate`, `dset` expressions (`sqrt`, `atan2`, `pi`, ...).
* `bfuse`/`bcut`/`bcommon` and `bop`/`bopcut`/`boptuc`/`bopfuse`/`bopcommon`.
* `blend` (staged blends too): edge picks by cached midpoint + tangent.
* `depouille` (draft): face pick by cached midpoint + normal; the neutral
  plane is an offset plane feature.

Not translated (counted `unsupported` or excluded): `vertex`, `edge`, `wire`,
`mkplane`, `mkface`, `profile`, `prism`, `revol`, `wedge`, `ptorus`, partial
spheres, `tscale`, `copy`, diagonal neutral planes, and data-driven scripts.

## Edge and face picks

DRAW's `explode` numbers sub-shapes in `TopExp_Explorer` order, skipping
duplicates; `nexplode` sorts by the DBRep centre key
(`X*999 + Y*99 + Z*0.9`). `tools/occt_picks.py` rebuilds the same base with
OCP, applies any earlier blends, and records:

* edges: midpoint + unit tangent (a corner point lies on up to three edges);
* faces: midpoint + outward normal (draft face and neutral face);
* staged blends: one pick set per blend, in script order.

`tools/occt_picks.json` is the committed cache, keyed by case. Normal runs
need no Python or OCP. Regenerate only when the selection changes:

```sh
python3.12 -c 'import OCP'
node tools/occt_corpus.mjs --occt /tmp/occt --write-picks tools/occt_picks.json
```

`--verify-picks` runs the box-fillet cases (`A1`, `A2`, `A3`, `P8`, `P9`) and
requires the selected edge to match the cached midpoint/tangent and the area
to agree.

## Run

```sh
tools/occt_fetch.sh                  # sparse checkout -> /tmp/occt
node tools/occt_corpus.mjs --occt /tmp/occt \
  --wasm /path/to/kit/apps/cad/public/kernel/ck_wasm.wasm
node tools/occt_corpus.mjs --occt /tmp/occt --only bopcut_simple --quiet --json out.json
node tools/occt_corpus.mjs --occt /tmp/occt --verify-picks

node tools/occt_corpus.mjs --occt /tmp/occt --update-baseline tools/occt_baseline.json
node tools/occt_corpus.mjs --occt /tmp/occt --check tools/occt_baseline.json
```

`--check` exits non-zero only when a case that was `ok` in the baseline now
fails; new passes are printed but are not errors. When `checkprops` gives both
`-s` and `-v`, the larger relative error is used.

## Draft and blend coverage (step 2)

* **Draft** (`tests/draft/angle`): 10 selected, 8 translated, 5 `ok`
  (`A1`, `A7`, `A8`, `D1`, `L3`). `A9` has a diagonal neutral plane (not
  representable), `K1` needs a partial sphere. `A5` (cylindrical side about a
  tangent plane) gives the wrong area; `B2` is `draftUnsupported` on a cone
  side; `B3` is `invalidValue`.
* **Blend** (`tests/blend/simple`): 32 selected, all 32 translated (0
  `unsupported`), 18 `ok`. The staged blends `Q7` and `X1` now run but fail
  (`Q7` wrong area, `X1` `blendFailed`); `Q4` still cannot find the second
  edge. The rest are the sector/seam groups below.

## Other groups (step 4)

There is no `tests/shell` or `tests/thickness` in OCCT master; those operations
live in `tests/offset` and `tests/bugs`. Counts from the fetched checkout:

| group | scripts | needs-data | translated | why not more |
|---|---|---|---|---|
| offset | 2030 | 1786 | 0 | `OFFSETSHAPE`, `offsetonface`, `offsetparameter`, `offsetperform`; the rest need data |
| sewing | 706 | 700 | 0 | 700 are `restore` + `sewing` + `checkfreebounds` data cases |
| heal | 1279 | 1188 | 0 | `fixshape`, `vertex`, `mkplane`, `unify`; mostly data |
| chamfer | 93 | 1 | 0 | driver files (`chamf_shape`, `.draw`/`.rle` data), not standalone geometry |
| bugs | 4089 | 3298 | 11 | the other no-data cases need `vertex`/`edge`/`wire`/`polyline`/`mkface` |

The 11 `bugs` cases use only primitives/booleans; 4 pass, 2 are
`unsupported` (a `copy` instance and a partial sphere), and 5 fail (2
`booleanFailed`, 1 `missingReference`, 1 `EDGE_OFF_SURFACE`, 1 wrong area).

## Failure triage

The original three-grid selection had 35 failures from about 24 distinct
scripts; the whole corpus now has 86. Distinct groups (new groups marked):

| # | geometry | cases | our class | expected | cause guess |
|---|---|---|---|---|---|
| 1 | sphere r1 at origin vs box [0,1]^3 | 12 | `booleanFailed`; `EDGE_OFF_SURFACE` | 14.6394 fuse, 13.3518 sphere-box cut, 5.2146 box-sphere cut | sphere seam/poles and the box corner |
| 2 | cross-cylinder cut / rotated box tangent to a cylinder | 9 | `invalid` `EULER`, `VERTEX_OFF_CURVE` | ZE7 33.8759; W7 18.5364 | non-manifold vertex or endpoint off the intersection curve |
| 3 | chained boolean (cyl+cone union, then pcyl cut) | 2 | `booleanFailed` | 727.481 | cone/cylinder seam boolean, not the lump selector |
| 4 | perpendicular cylinders r1/r0.5, intersect | 7 | `booleanFailed` / `combineEmpty` | 33.8757..33.8759 | cross-cylinder intersect does not assemble |
| 5 | cylinder + cone stacked (shared rim) | 6 | `booleanVolumeMismatch` | 348.192 | union sanity rejects a just-outside volume |
| 6 | cone/cylinder near-tangent | 5 | `wrongArea` 1.7e-4..2.9e-4 | 216.554..228.558 | marching intersection on the cone seam |
| 7 | box with three mixed-radius fillets | 1 | `blendFailed` | 23480.7 | three-edge corner, radii 10/5/5 |
| 8 | partial cylinder sector fillets | 3 | `blendUnsupported` | 20559.5, 44713.9, 43467.9 | sector seam / cap rim |
| 9 | partial cone sector fillets | 4 | `blendUnsupported` / `invalid` | 40663.6, 89366.1, 12471.9, 9640.68 | cone seam and sector caps |
| 10 | box + tiny cylindrical cut, 3-edge r0.2 fillet | 1 | `blendFailed` | 11.4579 | three edges at the cut rim |
| 11 | staged blend then union | 2 (`Q4`, `X1`) | `missingEdge` / `blendFailed` | 192.343, 60963.9 | staged blend geometry differs before the second pick |
| 12 | two cylinders union, bottom rim fillet | 1 | `invalid EULER` | 2485.86 | ambiguous seam vertex on the rim |
| 13 | **draft on a cylinder/cone side** | 3 (`A5`,`B2`,`B3`) | `wrongArea` / `draftUnsupported` / `invalidValue` | 2145.34, 987.866, 1138.79 | draft of a curved side about a tangent/offset plane |
| 14 | **staged blend on a box** | 1 (`Q7`) | `wrongArea` | 133.982 | second fillet on the first fillet's result |
| 15 | **bugs/modalg** | 5 | `booleanFailed`, `missingReference`, `EDGE_OFF_SURFACE`, wrong area | mixed | see `docs/mimo-brief-occt.md` |

Representative scripts, translated features and repro commands are in
`docs/mimo-brief-occt.md`, ranked by case count.

## One-command verifier

`tools/verify_wasm.sh <wasm> [--quick]` runs every check built here against one
wasm and prints one page: the OCCT corpus (when `/tmp/occt` exists), the
fillet, shell, boolean, shape, mass-properties, STEP round-trip,
tessellation, holes/patterns/mirror/draft sweeps, the 200-case crash fuzz, the two Fusion import samples (when their cache exists)
and the DeepCAD check (when `cases1000.json` exists).

```sh
tools/verify_wasm.sh /path/to/ck_wasm.wasm --quick   # 1 seed, 4 templates/sweep
tools/verify_wasm.sh /path/to/ck_wasm.wasm           # every template
tools/verify_wasm.sh /path/to/ck_wasm.wasm --accept  # full run, rewrite all baselines
```

Each sweep template runs in its own process and has a hard per-template
timeout (`alarm` plus a kill), so a wasm hang is reported as `ERROR: <template>:
hang` and never blocks the run. Checks that produce no result are errors, not
regressions. The page shows counts per class, new passes and regressions with case ids,
and exits non-zero only on a real regression (a case that passed in the
baseline and now fails). A check that cannot run or times out is printed as
`ERROR` with its reason and does not count as a regression. The OCCT corpus timeout defaults to 6 minutes (`--occt-timeout <seconds>`).
`--accept` is a full run that rewrites every baseline in one go with the
new wasm hash and the observed classes (the known 88324 import hang stays
`knownHang` in the 1234 baseline). The quick form
takes about 40 s here; the full form skips external corpora that are not
cached. Regression baselines are the ones listed under Sweeps and Files.

## Sweeps

Seeded parametric sweeps supplement the translated corpus. Each has an OCP
replay, a class-only baseline and a ranked Mimo brief.

| sweep | cases | result (kit wasm, seed 7) | docs |
|---|---|---|---|
| fillet / chamfer | 640 | 575 ok, 32 `blendOverflow`, 28 `wrongVolume`, 3 `blendFailed`, 2 `kernel-null` | `docs/fillet-sweep.md` |
| shell / thicken | 480 | 324 ok, 97 `wrongVolume`, 42 `refused`, 15 `shellFailed`, 2 `draftTooLarge` | `docs/shell-sweep.md` |
| boolean | 320 | 146 ok, 22 `wrongVolume`, 5 `invalid`, 2 `booleanFailed`, plus expected `okNoEffect` / `okEmpty` | `docs/boolean-sweep.md` |
| sweep / loft / revolve | 240 | 223 ok, 16 `wrongVolume`, 1 `ref-fails` | `docs/shape-sweep.md` |
| mass properties | 240 | 220 ok, 20 `wrongVolume` (all `loft3`) | `docs/massprops-sweep.md` |
| STEP round trip | 120 | 100 ok, 20 `ocp-wrongArea` (lofts) | `docs/step-roundtrip-sweep.md` |
| tessellation | 180 | 96 ok, 84 `meshVolume` (torus, cylinder, cone, loft3) | `docs/tess-sweep.md` |
| holes / patterns / mirror / draft | 120 | 120 ok | `docs/feature-sweep.md` |
| crash fuzz | 5000 | 662 ok, 4002 structured, 238 trap, 74 hang, 24 invalid | `docs/crash-fuzz.md` |
| Fusion import (rounds 1-3) | 800 | 745 ok, 55 invalid | `docs/mimo-brief-import.md` |

The baselines are `tools/fillet_sweep_baseline.json`,
`tools/fillet_sweep_baseline_seed11.json`, `tools/shell_sweep_baseline.json`,
`tools/boolean_sweep_baseline.json`, `tools/shape_sweep_baseline.json`,
`tools/massprops_sweep_baseline.json`, `tools/step_roundtrip_baseline.json`
and `tools/tess_sweep_baseline.json`, `tools/feature_sweep_baseline.json` and
`tools/crash_fuzz_baseline.json`.
Each stores the kit wasm hash (`18fcb4d451b97049` on this run), seed and
sample count; `--check` reports 0 regressions on the current wasm.

## Files

* `tools/occt_corpus.mjs` — selection, translator, runner, baseline/check.
* `tools/occt_picks.py` — OCP edge/face pick generator (cache only).
* `tools/occt_picks.json` — committed picks, keyed by case.
* `tools/occt_baseline.json` — current pass/fail baseline (1664 cases).
* `tools/occt_fetch.sh` — sparse OCCT test checkout.

OCCT is LGPL-2.1 with an exception. This repository does not vendor OCCT
source or test data: the runner reads a checkout, and the committed files
store derived geometry facts only. See `THIRD_PARTY.md`.
