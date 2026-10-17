# OCCT-derived corpus

`tools/occt_corpus.mjs` translates OCCT `tests/` scripts into the kernel's
feature tree and checks the result against OCCT's own `checkprops` values.
It is a frontier corpus: OCCT passes its own checks, so an `ok` here is
agreement with an independent kernel and a failure is a real gap.

## Selection and coverage

1653 cases are selected, all from a checkout of OCCT's public test scripts:

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
| blend/simple | 32 | 18 | 2 | 12 |
| draft/angle | 10 | 0 | 10 | 0 |
| **all** | **1653** | **1561** | **16** | **76** |

The boolean selection is every script in `tests/boolean` whose commands are in
the translated subset (primitives, `bop`/`bopcut`/`boptuc`/`bopfuse`/
`bopcommon`/`bfuse`/`bcut`/`bcommon`, transforms, `dset`, `checkprops`). The
2D/vertex/profile grids are excluded, not silently dropped. Blend is the
primitive-based `tests/blend/simple` set with cached edge picks; draft is
selected but untranslated (`needs-depouille`), so it is counted `unsupported`.

### Translation subset

* `box n dx dy dz` / `box n x y z dx dy dz` (negative extents normalised).
* `pcylinder n r h [angle]`, `pcone n r1 r2 h [angle]`, `psphere n r`
  (full sphere). `pcone` is built about world +Z via the `top` plane; this
  fixed 148 cases after an earlier version revolved it about Y.
* `trotate`, `ttranslate`, `dset` expressions (`sqrt`, `atan2`, `pi`, ...).
* `bfuse`/`bcut`/`bcommon` and the `bop` family: `bop a b` selects the pair,
  `bopcut` = `a - b`, `boptuc` = `b - a`, `bopfuse` = union,
  `bopcommon` = intersect.
* `blend result base r edge ...` -> `fillet` with cached midpoint + tangent.

Not translated (counted `unsupported` or excluded): `vertex`, `edge`, `wire`,
`mkplane`, `mkface`, `profile`, `prism`, `revol`, `wedge`, `ptorus`, partial
spheres, `tscale`, `copy`, `depouille`, and every command needing data files.

## `needs-data`

The fetched groups contain 7739 scripts; 4734 use `restore`/`locate_data_file`
and are counted `needs-data` (not selected). By group: boolean 2078, blend 292,
offset 1786, pipe 272, thrusection 248, mkface 44, draft 13, chamfer 1. The
separate OCCT test-data download is not vendored or fetched.

Chamfer (93 scripts) and offset (2030) were checked: none of their scripts use
only the translated command set. Chamfer scripts are driver files
(`chamf_shape` + `.draw`/`.rle` data), offset scripts need `OFFSETSHAPE`,
`offsetonface`, `offsetparameter`, etc. They are therefore not translated
yet; thickness lives in the offset group.

## Run

```sh
tools/occt_fetch.sh                  # sparse checkout -> /tmp/occt
node tools/occt_corpus.mjs --occt /tmp/occt \
  --wasm /path/to/kit/apps/cad/public/kernel/ck_wasm.wasm
node tools/occt_corpus.mjs --occt /tmp/occt --only bopcut_simple --quiet --json out.json
node tools/occt_corpus.mjs --occt /tmp/occt --verify-picks

# baseline / regression gate (one wasm instance, ~35 s for the full run)
node tools/occt_corpus.mjs --occt /tmp/occt --update-baseline tools/occt_baseline.json
node tools/occt_corpus.mjs --occt /tmp/occt --check tools/occt_baseline.json
```

`--check` exits non-zero only when a case that was `ok` in the baseline now
fails; new passes are printed but are not errors. `tools/occt_baseline.json`
holds `{case: {cls, rel, relVol}}`. When `checkprops` gives both `-s` and
`-v`, the larger relative error is used.

A normal run needs no Python or OCP; `tools/occt_picks.json` is the committed
edge-pick cache. Regenerate it only when the blend selection changes:

```sh
python3.12 -c 'import OCP'
node tools/occt_corpus.mjs --occt /tmp/occt --write-picks tools/occt_picks.json
```

## Edge picks: midpoint + direction

DRAW's `explode` numbers sub-shapes in `TopExp_Explorer` order, skipping
duplicates (`DBRep.cxx`). `tools/occt_picks.py` rebuilds the base with OCP,
snaps to the corresponding edge of the final body, and records its **midpoint
and unit tangent**. A corner point lies on up to three edges, so the corpus
resolves the kernel edge with both distance and direction. `--verify-picks`
runs the box-fillet cases (`A1`, `A2`, `A3`, `P8`, `P9`) and requires the
selected edge to match the cached midpoint/tangent and the area to agree.

## Failure triage

The failures group by operand geometry, not by test case. The original
three-grid selection had 35 failures from about 24 distinct scripts; after the
cone-orientation fix the whole corpus has 76. Distinct groups:

| # | geometry | cases | our class | expected | cause guess |
|---|---|---|---|---|---|
| 1 | sphere r1 at origin vs box [0,1]^3 | 12 (bfuse/bcut/bcommon A1,A2,A3,D3-D6) | `booleanFailed`; `EDGE_OFF_SURFACE` | area 14.6394 fuse, 13.3518 sphere-box cut, 5.2146 box-sphere cut | sphere seam/poles and the box corner; the arrangement is not closed or the edge is off-surface |
| 2 | cross-cylinder cut / rotated box tangent to a cylinder | 9 (`bopcut_simple/ZE7-9,ZF1,W7,Y2,Z4,Z6,ZB5`) | `invalid` `EULER`, `VERTEX_OFF_CURVE` | ZE7 33.8759; W7 18.5364 | non-manifold vertex or endpoint off the intersection curve |
| 3 | cylinder r9 h3 + cone r7-6 h4, cut by cylinder r1 at (5,0,-2) | 2 (`bcut_simple/G9,H3`) | `invalidValue` on the second `combine` | area 727.481 | chained boolean: the first result's lump selector is stale |
| 4 | perpendicular cylinders r1 and r0.5, intersect | 7 (`bopcommon_simple/ZE7-9,ZF1`; `boptuc_simple/ZE8,ZE9,ZF1`) | `booleanFailed` / `combineEmpty` | area 33.8757..33.8759 | cross-cylinder intersect does not assemble; ZF1 reports no overlap |
| 5 | cylinder + cone stacked (shared rim) | 6 (`bopfuse_simple/ZD5-7`, ...) | `booleanVolumeMismatch` | e.g. 348.192 | union sanity rejects a result whose volume is just outside the operand bound |
| 6 | cone/cylinder near-tangent | 5 (`bopcommon_simple/ZK5,ZK7,ZK8,ZK9,ZL1`) | `wrongArea` 1.7e-4..2.9e-4 | 216.554..228.558 | marching intersection on the cone seam, just over the 1e-4 band |
| 7 | box 100x100x10 with three mixed-radius fillets | 1 (`blend/simple/A4`) | `blendFailed` | area 23480.7 | three-edge corner with radii 10/5/5 |
| 8 | partial cylinder sector fillets | 3 (`blend/B3,B6,B7`) | `blendUnsupported` | 20559.5, 44713.9, 43467.9 | fillet on a sector seam / cap rim |
| 9 | partial cone sector fillets | 4 (`blend/C2,C6,C7,C8`) | `blendUnsupported` / `invalid` | 40663.6, 89366.1, 12471.9, 9640.68 | fillet across cone seam and sector caps |
| 10 | box with tiny cylindrical cut, 3-edge r0.2 fillet | 1 (`blend/Q2`) | `blendFailed` | 11.4579 | three edges meeting at the cut rim |
| 11 | staged blend then union | 1 (`blend/Q4`) | `missingEdge` | 192.343 | the first blend is not represented in the translated tree |
| 12 | two cylinders union, bottom rim fillet | 1 (`blend/Z1`) | `invalid EULER` | 2485.86 | union leaves an ambiguous seam vertex on the rim |

Representative scripts and translated features are in
`docs/mimo-brief-occt.md`, which ranks the kernel tasks by how many cases
each fixes and gives ready repro commands.

## Files

* `tools/occt_corpus.mjs` — selection, translator, runner, baseline/check.
* `tools/occt_picks.py` — OCP edge-pick generator (cache regeneration only).
* `tools/occt_picks.json` — committed picks, keyed by case.
* `tools/occt_baseline.json` — current pass/fail baseline.
* `tools/occt_fetch.sh` — sparse OCCT test checkout.

OCCT is LGPL-2.1 with an exception. This repository does not vendor OCCT
source: the runner reads a checkout, and the committed files store derived
geometry facts (edge points, expected values, classes) only. See
`THIRD_PARTY.md`.
