# OCCT-derived corpus (top ~200)

`tools/occt_corpus.mjs` translates the simple OCCT `tests/` scripts to the
kernel's feature tree and checks each result against OCCT's own `checkprops`
value. It is a frontier corpus: OCCT passes its own checks, so an `ok` here is
agreement with an independent kernel and a failure is a real gap.

Selection is exactly 200 cases:

| group | selected | what it covers |
|---|---|---|
| `tests/boolean/{bfuse,bcut,bcommon}_simple` | 141 | solid primitive booleans (box, full sphere, cylinder/sector, cone/sector), transforms, copies |
| `tests/blend/simple` | 49 | primitive-based fillets; 30 have cached edge picks and are runnable |
| `tests/draft/angle` | 10 | primitive draft scripts (reported `unsupported`: no draft-face pick format yet) |

The boolean grids were filtered to scripts whose commands are all in the
translatable subset (primitives, boolean, transforms, `checkprops`); the
2D/vertex/profile cases are not silently dropped, they are simply not in the
selection. The remaining blend entries (partial spheres, tori, wedges, staged
blends) stay in the 200 and are counted as `unsupported` with the reason.

## Run

```sh
tools/occt_fetch.sh                 # sparse checkout -> /tmp/occt (~40 MB)
node tools/occt_corpus.mjs --occt /tmp/occt
node tools/occt_corpus.mjs --occt /tmp/occt --only blend --quiet --json out.json
node tools/occt_corpus.mjs --occt /tmp/occt --verify-picks
```

A normal run needs no Python or OCP. Edge picks are cached in
`tools/occt_picks.json`; only regenerating them needs OCP:

```sh
python3.12 -c 'import OCP'          # cadquery-ocp / OCP 7.9.x
node tools/occt_corpus.mjs --occt /tmp/occt --write-picks tools/occt_picks.json
```

## What is translated

* `box n dx dy dz` and `box n x y z dx dy dz` -> sketch + extrude (offset
  plane when `z != 0`).
* `pcylinder n r h [angle]` -> extrude, or a `top`-plane revolve sector about
  world +Z (the sector start matches OCCT's).
* `pcone n r1 r2 h [angle]` -> revolve of the axis-touching trapezoid.
* `psphere n r` (full sphere only) -> revolve of a half arc.
* `trotate`, `ttranslate`, `dset` expressions (`sqrt(2)`, `r/2`) -> `moveBody`
  and evaluated feature parameters.
* `bfuse` / `bcut` / `bcommon` -> `combine` union / cut / intersect on separate
  lumps. A cut that only touches the host (`cutNoEffect`) is followed by a
  `deleteBody` of the tool, matching OCCT consuming the tool.
* `blend result base r edge ...` -> `fillet` with the edge resolved by cached
  midpoint + tangent (below).

Not translated (counted `unsupported`): `wedge`, `ptorus`, partial spheres,
`tscale` (no non-uniform scale in the wire protocol), `copy` instances,
`profile` / `pipe` / `revol` construction, and `depouille`.

## Edge picks: midpoint + direction

DRAW's `explode` numbers sub-shapes in `TopExp_Explorer` order, skipping
duplicates (`DBRep.cxx`). `tools/occt_picks.py` rebuilds the same base shape
with OCP, finds the requested edge, snaps to the corresponding edge of the
final body, and records its **midpoint and unit tangent**. A corner point lies
on up to three edges, so the corpus resolves the kernel edge with both
distance and direction, not nearest-point alone. The cache carries the OCCT
provenance too (`occt_point`, `occt_dir`).

`--verify-picks` is the edge-identity check. It runs the box-fillet cases
(`A1`, `A2`, `A3`, `P8`, `P9`), where the checkprops area is known, and fails
unless each selected edge is within 2e-3 of the cached midpoint, the tangent
dot is at least 0.95, and the resulting area matches:

```
ok   verify-pick blend/simple/A1: ok 59527.8760 (4.0e-7)
ok   verify-pick blend/simple/A2: ok 58633.8492 (8.4e-7)
ok   verify-pick blend/simple/A3: ok 23914.1593 (1.7e-6)
ok   verify-pick blend/simple/P8: ok 145.137167 (1.2e-6)
ok   verify-pick blend/simple/P9: ok 146.392864 (9.3e-7)
```

## Result (2026-10-18, release wasm)

`checkprops -s` is **surface area** (not volume). Tolerance: 1e-4 relative for
booleans, 2e-3 for blends.

```
occt_corpus: 132/167 ok  {"booleanFailed":9,"invalid":14,"ok":132,
  "invalidValue":2,"blendFailed":2,"blendUnsupported":7,"missingEdge":1}
  (33 unsupported of 200)
```

| group | selected | ok | unsupported | failing |
|---|---|---|---|---|
| boolean/bcut_simple | 65 | 54 | 2 | 9 |
| boolean/bfuse_simple | 38 | 30 | 1 | 7 |
| boolean/bcommon_simple | 38 | 30 | 1 | 7 |
| blend/simple | 49 | 18 | 19 | 12 |
| draft/angle | 10 | 0 | 10 | 0 |

The failures cluster exactly on the roadmap weak spots:

* **Sphere/box booleans** (`*.A1`-`A3`, `*.D3`-`D6`): `booleanFailed`, or a valid
  volume with `validate` reporting `EDGE_OFF_SURFACE`. These are the same
  sphere/box combinations already known from the BenchCAD corpus.
* **`bcut_simple/F9`, `G1`, `G2`, `G3`**: boolean failures on mixed
  cylinder/box/cone contact.
* **`bcut_simple/G9`, `H3`**: chained boolean; the second `combine` gets
  `invalidValue` (lump selector after the first combine).
* **Blend partial sectors** (`B3/B6/B7/C2/C6/C8/D1`): `blendUnsupported` on a
  seam edge of a sector or a cone rim; `A4` and `Q2` `blendFailed` on
  multi-edge corners; `C7` invalid; `Z1` `EULER` after a union; `Q4` a staged
  blend whose first blend is not represented.

## Files

* `tools/occt_corpus.mjs` — selection, translator, runner.
* `tools/occt_picks.py` — OCP edge-pick generator (only needed for the cache).
* `tools/occt_picks.json` — committed picks, keyed by `blend/simple/<case>`.
* `tools/occt_fetch.sh` — sparse OCCT test checkout.

OCCT is LGPL-2.1 with an exception. This repository does not vendor OCCT
source: the runner reads a checkout, and the committed cache stores derived
edge points and directions only. See `THIRD_PARTY.md`.
