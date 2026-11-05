# Reference topology: rim + concave vertical fillets on a petal ring

What a correct result looks like for the `daisy-ring-n5-*-r0.8-ref` cases of the fillet table
(`crates/ck-model/tests/fixtures/blend_cases.json`, base `daisy-ring10-5`). It was read from a result built by a
third-party CAD kernel and checked face by face against the reference engine's result. It is the target for the
vertex blend where a concave vertical fillet meets two convex rim fillets (and for selecting both sets in one fillet).

## The body

One closed outline extruded 10 high: 5 petals on a hub. Each petal has two radial flank lines from the hub circle
(radius 10) out to radius 30, closed by a tip arc (radius about 8.83) tangent to both. Between two petals the outline
is a convex hub arc (radius 10, centred on the axis). A flank meets the hub arc at a right angle, so each of the 10
vertical edges there is concave (inner angle 270 degrees). 22 faces: top, bottom, 10 flank planes, 5 tip cylinders,
5 hub cylinders.

Edge sets in the table: V = the 10 concave vertical edges; T = the 20 top outer edges (10 flank lines, 5 tip arcs,
5 hub arcs); VT = both; VTB = VT plus the 3 bottom outer edges (two flanks and the tip arc) of one petal. Radius 0.8.

## Face census

| result | planes | cylinders | tori | other | faces |
|---|---|---|---|---|---|
| base | 12 | 10 | 0 | 0 | 22 |
| V (reference engine; second kernel the same) | 12 | 20 | 0 | 0 | 32 |
| T (reference engine) | 12 | 20 | 10 | 0 | 42 |
| VT (reference engine; third-party kernel the same, see below) | 12 | 30 | 20 | 0 | 62 |
| VTB (third-party kernel) | 12 | 32 | 23 | 0 | 67 |
| VTB (reference engine) | 12 | 32 | 21 | 2 spline | 67 |

The VTB results of the third-party kernel and of the reference engine agree face by face (type, area to 1e-3,
centroid) on every face except the bottom corner patches of the one petal with bottom fillets (see "Bottom corner").
The VT result is therefore taken from the reference engine.

## One petal, top side (VT)

Per petal, at the top:

- top plane (shared), 2 flank planes, the tip cylinder, and the hub cylinder of the adjacent gap;
- rim fillets (convex, radius r = 0.8): one cylinder per flank line (axis horizontal, at depth r below the top and r
  inside the flank), one torus on the tip arc (major radius = tip radius - r), one torus on each hub arc (major
  radius 10 - r = 9.2, axis = the body axis);
- vertical fillets (concave, r = 0.8): one cylinder per concave edge, axis at distance 10 + r from the body axis.
  It runs from the bottom face up to height H - r (9.2), not to the top: its top boundary is a horizontal circle;
- one corner patch per concave vertical edge: a **torus** with its axis on the vertical fillet's axis, centre at
  height H - r, minor radius r and major radius 2r (1.6).

Corner patch boundaries (all four are circles, every junction tangent):

| corner torus edge | neighbour |
|---|---|
| inner equator side (horizontal circle of radius r around the vertical axis, height H - r) | the vertical fillet cylinder |
| meridian circle at the flank end | the flank's rim cylinder |
| meridian circle at the hub end | the hub arc's rim torus |
| top circle (radius 2r around the vertical axis, on the top plane) | the top plane |

Reading: the rim fillet's ball centre runs at r below the top and r inside the wall. Where the wall turns round the
concave vertical fillet (radius r), the centre path is the offset of that fillet cylinder by r, a circle of radius 2r
around its axis, so the rim blend simply continues around the corner as a surface of revolution about the vertical
fillet's axis. It is the same shape as filleting the vertical edges first and then the top outline as one
tangent-continuous chain (flank line, arc of radius r, hub arc). No vertex blend surface is needed: no sphere, no
spline, no flat mitre facet. The corner patch spans the turn of the outline at that corner (90 degrees here) around
the vertical axis.

Per petal (top only): 2 flank rim cylinders, 1 tip rim torus, 2 vertical fillet cylinders, 2 corner tori, and
(per gap) 1 hub rim torus. Times 5 plus the original faces gives the VT census: 12 planes, 30 cylinders, 20 tori.

## Bottom corner (VTB, one petal)

On the petal whose bottom flanks and tip arc are also filleted, the bottom hub arc edges are not. At each of the two
bottom corners there three things meet: the concave vertical fillet, the convex bottom flank fillet, and the sharp
convex bottom edge of the hub cylinder.

- third-party kernel: the same torus construction as at the top (major radius 2r, minor r, axis on the vertical
  fillet's axis, centre at height r). Its edges: circles to the bottom plane, the vertical fillet and the flank's
  bottom fillet, and a free-form intersection curve with the hub cylinder (the torus is simply trimmed by the
  unfilleted hub wall). 23 tori in total.
- reference engine: a small spline patch at each of these two corners instead (2 spline faces, 21 tori). Its
  volume is 0.42 mm^3 above the third-party result (0.8 % of the 52 mm^3 net change), which is why the VTB case
  carries the third-party volume and the VT case (no bottom corners) the reference engine's.

## Volumes (mm^3, base 22196.3106)

| case | reference | volume | ours now |
|---|---|---|---|
| V r0.8 | reference engine (second kernel agrees within 0.002) | 22208.5209 | 22208.5212 (passes since the contact-edge fix; B-spline wall fillets) |
| T r0.8 | reference engine (second kernel refuses) | 22141.6657 | 22141.6658, 42 faces (12 / 20 / 10), no corner face (`blend/rim_reflex.rs`); before: refused, or flat facets and 5 % over-removal |
| VT r0.8 | reference engine, equal face by face to the third-party result | 22154.4172 | 22154.4172, 62 faces (12 / 30 / 20), 10 corner tori (2r, r) (`blend/torus_corner.rs`); before: refused (`branching blend selection`) |
| VTB r0.8 | third-party kernel | 22144.1717 | 22144.1720, 67 faces (12 / 32 / 23), the third-party census; before: refused (`branching blend selection`) |

Analytic check of the VTB reference: base minus the rim removals (straight sections, Pappus for the arcs) plus the
vertical-fillet additions gives 22146.0, 1.8 above the reference; the difference is the corner patches, which the
straight-section estimate ignores. The base itself is exact: the outline area times 10 is 22196.3106, equal to the
other system's unfilleted export measured by both engines.

## What to implement (from this)

1. When a concave vertical fillet and the convex rim fillets of the same radius are selected together, build the
   corner as the rim blend continued around the vertical fillet's top circle (torus about the vertical fillet's
   axis), not as a separate vertex blend and not as a mitre facet. With equal radii the vertical fillet ends at
   height H - r.
2. Where the rim on one side is not filleted (bottom corner of VTB), use the same torus and trim it against the
   unfilleted wall; the trim curve is free-form.
3. Rim only (T, sharp concave corner): the reference engine has no corner face at all there (42 faces); the two
   rim fillets meet along their intersection curve. (Done, all three: see `docs/blend-notes.md`, "The daisy rim
   corners".)
