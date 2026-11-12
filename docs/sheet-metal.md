# Sheet metal (stages 1 to 4)

Stage 2 (hem, fold, contour flange, chained bends, unfold and refold, stable roles) is described from "Stage 2" below.

Three features and one result: `sheetRules` (the stock), `baseFlange` (the first plate), `edgeFlange` (a bent flange on
an edge of the plate) and, in the regen result, `sheetMetal` with the flat pattern and the bend table after each sheet
feature. Code: `crates/ck-model/src/features/sheet.rs` (model, bend arithmetic, flat pattern, pure and exact),
`sheet_features.rs` (the features and the folded solid). Tests: `crates/ck-model/tests/sheet_metal.rs`.

The behaviour was matched against black-box runs of a reference sheet metal tool with a 100 x 60 plate, thickness 2,
k-factor 0.44, radius 2 (flat 100 x 93.04778684 with two 20 mm flanges, 123.04778684 x 83.04778684 with four 15 mm flanges)
and probes with other thickness, radius, angle, height reference and position (listed below).

## Wire shapes

All lengths in mm, angles in degrees, names camelCase. A number may be an expression string (`"=Thickness*4"`; `Thickness`
is the rule's thickness inside `sheetRules`), or `"2 mm"` / `"90 deg"`.

```json
{"type":"sheetRules","id":"R1","name":"Steel2","thickness":2,"kFactor":0.44,"bendRadius":2,
 "reliefWidth":2,"reliefDepth":1,"cornerRelief":8,"cornerReliefShape":"notch","hemGap":0.01,"gap":2,"active":true}
{"type":"baseFlange","id":"B1","sketch":"S1","rule":"Steel2","flip":false}
{"type":"edgeFlange","id":"E1","edges":[[50,0,2],[50,60,2]],"height":20,"angle":90,"radius":2,
 "heightFrom":"outer","position":"inside","flip":false}
```

`sheetRules`: every field but `name` is optional. Defaults follow the thickness (2.5 when none is given): `bendRadius`
= thickness, `kFactor` 0.44, `reliefWidth` = thickness, `reliefDepth` = thickness / 2, `cornerRelief` = 4 x thickness, `gap` =
thickness, `hemGap` 0.01, `cornerReliefShape` `notch` (also `round`, `rectangular`, `tear`). A rule with an existing name
replaces it; `active` (default true) makes it the rule of the next base flange.

`baseFlange`: `sketch` (one closed region; holes are allowed), `rule` (a name; default the active rule, else the defaults),
`flip`. The region is extruded by the thickness along the sketch normal (against it with `flip`); `regions` selects as in
`extrude`. One base flange per part, as the first body feature.

`edgeFlange`: `edges` picks the edges of the base outline. A pick is a point on the edge, `[x,y,z]` or `{"at":[x,y,z]}`
(world coordinates; the nearest outline edge wins), or `{"edge": n}` (index into the outline, CCW from the sketch). A point
on the face the base ended on (the cap side) bends the flange towards that side (`direction: "up"` in the bend table), a point
on the face it started from bends it the other way (`"down"`); `flip` reverses. A pick given as an object may carry its own
`height`, `angle`, `radius`, `heightFrom`, `position`, `flip`.

* `height` (required), `angle` (default 90, the angle the leg turns through, 0 < angle < 180), `radius` (default the rule's).
* `heightFrom`: `outer` (default; from the outer virtual sharp, along the leg: leg = height - (r+t) tan(a/2)), `inner` (leg =
  height - r tan(a/2)), `tangent` (leg = height - (r+t) min(1, tan(a/2)); observed behaviour of the reference, equal to `outer`
  up to 90 degrees and to a fixed `r+t` beyond).
* `position`: `inside` (default: the outer face of the flange stays within the outline, the base stops `(r+t) tan(a/2)` short of
  the edge), `outside` (the bend starts at the outline edge), `middle` (half of the inside offset).

## Result

The regen result gains `sheetMetal`, one entry per sheet feature (sorted by id):

```json
{"id":"E1","type":"edgeFlange","rule":{...},
 "flatPattern":{"rule":"Steel2","thickness":2,"kFactor":0.44,"flatSize":[100,93.04778684233861,2],
   "outline":[[[100,-16.5238934],[100,76.5238934],[0,76.5238934],[0,-16.5238934]]],
   "bends":[{"bend":1,"feature":"E1","direction":"up","angleDeg":90,"includedAngleDeg":90,"radius":2,
             "allowance":4.523893421169302,"length":100,"start":[0,1.7380532894153489],"end":[100,1.7380532894153489]}],
   "bendZones":[[0,100,-0.5238934,4]],"unfolded":false}}
```

* `flatSize` = width, length (in the plane of the base sketch) and thickness. `outline`: the outer loop (CCW, collinear points
  dropped) then the holes of the base. The flat pattern keeps the base in place; flanges unfold about their bend lines.
* Bend allowance `BA = (r + k t) a` (a in radians). In the flat pattern the zone of a bend covers `[b - BA, b]` measured
  inward from the outline edge, `b` the position offset above; the bend line is the middle of the zone and runs along the edge,
  in the direction of the CCW outline. `length` is the length of the bend line. Bends are numbered in the order they were made.
* `bendZones`: `[xmin, xmax, ymin, ymax]` of each zone in flat coordinates.

Errors: `nothingToCut` (no sheet yet), `missingSketch`, `noClosedProfile`, `invalidValue {field, detail, hint}`,
`flangeHeightTooLow {minHeight, detail, hint}` (the height must reach the end of the bend; exactly the minimum gives a bend
without a leg), `sheetMetalUnsupported {detail, hint}` for what stage 1 does not do, and the boolean codes.

## Corners

Two flanges on neighbouring edges meet at a corner. The bend zone of each stops where the neighbour's bend begins (the
corner is left open by `b` of the neighbour), and the legs of flanges made in one `edgeFlange` stay `gap` apart: each leg
is pulled back to `(b - r) + gap / sqrt 2` from the corner along its edge (measured from the neighbour's wall), which keeps the
distance between the inner corners of the two walls equal to `gap` (both at 90 degrees, edges at right angles). A flange added
by a later feature next to an existing one trims the existing bend zone but not its leg, and its own leg is as wide as its zone.
This is the `notch` shape. `round`, `rectangular` and `tear` are stored in the rule and reported, but stage 1 builds and
unfolds every corner as `notch`.

## Stage 1 limits

* Flanges need a base outline of straight edges (arcs in the outline are fine without flanges); one flange per edge;
  sheet features must follow each other (a hole or cut between the base and a flange is `sheetMetalUnsupported`).
* The flat pattern is analytic (no unfold of the B-rep): holes of the base are copied, cuts across bends are not supported.
* Corner relief shapes other than `notch`, relief width and depth (they matter for partial-width flanges), convert-to-sheet, perforate: later stages (hem, fold, contour flange, refold: stage 2, below; DXF and bend table export: see Export).

## Probes matched to 1e-6 (reference: t 2, k 0.44, r 2, 100 x 60 base, one flange on the bottom edge unless noted)

| probe | flat length |
|---|---|
| height 20, 90, outer / inner / tangent | 76.5238934, 78.5238934, 76.5238934 |
| 45 degrees, outer / inner / tangent | 78.9482382, 79.7766653, 78.9482382 |
| 120 degrees, outer / inner / tangent | 72.1754514, 75.6395530, 75.1036547 |
| position inside / outside / middle | 76.5238934, 80.5238934, 78.5238934 |
| height exactly the bend (4.0) / 4.001 | 60.5238934 / 60.5248934 |
| r 3, middle | 78.5946897 |

## Export: DXF flat pattern and bend table

Code: `crates/ck-model/src/features/sheet_export.rs` (pure functions of the `flatPattern` of a `sheetMetal` entry, no body or
state). Tests: `crates/ck-model/tests/sheet_export.rs`, `crates/ck-wasm/tests/sheet_export.rs`. Matched against black-box
exports of the reference tool for the 100 x 60 plate with two and four 90 degree flanges (entity counts per layer, line
coordinates, the bend table rows to 1e-6) and a few probes (down bend, 45 degree flanges, rounded corner).

### Requests (stateless)

```json
{"type":"sheetDxf","flatPattern":{...},"version":"R2000","units":"mm","bendLines":true,"tangentLines":false,
 "bendNotes":false,"noteHeight":3,"layers":"roles","layerNames":"cadkit"}
{"type":"sheetBendTable","flatPattern":{...},"format":"csv","body":"Body1"}
```

`flatPattern` is the object of a `sheetMetal` entry of the regen result (the entry itself is accepted too). Replies:
`sheetDxf` gives `{dxf, bends, bounds:[xmin,ymin,xmax,ymax] (mm, outer contour), entities:{layer: count}}`;
`sheetBendTable` gives `{format, text, bends}`. Refusals name the field (`flatPattern`, `units`, `version`, `format`, ...).
In Rust: `flat_pattern_dxf(&FlatPattern, &DxfOptions)`, `bend_table_csv`, `bend_table_json` / `pretty`, `dxf_reply`,
`bend_table_reply`.

### DXF

* `version`: `R2000` (default, `AC1015`, handles and objects section) or `R12` (`AC1009`). Every file is read back clean by
  an outside reader (checked with ezdxf's audit for both versions).
* `units`: `mm` (default), `cm`, `m`, `in`, `ft`. `$INSUNITS` is 4, 5, 6, 1, 2 and `$MEASUREMENT` 1 (metric) or 0; every
  coordinate, radius and text height is scaled.
* Layers, all declared even when empty: `CUT_OUTER` (outer contour, colour 7), `CUT_INNER` (holes and cut-outs, 4),
  `BEND_UP` (1) and `BEND_DOWN` (5) (bend centre lines, dashed), `BEND_TANGENT` (3, dashed; two lines per bend, half a bend
  allowance either side of the centre line, so the two ends of the allowance; off unless `tangentLines`), `BEND_NOTES` (8,
  declared when `bendNotes`). `layers: "single"` puts everything on layer `0`. `layerNames: "reference"` names the contour
  layers `OUTLINE` and `CUT`, as the reference tool does.
* Entities: inner contours first, then the outer one, then per bend its centre line (start to end of the bend table) and
  its note. The flat pattern holds polylines (curves are sampled), so the writer reads them back: consecutive collinear
  pieces are one `LINE`, a run of at least 12 equally spaced points on one circle is an `ARC` (counter-clockwise start and
  end angle in degrees) or, when the loop closes on itself, a `CIRCLE`. A polygon of fewer than 12 vertices on a circle
  stays lines.
* Notes (`bendNotes`): a `TEXT` on `BEND_NOTES` reading `B1 UP 90%%d R2` (bend number, direction, bend angle, inside
  radius), centred on the middle of the bend line, rotated along it (0 to 180 degrees) and set half a text height to its
  left (up for a bend along x, to -x for a bend along y). `noteHeight` is in mm, default 3.
* `bendLines: false` leaves the centre lines out for a cutter that would cut them (notes and tangent lines are separate options).

### Bend table

* CSV: header `body,rule,thickness_mm,k_factor,bend,angle_deg,direction,radius_mm,allowance_mm,length_mm,start_x_mm,
  start_y_mm,end_x_mm,end_y_mm,included_angle_deg`, one row per bend, text fields quoted, numbers with 9 decimals, `\r\n`
  line ends. `angle_deg` is the angle the leg turns through, `included_angle_deg` is 180 less it.
* JSON (two-space indent, keys in alphabetical order, whole floats keep `.0`): `bend_zones`, `bends` (`allowance`,
  `angle_deg`, `bend`, `direction`, `end`, `included_angle_deg`, `length`, `radius`, `start`), `body`, `cutout_shapes`
  (`{circle:{center,radius}}` or `{segments:[{line:[a,b]} | {arc:{center,radius,start_deg,sweep_deg}}]}`, same order as
  `cutouts`), `cutouts` (the hole loops of the base), `flat_size_mm`, `k_factor`, `outline` (the outer loop), `perforations`
  and `pockets` (empty until those exist), `rule`, `thickness`, `unfolded`.

### Differences from the reference tool

* The contour layers are `CUT_OUTER` / `CUT_INNER` by default (the reference: `OUTLINE` / `CUT`); `layerNames: "reference"`
  gives its names. `BEND_TANGENT`, R12 output and the unit choice are additions (the reference writes R2000 millimetres).
* The outer loop starts at another corner than the reference's (same sides, same direction); the bend lines run exactly as
  in its bend table.
* Not exported: text (`TEXT` / `ETCH`), pockets, perforation layers, ellipses and splines (no such features yet),
  `curves: arcs`, per-body files.

# Stage 2

New features: `hem`, `fold`, `contourFlange`, `unfold`, `refold`; `edgeFlange` gains `then` (chained bends); every sheet
entry of the regen result gains `roles`. Code: `sheet_features.rs` (the features and the folded solid), `sheet.rs` (bend chain,
flat pattern), `sheet_fold.rs` (panels and zones of folds, pure 2D), `sheet_contour.rs` (the contour chain, pure 2D),
`sheet_roles.rs` (role grammar and matching). Tests: `crates/ck-model/tests/sheet_metal2.rs`. As in stage 1 the expected numbers
are black-box runs of the reference tool (rule t 2, k 0.44, r 2, base 100 x 60), including the volumes and bounding boxes of
its bodies, to 1e-6 (bounding boxes of bent bodies to 1e-3: the kernel's box of an arc is a grid).

## Wire shapes

```json
{"type":"hem","id":"H1","edges":[[50,0,2]],"length":10,"gap":0.01,"flip":false}
{"type":"edgeFlange","id":"E1","edges":[[50,0,2]],"height":20,"then":[{"angle":90,"height":10,"flip":false}]}
{"type":"fold","id":"F1","points":[[40,0,2],[40,60,2]],"angle":60,"radius":2,"position":"centerline","fixed":[90,30,2],"flip":false}
{"type":"fold","id":"F2","line":{"sketch":"S2","curve":"f1"},"angle":90}
{"type":"contourFlange","id":"C1","sketch":"S1","curves":["e1","e2"],"distance":40,"rule":"Steel2","flip":false,"reverse":false}
{"type":"unfold","id":"U1"}   {"type":"refold","id":"U2"}
```

Picks of `edgeFlange` and `hem` (`edges`): `[x,y,z]`, `{"at":[x,y,z]}`, `{"edge":n}` (index), and by role:
`{"role":"P0.side.e2"}`, `{"curve":"e2"}`, or the string `"P0.side.e2"` (see Roles).

**hem**: a 180 degree bend of inside radius `gap` (default the rule's `hemGap`, 0.01) that lays the leg back against the sheet.
`length` is measured to the outer face of the hem: the leg is `length - (gap + t)` long, the bend starts at the edge
(`position: outside`, no setback), the flat grows by `BA + leg`, `BA = (gap + k t) pi`. `length` must reach `gap + t`
(`flangeHeightTooLow`, `minHeight`). A pick on the top face (cap side) curls the hem towards the bottom (`direction: "down"`),
a pick on the bottom face towards the cap (`"up"`); `flip` reverses. Hems on neighbouring edges collide: refused
(`sheetMetalUnsupported`), as is a hem beside an edge flange.

**then** (chained bends, at most 16, not on a hem): each item is a flange on the far edge of the leg before it, with its own
`angle` (default 90), `height` and `flip`; radius, `heightFrom` and `position` are the flange's. `flip` turns that bend the other
way from the flange's own (relative to the first bend, not to the one before; the bend table says `up` / `down`). The leg of bend
`j` is `height_j - reach_j - b_{j+1}` where `reach_j` is as in `heightFrom` and `b_{j+1}` is the offset the next bend's start has
from the far edge for `position` (`inside`: its setback `(r+t) tan(a/2)`, `middle`: half of it, `outside`: 0); the last leg is
`height - reach`. A bend that the height does not reach is `flangeHeightTooLow {bend, minHeight}`. The bend rows are numbered
along the chain.

**fold**: `points` (two world points, extended to a line) or `line: {sketch, curve}` (a sketch line, followed when the sketch
changes; any sketch plane: the line is projected into the plane of the base). `angle` (default 90, 0 < angle < 180), `radius`
(default the rule's), `position`: where the line sits in the bend. The bend zone is `BA` wide, `BA = (r + k t) a`, and measured
from the line towards the moving side (`u`): `centerline` `[-BA/2, BA/2]`, `start` `[0, BA]` (the bend starts at the line, seen
from the fixed side), `end` `[-BA, 0]`, `mould` `[-S, BA - S]` with `S = (r+t) tan(a/2)` (the line is the outer virtual sharp).
`fixed` (a point, or a wall by role such as `"P0.side.e4"`) names the side that stays, default the larger one; later folds keep the
first fixed panel. `flip` turns the moving side towards the bottom (`direction: "down"`). The flat pattern does not change size
(the zone is part of the sheet); the bend row has `start` / `end` on the middle of the zone and the zone box is the strip within
the outline. There may be several folds if their zones do not overlap; they cut the base into panels (a chain of bends).

**contourFlange** (the base from an open chain of sketch lines): `curves` lists the sketch lines (default every line of the
sketch), joined end to end into one open chain; `distance` is the extrusion. The material lies to the right of the direction of
travel (left with `flip`) and the chain is the face it is drawn on: the corner is the virtual sharp of the bend, the setback is
`(r+t) tan(a/2)` where the chain is the outer face of the bend (the bend turns towards the material, `direction: "up"`) and
`r tan(a/2)` where it is the inner face (`"down"`). A segment must be at least as long as its two setbacks. `reverse` extrudes
against the sketch normal. The flat pattern runs along the chain (x) and the extrusion (y: `0..distance`, `-distance..0` when
exactly one of `flip` and `reverse` is set, as the reference shows it). Arcs in the chain, edge flanges on its walls, and more
than one chain: later.

**unfold**: the body becomes the flat sheet (the flat pattern extruded by the thickness where the base was; the reference's box
and face count match). While unfolded, flanges, hems and folds are refused (`sheetMetalUnsupported`, "refold first"). **refold**
rebuilds the folded solid from the model: the bend rows (angle, radius, allowance, zone) are the source of truth, the flat pattern
is derived from them and the folded solid from the same rows, so unfold and refold round-trip exactly (volume, face count, box).
Unfold of an unfolded sheet, refold of a folded one: `sheetMetalUnsupported`; no sheet: `nothingToCut`.

## Roles

A point pick breaks when the base changes. A role names a face or an edge by what it is in the sheet.

```text
role   := face | face '|' face            an edge is the seam of two faces
face   := panel '.' pface | bend '.' bface
panel  := 'P' n      P0 = the base plate (the root panel of a folded base; the first leg of a contour flange);
                     then the legs of the flanges in the order they were made, bend by bend (a chain's legs follow its flange);
                     folded base: the panels in walk order from the fixed one
bend   := 'B' n      numbered from 1 as in the bend table (`role` of every bend row)
pface  := 'front' | 'back' | 'tip' | 'side.' name
bface  := 'front' | 'back' | 'side.start' | 'side.end'
name   := the id of the sketch curve the wall runs along (panels of the base), or 'start' | 'end' (legs and contours)
```

`front` is the face the base started on (the sketch plane), `back` the cap face; they stay front and back through the bends, so
a bend's `front` is the outer face of an `up` bend and the inner face of a `down` bend. `tip` is the free end of a leg. Faces
that are coplanar in the body (a wall of the plate, its bends and legs) are one face: their roles name the same face.

* In features: `edgeFlange` / `hem` `edges` take `{"role":"P0.side.e2"}` (the wall along the sketch curve `e2`, the flange bends
  towards the cap), `{"role":"P0.back|P0.side.e2"}` (the edge of the back face: towards the cap) or `{"role":"P0.front|P0.side.e2"}`
  (towards the front). `fold` `fixed` takes a wall. The role is resolved against the model by the curve id, so the feature
  follows the curve when the sketch changes (points move, the outline is renumbered when a corner is added, the curve id stays).
  Only the walls of P0 take flanges in stage 2.
* Query: every `sheetMetal[]` entry of the regen result has `roles`:
  `{"faces":[{"role":"P0.front","face":"B1.C0"}...],"edges":[{"role":"P0.front|B1.front","faces":[..],"mid":[x,y,z]}...]}`.
  `face` is the id of the body face the role names (`null` for a piece that is not in the body, such as a leg of length 0),
  `edges` the seams of the named faces that share an edge, `mid` a point on it. The same from Rust:
  `sheet_features::sheet_roles(&Brep)`.

## Probes matched to 1e-6

| probe | result |
|---|---|
| hem length 10 / 5 / 2.01 / 20 / 50 (t 2, gap 0.01) | flat length 70.786017, 65.786017, 62.796017, 80.786017, 110.786017; length 2 refused |
| hem t 3 length 12, gap 1 length 10, k 0.3, rule hemGap 0.5 | 73.168318, 72.906194, 69.906372, 71.835398 |
| two hems on opposite edges | 81.572035; adjacent hems refused |
| fold 60 at x = 40, centerline / start / end / mould | zone 38.492036..41.507964, 36.984071..40, 40..43.015929, 39.293472..42.309401 |
| fold 90, centerline / start / end / mould | zone 37.738053..42.261947, 35.476107..40, 40..44.523893, 39.476107..44 |
| fold 60 with the left side fixed, start / mould | zone 40..43.015929, 37.690599..40.706528 |
| fold 60: box, volume | x 18.797845..100, z 0..36.335081, 12015.079645 |
| fold 90 / 120 / 60 down / 60 at x = 70 | x min 38.261947 / 39.015929 / 18.797845 (z -34.335081) / x max 86.202155 |
| folds 45 at 20 and 60 at 70 | box x 4.96013..86.202155, z 27.674827, volume 12026.389379 |
| one flange 20 then 10 (90, 90) | flat 83.047787, zones -0.523893..4 and -17.047787..-12.523893, volume 16684.955593 |
| then 45 / first 45 / 120 / r 3 | flat 85.472132 / 85.472132 / 78.699345 / 82.189379 |
| then with position outside / middle, heightFrom inner | 91.047787 / 87.047787 / 87.047787 |
| then flip, 45-45-45 chain | volume 16684.955592, 19825.491595 |
| contour L (0,0)-(0,20)-(30,20), t 2, distance 40 | flat 46.523893 x 40, bend line 18.261947, box 30 x 20 x 40 |
| contour Z / acute 45 / obtuse 135 / t 3 r 1.5 k 0.3 | 67.047787, 58.889364, 55.413257, 44.769911 |

## Differences from the reference and limits

* Folds: the outline must be straight and convex, and the fold line must meet the outline at right angles within its zone
  (the reference accepts oblique lines; their bend sides are not planar). A fold and an edge flange or hem cannot be on the same
  base in stage 2 (`sheetMetalUnsupported`; the reference allows both, with a relief where they meet). Zones that overlap, a
  hole in a zone, a fixed side that is not the first fixed panel: `sheetMetalUnsupported`. The fold line is always extended to
  the outline (the reference does too: lines that stop short of the edges fold the whole width).
* Contour flange: lines only; no edge flange, hem or fold on it yet.
* Picks by role are only for the walls of P0. Roles are recomputed at every rebuild; the sketch does not keep them.
* A pick point's height is used only for the side (cap or front): flanges after a fold would need the pick resolved on the folded
  panel.

## What stages 3 and 4 need from this

* DXF and the bend table: the flat pattern (`outline`, `bends` with `start` / `end` and `role`, `bendZones`, `unfolded`) is
  complete for every stage-2 feature; export only has to emit it.
* Flanges and hems after folds, folds after flanges, flanges on contour flange walls, cuts across bends: the sheet is a root
  panel with flange chains (stage 1) or a tree of panels (folds). A common panel tree (every panel with its flat polygon and rigid
  transform, every bend with its parent panel, line and zone) would carry all of them; `sheet_fold::layout` and `bend_xform` are
  that for folds already, and a flange chain hangs on any panel through `strip(...)` in the frame of its edge.
* Convert to sheet and perforate (flat pattern only) need a way to build the model from a solid (panels from faces); holes of the
  base are already copied to the flat pattern.


# Stage 3: the panel tree

Code: `sheet_tree.rs` (pure 2D: polygons, panels, bends, flat pattern, flanges, folds, reliefs), `sheet_tree3d.rs` (the folded
solid and roles of a tree), routing in `sheet_features.rs`. Tests: `crates/ck-model/tests/sheet_tree.rs`.

A sheet is a **tree of flat panels joined by bends**, kept in the flat layout: a panel is a polygon in flat coordinates (the tag of
each edge is its wall name), a bend is a strip (a line, a zone of width `BA = (r + k t) a` measured along the normal from the
parent panel to the child) with angle, radius and direction. The flat pattern is the union of panels and zones; the folded solid
is every panel extruded by the thickness and every bend, moved by the chain of bend transforms from the root panel. A flange,
a hem, a chained bend and a fold are all "a strip on a panel edge" or "a strip across a panel", so they combine freely.

The legacy forms (a base with flanges, a folded base, a contour flange) still run their stage 1/2 code. They become a tree, exactly
(same panels, bend rows, flat pattern, volume and face count; tested for every form), the moment a feature needs more:

* `edgeFlange` / `hem` go to the tree when the sheet is folded, is a contour flange, already is a tree, has a concave base, when a
  pick names a panel other than P0, or when the rule's `cornerReliefShape` is not `notch`.
* `fold` runs the stage 2 fold first and falls back to the tree where that refuses (flanges present, oblique line, concave panel,
  folds on a folded panel, a contour flange).

## What is new

* **Flanges and hems on any panel**: `edges` picks take `{"role":"P2.side.e3"}` / `"P2.tip"` / `"P1.side.start"`, `{"edge":n,"panel":k}`
  (n indexes the ring of panel k) or a point (the nearest wall of the folded sheet wins; the side of the face the point is on
  gives the direction). Panels: P0 root; the legs of flanges in the order made (a chain's legs follow); the pieces beyond a fold
  are appended in the order the folds were made. `P<n>.tip` is the free end of a leg, `P<n>.side.start|end` its sides, the
  walls of a panel cut from the base keep the sketch curve ids. A contour flange has P0..P(n-1) as its legs with the walls
  `side.start` / `side.end` (the extrusion ends), `side.cs` / `side.ce` (the chain ends).
* **Several folds / flanges / hems on one base**, in any order; a flange added beside a bend trims the bend as in stage 1.
* **Oblique folds**, **concave bases** (L, U shapes: a fold line may cross a concave panel several times; every piece beyond
  the line hangs on the root piece by its own bend, a zig-zag chain is refused), folds on folded panels (`"panel":k` names the
  panel when the line points are ambiguous; the side that stays is the one the panel hangs on). A fold crossing the edge a flange
  hangs on, a hole in the zone, a corner of the panel inside the zone: `sheetMetalUnsupported` with the reason.
* **Corner reliefs** (`cornerReliefShape` of the rule), built in the plate and visible in the flat pattern and the DXF:
  `round`: a circle of diameter `cornerRelief` centred on the corner bites the plate (a true cylinder face in the solid, an arc in
  the DXF); `rectangular`: the square of side `cornerRelief` centred on the corner; `tear`: a slit `reliefWidth` wide and
  `cornerRelief / 2` deep along the bisector. The bend zones end where the relief ends and the legs are as long as their zones.
  `notch` is unchanged. A relief larger than the edge beside it: `sheetMetalUnsupported`.
* `fold` accepts `"panel": k`; `edgeFlange` / `hem` picks accept `"panel": k` with `"edge": n`.

## Limits of stage 3

* Oblique fold lines: the flat pattern is exact, the bend body has square ends (the true wrapped end is not a plane), so the solid
  differs from `flat area x thickness` by a sliver at the two ends of each oblique bend.
* Curved base outlines still take no flanges; relief sizes are my own definition (no reference measured).
* Legs that overlap in the flat layout are refused (`sheetMetalUnsupported`, give the corner a relief).


# Stage 4: convert, cut, corners

All three work on the panel tree (a legacy sheet converts exactly, as in stage 3). Tests: `tests/sheet_tree.rs`.

## Wire shapes

```json
{"type":"sheetConvert","id":"V1","thickness":2,"rule":"Steel2"}
{"type":"sheetCut","id":"X1","sketch":"S2","regions":["e1","e2"]}
{"type":"sheetCorner","id":"K1","corners":[[x,y,z],{"panel":1,"vertex":2}],"mode":"round","radius":5}
{"type":"sheetCorner","id":"K2","corners":[[x,y,z]],"mode":"chamfer","distance":3}
```

All fields but the type are optional except where stated; every result has a `sheetMetal[]` entry (`type` `sheetConvert` /
`sheetCut` / `sheetCorner`) with `rule`, `flatPattern`, `roles` as before. Errors: `nothingToCut` (no body / sheet), `invalidValue
{field, detail, hint}`, `sheetMetalUnsupported {detail, hint}`.

**sheetConvert** (`sheet_convert.rs`): the current solid becomes a sheet. `thickness` is read from the body (the smallest gap
between facing planar faces) unless given; `rule` names the rule that supplies the k-factor (default the active rule or the
defaults), the converted rule is named `Converted` (thickness, bend radius of the first bend) and becomes active. The body must
be a bent plate: planar panels (pairs of parallel faces `t` apart), cylindrical bends (coaxial pairs `t` apart in radius, straight
axes) and planar walls. The panels are walked from the largest and unfolded into its plane (front = the face of the largest
panel with the first ring edge; `N` points into the material); the sheet is rebuilt from the tree and its volume must be within
1 % of the body's, else the body is kept and the feature refused. Refused: sharp corners between panels (no bend radius),
cones / tori / free-form bends, a body that is not one connected chain of panels, reliefs and embossed features (the volume check
catches these). The flat pattern, bend table and DXF are the sheet's. Tested by round trip: flanged, folded and chained sheets
converted back give the same volume, flat size, flat area, bend allowances, angles and radii.

**sheetCut** (`Tree::cut`, `sheet_tree3d::cut_tools`): the closed regions of a sketch are cut out of the sheet *through the flat
pattern*: the sketch is read in the flat layout (a sketch on the plane of the base or parallel to it; coordinates are the flat
coordinates). A region is subtracted from every panel it meets (a hole, or a notch in the outline; the panel keeps the tags of its
other edges) and stored on the tree; a bend zone it crosses loses the region from the flat pattern (so the DXF shows the hole
or slot across the bend) and the bend solid loses the wedge of the bend that the region maps to (the angle across the zone is
`u / BA x a`; a region bounded by lines along and across the bend is one exact wedge, any other boundary is a staircase of
wedges 0.25 mm of zone wide). Refused (`sheetMetalUnsupported`): a region that misses the sheet, one that removes or splits a
panel, a region with an island. The bend table keeps the full bend lines. `regions` selects sketch regions as in `extrude`.

**sheetCorner** (`Tree::round_corner`): `mode` `round` (default; `radius`) or `chamfer` (`distance`; `size` is accepted for either).
`corners`: world points (the nearest panel vertex wins) or `{"panel":k,"vertex":i}` (vertex `i` of the ring of panel `k`). Both
walls of the corner must be free (no bend or zone side) and the corner convex, and the walls long enough for the tangent
length (`r tan(turn/2)` / `distance`). The fillet is a true cylinder face in the solid and an arc in the DXF.

## Limits of stage 4

* `sheetCut`: staircase approximation of non-rectangular cuts across bends (the flat pattern is exact); the sketch is not
  followed onto folded panels (use a sketch parallel to the base plane); the cut does not split panels.
* A plain through `extrude` cut of a folded sheet is tried as a `sheetCut` first (the part stays a sheet). When the panel tree cannot take
  it (a region with an island, or a rebuild whose boolean is refused, for instance on a cut tool with a sliver wall from a sketch that
  is a few nanometres off axis), it runs as an ordinary cut of the solid instead of failing; the part is then a plain solid after it. Without
  that fallback a history read back from a file (sheet metal parts whose slots are plain cuts) stopped at the first such cut and ended in an imported remainder.
  The fallback is not silent: the cut's feature row carries the warning `sheetCutFallback` (with the sheet cut's refusal in `reason` / `reasonDetail`).
* `sheetConvert`: constant-thickness bent plates with straight cylindrical bends only (see above).
* `sheetCorner`: corners of free walls of one panel only (no corners across bends).


# Stage 5: partial width flanges, shaped flanges, curved bases

All of it runs on the panel tree (a feature that asks for it, or a base outline with arcs, converts the sheet like any stage 3
feature). Code: `sheet_tree.rs` (`carve_strip`, `profile_leg`), `sheet_features.rs` (`parse_profile`, `exec_flange_tree`). Tests:
`crates/ck-model/tests/sheet_tree.rs` (`partial_*`, `sketched_profile_*`, `flange_on_a_base_with_an_arc`). The existing wire shapes are
unchanged; everything below is a new optional key of `edgeFlange`, on the feature or on one pick (a pick's own value wins).

```json
{"type":"edgeFlange","id":"E1","edges":[{"at":[50,0,2],"start":20,"length":40}],"height":20,"position":"outside","relief":"rectangular"}
{"type":"edgeFlange","id":"E2","edges":[{"at":[50,0,2],"from":[20,0,1],"to":[60,0,1]}],"height":20}
{"type":"edgeFlange","id":"E3","edges":[{"at":[50,0,2],"start":30}],"profile":[{"line":[[0,0],[40,0]]},{"line":[[40,0],[40,12]]},
  {"arc":{"center":[30,12],"from":[40,12],"to":[30,22],"ccw":true}},{"line":[[30,22],[10,22]]},
  {"arc":{"center":[10,12],"from":[10,22],"to":[0,12],"ccw":true}},{"line":[[0,12],[0,0]]}]}
{"type":"edgeFlange","id":"E4","edges":[[50,0,2]],"profile":{"sketch":"S2"}}
```

**Partial width.** `start` and `length` (mm, expressions allowed) give the part of the picked edge the flange covers, measured along the
edge from its start vertex (the outline runs counter-clockwise, so "start" is the left end seen from inside the sheet); `start` alone runs to
the end of the edge, `length` alone starts at the vertex. Or two world points on the edge, `from` and `to` (any order, projected onto the
edge): the span between them. After a partial flange the edge it was on is split into pieces, and a pick names the piece it is nearest to
(`start` counts from the start of that piece; use `from` / `to` for positions that do not depend on it). A span that is not inside one free
piece of the edge (past its end, over another flange), or a whole-edge flange on an edge a partial one has split, is
`sheetMetalUnsupported`.

* The strip the flange hangs on becomes an edge of its own (tag `<tag>_s<n>`, also the role `P<k>.side.<tag>_s<n>`). With `position`
  `inside` / `middle` the strip sits `b` inside the old line and the plate keeps the old line beside it, so the plate is notched by
  the span x `b`; walls join the two (tags `rls<n>a`.., not pickable, not in the roles). With `outside` (`b = 0`) the plate is untouched.
* Where the span reaches an end of the edge (the corner), no wall is made: the neighbouring edge follows the moved strip, as for a
  whole-edge flange, and the corner rules of stage 1 to 3 apply (a flange beside an existing one stops where the neighbour's bend
  starts and trims the neighbour's bend zone; legs of one feature at right angles keep `gap` apart only for whole-edge flanges).
* `relief`: `"rectangular"` (the rule's `reliefWidth` x `reliefDepth`) or `{"width":w,"depth":d}` cuts a rectangle into the plate beside
  each end of the span that stops inside the edge: `w` wide, reaching `d` beyond the bend's start line. `"none"` (default) cuts
  only the notch. Not made where the span meets a corner.
* The bend is as wide as the span, the bend table row has that `length`, the flat pattern shows the notch, the bend zone and the leg
  at their place, and `unfold` / `refold` round-trip it like any other flange.

**Shaped leg (`profile`).** The outline of the flange as a closed loop of lines and arcs in `(v, h)`: `v` along the flange (from the
span's start: the start of the span given by `start` / `from`, else the start of the edge; with `from` after `to`, `v` runs the other
way), `h` outward from the height reference line (`heightFrom`: the outer virtual sharp by default), so a rectangle `[0, w] x [0, H]` is
the flange `height: H`, width `w`. `height` is the largest `h` of the profile. The part with `h` below the end of the bend (the
setback) is the bend, not the leg: the leg is the profile beyond it (clipped at that line), and the bend is as wide as the span, else as
wide as the profile at the end of the bend (so a `length` is not needed). Lines are `{"line":[[v,h],[v,h]]}`, arcs
`{"arc":{"center":[v,h],"from":[v,h],"to":[v,h],"ccw":bool}}` (less than a full turn; each segment starts where the one before ends),
or `{"sketch":"S2"}`: the outer loop of the sketch's region, drawn anywhere in the plane (the lower left corner of its box is the origin
`(v, h) = (0, 0)`); the sketch is followed when it changes. Arcs are true cylinder faces in the solid and sampled arcs (at most
5.6 degrees a chord) in the flat pattern and the DXF. `then` is not allowed with a profile (chain flanges on the leg's edges).
Holes in the leg: `holes`, a list of `{"circle":{"center":[v,h],"radius":r}}` (a true cylinder face) or closed loops of segments
(faceted), in the same `(v, h)`; they must lie beyond the end of the bend (`sheetMetalUnsupported` otherwise). With a `{"sketch"}`
profile the inner loops of the region are the holes.
Errors: `invalidValue` for a profile that is not closed or has no edge on the end of the bend.

**Base outlines with arcs.** A base flange whose outline has arcs now takes flanges on its straight edges (and partial or shaped ones):
the arcs are carried as arcs (tagged `ra<curve>`) through the panel tree. Picking an arc edge is refused as before.

Limits: flanges on a curved edge; a profile that crosses itself; `then` with a profile; relief shapes other than the rectangle for partial
flanges; a partial flange next to a flange of the same feature keeps no `gap` (give the span the clearance).


# Mirror of a sheet

`mirror` with `body: true` on a part that is a live sheet (the last feature was a sheet feature) keeps the part a sheet. A part holds
one sheet, so the result is one panel tree; code: `sheet_features::mirror_sheet`, `Tree::reflected`, `Tree::joined_mirror`. Tests:
`crates/ck-model/tests/sheet_mirror.rs`.

* `"replace": true`: the image takes the place of the sheet. The tree is the reflected tree (panels, bends, K-factor, thickness,
  rule, cut-outs; the layout frame is reflected and its flat y is negated, so a bend keeps its side and inside / outside), the flat
  pattern and bend table are the same, and it unfolds. Any plane: a standard plane, a plane feature, a flat face of the sheet.
  Faces keep their roles (`P0.front`, `B1.back`, ...).
* otherwise the image is joined to the sheet. That is one sheet only when the plane is square to the base plate and runs along one
  straight edge of it that carries no bend, and nothing else of the sheet touches the plane (the folded solid has exactly the four
  corners of the end face on it). The two plates become one panel, the bends and panels of the image hang on it, and the joined sheet
  unfolds as a whole (flat pattern twice as long for the L-bracket mirrored about its far end).
* every other plane (a face of the sheet, parallel to the plate, tilted, through a bend, across the sheet, a flange reaching the plane)
  is refused with `sheetMetalUnsupported {detail, hint}`: the two halves would not be one sheet (double wall, overlap, apart), and the
  part is unchanged. The hint points to `replace` or to a plane through a bend-free end.
* a part that is no longer the live sheet (another feature changed the body, or it is unfolded) mirrors as a solid, as before.
  `replace` on a plain solid replaces the body by its image (faces keep their names).

