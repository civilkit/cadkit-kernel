# In-place blend corner: prototype findings

Status: prototype, `crates/ck-ops/src/blend/inplace.rs` (536 lines, 448 without comments and blanks), off by default
(`CK_BLEND_INPLACE=1`). Tests: `crates/ck-model/tests/blend_inplace_proto.rs`, the table
`crates/ck-model/tests/blend_table.rs`.

## The question

Every blend of the engine is a swept tool solid applied by boolean. Where a blend ends against another blend (a vertical
fillet that runs into the rim fillet of the top outline) the tool has to be clipped and the boolean has to resolve
contacts that are tangent by construction; the failures recorded in the comparison runs (`foot 1`, `normal 1`,
`centre did not converge`, flat mitre facets) are of this kind. The prototype builds one such corner without a tool, on
the topology edit layer of `ck-topo`, to see what it costs and whether it is clean.

## The case

A concave vertical edge `E` between two walls whose top outline was rounded first (radius `rt`). The rim fillet leaves a
4-valent vertex `Vt` at the top of `E` (edges: `E`, the two wall / rim edges, the mitre `M` of the two rim faces) and a
3-valent vertex `Vb` at the bottom (`E` and the two wall / bottom edges). The vertical fillet of radius `r` then:

* is a cylinder `F` (axis at distance `r` from both walls),
* ends at the top on a torus `T` (major radius `r + rt`, minor `rt`, same axis): the rim ball turns the corner,
* touches everything along circles and lines that are known in closed form: the two vertical contact lines, the bottom
  arc, the arc between `F` and `T`, the two quarter circles between `T` and the rim cylinders, and the arc of radius
  `r + rt` on the top plane.

Construction, with no intersection routine:

1. recognise the pattern from the topology around `E` (faces, edge counts at both ends, plane / cylinder kinds);
2. compute the axis and the six contact points / the arcs;
3. move the shared edges (bottom edges, wall / rim edges, rim / top edges) to their new end vertices, which trims both
   faces each edge bounds at once;
4. replace `E` and `M` in the loops of the faces they bounded by the new edges, insert the two arcs into the bottom and
   top planes, remove `E` and `M`;
5. create `F` and `T` and chain their loops against the coedges the neighbours already have.

## Results

* Bracket (two perpendicular planar walls, rim cylinders): with `CK_BLEND_INPLACE=1` the three fixtures
  `l-bracket-rim-then-inner-r1/2/3` flip from `expected_fail` to `pass`; the other 155 table rows are unchanged.
* The body validates (`ck_check::validate`: no issues), the mesh is closed (`check_mesh_why`), 16 faces like the same shape
  built the other way round, and the volume agrees with that other order (corner first, then the whole top outline as one
  tangent chain, built by the tool + boolean engine) to better than 1e-6 relative.
* Sweep: 4 rim radii (0.25 .. 2) x 8 corner radii (0.25 .. 8) = 32 pairs agree with the other order (relative 1e-5); a
  bracket turned by 30 degrees and moved agrees as well. Nothing was refused in the sweep.
* The engine's own result for the same sequence (rim first, corner second) is valid but wrong: it keeps the flat mitre
  and is 0.33 / 1.06 / 2.2 mm3 off for r = 1 / 2 / 3 (about 0.005 to 0.04 percent of the body) with no error. That is the
  F8-style failure in a case where it does not even refuse.
* Edit layer: `ck-topo` offers `split_edge`, `split_face`, `delete_face`, `merge_*`, `gc`. None of them moves the end of a
  shared edge or replaces one edge of a loop by a chain, so the prototype carries four local helpers (`retarget`,
  `replace_in_loop`, `insert_arc`, `stitch_face`, about 110 lines) on top of the raw arena fields. The new faces need no
  pcurves (`None`); validation, mass properties and the tessellator all cope. Promoting the helpers to `ck-topo::edit` with
  tests is a day of work.
* Size: recognition about 120 lines, geometry 150, surgery 100, helpers 110.

## What did NOT work

* The plane / cylinder variant (a petal wall meeting a hub cylinder, the daisy and F8-style case) is written (axis from
  `|a - plane| = r`, `|a - hub axis| = R + r`; the contact points and circles follow the same formulas) but UNTESTED: no
  body the engine builds reaches it.
  * The rim fillet of a plane / hub concave corner is itself wrong in the engine: the hub rim torus runs past the wall into
    the material and is cut by the wall's rim cylinder along a spline mitre, and the tool's side faces stay behind as extra
    cylinder slivers (`...S.l2~k`). The top vertex of the vertical edge then has 5 to 8 incident edges instead of 4.
  * On the petal sample the same vertex has 8 edges.
  * The daisy rim sets (`T`, `VT`) are refused or invalid in the engine for N = 5 .. 8, so there is no clean pre-state for
    the daisy fixtures. The V-set failures (`normal 1`) are before any rim fillet and are not touched by this prototype.
  * So an in-place corner patch does not remove the need to fix the rim fillet at concave corners; it needs that pre-state
    (clean mitre, no leftover tool faces, cocylindrical faces merged).
* Only a single edge, perpendicular walls, a plane bottom and a plane top are recognised. The circle formulas for walls at
  other angles are the same up to the axis offset (`(n1 + n2) r / (1 + n1.n2)`), but were not exercised.

## Effort to generalise

| item | estimate |
|---|---|
| promote the four edit helpers to `ck-topo::edit` with unit tests | 1 day |
| clean pre-state: rim fillet at a concave corner without leftover tool faces, cocylindrical / cotoroidal merge in `simplify` | 1 to 2 weeks (separate defect, needed by every follow-on blend) |
| corner patches for the end conditions that matter: concave vertical edge x {plane, cylinder wall} x {rim end, plane end}; convex vertical edge x rim end; non-perpendicular walls; unequal radii | 2 to 3 weeks |
| chains: several vertical edges around one hub (the daisy), both ends of an edge on a rim, closed loops | 1 to 2 weeks |
| naming and the original numbering of the new faces and edges (`blend/layout.rs`), pcurves for the exporter | 1 week |

## Recommendation

Do not replace the tool + boolean engine. It builds the stripes (the blend face along an edge) well, with exact
analytic faces, and all passing corpus cases rest on it. Move the END and CORNER handling in place, one recognised
pattern at a time:

1. keep the swept-tool build for the stripe, but end it at the contact lines instead of letting it overrun and clip;
2. build the corner or end patch (torus, sphere, mitre) from the closed-form contact curves with the edit layer, as in
   this prototype, where the pre-state is clean;
3. a pattern the recogniser does not know falls back to the present path, and every result passes the validity gate
   (`blend/validity.rs`) and the fixture table, which now shows each pattern flipping from `expected_fail` to `pass`.

Reasons: the recorded failures sit where tangent contacts meet (the boolean cannot resolve them, the tools carry
heuristic overruns and clips), whereas the in-place construction never intersects: every contact is a circle or a line
given by the radii. The prototype got a valid, closed, volume-exact result at the first compile and held it over a
parameter sweep. The cost is the pre-state: patches need clean neighbours, so the rim fillet at concave corners (and the
merge of the leftover cocylindrical faces) has to be fixed first or in step; that is also what blocks the plane / hub case.
