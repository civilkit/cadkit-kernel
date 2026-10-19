# Requests from CivilKit

Append only. Each item: what CivilKit saw, a repro part, what it expected, and what CivilKit does meanwhile.
CivilKit pins the kernel by commit (`packages/kernel3d/VERSION` in CivilKit).

## 1. Overlapping circles in one cut sketch remove the wrong volume, with no error and a clean validate

Kernel commit: d3b11b936b028885f59c8ebf5b9ae99fcd93d016 (release wasm, sha256 7fa13c35...).
Found by: the CivilKit gusset pilot. A truss panel-point gusset puts two M20 bolts (22 mm holes) 21.2 mm apart, so
their holes overlap by 0.8 mm.

Repro (10 mm plate, two r = 11 circles whose centres are 21.18 mm apart, in ONE sketch, cut throughBoth):

```json
{"format":"ck-part","version":1,"features":[
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-300,"y":-150},{"id":"p2","x":300,"y":-150},{"id":"p3","x":300,"y":300},{"id":"p4","x":-300,"y":300}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
 {"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},"points":[{"id":"c1","x":185,"y":74.55},{"id":"c2","x":165.469,"y":82.7345}],
  "entities":[{"type":"circle","id":"h1","c":"c1","r":11},{"type":"circle","id":"h2","c":"c2","r":11}],"constraints":[]},
 {"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth","depth":10}]}
```

- Expected: the cut removes the union of the two discs x 10 = 7569.796 mm^3 (2 pi r^2 less the 3.29 mm^2 lens, x 10).
- Got: `regen` errors `[]`, `validate` `{ok: true, issues: []}`, but `massProps` volume is only 2501.36 mm^3 less than
  the plate without the cut. The same two circles as two separate sketch + cut features remove exactly 7569.796.
- Probably the sketch region builder: the profile of two crossing circles is not their union (nor an even-odd split
  that only drops the lens, which would remove 7536.9).
- The part is still reported valid, so a caller cannot tell. Either cut the union or return a per-feature error
  (for example `sketchRegionsOverlap`) so the regen says it did not do what was asked.

Meanwhile CivilKit cuts each hole that overlaps another with its own sketch and cut (`packages/kernel3d/plates.js`,
`platePart`), checks every plate's volume against the closed form, and reports the overlap to the user as a detailing
defect. Two bolts closer than a hole diameter is a defect in CivilKit's own layout, separately tracked there.

### Item 1: resolution

Already fixed on main; no kernel change was needed. Bisected (first-parent) from the reported commit: the
repro removes 2501.36 mm^3 at d3b11b9 and the exact union (7569.796) from merge 7145139 on, which brought in
13d83b3 ("coincident tool faces": a kept pair of fragments on one surface facing opposite ways with the same
bounding edges, a membrane inside an operand, cancels). The two crescents and the lens that the arrangement of
crossing circles produces were swept as touching lumps whose shared walls survived the boolean as membranes.

Semantics (unchanged, now pinned by tests): with no `regions` list an extrude / cut sweeps every enclosed region of
the sketch, so crossing loops give the union of their discs / outlines; a circle (or loop) nested inside another
stays a hole of it (even-odd, ring or pocket); an explicit `regions` list still picks individual regions (the
lens, a crescent). Tests: `crates/ck-model/tests/overlap_regions.rs` (the repro, three crossing circles, a
chain, tangent circles, a circle crossing the plate edge, add and cut, nested circles, rectangles + circle,
a seeded sweep of random 2-4 circle clusters against a numeric union area); corpus case
`tools/cases/overlap_regions/crossing_circles_cut.json`. If you still see the wrong volume, pin the kernel at or
after 7145139 (tests landed in 0cbeb65 on branch `sketch-overlap-regions`).
