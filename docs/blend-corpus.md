# Blend corpus: fillets and chamfers against OCCT

`tools/blend_corpus.mjs` runs 60+ fillet and chamfer cases through the wasm and compares each result with the volume OCCT
(`BRepFilletAPI_MakeFillet` / `MakeChamfer`) gives for the same base body and the same edges. Per case the check is: no
feature error, `validate` ok, volume within `1e-5` (planar bodies) or `1e-4` (cylinders, cones, rounded outlines, variable
radius: the radius law is not the same function in OCCT and here, see below) relative to OCCT. Two groups have wider bands,
marked "ok within band" in the table: variable radius at corners (`var_corner*`, `var_ring_top`: 5e-4, a planar mitre and a
slightly different radius law) and unequal radii at a vertex (`*unequal*`: 3e-4, a mitre where OCCT blends a patch). A case with
`expect` must produce that error code.

```
node tools/blend_corpus.mjs [--wasm path] [--only substr] [--json out.json]     # run (exit 1 on any failure)
node tools/blend_corpus.mjs --emit DIR [--wasm path]                            # base bodies as STEP + cases.json
python3.12 tools/blend_reference.py DIR [--only substr]                         # OCCT volumes -> tools/blend_reference.json
node tools/blend_corpus.mjs --edges <base>                                      # edge list of a base body (to author picks)
```

A pick is a point on the edge; both engines take the edge of the base body nearest to it. The base bodies are built by the
kernel, exported with `step`, and read by OCCT, so both start from the same geometry. Where OCCT's chamfer builder fails
(plane / cone edges) the reference is the Pappus integral of the removed triangle (`source: analytic` in the JSON).

## Conventions added with this work

* **Chamfer legs.** `chamferType` `distances` / `distanceDistance` (`distance`, `distance2`) and `angle` / `angleDistance`
  (`distance`, `angle` in degrees). `distance` is measured on the FIRST face: the face named by the feature's `face` (a face
  id, or `{kind:"face", id}`; every pick must border it, else `missingEdge`), else the first of each pick's `faces`. For `angle`
  the angle is between the chamfer and that first face and `distance` lies on it (OCCT: `Add(d1, d2, E, F)` and
  `AddDA(d, angle, E, F)` with `F` = `face`). `flip: true` swaps the legs (the other face becomes the first). Picks that list
  the two faces in different orders and no `face` are `unsupported`; naming `face` makes a ring of edges around one face agree.
  Works on planar, cylindrical and conical neighbours (the existing curved-edge machinery).
* **`propagate: true`** (fillet, chamfer): the selection grows along edges that continue the picked ones tangentially (same
  face on one side, parallel tangent at the shared vertex). OCCT does this for every selected edge; here it is opt-in so that
  existing parts keep their meaning.
* **Variable radius at corners.** `radii: [near, far]` as before (`near` is the end of the edge nearer the pick); a pick may
  carry its own `radii`. Edges that meet at a vertex must have the same radius there, else `variableRadiusMismatch` (with the
  vertex in `detail`). Every blend's tool is built on the input body, then the tools are applied one after the other, so
  edges that meet at a corner end in a mitre; three convex edges with one radius at the vertex get the rolling ball's sphere
  patch `V<k>`. Several edges with one constant radius through the variable path use the constant-radius builder.
* **Unequal radii.** `edges[i].radius` on a constant fillet. Equal radii take the old path; different radii use the same
  tool-per-edge construction as above. Where blend faces of different radii meet along a mitre the result is valid but not
  tangent-continuous: the feature gets a `cornerNotTangent` warning with `deviation` (the largest normal jump in degrees).
* **Mixed vertices** (`blend/mixed.rs`). A vertex whose three edges are all selected, two convex and one concave: the concave
  edge is filleted first (the corner cylinder), then the two rim edges and the arc where the cylinder meets the top face are
  filleted as one tangent chain (the ball rolls round the filled corner: a torus patch of major radius `2r`). This is the
  surface structure OCCT produces and does not depend on the selection order.


## What each fix was

1. **Two-distance and distance-angle chamfers.** The machinery was already there under the names `distances` and `angle`
   (planar, cylindrical and conical neighbours, rings, concave edges); the gap was the names (`distanceDistance`,
   `angleDistance` answered `unsupported`) and the face convention. They are now aliases, `face` names the face the first
   distance is measured on, and `flip` swaps. `face` is resolved per edge (`build::chamfer_legs_on_face`), so a chain whose
   edges list their faces in different orders works. 31 of the 32 chamfer cases match OCCT to 1e-5 (1e-4 on curved bodies) or better, the exception is the 3-edge
   corner below. OCCT's own chamfer builder fails on plane/cone edges (`Build failed`); the three cone cases use the Pappus
   volume of the removed triangle and agree with the kernel to 1e-15.
2. **Chains of tangent edges.** `propagate: true` closes the selection under tangent continuation (a rounded outline picked
   by one edge). Fillets need the tools of tangent neighbours to stop flush at the junction (a straight tool run on past the
   junction would cut outside the arc's band): `TANGENT_FLUSH` in `blend/build.rs`, also used for chamfers.
3. **Variable radius at corners** (`blend/interp.rs`). It failed because the tool of the second edge was built on the body the
   first tool had already cut, and its contact lines were tested against the cut faces (`blendTooLarge` / `blendFailed`).
   Now every tool is built on the input body. The two ruled (oblique-cone) tool surfaces of edges that meet at a vertex touch
   the same plane tangentially at one point, which the marching intersection cannot resolve, so each convex tool is clipped
   by the bisector plane through the shared vertex (`clip_halfspace`) and the cuts meet in a crease; three convex edges with
   one radius at the vertex also get the sphere patch. `variableRadiusMismatch` is reported before any geometry is built.
4. **Mixed vertices** (`blend/mixed.rs`): see above. The old result depended on the selection order (28770.81 or 28775.96 for
   OCCT's 28772.67 at r = 2); now 28772.665457 for every order, and r = 1, 2, 3 agree with OCCT to 1e-15.
5. **Reflex corners of two selected convex edges** (an L-bracket outline, a pocket rim): the two tools used to stop flush
   (fillet: a wedge was left, up to 3.8 mm3) or ran on past the vertex into the other leg (chamfer: `d^3` too much removed).
   OCCT trims both cylinders (planes, for chamfers) at the bisector plane through the vertex; so do we now (`Junc::mitre`).
   `lbr_*` and `pocket_rim_ring_r2` went from 1e-4..8e-4 off to exact.
6. **Unequal radii** (`edges[i].radius`, `build::fillet_unequal`). The radius is per edge through the ordinary builder (analytic
   cylinders, same rounds and junction rules). Two edges at a vertex match OCCT to 3e-9; three edges end in a mitre where
   OCCT blends a patch, so the volume is within 2e-4 (convex), 6e-5 (concave), 5e-5 (mixed) and the feature gets
   `cornerNotTangent {deviation}` (53 degrees for the convex 3/4/5 corner, 28 for the concave 1/2/3 one).

## What remains

* **Equal chamfer at a convex 3-edge corner** (`cube_ch_equal_corner3`): OCCT closes the corner with a triangular face through
  the three points where the chamfer faces meet the face planes (`x + y + z = 2d` for a cube corner) and so removes `d^3 / 12`
  more (5.33 mm3 for d = 4). The kernel keeps the union of the three cuts, the behaviour of the program it was reconstructed
  from; the wire gate compares against it, so this is left as a documented difference.
* **Variable radius** agrees with OCCT to 9e-5 on an isolated edge and 2e-4 .. 4e-4 at corners: the radius law is the same
  linear function but the section surface differs slightly (the kernel's is the pinned ruled cone, 7962.80235 for radii 1 -> 3
  on the 40 mm edge; OCCT gives 7962.09), plus the planar mitre. Conic, chord and curvature profiles at corners are still
  `blendUnsupported`; a variable edge ending at a concave corner is not mitred.
* **Unequal radii**: no blend patch (OCCT builds one); mixed vertices with unequal radii use the mitre.
* **Face-face fillets** (two faces that share no edge) and **conic / curvature fillets at corners**: not started.
* **Chamfers**: only equal chamfers are checked against OCCT at corners; asymmetric chamfers at a 3-edge corner follow the
  union rule above.

## Cases, before and after

"before" is `kernel main` at the start of this work (the wasm built from `496286e`), "after" the branch. Cells: `ok`, or the
signed volume difference to OCCT (relative).

| case | OCCT volume | before | after |
|---|---|---|---|
| `box_ch_equal_edge` | 7920.0000 | ok | ok |
| `box_ch2_top_edge` | 7880.0000 | unsupported | ok |
| `box_ch2_side_edge` | 7880.0000 | unsupported | ok |
| `box_ch2_vertical_edge` | 7970.0000 | unsupported | ok |
| `box_ch2_ring_top` | 7656.0000 | unsupported | ok |
| `box_ch2_ring_bottom` | 7765.3333 | unsupported | ok |
| `box_ch2_two_adjacent` | 7824.0000 | unsupported | ok |
| `box_ch2_two_opposite` | 7880.0000 | unsupported | ok |
| `box_ch2_big` | 7680.0000 | unsupported | ok |
| `box_chA_top_edge` | 7953.8120 | unsupported | ok |
| `box_chA_side_edge` | 7688.2309 | unsupported | ok |
| `box_chA_ring_top` | 7399.3562 | unsupported | ok |
| `box_chA_vertical` | 7980.0000 | unsupported | ok |
| `lbr_ch2_concave_edge` | 6030.0000 | unsupported | ok |
| `lbr_ch2_top_outline` | 5862.6667 | unsupported | ok |
| `lbr_ch_equal_top_outline` | 5847.0000 | diff -3.3750 (5.8e-4) | ok |
| `lbr_fillet_top_outline` | 5882.8908 | diff 0.7670 (1.3e-4) | ok |
| `lbr_ch2_two_edges_reflex` | 5949.3333 | unsupported | ok |
| `cube_ch_equal_corner3` | 26322.6667 | diff 5.3333 (2.0e-4) | diff 5.3333 (2.0e-4) |
| `boss_ch2_top_rim` | 18969.8523 | unsupported | ok |
| `boss_ch2_top_rim_cyl_first` | 18967.7579 | unsupported | ok |
| `boss_chA_top_rim` | 18962.7242 | unsupported | ok |
| `boss_ch2_base_rim` | 19116.0672 | unsupported | ok |
| `holed_ch2_rim` | 14792.4503 | unsupported | ok |
| `holed_chA_rim` | 14798.7304 | unsupported | ok |
| `cone_ch2_top_rim` | 5882.3907 | unsupported | ok |
| `cone_ch2_top_rim_cone_first` | 5882.3907 | unsupported | ok |
| `cone_chA_bottom_rim` | 5729.2248 | unsupported | ok |
| `cone_ch2_bore_rim` | 5874.7783 | unsupported | ok |
| `cone_fillet_top_rim` | 5910.7945 | ok | ok |
| `holed_ch2_bottom_rim` | 14829.2331 | unsupported | ok |
| `rounded_ch_propagate` | 11548.5961 | diff 106.4457 (9.2e-3) | ok |
| `rounded_ch2_propagate` | 11565.4630 | unsupported | ok |
| `rounded_chA_propagate` | 11515.2068 | unsupported | ok |
| `rounded_fillet_propagate` | 11629.3641 | diff 46.4664 (4.0e-3) | ok |
| `box_fillet_edge` | 7965.6637 | ok | ok |
| `box_fillet_corner3` | 26701.6872 | ok | ok |
| `box_fillet_ring_top` | 7900.0590 | ok | ok |
| `lbr_fillet_concave` | 6008.5841 | ok | ok |
| `boss_fillet_base` | 19061.4867 | ok | ok |
| `boss_fillet_top` | 18975.1902 | ok | ok |
| `holed_fillet_rim` | 14849.8070 | ok | ok |
| `var_isolated` | 7962.0949 | ok within band: diff 0.7074 (8.9e-05) | ok within band: diff 0.7074 (8.9e-05) |
| `var_concave_isolated` | 6009.4763 | ok within band: diff -0.1768 (2.9e-05) | ok within band: diff -0.1768 (2.9e-05) |
| `var_corner2` | 7944.8780 | blendFailed | ok within band: diff 1.7754 (2.2e-04) |
| `var_corner2_equal_ends` | 7949.2625 | blendFailed | ok |
| `var_corner3` | 7934.4619 | blendFailed | ok within band: diff 3.0355 (3.8e-04) |
| `var_corner3_equal_ends` | 26830.6924 | blendFailed | ok |
| `var_ring_top` | 7891.2548 | blendTooLarge | ok within band: diff 2.2557 (2.9e-04) |
| `var_two_vertical_far` | 7981.0475 | ok within band: diff 0.3537 (4.4e-05) | ok within band: diff 0.3537 (4.4e-05) |
| `var_corner2_mismatch` | 7942.4593 | expect variableRadiusMismatch, got blendFailed | ok |
| `pocket_mixed_r2` | 28772.6655 | diff -1.8513 (6.4e-5) | ok |
| `pocket_mixed_r1` | 28793.1496 | ok within band: diff -0.2314 (8.0e-06) | ok |
| `pocket_mixed_r3` | 28738.6486 | diff -6.2482 (2.2e-4) | ok |
| `pocket_mixed_r2_reversed_order` | 28772.6655 | diff 3.2991 (1.1e-4) | ok |
| `pocket_mixed_wall_first` | 28772.6655 | diff 3.2991 (1.1e-4) | ok |
| `pocket_rim_ring_r2` | 28728.2596 | diff -3.7994 (1.3e-4) | ok |
| `pocket_walls_concave3_r2` | 28839.8643 | ok | ok |
| `cube_unequal_corner3` | 26687.3950 | diff -149.3236 (5.6e-3) | ok within band: diff 5.3208 (2.0e-04) |
| `pocket_unequal_concave3` | 28835.7954 | diff 52.3927 (1.8e-3) | ok within band: diff -1.7932 (6.2e-05) |
| `pocket_unequal_mixed3` | 28794.5167 | diff -62.1163 (2.2e-3) | ok within band: diff -0.9570 (3.3e-05) |
| `cube_unequal_two_edges` | 26786.4698 | diff -96.3887 (3.6e-3) | ok |

before 14/62 ok, after 61/62 ok
