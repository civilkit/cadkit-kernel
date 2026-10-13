# BenchCAD findings (30 parts)

Source: Hugging Face `BenchCAD/BenchCAD`, config `code_gen`; dataset card says
cc-by-4.0. Ground truth from CadQuery 2.8.0 (volume, area, centre of mass,
bbox, STEP). Parameters translated by hand; no CadQuery source copied.
Run: `node tools/bench/run.mjs`. Gate: `bench.benchcad` 29/30 (soft; see the per-batch figures below, now 46/50 regen),
`bench.benchcad.step` 27/30 (soft). Figures as of 2026-10-12 after the
boolean-robustness fixes (i_beam, bellows, eyebolt, dome_cap now regen) and
the curved/tangent chamfer fix (tapered_boss, u_channel).

Failure causes: kernel-gap = feature or operation missing; kernel-bug = exists
but wrong; ground-truth-mismatch = the reference itself differs from the
intended geometry. No remaining failure is a translation mistake.

## Passing (regen)

First 10: round_flange, pulley, washer, fan_shroud, enclosure, heat_sink,
nozzle, worm_screw, twisted_drill.
Next 20: hex_nut, bolt, spur_gear, bevel_gear, spline_hub, venturi_tube,
t_pipe_fitting, dovetail_slide, sheet_metal_tray.
Boolean-robustness batch: i_beam, bellows, eyebolt, dome_cap (see below).
Chamfer batch: tapered_boss, u_channel.
Multi-piece sweep batch: pipe_elbow, duct_elbow.

## Failing

| part | cause | detail |
|---|---|---|
| pan_head_screw | ground-truth-mismatch | dataset thread removes 10.08 mm3; the geometric groove removes 24.07 |

Also: t_pipe_fitting regenerates but its ground-truth STEP fails `validate`
(`EDGE_OFF_SURFACE`), a reference-geometry issue, not ours.

## Ranked kernel gaps (parts blocked)

1. Boolean robustness: done for this set (was 4: i_beam, dome_cap, bellows, eyebolt). Root causes fixed: marching SSI closed small loops prematurely and shadowed seeds (i_beam, dome_cap), near-axis revolve arcs gave off-axis pcurves/poles, point-in-face sampled big circles with 24 chords (bellows), marched curves stopped short of loft/domain edges, torus fragments crossing both seams did not tessellate (eyebolt).
2. Chamfer across curved or tangent edges: done for this set (was 2: tapered_boss, u_channel). Root causes fixed: the blend code only took straight edges between planes and true `Circle` edges between analytic coaxial faces, so a spline rim of a lofted cone and a G1 plane/fillet-cylinder edge were `blendUnsupported`; split rims, tangent edges and rational-spline classification (winding-number fallback, 16 s) now handled. Remaining: tangent/curved *fillets* and concave tangent chamfers still `blendUnsupported`; a chamfered split ring yields one blend face (B0), not one per arc.
3. Multi-piece sweep path: done (was 2: pipe_elbow, duct_elbow). Each piece is swept with the section carried along, tangent joints unite in coincident caps, line-line corners are mitred (overshoot + bisector half-space clip); faces named `F.S.<tag>~k` per piece. Remaining: mitre joints involving arcs, twist, multi-region profiles on multi-piece paths.
4. Multi-body results and multi-solid STEP import: done (was 2: torus_link, torsion_spring). A body already holds one lump per solid (wire: `faces[].lump`; mass, validate, tessellate and STEP export are per lump), so no representation change was needed. Root causes: (a) torus_link: the boolean's untouched torus fragment has a loop of two seam twins and `assemble::build` shifted the twin pcurves by the u period for both seams, so the torus face got zero area (volume 2011 instead of 102954: it looked like a dropped solid); (b) torsion_spring/coil_spring STEP: a B-spline sweep face closed on itself (no period on `Surface::BSpline`) got both seam sides at the same u and its circle edges projected across the closing isoline, so the face had zero area and no mesh; (c) eyebolt STEP: a torus fragment bounded by one seam and one real edge failed the universal-cover tessellation fallback on a 1e-8 uv mismatch between independently sampled edge ends. `ck-check` now judges each lump on its own (`VOLUME_NONPOSITIVE` / `SOLID_MULTIPLE_OUTERS` name `lump#N`).
5. Helical sweep performance and hangs: torsion_spring done (regen 3 s, mass 1 s, validate 1 s, STEP import 7 s in release wasm; was 500 s). Root causes: (a) `BSplineSurface::closest_param` costs O(spans) per call (577 spans) and the SSI marcher seeds ~1250 closest points per face pair, so the exact grid search got a control-net-box lower bound, a bounded variant (`closest_param_within`: far points cost one box pass), a coarser per-cell grid on long surfaces and a Newton restart across the closing isoline / multi-start on overlapping sheets (it returned the wrong sheet or the seam for points just across `u = 2 pi`: the coil ground truth lost 9% volume with a coarser seed); (b) the boolean's local uv projection rejected the edges of an interpolated screw face (they are 2e-5 off it) and fell back to the global search; (c) `imprint` marched the whole 12-turn sweep for a leg's cap plane (24 loops, one per half turn): SSI now runs in the overlap of the two face boxes when one face is a long spline; (d) `region::classify` integrated the winding number over the sweep (4 s per face) although the nearest-feature test is exact at any distance: bodies with a long spline face trust it; (e) `face_integral_poly`: volume integrals of polynomial integrands use the Gauss points the degrees need (one panel per span, 5 points across the section) instead of two GL8 panels; (f) tessellation re-tested every triangle on every refinement pass (35 s for the 100k-triangle coil ground truth, now 4 s, same triangles), and rational surfaces evaluated through allocating generic code. The 0.11% volume difference to the dataset is the dataset's: its coil faces lie 0 to +4.2 um outside the ideal screw tube (`torsion_spring_gt_coil_vs_ideal_tube`), ours is within 3e-5 of it and equals A 2 pi R N (N = height / pitch = 11.9977, not 12); the bench carries a documented `tolerances` entry (volume 1.5e-3) for that part.
6. Rotated workplanes: done. All plane modes are implemented and diffed against the original with `node tools/plane_cmp.mjs` (157 cases, 8 documented deviations in tools/plane_known.json): angle/perpendicular/mid, parallel, threePoints, linePoint, normal, normalCurve, curve, tangent (cylinder and sphere; to a point, plane or line), axis refs (world axes, edges, sketch lines, `axis` features of two planes / an edge / a sketch line / a cylindrical face). Remaining deviations: `curve` over several straight edges (frame orientation), `curvesNotPlanar` dev value, tangent-to-line origin on cylinders, cone/torus tangents (planeNotTangent).
7. 3D spline sweep path: done (was 1: coil_spring). `sweep` accepts `{kind:"sketch"}` naming a `sketch3d` with one `spline` entity (natural cubic spline, chord-length parameter, end tangents from `tangent` constraints to lines: the client's `wt` / `$n`); the section is carried by a rotation-minimizing frame (probed against the original with a swept rectangle), side faces are interpolated B-spline surfaces over rows of the motion. coil_spring is now translated as one such sweep (24 points per turn of the variable-pitch helix) + two flat cuts: regen 7 s CPU incl. display tessellation in release wasm, volume 89306.5 (an independent clipped-mesh integral and a quadrature of the ideal screw give 89306 / 89244 + the tilt of the non-normal section) vs the dataset's 89283.7 (+0.026%). Root causes of the cut-end trouble: (a) the cut curve of a plane across the tube turns through a ~0.15 mm hairpin at the lens tip and its uniform-in-t pcurve fit was 1.3 mm off there (`pcurve_of` now also samples every knot span of a B-spline curve and `assemble` refines it with `pcurve_of_dense`); that made the display tessellation chase a mismatch to its 280k-point budget (2.3M triangles, now 109k); (b) the boundary integral of a cut face needed one O(spans) inner integral per boundary sample (9 s): `face_integral` now tabulates the cumulative integrals per v span once many boundary samples sit at distinct v, and takes up to 4096 panels along a dense B-spline pcurve (256 under-resolved the hairpin: +0.13% volume).
8. Twist-extrude and helical-sweep side faces as exact `Surface::Screw` patches (twisted_drill 13-16x -> ~4.5x of the original in timing.mjs; regen+massProps 1.3-1.5 s -> ~0.4 s): the side faces were interpolated B-spline surfaces (48 rows per turn), now `Screw { profile, axis, rise per radian, u range, v range }` = `o + R(u)(c(v) - o) + rise u axis` (kind string stays `spline`; same uv parametrisation, pcurves, rails and names). Closed-form evaluation / derivatives, a sampled-then-Newton closest point, exact coaxial intersections with cylinders / cones / planes (`screw_rev`), a cell-bounded line intersection (`line_screw`) for ray parity. Face integrals tabulate the cumulative u integral at pi/8 cuts for Chebyshev v nodes (24 for volume-type integrands, 56 for the area element, whose singularities lie close to the real axis on strongly twisted faces; the cap edges are integrated adaptively), and a cylinder / cone face with a dense spline boundary (cut by flutes) does the same. Side effect: the volumes and areas of twisted solids are now exact (twistBox 8000.000000, twistBoxCentre area 3685.339122, twisted_drill volume 263.656826 vs 263.656605 before: the interpolated surface was 8e-7 off); a global-search closest point or winding number over a 3-turn face was the 190 s cliff in the first version (ray parity needs `line_params`, windowed SSI needs `is_long_sweep`).
9. STEP-import-only failures: t_pipe_fitting, pan_head_screw,
   dome_cap (sphere pole boundary).

## Notes

- Item 6 is a feature-semantics bug that also affects interactive use.
- Re-rank after each fix; add the next batch of parts only when this list
  is short.

---

## Batch 3 (a-m)

10 more BenchCAD parts from families a to m (`"batch": "3a"` in the manifest): bucket, ball_knob, clevis, hinge,
j_hook, gridfinity_bin, cam, motor_end_cap, cable_routing_panel, helical_gear. Authoring script `gen3.mjs`
(+ `gen3_data.json` outline coordinates, `gen3_blends.json` probed edge refs); `probe_all.sh` probes them,
`mcp_check.mjs` rebuilds them through the MCP server (open, one `edit` batch, `check` volume/bbox/valid).
Run: `node tools/bench/run.mjs` gives 37/40 regen, 36/40 step import.

The `code_gen` config carries the CadQuery program and images only (no STEP, no metrics), so the reference
volume / area / centre of mass / bbox / STEP were produced by running the dataset program in CadQuery 2.8.0
(OCCT 7.9). Nothing here was derived from our own importer.

| part | tier | regen | step import | cause |
|---|---|---|---|---|
| bucket_003137 | easy | pass | pass | |
| ball_knob_007575 | medium | pass | FAIL | kernel-bug: imported full sphere face has zero signed volume |
| clevis_014687 | hard | pass | pass | |
| hinge_000079 | medium | pass | pass | |
| j_hook_001383 | hard | FAIL | FAIL | kernel-bug: silent wrong union (coincident circle rim on a cylinder); step: box misses the torus |
| gridfinity_bin_000166 | medium | pass | pass | |
| cam_012392 | hard | pass | pass | |
| motor_end_cap_000242 | hard | pass | FAIL | kernel-bug (step only): SHELL_ORIENTATION at the outer rim fillet |
| cable_routing_panel_004961 | hard | FAIL | pass | kernel-bug: fillet after chamfer, `blendFailed` at one of four symmetric corners |
| helical_gear_015542 | hard | pass | pass | |

Translation mistake found and fixed on the way (not a failure): j_hook's base boss. The dataset's polyline
drops the `moveTo` start vertex, so the boss is a pentagon, not a hexagon; with the hexagon the volume was
319 mm3 high (also in the nudged-radius control run), with the pentagon it matches CadQuery.

### Ranked gaps from batch 3

B3-1. FIXED. Silent wrong union (j_hook, worst kind: no error, validate clean). A cylinder (r 6) unioned with a
revolved 180 degree handle of the same radius whose start-cap rim lies exactly on the cylinder gave 14938.2 mm3 where
CadQuery gives 22782.6. Root cause (assemble.rs `select_with`, merge of kept fragments): the rod's cylinder band above
the rim was kept in three fragments (two outer wedges and the lower part) joined by cancelling shared edges in listing
order; the seam of the band (the two wedges meet only across it) was cancelled when it was listed before the cut edges that
join the wedges to the lower part, which left a dangling slit plus a second loop: lateral area 0, closed and manifold, so
validate was clean. Seam edges are now decided after every other shared edge. Tests: `hook_union_is_right` in
crates/ck-ops/tests/boolean_sanity.rs (22782.6 for the handle going up; 21176.2 for the handle going down, against a Monte
Carlo overlap of 1944.2). Also caught by the new sanity check when the merge is bypassed (below).

B3-2. FIXED (error), the part still fails on volume. Fillet ending on chamfer faces (cable_routing_panel): the four `|Z`
corner fillets (r 0.83) after a 0.95 top-rim chamfer were `blendFailed` for one of four symmetric corners and erratic in the
parameters. Root cause: `blend::build::apply_tool_hooked` ran a single imprint/select/build at the model tolerance; at that
corner the marched cut curve of the fillet cylinder with the mitred chamfer faces ended 1.5e-6 short of the existing vertex
(tolerance 1e-6), the arrangement stayed open (`OpenEdge`), and unlike `boolean::boolean` the blend path had no retry. It now
retries at 10x and 100x the welding tolerance. Volumes after F4 match the closed forms (73172.65 after the chamfer, 73170.93
after the fillets); test `corner_fillets_after_a_rim_chamfer` in crates/ck-model/tests/blend_retry.rs. The part's final
volume is 1.68% under the reference because the reference geometry differs (66 faces against 54), see the manifest.

B3-3. STEP import of primitives from OCCT (items 9 family): a full sphere face (ball_knob: lump volume 0,
`VOLUME_NONPOSITIVE`; same as dome_cap's pole case); a body with an outer-rim fillet torus (motor_end_cap:
`SHELL_ORIENTATION` on cylinder|torus and torus|plane edges, volume 8.7% low); a partial torus (j_hook
handle, volume and area right but the mesh-derived box ignores it). All three regen exactly from our own
translations; only reading the reference STEP is wrong.

Passing but notable: ball_knob (stem touches the sphere at one point: two lumps) and hinge (knuckles tangent
to the leaf along a line: three lumps) reproduce CadQuery's multi-solid results exactly; clevis, whose cross
hole sits beyond the block edge, reproduces the dataset's sliver cut exactly; helical_gear (twisted loft
of two 252-vertex outlines) matches in volume to 1e-12.

---

## brepjs mechanical

25 of the 41 examples in the brepjs playground category "mechanical" (https://brepjs.dev/playground/examples; source
`apps/playground/src/lib/examples/mechanical.ts` in https://github.com/andymai/brepjs, Apache-2.0), translated by hand into
our feature tree (`gen4.mjs` + `lib4.mjs`, parts and `manifest.json` in `tools/bench/brepjs/`). The playground shows no
difficulty; the tier in the manifest is a proxy by example source length. Reference figures (volume, area, bbox, centre
of mass, STEP) come from running the unmodified examples with brepjs 20.5.0 + occt-wasm 5.6.0 (`tools/bench/brepjs_ref/`);
all 41 examples ran. The 16 not translated are mostly many-solid assemblies (gears, vise, chuck, truss, enclosures).
Run: `node tools/bench/run.mjs --set brepjs` gives 20/25 regen, 22/25 step import (was 18/25 before BJ-1 and BJ-2 were fixed);
`node tools/bench/mcp_check.mjs --set brepjs` builds each part through the MCP server (edit batch, then `check`
volume / area / bbox / centre of mass / valid at 1e-3): 19 parts pass (rubber-foot too: its MCP bbox is exact), 5 are rejected
by `edit` (pipe-saddle, conduit-snap-clip, conical-pour-funnel, knuckle-hinge, modular-hose-segment), dovetail-joint fails
volume and area by 0.18%.

Translation notes. Cut extrudes run against the plane normal by default (a cut needs `flip` to go with it); the example
cutters that touch the body in a single corner point only (l-angle bracket heel, v-groove rail lead-ins) are no-ops in brepjs
too and are not translated; ruled lofts become one two-section loft per stretch; rounded boxes are rounded-rectangle
sketches. No translation mistake is left open.

| part | regen | step import | cause |
|---|---|---|---|
| o-ring, fluted-knob, flanged-tee, gridfinity-bin, fan-guard, gt2-pulley, nema-stepper, linear-bearing-block, tripod-rc2-plate, l-angle-gusset-bracket, hobby-rc-servo, button-top-battery-cell, axial-fan, finger-tab-split-joint, v-groove-slider-rail, vented-louvre-case, wall-mount-flanged-junction-box | pass | pass | |
| rotary-potentiometer | pass | FAIL | kernel-gap: STEP import of HYPERBOLA curves |
| rubber-foot | FAIL | FAIL | regen: kernel-bug, mesh-derived bbox 0.044 mm short (volume, area, com exact; MCP bbox passes); step: SHELL_ORIENTATION on the fillet torus |
| pipe-saddle-p-clamp | FAIL | pass | kernel-gap: fillet of a plane/cylinder line edge |
| conduit-snap-clip | FAIL | FAIL | kernel-gap (same fillet) + kernel-bug BJ-8 (its hole is tangent to the plate's side wall; the coincident countersink circle itself is fixed); step: HYPERBOLA |
| conical-pour-funnel | pass | pass | (BJ-1 fixed) |
| knuckle-hinge | FAIL | pass | reference-mismatch only now (BJ-1 fixed): the example's two leaves overlap, sum 5146.71, union 4990.04; our volume is 4990.04 |
| dovetail-joint | FAIL | pass | kernel-gap: no body-level intersect; male half exact (12415.619), female taper missing, 0.18% volume |
| modular-hose-segment | pass | pass | (BJ-2 fixed) |

### Ranked gaps from the brepjs batch

BJ-1. FIXED. Coincident circle on a cylinder wall (6 features in 3 parts: conduit-snap-clip, knuckle-hinge,
conical-pour-funnel; same family as B3-1). A countersink cone cut whose base circle has exactly the hole radius was
`booleanFailed`. Root cause (imprint.rs): the circle is an existing edge of the tool (shared by its cone and base disc)
lying in the interior of the hole's cylinder face. The SSI trims it at the cylinder's relocated seam, but because the cut
equals an existing edge it keeps that edge's own start instead of the relocated start, and the seam crossing was only
added to the vertex pool when the start moved (`p.dist(start_pt) > tol`, true for a seam at the closed-circle start). So
the circle was never split at the seam: in the cylinder's uv domain it was a closed non-contractible curve with no tie to
the seam, walked as two zero-area cycles, one fragment holding both sides of the cylinder, and the result was open and
non-manifold; the 10x / 100x retries cannot help because the geometry, not the tolerance, is wrong. Fix: when the
whole closed cut coincides with an existing edge, every interior break on a boundary edge stays a split point. Tests:
`countersink_base_on_hole_wall_*` in crates/ck-ops/tests/boolean_robust.rs (volume against the closed-form frustum and
cylinder volumes, both cut orders).

BJ-2. FIXED. Boolean with a sphere pole inside the other solid (modular-hose-segment, cascade of 7 errors). Two root
causes. (a) classify.rs `close_pole_spikes`: a loop that goes up the seam to the pole and back (the cap around a pole
that a cut circle encloses) is closed into a strip only when the polygon's u advance equals one period, but the advance
was read from the last sample, which stops one chord short of the loop end whenever the loop ENDS on the circle piece
(seam up, seam down, circle) rather than on the seam; the cap then had zero uv area and no sample point, so the union
failed. The advance now uses the loop's true end point. (b) apply.rs `relocate_seams`: moving the circle starts of a
revolved cone / cylinder tool whose circles both lie in the discarded region did not turn the radial edge of its disc caps,
so the cap polygon lost a wedge and point classification against the tool read a point outside it as inside (the
frustum cut of the hose collar failed). Tests: `ball_union_cylinder_*` (axis, off-axis x = 0.5, ending below the pole;
volume against a quadrature of the overlap) and `collar_cut_by_a_frustum_whose_circles_lie_in_a_pocket`.

BJ-3. Fillet of a straight edge between a plane and a cylinder whose axis is parallel to the plane (pipe-saddle-p-clamp
concave foot/tube edge, conduit-snap-clip lip). `blendUnsupported` at r 2, 0.5 and 0.05; chamfer of the same edge works.

BJ-4. STEP import: HYPERBOLA curve (rotary-potentiometer, conduit-snap-clip); torus fillet rim from OCCT gives
SHELL_ORIENTATION and 44% low volume (rubber-foot; same as B3-3).

BJ-5. No body-level boolean between two tools (dovetail-joint female cavity = prism minus two rotated slabs). Not
expressible with add/cut features; only `upToNext` intersects, and only against the existing body.

BJ-6. Several separate overlapping solids in one document (knuckle-hinge) cannot be represented: features union into one
body. Not fixable by translation; the figures of that part cannot be matched exactly.

BJ-8. FIXED. Hole tangent to the plate's side wall (conduit-snap-clip F13 at x = -12.7, r 2.3 against the wall at x = -15). The
hole itself was already silently wrong (volume 1417.8 instead of 1373.5: a third of the hole removed) and a further cut that met
the hole cylinder (the countersink cone, F15) was `booleanFailed` with "no sample point for a fragment". Root cause (assemble.rs): the cylinder
touches the wall along a line, which splits its kept fragments; the merge cancelled that line and also the seam, because
the seam test only knew a seam at u = 0 and `relocate_seams` had moved the hole's seam to 0.62 of the period. The cylinder came
out as a seamless two-loop band of lateral area 0. A seam is now an edge traversed both ways by one original boundary loop.
Tests: crates/ck-ops/tests/boolean_tangent.rs (hole and countersink against closed form and a quadrature of the clipped cone, at the
tangent position and 0.001 inside it). conduit-snap-clip still fails on F7 (BJ-3, lip fillet).

BJ-7. `body.box` is mesh-derived and short on curved rims (rubber-foot 0.044 mm), while the MCP `check` bbox matches.
Same quantity, two answers.

### MCP tool problems seen

- `edit` with a failing feature rejects the whole batch and leaves the document empty (atomic by design); the reply lists
  only the code. For `blendUnsupported` the hint is "see the features field table", which names neither the edge
  nor the face types; it should say which surface pair is unsupported and suggest a chamfer or another order.
- The `booleanFailed` hint ("check the tool and target overlap in a volume, not just a face") is wrong for BJ-1 and BJ-2:
  the tool does overlap in a volume there. It should mention tangent/coincident circles and sphere poles as suspects.
- The `check` bbox and `read`/`run.mjs` body box disagree on curved parts (BJ-7); `check` should say which one it uses.
- `mcp_check.mjs` used an absolute `tol: 0.001` on volume (kept for the BenchCAD mode, relative tolerance used for `--set brepjs`).

---

## Batch 4 (n-z)

10 more BenchCAD parts from families n to z (`"batch": "4"` in the manifest): ratchet_sector, shaft_collar, pipe_flange,
piston, pull_handle, vented_panel, waffle_plate, rect_frame, snap_clip, propeller. Rows come from the dataset-server
filter API (config `code_gen`); the tier is the dataset's own difficulty (7 hard, 3 medium; no easy part was drawn).
Authoring script `gen5.mjs` (+ `gen5_blends.json` probed edge refs), translated by hand from the programs
(parameters paraphrased, no CadQuery source copied).

Reference figures: every figure and STEP file in the manifest comes from executing the unmodified dataset program in
CadQuery 2.8.0 (`uv run --no-project --python 3.12 --with cadquery python ...`; a small runner that `exec`s the program, prints
volume, area, centre of mass, bbox, face and solid counts and exports the STEP). All 10 programs ran, each gives one valid
solid. No reference was edited after the fact. Diagnostic CadQuery runs used to classify failures (program without its last
chamfer, fillet per edge group, propeller with a subset of its blade unions) are not references and are not stored.

Run: `node tools/bench/run.mjs` gives 44/50 regen, 43/50 step import. `node tools/bench/mcp_check.mjs <ids>` (batch-4 parts now use the
relative volume / area / bbox / com / valid check): 7 parts pass every check; rect_frame passes volume (3.0e-4) but fails area;
snap_clip and propeller are rejected by `edit`.

| part | tier | regen | step import | our volume vs CadQuery | cause |
|---|---|---|---|---|---|
| ratchet_sector_001018 | hard | pass | pass | 146263.4918 vs 146263.4918 (1.4e-15) | |
| shaft_collar_000553 | hard | pass | pass | 2739.285 vs 2739.320 (1.3e-5) | |
| pipe_flange_000214 | hard | pass | pass | 226543.2305 vs 226543.2305 (4e-16) | |
| piston_002697 | hard | pass | pass | 764271.04 vs 764265.70 (7e-6) | |
| pull_handle_007593 | medium | pass | pass | 178178.2764 vs 178178.2767 (1.4e-9) | |
| vented_panel_003475 | medium | pass | pass | 109757.4456 vs 109757.4456 (8e-16) | |
| waffle_plate_010153 | medium | pass | pass | 69341.4467 vs 69341.4467 (1e-15) | |
| rect_frame_008694 | hard | FAIL (area) | FAIL | 43433.87 vs 43446.95 (3.0e-4); area 28419.0 vs 28324.4 (+0.33%) | kernel-bug: window fillet; step: SHELL_ORIENTATION |
| snap_clip_000052 | hard | FAIL | FAIL | no body (pre-chamfer body 4.381142 = CadQuery pre-chamfer 4.381142) | kernel-bug: chamfer; step: PARABOLA unsupported |
| propeller_000192 | hard | FAIL | FAIL | 20113.64 (one blade) vs 23868.82 | kernel-bug: union of overlapping lofted blades; step: VERTEX_OFF_CURVE |

Nothing here is a translation mistake. Two corrections made on the way, both caught by comparing with CadQuery:
(a) piston's cross hole: the program names a `cutThruAll` from the mid plane; CadQuery cuts both ways (volume only matches
with `throughBoth`: 764271 vs 764266; one way gave 785950); (b) the shaft_collar inner step cylinder lies inside the wall and is
not translated (the volume matches without it). Where a failure was investigated, the translation was shown right by matching
CadQuery on a sub-state of the same part (below), not by editing a reference.

### Ranked gaps from batch 4

B4-1. Silent wrong fillet at 270 degree material corners (rect_frame, no error, validate clean). Filleting the four edges of a
rectangular window in a plate (r 0.5 to 1.5, any radius tried) is `blendFailed` on its own; together with the outer rectangle
and the four hole rims on the same face it succeeds but removes 13.2 mm3 too much (volume 43433.87 vs 43446.95, area +94.6 mm2).
Outer rectangle alone, hole rims alone and their union are identical to CadQuery to 10 digits (43591.530 / 43730.228 / 43567.948).
The window corner has a concave vertical edge where two filleted edges meet.
Not caught by the new sanity check: the error is 13 mm3 of 43434 and the fillet bounds (below) only bound the overlap volume of a boolean, they cannot see a few mm3 error in a
blend; catching it needs the exact expected volume change of each blend (profile area times length plus end terms).

B4-2. FIXED. Chamfer of a curved rim that ends on a side face not exactly perpendicular to it (snap_clip). Three defects, none of them
the end-face angle in the SSI: (1) the contact-point check (`spring leaves its face`) judged a point that runs along an end line
perpendicular to the edge up to the 4-decimal sketch precision (off by 1e-5) outside its face; it is now on the boundary within
2e-5 of the body size (at most a quarter of the blend size). (2) a blend face split by a hole centred on the chamfered line was named
`B2~1`, `B2~2` and the "no blend face in result" test looked for `B2`. (3) one tool chamfers both pieces of a straight edge split by a
hole centred on its line (the margin runs on over the notch), so the second piece was gone and answered `missingEdge`; an edge
whose two faces both border an earlier blend face of the feature is now done. Volume and area match CadQuery to 1e-5 (test
`chamfer_on_the_rims_of_a_split_clip`). Remaining: that margin (`r.max(len * 0.05) + 1.0` along a straight edge) also chamfers
an UNSELECTED collinear edge within reach beyond a notch; not fixed, silent.

B4-3. NOT FIXED. Union of two lofted ellipse blades that overlap near the axis (propeller): `booleanFailed`. Hub exact (18679.4007), hub plus
one blade exact (20113.6368 vs 20113.6370), hub plus blades 180 degrees apart right (21540.32 vs 21540.24), any second blade turned by 90, 100, 120, 135 or 170 degrees `booleanFailed`, with `cpattern` and with a hand-built blade.
What was learned (blade A united with blade B alone fails the same way): the blades are thin plates in the same x slab (thickness 3.4 along x, width 16.8)
crossing like an X. The root of each blade sticks out of the hub (width 8.4 against hub radius 5.29), so the overlap is not hidden in the hub: applying the
instances one by one to the body instead of uniting them first does not help (tried, reverted). The arrangement has the intersection curves of the lateral faces
(two 45 degree diagonals through the point where the two seam lines cross, v11) and the root-plane sections, but at the vertex where blade A's root ellipse meets
curve A0xB0 (the diagonal) and A0xB2 (blade A half with blade B's root plane) the walk's angular order (A0xB0 then A0xB2 clockwise from the root ellipse) contradicts the
order of the other end vertices along the face boundary (the seam point v8 before v11), so the face walk closes one big cycle that uses curve A0xB0 in both directions
(4 faces on an edge: NonManifoldEdge). So one of the pieces is attached to the wrong end: most likely the trimmed piece of the marched spline A0xB2 (`kept (0.862, 1.0)`;
its mirror image A1xB2 is kept empty) or the join of the two A0xB0 branches at the seam crossing. Retries at 10x and 100x the tolerance give the identical arrangement.
Next step: dump the uv polylines of the cut edges on A0 and check that curve 23 and the chain 22-24-25 do not cross; a missing crossing vertex there would be the bug.

B4-4. STEP import (all three failing parts): rect_frame SHELL_ORIENTATION on OCCT fillet faces (volume +0.33%, area -1.18%, same family
as B3-3 / BJ-4), snap_clip `PARABOLA is not supported` (conic from the chamfer), propeller `VERTEX_OFF_CURVE` (volume -0.23%).

### MCP tool problems seen (batch 4)

- `mcp_check.mjs` used an absolute volume tolerance of 0.001 mm3 in benchcad mode: shaft_collar and piston showed as failing at 1e-5
  relative. Batch-4 parts now go through the relative check; batch 3 keeps the old one.
- A rejected `edit` (snap_clip, propeller) reports only `blendFailed` / `booleanFailed` with no edge or feature pair; for `blendFailed` it
  would help to say which edge failed and whether the end condition or the face pair was the problem (B4-2 took bisecting by hand).
- The guide's frame paragraph ("top = XZ", "local y maps to -z") and gen.mjs's CadQuery mapping describe the same thing differently;
  the CQ-to-ours rule (x,y,z)->(x,z,-y) is only in the bench scripts.
- `normalCurve` planes and `ellipse` sketch entities work but are undocumented in the guide's field table (`p` is the major-axis end point,
  `r` the minor radius; a plane from a line in the top-plane sketch has local x horizontal and local y vertical).

---

## Post-boolean sanity check (2026-10-13)

A boolean can close into a valid, manifold body that is wrong (j_hook, the tangent hole): validate cannot see it. `boolean::sanity`
(crates/ck-ops/src/boolean/sanity.rs) checks every result against what the operands allow, with signed volumes va, vb, vr and
ov = min(va, vb, volume of the overlap of the operands' padded boxes), tolerance 1e-3 of the operands:
union va + vb - ov <= vr <= va + vb; cut va - ov <= vr <= va; intersect 0 <= vr <= ov; a union's box contains both operand boxes.
The simple bound max(va, vb) <= vr alone does NOT catch j_hook (14938 > the rod's 12106); the box-overlap cap does (the dropped handle reads as an
overlap of 8178 against a box overlap of 432 mm3). It is skipped for sheets, open shells and non-positive volumes.
`boolean()` retries a failing result at 10x and 100x the tolerance, else returns it with `BoolOut::mismatch`; the feature layer answers
`booleanVolumeMismatch {what, expected_min, expected_max, got, hint}` (also pattern unions and `upToNext`), leaves the body as it was and
the rest of the tree regenerates; the blend tool boolean treats a mismatch as a failure and retries. No false positive on the gate or on the 80 bench parts
(only j_hook triggered before its root cause was fixed). Limits: only gross loss (a few percent of the smaller operand); a hole that removes 22 of 66 mm3 passes
(BJ-8) and a 13 mm3 error in a blend (B4-1) is far below it. Tests: crates/ck-ops/tests/boolean_sanity.rs, crates/ck-model/tests/boolean_sanity.rs.
The driver hint table (packages/driver/src/verify.ts) still has the old `booleanFailed` text; the new code carries its hint in the error params.
