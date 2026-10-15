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
| cable_routing_panel_004961 | hard | pass (2026-10-15) | pass | was: `blendFailed` (B3-2, fixed), then a 1.68 % volume gap that was a translation mistake: second slot row on y = 0 (B4-6) |
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
| propeller_000192 | hard | pass (2026-10-15, tolerances volume 1.5e-3, area 2e-3) | FAIL | 23892.60 vs 23868.82 (+0.10 %) | FIXED B4-3 (crossing cut curves, sliver edges, sliver classification); the reference's own booleans are imprecise; step: VERTEX_OFF_CURVE |

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

B4-3. FIXED (2026-10-15). Union of lofted ellipse blades that overlap near the axis (propeller) was `booleanFailed`. Three separate root causes, all in the
arrangement of near-degenerate configurations (the blades are thin plates crossing like an X, equal size and symmetric, and the fourth blade touches the second
tangentially at the root tips). They are fixed in a "robust mode" (`boolean::robust()`): the ordinary attempts run first, unchanged, and only when all of them
fail to assemble do three more attempts (10x, 1x, 100x the welding tolerance) run with these steps switched on, so every operation that already worked keeps
its bits and its speed.
 1. Two cut curves of ONE face pair that cross each other in their interiors (the two diagonals of the Steinmetz-like intersection of two equal tubes, meeting at
    the point where the seam lines cross) were two edges with no vertex at the crossing, so the face walk met one edge from four directions (NonManifoldEdge). This
    is the "missing crossing vertex" the first analysis suspected. `imprint::cut_crossings` finds interior crossings of the cut curves of a face pair (polyline
    pre-selection, alternating closest-point refinement, accepted within 2 tol) and adds them to the vertex pool.
 2. The 4-blade union still failed at the root tips of blades 2 and 4: their lateral surfaces are tangent there, so the cut curve of blade 4's lateral face with
    blade 2's root cap runs 7e-4 beside blade 2's root rim over 0.15 mm. That sliver lens (more than 10 tol wide at the 1e-5 rung) left two edges between the same two
    vertices, one used by the cap and one by the lateral face. Edge pieces between the same two vertices that stay within 100 tol are now one edge.
 3. The lens fragment left on the lateral face has a sample point 1e-4 from the other solid's boundary, where single rays disagree (6 rays: F T F T T T, winding
    number 0.5027, i.e. ON the boundary); the first ray won (Out) and the lens was kept on both sides of a cut (non-manifold). A sample within max(100 tol, 5e-4) of a
    face of the other solid now takes the majority of all rays (winding number on a near tie).
Result: hub + four blades 23892.60 (valid, 0 issues) against the reference 23868.82, +0.10 percent volume, +0.12 percent area. The reference itself is imprecise
there: OCCT's union of blades 0 and 90 is 2792.18 but its intersection is 143.52 (va + vb - union = 145.28); a Monte Carlo of the two solids with OCCT's own point
classifier (300k points) gives an overlap of 136.3 +- 1.1, ours is 134.70 (union 2802.77 and intersection 134.70 agree to 1e-7, test
`blade_pair_union_and_intersection_agree`). Each neighbouring pair is ~9 mm3 short in the reference's union, which is the 24 mm3. The manifest therefore carries
`tolerances` for the part (volume 1.5e-3, area 2e-3; `run.mjs` now reads an `area` tolerance too). Other subsets against OCCT: hub 18679.4007 (exact), hub +
blade 0 20113.6368 vs 20113.6370, hub + 0 + 180 21540.32 vs 21540.24; blades only, 4 blades: ours 5336.45 (OCCT's own blades-only 4-blade union returns 4131.0, a lost
blade: its result without the hub is wrong, with the hub it is right). Tests: crates/ck-model/tests/propeller_boolean.rs (two crossed blades, four blades without
and with the hub, union/intersection identity, failure report). STEP import of the reference still shows `VERTEX_OFF_CURVE` (vertices up to 2.8e-2 off their edges:
OCCT tolerances the STEP does not carry), B4-4 unchanged.

B4-5. FIXED. The `booleanFailed` error said nothing about where. It now carries `faces` (the faces bordering the open / non-manifold edges of the assembled
result, up to 8, for instance `F8.S.e1~1@F9.1`), `issue` (`openEdge` / `nonManifoldEdge` / `badPairing`), `at` (a point on the first bad edge) and a `hint`
(`FeatureError::new` parses `booleanFailed;edgeFaces=..;issue=..;at=..`; the `nearCoincident` form with `hostFace` / `toolFace` / `gap` is kept and wins when two
free-form walls nearly coincide). The driver hint (packages/driver/src/verify.ts `FIX.booleanFailed`) names them. Test `failed_union_reports_faces_and_point`.

B4-6. cable_routing_panel was a TRANSLATION mistake, not a reference difference (the earlier label was wrong). The program centres its second row of slots with
`center(0, -22.2)` on a workplane that is already at y = 22.2, so the row lands on y = 0 (over the four holes): CadQuery's reference has 8 slots at y = 22.2 and y = 0,
66 faces; ours cut y = +-22.2 (54 faces). With the row on y = 0 the part passes every check (volume 64258.66, area, centre of mass, bbox, validate; the former gap was
1.68 percent). gen3.mjs fixed, the `known` entry removed, `cable_panel_fillet_after_trapezoid_edit` expects the new volume 68835.28.

B4-7. pan_head_screw stays a ground-truth mismatch, now with evidence. CadQuery's own groove solid has 25.45 mm3 (isFrenet=False, the dataset's setting) and 24.14
(isFrenet=True), our geometric 24.07 matches the latter. The program then cuts the groove from the shank with the groove's outer edge lying exactly on the shank
cylinder (vertices at r = 3 = shank radius): OCCT's cut removes 10.31 mm3 from a bare shank cylinder (40 percent of its own tool volume, the dataset part removes
10.08), i.e. the reference is an OCCT boolean that only partly cut a tangent-touching groove. Reproducing it would mean reproducing the failure.

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
