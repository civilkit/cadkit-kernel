# BenchCAD findings (30 parts)

Source: Hugging Face `BenchCAD/BenchCAD`, config `code_gen`; dataset card says
cc-by-4.0. Ground truth from CadQuery 2.8.0 (volume, area, centre of mass,
bbox, STEP). Parameters translated by hand; no CadQuery source copied.
Run: `node tools/bench/run.mjs`. Gate: `bench.benchcad` 29/30 (soft),
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

B3-1. Silent wrong union (j_hook, worst kind: no error, validate clean). A cylinder (r 6) unioned with a
revolved 180 degree handle of the same radius whose start-cap rim lies exactly on the cylinder surface gives
14938.2 mm3 where CadQuery gives 22782.6 (the handle outside the rod is mostly dropped). The same union with
the handle radius 5.99 or the rod 6.01 is right (22746.7 / 22822.6), and the whole part with the 5.99
handle matches CadQuery run on the same input (32266.86 vs 32266.87), so the translation is right. The next
feature (a plate union) then fails `booleanFailed` on the bad body. Cause not yet located in the Rust:
coincident circle between a planar cap and a coaxial cylinder side (curve-on-surface contact).

B3-2. Fillet ending on chamfer faces (cable_routing_panel): the four `|Z` corner fillets (r 0.83) after a
0.95 top-rim chamfer. `blendFailed` for one of the four symmetric edges alone (S.e1|S.e2), the other three
work, and success is erratic in the parameters (fillet radius ok at 0.3 and 0.95, fails at 0.7 to 0.9, 1.0,
1.2; chamfer distance fails at 0.95 and 1.5, ok at 0.5 to 1.0). Looks like a numerically marginal corner
patch where the vertical fillet meets the mitred chamfer corner. Chamfering after the fillet instead also
fails (tangent rim chain), so there is no order that works.

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
Run: `node tools/bench/run.mjs --set brepjs` gives 18/25 regen, 22/25 step import;
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
| conduit-snap-clip | FAIL | FAIL | kernel-gap (same fillet) + kernel-bug (countersink cone coincident with the hole); step: HYPERBOLA |
| conical-pour-funnel | FAIL | pass | kernel-bug: cut with a cylinder whose wall contains the earlier cone cut's base circle |
| knuckle-hinge | FAIL | pass | kernel-bug (4 countersinks, same coincident circle) + reference-mismatch (the example's two leaves overlap: sum 5146.71, union 4990.04) |
| dovetail-joint | FAIL | pass | kernel-gap: no body-level intersect; male half exact (12415.619), female taper missing, 0.18% volume |
| modular-hose-segment | FAIL | pass | kernel-bug: boolean fails when a sphere pole lies inside the other solid |

### Ranked gaps from the brepjs batch

BJ-1. Coincident circle on a cylinder wall (6 features in 3 parts: conduit-snap-clip, knuckle-hinge, conical-pour-funnel;
same family as B3-1). A countersink cone cut (revolve) whose base circle has exactly the radius of the drilled hole is
`booleanFailed`; base radius +-0.01 off (2.29 / 2.31 for a 2.3 hole) works. Funnel: a bore cylinder cut after a cone cut
that starts on a circle of the bore's radius fails the same way. This is the common countersunk-hole shape.

BJ-2. Boolean with a sphere pole inside the other solid (modular-hose-segment, cascades to 7 errors). Union of the
revolved truncated sphere with a cylinder whose cap or side encloses the pole is `booleanFailed`, also for a full-sphere
cut from a cylinder whose body holds a pole, and off-axis (cylinder at x = 0.5); a cylinder ending below the pole (z = 8.0
of a pole at 8.33) works.

BJ-3. Fillet of a straight edge between a plane and a cylinder whose axis is parallel to the plane (pipe-saddle-p-clamp
concave foot/tube edge, conduit-snap-clip lip). `blendUnsupported` at r 2, 0.5 and 0.05; chamfer of the same edge works.

BJ-4. STEP import: HYPERBOLA curve (rotary-potentiometer, conduit-snap-clip); torus fillet rim from OCCT gives
SHELL_ORIENTATION and 44% low volume (rubber-foot; same as B3-3).

BJ-5. No body-level boolean between two tools (dovetail-joint female cavity = prism minus two rotated slabs). Not
expressible with add/cut features; only `upToNext` intersects, and only against the existing body.

BJ-6. Several separate overlapping solids in one document (knuckle-hinge) cannot be represented: features union into one
body. Not fixable by translation; the figures of that part cannot be matched exactly.

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
