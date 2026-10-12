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
