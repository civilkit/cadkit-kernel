# BenchCAD findings (30 parts)

Source: Hugging Face `BenchCAD/BenchCAD`, config `code_gen`; dataset card says
cc-by-4.0. Ground truth from CadQuery 2.8.0 (volume, area, centre of mass,
bbox, STEP). Parameters translated by hand; no CadQuery source copied.
Run: `node tools/bench/run.mjs`. Gate: `bench.benchcad` 28/30 (soft),
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
| coil_spring | kernel-gap | 3D spline sweep path: the variable-pitch coil is translated as three constant-pitch helix sweeps whose union (cap to cap, the 6 mm pitch coils overlap the 6.94 mm wire) fails `booleanFailed` after ~8 s (it used to hang). Its ground-truth STEP now imports, validates and tessellates in a few seconds |
| pan_head_screw | ground-truth-mismatch | dataset thread removes 10.08 mm3; the geometric groove removes 24.07 |

Also: t_pipe_fitting regenerates but its ground-truth STEP fails `validate`
(`EDGE_OFF_SURFACE`), a reference-geometry issue, not ours.

## Ranked kernel gaps (parts blocked)

1. Boolean robustness: done for this set (was 4: i_beam, dome_cap, bellows, eyebolt). Root causes fixed: marching SSI closed small loops prematurely and shadowed seeds (i_beam, dome_cap), near-axis revolve arcs gave off-axis pcurves/poles, point-in-face sampled big circles with 24 chords (bellows), marched curves stopped short of loft/domain edges, torus fragments crossing both seams did not tessellate (eyebolt).
2. Chamfer across curved or tangent edges: done for this set (was 2: tapered_boss, u_channel). Root causes fixed: the blend code only took straight edges between planes and true `Circle` edges between analytic coaxial faces, so a spline rim of a lofted cone and a G1 plane/fillet-cylinder edge were `blendUnsupported`; split rims, tangent edges and rational-spline classification (winding-number fallback, 16 s) now handled. Remaining: tangent/curved *fillets* and concave tangent chamfers still `blendUnsupported`; a chamfered split ring yields one blend face (B0), not one per arc.
3. Multi-piece sweep path: done (was 2: pipe_elbow, duct_elbow). Each piece is swept with the section carried along, tangent joints unite in coincident caps, line-line corners are mitred (overshoot + bisector half-space clip); faces named `F.S.<tag>~k` per piece. Remaining: mitre joints involving arcs, twist, multi-region profiles on multi-piece paths.
4. Multi-body results and multi-solid STEP import: done (was 2: torus_link, torsion_spring). A body already holds one lump per solid (wire: `faces[].lump`; mass, validate, tessellate and STEP export are per lump), so no representation change was needed. Root causes: (a) torus_link: the boolean's untouched torus fragment has a loop of two seam twins and `assemble::build` shifted the twin pcurves by the u period for both seams, so the torus face got zero area (volume 2011 instead of 102954: it looked like a dropped solid); (b) torsion_spring/coil_spring STEP: a B-spline sweep face closed on itself (no period on `Surface::BSpline`) got both seam sides at the same u and its circle edges projected across the closing isoline, so the face had zero area and no mesh; (c) eyebolt STEP: a torus fragment bounded by one seam and one real edge failed the universal-cover tessellation fallback on a 1e-8 uv mismatch between independently sampled edge ends. `ck-check` now judges each lump on its own (`VOLUME_NONPOSITIVE` / `SOLID_MULTIPLE_OUTERS` name `lump#N`). 5. Helical sweep performance and hangs: torsion_spring done (regen 3 s, mass 1 s, validate 1 s, STEP import 7 s in release wasm; was 500 s). Root causes: (a) `BSplineSurface::closest_param` costs O(spans) per call (577 spans) and the SSI marcher seeds ~1250 closest points per face pair, so the exact grid search got a control-net-box lower bound, a bounded variant (`closest_param_within`: far points cost one box pass), a coarser per-cell grid on long surfaces and a Newton restart across the closing isoline / multi-start on overlapping sheets (it returned the wrong sheet or the seam for points just across `u = 2 pi`: the coil ground truth lost 9% volume with a coarser seed); (b) the boolean's local uv projection rejected the edges of an interpolated screw face (they are 2e-5 off it) and fell back to the global search; (c) `imprint` marched the whole 12-turn sweep for a leg's cap plane (24 loops, one per half turn): SSI now runs in the overlap of the two face boxes when one face is a long spline; (d) `region::classify` integrated the winding number over the sweep (4 s per face) although the nearest-feature test is exact at any distance: bodies with a long spline face trust it; (e) `face_integral_poly`: volume integrals of polynomial integrands use the Gauss points the degrees need (one panel per span, 5 points across the section) instead of two GL8 panels; (f) tessellation re-tested every triangle on every refinement pass (35 s for the 100k-triangle coil ground truth, now 4 s, same triangles), and rational surfaces evaluated through allocating generic code. The 0.11% volume difference to the dataset is the dataset's: its coil faces lie 0 to +4.2 um outside the ideal screw tube (`torsion_spring_gt_coil_vs_ideal_tube`), ours is within 3e-5 of it and equals A 2 pi R N (N = height / pitch = 11.9977, not 12); the bench carries a documented `tolerances` entry (volume 1.5e-3) for that part.
6. Rotated workplanes: done. Plane modes `angle` (rotation about a world axis or straight-edge axis), `perpendicular` and `mid` are implemented and pinned against the original (plus `planeLineNormal`); torsion_spring now uses the tilted leg planes. Still offset-fallback: parallel, threePoints, linePoint, tangent, normal, normalCurve, curve (point/curve driven), and axis refs other than world axes / straight edges (sketch lines, axis features).
7. 3D spline sweep path: 1 (coil_spring).
8. STEP-import-only failures: t_pipe_fitting, pan_head_screw,
   dome_cap (sphere pole boundary), coil_spring (35 s tessellation, skipped).

## Notes

- Item 6 is a feature-semantics bug that also affects interactive use.
- Re-rank after each fix; add the next batch of parts only when this list
  is short.
