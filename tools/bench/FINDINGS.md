# BenchCAD findings (30 parts)

Source: Hugging Face `BenchCAD/BenchCAD`, config `code_gen`; dataset card says
cc-by-4.0. Ground truth from CadQuery 2.8.0 (volume, area, centre of mass,
bbox, STEP). Parameters translated by hand; no CadQuery source copied.
Run: `node tools/bench/run.mjs`. Gate: `bench.benchcad` 27/30 (soft),
`bench.benchcad.step` 26/30 (soft). Figures as of 2026-10-12 after the
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
| torsion_spring | kernel-gap | helical sweep time (12-turn sweep 19 s, union with a leg ~100 s); skipped in gate. Multi-solid result works (2 lumps, volume 0.11% off); its ground-truth STEP imports exactly |
| coil_spring | kernel-gap | 3D spline sweep path, helical sweep hangs or takes minutes; skipped in gate |
| pan_head_screw | ground-truth-mismatch | dataset thread removes 10.08 mm3; the geometric groove removes 24.07 |

Also: t_pipe_fitting regenerates but its ground-truth STEP fails `validate`
(`EDGE_OFF_SURFACE`), a reference-geometry issue, not ours.

## Ranked kernel gaps (parts blocked)

1. Boolean robustness: done for this set (was 4: i_beam, dome_cap, bellows, eyebolt). Root causes fixed: marching SSI closed small loops prematurely and shadowed seeds (i_beam, dome_cap), near-axis revolve arcs gave off-axis pcurves/poles, point-in-face sampled big circles with 24 chords (bellows), marched curves stopped short of loft/domain edges, torus fragments crossing both seams did not tessellate (eyebolt).
2. Chamfer across curved or tangent edges: done for this set (was 2: tapered_boss, u_channel). Root causes fixed: the blend code only took straight edges between planes and true `Circle` edges between analytic coaxial faces, so a spline rim of a lofted cone and a G1 plane/fillet-cylinder edge were `blendUnsupported`; split rims, tangent edges and rational-spline classification (winding-number fallback, 16 s) now handled. Remaining: tangent/curved *fillets* and concave tangent chamfers still `blendUnsupported`; a chamfered split ring yields one blend face (B0), not one per arc.
3. Multi-piece sweep path: done (was 2: pipe_elbow, duct_elbow). Each piece is swept with the section carried along, tangent joints unite in coincident caps, line-line corners are mitred (overshoot + bisector half-space clip); faces named `F.S.<tag>~k` per piece. Remaining: mitre joints involving arcs, twist, multi-region profiles on multi-piece paths.
4. Multi-body results and multi-solid STEP import: done (was 2: torus_link, torsion_spring). A body already holds one lump per solid (wire: `faces[].lump`; mass, validate, tessellate and STEP export are per lump), so no representation change was needed. Root causes: (a) torus_link: the boolean's untouched torus fragment has a loop of two seam twins and `assemble::build` shifted the twin pcurves by the u period for both seams, so the torus face got zero area (volume 2011 instead of 102954: it looked like a dropped solid); (b) torsion_spring/coil_spring STEP: a B-spline sweep face closed on itself (no period on `Surface::BSpline`) got both seam sides at the same u and its circle edges projected across the closing isoline, so the face had zero area and no mesh; (c) eyebolt STEP: a torus fragment bounded by one seam and one real edge failed the universal-cover tessellation fallback on a 1e-8 uv mismatch between independently sampled edge ends. `ck-check` now judges each lump on its own (`VOLUME_NONPOSITIVE` / `SOLID_MULTIPLE_OUTERS` name `lump#N`). Remaining: coil_spring STEP imports exactly but its 100k-triangle sweep face takes ~35 s to tessellate (skipped in the bench).
5. Helical sweep performance and hangs: 2 (coil_spring, torsion_spring).
6. Rotated workplanes: done. Plane modes `angle` (rotation about a world axis or straight-edge axis), `perpendicular` and `mid` are implemented and pinned against the original (plus `planeLineNormal`); torsion_spring now uses the tilted leg planes. Still offset-fallback: parallel, threePoints, linePoint, tangent, normal, normalCurve, curve (point/curve driven), and axis refs other than world axes / straight edges (sketch lines, axis features).
7. 3D spline sweep path: 1 (coil_spring).
8. STEP-import-only failures: t_pipe_fitting, pan_head_screw,
   dome_cap (sphere pole boundary), coil_spring (35 s tessellation, skipped).

## Notes

- Item 6 is a feature-semantics bug that also affects interactive use.
- Re-rank after each fix; add the next batch of parts only when this list
  is short.
