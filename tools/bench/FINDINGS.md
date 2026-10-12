# BenchCAD findings (30 parts)

Source: Hugging Face `BenchCAD/BenchCAD`, config `code_gen`; dataset card says
cc-by-4.0. Ground truth from CadQuery 2.8.0 (volume, area, centre of mass,
bbox, STEP). Parameters translated by hand; no CadQuery source copied.
Run: `node tools/bench/run.mjs`. Gate: `bench.benchcad` 18/30 (soft),
`bench.benchcad.step` 23/30 (soft). Figures as of 2026-10-12 after the
twist-extrude and validator fixes (the first-10 set is now 9/10 regen).

Failure causes: kernel-gap = feature or operation missing; kernel-bug = exists
but wrong; ground-truth-mismatch = the reference itself differs from the
intended geometry. No remaining failure is a translation mistake.

## Passing (regen)

First 10: round_flange, pulley, washer, fan_shroud, enclosure, heat_sink,
nozzle, worm_screw, twisted_drill.
Next 20: hex_nut, bolt, spur_gear, bevel_gear, spline_hub, venturi_tube,
t_pipe_fitting, dovetail_slide, sheet_metal_tray.

## Failing

| part | cause | detail |
|---|---|---|
| tapered_boss | kernel-gap | chamfer on a plane / lofted-spline edge: `blendUnsupported` |
| u_channel | kernel-gap | chamfer on a plane / fillet edge: `blendUnsupported` |
| i_beam | kernel-bug | hole axis lying in the end-face plane: `booleanFailed` |
| bellows | kernel-bug | 6-hole cut pattern: `booleanFailed` |
| eyebolt | kernel-bug | torus union: `booleanFailed` |
| dome_cap | kernel-bug | sphere built as a degenerate torus; `booleanFailed` |
| pipe_elbow | kernel-gap | sweep along a multi-piece path |
| duct_elbow | kernel-gap | sweep along a multi-piece path |
| torus_link | kernel-gap | point-contact union leaves two solids (multi-body result) |
| torsion_spring | kernel-gap | multi-body, rotated workplane, helical sweep time; skipped in gate |
| coil_spring | kernel-gap | 3D spline sweep path, helical sweep hangs or takes minutes; skipped in gate |
| pan_head_screw | ground-truth-mismatch | dataset thread removes 10.08 mm3; the geometric groove removes 24.07 |

Also: t_pipe_fitting regenerates but its ground-truth STEP fails `validate`
(`EDGE_OFF_SURFACE`), a reference-geometry issue, not ours.

## Ranked kernel gaps (parts blocked)

1. Boolean robustness: 4 (i_beam, dome_cap, bellows, eyebolt).
2. Chamfer across curved or tangent edges: 2 (tapered_boss, u_channel).
3. Multi-piece sweep path: 2 (pipe_elbow, duct_elbow).
4. Multi-body results and multi-solid STEP import: 2 (torus_link,
   torsion_spring).
5. Helical sweep performance and hangs: 2 (coil_spring, torsion_spring).
6. Rotated workplanes (every plane mode currently behaves as offset): 1
   (torsion_spring).
7. 3D spline sweep path: 1 (coil_spring).
8. Sphere from a revolved arc centred on the axis: 1 (dome_cap).
9. STEP-import-only failures: t_pipe_fitting, pan_head_screw, eyebolt,
   coil_spring.

## Notes

- Items 1 and 8 may share a cause (degenerate surfaces at axis points).
- Item 6 is a feature-semantics bug that also affects interactive use.
- Re-rank after each fix; add the next batch of parts only when this list
  is short.
