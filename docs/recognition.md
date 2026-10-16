# Feature recognition (ck-recog)

Imported solids (STEP, SolidWorks) are plain B-reps. `recognize` rebuilds an editable feature tree for them:
a deterministic pass over the face adjacency graph with exact surface kinds, then a numeric check against the original.

## Request

```
{"type":"recognize","tag":"imp","opts":{"apply":true}}
```

Reply `result`:

- `features`: ordered feature JSON (the shapes the app and scripts use): `plane` (offset), `sketch`, `extrude`, `revolve`,
  `hole`, `fillet`, `chamfer`, `lpattern`, `cpattern`.
- `report`: `strategy` (`prism` | `revolve` | `none`), per feature `{id, type, kind, confidence, faces, params}`, the
  `unexplained` faces (`face`, surface `kind`, `area`) and `explainedFraction`.
- `verify`: rebuilt volume, `symDiffVolume` ((A-B)+(B-A) by our booleans), `relativeError`, `bboxDeviation`,
  `exact` (< 1e-6) and `close` (< 1e-3).
- `errors` (regen codes of the rebuild), `unexplainedArea`, `unexplainedAreaFraction`, `applied`.

`apply: true` stores the rebuilt body under the tag `<tag>.rebuilt`.

## Algorithm

1. Faces are classified (plane, cylinder, cone, sphere, torus, other) with their outward sense.
2. Holes: concave cylinders are grown into coaxial groups (concave cylinders and cones, plus planar floors / steps that
   meet a member in a concave edge and have only coaxial circular edges). The mouths (circle edges onto planes normal to
   the axis) give entry and exit: through / blind, and the stage sequence (countersink cone, counterbore cylinder, main
   bore, drill-point cone) gives the kind and the sizes. Emitted as `hole` (simple, `diameter`); a counterbore adds an
   extrude cut, a countersink a revolve cut. Identical holes become one multi-point hole, or a seed plus `lpattern`
   (along a world axis) / `cpattern` (about a world axis through the origin).
3. Prismatic base, tried on the three principal axes: wall faces (planes parallel to the axis, cylinders with the axis
   parallel) form closed cycles through their vertical edges (arc side from a horizontal edge of the wall); the largest
   cycle is the outer profile, the others are windows. Planar caps at the two extremes. A face between a cap and a wall
   that is a constant-radius cylinder / torus tangent to both, or an equal-distance chamfer plane / cone, becomes a
   `fillet` / `chamfer` whose edges are named by the base extrude's persistent face names (`F3.C1`, `F3.S.e2`) plus an
   `at` point on the sharp edge. Sphere patches between same-radius blends are folded in.
4. If faces remain that a prism cannot explain, a revolve is tried: every face is a plane normal to the axis or a
   cylinder / cone / sphere / torus about it; the generatrices are chained through their circular edges into a closed
   half profile (arcs oriented by the face sense) with a construction axis line.
5. Faces left over are reported as unexplained; the tree contains the explained part.

## Corpus runner

`node tools/recog_check.mjs --wasm <ck_wasm.wasm>`: regen each kit sample, STEP export, re-import, recognise, compare. The
prismatic subset (samples made only of sketch/extrude/revolve/plane/hole/fillet/chamfer/pattern) is gated by
`tools/recog_baseline.json`. `--update-baseline` rewrites it.

## Not recognised yet

Stacked bodies (pockets and bosses at intermediate heights), obliquely oriented bases or holes, cross holes in curved
walls, B-spline / extruded-spline walls, lofts, sweeps, shells, variable and asymmetric blends, mirrored halves.

## Results (first run, kit corpus + SolidWorks samples)

`node tools/recog_check.mjs`: 102 parts, 2 skipped (the original does not regenerate). exact 34, close 0, partial 29, failed 37.
Prismatic subset (gated): 27 of 66 exact or close (41 %; target was 70 %).

Exact: bench-bolt, bucket, enclosure, fan_shroud, hex_nut, hinge, motor_end_cap, pulley, ratchet_sector, round_flange,
snap_clip, spur_gear, vented_panel, venturi_tube, washer, brepjs button_top_battery_cell, fan_guard,
l_angle_gusset_bracket, linear_bearing_block, nema_stepper, rubber_foot, tripod_rc2_plate, v_groove_slider_rail,
vented_louvre_case, mounting-plate, parametric-bracket, patterned-threaded-holes, surface-patch, threaded-bolt; the
SolidWorks single-feature parts cube, cube with a 1 mm fillet, cube with a 5 mm hole, cone and cylinder with a fillet.
Not recognised: SolidWorks sphere, torus, crossed cylinders (whole-body sphere / torus and cross-bores are not handled).

Timing: below 200 faces almost every part is under 0.5 s; the slow ones are free-form (springs, propeller, large sample:
the area integration of spline faces and the verification booleans) and the gear bodies (2 to 10 s, many layered extrudes).

Failure analysis (prismatic subset, not exact):

- Wall structure the cycle walker rejects ("wall cycle broken", "more than two vertical neighbours") on 14 parts with
  slanted faces, tapered walls or ribs: the walls are not all parallel to one principal axis, or the same wall meets
  neighbours at several heights (clevis, dovetail, u_channel, heat_sink, pipe_flange, rect_frame, gridfinity_bin).
- Holes whose mouth lies on a curved wall (radial bores in a collar, pulley or hinge): the mouth edges are not circles
  on a plane, so the hole group is left unexplained (shaft_collar, gt2_pulley, pull_handle, knuckle_hinge, piston).
- Cone / sphere / torus bodies that are neither pure revolves nor prisms (bellows, dome_cap, nozzle, funnel, hose, torus
  link): need a revolve with a prismatic cut on top.
- Free-form walls (extruded splines in gears, twisted drill) are `Other` faces.
- Several layered rebuilds are close but not exact (cam, hobby_rc_servo, i_beam: 1e-3 to 3e-3): blend edges named across
  several extrudes (`missingEdge`), and a boolean failure on an island in a pocket.

## Phase 2 (recommended)

1. Kit "Convert to editable": replace the view-only import by the `recognize` result (features + `<tag>.rebuilt`), keep the
   original as a reference body and show `verify` and the unexplained faces.
2. LLM-assisted repair of unexplained faces through the MCP tools: hand over `report.unexplained` with the surface data
   and the rebuilt tree, let the agent add features, and re-check with `refDiff` / `recognize`'s verify.
3. Robustness: radial holes (mouths on cylinders), a base axis that is not principal (plane from three points), revolve
   plus cut, fillet naming across several extrudes by geometric `resolve` descriptions, mirrored halves.
