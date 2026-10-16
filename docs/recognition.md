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
