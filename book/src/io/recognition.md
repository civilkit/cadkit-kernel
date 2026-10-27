# Feature recognition

An imported solid is a plain B-rep with no history. The `recognize` request rebuilds an editable feature list for it: it finds holes, a prismatic or revolved base and the blends on it, emits ordinary features, rebuilds them, and compares the result numerically with the imported body. The code is the crate `crates/ck-recog`. It is deterministic and uses no learned model.

## Request

```json
{"type":"recognize","tag":"imp","opts":{"apply":true}}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `tag` | string | `"main"` | State holding the body to analyse (usually an import) |
| `opts.apply` or `apply` | bool | `false` | Store the rebuilt body under the tag `<tag>.rebuilt` |

A tag with no state answers the usual `noState` error. A tag with state but no body answers `null`.

## Reply

`result` holds:

| Field | Meaning |
|---|---|
| `features` | The recognised feature list, in the same JSON as a part's `features` (`plane`, `sketch`, `extrude`, `revolve`, `hole`, `fillet`, `chamfer`, `lpattern`, `cpattern`). Feature ids restart at `F1` |
| `report.strategy` | `"prism"`, `"revolve"` or `"none"` |
| `report.note` | Why nothing was found, or `null` |
| `report.features` | One entry per recognised feature: `id`, `type`, `kind`, `confidence`, the imported `faces` it explains and its `params` |
| `report.unexplained` | Faces no feature explains: `face`, surface `kind`, `area` |
| `report.faces`, `report.explainedFraction` | Face count and the fraction of faces explained |
| `verify` | The numeric check, below. `null` when nothing was rebuilt |
| `errors` | Regeneration errors of the rebuilt feature list (strings) |
| `unexplainedArea`, `unexplainedAreaFraction` | Area of the unexplained faces, absolute and relative to the whole body |
| `applied` | `true` when `apply` was set and a body was rebuilt |

`verify` has `volume` (rebuilt), `volumeOriginal`, `symDiffVolume` (the volume of the symmetric difference, computed with the kernel's own booleans), `relativeError`, `areaRelativeError`, `bboxDeviation`, `exact` (relative error below 1e-6) and `close` (below 1e-3).

## Example

The plate of the [STEP chapter](step.md) (40 x 20 x 8 mm, sketched on the `front` plane, 10 mm through hole made by a `hole` feature) with one more feature, a 2 mm fillet on the 40 mm edge between the top cap and the side at y = 0:

```json
{"type":"fillet","id":"F6","radius":2,
 "edges":[{"faces":["F2.C1","F2.S.e1"],"at":[20,0,8]}]}
```

The part is built, exported with `step`, imported with `importStep` and then recognised. The part has volume 5737.345 mm3 (the volume depends on the edge: a fillet on a shorter edge removes less material). The report (trimmed):

```json
{"strategy":"prism",
 "features":[
  {"id":"F2","type":"extrude","kind":"base","confidence":0.95,
   "faces":["I1.F1","I1.F2","I1.F4","I1.F5","I1.F6","I1.F7"],
   "params":{"axis":"y","depth":20,"level":0,"profileLoops":1}},
  {"id":"F5","type":"hole","kind":"simple","confidence":0.9,"faces":["I1.F3"],
   "params":{"diameter":10,"through":true,"depth":null,"count":1,"pattern":"none","axis":"z"}},
  {"id":"F6","type":"fillet","kind":"fillet","confidence":0.85,"faces":["I1.F8"],"params":{"radius":2}}],
 "unexplained":[],"faces":8,"explainedFraction":1}
```

and `verify`:

```json
{"volume":5737.345,"volumeOriginal":5737.345,"symDiffVolume":0,
 "relativeError":0,"areaRelativeError":0,"bboxDeviation":0,"exact":true,"close":true}
```

The recognised feature list is a sketch, an extrude, a construction plane, a sketch with the hole centre, the `hole` feature and a `fillet` whose edge is named by the base extrude's persistent face names. With `apply: true` the rebuilt body is in the tag `imp.rebuilt` and has the same volume.

Two SolidWorks sample parts, imported with `importSldprt` and recognised:

| Part | Strategy | Features | `verify` |
|---|---|---|---|
| Cone, radius 5, height 10 | `revolve` | sketch, revolve | exact, relative error 2e-16 |
| Cube with a 5 mm hole | `prism` | sketch, extrude, plane, sketch, hole | exact |

A body the analysis cannot explain is reported as such. A whole sphere imported from STEP gives `strategy: "none"`, no features, `verify: null`, and the single face `I1.F1` (kind `sphere`, area 1256.6) in `unexplained`; `unexplainedAreaFraction` is 1.

## How it works

1. **Classify faces** as plane, cylinder, cone, sphere, torus or other, with their outward sense.
2. **Holes.** Concave cylinders are grown into coaxial groups: concave cylinders and cones, plus planar floors and steps that meet a group member at a concave edge and have only coaxial circular edges. The circles where the group meets planes normal to the axis give the entry and exit, so the hole is through or blind, and the stage sequence (countersink cone, counterbore cylinder, bore, drill-point cone) gives its kind and sizes. A simple hole becomes a `hole` feature; a counterbore adds an extrude cut and a countersink a revolve cut. Identical holes become one multi-point hole, or a seed plus a linear pattern (along a world axis) or circular pattern (about a world axis through the origin).
3. **Prismatic base**, tried on each of the three principal axes. Wall faces (planes parallel to the axis, cylinders with the axis parallel) form closed cycles through their vertical edges. The largest cycle is the outer profile, the others are windows. Planar caps sit at the two extremes. A constant-radius cylinder or torus tangent to a cap and a wall, or an equal-distance chamfer plane or cone, becomes a `fillet` or `chamfer` named through the base extrude's face names. Sphere patches between blends of the same radius are folded in. Pockets, bosses and islands come from the height intervals of the wall cycles; each cycle becomes an add or cut extrude.
4. **Revolve.** If a prism cannot explain the faces, a revolve is tried: every face must be a plane normal to the axis or a quadric about it. The generatrices are chained through their circular edges into a closed half profile.
5. **Verify.** Up to four candidate recognitions are rebuilt in preference order. The first exact one with no unexplained faces wins; otherwise the one with the smallest volume error is reported.

## Limits

Recognition covers what a few common construction patterns produce. It is not a general reverse modeller.

- Not recognised: obliquely oriented bases or holes (the principal axes only), holes whose mouth lies on a curved wall (radial bores), B-spline or extruded-spline walls, lofts and sweeps, shells, variable and asymmetric blends, mirrored halves, whole-body spheres and tori, and bodies with cross-bores.
- Walls that are not all parallel to one principal axis (tapered walls, ribs, slanted faces) make the wall-cycle walk fail.
- Parts that need a revolve plus a prismatic cut are not handled.
- Layered rebuilds can be close but not exact (1e-3 to 3e-3) when a blend edge is named across several extrudes.
- Free-form parts and gear bodies are slow, because the verification booleans and the area integration of spline faces dominate (seconds).
- The recognised tree reproduces the shape, not the intent: sketches are in principal planes, ids are renumbered, and dimensions are what the geometry gives.

Corpus runner: `tools/recog_check.mjs` regenerates each sample part, exports it as STEP, re-imports it, recognises it, and classifies the rebuild as `exact`, `close`, `partial` or `failed`. The sample parts it reads are not in the repository. On the prismatic subset (parts built only from sketch, extrude, revolve, plane, hole, fillet, chamfer and pattern features) 27 of 66 parts are exact or close; `tools/recog_baseline.json` stores that count and the check exits non-zero if it drops.

## See also

- [STEP](step.md), [SolidWorks](sldprt.md)
- [Solids from sketches](../format/features-solids.md) and [edge and face edits](../format/features-edits.md), the features recognition emits
- [Persistent names](../kernel/naming.md)
