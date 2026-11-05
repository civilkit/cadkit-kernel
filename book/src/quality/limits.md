# Known limits

This chapter lists what the kernel does not do or does not do reliably, ordered by how soon a user is likely to meet it. Each item says what happens and where the evidence is: a sweep result, an `#[ignore]`d test with its reason, or a run made for this book. Counts from sweeps are snapshots of a past build (see [corpora and sweeps](corpora.md)); re-run the sweep for the current state.

## 1. Things met in the first hour

**Unknown fields are ignored.** Part parsing is deliberately lenient. A misspelled field is skipped, and the feature runs with its default. Checked:

```json
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depht":8}
```

regenerates with no error and no warning, and the plate is 0.000001 mm thick (the default depth). A field with an unknown name (`bogusField`) is also accepted silently. The only structural errors are a part that is not an object, a wrong `format`, and `features` that is not an array; an unknown feature `type` is a per-feature error (`unknownFeature`, checked). Reporting ignored fields as errors or warnings is not implemented. Until then, compare the result with `check` or `massProps` instead of trusting an empty `errors` list.

**A valid-looking reply can be wrong.** The sweeps find `wrongVolume`, replies without an error whose volume differs from an independent computation. They occur in random overlapping curved primitives, outward shells and three-section lofts (details below). A volume sanity check exists for booleans (`booleanVolumeMismatch`) but does not catch everything. Call `validate` and compare `massProps` with what you expect.

**Large blends are refused.** A fillet radius that approaches half the shortest side of a box, or half the height of a cylinder, answers `blendOverflow`, although the geometry is valid and an independent geometry library builds it. In the fillet sweep this was 32 of 640 cases, starting at radii of about 6 mm on the sweep's random sizes. Checked: a 9 mm fillet on the top long edge of the 8 mm-thick 40 x 20 x 8 plate answers `blendOverflow` ("spring leaves its face away from its edges").

**Blends at corners are limited.** Variable-radius blends (`var_box` in the sweep: 9 of 40 ok, 28 `wrongVolume`, 3 `blendFailed`) are the weakest group. Unequal radii at a vertex where three edges meet end in a mitre, not a patch: valid, but not tangent-continuous, and flagged with a `cornerNotTangent` warning. A three-edge corner with radii 10, 5 and 5 (one case in the test-script corpus), face-face fillets, adjacent conic or curvature-continuous fillets (the ignored test `conic_ring`), and fillets of a ring at a blade root (the ignored test `closed_blunt_root_ring_fillets_cleanly`) answer `blendFailed` or `blendUnsupported`. Partial cylinder and cone sectors with fillets are `blendUnsupported` in the test-script corpus.

**A fillet of a rounded-rectangle loop can stop responding.** In the fillet sweep, 2 of 40 `rrect_loop` cases (3 with a second seed) returned no reply at all (`kernel-null`) for a top-loop fillet with `propagate`. A time budget may not stop such a request; run untrusted documents under a host-side timeout as well. See [robustness](robustness.md).

## 2. Shells, offsets and thickness

| Case | Result (shell sweep, 40 cases per template) |
|---|---|
| Inward shell of a box, one or two faces removed | Works; thick walls refused with `shellTooThick` (expected) |
| Outward shell with the top removed | All 40 `wrongVolume` (an extra roof slab remains) |
| Shell of a drafted cylinder (cone) | 38 `wrongVolume`, 2 `draftTooLarge` |
| Closed shell of a two-box L | 19 `wrongVolume`, 21 refused |
| Tube with the top removed | 15 `shellFailed` |
| Thicken of planar, cylindrical and conical sheets | Works |

Walls made of general B-spline surfaces are offset by fitting a grid and report a sampled radius; extruded spline walls are exact. Shell and thicken of a hull with a point-ended loft work only when the cavity can be cut off short of the apex; a plain cone loft and walls thicker than the keel radius are refused with a hint.

## 3. Booleans on curved shapes

Coincident, tangent, contained and sliver relations between boxes, cylinders and spheres are handled (145 of 147 cases in the coincidence corpus without any tolerance; the two failures are near-coincident cylinders 1e-3 apart, and with a `tolerance` of 1e-3 or `autoFuzzy` all 147 pass). Random overlaps of curved primitives are the weak spot. In the boolean sweep, 29 of the 175 cases that were not an expected no-effect or empty result (320 minus 80 `okNoEffect` and 65 `okEmpty`) failed (22 `wrongVolume`, 5 `invalid`, 2 `booleanFailed`):

- anything with a torus (10 cases, relative error 1.7e-3 to 0.95);
- cone against cone and cone against cylinder;
- sphere against torus or cone;
- chains of three bodies with a cone, torus or sphere.

In the test-script corpus, the clusters are a sphere against a box corner, perpendicular cylinders intersected, cone and cylinder near a seam, and staged blends followed by a union (86 failures among 1664 cases). Tool bodies with 100 instances (a long pattern) make a boolean take 5 to 30 s.

## 4. Lofts, sweeps and free-form surfaces

- Three-section circle lofts differ from a smooth thru-sections surface: relative volume error 3e-3 to 0.15 in the shape sweep (16 of 30); in the mass-property sweep all 20 three-section lofts differ from the thru-sections reference, by up to 6.7 percent in volume, 3.7 percent in area, 1.6 percent in centre of mass and 10 percent in inertia. The mass-property reply is exact for the shape the kernel builds; the shape itself is a different surface from the reference. Two-section lofts, rotated rectangles and point-ended lofts agree.
- No closed loft, no sweep normal control; multi-section sweeps are partial; mitre joints that involve arcs, twist and multi-region profiles on multi-piece paths are not built.
- Surface-surface intersection on arbitrary NURBS is the weak spot of the free-form side. There are no network or constrained plate surfaces, and no mesh-to-B-rep conversion or sewing from meshes.
- Booleans on free-form walls: spline pockets, bosses and rings work on spline walls; other tilted plane or extrusion sections still march and can be slow.
- Thread features: pipe-thread tapers, threads on conical faces and rounded roots are not modelled.

## 5. Import

See the import chapters for details; the main points:

| Format | Main limits |
|---|---|
| [STEP](../io/step.md) | No assemblies, placements, colours or names. Shell-based surface models and open shells are not imported. In four samples of 200 published files, 185 to 188 per sample imported valid and the rest were invalid (mostly `TESSELLATION_OPEN` and `EDGE_OFF_SURFACE`). Torus-fillet shell orientation and sphere pole boundaries are open |
| [SolidWorks](../io/sldprt.md) | Geometry of the active configuration only; the feature tree is read as a list, and [`sldprtHistory`](../io/sldprt.md#feature-history-sldprthistory) rebuilds it as editable features where it can (a part it cannot rebuild completely keeps the file's exact solid for the rest); the `(deltas)` transmit is not applied; sheet and wire bodies refused (`unsupported.body`); self-intersecting tori and some B-surface domains unsupported; blends and intersection curves are fitted (reported, refused above 0.05 mm) and some then fail the strict `validate` edge check |
| [SketchUp](../io/skp.md) | A surface body of planar faces; no curves, dimensions, scenes or text; refused above 50 000 faces unless `maxFaces` is raised |
| [ACIS SAT](../io/sat.md) | Exact splines only; no periodic splines, elliptical cones or reflecting transforms; no export. The bundled sphere and torus files (one face, no edges) import with volume 0 and `VOLUME_NONPOSITIVE` |
| Other formats | No STL, DXF, DWG, IGES or glTF import or export (listed as planned) |

[Feature recognition](../io/recognition.md) rebuilds a feature tree for simple prismatic and revolved parts: 27 of 66 prismatic sample parts were exact or close. Curved-wall holes, lofts, shells and variable blends are not recognised.

## 6. Output

- **Drawings** ([drawings](../io/drawings.md)): geometry only, as polylines; no dimensions, annotations or sheet layout; hidden-line handling of curved occluders is approximate, and thread symbols are not drawn.
- **STEP export** writes one product with geometry only: no colours, names, layers or assemblies.
- **Display mesh**: the default tolerance is `max(1e-3, 4e-4 x bounding-box diagonal)` mm. In the tessellation sweep 84 of 180 cases had a mesh volume more than 0.1 percent off the exact one, at worst 0.8 percent for a torus and 0.4 percent for a cylinder. Exact quantities come from `massProps`. A rare crack (`TESSELLATION_OPEN`) remains for one text case in about 1500 and for some imported parts; one imported part (`eyebolt`) loses its torus eye from the mesh (an ignored test).
- **Text** ([text and fonts](../io/text.md)) is planar and single-line; no wrapping onto curved faces, no complex scripts.

## 7. Features not built

Not implemented: sheet metal and steel bending (no flat pattern, no bend features); mate kinds `profileCenter`, `path`, `linearCoupler`, `cam` and `universalJoint`; hinge angle limits; `upToFace` for revolve and sweep, `offsetFromFace` on curved faces and `upToBody`; `thin` on sweep and loft and variable wall thickness on a thin extrude; moving a hole sideways with `moveFace`; a true n-ary union (booleans are sequential).

Six feature types are accepted as names (they are not `unknownFeature`) but are not built. `dome`, `splitLine`, `fillpattern`, `surfaceOffset` and `cutWithSurface` always fail with `TODO(rebuild): <type> needs ck-ops` (checked: each as an empty feature after the plate of the STEP chapter), and `spattern` does the same once it has a sketch (without one it answers `missingSketch`). Some options of built features are refused with the same prefix: a drafted `mid` or `throughBoth` extrude, a `lip` on curved or several rim faces, an `extend` with a shape other than `same`, and a surface `sweep` of an open profile. See [regeneration](../protocol/regen.md) and [patterns and transforms](../format/features-patterns.md).

## 8. Speed and concurrency

- The kernel is single-threaded. A trial with native threads gave 1.38 times; multithreaded booleans are open.
- Free-form parts (springs, propellers, shell-like lofts) and long patterns take seconds to tens of seconds in WebAssembly. The default per-feature budget is 120 s.
- Native runs are about two to three times faster than the module.
- Feature recognition of gear bodies takes 2 to 10 s.

## 9. Ignored tests

`cargo test -- --ignored` lists the documented open items (the others are developer probes that need an environment variable):

| Test | Reason in the attribute |
|---|---|
| `closed_blunt_root_ring_fillets_cleanly` (ck-model) | The ring path runs but the ball-centre solve diverges at the blade root |
| `marine_fuse_open_edge_spacings_build` (ck-model) | Three near-tangent rail spacings still leave an open edge |
| `the_concave_corner_patch_tessellates_watertight` (ck-model) | The sphere patch seam is sampled finer than the cylinder's end arc: 30 of 462 mesh edges are T-junctions (chord sag below 0.04 mm at r = 2) |
| `conic_ring` (ck-ops) | Adjacent conic or curvature fillets at a corner of two spline sections answer `blendFailed` |
| `eyebolt_torus_fragments_tessellate` (ck-io) | The torus eye of this imported part is dropped from the display mesh |

## 10. Common refusal codes

These codes mean the request was understood and refused; the kernel state is unchanged.

| Code | Typical cause |
|---|---|
| `unknownFeature` | A feature `type` the kernel does not have |
| `blendOverflow`, `blendTooLarge` | Blend radius or distance too large for the faces |
| `blendUnsupported`, `blendFailed` | A blend configuration not built, or whose construction failed |
| `booleanFailed`, `booleanVolumeMismatch` | The boolean could not assemble a result, or the result's volume failed the sanity check |
| `shellTooThick`, `shellFailed`, `shellUnsupported` | Wall thickness, construction or surface type |
| `draftUnsupported`, `draftTooLarge` | Draft of a face type or angle that is not supported |
| `thickenFailed`, `thickenReach` | Thicken could not close the offset sheet |
| `loft*`, `sweep*` | Section, guide or path problems (profile open, guide misses, path invalid) |
| `cutNoEffect` | A cut changed nothing |
| `featureBudgetExceeded`, `regenBudgetExceeded`, `booleanBudgetExceeded` | A time budget ran out |

The full list with parameters is in [errors](../protocol/errors.md) and [feature errors and diagnostics](../validation/diagnostics.md).

## See also

- [Corpora and sweeps](corpora.md)
- [Robustness under edits](robustness.md)
- [Testing](testing.md)
- [Errors](../protocol/errors.md)
- [STEP](../io/step.md)
