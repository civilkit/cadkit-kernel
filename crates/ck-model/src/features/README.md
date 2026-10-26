# features/

One module per feature family (file layout = rs_paths.txt). Each module
implements: payload parsing (ck-json), validation with the EXACT error
strings from kernel.strings, and dispatch to ck-ops/ck-topo.

## Payload field table (confirmed from .ckpart samples + app.pretty.js)

| type | fields |
|---|---|
| sketch | plane, points[{id,x,y,fixed}], entities[{type:id,...}], constraints[{type,refs,value,offset}], seq, visible |
| extrude | op(add/cut), sketch, end(blind/through/throughBoth/upToNext/mid), depth(expr), flip, draft(expr), draftOut, twist(expr, deg), twistCentre([x,y]), regions([entity ids], optional) |
| revolve | op, sketch, axis, end, angle(expr), flip, regions([entity ids], optional) |
| sphere | op(add/cut/newBody), center([x,y,z], exprs), radius(expr, above 0), body; no sketch: the revolve of a half disc about +Y through the centre, one face `<id>.S.e1` (a true sphere); `extrude.rs exec_sphere` |
| sweep | op, sketch, path ({kind: curve or sketch, id}; a `sketch` id may name a `sketch3d` holding one `spline`, tangent-constrained end lines allowed), twist (deg), (+guides, guideMode, surface: not reconstructed) |
| loft | op, sketches[], surface, guides[{sketch, influence next only}], centreLine ({sketch}: a planar open chain the loft follows, a moving frame carries the sections (ck-ops `loft/centre.rs`); a straight one is a no-op; errors loftCentreLineInvalid / Kink / Reach / Tight / Apex / Guides, each with detail + hint), startNormal / endNormal (tension; `true` = 1), connectors[null\|fraction]; startTangent / *Continuity -> `loftTangentEdges` (sketch profiles); regions with inner loops loft HOLLOW (outer loft minus the per-hole lofts, holes matched by centroid, a point section is the apex of each hole loft, inner-loop counts must agree else loftProfile {detail, hint}) |
| fillet | radius(expr), edges[] (a pick may carry radius, or radii [near, far] with filletType "variable"), propagate; see docs/blend-corpus.md |
| chamfer | chamferType(equal/distances=distanceDistance/angle=angleDistance/vertex), distance, angle, distance2, face, flip, propagate, edges[]; see docs/blend-corpus.md |
| draft | neutral, faces[], angle(expr), flip |
| shell | faces[], thickness(expr) |
| hole | sketch, standard(iso), size, diameter (simple holes only: plain diameter instead of size), fit(normal/tap), kind(simple/cbore/csink), end, depth, tip, flip |
| projectCurve | sketch \| curve (an earlier curve feature; wins over sketch), faces[] (empty/missing = every face of the solid and the surface bodies), degree(2..9, 3), tolerance(>0, 0.001), direction(both/forward/reverse); see project_curve.rs and docs/project-curve.md |
| helix | sketch, definedBy(pitchRevs/heightRevs/heightPitch), pitch, revolutions, height, startAngle, leftHand, reverse |
| scale | factor (number or [sx,sy,sz]; `factors` too), at (`origin`), bodies / body / target as moveBody; exact for planes, cylinders and cones stretched along their axis, B-spline faces, and (uniform) spheres and tori; the rest is refused `invalidValue` with detail and hint (`scale.rs`) |
| rib | sketch, thickness(expr) |
| lip | kind(material/move/offset/linear/turn/tension), faces[], edges[], width, height |
| lpattern | seeds[], dir, spacing(expr), count, reverse |
| cpattern | seeds[], axis, count, angle(expr), equal, reverse |
| mirror | seeds[], plane, body |
| trimSurface | surface, toolSurface, mutual, flip |
| surfacePlanar | sketch |
| surfaceRevolve | sketch, axis, end, angle |
| knit | surfaces[], solid |
| fill | edges[], merge, continuity(G0/G1/G2) |
| insertPart | part(nested .ckpart), place/source, fixed, dx,dy,dz, axis, angle |
| mate | kind(coincident/concentric/distance/angle/parallel), a, b, distance(expr), flip |
| refPlane | mode, base, offset, flip, visible |
| deleteBody | bodies[] ({face}/{surface}/{body}) |
| thread | face (cylinder face name), standard, size (validated against `crate::threads`), class, length (number or "full"), hand, start, modeled. Cosmetic: a `threads` entry + `warnings` (threadDiameterMismatch, threadModelFallback) in the regen result; modeled: helical groove tool cut from the body, faces `<id>.T0..T5` |
| moveFace | faces[], mode(offset/translate), distance, translate[x,y,z], flip. Offsets / translates faces via ck-ops `move_faces_body`; neighbours extend, tangent fillets travel, names unchanged; errors noFaces, missingReference, moveFaceUnsupported, moveFaceFailed (move_face.rs) |
| moveBody | bodies[]({face}/{surface}/{feature}/{body}, default all), translate[x,y,z], rotate{axis,origin,angle deg}, copy, body (name of the copies when copy:true); rigid, names unchanged (copy: `@<id>.1`); `{feature}`/bare non-face id = lumps of faces made by that feature incl. `@...` copies (move_body.rs) |
| pipeCut | radius(expr), edges[{faces,at}], faces[] (all their rim edges); the picks may sit in the solid or in a surface body; the result is a SURFACE body registered under the feature id (solid disappears) |

## Separate bodies (`bodies.rs`, docs/adr/0004)

- `op: "newBody"` (aliases `new`, `separate`) on extrude / revolve / loft / sweep / sphere: `Op::NewBody` -> `Combine::Separate`,
  the tool is appended as its own lump (`Brep::combine_at`), never united. Any other feature type with that op:
  `invalidValue {detail, hint}` (`bodies::check_feature`, run first in `exec_feature`).
- `body` (string) names the lump(s) a feature creates: newBody features, `moveBody` with `copy:true`, `combine` (result;
  default the target's name). Stored in `Lump::name` (ck-topo); `Brep::exec` puts the names of the lumps present before a
  feature back on the lumps with the same origin afterwards (`bodies::restore`), so every feature keeps them.
- Selector `{body: name}` in moveBody.bodies, combine target / tools, deleteBody.bodies. Errors: `bodyNameConflict {name,
  features}`, `missingReference {name, nearest}`, `ambiguousBody {name, candidates}`.
- `massProps {bodies:true}` entries and `interference` pairs carry the name; `validate` adds `info: [bodiesOverlap]`.

## Error strings (kernel.strings, verbatim per feature)

- sweep: sweepPathInvalid, sweepProfileNotNormal, sweepGuideMissing,
  sweepGuidesTwist, sweepGuideShort, sweepGuideStretchTwo, sweepSelfIntersects, sweepFailed
- loft: loftProfile, loftProfileOpen, loftProfileChains, loftGuideInvalid, loftGuideMiss,
  loftGuideOrder, loftGuideTangency, loftCentreLineInvalid/Tight/Reach/Kink/Apex/Guides, loftFailed, loftInvalidResult
- shell: shellTooThick, shellUnsupported, shellFailed, offsetUnsupported (shellTooThick / shellFailed / moveFace* carry `limit`, `minRadius`, `maxThickness`, `face` when known)
- draft: draftParallelFace, draftAxisNotParallel, draftUnsupported,
  draftTangentFaces, draftTooLarge, draftFailed (draftTooLarge from a closing taper carries `maxHeight`; a
  drafted circle extruded exactly to its tip builds a one-apex cone; the regen result lists `limits`
  `[[id, {maxHeight}]]` for such extrudes, see docs/spec-agent-interface.md "Taper limits")
- hole: holeNoPoints, holeInvalid, holeClose/Loose (fit); warning holeDowngraded (counterbore/countersink with a plain `diameter` but without its explicit step/cone dimensions is drilled as a simple hole)
- rib: ribNeedsOpenSketch, ribNoContact
- knit: knitNotClosed, knitNearMiss, knitFailed, knitted
- fill: fillNotClosed, fillInvalid, fillFailed, fillCrosses, fillContinuity
- thicken: thickenFailed, thickenPointedPole, thickenReach
- thread: threadSizeUnknown, threadFaceNotCylinder (warnings: threadDiameterMismatch, threadModelFallback)
- pipeCut: pipeCutRadius, pipeCutFailed, pipeCutNothingLeft, alignPipeCutOneBody
- projectCurve: projectCurveNoInput, projectCurveNoFaces, projectCurveMisses,
  projectCurveSeveral, projectCurveFailed, projectCurvePlane
  (payload: sketch|curve, faces[], degree, tolerance, direction both/forward/reverse); also editCurveDegree {min, max}
  and invalidValue (tolerance <= 0, unknown direction), missingSketch / missingReference; order in docs/project-curve.md
- extend: extendNotBoundary, extendTargetSelf, extendOneSurface,
  extendUnsupported, extendReflectTooFar, extendPointBehind, extendFolds
- blends: warning filletRadiusApproximated {requested, used, detail, hint} (variable fillet with an end radius of 0: built with the smallest positive end radius that works; both ends 0 / negative stay invalidValue with detail + hint), filletChamfer, chamferVertexEdges(two/points), chamferType+filletType,
  "edge of blend X is gone after an earlier blend", "X collapses and cannot be removed"
- assembly: mirrorOntoItself, mirrorNotKnitted
- boolean: booleanFailed (params `hostFace`, `toolFace`, `gap` when the failure is two nearly coincident free-form walls: the tool
  outline follows a wall to 1e-4..1e-2 but not exactly, so the section crosses it back and forth; use the sketch's own curve for the
  tool outline or keep it clear of the wall; raising the welding tolerance to 2-5x the gap was tried and still leaves open edges, so the
  walls are not merged in the kernel), cutNoEffect; `booleanVolumeMismatch` (ours, not in kernel.strings):
  the boolean closed into a valid body whose volume (or, for a union, box)
  falls outside what the operands allow (`ck_ops::boolean::sanity`: union
  between va+vb-overlap and va+vb, cut between va-overlap and va, overlap
  capped by the operands' box overlap). Params `what` (`volume`/`box`),
  XX  feature. Hint text lives in packages/driver/src/verify.ts.

## Surface bodies, rib, lip (reconstruction notes)

- `Brep::surfs` holds the surface (sheet) bodies by feature id; the wire
  `surfaces: [[id, body], ..]` follows the solid `body` (engine.rs adds
  `meta.surface: true` to every face and re-normalises plane normals).
  Consumers remove their inputs: knit (all inputs), trim (targets, + tool
  with `mutual`/`deleteTool`; an absent `deleteTool` keeps the tool), thicken,
  deleteFace; extend / rebuild / fill-merge append the result; a closed
  result (`fill {merge}` closing a knit skin, `knit {solid}`) is the solid body.
- Sheets are the solid sweep minus caps (`sheet()`); an open sketch chain is
  swept as `chain + closing chord` (chord faces/edges dropped, a clockwise
  revolve chain is reversed to CCW as a plain reverse). Probe parts: brep_cases
  `surf*`, `knit*`, `fill*`, `trim*`, `thicken*`, `extend*`, `deleteBody*`.
- Known gaps: trimSurface of curved surfaces (needs SSI), extend shapes
  linear/soft/reflective (separate `X<k>` spline face), freeform marker
  (`deviation`, `edges`, `bound`, `frames`), surface extrude draft / upTo ends,
  rib/lip with curved rim faces or chains that leave the plane.
- rib: thin wall (`thickness`, centred on the sketch plane) between the open
  chain and the body, extended to contact; errors `nothingToCut`,
  `missingSketch`, `invalidValue`, `ribNeedsOpenSketch`, `ribNoContact`.
- lip: band of `width` along the rim edges, `height` out of (lip) / into
  (groove) the rim face; errors `nothingToCut`, `lipFace`, `invalidValue`,
  `missingReference`.
- extrude `twist` (not an oracle option): degrees over the FULL depth, positive
  = counter-clockwise about the sketch normal; the rotation is zero at the
  sketch plane (so `mid` / `flip` twist on either side of it) about the axis
  through `twistCentre` (sketch xy, default the sketch origin; area per slice
  is preserved, so volume = area * depth). Side faces keep the extrude names
  (`F.S.e<k>`, caps `F.C0` / `F.C1`; a closed loop not about the axis is split
  `~1` / `~2`); they are screw surfaces: exact cylinders for circles about
  the axis, interpolated B-splines otherwise (volume error ~1e-8). Cannot be
  combined with `draft` or `upToNext` (`draftUnsupported`). A twisted CUT
  tool whose caps are coplanar with body faces currently fails with
  `booleanFailed`: let it overshoot the body (scale `twist` with `depth`).
- hole `diameter` (not an oracle option; counterbore/countersink only when no table `size` applies, see docs/spec-agent-interface.md "Hole"): plain diameter
  (expr) used instead of `size`/`fit`; `<= 0` -> `holeInvalid`.
