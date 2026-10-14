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
| sweep | op, sketch, path ({kind: curve or sketch, id}; a `sketch` id may name a `sketch3d` holding one `spline`, tangent-constrained end lines allowed), twist (deg), (+guides, guideMode, surface: not reconstructed) |
| loft | op, sketches[], surface, guides[{sketch, influence next only}], centreLine ({sketch}, straight only), startNormal / endNormal (tension; `true` = 1), connectors[null\|fraction]; startTangent / *Continuity -> `loftTangentEdges` (sketch profiles) |
| fillet | radius(expr), edges[] |
| chamfer | chamferType(equal/angleDistance/distanceDistance/vertex), distance, angle, distance2, edges[] |
| draft | neutral, faces[], angle(expr), flip |
| shell | faces[], thickness(expr) |
| hole | sketch, standard(iso), size, diameter (simple holes only: plain diameter instead of size), fit(normal/tap), kind(simple/cbore/csink), end, depth, tip, flip |
| helix | sketch, definedBy(pitchRevs/heightRevs/heightPitch), pitch, revolutions, height, startAngle, leftHand, reverse |
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
| deleteBody | bodies[] |
| pipeCut | radius(expr), edges[{faces,at}], faces[] (all their rim edges); the picks may sit in the solid or in a surface body; the result is a SURFACE body registered under the feature id (solid disappears) |

## Error strings (kernel.strings, verbatim per feature)

- sweep: sweepPathInvalid, sweepProfileNotNormal, sweepGuideMissing,
  sweepGuidesTwist, sweepGuideShort, sweepGuideStretchTwo, sweepSelfIntersects, sweepFailed
- loft: loftProfileOpen, loftProfileChains, loftGuideInvalid, loftGuideMiss,
  loftGuideOrder, loftGuideTangency, loftCentreLineInvalid/Tight/Reach/Kink, loftFailed
- shell: shellTooThick, shellUnsupported, shellFailed, offsetUnsupported
- draft: draftParallelFace, draftAxisNotParallel, draftUnsupported,
  draftTangentFaces, draftTooLarge, draftFailed
- hole: holeNoPoints, holeInvalid, holeClose/Loose (fit)
- rib: ribNeedsOpenSketch, ribNoContact
- knit: knitNotClosed, knitNearMiss, knitFailed, knitted
- fill: fillNotClosed, fillInvalid, fillFailed, fillCrosses, fillContinuity
- thicken: thickenFailed, thickenPointedPole, thickenReach
- pipeCut: pipeCutRadius, pipeCutFailed, pipeCutNothingLeft, alignPipeCutOneBody
- project: projectCurveNoFaces, projectCurveMisses, projectCurveSeverals,
  projectCurveFailed, projectCurveTolerance, projectCurvePlane/Line/Curve
- extend: extendNotBoundary, extendTargetSelf, extendOneSurface,
  extendUnsupported, extendReflectTooFar, extendPointBehind, extendFolds
- blends: filletChamfer, chamferVertexEdges(two/points), chamferType+filletType,
  "edge of blend X is gone after an earlier blend", "X collapses and cannot be removed"
- assembly: mirrorOntoItself, mirrorNotKnitted
- boolean: booleanFailed, cutNoEffect; `booleanVolumeMismatch` (ours, not in kernel.strings):
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
- hole `diameter` (simple holes only, not an oracle option): plain diameter
  (expr) used instead of `size`/`fit`; `<= 0` -> `holeInvalid`.
