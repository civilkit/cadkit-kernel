# features/

One module per feature family (file layout = rs_paths.txt). Each module
implements: payload parsing (ck-json), validation with the EXACT error
strings from kernel.strings, and dispatch to ck-ops/ck-topo.

## Payload field table (confirmed from .ckpart samples + app.pretty.js)

| type | fields |
|---|---|
| sketch | plane, points[{id,x,y,fixed}], entities[{type:id,...}], constraints[{type,refs,value,offset}], seq, visible |
| extrude | op(add/cut), sketch, end(blind/through/throughBoth/upToNext/mid), depth(expr), flip, draft(expr), draftOut |
| revolve | op, sketch, axis, end, angle(expr), flip |
| sweep | op, sketch, path ({kind: curve or sketch, id}), twist (deg), (+guides, guideMode, surface: not reconstructed) |
| loft | op, sketches[], surface, centreLine, guides |
| fillet | radius(expr), edges[] |
| chamfer | chamferType(equal/angleDistance/distanceDistance/vertex), distance, angle, distance2, edges[] |
| draft | neutral, faces[], angle(expr), flip |
| shell | faces[], thickness(expr) |
| hole | sketch, standard(iso), size, fit(normal/tap), kind(simple/cbore/csink), end, depth, tip, flip |
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
