# Errors

The kernel reports problems at four levels: the call itself can fail (a trap), the request can be refused (an error reply), the request can run and report a failure in its result, and a regen can run and report failures per feature. This chapter describes each level, how hints are attached, and lists the codes that the kernel can emit.

## The levels

| Level | Where it shows | Example |
|---|---|---|
| Trap | The call to `ck_request` throws a `WebAssembly.RuntimeError`; `ck_panic` got the text first. The instance is unusable; the host starts a new one ([Call it from Node](../getting-started/node.md)). The native service, when built with the `service` profile, answers HTTP 500 and closes the session (a `release` build aborts the process instead). | an internal panic |
| Error reply | `{"ok": false, "error": "<message>", "ms": ...}`. `error` is always a string. | `noState: nothing regenerated yet` |
| Failure in the result | `ok` is true; the result says the operation failed. | `{"error": "notFound"}`, an `issues` list, an import with `body: null` |
| Feature error | `ok` is true; the regen result has `errors` (and `warnings`) with one entry per failed feature. | `["F4", {"code": "holeInvalid", ...}]` |

The first rule of thumb: `ok: false` means the request was not carried out and nothing changed. A regen with failed features is not a failed request: `ok` is true, the tag is replaced with the partial result, and the failures are data.

## Error replies

The message is free text, but the part before the first colon is stable and names the request or the code, so clients can match on the prefix.

### Envelope and state

| Message | Cause |
|---|---|
| `bad request: the body must be JSON {id, msg}` | The body is not valid JSON. |
| `bad request: msg must be an object` | There is no `msg` key, or it is not an object (a top-level array or string gives the same). |
| `message without a type` | `msg.type` is missing, empty or not a string. |
| `unknown request <type>` | No such request type; the type is echoed as sent. |
| `noState: nothing regenerated yet` | A request that needs a tag was sent for a tag that has no state. Clients match on the prefix `noState`. |
| `invalid part: Format` | `regen`, `matchEdges`, `refDiff` or `checkProfile`/`sketchAnalyze`/`sketchFreedoms` with a `part` that is not a part document (not an object, wrong `format`, or `features` not an array). The tag keeps its state. |

### Arguments of single requests

| Message (prefix) | Request | Cause |
|---|---|---|
| `drag: face, point and target are required` | `drag` | `face` is not a string, or `point` / `target` is not a triple of numbers. |
| `bad sketch frame` | `convert` | `frame` is not an object with numeric triples `o`, `x`, `y`, `z`. |
| `sketchOffset without a distance` | `sketchOffset` | `distance` is missing or not a number. |
| `drawing: a view needs a name (front, back, left, right, top, bottom, iso) or x and y` | `drawing` | A view has neither a known `name` nor both `x` and `y`. |
| `massProps: densities has N entries but the body has M lump(s) (expected M)` | `massProps` | `densities` does not have one entry per lump. |
| `importStep: data must be a STEP file as a string`, `importSat: data (the SAT file text) is required`, `importSldprt: data (the .SLDPRT file as base64) is required`, `importSkp: data (the .skp file as base64) is required` | imports | `data` is missing. |
| `importSkp: opts.up must be "z", "y" or "yFlip", got ...` | `importSkp` | Unknown `opts.up`. |
| `cannot inspect nothing`, `cannot inspect <kind>`, `<kind> not found`, `vertex without a point`, `no vertex` | `inspect` | See [Queries](queries.md#inspect). |
| `<request>: ...` | measurement and analysis requests | A bad or missing argument, named after the request: `angles: edges ('all' \| [keys]), faces [a, b] or draft {pull} is required`, `closestPoint: at most 2000 points per request`, `closestPoint: on must be 'body', 'face', 'edge' or 'vertex'`, `extrema: dir must not be zero`, `probe: at most 2000 points per request` (and rays), `probe: points [[x, y, z], ...] or rays [{origin, dir}, ...] is required`, `probe: ray.dir must not be zero`, `section: plane.normal must not be zero`, `sections: axis must be 'x', 'y', 'z' or [nx, ny, nz]`, `sections: between 1 and 1000 positions`, `symmetry: plane.normal must not be zero`, `thickness: the body has no faces`, `thicknessAt: point [x, y, z] or face id is required`, `clearance: more than 4000 faces`, `maxDistance: nothing to sample`. Entity arguments that cannot be read give `unknown entity ...`, `no lump N (the body has M)` or `expected a face / edge / vertex id, a point, a lump or ...`. |
| `dfmMinWall: threshold must be a positive number` (and the like) | `dfm*` | The required limit is missing or out of range; the message starts with the request name. |
| `checkProfile ...`, `profile needs ...`, `rule <id>: ...` | `checkProfile` | Unknown profile, rule or parameter, a wrong type or a value out of range; the message lists the valid choices. |
| `sketchAnalyze: featureId must be a string`, `sketchAnalyze: sketch F9 not found` (and `sketchFreedoms: ...`) | sketch analysis | Missing or unknown `featureId`. |
| `textBadHeight: ...`, `textEmpty`, `textFontNotLoaded: font "x" is not loaded; ...`, `fontBadData: ...`, `fontBadName`, `fontReserved`, `fontNameConflict`, `fontTooLarge`, `fontTooMany`, `fontSubsetFailed: ...` | text and font requests | `<code>: <message>`; see [Text and fonts](../io/text.md). |
| `unknown thread standard zzz (known: metric, ...)`, `threadData: <key> must be a string` | `threadData` | Unknown standard or size, or a non-string argument. |
| `TODO(rebuild): <what> needs the geometry engine (no hook installed)` | `inspect` (with a body, for kinds other than `sketch`), `importSkp` in a module built without the `skp` feature | The request type exists but this case is not implemented. The wording is misleading: no hook is missing. |

## Failures in a result

These are answered with `ok: true`.

| Result | Request | Cause |
|---|---|---|
| `null` | most body queries | The tag has state but no body. |
| `{ "error": "missingReference" }` | `drag`, `refCurve` | The reference does not resolve, or there is no body. |
| `{ "error": "dragNoComponent" }`, `{ "error": "dragFixed" }` | `drag` | The face belongs to no inserted part; the part is fixed. |
| `{ "error": "notFound" }`, `{ "error": "notCurve" }` | `curvatureAt`, `comb` | Unknown face or edge; a reference that is not a curve. |
| `{ "error": "cutNoEffect" }` | `drawing` (per view) | The section plane misses the body. |
| `{ "error": "offsetTooLarge" }` | `sketchOffset` | An arc or circle would shrink to nothing. |
| `{ "ok": false, "error": { "code", "params" } }` | `bodyBoolean` | The boolean cannot be done; the object has the form of a feature error. |
| `{ "supported": false, "pass": null, "refusal": { "code", "reason" } }` | `dfm*` | The body is not one the rule can judge: `emptyBody`, `nonAnalyticFaces`, `tessellationFailed`. |
| `body: null` and an `errors` entry | imports | The file could not be read. The entry is `["import", { "code", "params" }]` for every importer (codes such as `parse`, `container.magic`, `container.format`, `sat.parse`). See [Import and export](../io/step.md). |
| `issues` list | `validate`, `heal`, `checkProfile` | Findings about a body; see [Topology and geometry checks](../validation/checks.md). |
| `errors` list | `refDiff` | Regen failures of either document as `[featureId, code]`, with the code as a plain string. |

## Feature errors in `regen`

Each failed feature has one entry in `errors`: `[featureId, { "code": "<code>", "params": { ... } }]`. If a feature has no id, the id is the empty string. An id appears once; a later entry for the same feature replaces an earlier one. A failed feature changes nothing: the body is as it was before the feature, and later features run on it.

`params` is an object that depends on the code. These conventions apply to many codes:

- `hint`: one sentence on what to change. Present on the codes whose cause the kernel can name (`holeInvalid`, `booleanFailed`, `invalidValue` for a scale factor, `threadSizeUnknown`, the loft codes, ...). Show it first when reporting the failure.
- `detail`: a sentence naming the faces or edges involved, often with numbers: `F3.B0: spring leaves its face away from its edges`.
- `field`: for `invalidValue` and `missingReference`, which field of the feature is wrong (`thin.thickness`, `regions`, `endRef`).
- Numbers that say how far off the request was: `max` (largest blend radius that fits), `limit`, `minRadius`, `maxThickness`, `maxHeight`, `budgetMs`, `elapsedMs`.
- `maxSize`: `{ "param", "value", "unit" }` on a refusal because a size is too large: `param` is the feature field (a dotted path for a nested one, `thin.thickness`), `value` the largest value of it that builds (the sign of the request kept, so a face pushed in by -12 gets -9.96), `unit` `mm` or `deg`. The `hint` says the same in words. Given only for a value the kernel built with its own checks (or the exact bound of a closed-form check), so a client can set the field to it without parsing text; when no such value is found it is absent. Given today by `shell` (`thickness`), `moveFace` (`distance`), `draft` (`angle`), a drafted `extrude` (`draft`), a thin wall (`thin.thickness`), `hole` (`cboreDepth`, `cskDiameter`) and `knurlBand` (`depth`); not yet by fillet and chamfer.
- `name` and `nearest`: for a body or face name that does not resolve, with up to three existing names that are close.
- `ref`, `candidates`, `hint`: for a face id that no face carries (`missingReference` from `shell`, `thread`, `combine`, `deleteFace`, `moveBody`, `pipeCut`, `fullRound`, the surface features and `projectCurve`). `candidates` lists up to six live face ids, those of the same feature first and the closest name first, or the three closest names of the whole model when the feature has no face left; `hint` says which of the two it is. A repair is a one-field edit: replace `ref` by one of `candidates`.
- `faces`, `gone`, `candidates`, `neighbours`, `hint`: for `missingEdge` on an edge pick `{faces, at}` that no edge joins any more. `gone` lists the names no face carries (with `candidates` as above); when both faces exist, `neighbours` lists the faces the first one now meets.

Because a failure removes a feature's output, the next features that depend on it fail too. The errors are in feature id order as strings, but the cause is the first failing feature in document order.

### Time budgets

`featureBudgetExceeded` and `regenBudgetExceeded` report that a time limit stopped the work. When the limit of a feature (`opts.featureTimeMs`, default two minutes) has run out, a feature that failed at that moment reports `featureBudgetExceeded` instead of its own code, unless its own code already ends in `BudgetExceeded` (a boolean stopped by the limit keeps `booleanBudgetExceeded`); its result is not applied. When the limit of the whole document (`opts.regenTimeMs`) has run out, the running feature and every later one report `regenBudgetExceeded`; the later ones are not run. The `params` are `budgetMs`, `elapsedMs`, `hint` and, if the feature had failed for another reason, `cause` (that code). A feature limit that has already passed when a feature starts (a few microseconds, for instance) is reported as `regenBudgetExceeded` with the feature limit's `hint`. Budgets need a host clock ([Protocol overview](overview.md#time-ms-and-the-host-clock)).

**Budget failures are not cached.** A failure caused by a time budget is not stored in the regen cache, so a later regen of the same part runs again with its own budget. Builds before this change replayed the stored error (and `body: null`) on the same instance; `opts.noCache: true` was the workaround.

### Codes of the driver and of references

| Code | Cause |
|---|---|
| `unknownFeature` | The feature's `type` is not a known type. `params.type` is the type as given (empty if missing). |
| `missingReference` | A reference does not resolve: a face, edge, plane, axis or body name that does not exist, a required reference field that is absent, or a feature that needs a body when there is none. Often has `field`, `face`, `ref` or `name` and `nearest`. |
| `missingSketch` | The feature's `sketch` is absent or names no sketch that was regenerated before it (also when that sketch failed). |
| `badExpression` | A numeric field or constraint value is an expression that does not evaluate: syntax error, unknown name or function, or a variable that is itself invalid or part of a cycle. |
| `invalidValue` | A value is out of range or the wrong shape: a radius, count, angle, factor or distance that must be positive, an unknown option value, a mix of options that does not work together. `params` has `detail`, `field` or `hint`. |
| `noClosedProfile` | The sketch has no closed profile to build from. `sketchAnalyze` says why. |
| `nothingToCut` | The feature modifies a body (cut, shell, rib, draft, mate, ...) but the document has no body yet. |
| `noBodies` | `deleteBody` with no body in the document. |
| `noFaces` | `moveFace`, or `draft` with a valid neutral plane, was given an empty `faces` list. |
| `noSeeds` | A pattern or mirror has an empty `seeds` list or nothing to copy. |
| `unsupported` | An option of a fillet or chamfer that the kernel does not build. `params.what` names it (`chamfer <type>`, `fillet profile <name>`, `fillet <filletType>`). Any `filletType` is refused this way, including `"circular"`: `filletType: "circular"` gives `unsupported { what: "fillet circular" }` although `circular` is also the name of the default `profile`. Leave `filletType` out, or use `"fullRound"`. |
| `TODO(rebuild): <what> needs ck-ops` | The feature type is accepted but not implemented (see [Regeneration](regen.md#limits-and-known-gaps)), or the particular option is refused. The code is this whole sentence. |
| `healed` | Not an error. With `autoHeal`, the feature's body had validation issues and was repaired; `params` has `fixed`, `codes` and `remaining`. It is listed in `errors`. |

The variable table of the part (`variables`) is resolved before the features run. A variable with an invalid name, one that depends on a cycle, and one whose expression does not evaluate are all left unset in the reply, so no code of their own is reported; the features that use them fail with `badExpression`.

### Reference planes and axes

| Code | Cause |
|---|---|
| `planeLineNormal` | The `perpendicular` plane mode: the axis line is parallel to the base plane's normal. |
| `planePointsInLine` | Three points (or the two directions of a `frame`) lie on one line, so they define no plane. |
| `planePointOnLine` | The `linePoint` mode: the point lies on the line. |
| `planesParallel` | Two planes are parallel where their intersection line (an axis, an angle plane) is needed. |
| `planeNotTangent` | The `tangent` mode: the face cannot be tangent to the given plane or line (for example a cylinder axis not parallel to the plane). |
| `missingPath` | The plane mode `normal` or `normalCurve`, or a sweep: the `path` is not given or does not name a usable open sketch. |
| `curvesNotPlanar` | The `curve` plane mode: the curves do not lie in one plane. `params.dev` is the deviation. |
| `curvesStraight` | The curves are straight, so they give no unique plane. |
| `faceNotCylindrical` | The `axis` feature was given a face that is not a cylinder. |

### Extrude, revolve and profiles

| Code | Cause |
|---|---|
| `extrudeRegionNone` | `regions` matched no closed region of the sketch (an empty list, or ids that are no region's outer boundary). It never falls back to "all regions". `params.regions` lists them. |
| `extrudeRegionUnknown` | `regions` names ids that are no entity of the sketch. `params.ids`. |
| `unknownEnd` | The `end` condition is not a known one. `params.end`. |
| `upToFaceBehind`, `upToPointBehind` | The end face (or the plane through the end point) is not ahead of the whole profile in the extrusion direction, or is parallel to it. |
| `upToFaceOutside` | `upToFace`: the face does not cover the whole profile along the direction. `params.face`. |
| `upToFaceUnsupported` | The end face is not planar. `params.face`. |
| `endClipFailed` | The boolean that trims a slanted end failed. By default it appears only as the `reason` of an `endConditionFallback` warning; with `strictEnd: true` it is the error. `params.detail`. |
| `upToNextNone` | `upToNext` cut: no face to stop at. |
| `upToNextOpen` | `upToNext` add: the tool never meets the body. |
| `missingAxis` | A revolve has no axis, or its axis does not resolve to a line of the sketch. |
| `profileCrossesAxis` | A revolve profile straddles the axis. `params.detail`. |
| `thinTooThick` | A thin-wall extrude or revolve: the thickness is too large for the profile (a corner or curvature limit). `params`: `limit` (`curvature` or `corner`), `minRadius`, `maxThickness`, `thickness`, `face`. |
| `offsetFitFailed` | The offset curve of a thin-wall profile could not be fitted within tolerance. |
| `zeroDepth` | An extrusion or twisted extrusion with no depth (below 1e-9). The `extrude` feature itself replaces a depth of zero or less by 1e-6, so this code shows only on paths that do not; the `extrude` feature raises the warning `extrudeZeroDepth` instead (see [Warnings](#warnings)). |
| `emptyProfile`, `openProfile` | The profile has no region or is not closed. |
| `extrudeDirectionInPlane` | The extrusion direction lies in the sketch plane. |
| `zeroAngle`, `revolveAxisNotInPlane`, `revolveDegenerate` | A revolve with a zero angle, an axis that is not in the sketch plane, or a profile that is degenerate about the axis. |
| `draftTooLarge` | A drafted extrude: the taper closes to a point, or an edge turns over. `params.detail`; for an extrude of one circle `params.maxHeight` is the greatest depth that works. |
| `draftUnsupported` | A twisted extrude combined with a draft or `upToNext`; a `draft` feature whose direction is skewed against the neutral plane or whose neutral plane cuts through the profile; or a face that is not a plane, a cylinder or a cylindrical cone (`params.detail` names the face and its surface kind). |
| `twistUnsupported` | A twisted extrusion whose direction is not the sketch normal. |

Two warnings of the end conditions are in [Warnings](#warnings): `endConditionFallback` (an unreachable end reference ran as a blind extrude of `depth`, unless the feature sets `strictEnd: true`, which makes the four unreachable codes above errors) and `endRefRelinked`.

### Booleans and bodies

| Code | Cause |
|---|---|
| `booleanFailed` | The result of a boolean could not be assembled into a valid solid. `params.hint` suggests moving the tool by about 0.01, uniting touching bodies first, or giving the feature a fuzzy `tolerance` / `autoFuzzy: true`. When the cause is known: `hostFace`, `toolFace`, `gap` (two nearly coincident free-form walls); or `faces`, `issue` (`openEdge`, `nonManifoldEdge`, `badPairing`) and `at` (the faces and a point at the first bad edge). |
| `booleanTangent` | The result has an odd Euler characteristic: the operands touch tangentially. `params`: `euler`, `hint`. |
| `booleanBudgetExceeded` | The boolean stopped retrying because it had already run long. `params`: `budgetMs`, `elapsedMs`, `hint`. |
| `booleanVolumeMismatch` | The boolean produced a valid body whose volume or box is impossible for the operands (material dropped or added); the body is left as before. `params`: `what` (`volume` or `box`), `expected_min`, `expected_max`, `got` and `hint`. |
| `cutNoEffect` | A cut that removes nothing (a tool that misses the body). The body is unchanged. It is reported in `errors`. |
| `combineEmpty` | `combine` `intersect` of lumps that do not overlap. |
| `bodyNameConflict` | A `body` name that a live body already has. `params`: `name`, `features` (the feature that made that body and the one that asked for the name). |
| `ambiguousBody` | `combine.target` names several bodies. `params`: `name`, `candidates` (`{ lump, face }`). |

### Holes and threads

| Code | Cause |
|---|---|
| `holeNoPoints` | The hole's sketch has no point entities. |
| `holeInvalid` | No usable hole size: an unknown `size`, a non-simple hole without a `size`, or a diameter that is zero or negative. `detail` and `hint` say which. |
| `holeCboreInvalid` | The counterbore diameter is not larger than the hole, or its depth is not positive or too deep. `params`: `cboreDiameter`, `cboreDepth`, `holeDiameter`, `hint`. |
| `holeCskDiameter` | The countersink head diameter is not larger than the hole, or the cone angle is outside 0 to 180 degrees. |
| `threadSizeUnknown` | The thread `standard` or `size` is not in the tables. `params`: `standard`, `size`, `suggestions` (the nearest sizes, or the known standards), `hint`. |
| `threadFaceNotCylinder` | The `thread` face is not cylindrical. `params`: `face`, `surface`, `hint`. |

### Patterns, moves and pipes

| Code | Cause |
|---|---|
| `moveFaceFailed` | A face move or offset produced no valid body. `params.detail`; for a curvature or thin-gap limit also `limit`, `minRadius`, `maxThickness`, `face`. |
| `moveFaceUnsupported` | A face move of a surface or option the kernel does not offset. |
| `alignPipeCutOneBody` | `pipeCut` edges that belong to different bodies. |
| `pipeCutFailed` | The pipe cannot be built: an edge without length, or an edge type that is not supported. `params.detail`. |
| `pipeCutNothingLeft` | The pipe removes every face. |
| `pipeCutRadius` | The pipe radius is larger than the bend radius of an edge, so the tube folds over itself. `params.max` is the largest radius that does not. |

### Assemblies and mates

| Code | Cause |
|---|---|
| `insertPartMissing` | `insertPart` without a valid `part` document. |
| `insertPartErrors` | The inserted part has errors. `params.detail` is `<feature>: <code>` of the first one. |
| `mateConflict` | The solver rejected the mate; it is dropped and the components return to the pose before it. `params.mates` lists the mate and its neighbours. |
| `mateGeometry` | The faces or edges of the mate have no geometry the mate kind can use. |
| `mateSameBody` | Both sides of the mate belong to the same component. |
| `mateFixed` | Both components are fixed, or the mate needs a floating component (for `rackPinion` the rack). |

### Ribs and lips

| Code | Cause |
|---|---|
| `ribNeedsOpenSketch` | The rib's sketch is not one open chain. |
| `ribNoContact` | The rib's profile does not touch the body. |
| `lipFace` | `lip` was given no faces, or none of them exist. |

### Blends: fillet, chamfer

| Code | Cause |
|---|---|
| `missingEdge` | A fillet or chamfer pick `{faces, at}` does not resolve to an edge. |
| `blendFailed` | The blend could not be built. `params.detail`. |
| `blendOverflow` | The blend leaves its faces: the radius is too large for the faces it rolls on. `params.detail`; `max` is the largest radius that fits, when it can be computed. |
| `blendG2TooLargeMax` | The same for a curvature-continuous fillet. `params`: `max`, `shape`. |
| `blendSize` | No blend fits between the two faces. |
| `variableRadiusMismatch` | A variable-radius fillet gives two different radii at one shared vertex. `detail` names both radii and the vertex. |
| `blendCorner` | The corner patch of a vertex with several blended edges cannot be built (the edges are not perpendicular). `params.hint`: fillet concave and convex edges in separate features. |
| `blendNonManifold` | A free-form blend met an edge that does not have exactly two faces. |
| `blendTolerantEdge` | An edge of imported geometry carries a tolerance, so there is no exact intersection to roll along. `params.detail` names the edge and the tolerance. |
| `blendTooLarge` | A vertex chamfer distance reaches the far end of an edge, or the contact line of an interpolated blend falls outside its face. |
| `blendUnsupported` | A blend on an edge type or configuration that is not built (a vertex chamfer on curved edges). |
| `chamferVertexEdges` | A vertex chamfer needs a vertex of exactly three edges. |
| `filletRho` | A conic fillet `rho` outside 0 to 1. |
| `filletShape` | A curvature-continuous fillet `shape` outside the allowed range. |
| `fullRoundUnsupported` | A full-round fillet with the same face twice, or a centre face that is not adjacent to both sides. |

### Shell, offset and draft

| Code | Cause |
|---|---|
| `shellTooThick` | The wall is too thick: an offset passes the centre of curvature of a face (`limit: "curvature"`), or two faces are closer than twice the wall (`limit: "thin"`). `params`: `detail`, `limit`, `minRadius`, `maxThickness` (a safe thickness), `face`. |
| `shellFailed` | The shell could not be built; curvature was not the limit, or the boolean failed. |
| `shellUnsupported` | An opened face of a shell is not planar. |
| `offsetUnsupported` | A shell of a surface or edge type that the offset cannot rebuild. `params.detail`, and a `hint` when the cause is a fillet on a drafted edge (shell before the fillet, or remove the draft). A face move reports the same cause as `moveFaceUnsupported`. |
| `offsetTooLarge` | A shell offset reaches the centre of curvature of a face. Carries the same limit params as `shellTooThick`. (`sketchOffset` uses the same name for an arc or circle that would vanish, in its result.) |
| `offsetDegenerate` | An offset met an edge that reverses (`detail`: "an edge reverses"). The offset code is `offsetDegenerate`; a `shell` reports it as `shellTooThick`, and a face move reports a failed offset as `moveFaceFailed` with this text in `detail`. |
| `draftParallelFace` | A draft of a face parallel to the neutral plane. |
| `draftTangentFaces` | Two faces of the draft are tangent to each other. |
| `draftAxisNotParallel` | A cylinder or cone face whose axis is not parallel to the pull direction. |
| `draftFailed` | The draft could not be built; no interior point on a face. |

### Loft, sweep and helix

| Code | Cause |
|---|---|
| `loftProfiles` | Fewer than two usable profiles. |
| `loftProfile` | A loft section sketch does not give exactly one profile: it has several closed profiles, or (for a surface loft) is not one open chain. |
| `loftProfileOpen` | A solid-loft section is not closed. |
| `loftProfileNotPlanar` | A section is not planar. |
| `loftProfileSelfIntersect` | A section's outline crosses itself. `detail`, `hint`. |
| `loftSectionTooSmall` | The same, with a section small against the loft length. |
| `loftInvalidResult` | The solid built from the sections failed validation under every matching strategy. `detail`, `hint`. |
| `loftFailed` | The loft could not be built (for example guide curves with a point section, an unsupported guide influence). `detail`, `hint`. |
| `loftTangentEdges` | Any truthy `startTangent`, `endTangent`, `startContinuity` or `endContinuity` on a loft. The check runs before the loft is built and always fails: tangency to adjacent faces is not available. Remove the fields. |
| `loftGuideInvalid` | A guide cannot be used: it is not a curve or has no length, two guides touch a profile at the same point, or the guides cannot be matched to the sections. `detail` says which. |
| `loftGuideMiss` | A guide misses one of the profiles (`detail`: "the guide misses profile N"). |
| `loftGuideOrder` | A guide meets the profiles out of order, or the guides meet the profiles in different orders. |
| `loftCentreLineInvalid` | The centre line is not a single open chain of supported curves in a sketch. |
| `loftCentreLineKink` | The centre line has a corner; its pieces must run on tangentially. |
| `loftCentreLineTight` | The centre line bends too tightly for the sections. |
| `loftCentreLineReach` | The centre line does not reach a profile. |
| `loftCentreLineApex` | A curved centre line with a point (apex) end. |
| `loftCentreLineGuides` | A curved centre line together with guide curves. |
| `sweepPathInvalid` | The path sketch is not one connected open chain of supported curves. |
| `sweepProfileNotNormal` | The profile plane is not nearly perpendicular (within 15 degrees) to the path at its start. |

| `sweepFailed` | A guided sweep whose profile is not valid. |
| `sweepGuidesCount` | More than two guides. |
| `sweepGuideStretchTwo` | Stretch mode needs exactly two guides. |
| `sweepGuideNotTouching` | A guide does not touch the profile. |
| `sweepGuideNotAlong` | A guide does not advance along the path direction at all, so the section planes never meet it. |
| `sweepGuideShort` | A guide advances along the path but ends before a section plane reaches it. |
| `sweepGuidesSamePoint` | Two guides meet the profile at the same point, or coincide at a section. |
| `sweepGuideDegenerate` | A guide is not a curve or has no length. |
| `sweepGuideOnPath` | A guide lies on the path itself at a section, so it gives nothing to scale or turn by. |
| `sweepGuideMissing` | A guide reference does not resolve to a curve. |
| `sweepGuidesTwist` | Guides together with a twist. |
| `helixNeedsCircle` | The helix sketch has no circle to take the radius from. |

### Curves

| Code | Cause |
|---|---|
| `editCurveNoInput` | `editCurve` with no `curves`. |
| `editCurveInputMissing` | A curve of the list does not exist. `params.curve`. |
| `editCurveDegree` | The degree is outside 2 to 9. `params`: `min`, `max`. |
| `editCurveFitFailed` | No curve could be fitted. |
| `editCurveNotChain` | The curves do not form one chain. |
| `editCurveNotSmooth` | The chain has corners and corners were not kept. `params.angle`. |
| `editCurveTooFewPoints`, `editCurveTooManyPoints` | The control point count is below its minimum or above the limit. |
| `editCurveWeight` | A weight is not positive. `params.entry`. |
| `editCurveNoPlane` | The planar option without a plane. |
| `projectCurveNoFaces` | `projectCurve` with no body or sheet to project onto. |
| `projectCurveNoInput` | No `curve` or `sketch` given, or fewer than two points. |
| `projectCurvePlane` | The input curve does not define a projection plane. |
| `projectCurveMisses` | The projection hits no face. |
| `projectCurveSeveral` | The projection gives several separate chains; `params`: `count`, `points`. |
| `projectCurveFailed` | The projected curve could not be fitted. `params.detail`. |

### Surfaces

| Code | Cause |
|---|---|
| `knitNotClosed` | `knit` with `solid` requested, but the sheets do not close; `params` lists the open points. |
| `knitNearMiss` | `knit`: two edges that should be joined diverge by more than 100 tolerances at their midpoints (`detail`: "edges diverge: ..."). |
| `fillInvalid` | The edges of a `fill` do not form one chain, or the patch is not a valid face. |
| `fillNotClosed` | The chain of a `fill` is open. |
| `fillFailed` | The patch of a `fill` is not a valid face (a parameter area of zero). `detail`. |
| `extendNotBoundary` | `extend` of an edge that is not an open boundary edge. |
| `extendNoTarget` | The extension never reaches its target. |
| `extendPointBehind` | The extension target lies behind the edge. |
| `extendFolds` | The extension would fold. |
| `extendUnsupported` | The sheet or edge is not one the extension handles: a boundary shared with other faces (only single-face sheets are extended), or an edge that is not a straight line of a planar sheet. |
| `freeformOneFace` | A `freeform` edit with control point edits on more than one face. |
| `freeformEntryInvalid` | An edit entry is invalid. `params.entry` (1-based). |
| `freeformEntryPlane` | An edit entry has no usable plane. `params.entry`. |
| `freeformCrosses` | The edited surface would cross other faces. |
| `freeformNotSymmetric` | A symmetric edit whose control net is not symmetric. |
| `rebuildPeriodicFace` | `freeform` rebuild of a periodic face. |
| `rebuildTooFine` | The rebuild asks for more than 50 spans. |
| `rebuildFailed` | The rebuild could not fit a surface. |
| `thickenFailed` | The thickened body is not closed. |
| `thickenReach` | The thickening offset cannot reach across an edge. |
| `thickenTooThick` | An edge vanishes at this thickness. |
| `trimNoIntersection` | The tool does not cut the surface. |
| `trimPieceLoop` | The kept piece of a face could not be closed into a loop. The trim is not applied. |
| `trimNothingLeft` | Nothing remains on the chosen side. |

## Warnings

`warnings` has the same shape as `errors` and means the feature ran. A client shows them as notes.

| Code | Cause |
|---|---|
| `booleanFuzzy` | The result depends on a fuzzy tolerance. `params.tolerance` is the value that decided it. |
| `endConditionFallback` | An unreachable end reference ran as a blind extrude. `params`: `end`, `reason`, `fallback`, `depth`, and the params of the reason. |
| `extrudeZeroDepth` | A `blind` or `mid` extrude has a depth of zero or less, or none (also when an expression evaluates to 0). It was built as a sliver of 1e-6 mm, as the reference does, which is not a usable solid and can make a later union fail with `booleanFailed` / `openEdge`. `params`: `depth`, `built`, `hint`. |
| `endRefRelinked` | The end face id no longer exists; the face was found again. `params`: `from`, `to`. |
| `holeDowngraded` | A counterbore or countersink was drilled as a simple hole because its step or cone dimension is missing. `params`: `requested`, `built`, `missing`, `detail`, `hint`. |
| `threadDiameterMismatch` | The hole or shaft diameter does not suit the thread. `params`: `expected`, `actual`, `size`, `standard`, `internal`, `nearest`, `hint`. |
| `threadModelFallback` | A modelled thread could not be built and stays cosmetic. `params`: `reason`, `hint`. |
| `threadNotCarried` | A pattern or mirror copy could not carry a thread over. `params`: `thread`, `face`, `reason`, `hint`. |
| `cornerNotTangent` | Blended edges meet at a corner that is not tangent. `params.deviation` in degrees. |
| `filletRadiusApproximated` | A fillet used an approximated radius. |
| `unknownField` | A feature carries a field its type does not read, usually a misspelling. `params`: `fields` (the names, in document order), `suggest` (only for names close to a known field: `{"depht": "depth"}`), `detail`, `hint`. See [misspelled fields](../validation/diagnostics.md#misspelled-fields). |
| `textTooSmall`, `textMissingGlyph`, `textTruncated`, `textOverlappingContours`, `textOverlap`, `textSpacingAdjusted` | Text sketch notes; see [Text and fonts](../io/text.md). `params`: `entity`, `detail`. |

## Text and font codes

Sketch features with text can fail with `textEmpty` (empty string), `textNothingToDraw` (only blanks or unknown letters), `textFontNotLoaded`, `textFontMismatch` (`fontHash` does not match the loaded file), `textBadHeight` (not a number in 0 to 1e5), `textBadArc` (radius not positive) and `textBadValue` (a position, angle or spacing is not finite, or spacing is outside 0 to 20). Loading a font can warn with `fontKerningIgnored`, `fontLigaturesIgnored`, `fontVariableDefaultOnly`, `fontCollectionFirstFace`, `fontCffApproximated`, `fontNoCapHeight`, `fontOverlappingContours`, `fontEmbeddingRestricted`, `fontNoSubsetting` and `fontNoLatin`. [Text and fonts](../io/text.md) gives the causes.

## Notes on specific codes

- An `insertPart` whose nested part is empty or has no solid body is not an error: it adds no component.
- The `badName` and `cyclicVariables` codes are produced inside the variable resolver but are not put in the reply (see above).
- `healed` is listed under `errors` although it marks a successful repair.
- `refDiff` reports regen failures as `[featureId, code]` with a string code, not the `{ code, params }` object.
- An import of a SAT file reports its failure as an object in `errors`, not the pair form.

## See also

- [Protocol overview](overview.md)
- [Regeneration](regen.md)
- [Queries](queries.md)
- [Feature errors and diagnostics](../validation/diagnostics.md)
- [Known limits](../quality/limits.md)
