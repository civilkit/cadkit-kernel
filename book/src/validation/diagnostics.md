# Feature errors and diagnostics

When a feature cannot be built, the regeneration reply says which one and why. This chapter explains how errors are produced, how to read the `errors` array, how to tell a root cause from a consequence, and what the main codes mean and what to try.

## Where errors come from

Three layers produce them.

1. **The regeneration driver** (`ck-model::regen`) checks what is true of every feature before any geometry runs: the type is known (`unknownFeature`), a sketch that a feature names exists and was regenerated before it (`missingSketch`), a nested part is present (`insertPartMissing`), and the time budget has not run out (`featureBudgetExceeded`, `regenBudgetExceeded`).
2. **The feature layer** (`ck-model::features`) reads the feature's fields, evaluates expressions, and resolves references. It produces the codes about input: `badExpression`, `invalidValue`, `missingReference`, `missingEdge`, `noClosedProfile`, `nothingToCut`, and the per-feature codes (`holeInvalid`, `ribNoContact`, ...). It also maps the failures of operations into errors, adding a `hint` where it knows a remedy.
3. **The operations** (`ck-ops`) return a code and a `detail` when the geometry cannot be built (`blendOverflow`, `shellTooThick`, `booleanFailed`, `loftFailed`, ...). The feature layer copies them into the error's parameters.

A failure never leaves a half-built body: the operations either return a new body or an error, and the body of the state stays as it was. See [regeneration](../kernel/regeneration.md).

## The shape of an error

```json
["F3", { "code": "blendOverflow",
         "params": { "detail": "F3.B0: spring leaves its face away from its edges" } }]
```

| part | meaning |
|---|---|
| `"F3"` | The feature id. The empty string when a failure belongs to no feature. |
| `code` | A short camelCase identifier; the part to switch on. |
| `params` | Anything else that helps: `detail` (a sentence about what failed, often naming faces), `hint` (what to change), and code-specific numbers (`max`, `limit`, `maxThickness`, `faces`, `at`, `gap`, `budgetMs`, `elapsedMs`, `cause`, ...). The object is empty when the code says it all. |

The code `notImplemented` marks a feature or option that is not built yet (for example a twisted sweep along a multi-piece path, or the `dome` feature). Its `what` parameter names the feature or option. The code is stable; the set of cases that return it shrinks as they are built.

A second array, `warnings`, has the same shape. A warning does not stop the feature: the body is built and the warning says that something was approximated or chosen. Warnings appear in the reply only when there is at least one.

## Misspelled fields

A field that a feature does not read is ignored, so a misspelling leaves the default in force: `"depht": 10` on an `extrude` builds the 1e-6 slab of `depth: 0`, with no error. For the feature types in the table below, the driver compares the feature's fields with the names the type reads and adds an `unknownField` warning per feature that has others.

```json
["F2", { "code": "unknownField",
         "params": { "detail": "extrude has no field `depht`; it is ignored",
                     "fields": ["depht"],
                     "suggest": { "depht": "depth" },
                     "hint": "`depht` -> `depth`" } }]
```

The warning above is what the kernel returned for an `extrude` of a 10 x 10 square sketch that says `"depht": 10` and no `depth`.

- It is a warning, never an error: the feature runs exactly as it would without the extra field, and the rest of the reply is unchanged. A part with no unknown fields has no `warnings` entry for this.
- `fields` lists the unknown names in document order. `suggest` has an entry only for names within two edits of a known field (a swapped pair of letters counts as one edit; so does a difference of case).
- Always accepted, for every feature: `id`, `type`, `num`, `name`, `suppressed`, the editor fields `group`, `autoName`, `visible`, `userHidden` and `after`, and the boolean fields `tolerance` and `autoFuzzy`. A `sketch` also carries the editor fields `seq`, `prop` and `gear`. Keys inside a field's value (the members of `thin`, `endRef` or an edge pick) are not checked.
- A suppressed feature is not checked. A feature type that is not in the table is not checked; the table starts with the most used types and grows with the format.

| Type | Fields it reads |
|---|---|
| `sketch` | `plane`, `points`, `entities`, `constraints` |
| `extrude` | `sketch`, `op`, `end`, `depth`, `flip`, `draft`, `draftOut`, `twist`, `twistCentre`, `regions`, `thin`, `endRef`, `offset`, `strictEnd`, `body` |
| `revolve` | `sketch`, `op`, `axis`, `angle`, `end`, `flip`, `regions`, `thin`, `body` |
| `fillet` | `radius`, `edges`, `profile`, `propagate`, `filletType`, `radii`, `rho`, `rhos`, `shape`, `side1`, `center`, `side2` |
| `chamfer` | `chamferType`, `distance`, `distance2`, `distance3`, `angle`, `face`, `flip`, `propagate`, `edges` |
| `hole` | `sketch`, `standard`, `size`, `fit`, `kind`, `diameter`, `end`, `depth`, `tip`, `flip`, `cboreDiameter`, `cboreDepth`, `cskDiameter`, `cskAngle` |

The list lives in one table in `ck-model` (`features/known_fields.rs`); a test checks that no part of the test corpus produces the warning.

## Reading the errors array

- **Sorting.** Errors are sorted by the feature id compared as a string, so `F10` comes before `F3`. There is one entry per feature; a second error for the same feature replaces the first. To read them in the order the features ran, sort by the feature's position in the document.
- **Features are independent.** After a failure the run goes on with the unchanged body. The reply has no link between an error and the feature that caused it, and no `blockedBy` field.
- **Root cause and consequences.** Take the errors in feature order. The first one is a root cause. A later error is probably a consequence if it has one of the codes `missingReference`, `missingEdge`, `missingSketch`, `nothingToCut` or `noBodies`, and the thing it cannot find is made by an earlier failed feature. Errors of another kind (a different feature failing for its own geometric reason) are independent roots. Fix the first root and regenerate; the consequences usually disappear.
- **A failed feature that nothing depends on** is not a consequence of anything and does not hide later features. In the run below F5 is built although F3 and F4 failed.

A run on a 40 x 20 x 10 block:

```json
[
  { "type": "fillet",  "id": "F3", "radius": 30, "edges": [ { "faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10] } ] },
  { "type": "chamfer", "id": "F4", "chamferType": "equal", "distance": 1,
    "edges": [ { "faces": ["F3.B0", "F2.C1"], "at": [20, 2, 10] } ] },
  { "type": "chamfer", "id": "F5", "chamferType": "equal", "distance": 1,
    "edges": [ { "faces": ["F2.C1", "F2.S.e3"], "at": [20, 20, 10] } ] }
]
```

```json
{ "errors": [
    ["F3", { "code": "blendOverflow", "params": { "detail": "F3.B0: spring leaves its face away from its edges" } }],
    ["F4", { "code": "missingEdge", "params": {} }]
  ] }
```

F3 is the root: a radius of 30 does not fit a 10 mm edge, so no face `F3.B0` exists. F4 refers to `F3.B0` and fails with `missingEdge`: a consequence. F5 is independent and builds (the body has 7 faces). With F3 suppressed, F4 still fails.

## The debug trace

A request with `"debug": true` returns a `trace` array of spans: one per feature with the result, nested spans inside (one per stage of a boolean, one per blend), and for failures a span that names the code:

```json
[
  { "at": 0, "ms": 0, "depth": 1, "label": "F3 fillet: blendOverflow {\"detail\":\"F3.B0: spring leaves its face away from its edges\"}", "level": "span" },
  { "at": 0, "ms": 0, "depth": 2, "label": "F3: Fillet(30.0, Circular) of 1 edge", "level": "span" },
  { "at": 0, "ms": 0, "depth": 2, "label": "F3 failed: blendOverflow {\"detail\":\"F3.B0: spring leaves its face away from its edges\"}", "level": "span" }
]
```

(`at` and `ms` are 0 here because the host in this run supplied no clock; a real host gives milliseconds.)

Successful features read `F2 extrude → 6 faces`. The trace is the quickest way to see where time went and which stage failed. Requests with `debug` bypass the regeneration cache.

## Hints and repair advice

The feature layer attaches a `hint` to codes where the remedy is known. Examples (the text is part of the reply):

| code | hint, shortened |
|---|---|
| `booleanFailed` | The operands' intersection could not be closed; usually tangent faces, coincident faces or a sliver thinner than 1e-3 mm. Move the tool by 0.01, unite bodies that touch first, split the tool, or give the feature a fuzzy `tolerance` (1e-5 to 1e-3) or `autoFuzzy`. |
| `booleanVolumeMismatch` | The result is valid but its volume is impossible for these operands; a circle or edge of the tool lies exactly on the other solid's wall. Nudge the tool by 0.01, change the feature order, or split the tool. |
| `holeInvalid` | `give a standard size (e.g. M6), or a plain diameter` |
| `invalidValue` with `newBody` on a feature that does not support it | `newBody is accepted by extrude, revolve, loft and sweep; build the solid with one of them (op newBody), then use combine, moveBody or a pattern` |
| `featureBudgetExceeded` | Reduce the amount of geometry the feature works on (pattern count, number of edges), split it, or raise `featureTimeMs`. |

The `verify` function of the agent driver package (`packages/driver`) wraps `regen`, `validate` and `check` in one call, and turns each error and issue into a `{feature, code, hint}` record with advice for about thirty codes. It is a client-side layer; the kernel itself returns the codes and parameters described here.

## The main codes

### Input and references

| code | meaning | try |
|---|---|---|
| `unknownFeature` (`type`) | The feature type is not one the kernel knows. | Check the spelling against the [feature list](../format/features.md). |
| `missingSketch` | `sketch` names no earlier sketch. | Point it at an existing sketch that comes before the feature. |
| `missingReference` | A face, plane, body or feature that the field names does not resolve. | Use the names in the latest `body.faces`; see [persistent names](../kernel/naming.md). `describe` suggests the nearest names. |
| `missingEdge` | An edge pick `{faces, at}` finds no edge between the two faces, or the selection is empty. | Check both face names exist and share an edge. |
| `missingAxis` | A revolve names no axis line. | Give the id of a line entity of the sketch. |
| `badExpression` | An expression in a field cannot be evaluated: unknown variable, syntax error, cyclic variables. | Check `variables` and the field. |
| `invalidValue` | A number is outside its range (radius or thickness not positive, unknown option), or an option the feature does not support. `detail` and `hint` explain. | Fix the value. |
| `noClosedProfile` | The sketch has no closed region to extrude or fill. | Close the profile; check for gaps between entities. |
| `nothingToCut` | The feature needs a body and none exists yet. | Put a body-making feature before it. |
| `noBodies`, `noFaces`, `noSeeds` | A delete, move or pattern has nothing to act on. | Check the selection. |
| `unsupported` (`what`) | A variant of a feature the kernel does not implement. | Use another variant. |

### Booleans

| code | meaning | try |
|---|---|---|
| `cutNoEffect` | A cut or intersection changed nothing: the tool misses the body, or only touches it. | Flip the direction (`flip`), move the sketch so it overlaps the body, or use `throughBoth`. |
| `booleanFailed` | The result did not close. Parameters may carry `faces`, `issue`, `at`, or `hostFace`, `toolFace`, `gap`. | See the hint above. |
| `booleanTangent` (`euler`) | An exact tangency leaves an odd Euler characteristic. | Move one operand or widen the model tolerance. |
| `booleanVolumeMismatch` | See the hint above. | |
| `booleanBudgetExceeded` | A failing boolean had already run long (12 s) when the retries would start. | As for `booleanFailed`. |
| `booleanFuzzy` (warning, `tolerance`) | The fuzzy value that decided the boolean. | Information. |

### Solids from sketches

| code | meaning |
|---|---|
| `extrudeRegionNone`, `extrudeRegionUnknown` | The `regions` selection of an extrude or revolve matches no closed region (an empty list never falls back to "all regions"), or names ids that are no entity of the sketch. |
| `unknownEnd` | The `end` is not a known end condition. |
| `upToFaceBehind`, `upToFaceOutside`, `upToFaceUnsupported`, `upToPointBehind`, `endClipFailed` | An up-to-face or up-to-point end cannot be reached or is not planar. The extrude falls back to a blind depth and warns with `endConditionFallback`; `endRefRelinked` warns that the face named by the end reference no longer existed and was found again by its `~k` family or its `at` anchor (`from`, `to`). |
| `upToNextNone`, `upToNextOpen` | Up-to-next finds no face to stop at, or an up-to-next add whose tool never meets the body. |
| `loftProfiles`, `loftProfileOpen`, `loftProfileNotPlanar`, `loftGuideMiss`, `loftCentreLineReach`, `loftFailed`, ... | See [operations](../kernel/operations.md#loft) for the list. Fewer than two sketches is `loftProfiles`. |
| `sweepProfileNotNormal`, `sweepPathInvalid`, `sweepGuide...`, `sweepFailed` | The profile plane must be perpendicular to the path at its start. See [operations](../kernel/operations.md#sweep). |
| `holeInvalid` (`detail`, `hint`), `holeNoPoints`, `holeCskDiameter`, `holeCboreInvalid` | A hole has no valid size, no centres (the sketch must hold point entities), or an impossible counterbore. `holeDowngraded` (warning) says a counterbore was drilled as a simple hole. |
| `ribNeedsOpenSketch`, `ribNoContact`, `lipFace` | Rib and lip preconditions. |

### Edits

| code | meaning | try |
|---|---|---|
| `blendOverflow` (`detail`, sometimes `max`) | The blend runs off its faces. `max` is the largest radius that fits where known. | Reduce the radius or distance. |
| `blendTooLarge`, `blendSize`, `blendCorner`, `blendNonManifold`, `blendFailed`, `blendUnsupported`, `blendTolerantEdge` | See [operations](../kernel/operations.md#blend-failures). | Blend the edges in separate features, or reorder. |
| `cornerNotTangent` (warning, `deviation`) | Edges of unequal radius meet at a vertex and the blend faces are not tangent there; `deviation` is the largest normal jump in degrees. | Use equal radii if tangency matters. |
| `filletRadiusApproximated` (warning) | A variable fillet with an end radius of 0 was built with the smallest positive end radius that works. | Give a positive end radius. |
| `shellTooThick` (`detail`, `limit`, `minRadius`, `maxThickness`) | The wall is too thick for the body. `maxThickness` is a safe thickness, not the exact limit. | Use at most `maxThickness`. |
| `shellUnsupported`, `shellFailed`, `offsetTooLarge`, `offsetUnsupported`, `offsetDegenerate`, `offsetFitFailed` | Shell and offset limits. | Reduce thickness; open another face. |
| `draftParallelFace`, `draftAxisNotParallel`, `draftTangentFaces`, `draftTooLarge`, `draftUnsupported`, `draftFailed` | Draft preconditions; `draftTooLarge` names the edge that turns over. | Reduce the angle; pick another neutral plane. |
| `moveFaceFailed`, `moveFaceUnsupported` | A move-face could not keep the neighbours consistent. `maxThickness` may give a limit. | Move less, or include tangent neighbours. |
| `pipeCutRadius` (`max`), `pipeCutFailed`, `pipeCutNothingLeft` | See [operations](../kernel/operations.md#pipe-cut). | Reduce the radius. |

### Surfaces and curves

`knitNotClosed`, `knitNearMiss`; `fillNotClosed`, `fillInvalid`, `fillFailed`; `trimNoIntersection`, `trimNothingLeft`, `trimPieceLoop`; `extendNotBoundary`, `extendNoTarget`, `extendPointBehind`, `extendUnsupported`, `extendFolds`; `thickenReach`, `thickenTooThick`, `thickenFailed`; `rebuildPeriodicFace`, `rebuildTooFine`, `rebuildFailed`; `projectCurveNoInput`, `projectCurveNoFaces`, `projectCurveMisses`, `projectCurveSeveral`, `projectCurvePlane`, `projectCurveFailed`; `editCurveNoInput`, `editCurveDegree`. Their meanings are in [operations](../kernel/operations.md) and [geometry](../kernel/geometry.md).

### Assemblies and bodies

`insertPartMissing`, `insertPartErrors` (the nested part had errors), `mateConflict`, `mateGeometry`, `mateSameBody`, `mateFixed`, `bodyNameConflict` (`name`, `features`), `ambiguousBody` (`name`, `candidates`), `combineEmpty` (`detail`), `threadSizeUnknown`, `threadFaceNotCylinder`, with the warnings `threadDiameterMismatch`, `threadModelFallback`, `threadNotCarried`.

### Budgets and repair

Budgets only fire when the host supplies a real clock through `ck_now`. A host whose `ck_now` returns a constant, such as the runner used for this book (it returns 0), never sees these errors.

| code | meaning |
|---|---|
| `featureBudgetExceeded` | The feature used more than `opts.featureTimeMs` (default 120000 ms). Its result is not applied. It is not stored in the regeneration cache, so a later regen of the same document runs again. Parameters: `budgetMs`, `elapsedMs`, `hint`, and `cause` (the code the feature would have reported). |
| `regenBudgetExceeded` | The document used more than `opts.regenTimeMs`. This feature and every later one did not run. It also appears, with the feature-budget hint, on every feature (sketches included) when the host clock advances by more than `featureTimeMs` between reads; see [errors](../protocol/errors.md). The same cache limitation applies. |
| `healed` (in `errors`) | `opts.autoHeal` repaired the body after the feature: `fixed`, `codes`, `remaining`. Although it is informational it is stored in `errors`; it is not added when the feature already has an entry of its own. |
| `partVersionNewer` | Not a feature error: a top-level key of the reply that says the document was written by a newer kernel (`file`, `kernel`). |

## What to do, in order

1. Read `errors` in feature order. Fix the first root cause.
2. If the code is a boolean or blend failure, read `detail` and `hint`, and change the geometry a little (a tenth of a millimetre) before reaching for tolerances.
3. If `errors` is empty, run [`validate`](validate.md). Issues there point at the body, not at a feature; the face names in `entity` tell which feature made the face (`F4.S.e1` was made by `F4`).
4. If the body is imported, [heal](healing.md) it.

## See also

- [Validation overview](overview.md)
- [Regeneration and caching](../kernel/regeneration.md)
- [Operations](../kernel/operations.md)
- [Errors (protocol)](../protocol/errors.md)
