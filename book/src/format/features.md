# Features

A part is a list of features, run in order. This chapter lists every feature type the kernel knows, with the chapter that documents it, and then describes what the features have in common: shared fields, boolean tolerance, time budgets and the way errors and warnings are reported.

## Index of feature types

The kernel knows 49 type names. A name that is not in this list is `unknownFeature`. Types marked "not built" are accepted as names but have no implementation: they fail with the error code `TODO(rebuild): <type> needs ck-ops`.

| Type | What it does | Chapter |
|---|---|---|
| `sketch` | 2D points, curves and constraints on a plane. | [sketches](sketches.md) |
| `sketch3d` | Points, curves and constraints in space. | [surfaces and curves](features-surfaces.md#3d-sketches) |
| `plane` | A reference plane (offset, angle, three points, ...). | [planes](planes.md) |
| `axis` | A reference line from an edge, a sketch line, a cylinder or two planes. | [planes](planes.md#the-axis-feature) |
| `extrude` | Sweeps sketch regions along the normal, with end conditions, taper, twist and thin walls. | [solids](features-solids.md#extrude) |
| `revolve` | Revolves sketch regions about a sketch line. | [solids](features-solids.md#revolve) |
| `sphere` | A ball by centre and radius. | [solids](features-solids.md#sphere) |
| `sweep` | Moves a profile along a path (sketch, helix or curve). | [solids](features-solids.md#sweep) |
| `loft` | Joins two or more sections, with point ends, guides and a centre line. | [solids](features-solids.md#loft) |
| `rib` | A thin wall from an open sketch chain to the body. | [solids](features-solids.md#rib) |
| `helix` | A helix curve for sweeps. | [solids](features-solids.md#helix) |
| `hole` | Cuts simple, counterbored or countersunk holes at sketch points. | [edits](features-edits.md#hole) |
| `fillet` | Rounds edges (constant, variable, full round). | [edits](features-edits.md#fillet) |
| `chamfer` | Bevels edges or a vertex. | [edits](features-edits.md#chamfer) |
| `draft` | Tilts faces about a neutral plane. | [edits](features-edits.md#draft) |
| `shell` | Hollows the body, opening chosen faces. | [edits](features-edits.md#shell) |
| `thread` | Cosmetic or modeled thread on a cylindrical face. | [edits](features-edits.md#thread) |
| `lip` | A raised band or a groove along rim edges. | [edits](features-edits.md#lip) |
| `moveFace` | Pushes or pulls faces; neighbours follow. | [edits](features-edits.md#moveface) |
| `deleteBody` | Removes or keeps whole bodies. | [edits](features-edits.md#deletebody) |
| `lpattern` | Copies features along one or two world axes, by spacing, extent or up to an object. | [patterns](features-patterns.md#lpattern) |
| `cpattern` | Copies features, or components, around an axis. | [patterns](features-patterns.md#cpattern) |
| `mirror` | Reflects features or the whole body about a plane. | [patterns](features-patterns.md#mirror) |
| `moveBody` | Moves, rotates or copies bodies. | [patterns](features-patterns.md#movebody) |
| `scale` | Scales bodies uniformly or per axis. | [patterns](features-patterns.md#scale) |
| `combine` | Boolean between whole bodies (also written `bodyBoolean`). | [patterns](features-patterns.md#combine) |
| `bodyBoolean` | Another name for `combine`. | [patterns](features-patterns.md#combine) |
| `surfacePlanar` | A flat sheet from a sketch profile. | [surfaces](features-surfaces.md#surfaceplanar) |
| `surfaceExtrude` | A sheet swept from a profile. | [surfaces](features-surfaces.md#surfaceextrude) |
| `surfaceRevolve` | A sheet revolved about an axis. | [surfaces](features-surfaces.md#surfacerevolve) |
| `knit` | Joins sheets into a sheet or a solid. | [surfaces](features-surfaces.md#knit) |
| `trimSurface` | Trims sheets with a sheet or plane. | [surfaces](features-surfaces.md#trimsurface) |
| `fill` | Patches a hole bounded by edges. | [surfaces](features-surfaces.md#fill) |
| `thicken` | Gives a sheet thickness. | [surfaces](features-surfaces.md#thicken) |
| `extend` | Extends a sheet past its boundary edges. | [surfaces](features-surfaces.md#extend) |
| `deleteFace` | Removes faces from a solid or sheet. | [surfaces](features-surfaces.md#deleteface) |
| `freeform` | Re-fits faces as B-spline surfaces and edits control points. | [surfaces](features-surfaces.md#freeform) |
| `pipeCut` | Rolls a ball along edges and cuts away what it touches. | [surfaces](features-surfaces.md#pipecut) |
| `projectCurve` | Projects a sketch onto faces as a B-spline curve. | [surfaces](features-surfaces.md#projectcurve) |
| `editCurve` | Rebuilds curves as a B-spline and edits its control points. | [surfaces](features-surfaces.md#editcurve) |
| `insertPart` | Inserts another part as a component. | [assemblies](features-assemblies.md#insertpart) |
| `importStep` | Holds a STEP body and brings its solids into the history. | [assemblies](features-assemblies.md#importstep) |
| `mate` | Constrains two components. | [assemblies](features-assemblies.md#mate) |
| `spattern` | Pattern driven by a sketch. Not built. | [patterns](features-patterns.md#not-built) |
| `curvepattern` | Pattern along a path: start point, flip, fixed or tangent orientation. | [patterns](features-patterns.md#curvepattern) |
| `fillpattern` | Pattern that fills a face. Not built. | [patterns](features-patterns.md#not-built) |
| `surfaceOffset` | Offsets a sheet. Not built. | [surfaces](features-surfaces.md#not-built) |
| `cutWithSurface` | Cuts the body with a sheet. Not built. | [surfaces](features-surfaces.md#not-built) |
| `dome` | Domes a face. Not built. | [surfaces](features-surfaces.md#not-built) |
| `splitLine` | Splits faces along a curve. Not built. | [surfaces](features-surfaces.md#not-built) |

## Reading a feature chapter

Each feature has the same parts: a field table (name, type, default, meaning), the names of the faces it creates (see [identifiers and references](references.md)), the error codes it raises and a run example. In the field tables:

- "number or expression" is a numeric field as described in [variables and expressions](expressions.md);
- a default of "required" means that leaving the field out makes the feature fail with the error named in the text (usually `missingSketch` or `missingReference`);
- a feature id field (`sketch`, `seeds`) names an earlier feature;
- examples that append to a base part say so. They were run through the kernel; the line "Run result" under each states what the kernel returned.

## Fields every feature reads

| Field | Applies to | Meaning |
|---|---|---|
| `suppressed` | all | Skips the feature, see [the overview](overview.md#suppression). |
| `tolerance` | features that combine a tool with the body (`extrude`, `revolve`, `sweep`, `loft`, `hole`, `thread`, patterns, `combine`) | The fuzzy value of the boolean, in mm (number or expression, default 0). |
| `autoFuzzy` | the same | `true` retries a failed or implausible boolean with a growing fuzzy value. |
| `origin` | all | A free label for where the feature came from (for example `script#3` for a feature created by a script). The kernel does not read it and does not warn about it. |
| `op`, `body` | `extrude`, `revolve`, `sweep`, `loft`, `sphere` | Operation and body name, see [solids](features-solids.md#operations-and-the-body). |

### Fuzzy booleans

With `tolerance` above 0, any planar, cylindrical or spherical face of the tool that lies within `tolerance` of an equal-kind face of the body is moved exactly onto it before the boolean, so nearly coincident faces are treated as coincident. Geometry closer than `tolerance` is merged, so a sliver thinner than that is removed and a gap thinner than that is closed. A snap that fails, makes the tool invalid or changes its volume by more than 4 x `tolerance` x area is discarded and the exact boolean runs. Useful values are 1e-6 to 1e-3 mm.

With `autoFuzzy: true`, a boolean that fails (`booleanFailed`), comes out implausible (the volume check) or comes out with more validity issues than its operands is retried with the fuzzy values 1e-7, 1e-5 and 1e-3 (those above `tolerance`); the first retry that is plausible and valid wins. If none is, the first error stands. A feature whose result depended on the fuzzy value carries the warning `booleanFuzzy {tolerance}`.

## Errors and warnings

A feature reports an error as `["<feature id>", {"code": "...", "params": {...}}]` in the `errors` list of the reply, sorted by id. A feature that fails contributes nothing and leaves the body as it was. Warnings have the same shape in `warnings`; the feature still ran. The codes that many features share:

| Code | Meaning |
|---|---|
| `unknownFeature` | The `type` is not in the index above (`params.type`). |
| `missingSketch` | A sketch-driven feature names no regenerated sketch. |
| `missingReference` | A reference (face, plane, feature, axis) does not resolve. Some carry `params` that name the reference. |
| `badExpression` | A numeric field holds an expression that does not evaluate. |
| `invalidValue` | A value is out of range (a radius of 0, a bad `op`). `params` often hold `detail` and `hint`. |
| `noClosedProfile` | The sketch has no closed region. |
| `nothingToCut` | The feature needs a body and there is none. |
| `cutNoEffect` | A cut removed nothing. |
| `booleanFailed`, `booleanVolumeMismatch` | The boolean with the body failed or gave an impossible volume. `booleanFailed` may carry `faces`, `issue`, `at` and a `hint`. |
| `featureBudgetExceeded`, `regenBudgetExceeded` | A time limit ran out (below). |

A field that the feature type does not read is reported as the warning `unknownField` (for `sketch`, `extrude`, `revolve`, `fillet`, `chamfer` and `hole`); see [misspelled fields](../validation/diagnostics.md#misspelled-fields).

The codes of each feature are in its own chapter; [feature errors and diagnostics](../validation/diagnostics.md) collects them.

### Time budgets

When the host provides a clock, one feature may run for `opts.featureTimeMs` milliseconds (default 120000; 0 or less means unlimited) and the whole regen for `opts.regenTimeMs` (default unlimited). A feature that runs out fails with `featureBudgetExceeded`; once the regen limit has passed, the features not yet run fail with `regenBudgetExceeded`. The error carries `budgetMs`, `elapsedMs` and a `hint`, and `cause` when another error was a consequence of the limit.

## See also

- [Overview of the part format](overview.md)
- [Solids from sketches](features-solids.md)
- [Edge and face edits](features-edits.md)
- [Patterns and transforms](features-patterns.md)
- [Surfaces and curves](features-surfaces.md)
- [Parts and assemblies](features-assemblies.md)
- [Feature errors and diagnostics](../validation/diagnostics.md)
