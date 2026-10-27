# Regeneration

`regen` runs a part document and returns everything a client needs to show it: the errors of the features, the solved sketches, the planes and curves, and the body with its faces, edges and display mesh. It also stores the body under a tag for the queries that follow. This chapter is the reference for the request and for every field of its result.

## Request

```json
{ "type": "regen", "tag": "main", "part": { "format": "ck-part", "version": 1, "features": [ ... ] }, "opts": { "upTo": 3 } }
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `type` | string | | `"regen"` |
| `tag` | string | `"main"` | The name under which the result is stored ([Tags and sessions](overview.md#tags-and-sessions)). |
| `part` | object | required | The part document. It must be an object with `format` equal to `"ck-part"` (only `ck-part` is written; format strings used by earlier versions are accepted and normalised to it, see [Versions and earlier files](../format/versions.md)) and `features` an array. Anything else is the request error `invalid part: Format`, and the tag keeps its previous state. `version`, `name` and the other fields are not checked here. |
| `opts` | object | `{}` | Options, below. Unknown keys are ignored. |
| `debug` | boolean | `false` | Adds `buffers` and a `trace` to the reply ([Protocol overview](overview.md#debug-timing-trace)). |

### Options

| Option | Type | Default | Meaning |
|---|---|---|---|
| `upTo` | number >= 0 | all | Regenerate only the first `n` features. Negative or non-numeric values are ignored. The part's `rollback` field limits the run in the same way; the smaller of the two applies. |
| `sketchStatus` | boolean | `false` | Add a solver `status` block to every entry of `sketches`. |
| `solveSketches` | boolean | `false` | Solve every sketch from its stored coordinates with all its constraints, and use the solved coordinates, in the reply and for the features built from them. Implies `sketchStatus`. |
| `featureTimeMs` | number | `120000` | Time budget of one feature, in milliseconds on the host clock. `0` or less means unlimited. A feature that exceeds it fails with `featureBudgetExceeded`. See the caching limitation below. |
| `regenTimeMs` | number | unlimited | Time budget of the whole regen. When it is used up, the feature that is running fails and later features are not started: each fails with `regenBudgetExceeded`. |
| `autoHeal` | boolean | `false` | After each feature, if the body has validation issues, try to repair a copy and keep the repair when it has fewer issues. |
| `healTol` | number | `1e-5` | The tolerance used by `autoHeal`, in mm. |
| `cacheCap` | number >= 0 | 256 MiB | Size cap in bytes of the regen cache; the display caches are scaled with it. |
| `noCache` | boolean | `false` | Do not read or write the caches for this call. |
| `cacheStats` | boolean | `false` | Add a `cacheStats` object to the result. |
| `noView` | boolean | `false` | Skip the display step: the reply has `body: null` and `surfaces: []`, while the tag still stores the body for later queries. For a client that already holds the view of this document and only needs the errors and the state. |
| `dofReport` | boolean | `false` | Always add the `assembly` block, even when no mate needs it. |

The budgets need a real clock from the host (`ck_now`); with a constant clock they never expire. A few more boolean options are test switches for the intersection code and are not part of the interface.

### What a regen does

The features run in document order. For each feature the kernel evaluates the numbers (expressions and variables, `=expr` fields), checks the references, builds the geometry and combines it with the body so far. Features that are `suppressed` are skipped. A feature that fails contributes nothing: the body stays as it was before the feature, and the failure is recorded in `errors` under the feature's id. The run continues, so a failure often produces a chain of follow-on failures: a feature that refers to a sketch or a face that the failed feature should have made fails with `missingSketch` or `missingReference`. The first entry in feature order is usually the one to fix. For example, pointing the extrude of a plate at a sketch that does not exist gives the errors `F2: missingSketch`, `F3: missingReference` (the next sketch is on a face of the missing body) and `F4: missingSketch`, and `body` is `null`.

Results are cached per feature prefix: when a document is sent again with only later features changed, the unchanged leading features are not run again, and a document that is identical to a cached one is answered from the cache. `cacheStats` shows this. Failures caused by a time budget (`featureBudgetExceeded`, `regenBudgetExceeded`) are not stored, so a later regen of the same document runs again. (Earlier builds replayed them; `noCache` was the workaround.)

Sending the same 4-feature plate twice with `cacheStats` gives `reusedFeatures: 0, ranFeatures: 4, viewHit: false` the first time and `reusedFeatures: 4, ranFeatures: 0, fullHit: true, viewHit: true` the second.

## Result

The result is an object. The first nine keys are always present, in this order:

```json
{
  "errors": [], "components": [], "planes": [], "markers": [],
  "sketches": [], "sketches3": [], "curves": [], "body": null, "surfaces": []
}
```

That is the whole result for a part with no features. Optional keys follow when they have content: `threads`, `limits`, `assembly`, `warnings`, then `cacheStats` and `partVersionNewer`.

| Key | Type | Content |
|---|---|---|
| `errors` | array | Failed features: `[featureId, { code, params }]`. See [Errors](errors.md). |
| `components` | array | One entry per inserted part: `{ id, dof, place }`. |
| `planes` | array | The reference planes: `[featureId, frame]`. |
| `markers` | array | Per-feature drawing aids: `[featureId, object]`. |
| `sketches` | array | The solved sketches: `[featureId, { frame, profiles, sketch, status? }]`. |
| `sketches3` | array | The 3D sketches: `[featureId, { sketch }]`. |
| `curves` | array | Curves made by features: `[featureId, { pts, axis, t0, t1, ... }]`. |
| `body` | object or `null` | The solid, or `null` when the document makes none. |
| `surfaces` | array | Sheet bodies: `[featureId, body]`. |
| `threads` | array | Thread features: one object each. Only when there are some. |
| `limits` | array | `[featureId, { maxHeight }]`. Only when there are some. |
| `assembly` | object | Degrees-of-freedom report. Only for added mate kinds, or with `dofReport`. |
| `warnings` | array | Non-fatal notes, same shape as `errors`. Only when there are some. |
| `cacheStats` | object | Only with `opts.cacheStats`. |
| `partVersionNewer` | object | `{ file, kernel }`, only when the document's `version` is higher than the kernel's. |

The arrays that are keyed by a feature id (`errors`, `planes`, `sketches`, `sketches3`, `components`, `threads`, `limits`, `warnings`) are sorted by the id as a string, so `F10` comes before `F2`. An id appears once: a later entry for the same feature replaces an earlier one. `curves` and `markers` keep the order in which the features ran.

### `errors` and `warnings`

Each entry is a two-element array: the id of the feature (an empty string if the feature has no id) and an object with the error `code` and a `params` object.

```json
"errors": [ [ "F4", { "code": "holeInvalid", "params": { "hint": "the hole diameter must be greater than zero (a standard `size`, or a positive `diameter`)" } } ] ]
```

`params` carries the facts of the failure and, for most failures, a `hint` that says what to change. `warnings` has the same shape and means that the feature ran but something was approximated or ignored; for example a thread whose size does not fit the hole:

```json
"warnings": [ [ "F5", { "code": "threadDiameterMismatch", "params": {
    "expected": 6.8, "actual": 8, "size": "M8", "standard": "metric", "internal": true, "nearest": "M10",
    "hint": "hole is 8, M8 tap drill is 6.8: drill the hole to 6.8, or choose M10 (tap drill 8.5)" } } ] ]
```

[Errors](errors.md) lists all codes. With `autoHeal` a repaired feature adds a `healed` entry (with `fixed`, `codes` and `remaining`) to `errors`; it marks a repair, not a failure.

### `sketches`

One entry per sketch feature that ran, in the form `[ "F1", { frame, profiles, sketch, status? } ]`.

| Field | Meaning |
|---|---|
| `frame` | The sketch plane as `{ o, x, y, z }`: origin and the unit x, y and normal axes, in world coordinates. |
| `profiles` | The closed regions the sketch contains, and whether it also has open chains (below). |
| `sketch` | The sketch as the kernel used it: the feature object with normalised fields. With `solveSketches` (or when an `expr` changed a constraint value) the point coordinates are the solved ones. |
| `status` | Only with `sketchStatus` or `solveSketches` (below). |

`profiles` is `{ "regions": [ { "outer": [segment, ...], "holes": [[segment, ...], ...] } ], "open": false }`. The regions are the closed areas that the sketch curves enclose, so curves that cross split an area into several regions; each region has one outer loop and the loops of the holes inside it. Construction geometry is left out. A segment is `{ "kind", "tag", "a", "b" }`: `kind` is `line`, `arc` (arcs, circles and ellipses) or `spline`; `tag` is the id of the sketch entity it came from; `a` and `b` are the start and end points in sketch coordinates (equal for a full circle, up to rounding). `open` is true when the sketch has curves but no closed region at all. Extrudes and the other profile consumers build from these regions.

`status` is `{ dof, fullyDefined, conflicts, redundant, solved, state, residual }`:

| Field | Meaning |
|---|---|
| `dof` | Unknowns minus the rank of the constraint system: the number of motions still free. A point that is not fixed has two unknowns; a circle or ellipse has one more for its radius. |
| `fullyDefined` | `true` when `state` is `"full"` and the sketch is solved. |
| `conflicts` | Ids of constraints that cannot hold together with the earlier ones. |
| `redundant` | Ids of constraints that are implied by the earlier ones. |
| `solved` | The stored coordinates satisfy every constraint (largest residual at most 1e-7), or a solve from them converges. |
| `state` | `"under"` (free motions left), `"full"` (none) or `"over"` (some equation depends on the earlier ones). |
| `residual` | The largest constraint residual at the coordinates in the reply. |

By default the kernel does not move the points of a sketch: the coordinates in the document are used as stored, and the constraints are for the editor. Sending a rectangle with horizontal, vertical and distance constraints whose coordinates are slightly off (corners at (98, 3), (97, 58) and (2, 61), sides constrained to 100 and 60) gives a body with the box `[0, 0, 0]` to `[98, 61, 10]`. The same document with `"solveSketches": true` gives the corners at (100, 0), (100, 60) and (0, 60) to within 1e-10, the box `[0, 0, 0]` to `[100, 60, 10]`, and `status` `{ "dof": 0, "fullyDefined": true, "conflicts": [], "redundant": [], "solved": true, "state": "full", "residual": 1.99e-11 }`. [The sketch solver](../format/sketch-solver.md) describes the solver.

### `planes`, `curves`, `markers`, `sketches3`

- `planes`: `[ "F3", { "o": [0,0,25], "x": [1,0,0], "y": [0,1,0], "z": [0,0,1] } ]` for each `plane` feature that resolved. Standard planes (`front`, `top`, ...) and the plane of a sketch are not listed. The entry is there even when the plane feature is hidden.
- `curves`: curves made by `helix`, `projectCurve` and `editCurve` features: `[ id, { pts, axis, t0, t1 } ]`. `pts` is a polyline of points `[x, y, z]`, `axis` is `false`, and `t0`, `t1` the parameter range. A helix adds `helix: { o, a, x, y, k, r, angle }` (origin, axis, start direction, perpendicular direction, rise per radian `k`, radius `r` and the total angle) and has `t1` equal to the total angle in radians. A helix with pitch 2, 5 revolutions and radius 5 gives `t1 = 31.4159...` (10 pi), `k = 0.3183...` (pitch over 2 pi) and `r = 5`.
- `markers`: extra data about a feature for display. A `loft` adds `{ loops, starts, guided, tension }`: for each profile 16 samples per piece as `{ p: [x, y, z], f: fraction }`, the matched start point of each profile, whether guides were used, and the tension markers. A `projectCurve` adds `{ degree, count, deviation, length, pieces, corners, closed }` for the fitted curve.
- `sketches3`: `[ "F1", { "sketch": { ... } } ]` echoes each `sketch3d` feature as the kernel read it.

### `components`, `assembly`

Each `insertPart` feature adds a component to the document. `components` has `{ "id": "F6", "dof": 0, "place": { o, x, y, z } }` for each, sorted by id: the id of the inserting feature, its remaining degrees of freedom (0 when fully mated or fixed), and the current placement as a frame. `assembly`, present when the document has mates of the extended kinds or with `dofReport`, has the totals:

```json
"assembly": {
  "dof": { "total": 2, "translational": 1, "rotational": 1 },
  "components": [ { "id": "F1", "fixed": true, "dof": 0, "translational": 0, "rotational": 0 }, ... ],
  "overConstrained": true, "redundant": ["F4"],
  "conflicts": [ { "mate": "F5", "with": ["F3", "F4"] } ]
}
```

A mate that the solver rejects is dropped (the components return to the pose before it) and reported as the error `mateConflict` on the mate and in `conflicts`. With more than 30 floating components the null-space part of the report is skipped and `truncated: true` is added. [Parts and assemblies](../format/features-assemblies.md) describes mates; `drag` ([Queries](queries.md#drag)) moves a component with its mates satisfied.

### `threads`, `limits`

`threads` has one object per `thread` feature, with the facts the feature resolved: for the M8 thread above on the 8 mm hole `{ "id": "F5", "face": "F4.S.h1", "standard": "metric", "size": "M8", "class": "6H", "pitch": 1.25, "major": 8, "minor": 6.646835, "pitchDiameter": 7.188101, "angle": 60, "length": 10, "hand": "right", "internal": true, "modeled": false, "tapered": false, "diameter": 8, "start": [20, 30, 10], "dir": [0, 0, -1] }`. A thread is cosmetic unless `modeled` is true.

`limits` records size limits of a feature: `[ "F9", { "maxHeight": 56.71 } ]`, for an extrude of a single circle (radius 10) with a draft of 10 degrees, is the extrude depth at which the tapered sides would meet in a point. A depth of 80 for the same extrude fails with `draftTooLarge` (`params.maxHeight` is again 56.71).

## `body`

`body` is the solid, or `null` when the document produced none (no features, only sketches, or every feature that makes material failed). It has these keys, in order:

```json
"body": {
  "box": { "min": [0, 0, 0], "max": [100, 60, 10] },
  "faces": [ ... ],
  "edges": [ ... ],
  "silhouette": { "$buf": 4, "type": "f32", "len": 0, "offset": 11648 },
  "mesh": { "positions": { "$buf": 0, ... }, "normals": { "$buf": 1, ... }, "mids": { "$buf": 2, ... }, "triFace": { "$buf": 3, ... } }
}
```

`box` is the axis-aligned bounds of the geometry, widened to include the display mesh. `silhouette` and `mesh` are buffer references ([Replies and buffers](reply.md)); the arrays are described there.

### `faces`

One entry per face, in the body's face order. The index of a face in this array is the value stored in `mesh.triFace` for its triangles.

| Field | Type | Meaning |
|---|---|---|
| `id` | string | The persistent name of the face ([Persistent names](../kernel/naming.md)): `<feature>.<role>`. The role is `C<k>` for the caps of an extrude, revolve or loft, `S.<entity>` for the side face made from a sketch entity, `B<k>` for a blend, or a feature-specific name (a hole's faces are `S.h0` to `S.h4`). A face split by a later feature gets `~<k>` for the region, and a face made inside an inserted part gets an `@<feature>.<n>` suffix. |
| `kind` | string | The surface type: `plane`, `cylinder`, `cone`, `sphere`, `torus`, `revolution`, `extrusion`, `spline` or `offset`. |
| `planar` | boolean | `true` when the face is geometrically flat (within 1e-9), whatever its `kind`. A free-form (`spline`) face is never reported planar. |
| `meta` | object | `feature`: the id of the feature that made the face. `entity`: for side faces, the sketch entity (with an instance suffix). On the faces of a sheet body in `surfaces`, `surface: true`. |
| `normal` | `[x, y, z]` | The outward unit normal; only on planar faces. |
| `lump` | number | The index of the separate body (lump) the face belongs to; only when the body has more than one lump. |

The faces of the plate with a hole:

```json
{ "id": "F2.C1",    "kind": "plane",    "planar": true,  "meta": { "feature": "F2" }, "normal": [0, 0, 1] },
{ "id": "F4.S.h1",  "kind": "cylinder", "planar": false, "meta": { "feature": "F4", "entity": "h1" } }
```

The reply carries no geometry for curved faces beyond the mesh; ask `measure`, `describe` or `inspect` for the radius, axis and area ([Queries](queries.md)).

### `edges`

One entry per edge of the body as a client would draw it. A closed curve that the kernel stores in several pieces (a circle split at a seam) is reported once, and edges that exist only to close periodic faces are not listed.

| Field | Type | Meaning |
|---|---|---|
| `pts` | array of `[x, y, z]` | A polyline of the edge, for drawing. A line has its two end points; curved edges are sampled finely enough to draw. |
| `faces` | array of strings | The faces on the two sides of the edge, sorted. An edge of an open sheet has one entry. |
| `key` | string | A key of the form `<face>\|<face>#<n>`: the two face ids and the number of the edge in the body's edge order, for example `F2.C1\|F2.S.e1#2`. The number depends on the order of the edges, so it can change when other edges appear or disappear; the face part stays. `describe` and `resolve` take keys. |
| `mid` | `[x, y, z]` | The point half-way along the edge by length. |
| `line` | `true` | Present on straight edges. |
| `circle` | `{ c, n, r }` | Present on edges that lie on a circle or circular arc: centre, unit normal and radius. |

An edge that is neither straight nor circular has neither `line` nor `circle`. A client refers to an edge by `{ "faces": [a, b], "at": [x, y, z] }`: the two face ids and a point near it, which picks one when two faces share several edges.

```json
{ "pts": [[0,0,0],[100,0,0]], "faces": ["F2.C0","F2.S.e1"], "key": "F2.C0|F2.S.e1#0", "mid": [50,0,0], "line": true }
```

### `surfaces`

Features that make sheets instead of solids (`surfaceExtrude` and the other surface features, a `loft` with `surface: true`) put them in `surfaces` as `[ featureId, body ]`, where `body` has the same keys as above (`box`, `faces`, `edges`, `silhouette`, `mesh`) and every face has `meta.surface: true`. Their buffers follow those of `body` in the binary area. A `surfaceExtrude` of a 20 x 20 square, depth 15, gives one entry with four faces.

## Limits and known gaps

- Time: a regen runs on one thread to completion; there is no cancel. Use `featureTimeMs` and `regenTimeMs` to bound it, and `upTo` to find the slow feature. The default per-feature budget is two minutes.
- Feature types that the kernel names but does not build yet fail with a code that starts `TODO(rebuild)`: `dome`, `splitLine`, `spattern` (`missingSketch` without a sketch, otherwise `TODO(rebuild)`), `fillpattern`, `curvepattern`, `surfaceOffset` and `cutWithSurface` (each a run on a plate, with an empty feature). The code refuses some options of implemented features the same way: a drafted mid-plane extrude that needs a union, a `lip` on curved or several rim faces, an `extend` with a shape other than `same`, a `deleteFace` with fill or heal, a surface `sweep` of an open profile, and the mirror of an offset surface. [Known limits](../quality/limits.md) collects the current gaps.
- `inspect` of a face, edge, curve or plane on a regenerated body is not implemented; see [Queries](queries.md#inspect).
- `body.faces` carries no areas or radii, `body.edges` no lengths: use `measure`, `describe` or `massProps`.

## See also

- [Protocol overview](overview.md)
- [Replies and buffers](reply.md)
- [Queries](queries.md)
- [Errors](errors.md)
- [Features](../format/features.md)
- [Regeneration and caching](../kernel/regeneration.md)
