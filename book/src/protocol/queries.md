# Queries

This chapter documents every request type other than `regen`: its parameters, its result, and what it answers when there is nothing to answer. The examples were run on three small documents and trimmed with `...`:

- **plate**: the 100 x 60 x 10 plate with an 8 mm through hole at (20, 30) from [A first part](../getting-started/first-part.md), regenerated under the tag `q`. Its faces are `F2.C0` (bottom), `F2.C1` (top), `F2.S.e1` to `F2.S.e4` (the sides) and `F4.S.h1` (the hole wall).
- **two bodies**: two 20 x 20 x 10 blocks made with `"op": "newBody"` and named `A` and `B` through the `body` field, with `B` shifted by (10, 10) so that they overlap. The document is:

```json
{"format":"ck-part","version":1,"name":"Two","features":[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":20,"y":0},{"id":"c","x":20,"y":20},{"id":"d","x":0,"y":20}],"entities":[{"id":"e1","type":"line","p1":"a","p2":"b"},{"id":"e2","type":"line","p1":"b","p2":"c"},{"id":"e3","type":"line","p1":"c","p2":"d"},{"id":"e4","type":"line","p1":"d","p2":"a"}],"constraints":[]},{"type":"extrude","id":"F2","op":"newBody","body":"A","sketch":"F1","end":"blind","depth":10},{"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":10,"y":10},{"id":"b","x":30,"y":10},{"id":"c","x":30,"y":30},{"id":"d","x":10,"y":30}],"entities":[{"id":"e1","type":"line","p1":"a","p2":"b"},{"id":"e2","type":"line","p1":"b","p2":"c"},{"id":"e3","type":"line","p1":"c","p2":"d"},{"id":"e4","type":"line","p1":"d","p2":"a"}],"constraints":[]},{"type":"extrude","id":"F4","op":"newBody","body":"B","sketch":"F3","end":"blind","depth":10}]}
```
- **rounded plate**: the plate without the hole and with feature `F3`, a fillet of radius 5 on the edge between `F2.C1` and `F2.S.e1`, whose face is `F3.B0`.

Unless an entry says otherwise, a request takes the optional string `tag` (default `main`). The behaviour that all body queries share:

- a tag that was never regenerated answers the error `noState: nothing regenerated yet`;
- a tag whose last regen made no body answers an empty value of the request's own shape, named under "No body" in each entry (`null` for most);
- bad arguments answer an error reply whose message names the request (`probe: at most 2000 points per request`).

Lengths are in millimetres and angles in degrees, except where an entry says radians. In the measurement requests an *entity* is written as described under [Entities](#entities).

## Document

### ping

`{"type":"ping"}` answers `{"pong":true}`, plus `"build":"<revision>"` if the module was built with `CK_BUILD_REV`. It takes no tag and touches no state.

### step

Exports the tag's body as a STEP file ([STEP](../io/step.md)).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `name` | string | `"part"` | The product name written into the file. |

The result is the file's text as one JSON string (`"ISO-10303-21;\nHEADER;..."`; 6.9 KB for the plate with a hole). Separate lumps that were built as voids are split into solids first. No body: `null`.

### validate

Runs the independent solid validator, and a watertightness check of the display mesh ([The validate request](../validation/validate.md)).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `tol` | number | `1e-6` | Geometric tolerance in mm. |
| `healTolerance` | number | the validator's cap | The largest edge or vertex tolerance a body may carry before it counts as an issue. |
| `mesh` | boolean | `true` | `false` skips the mesh check. |
| `opts.validateTimeMs` | number | `60000` | Time budget in ms (`0`: unlimited). When it runs out the remaining checks are not run and the reply carries `validateBudgetExceeded`. |
| `opts.quick` | boolean | `false` | Topology and bounding checks only (no volume checks, no mesh checks). |

```json
{ "type": "validate", "tag": "q" }
```
```json
{ "ok": true, "issues": [], "toleranceUsed": 0 }
```

| Result key | Meaning |
|---|---|
| `ok` | `true` when `issues` is empty. |
| `issues` | `[{ code, entity, detail }]`: the check that failed, the entity it concerns (`F2.S.e1`, `edge#4`, `lump#0`) and a sentence. |
| `toleranceUsed` | The largest edge or vertex tolerance in the body (0 for an exact body). |
| `stats` | `{ totalMs, shell, loops, edges, solids, tessellation, ... }`: time in ms of the request and of each check that ran. |
| `validateBudgetExceeded` | Only when the time budget ran out: `{ budgetMs, elapsedMs, checksNotRun, checksPartial, detail }`. `ok` is then `false`; `issues` holds what the checks that ran found. |
| `quick`, `checksSkipped` | Only with `opts.quick`: `true` and the checks left out. |
| `meshNotes` | Strings `"<face>: <note>"`, when the tessellator noted a problem on a face. Only when there are some. |
| `info` | For a body with overlapping lumps: `{ code: "bodiesOverlap", severity: "info", a, b, aName?, bName?, volume }` for each overlapping pair. Overlap is legal and is never an issue. |
| `notes` | A note when the body carries tolerances above zero. |

On the two overlapping blocks: `{"ok":true,"issues":[],"toleranceUsed":0,"info":[{"code":"bodiesOverlap","severity":"info","a":"F2.C0","b":"F4.C0","aName":"A","bName":"B","volume":1000}]}`. No body: `null`.

### heal

Repairs the body in place: merges near-coincident vertices, sews free edges, makes face orientations consistent and so on ([Healing](../validation/healing.md)).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `tol` | number | `1e-5` | Healing tolerance in mm. |
| `heal` | object or `"off"` | | Options: `tol`, `sew`, `fixSmall`, `fixOrientation`, `tolerant`, `cap`. |
| `healTolerance` | number | | Cap on edge and vertex tolerances. |
| `dryRun` | boolean | `false` | Report what would be repaired without changing the stored body. |
| `damage` | string | | Self-test: break a copy of the body in the named way first (`detach`, `shells`, `flip`, `invert`, `reverseCoedge`, `shuffle`, `tiny`) and report whether healing restored it. The stored body is never changed. |

```json
{ "type": "heal", "tag": "q", "damage": "flip", "dryRun": true }
```
```json
{ "fixed": [ { "code": "FACE_FLIPPED", "entity": "F2.S.e1", "detail": "face orientation made consistent with its neighbours" } ],
  "remaining": [],
  "volume": { "before": 59497.34517542564, "after": 59497.34517542564 },
  "damage": { "kind": "flip", "applied": true, "clean": true } }
```

`fixed` and `remaining` are lists of `{ code, entity, detail }`. A healthy body without `damage` answers empty lists. `damage.applied` is `false` when the body has nothing of that kind to break, or is not healthy to begin with; `clean` is `true` when the repaired copy validates and has the volume of the body before the damage. No body: `null`.

## Mass and interference

### massProps

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `bodies` | boolean | `false` | Add a per-lump list `bodies`. |
| `densities` | array of numbers | | One density per lump (mass per mm^3). Implies `bodies: true`, adds `mass` to each entry and a `byMass` object. A wrong length is an error naming the lump count. |

```json
{ "type": "massProps", "tag": "q" }
```
```json
{ "volume": 59497.34517542564, "area": 15350.796447372313, "com": [50.25345071604068, 30, 4.999999999999997],
  "principal": [18493800.59049692, 50037589.30162367, 67539767.47253016],
  "axes": [ [1, 1.4e-17, 8.3e-19], [-1.4e-17, 1, 2.9e-22], [-8.3e-19, -2.9e-22, 1] ] }
```

| Key | Meaning |
|---|---|
| `volume`, `area` | Total volume and surface area. |
| `com` | Centre of mass at unit density. |
| `principal` | The principal moments of inertia about the centre of mass, ascending, at unit density (volume stands for mass). |
| `axes` | The three unit principal directions, matching `principal`. For a body with equal moments (a cylinder, a sphere) the directions are arbitrary. |
| `bodies` | With `bodies` or `densities`: `[{ volume, area, com, name?, mass? }]` per lump. `name` is the body name if the lump has one. |
| `byMass` | With `densities`: `{ mass, com, principal, axes }` for the density-weighted whole. |

With two overlapping blocks and `"bodies": true`, `bodies` is `[ { "volume": 4000, "area": 1600, "com": [10,10,5], "name": "A" }, { ..., "name": "B" } ]` while `volume` is `8000`: overlap is counted twice, since the lumps are separate bodies. With `"densities": [0.0027]` on the plate, the entry gets `"mass": 160.64` and `byMass.mass` is `160.64`. No body: `null`. A body whose lumps are all sheets (open edges, no enclosed volume, for example a surface import) answers `volume: 0`, its `area`, `com` as the area centroid and `kind: "surface"`; `principal`, `axes`, `firstMoment`, `inertia` and `inertiaOrigin` are then weighted by area (surface density 1), and `mass` is 0. Earlier replies had `null` for `com` and `principal` here; a caller that reads only `volume` or `area` sees no change.

### interference

Pairs of lumps of the same state that overlap. No parameters.

```json
{ "bodies": 2, "pairs": [ { "a": "F2.C0", "b": "F4.C0", "aName": "A", "bName": "B",
                            "volume": 1000, "com": [15, 15, 5], "box": { "min": [10,10,0], "max": [20,20,10] } } ] }
```

`bodies` is the lump count. Each pair names one face of each lump (`a`, `b`), the lump names if they have any, and the common part: its volume, centre of mass and bounding box. A pair whose boolean fails has `error` instead. For the plate, `{"bodies":1,"pairs":[]}`. No body: `null`.

### bodyBoolean

Previews the `combine` feature on the state's separate bodies and stores nothing.

| Parameter | Type | Meaning |
|---|---|---|
| `op` | string | `union` (`add`, `join`), `cut` (`subtract`), `intersect` (`common`) or `split`. |
| `target` | selector | Exactly one lump: a face id (the lump holding it), `{ "face": id }`, `{ "lump": n }` or `{ "body": name }`. |
| `tools` | array of selectors | One or more lumps; none may be the target. |
| `keepTools` | boolean | Keep the tool lumps next to the result. |
| `tolerance`, `autoFuzzy` | number, boolean | The fuzzy-boolean controls that the features which combine bodies also take: a tolerance in mm for nearly coincident faces, and a retry at looser tolerances. |

```json
{ "type": "bodyBoolean", "tag": "t", "op": "intersect", "target": { "body": "A" }, "tools": [ { "body": "B" } ] }
```
```json
{ "ok": true, "lumps": 1, "volume": 1000, "area": 600, "faces": ["F2.C0","F2.C1","F2.S.e2","F2.S.e3","F4.S.e1","F4.S.e4"] }
```

The union of the same two blocks answers `"volume": 7000` (4000 + 4000 - 1000). A failure is a normal result, not an error reply: `{ "ok": false, "error": { "code": "invalidValue", "params": { "detail": "a tool is the target lump" } } }`. (This `error` is an object of the feature-error form, unlike the string of an error reply; see [Errors](errors.md).) No body: `null`.

## Selection and references

### measure

Facts about faces and edges and, for exactly two references, the relation between them.

| Parameter | Type | Meaning |
|---|---|---|
| `refs` | array | One or more references: `{ "kind": "face", "id": "F2.C1" }` or `{ "kind": "edge", "faces": ["F2.C1","F2.S.e1"], "at": [50,0,10] }`. |

The result is `{ "items": [...] }` with one item per reference, plus the pair keys when there are exactly two references. If any reference does not resolve, the result is `null`. Items:

| Item | Keys |
|---|---|
| plane face | `kind: "face"`, `surface: "plane"`, `area`, `normal` (outward) |
| cylinder face | `surface: "cylinder"`, `area`, `radius`, `center`, `axis` |
| cone face | `surface: "cone"`, `area`, `center`, `axis` |
| sphere face | `surface: "sphere"`, `area`, `radius`, `center` |
| other faces | `surface`, `area` |
| edge | `kind: "edge"`, `length`; a line adds `direction`; a circle `radius`, `center`, `axis`; then `wedge`: the angle in radians between the outward normals of the two faces, for edges with two faces |

Pair keys: `distance` and `points` (the closest point on each reference), `angle` in radians between the normals or directions (a plane against a line gives the complement), `normalDistance` for parallel planes, and `centerDistance` for cylinders, cones, spheres and circles. Distances are found by sampling and refinement, to the accuracy of the refinement; the `distance` request is the exact one.

```json
{ "type": "measure", "tag": "q", "refs": [ { "kind": "face", "id": "F2.C0" }, { "kind": "face", "id": "F2.C1" } ] }
```
```json
{ "items": [ { "kind": "face", "surface": "plane", "area": 5949.73, "normal": [0,0,-1] },
             { "kind": "face", "surface": "plane", "area": 5949.73, "normal": [0,0,1] } ],
  "distance": 10, "points": [[0,0,0],[0,0,10]], "angle": 0, "normalDistance": 10 }
```

For the hole wall and a side face: `items[0]` is `{ "surface": "cylinder", "area": 251.33, "radius": 4, "center": [20,30,10], "axis": [0,0,-1] }`, and the pair has `"distance": 16` with `points` `[16,30,10]` and `[0,30,10]`. A single edge: `{ "items": [ { "kind": "edge", "length": 100, "direction": [1,0,0], "wedge": 1.5707963267948966 } ] }`. No body: `null`.

### describe

Facts and a one-line description for named faces and edges.

| Parameter | Type | Meaning |
|---|---|---|
| `names` | array | Each: a face id, an edge key (`F2.C0\|F2.S.e1#0`), `{ "kind": "face", "id": ... }` or an edge reference `{ "faces": [a, b], "at": [x, y, z] }`. |

The result is `{ "items": [...] }`. An item for a face that exists has `name`, a `ref` that can be stored in a feature (`{ kind: "face", id }` or an edge reference), `kind`, `surface` or `curve`, `area` or `length`, `centroid` or `mid`, `feature`, `role` (`cap`, `side`, `blend`, `other`), the direction facts that apply (`normal`, `axis`, `dir`, `radius`), `concave` and `hole` for cylinders, `adjacent` faces, `outermost` (the sides of the body's box that a planar face lies on), `text`, and `found: true`. A name that no longer exists answers `found: false`, an `error`, and `nearest` candidates:

```json
{ "name": "F9.X", "found": false, "error": "face id not found", "nearest": ["F2.C0","F2.C1","F2.S.e1"] }
```

and for the hole wall: `"text": "hole barrel (concave) cylinder face F4.S.h1: radius 4, axis -z, area 251.3274, centre (20, 30, 5); side face of sketch entity h1 in F4; adjacent to F2.C0, F2.C1"`. No body: `null`.

### resolve

Turns descriptions of faces and edges into persistent names, with a report of how unambiguous the description was.

| Parameter | Type | Meaning |
|---|---|---|
| `descriptions` (or `description`) | array of objects | Each has `kind` (`face` or `edge`) and any of the filters below; all given filters must hold. |

Filters: `surface` (`plane cylinder cone sphere torus spline revolution extrusion offset`, string or list) and `normal`, `axis`, `radius`, `area`, `role`, `entity`, `concave`, `hole` for faces; `curve` (`line circle other`), `dir`, `length`, `between` for edges; and for both `feature`, `name` (exact, or a prefix ending `*`), `touches`, `body`, `extreme` with `along`, and `near` with `within`. Directions are `"x"`, `"+z"`, `"-y"` or `[x, y, z]` and match to about 0.08 degrees. Number values are `5` (relative 1e-6), `{ "value", "tol" }` or `{ "min", "max" }`. Filters apply in a fixed order (feature, name, touches, radius, shape fields, extreme, near), so `extreme` chooses among the faces already filtered.

Each description answers `{ description, status, ambiguous, count, matches, narrowing }`. `status` is `unique`, `ambiguous` (more than one match, all of them listed), `none` or `invalid` (an unknown field or bad value, with `error`). `narrowing` lists how many candidates remained after each filter; `hint` and `resolvedBy` (`"near"`) appear when they apply. Each match is a `describe` item.

```json
{ "type": "resolve", "tag": "q", "descriptions": [ { "kind": "face", "surface": "plane", "normal": "+z", "extreme": "max" } ] }
```
```json
{ "results": [ { "description": { ... }, "status": "unique", "ambiguous": false, "count": 1,
                 "matches": [ { "name": "F2.C1", "ref": { "kind": "face", "id": "F2.C1" }, "surface": "plane", "area": 5949.73,
                                "centroid": [50.25, 30, 10], "role": "cap", "normal": "+z", ... } ],
                 "narrowing": [ { "filter": "surface", "remaining": 6 }, { "filter": "normal", "remaining": 1 },
                                { "filter": "extreme", "remaining": 1 } ] } ] }
```

No body: `null`.

### check

Checks expectations about the body, for tests and for clients that verify a result.

| Parameter | Type | Meaning |
|---|---|---|
| `expect` | array or object | `[{ kind, equals \| min \| max, tol?, relTol?, id? }]`, or the shorthand `{ "volume": 1000, "faces": 7, "edges": { "min": 10 } }`. |
| `tol` | number | Tolerance of the `valid` check (default `1e-6`). |

Kinds: `volume`, `area` (default relative tolerance 1e-4); `mass` (volume times `density`); `bbox` (any of `min`, `max`, `size` as `[x, y, z]`, absolute `tol`); `com` (`at`, `tol`); the counts `faces`, `edges`, `vertices`, `solids`; `holes` (concave cylindrical faces bounded by full circles); `innerLoops`; and `valid` (validator issue count must be 0).

```json
{ "type": "check", "tag": "q", "expect": [ { "kind": "volume", "equals": 59497.345 }, { "kind": "faces", "equals": 7 }, { "kind": "holes", "equals": 1 } ] }
```
```json
{ "ok": true, "passed": 3, "failed": 0,
  "results": [ { "index": 0, "kind": "volume", "pass": true, "measured": 59497.345175426,
                 "expected": { "equals": 59497.345, "tol": 5.9497345 }, "detail": "volume 59497.3452 vs [59491.3953, 59503.2947]" }, ... ],
  "measured": { "volume": 59497.345175426, "area": 15350.796447372, "com": [50.253450716, 30, 5],
                "bbox": { "min": [0,0,0], "max": [100,60,10], "size": [100,60,10] },
                "faces": 7, "edges": 14, "vertices": 11, "solids": 1, "holes": 1, "innerLoops": 2 } }
```

An expectation that cannot be used (unknown kind, missing value) fails with an explanatory `detail`. No body: `null`.

### refDiff

Compares two versions of a document and reports references that now point at a different entity. It regenerates both documents privately (through the regen cache) and stores nothing.

| Parameter | Type | Meaning |
|---|---|---|
| `base` | object | The document before the edit. |
| `part` | object | The document after the edit. Both must be part documents (`invalid part: Format` otherwise). |
| `repair` | boolean | Default `false`. `true` adds `repaired` (the `part` with every unambiguous suggestion applied) and `repairs`. |

Result: `{ ok, checked, warnings, features, errors }`. `checked` is the number of references present in both. Each warning is `{ code, feature, ref, reason, before, after }`, with `code` `referenceChanged` (it resolves, to something that no longer matches in surface type, direction, neighbour kinds, size ratio between 0.25 and 4, or position) or `referenceLost`. `features` is `{ removed, added, reordered }` (feature ids). `errors` lists regen failures of either document as `[featureId, code]` pairs (the code is a plain string here). Ordinary parameter edits do not warn.

`base` is the plate of [A first part](../getting-started/first-part.md) without the hole and with the fillet `{ "type": "fillet", "id": "F3", "radius": 5, "edges": [ { "faces": ["F2.C1", "F2.S.e1"], "at": [50, 0, 10] } ] }` as third feature. `part` is the same document with the points of sketch `F1` reordered to `p1` (100, 60), `p2` (0, 60), `p3` (0, 0), `p4` (100, 0), the entities unchanged, so that `e1` is now the opposite side of the plate:

```json
{ "ok": false, "checked": 1,
  "warnings": [ { "code": "referenceChanged", "feature": "F3",
                  "ref": { "faces": ["F2.C1","F2.S.e1"], "at": [50,0,10] },
                  "reason": "position (50, 0, 10) -> (50, 60, 10)",
                  "before": "line edge F2.C1|F2.S.e1#5: length 100, direction +x, midpoint (50, 0, 10); between F2.C1 and F2.S.e1",
                  "after":  "line edge F2.C1|F2.S.e1#5: length 100, direction -x, midpoint (50, 60, 10); between F2.C1 and F2.S.e1" } ],
  "features": { "removed": [], "added": [], "reordered": false }, "errors": [] }
```

A `referenceLost` warning also carries `candidates` and, when exactly one of them fits, `suggest`. The candidates are the faces (`{ "id", "text" }`) or, for an edge pick, the edges (`{ "faces": [a, b], "text" }`) of the body at that feature that the old entity could be: the kernel keeps those whose kind, direction, neighbour kinds, size and position still match the old one under the rules above, closest first, at most five. `suggest` is present only when exactly one entity fits, so a repair is never a choice between two; with two or more the client or the agent picks from `candidates`. With `"repair": true` the reply adds `repaired`, the `part` with each suggestion applied (a face id replaced in that feature, an edge pick given the new face pair with its `at` kept), and `repairs`, `[{ feature, from, to }]`. Nothing else in the document is touched. A reference that does not resolve in `part` and has no suggestion stays as it is and keeps failing with `missingReference` or `missingEdge`, whose own params list what the body has now.

A client calls it with the document before an edit when it applies an edit, a deletion or a move. References inside sketches and inside nested inserted parts are not compared, and a feature whose id changes looks like a removal plus an addition.

### matchEdges

Finds edges that are like seed edges, for "select similar" commands. It regenerates the `part` given in the message, so it needs no tag.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `part` | object | required | The document. |
| `upTo` | number | all | Regenerate only this many features. |
| `mode` | string | `similar` | `similar`: same curve type, length and dihedral angle (convex and concave edges differ). `parallel`: lines parallel to a line seed, circles with a parallel axis to a circle seed. `tangent`: the chain of edges that continue the seed tangentially (angle under 1e-3 rad) through shared vertices. An unknown mode acts as `similar`. |
| `seeds` | array | | Edges as `{ "faces": [a, b], "at": [x, y, z] }`. |
| `exclude` | array | | Edges to leave out of the answer. |
| `noCache` | boolean | `false` | Do not use the regen cache. |

The result is an array of `{ faces, at }` edge references: the union over the seeds, without the seeds or the excluded edges, without duplicates. For the plate with a hole, `parallel` from the front top edge gives the three other edges parallel to it that lie in the top and bottom faces: `[ { "faces": ["F2.C0","F2.S.e1"], "at": [50,0,0] }, { "faces": ["F2.C0","F2.S.e3"], "at": [50,60,0] }, { "faces": ["F2.C1","F2.S.e3"], "at": [50,60,10] } ]`. An empty part gives `[]`.

### recognize

Recognises the features of a body that was imported (a solid with no history) and proposes a feature tree that rebuilds it ([Feature recognition](../io/recognition.md)).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `apply` | boolean | `false` | Also store the rebuilt body under the tag `<tag>.rebuilt` so that it can be compared (`distance` with `tagB`). May be given at the top level or in `opts`. |

Result keys: `features` (a feature list in part-document form that rebuilds the body), `report` (the recognition strategy, the features and what could not be explained), `verify` (the comparison of the rebuilt body with the input body, or `null`), `errors` (strings from the rebuild), `unexplainedArea`, `unexplainedAreaFraction`, `applied`. For the plate with a hole the answer is the plate sketch and extrude, a hole feature, and `strategy: "prism"`. No body: `null`.

### fitQuadrics

Refits the facet fans of an imported mesh body (rings and arcs of planar strips) as cylinders and cones and, when the result validates and keeps the volume, replaces the tag's body ([Cylinders and cones from facets](../io/fit-quadrics.md)).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `opts.tol` | number | 0.5 % of the diagonal | Largest facet-to-surface deviation (chord sag), mm. |
| `opts.fitTol` | number | `tol` / 10 | Largest corner-to-surface deviation, mm. |
| `opts.maxStepDeg`, `opts.minStrips`, `opts.smoothDeg` | number | 30, 3, 20 | Largest angle between rails, fewest strips, crease below which coaxial groups are a curved profile. |
| `opts.cones`, `opts.merge` | boolean | `true` | Recognise cones; merge coplanar faces first. |
| `opts.volumeTol` | number | 0.05 | Keep the fit only if the relative volume change is smaller. |
| `opts.dryRun` | boolean | `false` | Report without changing the body. |

Result keys: `applied`, `reason` (`nothing`, `invalid`, `volumeChanged`, `dryRun`), `faces`, `volume`, `issues`, `groups`, `declined`. No body: `null`.

## Geometry

### inspect

Diagnostics for one selected thing.

| Parameter | Type | Meaning |
|---|---|---|
| `ref` | object | `{ "kind": "sketch", "id": "F1" }`. |

Only `kind: "sketch"` is implemented, and it needs a body in the state. The result is `{ type: "sketch", frame, regions, areas, open, texts? }`: the sketch plane, the number of closed regions, the area of each (outer minus holes) and the `open` flag; `texts` lists text entities with their ink boxes. For the plate sketch: `{ "type": "sketch", "frame": { ... }, "regions": 1, "areas": [6000.000000000001], "open": false }`.

All other cases are errors: no `ref` or no string `kind` is `cannot inspect nothing`; an unknown kind (`body`, `FACE`) is `cannot inspect <kind>`; a sketch id that does not exist, or a state with no body, is `sketch not found`; on a state without a body `face`, `edge`, `curve`, `plane` give `<kind> not found`, `vertex` gives `vertex without a point` (no `at`) or `no vertex`, and `feature` answers `{ "type": "feature" }`. With a body, the kinds `face`, `edge`, `curve`, `plane`, `vertex` and `feature` are accepted but not implemented: they answer an error that begins `notImplemented: inspect <kind>`. Use `describe`, `measure` and `resolve` for faces and edges.

### continuity

G0 to G3 continuity along edges ([Geometry](../kernel/geometry.md)).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `refs` | array | | Which edges: `{ "kind": "face", "id" }` (all edges of the face) or `{ "kind": "edge", "faces": [a, b], "at": [x, y, z] }`. |
| `whole` | boolean | `false` | With no `refs`, judge every edge of the body that is smooth enough to judge (corners and open edges are left out). |
| `required` | string | `"G2"` | The level the edges must meet: `G0`, `G1`, `G2` or `G3` (any case). Anything else means `G2`. |
| `tol` | object | `{ gap: 0.01, angle: 0.1, curvature: 0.05, rate: 0.1 }` | Tolerances: gap in mm, angle in degrees, curvature and rate as fractions. Each key overrides its default when it is a number. |

Each edge is sampled 49 times along its length. At every sample the two faces are compared: the gap between them (G0), the angle between the normals (G1), the relative difference of the normal curvatures (G2) and of their rates (G3). The result is `{ required, tol, edges }`, and each edge is

| Key | Meaning |
|---|---|
| `key`, `faces`, `sides` | The edge key, its two faces sorted, and the same two faces in the order used for the comparison. |
| `kind` | `"judged"` (the faces meet smoothly enough to compare), `"corner"` (every sample is at least 100 tolerances off tangent) or `"open"` (the edge does not have exactly two faces). |
| `status` | `"ok"`, `"warn"` or `"fail"` for a judged edge, else `null`. |
| `length` | Edge length. |
| `max` | The largest `[gap, angle, curvature, rate]` found, `null` for levels that were not evaluated. |
| `worst` | The worst sample, with the `level` it concerns; `null` unless judged. |
| `samples` | All samples: `{ t, s, at, normal, gap, angle, curvature: [kA, kB], g2, rate, g3, fails, status }`; `fails` is the first level that failed (`"G1"`) or `null`. |

On the rounded plate, the edge between the cap `F2.C1` and the fillet `F3.B0` meets G1 (angle 0, gap 3.5e-9) and its curvature jumps from 0 to the fillet's 0.2: with `"required": "G1"` it is `"kind": "judged", "status": "ok"`; with the default `G2` it is reported as failing at G2. The edge of the plain plate between `F2.C1` and `F2.S.e1` is `"kind": "corner", "status": null` with `angle: 90` in every sample. With no body: `{ required, tol, edges: [] }`.

### curvatureAt

Principal curvatures of a face at the point of its surface closest to a probe point.

| Parameter | Type | Meaning |
|---|---|---|
| `points` | array | `[{ "face": id, "p": [x, y, z] }]`. |

The result is an array, one entry per point. An entry is `{ p, n, k1, k2, d1, d2, gaussian, mean }`: the projected point on the (untrimmed) surface, the outward normal, the principal curvatures `k1 >= k2` (positive where the surface bends away from the normal) with their directions `d1`, `d2`, and the Gaussian and mean curvature; `singular: true` is added at a singular point of the surface, such as the pole of a sphere. An unknown face, or an entry without `face` and `p`, is `{ "error": "notFound" }`. On the fillet of radius 5: `{ "p": [50,1,8], "n": [0,-0.8,0.6], "k1": 0.2, "k2": 0, "d1": [0,-0.6,-0.8], "d2": [1,0,0], "gaussian": 0, "mean": 0.1 }`. On the planar top: all curvatures 0. If `points` is not an array the result is `[]`. No body: one `{ "error": "notFound" }` per element.

### curvatureMap

A mesh of every face with per-vertex principal curvatures, for a false-colour display. No parameters.

```json
{ "bodies": [ { "id": null, "faces": ["F2.S.e1", ..., "F4.S.h1"],
                "mesh": { "positions": <f32 buffer>, "normals": <f32 buffer>, "k": <f32 buffer>, "triFace": <i32 buffer> } } ],
  "failed": [] }
```

`faces` are the face ids; `triFace` indexes into that list. `positions` and `normals` have 9 numbers per triangle, `k` has 6 (k1, k2 at each vertex), and the four buffers follow each other in the binary area ([Replies and buffers](reply.md)). Planar faces use the display triangulation and curved faces are subdivided; curvature values are exact. `failed` lists faces that could not be meshed. No body: `{ "bodies": [], "failed": [] }`.

### comb

Curvature combs of curves and faces.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `refs` | array | | `{ "kind": "edge", "faces", "at" }` or `{ "kind": "face", "id" }`. |
| `density` | number | 50 | Samples per curve, clamped to 10 to 300. |
| `lines` | number | 5 | Number of comb lines across a face, clamped to 1 to 30. |
| `dir` | string | `"uv"` | For faces, the parameter directions to comb: `"u"`, `"v"` or `"uv"`. |

The result has one entry per ref: `{ "curves": [ { p, k, dir? } ] }` or `{ "error": "notFound" }` (a reference that does not resolve, or `kind: "curve"`) or `{ "error": "notCurve" }` (any other kind). `p` is a flat array of sample points `[x0, y0, z0, x1, ...]` and `k` a flat array of the curvature vectors at those points (the comb's teeth: the direction toward the centre of curvature, with the curvature as length); `dir` (`"u"` or `"v"`) is added for face curves. For the fillet face with `density: 10, lines: 2` each comb curve has 11 points, and the curvature vector has length 0.2 (1/5). Without a body: `notFound` for face, edge and curve refs, `notCurve` for the others.

### drawing

2D views of the body: projected visible and hidden edges, optional section cuts with hatching.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `views` | array | `[]` | Each view: `{ "name": "front" }` with a name from `front back left right top bottom iso`, or `{ "x": [..], "y": [..] }`, the view's right and up directions as 3-vectors (the view direction is their cross product). A section: `"section": { "point": [x, y, z], "hatch": 3 }` cuts the body by the plane through `point` perpendicular to the view direction and draws the cut face hatched with the given line spacing (default 3). |
| `hidden` | boolean | `true` | Include hidden lines. With a body, an absent value means `true`. With no body, only a `hidden` sent as true (or a non-zero number) gives `{}`; absent or false gives `null`. |

A view that has neither a known `name` nor both `x` and `y` fails the whole request with `drawing: a view needs a name (front, back, left, right, top, bottom, iso) or x and y`. The result is `{ "views": [view, ...] }` in request order. A view is `{ min, max, lines, edges, axes }`:

| Key | Meaning |
|---|---|
| `min`, `max` | The 2D bounds of the view. |
| `lines` | The drawn pieces: `{ k, v, p, e? }`: `k` the kind (`e` edge, `o` outline of a curved face, `t` tangent edge, `h` hatch), `v` whether the piece is visible, `p` a flat polyline `[u0, v0, u1, v1, ...]`, `e` the index into `edges` when the piece belongs to an edge. |
| `edges` | The model edges in the view: `{ faces, at, line \| circle, mid, ends }` with the face pair and a point `at` on the edge in 3D (the same reference `measure` takes), `line` the 2D segment or `circle` `{ c, r }`, the 2D midpoint and end points; `outline: true` for silhouette edges. |
| `axes` | Axis lines of cylindrical features as 2D segments `[[u0, v0], [u1, v1]]`. |

The standard views are fixed frames, given as the view's right, up and normal directions (`x`, `y`, `z` of the view):

| View | right | up | normal |
|---|---|---|---|
| `front` | +X | +Y | +Z |
| `back` | -X | +Y | -Z |
| `right` | -Z | +Y | +X |
| `left` | +Z | +Y | -X |
| `top` | +X | -Z | +Y |
| `bottom` | +X | +Z | -Y |
| `iso` | (0.7071, 0, -0.7071) | the normal crossed with right | (0.5774, 0.5774, 0.5774) |

The plate is extruded along Z, so its `top` view looks along the Y axis: the plate seen from the side. That view of the plate with a hole has `min [0, -10]`, `max [100, 0]`, 6 line pieces, 17 edges and one entry in `axes`, `[[[20, -10], [20, 0]]]`, the axis line of the hole. A section view through (20, 30, 5) adds edges between `F4.S.h1` and the cut face `SEC.C1`, and hatch lines (`k: "h"`). A section whose plane misses the body gives `{ "error": "cutNoEffect" }` for that view. If `views` is missing or not an array: `{ "views": [] }`. With no body: `null`, unless `hidden` is sent as true, which gives `{}`.

### refCurve

An edge, or a curve of a sketch, as an exact NURBS curve.

| Parameter | Type | Meaning |
|---|---|---|
| `ref` | object | `{ "kind": "edge", "faces": [a, b], "at": [x, y, z] }` or `{ "kind": "sketchCurve", "sketch": id, "entity": id }`. |

Result: `{ curve, start, end, closed }` where `curve` is `{ P, p, knots, w? }`: the control points, degree, knot vector and, for rational curves, the weights. Or `{ "error": "missingReference" }`. A straight edge of the plate gives degree 1; the circle of the hole gives a rational degree-2 curve with 9 control points and weights `1, 0.7071, ...`, `closed: true`. Without a body: `{ "error": "missingReference" }`.

```json
{ "type": "refCurve", "tag": "q", "ref": { "kind": "edge", "faces": ["F2.C1","F2.S.e1"], "at": [50,0,10] } }
```
```json
{ "curve": { "P": [[0,0,10],[100,0,10]], "p": 1, "knots": [0,0,100,100] }, "start": [0,0,10], "end": [100,0,10], "closed": false }
```

### convert

Projects edges, faces or sketch curves into a sketch frame as new fixed sketch entities.

| Parameter | Type | Meaning |
|---|---|---|
| `sketch` | object | The sketch to add to (`points`, `entities`, optional `seq`). |
| `frame` | object | The sketch plane `{ o, x, y, z }`, each a 3-vector. Anything else is the error `bad sketch frame`. |
| `ref` | object | An edge reference, a face reference, or `{ "kind": "sketchCurve", ... }` for a curve of a sketch in the state. |

Result: `{ sketch, ids, skipped }`: the sketch with the added points (marked `fixed`) and entities, the ids of the new entities, and the count of projected pieces dropped as degenerate. A line becomes a `line`; a circle parallel to the plane a `circle` or `arc`; a circle seen edge-on a `line`; a circle seen at an angle an `ellipse`; anything else a `spline` through 129 samples. Each entity carries `from: { kind: "edge", faces, at, shape }`. `null` when the reference does not resolve, and for a state with no body.

```json
{ "sketch": { "id": "S", "points": [ { "id": "p1", "x": 0, "y": 0, "fixed": true }, { "id": "p2", "x": 100, "y": 0, "fixed": true } ],
              "entities": [ { "id": "e3", "type": "line", "p1": "p1", "p2": "p2",
                              "from": { "kind": "edge", "faces": ["F2.C1","F2.S.e1"], "at": [50,0,10], "shape": "line" } } ],
              "constraints": [], "seq": 4 },
  "ids": ["e3"], "skipped": 0 }
```

### sketchOffset

Offsets sketch entities and returns the resulting chains. It needs no tag.

| Parameter | Type | Meaning |
|---|---|---|
| `sketch` | object | The sketch (with `points` and `entities`). |
| `ids` | array of strings | The entities to offset. |
| `distance` | number | The offset; a missing or non-numeric value is the error `sketchOffset without a distance`. |

The result is `{ "chains": [chain, ...] }`; a chain is a list of segments `{ kind, a, b, ... }` in sketch coordinates. A sketch with no entities gives `{ "chains": [] }`. A segment is `{ kind: "line", a, b }`, `{ kind: "arc", c, r, a, b, ccw }` or `{ kind: "circle", c, r, a, b, ccw }`. A positive distance moves a counter-clockwise outline inwards and a negative one outwards. An offset that cannot be made, because it would shrink an arc or circle to nothing, answers `{ "error": "offsetTooLarge" }` as the result (a circle of radius 4 offset by 10). Offsetting the plate outline by -5:

```json
{ "chains": [ [ { "kind": "line", "a": [-5,-5], "b": [105,-5] }, { "kind": "line", "a": [105,-5], "b": [105,65] },
                { "kind": "line", "a": [105,65], "b": [-5,65] }, { "kind": "line", "a": [-5,65], "b": [-5,-5] } ] ] }
```

The same outline offset by 40 gives the inner rectangle from (40, 40) to (60, 20), and a circle of radius 4 offset by 1 gives a circle of radius 3.

### drag

Moves one component of an assembly so that a grabbed point follows the cursor, honouring the mates of the component.

| Parameter | Type | Meaning |
|---|---|---|
| `face` | string | A face of the component to move. |
| `point` | `[x, y, z]` | The grabbed point. |
| `target` | `[x, y, z]` | Where it should go. |

Result: `{ insert, delta, place, moves }`: the id of the moved `insertPart`, the rigid motion as a frame, the new placement, and `moves`, a list `[{ id, delta, place }]` (only the dragged component moves; others keep their mates unsatisfied). A component without mates translates by `target - point`; with mates it reaches the closest point of its mate manifold (a point on the circle for a concentric mate, an in-plane slide for a coincident mate). Errors in the result: `missingReference` (unknown face, or no body), `dragNoComponent` (a face that belongs to no component), `dragFixed` (the component is fixed). Missing or non-numeric `face`, `point` or `target` is the error reply `drag: face, point and target are required`.

Two 20 x 20 x 10 blocks inserted by `insertPart`, `F1` fixed and `F2` free at x = 40, then a drag of the top face of `F2` by (5, 0, 0):

```json
{ "type": "drag", "tag": "asm", "face": "F2.C1@F2.1", "point": [50,10,10], "target": [55,10,10] }
```
```json
{ "insert": "F2", "delta": { "o": [5,0,0], "x": [1,0,0], "y": [0,1,0], "z": [0,0,1] },
  "place": { "o": [45,0,0], "x": [1,0,0], "y": [0,1,0], "z": [0,0,1] },
  "moves": [ { "id": "F2", "delta": { ... }, "place": { ... } } ] }
```

The regen of this assembly with `"opts": { "dofReport": true }` reports `components` `[ { "id": "F1", "dof": 0, ... }, { "id": "F2", "dof": 6, ... } ]` and `assembly.dof` `{ "total": 6, "translational": 3, "rotational": 3 }`. Faces of an inserted part are named `<face>@<insert id>.<n>`. On the plate (no components) `drag` answers `{ "error": "dragNoComponent" }`.

## Verification and measurement

These requests are read-only and never change the body. The entity forms, the options and the common `elapsedMs` / `truncated` fields are described first.

### Entities

Where a request takes an entity (`a`, `b`, `of`, `ids`):

- a face id as in `describe` (a duplicated id selects every face with it);
- an edge key (`F2.C0\|F2.S.e1#0`) or `e<n>`;
- a vertex `V<n>`;
- a point `[x, y, z]`;
- `lump:<n>` (the faces of one lump), or `"body"` (everything);
- an array of any of these, or an object `{ face | edge | vertex | lump | point | faces | edges }`.

Replies name the face, edge or vertex a point lies on as `{ kind, id }`.

### Budget: `elapsedMs` and `truncated`

The measurement requests (`distance` to `extrema`) carry `elapsedMs`, the wall time inside the request, and `truncated`. They stop their loops after `opts.timeMs` (default 8000 ms) and answer with what they have; `truncated: true` then means the result is partial: a lower bound for a maximum, or an incomplete list. Options may sit in `opts` or at the top level of the request. The clock is the host's; with a constant clock `elapsedMs` is 0 and nothing is cut off.

### section

The exact cross-section of the body at a plane.

| Parameter | Type | Meaning |
|---|---|---|
| `plane` | object | `{ "origin": [x, y, z], "normal": [x, y, z] }`; the normal is normalised and must not be zero. |

Result: `{ loops, frame, area, meshArea, bbox2d }`. `frame` is the plane's frame (`z` the unit normal, `x` the world Z crossed with the normal, or world X when the normal is within 2.5 degrees of Z); loop coordinates are in it. `loops` are `{ kind: "outer" \| "inner", closed, area, edges, pts }`; outer loops run counter-clockwise around material (positive `area`), inner loops clockwise around holes. `area` of the result is the sum (outer minus inner), `meshArea` the same from the display mesh, `bbox2d` is `[minx, miny, maxx, maxy]`. Edge records are `{ type: "line", from, to, face }`, `{ type: "arc", centre, radius, start, end, startAngle, sweep, ccw, face }` or `{ type: "polyline", pts, face }`, with `face` the persistent face id. A plane that misses the body gives `loops: []`, `area: 0`. When the cut leaves chains that do not close (for instance where a face failed to mesh), the result adds `openChains` (a count) and `openEnds`, a list of `{ start, end, startFace, endFace }` with 3D end points and the faces they lie on. If gaps in a chain were bridged to close it, `bridgedGaps` lists them as `{ from, to, length, face }`; both keys are absent when nothing was open or bridged. Cutting the plate at y = 30, through the centre of the hole, leaves two pieces of the 10 mm thick plate: 76 mm wide (area 760) and 16 mm wide (area 160), so `area` is 920:

```json
{ "loops": [ { "kind": "outer", "closed": true, "area": 760, "edges": [ { "face": "F2.C0", "type": "line", "from": [-100,0], "to": [-24,0] }, ... ], "pts": [[-100,0],[-24,0],[-24,10],[-100,10]] },
             { "kind": "outer", "closed": true, "area": 160, ... } ],
  "frame": { "o": [0,30,0], "x": [-1,0,0], "y": [0,0,1], "z": [0,1,0] }, "area": 920, "meshArea": 920.455, "bbox2d": [-100,0,0,10] }
```

How exact: the display mesh is cut by the plane, the cut is chained into loops, every vertex is moved onto the exact surfaces that meet there, and curved stretches are subdivided until the sagitta is under 2e-5 of the body diagonal. Planar faces give exact lines; a run on one circle (a cylinder or cone cut perpendicular to its axis, any sphere cut) becomes an exact arc; everything else is a polyline on the exact surface. A plane exactly through a planar face counts on-plane vertices as on the positive side. No body: `null`.

### sections

A stack of parallel sections and the volume integrated from them.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `axis` | string or array | required | `"x"`, `"y"`, `"z"` or `[nx, ny, nz]` (not zero); sections are perpendicular to it. |
| `n` | number | 21 | Number of cell centres between the mesh extent of the body along the axis. |
| `positions` | array of numbers | | Explicit positions (signed distances along the unit axis), 1 to 1000; used instead of `n`. |
| `exact` | boolean | `true` | `false` skips the `massProps` cross-check, which takes seconds on large bodies. |

Result: `{ axis, range, rule, positions, areas, loops, volume, exactVolume, volumeError }`. `rule` is `midpoint` (cell centres) or `trapezoid` (explicit positions, covering only the span between the first and last); `areas` and `loops` (count per position) are per section; `volume` is the integral; `exactVolume` and `volumeError` (relative difference) come from `massProps`. For the plate with `"axis": "z", "n": 3`: `positions` `[1.667, 5, 8.333]`, each `areas` entry `5949.73`, `loops` `[2, 2, 2]`, `volume` 59497.3453 against `exactVolume` 59497.3452 (`volumeError` 2e-9). The rule's error is about 1e-3 at n = 41 for smooth profiles; a profile with steps needs far more sections, while each section area stays exact. No body: `null`.

### probe

Inside/outside classification of points and the hits of rays.

| Parameter | Type | Meaning |
|---|---|---|
| `points` | array | `[[x, y, z], ...]`, at most 2000. |
| `rays` | array | `[{ "origin": [..], "dir": [..] }, ...]`, at most 2000; `dir` is normalised and must not be zero. |

At least one of the two is required. The result has `points` and `rays` for what was requested. A point answers `{ inside: true \| false \| "boundary", distance, face, nearest }` where `distance` is the signed distance to the nearest face (negative inside), `face` its id and `nearest` the foot point. A ray answers `{ hits: [{ t, point, face, entering, tangent? }] }` sorted by `t` (distance along the unit direction); `entering` is true when the ray goes from outside to inside; a hit on a shared edge is reported once; a ray lying in a face skips that face; `tangent: true` marks grazing hits.

```json
{ "type": "probe", "tag": "q", "points": [[50,30,5],[20,30,5]], "rays": [ { "origin": [-10,30,5], "dir": [1,0,0] } ] }
```
```json
{ "points": [ { "inside": true,  "distance": -5, "face": "F2.C0",   "nearest": [50,30,0] },
              { "inside": false, "distance": 4,  "face": "F4.S.h1", "nearest": [24,30,5] } ],
  "rays": [ { "hits": [ { "t": 10,  "point": [0,30,5],   "face": "F2.S.e4", "entering": true },
                        { "t": 26,  "point": [16,30,5],  "face": "F4.S.h1", "entering": false },
                        { "t": 34,  "point": [24,30,5],  "face": "F4.S.h1", "entering": true },
                        { "t": 110, "point": [100,30,5], "face": "F2.S.e2", "entering": false } ] } ] }
```

The second point is in the hole, so it is outside the material, 4 mm from the hole wall. No body: `null`.

### distance

Minimum distance between two entities.

| Parameter | Type | Meaning |
|---|---|---|
| `a`, `b` | entities | The two entities. |
| `tagB` | string | Take `b` from the body of another tag (a rebuilt part against its source). |
| `opts.samples` | number | Grid size per face for the multi-start search (default 7). |

Result: `{ distance, pointA, pointB, a, b, intersects, direction, aLabel, bLabel, elapsedMs, truncated }`. `a` and `b` name what each point lies on. When the entities touch, `intersects` is true, `distance` 0 and `intersection` is `{ "kind": "point" \| "curve" \| "area" }`; across two tags, one solid inside the other is `kind: "containment"` with `boundaryDistance`. The method is exact projection of points onto faces and edges, the closed form for two segments, and otherwise a multi-start alternating projection (best four starts, up to 400 iterations, stop at a relative gain under 1e-15), so for doubly curved pairs far from every start the minimum can be missed (raise `opts.samples`).

```json
{ "type": "distance", "tag": "q", "a": { "face": "F4.S.h1" }, "b": { "face": "F2.S.e4" } }
```
```json
{ "distance": 16, "pointA": [16,30,9.2857], "pointB": [0,30,9.2857], "a": { "kind": "face", "id": "F4.S.h1" }, "b": { "kind": "face", "id": "F2.S.e4" },
  "intersects": false, "direction": [-1,0,0], "aLabel": null, "bLabel": null, "elapsedMs": 1, "truncated": false }
```

The two overlapping blocks (`a: "lump:0"`, `b: "lump:1"`) give `distance: 0`, `intersects: true`, `intersection: { "kind": "curve" }`. The point on a face is any point of the minimum set: a whole line of the cylinder is equally near. No body: `null`.

### maxDistance

The one-sided Hausdorff distance: the largest distance from the surface of `a` (default the body) to `b` (default the body).

| Parameter | Type | Meaning |
|---|---|---|
| `a`, `b` | entities | Default: the whole body. |
| `tagB` | string | `b` from another tag. |
| `opts.samples` | number | Sample points over the body (default 3000). |
| `opts.both` | boolean | Also measure `b` to `a`. |

Result: `{ maxDistance, location, nearest, errorBound, upperBound, rms, samples, coverage, face, elapsedMs, truncated }`. The distance function is 1-Lipschitz, so the true maximum lies in `[maxDistance, maxDistance + errorBound]` where `errorBound` is the covering radius of the sample grid; `coverage` is the share of sample points measured. With `both: true` the reply is `{ hausdorff, errorBound, aToB, bToA }`. A body against itself gives 3.6e-15. No body: `null`.

### thickness

Wall thickness of a solid, by rays from surface samples along the inward normal to the first exit.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `opts.samples` | number | 2500 | About this many samples over all faces. |
| `opts.top` | number | 10 | How many thin places to list. |
| `opts.radius` | number | the larger of 2 x min and 2 % of the diagonal | Minimum distance between listed places. |
| `opts.below` | number | | Drop places thicker than this. |
| `opts.method` | string | `"ray"` | `"sphere"` refines the thinnest places with the largest inscribed ball (thickness = 2 r; slower). |
| `opts.sphereSamples` | number | 200 | How many places the sphere method refines. |

Result: `{ method, samples, noExit, minThickness, location, meanThickness, clusterRadius, thin }`. `location` and the entries of `thin` are `{ thickness, point, farPoint, face, farFace, direction }`, thinnest first. `noExit` counts samples whose ray found no exit. For the plate with 300 samples: `minThickness: 10`, between `F2.C0` and `F2.C1`. The value is a sampled minimum: a feature thinner than the sample spacing can be missed, and `ray` overestimates in wedges and corners. No body: `null`.

### thicknessAt

Wall thickness at one place.

| Parameter | Type | Meaning |
|---|---|---|
| `point` | `[x, y, z]` | The surface point nearest to this point is used. |
| `face` | string | Or: the minimum, maximum and mean over this face. |
| `opts.samples` | number | Samples on the face. |

With `point`: `{ distanceToSurface, result: { thickness, point, farPoint, face, farFace, direction }, elapsedMs, truncated }`. With `face`: `{ min: {...}, max, mean, samples, ... }`. Neither is `thicknessAt: point [x, y, z] or face id is required`. No body: `null`.

### closestPoint

The nearest part of the body to each of many points.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `points` | array | required | Up to 2000 points. |
| `on` | string | `"body"` | `"body"`, `"face"`, `"edge"` or `"vertex"`. |
| `ids` | array | all | Restrict `face` / `edge` / `vertex` to these. |
| `k` | number | 1 | Return the `k` nearest entities per point as a list. |

With `on: "body"` each result is `{ signedDistance, inside, nearest: { distance, point, entity, face, uv } }` (negative inside, like `probe`). With the other values the result is the nearest entity of that kind with `uv` (face) or curve parameter `t` (edge). For the point (50, 30, 20) above the plate, `signedDistance` is 10 with `entity` `{ kind: "face", id: "F2.C1" }`; with `on: "face"`, `k: 2` the list adds `F2.C0` at distance 20. No body: `null`.

### angles

Dihedral angles of edges and faces, and draft angles.

| Parameter | Type | Meaning |
|---|---|---|
| `edges` | `"all"` or array of keys | Per two-faced edge: `{ edge, faces, angle, normalAngle, type, min, max, at }`. `angle` is the interior angle through the material (a box edge is 90, a re-entrant edge of an L is 270); `type` is `convex`, `concave` or `smooth` (normals within 0.5 degrees); `min` and `max` are over five points along the edge. |
| `faces` | `[a, b]` | The angle between two faces: with a shared edge the dihedral angle (`sharedEdge`, `at`), else the angle between the normals at their nearest points (`angle`, `normalAngle`, `distance`, `parallel`). |
| `draft` | `{ pull, faces?, min? }` | The draft angle of each face (or the listed faces) against the pull direction: `asin(n . pull)` in degrees over a 5 x 5 grid, as `{ face, draft, min, max, class }`; `class` is `vertical` (under 0.01 in magnitude), `undercut` (negative), `parting` (over 89.99) or `ok`. With `min`, walls below that draft are listed in `violations`. |

At least one of the three is required. For the plate, the straight edges are `convex` at 90; `draft` against +Z shows the four sides `vertical`, `F2.C0` an `undercut` at -90 and `F2.C1` the `parting` face at 90. No body: `null`.

### clearance

Face pairs closer than a minimum distance.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `min` | number | required | The distance under which pairs are reported. |
| `a`, `b` | entities | all faces | Restrict the pairs to these sets. |
| `opts.include` | string | `"both"` | `"gap"`, `"wall"` or `"both"`. |
| `opts.limit` | number | 200 | Maximum pairs listed. |
| `opts.maxPairs` | number | 4000 | Candidate pairs examined, closest boxes first. |

Result: `{ min, count, pairsTested, listedAll, pairs, elapsedMs, truncated }` with `pairs` sorted by distance: `{ a, b, distance, kind, pointA, pointB }`, where `kind` says what lies between the faces: `gap` (air), `wall` (material, for example a hole to the edge of the part, the figure `thickness` reports) or `contact`. Pairs of faces that share a vertex are skipped (they touch by construction), so the two faces of a tight V groove are not reported. For the plate with `min: 30`: the caps at 10 (`wall`), the hole wall to the left side at 16, and to the front and back sides at 26. No body: `null`.

### symmetry

Mirror symmetry planes and the deviation of the mirrored body.

| Parameter | Type | Meaning |
|---|---|---|
| `planes` | array | `[{ origin, normal }]`; default: the coordinate axes and the principal axes through the centre of mass. |
| `opts.samples` | number | Sample points on the surface (default 600). |
| `opts.tol` | number | Deviation under which a plane counts as symmetric (default 1e-4 of the diagonal). |
| `opts.full` | boolean | Measure every sample even after a plane already failed. |

Result: `{ tol, samples, symmetric, best, planes }`. Each plane record is `{ origin, normal, basis, maxDeviation, rmsDeviation, symmetric, exact, worstPoint }`: the surface is sampled, mirrored and measured against the body. `symmetric` lists the planes within `tol`, `best` the least deviating, `planes` all, sorted. A plane has `exact: false` when it was abandoned early and its `maxDeviation` is a lower bound. For the plate with a hole the planes normal to Z and Y through the centre of mass are symmetric (deviations 5e-15); the X plane is not. The X plane is listed twice in `planes`, once through the centre of mass (`basis`: "axis x through the centre of mass") and once through the bounding-box centre ("axis x through the bbox centre"), so the example returns four records. No body: `null`.

### extrema

The lowest and highest point of the body along a direction.

| Parameter | Type | Meaning |
|---|---|---|
| `dir` | `[x, y, z]` | Any length; not zero. |
| `of` | entity | Measure this instead of the whole body. |
| `opts.samples` | number | Grid size per face (default 16, at most 64). |

Result: `{ direction, min, max, extent, method, exact, samples, ofLabel, elapsedMs, truncated }`, where `min` and `max` are `{ value, point, entity, at }`: the value `dir . p`, one point where it is attained, what that point lies on, and everything that attains it (at most 20: faces whose interior reaches it, else edges, else vertices). `method: "vertices"`, `exact: true` means every face searched is planar and every edge straight, and the values are exact. Otherwise `"sampled+refined"`: curved faces and edges are searched on a grid and refined to about 1e-9 of the cell size; an extreme of a free-form face narrower than the grid spacing can be missed. For the plate along Z: `min.value` 0 on `F2.C0` (and all its edges and vertices), `max.value` 10 on `F2.C1`, `extent` 10. No body: `null`.

## Design rules and manufacturing

### profiles

Lists the built-in design-rule profiles and the rule catalogue. No parameters and no tag.

```json
{ "profiles": [ { "id": "fdm-print", "name": "FDM 3D printing", "description": "...", "rules": [
      { "id": "openShell", "title": "Watertight", "description": "...", "severity": "error", "params": [] }, ... ] },
    { "id": "cnc-3axis", ... }, { "id": "injection", ... } ],
  "rules": [ ... ] }
```

Each rule has `id`, `title`, `description`, `severity` (`error`, `warn` or `info`) and `params`: `{ name, type, default, value, unit, min?, max?, description }` where `type` is `number` or `vec3`, `value` is what the profile uses and `default` what the rule uses on its own. The three profiles are `fdm-print` (rules `openShell`, `bodyCount`, `minWall`, `overhang`, `bridge`, `minFeature`, `plateContact`), `cnc-3axis` (`openShell`, `minInternalRadius`, `minWall`, `undercut`) and `injection` (`openShell`, `draft`, `uniformWall`, `minWall`).

### checkProfile

Runs a profile on a body.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `profile` | string or object | required | A built-in id, or an inline `{ id?, name?, rules: [{ id, params?, severity? }] }`. Rules not listed do not run; omitted `params` keep the rule defaults. Unknown profile, rule or parameter, a wrong type or an out-of-range value are errors that list the valid choices. |
| `tag` | string | `main` | The body to check. |
| `part` | object | | Instead of the tag: regenerate this document privately (through the regen cache; no tag is created) and check it. A part that makes no body answers `{ profile, noBody: true, errors }`. |
| `opts.samples` | number | 3000 | Surface samples for the sampled rules. |
| `opts.maxIssues` | number | 100 | Cap per rule; the rest is counted in `summary.truncated`. |

Result: `{ profile, ok, issues, summary }`. `profile` is the effective profile with its parameters. `ok` is true when there is no `error`-severity issue. Each issue is `{ rule, severity, message, faces, edges, value, limit, unit, at? }`: an actionable sentence with the numbers, the faces and edges by persistent name, the measured value and the limit, and a point to focus on. `summary` is `{ errors, warnings, infos, total, rules: [{ rule, severity, issues, passed }], truncated, skippedFaces, samples }`. The rules are a first slice and not a certification: some are exact (hole diameters, concave radii, watertightness), some use the display mesh (overhang, bridge, draft), some sample rays (wall thickness) and can miss features thinner than the sample spacing. Sheet-metal rules, user rules and build orientation are not covered. With neither a body nor a `part`: `null`.

### dfmMinWall, dfmMinInternalRadius, dfmPocketRatio, dfmOverhang, dfmConstantSection

One design-for-manufacture rule each, with the limit as a required argument. They answer with the measured value, where it is (face names, an edge key where an edge is the thing, and a point) and which rule was broken. Replies are deterministic and numbers are rounded to 1e-6.

| Request | Parameters | Question |
|---|---|---|
| `dfmMinWall` | `threshold` (> 0); `opts.samples` | Is the thinnest wall at least `threshold`? (the `thickness` ray method; default 6000 samples) |
| `dfmMinInternalRadius` | `toolRadius`; `direction` (default `[0,0,1]`, the tool axis) | Are all concave corners parallel to the tool axis (within 5 degrees) at least as round as the cutter? |
| `dfmPocketRatio` | `maxRatio` | Is depth over width of every closed pocket (planar floor, every boundary edge concave) within the limit? |
| `dfmOverhang` | `direction` (build direction, up), `angle` (0 to 90, from vertical), `minArea` | Which downward faces are steeper than `angle` from vertical (0 wall, 90 flat ceiling)? Faces on the build plate are ignored. |
| `dfmConstantSection` | `axis` | Is the body a constant extrusion along `axis`, else where does the section first change? |

Common result keys: `rule` (`{ id, <limit>, unit, description }`), `supported` (`true`), `pass`, `violations` (each repeating the rule id, the measured value and the `limit`). Specific keys: `dfmMinWall` has `threshold`, `minimum` (`{ thickness, faces, point, farPoint }` or `null`), `samples`, `truncated`; `dfmMinInternalRadius` has `toolRadius`, `direction`, `minimum`, `corners` (all, ascending radius, at most 200; `kind` is `fillet` or `sharp`); `dfmPocketRatio` has `maximum` and `pockets` (`{ floor, walls, faces, depth, width, ratio, limit, fails, point }`, deepest first); `dfmOverhang` has `faces` (`{ face, overhangAngle, overhangArea, faceArea, supportVolume, point }`), `overhangArea`, `supportVolume` (a rough estimate); `dfmConstantSection` has `start`, `end`, `length`, `constant` and either `station` (`{ position, offset, faces, point, sectionAreaBefore, sectionAreaAfter }`) or, when constant, `sectionArea` and `loops`.

A body that a rule cannot judge is not guessed at. The reply is `{ "rule": { "id": ... }, "supported": false, "pass": null, "refusal": { "code", "reason", "faces"? } }` with `code` `emptyBody` (no faces; all rules), `nonAnalyticFaces` (a face is B-spline, screw or offset, or an extrusion or revolution of a spline profile: lofts, free-form and most imported bodies; for `dfmMinInternalRadius`, `dfmPocketRatio`, `dfmConstantSection`) or `tessellationFailed` (some faces did not mesh; for `dfmOverhang`, `dfmConstantSection`). Bad arguments are error replies prefixed with the request name (`dfmMinWall: threshold must be a positive number`). No body: `null`.

On the plate with a hole: `dfmMinWall` with `threshold: 12` fails with `minimum.thickness` 10 between `F2.C0` and `F2.C1`; `dfmMinInternalRadius` with `toolRadius: 3` passes with `corners: []` (a drilled hole is not a corner); `dfmPocketRatio` with `maxRatio: 4` passes with `pockets: []`; `dfmOverhang` upward with 45 degrees passes; `dfmConstantSection` along Z answers `constant: true`, `sectionArea` 5949.734518 and `loops` 2.

## Sketches

### sketchAnalyze

Structural analysis of one sketch: exact, structured feedback instead of a bare `noClosedProfile`.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `featureId` | string | required | The id of the sketch feature. |
| `tolerance` | number | `0.01` | Near-coincident distance in mm; `<= 0` disables. |
| `tiny` | number | `0.001` | Tiny-segment length in mm; `<= 0` disables. |
| `tag` or `part` | | | The sketch comes from the tag's regenerated state, or from a private regen of `part` (the way to analyse a document that made no body, for example one failing `noClosedProfile`). |

Result: `{ tolerance, tiny, findings, chains, profile }`. A finding has a `kind`, the `entities` and `points` it names, and `at` in sketch coordinates:

| kind | fires when | extra |
|---|---|---|
| `openEnd` | an entity end touches nothing | `gap` to the nearest other open end, `nearest` entity |
| `nearCoincident` | two points closer than `tolerance` with no coincident constraint | `distance` |
| `duplicate` | two entities of the same kind on the same points | |
| `overlap` | two collinear lines whose interiors overlap | |
| `zeroLength` | a curve of length or radius 0 or less | |
| `tinySegment` | length under `tiny` | `length` |
| `branch` | more than two entity ends at one point | `degree` |
| `isolatedPoint` | a point no entity uses | |
| `selfIntersection` | two entities cross inside their interiors | |

`chains` are the connected sets of entities, each `{ entities, closed, construction }`. `profile.closedChains` counts closed non-construction chains; when it is 0, `profile.reason` says why: the biggest `openEnd` finding, `{ "kind": "empty" }` or `{ "kind": "constructionOnly" }`. For the plate sketch: `{ "tolerance": 0.01, "tiny": 0.001, "findings": [], "chains": [ { "entities": ["e1","e2","e3","e4"], "closed": true, "construction": false } ], "profile": { "closedChains": 1 } }`. Errors: `sketchAnalyze: featureId must be a string`, `sketchAnalyze: sketch F9 not found`. With neither a body nor a `part`: `null`.

### sketchFreedoms

What the solver can still move in one sketch.

| Parameter | Type | Meaning |
|---|---|---|
| `featureId` | string | The sketch feature id. |
| `tag` or `part` | | As for `sketchAnalyze`. |

Result: `{ dof, solved, points, radii, entities }`. `dof` is the same number as the regen `status.dof`. `points` has every point with `free`: 0 (fixed or determined), 1 (moves along exactly `dir`, a unit sketch direction) or 2 (any direction). `radii` lists circles, ellipses and arcs with whether the radius is free. `entities` lists each entity with the sum of its points' `free` plus 1 for a free radius. For the unconstrained plate sketch: `dof: 8`, every point `free: 2`, every line `free: 4`. Errors carry the `sketchFreedoms:` prefix with the same texts as `sketchAnalyze`; `null` without a body or `part`.

## Import

The four import requests read a file into a body and store it under the tag, with the same rules: a failure leaves the tag's previous state alone, and the reply has the shape of a `regen` result (`errors`, `components`, ..., `body`, `surfaces`) plus an `import` object. The imported body has its own face names (`I1.F1` and so on for STEP, `skp.F<n>` for SketchUp) and can be used with every body query.

### importStep

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `data` | string | required | The STEP file text. Missing: the error reply `importStep: data must be a STEP file as a string`. |
| `name` | string | `"part"` | Name of the import. |
| `heal` | `"off"` or object | on | Healing options as in `heal`; `healTolerance` also works. |

The result's `import` is `{ solids, faces, unitScale, warnings, heal: { fixed, remaining, toleranceUsed } }`. A file that cannot be read answers `body: null` and an `errors` entry `["import", { "code": "parse", "params": { "message": "not a STEP file (missing ISO-10303-21 header)" } }]`. Round trip of the plate with a hole through `step` and `importStep`: `import` `{ "solids": 1, "faces": 7, "unitScale": 1, "warnings": [], "heal": { "fixed": 0, "remaining": 0, "toleranceUsed": 0 } }`, volume 59497.345175425646. See [STEP](../io/step.md).

### importSat

`data` is the SAT file text (required: `importSat: data (the SAT file text) is required`); `name` is optional. A failure answers `body: null` and `errors` in the same pair shape as the other importers: `[ ["import", { "code": "sat.parse", "params": { "message": "bad SAT header (version)", "entity": null, "kind": "", "line": 1 } }] ]`. See [ACIS SAT](../io/sat.md).

### importSldprt

`data` is the base64 of a `.SLDPRT` file (required: `importSldprt: data (the .SLDPRT file as base64) is required`); `name` is optional. The result's `import` describes the body read (`solids`, `faces`, `container`, `schema`, ...) and the file's metadata: `features` (the feature tree with ids, types and dimension values), `configurations`, `material`, `customProperties`, `configurationProperties`, `units`, `versions`, `thumbnail` (base64 PNG or `null`), `faceAttributes` (colour and building feature of each face, by face id) and `bodyAttributes`. User names and paths recorded in the file are not returned. A failed import still carries the metadata in `import` when the file's container opens. A file that is not a SolidWorks part answers `errors` `[ ["import", { "code": "container.format", "params": { "message": "neither an OLE2 compound file nor a SolidWorks block container" } }] ]`. See [SolidWorks](../io/sldprt.md).

### importSkp

`data` is the base64 of a `.skp` file (required: `importSkp: data (the .skp file as base64) is required`); `name` is optional. `opts` takes `hidden` (boolean), `maxFaces` (number) and `up` (`"z"` default, `"y"` or `"yFlip"`: how the file's axes map onto the kernel's; another value is an error reply). The result is a surface body: one planar face per SketchUp face (holes kept), vertices welded, and `import.faceMaterials` listing the colour index of each `skp.F<n>` face. A file that is not a SketchUp file answers `errors` `[ ["import", { "code": "container.magic", "params": { "message": "not a SketchUp file (bad header magic)", "offset": 0 } }] ]`. In a module built without the `skp` feature the request answers an error reply that begins `notImplemented: importSkp`. See [SketchUp](../io/skp.md).

## Text, fonts and tables

These requests use no tag. Two fonts (`gauge`, `engrave`) are compiled in; further fonts are registered for the session. Fonts are session state, shared by all tags, and are part of the regen cache key by hash. See [Text and fonts](../io/text.md).

### textMetrics

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `string` | string | required | The text. |
| `font` | string | `gauge` | A font name. |
| `height` | number | required | Cap height in mm; a non-positive value is `textBadHeight: text height must be a positive number`. |
| `spacing` | number | | Letter spacing. |

Result: `{ advance, bbox, width, capHeight, chars, advances, warnings }`: the advance width, the ink box `[minx, miny, maxx, maxy]`, its width, the number of characters and the advance of each. For "Hello" at height 5: `advance` 12.129, `width` 11.386, `chars` 5.

### textOutline

Takes the fields of a sketch text entity (`string`, `font`, `height`, `at`, `anchor`, `angle`, `spacing`, `mirror`, `arc`) and returns the outline: `{ regions, bbox, area, warnings }`, where `regions` is a list of `{ outer: [[x, y], ...], holes: [[[x, y], ...]] }` polylines in sketch coordinates. "Hi" at height 5 gives 3 regions (H, the stem of the i and its dot), `bbox` `[0, 0, 4.157, 5.121]` and `area` 10.696.

### loadFont, unloadFont, listFonts, subsetFont

| Request | Parameters | Result |
|---|---|---|
| `loadFont` | `name`, `data` (base64 TTF or OTF), `replace?` | `{ name, hash, builtin, family, bytes, glyphs, chars, unitsPerEm, capHeight, outlines, supported, coverage, warnings, alreadyLoaded, replaced }`. Loading the same name and file again answers the same record with `alreadyLoaded: true`. Errors: `fontBadData`, `fontBadName`, `fontReserved`, `fontNameConflict` (same name, other file, no `replace`), `fontTooLarge`, `fontTooMany`. |
| `unloadFont` | `name` | `{ name, unloaded }`; `unloaded` is `false` for a name that was not loaded. |
| `listFonts` | none | `{ fonts: [...] }`, the built-in fonts first; each entry has the `loadFont` fields. |
| `subsetFont` | `name`, `text` | `{ name, hash, bytes, glyphs, data, warnings }`, with `data` the base64 of a font file holding only the glyphs of `text`. An unknown font is the error reply `textFontNotLoaded: font "x" is not loaded; load it with {type:"loadFont", ...}`. |

Subsetting `gauge` to "Hi" gives a 756-byte font with 3 glyphs; loading that as `mini` answers `{ "name": "mini", "builtin": false, "bytes": 756, "glyphs": 3, "chars": 2, "unitsPerEm": 1000, "capHeight": 700, "outlines": "glyf", "supported": ["Basic Latin"], "coverage": [ { "block": "Basic Latin", "mapped": 2, "of": 95 } ], "warnings": [], "alreadyLoaded": false, "replaced": false, ... }`. A sketch text entity that names a font that is not loaded fails its sketch with `textFontNotLoaded`.

### threadData

The thread tables, for dropdowns and for scripts.

| Parameter | Type | Meaning |
|---|---|---|
| `standard` | string | A standard id: `metric` (coarse), `metric-fine`, `UNC`, `UNF`, `BSP`, `NPT`, `Tr`, `Acme`; aliases (`ISO`, `G`, `bspp`) and any case work. |
| `size` | string | A size designation (`M6`, `1/4-20`, `G1/4`). |

Without `standard`: `{ units: "mm", standards: [ { id, description, count, sizes: [...] } ] }`. With it: `{ standard, units, sizes: [...] }` with every number for each size. With both: the one entry. A size record is `{ size, standard, major, pitch, tpi, minorInternal, minorExternal, pitchDiameter, angle, tapDrill, tapDrillName?, classInternal, classExternal, tapered }`:

```json
{ "type": "threadData", "standard": "metric", "size": "M6" }
```
```json
{ "standard": "metric", "units": "mm", "sizes": [ { "size": "M6", "standard": "metric", "major": 6, "pitch": 1, "tpi": 25.4,
    "minorInternal": 4.9175, "minorExternal": 4.7731, "pitchDiameter": 5.3505, "angle": 60, "tapDrill": 5,
    "classInternal": "6H", "classExternal": "6g", "tapered": false } ] }
```

`major` is the nominal diameter (for NPT the pipe outside diameter); `minorInternal` is the crest of an internal thread and `minorExternal` the root of an external thread; `tapDrill` is the bore an internal thread is cut into. An unknown standard or size is an error reply that lists the known choices (`unknown thread standard zzz (known: metric, metric-fine, UNC, UNF, BSP, NPT, Tr, Acme)`). A `standard` or `size` that is not a string is `threadData: <key> must be a string`.

## See also

- [Protocol overview](overview.md)
- [Regeneration](regen.md)
- [Errors](errors.md)
- [Mass properties and interference](../validation/analysis.md)
- [The validate request](../validation/validate.md)
- [Drawings](../io/drawings.md)
