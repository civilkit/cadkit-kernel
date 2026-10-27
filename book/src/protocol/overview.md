# Protocol overview

The kernel is driven by messages: one JSON request in, one framed reply out. This chapter describes the request shape, the state the kernel keeps between messages, the options that apply to every message, and lists all 57 request types. The reply framing is in [Replies and buffers](reply.md), the main request in [Regeneration](regen.md), the others in [Queries](queries.md), the failure model in [Errors](errors.md), and the HTTP service that serves the same messages in [The native service](native-service.md).

## Request shape

A request body is a UTF-8 JSON document with two keys:

```json
{ "id": 7, "msg": { "type": "regen", "tag": "main", "part": { "...": "..." }, "opts": { "upTo": 3 } } }
```

| Key | Meaning |
|---|---|
| `id` | A number chosen by the host to match replies to requests. The kernel ignores it and does not echo it: the reply to a call is the return value of that call. |
| `msg` | The message: an object. The first `msg` key in the document is the one used. |
| `msg.type` | The request type, a non-empty string. The 57 types are listed [below](#request-types). |
| `msg.tag` | Which kernel state the message uses (see [Tags and sessions](#tags-and-sessions)). A string; anything else, or no `tag`, means `"main"`. |
| `msg.debug` | The boolean `true` adds timing information to the reply ([Debug](#debug-timing-trace)). Any other value is ignored. |

Other keys of the envelope are ignored. The remaining keys of `msg` depend on the type.

A body that is not JSON, a document without an object `msg`, a `msg` with no type, and an unknown type are answered with an error reply, not with a trap:

| Body | `error` |
|---|---|
| not valid JSON | `bad request: the body must be JSON {id, msg}` |
| `msg` missing, or not an object | `bad request: msg must be an object` |
| `msg.type` missing, empty or not a string | `message without a type` |
| unknown type, for example `nope` | `unknown request nope` |

## Tags and sessions

A **session** is one kernel instance: one instantiation of the wasm module, or one session of the native service. A session owns its state, and sessions do not share anything.

Inside a session the state is kept per **tag**. A tag is a name for one regenerated document. `regen` computes a document and stores the result under its tag: the body (the B-rep), the solved sketches and the assembly state. Later requests that name the same tag read that state without running the part again. For example, `massProps` with `"tag": "plate"` measures the body that the last `regen` with `"tag": "plate"` produced. Typical clients use fixed tags for fixed purposes (`main` for the document on screen, `preview` for an edit being tried) and different tags for different documents.

What the rules are:

- A `regen` replaces the state of its tag. It does not touch other tags. If the part produces no body (it has no features, or only sketches), the tag is stored with no body.
- A `regen` whose `part` is not a part document (`format` is not `"ck-part"`, or `features` is not an array) fails with `invalid part: Format` and **keeps** the previous state of the tag. A regen in which individual features fail is not a failed request: it succeeds, reports the failures in `errors` and stores what was built ([Regeneration](regen.md)).
- A request that needs state, for a tag that has never been stored, fails with `noState: nothing regenerated yet`. Clients that restore their state after a kernel restart can key on the prefix `noState`.
- A request that needs a body, for a tag that has state but no body, does not fail: it answers an empty value of its own shape, in most cases `null` ([Queries](queries.md) gives the answer of each type).
- `importStep`, `importSat`, `importSldprt` and `importSkp` store a body under their tag when the import succeeds, and leave the tag alone when it fails. `recognize` with `opts.apply` stores a rebuilt body under `<tag>.rebuilt`.
- State lives as long as the session. The warm caches that make a repeated regen fast (the regen cache, the display-mesh caches) live with the session too; a capped amount of memory, 256 MiB by default for the regen cache, can be changed with `opts.cacheCap` ([Regeneration](regen.md)). A new instance starts empty.

Some requests take everything they need in the message and use no tag: `ping`, `matchEdges`, `sketchOffset`, `refDiff`, `profiles`, `threadData`, `textMetrics`, `textOutline`, the font requests, and `checkProfile`, `sketchAnalyze` and `sketchFreedoms` when called with a `part`. They can be sent before any `regen`.

## `opts.upTo`

`regen` takes an options object. The one that matters for every client is `upTo`:

```json
{ "type": "regen", "tag": "preview", "part": { "...": "..." }, "opts": { "upTo": 2 } }
```

With `opts.upTo = n` (a number, 0 or more) only the first `n` features of the part are regenerated, as if the others were not there. A negative or non-numeric value is ignored. It is the way to look at a part at an earlier step (a history slider, "roll back to here") and to find the feature that broke a part. The part's own `rollback` field acts the same way, and the smaller of the two wins. Features marked `suppressed` are skipped. `matchEdges` takes `upTo` at the top level of the message. The other options of `regen` are described in [Regeneration](regen.md).

## Debug: timing trace

`"debug": true` in `msg` makes the kernel record a trace of its steps:

```json
{ "id": 1, "msg": { "type": "regen", "tag": "plate", "part": { "...": "..." }, "debug": true } }
```

The reply gets two more keys after `ms`: `buffers`, with the number and total size in bytes of the binary buffers of this reply (successful replies only), and `trace`, a list of steps in the order they started:

```json
"buffers": { "count": 5, "bytes": 11648 },
"trace": [
  { "at": 0.02,  "ms": 0.88,  "depth": 0, "label": "regenerate 4 features", "level": "span" },
  { "at": 0.03,  "ms": 0.02,  "depth": 1, "label": "F1 sketch", "level": "span" },
  { "at": 0.05,  "ms": 0.04,  "depth": 1, "label": "F2 extrude → 6 faces", "level": "span" },
  { "at": 0.09,  "ms": 0.80,  "depth": 1, "label": "F4 hole → 7 faces", "level": "span" },
  { "at": 0.88,  "ms": 0.01,  "depth": 2, "label": "boolean Subtract (6 × 3 faces) → 7 faces", "level": "span" },
  ...
  { "at": 0.92,  "ms": 1.37,  "depth": 0, "label": "snapshot for the view (display meshes, edges)", "level": "span" },
  { "at": 0.92,  "ms": 1.22,  "depth": 1, "label": "tessellate 7 faces → 104 triangles", "level": "span" }
]
```

Each entry is one span: `at` is the start in milliseconds since the request began, `ms` the duration, `depth` the nesting, and `label` what ran. A feature's label ends with `→ n faces` when it succeeded, or with `: <code>` when it failed; the stages of a boolean (`locate`, `imprint`, `apply`, `classify`, `assemble`, `simplify`, `check`) are listed under it. A request with `debug` never uses the regen cache or the view cache, so the trace shows real work. (The values above are from one real run on a 4-feature plate and rounded.) Error replies carry `trace` too, without `buffers`.

## Time: `ms` and the host clock

Every reply has `ms`, the time the call took, measured with the host's `ck_now` import and rounded to whole nanoseconds. A host whose `ck_now` returns a constant sees `ms: 0` (the runner used in this book does). `ck_now` also drives two time budgets of `regen`, `opts.featureTimeMs` and `opts.regenTimeMs`, and the `timeMs` budget of the measurement requests; without a real clock they never expire.

## Concurrency model

- A call is synchronous and runs to completion before it returns. One instance runs one request at a time; a host that has several requests to make queues them. Because every call may take seconds on a large part, hosts run the instance in a worker (a Node worker thread, a browser worker) so that their own thread stays free.
- There is no way to cancel a call in flight. A client that wants to abandon a request discards the reply (the native client's `cancel()` drops requests that are still queued, not the running one) or, in the extreme, throws the instance away.
- Parallelism is by instances: two instances share nothing and can run at the same time on two threads. The native service does this, one worker thread per session, with a configurable limit on sessions. Inside a call the kernel is single threaded.
- A panic is contained by the instance boundary. The wasm module is built to abort on panic, so the call traps, and the host starts a new instance ([Call it from Node](../getting-started/node.md)); the native service, when built with the `service` profile (`panic = "unwind"`), catches the panic and ends only that session with HTTP status 500.

## Compatibility

Replies grow additively: new keys appear, existing keys keep their meaning. Optional blocks (`warnings`, `threads`, `limits`, `assembly`, `cacheStats`, `partVersionNewer` in the regen result) are present only when they have content. A client should ignore keys it does not know. The wire protocol has one version number (`KERNEL_PROTOCOL`, currently 1 in the TypeScript client; the native service reports its own in `GET /version`), which changes only for an incompatible change of a message or of the framing. A part document with a `version` higher than the kernel knows still regenerates, and the result says so (`partVersionNewer`).

## Request types

There are 57 request types. "Tag" says what the request needs: **tag** means a regenerated tag (`tag`, default `main`) and answers `noState` without one; **body** means it also needs a body in that state; **none** means it uses no state; **part** means it regenerates a `part` given in the message and stores nothing; **sets tag** means it stores state under the tag.

### Document and session

| Type | Tag | Purpose |
|---|---|---|
| `ping` | none | Liveness check. Returns `{"pong": true}` and, if the module was built with `CK_BUILD_REV`, the source revision as `build`. |
| `regen` | sets tag | Regenerate a part document: run its features and store the body. Returns errors, sketches, planes, the body with its display mesh, and more. |
| `step` | body | Export the tag's body as a STEP file (the reply is the file text as a JSON string). |
| `validate` | body | Run the solid validator on the body, plus a mesh watertightness check. |
| `heal` | body | Repair the body in place, or report what would be repaired (`dryRun`). |

### Import

| Type | Tag | Purpose |
|---|---|---|
| `importStep` | sets tag | Read the solids of a STEP file (text) into a body. |
| `importSat` | sets tag | Read an ACIS SAT file (text) into a body. |
| `importSldprt` | sets tag | Read a SolidWorks part file (base64) into a body. |
| `importSkp` | sets tag | Read a SketchUp file (base64) into a surface body of planar faces. Needs a module built with the `skp` feature. |
| `recognize` | body | Recognise the features of an imported solid and propose a feature tree. |

### Geometry queries

| Type | Tag | Purpose |
|---|---|---|
| `measure` | body | Facts about faces and edges; for two references also their distance, angle and offsets. |
| `massProps` | body | Volume, area, centre of mass and principal moments; per body and with densities. |
| `interference` | body | Overlap volume of every pair of separate bodies in the state. |
| `inspect` | body | Diagnostics for a selected sketch (regions, areas). |
| `continuity` | body | G0 to G3 continuity along edges. |
| `curvatureAt` | body | Principal curvatures of faces at the points nearest to given points. |
| `curvatureMap` | body | A false-colour curvature mesh for every face. |
| `comb` | body | Curvature combs of edges and faces. |
| `drawing` | body | 2D views of the body: projected edges, hidden lines, sections with hatching. |
| `refCurve` | body | An edge or a sketch curve as an exact NURBS curve. |
| `convert` | body | Project edges, faces or sketch curves into a sketch frame as new sketch entities. |
| `drag` | body | Move one assembly component so that a grabbed point follows a target, honouring its mates. |

### Selection and naming

| Type | Tag | Purpose |
|---|---|---|
| `matchEdges` | part | Edges similar, parallel or tangent to seed edges, for edge picking. |
| `resolve` | body | Turn descriptions of faces and edges ("the top face", "the hole barrel") into persistent names. |
| `describe` | body | Facts and a one-line text for named faces and edges. |
| `refDiff` | part | Compare two versions of a part and report references that now point at a different entity. |
| `check` | body | Check expectations (volume, counts, size, ...) against the body. |

### Verification and measurement

| Type | Tag | Purpose |
|---|---|---|
| `section` | body | Exact cross-section loops of the body at a plane. |
| `sections` | body | A stack of sections along an axis and the volume integrated from them. |
| `probe` | body | Inside/outside and signed distance of points; hit lists of rays. |
| `distance` | body | Minimum distance of two entities. |
| `maxDistance` | body | One-sided Hausdorff distance between two entities or bodies. |
| `thickness` | body | Minimum wall thickness and the thinnest places. |
| `thicknessAt` | body | Wall thickness at a point or over a face. |
| `closestPoint` | body | The nearest body, face, edge or vertex for many points. |
| `angles` | body | Dihedral angles of edges and faces, and draft angles. |
| `clearance` | body | Face pairs closer than a given distance. |
| `symmetry` | body | Mirror-symmetry planes and the deviation of the mirrored body. |
| `extrema` | body | Lowest and highest point of the body along a direction. |
| `bodyBoolean` | body | Preview a boolean between the separate bodies of the state; nothing is stored. |

### Design rules

| Type | Tag | Purpose |
|---|---|---|
| `profiles` | none | List the built-in design-rule profiles and the rule catalogue. |
| `checkProfile` | body or part | Run a design-rule profile (3D printing, CNC, injection moulding or inline) on a body. |
| `dfmMinWall` | body | Thinnest wall against a minimum. |
| `dfmMinInternalRadius` | body | Smallest concave corner radius against a cutter radius. |
| `dfmPocketRatio` | body | Depth to width ratio of closed pockets against a limit. |
| `dfmOverhang` | body | Overhanging faces against a build direction. |
| `dfmConstantSection` | body | Whether the body is a constant extrusion along an axis. |

### Sketches

| Type | Tag | Purpose |
|---|---|---|
| `sketchOffset` | none | Offset sketch entities by a distance and return the resulting chains. |
| `sketchAnalyze` | body or part | Structural findings of one sketch: open ends, duplicates, overlaps, and why there is no closed profile. |
| `sketchFreedoms` | body or part | Degrees of freedom of one sketch, per point, radius and entity. |

### Text and data

| Type | Tag | Purpose |
|---|---|---|
| `textMetrics` | none | Advance width and ink box of a string. |
| `textOutline` | none | The outline loops of a text entity as polylines. |
| `loadFont` | none | Register a TrueType/OpenType font for the session (session state, not tag state). |
| `unloadFont` | none | Remove a loaded font. |
| `listFonts` | none | List the built-in and loaded fonts. |
| `subsetFont` | none | A font file with only the glyphs of a string. |
| `threadData` | none | The thread standards and size tables. |

## See also

- [Replies and buffers](reply.md)
- [Regeneration](regen.md)
- [Queries](queries.md)
- [Errors](errors.md)
- [The native service](native-service.md)
- [Call it from Node](../getting-started/node.md)
