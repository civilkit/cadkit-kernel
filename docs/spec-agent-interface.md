# Spec: agent interface

Status: Proposed. Describes a thin server that lets a language model (or any
program) build and edit models through the kernel. Implements ADR 0001-0003.
Nothing here requires kernel changes except where marked **(kernel)**.
The kernel pieces `check`, `resolve`, `describe` and the reference-change
warning (`refDiff`) are implemented: see `docs/agent-queries.md`.

## Goals

- An agent can create, edit, check and export a model using only structured
  messages. No screen coordinates, no hidden state.
- Every step returns enough information to decide the next step without a
  human: what changed, what failed, and whether the result is right.
- A person can watch and intervene in the same document.

## Shape

A small server (stdio or HTTP) wrapping the kernel module. It owns sessions;
each session holds one document (the feature tree) and the kernel state.

### Tools

| Tool | Input | Output |
|---|---|---|
| `open` / `new` | name or part JSON | session id, document, regen result |
| `read` | session, optional feature ids | document (or slice), per-feature status, a short text summary |
| `edit` | session, list of edit ops (below), `dryRun?` | new status, diff of the document, regen result; applied atomically or not at all |
| `query` | session, kind: `massProps`, `inspect`, `measure`, `interference`, `validate`, `names` | the kernel reply, trimmed |
| `resolve` | session, face/edge description(s) | matching persistent names, ambiguity flags (ADR 0002) |
| `describe` | session, names | readable description of each entity |
| `check` | session, expectations (volume, bbox, mass, counts) | per-expectation pass or fail with the measured value |
| `render` | session, view(s), highlight names | PNG image(s) and the mapping from highlights to names (see below) |
| `export` / `import` | format (`step`, `sat`), data | file or new document; import returns validate and heal results |
| `history` / `undo` | session | edit log; revert the last N edits |

### Edit ops

`add` (feature JSON, position), `set` (feature id, field path, value),
`remove` (feature id, `cascade?`), `move` (feature id, new position),
`replace` (feature id, feature JSON). Each op is validated against the
feature schema before it reaches the kernel. Each applied batch gets an edit
record `{id, author, time, ops, reason}` so edits are attributable and
reviewable. `reason` is free text supplied by the agent.

### Results the agent sees

- Per-feature status with error code and parameters (ADR 0003).
- A diff of resulting body facts: face, edge and solid counts, volume, bbox,
  and which named faces appeared or disappeared. This answers "did my edit
  change something I did not mean to" directly (the main failure mode of
  edit tasks).
- Warnings, including "reference changed after reorder or delete".
- Timing, so the agent can notice slow features.

## Visual feedback

Images help but are not required for correctness. `render` produces
orthographic and isometric views with optional highlighting of named
faces or edges and a legend that lists the names. A headless renderer is
needed (tessellation already comes from the kernel as binary buffers).

## Guardrails

- Edit batches are size-limited; features are schema-checked; numeric fields
  are range-checked before regen.
- Regen has a time budget; long features report a timeout error instead of
  hanging the session.
- The server never exposes the filesystem except through explicit `import` and
  `export`.
- Destructive operations (`remove` with `cascade`, overwrite on export)
  require an explicit flag.

## Authoring guidance served with the tools

A short, versioned document the server exposes to the agent: feature list and
field tables, the naming grammar, common recipes (plate with hole pattern,
revolve profile, shell), error code meanings with usual fixes, and the rule
"prefer referencing caps and sketch-entity sides over positions".

## Region selection (`extrude` / `revolve` `regions`)

By default `extrude` and `revolve` sweep every closed region of the referenced
sketch. The optional field `regions` (list of sketch entity ids) picks some of
them, so several shapes in one sketch can be used by different features:

```json
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10,"regions":["e5"]}
{"type":"extrude","id":"F3","op":"cut","sketch":"F1","end":"blind","depth":4,"regions":["e1","e6"],"flip":true}
{"type":"revolve","id":"F4","op":"add","sketch":"F1","axis":"ax","angle":360,"regions":["g"]}
```

Rules:

- A region is selected when ANY entity id in `regions` is a segment of its
  OUTER boundary (a GUI can pass whatever the user clicked; duplicates and
  several ids of one loop are fine). The selected region keeps all its holes.
- Nesting follows the sketch profile rules: a loop inside a region's outer loop
  is that region's hole; a loop inside a hole (an island) is a region of its
  own. Only regions named by `regions` are built, so selecting the outer ring
  leaves the island out, and selecting the island leaves the ring out.
- An id that bounds no region's outer loop but a hole loop (a circle drawn
  inside a rectangle) selects that hole's filled disc, without holes. Selecting
  the rectangle alone keeps the circle as a hole; add the circle's id for the
  disc. Such discs come after the real regions in the order.
- Absent field: every region, exactly as before. The field is optional and
  documents without it are unchanged.
- Errors (never a silent fallback to all regions): `extrudeRegionUnknown
  {ids}` (ids that are no entity of the sketch), `extrudeRegionNone
  {regions}` (empty list, or ids that bound no outer or hole loop, e.g. a
  construction line), `invalidValue {field:"regions"}` (not a list of strings).
- Works for `op` add and cut and every end condition (draft, flip, mid, ...);
  the regen cache keys on the whole feature JSON, so changing `regions`
  invalidates from that feature on.
- Persistent face names: the swept profile contains only the selected regions
  in sketch order, so one selected region is named exactly as if it were alone
  in the sketch (`F2.C0`, `F2.S.e5`, no `~k`); several selected regions get the
  `~1..~n` suffixes numbered over the selection, as multi-region sketches do
  today. Editing the selection can therefore renumber the `~k` of the other
  selected regions (not of other features' faces).
- Not covered: `sweep`, `loft`, `hole` and surface extrudes still use all regions.
  Check: `tools/region_select_check.mjs`, `crates/ck-model/tests/region_select.rs`.

## End references (`extrude` `upToFace`, `offsetFromFace`, `upToPoint`)

`extrude` (add or cut) can stop at a face, at a face moved by a distance, or at a point's plane.
The end condition names its target in `endRef`:

```json
{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"upToFace","endRef":{"kind":"face","id":"F2.C1","at":[10,20,-3]}}
{"type":"extrude","id":"F4","op":"cut","sketch":"F3","flip":true,"end":"offsetFromFace","endRef":"F2.C1","offset":-2.5}
{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"upToPoint","endRef":[0,12,0]}
{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"upToPoint","endRef":{"kind":"vertex","faces":["F2.C1","F2.S.e4","F2.S.e1"],"at":[0,20,0]}}
```

- `endRef` for the face ends: `{"kind":"face","id":<persistent face id>,"at":[x,y,z]?}` or the bare id string.
  `at` is optional: a point on the face when it was picked (see "Persistence" below).
- `endRef` for `upToPoint`: `[x,y,z]`, `{"kind":"point","at":[x,y,z]}`, or `{"kind":"vertex","faces":[ids],"at":[x,y,z]}`
  (the vertex shared by the named faces nearest `at`; follows the faces when the model is edited). The end plane is
  parallel to the sketch plane through the point.
- `offset` (expression, `offsetFromFace` only): the end plane is the face's plane moved along the face's OUTWARD normal
  by `offset`; negative moves into the material. Default 0.
- Direction: as for `blind`: along the sketch normal, `flip` reverses; for `op:"cut"` the sketch normal is reversed first
  (so a cut that must go along +normal needs `flip:true`, same as every cut).
- Targets: planar faces only. A plane that is slanted against the sketch is supported (the tool is trimmed with a boolean
  and the new end face takes the end cap's name `<id>.C<k>`). Cylinders, cones etc. are `upToFaceUnsupported`.
- Coverage: `upToFace` needs the face to cover the whole profile along the extrusion direction (`upToFaceOutside`);
  `offsetFromFace` and `upToPoint` use the (offset) plane without that test.
- Errors (oracle codes): `missingReference {field:"endRef", face?}` (absent, malformed or unresolvable `endRef`, or a
  face end with no body; this one is NEVER downgraded), `upToFaceBehind` (the face is not ahead of the whole profile
  or parallel to the direction), `upToFaceOutside {face}`, `upToFaceUnsupported {face}`, `upToPointBehind`.
- Fallback (REBUILD-ONLY): the four "unreachable" codes above are NOT errors by default. The feature runs as a blind
  extrude of `depth` and the regen result gets a warning
  `endConditionFallback {end, reason, fallback:"blind", depth, ...reason params}` (wire `warnings`, `[featureId, {code, params}]`).
  A GUI shows it as a yellow badge on the feature. `"strictEnd":true` on the feature turns it back into the error.
  A trim boolean that fails on a slanted end falls back the same way (`reason:"endClipFailed"`).
- Persistence: a face id that no longer exists is looked up again: first the unique face of the same `~k` family
  (`F2.C1~2` finds `F2.C1`), then, when `endRef.at` is given, the one planar face through that point. Both give a
  warning `endRefRelinked {from, to}`. An ambiguous anchor (a corner or an edge point) is `missingReference`. The
  normal case needs no relink: face ids come from the feature id, so editing parameters of earlier features (depth of
  the block, sketch dimensions) keeps `F2.C1` and the end follows the face. GUIs should store `at` with every picked face.
- `upToNext` is unchanged (already implemented: a through tool trimmed by the next faces of the body).
- Tool memory: a trimmed (slanted) tool has no `Recipe`, so mirror and pattern features reflect the finished body.
- Check: `tools/end_reference_check.mjs` (wasm, run by `tools/land.sh`), `crates/ck-model/tests/end_references.rs`.

## Hole: counterbore and countersink dimensions

`hole` cuts one tool per sketch `point`. Standard sizes (`size` M2..M24 with
`fit`) give the hole diameter `d`; `kind` picks the head shape.

- `kind:"simple"`: plain hole (or `diameter` instead of `size`).
- `kind:"cbore"`: counterbore. `cboreDiameter` and `cboreDepth` (mm or
  expressions) override the ISO 4762 table values of the size; each is
  independent (absent: that table value, so documents without them are
  unchanged). Errors `holeCboreInvalid {hint, ...}`: `cboreDiameter <= d`,
  `cboreDepth <= 0`, or (only when a field is given) `cboreDepth >= hole depth`.
- `kind:"countersink"` (alias `"csk"`): always a cone. Head diameter
  `cskDiameter` (default: ISO 10642 table value of the size), included angle
  `cskAngle` in degrees (default 90, 0 < angle < 180). The cone starts with
  the head diameter at the top face and tapers at half the angle to `d`; cone
  depth = `(cskDiameter - d) / 2 / tan(cskAngle / 2)`.
- `kind:"csink"` (the legacy wire spelling): a plain hole, exactly as before,
  UNLESS `cskDiameter` is present, then it is cut like `countersink`
  (`cskAngle` alone does not switch the cone on). A document that has
  `csink` without `cskDiameter` therefore never changes; new documents should
  use `countersink`.
- Errors `holeCskDiameter {cskDiameter|cskAngle, holeDiameter|coneDepth,
  holeDepth, hint}`: head diameter <= `d`, angle outside (0,180), or the cone
  deeper than the hole. `hint` is a one-line fix.
- Persistent face names (tags after `<id>.S.`, `~k` per point as before):
  simple `h1` wall, `h2` bottom (`tip` cone); cbore `h1` counterbore wall,
  `h2` step floor, `h3` hole wall, `h4` bottom; countersink `K0` cone,
  `h1` hole wall, `h2` bottom. The top disc lies in the existing face and has
  no name. Names do not depend on the dimensions.

```json
{"type":"hole","id":"F4","sketch":"F3","standard":"iso","size":"M8","fit":"close","kind":"cbore","end":"blind","depth":20,"cboreDiameter":15,"cboreDepth":8.6}
{"type":"hole","id":"F5","sketch":"F3","standard":"iso","size":"M5","fit":"normal","kind":"countersink","end":"through","cskDiameter":10.4,"cskAngle":90}
{"type":"hole","id":"F6","sketch":"F3","standard":"iso","size":"M6","fit":"normal","kind":"countersink","end":"through"}
```

Check: `tools/hole_dims_check.mjs`, `crates/ck-model/tests/hole_dims.rs`.

## Threads: `threadData` request and `thread` feature

### `threadData` (read-only, stateless)

The thread tables live in the kernel (`crates/ck-model/src/threads.rs`), so a
GUI fills its "standard" and "size" dropdowns from this request instead of
hard-coding them, and an agent uses the same data. No session state is needed
(no `tag`). All lengths are mm.

```json
{"type":"threadData"}
-> {"units":"mm","standards":[{"id":"metric","description":"ISO metric coarse, M1 to M64 (60 degrees)","count":35,"sizes":["M1","M1.2", "..."]}, {"id":"UNC",...}]}

{"type":"threadData","standard":"UNC"}
-> {"standard":"UNC","units":"mm","sizes":[{"size":"1/4-20","standard":"UNC","major":6.35,"pitch":1.27,"tpi":20,
    "minorInternal":4.9,"minorExternal":4.77,"pitchDiameter":5.5256,"angle":60,"tapDrill":5.105,"tapDrillName":"#7",
    "classInternal":"2B","classExternal":"2A","tapered":false}, ...]}

{"type":"threadData","standard":"BSP","size":"G1/4"}      // one entry
```

Standard ids: `metric` (coarse, `M6`), `metric-fine` (`M10x1.25`), `UNC`, `UNF`
(`1/4-20`, `#10-24`), `BSP` (parallel G, `G1/4`), `NPT` (`1/4 NPT`, tapered
1:16), `Tr` (`Tr20x4`), `Acme` (`Acme 1/2-10`). Aliases are accepted
(`ISO`, `G`, `bspp`, case-insensitive). `major` is the nominal diameter (for
NPT the pipe outside diameter, `pitchDiameter` the gauge-plane value);
`minorInternal` is D1 (the crest of an internal thread), `minorExternal` is d3
(the root of an external thread); `tapDrill` is the bore an internal thread is
cut into. An unknown standard or size is a wire error listing the choices.

### `thread` feature

A thread on a cylindrical face: the wall of a hole (internal) or a shaft
(external). Built from a `hole` feature, a circle cut through a body, or an
extruded circle: pick the face by its persistent name.

```json
{"type":"thread","id":"F5","face":"F4.S.c1","standard":"metric","size":"M6"}
{"type":"thread","id":"F6","face":"F2.S.c1","standard":"UNC","size":"1/4-20","length":12,"hand":"left","start":"top","modeled":true}
{"type":"thread","id":"F7","face":"F4.S.c1","standard":"metric-fine","size":"M12x1.5","class":"6H","length":"full"}
```

Fields: `face` (cylindrical face name), `standard` and `size` (a designation from
`threadData`, validated against the table), `class` (optional, default `6H` /
`6g`, `2B` / `2A`, `7H` / `7e`, ...; recorded, it does not change the geometry),
`length` (mm, or `"full"` = the whole face, the default; clamped to the face),
`hand` (`"right"` default, `"left"`), `start` (`"top"` / `"bottom"`: the end of
the face with the higher / lower coordinate along the axis, the axis oriented so
its dominant world component is positive; default the OPEN end: a hole's mouth,
a shaft's free end; `"top"` when both or neither end is open), `modeled`
(bool, default false). Internal or external is read from the face: concave =
internal, convex = external (reported as `internal`; it cannot be overridden).

**Cosmetic** (`modeled` false): no geometry change. The regen result gets, only
when there is at least one thread, a `threads` list (sorted by feature id):

```json
"threads":[{"id":"F5","face":"F4.S.c1","standard":"metric","size":"M6","class":"6H","pitch":1,"major":6,
  "minor":4.917468,"pitchDiameter":5.350481,"angle":60,"length":20,"hand":"right","internal":true,
  "modeled":false,"tapered":false,"diameter":5,"start":[20,20,-20],"dir":[0,-1,0]}]
```

`minor` is D1 for an internal thread, d3 for an external one; `diameter` is the
face's actual diameter; `start` / `dir` are the point on the axis where the
thread starts and the unit direction into the face. Drawings, a BOM and validation
read this list.

**Warnings** (a `warnings` list of `[featureId, {code, params}]`, same shape as
`errors`, only when non-empty; the document still regenerates and `ok` stays true):

- `threadDiameterMismatch {expected, actual, size, standard, internal, nearest,
  hint}`: an internal thread wants the face diameter within 3 percent of the tap
  drill (or of the minor diameter), an external one within 3 percent of the major
  diameter. `hint` is actionable, e.g. `hole is 6.5, M6 tap drill is 5: drill the
  hole to 5, or choose M8 (tap drill 6.8)`, `shaft is 7, M8 major diameter is 8:
  turn the shaft to 8, or choose M7 (major diameter 7)`.
- `threadModelFallback {reason, hint}`: `modeled` was asked for but not built;
  the feature stays cosmetic (see the fallback rules below).

**Errors** (per feature, the rest of the document regenerates):
`threadSizeUnknown {standard, size, suggestions, hint}` (unknown standard or
size; suggestions are the nearest designations, or the standard ids),
`threadFaceNotCylinder {face, surface, hint}`, `missingReference` (no / unknown
`face`, or no solid yet), `invalidValue {field}` (`length` <= 0, bad `hand` /
`start`).

**Modeled** (`modeled` true): the real helical groove, cut with the helix-sweep
and boolean machinery.

- Profile: a straight-flank trapezoid with the standard flank angle (ISO / UN 60,
  BSP 55, Tr 30, Acme 29 degrees). SIMPLIFICATION: crests and roots are the
  flats of the basic profile (P/8 for ISO / UN, 0.1665 P BSP, 0.366 P Tr,
  0.3707 P Acme), never the rounded roots of the real standards. An internal
  thread cuts from just inside the bore out to the major diameter (the bore stays
  the actual face diameter, so a bore at the tap drill gives the usual slightly
  truncated crests); an external thread cuts from the minor diameter d3 out past
  the shaft surface. Left hand mirrors the helix.
- The groove starts a little outside an open start end so it runs out cleanly,
  and stops with a flat end inside the face (the thread length).
- Persistent names: the groove tool's faces are renamed `<id>.T0` ... `<id>.T5`:
  `T0` inner (internal) / outer (external) surface of the groove, `T1` lower
  flank, `T2` root (internal) / bottom (external), `T3` upper flank, `T4` start
  cap, `T5` end cap. The inner / outer surface lies in the void, so after the cut
  the body carries `T1`, `T2`, `T3` and, where the groove ends inside the
  material, `T4` / `T5`.
- Fallback rules (the regen never fails because of a modeled thread): the
  feature stays cosmetic, `modeled` reads false, the body is exactly as before and
  a `threadModelFallback {reason}` warning says why: the thread is tapered (NPT;
  modeled tapered threads are not built), the groove would need more than 60
  turns (the stand-in for a time budget: there is no clock in the kernel), the
  thread length is shorter than one groove, the bore is not smaller than the
  major diameter or is too small for the groove to reach it (a hole far under the
  minor diameter), the shaft is smaller than the root or so far over the major
  that the groove cannot clear it, the boolean fails, the result is an invalid
  solid or splits into several lumps, or the removed volume is not within
  0.4x-1.6x of the analytic groove volume.
- Volume: the removed volume equals the analytic groove volume (ring integral of
  the groove width over the cut radii, per pitch, times length / pitch); the
  tests check it to 5 percent for full-length threads and to 10 percent for partial
  lengths (the flat groove end).
- Check: `tools/thread_check.mjs`, `crates/ck-model/tests/thread_feature.rs`.
## Design rules: `profiles` and `checkProfile`

A rule engine on top of `validate` / `check` (crates/ck-ops/src/profile). A
**profile** is a named list of rules, each with parameters and a severity. Every
**issue** names the failing faces and edges by persistent name, so the GUI
Diagnostics tab, a viewport heat map / highlight and an LLM agent all read the
same list. Both requests are read-only; no tag state changes. All lengths mm,
angles degrees, areas mm2.

### `profiles` (stateless)

Lists the built-in profiles (with the parameter values each uses) and the rule
catalogue, so a GUI can build a settings form from `params`.

```json
{"type":"profiles"}
-> {"profiles":[{"id":"fdm-print","name":"FDM 3D printing","description":"...","rules":[
      {"id":"minWall","title":"Minimum wall thickness","description":"...","severity":"error",
       "params":[{"name":"min","type":"number","default":1.2,"value":1.2,"unit":"mm","min":0,"description":"Thinnest wall allowed"}]},
      {"id":"overhang","severity":"warn","params":[
        {"name":"angle","type":"number","default":45,"value":45,"unit":"deg","min":0,"max":90,"description":"..."},
        {"name":"direction","type":"vec3","default":[0,1,0],"value":[0,1,0],"unit":"","description":"Build direction (up)"},
        {"name":"minArea","type":"number","default":0.5,"value":0.5,"unit":"mm2"}]}, ...]},
     {"id":"cnc-3axis",...}, {"id":"injection",...}],
    "rules":[{"id":"openShell","title":"Watertight","description":"...","severity":"error","params":[]}, ...]}
```

`value` is what the profile uses, `default` what the rule uses on its own.

### `checkProfile`

```json
{"type":"checkProfile","tag":"main","profile":"fdm-print","opts":{"samples":3000,"maxIssues":100}}
{"type":"checkProfile","part":<part>,"profile":{"id":"mine","name":"Mine","rules":[
   {"id":"minWall","params":{"min":2},"severity":"error"},{"id":"draft","params":{"minAngle":2,"direction":[0,0,1]}}]}}
```

* `profile`: a built-in id (`fdm-print`, `cnc-3axis`, `injection`) or an inline
  `{id?, name?, rules:[{id, params?, severity?}]}`. Rules not listed do not run;
  omitted `params` keep the rule defaults; `severity` is `error|warn|info`.
  Unknown profile / rule / parameter, a wrong type or an out-of-range value are
  wire errors that list the valid choices.
* Subject: the tag's regenerated body (`tag`, default `main`; no body gives
  `null` like `check`), or `part`: the document is regenerated privately
  through the same incremental regen cache as `regen`, so re-checking an edited
  document only replays the changed tail (and a tag is neither created nor
  changed). A part that makes no body returns `{profile, noBody:true, errors}`.
* `opts.samples` (default 3000): surface samples for the ray rules.
  `opts.maxIssues` (default 100): cap per rule, the rest is counted in
  `summary.truncated`.

```json
-> {"profile":{"id":"fdm-print","name":"FDM 3D printing","rules":[...effective rules and parameters...]},
    "ok":false,
    "issues":[
     {"rule":"minWall","severity":"error",
      "message":"Wall between F2.C1 and F4.C1 is 0.50 mm thick, under the 1.20 mm minimum: thicken it by 0.70 mm.",
      "faces":["F2.C1","F4.C1"],"edges":[],"value":0.5,"limit":1.2,"unit":"mm","at":[10.33,0.67,10]},
     {"rule":"minInternalRadius","severity":"warn",
      "message":"Sharp internal corner F2.S.e1|F4.S.e0#9 (radius 0) between ...",
      "faces":["F4.S.e0","F4.S.e1"],"edges":["F4.S.e0|F4.S.e1#9"],"value":0,"limit":1,"unit":"mm"}],
    "summary":{"errors":5,"warnings":1,"infos":0,"total":6,
      "rules":[{"rule":"minWall","severity":"error","issues":5,"passed":false}, ...],
      "truncated":[],"skippedFaces":[],"samples":3000}}
```

Issue fields: `rule`, `severity`, `message` (actionable, with the numbers),
`faces` (persistent face names, usable in `describe` / `resolve` and as
highlight targets), `edges` (persistent edge keys `faceA|faceB#n`, as in
`body.edges[].key`), `value` (what was measured) and `limit` (what was allowed),
`unit`, and `at` (a point on the part to focus on, when known). `ok` is true when
there is no `error`. `skippedFaces` lists faces whose triangulation failed (the
sampled rules cannot see them). Issues are ordered worst first within a rule.

### Built-in profiles and rules

| Profile | Rules (default severity; main defaults) |
|---|---|
| `fdm-print` | `openShell` (error), `bodyCount` (warn), `minWall` (error, 1.2), `overhang` (warn, 45 deg, +Y up), `bridge` (warn, 10 mm), `minFeature` (warn, 0.8 hole/pin diameter and gap), `plateContact` (warn, 20 mm2) |
| `cnc-3axis` | `openShell`, `minInternalRadius` (warn, cutter radius 1.0, tool axis +Z), `minWall` (warn, 1.0), `undercut` (error, from +Z) |
| `injection` | `openShell`, `draft` (warn, 1 deg, pull +Z), `uniformWall` (warn, +-25%), `minWall` (info, 0.8) |

Accuracy and limits (the rules are a first slice, not a certification):

* **Exact** (from the B-rep): hole and pin diameters (`minFeature`, cylinder
  radius, full-circle barrels only, blend faces ignored), concave cylinder
  radii and sharp concave edges parallel to the tool axis
  (`minInternalRadius`), watertightness (`openShell`: open / non-manifold
  edges, inside-out solids by volume sign) and solid count (`bodyCount`).
* **Mesh based** (the display triangulation, error about 4e-4 of the part
  diagonal): `overhang`, `bridge`, `plateContact`, `draft` and the facing-away
  half of `undercut` are exact for planes (constant normal) and a per-triangle
  area-weighted measure for curved faces. `draft` of a plane, a cone and an
  axis-parallel cylinder is exact; a face is flagged when at least half of its
  area is under the limit. `overhang` measures from vertical (0 wall, 90 flat
  ceiling) and ignores triangles lying on the build plate (lowest height).
  `bridge` takes a flat (within `flatTolerance`) downward face above the plate
  and reports its narrowest horizontal width as the span; it does not look for
  supports underneath.
* **Sampled** (rays from area-proportional samples, `opts.samples`):
  `minWall` (inward ray to the opposite wall; the hit must face along the ray
  within about 45 degrees), the gap half of `minFeature` (outward rays that
  meet a wall facing back), the hidden-under-material half of `undercut`, and
  `uniformWall` (median thickness per pair of faces against the part's median;
  only pairs much wider than their separation count as walls, so plate ends and
  solid blocks are ignored). A feature thinner than the sample spacing can be
  missed.
  Thickness is the straight distance along the surface normal, not the
  inscribed-sphere thickness: at the edge of a thin plate it reports the
  length, at a sharp wedge tip it reports the distance across.
* Not covered yet: sheet-metal rules (bend radius, flange length, flat
  pattern), custom user rules, draft sign / parting line, tool reach and
  stock, support volume, build orientation suggestion.

Checks: `crates/ck-model/tests/profile.rs` (native, small known parts: a box
with 0.5 mm walls, a 45 degree underside, a 0.5 mm hole, a 0.4 mm slot, drafted
and straight extrudes), `tools/profile_check.mjs` (wasm, run by `tools/land.sh`;
`--bench` sweeps the BenchCAD parts: no failures, every run under 5 s).

## Acceptance

- A scripted agent session (no human) builds the benchmark parts in
  `tools/bench` from text descriptions with a success rate that is reported by
  family.
- Every tool call returns within a bounded time on the benchmark set.
- The protocol lint passes for the server layer.
- Edit atomicity: a failing batch leaves the document unchanged.
