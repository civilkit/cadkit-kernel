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
