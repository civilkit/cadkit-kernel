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

## Thin wall (`extrude` / `revolve` `thin`)

`extrude` (add or cut) and `revolve` take an optional `thin` object: the profile CURVES are offset in the sketch plane
and the band between them is swept instead of the sketch regions, so an OPEN curve (a C-shaped spline arc) gives a wall in
one step and a closed loop gives a hollow wall. End conditions, `draft`, `flip`, `regions`, `angle` and the revolve axis
rules are unchanged.

```json
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30,"thin":{"thickness":3,"side":"mid","ends":"flat","cornerStyle":"mitre"}}
{"type":"revolve","id":"F4","op":"add","sketch":"F3","axis":"ax","angle":360,"thin":{"thickness":"=wall","side":"outside"}}
```

Fields (all but `thickness` optional): `thickness` (number or `=expression`, > 0); `side` `inside` (default) | `outside`
| `both` | `mid`; `thickness2` (`both` only: the thickness on the second side, default `thickness`); `ends` `flat` (default) |
`round` (open chains: half-disc caps); `cornerStyle` `mitre` (default) | `round`; `enabled:false` keeps the settings but
switches the wall off (a `null` or `false` `thin` is off too).

Rules:

- Chains: the closed region loops of the sketch (outer and hole loops, each once), then the open chains made of entities
  that bound no region (lines, arcs, circles, ellipses, splines and cv-splines, joined end to end, tangent-continuous or
  with corners). `regions` (entity ids) picks chains the same way it picks regions, with the same errors.
- Side: closed loops are oriented counter-clockwise, `inside` is the interior. Open chains are oriented so that they turn
  counter-clockwise (a straight chain keeps the sketch direction); `inside` is the left of that direction, i.e. the inside
  of the bend. `both` is `thickness` on that side plus `thickness2` on the other, `mid` is `thickness` centred on the curve.
- Offsetting is exact for lines, arcs and circles; ellipses and splines are fitted (cubic B-spline, within 1e-5). Inner
  corners are trimmed at the crossing of the offsets, outer corners are mitred (very sharp ones, over 4 thicknesses, are
  bevelled) or rounded about the corner point.
- Persistent face names (`<id>` = the feature): `Fn.S.<entity>.W0` the offset side (with `both`, the left offset),
  `Fn.S.<entity>.W1` the original curve (with `both`, the right offset), `Fn.S.<entity>.E0` / `.E1` the end caps of an open
  chain (at its first / last entity), `Fn.S.<entity>.W0r` / `.W0m` / `.W0b` round / mitre-connector / bevel pieces of a
  corner, `Fn.C0` / `Fn.C1` the usual caps. The names follow the sketch entities, so they survive edits that keep the
  entities.
- Errors: `thinTooThick {limit, minRadius, maxThickness, thickness, face}`: the offset passes a centre of curvature, a short
  segment is eaten by an inner corner or the wall overlaps itself. `limit` is `curvature` or `corner`, `minRadius` the
  tightest radius on the offset side, `maxThickness` a thickness that works (found by bisection, 2% safety), `face` the first
  entity of the chain. Never a panic. `invalidValue {field:"thin.thickness"|"thin.side"|"thin.ends"|"thin.cornerStyle"}`
  for bad input; a sketch without curves is `noClosedProfile`.
- Without `thin`, an open sketch is still `noClosedProfile` ("sketch profile is not closed").
- Not covered: `thin` on `sweep`, `loft`, `hole` and surface extrudes; non-uniform thickness.
  Check: `tools/thin_extrude_check.mjs` (wasm, run by `tools/land.sh`), `crates/ck-model/tests/thin_extrude.rs`.

## Variable thickness (`thicken` `thicknessAt`)

A surface `thicken` may taper instead of standing at one thickness: `thicknessAt` replaces `thickness` with a field that is
sampled per point of the sheet (each point moves along its own face normal by its own thickness), so a blade tapers root to
tip in one step. Both layers of the wall follow the ramp: a `both` body is the sheet moved by half the local thickness each
way.

```json
{"type":"thicken","id":"F2","surface":"F1","thicknessAt":{"along":[0,0,1],"stops":[[0,4],[48,1]]},"side":"both"}
```

Fields: `along` a non-zero direction (the ramp axis); `stops` two to six `[s, t]` pairs, `s` ascending, each `t` > 0 and
finite. The thickness at a sheet point `p` is the linear interpolation of the stops at `s = p·â` (`â` the unit `along`),
clamped to the first and last stop; `side` is as for `thickness` (`front` default).

Rules:

- `thickness` and `thicknessAt` are alternatives. `thickness` keeps its constant behaviour exactly (bodies and face names
  bit-identical); a `thicknessAt` whose stops all hold one thickness is routed to the same path.
- The stops must cover the sheet along `â`: a sheet point outside the stops' range is refused before any geometry runs
  (`invalidValue {detail: "the thicknessAt stops cover ... but the sheet spans ...", hint}`), with a hint to add a stop at
  each end. A stop list that is not 2..=6 ascending pairs of finite numbers, or a non-positive stop thickness, is
  `invalidValue {detail, hint}` at parse time.
- Thickness limits are per point: wherever a local thickness reaches its local radius of curvature the offset is
  `offsetTooLarge {limit: {minRadius, maxThickness, ...}, detail, hint}` on the tightest face (`maxThickness` a thickness
  that works there, in terms of the feature's thickness; the hint suggests reducing the stops or thickening the other
  way). Creases miter per point (`thickenReach` when an offset face cannot meet its neighbour, as for a constant
  thicken).
- Geometry: a variable thicken always runs the free-form offset machinery with per-point distances — surfaces, rim edges,
  crease miters and walls alike — so even an analytic sheet (plane, cylinder) is fitted (cubic, within 1e-5) and the face
  names follow the free-form convention. The constant path's exact quadric offsets do not apply.
- Not covered: variable `thin` walls, `moveFace`. Check: `crates/ck-model/tests/variable_thicken.rs`,
  `crates/ck-ops/src/surfacing/thicken.rs` (`surfacing::thicken::tests`), `tools/prop_probe.mjs`
  (`cap7_variable_thicken`, `ours-only` against the original kernel).

## Loft point sections (`loft` apex ends)

The first and/or last section of a solid `loft` can be a single POINT, so a hull bow, a cone-like body or a sail tip tapers to
a true point. A point section is a sketch that holds exactly ONE `point` entity and no other (non-construction) entity; the
point's position in the sketch plane is the apex. There is no extra field.

```json
{"type":"sketch","id":"F2","plane":{"kind":"feature","id":"P2"},"points":[{"id":"p","x":0,"y":0}],"entities":[{"type":"point","id":"e1","p":"p"}],"constraints":[]}
{"type":"loft","id":"F3","op":"add","sketches":["F1","F2"]}
```

Rules:

- Only the first and the last section can be a point (not both: that is `loftProfiles`). A point section in the middle,
  guide curves with a point section and a `surface` loft with a point section are `loftFailed {detail}`.
- Geometry: the lateral surface of each piece of the neighbouring section converges to the apex (the B-spline loft surface
  with its last control-point row collapsed to the apex, same degree, knots and weights as the neighbour, so a circle to a
  point is an exact cone and a polygon to a point is a pyramid of planar triangles). `startNormal` / `endNormal` are ignored
  at the point end. Several stations (a hull) are interpolated as usual and end in the apex.
- Topology: one vertex for the apex shared by every rail, NO cap face and NO end edges at a point end; each lateral face has
  a pole in its loop (the loop closes in 3D, in uv it jumps along the collapsed iso-line, the same convention as a cone apex
  or a sphere pole). A circle to a point is 3 faces (two lateral halves + the cap), a square to a point is 5. Face names are
  the usual `Fn.S.<entity>[~k]`, `Fn.C0` / `Fn.C1` only for the end that has a cap.
- Automatic snap: a non-point END section whose bounding box is smaller than 1e-3 of the loft length (end-to-end centroid
  distance) is treated as a point at its centroid, so shrinking a section to nothing does not leave a degenerate cap.
- Structured error instead of an invalid solid: a section whose outline crosses itself is `loftProfileSelfIntersect
  {detail}`, or `loftSectionTooSmall {detail}` when it is also small against the loft length (under 1e-2); the hint says to
  make the section a simple closed curve or a point section. Previously such a loft returned `ok` with an invalid body.
- Validation gate: a solid loft is checked with the same validity check as the `validate` query before it is returned. An
  invalid result is rebuilt under alternative section matchings (even cut distribution, shifted start points, flipped
  orientation); when none is valid the loft is `loftInvalidResult {detail, hint}`. An end section whose cap outline folds
  back on itself at a scale the coarse crossing test misses (a spline with a double-clicked point) reports that in the
  detail. Loft errors carry `hint` when they have one. `tools/loft_fuzz.mjs --strict [--mode guides|point|centre|tangent]`
  asserts no trap and ok-implies-valid.
- Display mesh: no cracks at the pole and unit normals (the limit normal is used at the apex). STEP export writes the faces as
  rational B-spline surfaces with a repeated control-point row; re-import (pcurves of edges that reach the pole stay on the
  iso-line) gives the same volume to 1e-6 and a clean `validate`.
- Rails at the apex: with three or more sections the rails arriving at the point are clamped to the chord from the neighbouring
  section (speed `apexTangent` times the chord speed) instead of the natural spline end (zero second derivative), which let the
  surface overshoot into a dent and, for sections that shrink quickly towards a far apex, made the cross-section area grow again
  (a fold). `apexTangent` (0 to 2, default 1) is the loft option: 1 = the straight cone taper at the tip, smaller = a narrower
  spike, larger = a wider tip angle (for a hull that tapers to its bow the body gets slimmer as the factor grows). With two sections the loft is the straight cone either way.
  Check: `tools/loft_apex_check.mjs` (hull variants, far apex, `apexTangent`, exact cones).
- Not covered: guide curves through the tip, `startTangent` / `endTangent` at a point, a surface (sheet) loft to a point,
  booleans whose intersection curve passes THROUGH the apex (a cut away from the tip works).
  Check: `tools/loft_point_check.mjs` (wasm), `crates/ck-wasm/tests/loft_point.rs`, unit tests in `crates/ck-ops/src/loft.rs`.

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

**Taper limits** (a `limits` list of `[featureId, {"maxHeight": h}]`, sorted by feature id, only when non-empty).
An `extrude` with `draft` (inward) on a profile that is one circle without holes (the "Cone" shape) reports
`maxHeight`: the extrude `depth` at which the taper closes to a point (`radius / tan(draft)`, less the start
offset). A UI can clamp its height field to it without recomputing:

```json
"limits":[["F3",{"maxHeight":12.5}]]
```

A depth equal to `maxHeight` (within 1e-9 of the radius) builds a valid cone with one apex vertex and no top cap;
a smaller one keeps the flat top; a larger one fails with `draftTooLarge {detail:"the taper closes to a point at
height H", maxHeight:H}` (also for the `draft` feature on a cylinder wall, where `maxHeight` is measured from the
neutral plane). Other profiles (a polygon whose edge collapses, several loops) keep the plain `draftTooLarge
{detail}` with no `maxHeight` and no `limits` entry.

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

### Threads in patterns and mirrors

A pattern (`lpattern`, `cpattern`) or `mirror` carries the threads of its seeds onto every
instance, so the copies of a threaded hole, boss or shaft are threaded too.

- Which threads: a `thread` feature whose `face` was made by a seed (`thread.face` starts with the
  seed's id: `F4.S.h1` is a face of seed `F4`) is carried automatically, whether it comes before
  the pattern in the history or after it (a thread added later to an already patterned hole is
  copied onto the existing instances; patterns are walked in history order, so a pattern of a
  pattern picks up the new copies). A thread feature can also be listed in `seeds` (`["F4","F5"]`): that
  copies exactly that thread, even with `carryThreads:false`; a thread seed that produced no
  entry (failed, suppressed) is a `threadNotCarried` warning. `carryThreads:false` on the
  pattern / mirror switches the automatic part off (default true). A thread can still be put on
  one instance face by hand (`F4.S.h1@F7.2`). `mirror` with `body:true` copies every thread of
  the body (only when the thread precedes the mirror).
- Result: one `threads` entry per instance, named `<thread id>@<pattern id>.<k>` (`F5@F7.1`),
  with `face` the persistent instance face (`F4.S.h1@F7.1`), `start` / `dir` transformed by the
  instance transform, the same standard / size / class / length / hand (a mirrored right-hand
  thread is again right-hand), and the extra fields `instanceOf` (the source thread id),
  `pattern` (the pattern id) and `instance` (`F7.1`). A pattern of a pattern chains the names
  (`F5@F7.1@F8.2`) and `instanceOf` stays the root thread.
- Modeled threads: the groove is cut again on each instance wall (the thread is rebuilt on the
  instance face from the same request, so the same fallback rules apply, per instance). The
  groove faces are named like the source's with the instance suffix: `F5.T1@F7.1`. Removed volume
  is N x one groove (tests: 1e-3). `modelRequested:true` on an entry records that the groove was
  asked for even where it fell back.
- Warnings (`warnings`, keyed by the pattern id): `threadNotCarried {thread, face, reason, hint}`
  when an instance face cannot carry the thread (the instance hole was cut away by another
  feature, the face is no longer a cylinder, the size is unknown); the other instances still get
  theirs. Modeled fallbacks of an instance appear as `threadModelFallback` with the extra param
  `thread` (the instance entry id).
- Editing the source thread (size, length, hand, modeled) or the hole regenerates the instances:
  they are derived, not stored. Check: `tools/thread_pattern_check.mjs`,
  `crates/ck-model/tests/thread_pattern.rs`.
## Direct manipulation: `moveFace` and `moveBody`

Two ordinary features for GUI handles (drag a face arrow, drag the move / rotate gizmo of a
body). Both fail with a structured error and leave the model unchanged.

### `moveFace`

```json
{"type":"moveFace","id":"F7","faces":["F2.C1"],"mode":"offset","distance":2.5}
{"type":"moveFace","id":"F8","faces":["F2.S.e1"],"mode":"translate","translate":[3,0,0]}
```

- `faces`: persistent face names of the solid (a face split into `<id>~k` pieces is named by
  its base id). `mode: "offset"` (default): each face moves along its OUTWARD normal by
  `distance` (numbers or `"=expr"`; `flip: true` negates). Positive adds material: a pushed
  plane grows the part, a cylinder's radius grows, and a hole wall with a positive distance
  shrinks the hole. `mode: "translate"`: planar faces move by the component of `translate`
  `[x,y,z]` along their outward normal (the sliding part changes nothing); curved faces are
  `moveFaceUnsupported` in this mode. A distance of 0 is a no-op.
- Supported surfaces: planes, cylinders, cones, spheres, tori, extruded walls (spline outlines, see "Offset limits"). Unselected neighbours keep
  their surfaces and are extended or trimmed: vertices are re-solved on the incident
  surfaces and edges follow (ck-ops `move_faces_body`, the machinery shell uses). Round
  faces (fillets, cylinders) tangent to a pushed plane travel with it (translate rigidly) so
  the blend stays tangent; if that gives no valid body, the plain extension is tried.
- Topology, face order and every persistent name are unchanged, so references and later
  features (sketches on the moved face, fillets) keep working. Pcurves are rebuilt.
- Result checks: edges must not collapse or reverse, `ck-check` finds no new issue, the
  volume moves the way the faces do (all growing: not smaller; all shrinking: not larger).
- Errors (`regen.errors`): `nothingToCut` (no solid), `noFaces`, `missingReference`,
  `invalidValue {detail}` (bad mode, missing `translate`, non-finite), `moveFaceUnsupported
  {detail}` (free-form or other surface, an edge of a kind the solve cannot follow such as
  the ellipse a chamfer leaves, translate mode on a curved face, a face of a surface body),
  `moveFaceFailed {detail}` (an edge collapses or reverses, the result is not valid, the
  volume went the wrong way; e.g. pushing a 10 mm slab face in by 12, or the radius of a
  fillet next to its tangent planes).
- Not done: translating a cylinder sideways (moving a hole), carrying chamfers, a face
  moved past other geometry (no interference check against unrelated parts of the body),
  surface-body faces.

### `moveBody`

```json
{"type":"moveBody","id":"F9","bodies":[{"face":"F2.C1"}],
 "translate":[10,0,0],"rotate":{"axis":[0,0,1],"origin":[0,0,0],"angle":30},"copy":false}
```

- `bodies` (also `body` / `target`; an array, an object or a string): `{"face": name}` or the
  bare name selects the lump holding that face, `{"feature": id}` (or a bare id that is not a
  face name, i.e. has no `.`) selects every lump holding a face made by that feature — the face
  names `F17.*` (caps, walls, blends alike, so the GUI need not know which) including instance
  and patterned copies (`F17.*@F9.1`, an `insertPart` component `@F17.1`, a patterned row
  `F17.*@F2.3`); several entries add up; a feature with no such face is `missingReference`.
  `{"surface": featureId}` a surface body. Missing or empty: every lump of the solid.
- Motion: the rotation first (`rotate.axis` a vector, `"x"|"y"|"z"` or `{kind:"axis",axis:"x"}`;
  `origin` default the world origin; `angle` in degrees), then `translate`. Numbers or
  `"=expr"`. The older `dx/dy/dz` + `axis` + `angle` keys still read. No motion is a no-op.
- Placement of a library part: one `moveBody` at the end of the inserted group rotates and
  translates everything built so far as one rigid body (`rotate.axis` through `rotate.origin`,
  90-degree steps or any angle). Select only the inserted group with `bodies` when the document
  already holds other solids. Names are unchanged, so references made after it still resolve
  (`crates/ck-model/tests/move_features.rs`, `bldc_rotates_*`, `face_references_after_a_bldc_transform`).
- `copy: true` leaves the original and appends a moved copy whose face names get the
  instance suffix `@<id>.1`; otherwise the entities move in place and all names are unchanged.
- Moved bodies are not united with what they touch or cross: the result may hold overlapping
  lumps (use a boolean feature after it).
- Errors: `nothingToCut`, `missingReference`, `invalidValue {detail}` (non-finite numbers,
  zero rotation axis, vector with the wrong length).

Checks: `crates/ck-model/tests/move_features.rs` (analytic volumes, names, the fillet carry,
a sweep of every face of several parts that must end valid or in a structured error),
`tools/move_check.mjs` (wasm, run by `tools/land.sh`).

## Offset limits: `shell` and `moveFace` hints, free-form walls

Shell and face offsets of extruded free-form walls (a closed sketch spline, a kidney / bean outline;
ck-ops `offset`, `shell`). The wall of such a part is an extrusion of a B-spline; its offset is the
extrusion of the offset curve (`offset_extrusion`: sampled along the profile, cubic interpolant refined to
1e-5 mm, no tensor grid), and the cap edge between that wall and a plane normal to the extrusion is the
offset profile placed in the plane (exact, no sampling).

- A closed sketch spline is a two-lap curve (domain twice the loop, split into two wall faces); a closed
  `cvspline` is one lap whose seam point is both end parameters. Every closest-parameter search on such a
  curve has two answers one lap apart, which scrambled pcurves and classification in the boolean (the
  generic `shellFailed`). `shell` and `moveFace` first cut those curves down to the lap window each face / edge
  uses (`offset::single_lap_body`, parameters kept).
- Errors now carry numbers (`shellTooThick`, `shellFailed`, `moveFaceFailed`, `moveFaceUnsupported`):
  `detail` (a sentence naming the face), `limit` (`"curvature"`: the offset passes a centre of curvature;
  `"thin"`: two sheets closer than twice the offset), `minRadius` (smallest radius of curvature found, or
  the narrowest gap for `thin`, mm), `maxThickness` (largest thickness / distance that works, 0.8 of the
  radius or 0.4 of the gap, rounded down to 0.01 mm; 0 when curvature is not the limit) and `face` (the limiting
  face). Example: `F2.S.e1~1 curves too tightly for an offset of 12.00 mm: its smallest radius of curvature
  is 7.04 mm, use at most 5.62 mm`. A cavity the boolean could not cut is diagnosed the same way: tightest
  free-form radius against the wall, then parallel planar faces closer than the wall allows (`thin`), else a
  `shellFailed` that says curvature is not the limit.
- `moveFace` accepts extruded walls (any extrusion surface) in `offset` mode and reports the same hints.
- Checks: `crates/ck-model/tests/shell_spline_wall.rs` (analytic wall volumes of inward / outward, open /
  closed shells of spline and cvspline kidneys against an independently offset polygon; the too-thick hint
  with the suggested thickness shelling; moveFace of the wall), `tools/spline_shell_check.mjs` (wasm, run by
  `tools/land.sh`).
- Limits: the suggested thickness is a safe fraction, not the exact limit; a part thinner than the wall
  reports the plane gap only after the curvature limit is met; free-form faces other than extrusions
  (B-spline surfaces) report a sampled radius (40 x 40 grid) and still fit through the tensor grid (slow for
  long faces); thickness larger than the part height with an opened face is reported from the plane gap.

## Booleans on free-form walls and spheres on faces

- A pocket, boss or ring cut whose profile is a closed spline, on a solid whose walls are extruded closed
  splines (or any solid), used to fail with `booleanFailed`. Two causes: the section of the tool's wall with the
  other body's cap plane was marched instead of taken exactly (a plane perpendicular to an extrusion's rulings
  cuts it in the profile itself: `intersect::ss::plane_extrusion_perp`, exact and fast), and the two-lap
  closest-parameter ambiguity of a closed sketch spline made "this edge lies on that face" tests and the uv
  loops of the wall faces wrong. `boolean` now cuts both operands' two-lap curves down to their lap window first
  (`offset::single_lap_body`, shared with `shell` / `moveFace`).
- A lone sphere (a revolved semicircle) whose pole lies on a surface of the other body (the typical sphere
  centred on a face with its axis in the face plane) met it in a circle through the poles, which the face's uv
  polygon cannot hold: the sphere fragments collapsed and the sphere face was dropped (an error, or a silently
  wrong volume). The sphere is now turned about its centre until no pole is on the other body
  (`apply::unpole`; same point set, the face keeps its name, only seam and poles move). The union box check of
  the sanity test uses the loose box of the result (a hemisphere bulge has no edge on it).
- Checks: `crates/ck-model/tests/spline_boolean.rs` (ring cut, pocket, boss, through pocket on a spline and a
  cvspline kidney, analytic volumes against the standalone inner extrusion; spline pocket in a block),
  `crates/ck-model/tests/sphere_on_face.rs` (hemisphere face area 2 pi r^2, volume, validity, add and cut, six
  faces, both axes in the sketch, sketch on the face), `tools/spline_boolean_check.mjs` (wasm, run by `tools/land.sh`).
- Limits: the pole rotation applies to a body that is a single sphere face; a sphere that was already trimmed
  and then cut again on a pole-containing plane is not rotated. Splines whose section with a tilted plane is
  needed still march.

## Boolean robustness: robust mode and the `booleanFailed` report

- `boolean()` runs its ordinary attempts first (welding tolerance 10x, 1x, 100x as before). Only when every one of them
  fails to assemble a closed manifold does it run three more attempts in **robust mode** (`boolean::robust()`, a
  thread-local switch set by `RobustGuard`; 10x, 1x, 100x the tolerance) with three sliver-tolerant steps that would
  change, and slow, operations that already work: `imprint::cut_crossings` splits two cut curves of one face pair where
  they cross each other in their interiors (the branches of two equal tubes meeting symmetrically meet at a singular
  point; without a vertex there one edge is met from four directions); edge pieces between the same two vertices that
  stay within 100 tol of each other are one edge (a cut curve running 7e-4 beside a rim where two surfaces touch
  tangentially); a fragment whose sample lies within max(100 tol, 5e-4) of a face of the other solid is classified by the
  majority of all six rays (winding number on a near tie), because single rays disagree on a sliver lens. Operations that
  succeed without it are bit-identical to before. BenchCAD propeller (four crossed lofted blades) went from
  `booleanFailed` to a valid body (tools/bench/FINDINGS.md B4-3; `crates/ck-model/tests/propeller_boolean.rs`).
- A boolean that still fails reports `booleanFailed` with params that say where. Free-form walls that nearly coincide:
  `hostFace`, `toolFace`, `gap` (as before). Otherwise: `faces` (the faces bordering the open / non-manifold edges of the
  assembled result, up to 8, for example `["F8.S.e1~1@F9.1", "F8.S.e1~1@F9.3"]`), `issue` (`openEdge`,
  `nonManifoldEdge`, `badPairing`), `at` (`[x, y, z]`, a point on the first bad edge) and `hint` (a one-line remedy:
  move the tool by about 0.01, unite the bodies that touch each other first, or make near-coincident faces exactly
  coincident). The driver's fix hint (`packages/driver/src/verify.ts`) quotes them.
- Reference tool: `node tools/sample_sweep.mjs --wasm <wasm> --dir <samples> --out run.json` regenerates every kit
  sample (errors, volume, validity, CPU time) and `--compare before.json after.json` flags a sample that now errors, whose
  volume moves by more than 1e-6 relative, or that is more than 20 percent (and 0.3 s) slower; run it before and after a
  kernel change (a past change made swept-handle 79 s).

## Fuzzy tolerance and body-level booleans

Three additions to the boolean machinery. The coincidence corpus that measures them is `tools/boolean_corpus.mjs`
(`docs/boolean-corpus.md`).

### Fuzzy `tolerance` and `autoFuzzy` (every boolean-capable feature)

Any feature that combines a tool with the body (`extrude`, `revolve`, `sweep`, `loft`, `hole`, `thread`, patterns,
`combine`) reads two optional fields:

```json
{"type":"extrude","id":"F5","op":"add","sketch":"S4","end":"blind","depth":10,"tolerance":1e-5,"autoFuzzy":false}
```

- `tolerance` (mm, number or `"=expr"`, default 0 = exact booleans exactly as before): geometry of the tool that lies
  closer than `tolerance` to the body's geometry is treated as coincident. Mechanism (`ck-ops/src/boolean/fuzzy.rs`):
  before the boolean, every planar, cylindrical or spherical face of the tool whose surface is within `tolerance` of an
  equal-kind surface of the body (parallel plane, same axis and a radius within `tolerance`, same sphere centre) is moved
  onto that surface exactly with the move-face offset machinery (vertices and edges re-solved, pcurves rebuilt). The
  boolean then sees exactly coincident faces and its coincident-face merging (identical surfaces, orientation voting,
  the `touch` classification) applies. Nothing else changes. A snap that fails, makes the tool invalid or changes its
  volume by more than 4 x `tolerance` x area is discarded and the plain boolean runs.
- Suggested values: 1e-6 .. 1e-3 mm. The result differs from the exact boolean of the unsnapped inputs by at most
  `tolerance` in position; a cut whose remaining sliver is thinner than `tolerance` is removed entirely, a gap thinner
  than `tolerance` is closed.
- `autoFuzzy: true`: if the boolean fails (`booleanFailed`), comes out implausible (the post-boolean volume / box check)
  or comes out with more validity issues than its operands had, it is retried with `tolerance` 1e-7, 1e-5, 1e-3 (those
  above the given `tolerance`); the first retry that is plausible and valid wins. A result that fails the sanity check is
  never accepted: if no retry passes, the original error (`booleanFailed`, `booleanVolumeMismatch`) stands.
- Warning `booleanFuzzy {tolerance}` (in `warnings`, on the feature) says which fuzzy value decided the result, whenever
  the fuzzy value moved at least one face or a retry was needed. No warning: the boolean was exact.
- The `booleanFailed` hint now names these fields.

### `combine` feature (alias `bodyBoolean`): booleans between lumps of one document

A document can hold several lumps (separate bodies: a feature that does not touch the existing body adds a lump;
`moveBody` can leave lumps overlapping). `combine` operates on whole lumps:

```json
{"type":"combine","id":"F9","op":"cut","target":{"face":"F2.C1"},"tools":[{"face":"F4.C1"},{"lump":2}],
 "keepTools":false,"tolerance":1e-5,"autoFuzzy":true}
```

- `op`: `union` (`add`, `join`), `cut` (`subtract`), `intersect` (`common`), `split`.
- `target` (also `a`): exactly one lump, as a face name (bare string or `{face}`: the lump holding that face) or
  `{lump: n}` (index in `massProps {bodies:true}` order). `tools` (also `tool`, `b`): one or more lumps, same selectors;
  a tool may not be the target.
- `union`: target united with each tool in turn (the arrangement is binary, so the n-ary union is sequential in the listed
  order; tools that overlap each other still end up in one lump). `cut`: target minus each tool. `intersect`: target inside
  every tool. `split`: the target cut into the part outside all tools and the part inside them (two lumps; the tools are
  united first).
- `keepTools` (default false): tool lumps stay as they were next to the result; otherwise they are consumed.
- Persistent names: result faces keep the names of the faces they come from (target faces and tool faces keep their own
  feature ids; a face split by the operation gets the usual `~k` suffix), so references on both operands survive. Lumps not
  involved stay untouched and first in lump order; the result lumps follow.
- Errors: `nothingToCut` (no body), `missingReference`, `invalidValue {detail}` (unknown op, not exactly one target, no
  tools, tool is the target), `booleanFailed`, `booleanVolumeMismatch`, `cutNoEffect` (cut / split changed nothing),
  `combineEmpty` (intersect of lumps that do not overlap). An error leaves the body as it was.
- Kit-facing JSON shape: exactly the object above; the kit builds it from the selected lumps' face names or lump indices.

### `bodyBoolean` request (preview, nothing committed)

`{"type":"bodyBoolean","tag":"main","op":"intersect","target":"F2.C1","tools":["F4.C1"],"tolerance":1e-5,"autoFuzzy":true}`
runs the same operation on a copy of the current body and answers
`{"ok":true,"lumps":n,"volume":v,"area":a,"faces":[...],"toleranceUsed"?:t}` or
`{"ok":false,"error":{"code":..,"params":{..}}}`. The state is not changed; to keep the result, add the `combine` feature.

### Edge contact in results

Two solids (or two parts of one result) that touch along an edge used to make a four-face edge: a union of two boxes
sharing only an edge was `booleanFailed`, and a later boolean on a body with such a contact could return an invalid body.
Now the union of two solids whose faces are all kept whole is the two lumps side by side, and any other four-face edge of
a result is split into two coincident two-face edges, one per material wedge (`ck-ops/src/boolean/contact.rs`: the faces
around the edge are sorted by angle, the consecutive pairs that bound material keep the edge and its twin). The pieces
stay joined at the end vertices only; the edge-contact corpus cases and 600 random grid sequences
(`crates/ck-ops/tests/boolean_grid_fuzz.rs`) give valid bodies of exactly the cell-count volume.

### Whole-sphere operands

Two whole spheres meet in a circle that crosses the seam of a sphere once only when its axis lies along the line of
centres; the sphere is turned to that axis before the boolean (face names unchanged), which makes overlapping balls
unite / cut. The post-boolean volume check no longer confuses a flipped whole sphere (a spherical void) with the
unflipped one (face-volume memo key includes the sense for seam-only faces), so a ball void in a block is accepted.

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

## Assembly mates and the degrees-of-freedom report

`mate` features constrain inserted components (`insertPart`). A mate has `a` and `b`, persistent face references
(`F2.C0@F2.1`: face of component `F2`), and moves the component of `a` onto the component of `b`; if that
component is `fixed` the other one moves. Both fixed is `mateFixed`; both faces on one component is
`mateSameBody`; an unresolved face is `missingReference`; geometry a kind cannot use is `mateGeometry`
with `params.detail` saying why; a mate the solver cannot satisfy together with the earlier ones is
`mateConflict` (below). No mate ever panics, and a rejected mate leaves every component where the earlier
mates put it.

Oracle kinds (`coincident`, `concentric`, `distance`, `parallel`) are unchanged. Kinds added here (the "Leaves"
column is the remaining freedom of a free component mated to a fixed one):

| `kind` | Fields (besides `a`, `b`) | Faces | Constraint | Leaves |
|---|---|---|---|---|
| `angle` | `angle` (degrees, default 0), `flip` (supplementary angle) | plane / cylinder / cone (normal or axis) | angle between the two directions | 5 |
| `perpendicular` | | plane or axis pairs; plane + axis puts the axis along the normal | directions at 90 degrees | 5 |
| `tangent` | `flip` | cylinder or sphere (a) on plane (b); plane (a) on cylinder / sphere (b); cylinder-cylinder (parallel axes), sphere-sphere, sphere on cylinder (sphere in `a`) | contact; the side of a plane is the one the faces start on, `flip` takes the other; `flip` on two round faces is internal contact (radius difference) instead of external (sum) | cylinder on plane 4 |
| `lock` | | any face of each component | relative pose when the mate is made, kept when either moves | 0 |
| `symmetric` | `c` (planar mirror face) | plane+plane, cylinder+cylinder, sphere+sphere | `a` is the mirror image of `b` about `c` (`c`'s component is treated as fixed) | plane pair 3 |
| `width` | `a2`, `b2` | `a`, `a2` parallel planes of the tab; `b`, `b2` parallel planes of the slot | tab mid-plane on slot mid-plane, normals parallel | 3 |
| `hinge` | `a2`, `b2` optional, `distance` (stop offset) | cylinders / cones (concentric); `a2` / `b2` planes (axial stop) | concentric axes, stop faces in contact | 1 rotation (2 without the stop) |
| `slot` | `b2` optional, `flip` | `a` pin (cylinder), `b` (and `b2`) planar slot wall(s) | pin axis parallel to the walls and centred between `b` and `b2` (or resting on `b`) | 4 |
| `screw` | `pitch` (length per turn, default 1), `angle` (drive, degrees, may exceed 360), `flip` (left hand) | `a` screw and `b` nut cylinders | concentric, and advance = pitch * turns; with `angle` the nut is placed that far along the thread | 1 helical; 0 with `angle` |
| `gear` | `ratio` (turns of `a` per turn of `b`, default 1), `angle` (drive of `a`, degrees), `flip` (same direction) | `a` driver and `b` follower axes (cylinders) | `b` turns `-angle / ratio` about its axis, measured from the pose when the mate was made | one coupling row, one more with `angle` |
| `rackPinion` | `radius` (pitch radius), `angle` (drive of the pinion), `dir` (unit vector, for a planar rack), `flip` | `a` pinion (cylinder), `b` rack: cylinder (its axis is the travel direction) or plane with `dir` | rack travel = radius * pinion angle | one coupling row |
| `limits` | `of` (`"distance"` default, or `"angle"`), `min`, `max` (length, or degrees) | two planes (distance along `b`'s normal), or two directions (angle) | the measured value is clamped into the range, by translating the mover (distance) or rotating it about the origin of face `a` (angle); inside the range the mate consumes no freedom | 6 inside the range |

Couplings (`gear`, `rackPinion`, `screw`) are measured from the placements the components have when the mate
is made (after the earlier mates were solved), so a later mate that moves the driver drags the follower. The
follower is the component of `b` (if `b` is fixed, `gear` and `screw` swap roles and invert the relation;
`rackPinion` then answers `mateFixed`). The rotation of a driver is read modulo 360 degrees; `angle` itself is
not (a screw with `angle: 1080` is three full turns). Not supported (`mateGeometry`): `symmetric` with `c` on a
moving component, `tangent` with the cylinder in `a` and a sphere in `b`, a rack that is neither a cylinder
nor a plane with `dir`.

Examples:

```json
{"type":"mate","id":"F5","kind":"hinge","a":"F2.S.e1@F2.1","b":"F2.S.e1@F1.1","a2":"F2.C0@F2.1","b2":"F2.C1@F1.1"}
{"type":"mate","id":"F7","kind":"gear","a":"F2.S.e1@F2.1","b":"F2.S.e1@F3.1","ratio":2,"angle":40}
{"type":"mate","id":"F8","kind":"screw","a":"F2.S.e1@F1.1","b":"F2.S.e1@F2.1","pitch":2,"angle":1080}
{"type":"mate","id":"F9","kind":"limits","a":"F2.C0@F2.1","b":"F2.C1@F1.1","min":0,"max":15}
{"type":"mate","id":"F6","kind":"width","a":"F2.C0@F2.1","a2":"F2.C1@F2.1","b":"F2.C0@F1.1","b2":"F2.C1@F1.1"}
```

### Degrees of freedom: `assembly` in the regen reply

When the document has a mate of an added kind (or a rejected one), or when the request has `opts.dofReport:
true`, the regen reply carries an `assembly` object next to `components` (assemblies of oracle kinds only keep
the oracle reply byte for byte unless asked):

```json
"assembly": {
  "dof": {"total": 2, "translational": 1, "rotational": 1},
  "components": [{"id": "F1", "fixed": true, "dof": 0, "translational": 0, "rotational": 0},
                 {"id": "F2", "fixed": false, "dof": 2, "translational": 1, "rotational": 1}],
  "overConstrained": true,
  "redundant": ["F4"],
  "conflicts": [{"mate": "F5", "with": ["F3", "F4"]}]
}
```

* `dof.total` is the dimension of the motion space of all floating components left by all mates together
  (couplings count once). `rotational` is the part that rotates (the rank of the rotation block of the motion
  space), `translational` the rest. Each component's own numbers are the motions it can still make, so a
  component locked to a floating one reports the freedoms of that one. Fixed components report 0.
  `components[].dof` in the regen reply carries the same number once an added kind is present.
* `redundant` lists mates that add no constraint (their rows depend on the earlier mates): harmless, but they
  are why `overConstrained` is true.
* `conflicts` lists each mate the solver rejected (also an error `mateConflict` on that mate, whose
  `params.mates` is the mate followed by its neighbours) with the earlier mates on the same components. A
  rejected mate is dropped and the components return to the pose before it.
* The rank comes from a numeric Jacobian of the mate residuals at the solved pose (singular values below
  1e-6 of the largest count as zero), so a near-singular pose can misreport. Assemblies of more than 30
  floating components skip the null-space part (`truncated: true`).
* Solve order is deterministic: the new mate's mover first, then the other components in insertion order,
  each against its own mates with the rest frozen, repeated until nothing moves (at most `8 n` moves, else
  `mateConflict`).

Native checks: `crates/ck-model/tests/mates_ext.rs` (hinge leaves one rotation, gear 2:1 turns the follower by
half, screw advance, rack travel, tangent distances, limits clamping, conflicts and redundancy named, structured
errors); wasm: `tools/mate_check.mjs` (run by `tools/land.sh`).

## Lettering: the sketch `text` entity, `textMetrics`, `textOutline`

Engraved / embossed text on parts. A sketch entity `{type:"text", string, font?:"gauge"|"engrave", height (cap height),
at:[x,y] | p:<point id>, anchor?:[h, v], angle?, spacing?, mirror?, polyline?, arc?:{centre|c, radius, angleCentre?,
side?:"outside"|"inside"}}` is fixed geometry (no constraints; the anchor point `p` may be constrained) whose outlines
are closed profile loops (letters with holes and islands are regions with holes) for `extrude` add / cut and the other
profile consumers. Stateless requests `textMetrics {string, font?, height, spacing?}` (advance, ink box) and
`textOutline {<entity fields>}` (polylines for a preview) size and preview it; `inspect` of a sketch lists its `texts`.
Errors `textEmpty`, `textNothingToDraw`, `textFontNotLoaded`, `textFontMismatch`, `textBadHeight`, `textBadArc`, `textBadValue`;
warnings `textTooSmall`, `textMissingGlyph`, `textTruncated`, `textOverlap`, `textSpacingAdjusted`. Two OFL fonts (`gauge`,
`engrave`) are compiled in; any other TTF / OTF is registered for the session with `loadFont {name, data:<base64>, replace?}`
(reply `{name, hash, glyphs, supported:[blocks], coverage, warnings, ..}`; errors `fontNameConflict`, `fontReserved`,
`fontBadName`, `fontBadData`, `fontTooLarge`, `fontTooMany`), `unloadFont {name}`, `listFonts {}` and
`subsetFont {name, text}` (TTF bytes with just those glyphs). Fonts are keyed by name and file hash; the hash is part of the regen
cache key and an entity may pin it with `fontHash`.
Details, limits and checks: `docs/text.md`; corpus `tools/text_corpus.mjs`.

## `importSkp` orientation: `opts.up`

`{type:"importSkp", data, opts:{up?:"z"|"y"|"yFlip"}}` (additive; absent means `"z"`, the file's own Z-up axes, so existing callers are
unchanged). `"y"` maps (x, y, z) -> (x, z, -y) (SketchUp Z becomes our Y, right-handed, SketchUp's front faces +Z), `"yFlip"` maps
(x, y, z) -> (-x, -z, -y) (upside-down fix). Face names, areas, normals relative to the geometry and edge counts are unchanged. The
reply's `import` gains `up` (the applied value) and `upMatrix` (row-major 3x3, nine numbers). Any other `up` is an error reply.
Details in `docs/skp.md`.

## Acceptance

- A scripted agent session (no human) builds the benchmark parts in
  `tools/bench` from text descriptions with a success rate that is reported by
  family.
- Every tool call returns within a bounded time on the benchmark set.
- The protocol lint passes for the server layer.
- Edit atomicity: a failing batch leaves the document unchanged.
