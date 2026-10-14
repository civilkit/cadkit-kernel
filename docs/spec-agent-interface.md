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

## Acceptance

- A scripted agent session (no human) builds the benchmark parts in
  `tools/bench` from text descriptions with a success rate that is reported by
  family.
- Every tool call returns within a bounded time on the benchmark set.
- The protocol lint passes for the server layer.
- Edit atomicity: a failing batch leaves the document unchanged.
