# Spec: agent interface

Status: Proposed. Describes a thin server that lets a language model (or any
program) build and edit models through the kernel. Implements ADR 0001-0003.
Nothing here requires kernel changes except where marked **(kernel)**.

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

## Acceptance

- A scripted agent session (no human) builds the benchmark parts in
  `tools/bench` from text descriptions with a success rate that is reported by
  family.
- Every tool call returns within a bounded time on the benchmark set.
- The protocol lint passes for the server layer.
- Edit atomicity: a failing batch leaves the document unchanged.
