# ADR 0004: Multiple bodies in one part

Status: Proposed. Not implemented; nothing here changes existing behaviour.

## Context

A part is one document that regenerates to one `Brep`. That `Brep` can hold several **lumps** (connected solids; "bodies" in
the UI). What exists today:

- A feature whose result does not touch the existing solid adds a lump. One that touches or crosses it is united with it.
- `moveBody` moves or copies lumps and may leave them overlapping ("use a boolean feature after it").
- `combine` (alias `bodyBoolean`) runs `union`, `cut`, `intersect` or `split` between whole lumps, with `keepTools`, and keeps
  persistent names on both operands.
- Lumps are selected by a face name (the lump holding that face), `{feature: id}` (every lump with a face made by that feature) or
  `{lump: n}`. Fillet, chamfer and shell already act on the lump their faces and edges belong to.
- The kit lists lumps as bodies and keeps their name, colour and visibility as **viewer state** (`bodyNames`, keyed by the lowest
  face name or by a feature id). None of it is in the feature history, so no feature can refer to a body by name.

The reference we compare ourselves with (SolidCraft's documented model, MIT OR Apache-2.0, not copied) is a different shape. A
document holds **named bodies that may overlap**. Every solid feature carries an operation (`new_body`, `join`, `cut`,
`intersect`) and an optional list of target bodies; fillet, chamfer and shell take a `body`. `move` (with copy), `combine`, `split
body` and `remove` act on whole bodies. A cut that lists targets leaves every other body alone, and a `new_body` feature stays
separate even where it overlaps another body.

Importing its files is where the gap shows. Of the 22 corpus files, `battery-tpu-case` (two versions), `move-bodies`,
`logo-up-down-join` and `m0-revolve-primitives` need either a new solid that stays separate where it overlaps, or a cut and an
intersect limited to a named body. Each is refused today with a reason, which is the correct behaviour but not the goal.

This also matters beyond importing. "Cut this body only" and "put a copy of the battery next to the case" are natural
instructions for an agent (the aim of this project: an LLM drives the kernel and gets exact feedback), and they are hard to say
today without knowing face names.

## Options

**A. Adopt their model as the document model.** Every solid feature gets `operation` and `targets`; bodies are named objects.
Most general and the closest to what people migrate from. Cost: it changes the meaning of existing features (a cut would have to
default to "every body" to stay compatible), touches every solid feature, and replaces lump selection by name with a second
identity system next to persistent face names (ADR 0002).

**B. Keep lumps and `combine`; add one new operation and body names (recommended).**
- A new extrude, revolve, sweep, loft, box-like feature operation `newBody` (name to settle): the result is added as its own lump
  and is **not united** with what it touches or overlaps. Today this cannot be said: an overlapping `add` merges.
- An optional `body` name on the feature that creates a lump (`newBody` features, `moveBody` with `copy`, `combine` results).
  The name is part of the document, travels with the lump through later features (a cut, a fillet, a `combine` whose target it
  was; the target's name wins) and is a new selector `{body: "Case"}` next to the existing ones. Unnamed lumps behave exactly as
  they do now. Viewer-only `bodyNames` stays for display and is not replaced.
- Their per-feature `targets` is a composition, not a primitive: `cut` limited to body X is `newBody` for the tool, then `combine`
  `cut` with target X and the tool; `intersect` likewise. An importer or the UI builds that. If use shows that people and
  agents write it all the time, `targets` can be added later as sugar that expands to the same two features.

**C. Do nothing and refuse.** Keeps the model small. Leaves roughly a third of the reference files unimportable and keeps
"cut one body" hard to express.

## Decision

Take B, in two steps, so each can be verified on its own:

1. `newBody` operation and the `body` name with the `{body}` selector, plus the importer mapping (disjoint and overlapping
   extra bodies, `move` with copy, `combine`, `remove`, cut and intersect with targets via `combine`).
2. Only if step 1 proves clumsy in use: `targets` on `cut` / `intersect` / `join` as sugar.

Assemblies are a separate decision and are not covered here. Components placed with mates (`insertPart`, `@id.1` instance names)
stay the model for assemblies; the reference's components and joints map onto it.

## Rules the implementation must follow

- Additive and default-preserving. A document without `newBody` or `body` regenerates exactly as before; the gate numbers do not
  move.
- Overlapping lumps are legal in a regen result (they already are after `moveBody`). Features that need a single solid say so
  with a structured error instead of working on garbage: `nothingToCut` stays for no body; a new `ambiguousBody {candidates}`
  when a feature that must act on one lump finds several and has no selector.
- Name rules: a duplicate body name is `bodyNameConflict {name, features}`; a selector that matches nothing is
  `missingReference` with the nearest names (as ADR 0002 already requires for faces).
- Persistent naming is unchanged: faces keep their names through `newBody`, `combine` and moves. A body name is a label on a
  lump, not a replacement for face names.
- Volume and validity: `validate` and `massProps {bodies:true}` already report per lump; overlapping lumps must be flagged as
  overlapping (with the pair), not as invalid.
- Every new request or feature field is covered by the protocol lint (`tools/lint_protocol.mjs`) and gets analytic volume
  oracles (two overlapping boxes: sum of the volumes, the union, the intersection) before it counts as done (ADR 0003).

## Consequences

- The importer can map the reference's multi-body files instead of refusing them, and `intersect`, `move` with copy and
  `combine` stop being unsupported. Expected unlock: `move-bodies`, `logo-up-down-join`, `m0-revolve-primitives`, and a start on
  both battery files (which also need edges on a named body).
- One new operation and one new optional field is a small kernel change, but `newBody` touches every solid-producing feature's
  operation handling: the brief for it must list them and name the tests per feature.
- The kit's body list gains a real name to show and to use in scripts; the viewer-state names stay as they are.
- Risk: lumps that overlap are not a valid solid for most downstream work (mass, STEP export, fillets across the overlap). The
  kit should show the overlap and offer `combine` rather than hide it.
- Open questions for the brief: the operation's name (`newBody` against `separate`); whether `sweep`, `loft`, `thread` and the
  rest take it in the first cut or only extrude, revolve and the primitives; whether a `body` name is also allowed on a `join`
  (naming the result) or only where a lump is created.

## Not decided

Assemblies and joints; sheet metal and other reference features without an equivalent here; whether `targets` is ever added.
