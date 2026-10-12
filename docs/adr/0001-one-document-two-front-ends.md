# ADR 0001: One document model, two front ends

Status: Accepted (2026-10-12). Implementation of the front ends: Proposed.

## Context

There are two ways to use a CAD kernel:

- **Direct manipulation (interactive):** a person picks things in a viewport,
  drags handles, sees live previews, and uses keyboard modifiers. Each command
  is a short interactive session (pick, drag, confirm or cancel). The state
  between commands is mostly "the current shapes".
- **Programmatic / agent-driven:** a program or language model reads the model,
  proposes a change, sends it, and reads back structured results. It cannot
  point, hover or watch a preview; it needs a document it can read and edit
  and feedback it can parse.

Interactive tools tend to model commands as stateful, asynchronous flows tied
to the UI (pickers, gizmos, dialogs waiting on the user). Those flows are good
for feel and poor as an automation surface, because there is no declarative
description of what was done that can be edited and replayed.

Our kernel already has a declarative description: the feature tree (JSON),
regenerated deterministically, with persistent face and edge names and
structured per-feature errors.

## Decision

1. **The feature tree is the single source of truth.** Every change, from any
   front end, is an edit to that document (add, modify, delete, reorder a
   feature; change a parameter). Nothing important lives only in UI state.
2. **Interactive front ends are editors of the document, not a second model.**
   A drag handle on a face edits a feature parameter (for example an extrude
   depth); a "fillet" tool appends a fillet feature referencing named edges.
   Live preview uses a speculative regen of the edited document that can be
   discarded.
3. **Agent front ends use the same edits plus read-only queries**, with
   machine-readable results (see ADR 0003).
4. **One kernel interface serves both.** The request/reply protocol stays the
   only entry point. Front-end specific needs are met with extension requests
   (as `validate`, `heal`, `importStep`, `importSat` were), not private APIs.
5. **Humans and agents can edit the same document concurrently in sequence.**
   Edits are small, ordered, and attributable (see the spec for the edit
   record) so a human can review what an agent changed and undo it.

## Consequences

- Feel features (snapping, handles, selection modes) are built later as a
  layer on the document model, and cannot add capabilities the document
  cannot express. This keeps the agent interface complete by construction.
- Every interactive operation needs a document representation. Operations
  without one (free-form dragging of vertices, sculpting) are out of scope
  until they have one.
- Undo/redo is document history, shared by both front ends.
- Testing is mostly at the document level (cases in `tools/brep_cases.mjs`,
  oracles, gates), which is cheap and deterministic; UI tests are thin.

## Alternatives considered

- **Separate command layers for each front end.** Rejected: duplicates
  behaviour and lets the two drift.
- **Make the interactive session the primary model and record macros.**
  Rejected: recorded sessions are brittle (they depend on picks and screen
  positions) and are hard for an agent to edit.
