# Notes: interface styles and what to take from each

Date: 2026-10-12. Informal notes behind ADR 0001-0003 and the two interface
specs. Based on reading source for an interactive direct-modelling editor and
the shipped bundle of a parametric web editor; neither was run.

## The two styles

| | Direct manipulation | Parametric document |
|---|---|---|
| Unit of work | A command session: pick, drag, confirm | A feature in an ordered, editable list |
| State | Mostly the current shapes | The document; shapes are derived |
| Strength | Speed, feel, exploration, free-form work | Design intent, change propagation, repeatability |
| Weakness | Hard to edit later or replay; hard to automate | Slower to explore; needs references that survive edits |
| Natural for programs? | Poor: flows wait on pickers, gizmos, dialogs | Good: read, patch, regen, read results |

The right design takes the document as the model and adds the interactive
layer as an editor of it (ADR 0001).

## Observations about the interactive style

- Commands are classes that create a dialog, one or more handle objects and
  object pickers, then await them. This is natural for UI code and means the
  "what was done" is not stored declaratively. Automating it means simulating
  user input.
- The interaction layer (selection, snapping, handles) is large: roughly
  as much code again as the visible UI. It is where the feel comes from and
  where most effort goes. Budget for it accordingly.
- Handles are specialised per command. A shared handle vocabulary (distance
  along an axis, angle about an axis, radius, count) reduces that cost and
  maps one-to-one to document fields.
- Modes (selection mode, temporary overrides with held keys) are valuable and
  should be data-driven.

## Observations about the parametric style

- Everything depends on stable references. Our naming holds up under
  parameter edits (214 of 216 in the test); losses occur where topology
  genuinely changes.
- Feedback granularity matters: per-feature status, not "the model failed".
- It is easy to test: documents in, measurements out. That is why the oracle,
  benchmark, naming and import gates exist here.

## What to take, and what to avoid

Take:
- Selection modes, snapping and inference, and handle behaviour as UI features.
- Quasi-modes (hold a key for a temporary mode) for fast selection edits.
- A strict separation of document, kernel client, viewport and tools.

Avoid:
- Commands as the persistent model.
- Positional references.
- UI state that is not recoverable from the document.

## Code quality criteria used for comparison

Measured on source we can read:
- Type strictness (strict mode on, count of `any` and suppressions).
- Size and cohesion of files (largest files; one concept per file).
- Test volume relative to code, and what level tests run at.
- Consistency of patterns across features.
- Age and drift of dependencies.
- Coupling to a proprietary component (kernel, services).

Observed for the interactive editor: strict typing, few suppressions (54 `any`,
16 ignores, 45 TODOs across about 300 source files), small files (largest
about 750 lines), about 100 test files, a consistent command/factory/handle/
dialog pattern, with dated dependencies and strong coupling between commands
and the editor object. For our own code the measurable quality gates are:
hard gate metrics (wire, validity, volume, oracles, imports, repairs), naming
stability, protocol lint, and the benchmark set. What we lack is a measure of
readability for the parts reconstructed from decompilation.

## Open items

- Agent interface server (spec-agent-interface.md): not started.
- Interactive front end (spec-interactive-interface.md): not started. A
  prior front end for the original kernel may run against ours unchanged
  (protocol replies matched on all 280 wire cases); untested.
- Intent checks (`check`) and description resolution (`resolve`): not
  implemented in the kernel.
- A warning when deleting or reordering an upstream feature re-points a
  reference: not implemented.
