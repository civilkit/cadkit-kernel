# Spec: interactive interface

Status: Proposed. A viewport front end for people, built as an editor of the
feature tree (ADR 0001). Nothing here may add a capability the document model
cannot express.

## Goals

- Direct, fast feel: pick, drag, preview, confirm.
- Every interaction ends as a small edit to the document, so history, undo and
  the agent interface keep working.
- Works in a browser, loading the kernel module in a worker.

## Layers (smallest useful first)

1. **Viewer:** tessellated body from the kernel (positions, normals, indices as
   binary buffers), orbit and pan, shaded and edges display, standard views.
   Highlighting by persistent name. Markers for sketches and reference planes.
2. **Selection:** pick by face, edge or vertex mode; box and lasso;
   add and subtract with modifiers; each pick resolves to a persistent name
   (ADR 0002). Hover shows the readable description of the entity.
3. **Feature tree panel:** list of features with status (ok, error with code,
   suppressed), rename, reorder, suppress, delete, and "roll back to here"
   (regen with `upTo`). Error features open with the message and the
   offending reference highlighted.
4. **Parameter editing:** typed fields for each feature from its schema,
   expression support where the schema allows, live regen with debounce.
5. **Handles:** on-screen handles that edit parameters in place: extrude depth,
   fillet radius, pattern spacing and count, hole position, draft angle.
   Dragging runs speculative regens of the edited document (cancellable);
   release commits one edit record. The kernel must stay interactive: target
   regen under 100 ms for typical single-feature edits, otherwise preview
   uses the previous mesh until the new one arrives.
6. **Sketch editing:** points, lines, arcs, circles with snapping (grid,
   endpoints, midpoints, centres, perpendicular, tangent) and constraints and
   dimensions through the sketch solver already in the kernel.
7. **Snapping and inference** in 3D for placing sketches and planes:
   to vertices, edge midpoints, face centres, axes, and offsets from named
   faces.
8. **Tools menu** with the features the document supports; each tool is a
   short flow that collects references, shows a preview, and appends or edits
   a feature on confirm. Cancel discards the speculative state.
9. **Inspection:** measure, section view, mass properties, interference, and
   the `validate` report shown on the model and in a list.
10. **Import and export:** STEP and SAT import with a validate and heal
    report; STEP export.

## Interaction rules

- One undoable step per committed edit, not per mouse move.
- A preview never changes the document. Only confirm does.
- Selection state is kept as names; after any regen the selection is
  re-resolved, and entities that no longer exist are dropped with a visible
  notice.
- Keyboard shortcuts for every tool and for selection modes; shortcuts are
  data (a table), not scattered handlers.
- The UI shows when an agent has edited the document (edit records with
  author) and lets the person step through and revert those edits.

## Code organisation (guidance)

- Keep a strict split: document model and edit operations (no DOM), kernel
  client (worker, request queue, cancellation), viewport (rendering,
  picking), and tools (flows built from the other three).
- Tools are declarative where possible: a tool is a description of the
  references it needs, the feature it builds, and the handles it offers, with
  code only for what cannot be described.
- Typed throughout; no implicit `any` at module boundaries; tests for edit
  operations and tools at the document level; a small number of end-to-end
  viewport tests.

## Acceptance

- Build the benchmark parts interactively (scripted with a headless driver)
  using only tools, producing documents identical to those in `tools/bench`.
- Dragging a depth handle on a 10-feature part stays interactive on a normal
  laptop.
- Undo and redo restore the exact document and the selection by name.
- An edit made by the agent interface appears in the tree with its author and
  can be reverted from the UI.
