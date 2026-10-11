# ck-server

Message dispatch and session state: the kernel's "server" half. Depends on
everything below.

## Modules

- `session.rs` — session/document state (the `{session:...}` envelope the
  frontend sends for server mode), undo/redo, rollback
- `model.rs` — part/assembly CRUD ops (the drag/move/edit commands)
- `geometry.rs` — geometry queries (curvatureAt, closest, sections)
- `query.rs` — selection/pick queries
- `view.rs` — view-dependent output (orientation frames, viewports)
- `drawing.rs` — 2D drawing extraction (views, hidden lines, dims)
- `comb.rs` — curvature combs
- `curvature.rs` — curvature maps

## Interface

ck_request(ptr,len) -> dispatch by msg.type -> build result JSON + buffers
-> ck_reply(). All message type strings (96 known) and result shapes are
pinned by app.pretty.js + oracle goldens. Error paths produce
`{"ok":false,"error":"..."}` with the EXACT strings from kernel.strings
(e.g. "bad request: msg must be an object", "message without a type").

## Rebuild notes

- Build the dispatch table from the type list in root README; each handler
  starts as a stub returning the pinned error string, then gets real
  behavior bottom-up.
