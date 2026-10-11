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
-> ck_reply(). All 17 message type strings and result shapes are
pinned by app.pretty.js + oracle goldens. Error paths produce
`{"ok":false,"error":"..."}` with the EXACT strings from kernel.strings
(e.g. "bad request: msg must be an object", "message without a type").

## Status (pinned against kernel.wasm)

- Exactly **17** request types exist (brute-forced: every identifier/substring
  in kernel.strings + app.pretty.js was sent as `msg.type`): ping, regen,
  matchEdges, drag, convert, sketchOffset, refCurve, measure, massProps,
  interference, step, inspect, continuity, curvatureAt, curvatureMap, comb,
  drawing. The "96" figure elsewhere counts feature types, not requests.
- Envelope, `ms` clock arithmetic, debug trace/buffers fields, tag/state
  semantics and every empty-state answer are byte-identical to the oracle
  (`node tools/diff.mjs`, tests-golden/wire.json).
- Geometry plugs in via `ck_server::set_hooks(Hooks{regen, query, ...})`;
  without hooks, body-bearing paths return `TODO(rebuild): ...` errors.
- Deps trimmed to ck-json only (hooks decouple it from ck-model).

## Rebuild notes

- Build the dispatch table from the type list in root README; each handler
  starts as a stub returning the pinned error string, then gets real
  behavior bottom-up.
