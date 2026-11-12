# Kernel interface

A summary of how a host talks to the kernel. The full reference is the book in `book/src/protocol/`
(overview, replies and buffers, regeneration, queries, errors, native service); this page is the short
version, checked against `crates/ck-wasm/src/lib.rs`, `crates/ck-server/src/lib.rs` and
`crates/ck-server/src/reply.rs`.

The transport is one JSON request in, one framed reply out. The same call is served by the WebAssembly
module (`ck-wasm`) and by the local HTTP service (`ck-native`), which wraps the same `ck_server::handle`.

## WebAssembly exports and imports

| Export | Meaning |
|---|---|
| `memory` | linear memory |
| `ck_alloc(n) -> ptr` | allocate `n` bytes (at least 1) for a request body; ownership passes to `ck_request` |
| `ck_request(ptr, len) -> u32` | run one request; returns the length in bytes of the whole reply frame |
| `ck_reply() -> ptr` | pointer to the last reply frame |
| `ck_commit() -> ptr`, `ck_commit_len() -> u32` | a static build identifier string |

The module imports three host functions from the module `env`: `ck_panic(ptr, len)` (called with the panic
text before the module traps), `ck_now() -> f64` (a millisecond clock, used for `ms` and the time budgets)
and `ck_unix_ms() -> f64`. The release profile uses `panic = "abort"`, so after a panic the host should
discard the instance and create a new one. A constant clock works but disables the time budgets.

A call is: `ck_alloc(len)`, copy the UTF-8 request body to the returned pointer, `ck_request(ptr, len)`, then
read that many bytes at `ck_reply()`.

## Request

```json
{"id": 7, "msg": {"type": "regen", "tag": "main", "part": { ... }, "opts": {"upTo": 3}}}
```

- `id` is ignored and not echoed.
- `msg` must be an object and `msg.type` a non-empty string. `msg.tag` names the kernel state the message
  uses (default `main`); `msg.debug: true` adds timing information to the reply.
- Failures of the envelope itself are error replies, not traps: `bad request: the body must be JSON {id, msg}`,
  `bad request: msg must be an object`, `message without a type`, `unknown request <type>`.

The kernel answers 64 request types (the `REQUEST_TYPES` table in `crates/ck-server/src/lib.rs`), for example
`ping`, `regen`, `massProps`, `measure`, `step`, `importStep`, `validate`, `heal`, `inspect`, `describe`,
`resolve`, `check`, `refDiff`, `section`, `probe` and the `dfm*` queries. Feature operations (extrude, fillet,
shell and so on) are not request types: they are feature types inside the part document sent with `regen`.
A part is a JSON document with `format: "ck-part"` and a `features` array; see `book/src/format/`.

## Reply frame

```
[u32 LE: json_len][json bytes][spaces up to a multiple of 8][buffer 0][buffer 1]...
```

- The JSON is `{"ok":true,"result":...,"ms":...}` or `{"ok":false,"error":"...","ms":...}`. An error reply
  has no binary area. `ok: true` only means the request ran; failures the request models (a feature that cannot
  be built, validator findings, an unparseable import) are reported inside `result`.
- Bulk arrays are views into the binary area: `{"$buf": i, "type": "f32|f64|u32|i32|u8", "len": n,
  "offset": o}`. Buffers start at the 8-aligned end of the padded JSON and are packed in creation order;
  `offset` is a byte offset from the start of the binary area.
- A `regen` result's `body.mesh` (`positions`, `normals`, `mids`, `triFace`) and `body.silhouette` are
  buffer-backed.

## `regen` result

`regen` runs a part document, stores the body under its tag for later queries, and returns an object whose
first keys are always present:

```jsonc
{
  "errors": [],       // failed features: [featureId, {code, params}]; a failing feature does not fail the request
  "components": [],   // inserted parts: {id, dof, place}
  "planes": [], "markers": [], "sketches": [], "sketches3": [], "curves": [],
  "body": null,       // the solid, or null when the document makes none
  "surfaces": []      // sheet bodies: [featureId, body]
}
```

`body` carries the bounding box, the faces (id, kind, whether planar, the feature and entity that made it),
the edges, the silhouette and the display mesh. Optional keys follow when they have content: `threads`,
`limits`, `assembly`, `warnings`, `cacheStats`, `partVersionNewer`. Options in `opts` include `upTo`
(regenerate only the first n features), `solveSketches`, `sketchStatus`, `featureTimeMs`, `regenTimeMs`,
`autoHeal`, `noCache`, `noView` and `cacheStats`. The complete field list is in `book/src/protocol/regen.md`.

## Local service

`ck-native` serves the same messages over HTTP on `127.0.0.1` (default port 7461), with a token and optional
browser pairing; one session per worker thread, each with its own kernel state. See
`book/src/protocol/native-service.md` and `docs/native-service.md`.

## History

The wire format was originally matched against an earlier build of the kernel; the regression gate can
still compare against such a build when one is available, but it is not required (see the README).
