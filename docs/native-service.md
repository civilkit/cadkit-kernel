# ck-native: local kernel service

A native binary that serves the kernel protocol (`ck_server::handle`: one JSON message in, framed reply bytes out) over HTTP on 127.0.0.1. Plan: kit `docs/plan-local-kernel-service.md`. This is phase 1 (minimal, secure, no async runtime).

## Build and run

```
cargo build --profile service -p ck-native      # target/service/ck-native
target/service/ck-native --port 7461 --site-origin https://app.example.com
```

Use the `service` profile (same optimisation as `release`, but `panic = "unwind"`): the release profile aborts on panic, which would kill every session; the service catches a kernel panic, answers 500 and closes only that session. `cargo build -p ck-native` also works (debug, slow). The crate is a normal workspace member, but nothing the gate builds depends on it (the gate uses `-p ck-wasm` and `-p ck-model/-ops/-text` only); ck-wasm is unchanged (its `engine::install` is already public in the rlib, ck-native links it).

No third-party dependencies at all: a hand-written HTTP/1.1 parser over `std::net` (keep-alive, `Content-Length` bodies; chunked request bodies and gzip are not supported). One thread per connection (max 64), one worker thread per session with a 256 MiB stack (the kernel recurses deeply in regen; virtual memory only, pages are touched on demand).

## Token and flags

On first start a 256-bit random token is written to `<config-dir>/token` (mode 600; default `$CK_NATIVE_CONFIG_DIR`, else `$XDG_CONFIG_HOME/ck-native`, else `~/.config/ck-native`). The path is printed at start, the token never is (not in logs either). `--print-token` prints it and exits (to paste into the app). `--pair` and `--token-file` are phase 3.

| flag | default | meaning |
|---|---|---|
| `--port N` | 7461 | 0 = random (printed) |
| `--config-dir DIR` | see above | token location |
| `--allow-origin URL`, `--site-origin URL` | `http://localhost:5173` always allowed | extra allowed Origins, repeatable |
| `--max-body-mib N` | 64 | request body limit (413, rejected from the header before reading) |
| `--max-reply-mib N` | 1024 | reply limit (413) |
| `--max-sessions N` | 8 | 429 when full |
| `--idle-secs N` | 600 | idle sessions are closed (404 afterwards) |
| `--stack-mib N` | 256 | session thread stack |
| `--quiet` | off | no request log (the log is method, route, status, size, ms only; never bodies or the token) |

## HTTP shape

Every request except an allowed `OPTIONS` preflight needs `Authorization: Bearer <token>`.

- `POST /session` -> `200 {"session":"<id>"}`; `429` at the session limit.
- `POST /session/<id>/rpc`, body = the message text exactly as the wasm gets it (`{"id":..,"msg":{..}}`) -> `200` with the reply bytes unchanged (`application/octet-stream`: u32 LE JSON length, the JSON, padding to 8, then the binary buffers; identical framing to `ck_reply`). A message the kernel refuses is still a 200 with an error reply (e.g. `ping` of an unknown type); transport problems use 4xx; a kernel panic gives `500` and closes the session. Unknown session `404`.
- `DELETE /session/<id>` -> `200 {"closed":true}`, `404` if unknown.
- `GET /version` -> `{"kernel":"0.1.0","protocol":1,"git":"<12-char commit of this build>","build":"<wasm-equivalent build id, ck_wasm::COMMIT>","cores":N,"sessions":N,"maxSessions":N}`. `protocol` is a number, bumped on any incompatible HTTP/framing change.

Errors are `{"error":"..."}` with 400 (bad request), 401 (token), 403 (Host/Origin), 404, 405, 411/413/431, 501 (chunked), 429, 500.

## Security checks (order)

1. `Host` must be `127.0.0.1:<port>` or `localhost:<port>` (DNS rebinding): else 403.
2. `Origin`, if present, must be in the allow-list (exact match): else 403. An absent Origin is allowed (non-browser clients) but still needs the token. `Origin: null` is refused.
3. `OPTIONS` with an allowed Origin answers 204 with `Access-Control-Allow-Origin`, `-Methods`, `-Headers: authorization, content-type`, `Access-Control-Allow-Private-Network: true`, no token needed; any other preflight is 403 with no CORS headers.
4. Token compared in constant time: 401.
5. Limits above; the only input is the protocol message (no file access of any kind); the bind address is 127.0.0.1 only. Idle connections are kept 300 s.

A kernel panic log line carries only the source location, never the panic message (which could contain document text).

## Sessions and state

A session is a thread with its own kernel state (thread-locals: regen cache, tags, reply buffer), exactly like one wasm instance per Worker. Each worker installs the native clock (monotonic ms, wall ms) and the geometry hooks (`ck_wasm::engine::install`) when it starts. The warm regen cache lives as long as the session.

## Compare with wasm

```
cargo build --profile service -p ck-native
cargo build -p ck-wasm --target wasm32-unknown-unknown --release
node tools/native_diff.mjs [--bin target/service/ck-native] [--wasm ...] [--time] [--verbose]
```

Starts the binary on a free port, sends every wire case of `tools/cases.mjs` (a fresh session per case, bodies in order) and the large sample and fan shroud regens, and compares with the wasm (`ms`/`at` normalised, JSON and binary part separately, as the gate does). `--time` adds CPU seconds (server process CPU vs this process for wasm; cache off).

Result: the 280 wire cases are byte-identical. The two big documents are NOT byte-identical: floats differ in the last digit (system libm in the native build vs the libm compiled into the wasm), so the script reports the number of differing floats, the maximum relative difference and structural differences, and passes them as "equivalent" only when there are no structural differences and the relative difference is at most 1e-6. Byte-identity for those is impossible without a shared libm.

Tests: `cargo test -p ck-native` runs the protocol tests over a loopback socket (regen reply equals a direct `handle`, auth/Origin/Host matrix, preflight, limits, max sessions, parallel session isolation, keep-alive, idle timeout).

## Not done (phase 3+)

gzip, WebSocket, binary mesh buffers, `--pair`/`--token-file`, cancel of an in-flight regen, TLS, per-session memory cap (only stack size and the body/reply limits), multi-core tessellation.
