# ck-native: local kernel service

A native binary that serves the kernel protocol (`ck_server::handle`: one JSON message in, framed reply bytes out) over HTTP on 127.0.0.1. Plan: kit `docs/plan-local-kernel-service.md`. Phase 1 (minimal, secure, no async runtime); v1.1 adds discovery (`/ping`) and pairing (below) without changing anything an existing client sends.

## Build and run

```
cargo build --profile service -p ck-native      # target/service/ck-native
target/service/ck-native --port 7461 --site-origin https://app.example.com
```

Use the `service` profile (same optimisation as `release`, but `panic = "unwind"`): the release profile aborts on panic, which would kill every session; the service catches a kernel panic, answers 500 and closes only that session. `cargo build -p ck-native` also works (debug, slow). The crate is a normal workspace member, but nothing the gate builds depends on it (the gate uses `-p ck-wasm` and `-p ck-model/-ops/-text` only); ck-wasm is unchanged (its `engine::install` is already public in the rlib, ck-native links it).

No third-party dependencies at all: a hand-written HTTP/1.1 parser over `std::net` (keep-alive, `Content-Length` bodies; chunked request bodies and gzip are not supported). One thread per connection (max 64), one worker thread per session with a 256 MiB stack (the kernel recurses deeply in regen; virtual memory only, pages are touched on demand).

## Discovery

The default port is **7461**. A client probes **7461 first, then 7462 to 7469** (the documented probe range) and takes the first port that answers `GET /ping`; a user-entered address is the fallback. `--port 0` binds a free port and prints it (discovery then ends at that address). `/ping` needs no token, so a page can tell "installed and running" from "not running" without credentials (see HTTP shape: it carries nothing but the two advertised fields).

## Token and flags

On first start a 256-bit random token is written to `<config-dir>/token` (mode 600; default `$CK_NATIVE_CONFIG_DIR`, else `$XDG_CONFIG_HOME/ck-native`, else `~/.config/ck-native`; the config dir itself is mode 700). The path is printed at start, the token never is (not in logs either). `--print-token` prints it and exits (to paste into the app). This **shared token** keeps working unchanged for scripts and CI; paired clients use per-client tokens instead (below).

| flag | default | meaning |
|---|---|---|
| `--port N` | 7461 | 0 = random (printed) |
| `--config-dir DIR` | see above | token and pairing records |
| `--allow-origin URL`, `--site-origin URL` | `http://localhost:5173` always allowed | extra allowed Origins, repeatable |
| `--max-body-mib N` | 64 | request body limit (413, rejected from the header before reading) |
| `--max-reply-mib N` | 1024 | reply limit (413) |
| `--max-sessions N` | 8 | 429 when full |
| `--idle-secs N` | 600 | idle sessions are closed (404 afterwards) |
| `--stack-mib N` | 256 | session thread stack |
| `--quiet` | off | no request log (the log is method, route, status, size, ms only; never bodies or the token) |

## Pairing and clients (v1.1)

```
ck-native --pair              # prints: a one-time code, valid 2 minutes, single use
ck-native clients             # list paired clients (id, label, origin, created)
ck-native revoke <id>         # revoke one; a running service applies it on the next request
```

`--pair` writes the code to `<config-dir>/pairing.json` (mode 600). A client then sends `POST /pair {code, origin, label}` and gets a **per-client token** back: `{"token":"<64 hex>","id":"<8 hex>","origin":"<origin>","label":"<label>"}`. The code is consumed under a lock (concurrent attempts: exactly one wins) and `pairing.json` is removed, so it is single use; an expired code (>120 s) is refused with `pairing code expired`.

Records live in `<config-dir>/clients.json` (dir mode 700, file mode 600): id, the **SHA-256 hash** of the token (never the token), bound origin, label, creation time. The service reloads the file when it changes, so `revoke` is effective immediately without a restart. A paired token only authenticates from the origin it was paired with (`403` elsewhere, and a page that sends an `Origin` must pair that same origin); without an `Origin` header the token works like any non-browser client. `--pair` needs the code file to be readable by the same user, so it can run before or after the service starts.

## HTTP shape

Every request except an allowed `OPTIONS` preflight, `GET /ping` and `POST /pair` needs `Authorization: Bearer <token>` (the shared token or a paired client's token).

- `GET /ping` -> `200` with **only** `{"service":"ck-native","protocol":N}` — no token, no version, git, build or counts; `CORS` headers only for allowed Origins (`403` and none otherwise; `Origin: null` refused). Unauthenticated capability probe for discovery.
- `POST /pair`, body `{"code":"...","origin":"...","label":"..."}` (the one-time code from `--pair`) -> `200 {"token","id","origin","label"}`; `401` for a wrong, expired, already-used or absent code; `403` when the claimed origin is not on the allow-list or differs from the request's `Origin`; `400` for a bad body; `501` when the service runs without a config dir (tests only).
- `POST /session` -> `200 {"session":"<id>"}`; `429` at the session limit.
- `POST /session/<id>/rpc`, body = the message text exactly as the wasm gets it (`{"id":..,"msg":{..}}`) -> `200` with the reply bytes unchanged (`application/octet-stream`: u32 LE JSON length, the JSON, padding to 8, then the binary buffers; identical framing to `ck_reply`). A message the kernel refuses is still a 200 with an error reply (e.g. `ping` of an unknown type); transport problems use 4xx; a kernel panic gives `500` and closes the session. Unknown session `404`.
- `DELETE /session/<id>` -> `200 {"closed":true}`, `404` if unknown.
- `GET /version` -> `{"kernel":"0.1.0","protocol":1,"git":"<12-char commit of this build>","build":"<wasm-equivalent build id, ck_wasm::COMMIT>","cores":N,"sessions":N,"maxSessions":N}`. Needs the token like every call except `GET /ping` and `POST /pair`. `protocol` is a number, bumped on any incompatible HTTP/framing change: `/ping` and `/pair` are additive, so it stays **1**.

Errors are `{"error":"..."}` with 400 (bad request), 401 (token or pairing code), 403 (Host/Origin), 404, 405, 411/413/431, 501 (chunked / pairing not configured), 429, 500.

## Security checks (order)

1. `Host` must be `127.0.0.1:<port>` or `localhost:<port>` (DNS rebinding): else 403.
2. `Origin`, if present, must be in the allow-list (exact match): else 403 with no CORS headers. An absent Origin is allowed (non-browser clients) but still needs the token (or, for `/pair`, the code). `Origin: null` is refused.
3. `OPTIONS` with an allowed Origin answers 204 with `Access-Control-Allow-Origin`, `-Methods`, `-Headers: authorization, content-type`, `Access-Control-Allow-Private-Network: true`, no token needed; any other preflight is 403 with no CORS headers.
4. `GET /ping` answers without a token (only `service` and `protocol` in the body).
5. `POST /pair` authenticates with the one-time code instead of a token; the claimed origin must be allow-listed and match the request's Origin when present.
6. Token compared in constant time: the shared token, or the SHA-256 of a paired client's token whose bound origin matches the request's Origin (a token from another origin is 403 without CORS headers, a wrong token 401).
7. Limits above; the only input is the protocol message (no file access of any kind); the bind address is 127.0.0.1 only. Idle connections are kept 300 s.

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

Tests (13): `cargo test -p ck-native` runs the protocol tests over a loopback socket (regen reply equals a direct `handle`, auth/Origin/Host matrix, preflight, limits, max sessions, parallel session isolation, keep-alive, idle timeout) plus the v1.1 matrix: `/ping` with and without an allowed Origin and with `Origin: null` (exact body, CORS only for allowed origins), wrong / expired / reused pairing codes and the header-vs-body origin rules, a revoked token dying live, a paired token used from the wrong origin, eight concurrent pairings winning exactly once, and an end-to-end run of the real binary (`--pair` -> `POST /pair` -> `clients` -> `revoke`) that asserts the config dir/file modes, that tokens are stored hashed, and that no token or code ever appears in the request log.

## Not done (phase 2+)

gzip, WebSocket, binary mesh buffers, `--token-file`, cancel of an in-flight regen, TLS, per-session memory cap (only stack size and the body/reply limits), multi-core tessellation. Kit side (Settings showing paired state, discovery probing the port list) is a separate follow-up once this lands.
