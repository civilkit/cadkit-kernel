# The native service

`ck-native` is an optional local HTTP service that runs the same request dispatch as the wasm module, outside the browser. This chapter covers what it is for, how to start it, its HTTP interface, authentication, limits, and how the TypeScript client uses it.

## What it is for

The service wraps `ck_server::handle`, the function behind the wasm entry point. A request body is the same `{"id":..,"msg":{..}}` text that you pass to the module, and the reply is the same framed reply (see [replies and buffers](reply.md)). Nothing in the protocol changes; only the host does.

Reasons to use it:

- Heavy parts regenerate natively, without the wasm linear-memory ceiling and without the wasm runtime's overhead.
- Each session runs on its own operating-system thread with a 256 MiB stack by default, so deep recursion that would be tight in a worker has room.
- A session keeps its kernel state between requests, exactly like one wasm instance: a `regen` followed by a query on the same `tag` works.

What it is not:

- It is not a remote or multi-user server. It binds to `127.0.0.1` only, speaks plain HTTP, and is meant for the person logged in to the machine.
- It is not a different kernel. There is no separate native protocol; the message types, errors and results are those of the wasm module.
- It is not required. Everything works with the wasm module alone; the client falls back to it (see [the TypeScript client](#the-typescript-client)).

## Build and start

The crate is `crates/ck-native` and produces a binary named `ck-native`. Build it with the `service` profile, which has the optimisations of `release` but unwinds on panic instead of aborting:

```sh
cargo build --profile service -p ck-native
./target/service/ck-native
```

On start it prints three lines to standard error: the version and address, the token file path, and the allowed origins. The default address is `http://127.0.0.1:7461`.

| Flag | Default | Meaning |
| --- | --- | --- |
| `--port N` | `7461` | TCP port on `127.0.0.1`, 0 to 65535. `0` lets the system pick one; the chosen port is printed. A larger number is refused. |
| `--config-dir DIR` | see below | Directory for the token, paired clients and pairing code. |
| `--allow-origin URL` | none | Add an allowed browser origin. Repeatable. |
| `--site-origin URL` | none | Synonym for `--allow-origin`. |
| `--max-body-mib N` | `64` | Largest request body, in MiB. |
| `--max-reply-mib N` | `1024` | Largest reply body, in MiB. |
| `--max-sessions N` | `8` | Sessions that may exist at once. |
| `--idle-secs N` | `600` | A session idle this long is removed. |
| `--stack-mib N` | `256` | Stack size of each session's worker thread. |
| `--quiet` | off | Do not log requests. |
| `--print-token` | off | Print the shared token and exit. |
| `--pair` | off | Write a one-time pairing code, print it and exit. |
| `-h`, `--help` | | Print usage. |

Two subcommands work on the config directory and exit: `ck-native clients` lists paired clients (id, label, origin, creation time), and `ck-native revoke <id>` removes one. An unknown flag prints the usage and exits with status 2.

The config directory is, in order: `CK_NATIVE_CONFIG_DIR`, `$XDG_CONFIG_HOME/ck-native`, `$HOME/.config/ck-native`. The listener always uses `127.0.0.1`; there is no flag to change the address.

The origin `http://localhost:5173` is always allowed. `--allow-origin` adds to it (a trailing slash is ignored; comparison is case-insensitive).

## HTTP interface

Every response carries `Content-Length`, `Cache-Control: no-store` and `X-Content-Type-Options: nosniff`. Connections are HTTP/1.1 keep-alive. Request bodies must have a `Content-Length`; `Transfer-Encoding` (chunked) is refused. Any query string is ignored.

Errors are JSON of the form `{"error":"message"}`.

| Method and path | Auth | Request | Success reply |
| --- | --- | --- | --- |
| `GET /ping` | none | none | `{"service":"ck-native","protocol":1}` |
| `POST /pair` | one-time code | `{"code","origin","label"}` | `{"token","id","origin","label"}` |
| `GET /version` | token | none | JSON, see below |
| `POST /session` | token | empty body | `{"session":"<id>"}` |
| `POST /session/<id>/rpc` | token | a protocol message | the framed reply, `application/octet-stream` |
| `DELETE /session/<id>` | token | none | `{"closed":true}` |
| `OPTIONS` (any path) | none | CORS preflight | `204` with CORS headers |

A known path with the wrong method answers `405`; an unknown path answers `404`. Both require valid authentication first, except for the two unauthenticated routes.

### Version and handshake

`GET /ping` returns exactly the service name and the protocol number, with no token. It reveals nothing else; it exists so a client can discover a running service.

`GET /version` requires a token and returns:

```json
{"kernel":"0.1.0","protocol":1,"git":"<12-hex>","build":"<wasm commit string>","cores":8,"sessions":0,"maxSessions":8}
```

| Field | Meaning |
| --- | --- |
| `kernel` | The crate version of `ck-native`. |
| `protocol` | The protocol number of the HTTP shape and message framing (currently `1`). |
| `git` | Short hash of the checkout the service was built from, or `unknown`. |
| `build` | The build identifier compiled into the kernel (the same string a wasm build from that source reports). |
| `cores` | Available parallelism. |
| `sessions`, `maxSessions` | Current and maximum number of sessions. |

A client checks compatibility by comparing `protocol` with the number it was built for. A different number means the HTTP shape or framing differs and the service must not be used. A client may additionally compare `build` with the build of its own wasm module, to refuse a service built from different source.

### Sessions and rpc

`POST /session` creates a session (a worker thread with its own kernel state) and returns a random 16-hex-digit id. At the session limit it answers `429`.

`POST /session/<id>/rpc` takes the message text as the body and returns the frame. The `Content-Type` of the request is not checked. A message that the kernel rejects is still a `200` with `"ok":false` inside the frame, as with the wasm module (see [errors](errors.md)). Status codes other than `200` mean the transport or service failed, not the kernel request.

Requests on one session run one after another on that session's thread; different sessions run in parallel. An unknown or expired id answers `404`.

`DELETE /session/<id>` removes the session and ends its worker. Unknown id: `404`.

A session whose last request finished more than the idle time ago (default 600 s) is removed by a background reaper; a session with a request in progress is not. The next request for it answers `404`.

## Authentication

Two kinds of bearer token are accepted, sent as `Authorization: Bearer <token>`:

- The shared token. On first start the service creates `<config-dir>/token`, a random 256-bit value as 64 hex digits, with file mode 600 in a directory of mode 700 (Unix). The file is reused afterwards if it holds at least 32 characters. `ck-native --print-token` prints it. It is not bound to an origin.
- A paired token, issued by `POST /pair`. It is bound to the origin it was paired with.

Tokens are compared in constant time; a paired token is compared by its SHA-256 only, and the service stores only the hash.

### Pairing

Pairing lets a web page obtain its own token without the user copying the shared one.

1. The user runs `ck-native --pair`. It writes `pairing.json` in the config dir and prints `pairing code: <8 hex digits>`. The code is valid for 2 minutes and single use.
2. The page sends `POST /pair` with `{"code":"..","origin":"..","label":".."}`. The `origin` must be an allowed origin, and if the request has an `Origin` header it must equal the body's `origin`. The label is truncated to 64 characters.
3. On success the service returns a new token (64 hex digits), a client `id` (8 hex digits), and the origin and label, and records the client in `clients.json` (hash only, mode 600).

Failures: `401` with `no pairing code`, `wrong pairing code` or `pairing code expired`; `403` for an origin that is not allowed or does not match the page; `400` for a malformed body; `413` for a body over 4096 bytes; `501` if the service has no config dir. A wrong guess does not consume the real code, but after 5 wrong guesses the pending code is cancelled (`too many wrong pairing codes; the code was cancelled`) and `ck-native --pair` must be run again. Concurrent attempts on one code: exactly one succeeds (tested with eight at once).

A paired token works only from its origin: a request that carries a different `Origin` answers `403` (`token not valid for this origin`), and one without an `Origin` header (a non-browser client) is accepted. `ck-native revoke <id>` takes effect on a running service at its next request, because the service reloads `clients.json` when the file changes.

## Origin and host checks

The `Host` and `Origin` checks run before any route, including the unauthenticated `GET /ping` and `POST /pair`, and a rejection closes the connection.

| Condition | Status |
| --- | --- |
| `Host` is not exactly `127.0.0.1:<port>` or `localhost:<port>` (port is the one the service listens on; missing `Host` included) | `403 bad Host` |
| `Origin` present and not on the allow-list (`null` included) | `403 Origin not allowed` |
| `OPTIONS` without an `Origin` | `403` |

After these checks, every route except `GET /ping` and `POST /pair` (and an allowed `OPTIONS` preflight) needs a valid bearer token; without one the answer is `401 missing or wrong token`.

A request with no `Origin` header is accepted, since non-browser clients do not send one. For an allowed origin the reply carries `Access-Control-Allow-Origin` (the origin itself) and `Vary: Origin`. A preflight (`OPTIONS`, `204`, no token needed) adds `Access-Control-Allow-Methods: GET, POST, DELETE, OPTIONS`, `Access-Control-Allow-Headers: authorization, content-type`, `Access-Control-Allow-Private-Network: true` and `Access-Control-Max-Age: 600`. Rejected origins receive no CORS headers.

The `Host` check means a name that resolves to the loopback address, such as a DNS-rebinding hostname, is refused; so is `[::1]` and `127.0.0.1` without the port.

## Limits, timeouts, concurrency and panics

| Limit | Value | Result |
| --- | --- | --- |
| Request headers | 32 KiB in total, 100 headers | `431` / `400` |
| Request body | `--max-body-mib`, default 64 MiB | `413`, decided from `Content-Length` before the body is read; connection closed |
| Missing or invalid `Content-Length` | | `411` / `400` |
| Chunked request body | | `501` |
| Reply body | `--max-reply-mib`, default 1 GiB | `413`; checked after the kernel has run, and the session stays usable |
| Sessions | `--max-sessions`, default 8 | `429` on create |
| Connections | 64 at once | an extra connection gets `503` with `Retry-After: 1` |
| Socket read timeout | 300 s per read | the connection ends |

There is no server-side time limit on a kernel request; any budgets inside the kernel are not covered here. The client enforces its own timeout (below).

Each connection has its own thread, and each session its own worker thread; the kernel keeps state per thread, so sessions share nothing.

A panic inside the kernel is caught with `catch_unwind` on the worker. This works only in a binary built with the `service` profile; the `release` profile aborts on panic, which would end the whole process. The worker then ends, because its state may be inconsistent, and the request answers `500` with `{"error":"kernel panic; the session was closed"}`. The session is removed, so later requests for it answer `404`. The panic hook logs only the source location, never document text. Other sessions are not affected. No test covers the panic path.

The request log (standard error unless `--quiet`) records method, path, status, size and time, and never a body, token or pairing code; a test checks this.

## Body framing

The request body is the message text, UTF-8 JSON, identical to the input of the wasm module. The success reply body is the binary frame described in [replies and buffers](reply.md): a little-endian `u32` JSON length, the JSON, padding to a multiple of 8, then the binary area. The tests decode it the same way for both transports and require the HTTP reply to equal a direct call of the dispatch (apart from the `ms` timing field).

## The TypeScript client

`packages/kernel-client/src/native.ts` implements the transport.

- `probeService({url, token, expect})` calls `GET /version` with a 5 s timeout. It never throws; it returns `{ok: true, version}` or a failure of kind `unreachable`, `unauthorized` (401 or 403), `invalid` (not a kernel service) or `mismatch`. A mismatch is a different `protocol`, or, when `expect.build` is given, a `build`/`git` that is not the same commit. A bare `127.0.0.1:7461` is read as `http://`.
- `NativeKernel.connect(options)` probes first and throws `NativeConnectError` on failure; the session opens lazily on the first request.
- `request(msg)` sends messages one at a time in call order and returns the same reply shape as the wasm transport. A transport failure is `{ok: false, trap}`, a refusal is `{ok: false, error}`. A `5xx` is reported as `kernel crashed`; a `4xx` other than 401, 403 or 404 is a refusal of that request and the session continues.
- A failure that makes the session unusable (service gone, token refused, session expired by `404`, timeout, unreadable reply) is reported once through `onFailure`. The application then switches to the built-in wasm kernel. The client never continues a session on another kernel by itself.
- Defaults: request timeout 5 minutes, and a `ping` message every 30 s while idle so the service does not expire the session. After a stale idle connection fails, the client re-sends once if `GET /version` still answers.
- `cancel()` aborts what is in flight and drops queued requests; `close()` also sends `DELETE /session/<id>`.
- Requests use `Authorization` headers only, never a URL parameter; they omit cookies and send a local-network hint when a page on a public address calls the loopback address.

The client does not call `/ping` or `/pair`; both are for other tools. Choosing when to use the service is left to the application.

## Security notes

- Keep the config directory private. Anyone who can read `token` can run kernel requests; the service has no other protection against local users.
- The service listens on loopback only and speaks unencrypted HTTP. Do not forward its port to other machines.
- The `Host` and `Origin` checks and the token together are what stop other web pages from using the service. Add an origin only for a site you trust.
- Prefer a paired token for a web page: it is bound to that origin and can be revoked without changing the shared token.
- The pairing code is 8 hex digits valid for 2 minutes. The service applies no limit on wrong guesses (a wrong guess leaves the code valid), so run `--pair` only when you are about to pair.
- The service creates its random values from `/dev/urandom`; it reads `/dev/urandom`, so the service targets Unix-like systems.
- The HTTP routes take only protocol messages, and no message type touches host files: the request dispatcher and the kernel crates do not use the filesystem (only the service itself reads and writes its token and pairing files).

## See also

- [Protocol overview](overview.md)
- [Replies and buffers](reply.md)
- [Errors](errors.md)
- [Call it from Node](../getting-started/node.md)
