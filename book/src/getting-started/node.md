# Call it from Node

This chapter loads the wasm module from Node, sends messages through its five exports and decodes the replies. The host at the end is small enough to copy; the same few steps are what the packaged clients do.

## The module's interface

The module exports its memory and five functions, and imports three functions from the module `env`.

| Export | Signature | Use |
|---|---|---|
| `memory` | `WebAssembly.Memory` | linear memory; requests and replies are exchanged in it |
| `ck_alloc(n)` | `(i32) -> i32` | reserve `n` bytes for a request body and return their address |
| `ck_request(ptr, len)` | `(i32, i32) -> i32` | run the message of `len` bytes at `ptr`; return the length of the reply in bytes |
| `ck_reply()` | `() -> i32` | address of the reply of the last `ck_request` |
| `ck_commit()`, `ck_commit_len()` | `() -> i32` | address and length of the module's build string |

| Import (module `env`) | Signature | Use |
|---|---|---|
| `ck_now` | `() -> f64` | monotonic milliseconds, for example `performance.now()`. Used for the `ms` field of replies and for timing budgets. |
| `ck_unix_ms` | `() -> f64` | wall-clock milliseconds, for example `Date.now()`. Used for time stamps in exported files. |
| `ck_panic` | `(ptr, len) -> ()` | receives the panic text in memory just before the module traps |

The sequence for one call is always the same:

1. Encode the request `{"id": n, "msg": {...}}` as UTF-8.
2. `ptr = ck_alloc(bytes.length)` and copy the bytes to `memory` at `ptr`. Ownership of the block passes to `ck_request`; the host never frees it.
3. `total = ck_request(ptr, bytes.length)`.
4. `at = ck_reply()` and copy `total` bytes from `at`. The reply is overwritten by the next request.

Two details matter in JavaScript. Memory can grow during any call, which detaches the old `memory.buffer`, so read `memory.buffer` again after every call into the module. And the reply must be copied out before the next request.

The module has one global state per instance, and a call runs to completion on the calling thread. Calls on one instance are therefore strictly sequential; for parallel work create several instances (see [Overview](../protocol/overview.md)).

If the kernel panics, the module is built with `panic = "abort"`: `ck_panic` is called with the text, then the call traps (a `WebAssembly.RuntimeError`). The instance cannot be used afterwards. A host carries on by creating a new instance from the same compiled module; all tags and caches of the old instance are gone.

## A complete host

`kernel.mjs`:

```js
// kernel.mjs: a minimal host for the wasm kernel.
import { readFileSync } from 'node:fs';

const ARRAYS = { f32: Float32Array, f64: Float64Array, u32: Uint32Array, i32: Int32Array, u8: Uint8Array };

export class Kernel {
  #module;
  #x;                // the instance's exports
  #panic = null;     // text of the last panic, set through the ck_panic import
  #nextId = 1;

  constructor(wasmPath) {
    this.#module = new WebAssembly.Module(readFileSync(wasmPath));
    this.#boot();
  }

  #boot() {
    const env = {
      ck_now: () => performance.now(),   // ms, fractional; used for the "ms" field of replies
      ck_unix_ms: () => Date.now(),      // wall clock, used for time stamps in exported files
      ck_panic: (ptr, len) => {          // called with the panic text just before the module traps
        this.#panic = new TextDecoder().decode(new Uint8Array(this.#x.memory.buffer, ptr, len));
      },
    };
    this.#x = new WebAssembly.Instance(this.#module, { env }).exports;
  }

  /** The build string of the module (ck_commit / ck_commit_len). */
  get commit() {
    const { memory, ck_commit, ck_commit_len } = this.#x;
    return new TextDecoder().decode(new Uint8Array(memory.buffer, ck_commit(), ck_commit_len()));
  }

  /** Send one message. Returns the parsed reply: { ok, result | error, ms }. Throws if the module traps. */
  send(msg) {
    const body = new TextEncoder().encode(JSON.stringify({ id: this.#nextId++, msg }));
    let reply;
    try {
      const x = this.#x;
      const ptr = x.ck_alloc(body.length);                          // room for the request; ck_request frees it
      new Uint8Array(x.memory.buffer, ptr, body.length).set(body);  // read memory.buffer after ck_alloc: memory may have grown
      const total = x.ck_request(ptr, body.length);                 // run the message; returns the reply length
      const at = x.ck_reply();                                      // where the reply is
      reply = new Uint8Array(x.memory.buffer.slice(at, at + total)); // copy it out: the next request overwrites it
    } catch (e) {
      const why = e.message + (this.#panic ? ` (panic: ${this.#panic})` : '');
      this.#panic = null;
      this.#boot();                                                 // the instance is unusable after a trap: start a new one
      throw new Error('kernel trap: ' + why);
    }
    return decodeReply(reply);
  }
}

/** [u32 LE json length][json][pad to 8][buffers...] -> object, with every {$buf} replaced by a typed array. */
export function decodeReply(bytes) {
  const jsonLength = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength).getUint32(0, true);
  const json = JSON.parse(new TextDecoder().decode(bytes.subarray(4, 4 + jsonLength)));
  const base = (4 + jsonLength + 7) & ~7;                           // the binary area starts at the next multiple of 8
  const resolve = (v) => {
    if (Array.isArray(v)) return v.map(resolve);
    if (v === null || typeof v !== 'object') return v;
    if ('$buf' in v) {
      const Array_ = ARRAYS[v.type];
      const start = base + v.offset;                                // offset is relative to the binary area
      return new Array_(bytes.slice(start, start + v.len * Array_.BYTES_PER_ELEMENT).buffer);
    }
    return Object.fromEntries(Object.entries(v).map(([k, x]) => [k, resolve(x)]));
  };
  return resolve(json);
}
```

The binary-area layout and the `$buf` views are specified in [Replies and buffers](../protocol/reply.md). `book/tools/run.mjs` in the repository is the same host reduced to a command-line runner: it runs one message or an array of messages on one instance and prints the replies (`node book/tools/run.mjs <wasm> <messages.json> [--full]`). Its `ck_now` returns 0, so every `ms` it prints is 0; the other examples in this book use it for that reason, and the `ms` values in them are placeholders.

### Using it

`main.mjs` regenerates a 40 x 20 x 5 plate and reads two results:

```js
import { Kernel } from './kernel.mjs';

const k = new Kernel(process.argv[2] ?? 'target/wasm32-unknown-unknown/release/ck_wasm.wasm');
console.log(k.send({ type: 'ping' }));

// a 40 x 20 x 5 plate: one sketch, one extrude
const rect = (id, w, h) => ({
  type: 'sketch', id, plane: { kind: 'std', id: 'front' },
  points: [{ id: 'a', x: 0, y: 0 }, { id: 'b', x: w, y: 0 }, { id: 'c', x: w, y: h }, { id: 'd', x: 0, y: h }],
  entities: [['e1', 'a', 'b'], ['e2', 'b', 'c'], ['e3', 'c', 'd'], ['e4', 'd', 'a']]
    .map(([id, p1, p2]) => ({ id, type: 'line', p1, p2 })),
  constraints: [],
});
const part = {
  format: 'ck-part', version: 1, name: 'Plate',
  features: [rect('F1', 40, 20), { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 5 }],
};

const r = k.send({ type: 'regen', tag: 'main', part });
const { body } = r.result;
console.log('ok', r.ok, 'errors', r.result.errors.length, 'faces', body.faces.length, 'edges', body.edges.length);
console.log('box', JSON.stringify(body.box));
console.log('mesh', body.mesh.positions.constructor.name, body.mesh.positions.length / 9, 'triangles');
console.log(k.send({ type: 'massProps', tag: 'main' }).result.volume);
console.log(k.send({ type: 'nope' }));
```

Run with `node main.mjs`, this prints (the `ms` values and the `build` string depend on the machine and the build):

```text
{ ok: true, result: { pong: true, build: '8302a07430fb' }, ms: 0.57 }
ok true errors 0 faces 6 edges 12
box {"min":[0,0,0],"max":[40,20,5]}
mesh Float32Array 12 triangles
4000.0000000000027
{ ok: false, error: 'unknown request nope', ms: 0.01 }
```

The volume is 40 x 20 x 5 = 4000 up to rounding. A request the kernel refuses is not an exception: it is a normal reply with `ok: false` and an `error` string ([Errors](../protocol/errors.md)). Only a trap throws.

## Alternatives to a hand-written host

The TypeScript package `packages/kernel-client` wraps the same steps in three transports with one shape: `request(msg)` returns a promise of `{ ok: true, result }`, `{ ok: false, error }` (the kernel refused the message) or `{ ok: false, trap }` (the module crashed or the transport failed). Replies come back already decoded, with typed arrays in place of `$buf` views.

| Transport | Entry point | Where the kernel runs |
|---|---|---|
| Node worker | `createWorkerKernel(wasmPath)` in `worker.ts` | a `worker_threads` worker that owns one `WasmHost` (`wasm-host.ts`); the main thread stays free during long regens. `wasmPath` defaults to `$CK_WASM`, else the release build in `target/`. |
| Browser worker | `createBrowserKernel(wasmUrl)` in `browser.ts` | a module worker that fetches and compiles the module, then hosts it with the same `WasmHost`. Large arrays are transferred, not copied. |
| Native service | `NativeKernel.connect({ url, token })` in `native.ts` | the `ck-native` process, over HTTP on the loopback interface |

`WasmHost` is the class that does what `Kernel` above does: it reboots the instance after a trap and returns a trap as a result of the form `{ ok: false, trap }`. Both worker transports exchange `{kind: 'init' | 'call' | 'done' | 'ready' | 'fail', ...}` messages with it.

The native service speaks the same protocol over HTTP. A client opens a session with `POST /session`, which answers `{"session": "<id>"}`, then posts each request body, exactly the text a wasm host would pass to `ck_request`, to `POST /session/<id>/rpc` and gets the reply bytes back unchanged (`application/octet-stream`, the same framing as `ck_reply`). Every call except `GET /ping` and the pairing call carries `Authorization: Bearer <token>`. A message the kernel refuses is still HTTP 200 with an error reply; a kernel panic (in a service built with the `service` profile) is HTTP 500 and closes that session. Each session has its own kernel state, like one wasm instance per worker. `NativeKernel` also checks `GET /version` against the protocol number it was built for (`KERNEL_PROTOCOL`, currently 1) before it opens a session. The full description is [The native service](../protocol/native-service.md); [Build the kernel](build.md) shows how to build the service.

## See also

- [Build the kernel](build.md)
- [A first part](first-part.md)
- [Protocol overview](../protocol/overview.md)
- [Replies and buffers](../protocol/reply.md)
