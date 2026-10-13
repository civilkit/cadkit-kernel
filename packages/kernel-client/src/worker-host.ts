// Runs inside node:worker_threads. Owns one WasmHost and answers
// {kind:'call', id, body} with {kind:'done', id, reply}. The host reboots the
// instance after a trap.
import { parentPort } from 'node:worker_threads';
import { readFileSync } from 'node:fs';
import { WasmHost } from './wasm-host.js';
import type { HostRequest, HostResponse } from './protocol.js';

const post = (m: HostResponse) => parentPort!.postMessage(m);

let host: WasmHost | null = null;

parentPort!.on('message', (m: HostRequest) => {
  if (m.kind === 'init') {
    try {
      host = new WasmHost(new WebAssembly.Module(readFileSync(m.wasmPath)));
      post({ kind: 'ready', commit: host.commit });
    } catch (e) {
      post({ kind: 'fail', error: String((e as Error).message ?? e) });
    }
    return;
  }
  if (host === null) {
    post({ kind: 'done', id: m.id, reply: { ok: false, trap: 'worker has no instance' } });
    return;
  }
  post({ kind: 'done', id: m.id, reply: host.call(m.body) });
});
