// Runs inside node:worker_threads. Owns one WasmHost and answers
// {kind:'call', id, body} with {kind:'done', id, reply}. The host reboots the
// instance after a trap.
import { parentPort } from 'node:worker_threads';
import { readFileSync } from 'node:fs';
import { WasmHost } from './wasm-host.js';
import { transferList, type HostRequest, type HostResponse } from './protocol.js';

const post = (m: HostResponse, transfer: ArrayBuffer[] = []) => parentPort!.postMessage(m, transfer);

let host: WasmHost | null = null;

parentPort!.on('message', (m: HostRequest) => {
  if (m.kind === 'init') {
    try {
      if (!('wasmPath' in m)) throw new Error('node worker needs a wasmPath');
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
  // the reply's bulk arrays (display mesh) are handed over, not copied
  const reply = host.call(m.body);
  post({ kind: 'done', id: m.id, reply }, transferList(reply));
});
