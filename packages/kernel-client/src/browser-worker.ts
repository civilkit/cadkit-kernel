// browser-worker.ts: module worker entry
import { WasmHost } from './wasm-host.js';
import { transferList, type HostRequest, type HostResponse } from './protocol.js';

let host: WasmHost | null = null;
const post = (m: HostResponse) => (self as unknown as Worker).postMessage(m);

self.onmessage = async (ev: MessageEvent<HostRequest>) => {
  const m = ev.data;
  if (m.kind === 'init') {
    try {
      if (!('wasmUrl' in m)) throw new Error('browser worker needs a wasmUrl');
      const res = await fetch(m.wasmUrl);
      host = new WasmHost(await WebAssembly.compile(await res.arrayBuffer()));
      post({ kind: 'ready', commit: host.commit });
    } catch (e) {
      post({ kind: 'fail', error: String((e as Error).message ?? e) });
    }
    return;
  }
  if (!host) return post({ kind: 'done', id: m.id, reply: { ok: false, trap: 'worker has no instance' } });
  const reply = host.call(m.body);
  // the reply's bulk arrays (display mesh) are handed over, not copied: a regen reply carries several MB of them
  (self as unknown as Worker).postMessage({ kind: 'done', id: m.id, reply } satisfies HostResponse, transferList(reply));
};
