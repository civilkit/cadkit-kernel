// browser-worker.ts: module worker entry
import { WasmHost } from './wasm-host.js';
import type { HostRequest, HostResponse } from './protocol.js';

let host: WasmHost | null = null;
const post = (m: HostResponse) => (self as unknown as Worker).postMessage(m);

self.onmessage = async (ev: MessageEvent<HostRequest & { wasmUrl?: string }>) => {
  const m = ev.data;
  if (m.kind === 'init') {
    try {
      const res = await fetch((m as unknown as { wasmUrl: string }).wasmUrl);
      host = new WasmHost(await WebAssembly.compile(await res.arrayBuffer()));
      post({ kind: 'ready', commit: host.commit });
    } catch (e) {
      post({ kind: 'fail', error: String((e as Error).message ?? e) });
    }
    return;
  }
  if (!host) return post({ kind: 'done', id: m.id, reply: { ok: false, trap: 'worker has no instance' } });
  post({ kind: 'done', id: m.id, reply: host.call(m.body) });
};
