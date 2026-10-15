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
  const reply = host.call(m.body);
  // the reply's bulk arrays (display mesh) are handed over, not copied: a regen reply carries several MB of them
  const bufs = new Set<ArrayBuffer>();
  const walk = (v: unknown): void => {
    if (ArrayBuffer.isView(v)) { if (v.buffer instanceof ArrayBuffer && v.byteLength > 0) bufs.add(v.buffer); return; }
    if (Array.isArray(v)) { for (const x of v) walk(x); return; }
    if (v && typeof v === 'object') for (const x of Object.values(v)) walk(x);
  };
  walk(reply);
  (self as unknown as Worker).postMessage({ kind: 'done', id: m.id, reply } satisfies HostResponse, [...bufs]);
};
