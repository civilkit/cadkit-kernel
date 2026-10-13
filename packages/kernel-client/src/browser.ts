// browser.ts: KernelTransport over a module Worker. The worker fetches the
// wasm and hosts it with WasmHost (same protocol as the Node worker).
import { encodeRequest, type HostResponse, type KernelReply } from './protocol.js';

export interface BrowserKernel {
  readonly commit: string;
  request(msg: Record<string, unknown>): Promise<KernelReply>;
  close(): void;
}

export async function createBrowserKernel(wasmUrl: string): Promise<BrowserKernel> {
  const worker = new Worker(new URL('./browser-worker.ts', import.meta.url), { type: 'module' });
  const pending = new Map<number, (r: KernelReply) => void>();
  let nextId = 1;
  const ready = await new Promise<{ commit: string }>((resolve, reject) => {
    worker.onerror = (e) => reject(new Error(e.message));
    worker.onmessage = (ev: MessageEvent<HostResponse>) => {
      const m = ev.data;
      if (m.kind === 'ready') resolve({ commit: m.commit });
      else if (m.kind === 'fail') reject(new Error(m.error));
      else if (m.kind === 'done') pending.get(m.id)?.(m.reply);
    };
    worker.postMessage({ kind: 'init', wasmUrl });
  });
  return {
    commit: ready.commit,
    request(msg) {
      const id = nextId++;
      return new Promise((resolve) => {
        pending.set(id, (r) => { pending.delete(id); resolve(r); });
        worker.postMessage({ kind: 'call', id, body: encodeRequest(id, msg) });
      });
    },
    close: () => worker.terminate(),
  };
}
