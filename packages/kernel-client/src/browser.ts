// browser.ts: KernelTransport over a module Worker. The worker fetches the
// wasm and hosts it with WasmHost (same protocol as the Node worker).
import { encodeRequest, type HostResponse, type KernelReply } from './protocol.js';
import { PendingCalls } from './pending.js';

export interface BrowserKernel {
  readonly commit: string;
  request(msg: Record<string, unknown>): Promise<KernelReply>;
  close(): void;
}

export async function createBrowserKernel(wasmUrl: string): Promise<BrowserKernel> {
  const worker = new Worker(new URL('./browser-worker.ts', import.meta.url), { type: 'module' });
  const pending = new PendingCalls();
  const ready = await new Promise<{ commit: string }>((resolve, reject) => {
    worker.onerror = (e) => reject(new Error(e.message));
    worker.onmessage = (ev: MessageEvent<HostResponse>) => {
      const m = ev.data;
      if (m.kind === 'ready') resolve({ commit: m.commit });
      else if (m.kind === 'fail') reject(new Error(m.error));
      else if (m.kind === 'done') pending.settle(m.id, m.reply);
    };
    worker.postMessage({ kind: 'init', wasmUrl });
  });
  // after startup: a crash or an unreadable message must not leave calls waiting forever
  worker.onerror = (e) => pending.fail(`kernel worker crashed: ${e.message}`);
  worker.onmessageerror = () => pending.fail('kernel worker sent an unreadable message');
  return {
    commit: ready.commit,
    request(msg) {
      const { id, reply } = pending.open();
      if (pending.dead !== null) return reply;
      try {
        worker.postMessage({ kind: 'call', id, body: encodeRequest(id, msg) });
      } catch (e) {
        pending.fail(`kernel worker refused the call: ${(e as Error).message}`);
      }
      return reply;
    },
    close() {
      pending.fail('kernel closed');
      worker.terminate();
    },
  };
}
