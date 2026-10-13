// Main-thread facade for the wasm worker host (./worker-host.js, compiled to
// dist next to this file). One WorkerKernel per instance: owns the worker,
// tags calls with numeric ids, resolves replies by id.
//
// No browser involved: the gate can boot this anywhere node runs.
import { existsSync } from 'node:fs';
import { Worker } from 'node:worker_threads';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { encodeRequest, type HostResponse, type KernelReply } from './protocol.js';

export type { KernelReply };

export interface WorkerKernel {
  readonly wasmPath: string;
  readonly commit: string;
  request(msg: Record<string, unknown>): Promise<KernelReply>;
  close(): Promise<void>;
}

/** CK_WASM wins; otherwise the release ck_wasm.wasm next to this repo. */
export function defaultWasmPath(): string {
  if (process.env.CK_WASM) return process.env.CK_WASM;
  return path.resolve(
    path.dirname(fileURLToPath(import.meta.url)),
    '../../../../target/wasm32-unknown-unknown/release/ck_wasm.wasm',
  );
}

/**
 * The compiled worker-host.js: beside this file in dist. When this module is
 * loaded from src (vitest aliasing), fall back to packages/dist. A plain path
 * string, not `new URL(..., import.meta.url)`: bundlers rewrite that pattern
 * to an http URL, which node:worker_threads refuses.
 */
export function hostScriptPath(): string {
  const dir = path.dirname(fileURLToPath(import.meta.url));
  const beside = path.join(dir, 'worker-host.js');
  if (existsSync(beside)) return beside;
  const fromSrc = path.resolve(dir, '../../dist/kernel-client/src/worker-host.js');
  if (existsSync(fromSrc)) return fromSrc;
  throw new Error(
    `worker-host.js not found beside ${dir} or at ${fromSrc}. Run npm run build in packages/.`,
  );
}

export async function createWorkerKernel(wasmPath = defaultWasmPath()): Promise<WorkerKernel> {
  if (!existsSync(wasmPath)) {
    throw new Error(
      `kernel wasm not found at ${wasmPath}. Build it (cargo build -p ck-wasm --target wasm32-unknown-unknown --release) ` +
      'or set CK_WASM / pass wasmPath to open.',
    );
  }
  const worker = new Worker(hostScriptPath());
  const pending = new Map<number, (r: KernelReply) => void>();
  let nextId = 1;

  const ready = await new Promise<{ commit: string }>((resolve, reject) => {
    worker.once('error', reject);
    worker.on('message', (m: HostResponse) => {
      if (m.kind === 'ready') resolve({ commit: m.commit });
      else if (m.kind === 'fail') reject(new Error(m.error));
      else if (m.kind === 'done') pending.get(m.id)?.(m.reply);
    });
    worker.postMessage({ kind: 'init', wasmPath });
  });

  return {
    wasmPath,
    commit: ready.commit,
    request(msg) {
      const id = nextId++;
      return new Promise<KernelReply>((resolve) => {
        pending.set(id, (r) => { pending.delete(id); resolve(r); });
        worker.postMessage({ kind: 'call', id, body: encodeRequest(id, msg) });
      });
    },
    async close() {
      await worker.terminate();
    },
  };
}
