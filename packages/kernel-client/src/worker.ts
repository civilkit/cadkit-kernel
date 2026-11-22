// Main-thread facade for the wasm worker host (./worker-host.js, compiled to
// dist next to this file). One WorkerKernel per instance: owns the worker,
// tags calls with numeric ids, resolves replies by id.
//
// No browser involved: the gate can boot this anywhere node runs.
import { existsSync, statSync } from 'node:fs';
import { Worker } from 'node:worker_threads';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { encodeRequest, type HostResponse, type KernelReply } from './protocol.js';
import { PendingCalls } from './pending.js';

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
 * loaded from src (vitest aliasing), fall back to packages/dist, but only if
 * it is not older than the sources it was built from: a stale host would
 * silently run old kernel glue. A plain path string, not
 * `new URL(..., import.meta.url)`: bundlers rewrite that pattern to an http
 * URL, which node:worker_threads refuses.
 */
export function hostScriptPath(): string {
  const dir = path.dirname(fileURLToPath(import.meta.url));
  const beside = path.join(dir, 'worker-host.js');
  if (existsSync(beside)) return beside;
  const packages = path.resolve(dir, '../..');
  const fromSrc = path.join(packages, 'dist/kernel-client/src/worker-host.js');
  const fix = `Run \`npm run build\` in ${packages} (tsc -p tsconfig.build.json), then retry.`;
  if (!existsSync(fromSrc)) {
    throw new Error(`worker-host.js not found beside ${dir} or at ${fromSrc}. ${fix}`);
  }
  const built = statSync(fromSrc).mtimeMs;
  for (const f of ['worker-host.ts', 'wasm-host.ts', 'protocol.ts']) {
    const src = path.join(dir, f);
    if (existsSync(src) && statSync(src).mtimeMs > built) {
      throw new Error(`${fromSrc} is older than ${f}, so it is stale. ${fix}`);
    }
  }
  return fromSrc;
}

export interface WorkerKernelOptions {
  /**
   * A call that takes longer than this many ms is answered with a trap and the
   * worker is terminated (a running wasm call cannot be interrupted), so the
   * kernel is dead afterwards. Default 0: no limit.
   */
  timeoutMs?: number;
}

export async function createWorkerKernel(wasmPath = defaultWasmPath(), opts: WorkerKernelOptions = {}): Promise<WorkerKernel> {
  if (!existsSync(wasmPath)) {
    throw new Error(
      `kernel wasm not found at ${wasmPath}. Build it (cargo build -p ck-wasm --target wasm32-unknown-unknown --release) ` +
      'or set CK_WASM / pass wasmPath to open.',
    );
  }
  const worker = new Worker(hostScriptPath());
  const pending = new PendingCalls();
  const timeoutMs = opts.timeoutMs ?? 0;

  const ready = await new Promise<{ commit: string }>((resolve, reject) => {
    worker.once('error', reject);
    worker.on('message', (m: HostResponse) => {
      if (m.kind === 'ready') resolve({ commit: m.commit });
      else if (m.kind === 'fail') reject(new Error(m.error));
      else if (m.kind === 'done') pending.settle(m.id, m.reply);
    });
    worker.postMessage({ kind: 'init', wasmPath });
  });

  // after startup: a crash, an exit or an unreadable message must not leave calls waiting forever
  worker.on('error', (e) => pending.fail(`kernel worker crashed: ${e?.message ?? e}`));
  worker.on('messageerror', (e) => pending.fail(`kernel worker sent an unreadable message: ${(e as Error)?.message ?? e}`));
  worker.on('exit', (code) => pending.fail(`kernel worker exited (code ${code})`));

  return {
    wasmPath,
    commit: ready.commit,
    request(msg) {
      const { id, reply } = pending.open();
      if (pending.dead !== null) return reply;
      let timer: ReturnType<typeof setTimeout> | null = null;
      if (timeoutMs > 0) {
        timer = setTimeout(() => {
          pending.fail(`kernel call timed out after ${Math.round(timeoutMs / 1000)} s; the worker was stopped`);
          void worker.terminate();
        }, timeoutMs);
        (timer as { unref?: () => void }).unref?.();
      }
      try {
        worker.postMessage({ kind: 'call', id, body: encodeRequest(id, msg) });
      } catch (e) {
        pending.fail(`kernel worker refused the call: ${(e as Error).message}`);
      }
      return timer ? reply.finally(() => clearTimeout(timer!)) : reply;
    },
    async close() {
      pending.fail('kernel closed');
      await worker.terminate();
    },
  };
}
