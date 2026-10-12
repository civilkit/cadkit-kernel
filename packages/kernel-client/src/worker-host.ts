// Runs inside node:worker_threads. Owns one wasm instance and answers
// {kind:'call', id, body} with {kind:'done', id, reply}. Reboots the instance
// after a trap, mirroring tools/kernel.mjs.
import { parentPort } from 'node:worker_threads';
import { readFileSync } from 'node:fs';
import { decodeReply, toKernelReply, type HostRequest, type HostResponse } from './protocol.js';

interface WasmExports {
  memory: WebAssembly.Memory;
  ck_alloc(n: number): number;
  ck_request(ptr: number, len: number): number;
  ck_reply(): number;
  ck_commit(): number;
  ck_commit_len(): number;
}

const post = (m: HostResponse) => parentPort!.postMessage(m);

let inst: WasmExports | null = null;
let panic: string | null = null;

function boot(wasmPath: string): string {
  const mod = new WebAssembly.Module(readFileSync(wasmPath));
  const env = {
    ck_now: () => performance.now(),
    ck_unix_ms: () => Date.now(),
    ck_panic: (p: number, l: number) => {
      panic = new TextDecoder().decode(new Uint8Array(inst!.memory.buffer, p, l));
    },
  };
  inst = new WebAssembly.Instance(mod, { env }).exports as unknown as WasmExports;
  return new TextDecoder().decode(new Uint8Array(inst.memory.buffer, inst.ck_commit(), inst.ck_commit_len()));
}

function call(body: string) {
  const b = new TextEncoder().encode(body);
  panic = null;
  try {
    const x = inst!;
    const p = x.ck_alloc(b.length);
    new Uint8Array(x.memory.buffer, p, b.length).set(b);
    const total = x.ck_request(p, b.length);
    const o = x.ck_reply();
    const bytes = new Uint8Array(x.memory.buffer).slice(o, o + total);
    const { value } = decodeReply(bytes);
    const r = value as { ok?: unknown; result?: unknown; error?: unknown };
    return toKernelReply(r);
  } catch (e) {
    const trap = `${(e as Error).message}${panic ? ' panic=' + panic : ''}`;
    try { boot(currentWasm); } catch { inst = null; }
    return { ok: false, trap };
  }
}

let currentWasm = '';

parentPort!.on('message', (m: HostRequest) => {
  if (m.kind === 'init') {
    currentWasm = m.wasmPath;
    try {
      post({ kind: 'ready', commit: boot(m.wasmPath) });
    } catch (e) {
      post({ kind: 'fail', error: String((e as Error).message ?? e) });
    }
    return;
  }
  if (inst === null) {
    post({ kind: 'done', id: m.id, reply: { ok: false, trap: 'worker has no instance' } });
    return;
  }
  post({ kind: 'done', id: m.id, reply: call(m.body) });
});
