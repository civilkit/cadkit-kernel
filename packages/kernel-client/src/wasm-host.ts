// wasm-host.ts: one wasm instance, call(body) -> KernelReply, reboots after a trap.
// No Node or DOM imports, so the Node worker and the browser worker share it.
import { decodeReply, toKernelReply, type KernelReply } from './protocol.js';

interface WasmExports {
  memory: WebAssembly.Memory;
  ck_alloc(n: number): number;
  ck_request(ptr: number, len: number): number;
  ck_reply(): number;
  ck_commit(): number;
  ck_commit_len(): number;
}

export class WasmHost {
  private inst!: WasmExports;
  private panic: string | null = null;
  commit = '';

  constructor(private readonly mod: WebAssembly.Module) {
    this.boot();
  }

  private boot(): void {
    const env = {
      ck_now: () => performance.now(),
      ck_unix_ms: () => Date.now(),
      ck_panic: (p: number, l: number) => {
        this.panic = new TextDecoder().decode(new Uint8Array(this.inst.memory.buffer, p, l));
      },
    };
    this.inst = new WebAssembly.Instance(this.mod, { env }).exports as unknown as WasmExports;
    this.commit = new TextDecoder().decode(
      new Uint8Array(this.inst.memory.buffer, this.inst.ck_commit(), this.inst.ck_commit_len()),
    );
  }

  /** One request; returns the raw framed reply bytes (what a native service sends over the wire). Throws on a trap (the instance is rebooted first). */
  callRaw(body: string): Uint8Array {
    const b = new TextEncoder().encode(body);
    this.panic = null;
    try {
      const x = this.inst;
      const p = x.ck_alloc(b.length);
      new Uint8Array(x.memory.buffer, p, b.length).set(b);
      const total = x.ck_request(p, b.length);
      const o = x.ck_reply();
      return new Uint8Array(x.memory.buffer).slice(o, o + total);
    } catch (e) {
      const trap = `${(e as Error).message}${this.panic ? ' panic=' + this.panic : ''}`;
      try { this.boot(); } catch { /* leave the old instance; next call traps again */ }
      throw new Error(trap);
    }
  }

  call(body: string): KernelReply {
    try {
      return toKernelReply(decodeReply(this.callRaw(body)).value as { ok?: unknown; result?: unknown; error?: unknown });
    } catch (e) {
      return { ok: false, trap: (e as Error).message };
    }
  }
}
