// Minimal host for a kernel wasm (a reference build or this one), mirroring
// kernelworker.js: env imports ck_now / ck_unix_ms / ck_panic, re-instantiate
// after a trap.
export class Kernel {
  constructor(mod, { now = () => 0, unixMs = () => 0 } = {}) {
    this.mod = mod;
    this.env = {
      ck_now: now, ck_unix_ms: unixMs,
      ck_panic: (p, l) => { this.panic = new TextDecoder().decode(new Uint8Array(this.x.memory.buffer, p, l)); },
    };
    this.boot();
  }
  boot() { this.x = new WebAssembly.Instance(this.mod, { env: this.env }).exports; }
  commit() { return new TextDecoder().decode(new Uint8Array(this.x.memory.buffer, this.x.ck_commit(), this.x.ck_commit_len())); }
  request(body) {
    const b = new TextEncoder().encode(body.replace(/"format"\s*:\s*"ck-part"/g, '"format":"formwerk-part"')); this.panic = null;   // the original kernel.wasm only accepts the format's old name; ours reads both
    try {
      const p = this.x.ck_alloc(b.length);
      new Uint8Array(this.x.memory.buffer, p, b.length).set(b);
      const total = this.x.ck_request(p, b.length), o = this.x.ck_reply();
      const bytes = new Uint8Array(this.x.memory.buffer.slice(o, o + total));
      const jl = new DataView(bytes.buffer).getUint32(0, true);
      return { total, bytes, json: new TextDecoder().decode(bytes.subarray(4, 4 + jl)) };
    } catch (e) { const r = { trap: `${e.message}${this.panic ? ' panic=' + this.panic : ''}`, json: null, total: 0 }; this.boot(); return r; }
  }
}
