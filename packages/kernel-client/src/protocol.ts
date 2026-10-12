// Binary reply envelope for the wasm kernel: [u32 json_len][json][pad to 8]
// [buffer 0][buffer 1]... Any JSON object shaped {$buf, type, len, offset}
// points at a bulk array. Mirrors tools/meshdump.mjs, except $buf payloads
// stay TypedArrays (structured-cloneable across the worker boundary) instead
// of expanding to plain arrays.

export type BufType = 'f32' | 'f64' | 'u32' | 'i32' | 'u8';

const CTORS = {
  f32: Float32Array,
  f64: Float64Array,
  u32: Uint32Array,
  i32: Int32Array,
  u8: Uint8Array,
} as const;

export type BufArray =
  | Float32Array | Float64Array | Uint32Array | Int32Array | Uint8Array;

/** A decoded bulk buffer: the $buf marker fields plus the typed data. */
export interface DecodedBuf {
  $buf: number;
  type: BufType;
  len: number;
  data: BufArray;
}

export function encodeRequest(id: number, msg: Record<string, unknown>): string {
  return JSON.stringify({ id, msg });
}

/** Split a raw reply into its JSON text and the decoded value ($bufs resolved). */
export function decodeReply(bytes: Uint8Array): { json: string; value: unknown } {
  const jl = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength).getUint32(0, true);
  const json = new TextDecoder().decode(bytes.subarray(4, 4 + jl));
  const base = (4 + jl + 7) & ~7;
  const dec = (v: unknown): unknown => {
    if (Array.isArray(v)) return v.map(dec);
    if (v && typeof v === 'object') {
      const o = v as Record<string, unknown>;
      if ('$buf' in o && typeof o.type === 'string' && o.type in CTORS) {
        const C = CTORS[o.type as BufType];
        const n = o.len as number;
        const data = new C(bytes.slice(base + (o.offset as number), base + (o.offset as number) + n * C.BYTES_PER_ELEMENT).buffer);
        return { $buf: o.$buf, type: o.type, len: n, data } as DecodedBuf;
      }
      return Object.fromEntries(Object.entries(o).map(([k, x]) => [k, dec(x)]));
    }
    return v;
  };
  return { json, value: dec(JSON.parse(json)) };
}

/** The kernel's envelope once parsed: ok/result or ok:false/error. */
export interface KernelReply {
  ok: boolean;
  result?: unknown;
  error?: string;
  trap?: string;
}

export function toKernelReply(parsed: { ok?: unknown; result?: unknown; error?: unknown }): KernelReply {
  if (parsed.ok) return { ok: true, result: parsed.result };
  return { ok: false, error: typeof parsed.error === 'string' ? parsed.error : 'unknown error' };
}

/** Messages between the main thread and the wasm worker host. */
export type HostRequest =
  | { kind: 'init'; wasmPath: string }
  | { kind: 'call'; id: number; body: string };

export type HostResponse =
  | { kind: 'ready'; commit: string }
  | { kind: 'fail'; error: string }
  | { kind: 'done'; id: number; reply: KernelReply };
