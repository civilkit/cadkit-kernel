// f3d-decoder.ts: the lazily loaded Fusion 360 decoder (docs/f3d-lazy.md).
//
// The main kernel wasm does not contain the .f3d decoder. It is a second wasm (`ck_f3d.wasm`, crate ck-wasm-f3d) with the
// kernel's own ABI, so WasmHost hosts it unchanged. No Node or DOM imports: the browser app and Node tests share this file.
//
//   configureF3dDecoder(url)            once, at startup: where the second wasm is served from (nothing is fetched yet)
//   const dec = await loadF3dDecoder()  on the first .f3d: fetches and compiles it once, later calls return the same decoder
//   dec.decode(bytes, {name, opts})     -> { decoded } for the main kernel's `importF3d` (field `decoded`), or { errors }
//   dec.dump(bytes, {name})             -> the design dump (JSON) per design + the measure of its imported body; the MAIN kernel's
//                                          `f3dFeatures` maps it to a ck-part feature list (it needs the kernel's regen)
//
// A missing or unloadable decoder is an `F3dDecoderMissingError` (code 'f3d.decoderMissing') naming what was tried.
import { WasmHost } from './wasm-host.js';
import { encodeRequest } from './protocol.js';

export const F3D_DECODER_MISSING = 'f3d.decoderMissing';

export class F3dDecoderMissingError extends Error {
  readonly code = F3D_DECODER_MISSING;
  constructor(detail: string) {
    super(`The Fusion 360 (.f3d) decoder is not available: ${detail}. It is a separate download (ck_f3d.wasm) that the app loads the first time a .f3d is opened.`);
    this.name = 'F3dDecoderMissingError';
  }
}

/** Where the decoder wasm comes from: a URL (fetched), its bytes, a compiled module, or a function that supplies one of those. */
export type F3dDecoderSource =
  | string
  | URL
  | ArrayBuffer
  | Uint8Array
  | WebAssembly.Module
  | (() => Promise<string | URL | ArrayBuffer | Uint8Array | WebAssembly.Module | Response>);

/** The `import` object of an `importF3d` reply (docs/f3d.md): `format`, `report[]`, `parts[]`, ... */
export type F3dImportInfo = Record<string, unknown>;

/** What `decode` hands to the main kernel: base64 of the lossless body form (ck-topo codec) of the solids and of each surface body. */
export interface F3dDecodedBodies {
  solid: string;
  surfaces: Array<[id: string, body: string]>;
  info: F3dImportInfo;
}

export type F3dImportError = [feature: string, err: { code: string; params: { message: string } }];

export type F3dDecodeResult = { decoded: F3dDecodedBodies; errors?: undefined } | { decoded?: undefined; errors: F3dImportError[] };

/** One design of `dump`: the dump IR as JSON (null without a timeline) and the volume / box of the body the import builds for it. */
export interface F3dDumpDesign {
  label: string;
  dump: Record<string, unknown> | null;
  reference: { volume: number; min: [number, number, number]; max: [number, number, number]; solids: number } | null;
  referenceError: string | null;
}

export type F3dDumpResult = { designs: F3dDumpDesign[]; errors?: undefined } | { designs?: undefined; errors: F3dImportError[] };

/** One design of the main kernel's `f3dFeatures` reply. */
export interface F3dFeatureDesign {
  label: string;
  /** the ck-part document (`{format:'ck-part', version:1, features, seq}`) */
  part: Record<string, unknown>;
  mapped: number;
  placeholders: number;
  items: Array<{ index: number; name: string; type: string; status: 'mapped' | 'placeholder'; features: string[]; note?: string }>;
  notes: Array<{ item: number | null; text: string }>;
  /**
   * the regenerate-and-compare verdict of the main kernel against the import's `reference` (volume within 2e-3 relative, box sides
   * within 0.01 mm + 1e-4 of the diagonal): the history is only the design's when it is `verified`
   */
  check: {
    verdict: 'verified' | 'unverified';
    reference: unknown;
    history: unknown;
    volumeDelta: number | null;
    boxDelta: number | null;
    reason: string | null;
    errors: string[];
  };
}

export type F3dFeaturesResult = { designs: F3dFeatureDesign[]; errors?: undefined } | { designs?: undefined; errors: F3dImportError[] };

export interface F3dDecoder {
  /** the decoder wasm's build id (`ck <revision>`), to compare with the kernel's */
  readonly commit: string;
  /** whether this build also answers `dump` (the design decoder is in it) */
  readonly hasFeatures: boolean;
  decode(bytes: Uint8Array, o?: { name?: string; history?: boolean; allBodies?: boolean }): F3dDecodeResult;
  dump(bytes: Uint8Array, o?: { name?: string }): F3dDumpResult;
}

export function bytesToBase64(bytes: Uint8Array): string {
  let s = '';
  const step = 0x8000;
  for (let i = 0; i < bytes.length; i += step) s += String.fromCharCode(...bytes.subarray(i, i + step));
  return btoa(s);
}

class Decoder implements F3dDecoder {
  private n = 0;
  readonly commit: string;
  readonly hasFeatures: boolean;
  constructor(private readonly host: WasmHost) {
    this.commit = host.commit;
    this.hasFeatures = (this.run({ type: 'ping' }) as { features?: boolean }).features === true;
  }
  private run(msg: Record<string, unknown>): unknown {
    const r = this.host.call(encodeRequest(++this.n, msg));
    if (r.trap) throw new Error(`f3d decoder trap: ${r.trap}`);
    if (!r.ok) throw new Error(`f3d decoder: ${r.error ?? 'request failed'}`);
    return r.result;
  }
  decode(bytes: Uint8Array, o: { name?: string; history?: boolean; allBodies?: boolean } = {}): F3dDecodeResult {
    const msg: Record<string, unknown> = { type: 'decodeF3d', name: o.name ?? 'import', data: bytesToBase64(bytes) };
    if (o.history !== undefined || o.allBodies !== undefined) msg.opts = { history: o.history, allBodies: o.allBodies };
    return this.run(msg) as F3dDecodeResult;
  }
  dump(bytes: Uint8Array, o: { name?: string } = {}): F3dDumpResult {
    if (!this.hasFeatures) throw new F3dDecoderMissingError('this ck_f3d.wasm was built without the design (f3dDump) request');
    return this.run({ type: 'f3dDump', name: o.name ?? 'import', data: bytesToBase64(bytes) }) as F3dDumpResult;
  }
}

let source: F3dDecoderSource | null = null;
let loading: Promise<F3dDecoder> | null = null;

/** Where the decoder wasm is served from. Nothing is fetched until `loadF3dDecoder`. Changing it drops a failed or loaded decoder. */
export function configureF3dDecoder(src: F3dDecoderSource | null): void {
  source = src;
  loading = null;
}

async function toModule(src: F3dDecoderSource): Promise<WebAssembly.Module> {
  let v: unknown = typeof src === 'function' ? await src() : src;
  if (v instanceof WebAssembly.Module) return v;
  if (typeof v === 'string' || v instanceof URL) {
    const url = String(v);
    let res: Response;
    try {
      res = await fetch(url);
    } catch (e) {
      throw new F3dDecoderMissingError(`fetching ${url} failed (${(e as Error).message})`);
    }
    if (!res.ok) throw new F3dDecoderMissingError(`fetching ${url} answered ${res.status}`);
    v = res;
  }
  if (typeof Response !== 'undefined' && v instanceof Response) {
    if (!v.ok) throw new F3dDecoderMissingError(`the decoder answered ${v.status}`);
    v = await v.arrayBuffer();
  }
  return WebAssembly.compile(v as BufferSource);
}

/**
 * The decoder: fetched and compiled on the first call (from `src`, else the configured source), then cached. A failure is not
 * cached, so a retry after the network is back works. Throws F3dDecoderMissingError when there is no source or it cannot be loaded.
 */
export function loadF3dDecoder(src?: F3dDecoderSource): Promise<F3dDecoder> {
  if (src !== undefined) configureF3dDecoder(src);
  if (loading) return loading;
  const s = source;
  if (s === null) return Promise.reject(new F3dDecoderMissingError('no decoder source configured (configureF3dDecoder)'));
  const p = toModule(s).then(
    (m) => {
      try {
        return new Decoder(new WasmHost(m));
      } catch (e) {
        throw new F3dDecoderMissingError(`the module is not a usable decoder (${(e as Error).message})`);
      }
    },
    (e) => {
      throw e instanceof F3dDecoderMissingError ? e : new F3dDecoderMissingError(`compiling failed (${(e as Error).message})`);
    },
  );
  loading = p;
  p.catch(() => {
    if (loading === p) loading = null;
  });
  return p;
}
