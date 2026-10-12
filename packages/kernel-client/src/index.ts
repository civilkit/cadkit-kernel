// Kernel wire-protocol types — the page-exposed `kernel.send({type, …}, cb)`
// channel used by the bundled web app (app/v/<hash>/app.js), as driven
// headlessly by the repo's CLI driver.
// Covers only the seams the driver uses today (regen / massProps / step +
// mesh positions). The app also sends measure, inspect, interference, drawing,
// curvatureMap, curvatureAt, convert, matchEdges, drag, comb (see README) —
// add them here as they get wrapped, one message type at a time.
//
// NOTE: the page-global name accessed below is an opaque wire-protocol fact
// of the bundled app — not a dependency on any brand.

import type { PartDoc } from '../../part-model/src/index.js';

/** Outgoing kernel messages (subset). `tag` is the app's dedup/replace key. */
export type KernelRequest =
  | { type: 'regen'; tag: string; part: PartDoc; opts?: unknown }
  | { type: 'massProps'; tag: string; bodies?: boolean; densities?: number[] }
  | { type: 'step'; tag: string; name: string };

export type KernelErrorRef = [featureId: string, err: { code: string; params: Record<string, unknown> }];

export interface RegenBody {
  empty?: boolean;
  box?: { min: number[]; max: number[] };
  faces?: Array<Record<string, unknown>>;
  edges?: Array<Record<string, unknown>>;
  mesh?: { positions: ArrayLike<number>; normals?: ArrayLike<number>; triFace?: ArrayLike<number> };
}

export interface RegenReply {
  errors?: KernelErrorRef[];
  body?: RegenBody;
}

export interface MassProps {
  volume?: number;
  area?: number;
  com?: number[];
  moments?: number[];
  axes?: unknown;
  [key: string]: unknown;
}

export interface KernelEvalResult {
  ms: number;
  errors: KernelErrorRef[];
  ok: boolean;
  box?: { min: number[]; max: number[] };
  faces?: number;
  edges?: number;
  triangles?: number;
  faceList?: Array<Record<string, unknown>>;
  mass?: MassProps;
  /** raw STEP text — present only when requested; stripped before returning */
  stepText?: string;
  /** raw mesh positions — present only when requested; stripped before returning */
  positions?: number[];
}

/** Minimal page surface the driver needs (Playwright Page subset). */
export interface EvalPage {
  evaluate<R, A>(fn: (arg: A) => R | Promise<R>, arg: A): Promise<R>;
}

/** The exact payload sent into page.evaluate — kept here so the browser-side
 *  closure and its typing live next to the protocol, not in the CLI. */
export interface EvalArgs {
  part: PartDoc;
  mass: boolean;
  step: boolean;
  stl: boolean;
  faces: boolean;
}

/** Browser-side kernel round-trip. Runs INSIDE the page via
 *  `page.evaluate(evalKernelInPage, args)` — so it takes a single arg and
 *  reads the page-exposed kernel handle from globalThis. Must stay
 *  self-contained (no outer imports) for Playwright function serialization. */
export async function evalKernelInPage(
  { part, mass, step, stl, faces }: EvalArgs,
): Promise<KernelEvalResult> {
  const a = (globalThis as unknown as { window: { cadkit: { kernel: { send(msg: Record<string, unknown>, cb: (v: never) => void): void } } } }).window.cadkit;
  const send = <T>(m: Record<string, unknown>, ms = 20000): Promise<T> =>
    new Promise((res, rej) => {
      const t = setTimeout(() => rej(new Error('kernel timeout: ' + (m as { type?: string }).type)), ms);
      a.kernel.send(m, (v: T) => { clearTimeout(t); res(v); });
    });
  const out: KernelEvalResult = { ms: 0, errors: [], ok: false };
  const t0 = performance.now();
  const rg = await send<RegenReply>({ type: 'regen', tag: 'drv', part });
  out.ms = Math.round(performance.now() - t0);
  out.errors = rg.errors ?? [];
  const b = rg.body;
  out.ok = !!b && !b.empty && out.errors.length === 0;
  if (b) {
    out.box = b.box as KernelEvalResult['box'];
    out.faces = b.faces?.length;
    out.edges = b.edges?.length;
    out.triangles = b.mesh ? b.mesh.positions.length / 9 : 0;
  }
  if (faces && b) {
    out.faceList = (b.faces ?? []).slice(0, 60).map((f) => {
      const { mesh: _mesh, ...rest } = f as Record<string, unknown>;
      return rest;
    });
  }
  if (b && !b.empty) {
    if (mass) out.mass = await send<MassProps>({ type: 'massProps', tag: 'drv' });
    if (step) out.stepText = await send<string>({ type: 'step', tag: 'drv', name: part.name || 'part' });
    if (stl) out.positions = Array.from(b.mesh!.positions as ArrayLike<number> as number[]);
  }
  return out;
}
