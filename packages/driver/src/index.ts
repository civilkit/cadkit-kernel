// Importable core of the repo's CLI driver: spec loading, STL encoding,
// result shaping.
// Browser launching stays with the caller (the CLI owns the Playwright
// launch flags via its external helper) — this package takes an EvalPage.

import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import type { PartDoc } from '../../part-model/src/index.js';
import { evalKernelInPage, type EvalPage, type KernelEvalResult } from '../../kernel-client/src/index.js';

export interface RunOpts {
  mass?: boolean | string;
  step?: boolean | string; // output path when string
  stl?: boolean | string; // output path when string
  png?: boolean | string;
  faces?: boolean | string;
  full?: boolean;
  view?: boolean | string;
}

export interface RunResult extends KernelEvalResult {
  step?: { file: string; bytes: number };
  stl?: { file: string; triangles: number };
  png?: string;
}

/** Load a part spec: JSON file, or a builder .mjs whose default export returns a Part. */
export async function loadSpec(file: string): Promise<PartDoc> {
  if (/\.(mjs|js)$/.test(file)) {
    const m = await import(pathToFileURL(path.resolve(file)).href + '?t=' + Date.now());
    const v = typeof m.default === 'function' ? await m.default() : m.default;
    return JSON.parse(JSON.stringify(v)) as PartDoc;
  }
  return JSON.parse(fs.readFileSync(file, 'utf8')) as PartDoc;
}

/** Binary STL from a Float32Array of xyz triples (3 vertices per triangle). */
export function stlBinary(positions: Float32Array, name?: string): Buffer {
  const n = Math.floor(positions.length / 9);
  const buf = Buffer.alloc(84 + n * 50);
  buf.write((name || 'part').slice(0, 79), 0);
  buf.writeUInt32LE(n, 80);
  for (let t = 0; t < n; t++) {
    const o = 84 + t * 50;
    const p = positions.slice(t * 9, t * 9 + 9);
    const u = [p[3] - p[0], p[4] - p[1], p[5] - p[2]];
    const v = [p[6] - p[0], p[7] - p[1], p[8] - p[2]];
    let nx = u[1] * v[2] - u[2] * v[1];
    let ny = u[2] * v[0] - u[0] * v[2];
    let nz = u[0] * v[1] - u[1] * v[0];
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l; ny /= l; nz /= l;
    [nx, ny, nz, ...p].forEach((x, i) => buf.writeFloatLE(x, o + i * 4));
  }
  return buf;
}

const round7 = (v: unknown): unknown =>
  typeof v === 'number' ? +v.toPrecision(7)
  : Array.isArray(v) ? v.map(round7)
  : v && typeof v === 'object'
    ? Object.fromEntries(Object.entries(v as Record<string, unknown>).map(([k, x]) => [k, round7(x)]))
    : v;

/**
 * Drive one regen (+ optional mass/STEP/mesh) through the page's kernel,
 * write step/stl files, and return the compact JSON result.
 * Screenshot (png) still lives in the CLI — it needs canvas picking +
 * setView/loadPartState orchestration that deserves the viewer-facade pass.
 */
export async function run(page: EvalPage, part: PartDoc, opts: RunOpts = {}): Promise<RunResult> {
  // evalKernelInPage is self-contained so Playwright can serialize it into the page.
  const r = await page.evaluate(evalKernelInPage, {
    part,
    mass: !!opts.mass,
    step: !!opts.step,
    stl: !!opts.stl,
    faces: !!opts.faces,
  }) as KernelEvalResult;
  const out = r as RunResult;
  if (r.stepText && opts.step && opts.step !== true) {
    fs.writeFileSync(opts.step, r.stepText);
    out.step = { file: opts.step, bytes: r.stepText.length };
  }
  if (r.positions && opts.stl && opts.stl !== true) {
    fs.writeFileSync(opts.stl, stlBinary(Float32Array.from(r.positions), part.name));
    out.stl = { file: opts.stl, triangles: r.triangles ?? 0 };
  }
  delete out.stepText;
  delete out.positions;
  if (out.mass && !opts.full) delete (out.mass as Record<string, unknown>).axes;
  for (const k of ['mass', 'box'] as const) {
    if (out[k]) (out as unknown as Record<string, unknown>)[k] = round7(out[k]);
  }
  return out;
}
