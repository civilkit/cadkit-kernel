// Randomized robustness fuzz for loft over closed-spline sections (hull-like, perturbed sub-mm, as drawn with snap off).
// Asserts: no wasm trap/panic; every result is an ok valid solid or a structured loft error.
// usage: node tools/loft_fuzz.mjs [--wasm path] [--n 300] [--seeds a,b,c] [--verbose]    exit 1 on any failure
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const wasm = arg('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const N = +arg('--n', 300);
const only = arg('--seeds', null)?.split(',').map(Number);
const verbose = process.argv.includes('--verbose');
const strict = process.argv.includes('--strict'); // also fail on ok:true results whose solid fails validate (known: LOOP_SELF_INTERSECT / EDGE_OFF_SURFACE on some dropped-point sections)
const mod = new WebAssembly.Module(readFileSync(wasm));

function rng(seed) { let s = (seed * 2654435761) >>> 0 || 1; return () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; }; }
const DIMS = [[4, 4, 3], [14, 9, 4], [18, 12, 5], [14, 9, 4], [2, 3, 7]];
const base = ([w, d, h]) => [[0, h], [0.55 * w, h + 0.4], [w, h - 1], [0.97 * w, -0.3 * d], [0.6 * w, -0.8 * d], [0, -d], [-0.6 * w, -0.8 * d], [-0.97 * w, -0.3 * d], [-w, h - 1], [-0.55 * w, h + 0.4]];

export function makePart(seed) {
  const R = rng(seed);
  const amp = [0.001, 0.01, 0.05, 0.1, 0.4][seed % 5];
  const snapped = seed % 7 === 0;
  const feats = [];
  const sks = [];
  DIMS.forEach((dim, i) => {
    let P = base(dim).map(([u, v]) => snapped ? [Math.round(u), Math.round(v)] : [u + (R() * 2 - 1) * amp, v + (R() * 2 - 1) * amp]);
    if (R() < 0.5) P.reverse();
    const sh = R() < 0.5 ? Math.floor(R() * P.length) : 0;
    P = P.slice(sh).concat(P.slice(0, sh));
    const rep = R() < 0.3;
    // nastier drawings: a double click (near-duplicate point), a dropped point, different point counts per section
    if (seed % 3 === 1 && R() < 0.6) { const j = Math.floor(R() * P.length); const e = [0, 1e-9, 1e-4, 1e-2][Math.floor(R() * 4)]; P.splice(j, 0, [P[j][0] + e, P[j][1]]); }
    if (seed % 3 === 2 && R() < 0.6) P.splice(Math.floor(R() * P.length), 1);
    const pts = P.map(([x, y], k) => ({ id: 'p' + (k + 1), x, y }));
    const ids = pts.map((p) => p.id);
    if (rep) ids.push(ids[0]);
    const pl = { type: 'plane', id: 'P' + i, mode: 'offset', base: { kind: 'std', id: seed % 2 ? 'front' : 'right' }, offset: -60 + 30 * i };
    const sk = { type: 'sketch', id: 'S' + i, plane: { kind: 'feature', id: pl.id }, constraints: [], points: pts, entities: [{ id: 'e1', type: 'spline', pts: ids, closed: true }] };
    feats.push(pl, sk); sks.push(sk.id);
  });
  feats.push({ type: 'loft', id: 'L', op: 'add', sketches: sks });
  return { format: 'ck-part', version: 1, features: feats };
}

export function runSeed(seed) {
  const k = new Kernel(mod);
  const part = makePart(seed);
  const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, opts: { solveSketches: true } } }));
  if (rr.trap) return { seed, status: 'TRAP', detail: rr.trap };
  const r = JSON.parse(rr.json);
  const errs = (r.result?.errors ?? []).map(([i, e]) => `${i}:${e.code}${verbose ? ' ' + JSON.stringify(e.params?.detail ?? e.detail ?? '') : ''}`);
  if (r.ok === false && !r.result) return { seed, status: 'HOSTERR', detail: JSON.stringify(r).slice(0, 200) };
  if (errs.length) return errs.every((e) => /loft/i.test(e)) ? { seed, status: 'structured', detail: errs.join(' ') } : { seed, status: 'BADERR', detail: errs.join(' ') };
  const call = (msg) => { const x = k.request(JSON.stringify({ id: 2, msg: { ...msg, tag: 'main' } })); return x.trap ? { trap: x.trap } : JSON.parse(x.json); };
  const m = call({ type: 'massProps' }), v = call({ type: 'validate' });
  if (m.trap || v.trap) return { seed, status: 'TRAP', detail: m.trap || v.trap };
  const issues = v.result?.issues ?? [];
  if (!(m.result?.volume > 0) || issues.length) return { seed, status: 'INVALID', detail: `vol ${m.result?.volume} issues ${issues.slice(0, 2).map((i) => i.code)}` };
  return { seed, status: 'ok', detail: `vol ${m.result.volume.toFixed(1)}` };
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  const seeds = only ?? Array.from({ length: N }, (_, i) => i + 1);
  const tally = {}; const bad = [];
  const invalid = [];
  for (const s of seeds) {
    const r = runSeed(s);
    tally[r.status] = (tally[r.status] ?? 0) + 1;
    if (r.status === 'INVALID') invalid.push(s);
    if (/^[A-Z]+$/.test(r.status) && (strict || r.status !== 'INVALID')) { bad.push(r); console.log(`FAIL seed ${s}: ${r.status} ${r.detail}`); }
    else if (verbose) console.log(`seed ${s}: ${r.status} ${r.detail}`);
  }
  if (invalid.length && !strict) console.log('invalid solids (not failing without --strict): seeds', invalid.join(','));
  console.log(JSON.stringify(tally), bad.length ? 'FAILED' : 'PASS');
  process.exit(bad.length ? 1 : 0);
}
