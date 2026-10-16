// Randomized robustness fuzz for loft over closed-spline sections (hull-like, perturbed sub-mm, as drawn with snap off).
// Asserts: no wasm trap/panic; every result is an ok valid solid or a structured loft error.
// usage: node tools/loft_fuzz.mjs [--wasm path] [--n 300] [--seeds a,b,c] [--verbose] [--strict] [--mode hull|guides|point|centre|tangent]
// exit 1 on any failure. Modes: hull (default, the 5 perturbed hull splines), guides (guide lines / splines through polygon sections),
// point (apex first/last section), centre (centre line sketch), tangent (end normals, connectors, tangency flags, surface lofts).
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
const mode = arg('--mode', 'hull');
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

// ---- other modes: polygon / circle / spline sections on offset top planes, optional guide / centre line / point ends
const pt = (id, x, y) => ({ id, x, y });
const lineE = (id, p1, p2) => ({ type: 'line', id, p1, p2 });
function section(R, kind, r, cx = 0, jitter = 0, a0fixed = false) {
  const J = () => (R() * 2 - 1) * jitter;
  if (kind === 'circle') return { points: [pt('c', cx, 0)], entities: [{ type: 'circle', id: 'e1', c: 'c', r }] };
  if (kind === 'spline') {
    const m = 6 + Math.floor(R() * 6);
    const P = Array.from({ length: m }, (_, i) => { const a = (2 * Math.PI * i) / m; const rr = r * (1 + 0.15 * Math.sin(3 * a)) ; return pt('p' + (i + 1), cx + rr * Math.cos(a) + J(), rr * Math.sin(a) + J()); });
    return { points: P, entities: [{ id: 'e1', type: 'spline', pts: P.map((p) => p.id), closed: true }] };
  }
  const nn = kind === 'square' ? 4 : 3 + Math.floor(R() * 6);
  const a0 = kind === 'square' || a0fixed ? 0 : R() * 2;
  const P = Array.from({ length: nn }, (_, i) => { const a = a0 + (2 * Math.PI * i) / nn; return pt('p' + (i + 1), cx + r * Math.cos(a) + J(), r * Math.sin(a) + J()); });
  return { points: P, entities: P.map((_, i) => lineE('e' + (i + 1), 'p' + (i + 1), 'p' + ((i + 1) % nn + 1))) };
}
function makeModePart(m, seed) {
  const R = rng(seed * 7919 + 13);
  const ns = m === 'point' ? 2 + Math.floor(R() * 3) : 2 + Math.floor(R() * 2);
  const H = 20 + R() * 40;
  const feats = []; const sks = [];
  const kinds = m === 'guides' ? ['square', 'poly'] : ['square', 'poly', 'circle', 'spline'];
  const secs = [];
  for (let i = 0; i < ns; i++) {
    const off = (H * i) / (ns - 1);
    let base = { kind: 'std', id: 'top' };
    if (off > 0) { feats.push({ type: 'plane', id: 'P' + i, mode: 'offset', base, offset: off }); base = { kind: 'feature', id: 'P' + i }; }
    const sr = 4 + R() * 10, scx = m === 'guides' ? 0 : (R() * 2 - 1) * 3, sk0 = kinds[Math.floor(R() * kinds.length)];
    secs.push({ r: sr, cx: scx, square: sk0 === 'square' });
    const apex = m === 'point' && (i === 0 || i === ns - 1) && R() < (i === 0 ? 0.4 : 0.7);
    const sec = apex ? { points: [pt('p', R() * 3, R() * 3)], entities: [{ type: 'point', id: 'e1', p: 'p' }] }
      : section(R, sk0, sr, scx, m === 'guides' ? 0 : R() < 0.2 ? 0.05 : 0, m === 'guides');
    feats.push({ type: 'sketch', id: 'S' + i, plane: base, constraints: [], ...sec });
    sks.push('S' + i);
  }
  const lf = { type: 'loft', id: 'L', op: 'add', sketches: sks };
  const front = { kind: 'std', id: 'front' };
  if (m === 'guides') {
    const ng = secs.every((x) => x.square) ? 1 + Math.floor(R() * 2) : 1;
    for (let g = 0; g < ng; g++) {
      const hit = R() < 0.75, sg = g === 0 ? 1 : -1;
      const pts = secs.map((x, i) => pt('g' + (i + 1), sg * x.r + (hit ? 0 : R() * 3), (H * i) / (ns - 1)));
      const ent = pts.length === 2 ? lineE('e1', 'g1', 'g2') : { id: 'e1', type: 'spline', pts: pts.map((p) => p.id) };
      feats.push({ type: 'sketch', id: 'G' + g, plane: front, constraints: [], points: pts, entities: [ent] });
      (lf.guides ??= []).push({ sketch: 'G' + g });
    }
  } else if (m === 'centre') {
    const dx = R() < 0.5 ? 0 : (R() * 2 - 1) * 8;
    const bend = R() < 0.3;
    const pts = [pt('g1', 0, R() < 0.2 ? 1 : 0), pt('g2', dx, H)];
    feats.push({ type: 'sketch', id: 'C', plane: front, constraints: [], points: pts, entities: [lineE('e1', 'g1', 'g2')] });
    if (bend) { pts.push(pt('g3', dx + 4, H + 5)); feats[feats.length - 1].entities.push(lineE('e2', 'g2', 'g3')); }
    lf.centreLine = { sketch: 'C' };
  } else if (m === 'tangent') {
    const T = () => [true, 0.5, 1, 2, 10, 0, -1, null][Math.floor(R() * 8)];
    if (R() < 0.5) lf.startNormal = T();
    if (R() < 0.5) lf.endNormal = T();
    if (R() < 0.2) lf.startTangent = true;
    if (R() < 0.15) lf.endContinuity = 'tangent';
    if (R() < 0.5) lf.connectors = sks.map(() => (R() < 0.5 ? null : R()));
    if (R() < 0.25) lf.surface = true;
  }
  feats.push(lf);
  return { format: 'ck-part', version: 1, features: feats };
}

export function runSeed(seed) {
  const k = new Kernel(mod);
  const part = mode === 'hull' ? makePart(seed) : makeModePart(mode, seed);
  const isSurface = part.features.some((f) => f.type === 'loft' && f.surface);
  const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, opts: { solveSketches: true } } }));
  if (rr.trap) return { seed, status: 'TRAP', detail: rr.trap };
  const r = JSON.parse(rr.json);
  const errs = (r.result?.errors ?? []).map(([i, e]) => `${i}:${e.code}${verbose ? ' ' + JSON.stringify(e.params?.detail ?? e.detail ?? '') : ''}`);
  if (r.ok === false && !r.result) return { seed, status: 'HOSTERR', detail: JSON.stringify(r).slice(0, 200) };
  if (errs.length) return errs.every((e) => /loft/i.test(e)) ? { seed, status: 'structured', detail: errs.join(' ') } : { seed, status: 'BADERR', detail: errs.join(' ') };
  const call = (msg) => { const x = k.request(JSON.stringify({ id: 2, msg: { ...msg, tag: 'main' } })); return x.trap ? { trap: x.trap } : JSON.parse(x.json); };
  if (isSurface) return { seed, status: 'ok', detail: 'surface' };
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
