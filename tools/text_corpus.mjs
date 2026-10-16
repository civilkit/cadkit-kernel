// Lettering corpus: text entities engraved into / embossed on plates, built through the wasm.
// Each case: a plate (box or disk, optionally with a hole), one sketch with text entities on a plane, an
// extrude cut (engrave) or add (emboss); checks: no error / expected error code, expected warning code,
// no booleanFailed, validate (with mesh:true) clean, volume = plate -/+ glyph area x depth (the glyph area
// comes from the `textOutline` request, a shoelace over the returned loops), regen time bound.
// usage: node tools/text_corpus.mjs [--wasm path] [--only substr] [--write dir] [--quiet]
//   --write dir: also writes every case as <dir>/<n>.ckpart (JSON) for the kit renderer.
// exit 1 when any case fails.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const arg = (n) => { const i = process.argv.indexOf(n); return i > 0 ? process.argv[i + 1] : null; };
const wasm = arg('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const only = arg('--only');
const outDir = arg('--write');
const mod = new WebAssembly.Module(readFileSync(wasm));
const rqk = (k, msg) => { const r = k.request(JSON.stringify({ id: 1, msg })); if (r.trap) return { trap: r.trap }; try { return JSON.parse(r.json); } catch { return { trap: 'unparsable reply' }; } };

const T = (string, o = {}) => ({ string, font: 'gauge', height: 6, at: [50, 30], anchor: ['center', 'middle'], ...o });
const cases = [];
const C = (name, o) => cases.push({ name, plane: 'front', plate: { kind: 'box', w: 100, h: 60, t: 5 }, depth: 0.2, op: 'cut', ...o });

// ---- the gauge texts
C('RPM x100', { texts: [T('RPM x100', { height: 8 })] });
C('KNOTS engrave font', { texts: [T('KNOTS', { font: 'engrave', height: 10 })] });
C('OIL PSI', { texts: [T('OIL PSI', { height: 9 })] });
C('TEMP degF', { texts: [T('TEMP °F', { height: 9 })] });
C('digits 0..9', { texts: [T('0123456789', { height: 8 })] });
C('digits 0..9 engrave font', { texts: [T('0123456789', { height: 8, font: 'engrave' })] });
C('nameplate part number', { texts: [T('PN 4711-B/22 REV.C', { height: 6 })] });
// ---- glyphs with holes and islands
for (const g of ['O', '8', 'B', 'A', 'Q', 'R', 'D', 'P', '6', '9', '0', '%', '&', '@', '°', 'i', 'j'])
  C(`glyph ${g}`, { texts: [T(g, { height: 30 })], depth: 0.5 });
for (const g of ['O', 'B', '8', '%', 'Q'])
  C(`glyph ${g} engrave font`, { texts: [T(g, { height: 30, font: 'engrave' })], depth: 0.5 });
C('accented', { texts: [T('ÀÉÎÖüñ', { height: 12 })] });
C('two lines in one sketch', { texts: [T('MAX', { at: [50, 40], height: 10 }), T('0.5 BAR', { at: [50, 20], height: 6 })] });
// ---- placement
C('angle 30', { texts: [T('ANGLE 30', { height: 8, angle: 30 })] });
C('angle 90 anchor right top', { texts: [T('UP', { height: 10, angle: 90, at: [90, 50], anchor: ['right', 'top'] })] });
C('mirror', { texts: [T('MIRROR', { height: 10, mirror: true })] });
C('spacing 1.4', { texts: [T('WIDE', { height: 10, spacing: 1.4 })] });
C('polyline mode', { texts: [T('FLAT 85', { height: 10, polyline: true })] });
for (const pl of ['top', 'right']) C(`plane ${pl}`, { plane: pl, texts: [T('PLANE', { height: 10, at: [50, 30] })] });
// ---- dial: numerals 0..80 on a circle, arc text
const dial = (extra = {}) => ({ plate: { kind: 'disk', r: 50, t: 3 }, ...extra });
const numerals = [];
for (let i = 0; i <= 8; i++) {
  const a = (225 - i * 33.75) * Math.PI / 180; // 270 degree sweep, 0 at lower left
  numerals.push(T(String(i * 10), { at: [38 * Math.cos(a), 38 * Math.sin(a)], height: 5 }));
}
C('dial numerals 0..80', dial({ texts: numerals, depth: 0.2 }));
C('dial numerals 0..80 emboss', dial({ texts: numerals, depth: 0.3, op: 'add' }));
C('dial KNOTS outside arc', dial({ texts: [T('KNOTS', { height: 5, at: [0, 0], anchor: ['center', 'baseline'], arc: { centre: [0, 0], radius: 28, angleCentre: 90, side: 'outside' } })] }));
C('dial TEMP inside arc', dial({ texts: [T('OIL PSI', { height: 5, at: [0, 0], anchor: ['center', 'baseline'], arc: { centre: [0, 0], radius: 24, angleCentre: 270, side: 'inside' } })] }));
C('dial numerals + RPM + arc', dial({ texts: [...numerals, T('RPM x100', { height: 4, at: [0, -15] }), T('ENGINE', { height: 4, at: [0, 0], anchor: ['center', 'baseline'], arc: { centre: [0, 0], radius: 14, angleCentre: 90, side: 'outside' } })] }));
// ---- booleans on awkward neighbours
C('text over a through hole', { plate: { kind: 'box', w: 100, h: 60, t: 5, hole: [50, 30, 6] }, texts: [T('HOLE', { height: 20 })], area: 'bounds' });
C('text past the plate edge', { texts: [T('EDGE RUN', { height: 14, at: [95, 30], anchor: ['center', 'middle'] })], area: 'bounds' });
C('twenty characters 0.2 deep', { texts: [T('OIL PSI TEMP 0123456', { height: 6 })], maxMs: 2000 });
C('emboss text', { op: 'add', depth: 1, texts: [T('RAISED', { height: 12 })] });
C('emboss digits', { op: 'add', depth: 0.5, texts: [T('0123456789', { height: 8 })] });
// ---- every printable character of both fonts (8 per chunk, spacing 1.15 so touching letters are not what is tested)
{
  const chars = []; for (let c = 0x21; c < 0x7f; c++) chars.push(String.fromCharCode(c));
  for (const c of [0xb0, 0xb1, 0xd7, 0xb7, 0xb5, 0xa7, 0xa9, 0xc5, 0xdf, 0xe9, 0xf8]) chars.push(String.fromCharCode(c));
  for (const font of ['gauge', 'engrave'])
    for (let i = 0; i < chars.length; i += 8)
      C(`sweep ${font} ${chars.slice(i, i + 8).join('')}`, { plate: { kind: 'box', w: 100, h: 30, t: 5 }, texts: [T(chars.slice(i, i + 8).join(''), { font, height: 10, at: [50, 15], spacing: 1.15 })], depth: 0.4 });
}
// ---- errors and warnings
C('empty string', { texts: [T('')], err: 'textEmpty' });
C('blank string', { texts: [T('   ')], err: 'textNothingToDraw' });
C('unknown font', { texts: [T('A', { font: 'comic' })], err: 'textBadFont' });
C('tiny text warns', { texts: [T('AB', { height: 0.5 })], warn: 'textTooSmall', noBoolean: true });
C('missing glyph warns', { texts: [T('A日B', { height: 8 })], warn: 'textMissingGlyph', area: 'any' });
C('long string capped', { texts: [T('A'.repeat(600), { height: 1 })], warn: 'textTruncated', noBoolean: true });

// ---------------------------------------------------------------- build
function part(c) {
  const f = [];
  const pl = { kind: 'std', id: c.plane };
  const P = c.plate;
  const sk = { type: 'sketch', id: 'S1', plane: pl, points: [], entities: [], constraints: [] };
  if (P.kind === 'box') {
    const pts = [[0, 0], [P.w, 0], [P.w, P.h], [0, P.h]];
    sk.points = pts.map(([x, y], i) => ({ id: 'p' + i, x, y }));
    sk.entities = pts.map((_, i) => ({ type: 'line', id: 'e' + (i + 1), p1: 'p' + i, p2: 'p' + ((i + 1) % 4) }));
  } else {
    sk.points = [{ id: 'c', x: 0, y: 0 }];
    sk.entities = [{ type: 'circle', id: 'e1', c: 'c', r: P.r }];
  }
  f.push(sk);
  f.push({ type: 'extrude', id: 'F1', op: 'add', sketch: 'S1', end: 'blind', depth: P.t });
  f.push({ type: 'plane', id: 'P1', mode: 'offset', base: pl, offset: P.t });
  if (P.hole) {
    f.push({ type: 'sketch', id: 'SH', plane: { kind: 'feature', id: 'P1' }, points: [{ id: 'h', x: P.hole[0], y: P.hole[1] }], entities: [{ type: 'circle', id: 'h1', c: 'h', r: P.hole[2] }], constraints: [] });
    f.push({ type: 'extrude', id: 'FH', op: 'cut', sketch: 'SH', end: 'blind', depth: P.t + 1 });
  }
  f.push({ type: 'sketch', id: 'S2', plane: { kind: 'feature', id: 'P1' }, points: [], entities: c.texts.map((t, i) => ({ type: 'text', id: 't' + (i + 1), ...t })), constraints: [] });
  if (!c.noBoolean) f.push({ type: 'extrude', id: 'F2', op: c.op, sketch: 'S2', end: 'blind', depth: c.depth });
  return { format: 'ck-part', version: 1, features: f };
}

const plateVolume = (P) => (P.kind === 'box' ? P.w * P.h * P.t : Math.PI * P.r * P.r * P.t) - (P.hole ? Math.PI * P.hole[2] ** 2 * P.t : 0);

// mesh edges used by exactly one triangle (0 = watertight display mesh)
function openMeshEdges(rr, r) {
  const mesh = r.result?.body?.mesh;
  if (!mesh || !rr.bytes) return -1;
  const jl = new DataView(rr.bytes.buffer, rr.bytes.byteOffset).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const v = mesh.positions ?? mesh.pos;
  const pos = new Float32Array(rr.bytes.buffer.slice(rr.bytes.byteOffset + base + v.offset, rr.bytes.byteOffset + base + v.offset + v.len * 4));
  const key = (i) => [0, 1, 2].map((q) => Math.round(pos[i * 3 + q] / 1e-4)).join(',');
  const uses = new Map();
  for (let t = 0; t < pos.length / 9; t++) {
    const vs = [3 * t, 3 * t + 1, 3 * t + 2].map(key);
    for (let e = 0; e < 3; e++) { const a = vs[e], b = vs[(e + 1) % 3]; if (a === b) continue; const kk = a < b ? a + '|' + b : b + '|' + a; uses.set(kk, (uses.get(kk) ?? 0) + 1); }
  }
  return [...uses.values()].filter((x) => x === 1).length;
}

// shoelace over the polylines of a textOutline reply
const shoelace = (loop) => { let s = 0; for (let i = 0; i < loop.length; i++) { const a = loop[i], b = loop[(i + 1) % loop.length]; s += a[0] * b[1] - b[0] * a[1]; } return s / 2; };
function glyphArea(k, texts) {
  let a = 0;
  for (const t of texts) {
    const o = rqk(k, { type: 'textOutline', ...t });
    if (!o.ok) return null;
    for (const r of o.result.regions) a += shoelace(r.outer) + r.holes.reduce((s, h) => s + shoelace(h), 0);
  }
  return a;
}

function run(c, idx) {
  const k = new Kernel(mod);
  const p = part(c);
  if (outDir) { mkdirSync(outDir, { recursive: true }); writeFileSync(path.join(outDir, `${String(idx).padStart(2, '0')}-${c.name.replace(/[^A-Za-z0-9]+/g, '_')}.ckpart`), JSON.stringify(p)); }
  const t0 = Date.now();
  const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: p } }));
  const ms = Date.now() - t0;
  if (rr.trap) return { ok: false, detail: 'trap ' + rr.trap, ms };
  const r = JSON.parse(rr.json);
  const errs = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code }));
  const warns = (r.result?.warnings ?? []).map(([id, w]) => ({ id, code: w.code, detail: w.params?.detail }));
  if (c.err) return errs.some((e) => e.code === c.err) ? { ok: true, detail: `error ${c.err}`, ms } : { ok: false, detail: `want error ${c.err}, got ${JSON.stringify(errs)}`, ms };
  if (errs.length) return { ok: false, detail: 'errors ' + JSON.stringify(errs), ms };
  if (c.warn && !warns.some((w) => w.code === c.warn)) return { ok: false, detail: `want warning ${c.warn}, got ${JSON.stringify(warns)}`, ms };
  if (c.noBoolean) {
    const sk = r.result.sketches?.find((s) => s[0] === 'S2');
    return { ok: true, detail: `${c.warn ?? 'sketch'} ok` + (sk ? '' : ''), ms };
  }
  const m = rqk(k, { type: 'massProps', tag: 'main' });
  const vol = m.result?.volume;
  if (!(vol > 0)) return { ok: false, detail: 'no body', ms };
  const v = rqk(k, { type: 'validate', tag: 'main', mesh: true });
  const issues = v.result?.issues ?? [];
  if (issues.length) return { ok: false, detail: 'validate ' + [...new Set(issues.map((i) => i.code))].join(','), ms };
  const open = openMeshEdges(rr, r);
  if (open > 0) return { ok: false, detail: `${open} open mesh edges`, ms };
  const base = plateVolume(c.plate);
  const area = glyphArea(k, c.texts);
  if (c.area == null && c.texts.some((t) => { const o = rqk(k, { type: 'textOutline', ...t }); const b = o.result?.bbox; return b && (b[0] < 0.01 || b[1] < 0.01 || b[2] > c.plate.w - 0.01 || b[3] > c.plate.h - 0.01); })) c = { ...c, area: 'bounds' }; // leaves the plate: only bounds
  const sign = c.op === 'add' ? 1 : -1;
  const removed = sign * (vol - base); // positive = |change|
  let detail = `vol ${vol.toFixed(3)} change ${(vol - base).toFixed(3)}`;
  if (c.area === 'bounds' || c.area === 'any') {
    if (area == null) return { ok: false, detail: 'no outline', ms };
    const max = area * c.depth;
    if (removed < 0.2 * max || removed > max * 1.003) return { ok: false, detail: `${detail} want within (0.2..1) x ${max.toFixed(3)}`, ms };
  } else {
    const want = area * c.depth;
    if (Math.abs(removed - want) > 0.01 * want + 1e-6) return { ok: false, detail: `${detail} want change ${(sign * want).toFixed(3)} (${((removed / want - 1) * 100).toFixed(2)}%)`, ms };
    detail += ` (${((removed / want - 1) * 100).toFixed(3)}% of glyph area x depth)`;
  }
  if (c.maxMs && ms > c.maxMs) return { ok: false, detail: `${ms} ms > ${c.maxMs}`, ms };
  return { ok: true, detail, ms };
}

// ad hoc: node tools/text_corpus.mjs --text 'STRING' [--font gauge|engrave] [--height 10] [--depth 0.4] [--write dir]
if (arg('--text')) {
  cases.length = 0;
  C(`text ${arg('--text')}`, { plate: { kind: 'box', w: Number(arg('--pw') ?? 100), h: 60, t: 5 }, texts: [T(arg('--text'), { font: arg('--font') ?? 'gauge', height: Number(arg('--height') ?? 10), at: (arg('--at') ?? '50,30').split(',').map(Number), anchor: [arg('--ah') ?? 'center', arg('--av') ?? 'middle'], ...(process.argv.includes('--poly') ? { polyline: true } : {}), ...(arg('--runspans') ? { runSpans: Number(arg('--runspans')) } : {}) })], depth: Number(arg('--depth') ?? 0.4) });
}
// random strings / sizes / depths / positions / fonts (seeded): node tools/text_corpus.mjs --fuzz 200 [--seed 1]
if (arg('--fuzz')) {
  cases.length = 0;
  let seed = Number(arg('--seed') ?? 1);
  const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
  const pick = (a) => a[Math.floor(rnd() * a.length)];
  const set = [];
  for (let c = 0x21; c < 0x7f; c++) set.push(String.fromCharCode(c));
  set.push('\u00b0', '\u00b1', '\u00d7', '\u00e9', '\u00fc', '\u00d8');
  for (let i = 0; i < Number(arg('--fuzz')); i++) {
    const len = 1 + Math.floor(rnd() * 6);
    let str = ''; for (let k = 0; k < len; k++) str += pick(set);
    const h = pick([3, 4, 6, 9, 14, 20]);
    C(`fuzz ${i} ${JSON.stringify(str)} h${h}`, { plate: { kind: 'box', w: 120, h: 60, t: 5 }, depth: pick([0.1, 0.2, 0.4, 0.8, 2]), op: rnd() < 0.2 ? 'add' : 'cut',
      texts: [T(str, { font: pick(['gauge', 'engrave']), height: h, at: [60 + (rnd() - 0.5) * 20, 30 + (rnd() - 0.5) * 12], angle: rnd() < 0.3 ? Math.floor(rnd() * 360) : 0, anchor: [pick(['left', 'center', 'right']), pick(['baseline', 'middle', 'top'])], spacing: pick([1, 1, 1.2]), ...(rnd() < 0.1 ? { polyline: true } : {}) })] });
  }
}
let bad = 0, n = 0;
cases.forEach((c, i) => {
  if (only && !c.name.includes(only)) return;
  n++;
  const r = run(c, i + 1);
  if (!r.ok) bad++;
  if (!r.ok || !process.argv.includes('--quiet')) console.log(`${r.ok ? 'ok  ' : 'FAIL'} ${c.name}: ${r.detail} (${r.ms} ms)`);
});
console.log(`text_corpus: ${n - bad}/${n} ok`);
process.exit(bad ? 1 : 0);
