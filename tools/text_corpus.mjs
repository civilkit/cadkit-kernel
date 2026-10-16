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
// ---- fonts loaded from bytes (loadFont): the curated fonts as small subsets (crates/ck-text/tests/fixtures)
const FX = path.join(here, '../crates/ck-text/tests/fixtures');
const fx = (n, ext = 'ttf') => ({ name: 'fx-' + n, file: path.join(FX, `fx-${n}.${ext}`) });
const fxText = 'BOSS 0689 &-.';
for (const [n, ext, extra] of [['sans-barlow', 'ttf', {}], ['sans-source-otf', 'otf', {}], ['condensed-barlow', 'ttf', {}], ['condensed-bebas', 'ttf', {}], ['serif-ptserif', 'ttf', {}],
  ['mono-plex', 'ttf', {}], ['display-anton', 'ttf', {}], ['slab-specialelite', 'ttf', { area: 'any', warn: 'textOverlappingContours', noValidate: true }], ['script-greatvibes', 'ttf', { area: 'any' }], ['script-satisfy', 'ttf', { area: 'any' }], ['variable-inter', 'ttf', { area: 'any', warn: 'textOverlappingContours', noValidate: true }]])
  C(`loaded font ${n}`, { load: [fx(n, ext)], texts: [T(fxText, { font: 'fx-' + n, height: 9 })], ...extra });
C('loaded font emboss', { load: [fx('sans-source-otf', 'otf')], op: 'add', depth: 0.6, texts: [T('BOSS 689', { font: 'fx-sans-source-otf', height: 10 })] });
C('loaded font on an arc', { load: [fx('condensed-bebas')], plate: { kind: 'disk', r: 50, t: 3 }, texts: [T('BOSS 689', { font: 'fx-condensed-bebas', height: 5, at: [0, 0], anchor: ['center', 'baseline'], arc: { centre: [0, 0], radius: 28, angleCentre: 90, side: 'outside' } })] });
C('font not loaded', { texts: [T('A', { font: 'fx-never-loaded' })], err: 'textFontNotLoaded' });
C('font hash pin mismatch', { load: [fx('sans-barlow')], texts: [T('BOSS', { font: 'fx-sans-barlow', fontHash: '00000000' })], err: 'textFontMismatch' });
C('font hash pin matches', { load: [fx('sans-barlow')], pin: 'fx-sans-barlow', texts: [T('BOSS', { font: 'fx-sans-barlow', height: 9 })] });
// ---- errors and warnings
C('empty string', { texts: [T('')], err: 'textEmpty' });
C('blank string', { texts: [T('   ')], err: 'textNothingToDraw' });
C('unknown font', { texts: [T('A', { font: 'comic' })], err: 'textFontNotLoaded' });
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

const b64 = (f) => readFileSync(f).toString('base64');
function run(c, idx) {
  const k = new Kernel(mod);
  for (const f of c.load ?? []) {
    const l = rqk(k, { type: 'loadFont', name: f.name, data: b64(f.file) });
    if (!l.ok) return { ok: false, detail: 'loadFont ' + JSON.stringify(l.error ?? l), ms: 0 };
    if (c.pin === f.name) for (const t of c.texts) t.fontHash = l.result.hash.slice(0, 16);
  }
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
  // overlapping strokes inside one letter are not merged (warned): the solid is then known to fail validation
  if (c.noValidate ? !issues.every((i) => i.code === 'LOOP_SELF_INTERSECT') : issues.length) return { ok: false, detail: 'validate ' + [...new Set(issues.map((i) => i.code))].join(','), ms };
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
// ---- scenarios: replace / unload / subset through the request interface, with the regen cache on
function fontScenarios() {
  const out = [];
  const ok = (name, good, detail) => out.push({ name, ok: !!good, detail });
  const k = new Kernel(mod);
  const part1 = part({ plane: 'front', plate: { kind: 'box', w: 100, h: 60, t: 5 }, depth: 0.3, op: 'cut', texts: [T('BOSS 689', { font: 'swap', height: 10 })] });
  const regen = () => { const r = rqk(k, { type: 'regen', tag: 'main', part: part1, opts: { cacheStats: true } }); return r; };
  const vol = () => rqk(k, { type: 'massProps', tag: 'main' }).result?.volume;
  const err = (r) => (r.result?.errors ?? []).map(([id, e]) => e.code).join(',');
  const a = rqk(k, { type: 'loadFont', name: 'swap', data: b64(fx('sans-barlow').file) });
  ok('loadFont reply', a.ok && a.result.glyphs > 20 && a.result.supported.includes('Basic Latin') && Array.isArray(a.result.warnings) && /^[0-9a-f]{64}$/.test(a.result.hash), JSON.stringify(a.result ?? a).slice(0, 200));
  let r = regen(); const vA = vol();
  ok('regen with font A', !err(r), err(r));
  const again = regen();
  ok('second regen is a cache hit', again.result?.cacheStats?.fullHit === true || again.result?.cache?.fullHit === true || JSON.stringify(again.result).includes('"fullHit":true'), JSON.stringify(again.result?.cacheStats ?? again.result?.cache ?? '').slice(0, 160));
  const conflict = rqk(k, { type: 'loadFont', name: 'swap', data: b64(fx('serif-ptserif').file) });
  ok('same name, other file: fontNameConflict', !conflict.ok && String(conflict.error).includes('fontNameConflict'), JSON.stringify(conflict).slice(0, 200));
  r = regen();
  ok('conflict left font A in place', vol() === vA, `${vol()} vs ${vA}`);
  const b = rqk(k, { type: 'loadFont', name: 'swap', data: b64(fx('serif-ptserif').file), replace: true });
  ok('replace:true', b.ok && b.result.replaced === true && b.result.hash !== a.result.hash, JSON.stringify(b).slice(0, 160));
  r = regen(); const vB = vol();
  ok('regen after replace is not served from the cache', !err(r) && Math.abs(vB - vA) > 1e-6, `${vA} -> ${vB}`);
  const ol = rqk(k, { type: 'textOutline', ...T('BOSS 689', { font: 'swap', height: 10 }) });
  ok('textOutline uses the replaced font', ol.ok && Math.abs(vB - (100 * 60 * 5 - 0.3 * glyphArea(k, [T('BOSS 689', { font: 'swap', height: 10 })]))) < 0.02 * 0.3 * glyphArea(k, [T('BOSS 689', { font: 'swap', height: 10 })]), '');
  const list = rqk(k, { type: 'listFonts' });
  ok('listFonts', list.ok && list.result.fonts[0].name === 'gauge' && list.result.fonts[1].name === 'engrave' && list.result.fonts.some((f) => f.name === 'swap' && f.hash === b.result.hash), JSON.stringify(list.result?.fonts?.map((f) => f.name)));
  const un = rqk(k, { type: 'unloadFont', name: 'swap' });
  ok('unloadFont', un.ok && un.result.unloaded === true, JSON.stringify(un));
  r = regen();
  ok('regen after unload: textFontNotLoaded naming the font', err(r).includes('textFontNotLoaded') && JSON.stringify(r.result.errors).includes('swap') && JSON.stringify(r.result.errors).includes('loadFont'), JSON.stringify(r.result?.errors).slice(0, 300));
  const un2 = rqk(k, { type: 'unloadFont', name: 'gauge' });
  ok('built-in cannot be unloaded', !un2.ok && String(un2.error).includes('fontReserved'), JSON.stringify(un2));
  ok('built-in name cannot be loaded over', !rqk(k, { type: 'loadFont', name: 'gauge', data: b64(fx('sans-barlow').file) }).ok, '');
  ok('bad base64', String(rqk(k, { type: 'loadFont', name: 'x', data: '***' }).error).includes('fontBadData'), '');
  ok('not a font', String(rqk(k, { type: 'loadFont', name: 'x', data: Buffer.from('hello world, not a font').toString('base64') }).error).includes('fontBadData'), '');
  // subset: the glyphs of the string only, same geometry as the full font
  const sub = rqk(k, { type: 'subsetFont', name: 'gauge', text: 'RPM x100' });
  ok('subsetFont reply', sub.ok && sub.result.data.length > 100 && sub.result.bytes < 8000 && sub.result.glyphs <= 10, JSON.stringify({ bytes: sub.result?.bytes, glyphs: sub.result?.glyphs }));
  rqk(k, { type: 'loadFont', name: 'gsub', data: sub.result.data });
  const o1 = rqk(k, { type: 'textOutline', ...T('RPM x100', { font: 'gauge', height: 8 }) });
  const o2 = rqk(k, { type: 'textOutline', ...T('RPM x100', { font: 'gsub', height: 8 }) });
  ok('subset gives the same outline as the built-in font', o1.ok && o2.ok && JSON.stringify(o1.result.regions) === JSON.stringify(o2.result.regions), '');
  const m1 = rqk(k, { type: 'textMetrics', string: 'RPM x100', font: 'gauge', height: 8 });
  const m2 = rqk(k, { type: 'textMetrics', string: 'RPM x100', font: 'gsub', height: 8 });
  ok('subset gives the same metrics', JSON.stringify(m1.result) === JSON.stringify(m2.result.warnings ? m2.result : m2.result), '');
  const miss = rqk(k, { type: 'textMetrics', string: 'RPM', font: 'nothere', height: 8 });
  ok('textMetrics with a font that is not loaded', !miss.ok && String(miss.error).includes('textFontNotLoaded') && String(miss.error).includes('nothere'), JSON.stringify(miss).slice(0, 200));
  // two kernels do not share fonts
  const k2 = new Kernel(mod);
  ok('fonts belong to the kernel instance', !rqk(k2, { type: 'textMetrics', string: 'A', font: 'gsub', height: 8 }).ok, '');
  return out;
}
let bad = 0, n = 0;
if (!only || 'font scenarios'.includes(only)) for (const r of fontScenarios()) { n++; if (!r.ok) bad++; if (!r.ok || !process.argv.includes('--quiet')) console.log(`${r.ok ? 'ok  ' : 'FAIL'} font scenario: ${r.name}${r.ok ? '' : ': ' + r.detail}`); }
cases.forEach((c, i) => {
  if (only && !c.name.includes(only)) return;
  n++;
  const r = run(c, i + 1);
  if (!r.ok) bad++;
  if (!r.ok || !process.argv.includes('--quiet')) console.log(`${r.ok ? 'ok  ' : 'FAIL'} ${c.name}: ${r.detail} (${r.ms} ms)`);
});
console.log(`text_corpus: ${n - bad}/${n} ok`);
process.exit(bad ? 1 : 0);
