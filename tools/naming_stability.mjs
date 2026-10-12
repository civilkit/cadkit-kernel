// Naming-stability test: do references to faces/edges of EARLIER features still
// point at the SAME geometric entities after an upstream dimension edit?
//
// usage: node tools/naming_stability.mjs [--wasm path] [--no-orig] [--all] [--quiet] [--case name]
//   Prints one row per (case, edit) pair and a summary. The original kernel.wasm
//   is run through the same edit sequence (--no-orig skips it) so inherent
//   losses can be told apart from our own.
//
// Method per case:
//   1. regen the baseline; collect every name a LATER feature references
//      (face ids anywhere in the feature JSON, `{faces:[a,b], at}` edge refs)
//      plus, for `watchAll` cases, every face id of the baseline body.
//      Record a geometric signature of each (see sigFace / sigEdge).
//   2. apply an upstream edit that should keep the topology (scale a sketch
//      dimension +/-25%, extrude depth +/-30%, move/resize a hole, pattern
//      count +1), regen, resolve each name in the new body (faces by id,
//      edges like the kernel: the edge between the two faces nearest to the
//      STALE `at` hint) and compare signatures.
//   3. negative edits (huge fillet radius, delete a sketch entity / sketch /
//      extrude) only record the structured error code (never a trap).
//
// Result classes:  ok | name-lost (missingReference/missingEdge or id gone) |
//   wrong-entity (name resolves to a geometrically different entity: the
//   dangerous one) | err:<code> (other structured feature error) | TRAP.
//
// Face signature: surface kind, normal (plane) / axis (others), area (kernel
// `measure`), centroid proxy (mean of the face's edge points) as a fraction of
// the body bbox, adjacent face kinds. Edge signature: curve kind, length, mid as
// a bbox fraction, direction, adjacent face kinds. "Same" = same kind, same
// normal/axis/direction, adjacent kinds equal, area/length ratio within [0.25, 4],
// and centroid/mid fraction within TOL (0.2) per axis. The position test is
// deliberately loose (edits legitimately move things) but separates opposite faces,
// pattern instances and hole vs. outer faces.
import { readFileSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import { parts } from './brep_cases.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const TOL = 0.2;
const clone = (x) => JSON.parse(JSON.stringify(x));

// ---------------------------------------------------------------- cases
const load = (f) => JSON.parse(readFileSync(path.resolve(here, f), 'utf8'));
const brep = (n, o = {}) => ({ name: n, part: () => clone(parts[n]), ...o });
const caseList = () => {
  const cs = [
    brep('faceSketch'), brep('pocket'), brep('boss'), brep('upToNextCut'),
    brep('filletOne'), brep('filletTwo'), brep('filletRail'), brep('chamferOne'), brep('chamferTop4'),
    brep('holeRimFillet'), brep('holeBothChamfer'),
    brep('pocketFloorFillet'), brep('bossTopFillet'), brep('bossBaseFillet'), brep('dbossBaseFillet'),
    brep('frustumBotFillet'), brep('ringInnerFillet'), brep('filletCylBot'),
    brep('draftBox'), brep('draftBoxTop'), brep('draftTwo'), brep('draftCyl'), brep('lipHollow'), brep('lipGroove'), brep('ribHollow'),
    brep('shellBoxOpen'), brep('shellBoxOpenTwo'), brep('shellCylOpen'),
    brep('lpatternCut', { watchAll: true }), { name: 'cpatternCut', watchAll: true, part: () => { const p = clone(parts.cpatternCut); p.features.at(-1).angle = 180; return p; } },
    brep('cpatternBoss', { watchAll: true }), brep('mirrorTwo', { watchAll: true }),
      ];
  // hole feature on a face-based sketch
  const hs = clone(parts.box);
  hs.features.push({ type: 'sketch', id: 'F3', plane: { kind: 'face', id: 'F2.C1' }, points: [{ id: 'c', x: 12, y: 8 }], entities: [{ type: 'point', id: 'e1', p: 'c' }], constraints: [] },
    { type: 'hole', id: 'F4', sketch: 'F3', standard: 'iso', size: 'M4', fit: 'normal', kind: 'simple', end: 'blind', depth: 6 });
  cs.push({ name: 'holeOnFace', part: () => clone(hs) });
  // sketch on a pocket floor (face of a LATER feature) then fillet its rim
  const pf = clone(parts.pocket);
  pf.features.push({ type: 'sketch', id: 'F5', plane: { kind: 'face', id: 'F4.C0' }, points: [{ id: 'c', x: 10, y: 10 }], entities: [{ type: 'circle', id: 'e1', c: 'c', r: 1 }], constraints: [] },
    { type: 'extrude', id: 'F6', op: 'cut', sketch: 'F5', end: 'blind', depth: 2 });
  cs.push({ name: 'pocketInPocket', part: () => clone(pf) });
  // fillet a boss rim, then fillet the pocket rim of a hole minted later (two-level names)
  const hf = clone(parts.holeRimFillet);
  hf.features.push({ type: 'fillet', id: 'F4b', radius: 0.3, edges: [{ faces: ['F2.C0', 'F4.S.e1'], at: [7, 0, -5] }] });
  cs.push({ name: 'twoFillets', part: () => clone(hf) });
  // ckpart files
  const art = path.dirname(realpathSync(path.join(here, '../../kernel.wasm')));
  for (const [n, f, k] of [['bracket', path.join(art, 'bracket.ckpart')], ['flangeHalf@11', 'cases/flangeHalf.ckpart', 11], ['flangeBolt@12', 'cases/flangeBolt.ckpart', 12]]) {
    try { const p = load(f); if (k) p.features = p.features.slice(0, k); cs.push({ name: n, part: () => clone(p), file: true }); } catch { /* optional */ }
  }
  return cs;
};

// ---------------------------------------------------------------- edits
const feats = (p, t) => p.features.filter((f) => f.type === t);
const num = (v) => typeof v === 'number';
const edits = [
  { name: 'sk+25', f: 1.25, apply: (p, f) => scaleSketch(p, f) },
  { name: 'sk-25', f: 0.75, apply: (p, f) => scaleSketch(p, f) },
  { name: 'skx+40', f: 1.4, apply: (p, f) => scaleSketch(p, f, true) },
  { name: 'dep+30', f: 1.3, apply: (p, f) => scaleDepth(p, f) },
  { name: 'dep-30', f: 0.7, apply: (p, f) => scaleDepth(p, f) },
  { name: 'hole-move', apply: (p) => moveHole(p) },
  { name: 'hole-r+25', apply: (p) => holeRadius(p, 1.25) },
  { name: 'count+1', loosePos: true, apply: (p) => patCount(p) },
  // negative / topology-changing
  { name: 'fil-huge', neg: true, apply: (p) => { let n = 0; for (const f of p.features) if (f.type === 'fillet') { f.radius = 60; n++; } else if (f.type === 'chamfer') { f.distance = 60; n++; } return n ? p : null; } },
  { name: 'del-entity', neg: true, apply: (p) => { const s = feats(p, 'sketch')[0]; if (!s || s.entities.length < 2) return null; s.entities = s.entities.filter((e) => e.id !== 'e2'); return p; } },
  { name: 'del-sketch', neg: true, apply: (p) => { const i = p.features.findIndex((f) => f.type === 'sketch'); if (i < 0) return null; p.features.splice(i, 1); return p; } },
  { name: 'del-extrude', neg: true, apply: (p) => { const i = p.features.findIndex((f) => f.type === 'extrude'); if (i < 0) return null; p.features.splice(i, 1); return p; } },
];
function scaleSketch(p, f, xOnly = false) {
  const s = feats(p, 'sketch')[0];
  if (!s) return null;
  const dims = (s.constraints ?? []).filter((c) => num(c.value) && /distance|length|radius|diam/i.test(c.type));
  if (dims.length) { for (const c of xOnly ? dims.slice(0, 1) : dims) c.value *= f; return p; }
  if (xOnly) { if (s.entities.some((e) => e.type !== 'line')) return null; for (const q of s.points) q.x *= f; return p; }
  for (const q of s.points) { q.x *= f; q.y *= f; }
  for (const e of s.entities) if (num(e.r)) e.r *= f;
  return p;
}
function scaleDepth(p, f) {
  const e = feats(p, 'extrude').find((x) => num(x.depth) && x.depth > 0 && !/through/.test(x.end ?? ''));
  if (!e) return null;
  e.depth *= f; return p;
}
function moveHole(p) {
  // the first circle that sits in a later (non-first) sketch
  const ss = feats(p, 'sketch').slice(1);
  for (const s of ss) {
    const c = s.entities.find((e) => e.type === 'circle');
    const pt = c && s.points.find((q) => q.id === c.c);
    if (pt && !(s.constraints ?? []).length) { pt.x += 2; pt.y += 1.5; return p; }
    if (!c && s.points.length === 1) { s.points[0].x += 2; s.points[0].y += 1.5; return p; }
  }
  return null;
}
function holeRadius(p, f) {
  for (const s of feats(p, 'sketch').slice(1)) {
    const c = s.entities.find((e) => e.type === 'circle' && num(e.r));
    if (c && !(s.constraints ?? []).length) { c.r *= f; return p; }
  }
  return null;
}
function patCount(p) {
  const f = p.features.find((x) => x.type === 'lpattern' || x.type === 'cpattern');
  if (!f) return null;
  f.count += 1; return p;
}

// ---------------------------------------------------------------- reference collection
const FACE_RE = /^F\d+\.[A-Za-z]/;
function collectRefs(part, body) {
  const ids = new Set(body.faces.map((f) => f.id));
  const faces = new Set(), edges = [];
  const walk = (v) => {
    if (typeof v === 'string') { if (FACE_RE.test(v) && ids.has(v)) faces.add(v); return; }
    if (Array.isArray(v)) { v.forEach(walk); return; }
    if (v && typeof v === 'object') {
      if (Array.isArray(v.faces) && v.faces.length === 2 && Array.isArray(v.at)) {
        if (v.faces.every((s) => ids.has(s))) edges.push({ faces: v.faces, at: v.at });
        return;
      }
      for (const [k, x] of Object.entries(v)) if (k !== 'id' || typeof x !== 'string' || !FACE_RE.test(x) || !('kind' in v)) walk(x); else walk(x);
    }
  };
  part.features.forEach((f, i) => { if (i > 0) walk(f); });
  return { faces: [...faces], edges };
}

// ---------------------------------------------------------------- kernel access
class Rig {
  constructor(file) { this.k = new Kernel(new WebAssembly.Module(readFileSync(file))); }
  q(msg) {
    const r = this.k.request(JSON.stringify({ id: 1, msg }));
    if (r.trap) return { trap: r.trap };
    try { return JSON.parse(r.json); } catch { return { trap: 'badjson' }; }
  }
  regen(part) { return this.q({ type: 'regen', tag: 'main', part }); }
  measureFace(id) { return this.q({ type: 'measure', tag: 'main', refs: [{ kind: 'face', id }] })?.result?.items?.[0] ?? null; }
}
const sub = (a, b) => a.map((x, i) => x - b[i]);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const nrm = (a) => { const l = Math.hypot(...a); return l > 1e-12 ? a.map((x) => x / l) : [0, 0, 0]; };
const bboxFrac = (box, p) => p.map((x, i) => { const d = box.max[i] - box.min[i]; return d > 1e-9 ? (x - box.min[i]) / d : 0.5; });

function bodyIndex(body) {
  const kinds = new Map(body.faces.map((f) => [f.id, f.kind]));
  const byFace = new Map();
  for (const e of body.edges) for (const f of e.faces ?? []) { if (!byFace.has(f)) byFace.set(f, []); byFace.get(f).push(e); }
  return { kinds, byFace };
}
const polyLen = (pts) => pts.slice(1).reduce((s, p, i) => s + Math.hypot(...sub(p, pts[i])), 0);
function segDist(p, a, b) {
  const ab = sub(b, a), t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / (dot(ab, ab) || 1)));
  return Math.hypot(...sub(p, a.map((x, i) => x + t * ab[i])));
}
const curveDist = (pts, at) => Math.min(...pts.slice(1).map((p, i) => segDist(at, pts[i], p)), pts.length === 1 ? Math.hypot(...sub(at, pts[0])) : Infinity);

function sigFace(rig, body, idx, id) {
  const f = body.faces.find((x) => x.id === id);
  if (!f) return null;
  const dup = body.faces.filter((x) => x.id === id).length;
  const m = rig.measureFace(id);
  const es = idx.byFace.get(id) ?? [];
  const pts = es.flatMap((e) => e.pts);
  const c = pts.length ? pts.reduce((s, p) => s.map((x, i) => x + p[i] / pts.length), [0, 0, 0]) : null;
  const adj = [...new Set(es.flatMap((e) => e.faces))].filter((x) => x !== id).map((x) => idx.kinds.get(x)); const adjS = [...new Set(adj)].sort().join(',');
  return {
    kind: f.kind, dup, adj: adjS, area: m?.area ?? null,
    dir: nrm(m?.normal ?? m?.axis ?? f.normal ?? [0, 0, 0]), planar: !!(m?.normal ?? f.normal),
    frac: c ? bboxFrac(body.box, c) : null, abs: c, diag: Math.hypot(...sub(body.box.max, body.box.min)),
  };
}
function resolveEdge(body, ref) {
  const [a, b] = ref.faces;
  const c = body.edges.filter((e) => e.faces?.length === 2 && e.faces.includes(a) && e.faces.includes(b));
  if (!c.length) return null;
  return c.reduce((best, e) => (curveDist(e.pts, ref.at) < curveDist(best.pts, ref.at) ? e : best));
}
function sigEdge(body, idx, e) {
  const p0 = e.pts[0], p1 = e.pts[e.pts.length - 1];
  return {
    line: !!e.line, len: polyLen(e.pts),
    dir: e.line ? nrm(sub(p1, p0)) : null,
    adj: [...new Set(e.faces.map((f) => idx.kinds.get(f)))].sort().join(','),
    frac: bboxFrac(body.box, e.mid ?? e.pts[Math.floor(e.pts.length / 2)]),
    abs: e.mid ?? e.pts[Math.floor(e.pts.length / 2)], diag: Math.hypot(...sub(body.box.max, body.box.min)),
  };
}
const fdist = (a, b) => Math.max(...a.map((x, i) => Math.abs(x - b[i])));
// position: stable as a bbox fraction OR as an absolute point (edits move things either way)
const posBad = (a, b) => a.frac && b.frac && fdist(a.frac, b.frac) > TOL && fdist(a.abs, b.abs) > 0.07 * Math.max(a.diag, b.diag);
const fmt = (v) => v.map((x) => +x.toFixed(2));
const ratioOk = (a, b) => a === null || b === null || (b > 1e-9 ? a / b : 1) > 0.25 && (b > 1e-9 ? a / b : 1) < 4;
function cmpFace(a, b, loose) {
  if (b.dup > 1) return 'duplicate id';
  if (a.kind !== b.kind) return `kind ${a.kind}->${b.kind}`;
  const d = Math.abs(dot(a.dir, b.dir));
  if (a.planar ? dot(a.dir, b.dir) < 0.999 : d < 0.99) return `dir ${a.dir.map((x) => +x.toFixed(2))}->${b.dir.map((x) => +x.toFixed(2))}`;
  if (a.adj !== b.adj) return `adj ${a.adj}->${b.adj}`;
  if (!ratioOk(b.area, a.area)) return `area ${a.area?.toFixed(1)}->${b.area?.toFixed(1)}`;
  if (!loose && posBad(a, b)) return `pos ${fmt(a.abs)}->${fmt(b.abs)}`;
  return null;
}
function cmpEdge(a, b, loose) {
  if (a.line !== b.line) return 'curve kind';
  if (a.line && Math.abs(dot(a.dir, b.dir)) < 0.999) return 'direction';
  if (a.adj !== b.adj) return `adj ${a.adj}->${b.adj}`;
  if (!ratioOk(b.len, a.len)) return `len ${a.len.toFixed(1)}->${b.len.toFixed(1)}`;
  if (!loose && posBad(a, b)) return `pos ${fmt(a.abs)}->${fmt(b.abs)}`;
  return null;
}

// ---------------------------------------------------------------- run
const LOST = /^(missingReference|missingEdge|missingFace|missingSketch)$/;
function baseline(rig, c) {
  const part = c.part();
  const r = rig.regen(part);
  const body = r.result?.body;
  if (!body || r.result.errors?.length) return { err: r.trap ?? JSON.stringify(r.result?.errors ?? r.error)?.slice(0, 120) };
  const idx = bodyIndex(body);
  const refs = collectRefs(part, body);
  if (c.watchAll) for (const f of body.faces) if (!refs.faces.includes(f.id)) refs.faces.push(f.id);
  const sigs = { faces: {}, edges: [] };
  for (const id of refs.faces) sigs.faces[id] = sigFace(rig, body, idx, id);
  for (const e of refs.edges) {
    const re = resolveEdge(body, e);
    sigs.edges.push({ ref: e, sig: re ? sigEdge(body, idx, re) : null });
  }
  return { part, sigs, nref: refs.faces.length + refs.edges.length, body };
}
function trial(rig, c, base, ed) {
  const p2 = ed.apply(clone(base.part), ed.f);
  if (!p2) return null;
  const r = rig.regen(p2);
  if (r.trap) return { cls: 'TRAP', detail: r.trap.slice(0, 100) };
  if (r.ok === false) return { cls: 'err:' + 'envelope', detail: String(r.error).slice(0, 100) };
  const errs = r.result?.errors ?? [];
  const body = r.result?.body;
  if (errs.length) {
    const [fid, e] = errs[0];
    return { cls: LOST.test(e.code) ? 'name-lost' : 'err:' + e.code, detail: `${fid} ${e.code}${e.params?.detail ? ' ' + e.params.detail : ''}`.slice(0, 110) };
  }
  if (!body) return { cls: 'err:nobody', detail: '' };
  const idx = bodyIndex(body);
  const lost = [], wrong = [];
  for (const [id, a] of Object.entries(base.sigs.faces)) {
    if (!a) continue;
    const b = sigFace(rig, body, idx, id);
    if (!b) { lost.push(id); continue; }
    const why = cmpFace(a, b, ed.loosePos && id.includes('@'));
    if (why) wrong.push(`${id}: ${why}`);
  }
  for (const { ref, sig } of base.sigs.edges) {
    if (!sig) continue;
    const e = resolveEdge(body, ref);
    const tag = `${ref.faces.join('|')}`;
    if (!e) { lost.push(tag); continue; }
    const why = cmpEdge(sig, sigEdge(body, idx, e), false);
    if (why) wrong.push(`${tag}: ${why}`);
  }
  if (wrong.length) return { cls: 'wrong-entity', detail: wrong.slice(0, 3).join(' ; ') };
  if (lost.length) return { cls: 'name-lost', detail: 'gone: ' + lost.slice(0, 4).join(',') };
  return { cls: 'ok', detail: '' };
}

export function runNaming({ ours, orig = null, only = null, quiet = true, log = () => {} } = {}) {
  const A = new Rig(ours), O = orig ? new Rig(orig) : null;
  const rows = [];
  for (const c of caseList()) {
    if (only && c.name !== only) continue;
    const base = baseline(A, c), baseO = O ? baseline(O, c) : null;
    if (base.err) { rows.push({ case: c.name, edit: '(baseline)', cls: 'base-error', detail: base.err, neg: false }); continue; }
    for (const ed of edits) {
      const t = trial(A, c, base, ed);
      if (!t) continue;
      const o = O && baseO && !baseO.err ? trial(O, c, baseO, ed) : null;
      const row = { case: c.name, edit: ed.name, neg: !!ed.neg, nref: base.nref, ...t, orig: o?.cls ?? (O ? 'base-error' : null) };
      rows.push(row); log(row);
    }
  }
  return rows;
}

export function summarize(rows) {
  const pos = rows.filter((r) => !r.neg && r.edit !== '(baseline)');
  const neg = rows.filter((r) => r.neg);
  const count = (rs) => rs.reduce((m, r) => ((m[r.cls.replace(/^err:.*/, 'err')] = (m[r.cls.replace(/^err:.*/, 'err')] ?? 0) + 1), m), {});
  return { pos: pos.length, posCounts: count(pos), neg: neg.length, negCodes: neg.reduce((m, r) => ((m[r.cls] = (m[r.cls] ?? 0) + 1), m), {}), base: rows.filter((r) => r.edit === '(baseline)').length };
}

// ---------------------------------------------------------------- CLI
if (process.argv[1] && realpathSync(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const a = process.argv.slice(2);
  const opt = (f) => { const i = a.indexOf(f); return i < 0 ? null : a[i + 1]; };
  const ours = opt('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
  const orig = a.includes('--no-orig') ? null : realpathSync(path.join(here, '../../kernel.wasm'));
  const t0 = performance.now();
  const rows = runNaming({ ours, orig, only: opt('--case') });
  const pad = (s, n) => String(s).padEnd(n);
  console.log(`${pad('case', 20)} ${pad('edit', 11)} ${pad('ours', 14)} ${pad('orig', 14)} detail`);
  for (const r of rows) if (a.includes('--all') || r.cls !== 'ok' || (orig && r.orig !== 'ok')) console.log(`${pad(r.case, 20)} ${pad(r.edit, 11)} ${pad(r.cls, 14)} ${pad(r.orig ?? '-', 14)} ${r.detail}`);
  const s = summarize(rows);
  console.log(`\npositive pairs: ${s.pos}  ${JSON.stringify(s.posCounts)}`);
  console.log(`negative edits: ${s.neg}  codes ${JSON.stringify(s.negCodes)}  baseline errors: ${s.base}`);
  if (orig) {
    const same = rows.filter((r) => !r.neg && r.cls !== 'ok' && r.orig === r.cls).length;
    console.log(`non-ok pairs where the original gives the same class: ${same}`);
  }
  console.log(`${((performance.now() - t0) / 1000).toFixed(1)}s`);
}
