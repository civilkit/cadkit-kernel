// OCCT-derived corpus: the OCCT tests/ scripts that can be translated to the
// kernel's feature tree, run through the wasm with OCCT's own checkprops area
// as the reference.
//
//   tools/occt_fetch.sh                      # sparse-clone tests/{boolean,blend,draft} into /tmp/occt
//   node tools/occt_corpus.mjs --occt /tmp/occt [--only substr] [--json out.json] [--md out.md]
//
// This is a *frontier* corpus: OCCT passes its own checks; a failing case here
// is a real gap in our kernel, not a broken test. Cases we cannot express with
// the current wire protocol are reported as `unsupported` (and counted, so the
// gap is visible) rather than silently dropped.
//
// Selection ("top ~200"): every primitive-only solid script in
// tests/boolean/{bfuse_simple,bcut_simple,bcommon_simple}, then the blend and
// draft scripts that use full primitives only. The translator is deliberately
// strict: a command it does not understand marks the whole case unsupported.
//
// OCCT test scripts are LGPL-2.1 with an exception; only their geometry is
// translated here, the scripts themselves are read from an OCCT checkout.
import { readFileSync, writeFileSync, existsSync, readdirSync, statSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { execFileSync } from 'node:child_process';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasmPath = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const occt = opt('--occt', '/tmp/occt');
const only = opt('--only');
const PI = Math.PI;
const picksFile = opt('--picks', path.join(here, 'occt_picks.json'));
const writePicksFile = opt('--write-picks');
const autoHeal = argv.includes('--auto-heal') ? Number(opt('--heal-tol') ?? 1e-5) : null;
let pickCache = existsSync(picksFile) ? JSON.parse(readFileSync(picksFile, 'utf8')) : {};

if (!existsSync(path.join(occt, 'tests'))) {
  console.error(`occt_corpus: no OCCT tests at ${occt}/tests\nrun tools/occt_fetch.sh first (or pass --occt DIR)`);
  process.exit(2);
}

// ---------------------------------------------------------------- the OCCT subset
// Only these commands may appear in a translated script. Anything else makes
// the case `unsupported` (with the offending command in `detail`).
const PRIMS = new Set(['box', 'psphere', 'pcylinder', 'pcone']);
const BOOLS = new Set(['bfuse', 'bcut', 'bcommon']);
const XFORMS = new Set(['trotate', 'ttranslate', 'tscale', 'copy']);
const IGNORE = new Set(['explode', 'nexplode', 'dset', 'lset', 'renamevar', 'donly', 'nurbsconvert', 'checkshape', 'checknbshapes', 'checkview']);
const CHECKS = new Set(['checkprops']);

function readScript(file) {
  const lines = [];
  for (const raw of readFileSync(file, 'utf8').split(/\r?\n/)) {
    const s = raw.trim();
    if (!s || s.startsWith('#') || s.startsWith('##')) continue;
    if (/^(puts|set|if|foreach|for|global|proc|return|unset|lappend|lset|incr|cpulimit|protect|pload)\b/.test(s)) continue;
    if (s === '}' || s === '{') continue;
    lines.push(s);
  }
  return lines;
}

function words(line) {
  // the scripts are whitespace-separated and do not quote numbers; keep it simple
  return line.split(/\s+/);
}

// ---------------------------------------------------------------- feature builders
const f = [];
let n = 0;
const seen = new Map();
let unsupported = null;
const unsupportedAt = (why) => { if (!unsupported) unsupported = why; };

const planeFor = (z) => {
  if (z === 0) return { plane: { kind: 'std', id: 'front' }, prefix: [] };
  const id = `P${++n}`;
  return { plane: { kind: 'feature', id }, prefix: [{ type: 'plane', id, mode: 'offset', base: { kind: 'std', id: 'front' }, offset: z }] };
};

function polygonSketch(id, plane, pts, construction = []) {
  const points = pts.map(([x, y], i) => ({ id: `p${i + 1}`, x, y }));
  const entities = pts.map((_, i) => ({ type: 'line', id: `e${i + 1}`, p1: `p${i + 1}`, p2: `p${((i + 1) % pts.length) + 1}` }));
  for (const c of construction) entities.push({ type: 'line', id: c.id, p1: c.p1, p2: c.p2, construction: true });
  return { type: 'sketch', id, plane, points, entities, constraints: [] };
}

function parkOffset() {
  // a tool lump must be disjoint from everything already built, so it is
  // created far away and then moved to its true position (same trick as
  // boolean_corpus's `two`). The stride is larger than any primitive in OCCT's
  // simple grids (the largest is a 1000-scaled 100 cube).
  return 1000 * (seen.size + 1);
}

function addFeature(feat) { f.push(feat); return feat.id; }

// Build one primitive as its own lump at the requested placement (offset by
// `park` while it is created, then moved back).
function buildPrimitive(name, kind, args) {
  const id = `F${++n}`;
  const park = parkOffset();
  const post = [];

  if (kind === 'box') {
    let [dx, dy, dz] = args;
    let [x0, y0, z0] = [0, 0, 0];
    if (args.length === 6) { [x0, y0, z0, dx, dy, dz] = args; }
    else if (args.length !== 3) return unsupportedAt(`${name}: box needs 3 or 6 numbers`);
    if (dx < 0) { x0 += dx; dx = -dx; }
    if (dy < 0) { y0 += dy; dy = -dy; }
    if (dz < 0) { z0 += dz; dz = -dz; }
    const { plane, prefix } = planeFor(z0);
    prefix.forEach(addFeature);
    const sid = `S${n}`;
    addFeature(polygonSketch(sid, plane, [[x0 + park, y0], [x0 + dx + park, y0], [x0 + dx + park, y0 + dy], [x0 + park, y0 + dy]]));
    const fid = addFeature({ type: 'extrude', id, op: 'add', sketch: sid, end: 'blind', depth: dz });
    post.push({ type: 'moveBody', id: `M${++n}`, bodies: [{ face: `${fid}.C1` }], translate: [-park, 0, 0] });
  } else if (kind === 'cylinder') {
    const [r, h, ang] = args;
    if ((args.length !== 2 && args.length !== 3) || !(r > 0) || !(h > 0)) return unsupportedAt(`${name}: pcylinder needs r h [angle]`);
    const { plane, prefix } = planeFor(0);
    prefix.forEach(addFeature);
    const sid = `S${n}`;
    let fid;
    if (args.length === 2) {
      addFeature({ type: 'sketch', id: sid, plane, points: [{ id: 'c', x: park, y: 0 }], entities: [{ type: 'circle', id: 'e1', c: 'c', r }], constraints: [] });
      fid = addFeature({ type: 'extrude', id, op: 'add', sketch: sid, end: 'blind', depth: h });
    } else {
      // a sector about world +Z: sketch on `top` (u=X, v=-Z), revolved by -ang
      const pts = [[park, 0], [park + r, 0], [park + r, -h], [park, -h]];
      addFeature(polygonSketch(sid, { kind: 'std', id: 'top' }, pts));
      fid = addFeature({ type: 'revolve', id, op: 'add', sketch: sid, axis: `e${pts.length}`, angle: -ang });
    }
    post.push({ type: 'moveBody', id: `M${++n}`, bodies: [{ face: `${fid}.${args.length === 2 ? 'C1' : 'S.e1'}` }], translate: [-park, 0, 0] });
  } else if (kind === 'cone') {
    const [r1, r2, h, ang] = args;
    if ((args.length !== 3 && args.length !== 4) || !(h > 0)) return unsupportedAt(`${name}: pcone needs r1 r2 h [angle]`);
    const { plane, prefix } = planeFor(0);
    prefix.forEach(addFeature);
    const sid = `S${n}`;
    // profile touching the axis; the last edge is the axis used by `revolve`
    // OCCT's pcone is about world +Z; the `top` sketch plane has v = -Z, so
    // the profile is (u, -z) and the revolve angle is negated.
    const pts = [[park, 0], [park + r1, 0], [park + r2, -h], [park, -h]];
    addFeature(polygonSketch(sid, { kind: 'std', id: 'top' }, pts));
    const fid = addFeature({ type: 'revolve', id, op: 'add', sketch: sid, axis: `e${pts.length}`, angle: args.length === 4 ? -ang : -360 });
    post.push({ type: 'moveBody', id: `M${++n}`, bodies: [{ face: `${fid}.S.e1` }], translate: [-park, 0, 0] });
  } else if (kind === 'sphere') {
    const [r] = args;
    if (args.length !== 1 || !(r > 0)) return unsupportedAt(`${name}: psphere needs r (full sphere)`);
    const { plane, prefix } = planeFor(0);
    prefix.forEach(addFeature);
    const sid = `S${n}`;
    addFeature({
      type: 'sketch', id: sid, plane,
      points: [{ id: 'c', x: park, y: 0 }, { id: 'a', x: park, y: -r }, { id: 'b', x: park, y: r }],
      entities: [{ type: 'arc', id: 'e1', c: 'c', p1: 'a', p2: 'b' }, { type: 'line', id: 'ax', p1: 'a', p2: 'b' }],
      constraints: [],
    });
    const fid = addFeature({ type: 'revolve', id, op: 'add', sketch: sid, axis: 'ax', angle: 360 });
    post.push({ type: 'moveBody', id: `M${++n}`, bodies: [{ face: `${fid}.S.e1` }], translate: [-park, 0, 0] });
  } else {
    return unsupportedAt(`${name}: ${kind} is not translated`);
  }
  post.forEach(addFeature);
  const entry = { face: post.length ? `${f[ f.length - 1 ].id}.C1` : null };
  // the moved lump's face is the one the moveBody named
  entry.face = post.length ? post[0].bodies[0].face : `${id}.C1`;
  entry.feature = id;
  entry.kind = kind;
  seen.set(name, entry);
  return null;
}

// ---------------------------------------------------------------- translate one script
function translate(file, caseName) {
  f.length = 0; n = 0; seen.clear(); unsupported = null;
  const lines = readScript(file);
  let wantArea = null, wantVolume = null;
  const bools = [];
  const cutTools = [];
  const blends = [];
  let pendingBop = null;
  let lumpCount = 0;
  const vars = {};
  const evalNum = (x) => {
    if (Object.prototype.hasOwnProperty.call(vars, x)) return vars[x];
    const n = Number(x);
    if (Number.isFinite(n)) return n;
    try {
      const names = Object.keys(vars);
      const fns = ['sqrt', 'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'abs', 'pow', 'exp', 'log'];
      return new Function(...names, ...fns, 'pi', 'PI', `return (${String(x).replace(/\^/g, '**')});`)(
        ...names.map((k) => vars[k]), ...fns.map((f) => Math[f]), Math.PI, Math.PI);
    } catch { return NaN; }
  };

  for (const line of lines) {
    const w = words(line);
    const c = w[0];
    if (PRIMS.has(c)) {
      const kind = c === 'box' ? 'box' : c === 'pcylinder' ? 'cylinder' : c === 'pcone' ? 'cone' : 'sphere';
      const nums = w.slice(2).map(evalNum);
      if (nums.some((x) => !Number.isFinite(x))) return { unsupported: `${c}: non-numeric arguments`, lines };
      const why = buildPrimitive(w[1], kind, nums);
      if (unsupported) return { unsupported, lines };
      lumpCount++;
      continue;
    }
    if (BOOLS.has(c)) {
      bools.push({ op: c, result: w[1], a: w[2], b: w[3] });
      continue;
    }
    if (c === 'trotate') {
      // trotate shape [cx cy cz] dx dy dz angle
      const name = w[1];
      const nums = w.slice(2).map(evalNum);
      if (nums.some((x) => !Number.isFinite(x))) return { unsupported: 'trotate: non-numeric arguments', lines };
      let origin = [0, 0, 0], axis, angle;
      if (nums.length === 4) { axis = nums.slice(0, 3); angle = nums[3]; }
      else if (nums.length === 7) { origin = nums.slice(0, 3); axis = nums.slice(3, 6); angle = nums[6]; }
      else return { unsupported: `trotate: ${nums.length} numbers`, lines };
      const e = seen.get(name);
      if (!e) continue;
      addFeature({ type: 'moveBody', id: `M${++n}`, bodies: [{ face: e.face }], rotate: { axis, origin, angle } });
      continue;
    }
    if (c === 'ttranslate') {
      const name = w[1];
      const nums = w.slice(2).map(evalNum);
      if (nums.length !== 3 || nums.some((x) => !Number.isFinite(x))) return { unsupported: 'ttranslate: needs dx dy dz', lines };
      const e = seen.get(name);
      if (!e) continue;
      addFeature({ type: 'moveBody', id: `M${++n}`, bodies: [{ face: e.face }], translate: nums });
      continue;
    }
    if (c === 'tscale') return { unsupported: 'tscale (not in the wire protocol)', lines };
    if (c === 'copy') {
      const e = seen.get(w[2]);
      if (!e) continue;
      addFeature({ type: 'moveBody', id: `M${++n}`, bodies: [{ face: e.face }], translate: [0, 0, 0], copy: true });
      return { unsupported: 'copy (instance naming not pinned for booleans)', lines };
    }
    if (c === 'bop') {
      if (w.length !== 3) return { unsupported: 'bop: expected two operands', lines };
      pendingBop = [w[1], w[2]];
      continue;
    }
    if (['bopcut', 'boptuc', 'bopfuse', 'bopcommon'].includes(c)) {
      if (!pendingBop) return { unsupported: `${c}: no bop operands`, lines };
      const [a, b] = pendingBop;
      const target = c === 'boptuc' ? b : a;
      const tool = c === 'boptuc' ? a : b;
      const op = c === 'bopfuse' ? 'union' : c === 'bopcommon' ? 'intersect' : 'cut';
      if (!seen.has(target) || !seen.has(tool)) return { unsupported: `${c}: operand not built (${target} ${tool})`, lines };
      const id = `C${++n}`;
      const resultLump = lumpCount - 2;
      const A = seen.get(target), B = seen.get(tool);
      addFeature({ type: 'combine', id, op, target: A.lump != null ? { lump: A.lump } : { face: A.face }, tools: [B.lump != null ? { lump: B.lump } : { face: B.face }] });
      if (op === 'cut') cutTools.push({ id, toolFace: B.face });
      seen.set(w[1], { face: A.face, lump: resultLump, feature: id, kind: 'boolean' });
      lumpCount = resultLump + 1;
      pendingBop = null;
      continue;
    }
    if (c === 'blend') {
      const rest = w.slice(3);
      if (rest.length % 2) return { unsupported: 'blend: odd argument count', lines };
      blends.push({ result: w[1], base: w[2] });
      continue;
    }
    if (c === 'checkprops') {
      const si = w.indexOf('-s');
      const vi = w.indexOf('-v');
      if (si < 0 && vi < 0) return { unsupported: 'checkprops without -s/-v', lines };
      if (si >= 0) {
        if (w[si + 1] === 'empty') wantArea = 0;
        else {
          wantArea = evalNum(w[si + 1]);
          if (!Number.isFinite(wantArea)) return { unsupported: 'checkprops area not a number', lines };
        }
      }
      if (vi >= 0) {
        if (w[vi + 1] === 'empty') wantVolume = 0;
        else {
          wantVolume = evalNum(w[vi + 1]);
          if (!Number.isFinite(wantVolume)) return { unsupported: 'checkprops volume not a number', lines };
        }
      }
      continue;
    }
    if (c === 'dset') { if (w[1]) vars[w[1]] = evalNum(w[2]); continue; }
    if (IGNORE.has(c)) continue;
    return { unsupported: `${c} (untranslated command)`, lines };
  }

  if (bools.length > 2) return { unsupported: `${bools.length} booleans`, lines };
  // Build every boolean on the two primitive lumps. The result lump keeps the
  // target's face name (union/cut/intersect all keep at least one target face
  // in the cases in this selection).
  for (const b of bools) {
    const A = seen.get(b.a), B = seen.get(b.b);
    if (!A || !B) return { unsupported: `boolean operand not built (${b.a} ${b.b})`, lines };
    const op = b.op === 'bfuse' ? 'union' : b.op === 'bcut' ? 'cut' : 'intersect';
    const id = `C${++n}`;
    // a boolean creates its result lump after the untouched lumps (see
    // combine.rs "untouched lumps first, then the result pieces")
    const resultLump = lumpCount - 2;
    const target = A.lump != null ? { lump: A.lump } : { face: A.face };
    addFeature({ type: 'combine', id, op, target, tools: [B.lump != null ? { lump: B.lump } : { face: B.face }] });
    if (op === 'cut') cutTools.push({ id, toolFace: B.face });
    const result = { face: A.face, lump: resultLump, feature: id, kind: 'boolean' };
    seen.set(b.result, result);
    lumpCount = resultLump + 1;
  }
  if (wantArea == null && wantVolume == null) return { unsupported: 'no checkprops expected value', lines };
  if (blends.length) {
    let picks = pickCache[caseName];
    if (!picks && writePicksFile) {
      try {
        picks = JSON.parse(execFileSync('python3.12', [path.join(here, 'occt_picks.py'), '--case', file], { encoding: 'utf8' }));
      } catch (e) {
        picks = { ok: false, why: e.message.split('\n')[0] };
      }
      if (picks.ok) pickCache[caseName] = picks;
    }
    if (!picks) return { unsupported: 'no cached edge picks (run --write-picks)', lines };
    if (!picks.ok) return { unsupported: `edge picks: ${picks.why}`, lines };
    if (!picks.picks.length) return { unsupported: 'blend without picks', lines };
    return { features: f.slice(), wantArea, wantVolume, lines, booleans: bools.length, cutTools, blendPicks: picks.picks };
  }
  return { features: f.slice(), wantArea, wantVolume, lines, booleans: bools.length, cutTools };
}

// ---------------------------------------------------------------- selection
const BOOL_GRIDS = ['bfuse_simple', 'bcut_simple', 'bcommon_simple'];

function listDir(dir) {
  if (!existsSync(dir)) return [];
  return readdirSync(dir).filter((x) => !x.startsWith('.')).sort();
}

const BOP_FAMILY = new Set(['bop', 'bopcut', 'boptuc', 'bopfuse', 'bopcommon']);

function select() {
  const out = [];
  const boolRoot = path.join(occt, 'tests/boolean');
  for (const g of listDir(boolRoot)) {
    const dir = path.join(boolRoot, g);
    if (!statSync(dir).isDirectory()) continue;
    for (const name of listDir(dir)) {
      const p = path.join(dir, name);
      if (!statSync(p).isFile()) continue;
      const ls = readScript(p);
      const cmds = new Set(ls.map((l) => l.split(/\s+/)[0]));
      // the exported n-ary/2D/vertex/profile cases need commands the wire
      // protocol has no equivalent for; keep the corpus to the translatable
      // solid primitive/boolean grids
      const allowed = new Set([...PRIMS, ...BOOLS, ...BOP_FAMILY, ...XFORMS, ...IGNORE, ...CHECKS]);
      if ([...cmds].some((x) => !allowed.has(x))) continue;
      if (!cmds.has('checkprops') || !([...BOOLS].some((b) => cmds.has(b)) || [...BOP_FAMILY].some((b) => cmds.has(b)))) continue;
      out.push({ group: `boolean/${g}`, name: `${g}/${name}`, file: p });
    }
  }
  // blend: only single-blend scripts that use a full primitive (no partial
  // primitives, no tscale — the wire protocol has no non-uniform scale)
  const blend = path.join(occt, 'tests/blend/simple');
  for (const name of listDir(blend)) {
    const p = path.join(blend, name);
    if (!statSync(p).isFile()) continue;
    const ls = readScript(p);
    if (!ls.some((l) => l.split(/\s+/)[0] === 'blend')) continue;
    const prims = ls.filter((l) => PRIMS.has(l.split(/\s+/)[0]));
    if (!prims.length) continue;
    const full = prims.every((l) => {
      const w = l.split(/\s+/);
      return (w[0] === 'box' && (w.length === 5 || w.length === 8)) ||
        (w[0] === 'psphere' && w.length === 3) ||
        (w[0] === 'pcylinder' && (w.length === 4 || w.length === 5)) ||
        (w[0] === 'pcone' && (w.length === 5 || w.length === 6));
    });
    if (!full) continue;
    if (ls.some((l) => l.startsWith('tscale') || l.startsWith('restore'))) continue;
    if (ls.some((l) => /^(wedge|ptorus|profile|vertex|edge|circle|mkedge|polyline|wire|mkplane|mkface|prism|revol|pipe)\b/.test(l))) continue;
    out.push({ group: 'blend/simple', name: `blend/simple/${name}`, file: p });
  }
  // draft: primitive + depouille scripts (the face pick is by DRAW's
  // exploded face name; the translator reports them unsupported until a face
  // pick format is added, but they are part of the selected frontier)
  const draft = path.join(occt, 'tests/draft/angle');
  for (const name of listDir(draft)) {
    const p = path.join(draft, name);
    if (!statSync(p).isFile()) continue;
    const ls = readScript(p);
    const cmds = new Set(ls.map((l) => l.split(/\s+/)[0]));
    const allowed = new Set([...PRIMS, ...BOOLS, ...XFORMS, ...IGNORE, ...CHECKS, 'depouille']);
    if ([...cmds].some((x) => !allowed.has(x))) continue;
    if (!cmds.has('depouille') || !cmds.has('checkprops')) continue;
    out.push({ group: 'draft/angle', name: `draft/angle/${name}`, file: p });
  }

  // fill the ~200 selection with the remaining simple blend scripts (partial
  // primitives / tori / preliminary blends); they stay visible as unsupported
  // rather than silently dropped
  if (out.length < 200) {
    for (const name of listDir(blend)) {
      if (out.length >= 200) break;
      const p = path.join(blend, name);
      if (!statSync(p).isFile()) continue;
      if (out.some((c) => c.file === p)) continue;
      const ls = readScript(p);
      const cmds = new Set(ls.map((l) => l.split(/\s+/)[0]));
      if (!cmds.has('blend') || !cmds.has('checkprops')) continue;
      out.push({ group: 'blend/simple', name: `blend/simple/${name}`, file: p });
    }
  }
  return out;
}

// ---------------------------------------------------------------- runner
const mod = new WebAssembly.Module(readFileSync(wasmPath));

function rq(k, msg) {
  const r = k.request(JSON.stringify({ id: 1, msg }));
  if (r.trap) return { trap: r.trap };
  try { return JSON.parse(r.json); } catch { return { trap: 'unparsable reply' }; }
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function distSeg(p, a, b) {
  const ab = sub(b, a), ap = sub(p, a), t = Math.max(0, Math.min(1, dot(ap, ab) / (dot(ab, ab) || 1)));
  const q = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
  const d = sub(p, q);
  return Math.hypot(d[0], d[1], d[2]);
}
// Choose the edge by midpoint AND tangent direction. A corner point sits on
// up to three edges, so distance alone can pick the wrong one; the cached pick
// carries the direction OCCT's edge has at that point.
function findEdge(body, at, dir) {
  let best = null;
  for (const e of body.edges ?? []) {
    const pts = e.pts?.length >= 2 ? e.pts : [e.mid, e.mid];
    let d = Math.hypot(...sub(at, e.mid)), dirE = null;
    for (let i = 0; i + 1 < pts.length; i++) {
      const dd = distSeg(at, pts[i], pts[i + 1]);
      if (dd <= d) { d = dd; dirE = sub(pts[i + 1], pts[i]); }
    }
    let pen = 0;
    if (dir && dirE) {
      const l = Math.hypot(...dirE);
      if (l > 0) pen = (1 - Math.abs(dot(dir, [dirE[0] / l, dirE[1] / l, dirE[2] / l]))) * 0.05;
    }
    const score = d + pen;
    if (!best || score < best.d) best = { d: score, dist: d, e };
  }
  return best && best.dist < 2e-3 ? best.e : null;
}

function edgeProbe(e, at, dir) {
  const pts = e.pts?.length >= 2 ? e.pts : [e.mid, e.mid];
  let d = Math.hypot(...sub(at, e.mid)), dirE = null;
  for (let i = 0; i + 1 < pts.length; i++) {
    const dd = distSeg(at, pts[i], pts[i + 1]);
    if (dd <= d) { d = dd; dirE = sub(pts[i + 1], pts[i]); }
  }
  let dotv = null;
  if (dir && dirE) {
    const l = Math.hypot(...dirE);
    if (l > 0) dotv = Math.abs(dot(dir, [dirE[0] / l, dirE[1] / l, dirE[2] / l]));
  }
  return { dist: d, dot: dotv };
}

function execPart(k, features) {
  const part = { format: 'ck-part', version: 1, features };
  const t0 = Date.now();
  const msg = { type: 'regen', tag: 'main', part };
  if (autoHeal != null) msg.opts = { autoHeal: true, healTol: autoHeal };
  const rr = k.request(JSON.stringify({ id: 1, msg }));
  const ms = Date.now() - t0;
  let r; try { r = JSON.parse(rr.json); } catch { r = { trap: 'unparsable reply' }; }
  return { r, ms };
}

// A cut whose tool only touches the host answers `cutNoEffect` and leaves
// the (separate) tool lump in the body. OCCT consumes the tool and leaves
// the host; express that by deleting the tool lumps and re-running.
function runWithCutFallback(k, tr, features) {
  let { r, ms } = execPart(k, features);
  if (r.trap || !r.ok) return { r, ms, features };
  let errs = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code, ...e }));
  const noEffect = errs.filter((e) => e.code === 'cutNoEffect');
  if (noEffect.length) {
    const drop = tr.cutTools.filter((t) => noEffect.some((e) => e.id === t.id));
    if (drop.length) {
      const extra = drop.map((t, i) => ({ type: 'deleteBody', id: `D${i + 1}`, bodies: [{ face: t.toolFace }] }));
      const retry = execPart(k, [...features, ...extra]);
      if (!retry.r.trap && retry.r.ok) { r = retry.r; ms += retry.ms; features = [...features, ...extra]; }
    }
  }
  return { r, ms, features };
}

function runCase(c, k) {
  const tr = translate(c.file, c.name);
  if (tr.unsupported) return { ...c, cls: 'unsupported', detail: tr.unsupported, ms: 0 };
  if (opt('--dump') && c.name.includes(opt('--dump'))) {
    console.log(JSON.stringify({ name: c.name, wantArea: tr.wantArea, wantVolume: tr.wantVolume, features: tr.features, blendPicks: tr.blendPicks ?? null }, null, 1));
    process.exit(0);
  }
  let { r, ms, features } = runWithCutFallback(k, tr, tr.features);
  if (r.trap) return { ...c, cls: 'trap', detail: r.trap, ms };
  if (!r.ok) return { ...c, cls: 'requestError', detail: r.error, ms };

  // A fillet pick is a point on an edge of the base body; the feature also
  // needs the two faces, which are only known after the base is built.
  let pickChecks = null;
  if (tr.blendPicks) {
    const body = r.result?.body;
    if (!body) return { ...c, cls: 'noBase', detail: 'blend base has no body', ms };
    const edges = [];
    pickChecks = [];
    for (const p of tr.blendPicks) {
      const e = findEdge(body, p.point, p.dir);
      if (!e) return { ...c, cls: 'missingEdge', detail: `no edge near ${p.point}`, ms };
      // the cached pick carries OCCT's midpoint and tangent; confirm the edge
      // our kernel selected is the same one (distance and direction)
      const pr = edgeProbe(e, p.point, p.dir);
      if (pr.dist > 2e-3 || (pr.dot != null && pr.dot < 0.95)) {
        return { ...c, cls: 'pickMismatch', detail: `edge ${p.edge}: dist ${pr.dist.toExponential(1)} dot ${pr.dot?.toFixed(3)}`, ms };
      }
      pickChecks.push({ edge: p.edge, dist: pr.dist, dot: pr.dot, mid: e.mid, dir: p.dir });
      const ref = { faces: [...e.faces], at: p.point };
      if (p.radius !== tr.blendPicks[0].radius) ref.radius = p.radius;
      edges.push(ref);
    }
    const fillet = { type: 'fillet', id: `B${features.length + 1}`, radius: tr.blendPicks[0].radius, edges };
    const retry = runWithCutFallback(k, tr, [...features, fillet]);
    r = retry.r; ms += retry.ms; features = retry.features;
    if (r.trap) return { ...c, cls: 'trap', detail: r.trap, ms };
    if (!r.ok) return { ...c, cls: 'requestError', detail: r.error, ms };
  }
  let errs = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code, ...e }));

  // cutNoEffect is the right answer when the tool only touches the host;
  // measure the body and let the area check decide.
  const harmless = new Set(['cutNoEffect']);
  const fatal = errs.filter((e) => !harmless.has(e.code));
  if (fatal.length) {
    const empty = fatal.every((e) => ['emptyResult', 'combineEmpty', 'cutEmpty', 'booleanEmpty', 'nothingToCut'].includes(e.code));
    if (empty && (tr.wantArea ?? 0) <= 1e-6 && (tr.wantVolume ?? 0) <= 1e-6) return { ...c, cls: 'ok', detail: 'empty', ms, want: tr.wantArea, got: 0 };
    return { ...c, cls: fatal[0].code, detail: `${fatal[0].id}: ${fatal[0].code}`, ms };
  }
  const m = rq(k, { type: 'massProps', tag: 'main', bodies: true });
  const area = m.ok ? m.result?.area ?? null : null;
  const vol = m.ok ? m.result?.volume ?? null : null;
  if (area == null) {
    if ((tr.wantArea ?? 0) <= 1e-6 && (tr.wantVolume ?? 0) <= 1e-6) return { ...c, cls: 'ok', detail: 'empty', ms, want: tr.wantArea, got: 0 };
    return { ...c, cls: 'noArea', detail: 'massProps gave no area', ms };
  }
  const v = rq(k, { type: 'validate', tag: 'main' });
  const issues = v.result?.issues ?? [];
  const tol = c.group.startsWith('blend') ? 2e-3 : 1e-4;
  const relOf = (got, want) => Math.abs(got - want) / Math.max(Math.abs(want), 1e-9);
  const areaRel = tr.wantArea == null ? null : relOf(area, tr.wantArea);
  const volRel = tr.wantVolume == null ? null : relOf(vol ?? 0, tr.wantVolume);
  const worst = Math.max(areaRel ?? 0, volRel ?? 0);
  const values = `area ${area.toPrecision(9)}${tr.wantArea == null ? '' : ` want ${tr.wantArea} (${areaRel.toExponential(1)})`}`
    + (tr.wantVolume == null ? '' : `; vol ${(vol ?? 0).toPrecision(9)} want ${tr.wantVolume} (${volRel.toExponential(1)})`);
  if (worst > tol) return { ...c, cls: 'wrongArea', detail: values, ms, want: tr.wantArea, got: area, rel: areaRel, wantVol: tr.wantVolume, gotVol: vol, relVol: volRel };
  if (issues.length) return { ...c, cls: 'invalid', detail: [...new Set(issues.map((i) => i.code))].join(','), ms, want: tr.wantArea, got: area, rel: areaRel, wantVol: tr.wantVolume, gotVol: vol, relVol: volRel };
  return { ...c, cls: 'ok', detail: values.replace(/ want .*?\)/g, ''), ms, want: tr.wantArea, got: area, rel: areaRel, wantVol: tr.wantVolume, gotVol: vol, relVol: volRel, ...(pickChecks ? { pickChecks } : {}) };
}

const cases = select().filter((c) => !only || c.name.includes(only));
const sharedKernel = new Kernel(mod);
if (writePicksFile) {
  let n = 0;
  for (const c of cases) {
    const tr = translate(c.file, c.name);
    if (tr.blendPicks) { pickCache[c.name] = { ok: true, picks: tr.blendPicks, base: c.name }; n++; }
  }
  writeFileSync(writePicksFile, JSON.stringify(pickCache, null, 1) + '\n');
  console.log(`occt_corpus: wrote ${n} blend pick sets to ${writePicksFile}`);
  process.exit(0);
}
const rows = [];
for (const c of cases) {
  const r = runCase(c, sharedKernel);
  rows.push(r);
  if (!argv.includes('--quiet')) {
    const tag = r.cls === 'ok' ? 'ok  ' : r.cls === 'unsupported' ? 'n/a ' : 'FAIL';
    console.log(`${tag} ${r.name}: ${r.cls}${r.detail ? ' ' + r.detail : ''} (${r.ms} ms)`);
  }
}
const counted = rows.filter((r) => r.cls !== 'unsupported');
const okN = counted.filter((r) => r.cls === 'ok').length;
const by = {};
for (const r of counted) by[r.cls] = (by[r.cls] ?? 0) + 1;
console.log(`\nocct_corpus: ${okN}/${counted.length} ok  ${JSON.stringify(by)}  (${rows.length - counted.length} unsupported of ${rows.length})`);

const baselineFile = opt('--baseline', path.join(here, 'occt_baseline.json'));
if (writePicksFile) { /* handled above */ }
if (argv.includes('--update-baseline')) {
  const cases = {};
  for (const r of rows) cases[r.name] = { cls: r.cls, rel: r.rel ?? null, relVol: r.relVol ?? null };
  writeFileSync(baselineFile, JSON.stringify({ version: 1, wasm: wasmPath, cases }, null, 1) + '\n');
  console.log(`occt_corpus: wrote baseline ${baselineFile} (${rows.length} cases)`);
}
if (argv.includes('--check')) {
  if (!existsSync(baselineFile)) {
    console.error(`occt_corpus: no baseline at ${baselineFile}; run --update-baseline first`);
    process.exit(2);
  }
  const base = JSON.parse(readFileSync(baselineFile, 'utf8')).cases ?? {};
  const regressions = [], newPasses = [], added = [];
  for (const r of rows) {
    const b = base[r.name];
    if (!b) { added.push(r.name); continue; }
    if (b.cls === 'ok' && r.cls !== 'ok') regressions.push(r);
    else if (b.cls !== 'ok' && r.cls === 'ok') newPasses.push(r);
  }
  const gone = Object.keys(base).filter((n) => !rows.some((r) => r.name === n));
  console.log(`\nocct_corpus check: ${regressions.length} regressions, ${newPasses.length} new passes, ${added.length} added, ${gone.length} no longer selected`);
  for (const r of regressions) console.log(`REGRESSION ${r.name}: ${r.cls} ${r.detail ?? ''}`);
  for (const r of newPasses) console.log(`new pass   ${r.name}: ${r.cls}`);
  if (regressions.length) process.exit(1);
}
if (argv.includes('--verify-picks')) {
  // The box-fillet cases have a known checkprops area and a cached OCCT pick;
  // passing them proves the edge our kernel selected is the edge OCCT filleted.
  const names = ['blend/simple/A1', 'blend/simple/A2', 'blend/simple/A3', 'blend/simple/P8', 'blend/simple/P9'];
  let bad = 0;
  for (const n of names) {
    const row = rows.find((r) => r.name === n);
    const ok = row?.cls === 'ok' && (row.pickChecks ?? []).every((p) => p.dist <= 2e-3 && (p.dot ?? 1) >= 0.95);
    console.log(`${ok ? 'ok  ' : 'FAIL'} verify-pick ${n}: ${row ? row.cls : 'not selected'} ${row?.detail ?? ''}`);
    if (!ok) bad++;
  }
  process.exit(bad ? 1 : 0);
}
const jo = opt('--json');
if (jo) writeFileSync(jo, JSON.stringify(rows, null, 1));
const mo = opt('--md');
if (mo) {
  const out = ['| group | case | result | detail | ms |', '|---|---|---|---|---|', ...rows.map((r) => `| ${r.group} | ${r.name} | ${r.cls} | ${r.detail ?? ''} | ${r.ms} |`), '', `ok ${okN}/${counted.length} ${JSON.stringify(by)}`];
  writeFileSync(mo, out.join('\n') + '\n');
}
process.exit(okN === counted.length ? 0 : 1);
