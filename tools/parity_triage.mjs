// Per-case parity triage of the brep set: original vs rebuilt wasm.
// usage: node tools/parity_triage.mjs [--wasm path] [--md | --json]   (default: one line per differing case)
// For every brep case whose faces / edges / box / mass differ from the original,
// prints the details needed to classify it (see docs/parity-triage.md).
import { readFileSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import { parts } from './brep_cases.mjs';
import { oracles } from './oracles/index.mjs';
import { checkOracle } from './oracles/check.mjs';
const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const wi = args.indexOf('--wasm');
const wasmB = wi >= 0 ? args[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const A = new WebAssembly.Module(readFileSync(realpathSync(path.join(here, '../../kernel.wasm'))));
const B = new WebAssembly.Module(readFileSync(wasmB));
const rel = (x, y) => Math.abs(x - y) / (1 + Math.abs(y));
const close = (a, b, t = 1e-9) => typeof a === 'number' && typeof b === 'number' ? rel(a, b) <= t : Array.isArray(a) && Array.isArray(b) ? a.length === b.length && a.every((x, i) => close(x, b[i], t)) : a === b;
// Structural/numeric diff of two JSON values: numeric leaves differing -> max abs diff;
// anything else (type, key set, array length, string, bool) -> signature path (indices removed).
function jdiff(a, b, p, acc) {
  if (typeof a === 'number' && typeof b === 'number') { const e = Math.abs(a - b); if (e > 0) { acc.num = Math.max(acc.num, e); acc.by[p] = Math.max(acc.by[p] ?? 0, e); } return; }
  if (Array.isArray(a) && Array.isArray(b)) {
    if (a.length !== b.length) { acc.sig.add(`${p}[].len ${a.length > b.length ? '>' : '<'}`); return; }
    a.forEach((x, i) => jdiff(x, b[i], p + '[]', acc)); return;
  }
  if (a && b && typeof a === 'object' && typeof b === 'object' && !Array.isArray(a) && !Array.isArray(b)) {
    for (const k of new Set([...Object.keys(a), ...Object.keys(b)])) {
      if (!(k in a)) acc.sig.add(`${p}.${k} only-ours`); else if (!(k in b)) acc.sig.add(`${p}.${k} only-oracle`); else jdiff(a[k], b[k], `${p}.${k}`, acc);
    } return;
  }
  if (a !== b) acc.sig.add(`${p} differs`);
}
// Mesh-derived volume (divergence theorem), area, centroid-free bbox from a regen reply (independent of massProps).
function meshStats(r, j) {
  const m = j?.result?.body?.mesh; if (!m) return null;
  const base = (4 + new DataView(r.bytes.buffer).getUint32(0, true) + 7) & ~7;
  const get = (v) => { const C = { f32: Float32Array, i32: Int32Array }[v.type]; return new C(r.bytes.buffer.slice(base + v.offset, base + v.offset + v.len * C.BYTES_PER_ELEMENT)); };
  const pos = get(m.positions ?? m.pos), tf = get(m.triFace);
  let vol = 0, area = 0; const mn = [1e300, 1e300, 1e300], mx = [-1e300, -1e300, -1e300];
  for (let i = 0; i < tf.length; i++) {
    const o = i * 9, P = [0, 3, 6].map((k) => [pos[o + k], pos[o + k + 1], pos[o + k + 2]]);
    for (const p of P) for (let c = 0; c < 3; c++) { mn[c] = Math.min(mn[c], p[c]); mx[c] = Math.max(mx[c], p[c]); }
    const [a, b, c] = P, ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    const n = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]];
    area += 0.5 * Math.hypot(...n); vol += (a[0] * n[0] + a[1] * n[1] + a[2] * n[2]) / 6;
  }
  return { vol, area, min: mn, max: mx, tris: tf.length };
}
const out = [];
for (const [name, part] of Object.entries(parts)) {
  const ka = new Kernel(A), kb = new Kernel(B);
  const body = JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } });
  const ra = ka.request(body), rb = kb.request(body);
  const pa = ra.trap ? null : JSON.parse(ra.json), pb = rb.trap ? null : JSON.parse(rb.json);
  const ba = pa?.result?.body, bb = pb?.result?.body;
  const cnt = (b) => b?.edges?.map(({ pts, ...e }) => ({ ...e, pts: pts.length }));
  const d = { name, faces: JSON.stringify(ba?.faces) === JSON.stringify(bb?.faces), edges: JSON.stringify(cnt(ba)) === JSON.stringify(cnt(bb)), box: JSON.stringify(ba?.box) === JSON.stringify(bb?.box) };
  d.oracleErrs = pa?.result?.errors?.length ?? (pa ? 0 : 'trap'); d.oursErrs = pb?.result?.errors?.length ?? (pb ? 0 : 'trap');
  { const acc = { num: 0, sig: new Set(), by: {} };
    if (ba && bb) { jdiff(ba.faces, bb.faces, 'faces', acc); jdiff(cnt(ba), cnt(bb), 'edges', acc); jdiff(ba.box, bb.box, 'box', acc); }
    d.maxNum = acc.num; d.by = acc.by; d.sig = [...acc.sig].sort(); }
  d.meshA = ra.trap ? null : meshStats(ra, pa); d.meshB = rb.trap ? null : meshStats(rb, pb);
  d.nf = [ba?.faces?.length, bb?.faces?.length]; d.ne = [ba?.edges?.length, bb?.edges?.length];
  d.boxA = ba?.box; d.boxB = bb?.box;
  d.errA = JSON.stringify(pa?.result?.errors ?? pa?.error ?? null).slice(0, 120); d.errB = JSON.stringify(pb?.result?.errors ?? pb?.error ?? null).slice(0, 120);
  if (ba && bb) {
    const q = JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } });
    const ma = JSON.parse(ka.request(q).json || 'null')?.result, mb = JSON.parse(kb.request(q).json || 'null')?.result;
    d.mass = ['volume', 'area', 'com'].every((k) => close(mb?.[k], ma?.[k]));
    d.mass6 = ['volume', 'area', 'com'].every((k) => close(mb?.[k], ma?.[k], 1e-6));
    d.volA = ma?.volume; d.volB = mb?.volume; d.areaA = ma?.area; d.areaB = mb?.area; d.comA = ma?.com; d.comB = mb?.com;
    d.volRel = Math.abs(mb.volume - ma.volume) / (Math.abs(ma.volume) || 1);
    d.oracleA = oracles[name] ? checkOracle(oracles[name], ma, ba.box) : null;
    d.oracle = oracles[name] ? checkOracle(oracles[name], mb, bb.box) : null;
    const vr = JSON.parse(kb.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json || 'null')?.result;
    d.valid = vr?.issues?.length ?? 'na';
    const vra = JSON.parse(ka.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json || 'null')?.result;
    d.validA = vra?.issues?.length ?? 'na';
    const kinds = (b) => { const m = {}; for (const f of b.faces) { const k = f.surf?.type ?? f.kind ?? f.type ?? '?'; m[k] = (m[k] ?? 0) + 1; } return m; };
    d.fkinds = [kinds(ba), kinds(bb)];
  } else d.mass = !ba && !bb;
  out.push(d);
}
// ---- classification (see docs/parity-triage.md for the rules and the evidence behind each group)
const GROUPS = {
  MESHBUG: ['c', 'tessellation defect: our mesh disagrees with our own massProps (fillet torus band folded over the period edge); original mesh is consistent'],
  TWIST: ['a', 'original ignores twist (un-twisted body); ours matches the closed-form twist oracle (volume, com, box)'],
  MASS_CF: ['a', 'mass props differ; the closed form is known and ours matches it to 1e-15 (hand-labelled)'],
  MASS_A: ['a', 'mass props differ >1e-9; ours agrees >=10x better with an independent integral (its own mesh); weak evidence, no closed form'],
  MASS_TOL: ['b', 'mass props differ by quadrature/fit tolerance (1e-9..4e-4); neither side proven closer'],
  TOPO: ['b', 'face/edge split policy differs (fullRound halves, curvature-fillet edge split, twist ring); volume, area, com equal'],
  META: ['b', 'edge/face classification metadata differs (line/circle flags, surface kind, circle normal sign, seam start); geometry identical'],
  ORDER: ['b', 'face/edge list order or edge-name pairing differs; same entities'],
  SAMPLING: ['b', 'edge polyline density / mesh-sampled box differs (tessellation of curved edges and faces)'],
  TIE: ['b', 'ambiguous input resolved differently: variable fillet picked at the exact midpoint of its edge ("end nearer the pick" is a tie); both results valid, same volume'],
  NOISE: ['b', 'floating-point noise <=1e-9 in edge mid / normals / circle params; ours is usually the exactly-rounded one'],
};
const MANUAL = { varTwoEdges: 'TIE', chamferCorner3: 'MASS_CF' };
const mrel = (m, v) => (m && v ? Math.abs(m.vol - v) / Math.abs(v) : 0);
function classify(d) {
  const labels = new Set();
  const ours = mrel(d.meshB, d.volB), orig = mrel(d.meshA, d.volA);
  const oursA = d.meshB ? Math.abs(d.meshB.area - d.areaB) / d.areaB : 0, origA = d.meshA ? Math.abs(d.meshA.area - d.areaA) / d.areaA : 0;
  if (d.meshB && ((ours > 2e-3 && ours > 3 * orig) || (oursA > 2e-3 && oursA > 3 * origA))) labels.add('MESHBUG');
  if (d.oracleA?.length && d.oracle?.length === 0) labels.add('TWIST');
  if (d.mass === false) {
    const dd = Math.max(d.volRel, rel(d.areaB, d.areaA), ...d.comA.map((c, i) => rel(d.comB[i], c)));
    if (ours * 10 < orig && orig > 1e-5 && !labels.has('TWIST')) labels.add('MASS_A'); else if (!labels.has('TWIST')) labels.add('MASS_TOL');
    d.massMax = dd;
  }
  if (d.nf[0] !== d.nf[1] || d.ne[0] !== d.ne[1]) labels.add('TOPO');
  const sig = d.sig.join(';');
  if (/kind differs|line only|circle only|planar differs|normal only|circle\.n|circle\.c/.test(sig) || Object.keys(d.by).some((k) => /circle\.n/.test(k)) || Object.keys(d.by).some((k) => k.endsWith('circle.c[]') && d.by[k] >= 1e-9)) labels.add('META');
  if (/key differs|id differs|faces\[\] differs|entity/.test(sig)) labels.add('ORDER');
  const big = (re) => Object.entries(d.by).some(([k, v]) => re.test(k) && v >= 1e-9);
  if (big(/pts$/) || big(/box/) || big(/mid/)) labels.add('SAMPLING');
  if (Object.entries(d.by).some(([k, v]) => v > 0 && v < 1e-9) && labels.size === 0) labels.add('NOISE');
  if (labels.size === 0 && d.sig.length === 0 && Object.keys(d.by).length === 0) labels.add('NOISE');
  // hand-labelled (documented in docs/parity-triage.md)
  if (MANUAL[d.name]) { labels.delete('MASS_A'); labels.delete('MASS_TOL'); labels.add(MANUAL[d.name]); }
  const order = ['MESHBUG', 'TWIST', 'MASS_CF', 'TIE', 'MASS_A', 'MASS_TOL', 'TOPO', 'META', 'ORDER', 'SAMPLING', 'NOISE'];
  return { labels: order.filter((l) => labels.has(l)), primary: order.find((l) => labels.has(l)) };
}
const diffs = out.filter((d) => !d.faces || !d.edges || !d.box || !d.mass);
for (const d of diffs) Object.assign(d, classify(d));
const f1 = (x) => (x === undefined || x === null ? '-' : x.toExponential(0));
if (args.includes('--md')) {
  const prim = {}, multi = {};
  for (const d of diffs) { (prim[d.primary] ??= []).push(d); for (const l of d.labels) (multi[l] ??= []).push(d); }
  const ordered = Object.keys(GROUPS).filter((g) => multi[g]).sort((a, b) => multi[b].length - multi[a].length);
  console.log(`cases ${out.length}, differing (faces|edges|box|mass) ${diffs.length}: faces ${out.filter((d) => !d.faces).length}, edges ${out.filter((d) => !d.edges).length}, box ${out.filter((d) => !d.box).length}, mass ${out.filter((d) => !d.mass).length}\n`);
  console.log('| group | class | cases (any label) | cases (primary) | meaning |\n|---|---|---|---|---|');
  for (const g of ordered) console.log(`| ${g} | ${GROUPS[g][0]} | ${multi[g].length} | ${prim[g]?.length ?? 0} | ${GROUPS[g][1]} |`);
  const byClass = {}; for (const d of diffs) { const c = GROUPS[d.primary]?.[0] ?? '?'; byClass[c] = (byClass[c] ?? 0) + 1; }
  console.log('\nprimary class counts: ' + JSON.stringify(byClass) + '\n');
  console.log('| case | differs | primary | all labels | nf O/U | ne O/U | max num diff | mass max rel | vol rel | meshVol vs mass O/U | oracle O/U |\n|---|---|---|---|---|---|---|---|---|---|---|');
  for (const d of diffs) console.log(`| ${d.name} | ${d.faces ? '' : 'F'}${d.edges ? '' : 'E'}${d.box ? '' : 'B'}${d.mass ? '' : 'M'} | ${d.primary} (${GROUPS[d.primary]?.[0]}) | ${d.labels.join(',')} | ${d.nf.join('/')} | ${d.ne.join('/')} | ${f1(d.maxNum)} | ${d.mass ? '-' : f1(d.massMax)} | ${f1(d.volRel)} | ${f1(mrel(d.meshA, d.volA))}/${f1(mrel(d.meshB, d.volB))} | ${d.oracle === null ? '-' : (d.oracleA.length ? 'fail' : 'ok') + '/' + (d.oracle.length ? 'fail' : 'ok')} |`);
} else if (args.includes('--json')) console.log(JSON.stringify(out, null, 1));
else for (const d of diffs) console.log(`${d.name}\t${d.faces ? '' : 'F'}${d.edges ? '' : 'E'}${d.box ? '' : 'B'}${d.mass ? '' : 'M'}\t${d.primary}\t${d.labels.join(',')}\tnf=${d.nf}\tne=${d.ne}\tnum=${f1(d.maxNum)}\tvolrel=${f1(d.volRel)}\toracle=${d.oracle === null ? '-' : (d.oracleA.length ? 'fail' : 'ok') + '/' + (d.oracle.length ? 'fail' : 'ok')}`);
