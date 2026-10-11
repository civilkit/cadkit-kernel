// Compare regen replies of the ORIGINAL kernel and the REBUILT wasm, with edge
// pts reduced to counts, reporting which body keys / result keys differ.
//
// usage: node tools/regencmp.mjs [--prefixes] [--debug] [--show key] <target> ...
//   target: a .ckpart file, or `brep` (all parts of tools/brep_cases.mjs),
//           or `brep:name`
//   --prefixes   run upTo = 1..N for a .ckpart (one line per prefix)
//   --mass       also compare massProps (values to 1e-9 relative)
//   --show key   print both sides of body[key] (faces|edges|box) on a diff
//   --trace      print both debug traces (labels only) on a diff
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';

const args = process.argv.slice(2);
const flag = (f) => { const i = args.indexOf(f); if (i < 0) return false; args.splice(i, 1); return true; };
const opt = (f) => { const i = args.indexOf(f); if (i < 0) return null; const v = args[i + 1]; args.splice(i, 2); return v; };
const loose = flag('--loose'), prefixes = flag('--prefixes'), mass = flag('--mass'), trace = flag('--trace'), show = opt('--show');
const wasmB = opt('--wasm') ?? new URL('../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url);
const A = new WebAssembly.Module(readFileSync(new URL('../../kernel.wasm', import.meta.url)));
const B = new WebAssembly.Module(readFileSync(wasmB));

const targets = [];
for (const t of args) {
  if (t === 'brep' || t.startsWith('brep:')) {
    const { parts } = await import('./brep_cases.mjs');
    for (const [n, p] of Object.entries(parts)) if (t === 'brep' || t === `brep:${n}`) targets.push({ name: n, part: p });
  } else {
    const part = JSON.parse(readFileSync(t, 'utf8'));
    const nm = t.replace(/.*\//, '').replace(/\.ckpart$/, '');
    if (prefixes) for (let k = 1; k <= part.features.length; k++) targets.push({ name: `${nm}@${k}`, part, upTo: k });
    else targets.push({ name: nm, part });
  }
}

// --loose: structure check only (numbers to 6 decimals, no mesh buffers / silhouette)
const rnd = (x) => typeof x === 'number' ? Math.round(x * 1e6) / 1e6 + 0 : Array.isArray(x) ? x.map(rnd) : x && typeof x === 'object' ? Object.fromEntries(Object.entries(x).map(([k, v]) => [k, rnd(v)])) : x;
const loosen = (b) => { if (!b) return b; delete b.mesh; delete b.silhouette; for (const e of b.edges ?? []) { delete e.mid; delete e.pts; } for (const k of ['faces', 'edges']) if (b[k]) b[k] = rnd(b[k]); if (b.box) b.box = JSON.parse(JSON.stringify(b.box), (_, v) => typeof v === 'number' ? Math.round(v * 1e2) / 1e2 + 0 : v); return b; };
const strip = (o) => { if (loose) { loosen(o?.result?.body); for (const [, sb] of o?.result?.surfaces ?? []) loosen(sb); } const b = o?.result?.body; if (b?.edges) for (const e of b.edges) e.pts = e.pts?.length; for (const [, sb] of o?.result?.surfaces ?? []) if (sb?.edges) for (const e of sb.edges) e.pts = e.pts?.length; return o; };
const rel = (x, y) => Math.abs(x - y) / (1 + Math.abs(y));
const maxdiff = (a, b) => {
  if (typeof a === 'number' && typeof b === 'number') return rel(a, b);
  if (Array.isArray(a) && Array.isArray(b) && a.length === b.length) return Math.max(0, ...a.map((x, i) => maxdiff(x, b[i])));
  if (a && b && typeof a === 'object' && typeof b === 'object') {
    const ka = Object.keys(a), kb = Object.keys(b);
    if (ka.join() !== kb.join()) return Infinity;
    return Math.max(0, ...ka.map((k) => maxdiff(a[k], b[k])));
  }
  return a === b ? 0 : Infinity;
};
const labels = (o) => (o?.trace ?? []).map((t) => `${'  '.repeat(t.depth)}${t.label}`);

let n = 0, same = 0;
for (const t of targets) {
  const msg = { type: 'regen', tag: 'main', part: t.part, ...(t.upTo ? { opts: { upTo: t.upTo } } : {}), ...(trace ? { debug: true } : {}) };
  const ka = new Kernel(A), kb = new Kernel(B);
  const [ra, rb] = [ka, kb].map((k) => k.request(JSON.stringify({ id: 1, msg })));
  if (ra.trap || rb.trap) { console.log(t.name.padEnd(16), `TRAP orig=${ra.trap ?? '-'} rebuilt=${rb.trap ?? '-'}`); n++; continue; }
  const oa = strip(JSON.parse(ra.json)), ob = strip(JSON.parse(rb.json));
  const ba = oa.result?.body, bb = ob.result?.body;
  const keys = ['box', 'faces', 'edges'].filter((k) => JSON.stringify(ba?.[k]) !== JSON.stringify(bb?.[k]));
  const notes = [];
  if (ba && bb) {
    if (ba.faces.length !== bb.faces.length) notes.push(`faces ${bb.faces.length}/${ba.faces.length}`);
    if (ba.edges.length !== bb.edges.length) notes.push(`edges ${bb.edges.length}/${ba.edges.length}`);
    else if (keys.includes('edges')) {
      const strip2 = (es) => es.map(({ pts, ...e }) => e);
      const pd = JSON.stringify(strip2(ba.edges)) === JSON.stringify(strip2(bb.edges));
      if (pd) notes.push('edges: pts counts only');
      else notes.push(`edges maxrel ${maxdiff(strip2(ba.edges).map(({ key, faces, ...e }) => e), strip2(bb.edges).map(({ key, faces, ...e }) => e)).toExponential(1)}${JSON.stringify(ba.edges.map((e) => e.key)) === JSON.stringify(bb.edges.map((e) => e.key)) ? '' : ' keys differ'}`);
    }
    if (keys.includes('box')) notes.push(`box maxrel ${maxdiff(ba.box, bb.box).toExponential(1)}`);
  } else if (!!ba !== !!bb) notes.push(`body ${bb ? 'yes' : 'no'}/${ba ? 'yes' : 'no'}`);
  const strip3 = (o) => { const r = { ...o.result }; delete r.body; return r; };
  const sfa = oa.result?.surfaces ?? [], sfb = ob.result?.surfaces ?? [];
  if (JSON.stringify(sfa) !== JSON.stringify(sfb)) {
    const ids = (s) => s.map(([id]) => id).join();
    if (ids(sfa) !== ids(sfb)) notes.push(`surfaces ours=[${ids(sfb)}] orig=[${ids(sfa)}]`);
    else for (const [i, [id, x]] of sfa.entries()) {
      const y = sfb[i][1], ks = ['box', 'faces', 'edges', 'silhouette', 'mesh'].filter((k) => JSON.stringify(x[k]) !== JSON.stringify(y[k]));
      if (ks.length) notes.push(`surface ${id}: ${ks.join('/')}${show && ks.includes(show) ? `\n   orig    ${JSON.stringify(x[show])}\n   rebuilt ${JSON.stringify(y[show])}` : ''}`);
    }
  }
  const ra2 = strip3(oa), rb2 = strip3(ob);
  const other = Object.keys({ ...ra2, ...rb2 }).filter((k) => JSON.stringify(ra2[k]) !== JSON.stringify(rb2[k]));
  if (oa.ok !== ob.ok) other.push(`ok ${ob.ok}/${oa.ok} ${ob.error ?? oa.error}`);
  if (other.includes('errors')) notes.push(`errors ours=${JSON.stringify(rb2.errors)} orig=${JSON.stringify(ra2.errors)}`);
  if (mass && ba && bb) {
    const q = JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } });
    const ma = JSON.parse(ka.request(q).json).result, mb = JSON.parse(kb.request(q).json).result;
    const d = Math.max(...['volume', 'area', 'com'].map((k) => maxdiff(mb?.[k], ma?.[k])));
    notes.push(d === 0 ? 'mass exact' : `mass maxrel ${d.toExponential(1)}`);
  }
  n++;
  const ok = !keys.length && !other.length;
  if (ok) same++;
  console.log(t.name.padEnd(16), ok ? 'MATCH' : `diff: ${[...keys, ...other].join(',')}`, notes.length ? `| ${notes.join('; ')}` : '');
  if (!ok && show && ba && bb) console.log(`  orig    ${JSON.stringify(ba[show])}\n  rebuilt ${JSON.stringify(bb[show])}`);
  if (!ok && trace) console.log(`  orig trace:\n    ${labels(oa).join('\n    ')}\n  rebuilt trace:\n    ${labels(ob).join('\n    ')}`);
}
console.log(`${same}/${n} match (body box/faces/edges with pts as counts, plus non-body result keys)`);
