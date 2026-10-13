// Regen cache correctness + timing (docs/regen-cache.md).
//
// A warm regen (cache hit on a document prefix) must be BYTE-IDENTICAL to a
// cold regen of the same document in a fresh kernel instance. Tests:
//   1. identity     every case + BenchCAD part: cold, then again in the same
//                   instance (full hit): identical bytes, fullHit reported;
//                   cache-on cold == opts.noCache cold
//   2. edits        change a feature at the start / middle / end, remove the
//                   last feature, remove then re-add (undo), swap two adjacent
//                   features, doc-level renames: each warm reply == cold reply
//                   of the edited document in a fresh instance
//   3. preview      `preview` after `main` reuses the prefix (cacheStats)
//   4. eviction     tiny cap (opts.cacheCap): results stay identical
//   5. options      sketchStatus / variables change never replay stale state
//   6. noCache      bypasses the cache
//
// usage: node tools/regen_cache_check.mjs [--wasm path] [--only substr] [--slow] [--bench] [--mem]
//   --slow   also run the 140 s largeSampleFull cold comparisons
//   --bench  print the timing table (large sample sequence + 50-part cold/warm)
//   --mem    wasm memory growth over many regens
import { readFileSync, readdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (f) => { const i = args.indexOf(f); if (i < 0) return null; const v = args[i + 1]; args.splice(i, 2); return v; };
const flag = (f) => { const i = args.indexOf(f); if (i < 0) return false; args.splice(i, 1); return true; };
const wasmPath = opt('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const only = opt('--only');
const slow = flag('--slow'), bench = flag('--bench'), memTest = flag('--mem');
const MOD = new WebAssembly.Module(readFileSync(wasmPath));

// ------------------------------------------------------------ documents
const docs = [];
const casesDir = path.join(here, 'cases');
for (const f of readdirSync(casesDir).filter((f) => f.endsWith('.ckpart')).sort()) {
  docs.push({ name: f.replace('.ckpart', ''), part: JSON.parse(readFileSync(path.join(casesDir, f), 'utf8')), heavy: f.startsWith('largeSampleFull') });
}
const benchDir = path.join(here, 'bench/benchcad');
for (const f of readdirSync(benchDir).filter((f) => f.endsWith('.ckpart.json')).sort()) {
  docs.push({ name: f.replace('.ckpart.json', ''), part: JSON.parse(readFileSync(path.join(benchDir, f), 'utf8')), heavy: false });
}
const selected = docs.filter((d) => (!only || d.name.includes(only)) && (slow || bench || !d.heavy || (only && d.name.includes(only) && slow)));
const light = selected.filter((d) => !d.heavy);

// ------------------------------------------------------------- helpers
const clone = (x) => JSON.parse(JSON.stringify(x));
const fresh = () => new Kernel(MOD);
function req(k, msg) {
  const r = k.request(JSON.stringify({ id: 1, msg }));
  if (r.trap) throw new Error('trap: ' + r.trap);
  return r;
}
const regen = (k, part, opts, tag = 'main') => req(k, { type: 'regen', tag, part, ...(opts ? { opts } : {}) });
const same = (a, b) => a.bytes.length === b.bytes.length && Buffer.compare(Buffer.from(a.bytes), Buffer.from(b.bytes)) === 0;
const align8 = (n) => (n + 7) & ~7;
/** reply with `cacheStats` removed: [json text, binary tail] */
function strip(r) {
  const o = JSON.parse(r.json);
  const stats = o.result?.cacheStats;
  if (o.result) delete o.result.cacheStats;
  const jl = new DataView(r.bytes.buffer, r.bytes.byteOffset).getUint32(0, true);
  return { text: JSON.stringify(o), tail: Buffer.from(r.bytes.subarray(align8(4 + jl))).toString('base64'), stats };
}
const sameStripped = (a, b) => { const x = strip(a), y = strip(b); return x.text === y.text && x.tail === y.tail; };
const stats = (r) => JSON.parse(r.json).result.cacheStats;
const ms = (f) => { const t = performance.now(); const v = f(); return [v, performance.now() - t]; };

let fails = 0, checks = 0;
function check(ok, what) { checks++; if (!ok) { fails++; console.log('FAIL', what); } }

// ----------------------------------------------------------- mutations
const SKIP_KEYS = new Set(['id', 'num', 'seq', 'order', 'version']);
const PREFER = ['depth', 'offset', 'angle', 'r', 'radius', 'distance', 'count', 'height', 'width', 'x', 'y'];
function numLeaves(o, p = [], out = []) {
  if (typeof o === 'number') out.push(p);
  else if (Array.isArray(o)) o.forEach((v, i) => numLeaves(v, [...p, i], out));
  else if (o && typeof o === 'object') for (const [k, v] of Object.entries(o)) if (!SKIP_KEYS.has(k)) numLeaves(v, [...p, k], out);
  return out;
}
function tweak(part, idx) {
  const d = clone(part);
  for (let j = 0; j < d.features.length; j++) {
    const f = d.features[(idx + j) % d.features.length];
    const leaves = numLeaves(f);
    if (!leaves.length) continue;
    const rank = (p) => { const i = PREFER.indexOf(String(p[p.length - 1])); return i < 0 ? 99 : i; };
    const leaf = [...leaves].sort((a, b) => rank(a) - rank(b))[0];
    let o = f; for (const k of leaf.slice(0, -1)) o = o[k];
    const key = leaf[leaf.length - 1];
    o[key] = o[key] === 0 ? 1.5 : o[key] * 1.25;
    return { part: d, at: (idx + j) % d.features.length };
  }
  return { part: d, at: -1 };
}
const dropLast = (part) => { const d = clone(part); d.features.pop(); return d; };
function swapAdj(part, i) { const d = clone(part); [d.features[i], d.features[i + 1]] = [d.features[i + 1], d.features[i]]; return d; }

// ----------------------------------------------------------- test groups
function testIdentity(d) {
  const k = fresh();
  const cold = regen(k, d.part);
  const warm = regen(k, d.part);
  check(same(cold, warm), `${d.name}: warm (full hit) != cold`);
  const st = stats(regen(k, d.part, { cacheStats: true }));
  const n = d.part.features.length;
  check(st.fullHit && st.reusedFeatures === n && st.ranFeatures === 0, `${d.name}: expected full hit ${JSON.stringify(st)}`);
  const nc = regen(fresh(), d.part, { noCache: true });
  check(same(cold, nc), `${d.name}: cache-on cold != noCache cold`);
  const nst = stats(regen(k, d.part, { noCache: true, cacheStats: true }));
  check(nst.enabled === false && nst.reusedFeatures === 0, `${d.name}: noCache still used the cache`);
  return cold;
}

function testEdits(d, cold) {
  const n = d.part.features.length;
  if (n < 2) return;
  const variants = [];
  for (const [label, idx] of [['edit-start', 0], ['edit-mid', n >> 1], ['edit-end', n - 1]]) variants.push([label, tweak(d.part, idx).part]);
  variants.push(['remove-last', dropLast(d.part)]);
  variants.push(['undo', d.part]);
  if (n >= 4) {
    let i = Math.max(1, (n >> 1) - 1);
    const t = (j) => d.part.features[j]?.type;
    for (let j = 1; j < n - 1; j++) if (t(j) === 'sketch' && t(j + 1) === 'sketch') { i = j; break; }
    variants.push(['swap-adjacent', swapAdj(d.part, i)]);
  }
  const renamed = clone(d.part); renamed.name = (renamed.name ?? '') + ' renamed'; renamed.seq = 999; renamed.counters = { sketch: 77 };
  variants.push(['doc-rename', renamed]);
  const warmK = fresh();
  regen(warmK, d.part); // main state
  for (const [label, doc] of variants) {
    const warm = regen(warmK, doc);
    const ref = label === 'undo' ? cold : regen(fresh(), doc);
    check(same(warm, ref), `${d.name}: ${label} warm != cold`);
  }
  // thread-local geometry memos live across regens: a noCache regen in this used
  // instance must still equal the cold one (guards against hidden state)
  for (const [label, doc] of variants) {
    const ref = label === 'undo' ? cold : regen(fresh(), doc);
    check(same(regen(warmK, doc, { noCache: true }), ref), `${d.name}: ${label} noCache in used instance != cold`);
  }
  // the same edits again on the now-populated instance (all full hits)
  for (const [label, doc] of variants) {
    const warm = regen(warmK, doc);
    const ref = label === 'undo' ? cold : regen(fresh(), doc);
    check(same(warm, ref), `${d.name}: ${label} (repeat) warm != cold`);
  }
}

function testPreview(d) {
  const n = d.part.features.length;
  if (n < 3) return;
  const k = fresh();
  regen(k, d.part, null, 'main');
  const { part: ed, at } = tweak(d.part, n - 1);
  if (at < 0) return;
  const r = regen(k, ed, { cacheStats: true }, 'preview');
  const st = strip(r).stats;
  check(st && st.reusedFeatures === at && st.ranFeatures === n - at, `${d.name}: preview reused ${st?.reusedFeatures}, expected ${at} (edit at ${at}/${n})`);
  check(sameStripped(r, regen(fresh(), ed, { cacheStats: true }, 'preview')), `${d.name}: preview != cold`);
  // main is untouched by the preview regen and still a full hit
  const again = regen(k, d.part, { cacheStats: true }, 'main');
  check(strip(again).stats.fullHit, `${d.name}: main after preview not a full hit`);
}

function testEviction(d) {
  const n = d.part.features.length;
  if (n < 4) return;
  const k = fresh();
  const docsSeq = [d.part, tweak(d.part, 0).part, tweak(d.part, n >> 1).part, d.part, dropLast(d.part), tweak(d.part, n - 1).part, d.part];
  // cap of roughly one third of what the unbounded cache needs for the first doc
  regen(k, d.part, { cacheCap: 1 << 40, cacheStats: true });
  const full = stats(regen(k, tweak(d.part, 2).part, { cacheStats: true })).bytes;
  const cap = Math.max(4096, Math.floor(full / 3));
  const k2 = fresh();
  let evicted = 0;
  for (const doc of docsSeq) {
    const r = regen(k2, doc, { cacheCap: cap, cacheStats: true });
    const s = strip(r).stats;
    evicted = s.evicted;
    check(s.bytes <= cap, `${d.name}: eviction cache bytes ${s.bytes} > cap ${cap}`);
    check(sameStripped(r, regen(fresh(), doc, { cacheStats: true })), `${d.name}: eviction result != cold`);
  }
  check(evicted > 0, `${d.name}: eviction test evicted nothing (cap ${cap}, full ${full})`);
  // cap 0 behaves as no cache
  const k3 = fresh();
  const a = regen(k3, d.part, { cacheCap: 0 });
  check(same(a, regen(k3, d.part, { cacheCap: 0 })), `${d.name}: cap 0 differs`);
  check(same(a, regen(fresh(), d.part, { noCache: true })), `${d.name}: cap 0 != noCache`);
}

function testOptions(d) {
  const k = fresh();
  regen(k, d.part);
  for (const o of [{ sketchStatus: true }, { solveSketches: true }, { upTo: Math.max(1, d.part.features.length >> 1) }]) {
    const warm = regen(k, d.part, o);
    check(same(warm, regen(fresh(), d.part, o)), `${d.name}: opts ${JSON.stringify(o)} warm != cold`);
    check(same(regen(k, d.part, o), warm), `${d.name}: opts ${JSON.stringify(o)} repeat differs`);
  }
  // after the option variants the plain doc must still reply as cold
  check(same(regen(k, d.part), regen(fresh(), d.part)), `${d.name}: plain after option variants differs`);
  // variables (document level) change the key of everything
  const dv = clone(d.part); dv.variables = [...(dv.variables ?? []), { name: 'zz_cache', expr: '1' }];
  check(same(regen(k, dv), regen(fresh(), dv)), `${d.name}: variables edit warm != cold`);
  const sv = strip(regen(k, dv, { cacheStats: true })).stats;
  check(sv.fullHit, `${d.name}: repeated variables doc not a full hit`);
}

// ------------------------------------------------------------------ run
if (!bench && !memTest) {
  let t0 = performance.now();
  for (const d of light) {
    const cold = testIdentity(d);
    testEdits(d, cold);
    testPreview(d);
    testEviction(d);
    testOptions(d);
    console.log(`${d.name.padEnd(28)} ${d.part.features.length} features ok-so-far=${fails === 0} (${((performance.now() - t0) / 1000).toFixed(1)}s)`);
    t0 = performance.now();
  }
  if (slow) for (const d of selected.filter((d) => d.heavy)) {
    const n = d.part.features.length;
    const k = fresh();
    const cold = regen(k, d.part);
    check(same(cold, regen(k, d.part)), `${d.name}: warm != cold`);
    for (const [label, doc] of [['edit-mid', tweak(d.part, n >> 1).part], ['edit-2', tweak(d.part, 1).part], ['remove-last', dropLast(d.part)], ['undo', d.part]]) {
      const [warm, t] = ms(() => regen(k, doc));
      const ref = label === 'undo' ? cold : regen(fresh(), doc);
      check(same(warm, ref), `${d.name}: ${label} warm != cold`);
      console.log(`${d.name} ${label}: ${t.toFixed(0)} ms warm, identical=${same(warm, ref)}`);
    }
  }
  console.log(fails ? `\n${fails} FAILED of ${checks} checks` : `\nregen cache ok: ${checks} checks`);
  process.exit(fails ? 1 : 0);
}

// ---------------------------------------------------------------- bench
if (bench) {
  const row = (label, t, extra = '') => console.log(`${label.padEnd(34)} ${t.toFixed(0).padStart(9)} ms  ${extra}`);
  const hd = docs.find((d) => d.heavy);
  if (hd && (!only || hd.name.includes(only))) {
    const n = hd.part.features.length;
    const k = fresh();
    console.log(`large sample (${n} features), one kernel instance, wasm`);
    const seq = [['cold', hd.part], ['remove last feature', dropLast(hd.part)], ['undo (put it back)', hd.part], ['edit middle feature', tweak(hd.part, n >> 1).part], ['edit feature 2', tweak(hd.part, 1).part], ['edit feature 2 again (repeat)', tweak(hd.part, 1).part]];
    for (const [label, doc] of seq) {
      const [r, t] = ms(() => regen(k, doc, { cacheStats: true }));
      const s = strip(r).stats;
      row(label, t, `reused ${s.reusedFeatures}/${s.reusedFeatures + s.ranFeatures}  cache ${(s.bytes / 1e6).toFixed(1)} MB / ${s.entries} entries  wasm mem ${(k.x.memory.buffer.byteLength / 1e6).toFixed(0)} MB`);
    }
  }
  const parts = docs.filter((d) => !d.heavy && d.name.match(/_\d{6}$/) && (!only || d.name.includes(only)));
  let tc = 0, tn = 0, tw = 0;
  console.log(`\n${parts.length} BenchCAD parts: cold (cache on) vs cold (noCache) vs warm (full hit), per-part fresh instance`);
  for (const d of parts) {
    const kn = fresh(); const [, a] = ms(() => regen(kn, d.part, { noCache: true }));
    const k = fresh(); const [, b] = ms(() => regen(k, d.part));
    const [, c] = ms(() => regen(k, d.part));
    tn += a; tc += b; tw += c;
    console.log(`${d.name.padEnd(28)} noCache ${a.toFixed(0).padStart(7)}  cold ${b.toFixed(0).padStart(7)}  warm ${c.toFixed(0).padStart(7)}`);
  }
  console.log(`TOTAL noCache ${tn.toFixed(0)} ms, cold ${tc.toFixed(0)} ms (${((tc / tn - 1) * 100).toFixed(1)}% overhead), warm ${tw.toFixed(0)} ms`);
}

// ------------------------------------------------------------------ mem
if (memTest) {
  const pick = docs.find((d) => d.name.startsWith(only ?? 'spur_gear')) ?? docs.find((d) => !d.heavy);
  const n = pick.part.features.length;
  for (const noCache of [true, false]) {
    const k = fresh();
    const mem = () => (k.x.memory.buffer.byteLength / 1e6).toFixed(0);
    const log = [];
    for (let i = 0; i < 60; i++) {
      const doc = tweak(pick.part, i % n).part;
      // vary the doc each round so the cache keeps filling
      const d2 = tweak(doc, (i * 7 + 3) % n).part;
      const r = regen(k, d2, { noCache, cacheStats: true });
      if (i % 10 === 9) log.push(`${i + 1}: ${mem()} MB (cache ${(strip(r).stats.bytes / 1e6).toFixed(1)} MB)`);
    }
    console.log(`${pick.name} noCache=${noCache}: wasm memory after N regens: ${log.join('  ')}`);
  }
}
