// Seeded schema-valid crash/hang fuzzer. One worker per case; a trap/hang
// kills only that case. Then shrink every distinct trap/hang.
import { readFileSync, writeFileSync } from 'node:fs';
import { Worker } from 'node:worker_threads';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', fileURLToPath(new URL('../../kit/apps/cad/public/kernel/ck_wasm.wasm', import.meta.url)));
const n = Number(opt('--cases', '5000'));
const seed = Number(opt('--seed', '7'));
const onlyShrink = argv.includes('--no-shrink');
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'crash_fuzz_baseline.json')));
const timeout = Number(opt('--timeout', '1500'));
const workers = Number(opt('--workers', '8'));
// --budget-ms N: regen with opts.featureTimeMs = N and regenTimeMs = 2N (the kernel's time budgets); a hang is then a case
// that runs past --timeout anyway (set it above 2N plus the unchecked remainder)
const budgetMs = Number(opt('--budget-ms', '0'));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const maybe = (p) => rnd() < p;
const pick = (a) => a[Math.floor(rnd() * a.length)];
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
const circ = (id, c, r) => ({ type: 'circle', id, c, r });

function runOne(features, t = timeout) {
  return new Promise((resolve) => {
    const w = new Worker(new URL('./crash_fuzz_worker.mjs', import.meta.url), { workerData: { wasm, features, budgetMs } });
    let done = false;
    const finish = (x) => { if (done) return; done = true; clearTimeout(timer); w.terminate().catch(() => {}); resolve(x); };
    const timer = setTimeout(() => finish({ cls: 'hang', last: features[features.length - 1]?.type ?? 'none' }), t);
    w.on('message', (m) => finish(m));
    w.on('error', (e) => finish({ cls: 'trap', trap: String(e), last: features[features.length - 1]?.type ?? 'none' }));
    w.on('exit', (code) => { if (code !== 0 && !done) finish({ cls: 'trap', trap: `worker exit ${code}`, last: features[features.length - 1]?.type ?? 'none' }); });
  });
}

async function mapLimit(items, limit, fn) {
  const out = new Array(items.length); let next = 0;
  async function worker() { while (true) { const i = next++; if (i >= items.length) return; out[i] = await fn(items[i], i); } }
  await Promise.all(Array.from({ length: Math.min(limit, items.length) }, worker));
  return out;
}

function baseFeatures(kind) {
  const w = pickSize(), d = pickSize(), h = pickSize();
  const f = [];
  let id = 0;
  const sid = () => `S${++id}`, fid = () => `F${++id}`;
  if (kind === 'box') {
    const P = [[0,0],[w,0],[w,d],[0,d]].map(([x,y],i)=>pt(`p${i}`,x,y));
    const E = [0,1,2,3].map((i)=>line(`e${i+1}`,`p${i}`,`p${(i+1)%4}`));
    f.push({ type:'sketch', id:sid(), plane:{kind:'std',id:'front'}, points:P, entities:E, constraints:[] });
    f.push({ type:'extrude', id:fid(), op:'add', sketch:'S1', end:'blind', depth:h });
  } else if (kind === 'cyl') {
    const r = pickSize();
    f.push({ type:'sketch', id:sid(), plane:{kind:'std',id:'front'}, points:[pt('c',0,0)], entities:[circ('e1','c',r)], constraints:[] });
    f.push({ type:'extrude', id:fid(), op:'add', sketch:'S1', end:'blind', depth:h });
  } else if (kind === 'loft') {
    const r1 = pickSize(), r2 = pickSize(), z = Math.abs(rr(0.001, 40));
    f.push({ type:'sketch', id:sid(), plane:{kind:'std',id:'front'}, points:[pt('c',0,0)], entities:[circ('e1','c',r1)], constraints:[] });
    f.push({ type:'plane', id:'P1', mode:'offset', base:{kind:'std',id:'front'}, offset:z });
    f.push({ type:'sketch', id:sid(), plane:{kind:'feature',id:'P1'}, points:[pt('c',0,0)], entities:[circ('e1','c',r2)], constraints:[] });
    f.push({ type:'loft', id:fid(), op:'add', sketches:['S1','S2'] });
  } else if (kind === 'revolve') {
    const w0 = Math.abs(pickSize()), w1 = Math.abs(pickSize()), hh = Math.abs(pickSize());
    f.push({ type:'sketch', id:sid(), plane:{kind:'std',id:'front'}, points:[pt('p0',w0,0),pt('p1',w1,hh),pt('p2',0,hh),pt('p3',0,0),pt('a1',0,0),pt('a2',0,hh)], entities:[line('e1','p0','p1'),line('e2','p1','p2'),line('e3','p2','p3'),line('e4','p3','p0'),line('ax','a1','a2',{construction:true})], constraints:[] });
    f.push({ type:'revolve', id:fid(), op:'add', sketch:'S1', axis:'ax', angle: pickAngle() });
  } else if (kind === 'sweep') {
    const len = Math.abs(pickSize()), r = Math.abs(pickSize());
    f.push({ type:'sketch', id:sid(), plane:{kind:'std',id:'front'}, points:[pt('a',0,0),pt('b',0,len)], entities:[line('e1','a','b')], constraints:[] });
    f.push({ type:'sketch', id:sid(), plane:{kind:'std',id:'top'}, points:[pt('c',0,0)], entities:[circ('e1','c',r)], constraints:[] });
    f.push({ type:'sweep', id:fid(), op:'add', sketch:'S2', path:{kind:'sketch',id:'S1'} });
  } else if (kind === 'surface') {
    const r = Math.abs(pickSize());
    f.push({ type:'sketch', id:sid(), plane:{kind:'std',id:'front'}, points:[pt('c',0,0)], entities:[circ('e1','c',r)], constraints:[] });
    f.push({ type:'surfaceExtrude', id:fid(), sketch:'S1', end:'blind', depth: Math.abs(pickSize()) });
    if (maybe(0.8)) f.push({ type:'thicken', id:fid(), surface:'F2', thickness: pickSize(), side: pick(['front','back','both']) });
  }
  return f;
}
function pickSize() {
  const q = rnd();
  if (q < 0.12) return 0;
  if (q < 0.24) return Math.pow(10, rr(-6, -2)) * (maybe(0.5) ? 1 : -1);
  if (q < 0.32) return Math.pow(10, rr(3, 8));
  if (q < 0.44) return -Math.abs(rr(0.1, 30));
  return rr(0.1, 50);
}
function pickAngle() {
  const q = rnd();
  if (q < 0.1) return 0;
  if (q < 0.2) return -rr(1, 360);
  if (q < 0.3) return rr(360, 10000);
  return rr(1, 360);
}
function randomFeature(f, state) {
  let id = state.id; state.id += 1; const fid = `F${state.id}`;
  const type = pick(['shell','fillet','chamfer','draft','cut','pattern','mirror','thicken']);
  if (type === 'shell') { f.push({ type:'shell', id:fid, faces: maybe(0.7) ? ['F2.C1'] : ['F2.S.e1'], thickness: pickSize(), ...(maybe(0.2) ? { outward:true } : {}) }); return 'shell'; }
  if (type === 'fillet') { f.push({ type:'fillet', id:fid, radius: pickSize(), edges:[{ faces:['F2.S.e1','F2.S.e2'], at:[state.w??10,0,state.h??10] }], ...(maybe(0.1) ? { propagate:true } : {}) }); return 'fillet'; }
  if (type === 'chamfer') { f.push({ type:'chamfer', id:fid, distance: pickSize(), edges:[{ faces:['F2.S.e1','F2.S.e2'], at:[state.w??10,0,state.h??10] }] }); return 'chamfer'; }
  if (type === 'draft') { f.push({ type:'draft', id:fid, neutral:{kind:'face',id:'F2.C0'}, faces:['F2.S.e1','F2.S.e2','F2.S.e3','F2.S.e4'], angle: pickAngle() }); return 'draft'; }
  if (type === 'cut') { f.push({ type:'sketch', id:`S${++state.id}`, plane:{kind:'std',id:'top'}, points:[pt('c',0,0)], entities:[circ('e1','c',Math.abs(pickSize()))], constraints:[] }); f.push({ type:'extrude', id:fid, op:'cut', sketch:`S${state.id}`, end:'blind', depth: Math.abs(pickSize()), flip:true }); return 'cut'; }
  if (type === 'pattern') { f.push({ type:'lpattern', id:fid, seeds:['F2'], dir:{kind:'axis',axis:pick(['x','y','z'])}, spacing: pickSize(), count: pick([0,1,2,3,12,100,-2]) }); return 'lpattern'; }
  if (type === 'mirror') { f.push({ type:'mirror', id:fid, seeds:['F2'], plane:{kind:'std',id:pick(['right','top','front'])} }); return 'mirror'; }
  f.push({ type:'thicken', id:fid, surface:'F2', thickness: pickSize(), side: pick(['front','back','both']) }); return 'thicken';
}

function makeCase(i) {
  const kind = pick(['box','box','box','cyl','cyl','loft','revolve','sweep','surface']);
  const f = baseFeatures(kind);
  const state = { id: f.reduce((m, x) => Math.max(m, Number(String(x.id).replace(/\D/g,'')) || 0), 0), w: 20, h: 20 };
  const extra = Math.floor(rr(0, 7));
  for (let j = 0; j < extra; j++) randomFeature(f, state);
  // Known hard cases: a rounded-rectangle loop with tangent propagation.
  if (maybe(0.02)) {
    f.length = 0; state.id = 0;
    const w = Math.abs(pickSize()) || 10, d = Math.abs(pickSize()) || 10, cr = Math.abs(pickSize()) || 2, h = Math.abs(pickSize()) || 10;
    f.push({ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:[pt('p1',cr,0),pt('p2',w-cr,0),pt('p3',w,cr),pt('p4',w,d-cr),pt('p5',w-cr,d),pt('p6',cr,d),pt('p7',0,d-cr),pt('p8',0,cr),pt('c1',w-cr,cr),pt('c2',w-cr,d-cr),pt('c3',cr,d-cr),pt('c4',cr,cr)], entities:[line('e1','p1','p2'),{type:'arc',id:'e2',c:'c1',p1:'p2',p2:'p3'},line('e3','p3','p4'),{type:'arc',id:'e4',c:'c2',p1:'p4',p2:'p5'},line('e5','p5','p6'),{type:'arc',id:'e6',c:'c3',p1:'p6',p2:'p7'},line('e7','p7','p8'),{type:'arc',id:'e8',c:'c4',p1:'p8',p2:'p1'}], constraints:[] });
    f.push({ type:'extrude', id:'F2', op:'add', sketch:'S1', end:'blind', depth:h });
    f.push({ type:'fillet', id:'F3', radius: Math.abs(pickSize()), propagate:true, edges:[] });
    return { id:`case_${i}`, kind:'rrect', features:f };
  }
  return { id:`case_${i}`, kind, features:f };
}

function classify(r) {
  if (r.cls === 'trap') return { cls:'trap', detail:r.trap, last:r.last };
  if (r.cls === 'hang') return { cls:'hang', detail:'timeout', last:r.last };
  if (r.errors?.length) return { cls:'structured-error', detail:r.errors.join(','), last:r.last };
  if (r.issues?.length) return { cls:'invalid-result', detail:r.issues.join(','), last:r.last };
  if (r.volume == null && r.area == null) return { cls:'surface-only', last:r.last };
  if (r.volume === 0 && r.area === 0) return { cls:'degenerate', last:r.last };
  if (!(r.volume > 0) || !(r.area > 0)) return { cls:'wrong-answer', detail:`volume ${r.volume} area ${r.area}`, last:r.last };
  return { cls:'ok', last:r.last };
}

async function shrink(c, seen) {
  let features = JSON.parse(JSON.stringify(c.features));
  let budget = 16;
  const still = async (fs) => {
    if (budget-- <= 0) return false;
    const r = await runOne(fs, 800);
    const k = classify(r).cls;
    return k === c.cls || (c.cls === 'trap' && k === 'hang') || (c.cls === 'hang' && k === 'trap');
  };
  let changed = true;
  while (changed) {
    changed = false;
    for (let i = features.length - 1; i >= 1; i--) {
      const f2 = features.filter((_, j) => j !== i);
      if (await still(f2)) { features = f2; changed = true; break; }
    }
    for (const ft of features) for (const k of Object.keys(ft)) {
      if (typeof ft[k] === 'number' && ft[k] !== 0) {
        const v = ft[k]; ft[k] = Math.sign(v) * Math.pow(10, Math.floor(Math.log10(Math.abs(v))));
        if (await still(features)) changed = true; else ft[k] = v;
      }
    }
  }
  return features;
}

const shrinkInput = opt('--shrink-input');
if (shrinkInput) {
  const src = JSON.parse(readFileSync(shrinkInput, 'utf8'));
  const groups = {};
  for (const r of src.results ?? []) if (r.cls === 'trap' || r.cls === 'hang') {
    const key = `${r.cls}:${r.detail}:${r.last}`;
    (groups[key] ??= []).push(r);
  }
  const minimal = [];
  for (const [key, list] of Object.entries(groups)) {
    const one = list[0];
    minimal.push({ key, count: list.length, id: one.id, kind: one.kind, features: await shrink(one, groups) });
  }
  console.log(JSON.stringify({ groups: Object.keys(groups).length, minimal: minimal.length }));
  const out = opt('--json') ?? shrinkInput;
  writeFileSync(out, JSON.stringify({ ...src, minimal }, null, 1));
  if (baselineFile && opt('--update-baseline')) {
    const base = JSON.parse(readFileSync(baselineFile, 'utf8'));
    base.minimal = minimal;
    writeFileSync(baselineFile, JSON.stringify(base, null, 1) + '\n');
  }
  process.exit(0);
}


const cases = Array.from({ length: n }, (_, i) => makeCase(i));
const results = await mapLimit(cases, workers, async (c) => {
  const r = await runOne(c.features);
  return { id: c.id, kind: c.kind, ...classify(r), features: c.features };
});
const counts = {}; for (const r of results) counts[r.cls] = (counts[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n, seed, wasmHash, ...counts }));
const groups = {};
for (const r of results) if (r.cls === 'trap' || r.cls === 'hang') { const key = `${r.cls}:${r.detail}:${r.last}`; (groups[key] ??= []).push(r); }
const minimal = [];
for (const [key, list] of Object.entries(groups).sort((a, b) => b[1].length - a[1].length)) {
  if (minimal.length >= 12) break;
  const pickOne = list[0];
  const small = onlyShrink ? pickOne.features : await shrink(pickOne, groups);
  minimal.push({ key, count:list.length, id:pickOne.id, kind:pickOne.kind, features:small });
}
console.log('groups:', Object.entries(groups).map(([k,v])=>`${v.length} ${k}`).join(' | '));
if (minimal.length) console.log('minimal:', JSON.stringify(minimal.map((m)=>({key:m.key,count:m.count,id:m.id,kind:m.kind})), null, 1));
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify({ results, minimal }, null, 1));
if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  const casesOut = Object.fromEntries(results.map((r)=>[r.id, r.cls]));
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version:1, wasm:opt('--wasm-label','kit'), wasmHash, seed, n, budgetMs, timeout, cases:casesOut, minimal }, null, 1)+'\n'); console.log(`wrote ${baselineFile}`); }
  else { const base = JSON.parse(readFileSync(baselineFile,'utf8')).cases ?? {}; const pass = (c)=>c==='ok'||c==='structured-error'; const reg = results.filter((r)=>pass(base[r.id]) && !pass(r.cls)); console.log(`crash_fuzz check: ${reg.length} regressions`); for (const r of reg) console.log(`  ${r.id}: ${base[r.id]} -> ${r.cls} ${r.detail??''}`); if (reg.length) process.exit(1); }
}
