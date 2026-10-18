// Mass-property sweep: volume, area, COG and inertia about the CG vs OCP.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', '/path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm');
const seed = Number(opt('--seed', '1'));
const samples = Number(opt('--samples', '20'));
const only = opt('--only', '');
const mod = new WebAssembly.Module(readFileSync(wasm));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const k = new Kernel(mod);
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'massprops_sweep_baseline.json')));
let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
const circ = (id, c, r) => ({ type: 'circle', id, c, r });
let n = 0;
const plane = (z) => z === 0 ? [{ kind: 'std', id: 'front' }, []] : [{ kind: 'feature', id: `P${++n}` }, [{ type: 'plane', id: `P${n}`, mode: 'offset', base: { kind: 'std', id: 'front' }, offset: z }]];
const circleSk = (id, r, z, cx = 0, cy = 0) => { const [pl, pre] = plane(z); return [...pre, { type: 'sketch', id, plane: pl, points: [pt('c', cx, cy)], entities: [circ('e1', 'c', r)], constraints: [] }]; };
function rectSk(id, w, d, z, rot = 0) {
  const c = Math.cos(rot * Math.PI / 180), s = Math.sin(rot * Math.PI / 180);
  const P = [[-w/2,-d/2],[w/2,-d/2],[w/2,d/2],[-w/2,d/2]].map(([x,y],i)=>pt(`p${i}`,x*c-y*s,x*s+y*c));
  const E = [0,1,2,3].map((i)=>line(`e${i+1}`,`p${i}`,`p${(i+1)%4}`));
  const [pl, pre] = plane(z); return [...pre, { type:'sketch', id, plane: pl, points: P, entities: E, constraints: [] }];
}
const ex = (id, sid, depth, extra = {}) => ({ type: 'extrude', id, op: 'add', sketch: sid, end: 'blind', depth, ...extra });
function prim(kind, p) {
  n = 0;
  if (kind === 'box') { const f = rectSk('S1', p.w, p.d, 0); return [...f, ex('F2', 'S1', p.h)]; }
  if (kind === 'cyl') { const f = circleSk('S1', p.r, 0); return [...f, ex('F2', 'S1', p.h)]; }
  if (kind === 'cone') { const f = circleSk('S1', p.r, 0); return [...f, ex('F2', 'S1', p.h, { draft: p.draft })]; }
  if (kind === 'sphere') return [{ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:[pt('c',0,0),pt('s',0,-p.r),pt('t',0,p.r),pt('a1',0,0),pt('a2',0,p.r)], entities:[{type:'arc',id:'e1',c:'c',p1:'s',p2:'t'},line('e2','t','s'),line('ax','a1','a2',{construction:true})], constraints:[] }, { type:'revolve', id:'F2', op:'add', sketch:'S1', axis:'ax', angle:360 }];
  if (kind === 'torus') return [{ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:[pt('c',p.R,0),pt('a1',0,0),pt('a2',0,2*p.r+1)], entities:[circ('e1','c',p.r),line('ax','a1','a2',{construction:true})], constraints:[] }, { type:'revolve', id:'F2', op:'add', sketch:'S1', axis:'ax', angle:360 }];
  if (kind === 'tube') { const f = circleSk('S1', p.ro, 0); f[0].entities.push(circ('e2','c',p.ri)); return [...f, ex('F2','S1',p.h)]; }
  throw new Error(kind);
}
function shape(kind, i) {
  if (kind === 'box') return { w: rr(10,60), d: rr(10,60), h: rr(10,60) };
  if (kind === 'cyl') return { r: rr(5,30), h: rr(10,60) };
  if (kind === 'cone') { const r = rr(8,25), h = rr(10,50); return { r, h, draft: rr(0, Math.min(20, Math.atan((r-0.5)/h)*180/Math.PI*0.8)) }; }
  if (kind === 'sphere') return { r: rr(5,30) };
  if (kind === 'torus') { const R = rr(10,30); return { R, r: rr(2, Math.min(8,R*0.5)) }; }
  const ro = rr(12,40); return { ro, ri: ro*rr(0.3,0.8), h: rr(10,50) };
}
function loftFeatures(N, i) {
  n = 0; const f = [], ids = [], specs = []; let z = 0;
  for (let j = 0; j < N; j++) { const id = `S${j}`; const r = rr(5,20); f.push(...circleSk(id, r, z)); ids.push(id); specs.push({ kind:'circle', r, z }); z += rr(10,30); }
  f.push({ type:'loft', id:'L1', op:'add', sketches: ids });
  return { features: f, ref: { kind:'loft', sketches: specs } };
}
function revolveCase() {
  n = 0; const h = rr(10,40), w0 = rr(2,15), w1 = rr(2,15), ang = rr(90,360);
  const f = [{ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:[pt('p0',w0,0),pt('p1',w1,h),pt('p2',0,h),pt('p3',0,0),pt('a1',0,0),pt('a2',0,h)], entities:[line('e1','p0','p1'),line('e2','p1','p2'),line('e3','p2','p3'),line('e4','p3','p0'),line('ax','a1','a2',{construction:true})], constraints:[] }, { type:'revolve', id:'R1', op:'add', sketch:'S1', axis:'ax', angle: ang }];
  return { features: f, ref: { kind:'revolve', profile:[[w0,0],[w1,h],[0,h],[0,0]], angle: ang } };
}
function sweepLineCase() {
  n = 0; const len = rr(15,50), r = rr(2,8);
  const f = [{ type:'sketch', id:'P1', plane:{kind:'std',id:'front'}, points:[pt('a',0,0),pt('b',0,len)], entities:[line('e1','a','b')], constraints:[] },
    { type:'sketch', id:'S1', plane:{kind:'std',id:'top'}, points:[pt('c',0,0)], entities:[circ('e1','c',r)], constraints:[] },
    { type:'sweep', id:'W1', op:'add', sketch:'S1', path:{kind:'sketch',id:'P1'} }];
  return { features: f, ref: { kind:'sweep_line', r, len } };
}
function sweepArcCase() {
  n = 0; const R = rr(10,30), r = rr(2,7), a1 = rr(45,180);
  const f = [{ type:'sketch', id:'P1', plane:{kind:'std',id:'front'}, points:[pt('c',-R,0),pt('a',0,0),pt('b',-R+R*Math.cos(a1*Math.PI/180),R*Math.sin(a1*Math.PI/180))], entities:[{type:'arc',id:'e1',c:'c',p1:'a',p2:'b'}], constraints:[] },
    { type:'sketch', id:'S1', plane:{kind:'std',id:'top'}, points:[pt('c',0,0)], entities:[circ('e1','c',r)], constraints:[] },
    { type:'sweep', id:'W1', op:'add', sketch:'S1', path:{kind:'sketch',id:'P1'} }];
  return { features: f, ref: { kind:'sweep_arc', R, r, a1 } };
}
function sweepLCase() {
  n = 0; const l1 = rr(10,25), l2 = rr(10,25), rw = rr(3,8), rd = rr(3,8);
  const f = [{ type:'sketch', id:'P1', plane:{kind:'std',id:'front'}, points:[pt('p0',0,0),pt('p1',0,l1),pt('p2',l2,l1)], entities:[line('e1','p0','p1'),line('e2','p1','p2')], constraints:[] },
    ...rectSk('S1', rw, rd, 0)];
  // rectSk uses front; replace with top
  f[f.length - 1].plane = { kind:'std', id:'top' };
  f.push({ type:'sweep', id:'W1', op:'add', sketch:'S1', path:{kind:'sketch',id:'P1'} });
  return { features: f, ref: { kind:'sweep_L', rw, rd, l1, l2 } };
}

const cases = [];
const add = (id, template, features, ref) => cases.push({ id, template, features, ref });
for (let i = 0; i < samples; i++) {
  for (const kind of ['box','cyl','cone','sphere','torus','tube']) { const p = shape(kind, i); add(`${kind}_${i}`, kind, prim(kind, p), { kind, ...p }); }
  const l2 = loftFeatures(2, i); add(`loft2_${i}`, 'loft2', l2.features, l2.ref);
  const l3 = loftFeatures(3, i); add(`loft3_${i}`, 'loft3', l3.features, l3.ref);
  const rv = revolveCase(); add(`revolve_${i}`, 'revolve', rv.features, rv.ref);
  const sl = sweepLineCase(); add(`sweep_line_${i}`, 'sweep_line', sl.features, sl.ref);
  const sa = sweepArcCase(); add(`sweep_arc_${i}`, 'sweep_arc', sa.features, sa.ref);
  const sL = sweepLCase(); add(`sweep_L_${i}`, 'sweep_L', sL.features, sL.ref);
}
const selected = only ? cases.filter((c) => c.id.includes(only)) : cases;
const rows = [], refCases = {};
for (const c of selected) {
  refCases[c.id] = c.ref;
  let r; try { r = JSON.parse(k.request(JSON.stringify({ id:1, msg:{ type:'regen', tag:'m', part:{ format:'ck-part', version:1, features:c.features }, opts:{ solveSketches:true } } })).json); } catch { r = null; }
  if (!r || !r.ok) { rows.push({ id:c.id, template:c.template, cls:'regen-fail' }); continue; }
  const errs = (r.result?.errors ?? []).map(([,e]) => e.code);
  if (errs.length) { rows.push({ id:c.id, template:c.template, cls:errs[0], detail:errs.join(',') }); continue; }
  const m = JSON.parse(k.request(JSON.stringify({ id:2, msg:{ type:'massProps', tag:'m', bodies:true } })).json);
  rows.push({ id:c.id, template:c.template, cls:'ok', mass: m.result ?? null });
}
const rp = '/tmp/massprops_sweep_cases.json', ro = '/tmp/massprops_sweep_ref.json';
writeFileSync(rp, JSON.stringify(refCases, null, 1));
try { execFileSync('python3.12', [path.join(here, 'massprops_sweep_ref.py'), '--cases', rp, '--out', ro], { stdio:'pipe', timeout:60000 }); } catch (e) { console.error('ref failed', e.message); }
const ref = existsSync(ro) ? JSON.parse(readFileSync(ro, 'utf8')) : {};
const rel = (a, b, scale = 1) => Math.abs(a - b) / Math.max(Math.abs(scale), 1e-12);
const mag = (v) => Math.hypot(v[0], v[1], v[2]);
for (const row of rows) {
  if (row.cls !== 'ok') continue;
  const m = row.mass, r = ref[row.id];
  if (!r) { row.cls = 'no-ref'; continue; }
  if (r.error) { row.cls = 'ref-fails'; row.detail = r.error; continue; }
  if (m?.volume == null || m?.area == null || !m?.com || !m?.principal || !m?.axes) { row.cls = 'missingMass'; continue; }
  const v = rel(m.volume, r.volume, r.volume);
  const a = rel(m.area, r.area, r.area);
  const c = rel(math3(m.com, r.com), 0, Math.max(1, Math.cbrt(r.volume)));
  const I = reconstruct(m);
  let ie = 0; for (let i=0;i<3;i++) for (let j=0;j<3;j++) ie = Math.max(ie, rel(I[i][j], r.inertia[i][j], Math.max(...r.inertia.flat().map(Math.abs))));
  row.errors = { volume:v, area:a, com:c, inertia:ie };
  row.worst = Math.max(v,a,c,ie);
  row.detail = `v ${v.toExponential(2)} a ${a.toExponential(2)} com ${c.toExponential(2)} I ${ie.toExponential(2)}`;
  if (v > 1e-6) row.cls = 'wrongVolume';
  else if (a > 1e-6) row.cls = 'wrongArea';
  else if (c > 1e-6) row.cls = 'wrongCom';
  else if (ie > 1e-6) row.cls = 'wrongInertia';
  else row.cls = 'ok';
}
function math3(a,b){ return Math.hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]); }
function reconstruct(m) {
  const I = [[0,0,0],[0,0,0],[0,0,0]];
  for (let k = 0; k < 3; k++) { const ax = m.axes[k]; for (let i=0;i<3;i++) for (let j=0;j<3;j++) I[i][j] += m.principal[k]*ax[i]*ax[j]; }
  return I;
}
const g = {}; for (const r of rows) { g[r.template] ??= { n:0, cls:{}, max:0, worst:null }; const x=g[r.template]; x.n++; x.cls[r.cls]=(x.cls[r.cls]??0)+1; if (r.worst > x.max) { x.max=r.worst; x.worst=r.id; } }
const tot = {}; for (const r of rows) tot[r.cls] = (tot[r.cls]??0)+1;
console.log(JSON.stringify({ n:rows.length, seed, samples, wasmHash, ...tot }));
for (const [t,x] of Object.entries(g)) { const bad=Object.entries(x.cls).filter(([k])=>k!=='ok').reduce((a,[,v])=>a+v,0); if (bad) console.log(`${t}: n=${x.n} ${JSON.stringify(x.cls)} maxRel=${x.max.toExponential(2)} worst=${x.worst}`); }
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows, null, 1));
if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  const casesOut = Object.fromEntries(rows.map((r)=>[r.id, r.cls]));
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version:1, wasm:opt('--wasm','kit'), wasmHash, seed, samples, cases:casesOut }, null, 1)+'\n'); console.log(`wrote ${baselineFile}`); }
  else { const base = JSON.parse(readFileSync(baselineFile,'utf8')).cases ?? {}; const pass = (c)=>c==='ok'; const reg = rows.filter((r)=>pass(base[r.id]) && !pass(r.cls)); console.log(`massprops_sweep check: ${reg.length} regressions`); for (const r of reg) console.log(`  ${r.id}: ${r.cls}`); if (reg.length) process.exit(1); }
}
