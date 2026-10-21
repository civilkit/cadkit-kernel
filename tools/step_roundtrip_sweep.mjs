// STEP export round trip: kernel step -> OCP read + kernel importStep.
import { readFileSync, writeFileSync, existsSync, mkdirSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', fileURLToPath(new URL('../../kit/apps/cad/public/kernel/ck_wasm.wasm', import.meta.url)));
const seed = Number(opt('--seed', '7'));
const samples = Number(opt('--samples', '10'));
const only = opt('--only', '');
const outDir = opt('--step-dir', '/tmp/step_roundtrip');
mkdirSync(outDir, { recursive: true });
const mod = new WebAssembly.Module(readFileSync(wasm));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const k = new Kernel(mod);
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'step_roundtrip_baseline.json')));
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
  const c = Math.cos(rot * Math.PI / 180), s2 = Math.sin(rot * Math.PI / 180);
  const P = [[-w/2,-d/2],[w/2,-d/2],[w/2,d/2],[-w/2,d/2]].map(([x,y],i)=>pt(`p${i}`,x*c-y*s2,x*s2+y*c));
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
function shape(kind) {
  if (kind === 'box') return { w: rr(10,60), d: rr(10,60), h: rr(10,60) };
  if (kind === 'cyl') return { r: rr(5,30), h: rr(10,60) };
  if (kind === 'cone') { const r = rr(8,25), h = rr(10,50); return { r, h, draft: rr(0, Math.min(20, Math.atan((r-0.5)/h)*180/Math.PI*0.8)) }; }
  if (kind === 'sphere') return { r: rr(5,30) };
  if (kind === 'torus') { const R = rr(10,30); return { R, r: rr(2, Math.min(8,R*0.5)) }; }
  const ro = rr(12,40); return { ro, ri: ro*rr(0.3,0.8), h: rr(10,50) };
}
function loftFeatures(N) {
  n = 0; const f = [], ids = []; let z = 0;
  for (let j = 0; j < N; j++) { const id = `S${j}`; const r = rr(5,20); f.push(...circleSk(id, r, z)); ids.push(id); z += rr(10,30); }
  f.push({ type:'loft', id:'L1', op:'add', sketches: ids }); return f;
}
function revolveCase() {
  n = 0; const h = rr(10,40), w0 = rr(2,15), w1 = rr(2,15), ang = rr(90,360);
  return [{ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:[pt('p0',w0,0),pt('p1',w1,h),pt('p2',0,h),pt('p3',0,0),pt('a1',0,0),pt('a2',0,h)], entities:[line('e1','p0','p1'),line('e2','p1','p2'),line('e3','p2','p3'),line('e4','p3','p0'),line('ax','a1','a2',{construction:true})], constraints:[] }, { type:'revolve', id:'R1', op:'add', sketch:'S1', axis:'ax', angle: ang }];
}
function sweepLineCase() {
  n = 0; const len = rr(15,50), r = rr(2,8);
  return [{ type:'sketch', id:'P1', plane:{kind:'std',id:'front'}, points:[pt('a',0,0),pt('b',0,len)], entities:[line('e1','a','b')], constraints:[] },
    { type:'sketch', id:'S1', plane:{kind:'std',id:'top'}, points:[pt('c',0,0)], entities:[circ('e1','c',r)], constraints:[] },
    { type:'sweep', id:'W1', op:'add', sketch:'S1', path:{kind:'sketch',id:'P1'} }];
}
function sweepArcCase() {
  n = 0; const R = rr(10,30), r = rr(2,7), a1 = rr(45,180);
  return [{ type:'sketch', id:'P1', plane:{kind:'std',id:'front'}, points:[pt('c',-R,0),pt('a',0,0),pt('b',-R+R*Math.cos(a1*Math.PI/180),R*Math.sin(a1*Math.PI/180))], entities:[{type:'arc',id:'e1',c:'c',p1:'a',p2:'b'}], constraints:[] },
    { type:'sketch', id:'S1', plane:{kind:'std',id:'top'}, points:[pt('c',0,0)], entities:[circ('e1','c',r)], constraints:[] },
    { type:'sweep', id:'W1', op:'add', sketch:'S1', path:{kind:'sketch',id:'P1'} }];
}
function sweepLCase() {
  n = 0; const l1 = rr(10,25), l2 = rr(10,25), rw = rr(3,8), rd = rr(3,8);
  const f = [{ type:'sketch', id:'P1', plane:{kind:'std',id:'front'}, points:[pt('p0',0,0),pt('p1',0,l1),pt('p2',l2,l1)], entities:[line('e1','p0','p1'),line('e2','p1','p2')], constraints:[] },
    ...rectSk('S1', rw, rd, 0)];
  f[f.length - 1].plane = { kind:'std', id:'top' };
  f.push({ type:'sweep', id:'W1', op:'add', sketch:'S1', path:{kind:'sketch',id:'P1'} });
  return f;
}

const cases = [];
const add = (id, template, features) => cases.push({ id, template, features });
for (let i = 0; i < samples; i++) {
  for (const kind of ['box','cyl','cone','sphere','torus','tube']) add(`${kind}_${i}`, kind, prim(kind, shape(kind)));
  add(`loft2_${i}`, 'loft2', loftFeatures(2));
  add(`loft3_${i}`, 'loft3', loftFeatures(3));
  add(`revolve_${i}`, 'revolve', revolveCase());
  add(`sweep_line_${i}`, 'sweep_line', sweepLineCase());
  add(`sweep_arc_${i}`, 'sweep_arc', sweepArcCase());
  add(`sweep_L_${i}`, 'sweep_L', sweepLCase());
}
const selected = only ? cases.filter((c) => c.id.includes(only)) : cases;
const rows = [], paths = {};
for (const c of selected) {
  const stepFile = path.join(outDir, `${c.id}.step`);
  try {
    const r = JSON.parse(k.request(JSON.stringify({ id:1, msg:{ type:'regen', tag:'main', part:{ format:'ck-part', version:1, features:c.features }, opts:{ solveSketches:true } } })).json);
    if (!r.ok || r.result?.errors?.length) { rows.push({ id:c.id, template:c.template, cls:'regen-fail' }); continue; }
    const m = JSON.parse(k.request(JSON.stringify({ id:2, msg:{ type:'massProps', tag:'main', bodies:true } })).json);
    const st = JSON.parse(k.request(JSON.stringify({ id:3, msg:{ type:'step', tag:'main', name:c.id } })).json);
    if (!st?.ok || typeof st.result !== 'string' || st.result.length < 100) { rows.push({ id:c.id, template:c.template, cls:'step-fail', detail:JSON.stringify(st?.error ?? st).slice(0,120) }); continue; }
    writeFileSync(stepFile, st.result);
    const imp = JSON.parse(k.request(JSON.stringify({ id:4, msg:{ type:'importStep', tag:'imp', name:c.id, data:st.result } })).json);
    const mi = JSON.parse(k.request(JSON.stringify({ id:5, msg:{ type:'massProps', tag:'imp', bodies:true } })).json);
    const vi = JSON.parse(k.request(JSON.stringify({ id:6, msg:{ type:'validate', tag:'imp' } })).json);
    rows.push({ id:c.id, template:c.template, cls:'ok', orig:{ volume:m.result?.volume, area:m.result?.area }, reimport:{ volume:mi.result?.volume, area:mi.result?.area, valid:vi.result?.ok, issues:(vi.result?.issues ?? []).length, ok:!!imp?.ok } });
    paths[c.id] = stepFile;
  } catch (e) { rows.push({ id:c.id, template:c.template, cls:'trap', detail:String(e).slice(0,120) }); }
}
const rp = '/tmp/step_roundtrip_cases.json', ro = '/tmp/step_roundtrip_ref.json';
writeFileSync(rp, JSON.stringify(Object.entries(paths).map(([id, file]) => ({ id, file })), null, 1));
try { execFileSync('python3.12', [path.join(here, 'step_roundtrip_ref.py'), '--cases', rp, '--out', ro], { stdio:'pipe', timeout:180000 }); } catch (e) { console.error('ref failed', e.message); }
const ref = existsSync(ro) ? JSON.parse(readFileSync(ro, 'utf8')) : {};
const rel = (a, b) => Math.abs(a - b) / Math.max(Math.abs(b), 1e-12);
for (const row of rows) {
  if (row.cls !== 'ok') continue;
  const r = ref[row.id];
  if (!r) { row.cls = 'no-ref'; continue; }
  if (r.error) { row.cls = 'ref-fails'; row.detail = r.error; continue; }
  const rv = rel(r.volume, row.orig.volume), ra = rel(r.area, row.orig.area);
  const iv = rel(row.reimport.volume ?? NaN, row.orig.volume), ia = rel(row.reimport.area ?? NaN, row.orig.area);
  row.entities = r.types ?? {};
  if (!r.valid) row.cls = 'ocp-invalid';
  else if (rv > 1e-6) row.cls = 'ocp-wrongVolume';
  else if (ra > 1e-6) row.cls = 'ocp-wrongArea';
  else if (!row.reimport.ok || row.reimport.valid !== true || row.reimport.issues) row.cls = 'reimport-invalid';
  else if (iv > 1e-6) row.cls = 'reimport-wrongVolume';
  else if (ia > 1e-6) row.cls = 'reimport-wrongArea';
  row.detail = `ocp v ${rv.toExponential(2)} a ${ra.toExponential(2)}; reimport v ${iv.toExponential(2)} a ${ia.toExponential(2)}`;
}
const g = {}; for (const r of rows) { g[r.template] ??= {}; g[r.template][r.cls] = (g[r.template][r.cls] ?? 0) + 1; }
const tot = {}; for (const r of rows) tot[r.cls] = (tot[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n:rows.length, seed, samples, wasmHash, ...tot }));
for (const [t,x] of Object.entries(g)) { if (Object.keys(x).some((k)=>k!=='ok')) console.log(`${t}: ${JSON.stringify(x)}`); }
const ents = {};
for (const r of rows) if (r.cls !== 'ok') for (const [k,v] of Object.entries(r.entities ?? {})) ents[k] = (ents[k] ?? 0) + v;
if (Object.keys(ents).length) console.log('entities in failures:', JSON.stringify(ents));
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows, null, 1));
if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version:1, wasm:opt('--wasm','kit'), wasmHash, seed, samples, cases:Object.fromEntries(rows.map((r)=>[r.id,r.cls])) }, null, 1)+'\n'); console.log(`wrote ${baselineFile}`); }
  else { const base = JSON.parse(readFileSync(baselineFile,'utf8')).cases ?? {}; const reg = rows.filter((r)=>base[r.id]==='ok' && r.cls!=='ok'); console.log(`step_roundtrip check: ${reg.length} regressions`); if (reg.length) process.exit(1); }
}
