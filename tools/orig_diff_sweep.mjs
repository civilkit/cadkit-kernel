// Differential sweep: original kernel vs our kernel, same feature documents.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', '/path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm');
const orig = opt('--orig', '/path/to/work/kernel.wasm');
const samples = Number(opt('--samples', '10'));
const quick = argv.includes('--quick');
const only = opt('--only', '');
const seed = Number(opt('--seed', '7'));
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'original_gaps_baseline.json')));
const hash = (p) => crypto.createHash('sha256').update(readFileSync(p)).digest('hex').slice(0, 16);
const wasmHash = hash(wasm), origHash = hash(orig);
const ours = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const origK = new Kernel(new WebAssembly.Module(readFileSync(orig)));
let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const pick = (a) => a[Math.floor(rnd() * a.length)];
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2) => ({ type: 'line', id, p1, p2 });

const HULL = [[0,750,600,0],[600,750,600,0],[1200,745,595,0],[1800,700,540,0],[2400,560,390,40],[3000,330,190,110],[3500,60,20,260]];
function hullFeatures(st, H = 450, t = 3, surface = false) {
  const f = [], ids = [];
  st.forEach(([x, wt, wb, yb], i) => {
    f.push({ type:'plane', id:`P${i}`, mode:'offset', base:{kind:'std',id:'right'}, offset:x });
    const pts = surface
      ? [[-wb,yb],[wb,yb],[wt,H]].map(([u,v],j)=>pt(`p${i}_${j}`,u,v))
      : [[-wb,yb],[wb,yb],[wt,H],[-wt,H]].map(([u,v],j)=>pt(`p${i}_${j}`,u,v));
    const n = pts.length, ents = Array.from({length:n},(_,j)=>line(`e${i}_${j}`,`p${i}_${j}`,`p${i}_${(j+1)%n}`));
    f.push({ type:'sketch', id:`S${i}`, plane:{kind:'feature',id:`P${i}`}, points:pts, entities:ents, constraints:[] });
    ids.push(`S${i}`);
  });
  f.push({ type:'loft', id:'F8', op:'add', sketches:ids, ...(surface ? { surface:true } : {}) });
  return f;
}
function seedHull(t = 3) { return [...hullFeatures(HULL, 450, t), { type:'shell', id:'F9', faces:['F8.S.e0_2'], thickness:t }]; }
function randomHull(t = 3, surface = false) {
  const n = 5 + Math.floor(rr(0,4)), st = [];
  let x = 0;
  for (let i = 0; i < n; i++) {
    const wt = rr(120, 800) * (i === n - 1 ? 0.15 : 1), wb = wt * rr(0.55, 0.9), yb = rr(0, 180);
    st.push([x, wt, wb, yb]); x += rr(250, 650);
  }
  return st;
}
function box(w,d,h) { const P=[[0,0],[w,0],[w,d],[0,d]].map(([x,y],i)=>pt(`p${i}`,x,y)); const E=[0,1,2,3].map(i=>line(`e${i+1}`,`p${i}`,`p${(i+1)%4}`)); return [{type:'sketch',id:'S1',plane:{kind:'std',id:'front'},points:P,entities:E,constraints:[]},{type:'extrude',id:'F2',op:'add',sketch:'S1',end:'blind',depth:h}]; }

const cases = [];
const add = (id, family, features) => cases.push({ id, family, features });
add('hull_seed_shell3', 'hull_shell', seedHull(3));
for (let i = 0; i < samples; i++) {
  const st = randomHull();
  add(`hull_shell_${i}`, 'hull_shell', [...hullFeatures(st, rr(300,550), rr(1,5)), { type:'shell', id:'F9', faces:['F8.S.e0_2'], thickness:rr(1,5) }]);
  add(`hull_thicken_${i}`, 'hull_thicken', [...hullFeatures(randomHull(), rr(300,550), 1, true), { type:'thicken', id:'F9', surface:'F8', thickness:rr(1,5), side:pick(['front','back','both']) }]);
  // fillet
  const w=rr(15,50),d=rr(15,50),h=rr(10,40),r=rr(0.5,Math.min(w,d)*0.4);
  add(`fillet_${i}`, 'fillet', [...box(w,d,h), {type:'fillet',id:'F3',radius:r,edges:[{faces:['F2.S.e1','F2.S.e2'],at:[w,0,h/2]}]}]);
  // boolean
  const A=box(rr(10,40),rr(10,40),rr(10,40)); const B=box(rr(10,40),rr(10,40),rr(10,40)).map((f)=>f.type==='sketch'?{...f,id:'S2'}:{...f,id:'F4',sketch:'S2'});
  add(`boolean_${i}`, 'boolean', [...A,...B,{type:'moveBody',id:'M1',bodies:[{face:'F4.C1'}],translate:[rr(-10,10),rr(-10,10),rr(-10,10)]},{type:'combine',id:'C1',op:pick(['union','cut','intersect']),target:{face:'F2.C1'},tools:[{face:'F4.C1'}]}]);
  // shape: loft3
  const f=[]; let z=0; const ids=[]; for(let j=0;j<3;j++){const r2=rr(5,20); f.push({type:'plane',id:`Q${j}`,mode:'offset',base:{kind:'std',id:'front'},offset:z}); f.push({type:'sketch',id:`T${j}`,plane:{kind:'feature',id:`Q${j}`},points:[pt('c',0,0)],entities:[{type:'circle',id:'e1',c:'c',r:r2}],constraints:[]}); ids.push(`T${j}`); z+=rr(10,30);} f.push({type:'loft',id:'F8',op:'add',sketches:ids}); add(`shape_${i}`, 'shape', f);
  // draft
  const dd=box(40,20,10); add(`draft_${i}`, 'draft', [...dd,{type:'draft',id:'F3',neutral:{kind:'face',id:'F2.C0'},faces:['F2.S.e1','F2.S.e2','F2.S.e3','F2.S.e4'],angle:rr(0.5,15)}]);
  // extrude variants
  add(`extrude_${i}`, 'extrude', [{type:'sketch',id:'S1',plane:{kind:'std',id:'front'},points:[[0,0],[30,0],[30,20],[0,20]].map(([x,y],j)=>pt(`p${j}`,x,y)),entities:[0,1,2,3].map(j=>line(`e${j+1}`,`p${j}`,`p${(j+1)%4}`)),constraints:[]},{type:'extrude',id:'F2',op:'add',sketch:'S1',end:pick(['blind','through']),depth:rr(1,40),...(rnd()<0.4?{draft:rr(0,15)}:{})}]);
}
const selected = only ? cases.filter((c)=>c.id.includes(only)) : (quick ? cases.filter((c)=>['hull_seed_shell3','hull_shell_0','hull_thicken_0','fillet_0','boolean_0','shape_0','draft_0','extrude_0'].includes(c.id)) : cases);

function runK(k, features) {
  const r = JSON.parse(k.request(JSON.stringify({ id:1, msg:{ type:'regen', tag:'m', part:{ format:'ck-part', version:1, features }, opts:{ solveSketches:true } } })).json);
  if (r.trap) return { cls:'trap', detail:r.trap };
  const errs = (r.result?.errors ?? []).map(([,e]) => ({ code:e.code, detail:e.params?.detail }));
  if (errs.length) return { cls:errs[0].code, detail:errs[0].detail, errors:errs };
  const m = JSON.parse(k.request(JSON.stringify({ id:2, msg:{ type:'massProps', tag:'m', bodies:true } })).json);
  return { cls:'ok', vol:m.result?.volume, area:m.result?.area };
}
const rows = [];
for (const c of selected) {
  const o = runK(ours, c.features), g = runK(origK, c.features);
  let cls = 'both-ok';
  if (o.cls !== 'ok' && g.cls !== 'ok') cls = 'both-fail';
  else if (g.cls === 'ok' && o.cls !== 'ok') cls = 'orig-only';
  else if (o.cls === 'ok' && g.cls !== 'ok') cls = 'ours-only';
  else if (o.cls === 'ok' && g.cls === 'ok') cls = 'both-ok';
  let rel = null;
  if (o.cls === 'ok' && g.cls === 'ok' && o.vol != null && g.vol != null) { rel = Math.abs(o.vol - g.vol) / Math.max(Math.abs(g.vol),1e-9); if (rel > 1e-3) cls = 'volume'; }
  rows.push({ id:c.id, family:c.family, cls, ours:o.cls, orig:g.cls, detail:o.detail ?? g.detail, volOurs:o.vol, volOrig:g.vol, rel, features:c.features });
}
const g = {}; for (const r of rows) { g[r.family] ??= {}; g[r.family][r.cls] = (g[r.family][r.cls]??0)+1; }
const tot = {}; for (const r of rows) tot[r.cls] = (tot[r.cls]??0)+1;
console.log(JSON.stringify({ n:rows.length, seed, wasmHash, origHash, ...tot }));
for (const [f,x] of Object.entries(g)) console.log(`${f}: ${JSON.stringify(x)}`);
const gaps = rows.filter((r)=>r.cls==='orig-only'||r.cls==='volume');
console.log('gaps', gaps.length);
for (const r of gaps.slice(0,20)) console.log(` ${r.id} ${r.family} ours=${r.ours} orig=${r.orig} rel=${r.rel ?? '-'} ${r.detail ?? ''}`);
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows, null, 1));
if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  const base = existsSync(baselineFile) ? JSON.parse(readFileSync(baselineFile,'utf8')) : { cases:{} };
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version:1, wasmHash, origHash, seed, cases:Object.fromEntries(rows.map((r)=>[r.id,r.cls])) }, null, 1)+'\n'); console.log(`wrote ${baselineFile}`); }
  else { const pass=(c)=>c==='both-ok'||c==='both-fail'||c==='ours-only'; const reg=rows.filter((r)=>pass(base.cases?.[r.id]) && !pass(r.cls)); console.log(`orig_diff check: ${reg.length} regressions`); for(const r of reg) console.log(`  ${r.id}: ${base.cases[r.id]} -> ${r.cls}`); if(reg.length)process.exit(1); }
}
