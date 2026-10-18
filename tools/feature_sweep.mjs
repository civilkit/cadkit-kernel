// Holes / patterns / mirror / draft sweep vs OCCT (analytic for draft).
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
const seed = Number(opt('--seed', '7'));
const samples = Number(opt('--samples', '15'));
const only = opt('--only', '');
const mod = new WebAssembly.Module(readFileSync(wasm));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const k = new Kernel(mod);
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'feature_sweep_baseline.json')));
let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const pick = (a) => a[Math.floor(rnd() * a.length)];
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
const circ = (id, c, r) => ({ type: 'circle', id, c, r });
let n = 0;
function box(w, d, h) {
  const P = [[0,0],[w,0],[w,d],[0,d]].map(([x,y],i)=>pt(`p${i}`,x,y));
  const E = [0,1,2,3].map((i)=>line(`e${i+1}`,`p${i}`,`p${(i+1)%4}`));
  return [{ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:P, entities:E, constraints:[] }, { type:'extrude', id:'F2', op:'add', sketch:'S1', end:'blind', depth:h }];
}
function cutCircle(id, sid, cx, cy, r, depth = null, flip = false) {
  return [{ type:'sketch', id:sid, plane:{kind:'face',id:'F2.C1'}, points:[pt('c',cx,cy)], entities:[circ('e1','c',r)], constraints:[] },
    { type:'extrude', id, op:'cut', sketch:sid, end: depth == null ? 'throughBoth' : 'blind', ...(depth == null ? {} : { depth }) }];
}
function casesGen() {
  const cases = [];
  for (let i = 0; i < samples; i++) {
    const w = rr(30, 80), d = rr(30, 80), h = rr(10, 40);
    const cx = w / 2, cy = d / 2, r = rr(2, Math.min(w, d) * 0.15), depth = rr(2, h * 0.8);
    cases.push({ id:`hole_through_${i}`, template:'hole_through', ref:{kind:'holes', w,d,h,holes:[{x:cx,y:cy,r,through:true}]}, features:[...box(w,d,h), ...cutCircle('F4','S2',cx,cy,r)] });
    cases.push({ id:`hole_blind_${i}`, template:'hole_blind', ref:{kind:'holes', w,d,h,holes:[{x:cx,y:cy,r,depth}]}, features:[...box(w,d,h), ...cutCircle('F4','S2',cx,cy,r,depth)] });
    const cb = rr(r * 1.5, r * 2.5), cbd = rr(1, Math.min(h * 0.4, depth));
    cases.push({ id:`counterbore_${i}`, template:'counterbore', ref:{kind:'holes', w,d,h,holes:[{x:cx,y:cy,r,depth:null},{x:cx,y:cy,r:cb,depth:cbd}]}, features:[...box(w,d,h), ...cutCircle('F4','S2',cx,cy,r), ...cutCircle('F5','S3',cx,cy,cb,cbd)] });
    // countersink: a cone cut from the top; approximates with a drafted circular cut
    const csr = rr(r * 1.3, r * 2.2), csd = rr(1, Math.min(h * 0.3, r)), csa = rr(10, 40);
    const f = [...box(w,d,h)];
    f.push({ type:'sketch', id:'S2', plane:{kind:'face',id:'F2.C1'}, points:[pt('c',cx,cy)], entities:[circ('e1','c',csr)], constraints:[] });
    f.push({ type:'extrude', id:'F4', op:'cut', sketch:'S2', end:'blind', depth:csd, draft: csa });
    cases.push({ id:`countersink_${i}`, template:'countersink', ref:{kind:'countersink', w,d,h,x:cx,y:cy,r:csr,depth:csd,angle:csa}, features:f });
    const count = pick([2,3,4,5,6,8,12]), spacing = rr(r * 2.2, r * 5);
    const lf = [...box(w,d,h), ...cutCircle('F4','S2',cx,cy,r)];
    lf.push({ type:'lpattern', id:'F5', seeds:['F4'], dir:{kind:'axis',axis:'x'}, spacing, count });
    cases.push({ id:`lpattern_${i}`, template:'lpattern', ref:{kind:'lpattern', w,d,h,x:cx,y:cy,r,spacing,count,axis:'x'}, features:lf });
    const cc = pick([3,4,5,6,8,12]), cr = rr(r * 2.5, Math.min(w,d) * 0.35);
    const cf = [...box(w,d,h), ...cutCircle('F4','S2',cx+cr,cy,r)];
    cf.push({ type:'cpattern', id:'F5', seeds:['F4'], axis:{kind:'axis',axis:'z'}, count:cc, angle:360, equal:true });
    cases.push({ id:`cpattern_${i}`, template:'cpattern', ref:{kind:'cpattern', w,d,h,cx:cx+cr,cy,r,count:cc,radius:cr}, features:cf });
    const mx = rr(r * 1.5, w * 0.3);
    const mf = [...box(w,d,h), ...cutCircle('F4','S2',cx-mx,cy,r)];
    mf.push({ type:'mirror', id:'F5', seeds:['F4'], plane:{kind:'std',id:'right'} });
    cases.push({ id:`mirror_${i}`, template:'mirror', ref:{kind:'mirror', w,d,h,x:cx-mx,y:cy,r,mirrorX:w}, features:mf });
    const ang = rr(0.5, 15);
    cases.push({ id:`draft_${i}`, template:'draft', ref:{kind:'draft', w,d,h,angle:ang}, features:[...box(w,d,h), {type:'draft', id:'F3', neutral:{kind:'face',id:'F2.C0'}, faces:['F2.S.e1','F2.S.e2','F2.S.e3','F2.S.e4'], angle:ang}] });
  }
  return cases;
}
const all = casesGen();
const selected = only ? all.filter((c)=>c.id.includes(only)) : all;
const rows = [], refCases = {};
for (const c of selected) {
  refCases[c.id] = c.ref;
  let r; try { r = JSON.parse(k.request(JSON.stringify({ id:1, msg:{ type:'regen', tag:'main', part:{ format:'ck-part', version:1, features:c.features }, opts:{ solveSketches:true } } })).json); } catch { r = null; }
  if (!r || !r.ok) { rows.push({ id:c.id, template:c.template, cls:'regen-fail' }); continue; }
  const errs = (r.result?.errors ?? []).map(([,e])=>e.code);
  if (errs.length) { rows.push({ id:c.id, template:c.template, cls:errs[0], detail:errs.join(',') }); continue; }
  const m = JSON.parse(k.request(JSON.stringify({ id:2, msg:{ type:'massProps', tag:'main', bodies:true } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id:3, msg:{ type:'validate', tag:'main' } })).json);
  rows.push({ id:c.id, template:c.template, cls:(v.result?.issues??[]).length ? 'invalid' : 'ok', detail:(v.result?.issues??[]).map((x)=>x.code).join(','), vol:m.result?.volume, ref:c.ref });
}
const rp = '/tmp/feature_sweep_cases.json', ro = '/tmp/feature_sweep_ref.json';
writeFileSync(rp, JSON.stringify(refCases, null, 1));
try { execFileSync('python3.12', [path.join(here,'feature_sweep_ref.py'), '--cases', rp, '--out', ro], {stdio:'pipe', timeout:180000}); } catch(e){ console.error('ref failed', e.message); }
const ref = existsSync(ro) ? JSON.parse(readFileSync(ro,'utf8')) : {};
for (const row of rows) {
  if (row.cls !== 'ok') continue;
  const r = ref[row.id];
  if (!r) { row.cls='no-ref'; continue; }
  if (r.error) { row.cls='ref-fails'; row.detail=r.error; continue; }
  const rel = Math.abs(row.vol - r.volume) / Math.max(Math.abs(r.volume),1e-9);
  row.rel = rel;
  if (rel > 1e-3) { row.cls='wrongVolume'; row.detail=`rel ${rel.toExponential(2)} vol ${row.vol.toFixed(3)} ref ${r.volume.toFixed(3)}`; }
}
const g={}; for(const r of rows){g[r.template]??={};g[r.template][r.cls]=(g[r.template][r.cls]??0)+1;}
const tot={}; for(const r of rows)tot[r.cls]=(tot[r.cls]??0)+1;
console.log(JSON.stringify({n:rows.length,seed,samples,wasmHash,...tot}));
for(const [t,x] of Object.entries(g)) console.log(`${t}: ${JSON.stringify(x)}`);
const out = opt('--json'); if(out) writeFileSync(out, JSON.stringify(rows,null,1));
if(baselineFile && (opt('--update-baseline')||opt('--check'))){
  if(opt('--update-baseline')){writeFileSync(baselineFile,JSON.stringify({version:1,wasm:opt('--wasm','kit'),wasmHash,seed,samples,cases:Object.fromEntries(rows.map((r)=>[r.id,r.cls]))},null,1)+'\n');console.log(`wrote ${baselineFile}`);}
  else{const base=JSON.parse(readFileSync(baselineFile,'utf8')).cases??{};const reg=rows.filter((r)=>base[r.id]==='ok'&&r.cls!=='ok');console.log(`feature_sweep check: ${reg.length} regressions`);if(reg.length)process.exit(1);}
}
