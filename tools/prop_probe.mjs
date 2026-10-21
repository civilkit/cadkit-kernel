// Propeller foundation probes: ours vs original, minimal feature documents.
import { readFileSync, writeFileSync, existsSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', fileURLToPath(new URL('../../kit/apps/cad/public/kernel/ck_wasm.wasm', import.meta.url)));
const orig = opt('--orig', fileURLToPath(new URL('../../kernel.wasm', import.meta.url)));
const outDir = opt('--out-dir', path.join(here, 'cases/prop_probe'));
const hash = (p) => crypto.createHash('sha256').update(readFileSync(p)).digest('hex').slice(0, 16);
const ours = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const origK = new Kernel(new WebAssembly.Module(readFileSync(orig)));
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2) => ({ type: 'line', id, p1, p2 });
const circ = (id, c, r) => ({ type: 'circle', id, c, r });

function naca(m, p, t, n = 12) {
  const pts = [];
  const yt = (x) => 5 * t * (0.2969 * Math.sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
  const yc = (x) => x < p ? (m / (p * p)) * (2 * p * x - x * x) : (m / ((1 - p) ** 2)) * ((1 - 2 * p) + 2 * p * x - x * x);
  const dyc = (x) => x < p ? (2 * m / (p * p)) * (p - x) : (2 * m / ((1 - p) ** 2)) * (p - x);
  const xs = Array.from({ length: n + 1 }, (_, i) => 0.5 * (1 - Math.cos(Math.PI * i / n)));
  for (const x of xs) { const th = Math.atan(dyc(x)), y = yc(x) + yt(x) * Math.cos(th), xo = x - yt(x) * Math.sin(th); pts.push([xo, y]); }
  for (let i = xs.length - 2; i >= 1; i--) { const x = xs[i], th = Math.atan(dyc(x)), y = yc(x) - yt(x) * Math.cos(th), xo = x + yt(x) * Math.sin(th); pts.push([xo, y]); }
  return pts;
}
function polygon(c, t) {
  const N = 10, pts = [];
  for (let i = 0; i <= N; i++) { const u = -1 + 2 * i / N; pts.push([u * c / 2, t * (1 - u * u)]); }
  for (let i = N - 1; i >= 1; i--) { const u = -1 + 2 * i / N; pts.push([u * c / 2, -0.35 * t * (1 - u * u)]); }
  return pts;
}
function rot(pts, a) { const c = Math.cos(a), s = Math.sin(a); return pts.map(([x,y]) => [x*c-y*s, x*s+y*c]); }

function sections(rs, shape, P = 90, guide = false) {
  const f = [], ids = [];
  rs.forEach((r, i) => {
    f.push({ type:'plane', id:`P${i}`, mode:'offset', base:{kind:'std',id:'right'}, offset:r });
    const a = Math.atan2(P, 2 * Math.PI * r);
    let pts;
    if (shape === 'spline') pts = rot(naca(0.04, 0.4, 0.12, 10).map(([x,y])=>[x*24-12,y*24]), a);
    else pts = rot(polygon(24, 3.4), a);
    const P2 = pts.map(([x,y],j)=>pt(`p${i}_${j}`,x,y));
    const sid = `S${i}`;
    if (shape === 'spline') f.push({ type:'sketch', id:sid, plane:{kind:'feature',id:`P${i}`}, points:P2, entities:[{type:'spline',id:'e1',pts:P2.map(p=>p.id),closed:true}], constraints:[] });
    else f.push({ type:'sketch', id:sid, plane:{kind:'feature',id:`P${i}`}, points:P2, entities:P2.map((_,j)=>line(`e${j}`,P2[j].id,P2[(j+1)%P2.length].id)), constraints:[] });
    ids.push(sid);
  });
  f.push({ type:'loft', id:'F8', op:'add', sketches:ids });
  return { features:f, ids };
}
function hub() {
  return [{ type:'sketch', id:'H1', plane:{kind:'std',id:'front'}, points:[pt('c',0,0)], entities:[circ('e1','c',12)], constraints:[] },
    { type:'extrude', id:'H2', op:'add', sketch:'H1', end:'blind', depth:26 }];
}
const cases = [];
function add(id, cap, features, meta = {}) { cases.push({ id, cap, features, ...meta }); }
const rs8 = [9, 16, 23, 30, 37, 44, 50, 55];
const rs12 = [9, 13, 17, 21, 25, 30, 35, 40, 44, 48, 52, 55];
{
  const a = sections(rs12, 'spline'); add('cap1_loft_spline12', 1, a.features, { numeric: rs12, shape:'spline' });
  const b = sections([9,20,32,43,55], 'spline'); add('cap1_loft_spline5', 1, b.features, { numeric: [9,20,32,43,55], shape:'spline' });
  const c = sections(rs8, 'polygon'); add('cap1_loft_polygon8', 1, c.features, { numeric: rs8, shape:'polygon' });
  const d = sections([8, 9, 16, 23, 30, 37, 44, 55], 'polygon'); add('cap1_root_hub_8', 1, [...hub(), ...d.features], { rootHub:true });
  const ds = sections([8, 9, 16, 23, 30, 37, 44, 55], 'spline'); add('cap1_root_hub_spline8', 1, [...hub(), ...ds.features], { rootHub:true });
}
{
  // guides: two stations plus a leading/trailing guide line
  const f = [];
  for (const [i, r] of [9, 55].entries()) {
    f.push({ type:'plane', id:`P${i}`, mode:'offset', base:{kind:'std',id:'right'}, offset:r });
    const pts = polygon(24, 3.4).map(([x,y],j)=>pt(`p${i}_${j}`,x,y));
    f.push({ type:'sketch', id:`S${i}`, plane:{kind:'feature',id:`P${i}`}, points:pts, entities:pts.map((_,j)=>line(`e${j}`,pts[j].id,pts[(j+1)%pts.length].id)), constraints:[] });
  }
  f.push({ type:'sketch', id:'G1', plane:{kind:'std',id:'front'}, points:[pt('a',-12,0),pt('b',-12,46)], entities:[line('e1','a','b')], constraints:[] });
  f.push({ type:'loft', id:'F8', op:'add', sketches:['S0','S1'], guides:[{sketch:'G1'}] });
  add('cap3_loft_guides', 3, f);
}
{
  const P0=[[-10,-10],[10,-10],[10,10],[-10,10]].map(([x,y],i)=>pt(`p${i}`,x,y));
  const E0=[0,1,2,3].map(i=>line(`e${i}`,`p${i}`,`p${(i+1)%4}`));
  const P1=[[-5,-5],[5,-5],[5,5],[-5,5]].map(([x,y],i)=>pt(`q${i}`,x,y));
  const E1=[0,1,2,3].map(i=>line(`f${i}`,`q${i}`,`q${(i+1)%4}`));
  const f=[{type:'sketch',id:'S0',plane:{kind:'std',id:'top'},points:P0,entities:E0,constraints:[]},
    {type:'plane',id:'P1',mode:'offset',base:{kind:'std',id:'top'},offset:30},
    {type:'sketch',id:'S1',plane:{kind:'feature',id:'P1'},points:P1,entities:E1,constraints:[]},
    {type:'sketch',id:'G1',plane:{kind:'std',id:'front'},points:[pt('a',10,0),pt('b',5,30)],entities:[line('e1','a','b')],constraints:[]},
    {type:'loft',id:'F5',op:'add',sketches:['S0','S1'],guides:[{sketch:'G1'}]}];
  add('cap3_loft_guides_valid',3,f);
}
{
  // helix + sweep
  const f = [{ type:'sketch', id:'H1', plane:{kind:'std',id:'front'}, points:[pt('c',0,0)], entities:[circ('e1','c',4)], constraints:[] },
    { type:'helix', id:'H2', sketch:'H1', definedBy:'pitchRevs', pitch:30, revolutions:3, height:90, startAngle:0, leftHand:false, reverse:false },
    { type:'sketch', id:'S1', plane:{kind:'std',id:'top'}, points:[pt('c',4,0)], entities:[circ('e1','c',1)], constraints:[] },
    { type:'sweep', id:'W1', op:'add', sketch:'S1', path:{kind:'curve',id:'H2'} }];
  add('cap4_helix_sweep', 4, f);
}
{
  // a ruled helicoid sheet: radial line sections, each rotated by the pitch angle, lofted
  // as a surface (capabilities 5 and 6 share it)
  const R = 20, PITCH = 20, N = 8, H = PITCH * 2;
  const f = [];
  for (let k = 0; k <= N; k++) {
    const z = H * k / N, th = 2 * Math.PI * z / PITCH;
    f.push({ type:'plane', id:`P${k}`, mode:'offset', base:{kind:'std',id:'front'}, offset:z });
    f.push({ type:'sketch', id:`S${k}`, plane:{kind:'feature',id:`P${k}`},
      points:[pt('a',0,0), pt('b', R*Math.cos(th), R*Math.sin(th))], entities:[line('e0','a','b')], constraints:[] });
  }
  f.push({ type:'loft', id:'F9', surface:true, sketches: Array.from({length:N+1},(_,k)=>`S${k}`) });
  add('cap5_ruled_helicoid', 5, f, { radius:R, pitch:PITCH, stations:N+1 });
  const o = [[-6,-6],[6,-6],[6,6],[-6,6]].map(([x,y],i)=>pt(`o${i}`,x,y));
  const g = [...f,
    { type:'sketch', id:'O1', plane:{kind:'std',id:'front'}, points:o, entities:o.map((_,i)=>line(`l${i}`,`o${i}`,`o${(i+1)%4}`)), constraints:[] },
    { type:'projectCurve', id:'PC1', sketch:'O1', faces:[], degree:3, tolerance:0.01, direction:'both' },
    { type:'surfaceExtrude', id:'SE1', sketch:'O1', end:'blind', depth:2*H, direction:'auto' },
    { type:'trimSurface', id:'T1', surface:'F9', toolSurface:'SE1', deleteTool:true }];
  add('cap6_helicoid_project_trim', 6, g);
}
{
  // open-section loft sheet + thicken
  const f = [];
  for (let i = 0; i < 5; i++) {
    const z = i * 12;
    f.push({ type:'plane', id:`P${i}`, mode:'offset', base:{kind:'std',id:'front'}, offset:z });
    const pts = [0,1,2,3,4].map(j=>pt(`p${i}_${j}`, -15+j*7.5, 5*Math.sin(j*0.9)+i));
    f.push({ type:'sketch', id:`S${i}`, plane:{kind:'feature',id:`P${i}`}, points:pts, entities:pts.slice(0,-1).map((_,j)=>line(`e${j}`,pts[j].id,pts[j+1].id)), constraints:[] });
  }
  f.push({ type:'loft', id:'F8', op:'add', surface:true, sketches:['S0','S1','S2','S3','S4'] });
  f.push({ type:'thicken', id:'F9', surface:'F8', thickness:2, side:'both' });
  add('cap7_thicken_open_loft', 7, f);
  // the same sheet with the new variable thickness: 4 mm at the root (z=0), 1 mm at the tip
  add('cap7_variable_thicken', 7, [...f.slice(0,-1), { type:'thicken', id:'F9', surface:'F8', thicknessAt:{ along:[0,0,1], stops:[[0,4],[48,1]] }, side:'both' }]);
}
{
  // root fillet: hub + blade + the blade-root outline (12 edges where the loft meets the
  // cylinder) filleted at r=2. Face names come from the fused body (`F8.S.e*` blade faces,
  // `H2.S.e1` the hub cylinder); the old pick `F8.S.e0_0` named no face (`missingEdge` on
  // both kernels before any blend ran).
  const d = sections([9, 20, 32, 43, 55], 'polygon');
  const root = [['F8.S.e15', [11.8, -2.2, 0.29]], ['F8.S.e16', [11.49, -3.47, 1.35]], ['F8.S.e17', [10.75, -5.34, 2.81]],
    ['F8.S.e18', [9.66, -7.12, 3.99]], ['F8.C0~1', [9, -7.94, 5.29]], ['F8.S.e0', [9.25, -7.64, 6.23]],
    ['F8.S.e1', [10.44, -5.92, 6.23]], ['F8.S.e2', [11.57, -3.18, 5.62]], ['F8.S.e3', [11.97, -0.89, 4.59]],
    ['F8.S.e4', [11.95, 1.14, 3.31]], ['F8.S.e5', [11.63, 2.97, 1.86]], ['F8.S.e6', [11.16, 4.41, 0.55]]];
  add('cap8_fillet_blade_hub', 8, [...hub(), ...d.features,
    { type:'fillet', id:'F9', radius:2, edges: root.map(([faces, at]) => ({ faces: [faces, 'H2.S.e1'], at })) }]);
}
for (const c of cases) { const p = path.join(outDir, `${c.id}.json`); writeFileSync(p, JSON.stringify({ id:c.id, cap:c.cap, features:c.features, ...(c.numeric?{numeric:c.numeric}:{}), ...(c.rootHub?{rootHub:true}:{}) }, null, 1)); }

function run(k, features) {
  const r = JSON.parse(k.request(JSON.stringify({ id:1, msg:{ type:'regen', tag:'m', part:{ format:'ck-part', version:1, features }, opts:{ solveSketches:true } } })).json);
  if (r.trap) return { cls:'trap', detail:r.trap };
  const errs = (r.result?.errors ?? []).map(([,e])=>({code:e.code,detail:e.params?.detail}));
  if (errs.length) return { cls:errs[0].code, detail:errs[0].detail };
  const m = JSON.parse(k.request(JSON.stringify({ id:2, msg:{ type:'massProps', tag:'m', bodies:true } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id:3, msg:{ type:'validate', tag:'m' } })).json);
  return { cls:(v.result?.issues??[]).length ? 'invalid' : 'ok', detail:(v.result?.issues??[]).map(x=>x.code).join(','), volume:m.result?.volume, area:m.result?.area };
}
const rows = [];
for (const c of cases) {
  const o = run(ours, c.features), g = run(origK, c.features);
  let outcome = 'both-fail';
  if (o.cls==='ok' && g.cls==='ok') outcome = 'both-ok';
  else if (o.cls==='ok') outcome = 'ours-only';
  else if (g.cls==='ok') outcome = 'orig-only';
  let rel = null;
  if (o.volume != null && g.volume != null) rel = Math.abs(o.volume-g.volume)/Math.max(Math.abs(g.volume),1e-9);
  rows.push({ id:c.id, cap:c.cap, outcome, ours:o.cls, orig:g.cls, detail:o.detail ?? g.detail, volumeOurs:o.volume, volumeOrig:g.volume, rel });
}
console.log(JSON.stringify({ wasmHash:hash(wasm), origHash:hash(orig), ...Object.fromEntries(Object.entries(rows.reduce((a,r)=>(a[r.outcome]=(a[r.outcome]??0)+1,a),{}))) }));
for (const r of rows) console.log(`${r.id}: ${r.outcome} ours=${r.ours} orig=${r.orig} vol=${r.volumeOurs ?? '-'} / ${r.volumeOrig ?? '-'}${r.rel!=null?` rel=${r.rel.toExponential(2)}`:''} ${r.detail??''}`);
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows,null,1));
