// Tessellation accuracy: display mesh volume/area vs exact mass properties.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', '/path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm');
const seed = Number(opt('--seed', '7'));
const samples = Number(opt('--samples', '20'));
const only = opt('--only', '');
const mod = new WebAssembly.Module(readFileSync(wasm));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const k = new Kernel(mod);
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'tess_sweep_baseline.json')));
let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
const circ = (id, c, r) => ({ type: 'circle', id, c, r });
let n = 0;
const plane = (z) => z === 0 ? [{ kind: 'std', id: 'front' }, []] : [{ kind: 'feature', id: `P${++n}` }, [{ type: 'plane', id: `P${n}`, mode: 'offset', base: { kind: 'std', id: 'front' }, offset: z }]];
const circleSk = (id, r, z) => { const [pl, pre] = plane(z); return [...pre, { type: 'sketch', id, plane: pl, points: [pt('c', 0, 0)], entities: [circ('e1', 'c', r)], constraints: [] }]; };
function rectSk(id, w, d, z) {
  const P = [[-w/2,-d/2],[w/2,-d/2],[w/2,d/2],[-w/2,d/2]].map(([x,y],i)=>pt(`p${i}`,x,y));
  const E = [0,1,2,3].map((i)=>line(`e${i+1}`,`p${i}`,`p${(i+1)%4}`));
  const [pl, pre] = plane(z); return [...pre, { type:'sketch', id, plane: pl, points: P, entities: E, constraints: [] }];
}
const ex = (id, sid, depth, extra = {}) => ({ type: 'extrude', id, op: 'add', sketch: sid, end: 'blind', depth, ...extra });
let lastGeom = null;
function build(t) {
  n = 0;
  if (t === 'box') { const w = rr(15,50), d = rr(15,50), h = rr(10,50); lastGeom = { kind:'box', w,d,h }; const f = rectSk('S1', w, d, 0); return [...f, ex('F2','S1',h)]; }
  if (t === 'cyl') { const r = rr(5,25), h = rr(10,50); lastGeom = { kind:'cyl', r,h }; const f = circleSk('S1', r, 0); return [...f, ex('F2','S1',h)]; }
  if (t === 'cone') { const r = rr(8,25), h = rr(10,50), draft = rr(0, Math.min(18, Math.atan((r-0.5)/h)*180/Math.PI*0.8)); lastGeom = { kind:'cone', r,h,draft }; const f = circleSk('S1', r, 0); return [...f, ex('F2','S1',h,{draft})]; }
  if (t === 'sphere') { const r = rr(5,25); return [{ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:[pt('c',0,0),pt('s',0,-r),pt('t',0,r),pt('a1',0,0),pt('a2',0,r)], entities:[{type:'arc',id:'e1',c:'c',p1:'s',p2:'t'},line('e2','t','s'),line('ax','a1','a2',{construction:true})], constraints:[] }, { type:'revolve', id:'F2', op:'add', sketch:'S1', axis:'ax', angle:360 }]; }
  if (t === 'torus') { const R = rr(10,30), r = rr(2,Math.min(8,R*0.5)); return [{ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:[pt('c',R,0),pt('a1',0,0),pt('a2',0,2*r+1)], entities:[circ('e1','c',r),line('ax','a1','a2',{construction:true})], constraints:[] }, { type:'revolve', id:'F2', op:'add', sketch:'S1', axis:'ax', angle:360 }]; }
  if (t === 'tube') { const ro = rr(12,35), ri = ro*rr(0.3,0.8), h = rr(10,45); const f = circleSk('S1', ro, 0); f[0].entities.push(circ('e2','c',ri)); return [...f, ex('F2','S1',h)]; }
  if (t === 'loft2' || t === 'loft3') { const N = t === 'loft3' ? 3 : 2; const f = [], ids = [], secs = []; let z = 0; for (let j=0;j<N;j++){ const id=`S${j}`, r=rr(5,20); f.push(...circleSk(id,r,z)); ids.push(id); secs.push({r,z}); z += rr(10,30); } lastGeom = { kind:'loft', secs }; f.push({type:'loft',id:'L1',op:'add',sketches:ids}); return f; }
  const h = rr(10,40), w0 = rr(2,15), w1 = rr(2,15), ang = rr(90,360);
  return [{ type:'sketch', id:'S1', plane:{kind:'std',id:'front'}, points:[pt('p0',w0,0),pt('p1',w1,h),pt('p2',0,h),pt('p3',0,0),pt('a1',0,0),pt('a2',0,h)], entities:[line('e1','p0','p1'),line('e2','p1','p2'),line('e3','p2','p3'),line('e4','p3','p0'),line('ax','a1','a2',{construction:true})], constraints:[] }, { type:'revolve', id:'R1', op:'add', sketch:'S1', axis:'ax', angle:ang }];
}
const templates = ['box','cyl','cone','sphere','torus','tube','loft2','loft3','revolve'];
const rows = [];
for (let i = 0; i < samples; i++) for (const t of templates) {
  const id = `${t}_${i}`;
  const features = build(t);
  if (only && !id.includes(only)) continue;
  let r;
  try { r = k.request(JSON.stringify({ id:1, msg:{ type:'regen', tag:'main', part:{ format:'ck-part', version:1, features }, opts:{ solveSketches:true } } })); } catch { rows.push({ id, template:t, cls:'trap' }); continue; }
  if (r.trap) { rows.push({ id, template:t, cls:'trap', detail:r.trap }); continue; }
  const jl = new DataView(r.bytes.buffer).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const j = JSON.parse(r.json);
  if (!j.ok || j.result?.errors?.length) { rows.push({ id, template:t, cls:'regen-fail' }); continue; }
  const m = JSON.parse(k.request(JSON.stringify({ id:2, msg:{ type:'massProps', tag:'main', bodies:true } })).json).result;
  const mesh = j.result?.body?.mesh;
  if (!mesh) { rows.push({ id, template:t, cls:'no-mesh' }); continue; }
  const get = (v) => { const C = { f32: Float32Array, i32: Int32Array }[v.type]; return new C(r.bytes.buffer.slice(base + v.offset, base + v.offset + v.len * C.BYTES_PER_ELEMENT)); };
  const pos = get(mesh.positions ?? mesh.pos), tf = get(mesh.triFace);
  let area = 0, vol = 0;
  const kindArea = {}, faceTris = {}, kindEdge = {}, kindSag = {};
  const faceKind = (fi) => j.result.body.faces[fi]?.kind ?? 'unknown';
  for (let i2 = 0; i2 < tf.length; i2++) {
    const o = i2 * 9;
    const ax = pos[o], ay = pos[o+1], az = pos[o+2], bx = pos[o+3], by = pos[o+4], bz = pos[o+5], cx = pos[o+6], cy = pos[o+7], cz = pos[o+8];
    const ux = bx-ax, uy = by-ay, uz = bz-az, vx = cx-ax, vy = cy-ay, vz = cz-az;
    const nx = uy*vz-uz*vy, ny = uz*vx-ux*vz, nz = ux*vy-uy*vx;
    const a2 = 0.5 * Math.hypot(nx, ny, nz);
    area += a2;
    vol += (ax*(by*cz-bz*cy) + ay*(bz*cx-bx*cz) + az*(bx*cy-by*cx)) / 6;
    const kk = faceKind(tf[i2]); kindArea[kk] = (kindArea[kk] ?? 0) + a2; faceTris[tf[i2]] = (faceTris[tf[i2]] ?? 0) + 1;
    if (lastGeom && (kk === 'cone' || kk === 'cylinder')) {
      const rOf = (z) => lastGeom.kind === 'cyl' ? lastGeom.r : lastGeom.kind === 'cone' ? lastGeom.r - z*Math.tan(lastGeom.draft*Math.PI/180) : lastGeom.kind === 'loft' ? (()=>{const S=lastGeom.secs; for(let q=1;q<S.length;q++) if(z>=S[q-1].z && z<=S[q].z) return S[q-1].r + (S[q].r-S[q-1].r)*(z-S[q-1].z)/(S[q].z-S[q-1].z); return S[S.length-1].r;})() : null;
      for (const [p,q] of [[[ax,ay,az],[bx,by,bz]],[[bx,by,bz],[cx,cy,cz]],[[cx,cy,cz],[ax,ay,az]]]) { const mx=(p[0]+q[0])/2,my=(p[1]+q[1])/2,mz=(p[2]+q[2])/2; const rr0=rOf(mz); if(rr0!=null) kindSag[kk]=Math.max(kindSag[kk]??0, Math.max(0, rr0 - Math.hypot(mx,my))); }
    }
    const e1 = Math.hypot(bx-ax,by-ay,bz-az), e2 = Math.hypot(cx-bx,cy-by,cz-bz), e3 = Math.hypot(ax-cx,ay-cy,az-cz);
    kindEdge[kk] = Math.max(kindEdge[kk] ?? 0, e1, e2, e3);
  }
  const rv = Math.abs(Math.abs(vol) - m.volume) / Math.max(m.volume, 1e-9);
  const ra = Math.abs(area - m.area) / Math.max(m.area, 1e-9);
  rows.push({ id, template:t, cls: rv > 1e-3 ? 'meshVolume' : ra > 1e-3 ? 'meshArea' : 'ok', relV:rv, relA:ra, tris:tf.length, kinds:[...new Set(Object.keys(kindArea))], kindArea, faceTris:Object.fromEntries(Object.keys(kindArea).map((kk)=>[kk,Object.entries(faceTris).filter(([fi])=>faceKind(+fi)===kk).reduce((a,[,c])=>a+c,0)])), kindEdge, kindSag });
}
const g = {}; for (const r of rows) { g[r.template] ??= { n:0, cls:{}, maxV:0, maxA:0, kinds:new Set() }; const x=g[r.template]; x.n++; x.cls[r.cls]=(x.cls[r.cls]??0)+1; x.maxV=Math.max(x.maxV,r.relV??0); x.maxA=Math.max(x.maxA,r.relA??0); for (const kk of r.kinds??[]) x.kinds.add(kk); }
const tot = {}; for (const r of rows) tot[r.cls] = (tot[r.cls]??0)+1;
console.log(JSON.stringify({ n:rows.length, seed, samples, wasmHash, ...tot }));
for (const [t,x] of Object.entries(g)) console.log(`${t}: ${JSON.stringify(x.cls)} maxV=${x.maxV.toExponential(2)} maxA=${x.maxA.toExponential(2)} kinds=${[...x.kinds].join(',')}`);
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows, null, 1));
if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version:1, wasm:opt('--wasm','kit'), wasmHash, seed, samples, cases:Object.fromEntries(rows.map((r)=>[r.id,r.cls])) }, null, 1)+'\n'); console.log(`wrote ${baselineFile}`); }
  else { const base = JSON.parse(readFileSync(baselineFile,'utf8')).cases ?? {}; const reg = rows.filter((r)=>base[r.id]==='ok' && r.cls!=='ok'); console.log(`tess_sweep check: ${reg.length} regressions`); if (reg.length) process.exit(1); }
}
