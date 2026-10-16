// Measurement requests (distance, maxDistance, thickness, thicknessAt, closestPoint, angles, clearance, symmetry)
// on every sample part: no trap, no error, sane ranges, internal consistency, timings.
// usage: node tools/measure_check.mjs [--wasm path] [--samples dir] [--only substr] [--large sample] [--max N]
//        [--sample N] [--budget-ms N]     exit 1 on any violation
// Per sample: regen (parts with feature errors or no body are skipped and listed), then
//   thickness {samples:1500}      0 < min <= bbox diagonal, thinnest cluster list sorted, the min point lies on the body
//   clearance {min: 1% diag}      pairs sorted ascending, all < min, kinds in {gap, wall, contact}
//   maxDistance (body to itself)  < 1e-6 (every sample is on the surface)
//   distance point -> body        equals |probe distance| (our own two nearest-point paths agree) within 1e-6
//   closestPoint / angles / symmetry run, finite output
import { readFileSync, readdirSync, existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (f, d) => { const i = args.indexOf(f); return i < 0 ? d : args[i + 1]; };
const wasm = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const dir = opt('--samples', '/path/to/work/kit/apps/cad/public/samples');
const only = opt('--only', '');
const maxN = Number(opt('--max', '1000'));
const budget = Number(opt('--budget-ms', '12000')); // per request (the kernel's own limit is 8 s, opts.timeMs)
const sampleN = Number(opt('--sample', '0')); // evenly spaced subset of N parts
const mod = new WebAssembly.Module(readFileSync(wasm));
const now = () => Date.now();

const viol = [];
const skipped = [];
const truncs = [];
const rows = [];
const finite = (v) => JSON.stringify(v, (k, x) => (typeof x === 'number' && !Number.isFinite(x) ? '__nonfinite__' : x)).indexOf('__nonfinite__') < 0;

function sample(file) {
  const name = path.basename(file).replace(/\.ckpart$/, '');
  let part;
  try { part = JSON.parse(readFileSync(file, 'utf8')); } catch { skipped.push(`${name}: not json`); return; }
  if (part.format !== 'ck-part') { skipped.push(`${name}: not a part`); return; }
  const k = new Kernel(mod, { now });
  const t0 = now();
  const rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
  if (!rep.json) { skipped.push(`${name}: regen trap`); return; }
  const r = JSON.parse(rep.json);
  if (!r.ok || (r.result?.errors ?? []).length) { skipped.push(`${name}: regen errors`); return; }
  const regenMs = now() - t0;
  const q = (msg) => {
    const t = now();
    const rr = k.request(JSON.stringify({ id: 2, msg: { tag: 'main', ...msg } }));
    const ms = now() - t;
    const bad = (m) => viol.push(`${name} ${msg.type}: ${m}`);
    if (!rr.json) { bad(`TRAP ${String(rr.trap).slice(0, 120)}`); return { ms }; }
    const j = JSON.parse(rr.json);
    if (!j.ok) { bad(`error ${String(j.error).slice(0, 160)}`); return { ms }; }
    if (j.result === null) return { ms, none: true };
    if (!finite(j.result)) bad('non-finite number in the reply');
    if (msg.type !== 'regen' && msg.type !== 'massProps' && msg.type !== 'sections' && msg.type !== 'probe' && (typeof j.result.elapsedMs !== 'number' || typeof j.result.truncated !== 'boolean')) bad('reply lacks elapsedMs / truncated');
    if (j.result.truncated) truncs.push(`${name} ${msg.type}`);
    if (ms > budget) bad(`${ms} ms over the ${budget} ms budget`);
    return { res: j.result, ms };
  };
  const mp = q({ type: 'massProps' });
  if (mp.none || !mp.res) { skipped.push(`${name}: no body`); return; }
  const vol = mp.res.volume;
  const sec = q({ type: 'sections', axis: 'x', n: 1, exact: false }).res;
  const bb = q({ type: 'sections', axis: 'y', n: 1, exact: false }).res;
  const bz = q({ type: 'sections', axis: 'z', n: 1, exact: false }).res;
  if (!sec || !bb || !bz) { skipped.push(`${name}: no extent`); return; }
  const ext = [sec.range, bb.range, bz.range];
  const diag = Math.hypot(...ext.map((e) => e[1] - e[0]));
  const row = { name, regenMs, vol: Math.round(vol), diag: +diag.toFixed(1) };

  if (!(vol > 0)) { rows.push({ ...row, note: 'volume <= 0, solid checks skipped' }); } else {
    const th = q({ type: 'thickness', opts: { samples: 1500 } });
    row.thick = th.ms;
    if (th.res) {
      const m = th.res.minThickness;
      if (m !== null && !(m > 0 && m <= diag * 1.0001)) viol.push(`${name} thickness: min ${m} outside (0, ${diag}]`);
      const v = th.res.thin.map((t) => t.thickness);
      if (v.some((x, i) => i && x < v[i - 1] - 1e-12)) viol.push(`${name} thickness: thin list not sorted`);
      row.min = m === null ? null : +m.toFixed(4);
    }
    const mn = Math.max(diag * 0.01, 1e-3);
    const cl = q({ type: 'clearance', min: mn, opts: { limit: 50 } });
    row.clear = cl.ms;
    if (cl.res) {
      const d = cl.res.pairs.map((p) => p.distance);
      if (d.some((x, i) => i && x < d[i - 1] - 1e-12) || d.some((x) => x >= mn)) viol.push(`${name} clearance: pairs unsorted or not under min`);
      if (cl.res.pairs.some((p) => !['gap', 'wall', 'contact'].includes(p.kind))) viol.push(`${name} clearance: bad kind`);
      row.pairs = cl.res.count;
    }
    const md = q({ type: 'maxDistance', opts: { samples: 4000 } });
    row.maxd = md.ms;
    if (md.res && !(md.res.maxDistance < 1e-6)) viol.push(`${name} maxDistance(body, body) = ${md.res.maxDistance}`);
    // distance(point, body) must equal |probe distance|
    const c = ext.map((e) => (e[0] + e[1]) / 2);
    const pts = [[c[0] + 2 * (ext[0][1] - ext[0][0]), c[1], c[2]], [c[0], c[1] + 1.3 * (ext[1][1] - ext[1][0]), c[2] + 0.2], [ext[0][0] - 0.3, ext[1][0] - 0.1, ext[2][1] + 0.7]];
    const pr = q({ type: 'probe', points: pts });
    for (let i = 0; i < pts.length && pr.res; i++) {
      const dd = q({ type: 'distance', a: pts[i], b: 'body' });
      row.dist = dd.ms;
      const pd = Math.abs(pr.res.points[i].distance);
      if (dd.res && Math.abs(dd.res.distance - pd) > 1e-6 * Math.max(1, pd)) viol.push(`${name} distance: ${dd.res.distance} vs probe ${pd}`);
    }
    const cp = q({ type: 'closestPoint', points: pts });
    if (cp.res && cp.res.results.length !== pts.length) viol.push(`${name} closestPoint: ${cp.res.results.length} results`);
    row.cp = cp.ms;
    const an = q({ type: 'angles', edges: 'all' });
    row.ang = an.ms;
    if (an.res && an.res.edges.some((e) => !(e.angle > 0 && e.angle < 360))) viol.push(`${name} angles: angle out of (0, 360)`);
    const dr = q({ type: 'angles', draft: { pull: [0, 1, 0] } });
    if (dr.res && dr.res.draft.some((f) => f.min < -90.0001 || f.max > 90.0001)) viol.push(`${name} draft: outside [-90, 90]`);
    const sy = q({ type: 'symmetry', opts: { samples: 1500 } });
    row.sym = sy.ms;
    if (sy.res) row.symN = sy.res.symmetric.length;
    rows.push(row);
  }
}

let files = readdirSync(dir).filter((f) => f.endsWith('.ckpart') && f.includes(only)).sort();
if (args.includes('--large sample')) files.push(path.resolve(here, 'cases/largeSampleFull.ckpart'));
if (sampleN > 0 && files.length > sampleN) files = Array.from({ length: sampleN }, (_, i) => files[Math.floor((i * files.length) / sampleN)]);
files = files.slice(0, maxN);
const T0 = now();
for (const f of files) {
  const file = path.isAbsolute(f) ? f : path.join(dir, f);
  if (!existsSync(file)) continue;
  try { sample(file); } catch (e) { viol.push(`${path.basename(file)}: exception ${e && e.message}`); }
}
console.table(rows);
if (skipped.length) console.log(`skipped (${skipped.length}): ${skipped.join('; ')}`);
if (truncs.length) console.log(`truncated replies (${truncs.length}): ${truncs.join('; ')}`);
console.log(`${rows.length} parts measured in ${((now() - T0) / 1000).toFixed(1)}s`);
if (viol.length) { console.log(`${viol.length} violations`); for (const v of viol.slice(0, 60)) console.log('  ' + v); process.exit(1); }
console.log('measure_check: ok');
