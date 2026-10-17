// projectCurve corpus (tools/cases/project_curve_cases.mjs): every case through the ORACLE kernel and ours.
// Per case: ok / error code (+ params), curve count, total length (marker.length and the polyline of the output
// curve), endpoints, deviation <= tolerance, Hausdorff distance between the two output polylines, and for our
// result that the output curve lies on the body (closestPoint distance of the samples).
// usage: node tools/project_curve_corpus.mjs [--wasm ours] [--only substr] [--orig-only] [--verbose] [--quiet] [--cache-all]
//        [--allow file.json]   known differences: { "<case>": "<reason>" } (those do not fail the run)
// exit 1 when a case differs without being in the allow list (or ours traps / puts curve points off the body).
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import { cases } from './cases/project_curve_cases.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const arg = (n) => { const i = process.argv.indexOf(n); return i > 0 ? process.argv[i + 1] : null; };
const has = (n) => process.argv.includes(n);
const wasm = arg('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const only = arg('--only');
const origOnly = has('--orig-only');
const verbose = has('--verbose');
const cacheAll = has('--cache-all'); // regen-cache identity on every case (default: every fourth)
const quiet = has('--quiet');
const allow = arg('--allow') ? JSON.parse(readFileSync(arg('--allow'), 'utf8')) : (() => { try { return JSON.parse(readFileSync(path.join(here, 'cases/project_curve_known.json'), 'utf8')); } catch { return {}; } })();
const oracleMod = new WebAssembly.Module(readFileSync(path.join(here, '../../kernel.wasm')));
const oursMod = origOnly ? null : new WebAssembly.Module(readFileSync(wasm));
const rq = (k, msg) => { const r = k.request(JSON.stringify({ id: 1, msg })); if (r.trap) return { trap: r.trap }; try { return JSON.parse(r.json); } catch { return { trap: 'unparsable reply' }; } };

const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const polyLen = (p) => p.slice(1).reduce((s, q, i) => s + dist(p[i], q), 0);
// resample a polyline to n points by arc length
function resample(p, n) {
  const L = polyLen(p); const out = []; let seg = 0, acc = 0;
  for (let i = 0; i <= n; i++) {
    const s = (L * i) / n;
    while (seg < p.length - 2 && acc + dist(p[seg], p[seg + 1]) < s) { acc += dist(p[seg], p[seg + 1]); seg++; }
    const l = dist(p[seg], p[seg + 1]) || 1; const t = Math.min(1, Math.max(0, (s - acc) / l));
    out.push([0, 1, 2].map((q) => p[seg][q] + (p[seg + 1][q] - p[seg][q]) * t));
  }
  return out;
}
const toSegDist = (q, p) => { let m = Infinity; for (let i = 0; i < p.length - 1; i++) { const a = p[i], b = p[i + 1]; const ab = [0, 1, 2].map((k) => b[k] - a[k]); const l2 = ab.reduce((s, x) => s + x * x, 0) || 1; let t = ([0, 1, 2].reduce((s, k) => s + (q[k] - a[k]) * ab[k], 0)) / l2; t = Math.min(1, Math.max(0, t)); m = Math.min(m, dist(q, [0, 1, 2].map((k) => a[k] + ab[k] * t))); } return m; };
const hausdorff = (a, b) => Math.max(...resample(a, 200).map((q) => toSegDist(q, b)), ...resample(b, 200).map((q) => toSegDist(q, a)));

function run(mod, part) {
  const k = new Kernel(mod);
  const r = rq(k, { type: 'regen', tag: 'main', part });
  if (r.trap) return { trap: r.trap };
  if (!r.ok && r.error) return { fail: JSON.stringify(r.error).slice(0, 200) };
  const res = r.result ?? {};
  const out = { errors: (res.errors ?? []).map(([id, e]) => ({ id, code: e.code, params: e.params })) };
  const pcs = part.features.filter((f) => f.type === 'projectCurve').map((f) => f.id);
  const id = pcs[pcs.length - 1];
  const cur = (res.curves ?? []).filter(([cid]) => cid === id);
  out.curves = cur.length;
  if (cur.length) out.pts = cur[0][1].pts;
  const mk = (res.markers ?? []).find(([mid]) => mid === id);
  if (mk) out.marker = mk[1];
  if (!origOnly && out.pts && res.body) {
    const sample = resample(out.pts, 40);
    const cp = rq(k, { type: 'closestPoint', tag: 'main', points: sample });
    const ds = (cp.result?.results ?? []).map((x) => x?.nearest?.distance).filter((x) => typeof x === 'number');
    out.offBody = ds.length ? Math.max(...ds) : null;
    out.cpRaw = ds.length ? undefined : JSON.stringify(cp).slice(0, 200);
  }
  return out;
}

// error params equal; projectCurveSeveral lists chain ends in no fixed order: compare count and the set of points
function sameParams(ea, eb) {
  if (ea.length !== eb.length) return false;
  return ea.every((x, i) => {
    const y = eb[i], px = x.params ?? {}, py = y.params ?? {};
    if (x.code === 'projectCurveSeveral') {
      if (px.count !== py.count || (px.points?.length ?? 0) !== (py.points?.length ?? 0)) return false;
      const left = [...(py.points ?? [])];
      return (px.points ?? []).every((p) => { const j = left.findIndex((q) => dist(p, q) < 2e-3); if (j < 0) return false; left.splice(j, 1); return true; });
    }
    return JSON.stringify(px) === JSON.stringify(py);
  });
}
const tolOf = (part) => { const f = part.features.filter((x) => x.type === 'projectCurve').pop(); return typeof f?.tolerance === 'number' ? Math.max(f.tolerance, 1e-9) : 0.001; };
const codeStr = (o) => (o.trap ? `TRAP ${o.trap.slice(0, 80)}` : o.fail ? `FAIL ${o.fail}` : o.errors.length ? o.errors.map((e) => `${e.id}:${e.code}`).join(',') : 'ok');


// ---- regen cache: a warm regen, a cache-off regen and a regen after an edit of the projectCurve feature must be
// byte-identical to a cold regen in a fresh kernel (docs/regen-cache.md)
const align8 = (x) => (x + 7) & ~7;
function stripped(r) {
  const o = JSON.parse(r.json);
  if (o.result) delete o.result.cacheStats;
  const jl = new DataView(r.bytes.buffer, r.bytes.byteOffset).getUint32(0, true);
  return JSON.stringify(o) + Buffer.from(r.bytes.subarray(align8(4 + jl))).toString('base64');
}
const regenRaw = (k, part, opts) => k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, ...(opts ? { opts } : {}) } }));
function cacheProblems(part) {
  const out = [];
  const k = new Kernel(oursMod);
  const cold = regenRaw(k, part), warm = regenRaw(k, part);
  const nc = regenRaw(new Kernel(oursMod), part, { noCache: true });
  if (cold.trap || warm.trap || nc.trap) return ['trap in cache check'];
  if (stripped(cold) !== stripped(warm)) out.push('warm != cold');
  if (stripped(cold) !== stripped(nc)) out.push('noCache != cold');
  // edit the last projectCurve (tolerance) and compare the warm result to a fresh cold one
  const ed = JSON.parse(JSON.stringify(part));
  const pc = [...ed.features].reverse().find((f) => f.type === 'projectCurve');
  if (pc) {
    pc.tolerance = typeof pc.tolerance === 'number' ? pc.tolerance * 4 : 0.004;
    const w2 = regenRaw(k, ed), c2 = regenRaw(new Kernel(oursMod), ed);
    if (w2.trap || c2.trap) out.push('trap after edit');
    else if (stripped(w2) !== stripped(c2)) out.push('warm after edit != cold');
  }
  return out;
}
let cacheBad = 0;
let n = 0, same = 0, known = 0, bad = 0, okBoth = 0, errBoth = 0;
const rows = [];
for (const c of cases) {
  if (only && !c.name.includes(only)) continue;
  n++;
  const a = run(oracleMod, c.part);
  const t0 = Date.now();
  const b = origOnly ? null : run(oursMod, c.part);
  const ms = Date.now() - t0;
  const diffs = [];
  if (b) {
    if (b.trap) diffs.push(`ours traps: ${b.trap}`);
    else if (codeStr(a) !== codeStr(b)) diffs.push(`result: oracle ${codeStr(a)} vs ours ${codeStr(b)}`);
    else if (!a.errors?.length && !a.trap) {
      okBoth++;
      if (a.curves !== b.curves) diffs.push(`curve count ${a.curves} vs ${b.curves}`);
      if (a.pts && b.pts) {
        const loose = (a.marker?.deviation ?? 0) > 5e-3; // a fit the user asked to be coarse: the shapes may differ by that much
        const la = polyLen(a.pts), lb = polyLen(b.pts);
        if (!loose && Math.abs(la - lb) > 1e-3 * Math.max(la, 1e-9)) diffs.push(`length ${la.toFixed(4)} vs ${lb.toFixed(4)}`);
        if (!a.marker?.closed) {
          const e0 = dist(a.pts[0], b.pts[0]), e1 = dist(a.pts.at(-1), b.pts.at(-1));
          const r0 = dist(a.pts[0], b.pts.at(-1)), r1 = dist(a.pts.at(-1), b.pts[0]);
          if (!((e0 < 1e-2 && e1 < 1e-2) || (r0 < 1e-2 && r1 < 1e-2))) diffs.push(`endpoints off by ${Math.min(Math.max(e0, e1), Math.max(r0, r1)).toExponential(2)}`);
        }
        const h = hausdorff(a.pts, b.pts);
        if (h > Math.max(1e-2, 2 * tolOf(c.part), loose ? 0.5 : 0)) diffs.push(`hausdorff ${h.toExponential(2)}`);
      }
      if (a.marker && b.marker) {
        for (const key of ['pieces', 'closed', 'corners', 'degree']) if (JSON.stringify(a.marker[key]) !== JSON.stringify(b.marker[key])) diffs.push(`marker.${key} ${a.marker[key]} vs ${b.marker[key]}`);
        if (Math.abs(a.marker.length - b.marker.length) > 1e-3 * Math.max(a.marker.length, 1e-9)) diffs.push(`marker.length ${a.marker.length} vs ${b.marker.length}`);
        const tol = tolOf(c.part);
        if (b.marker.count < 400 && b.marker.deviation > tol * 1.0001 + 1e-9) diffs.push(`ours deviation ${b.marker.deviation} > tolerance ${tol}`);
      }
      if (b.offBody == null) { /* no solid body to measure against (surface-only part) */ } else if (b.offBody > 2 * tolOf(c.part) + 1e-3) diffs.push(`ours points ${b.offBody.toExponential(2)} off the body`);
    } else {
      errBoth++;
      // same code: compare params that carry numbers
      const pa = JSON.stringify(a.errors.map((e) => e.params)), pb = JSON.stringify(b.errors.map((e) => e.params));
      if (!sameParams(a.errors, b.errors)) diffs.push(`error params ${pa?.slice(0, 120)} vs ${pb?.slice(0, 120)}`);
    }
  }
  if (b && !b.trap && (cacheAll || n % 4 === 1)) { const cp = cacheProblems(c.part); if (cp.length) { cacheBad++; diffs.push(`regen cache: ${cp.join(', ')}`); } }
  const status = !b ? 'ORACLE' : !diffs.length ? 'SAME' : allow[c.name] ? 'KNOWN' : 'DIFF';
  if (status === 'SAME') same++; else if (status === 'KNOWN') known++; else if (status === 'DIFF') bad++;
  rows.push({ name: c.name, ms, status, oracle: codeStr(a), ours: b ? codeStr(b) : '', diffs });
  if (!quiet || status === 'DIFF') {
    const m = a.marker ? ` deg ${a.marker.degree} count ${a.marker.count} len ${a.marker.length?.toFixed?.(3)} pieces ${a.marker.pieces} corners ${a.marker.corners} closed ${a.marker.closed} dev ${a.marker.deviation?.toExponential?.(1)}` : '';
    console.log(`${status.padEnd(6)} ${c.name.padEnd(34)} ${codeStr(a)}${m}${status !== 'SAME' && status !== 'ORACLE' ? '  | ' + diffs.join('; ') + (status === 'KNOWN' ? ` [known: ${allow[c.name]}]` : '') : ''}`);
    if (verbose && a.errors?.length) console.log('       oracle params', JSON.stringify(a.errors.map((e) => e.params)).slice(0, 300));
    if (verbose && b?.errors?.length) console.log('       ours   params', JSON.stringify(b.errors.map((e) => e.params)).slice(0, 300));
  }
}
if (rows.length) console.log(`slowest (ours): ${[...rows].sort((x, y) => y.ms - x.ms).slice(0, 4).map((r) => `${r.name} ${r.ms} ms`).join(', ')}`);
console.log(`\nproject_curve corpus: ${n} cases, ${origOnly ? 'oracle only' : `${same} same, ${known} known differences, ${bad} unexpected differences (${okBoth} successful cases compared, ${errBoth} error cases compared)`}`);
if (!origOnly) console.log(`regen cache check: ${cacheBad ? cacheBad + ' cases differ' : 'warm / cache-off / edited regens identical to cold on every checked case'}`);
process.exit(bad ? 1 : 0);
