// Wasm parity of the fillet regression table: runs every case of crates/ck-model/tests/fixtures/blend_cases.json
// through the kernel wasm the way the app does (a `regen` request, whose reply carries the display tessellation, then
// `validate` and `massProps`) and compares each outcome with the native table's (`BLEND_JSON` of
// crates/ck-model/tests/blend_table.rs). See docs/wasm-parity.md; tools/wasm_parity.sh runs both on the CI box.
//
//   node tools/wasm_parity.mjs --wasm ck_wasm.wasm [--native table.json] [--json out.json] [--case a,b] [--workers N]
//                              [--timeout-ms N]
//
// Per case and run (`fresh`: one regen of the whole document in a new instance; `cached`: the base (+ first fillet)
// regenerated first in the same instance, then the whole document, which resumes from the regen cache like an edit in
// the app): the regen outcome (`ok` or the first error code + detail), a trap (with the panic text), body validity
// (`validate` with `mesh:false`, the native table's check), volume, and the faces the display tessellator failed on
// (`validate` with the mesh witness: `TESSELLATION_OPEN` issues; `meshNotes` are counted, listed in the JSON).
// Prints one line per difference / trap / failed face and the counts; exit 1 when there is any trap or difference.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import { fileURLToPath } from 'node:url';
import { Worker, isMainThread, parentPort, workerData } from 'node:worker_threads';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const CASES = path.join(here, '../crates/ck-model/tests/fixtures/blend_cases.json');

function runCase(mod, bases, c) {
  const doc = (features) => ({ format: 'ck-part', version: 1, features });
  const feats = [...bases[c.base], ...(c.pre ? [c.pre] : []), c.fillet];
  const prefix = feats.slice(0, -1);
  const one = (mode) => {
    // the app's worker passes a real clock (budgets are live), not the 0 of kernel.mjs's default
    const k = new Kernel(mod, { now: () => performance.now(), unixMs: () => Date.now() });
    const req = (msg) => {
      const r = k.request(JSON.stringify({ id: 1, msg }));
      return r.json ? JSON.parse(r.json) : { trap: r.trap };
    };
    const t0 = performance.now();
    if (mode === 'cached') {
      const p = req({ type: 'regen', tag: 'main', part: doc(prefix) });
      if (p.trap) return { trap: 'prefix regen: ' + p.trap, secs: (performance.now() - t0) / 1000 };
    }
    const r = req({ type: 'regen', tag: 'main', part: doc(feats) });
    const o = { secs: 0 };
    if (r.trap) o.trap = 'regen: ' + r.trap;
    else {
      const err = r.result?.errors?.[0];
      if (err) {
        const d = err[1]?.params?.detail;
        o.code = err[1]?.code ?? 'error';
        if (d) o.detail = String(d);
      } else if (!r.result?.body) o.code = 'noBody';
      if (r.result?.body) {
        const v = req({ type: 'validate', tag: 'main', mesh: false, opts: { validateTimeMs: 0 } });
        if (v.trap) o.trap = 'validate: ' + v.trap;
        else {
          o.valid = !!v.result?.ok;
          if (!o.valid) o.issues = (v.result?.issues ?? []).slice(0, 3).map((i) => `${i.code} ${i.entity}`);
        }
        const m = req({ type: 'massProps', tag: 'main' });
        if (m.trap) o.trap = (o.trap ? o.trap + '; ' : '') + 'massProps: ' + m.trap;
        else o.volume = m.result?.volume ?? null;
        const w = req({ type: 'validate', tag: 'main', opts: { validateTimeMs: 0 } });
        if (w.trap) o.trap = (o.trap ? o.trap + '; ' : '') + 'validate mesh: ' + w.trap;
        else {
          o.failed = (w.result?.issues ?? []).filter((i) => i.code === 'TESSELLATION_OPEN').map((i) => `${i.entity}: ${i.detail}`);
          o.meshNotes = w.result?.meshNotes ?? [];
        }
      }
    }
    o.secs = (performance.now() - t0) / 1000;
    return o;
  };
  return { id: c.id, fresh: one('fresh'), cached: one('cached') };
}

if (!isMainThread) {
  const mod = new WebAssembly.Module(readFileSync(workerData.wasm));
  const { bases, cases } = JSON.parse(readFileSync(CASES, 'utf8'));
  parentPort.on('message', (i) => parentPort.postMessage({ i, r: runCase(mod, bases, cases[i]) }));
} else {
  const argv = process.argv.slice(2);
  const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
  const wasm = opt('--wasm');
  if (!wasm) { console.error('usage: node tools/wasm_parity.mjs --wasm FILE [--native table.json] [--json out] [--case a,b]'); process.exit(64); }
  const { cases } = JSON.parse(readFileSync(CASES, 'utf8'));
  const filt = opt('--case', process.env.BLEND_CASE || '');
  const sel = (id) => !filt || filt.split(',').some((t) => (t.startsWith('=') ? id === t.slice(1) : t.startsWith('^') ? id.startsWith(t.slice(1)) : id.includes(t)));
  const todo = cases.map((c, i) => [c.id, i]).filter(([id]) => sel(id)).map(([, i]) => i);
  const native = new Map();
  if (opt('--native')) for (const r of JSON.parse(readFileSync(opt('--native'), 'utf8'))) native.set(r.id, r);
  const nWorkers = Math.max(1, Math.min(+opt('--workers', Math.min(12, os.cpus().length)), todo.length));
  const timeout = +opt('--timeout-ms', 300000);
  const results = new Map();
  const t0 = performance.now();
  await new Promise((done) => {
    let next = 0, live = 0;
    const spawn = () => {
      if (next >= todo.length) { if (live === 0) done(); return; }
      const w = new Worker(fileURLToPath(import.meta.url), { workerData: { wasm } });
      live++;
      let cur = -1, timer = null;
      const feed = () => {
        if (next >= todo.length) { w.terminate(); return; }
        cur = todo[next++];
        timer = setTimeout(() => { results.set(cur, { id: cases[cur].id, fresh: { trap: `timeout after ${timeout} ms` }, cached: { trap: 'not run (timeout)' } }); w.terminate(); }, timeout);
        w.postMessage(cur);
      };
      w.on('message', ({ i, r }) => { clearTimeout(timer); results.set(i, r); feed(); });
      w.on('error', (e) => { clearTimeout(timer); if (cur >= 0 && !results.has(cur)) results.set(cur, { id: cases[cur].id, fresh: { trap: 'worker: ' + e.message }, cached: {} }); });
      w.on('exit', () => { clearTimeout(timer); live--; spawn(); });
      feed();
    };
    for (let k = 0; k < nWorkers; k++) spawn();
  });

  const built = (o) => !o.trap && !o.code;
  const outc = (o) => (o.trap ? 'trap' : o.code ? o.code : o.valid ? 'ok' : 'invalid');
  const nOut = (n) => (n.volume == null ? n.outcome : n.valid ? 'ok' : 'invalid');
  const lines = [], rows = [];
  let nDiff = 0, nTrap = 0, nFailed = 0, nCache = 0, nNotes = 0, nBudget = 0;
  for (const i of todo) {
    const r = results.get(i), id = cases[i].id, n = native.get(id), f = r.fresh, c = r.cached;
    const row = { id, native: n ? { outcome: n.outcome, detail: n.detail, valid: n.valid, volume: n.volume, secs: n.secs } : null, fresh: f, cached: c, diffs: [] };
    for (const [tag, o] of [['fresh', f], ['cached', c]]) {
      if (o.trap) { nTrap++; lines.push(`TRAP ${id} [${tag}] ${o.trap.replace(/\s+/g, ' ').slice(0, 200)}`); }
      for (const x of o.failed ?? []) { nFailed++; lines.push(`FAILED-FACE ${id} [${tag}] ${x.slice(0, 160)}`); }
      nNotes += (o.meshNotes ?? []).length;
    }
    if (n) {
      const no = nOut(n), wo = outc(f);
      // the app's clock makes the per-feature budget live (120 s by default); the native table has no clock
      if (f.code === 'featureBudgetExceeded') { nBudget++; row.diffs.push(`native ${no} in ${n.secs?.toFixed(0)} s vs wasm featureBudgetExceeded (the feature ran past its time budget; regen ${f.secs.toFixed(0)} s)`); }
      else if (no !== wo) row.diffs.push(`native ${no}${n.detail ? ' (' + n.detail.slice(0, 70) + ')' : ''} vs wasm ${wo}${f.detail ? ' (' + f.detail.slice(0, 70) + ')' : ''}`);
      else if (built(f) && n.volume != null && Math.abs(f.volume - n.volume) > 1e-6 * Math.abs(n.volume)) row.diffs.push(`volume native ${n.volume.toFixed(6)} vs wasm ${f.volume.toFixed(6)} (rel ${((f.volume - n.volume) / n.volume).toExponential(2)})`);
      else if (!built(f) && n.volume == null && (n.detail ?? '') !== (f.detail ?? '')) row.diffs.push(`detail native "${(n.detail ?? '').slice(0, 60)}" vs wasm "${(f.detail ?? '').slice(0, 60)}"`);
    }
    const fo = outc(f), co = outc(c);
    if (fo !== co || (built(f) && built(c) && Math.abs(f.volume - c.volume) > 1e-6 * Math.abs(f.volume))) {
      nCache++;
      row.diffs.push(`fresh ${fo}${built(f) ? ' ' + f.volume.toFixed(6) : ''} vs cached ${co}${built(c) ? ' ' + c.volume.toFixed(6) : ''}`);
    }
    if (row.diffs.length) { nDiff++; lines.push(`DIFF ${id} ${row.diffs.join('; ')}`); }
    rows.push(row);
  }
  console.log('wasm parity:');
  for (const l of lines) console.log(l);
  const ok = rows.filter((r) => outc(r.fresh) === 'ok').length;
  console.log(`wasm parity: ${todo.length} cases, wasm ok ${ok}, ${nDiff} differing (${nCache} fresh vs cached, ${nBudget} over the time budget), ${nTrap} traps, ${nFailed} failed faces, ${nNotes} mesh notes${native.size ? '' : ' (no native table: wasm only)'}  t=${((performance.now() - t0) / 1000).toFixed(1)}`);
  console.log('wasm parity end');
  if (opt('--json')) { mkdirSync(path.dirname(opt('--json')), { recursive: true }); writeFileSync(opt('--json'), JSON.stringify(rows, null, 1) + '\n'); }
  process.exit(nDiff || nTrap ? 1 : 0);
}
