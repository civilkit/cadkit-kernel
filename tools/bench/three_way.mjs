// Three-way build-time comparison: our kernel vs the original kernel.wasm vs OCCT (tools/bench/occt_times.py).
//
//   node tools/bench/three_way.mjs run    [--ours w.wasm] [--orig k.wasm] [--runs 5] [--only id,id] [--timeout 240] --out kernels.json
//   python3.12 tools/bench/occt_times.py <codedir> occt.json --tess id,id,...
//   node tools/bench/three_way.mjs report kernels.json occt.json > table.md
//
// Protocol per part and kernel: regen of the translated part (tools/bench/benchcad/*.ckpart.json) with
// opts.noCache and debug:true (trace spans, host clock = wall clock). `build` = the "regenerate" span (the
// feature program); `tess` = the "snapshot" span (display tessellation, which a regen reply always includes);
// `wall` = whole request. The original kernel has a regen cache that ignores noCache, so EVERY run of EVERY
// kernel uses a fresh instance of the (already compiled) module: the first run of a part is the "cold" number
// (warmup, discarded from the statistics), then N timed runs; median and min are reported. Ours also gets a
// second protocol, `one`: all runs on a single instance (noCache), which is how the editor calls it.
// Each (part, kernel) runs in a child process with a timeout so a hang cannot stall the table.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from '../kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const SET = process.env.CK_SET ?? 'benchcad';
const dir = path.join(here, SET);
const manifest = JSON.parse(readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
const args = process.argv.slice(2);
const opt = (f, d = null) => { const i = args.indexOf(f); if (i < 0) return d; const v = args[i + 1]; args.splice(i, 2); return v; };
const median = (a) => { const s = [...a].sort((x, y) => x - y); const n = s.length; return n % 2 ? s[(n - 1) / 2] : (s[n / 2 - 1] + s[n / 2]) / 2; };
const clock = () => performance.now();

// ---- child: one part, one wasm ------------------------------------------------------------------------
function child(wasm, id, runs, mode) {
  const B = new WebAssembly.Module(readFileSync(wasm));
  const part = JSON.parse(readFileSync(path.join(dir, `${id}.ckpart.json`), 'utf8'));
  const mk = () => new Kernel(B, { now: clock, unixMs: () => Date.now() });
  const once = (k) => {
    const t0 = clock();
    const rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, debug: true, opts: { noCache: true } } }));
    const wall = clock() - t0;
    if (rep.trap) return { trap: rep.trap.slice(0, 120), wall };
    const o = JSON.parse(rep.json);
    let build = null, tess = null, tris = null;
    for (const s of o.trace ?? []) {
      if (s.depth === 0 && s.label.startsWith('regenerate')) build = s.ms;
      if (s.depth === 0 && s.label.startsWith('snapshot')) tess = s.ms;
      const m = /^tessellate \d+ faces → (\d+) triangles/.exec(s.label);
      if (m) tris = +m[1];
    }
    return { wall, build, tess, tris, errors: (o.result?.errors ?? []).map(([f, e]) => `${f}:${e.code}`).join(','), body: !!o.result?.body };
  };
  const rows = [];
  let k = mk();
  let last = null;
  for (let r = 0; r < runs + 1; r++) {
    if (mode === 'fresh') k = mk();
    const x = once(k);
    rows.push(x);
    last = k;
    if (x.trap) { if (mode !== 'fresh') k = mk(); break; }
    // a part that already takes this long gets only a cold run and one more
    if (r === 0 && x.wall > 30000) { runs = 1; }
  }
  const out = { cold: rows[0], runs: rows.slice(1) };
  const good = rows.slice(1).filter((x) => !x.trap && x.build != null);
  if (good.length) {
    out.build_ms = median(good.map((x) => x.build)); out.build_min = Math.min(...good.map((x) => x.build));
    out.tess_ms = median(good.map((x) => x.tess)); out.tess_min = Math.min(...good.map((x) => x.tess));
    out.wall_ms = median(good.map((x) => x.wall)); out.wall_min = Math.min(...good.map((x) => x.wall));
    out.n = good.length;
  }
  const c = rows[0];
  out.tris = rows.find((x) => x.tris)?.tris ?? null;
  out.errors = c.errors ?? ''; out.trap = c.trap ?? null; out.has_body = c.body ?? false;
  // untimed: volume + validity from the last instance (the one whose state is the regen result)
  if (!out.trap && last) {
    const q = (id2, msg) => { const r = last.request(JSON.stringify({ id: id2, msg })); return r.trap ? null : JSON.parse(r.json).result; };
    const mp = q(2, { type: 'massProps', tag: 'main' });
    out.volume = mp?.volume ?? null; out.area = mp?.area ?? null;
    const v = q(3, { type: 'validate', tag: 'main' });
    out.issues = v?.issues?.length ?? null;
  }
  return out;
}

if (args[0] === 'child') {
  const [, wasm, id, runs, mode] = args;
  console.log(JSON.stringify(child(wasm, id, +runs, mode)));
  process.exit(0);
}

// ---- remote: whole comparison on the timing box under its lock ------------------------------------------
//   BENCH_HOST=<ssh host> node tools/bench/three_way.mjs --remote --outdir <dir> [--runs 5] [--lock-wait 60]
// Needs tools/bench/setup_box_env.sh to have been run. Takes the box's `mkdir ~/timing.lock` lock (waits up to
// --lock-wait minutes, polling), runs ours + original + OCCT in that one session, always releases the lock
// (rmdir), copies kernels.json / occt.json into --outdir and prints the report.
if (args.includes('--remote')) {
  args.splice(args.indexOf('--remote'), 1);
  const host = process.env.BENCH_HOST;
  if (!host) { console.error('set BENCH_HOST'); process.exit(2); }
  const outdir = opt('--outdir', '.');
  const runs = opt('--runs', '5');
  const waitMin = +opt('--lock-wait', 60);
  const only = opt('--only');
  const ssh = (cmd, inherit = false) => spawnSync('ssh', ['-o', 'BatchMode=yes', host, cmd], inherit ? { stdio: 'inherit' } : { encoding: 'utf8' });
  let held = false;
  const release = () => { if (held) { ssh('rmdir ~/timing.lock'); held = false; console.error('lock released'); } };
  process.on('SIGINT', () => { release(); process.exit(130); });
  process.on('SIGTERM', () => { release(); process.exit(143); });
  const deadline = Date.now() + waitMin * 60000;
  while (!held) {
    held = ssh('mkdir ~/timing.lock').status === 0;
    if (!held) {
      if (Date.now() > deadline) { console.error('lock wait timed out'); process.exit(3); }
      console.error('timing lock busy, waiting 30 s'); spawnSync('sleep', ['30']);
    }
  }
  console.error('lock taken');
  try {
    const onlyArg = only ? `--only ${only}` : '';
    ssh(`cd ~/bench-env && echo load before: $(cut -d' ' -f1-3 /proc/loadavg) && node tools/bench/three_way.mjs run --ours wasm/ours.wasm --orig wasm/orig.wasm --runs ${runs} ${onlyArg} --timeout 300 --out kernels.json`, true);
    const k = ssh('cat ~/bench-env/kernels.json');
    writeFileSync(path.join(outdir, 'kernels.json'), k.stdout);
    const K = JSON.parse(k.stdout);
    const top = Object.entries(K.parts).filter(([, p]) => p.ours?.build_ms != null).sort((a, b) => b[1].ours.build_ms - a[1].ours.build_ms).slice(0, 5).map(([id]) => id);
    ssh(`cd ~/bench-env && venv/bin/python tools/bench/occt_times.py cq occt.json --runs ${runs} --tess ${top.join(',')} ${only ? `--only ${only}` : ''} && echo load after: $(cut -d' ' -f1-3 /proc/loadavg)`, true);
    const o = ssh('cat ~/bench-env/occt.json');
    writeFileSync(path.join(outdir, 'occt.json'), o.stdout);
    // second OCCT pass with CadQuery's clean() (UnifySameDomain after each boolean) disabled: nearer raw OCCT calls
    ssh(`cd ~/bench-env && venv/bin/python tools/bench/occt_times.py cq occt_noclean.json --runs ${runs} --noclean ${only ? `--only ${only}` : ''}`, true);
    writeFileSync(path.join(outdir, 'occt_noclean.json'), ssh('cat ~/bench-env/occt_noclean.json').stdout);
  } finally { release(); }
  const r = spawnSync(process.execPath, [fileURLToPath(import.meta.url), 'report', path.join(outdir, 'kernels.json'), path.join(outdir, 'occt.json'), path.join(outdir, 'occt_noclean.json')], { stdio: 'inherit' });
  process.exit(r.status ?? 0);
}

// ---- driver --------------------------------------------------------------------------------------------
if (args[0] === 'run') {
  args.shift();
  const ours = opt('--ours', path.join(here, '../../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
  const orig = opt('--orig', path.join(here, '../../../kernel.wasm'));
  const runs = +opt('--runs', 5);
  const only = opt('--only')?.split(',');
  const timeout = +opt('--timeout', 240) * 1000;
  const outFile = opt('--out', 'three_way_kernels.json');
  const res = { set: SET, runs, load_start: os.loadavg(), cpus: os.cpus().length, node: process.version, parts: {} };
  const spawn = (wasm, id, mode) => {
    const r = spawnSync(process.execPath, [fileURLToPath(import.meta.url), 'child', wasm, id, String(runs), mode], { encoding: 'utf8', timeout, maxBuffer: 1 << 26 });
    if (r.error || r.status !== 0) return { timeout: true, note: String(r.error?.code ?? r.status) };
    return JSON.parse(r.stdout.trim().split('\n').pop());
  };
  for (const m of manifest.parts) {
    if (only && !only.includes(m.id)) continue;
    const skip = manifest.known?.[m.id]?.skip;
    const row = { family: m.family, tier: m.tier, gt_volume: m.ground_truth.volume };
    if (skip) { row.skipped = skip; res.parts[m.id] = row; console.error(`${m.id} skipped`); continue; }
    row.ours = spawn(ours, m.id, 'fresh');
    row.ours_one = spawn(ours, m.id, 'one');
    row.orig = spawn(orig, m.id, 'fresh');
    res.parts[m.id] = row;
    const f = (x) => (x?.build_ms != null ? x.build_ms.toFixed(0) : x?.timeout ? 'TIMEOUT' : '-');
    console.error(`${m.id.padEnd(28)} ours ${f(row.ours)} one ${f(row.ours_one)} orig ${f(row.orig)}  load ${os.loadavg()[0].toFixed(1)}`);
    writeFileSync(outFile, JSON.stringify(res, null, 1));
  }
  res.load_end = os.loadavg();
  writeFileSync(outFile, JSON.stringify(res, null, 1));
  process.exit(0);
}

// ---- report --------------------------------------------------------------------------------------------
if (args[0] === 'report') {
  const K = JSON.parse(readFileSync(args[1], 'utf8'));
  const O = JSON.parse(readFileSync(args[2], 'utf8'));
  const N = args[3] && existsSync(args[3]) ? JSON.parse(readFileSync(args[3], 'utf8')) : {};
  const rel = (a, b) => Math.abs(a - b) / Math.max(Math.abs(b), 1e-12);
  const gm = (a) => Math.exp(a.reduce((s, x) => s + Math.log(x), 0) / a.length);
  const rows = [];
  for (const [id, p] of Object.entries(K.parts)) {
    const o = O[id];
    const r = { id, family: p.family, ours: p.ours?.build_ms, oursMin: p.ours?.build_min, one: p.ours_one?.build_ms, orig: p.orig?.build_ms, origMin: p.orig?.build_min,
      occt: o?.ms, occtNc: N[id]?.ms, occtMin: o?.min_ms, oursCold: p.ours?.cold?.build, origCold: p.orig?.cold?.build, occtCold: o?.cold_ms };
    r.vOurs = p.ours?.volume; r.vOrig = p.orig?.volume; r.vOcct = o?.volume; r.vGt = p.gt_volume;
    r.oursErr = p.ours?.errors || p.ours?.trap || (p.ours?.timeout ? 'timeout' : '') || (p.skipped ? 'skipped' : '');
    r.origErr = p.orig?.errors || p.orig?.trap || (p.orig?.timeout ? 'timeout' : '');
    r.occtErr = o?.error ?? (o ? '' : 'no program');
    // the volume reference for "agrees": OCCT (same program) when present, else the dataset ground truth
    const ref = r.vOcct ?? r.vGt;
    r.volOurs = r.vOurs != null && rel(r.vOurs, ref) <= 1e-3;
    r.volOrig = r.vOrig != null && rel(r.vOrig, ref) <= 1e-3;
    r.volDev = r.vOurs != null ? rel(r.vOurs, ref) : null;
    r.valid = o?.valid; r.issues = p.ours?.issues; r.tessOurs = p.ours?.tess_ms; r.tessOcct = o?.tess_ms; r.trisOurs = p.ours?.tris; r.trisOcct = o?.tess_tris; r.tessOrig = p.orig?.tess_ms; r.trisOrig = p.orig?.tris;
    r.faces = o?.faces;
    rows.push(r);
  }
  const f0 = (x) => (x == null ? '-' : x >= 100 ? x.toFixed(0) : x >= 10 ? x.toFixed(1) : x.toFixed(2));
  const fr = (x) => (x == null || !isFinite(x) ? '-' : x >= 10 ? x.toFixed(0) + 'x' : x.toFixed(2) + 'x');
  const ok = rows.filter((r) => r.ours != null && !r.oursErr);
  const lines = [];
  lines.push('| part | ours ms | ours 1-inst ms | orig ms | OCCT ms | OCCT no-clean ms | ours/OCCT | ours/orig | ours min | orig min | OCCT min | volume ours / orig vs ref | OCCT valid |');
  lines.push('|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|');
  for (const r of rows.sort((a, b) => a.id.localeCompare(b.id))) {
    const bad = r.oursErr ? ` (${r.oursErr.slice(0, 22)})` : '';
    const vol = r.vOurs == null ? 'n/a' : `${r.volOurs ? 'yes' : '**NO ' + (r.volDev * 100).toFixed(2) + '%**'} / ${r.vOrig == null ? 'n/a' : r.volOrig ? 'yes' : '**NO**'}`;
    lines.push(`| ${r.id}${bad} | ${f0(r.ours)} | ${f0(r.one)} | ${f0(r.orig)} | ${f0(r.occt)} | ${f0(r.occtNc)} | ${fr(r.ours / r.occt)} | ${fr(r.ours / r.orig)} | ${f0(r.oursMin)} | ${f0(r.origMin)} | ${f0(r.occtMin)} | ${vol} | ${r.valid == null ? '-' : r.valid ? 'yes' : 'NO'} |`);
  }
  console.log(lines.join('\n'));
  // summary statistics over parts that built in all three engines with matching volumes
  const both = ok.filter((r) => r.occt != null && r.volOurs);
  const bo = ok.filter((r) => r.orig != null && r.volOurs && r.volOrig);
  const rO = both.map((r) => r.ours / r.occt), rG = bo.map((r) => r.ours / r.orig);
  const stat = (name, rr, arr) => `${name}: n=${rr.length}, geometric mean ${gm(rr).toFixed(2)}x, median ${median(rr).toFixed(2)}x, faster in ${rr.filter((x) => x < 1).length}, slower in ${rr.filter((x) => x >= 1).length}; ${arr}`;
  console.log('\n<!--STATS-->');
  console.log(stat('ours/OCCT (median ms, volumes agree)', rO, `sum ours ${both.reduce((s, r) => s + r.ours, 0).toFixed(0)} ms vs OCCT ${both.reduce((s, r) => s + r.occt, 0).toFixed(0)} ms`));
  console.log(stat('ours/orig (median ms, volumes agree)', rG, `sum ours ${bo.reduce((s, r) => s + r.ours, 0).toFixed(0)} ms vs orig ${bo.reduce((s, r) => s + r.orig, 0).toFixed(0)} ms`));
  const bm = ok.filter((r) => r.occtMin != null && r.volOurs);
  console.log(stat('ours/OCCT (min of runs)', bm.map((r) => r.oursMin / r.occtMin), ''));
  const bg = ok.filter((r) => r.origMin != null && r.volOurs && r.volOrig);
  console.log(stat('ours/orig (min of runs)', bg.map((r) => r.oursMin / r.origMin), ''));
  const bnc = ok.filter((r) => r.occtNc != null && r.volOurs);
  console.log(stat('ours/OCCT no-clean', bnc.map((r) => r.ours / r.occtNc), ''));
  const bo1 = ok.filter((r) => r.one != null && r.occt != null && r.volOurs);
  console.log(stat('ours single-instance / OCCT', bo1.map((r) => r.one / r.occt), ''));
  console.log('\nworst vs OCCT:'); for (const r of [...both].sort((a, b) => b.ours / b.occt - a.ours / a.occt).slice(0, 10)) console.log(`  ${r.id} ${fr(r.ours / r.occt)} (${f0(r.ours)} vs ${f0(r.occt)} ms)`);
  console.log('worst vs orig:'); for (const r of [...bo].sort((a, b) => b.ours / b.orig - a.ours / a.orig).slice(0, 10)) console.log(`  ${r.id} ${fr(r.ours / r.orig)} (${f0(r.ours)} vs ${f0(r.orig)} ms)`);
  console.log('volume mismatches (> 1e-3 vs OCCT):', rows.filter((r) => r.vOurs != null && !r.volOurs).map((r) => `${r.id} ours ${r.vOurs.toPrecision(6)} vs ${(r.vOcct ?? r.vGt)?.toPrecision(6)}`).join('; ') || 'none');
  console.log('orig mismatches:', rows.filter((r) => r.vOrig != null && !r.volOrig).map((r) => r.id).join(', ') || 'none');
  console.log('not built by ours:', rows.filter((r) => r.oursErr || r.ours == null).map((r) => `${r.id}:${r.oursErr || 'none'}`).join(', '));
  console.log('\nbiggest 5 by OCCT/ours time, tessellation (ms, tris):');
  for (const r of [...rows].filter((r) => r.tessOcct != null).sort((a, b) => b.ours - a.ours)) console.log(`  ${r.id.padEnd(26)} build ours ${f0(r.ours)} / OCCT ${f0(r.occt)} | tess ours ${f0(r.tessOurs)} (${r.trisOurs}) orig ${f0(r.tessOrig)} (${r.trisOrig}) OCCT ${f0(r.tessOcct)} (${r.trisOcct})`);
  console.log('\nload', JSON.stringify(K.load_start), '->', JSON.stringify(K.load_end), 'cpus', K.cpus);
  process.exit(0);
}
console.error('usage: three_way.mjs run|report ...');
