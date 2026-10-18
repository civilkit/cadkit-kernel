// Import a fixed directory/manifest of STEP files through the kernel and
// classify each as import-failed / invalid / ok. Opt-in, research-only:
// the Fusion 360 sample is non-commercial and stays outside git.
//
//   node tools/import_sample_check.mjs DIR [--manifest FILE] [--only SUBSTR] [--json OUT]
//   node tools/import_sample_check.mjs DIR --manifest FILE --update-baseline FILE
//   node tools/import_sample_check.mjs DIR --manifest FILE --check FILE
import { readFileSync, readdirSync, statSync, writeFileSync, existsSync } from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { Kernel } from './kernel.mjs';

const argv = process.argv.slice(2);
const dir = argv[0];
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', '/path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm');
const only = opt('--only');
const manifestFile = opt('--manifest');
const mod = new WebAssembly.Module(readFileSync(wasm));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const k = new Kernel(mod);

let ids;
if (manifestFile) {
  const m = JSON.parse(readFileSync(manifestFile, 'utf8'));
  ids = Array.isArray(m) ? m : (m.ids ?? []);
} else {
  ids = readdirSync(dir).filter((f) => /\.(step|stp)$/i.test(f)).sort();
}
if (only) ids = ids.filter((f) => f.includes(only));

const rows = [];
for (const f of ids) {
  const p = path.join(dir, f);
  let cls = 'ok', detail = '';
  if (!existsSync(p)) { cls = 'missing'; detail = 'not in the local sample cache'; }
  else try {
    const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'importStep', tag: 'm', name: 'p', data: readFileSync(p, 'utf8') } }));
    const r = JSON.parse(rr.json);
    if (!r.ok || !r.result?.body) { cls = 'import-failed'; detail = r.error ?? (r.result?.errors?.[0]?.[1]?.code ?? 'no body'); }
    else {
      const v = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'validate', tag: 'm' } })).json);
      const issues = v.result?.issues ?? [];
      if (issues.length) { cls = 'invalid'; detail = [...new Set(issues.map((i) => i.code))].join(','); }
    }
  } catch (e) { cls = 'import-failed'; detail = String(e.message ?? e); }
  rows.push({ file: f, cls, detail });
  if (rows.length % 10 === 0) process.stderr.write(`  ${rows.length}/${ids.length}\n`);
}

const by = {};
for (const r of rows) by[r.cls] = (by[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ dir, n: rows.length, wasm: wasmHash, ...by }));

const out = opt('--json');
if (out) writeFileSync(out, JSON.stringify(rows, null, 1));

const baselineFile = opt('--update-baseline') ?? opt('--check');
if (baselineFile) {
  const cases = Object.fromEntries(rows.map((r) => [r.file, { cls: r.cls, detail: r.detail }]));
  if (opt('--update-baseline')) {
    writeFileSync(baselineFile, JSON.stringify({ version: 1, wasm: wasmHash, date: new Date().toISOString().slice(0, 10), cases }, null, 1) + '\n');
    console.log(`import_sample_check: wrote baseline ${baselineFile} (${rows.length} ids)`);
  }
  if (opt('--check')) {
    if (!existsSync(baselineFile)) { console.error(`no baseline at ${baselineFile}`); process.exit(2); }
    const base = JSON.parse(readFileSync(baselineFile, 'utf8')).cases ?? {};
    const regressions = [], newPasses = [], added = [];
    for (const r of rows) {
      const b = base[r.file];
      if (!b) { added.push(r.file); continue; }
      if (b.cls === 'ok' && r.cls !== 'ok') regressions.push(r);
      else if (b.cls !== 'ok' && r.cls === 'ok') newPasses.push(r);
    }
    const gone = Object.keys(base).filter((id) => !rows.some((r) => r.file === id));
    console.log(`import_sample_check check: ${regressions.length} regressions, ${newPasses.length} new passes, ${added.length} added, ${gone.length} no longer sampled`);
    for (const r of regressions) console.log(`REGRESSION ${r.file}: ${r.cls} ${r.detail}`);
    for (const r of newPasses) console.log(`new pass   ${r.file}: ${r.cls}`);
    if (regressions.length) process.exit(1);
  }
}
