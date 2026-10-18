// Import a directory of STEP files through the kernel and classify each as
// import-failed / invalid / ok. Used for the public B-rep corpus sample.
//
//   node tools/import_sample_check.mjs DIR [--wasm PATH] [--json OUT]
import { readFileSync, readdirSync, statSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { Kernel } from './kernel.mjs';

const argv = process.argv.slice(2);
const dir = argv[0];
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', '/path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));
const k = new Kernel(mod);
const files = readdirSync(dir).filter((f) => /\.(step|stp)$/i.test(f)).sort();
const rows = [];
for (const f of files) {
  const p = path.join(dir, f);
  if (!statSync(p).isFile()) continue;
  let cls = 'ok', detail = '';
  try {
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
  if (rows.length % 10 === 0) process.stderr.write(`  ${rows.length}/${files.length}\n`);
}
const by = {};
for (const r of rows) by[r.cls] = (by[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ dir, n: rows.length, ...by }));
const out = opt('--json');
if (out) writeFileSync(out, JSON.stringify(rows, null, 1));
