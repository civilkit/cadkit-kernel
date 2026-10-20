// Runs every SketchUp (.skp) file under one or more directories through the wasm `importSkp` request and prints, per file:
// container, version, faces, lumps, open edges, bbox, scene materials, import ms. Exit 1 when a file traps or (with --strict)
// does not import.
//
// usage: node tools/skp_check.mjs [--wasm PATH] [--filter SUBSTR] [--json] [--hidden] [--strict] DIR_OR_FILE...
// default: crates/ck-skp/tests/fixtures
import { readFileSync, readdirSync, statSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (n, d) => { const i = args.indexOf(n); return i >= 0 ? args[i + 1] : d; };
const flags = new Set(['--json', '--hidden', '--strict']);
const wasmPath = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const filter = opt('--filter', '');
const inputs = [];
for (let i = 0; i < args.length; i++) {
  if (['--wasm', '--filter'].includes(args[i])) { i++; continue; }
  if (flags.has(args[i])) continue;
  inputs.push(args[i]);
}
if (!inputs.length) inputs.push(path.join(here, '../crates/ck-skp/tests/fixtures'));

const { Kernel } = await import(path.join(here, 'kernel.mjs'));
const mod = await WebAssembly.compile(readFileSync(wasmPath));

function walk(d, out = []) {
  if (!statSync(d).isDirectory()) { out.push(d); return out; }
  for (const n of readdirSync(d).sort()) {
    const p = path.join(d, n);
    if (statSync(p).isDirectory()) walk(p, out);
    else if (/\.skp$/i.test(n)) out.push(p);
  }
  return out;
}

const rows = [];
let trapped = 0, failed = 0;
for (const f of inputs.flatMap((i) => walk(i)).filter((f) => f.includes(filter))) {
  const row = { name: path.basename(f) };
  const k = new Kernel(mod);
  const data = readFileSync(f).toString('base64');
  const msg = { type: 'importSkp', tag: 'm', name: row.name, data };
  if (args.includes('--hidden')) msg.opts = { hidden: true };
  const t0 = performance.now();
  const r = k.request(JSON.stringify({ id: 1, msg }));
  row.ms = Math.round(performance.now() - t0);
  if (!r.json) { row.error = 'TRAP ' + r.trap; trapped++; rows.push(row); continue; }
  const res = JSON.parse(r.json).result;
  if (!res?.body) {
    const e = res?.errors?.[0];
    row.error = e ? `${e[1]?.code}: ${e[1]?.params?.message ?? ''}`.slice(0, 200) : 'no body';
    failed++;
  } else {
    const im = res.import;
    Object.assign(row, { container: im.container, version: im.version, faces: im.faces, lumps: im.lumps, open: im.openEdges, nonManifold: im.nonManifoldEdges, materials: im.materials.length, bboxMm: im.bboxMm.map((x) => +x.toFixed(1)) });
  }
  rows.push(row);
}
if (args.includes('--json')) console.log(JSON.stringify(rows, null, 1));
else for (const r of rows) {
  console.log(r.error ? `${r.name.padEnd(40)} FAIL ${r.error} (${r.ms} ms)` : `${r.name.padEnd(40)} ${r.container.padEnd(7)} ${String(r.version).padEnd(14)} faces ${String(r.faces).padStart(6)} lumps ${String(r.lumps).padStart(4)} open ${String(r.open).padStart(5)} mats ${String(r.materials).padStart(3)} ${r.ms} ms  bbox ${r.bboxMm.join(' ')}`);
}
console.log(`${rows.length} files, ${rows.length - failed - trapped} imported, ${failed} refused, ${trapped} trapped`);
process.exit(trapped || (args.includes('--strict') && failed) ? 1 : 0);
