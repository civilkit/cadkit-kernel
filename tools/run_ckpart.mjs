// Regenerate part documents with a kernel wasm and print the errors and the volume of each.
//   node tools/run_ckpart.mjs <wasm> <file.ckpart|folder> [more files or folders...]
// Exit code 1 when any document has a regen error. Example: node tools/run_ckpart.mjs target/wasm32-unknown-unknown/release/ck_wasm.wasm tools/cases/marine_fuse
import { readFileSync, readdirSync, statSync } from 'node:fs';
import { join } from 'node:path';
import { Kernel } from './kernel.mjs';

const [wasm, ...paths] = process.argv.slice(2);
if (!wasm || !paths.length) { console.error('usage: node tools/run_ckpart.mjs <wasm> <file|folder>...'); process.exit(64); }
const files = paths.flatMap((p) => (statSync(p).isDirectory() ? readdirSync(p).filter((f) => f.endsWith('.ckpart') || f.endsWith('.json')).sort().map((f) => join(p, f)) : [p]));
const module = new WebAssembly.Module(readFileSync(wasm));
let bad = 0;
for (const f of files) {
  const k = new Kernel(module);
  const t0 = Date.now();
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 't', part: JSON.parse(readFileSync(f, 'utf8')) } })).json);
  const errs = r.result?.errors ?? [];
  let vol = null;
  if (!errs.length) {
    vol = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 't' } })).json).result?.volume ?? null;
  }
  if (errs.length) bad++;
  const e0 = errs[0];
  console.log(`${errs.length ? 'FAIL' : 'ok  '} ${f.split('/').pop().padEnd(22)} ${String(Date.now() - t0).padStart(6)} ms  ` + (errs.length ? `${e0?.[0] ?? e0?.feature} ${e0?.[1]?.code ?? e0?.code} ${JSON.stringify(e0?.[1]?.params?.issue ?? '')} at ${JSON.stringify(e0?.[1]?.params?.at ?? '')}` : `volume ${vol?.toFixed?.(1)}`));
}
process.exit(bad ? 1 : 0);
