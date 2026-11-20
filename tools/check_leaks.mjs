// Leak check: the built wasm must not contain any pattern of the list below (case-insensitive, anywhere in the
// binary: data, custom/name sections, imports). Joins the gate (tools/gate.mjs, metric `leak`) and runs alone:
//   node tools/check_leaks.mjs [--wasm path]
// Patterns are built from pieces so no source file carries the literal.
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

export const LEAK_PATTERNS = [['purra', 'metric'].join('')];

/** The patterns found in `bytes` (a Buffer / Uint8Array), case-insensitive. */
export function findLeaks(bytes) {
  const text = Buffer.from(bytes).toString('latin1').toLowerCase();
  return LEAK_PATTERNS.filter((p) => text.includes(p));
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const i = process.argv.indexOf('--wasm');
  const wasm = i > 0 ? process.argv[i + 1] : path.join(path.dirname(fileURLToPath(import.meta.url)), '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
  const found = findLeaks(readFileSync(wasm));
  if (found.length) { console.error(`LEAK: ${wasm} contains ${found.length} forbidden pattern(s)`); process.exit(1); }
  console.log(`leak check ok: ${wasm}`);
}
