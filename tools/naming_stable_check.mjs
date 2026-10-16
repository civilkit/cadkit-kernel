// Runs only the naming.stable metric of the gate and prints every case/edit pair that is not ok.
// usage: node tools/naming_stable_check.mjs [--wasm path] [--case name] [--all]
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { runNaming } from './naming_stability.mjs';
const here = path.dirname(fileURLToPath(import.meta.url));
const a = process.argv.slice(2);
const opt = (f) => { const i = a.indexOf(f); return i < 0 ? null : a[i + 1]; };
const ours = opt('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const rows = runNaming({ ours, only: opt('--case') }).filter((r) => !r.neg && r.edit !== '(baseline)');
const ok = rows.filter((r) => r.cls === 'ok').length;
for (const r of rows) if (a.includes('--all') || r.cls !== 'ok') console.log(`${r.case} ${r.edit} ${r.cls} ${r.detail}`);
const cases = new Set(rows.map((r) => r.case));
console.log(`naming.stable ${ok}/${rows.length} cases=${cases.size} [${[...cases].join(',')}]`);
