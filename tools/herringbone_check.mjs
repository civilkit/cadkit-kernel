// usage: node tools/herringbone_check.mjs [--wasm path]
// A 20 tooth herringbone gear as the gear generator builds it (two opposite-twist extrudes of one spline profile, a hub, a
// bore and a centre groove), regenerated in the wasm with the host clock running. Asserts no feature errors, a validate-clean
// solid and the volume. This build is slow and its booleans sometimes only succeed on a retry at a looser tolerance; a
// time-budgeted retry ladder once skipped that retry on the slower box and failed the whole gear (booleanBudgetExceeded),
// which no native test could show because native runs have no host clock. Keep this check on the box.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';

const args = process.argv.slice(2);
const opt = (f, d) => { const i = args.indexOf(f); return i < 0 ? d : args[i + 1]; };
const wasm = opt('--wasm', new URL('../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url));
const part = JSON.parse(readFileSync(new URL('./cases/herringbone20.ckpart', import.meta.url), 'utf8'));
const VOLUME = 18183.388847515358;   // the kernel's own result when the case was recorded; tolerance below allows measurement noise only

const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)), { now: () => Date.now() });
const t0 = Date.now();
const rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
const bad = [];
let secs = (Date.now() - t0) / 1000;
if (!rep.json) bad.push(`trap ${rep.trap}`);
else {
  const r = JSON.parse(rep.json).result;
  const q = (id, type) => JSON.parse(k.request(JSON.stringify({ id, msg: { type, tag: 'main' } })).json).result;
  secs = (Date.now() - t0) / 1000;
  if (r.errors.length) bad.push(`feature errors: ${JSON.stringify(r.errors).slice(0, 600)}`);
  else {
    const mass = q(2, 'massProps'), valid = q(3, 'validate');
    if (valid.issues.length) bad.push(`validate: ${JSON.stringify(valid.issues.slice(0, 3))}`);
    if (!(Math.abs(mass.volume - VOLUME) <= 1e-4 * VOLUME)) bad.push(`volume ${mass.volume} vs ${VOLUME} (tolerance 1e-4 relative)`);
    console.log(`herringbone 20T: ${r.errors.length} errors, volume ${mass.volume}, valid ${valid.ok}, ${secs}s`);
  }
}
if (bad.length) { console.log('FAIL\n' + bad.join('\n')); process.exit(1); }
console.log('herringbone ok');
