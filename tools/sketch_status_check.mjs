// Sketch solver status through the wasm regen reply (opts.sketchStatus).
// usage: node tools/sketch_status_check.mjs [--wasm path]    exit 1 on any failure
// Checks the closed-form cases of tools/probe_sketch_status.mjs, and that the
// option is additive (reply without it unchanged; with it only `status` is added).
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
import { cases, mkPart } from './probe_sketch_status.mjs';

const args = process.argv.slice(2);
const wi = args.indexOf('--wasm');
const wasm = wi >= 0 ? args[wi + 1] : new URL('../target/wasm32-unknown-unknown/fast/ck_wasm.wasm', import.meta.url);
const mod = new WebAssembly.Module(readFileSync(wasm));
const regen = (part, opts) => {
  const r = new Kernel(mod).request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, ...(opts ? { opts } : {}) } }));
  if (r.trap) throw new Error(r.trap);
  return JSON.parse(r.json).result;
};
let bad = 0;
const check = (name, ok, extra = '') => { if (!ok) { bad++; console.log('FAIL', name, extra); } else console.log('ok  ', name); };
for (const [name, c] of Object.entries(cases)) {
  const plain = regen(mkPart(c));
  const withS = regen(mkPart(c), { sketchStatus: true });
  const st = withS.sketches[0][1].status;
  check(`${name}: no status without the option`, plain.sketches[0][1].status === undefined);
  const stripped = JSON.parse(JSON.stringify(withS)); delete stripped.sketches[0][1].status;
  check(`${name}: additive`, JSON.stringify(stripped) === JSON.stringify(plain));
  const w = c.want;
  const exp = (k) => w[k] === undefined || JSON.stringify(st[k]) === JSON.stringify(w[k]);
  check(`${name}: dof/conflicts/redundant`, exp('dof') && exp('conflicts') && exp('redundant'), JSON.stringify(st));
  check(`${name}: fullyDefined consistent`, st.fullyDefined === (st.dof === 0 && !st.conflicts.length && !st.redundant.length && st.solved), JSON.stringify(st));
}
process.exit(bad ? 1 : 0);
