// places.mjs <cases> [name..]: components side by side, max |Δ| per component
import * as L from './lib.mjs';
import { readFileSync } from 'node:fs';
const src = process.argv[2];
const mod = src.endsWith('.ckpart') ? (() => { const p = JSON.parse(readFileSync(src, 'utf8')); const c = {}; if (process.env.PREFIX) for (let k = 1; k <= p.features.length; k++) c['@' + k] = [p, { opts: { upTo: k } }]; else c.all = [p]; return { cases: c, raw: true }; })() : await import(src);
const names = process.argv.slice(3), B = L.B();
for (const [name, [f, o]] of Object.entries(mod.cases)) {
  if (names.length && !names.includes(name)) continue;
  const msg = L.regen(mod.raw ? f : L.part(f), o ?? {});
  const a = L.run(L.A, msg).o.result, b = L.run(B, msg).o.result;
  const out = [];
  if (JSON.stringify(a.errors) !== JSON.stringify(b.errors)) out.push(`errors orig=${JSON.stringify(a.errors)} ours=${JSON.stringify(b.errors)}`);
  for (const ca of a.components) {
    const cb = b.components.find((c) => c.id === ca.id);
    if (!cb) { out.push(`${ca.id} missing`); continue; }
    let d = 0; for (const k of ['o', 'x', 'y', 'z']) for (let i = 0; i < 3; i++) d = Math.max(d, Math.abs(ca.place[k][i] - cb.place[k][i]));
    out.push(`${ca.id} dof ${cb.dof}/${ca.dof} Δ${d.toExponential(1)}`);
  }
  console.log(name.padEnd(12), out.join('  '));
  if (process.env.V) { console.log('  orig', JSON.stringify(a.components)); console.log('  ours', JSON.stringify(b.components)); }
}
