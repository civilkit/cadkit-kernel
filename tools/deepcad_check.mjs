// Run the translated DeepCAD cases through the kernel and compare volume/area
// with the OCP reference stored in tools/deepcad_cases.json.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';

const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', '/path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm');
const cases = JSON.parse(readFileSync(opt('--cases', 'tools/deepcad_cases.json'), 'utf8'));
const mod = new WebAssembly.Module(readFileSync(wasm));
const k = new Kernel(mod);
const rows = [];
for (const [name, c] of Object.entries(cases)) {
  let cls = 'ok', detail = '';
  try {
    const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'm', part: c.part } }));
    const r = JSON.parse(rr.json);
    if (!r.ok || r.result?.errors?.length) { cls = 'feature-error'; detail = JSON.stringify(r.result?.errors ?? r.error).slice(0, 120); }
    else {
      const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'm', bodies: true } })).json);
      const v = m.result?.volume ?? null, a = m.result?.area ?? null;
      if (v == null) { cls = 'no-body'; }
      else {
        const vr = Math.abs(v - c.occt.volume) / Math.max(Math.abs(c.occt.volume), 1e-12);
        const ar = Math.abs(a - c.occt.area) / Math.max(Math.abs(c.occt.area), 1e-12);
        detail = `vol ${v.toExponential(6)} occt ${c.occt.volume.toExponential(6)} (${vr.toExponential(1)}); area ${a.toExponential(6)} (${ar.toExponential(1)})`;
        if (Math.max(vr, ar) > 1e-4) cls = 'wrong';
        else if (vr > 1e-6 || ar > 1e-4) cls = 'ok-loose';
      }
    }
  } catch (e) { cls = 'trap'; detail = String(e.message ?? e); }
  rows.push({ name, cls, detail });
}
const by = {};
for (const r of rows) by[r.cls] = (by[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n: rows.length, ...by }));
for (const r of rows) if (r.cls !== 'ok') console.log(`${r.cls} ${r.name}: ${r.detail}`);
const out = opt('--json'); if (out) (await import('node:fs')).writeFileSync(out, JSON.stringify(rows, null, 1));
