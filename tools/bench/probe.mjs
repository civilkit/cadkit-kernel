// usage: node tools/bench/probe.mjs <part.ckpart.json> [--upto Fn] [--edges] [--faces]
// regen a translated part (optionally truncated after feature Fn) and print errors,
// body box, and the face / edge names with edge midpoints (our Y-up frame) for authoring blend refs.
import { readFileSync } from 'node:fs';
import { Kernel } from '../kernel.mjs';
const B = new WebAssembly.Module(readFileSync(new URL('../../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url)));
const k = new Kernel(B);
const q = (id, msg) => { const r = k.request(JSON.stringify({ id, msg })); return r.trap ? { trap: r.trap } : JSON.parse(r.json); };
const a = process.argv.slice(2);
const part = JSON.parse(readFileSync(a[0], 'utf8'));
const ui = a.indexOf('--upto');
if (ui >= 0) { const i = part.features.findIndex((f) => f.id === a[ui + 1]); part.features = part.features.slice(0, i + 1); }
const r = q(1, { type: 'regen', tag: 'main', part });
console.log('errors', JSON.stringify(r.result?.errors ?? r));
const b = r.result?.body;
if (b) {
  const mp = q(2, { type: 'massProps', tag: 'main' }).result;
  const va = q(3, { type: 'validate', tag: 'main' }).result;
  console.log('box', JSON.stringify(b.box), 'vol', mp?.volume, 'area', mp?.area, 'validate', (va?.issues ?? []).map((i) => i.code).join(',') || 'ok');
  const f3 = (v) => v.map((x) => +x.toFixed(3)).join(',');
  if (a.includes('--faces')) for (const f of b.faces) console.log('face', f.id, f.kind);
  if (a.includes('--edges')) for (const e of b.edges) console.log('edge', e.faces.join(' | '), '@', f3(e.mid), e.circle ? `circle r=${+e.circle.r.toFixed(3)}` : '');
}
