import { readFileSync } from 'node:fs';
import { Kernel } from '../kernel.mjs';
export const A = new WebAssembly.Module(readFileSync(new URL('../../../kernel.wasm', import.meta.url)));
export const B = () => new WebAssembly.Module(readFileSync(new URL('../../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url)));
export const pt = (id, x, y) => ({ id, x, y });
export const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
export const STD = (id) => ({ kind: 'std', id });
export function rect(x0, y0, x1, y1, tags = ['e1', 'e2', 'e3', 'e4'], pfx = 'p') {
  const P = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([x, y], i) => pt(`${pfx}${i + 1}`, x, y));
  const E = tags.map((t, i) => line(t, `${pfx}${i + 1}`, `${pfx}${((i + 1) % 4) + 1}`));
  return [P, E];
}
export const sketch = (id, plane, points, entities) => ({ type: 'sketch', id, plane: typeof plane === 'string' ? STD(plane) : plane, points, entities, constraints: [] });
export const extrude = (id, sk, o = {}) => ({ type: 'extrude', id, op: 'add', sketch: sk, end: 'blind', depth: 10, ...o });
export const part = (features, o = {}) => ({ format: 'ck-part', version: 1, features, ...o });
export const boxPart = (w = 40, d = 20, h = 10) => { const [P, E] = rect(0, 0, w, d); return part([sketch('F1', 'top', P, E), extrude('F2', 'F1', { depth: h })]); };
export const cylPart = (r = 5, h = 20, plane = 'top', cx = 0, cy = 0) => part([sketch('F1', plane, [pt('c', cx, cy)], [{ type: 'circle', id: 'e1', c: 'c', r }]), extrude('F2', 'F1', { depth: h })]);
export const ins = (id, p, o = {}) => ({ type: 'insertPart', id, name: id, part: p, ...o });
export function run(mod, msg, extra = []) {
  const k = new Kernel(mod);
  const r = k.request(JSON.stringify({ id: 1, msg }));
  if (r.trap) return { trap: r.trap };
  const o = JSON.parse(r.json);
  const b = o?.result?.body; if (b?.edges) for (const e of b.edges) e.pts = e.pts.length;
  if (b) { delete b.mesh; delete b.silhouette; }
  const ex = extra.map((m) => JSON.parse(k.request(JSON.stringify({ id: 2, msg: m })).json));
  return { o, ex, trace: (o.trace ?? []).map((t) => `${'  '.repeat(t.depth)}${t.label}`) };
}
export const regen = (p, o = {}) => ({ type: 'regen', tag: 'main', part: p, ...o });
export const show = (o, keys = ['errors', 'components']) => { const r = o.result ?? o; for (const k of keys) console.log(k, JSON.stringify(r[k])); };
export const faces = (o) => (o.result?.body?.faces ?? []).map((f) => `${f.id}${f.lump !== undefined ? ' L' + f.lump : ''}${f.normal ? ' n' + JSON.stringify(f.normal) : ''}`);
