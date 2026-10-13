// Differential probe: the ORIGINAL kernel vs a rebuilt wasm on the parts of
// tools/stub_cases.mjs (features that were stubs in ck-ops).
// usage: node tools/cmp_stubs.mjs [--ours wasm] [--orig-only] [--json] [name-substring ...]
// Per part prints errors (codes), solid faces/edges/volume/area and, per
// surface body, faces/edges and the summed face areas (measure query).
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
import { parts } from './stub_cases.mjs';

const args = process.argv.slice(2);
const take = (f, v) => { const i = args.indexOf(f); if (i < 0) return null; const x = v ? args[i + 1] : true; args.splice(i, v ? 2 : 1); return x; };
const oursPath = take('--ours', true) ?? new URL('../target/wasm32-unknown-unknown/fast/ck_wasm.wasm', import.meta.url);
const origOnly = take('--orig-only');
const asJson = take('--json');
const orig = new WebAssembly.Module(readFileSync(new URL('../../kernel.wasm', import.meta.url)));
const ours = origOnly ? null : new WebAssembly.Module(readFileSync(oursPath));

export function summarize(mod, part) {
  const k = new Kernel(mod);
  const rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
  if (rep.trap) return { trap: rep.trap.slice(0, 120) };
  const r = JSON.parse(rep.json);
  if (!r.ok && r.error) return { error: JSON.stringify(r.error).slice(0, 160) };
  const res = r.result ?? {};
  const out = { errors: (res.errors ?? []).map(([id, e]) => `${id}:${e.code}${e.params?.detail ? `(${String(e.params.detail).slice(0, 60)})` : ''}`) };
  // mesh area (triangles of the reply's binary buffer): works for surface bodies, which
  // the rebuilt kernel cannot `measure` (no solid body -> empty state)
  const jl = new DataView(rep.bytes.buffer).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const meshArea = (m) => {
    if (!m) return 0;
    const get = (v) => { const C = { f32: Float32Array, i32: Int32Array }[v.type]; return new C(rep.bytes.buffer.slice(rep.bytes.byteOffset + base + v.offset, rep.bytes.byteOffset + base + v.offset + v.len * C.BYTES_PER_ELEMENT)); };
    const pos = get(m.positions ?? m.pos), n = get(m.triFace).length;
    let a = 0;
    for (let i = 0; i < n; i++) {
      const o = i * 9, ax = pos[o + 3] - pos[o], ay = pos[o + 4] - pos[o + 1], az = pos[o + 5] - pos[o + 2], bx = pos[o + 6] - pos[o], by = pos[o + 7] - pos[o + 1], bz = pos[o + 8] - pos[o + 2];
      a += 0.5 * Math.hypot(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
    }
    return a;
  };
  if (res.body) {
    out.solid = { faces: res.body.faces.length, edges: res.body.edges.length };
    const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json ?? 'null')?.result;
    if (m) { out.solid.vol = +m.volume.toFixed(5); out.solid.area = +m.area.toFixed(5); }
    const v = JSON.parse(k.request(JSON.stringify({ id: 4, msg: { type: 'validate', tag: 'main' } })).json ?? 'null')?.result;
    if (v?.issues?.length) out.solid.issues = v.issues.slice(0, 2).map((i) => `${i.code}`);
  }
  out.surfaces = (res.surfaces ?? []).map(([id, s]) => ({
    id, faces: s.faces.length, edges: s.edges.length,
    area: +meshArea(s.mesh).toFixed(3),
  }));
  const r3 = (p) => p.map((x) => +x.toFixed(3));
  out.curves = (res.curves ?? []).filter(([, c]) => c.pts).map(([id, c]) => ({ id, a: r3(c.pts[0]), b: r3(c.pts.at(-1)) }));
  // editCurve-style markers: informational (the fits differ), not part of the SAME test
  out.info = (res.markers ?? []).filter(([, m]) => m.cvs).map(([id, m]) => `${id}: deg ${m.degree} count ${m.count} corners ${m.corners} dev ${m.deviation} edited ${m.edited} w ${m.weights.some((w) => w !== 1) ? 'yes' : 'no'}`);
  return out;
}

const filt = args;
for (const [name, part] of Object.entries(parts)) {
  if (filt.length && !filt.some((f) => name.includes(f))) continue;
  const a = summarize(orig, part);
  const b = ours ? summarize(ours, part) : null;
  if (asJson) { console.log(JSON.stringify({ name, orig: a, ours: b })); continue; }
  const core = (x) => JSON.stringify({ ...x, info: undefined });
  const same = b && core(a) === core(b);
  console.log(`${name.padEnd(22)} ORIG ${core(a)}${a.info?.length ? '  ' + a.info.join('; ') : ''}`);
  if (b) console.log(`${''.padEnd(22)} ${same ? 'SAME' : 'OURS'} ${same ? '' : core(b)}${b.info?.length ? '  ' + b.info.join('; ') : ''}`);
}
