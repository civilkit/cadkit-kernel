// Shell and moveFace of an extruded closed spline (kidney) outline through the wasm: wall volumes against an
// independently offset polygon, structured too-thick hints (docs/spec-agent-interface.md, "Offset limits").
// usage: node tools/spline_shell_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

let bad = 0;
const check = (name, ok, detail = '') => {
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${detail ? `: ${detail}` : ''}`);
};
const near = (a, b, tol) => Math.abs(a - b) <= tol * Math.max(1, Math.abs(b));

const KIDNEY = [[-30, 0], [-20, 15], [0, 9], [20, 15], [30, 0], [15, -15], [-15, -15]];
const part = (h, ...extra) => [
  { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' }, points: KIDNEY.map(([x, y], i) => ({ id: `p${i}`, x, y })),
    entities: [{ type: 'spline', id: 'e1', pts: KIDNEY.map((_, i) => `p${i}`), closed: true }], constraints: [] },
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: h },
  ...extra,
];

function regen(features) {
  const k = new Kernel(mod);
  const ask = (id, msg) => JSON.parse(k.request(JSON.stringify({ id, msg })).json);
  const r = ask(1, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features } });
  const m = ask(2, { type: 'massProps', tag: 'main' });
  const v = ask(3, { type: 'validate', tag: 'main' });
  const errors = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code, params: e.params ?? e }));
  return { errors, vol: m.ok ? m.result?.volume : null, valid: v.ok ? v.result?.ok : null };
}

// The outline area and the areas offset by +-1 come from the kernel itself (base volume, moveFace of both walls,
// which the native test checks against an independently offset polygon); the shell volumes must then follow
// the wall-volume identity (outer area - inner area) x height.
const H = 5;
const base = regen(part(H));
check('kidney extrudes', base.errors.length === 0 && base.vol > 0 && base.valid === true, `${base.errors.map((e) => e.code)}`);
const A = base.vol / H;

// offset areas from moveFace of both walls (kernel offset of the same wall, validated separately in the native test)
const walls = ['F2.S.e1~1', 'F2.S.e1~2'];
const mf = (d) => regen(part(H, { type: 'moveFace', id: 'M1', faces: walls, distance: d }));
const grow = mf(1);
const shrink = mf(-1);
check('moveFace on the spline wall', grow.errors.length === 0 && shrink.errors.length === 0 && grow.valid === true && shrink.valid === true, `${grow.errors.map((e) => e.code)} ${shrink.errors.map((e) => e.code)}`);
const Aout = grow.vol / H;
const Ain = shrink.vol / H;
// first-order: A_out - A and A - A_in both ~ perimeter x 1 (second order terms are the dent / ends turning)
check('offset areas bracket the outline', Aout > A && Ain < A && near(Aout - A, A - Ain, 0.1), `A ${A.toFixed(2)} out ${Aout.toFixed(2)} in ${Ain.toFixed(2)}`);

const shell = (faces, t, outward) => regen(part(H, { type: 'shell', id: 'S1', faces, thickness: t, outward }));
{
  const r = shell(['F2.C1'], 1, false);
  check('inward shell, top open', r.errors.length === 0 && r.valid === true && near(r.vol, base.vol - Ain * (H - 1), 4e-3), `${r.errors.map((e) => e.code)} vol ${r.vol} want ${base.vol - Ain * (H - 1)}`);
  const c = shell([], 1, false);
  check('inward shell, closed', c.errors.length === 0 && c.valid === true && near(c.vol, base.vol - Ain * (H - 2), 4e-3), `${c.errors.map((e) => e.code)} vol ${c.vol} want ${base.vol - Ain * (H - 2)}`);
  const o = shell(['F2.C1'], 1, true);
  check('outward shell, top open', o.errors.length === 0 && o.valid === true && near(o.vol, Aout * (H + 1) - base.vol, 4e-3), `${o.errors.map((e) => e.code)} vol ${o.vol} want ${Aout * (H + 1) - base.vol}`);
}
{
  const tall = regen(part(20, { type: 'shell', id: 'S1', faces: ['F2.C1'], thickness: 12, outward: false }));
  const e = tall.errors[0];
  const p = e && (Array.isArray(e.params) ? Object.fromEntries(e.params) : e.params);
  check('too thick is a structured hint', e?.code === 'shellTooThick' && typeof p?.maxThickness === 'number' && typeof p?.minRadius === 'number' && p.maxThickness < 12 && /use at most/.test(p.detail ?? ''), JSON.stringify(e));
  const ok = regen(part(20, { type: 'shell', id: 'S1', faces: ['F2.C1'], thickness: p?.maxThickness ?? 1, outward: false }));
  check('the suggested thickness shells', ok.errors.length === 0 && ok.valid === true, `${ok.errors.map((x) => x.code)}`);
}
console.log(bad ? `spline_shell_check: ${bad} FAILED` : 'spline_shell_check: ok');
process.exit(bad ? 1 : 0);
