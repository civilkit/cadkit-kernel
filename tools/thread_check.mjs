// Threads through the wasm (`threadData` request and the `thread` feature, docs/spec-agent-interface.md).
// usage: node tools/thread_check.mjs [--wasm path] [--time]     exit 1 on any mismatch
// --time prints the modeled-thread timings (M6 and M12 internal) without asserting on them.
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));
const showTime = process.argv.includes('--time');

let bad = 0;
const check = (name, ok, detail = '') => {
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${detail ? `: ${detail}` : ''}`);
};
const near = (a, b, tol) => Math.abs(a - b) <= tol;

// ---- threadData
{
  const k = new Kernel(mod);
  const ask = (msg) => JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'threadData', ...msg } })).json);
  const all = ask({});
  const ids = (all.result?.standards ?? []).map((s) => s.id);
  check('threadData standards', all.ok && ['metric', 'metric-fine', 'UNC', 'UNF', 'BSP', 'NPT', 'Tr', 'Acme'].every((s) => ids.includes(s)), ids.join(','));
  const m = ask({ standard: 'metric', size: 'M6' }).result?.sizes?.[0];
  check('M6 numbers', m && m.pitch === 1 && near(m.minorInternal, 4.9175, 1e-3) && m.tapDrill === 5 && m.classInternal === '6H', JSON.stringify(m));
  const u = ask({ standard: 'UNC', size: '1/4-20' }).result?.sizes?.[0];
  check('1/4-20 UNC', u && near(u.major, 6.35, 1e-9) && near(u.tapDrill, 5.105, 0.01) && u.tapDrillName === '#7', JSON.stringify(u));
  const g = ask({ standard: 'BSP', size: 'G1/4' }).result?.sizes?.[0];
  check('G1/4', g && g.major === 13.157 && near(g.pitch, 1.337, 1e-3) && g.angle === 55, JSON.stringify(g));
  const n = ask({ standard: 'NPT', size: '1/4 NPT' }).result?.sizes?.[0];
  check('1/4 NPT', n && near(n.major, 13.7, 0.05) && near(n.tpi, 18, 1e-6) && n.tapered === true, JSON.stringify(n));
  const bad1 = ask({ standard: 'UNC', size: '1/4-2O' });
  check('unknown size lists suggestions', !bad1.ok && /1\/4-20/.test(bad1.error), bad1.error);
  check('unknown standard', !ask({ standard: 'zzz' }).ok);
}

// ---- thread feature
const BLOCK = [
  { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: [{ id: 'p1', x: 0, y: 0 }, { id: 'p2', x: 40, y: 0 }, { id: 'p3', x: 40, y: 40 }, { id: 'p4', x: 0, y: 40 }],
    entities: [['p1', 'p2'], ['p2', 'p3'], ['p3', 'p4'], ['p4', 'p1']].map(([a, b], i) => ({ type: 'line', id: `e${i + 1}`, p1: a, p2: b })), constraints: [] },
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 20 },
];
const hole = (d) => [...BLOCK,
  { type: 'sketch', id: 'F3', plane: { kind: 'std', id: 'top' }, points: [{ id: 'c', x: 20, y: 20 }], entities: [{ type: 'circle', id: 'c1', c: 'c', r: d / 2 }], constraints: [] },
  { type: 'extrude', id: 'F4', op: 'cut', sketch: 'F3', end: 'throughBoth', depth: 0 }];
const shaft = (d) => [
  { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: [{ id: 'c', x: 0, y: 0 }], entities: [{ type: 'circle', id: 'c1', c: 'c', r: d / 2 }], constraints: [] },
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 30 }];

function regen(features) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features };
  const t0 = performance.now();
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const ms = performance.now() - t0;
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json);
  return { res: r.result, errs: (r.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}`), vol: m.ok ? m.result?.volume : null, valid: v.ok ? v.result?.ok : null, ms };
}
const th = (o) => ({ type: 'thread', id: 'T1', ...o });

{
  const plain = regen(hole(5));
  const r = regen([...hole(5), th({ face: 'F4.S.c1', standard: 'metric', size: 'M6' })]);
  const t = r.res.threads?.[0];
  check('cosmetic internal M6', r.errs.length === 0 && t && t.size === 'M6' && t.internal === true && t.modeled === false && near(t.length, 20, 1e-6) && r.vol === plain.vol && !r.res.warnings, JSON.stringify(t));
  check('plain regen has no threads/warnings keys', !('threads' in plain.res) && !('warnings' in plain.res));
}
{
  const r = regen([...hole(6.5), th({ face: 'F4.S.c1', standard: 'metric', size: 'M6' })]);
  const w = r.res.warnings?.[0];
  check('mismatch hint on a hole is a warning', r.errs.length === 0 && w && w[1].code === 'threadDiameterMismatch' && w[1].params.expected === 5 && w[1].params.actual === 6.5 && /hole is 6.5, M6 tap drill is 5/.test(w[1].params.hint), JSON.stringify(w));
}
{
  const r = regen([...shaft(7), th({ face: 'F2.S.c1', standard: 'metric', size: 'M8' })]);
  const w = r.res.warnings?.[0];
  check('mismatch hint on a shaft', r.errs.length === 0 && w && w[1].code === 'threadDiameterMismatch' && /shaft is 7, M8 major diameter is 8/.test(w[1].params.hint), JSON.stringify(w));
}
{
  const r = regen([...hole(5), th({ face: 'F4.S.c1', standard: 'metric', size: 'M7.5' })]);
  check('size unknown', r.errs.join() === 'T1:threadSizeUnknown', r.errs.join());
  const r2 = regen([...hole(5), th({ face: 'F2.C0', standard: 'metric', size: 'M6' })]);
  check('non-cylinder face', r2.errs.join() === 'T1:threadFaceNotCylinder', r2.errs.join());
}

// ---- modeled (enabled once the modeled build lands)
const WITH_MODELED = true;
function modeled(name, features, face, o, wantTurns) {
  const base = regen(features);
  const r = regen([...features, th({ face, modeled: true, ...o })]);
  const t = r.res.threads?.[0];
  const dv = base.vol - r.vol;
  if (showTime) console.log(`time ${name}: ${r.ms.toFixed(0)} ms (base ${base.ms.toFixed(0)} ms), groove volume ${dv.toFixed(3)}`);
  return { r, t, dv, base };
}
function runModeled() {
  // analytic internal groove volume (ISO 60 degrees): the area of the groove trapezoid from the bore radius to the major
  // radius, swept about the axis at the centroid radius, per turn (Pappus), times the turns of the threaded length
  const grooveVolume = (P, major, bore, length) => {
    const t = Math.tan(Math.PI / 6);
    const rM = major / 2, rB = bore / 2;
    const gM = P / 8, gB = gM + 2 * (rM - rB) * t;
    const h = rM - rB;
    const area = 0.5 * (gM + gB) * h;
    const cr = rB + (h * (gB + 2 * gM)) / (3 * (gB + gM));
    return (area * 2 * Math.PI * cr * length) / P;
  };
  for (const [name, size, P, major, bore, len] of [['M6', 'M6', 1, 6, 5, 12], ['M12', 'M12', 1.75, 12, 10.2, 12]]) {
    const { r, t, dv } = modeled(`modeled internal ${name}`, hole(bore), 'F4.S.c1', { standard: 'metric', size, length: len }, len / P);
    const want = grooveVolume(P, major, bore, len);
    const fell = r.res.warnings?.some((w) => w[1].code === 'threadModelFallback');
    check(`modeled internal ${name}`, r.errs.length === 0 && t?.modeled === true && !fell && r.valid === true && Math.abs(dv - want) <= 0.10 * want, `dV ${dv.toFixed(2)} vs ${want.toFixed(2)}, valid ${r.valid}, ${JSON.stringify(r.res.warnings ?? [])}`);
  }
}
if (WITH_MODELED) runModeled();
console.log(bad ? `thread_check: ${bad} FAILED` : 'thread_check: ok');
process.exit(bad ? 1 : 0);
