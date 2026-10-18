// Run translated DeepCAD cases through the kernel and compare volume/area
// with the OCP reference. Supports a uniform scale and a class-only baseline.
//
//   node tools/deepcad_check.mjs --cases FILE [--scale 100]
//   node tools/deepcad_check.mjs --cases FILE --update-baseline tools/deepcad_baseline.json
//   node tools/deepcad_check.mjs --cases FILE --check tools/deepcad_baseline.json
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { Kernel } from './kernel.mjs';

const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', '/path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm');
const cases = JSON.parse(readFileSync(opt('--cases', 'tools/deepcad_cases.json'), 'utf8'));
const scale = Number(opt('--scale', '1'));
const mod = new WebAssembly.Module(readFileSync(wasm));
const k = new Kernel(mod);

function scaled(part, f) {
  const p = JSON.parse(JSON.stringify(part));
  for (const ft of p.features) {
    if (ft.type === 'sketch') {
      for (const pt of ft.points ?? []) { pt.x *= f; pt.y *= f; }
      for (const e of ft.entities ?? []) if (e.type === 'circle') e.r *= f;
    } else if (ft.type === 'plane') {
      ft.offset = (ft.offset ?? 0) * f;
    } else if (ft.type === 'extrude' || ft.type === 'revolve') {
      if (ft.depth != null) ft.depth *= f;
    } else if (ft.type === 'moveBody') {
      if (ft.translate) ft.translate = ft.translate.map((x) => x * f);
      if (ft.rotate?.origin) ft.rotate.origin = ft.rotate.origin.map((x) => x * f);
    } else if (ft.type === 'fillet' || ft.type === 'chamfer') {
      if (ft.radius != null) ft.radius *= f;
      if (ft.distance != null) ft.distance *= f;
    }
  }
  return p;
}

const rows = [];
for (const [name, c] of Object.entries(cases)) {
  let cls = 'ok', detail = '', ms = 0;
  const t0 = Date.now();
  try {
    const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'm', part: scaled(c.part, scale) } }));
    const r = JSON.parse(rr.json);
    const errs = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code, ...e }));
    const fatal = errs.filter((e) => e.code !== 'cutNoEffect');
    if (!r.ok || fatal.length) { cls = 'feature-error'; detail = JSON.stringify(r.result?.errors ?? r.error).slice(0, 160); }
    else if (c.occt.volume <= 1e-12) { cls = 'unusable-reference'; detail = `occt volume ${c.occt.volume}`; }
    else {
      const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'm', bodies: true } })).json);
      const v = m.result?.volume ?? null, a = m.result?.area ?? null;
      if (v == null) { cls = 'no-body'; }
      else {
        const vr = Math.abs(v - c.occt.volume * scale ** 3) / Math.max(Math.abs(c.occt.volume * scale ** 3), 1e-12);
        const ar = Math.abs(a - c.occt.area * scale ** 2) / Math.max(Math.abs(c.occt.area * scale ** 2), 1e-12);
        detail = `vol ${v.toExponential(6)} occt ${(c.occt.volume * scale ** 3).toExponential(6)} (${vr.toExponential(1)}); area ${a.toExponential(6)} (${ar.toExponential(1)})`;
        if (Math.max(vr, ar) > 1e-4) cls = 'wrong';
        else if (vr > 1e-6 || ar > 1e-4) cls = 'ok-loose';
        else if (errs.some((e) => e.code === 'cutNoEffect')) cls = 'okNoEffect';
      }
    }
  } catch (e) { cls = 'trap'; detail = String(e.message ?? e); }
  ms = Date.now() - t0;
  rows.push({ name, cls, detail, ms });
}

const by = {};
for (const r of rows) by[r.cls] = (by[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n: rows.length, scale, ...by }));
for (const r of rows) if (!['ok', 'okNoEffect', 'ok-loose'].includes(r.cls)) console.log(`${r.cls} ${r.name}: ${r.detail}`);

const out = opt('--json');
if (out) writeFileSync(out, JSON.stringify(rows, null, 1));

const baselineFile = opt('--update-baseline') ?? opt('--check');
if (baselineFile) {
  if (opt('--update-baseline')) {
    const casesOut = Object.fromEntries(rows.map((r) => [r.name, r.cls]));
    writeFileSync(baselineFile, JSON.stringify({ version: 1, scale, cases: casesOut }, null, 1) + '\n');
    console.log(`deepcad_check: wrote baseline ${baselineFile} (${rows.length} ids)`);
  }
  if (opt('--check')) {
    if (!existsSync(baselineFile)) { console.error(`no baseline at ${baselineFile}`); process.exit(2); }
    const base = JSON.parse(readFileSync(baselineFile, 'utf8')).cases ?? {};
    const pass = new Set(['ok', 'okNoEffect', 'ok-loose']);
    const regressions = [], newPasses = [], added = [];
    for (const r of rows) {
      const b = base[r.name];
      if (!b) { added.push(r.name); continue; }
      if (pass.has(b) && !pass.has(r.cls)) regressions.push(r);
      else if (!pass.has(b) && pass.has(r.cls)) newPasses.push(r);
    }
    console.log(`deepcad_check check: ${regressions.length} regressions, ${newPasses.length} new passes, ${added.length} added`);
    for (const r of regressions) console.log(`REGRESSION ${r.name}: ${r.cls} ${r.detail}`);
    for (const r of newPasses) console.log(`new pass   ${r.name}: ${r.cls}`);
    if (regressions.length) process.exit(1);
  }
}
