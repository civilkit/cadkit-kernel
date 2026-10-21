// Feature-recognition corpus runner (docs/recognition.md).
//
// For every sample part: regen with the kernel, STEP-export, re-import (a history-free body), `recognize` it, and classify
//   exact   symmetric difference of the rebuilt solid and the original < 1e-6 (relative volume)
//   close   < 1e-3
//   partial some faces explained but the rebuild is not close (or recognition left faces unexplained)
//   failed  nothing recognised / rebuild failed / import failed
// The prismatic subset (samples whose features are all sketch/extrude/revolve/plane/hole/fillet/chamfer/patterns) is gated:
// exit code 1 when exact+close on it drops below tools/recog_baseline.json.
//
// usage: node tools/recog_check.mjs [--wasm PATH] [--samples DIR] [--sldprt DIR] [--filter SUBSTR] [--json]
//                                   [--update-baseline] [--no-gate] [--limit N]
import { readFileSync, writeFileSync, readdirSync, existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (n, d) => { const i = args.indexOf(n); return i >= 0 ? args[i + 1] : d; };
const wasmPath = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const samplesDir = opt('--samples', fileURLToPath(new URL('../../kit/apps/cad/public/samples', import.meta.url)));
const sldDir = opt('--sldprt', fileURLToPath(new URL('../../scratch-sldprt/samples', import.meta.url)));
const filter = opt('--filter', '');
const limit = Number(opt('--limit', '0')) || Infinity;
const asJson = args.includes('--json');
const baselinePath = path.join(here, 'recog_baseline.json');

const { Kernel } = await import(path.join(here, 'kernel.mjs'));
const mod = await WebAssembly.compile(readFileSync(wasmPath));
const k = new Kernel(mod, { now: () => performance.now() });
const call = (msg) => {
  const r = k.request(JSON.stringify({ id: 1, msg }));
  if (r.trap) return { ok: false, error: 'trap ' + r.trap };
  return JSON.parse(r.json);
};

const PRISMATIC = new Set(['sketch', 'extrude', 'revolve', 'plane', 'hole', 'fillet', 'chamfer', 'cpattern', 'lpattern', 'thread', 'axis']);

function classify(rec) {
  if (!rec.ok) return 'failed';
  const r = rec.result;
  if (!r.features.length) return 'failed';
  const v = r.verify;
  if (!v) return 'failed';
  if (v.exact && r.report.unexplained.length === 0) return 'exact';
  if (v.close && r.report.unexplained.length === 0) return 'close';
  return 'partial';
}

function runOne(name, importMsg, sub) {
  const t0 = Date.now();
  const imp = call({ ...importMsg, tag: 'imp' });
  if (!imp.ok || (imp.result && imp.result.body == null)) return { name, sub, status: 'failed', why: 'import', faces: 0, ms: 0 };
  const faces = imp.result.import ? imp.result.import.faces : (imp.result.body?.faces?.length ?? 0);
  const rec = call({ type: 'recognize', tag: 'imp' });
  const status = classify(rec);
  const r = rec.ok ? rec.result : null;
  return {
    name, sub, status, faces,
    ms: rec.ms ?? 0, wall: Date.now() - t0,
    strategy: r?.report?.strategy, features: r ? r.features.map((f) => f.type).join(',') : '',
    rel: r?.verify?.relativeError, explained: r?.report?.explainedFraction,
    unexplained: r ? r.report.unexplained.map((u) => `${u.kind}:${u.area.toFixed(1)}`).slice(0, 4).join(' ') : '',
    why: rec.ok ? (r.errors.length ? r.errors.join(';') : (r.report.note || '')) : rec.error,
  };
}

const rows = [];
let n = 0;
for (const f of readdirSync(samplesDir).filter((x) => x.endsWith('.ckpart')).sort()) {
  if (filter && !f.includes(filter)) continue;
  if (n++ >= limit) break;
  const part = JSON.parse(readFileSync(path.join(samplesDir, f), 'utf8'));
  const types = new Set(part.features.map((x) => x.type));
  const sub = [...types].every((t) => PRISMATIC.has(t)) && !f.startsWith('assembly-');
  const rg = call({ type: 'regen', tag: 'orig', part, opts: { noView: true } });
  if (!rg.ok || !rg.result || !rg.result.errors || rg.result.errors.length || !call({ type: 'massProps', tag: 'orig' }).result) {
    rows.push({ name: f.replace('.ckpart', ''), sub, status: 'skipped', why: 'original regen failed' });
    continue;
  }
  const st = call({ type: 'step', tag: 'orig', name: 'part' });
  if (!st.ok || typeof st.result !== 'string') { rows.push({ name: f, sub, status: 'skipped', why: 'step export' }); continue; }
  rows.push(runOne(f.replace('.ckpart', ''), { type: 'importStep', name: 'part', data: st.result }, sub));
}
if (existsSync(sldDir) && !filter) {
  for (const d of readdirSync(sldDir).filter((x) => /SW2022_C(00|03|04|13|14|15|20|21)/.test(x)).sort()) {
    const p = path.join(sldDir, d, 'model.SLDPRT');
    if (!existsSync(p)) continue;
    const b64 = readFileSync(p).toString('base64');
    rows.push(runOne('sw:' + d.replace('dump_', ''), { type: 'importSldprt', name: d, data: b64 }, false));
  }
}

const count = (rs, s) => rs.filter((r) => r.status === s).length;
const prism = rows.filter((r) => r.sub && r.status !== 'skipped');
const summary = {
  total: rows.length, skipped: count(rows, 'skipped'),
  exact: count(rows, 'exact'), close: count(rows, 'close'), partial: count(rows, 'partial'), failed: count(rows, 'failed'),
  prismatic: { total: prism.length, exactPlusClose: count(prism, 'exact') + count(prism, 'close') },
};
if (asJson) console.log(JSON.stringify({ summary, rows }, null, 1));
else {
  const pad = (s, w) => String(s ?? '').padEnd(w).slice(0, w);
  console.log(pad('part', 34), pad('status', 8), pad('faces', 6), pad('ms', 7), pad('strategy', 9), pad('relErr', 10), pad('expl', 5), 'features / unexplained');
  for (const r of rows) {
    console.log(pad(r.name + (r.sub ? ' *' : ''), 34), pad(r.status, 8), pad(r.faces, 6), pad(r.ms?.toFixed?.(0), 7), pad(r.strategy, 9),
      pad(r.rel == null ? '' : r.rel.toExponential(1), 10), pad(r.explained == null ? '' : r.explained.toFixed(2), 5), (r.features || '') + (r.unexplained ? '  | ' + r.unexplained : '') + (r.why ? '  ! ' + r.why : ''));
  }
  console.log(JSON.stringify(summary));
}

if (args.includes('--update-baseline')) {
  writeFileSync(baselinePath, JSON.stringify({ prismatic: summary.prismatic }, null, 1) + '\n');
  console.log('baseline written');
} else if (!args.includes('--no-gate') && !filter && existsSync(baselinePath)) {
  const base = JSON.parse(readFileSync(baselinePath, 'utf8')).prismatic;
  if (summary.prismatic.exactPlusClose < base.exactPlusClose) {
    console.error(`recognition regressed: prismatic exact+close ${summary.prismatic.exactPlusClose} < baseline ${base.exactPlusClose}`);
    process.exit(1);
  }
}
