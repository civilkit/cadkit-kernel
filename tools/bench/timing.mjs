// Timing comparison: rebuilt kernel.wasm vs the ORIGINAL kernel.wasm on the
// same BenchCAD parts (regen + massProps). Reports median wall time per part
// and the ratio ours/original. Not a gate metric — a diagnostic for finding
// speed-ups.
//
// usage: node tools/bench/timing.mjs [--ours path] [--runs N] [--only id]
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from '../kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const dir = path.join(here, 'benchcad');

const args = process.argv.slice(2);
const opt = (f) => { const i = args.indexOf(f); if (i < 0) return null; const v = args[i + 1]; args.splice(i, 2); return v; };
const oursPath = opt('--ours') ?? path.join(here, '../../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const origPath = opt('--orig') ?? path.join(here, '../../kernel.wasm');
const runs = Number(opt('--runs') ?? 3);
const only = opt('--only');

function makeKernel(wasmPath) {
  const B = new WebAssembly.Module(readFileSync(wasmPath));
  return (id, msg) => {
    const k = new Kernel(B);
    const r = k.request(JSON.stringify({ id, msg }));
    return r.trap ? { trap: r.trap } : { json: r.json };
  };
}

const median = (a) => { const s = [...a].sort((x, y) => x - y); return s[Math.floor(s.length / 2)]; };

const manifest = JSON.parse(readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
const parts = manifest.parts.filter((m) => !only || m.id === only);
const ourK = makeKernel(oursPath);
const origK = makeKernel(origPath);

console.log(`${'part'.padEnd(22)} ${'ours_ms'.padStart(9)} ${'orig_ms'.padStart(9)} ${'ratio'.padStart(7)}  status`);
const rows = [];
for (const m of parts) {
  if (manifest.known?.[m.id]?.skip) { console.log(`${m.id.padEnd(22)} ${'-'.padStart(9)} ${'-'.padStart(9)} ${'-'.padStart(7)}  skipped: ${manifest.known[m.id].skip.slice(0, 40)}`); continue; }
  const part = JSON.parse(readFileSync(path.join(dir, `${m.id}.ckpart.json`), 'utf8'));
  const time = (K) => {
    const ts = [];
    let ok = true, vol = null;
    for (let i = 0; i < runs; i++) {
      const t0 = performance.now();
      const reg = K(i * 1000 + 1, { type: 'regen', tag: `t${i}`, part });
      if (reg.trap) { ok = false; break; }
      const r = JSON.parse(reg.json);
      if (r.result?.errors?.length) ok = false;
      const mp = K(i * 1000 + 2, { type: 'massProps', tag: `t${i}` });
      if (!mp.trap) vol = JSON.parse(mp.json).result?.volume ?? vol;
      ts.push(performance.now() - t0);
    }
    return { ms: ok ? median(ts) : null, ok, vol };
  };
  const o = time(ourK), g = time(origK);
  const ratio = o.ms != null && g.ms != null ? (o.ms / g.ms) : null;
  const status = !o.ok ? 'OURS-FAIL' : !g.ok ? 'ORIG-FAIL' : 'ok';
  console.log(`${m.id.padEnd(22)} ${(o.ms != null ? o.ms.toFixed(0) : '-').padStart(9)} ${(g.ms != null ? g.ms.toFixed(0) : '-').padStart(9)} ${(ratio != null ? ratio.toFixed(2) + 'x' : '-').padStart(7)}  ${status}`);
  rows.push({ id: m.id, ours: o.ms, orig: g.ms, ratio, status, oursVol: o.vol, origVol: g.vol });
}
const ok = rows.filter((r) => r.ratio != null);
if (ok.length) {
  const ratios = ok.map((r) => r.ratio).sort((a, b) => a - b);
  console.log(`\n${ok.length} parts timed. ratio median ${median(ratios).toFixed(2)}x, min ${ratios[0].toFixed(2)}x, max ${ratios[ratios.length - 1].toFixed(2)}x (ours/orig; >1 = we are slower)`);
  console.log('slowest vs original:');
  for (const r of [...ok].sort((a, b) => b.ratio - a.ratio).slice(0, 8)) console.log(`  ${r.id.padEnd(22)} ${r.ratio.toFixed(2)}x  ours ${r.ours.toFixed(0)}ms vs orig ${r.orig.toFixed(0)}ms`);
  console.log('fastest vs original:');
  for (const r of [...ok].sort((a, b) => a.ratio - b.ratio).slice(0, 5)) console.log(`  ${r.id.padEnd(22)} ${r.ratio.toFixed(2)}x  ours ${r.ours.toFixed(0)}ms vs orig ${r.orig.toFixed(0)}ms`);
}
