// Wall + CPU time and result summary of benchcad parts through one or more wasms (certified-SSI threshold study).
// usage: node tools/bench/cert_timing.mjs --wasm name=path [--wasm name=path ...] [--runs N] [--parts id,id] [--set benchcad|brepjs] [--json out.json]
// Each run is a fresh Kernel: regen, massProps, validate. Reports min/median wall and CPU ms, volume, validity, face count.
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from '../kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const wasms = [];
let runs = 3, parts = null, json = null, set = 'benchcad';
for (let i = 0; i < args.length; i++) {
  if (args[i] === '--wasm') { const [n, p] = args[++i].split('='); wasms.push({ name: n, mod: new WebAssembly.Module(readFileSync(p)) }); }
  else if (args[i] === '--runs') runs = +args[++i];
  else if (args[i] === '--parts') parts = args[++i].split(',');
  else if (args[i] === '--set') set = args[++i];
  else if (args[i] === '--json') json = args[++i];
}
const dir = path.join(here, set);
const manifest = JSON.parse(readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
const ids = parts ?? manifest.parts.filter((m) => !manifest.known?.[m.id]?.skip).map((m) => m.id);
const med = (a) => [...a].sort((x, y) => x - y)[Math.floor(a.length / 2)];
const cpu = () => { const u = process.cpuUsage(); return (u.user + u.system) / 1000; };
const out = [];
for (const id of ids) {
  const part = JSON.parse(readFileSync(path.join(dir, `${id}.ckpart.json`), 'utf8'));
  for (const w of wasms) {
    const wall = [], cp = [];
    let info = {};
    for (let i = 0; i < runs; i++) {
      const k = new Kernel(w.mod);
      const q = (n, msg) => { const r = k.request(JSON.stringify({ id: n, msg })); return r.trap ? { trap: r.trap } : JSON.parse(r.json); };
      const t0 = performance.now(), c0 = cpu();
      const reg = q(1, { type: 'regen', tag: 't', part });
      wall.push(performance.now() - t0); cp.push(cpu() - c0);
      if (i === 0) {
        const mp = q(2, { type: 'massProps', tag: 't' }).result;
        const va = q(3, { type: 'validate', tag: 't' }).result;
        const b = reg.result?.body;
        info = { trap: reg.trap ?? null, errors: (reg.result?.errors ?? []).length, vol: mp?.volume ?? null, area: mp?.area ?? null, faces: b?.faces?.length ?? null, edges: b?.edges?.length ?? null, issues: (va?.issues ?? []).map((x) => x.code).join(',') };
      }
    }
    const row = { id, wasm: w.name, wallMin: Math.min(...wall), wallMed: med(wall), cpuMin: Math.min(...cp), cpuMed: med(cp), ...info };
    out.push(row);
    console.log(`${id.padEnd(28)} ${w.name.padEnd(4)} wall ${row.wallMin.toFixed(0).padStart(6)}/${row.wallMed.toFixed(0).padStart(6)}  cpu ${row.cpuMin.toFixed(0).padStart(6)}/${row.cpuMed.toFixed(0).padStart(6)}  vol ${row.vol} faces ${row.faces} err ${row.errors} ${row.issues}${row.trap ? ' TRAP' : ''}`);
  }
}
if (json) writeFileSync(json, JSON.stringify(out, null, 1));
