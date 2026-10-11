// Differential harness: run every case in tools/cases.mjs (plus any extra
// raw bodies given as args / --file) against the ORIGINAL kernel.wasm and the
// REBUILT wasm, and diff the replies byte-for-byte with "ms" normalised
// (both hosts' clocks return 0, so ms only differs if the rebuild times
// differently).
//
// usage: node tools/diff.mjs [--wasm path] [--verbose] [--file bodies.json] ['<body>' ...]
//   default rebuilt wasm: target/wasm32-unknown-unknown/release/ck_wasm.wasm
//   (build: cargo build -p ck-wasm --target wasm32-unknown-unknown --release)
import { readFileSync } from 'node:fs';
import cases from './cases.mjs';
import { Kernel } from './kernel.mjs';

const args = process.argv.slice(2);
let rebuilt = new URL('../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url);
let verbose = false; const extra = [];
for (let i = 0; i < args.length; i++) {
  if (args[i] === '--wasm') rebuilt = args[++i];
  else if (args[i] === '--verbose') verbose = true;
  else if (args[i] === '--file') extra.push(...JSON.parse(readFileSync(args[++i], 'utf8')).map((b) => (typeof b === 'string' ? b : JSON.stringify(b))));
  else extra.push(args[i]);
}
const A = new WebAssembly.Module(readFileSync(new URL('../../kernel.wasm', import.meta.url)));
const B = new WebAssembly.Module(readFileSync(rebuilt));

// ABI shape must match exactly.
const sig = (m) => JSON.stringify({ imports: WebAssembly.Module.imports(m).map((x) => `${x.module}.${x.name}:${x.kind}`).sort(),
  exports: WebAssembly.Module.exports(m).map((x) => `${x.name}:${x.kind}`).filter((s) => !s.startsWith('__')).sort() });
if (sig(A) !== sig(B)) console.log(`ABI MISMATCH\n  original ${sig(A)}\n  rebuilt  ${sig(B)}`);
const ka = new Kernel(A), kb = new Kernel(B);
if (ka.commit() !== kb.commit()) console.log(`COMMIT MISMATCH ${ka.commit()} vs ${kb.commit()}`);

const norm = (r) => {
  if (r.trap) return { s: `TRAP ${r.trap.replace(/\/rustc\/[0-9a-f]+\/|:\d+:\d+:/g, '')}`, bin: new Uint8Array() };
  // replace "ms" values (top-level + trace) with 0 while keeping frame layout comparable
  const jl = new DataView(r.bytes.buffer).getUint32(0, true);
  const base = (4 + jl + 7) & -8;
  return { s: r.json.replace(/"ms":[0-9.e+-]+/g, '"ms":0').replace(/"at":[0-9.e+-]+/g, '"at":0'), bin: r.bytes.subarray(base) };
};
const same = (a, b) => a.s === b.s && a.bin.length === b.bin.length && a.bin.every((x, i) => x === b.bin[i]);

const KNOWN = new Set(['ping', 'regen', 'matchEdges', 'drag', 'convert', 'sketchOffset', 'refCurve', 'measure', 'massProps',
  'interference', 'step', 'inspect', 'continuity', 'curvatureAt', 'curvatureMap', 'comb', 'drawing']);
const all = [...cases, ...extra.map((b, i) => ({ name: `arg#${i}`, bodies: [b] }))];
let ok = 0, bad = 0; const byType = new Map();
for (const c of all) {
  const a = new Kernel(A), b = new Kernel(B);
  for (const body of c.bodies) {
    const ra = norm(a.request(body)), rb = norm(b.request(body));
    let ty = '?'; try { ty = JSON.parse(body).msg?.type ?? '(envelope)'; } catch { ty = '(envelope)'; }
    if (typeof ty !== 'string' || !ty) ty = '(envelope)';
    else if (ty !== '(envelope)' && !KNOWN.has(ty)) ty = '(unknown type)';
    const t = byType.get(ty) || { ok: 0, bad: 0 }; byType.set(ty, t);
    if (same(ra, rb)) { ok++; t.ok++; if (verbose) console.log(`ok   ${c.name}`); continue; }
    bad++; t.bad++;
    console.log(`DIFF ${c.name}: ${body.slice(0, 160)}\n  orig    ${ra.s.slice(0, 300)} [+${ra.bin.length}B]\n  rebuilt ${rb.s.slice(0, 300)} [+${rb.bin.length}B]`);
  }
}
console.log(`\n${ok} match, ${bad} differ (${all.length} cases)`);
for (const [t, v] of [...byType].sort()) console.log(`  ${t.padEnd(14)} ${v.ok} ok${v.bad ? `, ${v.bad} DIFF` : ''}`);
process.exitCode = bad ? 1 : 0;
