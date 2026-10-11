// Regression gate + scoreboard: original kernel.wasm vs the rebuilt wasm.
//
// usage: node tools/gate.mjs [--fast] [--update] [--wasm path] [--quiet]
//   Prints a scoreboard and compares it with tools/scoreboard.json (the
//   committed baseline). Exit 1 if ANY metric went down. --update rewrites
//   the baseline (do this in the commit that improves a metric).
//   Build first: cargo build -p ck-wasm --target wasm32-unknown-unknown --release
//
// Metrics (counts of matching items; higher is better):
//   wire.*        tools/cases.mjs replies byte-identical (ms normalised)
//   regen.<set>.* per target: faces / edges (pts as counts) / box / result
//                 keys outside body / whole JSON reply (edge pts included)
//   mass.<set>    massProps volume/area/com within 1e-9 relative
//   step          STEP replies byte-identical (ck-io golden request bodies)
import { readFileSync, writeFileSync, existsSync, readdirSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import cases from './cases.mjs';
import { parts as brepParts } from './brep_cases.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const flag = (f) => { const i = args.indexOf(f); if (i < 0) return false; args.splice(i, 1); return true; };
const opt = (f) => { const i = args.indexOf(f); if (i < 0) return null; const v = args[i + 1]; args.splice(i, 2); return v; };
const update = flag('--update'), quiet = flag('--quiet'), fast = flag('--fast');
const wasmB = opt('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
// Artifacts live next to the ORIGINAL kernel.wasm (follows the worktree symlink).
const origWasm = realpathSync(path.join(here, '../../kernel.wasm'));
const art = path.dirname(origWasm);
const A = new WebAssembly.Module(readFileSync(origWasm));
const B = new WebAssembly.Module(readFileSync(wasmB));

const score = {};
const bump = (k, ok) => { score[k] ??= [0, 0]; score[k][1]++; if (ok) score[k][0]++; };
const norm = (r) => (r.trap ? `TRAP ${r.trap.replace(/\/rustc\/[0-9a-f]+\/|:\d+:\d+:/g, '')}` : r.json.replace(/"ms":[0-9.e+-]+/g, '"ms":0').replace(/"at":[0-9.e+-]+/g, '"at":0'));
const bin = (r) => (r.trap ? '' : Buffer.from(r.bytes.subarray((4 + new DataView(r.bytes.buffer).getUint32(0, true) + 7) & -8)).toString('base64'));

// 1. wire cases (stateful sequences)
for (const c of cases) {
  const ka = new Kernel(A), kb = new Kernel(B);
  for (const [i, body] of c.bodies.entries()) {
    const ra = ka.request(body), rb = kb.request(body);
    if (c.ignore?.includes(i)) continue;
    bump('wire', norm(ra) === norm(rb) && bin(ra) === bin(rb));
  }
}

// 2. regen targets
const rel = (x, y) => Math.abs(x - y) / (1 + Math.abs(y));
const close = (a, b) => {
  if (typeof a === 'number' && typeof b === 'number') return rel(a, b) <= 1e-9;
  if (Array.isArray(a) && Array.isArray(b)) return a.length === b.length && a.every((x, i) => close(x, b[i]));
  if (a && b && typeof a === 'object') return Object.keys(a).join() === Object.keys(b).join() && Object.keys(a).every((k) => close(a[k], b[k]));
  return a === b;
};
function regenSet(set, targets) {
  for (const t of targets) {
    const t0 = performance.now();
    const msg = { type: 'regen', tag: 'main', part: t.part, ...(t.upTo ? { opts: { upTo: t.upTo } } : {}) };
    const ka = new Kernel(A), kb = new Kernel(B);
    const body = JSON.stringify({ id: 1, msg });
    const ra = ka.request(body), rb = kb.request(body);
    const pa = ra.trap ? null : JSON.parse(ra.json), pb = rb.trap ? null : JSON.parse(rb.json);
    const ba = pa?.result?.body, bb = pb?.result?.body;
    const cnt = (b) => b?.edges?.map(({ pts, ...e }) => ({ ...e, pts: pts.length }));
    const rest = (p) => { const r = { ...(p?.result ?? {}) }; delete r.body; return JSON.stringify([p?.ok, p?.error, r]); };
    bump(`regen.${set}.faces`, !!pa && !!pb && JSON.stringify(ba?.faces) === JSON.stringify(bb?.faces));
    bump(`regen.${set}.edges`, !!pa && !!pb && JSON.stringify(cnt(ba)) === JSON.stringify(cnt(bb)));
    bump(`regen.${set}.box`, !!pa && !!pb && JSON.stringify(ba?.box) === JSON.stringify(bb?.box));
    bump(`regen.${set}.rest`, !!pa && !!pb && rest(pa) === rest(pb));
    bump(`regen.${set}.json`, !!pa && !!pb && norm(ra) === norm(rb));
    bump(`regen.${set}.mesh`, !!pa && !!pb && norm(ra) === norm(rb) && bin(ra) === bin(rb));
    if (ba && bb) {
      const q = JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } });
      const ma = JSON.parse(ka.request(q).json || 'null')?.result, mb = JSON.parse(kb.request(q).json || 'null')?.result;
      bump(`mass.${set}`, ['volume', 'area', 'com'].every((k) => close(mb?.[k], ma?.[k])));
    } else bump(`mass.${set}`, !ba && !bb);
    if (!quiet) process.stderr.write(`${t.name} ${((performance.now() - t0) / 1000).toFixed(1)}s\n`);
  }
}
regenSet('brep', Object.entries(brepParts).map(([name, part]) => ({ name, part })));
// --fast skips the sample parts (minutes while booleans are slow)
for (const f of fast ? [] : ['bracket', 'stepper-sample', 'coupling-sample', 'large-sample']) {
  const p = path.join(art, `${f}.ckpart`);
  if (!existsSync(p)) continue;
  const part = JSON.parse(readFileSync(p, 'utf8'));
  regenSet(f, part.features.map((_, k) => ({ name: `${f}@${k + 1}`, part, upTo: k + 1 })));
}

// 3. STEP export
const gdir = path.join(here, '../crates/ck-io/tests/golden');
if (existsSync(gdir)) for (const f of readdirSync(gdir).filter((f) => f.endsWith('.req.json')).sort()) {
  const req = readFileSync(path.join(gdir, f), 'utf8');
  const st = '{"id":2,"msg":{"type":"step","name":"t"}}';
  const [a, b] = [A, B].map((m) => { const k = new Kernel(m); k.request(req); return norm(k.request(st)); });
  bump('step', a === b);
}
if (!quiet) process.stderr.write('\n');

// report + compare with baseline
const basePath = path.join(here, 'scoreboard.json');
const base = existsSync(basePath) ? JSON.parse(readFileSync(basePath, 'utf8')) : {};
let worse = 0;
const skipped = (k) => fast && /^(regen|mass)\.(bracket|stepper|flange|hair)/.test(k);
for (const k of Object.keys({ ...base, ...score }).sort()) {
  const [m, n] = score[k] ?? [0, 0], b = base[k]?.[0];
  const d = b === undefined ? ' (new)' : m > b ? ` (+${m - b})` : m < b ? ` (-${b - m}) REGRESSION` : '';
  if (skipped(k)) continue;
  if (b !== undefined && m < b) worse++;
  console.log(`${k.padEnd(40)} ${String(m).padStart(4)}/${String(n).padEnd(4)}${d}`);
}
if (update) { writeFileSync(basePath, JSON.stringify(score, null, 1) + '\n'); console.log(`baseline written: ${basePath}`); }
else if (worse) { console.log(`\nGATE FAILED: ${worse} metric(s) regressed`); process.exitCode = 1; }
else console.log('\ngate ok');
