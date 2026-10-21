// Compare the CDT operation trace of the ORIGINAL kernel (instrumented by tools/cdt_trace.py) with the rebuilt wasm built with
// `--features ck-tess/trace`, face by face. Prints the first diverging operation of every face block.
//
// usage: node tools/cdtcmp.mjs <orig_trace.wasm> <ours_trace.wasm> <part.json> [--all]
import { readFileSync } from 'node:fs';

async function trace(wasm, partFile) {
  const mod = await WebAssembly.compile(readFileSync(wasm));
  let inst; const logs = [];
  const env = { ck_now: () => 0, ck_unix_ms: () => 0, ck_panic: (p, l) => console.log('PANIC', new TextDecoder().decode(new Uint8Array(inst.memory.buffer, p, l))),
    ck_log: (k, a, b, c, d) => logs.push([k, a, b, c, d]) };
  inst = new WebAssembly.Instance(mod, { env }).exports;
  const part = JSON.parse(readFileSync(partFile, 'utf8'));
  const body = new TextEncoder().encode(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'm', part } }).replace(/"format"\s*:\s*"ck-part"/g, '"format":"ck-part"'));   // the original kernel.wasm only accepts the format's old name
  const p = inst.ck_alloc(body.length); new Uint8Array(inst.memory.buffer, p, body.length).set(body);
  inst.ck_request(p, body.length);
  return logs;
}
// which columns of each id are meaningful (the rest are pointers / padding)
const COLS = { 1: [2, 3], 2: [2, 3, 4], 3: [3, 4], 4: [3, 4], 6: [2, 3], 9: [] };
const norm = (l) => (COLS[l[0]] ? `${l[0]}:` + COLS[l[0]].map((c) => l[c]).join(',') : null);
function blocks(logs) {
  const out = []; let cur = null;
  for (const l of logs) {
    if (l[0] === 9) { cur = []; out.push(cur); }
    const n = norm(l);
    if (cur && n) cur.push(n);
  }
  return out;
}
const [a, b, part] = process.argv.slice(2);
const all = process.argv.includes('--all');
const [A, B] = [blocks(await trace(a, part)), blocks(await trace(b, part))];
console.log('face blocks', A.length, B.length);
if (process.argv.includes('--dump')) { for (let f = 0; f < A.length; f++) { console.log('face', f, 'orig', A[f].filter((x) => x.startsWith('4:')).slice(0, 3).join(' '), '\n       ours', B[f].filter((x) => x.startsWith('4:')).slice(0, 3).join(' ')); } }
let bad = 0;
for (let f = 0; f < Math.min(A.length, B.length); f++) {
  const x = A[f], y = B[f];
  let i = 0; while (i < x.length && i < y.length && x[i] === y[i]) i++;
  if (i === x.length && i === y.length) { if (all) console.log(`face ${f}: same (${x.length} ops)`); continue; }
  bad++;
  console.log(`face ${f}: diverge at op ${i}/${x.length},${y.length}`);
  console.log('   orig:', x.slice(Math.max(0, i - 2), i + 4).join(' | '));
  console.log('   ours:', y.slice(Math.max(0, i - 2), i + 4).join(' | '));
}
console.log('differing faces', bad);
