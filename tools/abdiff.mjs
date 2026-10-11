// A/B reply differ: original kernel vs rebuilt, decoded buffers included.
// usage: node tools/abdiff.mjs <part.json> [name]
import { readFileSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const origWasm = realpathSync(path.join(here, '../../kernel.wasm'));
const rebuilt = path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const A = await WebAssembly.compile(readFileSync(origWasm));
const B = await WebAssembly.compile(readFileSync(rebuilt));

const T = { f32: Float32Array, f64: Float64Array, u32: Uint32Array, i32: Int32Array, u8: Uint8Array };
function decode(r) {
  const jl = new DataView(r.bytes.buffer, r.bytes.byteOffset).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const dec = (v) => {
    if (Array.isArray(v)) return v.map(dec);
    if (v && typeof v === 'object') {
      if ('$buf' in v) {
        const C = T[v.type];
        const s = r.bytes.buffer.slice(r.bytes.byteOffset + base + v.offset, r.bytes.byteOffset + base + v.offset + v.len * C.BYTES_PER_ELEMENT);
        return Array.from(new C(s));
      }
      const o = {}; for (const [k, x] of Object.entries(v)) o[k] = dec(x); return o;
    }
    return v;
  };
  return { json: JSON.parse(r.json), bin: dec(JSON.parse(r.json)) };
}

const inp = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const msg = inp.type ? inp : { type: 'regen', tag: 'main', part: inp };
const name = process.argv[3] ?? 'part';
const body = JSON.stringify({ id: 1, msg });
const ra = new Kernel(A).request(body);
const rb = new Kernel(B).request(body);
if (ra.trap || rb.trap) { console.log('TRAP', ra.trap, rb.trap); process.exit(1); }
const a = decode(ra), b = decode(rb);

const ba = a.bin.result.body, bb = b.bin.result.body;
// edges
if (JSON.stringify(ba?.edges) !== JSON.stringify(bb?.edges)) {
  const ea = ba?.edges ?? [], eb = bb?.edges ?? [];
  console.log(`edges: ${ea.length} vs ${eb.length}`);
  for (let i = 0; i < Math.max(ea.length, eb.length); i++) {
    const x = ea[i], y = eb[i];
    if (!x || !y) { console.log(` [${i}] missing side`); continue; }
    if (JSON.stringify(x) !== JSON.stringify(y)) {
      const pa = x.pts, pb = y.pts;
      const pmsg = JSON.stringify(pa) === JSON.stringify(pb) ? 'pts same' : `pts ${pa.length} vs ${pb.length} first-diff@${pa.findIndex((p, k) => JSON.stringify(p) !== JSON.stringify(pb[k]))}`;
      console.log(` [${i}] ${x.key} | ${pmsg} | other-same=${JSON.stringify({ ...x, pts: 0 }) === JSON.stringify({ ...y, pts: 0 })}`);
    }
  }
} else console.log('edges: identical');
// mesh
const meshA = ba?.mesh, meshB = bb?.mesh;
if (JSON.stringify(meshA) !== JSON.stringify(meshB)) {
  for (const k of ['positions', 'normals', 'mids', 'triFace']) {
    const x = meshA?.[k], y = meshB?.[k];
    if (!x || !y) { console.log(`mesh.${k}: missing`); continue; }
    let d = -1;
    for (let i = 0; i < Math.min(x.length, y.length); i++) if (x[i] !== y[i]) { d = i; break; }
    const same = JSON.stringify(x) === JSON.stringify(y);
    console.log(`mesh.${k}: len ${x.length} vs ${y.length}, firstDiff@${d}, identical=${same}`);
  }
} else console.log('mesh: identical');
if (JSON.stringify(a.bin.result.body?.faces) !== JSON.stringify(b.bin.result.body?.faces)) console.log('faces DIFFER');
const restA = JSON.stringify({ ...a.bin.result, body: undefined }), restB = JSON.stringify({ ...b.bin.result, body: undefined });
if (restA !== restB) console.log('rest DIFFERS');
