// Native service vs wasm: every wire case of tools/cases.mjs plus the large sample and the fan
// shroud regens, sent through ck-native over HTTP and compared with the wasm replies
// (ms / at normalised, mesh binary compared separately, exactly like the wire gate).
//
// usage: node tools/native_diff.mjs [--bin target/service/ck-native] [--wasm ck_wasm.wasm]
//          [--no-cases] [--no-big] [--time] [--verbose]
//   Starts the binary on a free port with a throw-away config dir and token, shuts it down at the end.
//   --time also reports CPU seconds of native (server process) vs wasm (this process) for the two
//   big regens (cache off), the large sample_times.mjs method.
// Exit 0 only if everything matches.
import { readFileSync, mkdtempSync, rmSync, existsSync } from 'node:fs';
import { spawn, execFileSync } from 'node:child_process';
import http from 'node:http';
import net from 'node:net';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import cases from './cases.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const flag = (f) => { const i = args.indexOf(f); if (i < 0) return false; args.splice(i, 1); return true; };
const opt = (f, d) => { const i = args.indexOf(f); if (i < 0) return d; const v = args[i + 1]; args.splice(i, 2); return v; };
const noCases = flag('--no-cases'), noBig = flag('--no-big'), doTime = flag('--time'), verbose = flag('--verbose');
const bin = opt('--bin', path.join(here, '../target/service/ck-native'));
const wasmPath = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const mod = new WebAssembly.Module(readFileSync(wasmPath));

// ---- normalisation (same as tools/gate.mjs)
const frame = (bytes) => {
  const b = Buffer.from(bytes), jl = b.readUInt32LE(0);
  return { json: b.subarray(4, 4 + jl).toString('utf8'), bin: b.subarray((4 + jl + 7) & -8).toString('base64') };
};
const nj = (j) => j.replace(/"ms":[0-9.e+-]+/g, '"ms":0').replace(/"at":[0-9.e+-]+/g, '"at":0').replace(/,"build":"[0-9a-f]+(-dirty)?"/g, '');
const key = (f) => nj(f.json) + '\u0000' + f.bin;
const wasmKey = (r) => (r.trap ? `TRAP ${r.trap}` : key(frame(r.bytes)));

// ---- server
const dir = mkdtempSync(path.join(os.tmpdir(), 'fwnative-'));
const free = await new Promise((res) => { const s = net.createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => res(p)); }); });
if (!existsSync(bin)) { console.error(`no binary at ${bin} (cargo build --profile service -p ck-native)`); process.exit(2); }
// create the token first (the server and --print-token would race to create it)
const token = execFileSync(bin, ['--config-dir', dir, '--print-token']).toString().trim();
const srv = spawn(bin, ['--port', String(free), '--config-dir', dir, '--quiet', '--max-sessions', '64'], { stdio: ['ignore', 'ignore', 'inherit'] });
const agent = new http.Agent({ keepAlive: true, maxSockets: 4 });
const call = (method, p, body) => new Promise((res, rej) => {
  const r = http.request({ host: '127.0.0.1', port: free, method, path: p, agent, headers: { Authorization: `Bearer ${token}`, ...(body ? { 'Content-Length': Buffer.byteLength(body) } : {}) } }, (rs) => {
    const ch = []; rs.on('data', (c) => ch.push(c)); rs.on('end', () => res({ status: rs.statusCode, body: Buffer.concat(ch) }));
  });
  r.on('error', rej); r.end(body);
});
for (let i = 0; ; i++) { try { await call('GET', '/version'); break; } catch { if (i > 100) throw new Error('server did not start'); await new Promise((r) => setTimeout(r, 100)); } }
const version = JSON.parse((await call('GET', '/version')).body.toString());
console.log(`server: ${JSON.stringify(version)}`);
const open = async () => JSON.parse((await call('POST', '/session', '')).body.toString()).session;
const close = (id) => call('DELETE', `/session/${id}`);
const rpc = async (id, body) => {
  const r = await call('POST', `/session/${id}/rpc`, body);
  return r.status === 200 ? { ok: true, bytes: r.body } : { ok: false, status: r.status, text: r.body.toString() };
};

// Structural comparison with a relative tolerance: the two builds use different libm
// implementations (system libm vs the one compiled into the wasm), so big documents can differ in
// the last digit of a float without any logic difference.
function deepDiff(a, b, st = { n: 0, max: 0, struct: 0, where: [] }, at = '') {
  const sd = () => { st.struct++; if (st.where.length < 5) st.where.push(at); };
  if (typeof a === 'number' && typeof b === 'number') {
    if (a !== b) { st.n++; st.max = Math.max(st.max, Math.abs(a - b) / (1e-12 + Math.max(Math.abs(a), Math.abs(b)))); }
  } else if (Array.isArray(a) && Array.isArray(b)) {
    if (a.length !== b.length) sd(); else for (let i = 0; i < a.length; i++) deepDiff(a[i], b[i], st, at + '/' + i);
  } else if (a && b && typeof a === 'object' && typeof b === 'object') {
    const ka = Object.keys(a), kb = Object.keys(b);
    if (ka.join() !== kb.join()) sd(); else for (const k of ka) deepDiff(a[k], b[k], st, at + '/' + k);
  } else if (a !== b) sd();
  return st;
}

let fail = 0;
const same_close = []; let continue_ok = false;
try {
  // ---- 1. wire cases
  if (!noCases) {
    let ok = 0, total = 0; const bad = [];
    for (const c of cases) {
      const k = new Kernel(mod), id = await open();
      for (const [i, body] of c.bodies.entries()) {
        const w = k.request(body), n = await rpc(id, body);
        if (c.ignore?.includes(i)) continue;
        total++;
        const same = w.trap ? !n.ok : n.ok && wasmKey(w) === key(frame(n.bytes));
        if (same) ok++; else bad.push(`${c.name}#${i}${w.trap ? ' (wasm trap, native ' + (n.ok ? 'reply' : n.status) + ')' : ''}`);
      }
      await close(id);
    }
    console.log(`wire cases: ${ok}/${total} byte-identical (ms normalised)`);
    for (const b of bad.slice(0, 30)) console.log('  DIFF', b);
    if (ok !== total) fail++;
  }
  // ---- 2. big documents
  if (!noBig) {
    const docs = [['large sample', 'cases/largeSampleFull.ckpart'], ['fan shroud', 'cases/perf/fan_shroud.ckpart']];
    const nativeCpu = () => {
      if (existsSync(`/proc/${srv.pid}/stat`)) { const f = readFileSync(`/proc/${srv.pid}/stat`, 'utf8').split(') ')[1].split(' '); return (+f[11] + +f[12]) / 100; }
      const t = execFileSync('ps', ['-o', 'cputime=', '-p', String(srv.pid)]).toString().trim(); // [[h:]m:]s.cc
      return t.split(':').reduce((a, x) => a * 60 + parseFloat(x), 0);
    };
    const wcpu = () => { const u = process.cpuUsage(); return (u.user + u.system) / 1e6; };
    for (const [name, file] of docs) {
      const part = JSON.parse(readFileSync(path.join(here, file), 'utf8'));
      const body = JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, ...(doTime ? { opts: { noCache: true } } : {}) } });
      const k = new Kernel(mod, { now: () => performance.now(), unixMs: () => Date.now() });
      const c0 = wcpu(), w = k.request(body), wt = wcpu() - c0;
      const id = await open();
      const n0 = nativeCpu(), t0 = performance.now(), n = await rpc(id, body), nwall = (performance.now() - t0) / 1000, nt = nativeCpu() - n0;
      if (verbose && n.ok && n.bytes.length < 2000) console.log(frame(n.bytes).json);
      const same = !w.trap && n.ok && wasmKey(w) === key(frame(n.bytes));
      console.log(`${name}: ${same ? 'IDENTICAL' : 'DIFFERENT'} (${n.ok ? n.bytes.length : n.status} bytes native, ${w.total} wasm)` +
        (doTime ? `; cpu s: native ${nt.toFixed(1)} (wall ${nwall.toFixed(1)}), wasm ${wt.toFixed(1)}, ratio ${(wt / nt).toFixed(2)}x` : ''));
      if (!same && n.ok && !w.trap) {
        const fa = frame(n.bytes), fw = frame(w.bytes);
        const ja = JSON.parse(nj(fa.json)), jw = JSON.parse(nj(fw.json));
        // display polylines (edge pts) and the mesh are adaptively sampled: one ulp can move a sample count
        const ptsN = (j) => (j.result?.body?.edges ?? []).map((e) => e.pts?.length ?? 0);
        const pa = ptsN(ja), pw = ptsN(jw), ptsDiff = pa.filter((x, i) => x !== pw[i]).length;
        for (const j of [ja, jw]) for (const e of j.result?.body?.edges ?? []) { delete e.pts; for (const k of Object.keys(e)) if (Array.isArray(e[k]) && /^(t|ts|params?)$/.test(k)) delete e[k]; }
        if (verbose) { const ea = ja.result?.body?.edges ?? [], ew = jw.result?.body?.edges ?? []; console.log(`  edges native ${ea.length} wasm ${ew.length}; faces ${ja.result?.body?.faces?.length} / ${jw.result?.body?.faces?.length}`); const ka = new Set(ea.map((e) => e.key)), kw = new Set(ew.map((e) => e.key)); console.log('  only native:', JSON.stringify([...ka].filter((x) => !kw.has(x)).slice(0, 5)), 'only wasm:', JSON.stringify([...kw].filter((x) => !ka.has(x)).slice(0, 5))); }
        const st = deepDiff(ja, jw);
        const binSame = fa.bin === fw.bin;
        const mp = async () => {
          const q = JSON.stringify({ id: 9, msg: { type: 'massProps', tag: 'main' } });
          const a = JSON.parse(frame((await rpc(id, q)).bytes).json).result, b = JSON.parse(frame(k.request(q).bytes).json).result;
          return deepDiff(a, b);
        };
        const m = await mp();
        const close = st.struct === 0 && st.max <= 1e-6 && m.struct === 0 && m.max <= 1e-9;
        console.log(`  without edge display points: ${st.n} floats differ, max relative ${st.max.toExponential(1)}, ${st.struct} structural differences${st.where.length ? ' at ' + st.where.join(', ') : ''}; ${ptsDiff} edges have a different display sample count; massProps max relative ${m.max.toExponential(1)}; mesh binary ${binSame ? 'identical' : 'differs'} => ${close ? 'EQUIVALENT (topology, names, numbers within 1e-6; sampling differs)' : 'REAL DIFFERENCE'}`);
        if (close) { same_close.push(name); continue_ok = true; }
      }
      if (!same) {
        if (!continue_ok) fail++;
        continue_ok = false;
        if (verbose && n.ok) { const a = nj(frame(n.bytes).json), b = nj(frame(w.bytes).json); let i = 0; while (i < a.length && a[i] === b[i]) i++; console.log('  first json diff at', i, a.slice(i - 60, i + 60), '|', b.slice(i - 60, i + 60)); }
      }
      await close(id);
    }
  }
} finally {
  srv.kill();
  agent.destroy();
  rmSync(dir, { recursive: true, force: true });
}
process.exit(fail ? 1 : 0);
