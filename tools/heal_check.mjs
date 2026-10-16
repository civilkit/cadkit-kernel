// Import the SolidWorks sample STEP files (the NIST PMI reference files and the other samples' STEP exports) with
// healing off and on, and print a table: valid before / after, volume against the OCCT reference
// (tools/step_nist_reference.json), the tolerance the healer had to use, and the cpu time of the import.
//   node tools/heal_check.mjs [--wasm path] [--vol-tol 1e-3] [--max-ms 10000] [--no-off] [dir-or-file ...]
// Directories are searched one level down for *.step / *.stp. Exit code 1 when a file with a reference is invalid or
// off its volume after healing, when healing changed a body that was already valid, or when an import exceeds
// --max-ms (cpu).
import { readFileSync, readdirSync, statSync, existsSync } from 'node:fs';
import { join, basename, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const flag = (name) => { const i = args.indexOf(name); if (i < 0) return false; args.splice(i, 1); return true; };
const opt = (name, def) => { const i = args.indexOf(name); if (i < 0) return def; const v = args[i + 1]; args.splice(i, 2); return v; };
const wasmPath = opt('--wasm', join(here, '..', 'target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const volTol = Number(opt('--vol-tol', '1e-3'));
const maxMs = Number(opt('--max-ms', '10000'));
const noOff = flag('--no-off');
const DEFAULT_DIR = '/path/to/work/scratch-sldprt/samples';
const ref = JSON.parse(readFileSync(join(here, 'step_nist_reference.json'), 'utf8')).files;

function collect(p, depth = 0) {
  const st = statSync(p);
  if (st.isFile()) return /\.(step|stp)$/i.test(p) ? [p] : [];
  if (depth > 1) return [];
  return readdirSync(p).sort().flatMap((f) => collect(join(p, f), depth + 1));
}
const roots = args.length ? args : [DEFAULT_DIR];
const files = roots.flatMap((r) => (existsSync(r) ? collect(r) : (console.error(`no such path: ${r}`), [])));
const key = (f) => (/^(ref|model|part)\.(step|stp)$/i.test(basename(f)) ? basename(dirname(f)) : basename(f).replace(/\.(step|stp)$/i, ''));
const mod = await WebAssembly.compile(readFileSync(wasmPath));

function run(text, heal) {
  const kern = new Kernel(mod);
  const rq = (id, msg) => { const o = kern.request(JSON.stringify({ id, msg })); return o.json ? JSON.parse(o.json) : { trap: o.trap }; };
  const c0 = process.cpuUsage();
  const imp = rq(1, { type: 'importStep', tag: 'm', name: 'p', data: text, ...(heal ? {} : { heal: 'off' }) });
  const cpu = (() => { const c = process.cpuUsage(c0); return Math.round((c.user + c.system) / 1000); })();
  const r = { cpu };
  if (!imp?.result?.body) {
    const e = imp?.result?.errors?.[0]?.[1];
    r.err = e ? `${e.code}: ${e.params?.message ?? ''}` : (imp?.error ?? imp?.trap ?? 'no body');
    return r;
  }
  r.faces = imp.result.body.faces?.length;
  r.heal = imp.result.import?.heal;
  r.vol = rq(2, { type: 'massProps', tag: 'm' })?.result?.volume;
  const v = rq(3, { type: 'validate', tag: 'm' })?.result;
  r.valid = !!v?.ok;
  r.issues = (v?.issues ?? []).map((i) => i.code);
  r.tolUsed = v?.toleranceUsed ?? 0;
  return r;
}

const e = (x) => (x == null ? '-' : x === 0 ? '0' : x.toExponential(1));
const rows = [];
let bad = 0;
for (const f of files) {
  const k = key(f);
  const text = readFileSync(f, 'utf8');
  const off = noOff ? null : run(text, false);
  const on = run(text, true);
  const o = ref[k];
  const dv = (r) => (r?.vol != null && o ? Math.abs(r.vol - o.volume) / o.volume : null);
  const row = { k, off, on, dvOff: dv(off), dvOn: dv(on) };
  const problems = [];
  if (on.err) problems.push(`import error ${on.err}`);
  else {
    if (o && !on.valid) problems.push(`invalid after heal (${[...new Set(on.issues)].join(',')})`);
    if (o && !(row.dvOn <= volTol)) problems.push(`volume off ${e(row.dvOn)}`);
    if (on.cpu > maxMs) problems.push(`import ${on.cpu} ms > ${maxMs}`);
    // healing must not change a body that was valid without it
    if (off?.valid && off.vol != null && Math.abs(on.vol - off.vol) > 1e-12 * Math.abs(off.vol)) problems.push('healing changed a valid body');
    if (off?.valid && on.tolUsed > 0) problems.push('healing gave a valid body tolerances');
  }
  row.problems = problems;
  if (problems.length) bad++;
  rows.push(row);
}

console.log(['file'.padEnd(34), 'valid off'.padStart(9), 'valid on'.padStart(9), 'dV off'.padStart(8), 'dV on'.padStart(8), 'tolUsed'.padStart(8), 'cpu off'.padStart(8), 'cpu on'.padStart(8), ' verdict'].join(' '));
for (const r of rows) {
  const { off, on } = r;
  console.log([
    r.k.padEnd(34), String(off?.valid ?? '-').padStart(9), String(on.err ? 'ERR' : on.valid).padStart(9), e(r.dvOff).padStart(8), e(r.dvOn).padStart(8),
    e(on.tolUsed).padStart(8), String(off?.cpu ?? '-').padStart(8), String(on.cpu).padStart(8), ' ' + (r.problems.length ? 'FAIL ' + r.problems.join('; ') : (ref[r.k] ? 'ok' : '(no reference)')),
  ].join(' '));
}
const validOff = rows.filter((r) => r.off?.valid).length, validOn = rows.filter((r) => r.on.valid).length;
console.log(`${rows.length} files, valid ${validOff} -> ${validOn}, ${bad} problem(s)`);
process.exit(bad ? 1 : 0);
