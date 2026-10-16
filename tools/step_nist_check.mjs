// Run a directory of STEP files (default: the NIST PMI SolidWorks reference files) through importStep, massProps and
// validate, and print a table with the OCCT reference volume / area from tools/step_nist_reference.json.
//   node tools/step_nist_check.mjs [--wasm path] [--tol 1e-4] [--tol-spline 1e-3] [dir-or-file ...]
// A directory is searched (one level, and one level below) for *.step / *.stp files; the reference key of a file is its
// base name, or the parent directory's name for files called ref.step. Exit code 1 when a file with a reference does not
// import valid within tolerance. Files without a reference are listed without a verdict.
import { readFileSync, readdirSync, statSync, existsSync } from 'node:fs';
import { join, basename, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (name, def) => { const i = args.indexOf(name); if (i < 0) return def; const v = args[i + 1]; args.splice(i, 2); return v; };
const wasmPath = opt('--wasm', join(here, '..', 'target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const tol = Number(opt('--tol', '1e-4'));
const tolSpline = Number(opt('--tol-spline', '1e-3'));
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
// the default folder also holds the other samples' STEP exports: keep the NIST ones unless paths were given
const list = args.length ? files : files.filter((f) => /nist_/.test(f));
const key = (f) => (/^ref\.step$/i.test(basename(f)) ? basename(dirname(f)) : basename(f).replace(/\.(step|stp)$/i, ''));

const mod = await WebAssembly.compile(readFileSync(wasmPath));
const rows = [];
let bad = 0;
for (const f of list) {
  const k = key(f);
  const r = { name: k };
  const kern = new Kernel(mod);
  const rq = (id, msg) => { const o = kern.request(JSON.stringify({ id, msg })); return o.json ? JSON.parse(o.json) : { trap: o.trap }; };
  const t0 = Date.now();
  const imp = rq(1, { type: 'importStep', tag: 'm', name: 'p', data: readFileSync(f, 'utf8') });
  r.ms = Date.now() - t0;
  const body = imp?.result?.body;
  const errs = imp?.result?.errors ?? [];
  if (!body) {
    const e = errs[0]?.[1];
    r.err = e ? `${e.code}: ${e.params?.message ?? ''}${e.params?.entity != null ? ` (#${e.params.entity})` : ''}` : (imp?.error ?? imp?.trap ?? 'no body');
  } else {
    r.faces = body.faces?.length;
    const m = rq(2, { type: 'massProps', tag: 'm' })?.result;
    r.vol = m?.volume; r.area = m?.area;
    const v = rq(3, { type: 'validate', tag: 'm' })?.result;
    r.valid = !!v?.ok; r.issues = v?.issues?.length ?? 0;
  }
  const o = ref[k];
  if (o) {
    r.refVol = o.volume; r.refArea = o.area; r.refFaces = o.faces;
    if (r.vol != null) { r.dv = Math.abs(r.vol - o.volume) / o.volume; r.da = Math.abs(r.area - o.area) / o.area; }
    // B-spline heavy files get the looser tolerance
    const t = (o.splineHeavy ? tolSpline : tol);
    r.ok = !!(r.valid && r.dv <= t && r.da <= t && r.faces === o.faces);
    if (!r.ok) bad++;
  }
  rows.push(r);
}

const f = (x, d = 0) => (x == null ? '-' : x.toFixed(d));
const e = (x) => (x == null ? '-' : x.toExponential(1));
console.log(['file'.padEnd(13), 'ms'.padStart(6), 'faces'.padStart(6), 'valid'.padStart(6), 'volume'.padStart(14), 'OCCT volume'.padStart(14), 'dV'.padStart(8), 'dA'.padStart(8), ' verdict'].join(' '));
for (const r of rows) {
  const verdict = r.err ? `IMPORT ERROR ${r.err}` : r.ok === undefined ? '(no reference)' : r.ok ? 'ok' : `FAIL${r.faces !== r.refFaces ? ` faces ${r.faces}/${r.refFaces}` : ''}${r.valid ? '' : ` ${r.issues} issues`}`;
  console.log([r.name.padEnd(13), String(r.ms).padStart(6), String(r.faces ?? '-').padStart(6), String(r.valid ?? '-').padStart(6), f(r.vol, 1).padStart(14), f(r.refVol, 1).padStart(14), e(r.dv).padStart(8), e(r.da).padStart(8), ' ' + verdict].join(' '));
}
console.log(`${rows.length} files, ${rows.filter((r) => r.ok).length} ok, ${bad} failing against a reference`);
process.exit(bad ? 1 : 0);
