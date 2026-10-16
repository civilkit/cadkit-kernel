// Runs every SolidWorks part (.sldprt) under a directory through the wasm `importSldprt` request and prints, per file:
// faces, volume, area, bbox, valid (kernel validator), import ms, and the volume compared with a reference when one exists.
//
// usage: node tools/sldprt_check.mjs [--dir DIR] [--refs DIR] [--wasm PATH] [--filter SUBSTR] [--json] [--strict]
//   --dir     directory searched recursively for .sldprt files (default: the scratch sample set)
//   --refs    directory of reference STEPs named <sample-dir-name>/s2s.step (sldprt2step output), used when the sample
//             has no reference of its own (default: the scratch out dir)
//   --strict  exit 1 when a file with a reference does not match, or a file with a reference is invalid
//
// Reference volume, first that exists: a STEP next to the part (model.step, ref.step, <name>.step), else
// <refs>/<dir name>/s2s.step (sldprt2step), else the built-in table of sldprt2step volumes (rounded, so compared with 5e-4).
// Tolerance 1e-4 relative, 1e-3 when the part has B-spline surfaces (loft/spline) -- those names are listed in SPLINE_HEAVY.
import { readFileSync, readdirSync, statSync, existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (n, d) => { const i = args.indexOf(n); return i >= 0 ? args[i + 1] : d; };
const SCRATCH = '/path/to/work/scratch-sldprt';
const dir = opt('--dir', path.join(SCRATCH, 'samples'));
const refsDir = opt('--refs', path.join(SCRATCH, 'out'));
const wasmPath = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const filter = opt('--filter', '');
const asJson = args.includes('--json');
const strict = args.includes('--strict');

const { Kernel } = await import(path.join(here, 'kernel.mjs'));
const mod = await WebAssembly.compile(readFileSync(wasmPath));

// sldprt2step volumes (mm^3) from the spike results doc, files it converted; rounded to 4 significant digits
const TABLE = {
  drone_157: 18.11, drone_qfn: 36, SW2011_C03_cube_fillet_1mm: 997.9, SW2011_C04_cube_hole_5mm: 803.7, SW2011_C10_cube_shell_1mm: 424,
  SW2022_C00_cube_10mm: 1000, SW2022_C03_cube_fillet_1mm: 997.9, SW2022_C04_cube_hole_5mm: 803.7, SW2022_C10_cube_shell_1mm: 424,
  SW2022_C20_cylinders_crossed: 2475, SW2022_C21_cylinder_fillet: 779, nist_ctc_03: 3.314e5, nist_ctc_04: 1.754e7, nist_ftc_06: 3.287e6,
  nist_ftc_08: 5.036e5, nist_ftc_09: 1.364e5, nist_ftc_10: 1.879e5, nist_ftc_11: 5129, 'pend_small_motor_385': 2.356e4, pend_bearing_holder: 1.529e4,
  pend_encoder: 4.165e4, pend_pole: 3393, pend_table_mount: 5.261e4, 'user_M6-MV': 530.6, user_joint__L_90: 2.262e4, user_motor_stepper_40mm: 7.13e4,
};
// closed-form volumes where the kernel's own STEP import of SolidWorks' reference is known to be wrong (cone with an apex: half the volume)
const ANALYTIC = { SW2011_C13_cone: Math.PI * 25 * 10 / 3, SW2022_C13_cone: Math.PI * 25 * 10 / 3 };
const SPLINE_HEAVY = /loft|spline|small_motor|wheel|bolt|nist_ctc_0[1245]|nist_ftc_0[7-9]|nist_ftc_10/i;

function walk(d, out = []) {
  for (const n of readdirSync(d).sort()) {
    const p = path.join(d, n);
    if (statSync(p).isDirectory()) walk(p, out);
    else if (/\.sldprt$/i.test(n)) out.push(p);
  }
  return out;
}

function session() {
  const k = new Kernel(mod);
  let id = 0;
  const rq = (msg) => { const r = k.request(JSON.stringify({ id: ++id, msg })); return r.json ? JSON.parse(r.json) : { trap: r.trap }; };
  return rq;
}

function stepVolume(file) {
  const rq = session();
  const imp = rq({ type: 'importStep', tag: 'r', name: 'r', data: readFileSync(file, 'utf8') });
  if (!imp?.result?.body) return null;
  const m = rq({ type: 'massProps', tag: 'r' })?.result;
  const v = rq({ type: 'validate', tag: 'r' })?.result;
  return m ? { volume: m.volume, valid: v?.ok ?? null } : null;
}

const files = walk(dir).filter((f) => f.includes(filter));
const rows = [];
let bad = 0;
for (const f of files) {
  const dname = path.basename(path.dirname(f));
  const sname = dname === path.basename(dir) ? path.basename(f, path.extname(f)) : dname;
  const row = { name: sname };
  try {
    const rq = session();
    const data = readFileSync(f).toString('base64');
    const t0 = performance.now();
    const imp = rq({ type: 'importSldprt', tag: 'm', name: sname, data });
    row.ms = Math.round(performance.now() - t0);
    if (imp.trap) { row.error = 'TRAP ' + imp.trap; rows.push(row); bad++; continue; }
    const res = imp?.result;
    if (!imp.ok || !res?.body) {
      const e = res?.errors?.[0];
      row.error = e ? `${e[1]?.code ?? e.code}: ${e[1]?.params?.message ?? e.message ?? ''}`.slice(0, 160) : (imp.error ?? 'no body');
      rows.push(row);
      continue;
    }
    const b = res.body;
    row.faces = b.faces?.length;
    const bx = b.box;
    row.bbox = bx ? bx.max.map((x, i) => +(x - bx.min[i]).toFixed(3)) : null;
    const m = rq({ type: 'massProps', tag: 'm' })?.result;
    row.volume = m?.volume; row.area = m?.area;
    const v = rq({ type: 'validate', tag: 'm' })?.result;
    row.valid = v ? v.ok : null;
    row.issues = v && !v.ok ? [...new Set(v.issues.map((i) => i.code))].join(',') : '';
    row.approx = (res.import?.approximations ?? []).map((a) => a.maxErrorMm);
    row.container = res.import?.container;
    // reference
    const cands = [path.join(path.dirname(f), 'model.step'), path.join(path.dirname(f), 'ref.step'), f.replace(/\.sldprt$/i, '.step')];
    const own = cands.find((c) => existsSync(c));
    const s2s = path.join(refsDir, sname, 's2s.step');
    // expected volume: the sldprt2step-derived STEP when it imports valid, else the part's own reference STEP, else the table
    const sw = own ? stepVolume(own) : null;
    const s2 = existsSync(s2s) ? stepVolume(s2s) : null;
    // candidates in priority order; the first is the headline reference, MATCH when any valid-importing candidate is within tolerance
    const cand = [];
    if (ANALYTIC[sname] !== undefined) cand.push({ src: 'analytic', v: ANALYTIC[sname] });
    if (s2?.volume && s2.valid) cand.push({ src: 's2s', v: s2.volume });
    if (sw?.volume && sw.valid !== false) cand.push({ src: 'sw', v: sw.volume });
    if (!cand.length && s2?.volume) cand.push({ src: 's2s*', v: s2.volume });
    if (!cand.length && sw?.volume) cand.push({ src: 'sw*', v: sw.volume });
    if (!cand.length && TABLE[sname] !== undefined) cand.push({ src: 'table', v: TABLE[sname] });
    if (cand.length) {
      row.refs = cand.map((c) => ({ src: c.src, volume: c.v, rel: Math.abs(row.volume - c.v) / Math.abs(c.v) }));
      const best = row.refs.reduce((a, b) => (b.rel < a.rel ? b : a));
      row.refVolume = best.volume; row.refSrc = best.src; row.rel = best.rel;
      const tol = best.src === 'table' ? 5e-4 : SPLINE_HEAVY.test(sname) ? 1e-3 : 1e-4;
      row.match = row.rel <= tol;
      if (!row.match || row.valid === false) bad++;
    }
  } catch (e) { row.error = String(e).slice(0, 160); }
  rows.push(row);
}

if (asJson) console.log(JSON.stringify(rows, null, 1));
else {
  const f = (x, d = 4) => (x === undefined || x === null ? '-' : Math.abs(x) >= 1e5 || (Math.abs(x) < 1e-2 && x !== 0) ? x.toExponential(3) : (+x.toFixed(d)).toString());
  console.log(['file'.padEnd(32), 'faces', 'volume'.padStart(11), 'area'.padStart(11), 'bbox(mm)'.padEnd(28), 'valid', 'ms'.padStart(5), 'ref', 'rel'].join('  '));
  for (const r of rows) {
    if (r.error) { console.log(`${r.name.padEnd(32)}  FAIL  ${r.error}  (${r.ms ?? '-'} ms)`); continue; }
    console.log([
      r.name.padEnd(32), String(r.faces).padStart(5), f(r.volume).padStart(11), f(r.area).padStart(11), (r.bbox ?? []).join('x').padEnd(28),
      (r.valid === null ? '?' : r.valid ? 'ok' : 'INVALID(' + r.issues + ')').padEnd(5), String(r.ms).padStart(5),
      r.refVolume !== undefined ? `${r.refSrc}:${f(r.refVolume)}${r.refs?.length > 1 ? " (also " + r.refs.filter((x) => x.src !== r.refSrc).map((x) => x.src + ":" + f(x.volume)).join(" ") + ")" : ""}` : '-', r.rel !== undefined ? (r.match ? 'MATCH ' : 'DIFF  ') + r.rel.toExponential(2) : '',
      r.approx?.length ? `approx(max ${Math.max(...r.approx).toExponential(2)} mm, ${r.approx.length})` : '',
    ].join('  '));
  }
  const ok = rows.filter((r) => !r.error).length;
  const valid = rows.filter((r) => r.valid).length;
  const matched = rows.filter((r) => r.match).length, withRef = rows.filter((r) => r.refVolume !== undefined && !r.error).length;
  console.log(`\n${rows.length} files: ${ok} imported, ${valid} valid, ${matched}/${withRef} volume matches with a reference`);
}
if (strict && bad) process.exit(1);
