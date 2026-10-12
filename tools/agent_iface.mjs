// Self-consistency of the agent-interface queries (check / resolve / describe /
// refDiff, docs/agent-queries.md) over every brep case that regenerates a body.
//
// usage: node tools/agent_iface.mjs [--wasm path] [--quiet]
// Per part (OURS alone, SOFT gate metrics iface.*):
//   iface.describe  every face id and edge key of the regen reply describes (found, text)
//   iface.resolve   resolving each face id / edge key by `name` is unique (or flagged ambiguous
//                   exactly when the id is duplicated)
//   iface.check     check{volume, area, faces, edges, solids, bbox} against the regen/massProps
//                   values passes, and a deliberately wrong volume fails
//   iface.refdiff   refDiff(part, part) reports no warning (identity); edits are covered by
//                   crates/ck-wasm/tests/agent_iface.rs and tools/naming_stability.mjs
// The gate runs every 4th part (--stride 4); standalone runs every part.
import { readFileSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import { parts } from './brep_cases.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));

export function runIface({ wasm, stride = 1 }) {
  const mod = new WebAssembly.Module(readFileSync(wasm));
  const out = [];
  for (const [name, part] of Object.entries(parts).filter((_, i) => i % stride === 0)) {
    const k = new Kernel(mod);
    const q = (msg) => {
      const r = k.request(JSON.stringify({ id: 1, msg }));
      if (r.trap) return { trap: r.trap };
      try { return JSON.parse(r.json); } catch { return { trap: 'badjson' }; }
    };
    const t0 = performance.now();
    const rg = q({ type: 'regen', tag: 't', part });
    const slow = performance.now() - t0 > 300; // refDiff regenerates twice more: skip slow parts
    const body = rg.result?.body;
    if (!body || rg.result.errors?.length) continue;
    const res = (m) => { const r = q({ tag: 't', ...m }); return r.trap ? { trap: r.trap } : r.ok ? r.result : { error: r.error }; };
    const fail = {};
    const note = (k2, why) => { fail[k2] ??= why; };

    const faceIds = body.faces.map((f) => f.id);
    const edgeKeys = body.edges.map((e) => e.key);
    const dupFace = new Set(faceIds.filter((f, i) => faceIds.indexOf(f) !== i));
    const d = res({ type: 'describe', names: [...faceIds, ...edgeKeys] });
    if (d.trap || d.error) note('describe', d.trap ?? d.error);
    else d.items.forEach((it, i) => { if (!it.found || !it.text) note('describe', `${[...faceIds, ...edgeKeys][i]} not described`); });

    const descs = [...faceIds.map((id) => ({ kind: 'face', name: id })), ...edgeKeys.map((key) => ({ kind: 'edge', name: key }))];
    const rs = res({ type: 'resolve', descriptions: descs });
    if (rs.trap || rs.error) note('resolve', rs.trap ?? rs.error);
    else rs.results.forEach((r, i) => {
      const want = descs[i].kind === 'face' && dupFace.has(descs[i].name) ? 'ambiguous' : 'unique';
      if (r.status !== want) note('resolve', `${descs[i].name}: ${r.status} (${r.count}) want ${want}`);
    });

    const mp = res({ type: 'massProps' });
    const diag = Math.hypot(...body.box.max.map((x, i) => x - body.box.min[i]));
    const exp = [
      { kind: 'volume', equals: mp.volume }, { kind: 'area', equals: mp.area },
      { kind: 'faces', equals: body.faces.length },
      { kind: 'bbox', min: body.box.min, max: body.box.max, tol: 1e-2 * diag },
      { kind: 'volume', equals: mp.volume * 1.5 + 1 },
    ];
    const c = res({ type: 'check', expect: exp });
    if (c.trap || c.error) note('check', c.trap ?? c.error);
    else {
      const bad = c.results.map((r, i) => (r.pass === (i < 4) ? null : `${r.kind}[${i}] pass=${r.pass} ${r.detail}`)).filter(Boolean);
      if (bad.length) note('check', bad.join(' | '));
    }

    const rd = slow ? { ok: true, result: { warnings: [] } } : q({ type: 'refDiff', base: part, part });
    if (rd.trap || !rd.ok) note('refdiff', rd.trap ?? rd.error);
    else if (rd.result.warnings.length) note('refdiff', JSON.stringify(rd.result.warnings[0]).slice(0, 160));

    for (const key of ['describe', 'resolve', 'check', 'refdiff']) out.push({ name, key, ok: !(key in fail), why: fail[key] });
  }
  return out;
}

if (process.argv[1] && realpathSync(process.argv[1]) === realpathSync(fileURLToPath(import.meta.url))) {
  const a = process.argv.slice(2);
  const wi = a.indexOf('--wasm');
  const wasm = wi >= 0 ? a[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
  const si = a.indexOf('--stride');
  const rows = runIface({ wasm, stride: si >= 0 ? +a[si + 1] : 1 });
  const tally = {};
  for (const r of rows) { tally[r.key] ??= [0, 0]; tally[r.key][1]++; if (r.ok) tally[r.key][0]++; else if (!a.includes('--quiet')) console.log(`FAIL ${r.key} ${r.name}: ${r.why}`); }
  for (const [k, [m, n]] of Object.entries(tally)) console.log(`iface.${k} ${m}/${n}`);
}
