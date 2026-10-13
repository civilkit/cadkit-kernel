// Build translated bench parts through the MCP server the way an LLM user would (open an empty document, one
// `edit` batch of `add` ops from the translated feature list, then `check` volume/area/bbox/centre of mass and
// `valid` against the reference from the set's manifest.json).
// usage: node tools/bench/mcp_check.mjs [--set benchcad|brepjs] [id ...]      (needs `npm --prefix packages run build` once)
//   benchcad (default): batch-3a parts, volume + bbox + valid (the original check)
//   brepjs: every part of tools/bench/brepjs, relative tolerance 1e-3 on volume and area, 1e-3 of the diagonal on
//           bbox and centre of mass; prints one summary line per part
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Client } from '../../packages/node_modules/@modelcontextprotocol/sdk/dist/esm/client/index.js';
import { StdioClientTransport } from '../../packages/node_modules/@modelcontextprotocol/sdk/dist/esm/client/stdio.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const si = args.indexOf('--set');
const set = si >= 0 ? args.splice(si, 2)[1] : 'benchcad';
const dir = path.join(here, set);
const manifest = JSON.parse(readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
const ids = args;
const sel = manifest.parts.filter((m) => (ids.length ? ids.includes(m.id) : set !== 'benchcad' || m.batch === '3a'));

const transport = new StdioClientTransport({ command: 'node', args: [path.join(here, '../../packages/mcp.mjs')], env: { ...process.env } });
const client = new Client({ name: 'bench-mcp', version: '1' });
await client.connect(transport);
const call = async (name, a = {}) => { const r = await client.callTool({ name, arguments: a }); try { return JSON.parse(r.content[0].text); } catch { return r.content[0].text; } };
const rot = (v) => [v[0], v[2], -v[1]];
for (const m of sel) {
  const part = JSON.parse(readFileSync(path.join(dir, `${m.id}.ckpart.json`), 'utf8'));
  const { session } = await call('open', { name: m.id });
  const t0 = Date.now();
  const edit = await call('edit', { session, ops: part.features.map((feature) => ({ op: 'add', feature })) });
  const g = m.ground_truth, lo = rot(g.bbox.slice(0, 3)), hi = rot(g.bbox.slice(3));
  const min = lo.map((x, i) => Math.min(x, hi[i])), max = lo.map((x, i) => Math.max(x, hi[i]));
  const diag = Math.hypot(...g.bbox.slice(3).map((x, i) => x - g.bbox[i]));
  if (set === 'benchcad') {
    const out = { id: m.id, editOk: edit.ok, errors: edit.errors ?? edit.hints ?? null };
    out.check = await call('check', { session, expect: [{ kind: 'volume', value: g.volume, tol: 0.001 }, { kind: 'bbox', min, max, tol: 0.05 }, { kind: 'valid' }] });
    console.log(JSON.stringify(out).slice(0, 1500));
  } else {
    const chk = await call('check', { session, expect: [
      { kind: 'volume', value: g.volume, tol: 1e-3 * g.volume }, { kind: 'area', value: g.area, tol: 1e-3 * g.area },
      { kind: 'bbox', min, max, tol: 1e-3 * diag }, { kind: 'com', at: rot(g.com), tol: 1e-3 * diag }, { kind: 'valid' }] });
    const r = chk.results ?? chk.result?.results ?? [];
    const pf = (k) => { const x = r.find((e) => e.kind === k); return x ? (x.pass ? 'ok' : 'FAIL') : '?'; };
    const vol = r.find((e) => e.kind === 'volume')?.measured;
    const errs = edit.ok ? '' : ` edit-not-ok: ${JSON.stringify(edit.errors ?? edit.hints ?? edit.schemaError).slice(0, 160)}`;
    console.log(`${m.id.padEnd(32)} edit ${edit.ok ? 'ok' : 'REJECTED'} volume ${pf('volume')} (${vol === undefined ? 'n/a' : Number(vol).toPrecision(6)} vs ${g.volume.toPrecision(6)}) area ${pf('area')} bbox ${pf('bbox')} com ${pf('com')} valid ${pf('valid')} ${Date.now() - t0} ms${errs}`);
    if (edit.ok === undefined || (edit.warnings ?? []).length) console.log('   edit reply extras:', JSON.stringify({ ok: edit.ok, warnings: edit.warnings }).slice(0, 300));
  }
  await call('close', { session });
}
await client.close();
