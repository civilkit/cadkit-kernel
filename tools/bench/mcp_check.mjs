// Build each BenchCAD batch-3 part through the MCP server the way an LLM user would (open an empty document,
// one `edit` batch of `add` ops from the translated feature list, then `check` volume/bbox against the
// CadQuery ground truth from manifest.json and `query validate`).
// usage: node tools/bench/mcp_check.mjs [id ...]      (needs `npm --prefix packages run build` once)
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Client } from '../../packages/node_modules/@modelcontextprotocol/sdk/dist/esm/client/index.js';
import { StdioClientTransport } from '../../packages/node_modules/@modelcontextprotocol/sdk/dist/esm/client/stdio.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const dir = path.join(here, 'benchcad');
const manifest = JSON.parse(readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
const ids = process.argv.slice(2);
const sel = manifest.parts.filter((m) => (ids.length ? ids.includes(m.id) : m.batch === '3a'));

const transport = new StdioClientTransport({ command: 'node', args: [path.join(here, '../../packages/mcp.mjs')], env: { ...process.env } });
const client = new Client({ name: 'bench-mcp', version: '1' });
await client.connect(transport);
const call = async (name, args = {}) => { const r = await client.callTool({ name, arguments: args }); try { return JSON.parse(r.content[0].text); } catch { return r.content[0].text; } };
const rot = (v) => [v[0], v[2], -v[1]];
for (const m of sel) {
  const part = JSON.parse(readFileSync(path.join(dir, `${m.id}.ckpart.json`), 'utf8'));
  const { session } = await call('open', { name: m.id });
  const edit = await call('edit', { session, ops: part.features.map((feature) => ({ op: 'add', feature })) });
  const g = m.ground_truth, lo = rot(g.bbox.slice(0, 3)), hi = rot(g.bbox.slice(3));
  const min = lo.map((x, i) => Math.min(x, hi[i])), max = lo.map((x, i) => Math.max(x, hi[i]));
  const out = { id: m.id, editOk: edit.ok, errors: edit.errors ?? edit.hints ?? null };
  const chk = await call('check', { session, expect: [{ kind: 'volume', value: g.volume, tol: 0.001 }, { kind: 'bbox', min, max, tol: 0.05 }, { kind: 'valid' }] });
  out.check = chk;
  console.log(JSON.stringify(out).slice(0, 1500));
  await call('close', { session });
}
await client.close();
