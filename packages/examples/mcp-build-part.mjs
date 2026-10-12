// Worked example: build a part from an empty document through the MCP server,
// the way a model would. Copy this pattern; do not import session code.
// run: npm --prefix packages run build && node packages/examples/mcp-build-part.mjs
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StdioClientTransport } from '@modelcontextprotocol/sdk/client/stdio.js';
import { fileURLToPath } from 'node:url';

const here = (p) => fileURLToPath(new URL(p, import.meta.url));
const transport = new StdioClientTransport({
  command: 'node',
  args: [here('../mcp.mjs')],
  // CK_WASM defaults to <repo>/target/wasm32-unknown-unknown/release/ck_wasm.wasm
  env: { ...process.env },
});
const client = new Client({ name: 'example', version: '1' });
await client.connect(transport);
const call = async (name, args = {}) => {
  const r = await client.callTool({ name, arguments: args });
  return JSON.parse(r.content[0].text);
};

await call('guide');                                  // read once: fields, frames and traps
const { session } = await call('open', { name: 'plate' });
const poly = (id, pts) => ({
  type: 'sketch', id, plane: { kind: 'std', id: 'front' },
  points: pts.map(([x, y], i) => ({ id: `p${i + 1}`, x, y })),
  entities: pts.map((_, i) => ({ id: `e${i + 1}`, type: 'line', p1: `p${i + 1}`, p2: `p${((i + 1) % pts.length) + 1}` })),
  constraints: [],
});
const edit = await call('edit', { session, ops: [
  { op: 'add', feature: poly('F1', [[0, 0], [60, 0], [60, 40], [0, 40]]) },
  { op: 'add', feature: { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 10 } },
] });
if (!edit.ok) throw new Error(`edit rejected: ${JSON.stringify(edit.hints ?? edit.schemaError)}`);  // ok:false is a normal reply
const report = await call('verify', { session, expect: [{ kind: 'volume', value: 24000, tol: 0.01 }, { kind: 'valid' }] });
console.log(report.ok ? 'verified' : JSON.stringify(report.hints));
await call('close', { session });
await client.close();
