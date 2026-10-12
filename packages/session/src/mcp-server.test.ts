import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { InMemoryTransport } from '@modelcontextprotocol/sdk/inMemory.js';
import { createMcpServer } from './mcp-server.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../../..');

function textOf(call: unknown): unknown {
  const c = (call as { content?: Array<{ type?: string; text?: string }> }).content?.[0];
  if (!c || c.type !== 'text' || typeof c.text !== 'string') throw new Error('tool result had no text content');
  return JSON.parse(c.text);
}

async function linked(): Promise<Client> {
  const { server } = createMcpServer();
  const [ct, st] = InMemoryTransport.createLinkedPair();
  const client = new Client({ name: 't', version: '0' });
  await Promise.all([server.connect(st), client.connect(ct)]);
  return client;
}

describe('mcp server', () => {
  it('builds the plate through tool calls and verifies it', async () => {
    const client = await linked();
    const tools = await client.listTools();
    assert.ok(tools.tools.some((t) => t.name === 'edit'));
    assert.ok(tools.tools.some((t) => t.name === 'verify'));

    const opened = textOf(await client.callTool({ name: 'open', arguments: { name: 'Plate' } })) as { session: string };
    const s = opened.session;
    const edit = textOf(await client.callTool({
      name: 'edit',
      arguments: {
        session: s,
        reason: 'plate with holes',
        ops: [
          { op: 'add', feature: { type: 'sketch', name: 'Outline', plane: { kind: 'std', id: 'front' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'p1', x: 0, y: 0 }, { id: 'p2', x: 100, y: 0 }, { id: 'p3', x: 100, y: 60 }, { id: 'p4', x: 0, y: 60 }], entities: [{ id: 'e5', type: 'line', p1: 'p1', p2: 'p2' }, { id: 'e6', type: 'line', p1: 'p2', p2: 'p3' }, { id: 'e7', type: 'line', p1: 'p3', p2: 'p4' }, { id: 'e8', type: 'line', p1: 'p4', p2: 'p1' }], constraints: [] } },
          { op: 'add', feature: { type: 'extrude', op: 'add', sketch: 'F1', end: 'blind', depth: 10 } },
        ],
      },
    })) as { ok: boolean; diff: { volume: { after: number } } };
    assert.equal(edit.ok, true);
    assert.ok(Math.abs(edit.diff.volume.after - 60000) < 1);

    const v = textOf(await client.callTool({ name: 'verify', arguments: { session: s, expect: [{ kind: 'volume', min: 1 }] } })) as { ok: boolean };
    assert.equal(v.ok, true);

    const guide = await client.readResource({ uri: 'ck-guide://v1' });
    const text = (guide.contents[0] as { text: string }).text;
    assert.ok(text.includes('ck-guide/1'));

    const closed = textOf(await client.callTool({ name: 'close', arguments: { session: s } })) as { closed: string };
    assert.equal(closed.closed, s);
    await client.close();
  });

  it('rejects unknown sessions and bad ops', async () => {
    const client = await linked();
    const err = (await client.callTool({ name: 'read', arguments: { session: 'sess-99' } })) as { isError?: boolean; content?: Array<{ text?: string }> };
    assert.equal(err.isError, true);
    assert.match(err.content?.[0]?.text ?? '', /unknown session/);
    const opened = textOf(await client.callTool({ name: 'open', arguments: {} })) as { session: string };
    const bad = textOf(await client.callTool({
      name: 'edit', arguments: { session: opened.session, ops: [{ op: 'add', feature: { type: 'nope' } }] },
    })) as { ok: boolean; schemaError: string };
    assert.equal(bad.ok, false);
    assert.match(bad.schemaError, /unknown feature type/);
    await client.callTool({ name: 'close', arguments: { session: opened.session } });
    await client.close();
  });

  it('stdio entry lists tools', async () => {
    const { Client: C } = await import('@modelcontextprotocol/sdk/client/index.js');
    const { StdioClientTransport } = await import('@modelcontextprotocol/sdk/client/stdio.js');
    const client = new C({ name: 't', version: '0' });
    const transport = new StdioClientTransport({ command: 'node', args: [path.join(ROOT, 'packages/mcp.mjs')] });
    await client.connect(transport);
    try {
      const tools = await client.listTools();
      assert.ok(tools.tools.length >= 8);
    } finally {
      await client.close();
    }
  });
});
