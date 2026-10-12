// MCP server over the session layer: open/read/edit/query/resolve/check/
// verify/close as tools, the authoring guide as a resource. All logic lives
// in Session and the query wrappers; this file is transport + schemas only.
import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import { z } from 'zod';
import { Session, type EditOp } from './session.js';
import { check, describe, heal, massProps, resolveDescriptions, validate } from './queries.js';
import { getGuide, GUIDE_VERSION } from './guide.js';

const FeatureJson = z.record(z.string(), z.unknown());

const EditOpSchema = z.union([
  z.object({ op: z.literal('add'), feature: FeatureJson, index: z.number().optional() }),
  z.object({ op: z.literal('set'), id: z.string(), path: z.string(), value: z.unknown() }),
  z.object({ op: z.literal('remove'), id: z.string(), cascade: z.boolean().optional() }),
  z.object({ op: z.literal('move'), id: z.string(), to: z.number() }),
  z.object({ op: z.literal('replace'), id: z.string(), feature: FeatureJson }),
]);

const text = (v: unknown) => ({ content: [{ type: 'text' as const, text: JSON.stringify(v) }] });

export function createMcpServer(): { server: McpServer; sessions: Map<string, Session> } {
  const server = new McpServer({ name: 'ck-cad', version: '0.1.0' });
  const sessions = new Map<string, Session>();
  let seq = 1;

  const get = (id: string): Session => {
    const s = sessions.get(id);
    if (!s) throw new Error(`unknown session ${id}`);
    return s;
  };

  server.tool('open', 'Open a session holding one document (empty or from part JSON)', {
    name: z.string().optional(),
    doc: FeatureJson.optional(),
  }, async ({ name, doc }) => {
    const { emptyPart } = await import('../../part-model/src/index.js');
    const part = doc && typeof doc === 'object' && Array.isArray((doc as Record<string, unknown>).features)
      ? (doc as unknown as Parameters<typeof Session.open>[0])
      : emptyPart(name ?? 'Part');
    const id = `sess-${seq++}`;
    sessions.set(id, await Session.open(part));
    return text({ session: id });
  });

  server.tool('read', 'Read the document, body facts and edit history', {
    session: z.string(),
  }, async ({ session }) => {
    const s = get(session);
    return text({ document: s.document, facts: await s.facts(), history: s.history() });
  });

  server.tool('edit', 'Apply an atomic batch of edit ops (add/set/remove/move/replace)', {
    session: z.string(),
    ops: z.array(EditOpSchema),
    reason: z.string().optional(),
  }, async ({ session, ops, reason }) => {
    const r = await get(session).edit(ops as EditOp[], reason);
    return text(r);
  });

  server.tool('query', 'Body queries: validate, heal, massProps, step, describe', {
    session: z.string(),
    kind: z.enum(['validate', 'heal', 'massProps', 'step', 'describe']),
    params: FeatureJson.optional(),
  }, async ({ session, kind, params }) => {
    const s = get(session);
    const p = (params ?? {}) as Record<string, unknown>;
    switch (kind) {
      case 'validate': return text(await s.validate(p.tol as number | undefined));
      case 'heal': return text(await s.heal(p as { damage?: string; dryRun?: boolean; tol?: number }));
      case 'massProps': return text(await s.massProps(p as { bodies?: boolean; densities?: number[] }));
      case 'step': return text({ step: await s.step(p.name as string | undefined) });
      case 'describe': return text(await s.describe((p.names as unknown[]) ?? []));
    }
  });

  server.tool('resolve', 'Resolve face/edge descriptions to persistent names', {
    session: z.string(),
    descriptions: z.array(FeatureJson),
  }, async ({ session, descriptions }) => {
    return text(await get(session).resolve(descriptions as Array<Record<string, unknown>>));
  });

  server.tool('check', 'Check expectations (volume, bbox, counts, valid)', {
    session: z.string(),
    expect: z.union([z.array(FeatureJson), FeatureJson]),
  }, async ({ session, expect }) => {
    return text(await get(session).check(expect as Array<Record<string, unknown>> | Record<string, unknown>));
  });

  server.tool('verify', 'One call: regen + validate + check with repair hints', {
    session: z.string(),
    expect: z.array(FeatureJson).optional(),
  }, async ({ session, expect }) => {
    return text(await get(session).verify(expect as Array<Record<string, unknown>> | undefined));
  });

  server.tool('close', 'Close a session and its kernel worker', {
    session: z.string(),
  }, async ({ session }) => {
    await get(session).close();
    sessions.delete(session);
    return text({ closed: session });
  });

  server.resource('guide', 'ck-guide://v1', async (uri) => ({
    contents: [{ uri: uri.href, text: getGuide().text }],
  }));

  return { server, sessions };
}
