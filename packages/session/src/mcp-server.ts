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
    wasmPath: z.string().optional(),
  }, async ({ name, doc, wasmPath }) => {
    const { emptyPart } = await import('../../part-model/src/index.js');
    const part = doc && typeof doc === 'object' && Array.isArray((doc as Record<string, unknown>).features)
      ? (doc as unknown as Parameters<typeof Session.open>[0])
      : emptyPart(name ?? 'Part');
    const id = `sess-${seq++}`;
    sessions.set(id, await Session.open(part, wasmPath ? { wasmPath } : undefined));
    return text({ session: id });
  });

  server.tool('guide', 'The authoring guide: feature fields, frames and traps, naming, recipes, error fixes. Read first.', {}, async () => text(getGuide()));

  server.tool('schema', 'Feature types and a minimal example edit batch (a plate)', {}, async () => {
    const { FEATURE_TYPES } = await import('../../part-model/src/index.js');
    const sk = (id: string, plane: string, pts: number[][]) => ({
      type: 'sketch', id, plane: { kind: 'std', id: plane },
      points: pts.map(([x, y], i) => ({ id: `p${i + 1}`, x, y })),
      entities: pts.map((_, i) => ({ id: `e${i + 1}`, type: 'line', p1: `p${i + 1}`, p2: `p${((i + 1) % pts.length) + 1}` })),
      constraints: [],
    });
    return text({
      featureTypes: FEATURE_TYPES,
      profiles: ['fdm-print', 'cnc-3axis', 'injection'],
      threadStandards: ['metric', 'metric-fine', 'UNC', 'UNF', 'BSP', 'NPT', 'Tr', 'Acme'],
      planes: { front: 'XY, normal +Z', top: 'XZ, local y maps to -z', right: 'ZY' },
      exampleOps: [
        { op: 'add', feature: sk('F1', 'front', [[0, 0], [60, 0], [60, 40], [0, 40]]) },
        { op: 'add', feature: { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 10 } },
      ],
      exampleExpect: [{ kind: 'volume', value: 24000, tol: 0.01 }, { kind: 'valid' }],
    });
  });

  server.tool('threadData', 'Thread standards and sizes (ISO metric, UNC/UNF, BSP, NPT, Tr, Acme) with diameters, pitch, tap drill and default class; use for the thread feature', {
    session: z.string(),
    standard: z.string().optional(),
    size: z.string().optional(),
  }, async ({ session, standard, size }) => {
    return text(await get(session).threadData(standard, size));
  });

  server.tool('profiles', 'Design-rule profiles (fdm-print, cnc-3axis, injection) with each rule\'s parameters, units and defaults', {
    session: z.string(),
  }, async ({ session }) => text(await get(session).profiles()));

  server.tool('checkProfile', 'Run a manufacturability profile on the body: issues name failing faces / edges by persistent name, with value, limit and severity. profile = id or inline {rules:[{id, params?, severity?}]}', {
    session: z.string(),
    profile: z.union([z.string(), z.object({ rules: z.array(z.object({ id: z.string(), params: z.record(z.any()).optional(), severity: z.enum(['error', 'warn', 'info']).optional() })) }).passthrough()]),
    samples: z.number().optional(),
    maxIssues: z.number().optional(),
  }, async ({ session, profile, samples, maxIssues }) => {
    return text(await get(session).checkProfile(profile as string | Record<string, unknown>, { samples, maxIssues }));
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

  server.tool('render', 'Render the body to PNG with highlighted faces + legend', {
    session: z.string(),
    view: z.enum(['iso', 'front', 'top', 'right']).optional(),
    highlights: z.array(z.string()).optional(),
    width: z.number().optional(),
    height: z.number().optional(),
  }, async ({ session, view, highlights, width, height }) => {
    const { renderMesh } = await import('../../render/src/render.js');
    const s = get(session);
    const r = renderMesh(await s.mesh(), { view, highlights, width, height });
    return text({ png: r.png.toString('base64'), width: r.width, height: r.height, legend: r.legend });
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
