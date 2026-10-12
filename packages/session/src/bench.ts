// Offline scripted agent benchmark: a model (or a recorded stub) gets a text
// description of a bench part and must produce it as session edits; the result
// is checked against the CadQuery ground-truth volume from the bench manifest.
// Reports success by family. No live model call needed: the stub replays
// recorded edits (proves the harness), a real model implements Model.
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Session, type EditOp } from './session.js';
import { emptyPart, type PartDoc } from '../../part-model/src/index.js';
import { Part } from '../../builder/src/index.js';

export interface BenchTask {
  id: string;
  family: string;
  tier: string;
  description: string;
  volume: number;
}

/** What a model returns for a task: a batch of session edits. */
export type Model = {
  attempt(task: BenchTask): Promise<EditOp[]>;
};

export interface BenchResult {
  id: string;
  family: string;
  pass: boolean;
  volumeGot: number | null;
  volumeWant: number;
  detail: string;
  ms: number;
}

export interface BenchReport {
  results: BenchResult[];
  byFamily: Record<string, { pass: number; total: number }>;
}

const HERE = path.dirname(fileURLToPath(import.meta.url));

export function loadTasks(opts?: { manifest?: string; first?: number }): BenchTask[] {
  const file = opts?.manifest ?? path.resolve(HERE, '../../../../tools/bench/benchcad/manifest.json');
  const m = JSON.parse(readFileSync(file, 'utf8')) as {
    parts: Array<{ id: string; family: string; tier: string; ops?: string[]; ground_truth?: { volume?: number; bbox?: number[] } }>;
    known?: Record<string, { skip?: string }>;
  };
  return m.parts
    .filter((p) => !m.known?.[p.id]?.skip && typeof p.ground_truth?.volume === 'number')
    .slice(0, opts?.first ?? 10)
    .map((p) => ({
      id: p.id,
      family: p.family,
      tier: p.tier,
      description: `${p.id} (${p.family}, ${p.tier}): model with ${p.ops?.join(', ') ?? 'unknown ops'}; bbox ${JSON.stringify(p.ground_truth?.bbox)} (Z-up mm)`,
      volume: p.ground_truth!.volume as number,
    }));
}

/** Stub model: replays a recorded feature list as add-ops for each task id. */
export function replayModel(docs: Map<string, { features: Array<Record<string, unknown>> }>): Model {
  return {
    async attempt(task: BenchTask): Promise<EditOp[]> {
      const doc = docs.get(task.id);
      if (!doc) return [];
      return doc.features.map((feature) => ({ op: 'add', feature }));
    },
  };
}

export async function runAgentBench(opts: {
  wasmPath?: string;
  tasks: BenchTask[];
  model: Model;
  volumeTol?: number;
  log?: (line: string) => void;
}): Promise<BenchReport> {
  const tol = opts.volumeTol ?? 1e-3;
  const results: BenchResult[] = [];
  for (const task of opts.tasks) {
    const t0 = Date.now();
    const s = await Session.open(emptyPart(task.id), { wasmPath: opts.wasmPath, author: `model:${task.id}` });
    try {
      let edits: EditOp[];
      try {
        edits = await opts.model.attempt(task);
      } catch (e) {
        results.push({ id: task.id, family: task.family, pass: false, volumeGot: null, volumeWant: task.volume, detail: `model threw: ${String((e as Error).message ?? e)}`, ms: Date.now() - t0 });
        continue;
      }
      const er = await s.edit(edits, `model attempt: ${task.id}`);
      if (!er.ok) {
        results.push({ id: task.id, family: task.family, pass: false, volumeGot: null, volumeWant: task.volume, detail: er.schemaError ?? er.errors?.map(([f, x]) => `${f}:${x.code}`).join(',') ?? 'edit failed', ms: Date.now() - t0 });
        continue;
      }
      const v = await s.verify([{ kind: 'volume', equals: task.volume, relTol: tol }]);
      const got = (v.check?.results[0]?.measured as number | undefined) ?? null;
      const pass = v.ok;
      const detail = pass ? `volume ${got}` : (v.hints.map((h) => h.hint).join(' | ') || 'verify failed');
      results.push({ id: task.id, family: task.family, pass, volumeGot: got, volumeWant: task.volume, detail, ms: Date.now() - t0 });
      opts.log?.(`${pass ? 'PASS' : 'FAIL'} ${task.id} ${detail}`);
    } finally {
      await s.close();
    }
  }
  const byFamily: BenchReport['byFamily'] = {};
  for (const r of results) {
    byFamily[r.family] ??= { pass: 0, total: 0 };
    byFamily[r.family].total++;
    if (r.pass) byFamily[r.family].pass++;
  }
  return { results, byFamily };
}

// --- Edit tasks: change a built part, assert exact consequences -----------

export interface EditBenchTask {
  id: string;
  /** ops from an empty document producing the base part */
  base: EditOp[];
  /** the edit under test */
  edits: EditOp[];
  /** expected volume after the edit */
  expectVolume: { equals: number; relTol?: number };
  /** expected refDiff warning codes from the edit (default: none) */
  expectRefCodes?: string[];
}

export interface EditBenchResult {
  id: string;
  pass: boolean;
  detail: string;
  ms: number;
}

function plateWithHoles(r: number): Part {
  const p = new Part('Plate', 'aluminium');
  const base = p.sketch('Outline', 'front', (sk) => sk.rect(0, 0, 100, 60));
  p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 10, name: 'Plate' });
  const holes = p.sketch('Holes', 'front', (sk) => {
    for (const [x, y] of [[10, 10], [90, 10], [10, 50], [90, 50]] as Array<[number, number]>) sk.circle(x, y, r);
  });
  p.extrude({ sketch: holes, op: 'cut', end: 'throughBoth', name: 'Drill' });
  return p;
}

const HOLE_R4 = 4 * Math.PI * 16 * 10; // volume of four r4 through-holes in 10mm stock
const PLATE_R4 = 60000 - HOLE_R4;

function adds(p: Part): EditOp[] {
  return p.toJSON().features.map((f) => ({ op: 'add', feature: f as unknown as Record<string, unknown> }));
}

function circleIds(p: Part, sketchId: string): string[] {
  const sk = p.toJSON().features.find((f) => f.id === sketchId) as unknown as {
    entities: Array<{ id: string; type: string; r?: number }>;
  };
  return sk.entities.filter((e) => e.type === 'circle').map((e) => e.id);
}

/** Plate with r4 holes; widen every hole circle to r6. */
function holeDiameterTask(): EditBenchTask {
  const p = plateWithHoles(4);
  const widen = circleIds(p, 'F3').map((id) => ({ op: 'set', id: 'F3', path: `entities.${entityIndex(p, 'F3', id)}.r`, value: 6 }));
  return {
    id: 'hole-diameter',
    base: adds(p),
    edits: widen as EditOp[],
    expectVolume: { equals: 60000 - 4 * Math.PI * 36 * 10, relTol: 1e-6 },
  };
}

function entityIndex(p: Part, sketchId: string, entityId: string): number {
  const sk = p.toJSON().features.find((f) => f.id === sketchId) as unknown as { entities: Array<{ id: string }> };
  return sk.entities.findIndex((e) => e.id === entityId);
}

/** Plate with r4 holes; drill a fifth r4 hole at the centre. */
function fifthHoleTask(): EditBenchTask {
  const p = plateWithHoles(4);
  return {
    id: 'fifth-hole',
    base: adds(p),
    edits: [
      { op: 'add', feature: { type: 'sketch', name: 'Extra', plane: { kind: 'std', id: 'front' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'p1', x: 50, y: 30 }], entities: [{ id: 'e2', type: 'circle', c: 'p1', r: 4 }], constraints: [] } },
      { op: 'add', feature: { type: 'extrude', op: 'cut', sketch: 'F5', end: 'throughBoth', name: 'Extra cut' } },
    ],
    expectVolume: { equals: PLATE_R4 - Math.PI * 16 * 10, relTol: 1e-6 },
  };
}

/** Move the holes sketch first: order-only, body identical. */
function moveFeatureTask(): EditBenchTask {
  return {
    id: 'move-feature',
    base: adds(plateWithHoles(4)),
    edits: [{ op: 'move', id: 'F3', to: 0 }],
    expectVolume: { equals: PLATE_R4, relTol: 1e-9 },
  };
}

/** Remove the holes sketch with cascade: the dependent cut goes too. */
function removeCascadeTask(): EditBenchTask {
  return {
    id: 'remove-cascade',
    base: adds(plateWithHoles(4)),
    edits: [{ op: 'remove', id: 'F3', cascade: true }],
    expectVolume: { equals: 60000, relTol: 1e-9 },
  };
}

export const EDIT_TASKS: EditBenchTask[] = [holeDiameterTask(), fifthHoleTask(), moveFeatureTask(), removeCascadeTask()];

export async function runEditBench(opts: {
  wasmPath?: string;
  tasks?: EditBenchTask[];
  log?: (line: string) => void;
}): Promise<{ results: EditBenchResult[] }> {
  const results: EditBenchResult[] = [];
  for (const task of opts.tasks ?? EDIT_TASKS) {
    const t0 = Date.now();
    const s = await Session.open(emptyPart(task.id), { wasmPath: opts.wasmPath, author: 'edit-bench' });
    try {
      const b = await s.edit(task.base, 'base');
      if (!b.ok) {
        results.push({ id: task.id, pass: false, detail: `base failed: ${b.schemaError ?? JSON.stringify(b.errors)}`, ms: Date.now() - t0 });
        continue;
      }
      const e = await s.edit(task.edits, 'test');
      if (!e.ok) {
        results.push({ id: task.id, pass: false, detail: `edit failed: ${e.schemaError ?? JSON.stringify(e.errors)}`, ms: Date.now() - t0 });
        continue;
      }
      const codes = (e.refWarnings ?? []).map((w) => w.code).sort();
      const want = [...(task.expectRefCodes ?? [])].sort();
      if (JSON.stringify(codes) !== JSON.stringify(want)) {
        results.push({ id: task.id, pass: false, detail: `refWarnings [${codes}] want [${want}]`, ms: Date.now() - t0 });
        continue;
      }
      const v = await s.verify([{ kind: 'volume', equals: task.expectVolume.equals, relTol: task.expectVolume.relTol ?? 1e-6 }]);
      const got = v.check?.results[0]?.measured as number | undefined;
      const pass = v.ok;
      const detail = pass ? `volume ${got}` : (v.hints.map((h) => h.hint).join(' | ') || 'verify failed');
      results.push({ id: task.id, pass, detail, ms: Date.now() - t0 });
      opts.log?.(`${pass ? 'PASS' : 'FAIL'} edit:${task.id} ${detail.slice(0, 160)}`);
    } finally {
      await s.close();
    }
  }
  return { results };
}

export type { PartDoc };
