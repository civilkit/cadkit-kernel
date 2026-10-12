// Offline scripted agent benchmark: a model (or a recorded stub) gets a text
// description of a bench part and must produce it as session edits; the result
// is checked against the CadQuery ground-truth volume from the bench manifest.
// Reports success by family. No live model call needed: the stub replays
// recorded edits (proves the harness), a real model implements Model.
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Session, type EditOp } from './session.js';
import { emptyPart } from '../../part-model/src/index.js';

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
