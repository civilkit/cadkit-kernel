// Session: one document + one kernel worker, per docs/spec-agent-interface.md.
// Edit ops are add/set/remove/move/replace. Each op is schema-checked against
// the typed features before it reaches the kernel; a batch applies atomically
// (regen errors leave the document unchanged). Every applied batch records
// {id, author, time, ops, reason}; undo reverts the last N batches.
import { createWorkerKernel, type WorkerKernel } from '../../kernel-client/src/worker.js';
import { emptyPart, FEATURE_TYPES, type PartDoc } from '../../part-model/src/index.js';
import { hintFor, verify, type RegenError, type VerifyReport } from '../../driver/src/verify.js';

export type EditOp =
  | { op: 'add'; feature: Record<string, unknown>; index?: number }
  | { op: 'set'; id: string; path: string; value: unknown }
  | { op: 'remove'; id: string; cascade?: boolean }
  | { op: 'move'; id: string; to: number }
  | { op: 'replace'; id: string; feature: Record<string, unknown> };

export interface EditRecord {
  id: string;
  author: string;
  time: number;
  ops: EditOp[];
  reason?: string;
}

export interface BodyFacts {
  faces: string[];
  edges: string[];
  solids: number;
  volume: number | null;
  box: { min: number[]; max: number[] } | null;
}

export interface CountDiff<T> {
  before: T;
  after: T;
}

export interface BodyDiff {
  faces: CountDiff<number> & { added: string[]; removed: string[] };
  edges: CountDiff<number> & { added: string[]; removed: string[] };
  solids: CountDiff<number>;
  volume: CountDiff<number | null>;
  box: CountDiff<{ min: number[]; max: number[] } | null>;
}

export interface RefWarning {
  code: string;
  feature?: string;
  ref?: unknown;
  reason?: string;
  [key: string]: unknown;
}

export interface EditResult {
  ok: boolean;
  record: EditRecord | null;
  errors?: RegenError[];
  hints?: ReturnType<typeof hintFor>[];
  schemaError?: string;
  diff?: BodyDiff | null;
  refWarnings?: RefWarning[];
}

const ID = /^F\d+$/;

function clone<T>(v: T): T {
  return JSON.parse(JSON.stringify(v)) as T;
}

/** ids of features whose JSON mentions id (excluding the feature itself). */
export function referencers(doc: PartDoc, id: string): string[] {
  const needle = `"${id}"`;
  return doc.features
    .filter((f) => f.id !== id && JSON.stringify(f).includes(needle))
    .map((f) => f.id);
}

/** Structural check of one op against the document. Returns an error or null. */
export function checkOp(doc: PartDoc, op: EditOp): string | null {
  const ids = new Set(doc.features.map((f) => f.id));
  switch (op.op) {
    case 'add': {
      const t = (op.feature as { type?: unknown }).type;
      if (typeof t !== 'string' || !(FEATURE_TYPES as readonly string[]).includes(t)) {
        return `unknown feature type ${JSON.stringify(t)}`;
      }
      const id = (op.feature as { id?: unknown }).id;
      if (id !== undefined && (typeof id !== 'string' || !ID.test(id))) return `bad id ${JSON.stringify(id)}`;
      if (typeof id === 'string' && ids.has(id)) return `id ${id} already exists`;
      if (op.index !== undefined && (op.index < 0 || op.index > doc.features.length)) {
        return `index ${op.index} out of range`;
      }
      return null;
    }
    case 'set': {
      if (!ids.has(op.id)) return `unknown feature ${op.id}`;
      if (!op.path) return 'empty path';
      const head = op.path.split('.')[0];
      if (head === 'id' || head === 'type') return `cannot set ${head}`;
      return null;
    }
    case 'remove': {
      if (!ids.has(op.id)) return `unknown feature ${op.id}`;
      if (!op.cascade) {
        const refs = referencers(doc, op.id);
        if (refs.length) return `${op.id} is still referenced by ${refs.join(', ')} (use cascade to remove them too)`;
      }
      return null;
    }
    case 'move': {
      if (!ids.has(op.id)) return `unknown feature ${op.id}`;
      if (op.to < 0 || op.to >= doc.features.length) return `index ${op.to} out of range`;
      return null;
    }
    case 'replace': {
      if (!ids.has(op.id)) return `unknown feature ${op.id}`;
      const t = (op.feature as { type?: unknown }).type;
      if (typeof t !== 'string' || !(FEATURE_TYPES as readonly string[]).includes(t)) {
        return `unknown feature type ${JSON.stringify(t)}`;
      }
      return null;
    }
  }
}

function setPath(feature: Record<string, unknown>, path: string, value: unknown): boolean {
  const parts = path.split('.');
  let node: unknown = feature;
  for (let i = 0; i < parts.length - 1; i++) {
    const next = (node as Record<string, unknown>)[parts[i]];
    if (next === null || typeof next !== 'object') return false;
    node = next;
  }
  const last = parts[parts.length - 1];
  if (Array.isArray(node)) {
    const idx = Number(last);
    if (!Number.isInteger(idx) || idx < 0 || idx >= node.length) return false;
    node[idx] = value;
    return true;
  }
  if (node === null || typeof node !== 'object') return false;
  (node as Record<string, unknown>)[last] = value;
  return true;
}

function applyOp(doc: PartDoc, op: EditOp): void {
  switch (op.op) {
    case 'add': {
      const f = clone(op.feature);
      if (f.id === undefined) f.id = `F${doc.seq++}`;
      const rec = f as unknown as import('../../part-model/src/index.js').Feature;
      if (op.index === undefined) doc.features.push(rec);
      else doc.features.splice(op.index, 0, rec);
      return;
    }
    case 'set': {
      const f = doc.features.find((x) => x.id === op.id)! as unknown as Record<string, unknown>;
      if (!setPath(f, op.path, clone(op.value))) throw new Error(`cannot set ${op.path} on ${op.id}`);
      return;
    }
    case 'remove': {
      const gone = new Set<string>([op.id]);
      if (op.cascade) {
        let grew = true;
        while (grew) {
          grew = false;
          for (const f of doc.features) {
            if (gone.has(f.id)) continue;
            const hit = [...gone].some((g) => JSON.stringify(f).includes(`"${g}"`));
            if (hit) { gone.add(f.id); grew = true; }
          }
        }
      }
      doc.features = doc.features.filter((f) => !gone.has(f.id));
      return;
    }
    case 'move': {
      const i = doc.features.findIndex((x) => x.id === op.id);
      const [f] = doc.features.splice(i, 1);
      doc.features.splice(op.to, 0, f);
      return;
    }
    case 'replace': {
      const i = doc.features.findIndex((x) => x.id === op.id);
      const f = clone(op.feature) as unknown as import('../../part-model/src/index.js').Feature;
      (f as unknown as Record<string, unknown>).id = op.id;
      doc.features[i] = f;
      return;
    }
  }
}

export function diffFacts(before: BodyFacts, after: BodyFacts): BodyDiff {
  const added = (a: string[], b: string[]) => b.filter((x) => !a.includes(x)).slice(0, 50);
  return {
    faces: { before: before.faces.length, after: after.faces.length, added: added(before.faces, after.faces), removed: added(after.faces, before.faces) },
    edges: { before: before.edges.length, after: after.edges.length, added: added(before.edges, after.edges), removed: added(after.edges, before.edges) },
    solids: { before: before.solids, after: after.solids },
    volume: { before: before.volume, after: after.volume },
    box: { before: before.box, after: after.box },
  };
}

let sessionSeq = 1;

export class Session {
  private kernel: import('../../driver/src/verify.js').KernelTransport;
  private worker?: WorkerKernel;
  private doc: PartDoc;
  private tag: string;
  readonly author: string;
  private stack: Array<{ record: EditRecord; before: PartDoc }> = [];
  private editSeq = 1;

  constructor(doc: PartDoc, kernel: import('../../driver/src/verify.js').KernelTransport, author = 'agent', tag?: string) {
    this.doc = clone(doc);
    this.kernel = kernel;
    this.author = author;
    this.tag = tag ?? `sess${sessionSeq++}`;
  }

  static async open(doc?: PartDoc, opts?: { wasmPath?: string; author?: string }): Promise<Session> {
    const worker = await createWorkerKernel(opts?.wasmPath);
    const s = new Session(doc ?? emptyPart('Part'), worker, opts?.author ?? 'agent');
    s.worker = worker;
    return s;
  }

  get document(): PartDoc {
    return clone(this.doc);
  }

  history(): EditRecord[] {
    return this.stack.map((e) => e.record);
  }

  private async regen(doc: PartDoc): Promise<{ errors: RegenError[]; body: Record<string, unknown> | null }> {
    const r = await this.kernel.request({ type: 'regen', tag: this.tag, part: doc });
    if (r.trap || !r.ok || !r.result) throw new Error(r.trap ?? (typeof r.error === 'string' ? r.error : 'regen failed'));
    const res = r.result as { errors?: RegenError[]; body?: Record<string, unknown> | null };
    return { errors: res.errors ?? [], body: res.body ?? null };
  }

  private async mass(): Promise<{ volume: number | null; solids: number }> {
    try {
      const r = await this.kernel.request({ type: 'massProps', tag: this.tag, bodies: true });
      if (r.trap || !r.ok || !r.result) return { volume: null, solids: 0 };
      const res = r.result as { volume?: number; bodies?: unknown[] };
      return {
        volume: typeof res.volume === 'number' ? res.volume : null,
        solids: Array.isArray(res.bodies) ? res.bodies.length : 0,
      };
    } catch {
      return { volume: null, solids: 0 };
    }
  }

  async facts(): Promise<BodyFacts> {
    const { body } = await this.regen(this.doc);
    const faces = ((body?.faces as Array<{ id: string }> | undefined) ?? []).map((f) => f.id);
    const edges = ((body?.edges as Array<{ key: string }> | undefined) ?? []).map((e) => e.key);
    const box = (body?.box as BodyFacts['box']) ?? null;
    const { volume, solids } = await this.mass();
    return { faces, edges, solids, volume, box };
  }

  async edit(ops: EditOp[], reason?: string): Promise<EditResult> {
    for (const op of ops) {
      const problem = checkOp(this.doc, op);
      if (problem) return { ok: false, record: null, schemaError: problem };
    }
    const before = clone(this.doc);
    const candidate = clone(this.doc);
    try {
      for (const op of ops) applyOp(candidate, op);
    } catch (e) {
      return { ok: false, record: null, schemaError: String((e as Error).message ?? e) };
    }

    const pre = await this.facts();
    let regenRes: { errors: RegenError[]; body: Record<string, unknown> | null };
    try {
      regenRes = await this.regen(candidate);
    } catch (e) {
      return { ok: false, record: null, errors: [], hints: [{ feature: '', code: 'trap', hint: String((e as Error).message ?? e) }] };
    }
    if (regenRes.errors.length) {
      // Atomicity: the document stays unchanged; report what failed.
      await this.regen(this.doc).catch(() => null); // restore tag state
      return {
        ok: false,
        record: null,
        errors: regenRes.errors,
        hints: regenRes.errors.map(([fid, err]) => hintFor(fid, err.code, err.params ?? {})),
      };
    }

    const faces = ((regenRes.body?.faces as Array<{ id: string }> | undefined) ?? []).map((f) => f.id);
    const edges = ((regenRes.body?.edges as Array<{ key: string }> | undefined) ?? []).map((e) => e.key);
    const box = (regenRes.body?.box as BodyFacts['box']) ?? null;
    const { volume, solids } = await this.mass();
    const post: BodyFacts = { faces, edges, solids, volume, box };

    let refWarnings: RefWarning[] = [];
    try {
      const rd = await this.kernel.request({ type: 'refDiff', base: before, part: candidate });
      if (!rd.trap && rd.ok && rd.result) {
        refWarnings = ((rd.result as { warnings?: RefWarning[] }).warnings ?? []) as RefWarning[];
      }
    } catch { refWarnings = []; }

    this.doc = candidate;
    const record: EditRecord = { id: `edit-${this.editSeq++}`, author: this.author, time: Date.now(), ops: clone(ops), reason };
    this.stack.push({ record, before });
    return { ok: true, record, diff: diffFacts(pre, post), refWarnings };
  }

  /** Revert the last n applied batches. The document is the source of truth;
   *  the kernel tag is re-synced (resynced:false if that regen itself fails). */
  async undo(n = 1): Promise<{ records: EditRecord[]; resynced: boolean }> {
    const popped = this.stack.splice(Math.max(0, this.stack.length - n), n);
    if (popped.length) this.doc = clone(popped[0].before);
    let resynced = true;
    try {
      await this.regen(this.doc);
    } catch {
      resynced = false;
    }
    return { records: popped.map((p) => p.record), resynced };
  }

  verify(expect?: Array<Record<string, unknown>>): Promise<VerifyReport> {
    return verify(this.kernel, this.doc, { tag: this.tag, expect });
  }

  async close(): Promise<void> {
    await this.worker?.close();
  }
}
