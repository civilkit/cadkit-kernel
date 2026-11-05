// Session: one document + one kernel worker, per docs/spec-agent-interface.md.
// Edit ops are add/set/remove/move/replace. Each op is schema-checked against
// the typed features before it reaches the kernel; a batch applies atomically
// (regen errors leave the document unchanged). Every applied batch records
// {id, author, time, ops, reason}; undo reverts the last N batches.
import { emptyPart, FEATURE_TYPES, type PartDoc } from '../../part-model/src/index.js';
import { hintFor, verify, type CheckResultItem, type KernelTransport, type RegenError, type VerifyReport } from '../../driver/src/verify.js';
import {
  angles, check, checkProfile, clearance, closestPoint, describe, distance, exportMesh, extrema, maxDistance, symmetry, thickness, thicknessAt, heal, importSat, importSkp, importSldprt, importStep, massProps, probe, profiles, resolveDescriptions, section, sections, step, textMetrics, textOutline, loadFont, unloadFont, listFonts, subsetFont, threadData, validate,
  type CheckReport, type DescribeItem, type ExportMeshOpts, type ExportMeshReport, type HealReport, type ImportReport, type SkpUp, type MassReport, type ProbeReport,
  type AnglesReport, type ClearanceReport, type ClosestPointReport, type DistanceReport, type HausdorffReport, type MaxDistanceReport, type MeasureEntity,
  type ExtremaReport, type SymmetryReport, type ThicknessAtReport, type ThicknessReport,
  type ResolveMatch, type SectionReport, type SectionsReport, type ValidateIssue,
} from './queries.js';
export type { ExportMeshOpts, ExportMeshReport };
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

/** A sketch/plane placement frame: origin plus three unit axes (column vectors). */
export interface ViewFrame {
  o: number[];
  x: number[];
  y: number[];
  z: number[];
}

/** What bodyForView (and a successful preview) hands a viewer. */
export interface ViewBody {
  mesh: import('../../render/src/mesh.js').MeshData;
  edges: Array<{ key?: string; pts: number[][] }>;
  box: { min: number[]; max: number[] };
  sketches: Array<{ id: string; frame: ViewFrame; sketch: import('../../part-model/src/index.js').SketchFeature; status?: import('../../part-model/src/index.js').SketchSolverStatus }>;
  curves: Array<{ id: string; pts: number[][] }>;
  planes: Array<{ id: string; frame: ViewFrame }>;
  markers: unknown[];
}

export interface PreviewResult {
  ok: boolean;
  errors: RegenError[];
  hints: ReturnType<typeof hintFor>[];
  body: ViewBody | null;
  /** The solved sketches of the regen, present on every successful preview. A prefix that has no solid yet
   *  (a first sketch, `upTo` cutting off the extrude) has `body: null` but still has its sketches. */
  sketches?: ViewBody['sketches'];
  schemaError?: string;
  /** Surface bodies, cosmetic thread markers and warnings of the same regen (so a viewer needs no second request). */
  surfaces?: unknown;
  threads?: unknown[];
  warnings?: unknown[];
  /** true when a newer preview superseded this one before it finished: the result is dropped. */
  stale?: boolean;
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
  /** Ids of the features this batch added (assigned when the op had none). */
  added?: string[];
  diff?: BodyDiff | null;
  refWarnings?: RefWarning[];
  /** With `edit(..., {lean: true})`: the reference warnings, when they could not be skipped, arrive here after the edit
   *  has been applied (a refDiff regenerates both documents; a viewer does not need to wait for it). */
  refWarningsLater?: Promise<RefWarning[]>;
  /** Document expectations check (present when the doc carries expect). */
  intent?: { passed: number; failed: number; results: CheckResultItem[] } | null;
}

/** Any non-empty string identifies a feature (the app mints F-ids; the kernel
 *  and hand-written docs use readable ids like P, W, K1). */
function isId(id: unknown): id is string {
  return typeof id === 'string' && id.length > 0;
}

function seqOf(id: string): number | null {
  const n = /^F(\d+)$/.exec(id);
  return n ? +n[1] : null;
}

/** Make `doc.seq` a finite counter above every F<n> id in use. A document without `seq` (or with a
 *  corrupt one) used to mint FNaN / Fnull / F0 and then duplicate F1. */
export function normalizeSeq(doc: PartDoc): PartDoc {
  let top = 0;
  for (const f of doc.features ?? []) {
    const n = typeof f?.id === 'string' ? seqOf(f.id) : null;
    if (n !== null && n > top) top = n;
  }
  const cur = Number.isFinite(doc.seq) && Number.isInteger(doc.seq) ? doc.seq : 0;
  doc.seq = Math.max(cur, top + 1, 1);
  return doc;
}

const BAD_ID = /^F(NaN|null|undefined|Infinity)$/;

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

/** Face / edge name references inside a feature's JSON, the way the kernel's reference check (refDiff) reads them: a
 *  face-name string (`F12.C1`, `F3.S.e2`) or an edge reference `{faces: [a, b], at: [x, y, z]}`. */
export function hasNameRefs(j: unknown): boolean {
  if (typeof j === 'string') return /^F\d+\.[A-Za-z]/.test(j);
  if (Array.isArray(j)) return j.some(hasNameRefs);
  if (j && typeof j === 'object') {
    const o = j as Record<string, unknown>;
    if (Array.isArray(o.faces) && o.faces.length === 2 && o.faces.every((x) => typeof x === 'string') && Array.isArray(o.at) && o.at.length === 3 && o.at.every((x) => typeof x === 'number')) return true;
    return Object.values(o).some(hasNameRefs);
  }
  return false;
}

/** Feature keys that only label a feature in the tree; the kernel leaves them out of its regen cache key and never reads
 *  them (docs/regen-cache.md, "Cosmetic fields"). */
export const COSMETIC_KEYS: readonly string[] = ['group', 'name', 'autoName', 'visible', 'userHidden'];

/** Does this op change nothing but cosmetic keys of the features it names? A `set` of one (path head), or a `replace` whose
 *  feature equals the current one but for them. */
export function isCosmeticOp(doc: PartDoc, op: EditOp): boolean {
  if (op.op === 'set') return COSMETIC_KEYS.includes(op.path.split('.')[0]);
  if (op.op !== 'replace') return false;
  const cur = doc.features.find((f) => f.id === op.id) as unknown as Record<string, unknown> | undefined;
  if (!cur) return false;
  const strip = (f: Record<string, unknown>) => JSON.stringify(Object.fromEntries(Object.entries(f).filter(([k]) => !COSMETIC_KEYS.includes(k))));
  return strip(cur) === strip(op.feature as unknown as Record<string, unknown>);
}

/** Can the reference check say anything about this batch? Not when it only appends features, and not when it only `set`s
 *  parameters of features at or after which no feature carries a face / edge name reference: a reference is compared
 *  against the body just before the feature that holds it, and that body only changes for features after the first
 *  edited one. (`remove`, `move`, `add` in the middle and `replace` always need the check.) */
export function refCheckNeeded(before: PartDoc, candidate: PartDoc, ops: EditOp[]): boolean {
  if (ops.every((o) => o.op === 'add' && (o.index === undefined || o.index >= before.features.length))) return false;
  if (ops.every((o) => isCosmeticOp(before, o))) return false;   // a label cannot re-point a reference
  if (!ops.every((o) => o.op === 'set')) return true;
  const first = Math.min(...ops.map((o) => before.features.findIndex((f) => f.id === (o as { id: string }).id)));
  if (!Number.isFinite(first) || first < 0) return true;
  const later = (d: PartDoc) => d.features.slice(first + 1).some(hasNameRefs);
  return later(before) || later(candidate);
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
      if (id !== undefined && !isId(id)) return `bad id ${JSON.stringify(id)}`;
      if (typeof id === 'string' && BAD_ID.test(id)) return `bad id ${id}`;
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
      // Definition order: sketch/seeds references must keep pointing backwards.
      const order = doc.features.map((f) => f.id);
      order.splice(order.indexOf(op.id), 1);
      order.splice(op.to, 0, op.id);
      const pos = new Map(order.map((id, i) => [id, i]));
      for (const f of doc.features) {
        const refs: string[] = [];
        const g = f as unknown as Record<string, unknown>;
        if (typeof g.sketch === 'string') refs.push(g.sketch);
        for (const k of ['sketches', 'seeds']) {
          if (Array.isArray(g[k])) refs.push(...(g[k] as unknown[]).filter((x): x is string => typeof x === 'string'));
        }
        for (const r of refs) {
          if (typeof r === 'string' && pos.has(r) && (pos.get(r) as number) > (pos.get(f.id) as number)) {
            return `moving ${op.id} to ${op.to} leaves ${f.id} referencing later feature ${r}`;
          }
        }
      }
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

/** Check a batch the way it will be applied: every op against the document as it is (a cascading remove may be followed by
 *  the removes of what it took along), plus ids added twice within the batch. Returns the first problem or null. */
export function checkBatch(doc: PartDoc, ops: EditOp[]): string | null {
  const added = new Set<string>();
  for (const op of ops) {
    const problem = checkOp(doc, op);
    if (problem) return problem;
    if (op.op === 'add') {
      const id = (op.feature as { id?: unknown }).id;
      if (typeof id === 'string') {
        if (added.has(id)) return `id ${id} already exists`;
        added.add(id);
      }
    }
  }
  return null;
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
      if (f.id === undefined) {
        normalizeSeq(doc);
        const used = new Set(doc.features.map((x) => x.id));
        while (used.has(`F${doc.seq}`)) doc.seq++;
        f.id = `F${doc.seq++}`;
      } else if (typeof f.id === 'string') {
        const n = seqOf(f.id);
        if (n !== null) doc.seq = Math.max(doc.seq, n + 1);
      }
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

/** The parts of a regen reply a Session keeps. */
interface RegenRes {
  errors: RegenError[];
  body: Record<string, unknown> | null;
  sketches: Array<[string, Record<string, unknown>]>;
  curves: Array<[string, { pts: number[][] }]>;
  planes: Array<[string, { o: number[]; x: number[]; y: number[]; z: number[] }]>;
  markers: unknown[];
  surfaces: unknown;
  threads: unknown[];
  warnings: unknown[];
}

export class Session {
  private kernel: import('../../driver/src/verify.js').KernelTransport;
  private worker?: { close(): Promise<void> | void };
  private doc: PartDoc;
  private tag: string;
  readonly author: string;
  private stack: Array<{ record: EditRecord; before: PartDoc }> = [];
  private editSeq = 1;
  /** The reply of the last plain regen of the document on this session's tag, valid while the kernel's tag state is still
   *  that document (any other regen on the tag drops it). Everything a viewer asks about the committed document after an
   *  edit (body, surfaces, markers, status, mass) is answered from it instead of asking the kernel again. */
  private memoMain: { key: string; res: RegenRes } | null = null;
  private massMemo: { key: string; val: { volume: number | null; solids: number } } | null = null;
  /** The reply of the last plain preview: committing the same ops asks the kernel only to move its tag (`noView`). */
  private memoPreview: { key: string; res: RegenRes } | null = null;
  /** When the session last started a request of its own (the deferred reference check waits for it to go quiet). */
  private lastActive = 0;
  /** Counts every request that reaches the kernel (the session's own and a viewer's, through `session.kernel`) except the
   *  reference check: a deferred check that sees it change has been overtaken by user work and is dropped. */
  private activity = 0;
  /** How long the session must have been idle before a deferred reference check (`edit(..., {lean: true})`) starts, ms.
   *  The kernel is one thread, so a check that starts runs to the end before the next request is answered. */
  refCheckIdleMs = 3000;

  constructor(doc: PartDoc, kernel: import('../../driver/src/verify.js').KernelTransport, author = 'agent', tag?: string) {
    this.doc = normalizeSeq(clone(doc));
    this.kernel = new Proxy(kernel, {
      get: (t, k, r) => {
        const v = Reflect.get(t, k, r);
        if (k !== 'request' || typeof v !== 'function') return v;
        return (msg: Record<string, unknown>) => {
          if (msg?.type !== 'refDiff') { this.activity++; this.lastActive = Date.now(); }
          return (v as (m: Record<string, unknown>) => Promise<unknown>).call(t, msg);
        };
      },
    });
    this.author = author;
    this.tag = tag ?? `sess${sessionSeq++}`;
  }

  /** The kernel for a new session: a local native service when `kernelUrl` is given or CK_KERNEL_URL (+ CK_KERNEL_TOKEN) is set
   *  (the environment wins over `wasmPath`, so suites that pass CK_WASM explicitly follow it too); otherwise the wasm kernel in a
   *  worker (CK_WASM or `wasmPath`). This is how the whole kernel test suite runs against a service. A service that is unreachable
   *  or from another build is an error here, never a silent wasm. */
  private static async openKernel(opts?: { wasmPath?: string; kernelUrl?: string; kernelToken?: string }): Promise<KernelTransport & { close(): Promise<void> | void }> {
    const env = typeof process !== 'undefined' ? process.env : ({} as Record<string, string | undefined>);
    const url = opts?.kernelUrl ?? env.CK_KERNEL_URL;
    if (url) {
      const { NativeKernel } = await import('../../kernel-client/src/native.js');
      return NativeKernel.connect({ url, token: opts?.kernelToken ?? env.CK_KERNEL_TOKEN ?? '' });
    }
    const { createWorkerKernel } = await import('../../kernel-client/src/worker.js');
    return createWorkerKernel(opts?.wasmPath);
  }

  static async open(doc?: PartDoc, opts?: { wasmPath?: string; author?: string; kernelUrl?: string; kernelToken?: string }): Promise<Session> {
    const worker = await Session.openKernel(opts);
    const s = new Session(doc ?? emptyPart('Part'), worker, opts?.author ?? 'agent');
    s.worker = worker;
    // Warm the session tag so body queries work immediately (best effort:
    // an empty or failing document simply leaves the tag without a body).
    await s.regen(s.document).catch(() => null);
    return s;
  }

  get document(): PartDoc {
    return clone(this.doc);
  }

  history(): EditRecord[] {
    return this.stack.map((e) => e.record);
  }

  private async regen(doc: PartDoc, tag = this.tag, opts?: { upTo?: number; sketchStatus?: boolean; solveSketches?: boolean }): Promise<RegenRes> {
    this.lastActive = Date.now();
    const mainPlain = tag === this.tag && !opts;
    const key = mainPlain ? JSON.stringify(doc) : '';
    if (mainPlain && this.memoMain?.key === key) return this.memoMain.res;
    if (tag === this.tag) { this.memoMain = null; this.massMemo = null; }   // the kernel's state of the tag is about to change
    const r = await this.kernel.request({ type: 'regen', tag, part: doc, ...(opts ? { opts } : {}) });
    if (r.trap || !r.ok || !r.result) throw new Error(r.trap ?? (typeof r.error === 'string' ? r.error : 'regen failed'));
    const res = r.result as {
      errors?: RegenError[];
      body?: Record<string, unknown> | null;
      sketches?: Array<[string, Record<string, unknown>]>;
      curves?: Array<[string, { pts: number[][] }]>;
      planes?: Array<[string, { o: number[]; x: number[]; y: number[]; z: number[] }]>;
      markers?: unknown[];
      surfaces?: unknown;
      threads?: unknown[];
      warnings?: unknown[];
    };
    const out: RegenRes = {
      errors: res.errors ?? [],
      body: res.body ?? null,
      sketches: res.sketches ?? [],
      curves: res.curves ?? [],
      planes: res.planes ?? [],
      markers: res.markers ?? [],
      surfaces: res.surfaces ?? [],
      threads: res.threads ?? [],
      warnings: res.warnings ?? [],
    };
    if (mainPlain) this.memoMain = { key, res: out };
    if (tag === `${this.tag}-preview`) this.memoPreview = opts ? null : { key: JSON.stringify(doc), res: out };
    return out;
  }

  /** Drop the kept regen replies: the next ask goes to the kernel again (a viewer that wraps the transport after open()
   *  and wants to see the first reply of the document). */
  forgetReplies(): void {
    this.memoMain = null;
    this.massMemo = null;
    this.memoPreview = null;
  }

  /** Errors with their repair hints, cosmetic thread markers, warnings and surface bodies of the current document: the
   *  regen of the document itself (answered from the last one when the kernel already holds it). No validate / check. */
  async regenReport(): Promise<{ errors: RegenError[]; hints: ReturnType<typeof hintFor>[]; threads: unknown[]; warnings: unknown[]; surfaces: unknown }> {
    const r = await this.regen(this.doc);
    return { errors: r.errors, hints: r.errors.map(([fid, err]) => hintFor(fid, err.code, err.params ?? {})), threads: r.threads, warnings: r.warnings, surfaces: r.surfaces };
  }

  private async mass(): Promise<{ volume: number | null; solids: number }> {
    const key = this.memoMain?.key;
    if (key && this.massMemo?.key === key) return this.massMemo.val;
    const val = await this.massRaw();
    if (key && this.memoMain?.key === key && val.volume !== null) this.massMemo = { key, val };
    return val;
  }

  private async massRaw(): Promise<{ volume: number | null; solids: number }> {
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

  async facts(): Promise<BodyFacts> {    const { body } = await this.regen(this.doc);
    const faces = ((body?.faces as Array<{ id: string }> | undefined) ?? []).map((f) => f.id);
    const edges = ((body?.edges as Array<{ key: string }> | undefined) ?? []).map((e) => e.key);
    const box = (body?.box as BodyFacts['box']) ?? null;
    const { volume, solids } = await this.mass();
    return { faces, edges, solids, volume, box };
  }

  /** Display mesh of the current document for the renderer. */
  async mesh(): Promise<import('../../render/src/mesh.js').MeshData> {
    const { meshFromBody } = await import('../../render/src/mesh.js');
    const { body } = await this.regen(this.doc);
    if (!body) throw new Error('no body to mesh');
    return meshFromBody(body as Parameters<typeof meshFromBody>[0]);
  }

  /** Display body for a viewer: mesh, edge polylines, the box, plus the sketches,
   *  curves, planes and markers of the current document (regen reply passthrough). */
  async bodyForView(): Promise<ViewBody | null> {
    const { body, sketches, curves, planes, markers } = await this.regen(this.doc);
    // No solid yet is not an empty document: a part that is only sketches, curves or planes still has
    // something to show, so hand the viewer an empty mesh with those layers instead of nothing.
    if (!body && !sketches.length && !curves.length && !planes.length) return null;
    const { meshFromBody } = await import('../../render/src/mesh.js');
    return {
      mesh: meshFromBody((body ?? {}) as Parameters<typeof meshFromBody>[0]),
      edges: (body?.edges as Array<{ key?: string; pts: number[][] }> | undefined) ?? [],
      box: (body?.box as { min: number[]; max: number[] } | undefined) ?? { min: [0, 0, 0], max: [0, 0, 0] },
      sketches: sketches.map(([id, e]) => ({
        id,
        frame: e.frame as ViewFrame,
        sketch: e.sketch as import('../../part-model/src/index.js').SketchFeature,
        ...(e.status ? { status: e.status as import('../../part-model/src/index.js').SketchSolverStatus } : {}),
      })),
      curves: curves.map(([id, e]) => ({ id, pts: e.pts })),
      planes: planes.map(([id, frame]) => ({ id, frame: frame as ViewFrame })),
      markers,
    };
  }

  private previewSeq = 0;

  /** Apply `ops` to a copy of the document and regen it on a scratch tag: the
   *  document, the edit stack and this session's tag are untouched. A newer
   *  preview supersedes an older one — the stale call resolves with `stale:true`
   *  and no body, so it can never overwrite a newer result. `opts.upTo` regens
   *  only the first N features (the kernel's sketch-mode partial regen). */
  async preview(ops: EditOp[], opts?: { upTo?: number; sketchStatus?: boolean; solveSketches?: boolean }):Promise<PreviewResult> {
    this.lastActive = Date.now();
    const seq = ++this.previewSeq;
    const problem = checkBatch(this.doc, ops);
    if (problem) return { ok: false, errors: [], hints: [], body: null, schemaError: problem };
    const candidate = clone(this.doc);
    try {
      for (const op of ops) applyOp(candidate, op);
    } catch (e) {
      return { ok: false, errors: [], hints: [], body: null, schemaError: String((e as Error).message ?? e) };
    }
    let regenRes: Awaited<ReturnType<Session['regen']>>;
    try {
      regenRes = await this.regen(candidate, `${this.tag}-preview`, opts);
    } catch (e) {
      return { ok: false, errors: [], hints: [{ feature: '', code: 'trap', hint: String((e as Error).message ?? e) }], body: null };
    }
    if (seq !== this.previewSeq) return { ok: true, errors: [], hints: [], body: null, stale: true };
    if (regenRes.errors.length) {
      return {
        ok: false,
        errors: regenRes.errors,
        hints: regenRes.errors.map(([fid, err]) => hintFor(fid, err.code, err.params ?? {})),
        body: null,
      };
    }
    const { body, sketches, curves, planes, markers } = regenRes;
    const viewSketches: ViewBody['sketches'] = sketches.map(([id, e]) => ({
      id,
      frame: e.frame as ViewFrame,
      sketch: e.sketch as import('../../part-model/src/index.js').SketchFeature,
      ...(e.status ? { status: e.status as import('../../part-model/src/index.js').SketchSolverStatus } : {}),
    }));
    const extras = { surfaces: regenRes.surfaces, threads: regenRes.threads, warnings: regenRes.warnings };
    if (!body) return { ok: true, errors: [], hints: [], body: null, sketches: viewSketches, ...extras };
    const { meshFromBody } = await import('../../render/src/mesh.js');
    return {
      ok: true,
      errors: [],
      hints: [],
      ...extras,
      body: {
        mesh: meshFromBody(body as Parameters<typeof meshFromBody>[0]),
        edges: (body.edges as Array<{ key?: string; pts: number[][] }> | undefined) ?? [],
        box: body.box as { min: number[]; max: number[] },
        sketches: sketches.map(([id, e]) => ({
          id,
          frame: e.frame as ViewFrame,
          sketch: e.sketch as import('../../part-model/src/index.js').SketchFeature,
          ...(e.status ? { status: e.status as import('../../part-model/src/index.js').SketchSolverStatus } : {}),
        })),
        curves: curves.map(([id, e]) => ({ id, pts: e.pts })),
        planes: planes.map(([id, frame]) => ({ id, frame: frame as ViewFrame })),
        markers,
      },
      sketches: viewSketches,
    };
  }

  /** Apply a batch atomically. `lean` is for an interactive viewer: no before / after body facts (`diff`), and the
   *  reference check is skipped for a batch that only appends features (nothing earlier can be re-pointed) or delivered
   *  later as `refWarningsLater`. */
  async edit(ops: EditOp[], reason?: string, opts?: { lean?: boolean; refCheck?: 'auto' | 'none' }): Promise<EditResult> {
    const lean = opts?.lean === true;
    const refCheckOpt = opts?.refCheck ?? 'auto';
    this.lastActive = Date.now();
    const problem = checkBatch(this.doc, ops);
    if (problem) return { ok: false, record: null, schemaError: problem };
    const before = clone(this.doc);
    const candidate = clone(this.doc);
    try {
      for (const op of ops) applyOp(candidate, op);
    } catch (e) {
      return { ok: false, record: null, schemaError: String((e as Error).message ?? e) };
    }

    const pre = lean ? null : await this.facts();
    let regenRes: { errors: RegenError[]; body: Record<string, unknown> | null };
    try {
      const key = JSON.stringify(candidate);
      const pv = this.memoPreview;
      if (pv && pv.key === key && pv.res.errors.length === 0) {
        // the user is committing what the last preview showed: the kernel only has to adopt the document on its tag
        this.memoMain = null; this.massMemo = null;
        const r = await this.kernel.request({ type: 'regen', tag: this.tag, part: candidate, opts: { noView: true } });
        if (r.trap || !r.ok || !r.result) throw new Error(r.trap ?? (typeof r.error === 'string' ? r.error : 'regen failed'));
        const errors = ((r.result as { errors?: RegenError[] }).errors ?? []) as RegenError[];
        regenRes = { ...pv.res, errors };
        if (errors.length === 0) this.memoMain = { key, res: regenRes as RegenRes };
      } else {
        regenRes = await this.regen(candidate);
      }
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
    let post: BodyFacts | null = null;
    if (!lean) {
      const { volume, solids } = await this.mass();
      post = { faces, edges, solids, volume, box };
    }

    let refWarnings: RefWarning[] = [];
    let refWarningsLater: Promise<RefWarning[]> | undefined;
    const needLater = lean && refCheckOpt === 'auto' && refCheckNeeded(before, candidate, ops);
    if (!lean && refCheckOpt !== 'none') refWarnings = await this.refDiffOf(before, candidate);

    this.doc = candidate;
    const record: EditRecord = { id: `edit-${this.editSeq++}`, author: this.author, time: Date.now(), ops: clone(ops), reason };
    this.stack.push({ record, before });

    // Intent: a document carrying expectations flags breaks automatically.
    // Flagged, not reverted — redesigning away from stated intent is legal.
    let intent: EditResult['intent'] = null;
    const hints: ReturnType<typeof hintFor>[] = [];
    if (candidate.expect?.length) {
      try {
        const c = await this.kernel.request({ type: 'check', tag: this.tag, expect: candidate.expect });
        if (!c.trap && c.ok && c.result) {
          const res = c.result as { passed: number; failed: number; results: CheckResultItem[] };
          intent = { passed: res.passed ?? 0, failed: res.failed ?? 0, results: res.results ?? [] };
          for (const item of intent.results) {
            if (!item.pass) {
              hints.push({
                feature: '',
                code: `expect:${item.kind}`,
                hint: `document expectation ${item.kind} broken by ${record.id} — measured ${JSON.stringify(item.measured)} vs expected ${JSON.stringify(item.expected)}${item.detail ? ` (${item.detail})` : ''}`,
              });
            }
          }
        }
      } catch { intent = null; }
    }
    const had = new Set(before.features.map((f) => f.id));
    const added = candidate.features.map((f) => f.id).filter((id) => !had.has(id));
    if (needLater) {
      // Advisory and slow: it starts only after the session has been idle for refCheckIdleMs and is dropped (resolves to
      // []) when anything else reached the kernel meanwhile; `refWarnings()` asks for it on demand.
      const seen = this.activity;
      refWarningsLater = (async () => {
        while (Date.now() - this.lastActive < this.refCheckIdleMs) {
          if (this.activity !== seen || this.closed) return [];
          await new Promise((r) => setTimeout(r, 200));
        }
        if (this.activity !== seen || this.closed) return [];
        return this.refDiffOf(before, candidate);
      })();
    }
    return { ok: true, record, added, diff: pre && post ? diffFacts(pre, post) : null, refWarnings, ...(refWarningsLater ? { refWarningsLater } : {}), intent, hints: hints.length ? hints : undefined };
  }

  private closed = false;

  private async refDiffOf(base: PartDoc, part: PartDoc): Promise<RefWarning[]> {
    try {
      const rd = await this.kernel.request({ type: 'refDiff', base, part });
      if (!rd.trap && rd.ok && rd.result) return ((rd.result as { warnings?: RefWarning[] }).warnings ?? []) as RefWarning[];
    } catch { /* no warnings */ }
    return [];
  }

  /** The reference check on demand: warnings for references that the last `n` applied batches re-pointed or lost (what a
   *  non-lean `edit` returns as `refWarnings`). It rebuilds the part from the first changed feature on, so it takes as long
   *  as that part of a regen; ask when the user is not waiting. */
  async refWarnings(n = 1): Promise<RefWarning[]> {
    const e = this.stack[this.stack.length - n];
    if (!e || n < 1) return [];
    return this.refDiffOf(e.before, this.doc);
  }

  /** Set the document's intent expectations (check grammar). Recorded like an
   *  edit so undo reverts it; does not touch the kernel (checked on edit/verify). */
  setExpect(expect: Array<Record<string, unknown>>, reason?: string): EditRecord {
    if (!Array.isArray(expect)) throw new Error('expect must be an array');
    const before = clone(this.doc);
    this.doc = { ...clone(this.doc), expect: clone(expect) };
    if (expect.length === 0) delete (this.doc as { expect?: unknown }).expect;
    const record: EditRecord = { id: `edit-${this.editSeq++}`, author: this.author, time: Date.now(), ops: [], reason: reason ?? 'set expectations' };
    this.stack.push({ record, before });
    return record;
  }

  /** Revert the last n applied batches. The document is the source of truth;   *  the kernel tag is re-synced (resynced:false if that regen itself fails). */
  async undo(n = 1): Promise<{ records: EditRecord[]; resynced: boolean }> {
    this.lastActive = Date.now();
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

  /** Save the document as name.ckpart plus the edit log as name.edits.json. */
  async save(dir: string, name?: string): Promise<{ part: string; log: string }> {
    const { writeFileSync, mkdirSync } = await import('node:fs');
    const { default: path } = await import('node:path');
    mkdirSync(dir, { recursive: true });
    const base = name ?? (this.doc.name.replace(/[^\w\- ]+/g, '_').trim() || 'part');
    const partFile = path.join(dir, `${base}.ckpart`);
    const logFile = path.join(dir, `${base}.edits.json`);
    writeFileSync(partFile, JSON.stringify(this.doc, null, 1));
    writeFileSync(logFile, JSON.stringify({
      format: 'ck-edit-log',
      version: 1,
      base: this.stack.length ? this.stack[0].before : this.doc,
      entries: this.stack.map((e) => ({ record: e.record, before: e.before })),
    }, null, 1));
    return { part: partFile, log: logFile };
  }

  /** Reopen a saved session: document, records and undo states restored. */
  static async openSaved(dir: string, name: string, opts?: { wasmPath?: string; kernelUrl?: string; kernelToken?: string }): Promise<Session> {
    const { readFileSync } = await import('node:fs');
    const { default: path } = await import('node:path');
    const doc = JSON.parse(readFileSync(path.join(dir, `${name}.ckpart`), 'utf8')) as PartDoc;
    const log = JSON.parse(readFileSync(path.join(dir, `${name}.edits.json`), 'utf8')) as {
      format: string; version: number; entries: Array<{ record: EditRecord; before: PartDoc }>;
    };
    if (log.format !== 'ck-edit-log' || log.version !== 1) throw new Error(`unreadable edit log ${name}`);
    const worker = await Session.openKernel(opts);
    const s = new Session(doc, worker, log.entries.at(-1)?.record.author ?? 'agent');
    s.worker = worker;
    s.editSeq = log.entries.length + 1;
    s.stack = log.entries.map((e) => ({ record: e.record, before: clone(e.before) }));
    await s.regen(s.document).catch(() => null);
    return s;
  }

  /** The spec's query tools, bound to this session's tag. */
  check(expect: Array<Record<string, unknown>> | Record<string, unknown>, tol?: number): Promise<CheckReport> {
    return check(this.kernel, expect, { tag: this.tag, tol });
  }

  resolve(descriptions: Array<Record<string, unknown>>): Promise<ResolveMatch[]> {
    return resolveDescriptions(this.kernel, descriptions, { tag: this.tag });
  }

  describe(names: unknown[]): Promise<DescribeItem[]> {
    return describe(this.kernel, names, { tag: this.tag });
  }

  /** Advance width and ink box of a string (stateless; docs/text.md). */
  textMetrics(string: string, opts: { font?: string; height: number; spacing?: number }) {
    return textMetrics(this.kernel, string, opts);
  }

  /** Loops of a text entity as polylines (stateless; docs/text.md). */
  textOutline(spec: Record<string, unknown>) {
    return textOutline(this.kernel, spec);
  }

  /** Register a TTF / OTF (base64) for lettering under `name`; the `text` entity's `font` then accepts the name (docs/text.md). */
  loadFont(name: string, data: string, opts: { replace?: boolean } = {}) {
    return loadFont(this.kernel, name, data, opts);
  }

  unloadFont(name: string) {
    return unloadFont(this.kernel, name);
  }

  listFonts() {
    return listFonts(this.kernel);
  }

  /** TrueType bytes (base64) of a loaded font reduced to the glyphs of `text`. */
  subsetFont(name: string, text: string) {
    return subsetFont(this.kernel, name, text);
  }

  /** Thread standards and sizes (stateless; see docs/spec-agent-interface.md). */
  threadData(standard?: string, size?: string) {
    return threadData(this.kernel, standard, size);
  }

  /** Design-rule profiles (fdm-print, cnc-3axis, injection): the list with rule parameters. */
  profiles() {
    return profiles(this.kernel);
  }

  /** Run a design-rule profile on the current body: issues name faces / edges by persistent name. */
  checkProfile(profile: string | Record<string, unknown>, opts?: { samples?: number; maxIssues?: number }) {
    return checkProfile(this.kernel, profile, { tag: this.tag, opts });
  }

  validate(tol?: number): Promise<{ ok: boolean; issues: ValidateIssue[] }> {
    return validate(this.kernel, { tag: this.tag, tol });
  }

  heal(opts?: { damage?: string; dryRun?: boolean; tol?: number }): Promise<HealReport> {
    return heal(this.kernel, { tag: this.tag, ...opts });
  }

  massProps(opts?: { bodies?: boolean; densities?: number[] }): Promise<MassReport> {
    return massProps(this.kernel, { tag: this.tag, ...opts });
  }

  /** Exact cross-section loops at a plane. */
  section(plane: { origin: number[]; normal: number[] }): Promise<SectionReport> {
    return section(this.kernel, plane, { tag: this.tag });
  }

  /** Section stack along an axis with the integrated volume checked against massProps. */
  sections(opts?: { axis?: 'x' | 'y' | 'z' | number[]; n?: number; positions?: number[]; exact?: boolean }): Promise<SectionsReport> {
    return sections(this.kernel, { tag: this.tag, ...opts });
  }

  /** Inside / signed distance of points and hit lists of rays. */
  probe(q: { points?: number[][]; rays?: Array<{ origin: number[]; dir: number[] }> }): Promise<ProbeReport> {
    return probe(this.kernel, q, { tag: this.tag });
  }

  /** Minimum distance of two entities (face id | edge key | V<n> | lump:<n> | body | [x, y, z]); `tagB` measures to another tag's body. */
  distance(a: MeasureEntity, b: MeasureEntity, o?: { tagB?: string; samples?: number }): Promise<DistanceReport> {
    return distance(this.kernel, a, b, { tag: this.tag, ...o });
  }

  /** One-sided Hausdorff distance a to b (default whole bodies) with an error bound. */
  maxDistance(q?: { a?: MeasureEntity; b?: MeasureEntity; tagB?: string; samples?: number; both?: false }): Promise<MaxDistanceReport>;
  maxDistance(q: { a?: MeasureEntity; b?: MeasureEntity; tagB?: string; samples?: number; both: true }): Promise<HausdorffReport>;
  maxDistance(q: { a?: MeasureEntity; b?: MeasureEntity; tagB?: string; samples?: number; both?: boolean } = {}): Promise<MaxDistanceReport | HausdorffReport> {
    return maxDistance(this.kernel, q as { both: true }, { tag: this.tag });
  }

  /** Minimum wall thickness and the thinnest places. */
  thickness(q?: Parameters<typeof thickness>[1]): Promise<ThicknessReport> {
    return thickness(this.kernel, q, { tag: this.tag });
  }

  thicknessAt(q: { point: number[] } | { face: string; samples?: number }): Promise<ThicknessAtReport> {
    return thicknessAt(this.kernel, q, { tag: this.tag });
  }

  /** Closest face / edge / vertex to many points. */
  closestPoint(points: number[][], q?: Parameters<typeof closestPoint>[2]): Promise<ClosestPointReport> {
    return closestPoint(this.kernel, points, q, { tag: this.tag });
  }

  /** Dihedral angles of edges / faces and draft angles. */
  angles(q: Parameters<typeof angles>[1]): Promise<AnglesReport> {
    return angles(this.kernel, q, { tag: this.tag });
  }

  /** Face pairs closer than `min`. */
  clearance(q: Parameters<typeof clearance>[1]): Promise<ClearanceReport> {
    return clearance(this.kernel, q, { tag: this.tag });
  }

  /** Mirror-symmetry planes with the sampled deviation. */
  symmetry(q?: Parameters<typeof symmetry>[1]): Promise<SymmetryReport> {
    return symmetry(this.kernel, q, { tag: this.tag });
  }

  /** Lowest and highest point of the body (or `of`) along a direction. */
  extrema(q: Parameters<typeof extrema>[1]): Promise<ExtremaReport> {
    return extrema(this.kernel, q, { tag: this.tag });
  }

  /** The body meshed at an export quality / chord height (the display mesh is untouched). */
  exportMesh(o: ExportMeshOpts = {}): Promise<ExportMeshReport> {
    return exportMesh(this.kernel, o, { tag: this.tag });
  }

  step(name?: string): Promise<string> {
    return step(this.kernel, name ?? this.doc.name, { tag: this.tag });
  }

  importStep(data: string, name?: string): Promise<ImportReport> {
    return importStep(this.kernel, data, { name });
  }

  /** A SolidWorks .SLDPRT part: `data` is the file bytes as base64. */
  importSldprt(data: string, name?: string): Promise<ImportReport> {
    return importSldprt(this.kernel, data, { name });
  }

  /** A SketchUp .skp file: `data` is the file bytes as base64; hidden geometry is skipped unless `hidden`. */
  importSkp(data: string, name?: string, opts: { hidden?: boolean; maxFaces?: number; up?: SkpUp } = {}): Promise<ImportReport> {
    return importSkp(this.kernel, data, { name, ...opts });
  }

  importSat(data: string, name?: string): Promise<ImportReport> {
    return importSat(this.kernel, data, { name });
  }

  async close(): Promise<void> {
    this.closed = true;
    await this.worker?.close();
  }
}

/** Log file shape written by save(). */
export interface EditLogFile {
  format: 'ck-edit-log';
  version: 1;
  base: PartDoc;
  entries: Array<{ record: EditRecord; before: PartDoc }>;
}

/** Replay a saved log onto a session step by step (for "step through the
 *  agent's edits"). The session should hold the log's base document first;
 *  each step applies one record's ops. Stops at the first failure. */
export async function replaySteps(
  session: Session,
  log: EditLogFile,
  onStep?: (record: EditRecord, result: EditResult) => void,
): Promise<{ applied: number; failed: EditRecord | null; result: EditResult | null }> {
  let applied = 0;
  for (const e of log.entries) {
    const result = await session.edit(e.record.ops, e.record.reason);
    onStep?.(e.record, result);
    if (!result.ok) return { applied, failed: e.record, result };
    applied++;
  }
  return { applied, failed: null, result: null };
}
