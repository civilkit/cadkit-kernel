// Typed client for the kernel's agent queries (docs/agent-queries.md):
// check, resolve, describe, validate, heal, massProps, importStep, importSat.
// Every wrapper throws on trap / envelope error / null result so callers only
// handle real replies. This finishes the spec's tool table alongside Session
// (open/read/edit/history/undo) and verify.
import type { KernelTransport } from '../../driver/src/verify.js';

export interface QueryOpts {
  tag?: string;
  tol?: number;
}

async function call<T>(kernel: KernelTransport, msg: Record<string, unknown>): Promise<T> {
  const r = await kernel.request(msg);
  if (r.trap) throw new Error(`trap: ${r.trap}`);
  if (!r.ok) throw new Error(typeof r.error === 'string' ? r.error : 'request failed');
  if (r.result === undefined || r.result === null) throw new Error(`empty result for ${msg.type}`);
  return r.result as T;
}

export interface CheckItem {
  index: number;
  id?: string;
  kind: string;
  pass: boolean;
  measured?: unknown;
  expected?: unknown;
  detail?: string;
}

export interface CheckReport {
  passed: number;
  failed: number;
  results: CheckItem[];
  measured?: Record<string, unknown>;
}

/** Expectations: array form or the shorthand object ({volume:1000, faces:7}). */
export async function check(
  kernel: KernelTransport,
  expect: Array<Record<string, unknown>> | Record<string, unknown>,
  opts: QueryOpts = {},
): Promise<CheckReport> {
  const msg: Record<string, unknown> = { type: 'check', tag: opts.tag ?? 'main', expect };
  if (opts.tol !== undefined) msg.tol = opts.tol;
  const res = await call<{ passed: number; failed: number; results: CheckItem[]; measured?: Record<string, unknown> }>(kernel, msg);
  return { passed: res.passed ?? 0, failed: res.failed ?? 0, results: res.results ?? [], measured: res.measured };
}

export interface ResolveMatch {
  description: unknown;
  status: 'unique' | 'ambiguous' | 'none' | 'invalid' | string;
  count?: number;
  matches?: unknown[];
  hint?: string;
  error?: string;
}

export async function resolveDescriptions(
  kernel: KernelTransport,
  descriptions: Array<Record<string, unknown>>,
  opts: QueryOpts = {},
): Promise<ResolveMatch[]> {
  const res = await call<{ results: ResolveMatch[] }>(kernel, { type: 'resolve', tag: opts.tag ?? 'main', descriptions });
  return res.results ?? [];
}

export interface DescribeItem {
  found: boolean;
  text?: string;
  error?: string;
  [key: string]: unknown;
}

export async function describe(
  kernel: KernelTransport,
  names: unknown[],
  opts: QueryOpts = {},
): Promise<DescribeItem[]> {
  const res = await call<{ items: DescribeItem[] }>(kernel, { type: 'describe', tag: opts.tag ?? 'main', names });
  return res.items ?? [];
}

export interface ThreadSize {
  size: string;
  standard: string;
  major: number;
  pitch: number;
  tpi: number;
  minorInternal: number;
  minorExternal: number;
  pitchDiameter: number;
  angle: number;
  tapDrill: number;
  tapDrillName?: string;
  classInternal: string;
  classExternal: string;
  tapered: boolean;
}

export interface ThreadStandardInfo {
  id: string;
  description: string;
  count: number;
  sizes: string[];
}

/** Thread tables for dropdowns (stateless). No standard: the standards; with one: its sizes with all numbers (mm). */
export async function threadData(kernel: KernelTransport, standard?: string, size?: string): Promise<{ standards?: ThreadStandardInfo[]; standard?: string; units: string; sizes?: ThreadSize[] }> {
  const msg: Record<string, unknown> = { type: 'threadData' };
  if (standard !== undefined) msg.standard = standard;
  if (size !== undefined) msg.size = size;
  return call(kernel, msg);
}

export interface ProfileIssue {
  rule: string;
  severity: 'error' | 'warn' | 'info';
  message: string;
  /** Persistent face names (heat-map / highlight targets). */
  faces: string[];
  /** Persistent edge keys. */
  edges: string[];
  value: number | null;
  limit: number | null;
  unit: string;
  /** A point on the part to focus the viewport on. */
  at?: [number, number, number];
}

export interface ProfileReport {
  profile: { id: string; name: string; rules?: unknown[] };
  ok: boolean;
  issues: ProfileIssue[];
  summary: {
    errors: number; warnings: number; infos: number; total: number;
    rules: Array<{ rule: string; severity: string; issues: number; passed: boolean }>;
    truncated: Array<{ rule: string; total: number; shown: number }>;
    skippedFaces: string[];
    samples: number;
  };
}

export interface ProfileInfo {
  id: string;
  name: string;
  description: string;
  rules: Array<{ id: string; title: string; description: string; severity: string; params: Array<{ name: string; type: 'number' | 'vec3'; default: unknown; value: unknown; unit: string; min?: number; max?: number; description: string }> }>;
}

/** Built-in design-rule profiles and the rule catalogue (stateless). */
export async function profiles(kernel: KernelTransport): Promise<{ profiles: ProfileInfo[]; rules: ProfileInfo['rules'] }> {
  return call(kernel, { type: 'profiles' });
}

/** Run a profile (built-in id or inline `{rules:[{id, params?, severity?}]}`) on the tag's body. Read-only. */
export async function checkProfile(
  kernel: KernelTransport,
  profile: string | Record<string, unknown>,
  opts: QueryOpts & { opts?: { samples?: number; maxIssues?: number } } = {},
): Promise<ProfileReport> {
  const msg: Record<string, unknown> = { type: 'checkProfile', tag: opts.tag ?? 'main', profile };
  if (opts.opts) msg.opts = opts.opts;
  return call(kernel, msg);
}

export interface ValidateIssue {
  code: string;
  entity: string;
  detail: string;
}

export async function validate(kernel: KernelTransport, opts: QueryOpts = {}): Promise<{ ok: boolean; issues: ValidateIssue[] }> {
  const msg: Record<string, unknown> = { type: 'validate', tag: opts.tag ?? 'main' };
  if (opts.tol !== undefined) msg.tol = opts.tol;
  const res = await call<{ ok: boolean; issues: ValidateIssue[] }>(kernel, msg);
  return { ok: res.ok, issues: res.issues ?? [] };
}

export interface HealReport {
  fixed: unknown[];
  remaining: unknown[];
  volume?: { before: number; after: number };
  [key: string]: unknown;
}

export async function heal(kernel: KernelTransport, opts: QueryOpts & { damage?: string; dryRun?: boolean } = {}): Promise<HealReport> {
  const msg: Record<string, unknown> = { type: 'heal', tag: opts.tag ?? 'main' };
  if (opts.tol !== undefined) msg.tol = opts.tol;
  if (opts.damage !== undefined) msg.damage = opts.damage;
  if (opts.dryRun !== undefined) msg.dryRun = opts.dryRun;
  const res = await call<HealReport>(kernel, msg);
  return { ...res, fixed: res.fixed ?? [], remaining: res.remaining ?? [] };
}

export interface MassReport {
  volume?: number;
  area?: number;
  com?: number[];
  bodies?: Array<{ volume: number; area: number; com: number[] }>;
  [key: string]: unknown;
}

export async function massProps(
  kernel: KernelTransport,
  opts: QueryOpts & { bodies?: boolean; densities?: number[] } = {},
): Promise<MassReport> {
  const msg: Record<string, unknown> = { type: 'massProps', tag: opts.tag ?? 'main' };
  if (opts.bodies !== undefined) msg.bodies = opts.bodies;
  if (opts.densities !== undefined) msg.densities = opts.densities;
  return call<MassReport>(kernel, msg);
}

export type SectionEdge =
  | { type: 'line'; from: [number, number]; to: [number, number]; face: string }
  | { type: 'arc'; centre: [number, number]; radius: number; start: [number, number]; end: [number, number]; startAngle: number; sweep: number; ccw: boolean; face: string }
  | { type: 'polyline'; pts: Array<[number, number]>; face: string };

export interface SectionLoop {
  kind: 'outer' | 'inner';
  closed: true;
  area: number;
  edges: SectionEdge[];
  pts: Array<[number, number]>;
}

export interface SectionReport {
  loops: SectionLoop[];
  frame: { o: number[]; x: number[]; y: number[]; z: number[] };
  /** Signed sum of the loop areas: outer minus inner. */
  area: number;
  bbox2d: [number, number, number, number];
  openChains?: number;
}

/** Exact cross-section loops of the tag's body at a plane. */
export async function section(
  kernel: KernelTransport,
  plane: { origin: number[]; normal: number[] },
  opts: QueryOpts = {},
): Promise<SectionReport> {
  return call<SectionReport>(kernel, { type: 'section', tag: opts.tag ?? 'main', plane });
}

export interface SectionsReport {
  axis: number[];
  range: [number, number];
  rule: 'midpoint' | 'trapezoid';
  positions: number[];
  areas: number[];
  loops: number[];
  /** Volume integrated from the areas, the exact massProps volume and their relative difference. */
  volume: number;
  /** Absent with `exact: false` (massProps skipped). */
  exactVolume?: number;
  volumeError?: number;
}

/** A stack of sections along an axis (default 21 cell centres) and the volume they integrate to. */
export async function sections(
  kernel: KernelTransport,
  opts: QueryOpts & { axis?: 'x' | 'y' | 'z' | number[]; n?: number; positions?: number[]; exact?: boolean } = {},
): Promise<SectionsReport> {
  const msg: Record<string, unknown> = { type: 'sections', tag: opts.tag ?? 'main' };
  if (opts.axis !== undefined) msg.axis = opts.axis;
  if (opts.n !== undefined) msg.n = opts.n;
  if (opts.positions !== undefined) msg.positions = opts.positions;
  if (opts.exact !== undefined) msg.exact = opts.exact;
  return call<SectionsReport>(kernel, msg);
}

export interface ProbePointResult {
  inside: boolean | 'boundary';
  /** Signed distance to the nearest face (negative inside). */
  distance: number | null;
  face: string | null;
  nearest: number[] | null;
}

export interface ProbeRayHit {
  t: number;
  point: number[];
  face: string;
  entering: boolean;
  tangent?: boolean;
}

export interface ProbeReport {
  points?: ProbePointResult[];
  rays?: Array<{ hits: ProbeRayHit[] }>;
}

/** Point containment / signed distance and ray hit lists (at most 2000 of each per call). */
export async function probe(
  kernel: KernelTransport,
  q: { points?: number[][]; rays?: Array<{ origin: number[]; dir: number[] }> },
  opts: QueryOpts = {},
): Promise<ProbeReport> {
  return call<ProbeReport>(kernel, { type: 'probe', tag: opts.tag ?? 'main', ...q });
}

export interface ImportReport {
  errors: Array<[string, { code: string; params: Record<string, unknown> }]>;
  body?: { faces?: unknown[]; edges?: unknown[]; box?: { min: number[]; max: number[] } } | null;
  [key: string]: unknown;
}

/** STEP export of the tag's body as text. */
export async function step(kernel: KernelTransport, name: string, opts: QueryOpts = {}): Promise<string> {
  const res = await call<string>(kernel, { type: 'step', tag: opts.tag ?? 'main', name });
  if (typeof res !== 'string') throw new Error('step did not return text');
  return res;
}

/** Import STEP/SAT text into a fresh tag; reply is regen-shaped. */export async function importStep(
  kernel: KernelTransport,
  data: string,
  opts: { name?: string; tag?: string } = {},
): Promise<ImportReport> {
  const res = await call<ImportReport>(kernel, { type: 'importStep', tag: opts.tag ?? 'imp', name: opts.name ?? 'import', data });
  return { ...res, errors: res.errors ?? [] };
}

export async function importSat(
  kernel: KernelTransport,
  data: string,
  opts: { name?: string; tag?: string } = {},
): Promise<ImportReport> {
  const res = await call<ImportReport>(kernel, { type: 'importSat', tag: opts.tag ?? 'imp', name: opts.name ?? 'import', data });
  return { ...res, errors: res.errors ?? [] };
}
