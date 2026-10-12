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
