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

export interface TextMetrics {
  /** pen advance of the whole string */
  advance: number;
  /** ink box relative to the pen start on the baseline: [xmin, ymin, xmax, ymax] */
  bbox: [number, number, number, number];
  width: number;
  capHeight: number;
  chars: number;
  advances: number[];
  warnings: Array<{ code: string; detail: string }>;
}

/** Advance width and ink box of a string, to centre and size lettering (stateless; docs/text.md). */
export async function textMetrics(kernel: KernelTransport, string: string, opts: { font?: string; height: number; spacing?: number }): Promise<TextMetrics> {
  return call(kernel, { type: 'textMetrics', string, ...opts });
}

export interface TextOutline {
  regions: Array<{ outer: Array<[number, number]>; holes: Array<Array<[number, number]>> }>;
  bbox: [number, number, number, number];
  area: number;
  warnings: Array<{ code: string; detail: string }>;
}

/** The loops of a text entity as polylines, for a viewport preview (stateless; docs/text.md). `spec` = the entity fields. */
export async function textOutline(kernel: KernelTransport, spec: Record<string, unknown>): Promise<TextOutline> {
  return call(kernel, { type: 'textOutline', ...spec });
}

export interface FontInfo {
  name: string;
  /** SHA-256 of the file (hex); an entity may pin a prefix of it as `fontHash` */
  hash: string;
  builtin: boolean;
  family: string | null;
  bytes: number;
  glyphs: number;
  /** number of mapped characters */
  chars: number;
  unitsPerEm: number;
  capHeight: number;
  outlines: 'glyf' | 'cff';
  /** Unicode blocks with at least one mapped character */
  supported: string[];
  coverage: Array<{ block: string; mapped: number; of: number }>;
  warnings: Array<{ code: string; detail: string }>;
  /** loaded fonts only: the same name already held the same file / another file that was replaced */
  alreadyLoaded?: boolean;
  replaced?: boolean;
}

/** Register a TTF / OTF (base64) under `name` for the session; docs/text.md "Loading fonts". Errors: fontNameConflict, fontReserved, fontBadName, fontBadData, fontTooLarge, fontTooMany. */
export async function loadFont(kernel: KernelTransport, name: string, data: string, opts: { replace?: boolean } = {}): Promise<FontInfo> {
  return call(kernel, { type: 'loadFont', name, data, ...opts });
}

export async function unloadFont(kernel: KernelTransport, name: string): Promise<{ name: string; unloaded: boolean }> {
  return call(kernel, { type: 'unloadFont', name });
}

/** The built-in fonts, then the loaded ones. */
export async function listFonts(kernel: KernelTransport): Promise<{ fonts: FontInfo[] }> {
  return call(kernel, { type: 'listFonts' });
}

/** A plain TrueType font (base64 in `data`) with just the glyphs of `text` (for embedding in a document). */
export async function subsetFont(kernel: KernelTransport, name: string, text: string): Promise<{ name: string; hash: string; bytes: number; glyphs: number; data: string; warnings: Array<{ code: string; detail: string }> }> {
  return call(kernel, { type: 'subsetFont', name, text });
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
  /** principal moments of inertia (ascending, unit density) and their unit axes */
  principal?: number[];
  axes?: number[][];
  /** integral of x dV about the origin (volume times com; mass times com in byMass) */
  firstMoment?: number[];
  /** full 3x3 inertia tensor (rows, world axes, unit density) about the centre of mass and about the origin */
  inertia?: number[][];
  inertiaOrigin?: number[][];
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

// ---------------------------------------------------------------- measurement and analysis (read-only)

/** An entity: face id | edge key | `V<n>` | `lump:<n>` | `body` | [x, y, z] | `{face|edge|vertex|lump|point}`. */
export type MeasureEntity = string | number[] | Record<string, unknown> | Array<string | number[] | Record<string, unknown>>;

export interface EntityRef {
  kind: 'face' | 'edge' | 'vertex' | 'point';
  id?: string;
}

export interface DistanceReport {
  /** Minimum distance (0 when the entities touch or one solid lies inside the other). */
  distance: number;
  pointA: number[];
  pointB: number[];
  /** The face / edge / vertex the closest point lies on. */
  a: EntityRef;
  b: EntityRef;
  intersects: boolean;
  intersection?: { kind: 'point' | 'curve' | 'area' | 'containment' };
  /** Unit vector A to B, null when the distance is 0. */
  direction: number[] | null;
  boundaryDistance?: number;
}

/** Minimum distance of two entities of the tag's body (or `b` in another tag via `tagB`). */
export async function distance(
  kernel: KernelTransport,
  a: MeasureEntity,
  b: MeasureEntity,
  opts: QueryOpts & { tagB?: string; samples?: number } = {},
): Promise<DistanceReport> {
  const msg: Record<string, unknown> = { type: 'distance', tag: opts.tag ?? 'main', a, b };
  if (opts.tagB !== undefined) msg.tagB = opts.tagB;
  if (opts.samples !== undefined) msg.opts = { samples: opts.samples };
  return call<DistanceReport>(kernel, msg);
}

export interface MaxDistanceReport {
  maxDistance: number;
  location: number[];
  nearest: number[];
  /** The true one-sided distance is at most `maxDistance + errorBound`. */
  errorBound: number;
  upperBound: number;
  rms: number;
  samples: number;
  face?: string;
}

export interface HausdorffReport {
  hausdorff: number;
  errorBound: number;
  aToB: MaxDistanceReport;
  bToA: MaxDistanceReport;
}

/** One-sided maximum of the minimum distance from `a` to `b` (default: whole bodies), by dense sampling. */
export async function maxDistance(
  kernel: KernelTransport,
  q: { a?: MeasureEntity; b?: MeasureEntity; tagB?: string; samples?: number; both?: false },
  opts?: QueryOpts,
): Promise<MaxDistanceReport>;
export async function maxDistance(
  kernel: KernelTransport,
  q: { a?: MeasureEntity; b?: MeasureEntity; tagB?: string; samples?: number; both: true },
  opts?: QueryOpts,
): Promise<HausdorffReport>;
export async function maxDistance(
  kernel: KernelTransport,
  q: { a?: MeasureEntity; b?: MeasureEntity; tagB?: string; samples?: number; both?: boolean } = {},
  opts: QueryOpts = {},
): Promise<MaxDistanceReport | HausdorffReport> {
  const msg: Record<string, unknown> = { type: 'maxDistance', tag: opts.tag ?? 'main' };
  if (q.a !== undefined) msg.a = q.a;
  if (q.b !== undefined) msg.b = q.b;
  if (q.tagB !== undefined) msg.tagB = q.tagB;
  const o: Record<string, unknown> = {};
  if (q.samples !== undefined) o.samples = q.samples;
  if (q.both !== undefined) o.both = q.both;
  if (Object.keys(o).length) msg.opts = o;
  return call<MaxDistanceReport | HausdorffReport>(kernel, msg);
}

export interface ThicknessSample {
  thickness: number;
  point: number[];
  farPoint: number[];
  face: string;
  farFace: string;
  direction: number[];
}

export interface ThicknessReport {
  method: 'ray' | 'sphere';
  samples: number;
  noExit: number;
  minThickness: number | null;
  location?: ThicknessSample;
  meanThickness?: number;
  clusterRadius?: number;
  /** The thinnest places, thinnest first, at least `clusterRadius` apart. */
  thin: ThicknessSample[];
}

/** Minimum wall thickness (ray cast along the inward normal; `method: 'sphere'` refines with inscribed balls). */
export async function thickness(
  kernel: KernelTransport,
  q: { samples?: number; top?: number; radius?: number; below?: number; method?: 'ray' | 'sphere'; sphereSamples?: number } = {},
  opts: QueryOpts = {},
): Promise<ThicknessReport> {
  return call<ThicknessReport>(kernel, { type: 'thickness', tag: opts.tag ?? 'main', opts: q });
}

export type ThicknessAtReport =
  | { distanceToSurface: number; result: ThicknessSample }
  | { min: ThicknessSample; max: number; mean: number; samples: number };

/** Thickness at the surface point nearest to `point`, or min / max / mean over a face. */
export async function thicknessAt(
  kernel: KernelTransport,
  q: { point: number[] } | { face: string; samples?: number },
  opts: QueryOpts = {},
): Promise<ThicknessAtReport> {
  return call<ThicknessAtReport>(kernel, { type: 'thicknessAt', tag: opts.tag ?? 'main', ...q });
}

export interface ClosestPointResult {
  distance: number;
  point: number[];
  entity: EntityRef & { face?: string };
  uv?: number[];
  t?: number;
  /** `on: 'body'` only: signed distance (negative inside) and the unwrapped nearest hit. */
  signedDistance?: number;
  inside?: boolean;
  nearest?: ClosestPointResult;
}

export interface ClosestPointReport {
  on: 'body' | 'face' | 'edge' | 'vertex';
  results: Array<ClosestPointResult | ClosestPointResult[] | null>;
}

/** Closest face / edge / vertex of the body to many points (at most 2000). `k` > 1 lists the k nearest entities. */
export async function closestPoint(
  kernel: KernelTransport,
  points: number[][],
  q: { on?: 'body' | 'face' | 'edge' | 'vertex'; ids?: MeasureEntity; k?: number } = {},
  opts: QueryOpts = {},
): Promise<ClosestPointReport> {
  return call<ClosestPointReport>(kernel, { type: 'closestPoint', tag: opts.tag ?? 'main', points, ...q });
}

export interface EdgeAngle {
  edge: string;
  faces: string[];
  /** Interior dihedral angle in degrees, through the material (90 for a box edge, 270 for a re-entrant one). */
  angle: number;
  normalAngle: number;
  type: 'convex' | 'concave' | 'smooth';
  min: number;
  max: number;
  at: number[];
}

export interface DraftFace {
  face: string;
  draft: number;
  min: number;
  max: number;
  class: 'ok' | 'vertical' | 'undercut' | 'parting';
}

export interface AnglesReport {
  edges?: EdgeAngle[];
  between?: { angle?: number; normalAngle: number; type?: string; sharedEdge?: string; distance?: number; parallel?: boolean };
  draft?: DraftFace[];
  violations?: string[];
}

/** Dihedral angles of edges (`'all'` or keys), the angle between two faces, and draft angles against a pull direction. */
export async function angles(
  kernel: KernelTransport,
  q: { edges?: 'all' | string[]; faces?: [string, string]; draft?: { pull: number[]; faces?: 'all' | string[]; min?: number } },
  opts: QueryOpts = {},
): Promise<AnglesReport> {
  return call<AnglesReport>(kernel, { type: 'angles', tag: opts.tag ?? 'main', ...q });
}

export interface ClearancePair {
  a: string;
  b: string;
  distance: number;
  /** `gap`: air between the faces; `wall`: material between them (a thin wall); `contact`. */
  kind: 'gap' | 'wall' | 'contact';
  pointA: number[];
  pointB: number[];
}

export interface ClearanceReport {
  min: number;
  count: number;
  pairsTested: number;
  truncated: boolean;
  pairs: ClearancePair[];
}

/** Face pairs closer than `min` (non-adjacent), sorted by distance. */
export async function clearance(
  kernel: KernelTransport,
  q: { min: number; a?: MeasureEntity; b?: MeasureEntity; include?: 'gap' | 'wall' | 'both'; limit?: number },
  opts: QueryOpts = {},
): Promise<ClearanceReport> {
  const { include, limit, ...rest } = q;
  const msg: Record<string, unknown> = { type: 'clearance', tag: opts.tag ?? 'main', ...rest };
  const o: Record<string, unknown> = {};
  if (include !== undefined) o.include = include;
  if (limit !== undefined) o.limit = limit;
  if (Object.keys(o).length) msg.opts = o;
  return call<ClearanceReport>(kernel, msg);
}

export interface SymmetryPlane {
  origin: number[];
  normal: number[];
  basis: string;
  maxDeviation: number;
  rmsDeviation: number;
  symmetric: boolean;
  worstPoint: number[];
}

export interface SymmetryReport {
  tol: number;
  samples: number;
  symmetric: SymmetryPlane[];
  best: SymmetryPlane | null;
  planes: SymmetryPlane[];
}

/** Mirror symmetry: candidate planes (coordinate and principal axes through the centre of mass, or `planes`) with the sampled deviation of the mirrored surface. */
export async function symmetry(
  kernel: KernelTransport,
  q: { planes?: Array<{ origin: number[]; normal: number[] }>; samples?: number; tol?: number } = {},
  opts: QueryOpts = {},
): Promise<SymmetryReport> {
  const msg: Record<string, unknown> = { type: 'symmetry', tag: opts.tag ?? 'main' };
  if (q.planes !== undefined) msg.planes = q.planes;
  const o: Record<string, unknown> = {};
  if (q.samples !== undefined) o.samples = q.samples;
  if (q.tol !== undefined) o.tol = q.tol;
  if (Object.keys(o).length) msg.opts = o;
  return call<SymmetryReport>(kernel, msg);
}

export interface ExtremeEnd {
  /** Signed position along `direction` (dot product of the point with the unit direction). */
  value: number;
  point: number[];
  /** The face / edge / vertex the extreme lies on (faces first, then edges, then vertices). */
  entity: EntityRef | null;
  /** Everything that attains the extreme (at most 20). */
  at: EntityRef[];
}

export interface ExtremaReport {
  /** The unit direction measured along. */
  direction: number[];
  min: ExtremeEnd;
  max: ExtremeEnd;
  /** `max.value - min.value`: the extent of the entity along `direction`. */
  extent: number;
  /** `vertices`: exact (planar faces, straight edges); `sampled+refined`: curved faces / edges, see docs. */
  method: 'vertices' | 'sampled+refined';
  exact: boolean;
  samples: number;
  truncated: boolean;
  elapsedMs: number;
}

/** Lowest and highest point of the body (or the entity `of`) along `dir`, with what they lie on. */
export async function extrema(
  kernel: KernelTransport,
  q: { dir: number[]; of?: MeasureEntity; samples?: number },
  opts: QueryOpts = {},
): Promise<ExtremaReport> {
  const msg: Record<string, unknown> = { type: 'extrema', tag: opts.tag ?? 'main', dir: q.dir };
  if (q.of !== undefined) msg.of = q.of;
  if (q.samples !== undefined) msg.opts = { samples: q.samples };
  return call<ExtremaReport>(kernel, msg);
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

/** Import a SolidWorks .SLDPRT part (base64 of the file bytes) into a fresh tag; reply is regen-shaped like importStep. */
export async function importSldprt(
  kernel: KernelTransport,
  data: string,
  opts: { name?: string; tag?: string } = {},
): Promise<ImportReport> {
  const res = await call<ImportReport>(kernel, { type: 'importSldprt', tag: opts.tag ?? 'imp', name: opts.name ?? 'import', data });
  return { ...res, errors: res.errors ?? [] };
}

/**
 * Import a SketchUp .skp file (base64 of the file bytes; both the 2021+ and the classic containers) into a fresh tag.
 * The body is a surface model of planar faces (holes kept); `import.faceMaterials[n]` is the index into `import.materials`
 * (colour, opacity) of face `skp.F<n>`. Hidden geometry is skipped unless `opts.hidden`.
 */
/** Orientation of an imported SketchUp model: 'z' as in the file (Z up, kernel default), 'y' Z becomes Y, (x,y,z) -> (x,z,-y), 'yFlip' upside down variant of 'y', (x,y,z) -> (-x,-z,-y). */
export type SkpUp = 'z' | 'y' | 'yFlip';

export async function importSkp(
  kernel: KernelTransport,
  data: string,
  opts: { name?: string; tag?: string; hidden?: boolean; maxFaces?: number; up?: SkpUp } = {},
): Promise<ImportReport> {
  const msg: Record<string, unknown> = { type: 'importSkp', tag: opts.tag ?? 'imp', name: opts.name ?? 'import', data };
  const o: Record<string, unknown> = {};
  if (opts.hidden !== undefined) o.hidden = opts.hidden;
  if (opts.maxFaces !== undefined) o.maxFaces = opts.maxFaces;
  if (opts.up !== undefined) o.up = opts.up;
  if (Object.keys(o).length) msg.opts = o;
  const res = await call<ImportReport>(kernel, msg);
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
