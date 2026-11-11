// Kernel -> MjPlan: the solved component placements, the mates turned into joints of a kinematic tree, and per part the local
// mesh and mass properties. Only the kernel's public requests are used (regen, massProps, measure, describe).
import type { KernelTransport } from '../../driver/src/verify.js';
import { meshFromBody } from '../../render/src/mesh.js';
import { type Pose, type V3, apply, axisAngle, clean, cross, dot, norm, rot, rotTensor, scale, sub, unit, v3 } from './math.js';
import { fitPrimitive } from './geom.js';
import { DENSITIES, type JointOverride, type MjcfOptions } from './options.js';
import type { MjBody, MjCoupling, MjJoint, MjPart, MjPlan } from './plan.js';

type Feat = { id: string; type: string; name?: string; suppressed?: boolean; fixed?: boolean; part?: PartLike; [k: string]: unknown };
type PartLike = { name?: string; material?: string; features?: unknown[]; [k: string]: unknown };
export interface AssemblyDoc { name?: string; material?: string; features: Feat[]; [k: string]: unknown }

/** Face geometry at the exported pose, world frame, mm. */
export type Geo =
  | { kind: 'plane'; o: V3; n: V3 }
  | { kind: 'cylinder' | 'cone'; o: V3; d: V3; r?: number }
  | { kind: 'sphere'; c: V3; r: number }
  | { kind: 'other' };

const SUFFIX = /@(F\d+)\.(\d+)/g;
/** The component (insertPart id) that owns a face name: the last `@<insert id>.<k>` suffix naming an insert. */
export function componentOfFace(name: string, inserts: ReadonlySet<string>): string | null {
  const ids = [...name.split('|')[0].replace(/#\d+$/, '').matchAll(SUFFIX)].map((m) => m[1]);
  for (let i = ids.length - 1; i >= 0; i--) if (inserts.has(ids[i])) return ids[i];
  return null;
}

/** MJCF-safe name: letters, digits, `_ - .`; never empty, never starting with a digit-only token. */
export function safeName(s: string): string {
  const t = s.trim().replace(/[^A-Za-z0-9_.-]+/g, '_').replace(/^_+|_+$/g, '');
  return t || 'unnamed';
}
function uniquer(): (s: string) => string {
  const used = new Set<string>();
  return (s) => { let n = safeName(s), k = 2; const base = n; while (used.has(n)) n = `${base}_${k++}`; used.add(n); return n; };
}

const num = (v: unknown): number | null => {
  if (typeof v === 'number' && Number.isFinite(v)) return v;
  if (typeof v === 'string' && v.trim() !== '' && Number.isFinite(Number(v))) return Number(v);
  return null;
};

type Reply = { ok?: boolean; result?: unknown; error?: unknown };
async function call<T>(k: KernelTransport, msg: Record<string, unknown>): Promise<T> {
  const r = (await k.request(msg)) as Reply;
  if (!r || !r.ok) throw new Error(`${String(msg.type)}: ${JSON.stringify(r?.error ?? 'failed')}`);
  return r.result as T;
}

interface RegenResult {
  errors?: Array<[string, { code: string; params?: Record<string, unknown> }]>;
  components?: Array<{ id: string; dof?: number; place?: { o: number[]; x: number[]; y: number[]; z: number[] } }>;
  assembly?: { components?: Array<{ id: string; fixed?: boolean; dof?: number; translational?: number; rotational?: number }> };
  body?: { faces?: Array<{ id: string; kind?: string; lump?: number }>; mesh?: Record<string, unknown> } | null;
}
interface Mass { volume?: number; com?: number[]; inertia?: number[][]; principal?: number[]; axes?: number[][] }

export interface AnalyzeOptions extends MjcfOptions {
  /** kernel tag prefix for the regens the export makes (default 'mjcf'); keep it apart from a session's own tag */
  tag?: string;
}

/** Read an assembly document into an MjPlan. */
export async function analyzeAssembly(kernel: KernelTransport, doc: AssemblyDoc, opts: AnalyzeOptions = {}): Promise<MjPlan> {
  const warnings: string[] = [];
  const tag = opts.tag ?? 'mjcf';
  const res = await call<RegenResult>(kernel, { type: 'regen', tag, part: doc, opts: { dofReport: true } });
  const failed = new Map((res.errors ?? []).map(([id, e]) => [id, e.code]));
  for (const [id, code] of failed) warnings.push(`feature ${id}: ${code} (left out of the export)`);
  const live = doc.features.filter((f) => f.suppressed !== true && !failed.has(f.id));
  const inserts = live.filter((f) => f.type === 'insertPart' && f.part);
  const insertIds = new Set(inserts.map((f) => f.id));
  const place = new Map((res.components ?? []).map((c) => [c.id, clean(c.place as never)]));
  const report = new Map((res.assembly?.components ?? []).map((c) => [c.id, c]));
  for (const f of live) if ((f.type === 'cpattern' || f.type === 'lpattern' || f.type === 'mirror') && JSON.stringify(f).includes('@F')) warnings.push(`feature ${f.id} (${f.type}): copies of components are not exported`);

  // ---- parts (deduplicated by content)
  const parts: Record<string, MjPart> = {};
  const partKeyOf = new Map<string, string>();
  const partName = uniquer();
  let pi = 0;
  for (const f of inserts) {
    const text = JSON.stringify(f.part);
    let key = [...Object.values(parts)].find((p) => (p as MjPart & { _text?: string })._text === text)?.key;
    if (!key) {
      key = `P${pi++}`;
      const part = f.part as PartLike;
      const ptag = `${tag}-${key}`;
      const r = await call<RegenResult>(kernel, { type: 'regen', tag: ptag, part });
      if (r.errors?.length) warnings.push(`part ${part.name ?? key}: ${r.errors.length} rebuild error(s) in the part alone`);
      const body = r.body ?? { faces: [] };
      const mesh = meshFromBody(body as never);
      const faces = body.faces ?? [];
      const lumpOfTri = new Int32Array(mesh.triFace.length);
      let lumps = 0;
      for (let t = 0; t < lumpOfTri.length; t++) { const l = faces[mesh.triFace[t]]?.lump ?? 0; lumpOfTri[t] = l; lumps = Math.max(lumps, l + 1); }
      let m: Mass = {};
      try { m = await call<Mass>(kernel, { type: 'massProps', tag: ptag }); } catch (e) { warnings.push(`part ${part.name ?? key}: massProps failed (${(e as Error).message})`); }
      const material = String(part.material ?? doc.material ?? '');
      const dens = densityFor(material, opts, warnings, part.name ?? key);
      const volume = m.volume ?? 0;
      const kgPerMm3 = dens * 1e-6;
      const I = inertiaOf(m, mesh.positions, warnings, part.name ?? key);
      const p: MjPart & { _text?: string } = {
        key, name: partName(part.name ?? `part_${key}`), material, density: dens, volume, mass: volume * kgPerMm3,
        com: v3(m.com), inertia: I.map((row) => row.map((x) => x * kgPerMm3 * 1e-6)),
        positions: mesh.positions, lumpOfTri, lumps: Math.max(1, lumps), faceKinds: faces.map((x) => String(x.kind ?? 'other')),
      };
      const conv = await solidConvexity(kernel, ptag, faces, p.lumps);
      if (conv) p.convexSolids = conv;
      if (m.principal && m.axes) {
        const pr = fitPrimitive({ faceKinds: p.faceKinds, lumps: p.lumps, positions: p.positions, volume, com: p.com, principal: m.principal, axes: m.axes });
        if (pr) p.primitive = pr;
      }
      Object.defineProperty(p, '_text', { value: text, enumerable: false });
      parts[key] = p;
    }
    partKeyOf.set(f.id, key);
  }

  // ---- components
  const bodyName = uniquer();
  const isStatic = (f: Feat) => !!opts.static && (opts.static.includes(f.id) || opts.static.includes(String(f.name ?? '')));
  const comps = inserts.map((f) => ({
    f, id: f.id, name: bodyName(String(f.name ?? f.id)), fixed: f.fixed === true || isStatic(f), place: place.get(f.id) ?? clean(undefined),
  }));
  const byId = new Map(comps.map((c) => [c.id, c]));
  const byName = (s: string) => comps.find((c) => c.id === s || c.name === s || c.f.name === s);
  for (const c of comps) if (!place.has(c.id)) warnings.push(`component ${c.name}: the kernel gave no placement; exported at the origin`);

  // ---- mates and face geometry
  const mates = live.filter((f) => f.type === 'mate');
  const faceNames = new Set<string>();
  for (const m of mates) for (const k of ['a', 'b', 'a2', 'b2', 'c']) if (typeof m[k] === 'string') faceNames.add(m[k] as string);
  const geo = await faceGeometry(kernel, tag, [...faceNames], warnings);
  const compOf = (face: unknown): string | null => (typeof face === 'string' ? componentOfFace(face, insertIds) : null);

  // ---- tree
  const GEOMETRIC = new Set(['coincident', 'concentric', 'distance', 'angle', 'parallel', 'perpendicular', 'tangent', 'lock', 'symmetric', 'width', 'hinge', 'slot', 'screw']);
  const links = mates.filter((m) => GEOMETRIC.has(String(m.kind))).map((m) => ({ m, a: compOf(m.a), b: compOf(m.b) })).filter((l) => l.a && l.b && l.a !== l.b) as Array<{ m: Feat; a: string; b: string }>;
  const parent = new Map<string, string | null>();
  const override = (c: { id: string; name: string; f: Feat }): JointOverride | undefined => opts.joints?.[c.id] ?? opts.joints?.[c.name] ?? (c.f.name ? opts.joints?.[String(c.f.name)] : undefined);
  for (const c of comps) {
    const o = override(c);
    if (c.fixed) parent.set(c.id, null);
    else if (o && o.parent !== undefined) parent.set(c.id, o.parent === null ? null : byName(o.parent)?.id ?? null);
  }
  const bfs = () => {
    let grew = true;
    while (grew) {
      grew = false;
      for (const l of links) {
        for (const [x, y] of [[l.a, l.b], [l.b, l.a]] as const) {
          if (parent.has(x) && !parent.has(y)) { parent.set(y, x); grew = true; }
        }
      }
    }
  };
  bfs();
  for (const c of comps) {
    if (parent.has(c.id)) continue;
    const mated = links.some((l) => l.a === c.id || l.b === c.id);
    parent.set(c.id, null);
    if (!mated && (opts.unmated ?? 'free') === 'fixed') c.fixed = true;
    bfs();
  }
  for (const l of links) {
    if (parent.get(l.a) !== l.b && parent.get(l.b) !== l.a) warnings.push(`mate ${l.m.name ?? l.m.id}: closes a loop between ${byId.get(l.a)!.name} and ${byId.get(l.b)!.name}; MuJoCo does not enforce it (add an equality in sim.raw)`);
  }

  // ---- joints
  const jointName = uniquer();
  const bodies: MjBody[] = [];
  const comById = (id: string): V3 => apply(byId.get(id)!.place, parts[partKeyOf.get(id)!]?.com ?? [0, 0, 0]);
  for (const c of comps) {
    const par = parent.get(c.id) ?? null;
    const b: MjBody = { id: c.id, name: c.name, parent: par, place: c.place, part: partKeyOf.get(c.id)!, joints: [], fixed: c.fixed };
    const o = override(c);
    if (o) b.joints = jointsFromOverride(c.name, o, comById(c.id), jointName);
    else if (!c.fixed && par === null && !links.some((l) => l.a === c.id || l.b === c.id)) b.joints = [{ name: jointName(`${c.name}_free`), type: 'free', axis: [0, 0, 1], pos: comById(c.id) }];
    else if (!c.fixed && par !== null) {
      const between = links.filter((l) => (l.a === c.id && l.b === par) || (l.b === c.id && l.a === par)).map((l) => l.m);
      const inf = inferJoints(c.name, between, geo, comById(c.id), jointName, warnings);
      b.joints = inf;
      const rep = report.get(c.id);
      if (rep && byId.get(par)?.fixed && rep.translational !== undefined && rep.rotational !== undefined) {
        const t = inf.filter((j) => j.type === 'slide').length + (inf.some((j) => j.type === 'free') ? 3 : 0);
        const r = inf.filter((j) => j.type === 'hinge').length + (inf.some((j) => j.type === 'ball' || j.type === 'free') ? 3 : 0);
        if (rep.dof === 0 && inf.length) { warnings.push(`component ${c.name}: the kernel reports no freedom; welded instead of ${inf.map((j) => j.type).join('+')}`); b.joints = []; }
        else if (t !== rep.translational || r !== rep.rotational) warnings.push(`component ${c.name}: joints give ${t} translational + ${r} rotational DOF, the kernel reports ${rep.translational} + ${rep.rotational}; check the joints or set options.joints`);
      }
    } else if (!c.fixed && par === null) {
      // mated, but to nothing fixed: the root of a floating mechanism
      b.joints = [{ name: jointName(`${c.name}_free`), type: 'free', axis: [0, 0, 1], pos: comById(c.id) }];
    }
    bodies.push(b);
  }

  // ---- limits -> ranges
  const bodyOf = new Map(bodies.map((b) => [b.id, b]));
  for (const m of mates.filter((x) => x.kind === 'limits')) applyLimit(m, compOf, bodyOf, byId, geo, warnings);

  // ---- couplings
  const couplings: MjCoupling[] = [];
  const eqName = uniquer();
  for (const m of mates.filter((x) => x.kind === 'gear' || x.kind === 'rackPinion' || x.kind === 'screw')) {
    const cpl = coupling(m, compOf, bodyOf, byId, geo, warnings);
    if (cpl) couplings.push({ ...cpl, name: eqName(m.name ?? m.id), mate: m.id });
  }

  // parents before children
  const ordered: MjBody[] = [];
  const seen = new Set<string>();
  const visit = (b: MjBody) => { if (seen.has(b.id)) return; if (b.parent && bodyOf.has(b.parent)) visit(bodyOf.get(b.parent)!); seen.add(b.id); ordered.push(b); };
  bodies.forEach(visit);
  return { model: safeName(String(doc.name ?? 'assembly')), bodies: ordered, parts, couplings, warnings };
}

/** Unit-density inertia about the centre of mass (mm⁵): the kernel's tensor, else rebuilt from its principal moments and axes
 *  (older kernels), else a solid box of the mesh extents (with a warning). */
function inertiaOf(m: Mass, positions: Float32Array, warnings: string[], who: string): number[][] {
  if (m.inertia && m.inertia.length === 3) return m.inertia;
  if (m.principal?.length === 3 && m.axes?.length === 3) {
    const I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    m.principal.forEach((p, k) => { const a = m.axes![k]; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) I[i][j] += p * a[i] * a[j]; });
    return I;
  }
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i++) { const a = i % 3; lo[a] = Math.min(lo[a], positions[i]); hi[a] = Math.max(hi[a], positions[i]); }
  const e = [0, 1, 2].map((a) => (Number.isFinite(lo[a]) ? hi[a] - lo[a] : 0));
  const V = m.volume ?? 0;
  warnings.push(`part ${who}: the kernel gave no inertia; a box of the part's extents is used`);
  return [[V * (e[1] ** 2 + e[2] ** 2) / 12, 0, 0], [0, V * (e[0] ** 2 + e[2] ** 2) / 12, 0], [0, 0, V * (e[0] ** 2 + e[1] ** 2) / 12]];
}

/** Per solid: convex from the B-rep (no concave edge, no concave face), or null when the kernel cannot tell. */
async function solidConvexity(kernel: KernelTransport, tag: string, faces: Array<{ id: string; lump?: number }>, lumps: number): Promise<boolean[] | null> {
  if (!faces.length) return null;
  try {
    const lumpOf = new Map(faces.map((f) => [f.id, f.lump ?? 0]));
    const out = new Array<boolean>(lumps).fill(true);
    const a = await call<{ edges?: Array<{ faces: string[]; type: string }> }>(kernel, { type: 'angles', tag, edges: 'all' });
    for (const e of a.edges ?? []) if (e.type === 'concave') out[lumpOf.get(e.faces[0]) ?? 0] = false;
    const d = await call<{ items: Array<{ concave?: boolean; found?: boolean }> }>(kernel, { type: 'describe', tag, names: faces.map((f) => f.id) });
    (d.items ?? []).forEach((it, i) => { if (it.concave === true) out[faces[i].lump ?? 0] = false; });
    return out;
  } catch { return null; }
}

function densityFor(material: string, opts: MjcfOptions, warnings: string[], who: string): number {
  const k = material.trim().toLowerCase();
  const fromOpts = opts.densities ? Object.entries(opts.densities).find(([n]) => n.trim().toLowerCase() === k)?.[1] : undefined;
  const d = fromOpts ?? DENSITIES[k];
  if (d !== undefined) return d;
  const dd = opts.defaultDensity ?? 1;
  warnings.push(`part ${who}: material "${material}" has no density; ${dd} g/cm3 used (options.densities)`);
  return dd;
}

async function faceGeometry(kernel: KernelTransport, tag: string, names: string[], warnings: string[]): Promise<Map<string, Geo>> {
  const out = new Map<string, Geo>();
  if (!names.length) return out;
  type Item = { surface?: string; normal?: number[]; center?: number[]; axis?: number[]; radius?: number; centroid?: number[] };
  const one = async (ns: string[]): Promise<Array<Item | null>> => {
    const r = await call<{ items: Item[] }>(kernel, { type: 'measure', tag, refs: ns.map((id) => ({ kind: 'face', id })) });
    return r.items ?? [];
  };
  let items: Array<Item | null>;
  try { items = await one(names); } catch {
    items = [];
    for (const n of names) { try { items.push((await one([n]))[0] ?? null); } catch { items.push(null); warnings.push(`face ${n}: not found at the exported pose`); } }
  }
  let cents: Array<{ centroid?: number[]; found?: boolean }> = [];
  try { cents = (await call<{ items: Array<{ centroid?: number[]; found?: boolean }> }>(kernel, { type: 'describe', tag, names })).items ?? []; } catch { /* centroids are optional */ }
  names.forEach((n, i) => {
    const it = items[i];
    if (!it) { out.set(n, { kind: 'other' }); return; }
    const c = v3(cents[i]?.centroid);
    if (it.surface === 'plane' && it.normal) out.set(n, { kind: 'plane', o: c, n: unit(v3(it.normal)) });
    else if ((it.surface === 'cylinder' || it.surface === 'cone') && it.axis) out.set(n, { kind: it.surface, o: v3(it.center), d: unit(v3(it.axis)), ...(it.radius ? { r: it.radius } : {}) });
    else if (it.surface === 'sphere' && it.center) out.set(n, { kind: 'sphere', c: v3(it.center), r: it.radius ?? 0 });
    else out.set(n, { kind: 'other' });
  });
  return out;
}

// ---- linear algebra on direction sets

/** Orthonormal basis of the span of `vs` (tolerance 1e-6). */
function span(vs: V3[]): V3[] {
  const basis: V3[] = [];
  for (const v of vs) {
    let w = v;
    for (const b of basis) w = sub(w, scale(b, dot(w, b)));
    if (norm(w) > 1e-6) basis.push(unit(w));
    if (basis.length === 3) break;
  }
  return basis;
}
/** Orthonormal basis of the orthogonal complement of the span of `vs`. */
function complement(vs: V3[]): V3[] {
  const b = span(vs);
  return span([...b, [1, 0, 0], [0, 1, 0], [0, 0, 1]]).slice(b.length);
}
function perp2(d: V3): V3[] {
  const a: V3 = Math.abs(d[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const x = unit(cross(d, a));
  return [x, cross(d, x)];
}
const XYZ: V3[] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const isAxis = (g: Geo | undefined): g is Extract<Geo, { d: V3 }> => !!g && (g.kind === 'cylinder' || g.kind === 'cone');
const dirOf = (g: Geo | undefined): V3 | null => (!g ? null : g.kind === 'plane' ? g.n : isAxis(g) ? g.d : null);
/** Canonical sign: the largest component positive. */
function canon(v: V3): V3 {
  const i = [0, 1, 2].reduce((a, b) => (Math.abs(v[b]) > Math.abs(v[a]) + 1e-9 ? b : a), 0);
  return v[i] < 0 ? scale(v, -1) : v;
}
const parallel = (a: V3, b: V3) => Math.abs(dot(unit(a), unit(b))) > 1 - 1e-6;

interface Line { o: V3; d: V3; mate: Feat; stop: boolean }

/** The joints a set of mates between a component and its parent leaves free. */
export function inferJoints(body: string, mates: Feat[], geo: Map<string, Geo>, com: V3, nameOf: (s: string) => string, warnings: string[]): MjJoint[] {
  const T: V3[] = [], R: V3[] = [];
  const lines: Line[] = [];
  const centres: V3[] = [];
  const g = (k: unknown) => (typeof k === 'string' ? geo.get(k) : undefined);
  for (const m of mates) {
    const kind = String(m.kind), ga = g(m.a), gb = g(m.b);
    const pp = ga?.kind === 'plane' && gb?.kind === 'plane';
    const aa = isAxis(ga) && isAxis(gb);
    const ss = ga?.kind === 'sphere' && gb?.kind === 'sphere';
    const unknown = () => warnings.push(`mate ${m.name ?? m.id} (${kind}): face geometry ${ga?.kind ?? '?'} / ${gb?.kind ?? '?'} not mapped to a joint`);
    const axisMate = (stop: boolean) => {
      const t = gb as Extract<Geo, { d: V3 }>;
      T.push(...perp2(t.d)); R.push(...perp2(t.d));
      if (stop) T.push(t.d);
      lines.push({ o: t.o, d: t.d, mate: m, stop });
    };
    switch (kind) {
      case 'coincident': case 'distance': case 'symmetric':
        if (pp) { T.push(gb.n); R.push(...perp2(gb.n)); }
        else if (aa) axisMate(false);
        else if (ss) { T.push(...XYZ); centres.push((gb as { c: V3 }).c); }
        else if (ga?.kind === 'plane' && gb?.kind === 'sphere') T.push(ga.n);
        else if (ga?.kind === 'sphere' && gb?.kind === 'plane') T.push(gb.n);
        else if (ga?.kind === 'plane' && isAxis(gb)) { T.push(ga.n); R.push(cross(gb.d, ga.n)); }
        else if (isAxis(ga) && gb?.kind === 'plane') { T.push(gb.n); R.push(cross(ga.d, gb.n)); }
        else unknown();
        break;
      case 'concentric':
        if (aa) axisMate(false);
        else if (ss) { T.push(...XYZ); centres.push((gb as { c: V3 }).c); }
        else unknown();
        break;
      case 'hinge':
        if (aa) axisMate(typeof m.a2 === 'string' && typeof m.b2 === 'string');
        else unknown();
        break;
      case 'screw':
        if (aa) axisMate(false); else unknown();
        break;
      case 'parallel': {
        const da = dirOf(ga), db = dirOf(gb);
        if (!da || !db) { unknown(); break; }
        if ((ga!.kind === 'plane') === (gb!.kind === 'plane')) R.push(...perp2(db)); else R.push(cross(da, db));
        break;
      }
      case 'perpendicular': case 'angle': {
        const da = dirOf(ga), db = dirOf(gb);
        if (!da || !db) { unknown(); break; }
        if (kind === 'perpendicular' && (ga!.kind === 'plane') !== (gb!.kind === 'plane')) R.push(...perp2(db));
        else R.push(cross(da, db));
        break;
      }
      case 'tangent':
        if (isAxis(ga) && gb?.kind === 'plane') { T.push(gb.n); R.push(cross(ga.d, gb.n)); }
        else if (ga?.kind === 'plane' && isAxis(gb)) { T.push(ga.n); R.push(cross(gb.d, ga.n)); }
        else if (ga?.kind === 'sphere' && gb?.kind === 'plane') T.push(gb.n);
        else if (ga?.kind === 'plane' && gb?.kind === 'sphere') T.push(ga.n);
        else if (aa) { const d = sub(ga.o, gb.o); T.push(sub(d, scale(gb.d, dot(d, gb.d)))); R.push(...perp2(gb.d)); }
        else if (ss) T.push(sub(ga.c, gb.c));
        else unknown();
        break;
      case 'lock': T.push(...XYZ); R.push(...XYZ); break;
      case 'width': if (pp) { T.push(gb.n); R.push(...perp2(gb.n)); } else unknown(); break;
      case 'slot':
        if (isAxis(ga) && gb?.kind === 'plane') { T.push(gb.n); R.push(cross(ga.d, gb.n)); } else unknown();
        break;
      default: unknown();
    }
  }
  const tf = complement(T), rf = complement(R);
  const joints: MjJoint[] = [];
  const nameFor = (m: Feat | undefined, type: string, multi: boolean) => nameOf(m ? `${m.name ?? m.id}${multi ? `_${type}` : ''}` : `${body}_${type}`);
  const first = mates[0];
  const multi = tf.length + rf.length > 1;
  if (rf.length === 3 && tf.length === 0) {
    const c = centres[0] ?? com;
    if (!centres.length) warnings.push(`body ${body}: ball joint without a sphere mate, centred on the centre of mass`);
    return [{ name: nameFor(first, 'ball', false), type: 'ball', axis: [0, 0, 1], pos: c, mate: first?.id }];
  }
  for (const d of tf) {
    const line = lines.find((l) => parallel(l.d, d));
    const axis = line ? line.d : canon(d);
    const pos = line ? sub(line.o, scale(line.d, dot(sub(line.o, com), line.d))) : com;
    joints.push({ name: nameFor(line?.mate ?? first, 'slide', multi), type: 'slide', axis, pos, mate: (line?.mate ?? first)?.id });
  }
  if (rf.length === 3) {
    const c = centres[0] ?? com;
    joints.push({ name: nameFor(first, 'ball', multi), type: 'ball', axis: [0, 0, 1], pos: c, mate: first?.id });
  } else {
    for (const d of rf) {
      const line = lines.find((l) => parallel(l.d, d));
      if (!line) warnings.push(`body ${body}: rotation about ${canon(d).map((x) => +x.toFixed(4)).join(' ')} has no axis mate; the hinge passes through the centre of mass`);
      const axis = line ? line.d : canon(d);
      // the point of the axis line nearest the centre of mass
      const pos = line ? sub(line.o, scale(line.d, dot(sub(line.o, com), line.d))) : com;
      joints.push({ name: nameFor(line?.mate ?? first, 'hinge', multi), type: 'hinge', axis, pos, mate: (line?.mate ?? first)?.id });
    }
  }
  return joints;
}

function jointsFromOverride(body: string, o: JointOverride, com: V3, nameOf: (s: string) => string): MjJoint[] {
  if (o.type === 'weld') return [];
  return [{ name: nameOf(o.name ?? `${body}_${o.type}`), type: o.type, axis: unit(o.axis ?? [0, 0, 1]), pos: o.pos ?? com, ...(o.range ? { range: o.range } : {}) }];
}

type Comp = { id: string; name: string; fixed: boolean; place: Pose };

/** The joints of body `c` that move the face `face` of component `c` relative to component `other`. */
function moving(c: string, other: string, bodyOf: Map<string, MjBody>): MjBody | null {
  const b = bodyOf.get(c);
  return b && b.parent === other ? b : null;
}

function applyLimit(m: Feat, compOf: (f: unknown) => string | null, bodyOf: Map<string, MjBody>, byId: Map<string, Comp>, geo: Map<string, Geo>, warnings: string[]): void {
  const ca = compOf(m.a), cb = compOf(m.b);
  const label = String(m.name ?? m.id);
  if (!ca || !cb) { warnings.push(`limits ${label}: faces not on components`); return; }
  const lo0 = num(m.min), hi0 = num(m.max);
  const ofAngle = m.of === 'angle';
  // the kernel measures from the mover (a unless a is fixed) to the target
  const swap = byId.get(ca)?.fixed === true;
  const [mvF, tgF] = swap ? [m.b, m.a] : [m.a, m.b];
  const [mvC, tgC] = swap ? [cb, ca] : [ca, cb];
  // the body carrying the joint: the child of the other
  const child = moving(mvC, tgC, bodyOf) ?? moving(tgC, mvC, bodyOf);
  if (!child) { warnings.push(`limits ${label}: ${byId.get(ca)?.name} and ${byId.get(cb)?.name} are not parent and child; range not exported`); return; }
  const childIsMover = child.id === mvC;
  const gm = geo.get(String(mvF)), gt = geo.get(String(tgF));
  if (ofAngle) {
    const da = dirOf(gm), dt = dirOf(gt);
    const j = child.joints.find((x) => x.type === 'hinge');
    if (!da || !dt || !j) { warnings.push(`limits ${label}: needs face directions and a hinge on ${child.name}`); return; }
    const lo = lo0 === null ? null : (lo0 * Math.PI) / 180, hi = hi0 === null ? null : (hi0 * Math.PI) / 180;
    const theta = (q: number) => {
      const r = axisAngle(j.axis, q);
      const a = childIsMover ? rot(r, da) : da, t = childIsMover ? dt : rot(r, dt);
      return Math.acos(Math.max(-1, Math.min(1, dot(unit(a), unit(t)))));
    };
    const inside = (q: number) => { const v = theta(q); return (lo === null || v >= lo - 1e-9) && (hi === null || v <= hi + 1e-9); };
    if (!inside(0)) warnings.push(`limits ${label}: the exported pose is outside ${lo0}..${hi0} deg`);
    const edge = (sg: number): number | null => {
      const step = Math.PI / 720;
      let prev = 0;
      for (let q = step; q <= Math.PI + 1e-12; q += step) {
        if (!inside(sg * q)) {
          let a = prev, b = q;
          for (let i = 0; i < 60; i++) { const mid = (a + b) / 2; if (inside(sg * mid)) a = mid; else b = mid; }
          return sg * a;
        }
        prev = q;
      }
      return null;
    };
    const lo2 = edge(-1), hi2 = edge(1);
    if (lo2 === null && hi2 === null) return;
    j.range = [lo2 ?? -1e3, hi2 ?? 1e3];
    j.limit = m.id;
  } else {
    if (gm?.kind !== 'plane' || gt?.kind !== 'plane') { warnings.push(`limits ${label}: distance limits need two planar faces`); return; }
    const n = gt.n;
    const v0 = dot(n, sub(gm.o, gt.o));
    const slides = child.joints.filter((x) => x.type === 'slide');
    const j = slides.sort((x, y) => Math.abs(dot(n, y.axis)) - Math.abs(dot(n, x.axis)))[0];
    const k = j ? dot(n, j.axis) * (childIsMover ? 1 : -1) : 0;
    if (!j || Math.abs(k) < 1e-9) { warnings.push(`limits ${label}: no slide on ${child.name} along the limited distance`); return; }
    const a = lo0 === null ? (k > 0 ? -1e6 : 1e6) : (lo0 - v0) / k;
    const b = hi0 === null ? (k > 0 ? 1e6 : -1e6) : (hi0 - v0) / k;
    if ((lo0 !== null && v0 < lo0 - 1e-9) || (hi0 !== null && v0 > hi0 + 1e-9)) warnings.push(`limits ${label}: the exported pose (${+v0.toFixed(6)} mm) is outside ${lo0}..${hi0}`);
    j.range = [Math.min(a, b), Math.max(a, b)];
    j.limit = m.id;
  }
}

function coupling(m: Feat, compOf: (f: unknown) => string | null, bodyOf: Map<string, MjBody>, byId: Map<string, Comp>, geo: Map<string, Geo>, warnings: string[]): Omit<MjCoupling, 'name' | 'mate'> | null {
  const label = String(m.name ?? m.id);
  const ca = compOf(m.a), cb = compOf(m.b);
  if (!ca || !cb) { warnings.push(`${m.kind} ${label}: faces not on components`); return null; }
  const flip = m.flip === true;
  const ga = geo.get(String(m.a)), gb = geo.get(String(m.b));
  const sgn = (x: number) => (x < 0 ? -1 : 1);
  const jointOn = (c: string, type: string, along: V3 | null): MjJoint | undefined => {
    const js = bodyOf.get(c)?.joints.filter((j) => j.type === type) ?? [];
    return along ? js.find((j) => parallel(j.axis, along)) : js[0];
  };
  if (m.kind === 'gear') {
    if (!isAxis(ga) || !isAxis(gb)) { warnings.push(`gear ${label}: a and b must be axes`); return null; }
    const swapped = byId.get(cb)?.fixed === true;
    const ratio0 = num(m.ratio) ?? 1;
    const [drv, fol, axT, axM, ratio] = swapped ? [cb, ca, gb.d, ga.d, 1 / ratio0] : [ca, cb, ga.d, gb.d, ratio0];
    const k = (flip ? 1 : -1) * sgn(dot(axT, axM)) / ratio;
    const jf = jointOn(fol, 'hinge', axM), jd = jointOn(drv, 'hinge', axT);
    if (!jf || (!jd && !byId.get(drv)?.fixed)) { warnings.push(`gear ${label}: needs a hinge on both gears; not coupled`); return null; }
    if (!jd) return null; // driver fixed: the follower is locked by the gear, nothing to couple
    return { joint1: jf.name, joint2: jd.name, c0: 0, c1: sgn(dot(jf.axis, axM)) * k * sgn(dot(jd.axis, axT)) };
  }
  if (m.kind === 'rackPinion') {
    if (!isAxis(ga)) { warnings.push(`rackPinion ${label}: a must be the pinion axis`); return null; }
    const u = Array.isArray(m.dir) ? unit(v3(m.dir)) : isAxis(gb) ? gb.d : null;
    if (!u) { warnings.push(`rackPinion ${label}: no rack direction`); return null; }
    const radius = num(m.radius) ?? 1;
    const k = (flip ? -1 : 1) * radius;
    const js = jointOn(cb, 'slide', u), jh = jointOn(ca, 'hinge', ga.d);
    if (!js || !jh) { warnings.push(`rackPinion ${label}: needs a hinge on the pinion and a slide on the rack; not coupled`); return null; }
    return { joint1: js.name, joint2: jh.name, c0: 0, c1: sgn(dot(js.axis, u)) * k * sgn(dot(jh.axis, ga.d)) };
  }
  // screw: the follower (b unless fixed) turns and travels on the target's axis
  const swapped = byId.get(cb)?.fixed === true;
  const fol = swapped ? ca : cb;
  const tgt = (swapped ? gb : ga) as Geo | undefined;
  if (!isAxis(tgt)) { warnings.push(`screw ${label}: needs axes`); return null; }
  const pitch = num(m.pitch) ?? 1;
  const lh = flip ? -1 : 1;
  const js = jointOn(fol, 'slide', tgt.d), jh = jointOn(fol, 'hinge', tgt.d);
  if (!js || !jh) { warnings.push(`screw ${label}: needs a hinge and a slide on ${byId.get(fol)?.name}; not coupled`); return null; }
  return { joint1: js.name, joint2: jh.name, c0: 0, c1: sgn(dot(js.axis, tgt.d)) * sgn(dot(jh.axis, tgt.d)) * (lh * pitch) / (2 * Math.PI) };
}

/** World inertia of a part placed at `p` (for checks). */
export const worldInertia = (p: Pose, part: MjPart): number[][] => rotTensor(p, part.inertia);
