// Builder for part-document JSON (typed port of the repo's lib.mjs).
// API is intentionally identical (Part, sketch(), extrude(), offsetPlane(), add())
// so existing builders (examples/*.mjs) keep working unchanged.

import {
  emptyPart,
  type EdgeRef,
  type ExtrudeEnd,
  type ExtrudeOp,
  type Feature,
  type FeatureId,
  type GenericFeature,
  type GeomRef,
  type Material,
  type PartDoc,
  type PlaneId,
  type SketchConstraint,
  type SketchEntity,
  type SketchPlane,
  type SketchText,
  type SketchPoint,
} from '../../part-model/src/index.js';

export type BuildFn = (sk: Sketch) => void;

export class Sketch {
  points: SketchPoint[] = [{ id: 'O', x: 0, y: 0, fixed: true }];
  entities: SketchEntity[] = [];
  constraints: SketchConstraint[] = [];
  private n = 0;

  private id(p: string): string { return p + (++this.n); }
  pt(x: number, y: number): string {
    const id = this.id('p');
    this.points.push({ id, x, y });
    return id;
  }
  line(a: string, b: string): string {
    const id = this.id('e');
    this.entities.push({ id, type: 'line', p1: a, p2: b });
    return id;
  }
  circle(cx: number, cy: number, r: number): string {
    const c = this.pt(cx, cy);
    const id = this.id('e');
    this.entities.push({ id, type: 'circle', c, r });
    return id;
  }
  /** lettering (docs/text.md); the anchor point is a sketch point, so it can be constrained */
  text(string: string, x: number, y: number, o: Omit<SketchText, 'id' | 'type' | 'string' | 'p' | 'at'> = { height: 5 }): string {
    const p = this.pt(x, y);
    const id = this.id('t');
    this.entities.push({ id, type: 'text', string, p, ...o });
    return id;
  }
  /** hole centre: a point entity over a new point (what the hole feature reads) */
  point(x: number, y: number): string {
    const p = this.pt(x, y);
    const id = this.id('e');
    this.entities.push({ id, type: 'point', p });
    return id;
  }
  /** closed polygon through [x,y] vertices */
  poly(pts: Array<[number, number]>): string[] {
    const ids = pts.map(([x, y]) => this.pt(x, y));
    ids.forEach((p, i) => this.line(p, ids[(i + 1) % ids.length]));
    return ids;
  }
  rect(x0: number, y0: number, x1: number, y1: number): string[] {
    return this.poly([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]);
  }
}

export interface ExtrudeOpts {
  sketch: string;
  op?: ExtrudeOp;
  end?: ExtrudeEnd;
  depth?: number;
  flip?: boolean;
  name?: string;
}

export class Part {
  name: string;
  material: Material;
  features: Feature[] = [];
  seq = 1;
  expect?: Array<Record<string, unknown>>;

  constructor(name = 'Part', material: Material = 'steel') {
    this.name = name;
    this.material = material;
  }

  /** Adopt an existing document (including its expectations) for editing. */
  static fromJSON(doc: PartDoc): Part {
    const p = new Part(doc.name, doc.material);
    p.features = JSON.parse(JSON.stringify(doc.features)) as Feature[];
    p.seq = doc.seq;
    if (doc.expect) p.expect = JSON.parse(JSON.stringify(doc.expect)) as Array<Record<string, unknown>>;
    return p;
  }

  nextId(): FeatureId { return (`F${this.seq++}`) as FeatureId; }

  /** plane: 'front' (XY, normal +Z), 'top', 'right'. build(sk) draws with sk.rect / sk.circle / sk.line. */
  sketch(name: string, plane: PlaneId | SketchPlane, build: BuildFn): FeatureId {
    const sk = new Sketch();
    build(sk);
    const f: Feature = {
      type: 'sketch', name, id: this.nextId(),
      plane: typeof plane === 'string' ? { kind: 'std', id: plane } : plane,
      points: sk.points, entities: sk.entities, constraints: sk.constraints,
    };
    this.features.push(f);
    return f.id as FeatureId;
  }

  /** op 'add' | 'cut'; end 'blind' | 'mid' | 'throughBoth'; depth in mm (blind/mid). */
  extrude({ sketch, op = 'add', end = 'blind', depth, flip = false, name }: ExtrudeOpts): FeatureId {
    const f: Feature = {
      type: 'extrude', op, sketch, end,
      id: this.nextId(), name: name ?? `${op} extrude`,
      ...(end !== 'throughBoth' ? { depth } : {}),
      ...(flip ? { flip: true } : {}),
    };
    this.features.push(f);
    return f.id as FeatureId;
  }

  /** reference plane parallel to a standard plane at `offset` mm; returns its feature id */
  offsetPlane(base: PlaneId, offset: number, name?: string, flip = false): FeatureId {
    return this.add({
      type: 'plane', mode: 'offset',
      base: { kind: 'std', id: base }, offset, flip,
      name: name ?? `Plane ${offset}`, visible: false,
    } as unknown as GenericFeature);
  }

  private push(type: string, fields: Record<string, unknown>, name?: string): FeatureId {
    return this.add({ type, name: name ?? type, ...fields } as unknown as GenericFeature);
  }

  revolve(o: { sketch: string; op?: 'add' | 'cut'; end?: string; angle?: number; axis?: GeomRef | string | null; flip?: boolean; name?: string }): FeatureId {
    const { sketch, op = 'add', end = 'blind', angle = 360, flip = false, name } = o;
    // Like the app: default to the sketch's first line entity when no axis given.
    // Explicit null stays null (the kernel reports missingAxis).
    let axis: GeomRef | string | null = o.axis ?? null;
    if (o.axis === undefined) {
      const sk = this.features.find((f) => f.id === sketch);
      const lines = sk && 'entities' in sk
        ? (sk.entities as Array<{ id: string; type: string }>).filter((e) => e.type === 'line').map((e) => e.id)
        : [];
      axis = lines[0] ?? null;
    }
    return this.push('revolve', { op, sketch, axis, end, angle, ...(flip ? { flip: true } : {}) }, name ?? `${op} revolve`);
  }

  /** edge = two faces + a point on it: {faces:[a,b], at:[x,y,z]} */
  fillet(o: { radius: number; edges: EdgeRef[]; name?: string }): FeatureId {
    return this.push('fillet', { radius: o.radius, edges: o.edges }, o.name);
  }

  chamfer(o: { chamferType?: string; distance: number; distance2?: number; angle?: number; edges: EdgeRef[]; flip?: boolean; name?: string }): FeatureId {
    const { chamferType = 'equal', distance, distance2, angle, edges, flip = false, name } = o;
    return this.push('chamfer', {
      chamferType, distance, edges,
      ...(distance2 !== undefined ? { distance2 } : {}),
      ...(angle !== undefined ? { angle } : {}),
      ...(flip ? { flip: true } : {}),
    }, name);
  }

  shell(o: { faces?: string[]; thickness: number; outward?: boolean; name?: string }): FeatureId {
    // faces[] empty = remove all faces (hollow the whole body)
    return this.push('shell', { faces: o.faces ?? [], thickness: o.thickness, ...(o.outward ? { outward: true } : {}) }, o.name);
  }

  hole(o: { sketch: string | null; kind?: string; diameter?: number; end?: string; depth?: number; tip?: boolean; flip?: boolean; cboreDiameter?: number; cboreDepth?: number; cskDiameter?: number; cskAngle?: number; name?: string }): FeatureId {
    const { sketch, kind = 'simple', diameter = 6, end = 'blind', depth = 10, tip = true, flip = false, name, ...rest } = o;
    return this.push('hole', {
      sketch, kind, diameter, end, depth, tip,
      ...(flip ? { flip: true } : {}),
      ...Object.fromEntries(Object.entries(rest).filter(([, v]) => v !== undefined)),
    }, name);
  }

  draft(o: { neutral?: GeomRef | null; faces: string[]; angle: number; flip?: boolean; name?: string }): FeatureId {
    return this.push('draft', { neutral: o.neutral ?? null, faces: o.faces, angle: o.angle, ...(o.flip ? { flip: true } : {}) }, o.name);
  }

  rib(o: { sketch: string | null; thickness: number; flip?: boolean; name?: string }): FeatureId {
    return this.push('rib', { sketch: o.sketch, thickness: o.thickness, ...(o.flip ? { flip: true } : {}) }, o.name);
  }

  lip(o: { kind?: string; faces?: string[]; edges?: EdgeRef[]; width: number; height: number; name?: string }): FeatureId {
    return this.push('lip', { kind: o.kind ?? 'lip', faces: o.faces ?? [], edges: o.edges ?? [], width: o.width, height: o.height }, o.name);
  }

  helix(o: { sketch: string | null; definedBy?: string; pitch?: number; revolutions?: number; height?: number; startAngle?: number; leftHand?: boolean; reverse?: boolean; name?: string }): FeatureId {
    const { sketch, definedBy = 'pitchRevs', pitch = 1, revolutions = 10, height = 10, startAngle = 0, leftHand = false, reverse = false, name } = o;
    return this.push('helix', {
      sketch, definedBy, pitch, revolutions, height, startAngle,
      ...(leftHand ? { leftHand: true } : {}), ...(reverse ? { reverse: true } : {}),
    }, name);
  }

  sweep(o: { sketch: string | null; path?: GeomRef | null; op?: 'add' | 'cut'; twist?: number; name?: string }): FeatureId {
    const { sketch, path = null, op = 'add', twist, name } = o;
    return this.push('sweep', { op, sketch, path, ...(twist !== undefined ? { twist } : {}) }, name ?? `${op} sweep`);
  }

  loft(o: { sketches: string[]; op?: 'add' | 'cut'; name?: string }): FeatureId {
    const { sketches, op = 'add', name } = o;
    return this.push('loft', { op, sketches }, name ?? `${op} loft`);
  }

  lpattern(o: { seeds: string[]; dir?: GeomRef | null; spacing: number; count: number; reverse?: boolean; name?: string }): FeatureId {
    return this.push('lpattern', {
      seeds: o.seeds, dir: o.dir ?? { kind: 'axis', axis: 'x' },
      spacing: o.spacing, count: o.count, ...(o.reverse ? { reverse: true } : {}),
    }, o.name);
  }

  cpattern(o: { seeds: string[]; axis?: GeomRef | null; count: number; angle?: number; equal?: boolean; reverse?: boolean; name?: string }): FeatureId {
    const { seeds, axis = null, count, angle = 360, equal = true, reverse = false, name } = o;
    return this.push('cpattern', {
      seeds, axis, count, angle, equal, ...(reverse ? { reverse: true } : {}),
    }, name);
  }

  mirror(o: { seeds: string[]; plane?: GeomRef | string | null; body?: boolean; name?: string }): FeatureId {
    return this.push('mirror', { seeds: o.seeds, plane: o.plane ?? null, ...(o.body ? { body: true } : {}) }, o.name);
  }

  scale(o: { factor: number; about?: string; name?: string }): FeatureId {
    return this.push('scale', { factor: o.factor, about: o.about ?? 'centroid' }, o.name);
  }

  mate(o: { kind?: string; a?: string | null; b?: string | null; distance?: number; flip?: boolean; name?: string }): FeatureId {
    const { kind = 'coincident', a = null, b = null, distance = 0, flip = false, name } = o;
    return this.push('mate', { kind, a, b, distance, ...(flip ? { flip: true } : {}) }, name);
  }

  deleteBody(o: { bodies?: unknown[]; mode?: string; name?: string }): FeatureId {
    return this.push('deleteBody', { mode: o.mode ?? 'delete', bodies: o.bodies ?? [] }, o.name);
  }

  /** Thread on a cylindrical face; list standards / sizes with the threadData request. */
  thread(o: { face: string; standard: string; size: string; class?: string; length?: number | 'full'; hand?: 'right' | 'left'; start?: 'top' | 'bottom'; modeled?: boolean; name?: string }): FeatureId {
    const { face, standard, size, name } = o;
    return this.push('thread', {
      face, standard, size,
      ...(o.class ? { class: o.class } : {}),
      ...(o.length !== undefined ? { length: o.length } : {}),
      ...(o.hand ? { hand: o.hand } : {}),
      ...(o.start ? { start: o.start } : {}),
      ...(o.modeled ? { modeled: true } : {}),
    }, name);
  }

  deleteFace(o: { faces: string[]; edges?: EdgeRef[]; mode?: string; continuity?: string; name?: string }): FeatureId {
    return this.push('deleteFace', {
      faces: o.faces, edges: o.edges ?? [], mode: o.mode ?? 'delete', continuity: o.continuity ?? 'G0',
    }, o.name);
  }

  moveFace(o: { faces: string[]; distance?: number; flip?: boolean; mode?: 'offset' | 'translate'; translate?: [number, number, number]; name?: string }): FeatureId {
    return this.push('moveFace', {
      faces: o.faces,
      ...(o.mode ? { mode: o.mode } : {}),
      ...(o.distance !== undefined ? { distance: o.distance } : o.mode === 'translate' ? {} : { distance: 0 }),
      ...(o.translate ? { translate: o.translate } : {}),
      ...(o.flip ? { flip: true } : {}),
    }, o.name);
  }

  moveBody(o: {
    bodies?: Array<string | { face: string } | { surface: string }>;
    translate?: [number, number, number];
    rotate?: { axis: [number, number, number] | 'x' | 'y' | 'z'; origin?: [number, number, number]; angle: number };
    copy?: boolean; name?: string;
  }): FeatureId {
    return this.push('moveBody', {
      ...(o.bodies ? { bodies: o.bodies } : {}),
      ...(o.translate ? { translate: o.translate } : {}),
      ...(o.rotate ? { rotate: o.rotate } : {}),
      ...(o.copy ? { copy: true } : {}),
    }, o.name);
  }

  dome(o: { face?: string | null; height: number; elliptical?: boolean; reverse?: boolean; name?: string }): FeatureId {
    return this.push('dome', {
      face: o.face ?? null, height: o.height,
      ...(o.elliptical ? { elliptical: true } : {}), ...(o.reverse ? { reverse: true } : {}),
    }, o.name);
  }

  splitLine(o: { sketch?: string | null; faces: string[]; name?: string }): FeatureId {
    return this.push('splitLine', { sketch: o.sketch ?? null, faces: o.faces }, o.name);
  }

  axis(o: { refs: string[]; name?: string }): FeatureId {
    return this.push('axis', { refs: o.refs }, o.name);
  }

  // Sheet/surface features (surfaceExtrude, surfaceRevolve, surfacePlanar,
  // surfaceOffset, trimSurface, knit, fill, thicken, extend), freeform,
  // editCurve, projectCurve, pipeCut, cutWithSurface, insertPart, moveBody
  // and the sketch/curve/fill patterns have no convenience method yet —
  // build their payloads with add() (shapes in part-model).

  add(feature: Omit<GenericFeature, 'id'> & { id?: string }): FeatureId {
    const f = feature as GenericFeature;
    if (f.id === undefined) f.id = this.nextId();
    else if (typeof f.id === 'string') {
      const n = /^F(\d+)$/.exec(f.id);
      if (n) this.seq = Math.max(this.seq, +n[1] + 1);
    }
    this.features.push(feature as Feature);
    return (feature as GenericFeature).id as FeatureId;
  }

  toJSON(): PartDoc {
    const doc: PartDoc = {
      ...emptyPart(this.name, this.material),
      features: this.features,
      seq: this.seq,
    };
    if (this.expect) doc.expect = JSON.parse(JSON.stringify(this.expect)) as Array<Record<string, unknown>>;
    return doc;
  }
}
