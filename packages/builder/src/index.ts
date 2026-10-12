// Builder for part-document JSON (typed port of the repo's lib.mjs).
// API is intentionally identical (Part, sketch(), extrude(), offsetPlane(), add())
// so existing builders (examples/*.mjs) keep working unchanged.

import {
  emptyPart,
  type ExtrudeEnd,
  type ExtrudeOp,
  type Feature,
  type FeatureId,
  type GenericFeature,
  type Material,
  type PartDoc,
  type PlaneId,
  type SketchConstraint,
  type SketchEntity,
  type SketchPlane,
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

  constructor(name = 'Part', material: Material = 'steel') {
    this.name = name;
    this.material = material;
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

  add(feature: Omit<GenericFeature, 'id'> & { id?: string }): FeatureId {
    (feature as GenericFeature).id ||= this.nextId();
    this.features.push(feature as Feature);
    return (feature as GenericFeature).id as FeatureId;
  }

  toJSON(): PartDoc {
    return {
      ...emptyPart(this.name, this.material),
      features: this.features,
      seq: this.seq,
    };
  }
}
