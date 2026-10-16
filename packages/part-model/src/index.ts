// Part-document model v1 — shapes observed from the bundled web-app JS
// (app/v/<hash>/app.js) and the repo README. Units: mm.
// This package holds pure types + defaults. No kernel / DOM / Node APIs.
//
// NOTE: the 'ck-part' format tag below is an opaque wire-protocol
// constant required by the kernel — it is not a brand reference.

export type FeatureId = `F${number}`;
export type PlaneId = 'front' | 'top' | 'right';
// front = XY (normal +Z), top = XZ, right = ZY (see README + app.js Ot table).
export interface StdPlane { kind: 'std'; id: PlaneId }
export type SketchPlane = StdPlane | { kind: string; id: string };

export interface SketchPoint { id: string; x: number; y: number; fixed?: boolean }

export type SketchEntity =
  | { id: string; type: 'line'; p1: string; p2: string }
  | { id: string; type: 'circle'; c: string; r: number }
  | { id: string; type: 'arc'; c: string; p1: string; p2: string }
  | { id: string; type: 'ellipse'; c: string; p: string; r: number }
  | { id: string; type: 'point'; p: string }
  | { id: string; type: 'spline'; pts: string[]; closed?: boolean }
  | { id: string; type: 'cvspline'; pts: string[]; degree?: number; knots?: number[]; closed?: boolean }
  | SketchText;

/** Lettering (docs/text.md): fixed geometry, outlines of a baked-in font as closed loops. */
export interface SketchText {
  id: string;
  type: 'text';
  string: string;
  /** 'gauge' (condensed sans, default) or 'engrave' (serif) */
  font?: 'gauge' | 'engrave';
  /** cap height in sketch units (mm) */
  height: number;
  /** anchor position; `p` (a sketch point id) wins when given so the position can be constrained */
  at?: [number, number];
  p?: string;
  /** [horizontal, vertical]; horizontal refers to the ink box. Default ['left','baseline'] */
  anchor?: [('left' | 'center' | 'right')?, ('baseline' | 'middle' | 'top')?];
  /** degrees, counter-clockwise about the anchor */
  angle?: number;
  /** letter pitch factor (default 1) */
  spacing?: number;
  mirror?: boolean;
  /** lines within a chord tolerance instead of B-spline edges */
  polyline?: boolean;
  /** lay the letters along a circle: outside = tops away from the centre, reads clockwise */
  arc?: { centre?: [number, number]; c?: string; radius: number; angleCentre?: number; side?: 'outside' | 'inside' };
}

export interface SketchConstraint {
  id: string;
  type:
    | 'horizontal' | 'vertical' | 'distance' | 'equal' | 'coincident'
    | 'tangent' | 'parallel' | 'perpendicular' | 'concentric'
    | 'collinear' | 'symmetric' | 'midpoint' | 'fix'
    | 'radius' | 'diameter' | 'angle';
  refs?: string[];
  value?: number;
}

export interface SketchFeature {
  type: 'sketch';
  id: string;
  name?: string;
  plane: SketchPlane;
  points: SketchPoint[];
  entities: SketchEntity[];
  constraints: SketchConstraint[];
}

export type ExtrudeOp = 'add' | 'cut';
export type ExtrudeEnd = 'blind' | 'mid' | 'throughBoth';

export interface ExtrudeFeature {
  type: 'extrude';
  id: string;
  name?: string;
  op: ExtrudeOp;
  sketch: string; // FeatureId of a sketch feature
  end: ExtrudeEnd;
  depth?: number; // required unless end === 'throughBoth'
  flip?: boolean;
  draft?: number;
  draftOut?: boolean;
  /** optional: sketch entity ids selecting which closed regions to sweep (default all) */
  regions?: string[];
}

/** Every other feature the app's table knows (README §Feature defaults).
 *  Kept open — copy the shape, change the values; sketch/faces/edges refer to ids. */
export interface GenericFeature {
  type: string;
  id: string;
  name?: string;
  [key: string]: unknown;
}

export type Feature = SketchFeature | ExtrudeFeature | KnownFeature | GenericFeature;

import type { KnownFeature } from './features.js';
export type { KnownFeature, FeatureTypeName, EdgeRef, GeomRef, Expr, SketchSolverStatus } from './features.js';
export { FEATURE_TYPES } from './features.js';

export type Material = 'steel' | 'aluminium' | string;

export interface PartDoc {
  format: 'ck-part';
  version: 1;
  name: string;
  features: Feature[];
  /** next feature id counter — ids are F1, F2, … */
  seq: number;
  rollback: null;
  material: Material;
  counters: Record<string, unknown>;
  /** Intent expectations (check grammar): verified by default, Survives round-trips. */
  expect?: Array<Record<string, unknown>>;
}

export function isSketchFeature(f: Feature): f is SketchFeature {
  return f.type === 'sketch';
}

export function emptyPart(name = 'Part1', material: Material = 'steel'): PartDoc {
  return { format: 'ck-part', version: 1, name, features: [], seq: 1, rollback: null, material, counters: {} };
}
