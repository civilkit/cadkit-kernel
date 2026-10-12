// Every remaining ck-part feature type. Field shapes come from the
// kernel's dispatch (crates/ck-model/src/features/mod.rs) and the app's own
// defaults table; `Expr` covers numeric fields the kernel parses as
// expressions. All interfaces are open-ended (extra wire fields pass through)
// so new kernel-side fields never break the document model.

import type { FeatureId, SketchConstraint, SketchEntity, SketchPlane, SketchPoint } from './index.js';

/** Numeric field the kernel parses as an expression (plain numbers everywhere today). */
export type Expr = number | string;

/** Geometry ref object: {kind:'axis',axis:'x'}, {kind:'std',id:'front'}, ... */
export interface GeomRef {
  kind: string;
  [key: string]: unknown;
}

/** Edge reference: two faces plus a point on the edge. */
export interface EdgeRef {
  faces: string[];
  at: number[];
}

export interface Sketch3DPoint {
  id: string;
  x: number;
  y: number;
  z: number;
  fixed?: boolean;
}

export type Sketch3DEntity =
  | SketchEntity
  | { id: string; type: 'spline'; pts?: string[]; [key: string]: unknown };

export interface Sketch3DFeature {
  type: 'sketch3d';
  id: string;
  name?: string;
  points: Sketch3DPoint[];
  entities: Sketch3DEntity[];
  constraints: SketchConstraint[];
  [key: string]: unknown;
}

export interface RevolveFeature {
  type: 'revolve';
  id: string;
  name?: string;
  op: 'add' | 'cut';
  sketch: string | null;
  axis?: GeomRef | string | null;
  end: string;
  angle?: Expr;
  flip?: boolean;
  [key: string]: unknown;
}

export interface SweepFeature {
  type: 'sweep';
  id: string;
  name?: string;
  op: 'add' | 'cut';
  sketch: string | null;
  path?: GeomRef | null;
  twist?: Expr;
  surface?: boolean;
  [key: string]: unknown;
}

export interface LoftFeature {
  type: 'loft';
  id: string;
  name?: string;
  op: 'add' | 'cut';
  sketches: string[];
  surface?: boolean;
  centreLine?: unknown;
  guides?: unknown;
  [key: string]: unknown;
}

export interface FilletFeature {
  type: 'fillet';
  id: string;
  name?: string;
  radius: Expr;
  edges: EdgeRef[];
  [key: string]: unknown;
}

export interface ChamferFeature {
  type: 'chamfer';
  id: string;
  name?: string;
  chamferType: 'equal' | 'angleDistance' | 'distanceDistance' | 'vertex' | string;
  distance: Expr;
  distance2?: Expr;
  angle?: Expr;
  flip?: boolean;
  edges: EdgeRef[];
  [key: string]: unknown;
}

export interface DraftFeature {
  type: 'draft';
  id: string;
  name?: string;
  neutral?: GeomRef | null;
  faces: string[];
  angle: Expr;
  flip?: boolean;
  [key: string]: unknown;
}

export interface ShellFeature {
  type: 'shell';
  id: string;
  name?: string;
  faces: string[];
  thickness: Expr;
  outward?: boolean;
  [key: string]: unknown;
}

export interface HoleFeature {
  type: 'hole';
  id: string;
  name?: string;
  sketch: string | null;
  standard?: string;
  size?: string;
  fit?: 'normal' | 'tap' | 'close' | 'loose' | string;
  kind?: 'simple' | 'cbore' | 'csink' | string;
  diameter?: Expr;
  end?: string;
  depth?: Expr;
  tip?: boolean;
  flip?: boolean;
  cboreDiameter?: Expr;
  cboreDepth?: Expr;
  cskDiameter?: Expr;
  cskAngle?: Expr;
  [key: string]: unknown;
}

export interface HelixFeature {
  type: 'helix';
  id: string;
  name?: string;
  sketch: string | null;
  definedBy?: 'pitchRevs' | 'pitchHeight' | 'revsHeight' | string;
  pitch?: Expr;
  revolutions?: Expr;
  height?: Expr;
  startAngle?: Expr;
  leftHand?: boolean;
  reverse?: boolean;
  [key: string]: unknown;
}

export interface RibFeature {
  type: 'rib';
  id: string;
  name?: string;
  sketch: string | null;
  thickness: Expr;
  flip?: boolean;
  [key: string]: unknown;
}

export interface LipFeature {
  type: 'lip';
  id: string;
  name?: string;
  kind?: string;
  faces: string[];
  edges: EdgeRef[];
  width: Expr;
  height: Expr;
  [key: string]: unknown;
}

export interface LinearPatternFeature {
  type: 'lpattern';
  id: string;
  name?: string;
  seeds: string[];
  dir?: GeomRef | null;
  spacing: Expr;
  count: Expr;
  reverse?: boolean;
  [key: string]: unknown;
}

export interface CircularPatternFeature {
  type: 'cpattern';
  id: string;
  name?: string;
  seeds: string[];
  axis?: GeomRef | null;
  count: Expr;
  angle?: Expr;
  equal?: boolean;
  reverse?: boolean;
  [key: string]: unknown;
}

export interface CurvePatternFeature {
  type: 'curvepattern';
  id: string;
  name?: string;
  seeds: string[];
  path?: string | GeomRef | null;
  count: Expr;
  equal?: boolean;
  spacing?: Expr;
  align?: boolean;
  reverse?: boolean;
  [key: string]: unknown;
}

export interface FillPatternFeature {
  type: 'fillpattern';
  id: string;
  name?: string;
  seeds: string[];
  face?: string | null;
  spacing?: Expr;
  stagger?: Expr;
  margin?: Expr;
  [key: string]: unknown;
}

export interface SketchPatternFeature {
  type: 'spattern';
  id: string;
  name?: string;
  seeds: string[];
  sketch?: string | null;
  [key: string]: unknown;
}

export interface MirrorFeature {
  type: 'mirror';
  id: string;
  name?: string;
  seeds: string[];
  plane?: GeomRef | string | null;
  body?: boolean;
  [key: string]: unknown;
}

export interface TrimSurfaceFeature {
  type: 'trimSurface';
  id: string;
  name?: string;
  surface?: string | null;
  surfaces?: string[];
  toolSurface?: string | null;
  plane?: GeomRef | null;
  mutual?: boolean;
  flip?: boolean;
  deleteTool?: boolean;
  [key: string]: unknown;
}

export interface SurfacePlanarFeature {
  type: 'surfacePlanar';
  id: string;
  name?: string;
  sketch: string | null;
  [key: string]: unknown;
}

export interface SurfaceRevolveFeature {
  type: 'surfaceRevolve';
  id: string;
  name?: string;
  sketch: string | null;
  axis?: GeomRef | null;
  end?: string;
  angle?: Expr;
  flip?: boolean;
  [key: string]: unknown;
}

export interface SurfaceExtrudeFeature {
  type: 'surfaceExtrude';
  id: string;
  name?: string;
  sketch: string | null;
  end?: string;
  depth?: Expr;
  flip?: boolean;
  curves?: unknown;
  [key: string]: unknown;
}

export interface SurfaceOffsetFeature {
  type: 'surfaceOffset';
  id: string;
  name?: string;
  faces: string[];
  distance: Expr;
  flip?: boolean;
  [key: string]: unknown;
}

export interface KnitFeature {
  type: 'knit';
  id: string;
  name?: string;
  surfaces: string[];
  solid?: boolean;
  [key: string]: unknown;
}

export interface FillFeature {
  type: 'fill';
  id: string;
  name?: string;
  edges: EdgeRef[];
  merge?: boolean;
  continuity?: 'G0' | 'G1' | 'G2' | 'G3' | string;
  fairing?: string;
  [key: string]: unknown;
}

export interface InsertPartFeature {
  type: 'insertPart';
  id: string;
  name?: string;
  part?: import('./index.js').PartDoc | null;
  source?: string | null;
  fixed?: boolean;
  dx?: number;
  dy?: number;
  dz?: number;
  axis?: GeomRef | null;
  angle?: Expr;
  [key: string]: unknown;
}

export interface MateFeature {
  type: 'mate';
  id: string;
  name?: string;
  kind: 'coincident' | 'concentric' | 'distance' | 'angle' | 'parallel' | string;
  a?: string | null;
  b?: string | null;
  distance?: Expr;
  flip?: boolean;
  [key: string]: unknown;
}

/** Wire type is "plane" (reference plane). */
export interface PlaneFeature {
  type: 'plane';
  id: string;
  name?: string;
  mode?: string;
  base?: GeomRef | null;
  offset?: Expr;
  flip?: boolean;
  axis?: GeomRef | null;
  angle?: Expr;
  base2?: GeomRef | null;
  visible?: boolean;
  [key: string]: unknown;
}

export interface DeleteBodyFeature {
  type: 'deleteBody';
  id: string;
  name?: string;
  mode?: string;
  bodies: unknown[];
  [key: string]: unknown;
}

export interface DeleteFaceFeature {
  type: 'deleteFace';
  id: string;
  name?: string;
  faces: string[];
  mode?: string;
  continuity?: string;
  edges: EdgeRef[];
  [key: string]: unknown;
}

export interface ThickenFeature {
  type: 'thicken';
  id: string;
  name?: string;
  surface?: string | null;
  thickness: Expr;
  side?: string;
  op?: 'add' | 'cut' | string;
  [key: string]: unknown;
}

export interface CutWithSurfaceFeature {
  type: 'cutWithSurface';
  id: string;
  name?: string;
  surface?: string | null;
  flip?: boolean;
  [key: string]: unknown;
}

export interface PipeCutFeature {
  type: 'pipeCut';
  id: string;
  name?: string;
  radius: Expr;
  edges: EdgeRef[];
  faces: string[];
  [key: string]: unknown;
}

export interface FreeformFeature {
  type: 'freeform';
  id: string;
  name?: string;
  faces: string[];
  direction?: unknown;
  edges: EdgeRef[];
  edits?: unknown[];
  deviation?: Expr;
  bound?: unknown;
  frames?: unknown;
  [key: string]: unknown;
}

export interface EditCurveFeature {
  type: 'editCurve';
  id: string;
  name?: string;
  curves?: unknown[];
  method?: string;
  degree?: Expr;
  count?: Expr;
  tolerance?: Expr;
  keepEnds?: boolean;
  keepTangents?: boolean;
  keepCorners?: boolean;
  entries?: unknown[];
  planar?: boolean;
  plane?: GeomRef | null;
  [key: string]: unknown;
}

export interface ProjectCurveFeature {
  type: 'projectCurve';
  id: string;
  name?: string;
  sketch?: string | null;
  faces: string[];
  degree?: Expr;
  tolerance?: Expr;
  [key: string]: unknown;
}

export interface ExtendFeature {
  type: 'extend';
  id: string;
  name?: string;
  edges: EdgeRef[];
  distance: Expr;
  [key: string]: unknown;
}

export interface DomeFeature {
  type: 'dome';
  id: string;
  name?: string;
  face?: string | null;
  height: Expr;
  elliptical?: boolean;
  reverse?: boolean;
  [key: string]: unknown;
}

export interface SplitLineFeature {
  type: 'splitLine';
  id: string;
  name?: string;
  sketch?: string | null;
  faces: string[];
  [key: string]: unknown;
}

export interface MoveFaceFeature {
  type: 'moveFace';
  id: string;
  name?: string;
  faces: string[];
  distance: Expr;
  flip?: boolean;
  [key: string]: unknown;
}

export interface MoveBodyFeature {
  type: 'moveBody';
  id: string;
  name?: string;
  copy?: boolean;
  dx?: number;
  dy?: number;
  dz?: number;
  axis?: GeomRef | null;
  angle?: Expr;
  bodies?: unknown[];
  [key: string]: unknown;
}

export interface ScaleFeature {
  type: 'scale';
  id: string;
  name?: string;
  factor: Expr;
  about?: string;
  [key: string]: unknown;
}

export interface AxisFeature {
  type: 'axis';
  id: string;
  name?: string;
  refs: string[];
  [key: string]: unknown;
}

/** Every typed feature except sketch/extrude (declared in index.ts). */
export type KnownFeature =
  | Sketch3DFeature | RevolveFeature | SweepFeature | LoftFeature
  | FilletFeature | ChamferFeature | DraftFeature | ShellFeature | HoleFeature
  | HelixFeature | RibFeature | LipFeature
  | LinearPatternFeature | CircularPatternFeature | CurvePatternFeature
  | FillPatternFeature | SketchPatternFeature | MirrorFeature
  | TrimSurfaceFeature | SurfacePlanarFeature | SurfaceRevolveFeature
  | SurfaceExtrudeFeature | SurfaceOffsetFeature | KnitFeature | FillFeature
  | InsertPartFeature | MateFeature | PlaneFeature | DeleteBodyFeature
  | DeleteFaceFeature | ThickenFeature | CutWithSurfaceFeature | PipeCutFeature
  | FreeformFeature | EditCurveFeature | ProjectCurveFeature | ExtendFeature
  | DomeFeature | SplitLineFeature | MoveFaceFeature | MoveBodyFeature
  | ScaleFeature | AxisFeature;

/** All wire type strings the kernel dispatches on. */
export const FEATURE_TYPES = [
  'sketch', 'sketch3d', 'extrude', 'revolve', 'sweep', 'loft',
  'fillet', 'chamfer', 'draft', 'shell', 'hole', 'helix', 'rib', 'lip',
  'lpattern', 'cpattern', 'curvepattern', 'fillpattern', 'spattern', 'mirror',
  'trimSurface', 'surfacePlanar', 'surfaceRevolve', 'surfaceExtrude',
  'surfaceOffset', 'knit', 'fill', 'insertPart', 'mate', 'plane',
  'deleteBody', 'deleteFace', 'thicken', 'cutWithSurface', 'pipeCut',
  'freeform', 'editCurve', 'projectCurve', 'extend', 'dome', 'splitLine',
  'moveFace', 'moveBody', 'scale', 'axis',
] as const;

export type FeatureTypeName = (typeof FEATURE_TYPES)[number];

export interface SketchSpec {
  points: SketchPoint[];
  entities: SketchEntity[];
  constraints: SketchConstraint[];
}
export type { FeatureId };
