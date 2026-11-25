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

export interface SphereFeature {
  type: 'sphere';
  id: string;
  name?: string;
  op: 'add' | 'cut' | 'newBody';
  /** with `op: 'newBody'`: the name of the body (lump) it makes */
  body?: string;
  /** the centre, in the part's world (Y up) */
  center: [Expr, Expr, Expr];
  radius: Expr;
  [key: string]: unknown;
}

export interface TorusFeature {
  type: 'torus';
  id: string;
  name?: string;
  op: 'add' | 'cut' | 'newBody';
  body?: string;
  /** the centre, in the part's world (Y up) */
  center?: [Expr, Expr, Expr];
  /** 'x' | 'y' | 'z' or a direction [x, y, z]; default z */
  axis?: string | [Expr, Expr, Expr];
  majorRadius: Expr;
  minorRadius: Expr;
  [key: string]: unknown;
}

export interface PipeFeature {
  type: 'pipe';
  id: string;
  name?: string;
  op: 'add' | 'cut' | 'newBody';
  body?: string;
  /** the path: a sketch or a curve of the part */
  path: { kind: 'sketch' | 'curve'; id: string };
  radius: Expr;
  /** wall thickness for a hollow tube; absent = solid */
  wall?: Expr;
  [key: string]: unknown;
}

export interface CoilFeature {
  type: 'coil';
  id: string;
  name?: string;
  op: 'add' | 'cut' | 'newBody';
  body?: string;
  center?: [Expr, Expr, Expr];
  axis?: string | [Expr, Expr, Expr];
  radius: Expr;
  wireRadius: Expr;
  pitch: Expr;
  /** give revolutions or height */
  revolutions?: Expr;
  height?: Expr;
  leftHand?: boolean;
  [key: string]: unknown;
}

export interface BoundingSolidFeature {
  type: 'boundingSolid';
  id: string;
  name?: string;
  /** the bodies, faces or features the box surrounds */
  bodies: Array<{ body: string } | { face: string } | { feature: string }>;
  padding?: Expr;
  [key: string]: unknown;
}

export interface RevolveFeature {
  type: 'revolve';
  id: string;
  name?: string;
  op: 'add' | 'cut' | 'newBody';
  /** with `op: 'newBody'` (or on any creating feature): the name of the body (lump) it makes */
  body?: string;
  sketch: string | null;
  axis?: GeomRef | string | null;
  end: string;
  angle?: Expr;
  flip?: boolean;
  /** optional: sketch entity ids selecting which closed regions to sweep (default all) */
  regions?: string[];
  [key: string]: unknown;
}

export interface SweepFeature {
  type: 'sweep';
  id: string;
  name?: string;
  op: 'add' | 'cut' | 'newBody';
  body?: string;
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
  op: 'add' | 'cut' | 'newBody';
  body?: string;
  sketches: string[];
  surface?: boolean;
  centreLine?: unknown;
  guides?: unknown;
  /** Per profile, the fraction (0 to 1) of a closed profile's length where its start point sits; null = chosen by the kernel (0 is the fraction 0, not "automatic"). */
  connectors?: Array<number | null>;
  startNormal?: number | boolean;
  endNormal?: number | boolean;
  apexTangent?: number;
  railSmoothing?: 'natural' | 'local' | 'auto';
  /**
   * End conditions, one entry per section (same length as `sketches`; null / missing = free). Only the first and last may be non-free.
   * `directional`: `angle` in degrees from the profile plane (0 < angle < 180, default 90 = along the profile normal; below 90 the loft flares
   * outward, above 90 it leans inward) and `weight` (> 0, default 1, the tension). `tangent` / `curvature` are carried but refused by the kernel.
   */
  conditions?: Array<LoftCondition | null>;
  [key: string]: unknown;
}

export interface LoftCondition {
  type: 'free' | 'directional' | 'tangent' | 'curvature';
  angle?: number;
  weight?: number;
  face?: unknown;
}

export interface FilletFeature {
  type: 'fillet';
  id: string;
  name?: string;
  radius: Expr;
  edges: EdgeRef[];
  /** partial edge: the round stops this many mm short of the end of each picked edge that is nearer its pick point (`at`); >= 0, 0 or absent = to the end. Not with `propagate`. Applies to every edge of the feature. */
  startOffset?: Expr;
  /** partial edge: the round stops this many mm short of the other end of each picked edge; >= 0. `startOffset + endOffset` must be below the edge length (`invalidValue {field, detail, hint, length}`). */
  endOffset?: Expr;
  /** corner setback in mm: distance along each edge from a vertex of three selected edges to the corner patch; 0 or absent = automatic (rolling-ball sphere; the natural setback is the radius). Warnings `blendSetbackAdjusted` / `blendSetbackUnused`. */
  setback?: Expr;
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
  /** partial edge: the chamfer stops this many mm short of the end of each picked edge that is nearer its pick point (`at`); >= 0, 0 or absent = to the end. Not with `propagate`. */
  startOffset?: Expr;
  /** partial edge: the chamfer stops this many mm short of the other end of each picked edge; >= 0. `startOffset + endOffset` must be below the edge length. */
  endOffset?: Expr;
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
  /** distance from the path to the band (a centred tongue); default 0: the band touches the path */
  inset?: Expr;
  /** groove only: wider by this on each side and deeper by it (needs inset >= clearance when inset > 0) */
  clearance?: Expr;
  [key: string]: unknown;
}

/** A plane reference: a standard plane, a reference plane feature or a planar face. */
export type PlaneRefJson = { kind: 'std'; id: string } | { kind: 'feature'; id: string } | { kind: 'face'; id: string };

/** Cut a body by a plane into two bodies (`names`: [below, above], below = away from the plane normal). */
export interface SplitFeature {
  type: 'split';
  id: string;
  name?: string;
  plane: PlaneRefJson;
  offset?: Expr;
  flip?: boolean;
  target?: BodyRef | string;
  names?: [string, string];
  keep?: 'both' | 'below' | 'above';
  [key: string]: unknown;
}

/** A split with a tongue on one half and a groove (wider and deeper by `clearance`) on the other. */
export interface LipJointFeature {
  type: 'lipJoint';
  id: string;
  name?: string;
  plane: PlaneRefJson;
  offset?: Expr;
  flip?: boolean;
  target?: BodyRef | string;
  names?: [string, string];
  height: Expr;
  /** tongue width in mm; or `widthRatio` (fraction of the wall, default 0.5) */
  width?: Expr;
  widthRatio?: Expr;
  /** distance from the outer wall; default: centred */
  inset?: Expr;
  /** default 0.2 */
  clearance?: Expr;
  tongue?: 'below' | 'above';
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
  /** How the copies are spread: `spacing` between neighbours (default), `extent` seed to last copy, `toObject` the last copy reaches `to`. */
  distanceType?: 'spacing' | 'extent' | 'toObject';
  extent?: Expr;
  /** To Object: a plane or flat face reference, or a point `[x, y, z]` / `{kind:'point', at}` / `{kind:'vertex', faces, at}`. */
  to?: GeomRef | number[] | null;
  toOffset?: Expr;
  /** A second direction: the same fields with the suffix 2. */
  dir2?: GeomRef | null;
  count2?: Expr;
  spacing2?: Expr;
  reverse2?: boolean;
  distanceType2?: 'spacing' | 'extent' | 'toObject';
  extent2?: Expr;
  to2?: GeomRef | number[] | null;
  toOffset2?: Expr;
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
  /** `spacing` between neighbours (default) or `extent` from the seed to the last copy, along the path. */
  distanceType?: 'spacing' | 'extent';
  extent?: Expr;
  /** Where the seed sits along the path: mm, or a fraction 0..1 of its length. */
  start?: Expr;
  startFraction?: Expr;
  /** Run the path from its other end (the original `reverse`). */
  flip?: boolean;
  /** `fixed` (default) or `tangent` (the original `align: true`). */
  orientation?: 'fixed' | 'tangent';
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
  /** With `body`: the image replaces the body (a sheet metal part stays a sheet) instead of being united with it. */
  replace?: boolean;
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
  faces?: string[];
  /** a surface body to offset, instead of faces */
  surface?: string;
  distance: Expr;
  flip?: boolean;
  /** keep the source surface body (default true) */
  keep?: boolean;
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

/** Anchor for a Mate Connector (`type: 'connector'`). */
export type ConnectorAnchor =
  | { kind: 'face'; id: string }
  | { kind: 'circle'; face?: string; id?: string; faces?: string[]; at?: [number, number, number] }
  | { kind: 'vertex'; faces: string[]; at?: [number, number, number] }
  | { kind: 'edge'; faces: string[]; at?: [number, number, number] }
  | { kind: 'std'; id: string }
  | { kind: 'feature'; id: string }
  | { kind: 'connector'; id: string }
  | { kind: 'frame'; origin?: [number, number, number]; xDir: [number, number, number]; yDir?: [number, number, number]; zDir?: [number, number, number] };

/** First-class Mate Connector: a named local coordinate frame `(origin, x, y, z)`. */
export interface ConnectorFeature {
  type: 'connector';
  id: string;
  name?: string;
  anchor: ConnectorAnchor;
  /** Local translation offset `[dx, dy, dz]` in anchor frame coordinates. */
  offset?: [Expr, Expr, Expr];
  /** Local Euler rotations `[rx, ry, rz]` in degrees. */
  rotate?: [Expr, Expr, Expr];
  /** Flip the Z axis (and X to preserve right-handedness). */
  flip?: boolean;
  /** Rotation about local Z in degrees. */
  rotateZ?: Expr;
  [key: string]: unknown;
}

/** Reference to a Mate Connector (on a component or at assembly top-level). */
export interface ConnectorRef {
  kind: 'connector';
  id: string;
  component?: string;
}

export interface MateFeature {
  type: 'mate';
  id: string;
  name?: string;
  kind:
    | 'coincident' | 'concentric' | 'distance' | 'angle' | 'parallel'
    | 'perpendicular' | 'tangent' | 'lock' | 'symmetric' | 'width'
    | 'hinge' | 'slot' | 'screw' | 'gear' | 'rackPinion' | 'limits'
    | 'fastened' | 'revolute' | 'slider' | 'cylindrical' | 'planar'
    | string;
  a?: string | ConnectorRef | null;
  b?: string | ConnectorRef | null;
  /** Second face of the mover / target side (width, hinge stop, slot wall). */
  a2?: string | null;
  b2?: string | null;
  /** Mirror face of `symmetric`. */
  c?: string | null;
  distance?: Expr;
  /** Mate-level local offset `[dx, dy, dz]` (or scalar Z offset) for connector mates. */
  offset?: [Expr, Expr, Expr] | Expr;
  /** Mate-level Z rotation (degrees) for connector mates. */
  rotateZ?: Expr;
  /** Degrees: the `angle` mate's angle, or the drive of gear / rackPinion / screw. */
  angle?: Expr;
  ratio?: Expr;
  pitch?: Expr;
  radius?: Expr;
  /** `limits` / `revolute` / `slider`: range of the measured distance (mm) or angle (degrees). */
  min?: Expr;
  max?: Expr;
  of?: 'distance' | 'angle';
  /** Travel direction of a planar rack. */
  dir?: [number, number, number];
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

/** Boolean of whole bodies: `target` (one named body) combined with `tools` (named bodies). */
export interface CombineFeature {
  type: 'combine';
  id: string;
  name?: string;
  op: 'union' | 'cut' | 'intersect' | 'split';
  /** names the result body */
  body?: string;
  target: BodyRef;
  tools: BodyRef[];
  /** keep the tool bodies after the operation (default false) */
  keepTools?: boolean;
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

/**
 * Project a sketch (its non-construction curves) or a planar curve feature perpendicular to its plane onto faces:
 * one fitted B-spline on the surface, result in `curves[]` with a marker {degree, count, deviation, length,
 * pieces, corners, closed}. `curve` wins over `sketch` when both are given. `faces` empty or missing = every face
 * of the solid and of the surface bodies. `direction`: along (`forward`), against (`reverse`) or both ways from
 * the plane normal (for a `curve` input the normal is the plane's with its largest component positive).
 * A projection that does not form one connected curve is `projectCurveSeveral`.
 *
 * `procedural: true` instead keeps the projection as its definition (a ck-geom projected curve) onto exactly ONE
 * face: the input chain (`curves` refs in chain order, else `curve`, else `sketch`) to the closest point
 * (`along: 'normal'`) or along a direction (`along: [x, y, z]`; default the sketch plane normal / the plane of the
 * curve, else the closest point). `degree` and `direction` are not read then; `tolerance` bounds the B-spline the
 * definition converts to. Refused when the projection leaves the face (`projectCurveMisses`).
 */
export interface ProjectCurveFeature {
  type: 'projectCurve';
  id: string;
  name?: string;
  sketch?: string | null;
  curve?: string | null;
  faces?: string[];
  /** B-spline degree, 2..9 (default 3). */
  degree?: Expr;
  /** Largest distance of the fit from the projected data, mm (default 0.001); more control points (up to 400) until met. */
  tolerance?: Expr;
  direction?: 'both' | 'forward' | 'reverse';
  /** Keep the projection as a procedural curve onto one face (see above). */
  procedural?: boolean;
  /** Procedural only: `'normal'` (closest point) or a direction. */
  along?: 'normal' | [number, number, number];
  /** Procedural only: curve references in chain order (win over `curve` and `sketch`). */
  curves?: unknown[];
  /** Procedural only: largest gap between consecutive pieces, mm (default 1e-5). */
  gapTolerance?: Expr;
  /**
   * `project` (default: the projection above) or `wrap` (docs/proposals/wrap-feature.md 2.2): lay the sketch's curves on ONE
   * cylindrical face (`faces` of length 1) by length along the surface (theta = s / r, z = y), open or closed, any number of turns.
   * `wrap` takes a `sketch` only (not `curve`, not `procedural`) and does not read `direction`; the marker adds `turns`.
   */
  method?: 'project' | 'wrap';
  /** `wrap` only, as the solid `wrap`: mm along the axis (the direction whose largest component is positive). */
  alongAxis?: Expr;
  /** `wrap` only: degrees about the axis, along the reading direction. */
  around?: Expr;
  /** `wrap` only: degrees about the middle of the curves, in the sketch. */
  rotation?: Expr;
  /** `wrap` only: mirror along the circumference. */
  flip?: boolean;
  /** `wrap` only: subset of the sketch's entity ids (default every non-construction entity). */
  entities?: string[];
  [key: string]: unknown;
}

/**
 * Offset a curve or chain by `distance` along unit(C' x N) (positive: to the right of travel seen from the side
 * N points to; for a sketch N is its plane normal), N from `normal`, `plane`, else the sketch plane / the plane of
 * the curve; or in the tangent planes of a support `face` (N = the face's outward normal). The input is `curves`
 * (refs in chain order), else `curve`, else `sketch`. Kept as a procedural curve; the offset of lines and arcs in
 * their plane is exact. Refused: `offsetCurveCorner` (a tangent discontinuity), `offsetCurveFold` (the distance
 * passes the radius of curvature, or the offset comes closer than the distance to another part of the curve),
 * `offsetCurveDirection` (zero tangent, tangent along N, zero normal), `offsetCurvePlane` (no plane to offset in).
 */
export interface OffsetCurveFeature {
  type: 'offsetCurve';
  id: string;
  name?: string;
  curves?: unknown[];
  curve?: string | null;
  sketch?: string | null;
  distance: Expr;
  normal?: [number, number, number];
  plane?: GeomRef | null;
  face?: string;
  /** Largest gap between consecutive pieces, mm (default 1e-5). */
  gapTolerance?: Expr;
  /** Bound of the B-spline the definition converts to, mm (default 0.001). */
  tolerance?: Expr;
  [key: string]: unknown;
}

/**
 * Ordered curves / edges joined end to end into one curve (each piece turned to start where the previous one ends;
 * a gap above `gapTolerance` is `compositeCurveGap`). Kept as a procedural composite; a chain of lines, arcs and
 * B-splines converts exactly.
 */
export interface CompositeCurveFeature {
  type: 'compositeCurve';
  id: string;
  name?: string;
  curves?: unknown[];
  curve?: string | null;
  sketch?: string | null;
  gapTolerance?: Expr;
  tolerance?: Expr;
  [key: string]: unknown;
}

/**
 * A curve input of `sweepSurface`: a string (an earlier curve feature id, else a sketch whose non-construction
 * curves form one chain), one curve reference (`{kind: 'sketchCurve', sketch, entity}`, `{kind: 'edge', faces, at}`,
 * `{kind: 'curve', id}`) or a list of them in chain order.
 */
export type CurveInput = string | Record<string, unknown> | Record<string, unknown>[];

/**
 * A sheet whose one face is a profile swept along a path, kept as its definition (a procedural sweep surface).
 * `frame: 'tangent'` (default) turns the profile with the path tangent and the `up` direction (default: the normal of
 * a plane path, a world axis across a straight one, else +z); `'translate'` carries it without turning. `caps: true`
 * closes a closed planar profile on an open path with plane faces `C0` (start) and `C1` (end). Faces `<id>.W`, caps
 * `<id>.C0` / `<id>.C1`. Marker `{kind: 'sweep', faces, area, frame, closed: [path, profile], caps, pathLength, up?,
 * volume?, degree, count, deviation}`. Refused: `sweepSurfaceNoInput`, `sweepSurfaceFrame` (up along the tangent),
 * `sweepSurfaceDegenerate` (no area somewhere), `sweepSurfaceFold` (the profile reaches past the path's centre of
 * curvature), `sweepSurfaceCaps`, `sweepSurfaceFailed` (not converted within `tolerance`).
 */
export interface SweepSurfaceFeature {
  type: 'sweepSurface';
  id: string;
  name?: string;
  profile: CurveInput;
  path: CurveInput;
  frame?: 'tangent' | 'translate';
  /** Tangent frame only: a direction never along the path tangent. */
  up?: [number, number, number];
  caps?: boolean;
  /** Largest gap between consecutive pieces of an input chain, mm (default 1e-5). */
  gapTolerance?: Expr;
  /** Bound of the B-spline the definition converts to, mm (default 0.001). */
  tolerance?: Expr;
  [key: string]: unknown;
}

/**
 * A sheet whose one face is the rolling-ball blend between two faces, kept as its definition. `radius`: a number, or
 * `[start, end]` linear along the span. `sides`: +1 where the ball lies on the side a face's outward normal points
 * to, -1 against it (default: each toward the other face, a round of a convex edge). The span: `from` / `to` (a line
 * spine between them), else the edge the faces share (nearest `at`). Face `<id>.R`. Marker `{kind: 'blend', faces,
 * area, radius: [start, end], sides, span: 'edge' | 'points', spineLength, closed, degree, count, deviation}`.
 * Refused: `blendSurfaceNoInput`, `blendSurfaceSpan`, `blendSurfaceSides`, `blendSurfaceNoContact {at, u}`,
 * `blendSurfaceRadius {radius, maxRadius}`, `blendSurfaceFailed`.
 */
export interface BlendSurfaceFeature {
  type: 'blendSurface';
  id: string;
  name?: string;
  faces: [string, string];
  radius: Expr | [Expr, Expr];
  sides?: [1 | -1, 1 | -1];
  from?: [number, number, number];
  to?: [number, number, number];
  at?: [number, number, number];
  /** Bound of the B-spline the definition converts to, mm (default 0.001). */
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
  /** cut by a plane instead of a sketch */
  plane?: unknown;
  offset?: Expr;
  flip?: boolean;
  /** sketch lines are projected along 'normal' (default), 'sketch' or a vector */
  direction?: string | [Expr, Expr, Expr];
  [key: string]: unknown;
}

/**
 * Push / pull faces; unselected neighbours extend or trim, tangent round neighbours of a
 * pushed plane travel with it. `mode: 'offset'` (default) moves each face along its outward
 * normal by `distance` (positive adds material; planes slide, cylinders / cones / spheres /
 * tori change radius, a hole wall with a positive distance shrinks the hole); `mode:
 * 'translate'` moves planar faces by the component of `translate` along their outward normal.
 * Errors: noFaces, missingReference, moveFaceUnsupported {detail}, moveFaceFailed {detail}.
 */
export interface MoveFaceFeature {
  type: 'moveFace';
  id: string;
  name?: string;
  faces: string[];
  mode?: 'offset' | 'translate';
  distance?: Expr;
  translate?: [Expr, Expr, Expr];
  flip?: boolean;
  [key: string]: unknown;
}

/**
 * Continue a flat face straight out of the solid (positive `distance`, joined) or into it (negative, cut), as if the face
 * were extruded. `taper`: `none` (default), `draft` (`angle` degrees, -85..85, outward positive: the outline grows) or
 * `scale` (`endScale` > 0, about the face's area centre, holes included). The moved end keeps the face's name; new walls are
 * `<id>.S.<tag>`. `continue: 'along'` (end follows a bend in the walls) is not built yet.
 * Errors: noFaces, missingReference, invalidValue {field, detail, hint}, pressPullUnsupported {detail}, draftTooLarge,
 * moveFaceFailed {detail} (+ maxSize).
 */
export interface PressPullFeature {
  type: 'pressPull';
  id: string;
  name?: string;
  faces: string[];
  continue?: 'straight' | 'along';
  distance: Expr;
  taper?: { type: 'none' } | { type: 'draft'; angle: Expr } | { type: 'scale'; endScale: Expr };
  [key: string]: unknown;
}

/** A body of a `moveBody`: the lump holding a face, or a surface body by its feature id. */
export type BodyRef = string | { face: string } | { surface: string } | { body: string } | { feature: string } | { lump: number };

/**
 * Rigid move (rotate about `rotate.axis` through `rotate.origin` by `rotate.angle` degrees,
 * then translate) of the listed bodies (default: the whole solid), optionally as a copy
 * (`copy: true`; the copy's face names carry `@<id>.1`). Face names of a moved body are
 * unchanged. The legacy `dx/dy/dz` + `axis` + `angle` form still reads.
 */
export interface MoveBodyFeature {
  type: 'moveBody';
  id: string;
  name?: string;
  copy?: boolean;
  /** with `copy: true`: the name of the copies (the original keeps its name) */
  body?: string;
  /** with `copy: true`: the copies are the mirror images about this plane (a standard plane, plane feature or planar face) */
  mirror?: GeomRef;
  bodies?: BodyRef[];
  translate?: [Expr, Expr, Expr];
  rotate?: { axis: [Expr, Expr, Expr] | 'x' | 'y' | 'z'; origin?: [Expr, Expr, Expr]; angle: Expr };
  dx?: number;
  dy?: number;
  dz?: number;
  axis?: GeomRef | null;
  angle?: Expr;
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

/**
 * A thread on a cylindrical face (a hole wall or a shaft). `standard` / `size`
 * come from the `threadData` request. Cosmetic unless `modeled` is true; the regen
 * result lists it under `threads` and warns (`warnings`) when the diameter does
 * not fit the thread.
 */
export interface ThreadFeature {
  type: 'thread';
  id: string;
  name?: string;
  face: string;
  standard: string;
  size: string;
  class?: string;
  /** mm, or "full" (default) = the whole face */
  length?: Expr | 'full';
  hand?: 'right' | 'left';
  start?: 'top' | 'bottom';
  modeled?: boolean;
  [key: string]: unknown;
}

/**
 * A diamond or pyramid knurl built directly as faces and cut from the body in one boolean (docs/knurl-band.md).
 * The cylinder is given by axis, centre, radius and axial range, not by a face.
 */
export interface KnurlBandFeature {
  type: 'knurlBand';
  id: string;
  name?: string;
  axis?: 'x' | 'y' | 'z';
  /** [x, y] for z, [x, z] for y, [y, z] for x */
  center?: [Expr, Expr];
  radius: Expr;
  from: Expr;
  to: Expr;
  pattern: 'diamond' | 'pyramid';
  count?: Expr;
  pitch?: Expr;
  depth?: Expr;
  angle?: Expr;
  helixAngle?: Expr;
  ringPitch?: Expr;
  [key: string]: unknown;
}

/**
 * Text and closed profiles laid on a curved face by length along the surface (docs/proposals/wrap-feature.md; phase 1: a cylindrical
 * face, outside or in a bore). Widths are kept (the development of the cylinder is isometric); walls stand along the face normals
 * unless `taper` is set. Faces: walls `<id>.S.<tag>`, the cap that stays `<id>.C0`.
 */
export interface WrapFeature {
  type: 'wrap';
  id: string;
  name?: string;
  /** Sketch feature id whose closed regions (text, loops) are wrapped. */
  sketch: string;
  /** Subset of the sketch's profiles (a region's outer entity id, or a text entity id = all its glyphs); default all closed regions. */
  profiles?: string[];
  /** One cylindrical face name. */
  face: string;
  op: 'add' | 'cut';
  /** Height raised / depth engraved along the face normals, mm. */
  depth: Expr;
  /** `auto` and `wrap` develop the cylinder; `project` is not built yet (wrapFaceUnsupported). */
  method?: 'auto' | 'wrap' | 'project';
  /** mm along the axis direction whose largest component is positive. */
  alongAxis?: Expr;
  /** degrees about the axis, along the reading direction. */
  around?: Expr;
  /** degrees, counter-clockwise in the sketch, about the middle of the selected profiles. */
  rotation?: Expr;
  /** degrees, -45..45: positive widens the profile as it rises from the face. */
  taper?: Expr;
  /** Mirror along the circumference (about the middle of the selected profiles). */
  flip?: boolean;
  /** `project` only (not built yet). */
  direction?: unknown;
  /** As `extrude`: the body the add / cut acts on. */
  target?: unknown;
  /** Fit tolerance of the curved edges, mm (default 0.001). */
  tolerance?: Expr;
  [key: string]: unknown;
}

/**
 * A body from a STEP file, held by the part. The STEP text is an entry of the part's `blobs` table (`PartDoc.blobs`) named by `blob`,
 * the key being the lower-case hex SHA-256 of the text; `step` holds the text in the feature itself (small files only).
 * Faces are named `<id>.I<solid>F<n>` (docs/import-step.md).
 */
export interface ImportStepFeature {
  type: 'importStep';
  id: string;
  name?: string;
  blob?: string;
  step?: string;
  /** name of the body; several solids are `<body>`, `<body> 2`, ... */
  body?: string;
  /** repair the solids after reading, as the `importStep` message does (faces turned the wrong way, free edges); for a STEP a converter restated */
  heal?: boolean;
  [key: string]: unknown;
}

/**
 * Repair Part: repairs the surface bodies an `importStep` left (the parts of its file that are not valid solids, `<target>.P<k>`):
 * faces the reader left out are rebuilt on their own surface, free edges within `gap` are sewn, flat holes are filled. A part
 * that comes out a valid solid joins the solid body. Reports the warning `repairReport`; refused with `repairTarget`,
 * `repairNothing` or `repairGap`.
 */
export interface RepairPartFeature {
  type: 'repairPart';
  id: string;
  name?: string;
  /** the importStep feature whose surface bodies are repaired */
  target: string;
  /** largest gap sewn, mm (default 0.2, at most 1) */
  gap?: number;
  /** fill flat holes with a face (default true) */
  patchHoles?: boolean;
  [key: string]: unknown;
}

/** One section of a `frame` member: a standard shape and its dimensions (mm); `id` / `label` name the stock in the cut list. */
export interface FrameProfile {
  kind: 'rectTube' | 'rect' | 'roundTube' | 'roundBar' | 'angle' | 'channel' | 'ibeam' | 'tslot';
  id?: string;
  label?: string;
  [dim: string]: unknown;
}

/**
 * A structural frame (weldment): every line of the skeleton sketch (or each world line of `lines`) becomes a member, the section
 * swept along it and trimmed where members meet (mitre or butt; an end on the middle of another member is butted against it).
 * Each member is its own body `<id> M<k>`. The regen reply lists the cut list under `frames` (stock length, cut angles, ends).
 */
export interface FrameFeature {
  type: 'frame';
  id: string;
  name?: string;
  sketch?: string;
  /** line entity ids of the sketch to use (default: every non-construction line) */
  segments?: string[];
  lines?: Array<[[number, number, number], [number, number, number]]>;
  profile: FrameProfile;
  /** turn of the section about each member, deg */
  rotate?: Expr;
  corner?: 'mitre' | 'butt';
  members?: Array<{ segment: string; profile?: FrameProfile; rotate?: Expr }>;
  joints?: Array<{ segments: [string, string]; corner: 'mitre' | 'butt'; through?: string }>;
  [key: string]: unknown;
}

// ---- sheet metal (docs/sheet-metal.md, book/src/format/features-sheet.md; field names: crates/ck-model/src/features/known_fields.rs) ----

/** A pick of a sheet edge: a point on it (`[x,y,z]` or `{at}`, the nearest outline edge wins), `{edge: n}` (index into
 *  the outline), or a role name such as `"P0.side.e2"`. A partial flange may carry `start`/`length`/`from`/`to` on a pick. */
export type SheetEdgePick = number[] | string | { at?: number[]; edge?: number; role?: string; panel?: number; [key: string]: unknown };

/** The stock a sheet is made of. Only `name` is required; the rest default from the thickness. A rule with an existing name replaces it. */
export interface SheetRulesFeature {
  type: 'sheetRules';
  id: string;
  name?: string;
  thickness?: Expr;
  kFactor?: Expr;
  bendRadius?: Expr;
  reliefWidth?: Expr;
  reliefDepth?: Expr;
  cornerRelief?: Expr | string;
  cornerReliefShape?: 'notch' | 'round' | 'rectangular' | 'tear' | string;
  hemGap?: Expr;
  gap?: Expr;
  /** make this rule the active one (default true) */
  active?: boolean;
  [key: string]: unknown;
}

/** The first plate: one closed sketch region (holes allowed), extruded by the rule's thickness. */
export interface BaseFlangeFeature {
  type: 'baseFlange';
  id: string;
  name?: string;
  sketch: string;
  /** rule name; default the active rule */
  rule?: string;
  flip?: boolean;
  regions?: string[];
  [key: string]: unknown;
}

/** The base from an open chain of sketch lines. */
export interface ContourFlangeFeature {
  type: 'contourFlange';
  id: string;
  name?: string;
  sketch: string;
  /** sketch curve ids; default every line */
  curves?: string[];
  distance?: Expr;
  rule?: string;
  flip?: boolean;
  reverse?: boolean;
  [key: string]: unknown;
}

/** A further bend chained on the far edge of an edge flange (at most 16). */
export interface SheetFlangeNext {
  /** degrees, default 90 */
  angle?: Expr;
  height: Expr;
  flip?: boolean;
}

/** A shaped flange leg: a closed loop of `{line:[[v,h],[v,h]]}` / `{arc:{center,from,to,ccw}}` segments in leg coordinates. */
export type SheetProfile = Array<{ line?: number[][]; arc?: { center: number[]; from: number[]; to: number[]; ccw?: boolean }; [key: string]: unknown }>;

/** A bent flange on edges of the plate. `height` is required. */
export interface EdgeFlangeFeature {
  type: 'edgeFlange';
  id: string;
  name?: string;
  edges: SheetEdgePick[];
  height: Expr;
  /** degrees, default 90 */
  angle?: Expr;
  radius?: Expr;
  heightFrom?: 'outer' | 'inner' | 'tangent' | string;
  position?: 'inside' | 'outside' | 'middle' | string;
  flip?: boolean;
  /** chained bends */
  then?: SheetFlangeNext[];
  /** partial width: from the start of the edge by `start`/`length`, or between world points `from` and `to` on it */
  start?: Expr;
  length?: Expr;
  from?: number[];
  to?: number[];
  profile?: SheetProfile;
  /** each `{circle:{center:[v,h],radius}}` or a closed loop of segments */
  holes?: Array<Record<string, unknown> | SheetProfile>;
  /** the notch beside a partial flange */
  relief?: 'none' | 'rectangular' | { width?: Expr; depth?: Expr };
  [key: string]: unknown;
}

/** A 180 degree bend that lays a leg back against the sheet. */
export interface HemFeature {
  type: 'hem';
  id: string;
  name?: string;
  edges: SheetEdgePick[];
  length?: Expr;
  /** inside radius of the hem */
  gap?: Expr;
  flip?: boolean;
  start?: Expr;
  from?: number[];
  to?: number[];
  profile?: SheetProfile;
  holes?: Array<Record<string, unknown> | SheetProfile>;
  relief?: 'none' | 'rectangular' | { width?: Expr; depth?: Expr };
  [key: string]: unknown;
}

/** A bend across the sheet along a line: world `points`, or `line` naming a sketch line. */
export interface FoldFeature {
  type: 'fold';
  id: string;
  name?: string;
  points?: number[][];
  line?: { sketch: string; curve: string };
  angle?: Expr;
  radius?: Expr;
  position?: 'centerline' | 'start' | 'end' | 'mould' | string;
  /** the side that stays: a point [x,y,z] or a wall role such as `P0.side.e1` */
  fixed?: number[] | string;
  flip?: boolean;
  [key: string]: unknown;
}

/** Make the body the flat sheet. */
export interface UnfoldFeature {
  type: 'unfold';
  id: string;
  name?: string;
  [key: string]: unknown;
}

/** Fold the unfolded sheet again from the same bend rows. */
export interface RefoldFeature {
  type: 'refold';
  id: string;
  name?: string;
  [key: string]: unknown;
}

/** Read a bent plate of constant thickness as a sheet (refused when the volume differs by more than 1 percent). */
export interface SheetConvertFeature {
  type: 'sheetConvert';
  id: string;
  name?: string;
  thickness?: Expr;
  rule?: string;
  [key: string]: unknown;
}

/** Cut the closed regions of a sketch out of the sheet through the flat pattern. */
export interface SheetCutFeature {
  type: 'sheetCut';
  id: string;
  name?: string;
  sketch: string;
  regions?: string[];
  [key: string]: unknown;
}

/** Round or chamfer corners of free walls. */
export interface SheetCornerFeature {
  type: 'sheetCorner';
  id: string;
  name?: string;
  /** each a point [x,y,z] or `{panel, vertex}` */
  corners: Array<number[] | { panel: number; vertex: number }>;
  mode?: 'round' | 'chamfer' | string;
  radius?: Expr;
  distance?: Expr;
  /** alias of radius/distance */
  size?: Expr;
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
  | FreeformFeature | EditCurveFeature | ProjectCurveFeature | OffsetCurveFeature | CompositeCurveFeature | SweepSurfaceFeature | BlendSurfaceFeature | ExtendFeature
  | DomeFeature | SplitLineFeature | MoveFaceFeature | PressPullFeature | MoveBodyFeature
  | ScaleFeature | AxisFeature | ConnectorFeature | ThreadFeature | CombineFeature | SplitFeature | LipJointFeature | SphereFeature | KnurlBandFeature | WrapFeature | ImportStepFeature | RepairPartFeature | FrameFeature
  | TorusFeature | PipeFeature | CoilFeature | BoundingSolidFeature
  | SheetRulesFeature | BaseFlangeFeature | ContourFlangeFeature | EdgeFlangeFeature | HemFeature | FoldFeature
  | UnfoldFeature | RefoldFeature | SheetConvertFeature | SheetCutFeature | SheetCornerFeature;

/** All wire type strings the kernel dispatches on. */
export const FEATURE_TYPES = [
  'sketch', 'sketch3d', 'extrude', 'revolve', 'sweep', 'loft',
  'fillet', 'chamfer', 'draft', 'shell', 'hole', 'helix', 'rib', 'lip',
  'lpattern', 'cpattern', 'curvepattern', 'fillpattern', 'spattern', 'mirror',
  'trimSurface', 'surfacePlanar', 'surfaceRevolve', 'surfaceExtrude',
  'surfaceOffset', 'knit', 'fill', 'insertPart', 'mate', 'plane',
  'deleteBody', 'deleteFace', 'thicken', 'cutWithSurface', 'pipeCut',
  'freeform', 'editCurve', 'projectCurve', 'offsetCurve', 'compositeCurve', 'sweepSurface', 'blendSurface', 'extend', 'dome', 'splitLine',
  'moveFace', 'pressPull', 'moveBody', 'scale', 'axis', 'connector', 'thread', 'combine', 'split', 'lipJoint', 'sphere', 'knurlBand', 'wrap', 'importStep', 'repairPart', 'frame',
  'torus', 'pipe', 'coil', 'boundingSolid',
  'sheetRules', 'baseFlange', 'edgeFlange', 'hem', 'fold', 'contourFlange', 'unfold', 'refold', 'sheetConvert', 'sheetCut', 'sheetCorner',
] as const;

export type FeatureTypeName = (typeof FEATURE_TYPES)[number];

export interface SketchSpec {
  points: SketchPoint[];
  entities: SketchEntity[];
  constraints: SketchConstraint[];
}
export type { FeatureId };

/**
 * Solver status of one solved sketch, present on a regen reply's `sketches[i][1].status`
 * only when the request carried `opts: { sketchStatus: true }` (docs/sketch-status.md).
 * The original kernel has no such field.
 */
export interface SketchSolverStatus {
  /** Unknowns (x,y of every non-fixed point, r of circles/ellipses) minus Jacobian rank. */
  dof: number;
  /** state === 'full' and solved. */
  fullyDefined: boolean;
  /** Constraint ids that cannot hold together with earlier ones (document order). */
  conflicts: string[];
  /** Constraint ids implied by earlier ones (document order). */
  redundant: string[];
  /** The coordinates satisfy every constraint, or a solve from them converges. */
  solved: boolean;
  /** 'over' when any constraint is dependent, else 'full' at dof 0, else 'under'. */
  state: 'under' | 'full' | 'over';
  /** max |constraint residual| at the coordinates in the reply (-1 if unavailable). */
  residual: number;
  /** Coded problems (docs/sketch-status.md "Problems"); the key is absent when there are none. */
  problems?: SketchProblem[];
}

/** One sketch problem: `sketchCurveShrunk` (the solve was not taken), `sketchSameCurveTwice`,
 *  `sketchMidpointOwnEnd`, `sketchLineEndsCoincident`. */
export interface SketchProblem {
  code: string;
  /** Plain words, e.g. "that would shrink e4 to a point: its constraints can only be met without it". */
  message: string;
  constraint?: string;
  entity?: string;
}
