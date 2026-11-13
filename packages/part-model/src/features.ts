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
  kind:
    | 'coincident' | 'concentric' | 'distance' | 'angle' | 'parallel'
    | 'perpendicular' | 'tangent' | 'lock' | 'symmetric' | 'width'
    | 'hinge' | 'slot' | 'screw' | 'gear' | 'rackPinion' | 'limits'
    | string;
  a?: string | null;
  b?: string | null;
  /** Second face of the mover / target side (width, hinge stop, slot wall). */
  a2?: string | null;
  b2?: string | null;
  /** Mirror face of `symmetric`. */
  c?: string | null;
  distance?: Expr;
  /** Degrees: the `angle` mate's angle, or the drive of gear / rackPinion / screw. */
  angle?: Expr;
  ratio?: Expr;
  pitch?: Expr;
  radius?: Expr;
  /** `limits`: range of the measured distance (or of the angle, degrees, with `of: 'angle'`). */
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
  | ScaleFeature | AxisFeature | ThreadFeature | CombineFeature | SplitFeature | LipJointFeature | SphereFeature | KnurlBandFeature | ImportStepFeature | RepairPartFeature | FrameFeature;

/** All wire type strings the kernel dispatches on. */
export const FEATURE_TYPES = [
  'sketch', 'sketch3d', 'extrude', 'revolve', 'sweep', 'loft',
  'fillet', 'chamfer', 'draft', 'shell', 'hole', 'helix', 'rib', 'lip',
  'lpattern', 'cpattern', 'curvepattern', 'fillpattern', 'spattern', 'mirror',
  'trimSurface', 'surfacePlanar', 'surfaceRevolve', 'surfaceExtrude',
  'surfaceOffset', 'knit', 'fill', 'insertPart', 'mate', 'plane',
  'deleteBody', 'deleteFace', 'thicken', 'cutWithSurface', 'pipeCut',
  'freeform', 'editCurve', 'projectCurve', 'extend', 'dome', 'splitLine',
  'moveFace', 'moveBody', 'scale', 'axis', 'thread', 'combine', 'split', 'lipJoint', 'sphere', 'knurlBand', 'importStep', 'repairPart', 'frame',
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
