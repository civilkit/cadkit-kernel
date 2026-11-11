// The exporter's intermediate model: what `analyzeAssembly` reads from the kernel and `toMjcf` writes. CAD units throughout
// (millimetres, radians for hinge values and ranges, millimetres for slide values and ranges); masses in kg, inertia in kg m².
import type { Pose, V3 } from './math.js';

export type JointType = 'hinge' | 'slide' | 'ball' | 'free';

export interface MjJoint {
  name: string;
  type: JointType;
  /** world direction (unit), at the exported pose; unused for ball / free */
  axis: V3;
  /** a world point on the axis (mm); the ball centre */
  pos: V3;
  /** [lo, hi] relative to the exported pose: rad for hinge, mm for slide */
  range?: [number, number];
  /** the mate the joint comes from (the trajectory import and the limits use it) */
  mate?: string;
  /** the limits mate that gave the range */
  limit?: string;
}

/** A primitive fitted to a part, in the part's own frame (mm). Sizes follow MJCF: box half extents, cylinder / capsule [r, half length], sphere [r]. */
export interface MjPrimitive { type: 'box' | 'cylinder' | 'sphere' | 'capsule'; size: number[]; pose: Pose }

export interface MjPart {
  key: string;
  /** sanitised, unique mesh base name */
  name: string;
  material: string;
  /** g/cm³ */
  density: number;
  /** mm³ */
  volume: number;
  /** kg */
  mass: number;
  /** centre of mass in the part frame, mm */
  com: V3;
  /** inertia tensor about the centre of mass, part axes, kg m² */
  inertia: number[][];
  /** triangle soup in the part frame, mm */
  positions: Float32Array;
  /** solid (lump) index per triangle */
  lumpOfTri: Int32Array;
  lumps: number;
  faceKinds: string[];
  /** per solid, from the B-rep (concave edges / faces); when absent the mesh is tested */
  convexSolids?: boolean[];
  primitive?: MjPrimitive;
}

export interface MjBody {
  /** component id (insertPart feature id) */
  id: string;
  /** sanitised, unique body name (the component's name) */
  name: string;
  /** parent component id, null = world */
  parent: string | null;
  /** world placement of the part frame, mm */
  place: Pose;
  part: string;
  joints: MjJoint[];
  /** fixed in the CAD or listed as static: welded to the world */
  fixed: boolean;
}

/** `joint1 = c0 + c1 * joint2` (MJCF joint equality), CAD units: rad for hinges, mm for slides. */
export interface MjCoupling { name: string; joint1: string; joint2: string; c0: number; c1: number; mate: string }

export interface MjPlan {
  model: string;
  /** parents before children */
  bodies: MjBody[];
  parts: Record<string, MjPart>;
  couplings: MjCoupling[];
  warnings: string[];
}
