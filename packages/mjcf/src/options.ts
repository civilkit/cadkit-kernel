// Options of the MJCF export and the sim-only block (data a CAD model does not carry).
import type { V3 } from './math.js';
import type { JointType } from './plan.js';

export type CollisionMode = 'hull' | 'solids' | 'primitive' | 'none';

/** A joint stated by hand for a component, replacing what the mates give. World frame, mm, CAD pose = 0. */
export interface JointOverride {
  type: JointType | 'weld';
  axis?: V3;
  pos?: V3;
  /** rad (hinge) / mm (slide) */
  range?: [number, number];
  name?: string;
  /** parent component id or name; null = world */
  parent?: string | null;
}

/** Contact parameters, MJCF names and meaning. */
export interface ContactParams {
  friction?: number[]; solref?: number[]; solimp?: number[]; condim?: number; margin?: number; gap?: number;
  priority?: number; contype?: number; conaffinity?: number;
}

export type Attrs = Record<string, string | number | number[] | boolean>;

/** Sim-only data: everything here is written as MJCF and never read back into the CAD. Names refer to the exported
 *  body / joint names (component and mate names, sanitised). The same object can live in a JSON side file (`readSimFile`). */
export interface SimBlock {
  /** `<option>` attributes (timestep, gravity, integrator, ...) */
  option?: Attrs;
  /** extra `<compiler>` attributes */
  compiler?: Attrs;
  /** `<size>`, `<visual>`... as raw attribute maps are rare; use `raw` */
  contact?: {
    /** every collision geom */
    default?: ContactParams;
    /** by body (component) name or part name */
    bodies?: Record<string, ContactParams>;
    /** body pairs that never collide */
    exclude?: Array<[string, string]>;
  };
  /** joint attributes by joint name; `*` applies to every exported joint */
  joints?: Record<string, Attrs>;
  /** `<actuator>` children: `{type: 'motor' | 'position' | 'velocity' | 'general' | ..., name, joint, ...attributes}` */
  actuators?: Array<{ type?: string } & Attrs>;
  /** `<sensor>` children: `{type: 'jointpos' | 'framepos' | ..., ...attributes}` */
  sensors?: Array<{ type: string } & Attrs>;
  /** raw MJCF appended inside `<worldbody>` (extra bodies: balls, floors, lights, cameras) */
  worldbody?: string;
  /** raw MJCF appended inside `<asset>` */
  asset?: string;
  /** raw MJCF appended at the end of `<mujoco>` (equality, tendon, contact pairs, keyframes...) */
  raw?: string;
  /** `<include file>` entries, relative to the model file */
  include?: string[];
}

export interface MjcfOptions {
  /** model name (default: the document name) */
  model?: string;
  /** 'm' (default) writes metres; 'mm' keeps millimetres (then set `sim.option.gravity` yourself) */
  units?: 'm' | 'mm';
  /** the CAD axis that points up: 'z' (default) keeps the frame; 'y' turns the model so CAD +y is MuJoCo +z (gravity -z) */
  up?: 'z' | 'y';
  /** folder of the mesh files, relative to the model file (default 'meshes') */
  meshDir?: string;
  /** collision geometry (default 'hull') */
  collision?: CollisionMode;
  /** per component or part name */
  collisionByPart?: Record<string, CollisionMode>;
  /** component or part names that get a visual mesh only */
  visualOnly?: string[];
  /** component names welded to the world whatever their mates say */
  static?: string[];
  /** components with no mate: 'free' (default) gets a freejoint, 'fixed' is welded to the world */
  unmated?: 'free' | 'fixed';
  /** g/cm³ by material name (case-insensitive), over the built-in table */
  densities?: Record<string, number>;
  /** g/cm³ for a material the table does not know (default 1, with a warning) */
  defaultDensity?: number;
  /** joints by component id or name, replacing the ones the mates give */
  joints?: Record<string, JointOverride>;
  sim?: SimBlock;
  /** rgba of the visual geoms by part / component name; default a grey */
  colors?: Record<string, [number, number, number, number]>;
}

/** Parse and check a JSON sim side file. */
export function readSimFile(text: string): SimBlock {
  const o = JSON.parse(text) as unknown;
  if (!o || typeof o !== 'object' || Array.isArray(o)) throw new Error('sim file: expected a JSON object');
  const known = new Set(['option', 'compiler', 'contact', 'joints', 'actuators', 'sensors', 'worldbody', 'asset', 'raw', 'include']);
  for (const k of Object.keys(o)) if (!known.has(k)) throw new Error(`sim file: unknown key "${k}" (known: ${[...known].join(', ')})`);
  return o as SimBlock;
}

/** Densities in g/cm³ (same table as the GUI's mass properties); matched ignoring case. */
export const DENSITIES: Record<string, number> = {
  steel: 7.85, 'stainless steel': 8.0, stainless: 8.0, iron: 7.87, aluminium: 2.7, aluminum: 2.7, brass: 8.5, bronze: 8.8, copper: 8.96,
  titanium: 4.5, zinc: 7.14, lead: 11.34, magnesium: 1.74, nickel: 8.9,
  abs: 1.04, pla: 1.24, petg: 1.27, nylon: 1.15, pc: 1.2, polycarbonate: 1.2, pom: 1.41, acetal: 1.41, pp: 0.9, hdpe: 0.95, resin: 1.1,
  wood: 0.7, oak: 0.75, pine: 0.5, glass: 2.5, concrete: 2.4,
};
