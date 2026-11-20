// Kernel wire-protocol types (regen / massProps / step + mesh positions).
// Other message types are added here as they get wrapped, one at a time.

import type { PartDoc } from '../../part-model/src/index.js';

/** Outgoing kernel messages (subset). `tag` is the app's dedup/replace key. */
export type KernelRequest =
  | { type: 'regen'; tag: string; part: PartDoc; opts?: unknown }
  | { type: 'massProps'; tag: string; bodies?: boolean; densities?: number[] }
  | { type: 'step'; tag: string; name: string };

export type KernelErrorRef = [featureId: string, err: { code: string; params: Record<string, unknown> }];

export interface RegenBody {
  empty?: boolean;
  box?: { min: number[]; max: number[] };
  faces?: Array<Record<string, unknown>>;
  edges?: Array<Record<string, unknown>>;
  mesh?: { positions: ArrayLike<number>; normals?: ArrayLike<number>; triFace?: ArrayLike<number> };
}

export interface RegenReply {
  errors?: KernelErrorRef[];
  body?: RegenBody;
}

export interface MassProps {
  volume?: number;
  area?: number;
  com?: number[];
  moments?: number[];
  axes?: unknown;
  [key: string]: unknown;
}

export interface KernelEvalResult {
  ms: number;
  errors: KernelErrorRef[];
  ok: boolean;
  box?: { min: number[]; max: number[] };
  faces?: number;
  edges?: number;
  triangles?: number;
  faceList?: Array<Record<string, unknown>>;
  mass?: MassProps;
  /** raw STEP text — present only when requested; stripped before returning */
  stepText?: string;
  /** raw mesh positions — present only when requested; stripped before returning */
  positions?: number[];
}
