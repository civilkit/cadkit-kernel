// MuJoCo interop for assemblies: MJCF export (bodies, joints, meshes, inertials, sim-only block) and joint-trajectory import
// as a motion study. See docs/mjcf.md.
import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';
import type { KernelTransport } from '../../driver/src/verify.js';
import { analyzeAssembly, type AnalyzeOptions, type AssemblyDoc } from './analyze.js';
import { toMjcf, type MjcfResult } from './mjcf.js';
import type { MjPlan } from './plan.js';

export { analyzeAssembly, componentOfFace, inferJoints, safeName, type AnalyzeOptions, type AssemblyDoc, type Geo } from './analyze.js';
export { toMjcf, bodyPoses, type MjcfResult, type MjcfMeta, type DecompositionHint } from './mjcf.js';
export { readSimFile, DENSITIES, type MjcfOptions, type SimBlock, type JointOverride, type CollisionMode, type ContactParams } from './options.js';
export { fitPrimitive, isConvex, meshVolume } from './geom.js';
export { readStl, writeStl } from './stl.js';
export { parse as parseXml, serialize as serializeXml, type XmlNode } from './xml.js';
export * from './trajectory.js';
export type * from './plan.js';
export type { Pose, V3 } from './math.js';

/** Analyse the assembly and write the MJCF in memory. */
export async function exportMjcf(kernel: KernelTransport, doc: AssemblyDoc, opts: AnalyzeOptions = {}): Promise<MjcfResult & { plan: MjPlan }> {
  const plan = await analyzeAssembly(kernel, doc, opts);
  return { ...toMjcf(plan, opts), plan };
}

/** Write `<dir>/<file>` (default `<model>.xml`), the mesh files and `<model>.meta.json`. Returns the model file path. */
export async function writeMjcf(dir: string, r: MjcfResult, file?: string): Promise<string> {
  await mkdir(dir, { recursive: true });
  const out = path.join(dir, file ?? `${r.meta.model}.xml`);
  await writeFile(out, r.xml);
  for (const [rel, bytes] of Object.entries(r.files)) {
    const p = path.join(dir, rel);
    await mkdir(path.dirname(p), { recursive: true });
    await writeFile(p, bytes);
  }
  await writeFile(path.join(dir, `${path.basename(out, '.xml')}.meta.json`), JSON.stringify(r.meta, null, 1));
  return out;
}
