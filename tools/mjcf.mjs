#!/usr/bin/env node
// MuJoCo interop from the command line (build the packages first: cd packages && npm run build). See docs/mjcf.md.
//
//   node tools/mjcf.mjs export <assembly.ckpart> <out dir> [--options opts.json] [--sim sim.json] [--wasm kernel.wasm]
//       writes <out dir>/<model>.xml, <out dir>/meshes/*.stl, <out dir>/<model>.meta.json; prints warnings and hints
//   node tools/mjcf.mjs motion <assembly.ckpart> <log.csv|log.json> <out.ckpart> --meta <model.meta.json> [--name N] [--units m|mm] [--max-keys K]
//       stores the logged joint values as a motion study (doc.motion.studies) in a copy of the assembly
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const dist = path.resolve(here, '../packages/dist');
const args = process.argv.slice(2);
const flag = (name) => { const i = args.indexOf(name); if (i < 0) return undefined; const v = args[i + 1]; args.splice(i, 2); return v; };
const usage = () => { console.error(readFileSync(fileURLToPath(import.meta.url), 'utf8').split('\n').slice(1, 8).join('\n')); process.exit(2); };

const mj = await import(path.join(dist, 'mjcf/src/index.js'));
const cmd = args.shift();
if (cmd === 'export') {
  const optsFile = flag('--options'), simFile = flag('--sim'), wasm = flag('--wasm');
  const [docFile, out] = args;
  if (!docFile || !out) usage();
  const opts = optsFile ? JSON.parse(readFileSync(optsFile, 'utf8')) : {};
  if (simFile) opts.sim = { ...(opts.sim ?? {}), ...mj.readSimFile(readFileSync(simFile, 'utf8')) };
  const { createWorkerKernel } = await import(path.join(dist, 'kernel-client/src/worker.js'));
  const k = await createWorkerKernel(wasm);
  try {
    const r = await mj.exportMjcf(k, JSON.parse(readFileSync(docFile, 'utf8')), opts);
    const file = await mj.writeMjcf(out, r);
    console.log(`wrote ${file} (${r.plan.bodies.length} bodies, ${Object.keys(r.meta.joints).length} joints, ${Object.keys(r.files).length} meshes)`);
    for (const w of r.meta.warnings) console.log(`warning: ${w}`);
    for (const h of r.hints.filter((x) => !x.convex)) console.log(`hint: ${h.file}: ${h.suggestion}`);
  } finally { await k.close(); }
} else if (cmd === 'motion') {
  const meta = flag('--meta'), name = flag('--name'), units = flag('--units'), maxKeys = flag('--max-keys');
  const [docFile, logFile, out] = args;
  if (!docFile || !logFile || !out || !meta) usage();
  const doc = JSON.parse(readFileSync(docFile, 'utf8'));
  const traj = mj.readTrajectory(readFileSync(logFile, 'utf8'));
  const { study, unknown } = mj.motionStudy(JSON.parse(readFileSync(meta, 'utf8')), traj, { name, units, maxKeys: maxKeys ? Number(maxKeys) : undefined });
  writeFileSync(out, JSON.stringify(mj.withMotionStudy(doc, study)));
  console.log(`wrote ${out}: study "${study.name}", ${study.tracks.length} tracks, ${study.duration} s`);
  if (unknown.length) console.log(`not exported joints (ignored): ${unknown.join(', ')}`);
} else usage();
