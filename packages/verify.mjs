// One command for the agent loop: verify a part file, print the report.
// usage: node packages/verify.mjs <part.json> [--expect expect.json] [--tag t] [--wasm path] [--tol n]
// (build first: npm --prefix packages run build)
import { readFileSync } from 'node:fs';
import { createWorkerKernel } from './dist/kernel-client/src/worker.js';
import { verify } from './dist/driver/src/verify.js';

const args = process.argv.slice(2);
const opt = (f) => { const i = args.indexOf(f); return i < 0 ? null : args[i + 1]; };
const file = args.find((a) => !a.startsWith('--'));
if (!file) { console.error('usage: node packages/verify.mjs <part.json> [--expect expect.json] [--tag t] [--wasm path] [--tol n]'); process.exit(2); }

const part = JSON.parse(readFileSync(file, 'utf8'));
const expect = opt('--expect') ? JSON.parse(readFileSync(opt('--expect'), 'utf8')) : undefined;
if (opt('--wasm')) process.env.CK_WASM = opt('--wasm');
const k = await createWorkerKernel();
try {
  console.log(JSON.stringify(await verify(k, part, {
    tag: opt('--tag') ?? undefined,
    tol: opt('--tol') ? +opt('--tol') : undefined,
    expect,
  })));
} finally {
  await k.close();
}
