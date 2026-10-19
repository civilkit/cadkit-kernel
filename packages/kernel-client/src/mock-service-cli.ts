// Run the mock kernel service on its own (node dist/kernel-client/src/mock-service-cli.js [port]): the wasm kernel behind the same HTTP
// shape as ck-native, for trying the app's "Local service" setting or running the kernel suites with CK_KERNEL_URL before ck-native exists.
// CK_WASM picks the kernel; the token is MOCK_TOKEN or a fresh random one printed on start.
import { randomBytes } from 'node:crypto';
import { startMockService } from './mock-service.js';
import { defaultWasmPath } from './worker.js';
import { WasmHost } from './wasm-host.js';
import { readFileSync } from 'node:fs';

const wasmPath = defaultWasmPath();
const commit = new WasmHost(new WebAssembly.Module(readFileSync(wasmPath))).commit;
const svc = await startMockService({
  wasmPath, port: Number(process.argv[2] ?? 7461), token: process.env.MOCK_TOKEN ?? randomBytes(16).toString('hex'),
  version: { kernel: 'mock', build: commit, git: commit.split(' ').pop() ?? '' },
});
console.log(`mock kernel service on ${svc.url}`);
console.log(`token: ${svc.token}`);
