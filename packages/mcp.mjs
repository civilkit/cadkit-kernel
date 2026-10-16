// MCP stdio entry: run with `node packages/mcp.mjs`.
// env: CK_WASM overrides the kernel wasm path; CK_NO_AUTOBUILD=1 skips the stale-build check.
import { execFileSync } from 'node:child_process';
import { existsSync, readdirSync, statSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));

/** Newest modification time (ms) under dir for files whose name passes `keep`. */
function newest(dir, keep) {
  let best = 0;
  if (!existsSync(dir)) return best;
  for (const e of readdirSync(dir, { withFileTypes: true })) {
    if (e.name === 'node_modules' || e.name === 'dist' || e.name === 'target') continue;
    const p = join(dir, e.name);
    if (e.isDirectory()) best = Math.max(best, newest(p, keep));
    else if (keep(e.name)) best = Math.max(best, statSync(p).mtimeMs);
  }
  return best;
}

// 1. compiled packages older than their sources: rebuild before serving (a stale dist silently
//    rejects features the source already has, e.g. "unknown feature type").
const entry = join(here, 'dist/session/src/mcp-server.js');
const srcTime = newest(here, (n) => n.endsWith('.ts') && !n.endsWith('.d.ts'));
if (process.env.CK_NO_AUTOBUILD !== '1' && (!existsSync(entry) || statSync(entry).mtimeMs < srcTime)) {
  console.error('mcp: compiled packages are older than the sources, rebuilding (tsc)...');
  execFileSync('npm', ['run', 'build'], { cwd: here, stdio: ['ignore', 2, 2] });
}

// 2. kernel wasm older than the Rust sources: serve it, but say so (a wasm build needs the CI box:
//    tools/remote_build.sh, see docs/remote-build.md).
const wasm = process.env.CK_WASM || join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
if (!process.env.CK_WASM && existsSync(wasm)) {
  const rustTime = newest(join(here, '../crates'), (n) => n.endsWith('.rs') || n === 'Cargo.toml');
  if (statSync(wasm).mtimeMs < rustTime) {
    console.error('mcp: WARNING kernel wasm is older than the Rust sources; run tools/sync_stack.sh --wasm to rebuild it');
  }
}

const { StdioServerTransport } = await import('@modelcontextprotocol/sdk/server/stdio.js');
const { createMcpServer } = await import('./dist/session/src/mcp-server.js');

const { server } = createMcpServer();
await server.connect(new StdioServerTransport());
