import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

describe('session loads without Node-only imports at module load', () => {
  it('has no static node: or worker import in session.ts', () => {
    // Run from dist/session/src/, so reach back to the source tree (verified: ../../../session/src/session.ts).
    const src = readFileSync(fileURLToPath(new URL('../../../session/src/session.ts', import.meta.url)), 'utf8')
      .replace(/^\s*\/\/.*$/gm, '');
    const staticImports = [...src.matchAll(/^import[^;]*from\s+'([^']+)';/gms)].map((m) => m[1]);
    const bad = staticImports.filter((s) => s.startsWith('node:') || s.includes('kernel-client/src/worker'));
    assert.deepEqual(bad, []);
  });
});
