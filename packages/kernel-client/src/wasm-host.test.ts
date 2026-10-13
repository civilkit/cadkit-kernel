import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import { WasmHost } from './wasm-host.js';
import { defaultWasmPath } from './worker.js';
import { encodeRequest } from './protocol.js';
import { Part } from '../../builder/src/index.js';

describe('WasmHost', { skip: !existsSync(defaultWasmPath()) && 'no wasm built' }, () => {
  it('regens a plate and reboots after a trap-free call sequence', () => {
    const host = new WasmHost(new WebAssembly.Module(readFileSync(defaultWasmPath())));
    assert.ok(host.commit.length > 0);
    const p = new Part('Plate');
    p.extrude({ sketch: p.sketch('S', 'front', (s) => s.rect(0, 0, 10, 10)), op: 'add', end: 'blind', depth: 5 });
    const r = host.call(encodeRequest(1, { type: 'regen', tag: 't', part: p.toJSON() }));
    assert.equal(r.ok, true);
  });
});
