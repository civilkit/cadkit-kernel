import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { WasmHost } from './wasm-host.js';
import { defaultWasmPath } from './worker.js';
import { encodeRequest } from './protocol.js';
import { bytesToBase64, configureF3dDecoder, F3D_DECODER_MISSING, F3dDecoderMissingError, loadF3dDecoder } from './f3d-decoder.js';

const here = path.dirname(fileURLToPath(import.meta.url));
// CK_F3D_WASM wins; otherwise the release build next to the kernel wasm
const f3dWasm = process.env.CK_F3D_WASM ?? path.join(path.dirname(defaultWasmPath()), 'ck_wasm_f3d.wasm');
// next to this file when run from src, under packages/kernel-client/src when run from the compiled dist
const fixture = [path.join(here, 'fixtures/cube.smb'), path.resolve(here, '../../../kernel-client/src/fixtures/cube.smb')].find((p) => existsSync(p));
const cube = fixture ? readFileSync(fixture) : null;

describe('the lazily loaded f3d decoder', () => {
  it('says so when there is no decoder', async () => {
    configureF3dDecoder(null);
    await assert.rejects(loadF3dDecoder(), (e: unknown) => e instanceof F3dDecoderMissingError && e.code === F3D_DECODER_MISSING);
    configureF3dDecoder(async () => new Uint8Array([1, 2, 3]));
    await assert.rejects(loadF3dDecoder(), (e: unknown) => e instanceof F3dDecoderMissingError);
    configureF3dDecoder(null);
  });

  const skip = (!existsSync(f3dWasm) && `no ${f3dWasm} built`) || (!existsSync(defaultWasmPath()) && 'no kernel wasm built') || (!cube && 'no cube.smb fixture');
  it('decodes a blob in the second wasm and the main kernel imports it', { skip }, async () => {
    configureF3dDecoder(() => Promise.resolve(readFileSync(f3dWasm)));
    const dec = await loadF3dDecoder();
    assert.equal(loadF3dDecoder(), loadF3dDecoder()); // cached
    assert.match(dec.commit, /^ck /);
    const d = dec.decode(cube!, { name: 'cube' });
    assert.ok(d.decoded, JSON.stringify(d));
    assert.equal(d.decoded.info.format, 'f3d');
    assert.equal(d.decoded.info.solids, 1);

    const kernel = new WasmHost(new WebAssembly.Module(readFileSync(defaultWasmPath())));
    const r = kernel.call(encodeRequest(1, { type: 'importF3d', tag: 't', name: 'cube', decoded: d.decoded }));
    assert.equal(r.ok, true, JSON.stringify(r));
    const res = r.result as { body: { faces: unknown[] }; import: { faces: number; report: string[] } };
    assert.equal(res.body.faces.length, 6);
    assert.equal(res.import.faces, 6);

    // the main wasm cannot decode by itself, and says what to do
    const raw = kernel.call(encodeRequest(2, { type: 'importF3d', tag: 'u', data: bytesToBase64(cube!) }));
    const err = (raw.result as { errors: Array<[string, { code: string }]> }).errors[0]!;
    assert.equal(err[1].code, F3D_DECODER_MISSING);

    // an unreadable file is an error from the decoder, not a throw
    const bad = dec.decode(new TextEncoder().encode('not a fusion file'));
    assert.ok(bad.errors && bad.errors[0]![1].code.startsWith('f3d.'));
  });
});
