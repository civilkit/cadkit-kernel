import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { decodeReply, encodeRequest, toKernelReply, type DecodedBuf } from './protocol.js';

function envelope(value: unknown, bufs: Array<{ type: 'f32'; data: number[] }>): Uint8Array {
  const json = new TextEncoder().encode(JSON.stringify(value));
  const base = (4 + json.length + 7) & ~7;
  const payload = new Float32Array(bufs.flatMap((b) => b.data));
  const out = new Uint8Array(base + payload.byteLength);
  new DataView(out.buffer).setUint32(0, json.length, true);
  out.set(json, 4);
  out.set(new Uint8Array(payload.buffer), base);
  return out;
}

describe('protocol', () => {
  it('encodes id+msg envelope', () => {
    assert.equal(encodeRequest(7, { type: 'ping' }), '{"id":7,"msg":{"type":"ping"}}');
  });

  it('decodes $buf markers to typed arrays', () => {
    const bytes = envelope({ ok: true, result: { mesh: { $buf: 0, type: 'f32', len: 6, offset: 0 } } }, [{ type: 'f32', data: [1, 2, 3, 4, 5, 6] }]);
    const { value } = decodeReply(bytes);
    const mesh = (value as { result: { mesh: DecodedBuf } }).result.mesh;
    assert.equal(mesh.$buf, 0);
    assert.equal(mesh.len, 6);
    assert.ok(mesh.data instanceof Float32Array);
    assert.deepEqual(Array.from(mesh.data), [1, 2, 3, 4, 5, 6]);
  });

  it('passes plain JSON through untouched', () => {
    const bytes = envelope({ ok: false, error: 'nope' }, []);
    const { json, value } = decodeReply(bytes);
    assert.deepEqual(value, JSON.parse(json));
  });

  it('maps envelopes to KernelReply', () => {
    assert.deepEqual(toKernelReply({ ok: true, result: { a: 1 } }), { ok: true, result: { a: 1 } });
    assert.deepEqual(toKernelReply({ ok: false, error: 'x' }), { ok: false, error: 'x' });
  });
});
