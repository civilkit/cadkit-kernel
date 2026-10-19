import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { NativeKernel, NativeConnectError, normalizeServiceUrl, probeService, readVersion, versionMismatch, type NativeFailure } from './native.js';
import { startMockService } from './mock-service.js';
import { KERNEL_PROTOCOL } from './protocol.js';
import { createWorkerKernel, defaultWasmPath } from './worker.js';
import { Part } from '../../builder/src/index.js';

const quiet = { keepAliveMs: 0 };

describe('service address and version', () => {
  it('normalizes the address and refuses odd ones', () => {
    assert.equal(normalizeServiceUrl('127.0.0.1:7461'), 'http://127.0.0.1:7461');
    assert.equal(normalizeServiceUrl(' http://localhost:7461/ '), 'http://localhost:7461');
    assert.throws(() => normalizeServiceUrl('ftp://x'));
    assert.throws(() => normalizeServiceUrl('http://user:pw@localhost'));
    assert.throws(() => normalizeServiceUrl(''));
  });
  it('reads the /version document under either naming and compares it', () => {
    const v = readVersion({ kernel: '1.2.3', protocol: 1, git_hash: 'abc', build_id: 'b1', cores: 8 })!;
    assert.deepEqual(v, { kernel: '1.2.3', protocol: 1, git: 'abc', build: 'b1', cores: 8 });
    assert.equal(readVersion({ nothing: true }), null);
    assert.equal(versionMismatch(v, { protocol: 1 }), null);
    assert.equal(versionMismatch(v, { protocol: 1, build: 'b1' }), null);
    assert.match(versionMismatch(v, { protocol: 2 })!, /protocol 1/);
    assert.match(versionMismatch(v, { protocol: 1, build: 'other' })!, /build b1/);
    const w = { ...v, build: '0123456789ab', git: '0123456789ab' };
    assert.equal(versionMismatch(w, { protocol: 1, build: 'class-a-surfacing 0123456789abcdef0123456789abcdef01234567' }), null, 'short hash of the same commit');
    assert.match(versionMismatch(w, { protocol: 1, build: 'class-a-surfacing fedcba9876540000000000000000000000000000' })!, /build 0123456789ab/);
  });
});

describe('handshake', () => {
  it('accepts a matching service and sends the token only as a header', async () => {
    const svc = await startMockService();
    try {
      const p = await probeService({ url: svc.url, token: svc.token });
      assert.equal(p.ok, true);
      assert.equal(svc.requests[0].auth, `Bearer ${svc.token}`);
      assert.ok(!svc.requests[0].path.includes(svc.token));
    } finally { await svc.stop(); }
  });
  it('refuses a wrong protocol or build, a wrong token, a dead address and a non-service', async () => {
    const svc = await startMockService({ version: { protocol: KERNEL_PROTOCOL + 1 } });
    try {
      const m = await probeService({ url: svc.url, token: svc.token });
      assert.ok(!m.ok && m.kind === 'mismatch' && /protocol/.test(m.message));
      const b = await probeService({ url: svc.url, token: svc.token, expect: { protocol: KERNEL_PROTOCOL + 1, build: 'not-this' } });
      assert.ok(!b.ok && b.kind === 'mismatch' && /build/.test(b.message));
      const u = await probeService({ url: svc.url, token: 'wrong' });
      assert.ok(!u.ok && u.kind === 'unauthorized');
      await assert.rejects(NativeKernel.connect({ url: svc.url, token: svc.token }), (e) => e instanceof NativeConnectError && e.kind === 'mismatch');
    } finally { await svc.stop(); }
    const gone = await probeService({ url: svc.url, token: svc.token });
    assert.ok(!gone.ok && gone.kind === 'unreachable');
    const bad = await probeService({ url: 'not a url ::', token: 'x' });
    assert.ok(!bad.ok && bad.kind === 'invalid');
  });
});

describe('NativeKernel transport', () => {
  it('opens its session on the first request, reuses it, sends the wasm request body, and deletes it on close', async () => {
    const svc = await startMockService();
    const k = await NativeKernel.connect({ url: svc.url, token: svc.token, ...quiet });
    try {
      assert.equal(svc.requests.filter((r) => r.path === '/session').length, 0);
      const a = await k.request({ type: 'one' });
      const b = await k.request({ type: 'two' });
      assert.deepEqual(a, { ok: true, result: { echo: 'one' } });
      assert.deepEqual(b, { ok: true, result: { echo: 'two' } });
      assert.equal(svc.requests.filter((r) => r.path === '/session').length, 1);
      const rpc = svc.requests.filter((r) => r.path.endsWith('/rpc'));
      assert.equal(rpc.length, 2);
      assert.deepEqual(JSON.parse(rpc[0].body).msg, { type: 'one' });
      assert.ok(svc.requests.every((r) => r.auth === `Bearer ${svc.token}`));
      assert.ok(svc.requests.every((r) => !r.path.includes(svc.token)));
      await k.close();
      assert.deepEqual(svc.deleted, ['s1']);
    } finally { await k.close(); await svc.stop(); }
  });

  it('sends concurrent requests one at a time in call order', async () => {
    const order: string[] = [];
    const svc = await startMockService({ onRpc: async (body) => { order.push(JSON.parse(body).msg.type); return { delayMs: order.length === 1 ? 30 : 0 }; } });
    const k = await NativeKernel.connect({ url: svc.url, token: svc.token, ...quiet });
    try {
      await Promise.all(['a', 'b', 'c'].map((type) => k.request({ type })));
      assert.deepEqual(order, ['a', 'b', 'c']);
    } finally { await k.close(); await svc.stop(); }
  });

  it('cancel aborts the request in flight and drops the queued ones; the session stays usable', async () => {
    const svc = await startMockService({ onRpc: (body) => (JSON.parse(body).msg.type === 'slow' ? { hang: true } : undefined) });
    const failures: NativeFailure[] = [];
    const k = await NativeKernel.connect({ url: svc.url, token: svc.token, onFailure: (f) => failures.push(f), ...quiet });
    try {
      const slow = k.request({ type: 'slow' });
      const queued = k.request({ type: 'queued' });
      await new Promise((r) => setTimeout(r, 50));
      k.cancel();
      assert.deepEqual(await slow, { ok: false, error: 'cancelled' });
      assert.deepEqual(await queued, { ok: false, error: 'cancelled' });
      await new Promise((r) => setTimeout(r, 30));
      assert.equal(svc.aborted, 1, 'the server saw the connection dropped');
      assert.equal(svc.requests.filter((r) => r.body.includes('queued')).length, 0);
      assert.deepEqual(await k.request({ type: 'after' }), { ok: true, result: { echo: 'after' } });
      assert.equal(failures.length, 0, 'a cancel is not a failure');
    } finally { await k.close(); await svc.stop(); }
  });

  it('maps a crash reply to a trap and keeps going; a refusal to an error', async () => {
    const svc = await startMockService({ onRpc: (body) => {
      const t = JSON.parse(body).msg.type;
      return t === 'boom' ? { status: 500, body: 'panic: index out of bounds' } : t === 'big' ? { status: 413, body: 'body too large' } : undefined;
    } });
    const failures: NativeFailure[] = [];
    const k = await NativeKernel.connect({ url: svc.url, token: svc.token, onFailure: (f) => failures.push(f), ...quiet });
    try {
      const a = await k.request({ type: 'boom' });
      assert.equal(a.ok, false);
      assert.match(a.trap ?? '', /kernel crashed \(500\).*index out of bounds/);
      const b = await k.request({ type: 'big' });
      assert.equal(b.ok, false);
      assert.equal(b.trap, undefined);
      assert.match(b.error ?? '', /413/);
      assert.equal((await k.request({ type: 'fine' })).ok, true);
      assert.equal(failures.length, 0);
    } finally { await k.close(); await svc.stop(); }
  });

  it('reports a vanished service once, as a trap, and refuses later requests without touching the network', async () => {
    const svc = await startMockService();
    const failures: NativeFailure[] = [];
    const k = await NativeKernel.connect({ url: svc.url, token: svc.token, onFailure: (f) => failures.push(f), ...quiet });
    assert.equal((await k.request({ type: 'x' })).ok, true);
    await svc.stop();
    const r = await k.request({ type: 'y' });
    assert.equal(r.ok, false);
    assert.match(r.trap ?? '', /local kernel service/);
    const r2 = await k.request({ type: 'z' });
    assert.equal(r2.ok, false);
    assert.deepEqual(failures.map((f) => f.kind), ['unreachable']);
    assert.ok(!JSON.stringify([r, r2, failures]).includes(svc.token));
    await k.close();
  });

  it('reports an expired session, a rejected token and a timeout as failures', async () => {
    const svc = await startMockService({ onRpc: (body) => (JSON.parse(body).msg.type === 'stall' ? { hang: true } : undefined) });
    try {
      const f1: NativeFailure[] = [];
      const k1 = await NativeKernel.connect({ url: svc.url, token: svc.token, onFailure: (f) => f1.push(f), ...quiet });
      await k1.request({ type: 'x' });
      svc.expire('s1');
      assert.equal((await k1.request({ type: 'y' })).ok, false);
      assert.deepEqual(f1.map((f) => f.kind), ['expired']);

      const f2: NativeFailure[] = [];
      const k2 = new NativeKernel({ url: svc.url, token: 'wrong', onFailure: (f) => f2.push(f), ...quiet });
      const r2 = await k2.request({ type: 'x' });
      assert.equal(r2.ok, false);
      assert.deepEqual(f2.map((f) => f.kind), ['unauthorized']);

      const f3: NativeFailure[] = [];
      const k3 = await NativeKernel.connect({ url: svc.url, token: svc.token, timeoutMs: 80, onFailure: (f) => f3.push(f), ...quiet });
      const r3 = await k3.request({ type: 'stall' });
      assert.equal(r3.ok, false);
      assert.match(r3.trap ?? '', /no reply within/);
      assert.deepEqual(f3.map((f) => f.kind), ['timeout']);
      await Promise.all([k1.close(), k2.close(), k3.close()]);
    } finally { await svc.stop(); }
  });

  it('survives an idle gap longer than the server keep-alive, and reports a service that is gone', async () => {
    const svc = await startMockService({ keepAliveMs: 100 });
    const failures: NativeFailure[] = [];
    const k = await NativeKernel.connect({ url: svc.url, token: svc.token, onFailure: (f) => failures.push(f), ...quiet });
    try {
      assert.equal((await k.request({ type: 'a' })).ok, true);
      await new Promise((r) => setTimeout(r, 1300));   // the server drops the idle connection
      assert.equal((await k.request({ type: 'b' })).ok, true);
      assert.equal(failures.length, 0);
      await new Promise((r) => setTimeout(r, 1300));
      await svc.stop();
      assert.equal((await k.request({ type: 'c' })).ok, false);
      assert.deepEqual(failures.map((f) => f.kind), ['unreachable']);
    } finally { await k.close(); await svc.stop(); }
  });

  it('pings while idle so the service does not expire the session', async () => {
    const svc = await startMockService();
    const k = await NativeKernel.connect({ url: svc.url, token: svc.token, keepAliveMs: 40 });
    try {
      await k.request({ type: 'x' });
      await new Promise((r) => setTimeout(r, 200));
      assert.ok(svc.requests.filter((r) => r.body.includes('"ping"')).length >= 1);
    } finally { await k.close(); await svc.stop(); }
  });
});

const plate = () => {
  const p = new Part('Plate', 'aluminium');
  const base = p.sketch('Outline', 'front', (s) => s.rect(0, 0, 100, 60));
  p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 10, name: 'Plate' });
  return p.toJSON();
};
const norm = (v: unknown): unknown => JSON.parse(JSON.stringify(v, (k, x) => (k === 'ms' ? 0 : x)));

describe('NativeKernel against the real kernel through the mock service', { skip: !existsSync(defaultWasmPath()) && 'no wasm built' }, () => {
  it('returns the same replies as the wasm worker, including binary mesh buffers', async () => {
    const svc = await startMockService({ wasmPath: defaultWasmPath() });
    const nat = await NativeKernel.connect({ url: svc.url, token: svc.token, ...quiet });
    const wasm = await createWorkerKernel();
    try {
      for (const msg of [{ type: 'regen', tag: 't', part: plate() }, { type: 'massProps', tag: 't' }, { type: 'nonsense' }]) {
        const a = await nat.request(msg);
        const b = await wasm.request(msg);
        assert.deepEqual(norm(a), norm(b), String(msg.type));
      }
      const r = (await nat.request({ type: 'regen', tag: 't2', part: plate() })).result as { body: { mesh: { positions: unknown } } };
      const pos = r.body.mesh.positions as { data?: ArrayLike<number> } | ArrayLike<number>;
      assert.ok(('data' in (pos as object) ? (pos as { data: ArrayLike<number> }).data : (pos as ArrayLike<number>)).length > 0);
    } finally { await nat.close(); await wasm.close(); await svc.stop(); }
  });
});
