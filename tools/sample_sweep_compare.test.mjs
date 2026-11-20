// node --test tools/sample_sweep_compare.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const script = path.join(path.dirname(fileURLToPath(import.meta.url)), 'sample_sweep.mjs');
const row = (seconds, volume, extra = {}) => ({ seconds, wall: seconds, errors: [], volume, valid: true, ...extra });

function cmp(A, B, extra = [], accepted = {}) {
  const d = mkdtempSync(path.join(tmpdir(), 'sweepcmp-'));
  const w = (n, o) => { const f = path.join(d, n); writeFileSync(f, JSON.stringify(o)); return f; };
  const r = spawnSync('node', [script, '--compare', w('a.json', A), w('b.json', B), '--accepted', w('acc.json', accepted), ...extra], { encoding: 'utf8' });
  return { code: r.status, out: r.stdout };
}

test('new samples never count', () => {
  const r = cmp({ a: row(1, 10) }, { a: row(1, 10), n: row(99, 5) });
  assert.equal(r.code, 0);
  assert.match(r.out, /n: new \(not counted\)/);
  assert.match(r.out, /1 new/);
});

test('volume change fails, accepted change passes, other change to the same sample fails', () => {
  const A = { a: row(1, 100) };
  assert.equal(cmp(A, { a: row(1, 101) }).code, 1);
  const acc = { a: { from: 100, to: 101, reason: 'audited' } };
  const ok = cmp(A, { a: row(1, 101) }, [], acc);
  assert.equal(ok.code, 0);
  assert.match(ok.out, /accepted.*audited/);
  assert.equal(cmp(A, { a: row(1, 102) }, [], acc).code, 1);
});

test('timing needs 3x and 5 s', () => {
  assert.equal(cmp({ a: row(1, 1) }, { a: row(4, 1) }).code, 0); // 4x but only 3 s
  assert.equal(cmp({ a: row(10, 1) }, { a: row(20, 1) }).code, 0); // 10 s but only 2x
  const r = cmp({ a: row(2, 1) }, { a: row(9, 1) });
  assert.equal(r.code, 1);
  assert.match(r.out, /SLOWER/);
});

test('timing skipped on a loaded box', () => {
  const r = cmp({ a: row(2, 1) }, { a: row(30, 1), _meta: { load1: 12 } });
  assert.equal(r.code, 0);
  assert.match(r.out, /timing skipped: box load 12/);
  assert.equal(cmp({ a: row(2, 1) }, { a: row(30, 1), _meta: { load1: 3 } }).code, 1);
});

test('a new error and a validity loss still fail even on a loaded box', () => {
  const r = cmp({ a: row(1, 1) }, { a: row(1, 1, { errors: ['F1:boom'], valid: false }), _meta: { load1: 20 } });
  assert.equal(r.code, 1);
  assert.match(r.out, /ERRORS/);
  assert.match(r.out, /VALID/);
});
