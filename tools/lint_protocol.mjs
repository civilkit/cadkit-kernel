// Protocol convention lint (in the spirit of CivilKit's ADR 0014 export lint).
//
// usage: node tools/lint_protocol.mjs [--wasm path] [--quiet]
//
// Sends every request type (parsed from REQUEST_TYPES in crates/ck-server/src/lib.rs,
// so extension types such as `validate` are included automatically) with
//   - a minimal valid body,
//   - malformed envelopes (bad JSON, missing/ill-typed msg and type),
//   - field fuzz (wrong types, huge numbers, NaN-like strings, empty arrays/objects),
//   - structural mutations of real parts through `regen`,
// against the rebuilt wasm, in a fresh session and in a session holding a regen'd body, and asserts:
//   1. no trap / panic / hang (a hang is caught by a watchdog),
//   2. the reply is a framed JSON envelope { ok:bool, result (ok) | error (!ok), ms } --
//      `id` is NOT echoed by the kernel (the host owns correlation; the original does
//      the same and the wire gate pins it), so it must be absent or equal to the request id,
//   3. error is a non-empty string; ms is a finite number >= 0; no unknown top-level keys.
// Exit 1 on any violation.
import { readFileSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Worker, isMainThread, workerData, parentPort } from 'node:worker_threads';

const here = path.dirname(fileURLToPath(import.meta.url));

if (isMainThread) {
  const args = process.argv.slice(2);
  const wi = args.indexOf('--wasm');
  const wasm = wi >= 0 ? args[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
  const quiet = args.includes('--quiet');
  const w = new Worker(fileURLToPath(import.meta.url), { workerData: { wasm } });
  let last = '', n = 0, timer;
  const arm = () => { clearTimeout(timer); timer = setTimeout(() => { console.log(`lint_protocol: HANG (no reply for 60s) after: ${last}`); process.exit(1); }, 60000); };
  arm();
  w.on('message', (m) => {
    arm();
    if (m.case) { last = m.case; n++; return; }
    clearTimeout(timer);
    for (const v of m.violations) console.log(`VIOLATION ${v}`);
    if (!quiet || m.violations.length) console.log(`lint_protocol: ${m.sent} requests over ${m.types.length} types (${m.types.join(',')}), ${m.violations.length} violation(s)`);
    process.exit(m.violations.length ? 1 : 0);
  });
  w.on('error', (e) => { console.log(`lint_protocol: worker crashed: ${e.message} after: ${last}`); process.exit(1); });
} else {
  const { Kernel } = await import('./kernel.mjs');
  const { parts } = await import('./brep_cases.mjs');
  const mod = new WebAssembly.Module(readFileSync(workerData.wasm));
  const src = readFileSync(path.join(here, '../crates/ck-server/src/lib.rs'), 'utf8');
  const types = [...src.matchAll(/^\s*\("([A-Za-z]+)",\s*[\w:]+\),/gm)].map((m) => m[1]);
  const violations = [];
  let sent = 0;
  const KEYS = new Set(['ok', 'result', 'error', 'ms', 'buffers', 'trace', 'id']);

  const BOX_REGEN = { type: 'regen', tag: 'main', part: parts.box };
  // minimal valid body per request type (the session already holds a regen'd box when needed)
  const MIN = {
    ping: {}, regen: { part: parts.box }, matchEdges: { part: parts.box, refs: [] }, drag: {}, convert: { ref: { kind: 'edge', id: 'F2.C1' } },
    sketchOffset: { sketch: { type: 'sketch', points: [], entities: [] }, ids: [], distance: 1 }, refCurve: { ref: { kind: 'edge' } },
    measure: { a: { kind: 'face', id: 'F2.C1' }, b: { kind: 'face', id: 'F2.C0' } }, massProps: {}, interference: {}, step: { name: 't' },
    validate: {}, inspect: { ref: { kind: 'sketch', id: 'F1' } }, continuity: { refs: [] }, curvatureAt: { ref: { kind: 'face', id: 'F2.C1' }, u: 0.5, v: 0.5 },
    curvatureMap: {}, comb: { refs: [{ kind: 'face', id: 'F2.C1' }] }, drawing: {},
    check: { expect: [{ kind: 'volume', min: 0 }] }, resolve: { descriptions: [{ kind: 'face', surface: 'plane' }] }, describe: { names: ['F2.C1'] },
    refDiff: { base: parts.box, part: parts.box },
    section: { plane: { origin: [0, 0, 0], normal: [0, 0, 1] } }, sections: { n: 3 }, bodyBoolean: { op: 'cut', target: 'F2.C1', tools: ['F2.C0'] }, probe: { points: [[0, 0, 0]], rays: [{ origin: [0, 0, 0], dir: [0, 0, 1] }] },
    distance: { a: [0, 0, 0], b: 'body' }, maxDistance: { opts: { samples: 200 } }, thickness: { opts: { samples: 200 } }, thicknessAt: { point: [0, 0, 0] },
    closestPoint: { points: [[0, 0, 0]] }, angles: { edges: 'all' }, clearance: { min: 1 }, symmetry: { opts: { samples: 200 } },
    textMetrics: { string: 'RPM x100', font: 'gauge', height: 5 }, textOutline: { string: 'RPM x100', font: 'gauge', height: 5, at: [1, 2], anchor: ['center', 'middle'] },
  };
  const MUST_OK = new Set(['ping', 'regen', 'massProps', 'validate', 'interference', 'step', 'check', 'resolve', 'describe', 'refDiff', 'section', 'sections', 'probe', 'distance', 'maxDistance', 'thickness', 'closestPoint', 'angles', 'clearance', 'symmetry', 'textMetrics', 'textOutline']); // minimal body must succeed once a body exists
  const FIELDS = ['part', 'tag', 'opts', 'ref', 'refs', 'ids', 'sketch', 'distance', 'name', 'upTo', 'densities', 'bodies', 'dir', 'tol', 'a', 'b', 'u', 'at', 'expect', 'descriptions', 'description', 'names', 'base', 'plane', 'axis', 'n', 'positions', 'points', 'rays', 'min', 'on', 'edges', 'faces', 'pull', 'tagB', 'planes', 'draft', 'point', 'k', 'string', 'font', 'height', 'spacing', 'angle', 'arc', 'anchor', 'mirror'];
  const VALUES = [null, 'x', 'NaN', '-1e999', 1e308, -1e308, -1, 0, [], {}, [[]], [null], true, { kind: 'face', id: 'nope' }, { kind: 5 }, [1, 2, 3]];

  function check(label, k, body) {
    sent++;
    parentPort.postMessage({ case: label });
    const r = k.request(typeof body === 'string' ? body : JSON.stringify(body));
    const bad = (m) => violations.push(`${label}: ${m}`);
    if (r.trap) { bad(`TRAP ${r.trap.slice(0, 160)}`); return null; }
    let j;
    try { j = JSON.parse(r.json); } catch { bad(`reply is not JSON: ${String(r.json).slice(0, 80)}`); return null; }
    if (!j || typeof j !== 'object' || Array.isArray(j)) { bad('reply is not an object'); return null; }
    if (typeof j.ok !== 'boolean') bad('ok is not a boolean');
    if (j.ok === true && !('result' in j)) bad('ok:true without result');
    if (j.ok === true && 'error' in j) bad('ok:true with error');
    if (j.ok === false && (typeof j.error !== 'string' || !j.error.trim())) bad(`error is not a non-empty string (${JSON.stringify(j.error)})`);
    if (j.ok === false && 'result' in j) bad('ok:false with result');
    if (typeof j.ms !== 'number' || !Number.isFinite(j.ms) || j.ms < 0) bad(`ms is not a finite number >= 0 (${JSON.stringify(j.ms)})`);
    for (const key of Object.keys(j)) if (!KEYS.has(key)) bad(`unknown top-level key ${key}`);
    if ('id' in j && body && typeof body === 'object' && j.id !== body.id) bad('id present but differs from the request id');
    return j;
  }
  const sessions = (fn) => {
    for (const withBody of [false, true]) {
      const k = new Kernel(mod);
      if (withBody) check('setup regen box', k, { id: 0, msg: BOX_REGEN });
      fn(k, withBody ? 'body' : 'empty');
    }
  };

  // 1. envelope-level malformations
  sessions((k, s) => {
    for (const raw of ['', '{', 'null', '[]', '1', '"x"', '{"id":1}', '{"id":1,"msg":null}', '{"id":1,"msg":[]}', '{"id":1,"msg":"ping"}', '{"id":1,"msg":5}',
      '{"id":1,"msg":{}}', '{"id":1,"msg":{"type":""}}', '{"id":1,"msg":{"type":5}}', '{"id":1,"msg":{"type":null}}', '{"id":1,"msg":{"type":["ping"]}}',
      '{"id":1,"msg":{"type":"nosuchtype"}}', '{"id":1,"msg":{"type":"ping"},"msg":{"type":"x"}}', '\u0000', '{"id":1,"msg":{"type":"ping"}} trailing',
      '{"id":"s","msg":{"type":"ping"}}', '{"id":null,"msg":{"type":"ping"}}', '{"msg":{"type":"ping"}}', '{"id":1,"msg":{"type":"ping","debug":"yes"}}', '{"id":1,"msg":{"type":"ping","debug":true}}'])
      check(`${s}: envelope ${JSON.stringify(raw).slice(0, 60)}`, k, raw);
  });

  // 2. per-type: minimal valid, then each field replaced by every fuzz value
  for (const ty of types) {
    sessions((k, s) => {
      const min = MIN[ty] ?? {};
      const ok = check(`${s}: ${ty} minimal`, k, { id: 1, msg: { type: ty, tag: 'main', ...min } });
      if (ok && ok.ok === false && s === 'body' && MUST_OK.has(ty)) violations.push(`${s}: ${ty} minimal: valid minimal body was rejected: ${ok.error}`);
      for (const f of FIELDS) for (const v of VALUES) {
        check(`${s}: ${ty} ${f}=${JSON.stringify(v)}`, k, { id: 2, msg: { type: ty, tag: 'main', ...min, [f]: v } });
      }
      check(`${s}: ${ty} only type`, k, { id: 3, msg: { type: ty } });
    });
  }

  // 3. structural mutations of real parts through regen (then a query on the survivor)
  const mutate = function* (v, p = []) {
    yield [p, undefined]; // delete
    if (typeof v === 'number') for (const m of [1e308, -1e308, 0, -1, '1e999', 'NaN', null, 1e-320, 2 ** 53]) yield [p, m];
    else if (typeof v === 'string') for (const m of ['', 'NaN', 5, null, '=1/0', '=x']) yield [p, m];
    else if (Array.isArray(v)) { yield [p, []]; yield [p, null]; yield [p, {}]; for (let i = 0; i < Math.min(v.length, 6); i++) yield* mutate(v[i], [...p, i]); }
    else if (v && typeof v === 'object') { yield [p, {}]; yield [p, []]; for (const [kk, x] of Object.entries(v)) yield* mutate(x, [...p, kk]); }
    else yield [p, null];
  };
  const withPath = (obj, p, val) => {
    const c = JSON.parse(JSON.stringify(obj));
    let t = c;
    for (const key of p.slice(0, -1)) t = t[key];
    const last = p[p.length - 1];
    if (p.length === 0) return val;
    if (val === undefined) { if (Array.isArray(t)) t.splice(last, 1); else delete t[last]; } else t[last] = val;
    return c;
  };
  // an engraved plate with a straight and an arc text entity (lettering, docs/text.md)
  parts.textEngrave = { format: 'ck-part', version: 1, features: [
    { type: 'sketch', id: 'S1', plane: { kind: 'std', id: 'front' }, points: [{ id: 'a', x: 0, y: 0 }, { id: 'b', x: 60, y: 0 }, { id: 'c', x: 60, y: 30 }, { id: 'd', x: 0, y: 30 }], entities: [['a', 'b'], ['b', 'c'], ['c', 'd'], ['d', 'a']].map((q, i) => ({ type: 'line', id: 'e' + i, p1: q[0], p2: q[1] })), constraints: [] },
    { type: 'extrude', id: 'F1', op: 'add', sketch: 'S1', end: 'blind', depth: 4 },
    { type: 'plane', id: 'P1', mode: 'offset', base: { kind: 'std', id: 'front' }, offset: 4 },
    { type: 'sketch', id: 'S2', plane: { kind: 'feature', id: 'P1' }, points: [{ id: 'q', x: 30, y: 12 }], entities: [
      { type: 'text', id: 't1', string: 'RPM x100', font: 'gauge', height: 5, p: 'q', anchor: ['center', 'middle'], angle: 10, spacing: 1.1 },
      { type: 'text', id: 't2', string: 'KNOTS', font: 'engrave', height: 3, at: [30, 15], anchor: ['center', 'baseline'], arc: { centre: [30, 15], radius: 9, angleCentre: 90, side: 'outside' } }], constraints: [] },
    { type: 'extrude', id: 'F2', op: 'cut', sketch: 'S2', end: 'blind', depth: 0.3 }] };
  const targets = ['box', 'rev90', 'cutHole', 'shellBoxOpen', 'sweepLine', 'lpatternCut', 'filletOne', 'textEngrave'];
  const k = new Kernel(mod);
  for (const name of targets) {
    const part = parts[name];
    if (!part) continue;
    let i = 0;
    for (const [p, val] of mutate(part)) {
      const mp = withPath(part, p, val);
      const lbl = `regen ${name} ${p.join('.')}=${val === undefined ? '<del>' : JSON.stringify(val)}`;
      const j = check(lbl, k, { id: 9, msg: { type: 'regen', tag: 'main', part: mp } });
      if (j?.ok && i++ % 5 === 0) for (const q of ['massProps', 'validate', 'interference']) check(`${lbl} then ${q}`, k, { id: 10, msg: { type: q, tag: 'main' } });
    }
  }
  parentPort.postMessage({ sent, types, violations: [...new Set(violations)] });
}
