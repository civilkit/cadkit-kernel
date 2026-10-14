// Design-rule profiles through the wasm (`profiles`, `checkProfile`, docs/spec-agent-interface.md "Profiles").
// usage: node tools/profile_check.mjs [--wasm path] [--bench]     exit 1 on any mismatch
//   --bench  also run every profile on the BenchCAD parts (tools/bench/benchcad): never crash,
//            each run under 5 s; prints issue counts per profile and the slowest runs.
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

let bad = 0;
const ok = (cond, name, detail = '') => {
  if (!cond) bad++;
  console.log(`${cond ? 'ok  ' : 'FAIL'} ${name}${detail ? ': ' + detail : ''}`);
};
let seq = 0;
const call = (k, msg) => {
  const r = k.request(JSON.stringify({ id: ++seq, msg }));
  if (r.trap) return { ok: false, error: `trap ${r.trap}` };
  return JSON.parse(r.json);
};

const poly = (id, plane, pts) => ({
  type: 'sketch', id, plane: { kind: 'std', id: plane },
  points: pts.map(([x, y], i) => ({ id: `p${i}`, x, y })),
  entities: pts.map((_, i) => ({ type: 'line', id: `e${i}`, p1: `p${i}`, p2: `p${(i + 1) % pts.length}` })),
  constraints: [],
});
const rect = (id, plane, x0, y0, x1, y1) => poly(id, plane, [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]);
const ext = (id, sketch, op, depth, o = {}) => ({ type: 'extrude', id, op, sketch, end: 'blind', depth, ...o });
const partOf = (features) => ({ format: 'ck-part', version: 1, features });

// 20x20x10 box pocketed from below: 0.5 mm side walls and roof
const thin = partOf([
  rect('F1', 'front', 0, 0, 20, 20), ext('F2', 'F1', 'add', 10),
  rect('F3', 'front', 0.5, 0.5, 19.5, 19.5), ext('F4', 'F3', 'cut', 9.5, { flip: true }),
]);

const k = new Kernel(mod);

// catalogue
{
  const r = call(k, { type: 'profiles' });
  const ids = r.ok ? r.result.profiles.map((p) => p.id) : [];
  ok(r.ok && ids.join() === 'fdm-print,cnc-3axis,injection', 'profiles lists the built-in profiles', ids.join());
  const wall = r.ok && r.result.profiles[0].rules.find((x) => x.id === 'minWall');
  ok(wall && wall.params[0].name === 'min' && wall.params[0].value === 1.2 && wall.params[0].unit === 'mm', 'rule parameters carry name, value, unit');
  ok(r.ok && r.result.rules.length >= 11, 'rule catalogue', String(r.result?.rules?.length));
}

// by tag, after a regen
{
  const rg = call(k, { type: 'regen', tag: 'main', part: thin });
  ok(rg.ok && !rg.result.errors.length, 'regen the thin part');
  const faces = new Set(rg.result.body.faces.map((f) => f.id));
  const before = JSON.stringify(call(k, { type: 'massProps', tag: 'main' }));
  const r = call(k, { type: 'checkProfile', tag: 'main', profile: 'fdm-print' });
  ok(r.ok, 'checkProfile by tag', r.error);
  const walls = r.ok ? r.result.issues.filter((i) => i.rule === 'minWall') : [];
  ok(walls.length >= 5 && walls.every((w) => Math.abs(w.value - 0.5) < 0.01 && w.limit === 1.2 && w.severity === 'error'), 'minWall issues: 0.5 mm against 1.2 mm', String(walls.length));
  ok(walls.every((w) => w.faces.length === 2 && w.faces.every((f) => faces.has(f))), 'issues name faces by persistent name');
  ok(r.ok && r.result.profile.id === 'fdm-print' && r.result.ok === false && r.result.summary.errors === walls.length + r.result.issues.filter((i) => i.severity === 'error' && i.rule !== 'minWall').length, 'summary counts');
  ok(JSON.stringify(call(k, { type: 'massProps', tag: 'main' })) === before, 'read-only: massProps unchanged');
  // a different parameter set, inline
  const inl = call(k, { type: 'checkProfile', tag: 'main', profile: { id: 'mine', name: 'Mine', rules: [{ id: 'minWall', params: { min: 0.4 }, severity: 'info' }] } });
  ok(inl.ok && inl.result.issues.length === 0 && inl.result.ok === true, 'inline profile with a lower limit passes', inl.error);
  const inl2 = call(k, { type: 'checkProfile', tag: 'main', profile: { rules: [{ id: 'minWall', params: { min: 2 }, severity: 'info' }] } });
  ok(inl2.ok && inl2.result.issues.length > 0 && inl2.result.issues.every((i) => i.severity === 'info') && inl2.result.ok === true, 'severity override (info does not fail ok)');
  // errors
  const e1 = call(k, { type: 'checkProfile', tag: 'main', profile: 'nope' });
  ok(!e1.ok && /unknown profile/.test(e1.error) && /fdm-print/.test(e1.error), 'unknown profile lists the choices', e1.error);
  const e2 = call(k, { type: 'checkProfile', tag: 'main', profile: { rules: [{ id: 'minWall', params: { thick: 1 } }] } });
  ok(!e2.ok && /unknown parameter/.test(e2.error), 'unknown parameter', e2.error);
  const e3 = call(k, { type: 'checkProfile', tag: 'main', profile: { rules: [{ id: 'zzz' }] } });
  ok(!e3.ok && /unknown rule/.test(e3.error), 'unknown rule', e3.error);
  const e4 = call(k, { type: 'checkProfile', tag: 'main', profile: { rules: [{ id: 'minWall', severity: 'loud' }] } });
  ok(!e4.ok && /severity/.test(e4.error), 'bad severity', e4.error);
  const none = call(k, { type: 'checkProfile', tag: 'nothing-here', profile: 'fdm-print' });
  ok(!none.ok, 'unknown tag is an error', none.error);
}

// by part (private regen through the cache), same answer as by tag
{
  const k2 = new Kernel(mod);
  const a = call(k2, { type: 'checkProfile', part: thin, profile: 'fdm-print' });
  const b = call(k2, { type: 'checkProfile', part: thin, profile: 'fdm-print' });
  ok(a.ok && b.ok && JSON.stringify(a.result) === JSON.stringify(b.result), 'checkProfile by part is deterministic (cache hit)');
  const ref = call(k, { type: 'checkProfile', tag: 'main', profile: 'fdm-print' });
  ok(ref.ok && JSON.stringify(ref.result) === JSON.stringify(a.result), 'by part equals by tag');
  // an edited document re-checks correctly (thicker walls): the cache must not return the old body
  const thick = partOf([
    rect('F1', 'front', 0, 0, 20, 20), ext('F2', 'F1', 'add', 10),
    rect('F3', 'front', 2, 2, 18, 18), ext('F4', 'F3', 'cut', 8, { flip: true }),
  ]);
  const c = call(k2, { type: 'checkProfile', part: thick, profile: 'fdm-print' });
  ok(c.ok && c.result.issues.filter((i) => i.rule === 'minWall').length === 0, 'edited part: 2 mm walls pass', c.error);
  const d = call(k2, { type: 'checkProfile', part: thin, profile: 'fdm-print' });
  ok(d.ok && JSON.stringify(d.result) === JSON.stringify(a.result), 'back to the first document gives the first answer');
  const badPart = call(k2, { type: 'checkProfile', part: { nope: 1 }, profile: 'fdm-print' });
  ok(!badPart.ok && /invalid part/.test(badPart.error), 'invalid part', badPart.error);
}

// cnc and injection profiles on small parts
{
  const kk = new Kernel(mod);
  const pocket = partOf([
    rect('F1', 'front', 0, 0, 40, 40), ext('F2', 'F1', 'add', 10, { flip: true }),
    rect('F3', 'front', 10, 10, 30, 30), ext('F4', 'F3', 'cut', 5),
  ]);
  const r = call(kk, { type: 'checkProfile', part: pocket, profile: 'cnc-3axis' });
  const rad = r.ok ? r.result.issues.filter((i) => i.rule === 'minInternalRadius') : [];
  ok(rad.length === 4 && rad.every((i) => i.edges.length === 1 && /\|.*#\d+$/.test(i.edges[0])), 'cnc: four sharp pocket corners named by edge key', String(rad.length));
  const drafted = (d) => partOf([rect('F1', 'front', 0, 0, 20, 20), ext('F2', 'F1', 'add', 10, { draft: d })]);
  const a = call(kk, { type: 'checkProfile', part: drafted(0), profile: 'injection' });
  const b = call(kk, { type: 'checkProfile', part: drafted(3), profile: 'injection' });
  ok(a.ok && a.result.issues.filter((i) => i.rule === 'draft').length === 4, 'injection: straight walls need draft');
  ok(b.ok && b.result.issues.filter((i) => i.rule === 'draft').length === 0, 'injection: 3 degrees of draft pass');
}

// BenchCAD sweep
if (process.argv.includes('--bench')) {
  const dir = path.join(here, 'bench/benchcad');
  const manifest = JSON.parse(readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
  const totals = {};
  const rows = [];
  for (const m of manifest.parts) {
    if (manifest.known?.[m.id]?.skip) continue;
    const part = JSON.parse(readFileSync(path.join(dir, `${m.id}.ckpart.json`), 'utf8'));
    const kb = new Kernel(mod);
    const rg = call(kb, { type: 'regen', tag: 'main', part });
    if (!rg.ok || !rg.result.body) continue; // translation failed: nothing to check
    for (const prof of ['fdm-print', 'cnc-3axis', 'injection']) {
      const t0 = performance.now();
      const r = call(kb, { type: 'checkProfile', tag: 'main', profile: prof });
      const ms = performance.now() - t0;
      const t = (totals[prof] ??= { parts: 0, error: 0, warn: 0, info: 0, parts_with_issues: 0, crashes: 0, max_ms: 0, sum_ms: 0, byRule: {} });
      t.parts++;
      t.sum_ms += ms;
      t.max_ms = Math.max(t.max_ms, ms);
      rows.push({ id: m.id, prof, ms });
      if (!r.ok) { t.crashes++; ok(false, `${prof} on ${m.id}`, r.error); continue; }
      const iss = r.result.issues;
      if (iss.length) t.parts_with_issues++;
      for (const i of iss) { t[i.severity]++; t.byRule[i.rule] = (t.byRule[i.rule] ?? 0) + 1; }
    }
  }
  for (const [prof, t] of Object.entries(totals)) {
    console.log(`${prof}: ${t.parts} parts, ${t.error} errors, ${t.warn} warnings, ${t.info} infos, ${t.parts_with_issues} parts with issues, ${t.crashes} failures, avg ${(t.sum_ms / t.parts).toFixed(0)} ms, max ${t.max_ms.toFixed(0)} ms`);
    console.log(`  by rule: ${JSON.stringify(t.byRule)}`);
  }
  rows.sort((a, b) => b.ms - a.ms);
  console.log('slowest: ' + rows.slice(0, 5).map((r) => `${r.id}/${r.prof} ${r.ms.toFixed(0)} ms`).join(', '));
  ok(rows.length > 0 && rows.every((r) => r.ms < 5000), 'bench: every run under 5 s', `max ${rows[0]?.ms.toFixed(0)} ms`);
  ok(Object.values(totals).every((t) => t.crashes === 0), 'bench: no failures');
}

console.log(bad ? `profile_check: ${bad} FAILED` : 'profile_check: ok');
process.exit(bad ? 1 : 0);
