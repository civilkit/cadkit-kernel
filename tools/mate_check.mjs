// Extended assembly mates and the DOF report through the wasm
// (docs/spec-agent-interface.md, "Mate kinds").
// usage: node tools/mate_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

let bad = 0;
const check = (name, ok, detail = '') => {
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${detail ? `: ${detail}` : ''}`);
};
const near = (a, b, tol = 1e-6) => Math.abs(a - b) <= tol * Math.max(1, Math.abs(b));

const part = (features) => ({ format: 'ck-part', version: 1, features });
const rect = (w, d) => [
  { id: 'p1', x: 0, y: 0 }, { id: 'p2', x: w, y: 0 }, { id: 'p3', x: w, y: d }, { id: 'p4', x: 0, y: d }];
const box = (w = 40, d = 20, h = 10) => part([
  { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: rect(w, d),
    entities: [['p1', 'p2'], ['p2', 'p3'], ['p3', 'p4'], ['p4', 'p1']].map(([a, b], i) => ({ type: 'line', id: `e${i + 1}`, p1: a, p2: b })), constraints: [] },
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: h }]);
const cyl = (r = 5, h = 20) => part([
  { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: [{ id: 'c', x: 0, y: 0 }], entities: [{ type: 'circle', id: 'e1', c: 'c', r }], constraints: [] },
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: h }]);
const ins = (id, p, o = {}) => ({ type: 'insertPart', id, name: id, part: p, ...o });
const mate = (id, kind, a, b, o = {}) => ({ type: 'mate', id, kind, a, b, ...o });

function regen(features, opts) {
  const k = new Kernel(mod);
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: part(features), ...(opts ? { opts } : {}) } })).json);
  const res = r.result ?? {};
  return { res, errs: (res.errors ?? []).map(([id, e]) => `${id}:${e.code}`), asm: res.assembly, comp: (id) => (res.components ?? []).find((c) => c.id === id) };
}
const rotY = (c) => -Math.atan2(c.place.x[2], c.place.x[0]) * 180 / Math.PI; // turn about +y (x column of R is (cos, 0, -sin))

const C = cyl(), B = box();
const CYL = (id, o = {}) => ins(id, C, o);

{
  const r = regen([CYL('F1', { fixed: true }), CYL('F2', { dx: 100, dy: 7, dz: 3 }),
    mate('F3', 'hinge', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1', { a2: 'F2.C0@F2.1', b2: 'F2.C1@F1.1' })]);
  const a = r.asm;
  check('hinge: 1 rotational DOF', r.errs.length === 0 && a?.dof.total === 1 && a.dof.rotational === 1 && a.dof.translational === 0 && !a.overConstrained, JSON.stringify(a?.dof) + r.errs);
  check('hinge: component dof', r.comp('F2')?.dof === 1 && r.comp('F1')?.dof === 0 && a?.components.find((c) => c.id === 'F1').fixed === true);
}
{
  const f = [CYL('F1', { fixed: true }), CYL('F4', { fixed: true, dx: 30 }), CYL('F2', { dx: 3, dy: 5 }), CYL('F3', { dx: 33, dy: 2 }),
    mate('F5', 'hinge', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1'), mate('F6', 'hinge', 'F2.S.e1@F3.1', 'F2.S.e1@F4.1')];
  const r = regen([...f, mate('F7', 'gear', 'F2.S.e1@F2.1', 'F2.S.e1@F3.1', { ratio: 2, angle: 40 })]);
  check('gear 2:1 turns the follower by half, opposite', r.errs.length === 0 && near(rotY(r.comp('F2')), 40, 1e-6) && near(rotY(r.comp('F3')), -20, 1e-6), `${rotY(r.comp('F2'))} ${rotY(r.comp('F3'))} ${r.errs}`);
  check('gear: DOF report', r.asm?.dof.total === 2 && r.asm.dof.rotational === 0, JSON.stringify(r.asm?.dof));
}
{
  const r = regen([CYL('F1', { fixed: true }), CYL('F2', { dx: 50, dz: 9 }), mate('F3', 'screw', 'F2.S.e1@F1.1', 'F2.S.e1@F2.1', { pitch: 2, angle: 1080 })]);
  check('screw: 3 turns advance 6', r.errs.length === 0 && near(r.comp('F2').place.o[1], 6) && r.asm?.dof.total === 0, `${JSON.stringify(r.comp('F2')?.place?.o)} ${r.errs}`);
}
{
  const r = regen([ins('F1', B, { fixed: true }), ins('F2', B, { dx: 100, dy: 7, dz: 3 }),
    mate('F3', 'perpendicular', 'F2.C0@F2.1', 'F2.C1@F1.1'), mate('F4', 'angle', 'F2.C0@F2.1', 'F2.C1@F1.1', { angle: 30 })]);
  check('conflict names the mates', r.errs.length === 1 && r.errs[0] === 'F4:mateConflict' && r.asm?.overConstrained === true
    && r.asm.conflicts[0]?.mate === 'F4' && r.asm.conflicts[0].with.includes('F3'), `${r.errs} ${JSON.stringify(r.asm?.conflicts)}`);
  check('conflict error lists the mates', JSON.stringify(r.res.errors[0][1].params.mates) === '["F4","F3"]', JSON.stringify(r.res.errors));
  const d = regen([ins('F1', B, { fixed: true }), ins('F2', B, { dx: 100 }), mate('F3', 'perpendicular', 'F2.C0@F2.1', 'F2.C1@F1.1'), mate('F4', 'perpendicular', 'F2.C0@F2.1', 'F2.C1@F1.1')]);
  check('duplicate mate is redundant', d.errs.length === 0 && d.asm?.overConstrained === true && d.asm.redundant.join() === 'F4', JSON.stringify(d.asm?.redundant));
}
{
  const bx = [ins('F1', B, { fixed: true }), ins('F2', B, { dx: 100, dy: 7, dz: 3 })];
  const old = regen([...bx, mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')]);
  check('oracle kinds: no assembly key', old.errs.length === 0 && old.asm === undefined);
  const forced = regen([...bx, mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')], { dofReport: true });
  check('dofReport opt reports oracle kinds', forced.asm?.dof.total === 3 && forced.asm.dof.translational === 2 && forced.asm.dof.rotational === 1, JSON.stringify(forced.asm?.dof));
  const bad1 = regen([...bx, mate('F3', 'hinge', 'F2.C0@F2.1', 'F2.C1@F1.1')]);
  check('unsupported geometry is a structured error', bad1.errs.join() === 'F3:mateGeometry', bad1.errs.join());
  const bad2 = regen([...bx, mate('F3', 'cam', 'F2.C0@F2.1', 'F2.C1@F1.1')]);
  check('unknown kind is a structured error', bad2.errs.join() === 'F3:mateGeometry', bad2.errs.join());
}
console.log(bad ? `mate_check: ${bad} FAILED` : 'mate_check: ok');
process.exit(bad ? 1 : 0);
