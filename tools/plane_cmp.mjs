// Differential test of workplane modes: the original kernel.wasm vs the rebuilt wasm.
// usage: node tools/plane_cmp.mjs [--wasm rebuilt.wasm] [-v]
// Each case is a feature list whose last plane feature "P" is compared
// (frame within 1e-6, same error codes). Known deviations are listed in KNOWN.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const args = process.argv.slice(2);
const wi = args.indexOf('--wasm');
const ours = wi >= 0 ? args[wi + 1] : new URL('../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url).pathname;
const verbose = args.includes('-v');
const load = (p) => new Kernel(new WebAssembly.Module(readFileSync(p)));
const ref = load(new URL('../../kernel.wasm', import.meta.url).pathname), mine = load(ours);

const std = (id) => ({ kind: 'std', id });
const P = (o) => ({ type: 'plane', id: 'P', ...o });
const sp = (s, p) => ({ kind: 'sketchPoint', sketch: s, point: p });
const sc = (s, e) => ({ kind: 'sketchCurve', sketch: s, entity: e });
const sl = (s, e) => ({ kind: 'sketchLine', sketch: s, entity: e });
const V = (faces, at) => ({ kind: 'vertex', faces, at });
const AX = (a) => ({ kind: 'axis', axis: a });
const fp = (id) => ({ kind: 'feature', id });
const A = (refs, id = 'A') => ({ type: 'axis', id, refs });
const ax = fp('A');
const SK = (id, plane, pts, ents) => ({ type: 'sketch', id, plane: std(plane), points: pts.map(([x, y], i) => ({ id: 'p' + i, x, y })), entities: ents, constraints: [] });
const ln = (id, a, b) => ({ id, type: 'line', p1: 'p' + a, p2: 'p' + b });
const S1 = SK('S1', 'front', [[0, 0], [10, 0], [0, 10], [5, 5], [20, 0], [3, 4]], [ln('e1', 0, 1), ln('e2', 0, 2), ln('e3', 1, 2), ln('e4', 3, 5)]);
const S2 = SK('S2', 'right', [[1, 2], [7, 3], [-4, 9]], []);
const PL = { id: 'PL', type: 'sketch', plane: std('front'), points: [{ id: 'p0', x: 0, y: 0 }, { id: 'p1', x: 10, y: 0 }, { id: 'p2', x: 10, y: 10 }], entities: [ln('e1', 0, 1), ln('e2', 1, 2)], constraints: [] };
const PA = { id: 'PA', type: 'sketch', plane: std('top'), points: [{ id: 'c', x: 1, y: 2 }, { id: 'a', x: 10, y: 2 }, { id: 'b', x: 1, y: 10 }], entities: [{ id: 'e1', type: 'arc', c: 'c', p1: 'a', p2: 'b' }], constraints: [] };
const PC = { id: 'PC', type: 'sketch', plane: std('right'), points: [{ id: 'c', x: 3, y: 4 }], entities: [{ id: 'e1', type: 'circle', c: 'c', r: 5 }], constraints: [] };
const BOX = [SK('B', 'top', [[0, 0], [20, 0], [20, 10], [0, 10]], [ln('e1', 0, 1), ln('e2', 1, 2), ln('e3', 2, 3), ln('e4', 3, 0)]), { type: 'extrude', id: 'BX', op: 'add', sketch: 'B', end: 'blind', depth: 10 }];
const CYL = [{ type: 'sketch', id: 'CS', plane: std('top'), points: [{ id: 'c', x: 0, y: 0 }, { id: 'q', x: 20, y: 5 }, { id: 'q2', x: -10, y: -30 }, { id: 'q3', x: 0, y: -20 }], entities: [{ id: 'e1', type: 'circle', c: 'c', r: 10 }], constraints: [] }, { type: 'extrude', id: 'CY', op: 'add', sketch: 'CS', end: 'blind', depth: 10 }];
const CL = SK('CL', 'front', [[30, -5], [30, 15]], [ln('e1', 0, 1), ln('e2', 1, 0)]);
const SPH = [{ type: 'sketch', id: 'SS', plane: std('front'), points: [{ id: 'o', x: 0, y: 0 }, { id: 't', x: 0, y: 10 }, { id: 'b', x: 0, y: -10 }, { id: 'a1', x: 0, y: -15 }, { id: 'a2', x: 0, y: 15 }, { id: 'q', x: 30, y: 4 }, { id: 'q3', x: 0, y: 25 }, { id: 'l1', x: 0, y: 20 }, { id: 'l2', x: 25, y: 15 }],
  entities: [{ id: 'arc', type: 'arc', c: 'o', p1: 'b', p2: 't' }, { id: 'cl', type: 'line', p1: 't', p2: 'b' }, { id: 'ax', type: 'line', p1: 'a1', p2: 'a2', construction: true }, { id: 'sl', type: 'line', p1: 'l1', p2: 'l2' }], constraints: [] }, { type: 'revolve', id: 'SR', op: 'add', sketch: 'SS', axis: 'ax', angle: 360 }];

const cases = {};
const add = (name, feats) => { cases[name] = feats; };
const base = [S1, S2];
// threePoints
add('3pt', [...base, P({ mode: 'threePoints', points: [sp('S1', 'p0'), sp('S1', 'p1'), sp('S1', 'p2')] })]);
add('3pt perm', [...base, P({ mode: 'threePoints', points: [sp('S1', 'p1'), sp('S1', 'p0'), sp('S1', 'p2')] })]);
add('3pt flip', [...base, P({ mode: 'threePoints', flip: true, points: [sp('S1', 'p0'), sp('S1', 'p1'), sp('S1', 'p2')] })]);
add('3pt collinear', [...base, P({ mode: 'threePoints', points: [sp('S1', 'p0'), sp('S1', 'p1'), sp('S1', 'p4')] })]);
add('3pt dup', [...base, P({ mode: 'threePoints', points: [sp('S1', 'p0'), sp('S1', 'p1'), sp('S1', 'p0')] })]);
add('3pt S2', [...base, P({ mode: 'threePoints', points: [sp('S2', 'p0'), sp('S2', 'p1'), sp('S2', 'p2')] })]);
add('3pt mixed', [...base, P({ mode: 'threePoints', points: [sp('S1', 'p0'), sp('S2', 'p1'), sp('S2', 'p2')] })]);
add('3pt two', [...base, P({ mode: 'threePoints', points: [sp('S1', 'p0'), sp('S1', 'p1')] })]);
add('3pt 4', [...base, P({ mode: 'threePoints', points: [sp('S1', 'p0'), sp('S1', 'p1'), sp('S1', 'p2'), sp('S1', 'p3')] })]);
add('3pt vertices', [...BOX, P({ mode: 'threePoints', points: [V(['BX.C1', 'BX.S.e1', 'BX.S.e2'], [20, 0, -10]), V(['BX.C1', 'BX.S.e2', 'BX.S.e3'], [20, 10, -10]), V(['BX.C0', 'BX.S.e1', 'BX.S.e4'], [0, 0, 0])] })]);
// parallel
for (const [n, b, p, extra] of [['top', 'top', ['S1', 'p3']], ['front flip', 'front', ['S1', 'p3'], { flip: true }], ['right', 'right', ['S1', 'p1']], ['S2 pt', 'top', ['S2', 'p1']]])
  add('parallel ' + n, [...base, P({ mode: 'parallel', base: std(b), points: [sp(...p)], ...(extra || {}) })]);
add('parallel offset-base', [...base, { type: 'plane', id: 'O', mode: 'offset', base: std('right'), offset: 5 }, P({ mode: 'parallel', base: fp('O'), points: [sp('S2', 'p1')] })]);
add('parallel tilted base', [...base, { type: 'plane', id: 'O', mode: 'angle', base: std('top'), axis: AX('x'), angle: 30 }, P({ mode: 'parallel', base: fp('O'), points: [sp('S2', 'p1')] })]);
add('parallel nobase', [...base, P({ mode: 'parallel', points: [sp('S1', 'p3')] })]);
add('parallel nopoint', [...base, P({ mode: 'parallel', base: std('top') })]);
add('parallel vertex', [...BOX, P({ mode: 'parallel', base: std('top'), points: [V(['BX.C1', 'BX.S.e1', 'BX.S.e2'], [20, 0, -10])] })]);
// linePoint
add('lp axis x', [...base, P({ mode: 'linePoint', axis: AX('x'), points: [sp('S1', 'p2')] })]);
add('lp axis x flip', [...base, P({ mode: 'linePoint', axis: AX('x'), flip: true, points: [sp('S1', 'p2')] })]);
add('lp axis x S2', [...base, P({ mode: 'linePoint', axis: AX('x'), points: [sp('S2', 'p2')] })]);
add('lp axis y S2', [...base, P({ mode: 'linePoint', axis: AX('y'), points: [sp('S2', 'p1')] })]);
add('lp axis z S2', [...base, P({ mode: 'linePoint', axis: AX('z'), points: [sp('S2', 'p1')] })]);
add('lp on line', [...base, P({ mode: 'linePoint', axis: AX('y'), points: [sp('S1', 'p2')] })]);
add('lp noaxis', [...base, P({ mode: 'linePoint', points: [sp('S1', 'p2')] })]);
add('lp nopoint', [...base, P({ mode: 'linePoint', axis: AX('x') })]);
add('lp sketchLine', [...base, P({ mode: 'linePoint', axis: sl('S1', 'e4'), points: [sp('S1', 'p2')] })]);
add('lp sketchLine S2', [...base, P({ mode: 'linePoint', axis: sl('S1', 'e4'), points: [sp('S2', 'p1')] })]);
add('lp edge+vertex', [...BOX, P({ mode: 'linePoint', axis: { kind: 'edge', faces: ['BX.C1', 'BX.S.e1'], at: [10, 10, 0] }, points: [V(['BX.C1', 'BX.S.e2', 'BX.S.e3'], [20, 10, -10])] })]);
// axis features
const PLN = [{ type: 'plane', id: 'A1', mode: 'offset', base: std('top'), offset: 5 }, { type: 'plane', id: 'A2', mode: 'offset', base: std('front'), offset: 3 }];
add('axisfeat planes lp', [...base, ...PLN, A([fp('A1'), fp('A2')]), P({ mode: 'linePoint', axis: ax, points: [sp('S2', 'p1')] })]);
add('axisfeat planes angle', [...base, ...PLN, A([fp('A1'), fp('A2')]), P({ mode: 'angle', base: std('top'), axis: ax, angle: 20 })]);
add('axisfeat planes swapped', [...base, ...PLN, A([fp('A2'), fp('A1')]), P({ mode: 'angle', base: std('top'), axis: ax, angle: 20 })]);
add('axisfeat planes perp', [...base, ...PLN, A([fp('A1'), fp('A2')]), P({ mode: 'perpendicular', base: std('front'), axis: ax })]);
add('axisfeat std planes', [...base, A([std('top'), std('front')]), P({ mode: 'angle', base: std('front'), axis: ax, angle: 20 })]);
add('axisfeat world', [...base, A([AX('z')]), P({ mode: 'angle', base: std('front'), axis: ax, angle: 20 })]);
add('axisfeat sketchLine', [...base, A([sl('S1', 'e4')]), P({ mode: 'angle', base: std('top'), axis: ax, angle: 20 })]);
add('axisfeat parallel planes', [...base, A([std('top'), std('top')]), P({ mode: 'angle', base: std('top'), axis: ax, angle: 20 })]);
add('axisfeat empty', [...base, A([]), P({ mode: 'angle', base: std('top'), axis: ax, angle: 20 })]);
add('axisfeat one plane', [...base, A([std('top')]), P({ mode: 'angle', base: std('top'), axis: ax, angle: 20 })]);
add('axisfeat cyl face', [...CYL, A([{ kind: 'face', id: 'CY.S.e1' }]), P({ mode: 'angle', base: std('front'), axis: ax, angle: 20 })]);
add('axisfeat planar face', [...CYL, A([{ kind: 'face', id: 'CY.C0' }]), P({ mode: 'angle', base: std('front'), axis: ax, angle: 20 })]);
add('axisfeat edge', [...BOX, A([{ kind: 'edge', faces: ['BX.C0', 'BX.S.e3'], at: [10, 0, -10] }]), P({ mode: 'angle', base: std('front'), axis: ax, angle: 20 })]);
// angle/perp with sketch-line and edges
add('angle sketchLine', [...base, P({ mode: 'angle', base: std('top'), axis: sl('S1', 'e4'), angle: 30 })]);
add('perp sketchLine', [...base, P({ mode: 'perpendicular', base: std('top'), axis: sl('S1', 'e4') })]);
let ei = 0;
for (const [fa, fb, at] of [['BX.C0', 'BX.S.e1', [10, 0, 0]], ['BX.C0', 'BX.S.e2', [20, 0, -5]], ['BX.C0', 'BX.S.e3', [10, 0, -10]], ['BX.C0', 'BX.S.e4', [0, 0, -5]], ['BX.C1', 'BX.S.e1', [10, 10, 0]], ['BX.C1', 'BX.S.e2', [20, 10, -5]], ['BX.C1', 'BX.S.e3', [10, 10, -10]], ['BX.C1', 'BX.S.e4', [0, 10, -5]], ['BX.S.e1', 'BX.S.e2', [20, 5, 0]], ['BX.S.e2', 'BX.S.e3', [20, 5, -10]], ['BX.S.e3', 'BX.S.e4', [0, 5, -10]], ['BX.S.e4', 'BX.S.e1', [0, 5, 0]]]) {
  add('edge angle ' + ei++, [...BOX, P({ mode: 'angle', base: std('front'), axis: { kind: 'edge', faces: [fa, fb], at }, angle: 25 })]);
}
// normal / normalCurve / curve
for (const [n, o] of [['S1', {}], ['S1 end', { atEnd: true }], ['S1 flip', { flip: true }]]) add('normal ' + n, [PL, PA, PC, P({ mode: 'normal', path: 'PL', ...o })]);
for (const [s, o] of [['PA', {}], ['PA', { atEnd: true }], ['PC', {}], ['PC', { atEnd: true }]]) add('normal ' + s + JSON.stringify(o), [PL, PA, PC, P({ mode: 'normal', path: s, ...o })]);
add('normal nopath', [PL, P({ mode: 'normal' })]);
add('normal badpath', [PL, P({ mode: 'normal', path: 'Z' })]);
for (const [n, c, o] of [['line', sc('PL', 'e2'), {}], ['line end', sc('PL', 'e2'), { atEnd: true }], ['line flip', sc('PL', 'e2'), { flip: true }], ['line pt', sc('PL', 'e2'), { points: [sp('S1', 'p3')] }], ['arc', sc('PA', 'e1'), {}], ['arc end', sc('PA', 'e1'), { atEnd: true }], ['circle', sc('PC', 'e1'), {}], ['circle end', sc('PC', 'e1'), { atEnd: true }]])
  add('normalCurve ' + n, [PL, PA, PC, S1, P({ mode: 'normalCurve', curves: [c], ...o })]);
add('normalCurve empty', [PL, P({ mode: 'normalCurve', curves: [] })]);
const dirs = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [1, -2], [-3, 1]];
for (const pl of ['front', 'top', 'right']) {
  const pts = [[2, 3], ...dirs.map(([x, y]) => [2 + 4 * x, 3 + 4 * y])];
  const S = SK('S', pl, pts, dirs.map((_, i) => ln('e' + i, 0, i + 1)));
  dirs.forEach((d, i) => add(`normalCurve ${pl} dir ${d}`, [S, P({ mode: 'normalCurve', curves: [sc('S', 'e' + i)] })]));
}
add('normalCurve edge', [...BOX, P({ mode: 'normalCurve', curves: [{ kind: 'edge', faces: ['BX.C0', 'BX.S.e1'], at: [10, 0, 0] }] })]);
add('normalCurve circ edge', [...CYL, P({ mode: 'normalCurve', curves: [{ kind: 'edge', faces: ['CY.C1', 'CY.S.e1'], at: [10, 10, 0] }] })]);
for (const [n, cs, o] of [['circle', [sc('PC', 'e1')], {}], ['circle off5', [sc('PC', 'e1')], { offset: 5 }], ['circle flip', [sc('PC', 'e1')], { flip: true }], ['arc', [sc('PA', 'e1')], {}], ['line', [sc('PL', 'e1')], {}], ['2 lines', [sc('PL', 'e1'), sc('PL', 'e2')], {}], ['2 lines off', [sc('S1', 'e1'), sc('S1', 'e3')], { offset: 2 }], ['non planar', [sc('PL', 'e1'), sc('PC', 'e1')], {}], ['empty', [], {}]])
  add('curve ' + n, [PL, PA, PC, S1, P({ mode: 'curve', curves: cs, ...o })]);
add('curve nobase+base', [PC, P({ mode: 'curve', base: std('top'), curves: [sc('PC', 'e1')] })]);
add('curve circle edge', [...CYL, P({ mode: 'curve', curves: [{ kind: 'edge', faces: ['CY.C1', 'CY.S.e1'], at: [10, 10, 0] }] })]);
// tangent
const tf = 'CY.S.e1';
const q = (n) => sp('CS', n);
for (const [n, o] of [['pt q', { tangentTo: 'point', points: [q('q')] }], ['pt default', { points: [q('q')] }], ['at', { at: [20, 5, -5] }], ['pt q2', { points: [q('q2')] }], ['pt q3', { points: [q('q3')] }], ['pt q3 flip', { points: [q('q3')], flip: true }], ['pt axis', { points: [q('c')] }],
  ['plane right', { tangentTo: 'plane', base: std('right') }], ['plane right 2nd', { tangentTo: 'plane', base: std('right'), second: true }], ['plane top', { tangentTo: 'plane', base: std('top') }], ['plane front', { tangentTo: 'plane', base: std('front') }], ['plane none', { tangentTo: 'plane' }],
  ['line y', { tangentTo: 'line', axis: AX('y') }], ['line par', { tangentTo: 'line', axis: sl('CL', 'e1') }], ['line par 2nd', { tangentTo: 'line', axis: sl('CL', 'e1'), second: true }], ['line par rev', { tangentTo: 'line', axis: sl('CL', 'e2') }]])
  add('tangent cyl ' + n, [...CYL, CL, P({ mode: 'tangent', face: tf, ...o })]);
add('tangent planar face', [...CYL, P({ mode: 'tangent', face: 'CY.C1', tangentTo: 'plane', base: std('right') })]);
add('tangent noface', [...CYL, P({ mode: 'tangent', tangentTo: 'plane', base: std('right') })]);
const sf = 'SR.S.arc';
for (const [n, o] of [['pt q', { points: [sp('SS', 'q')] }], ['pt pole', { points: [sp('SS', 'q3')] }], ['pt center', { points: [sp('SS', 'o')] }], ['plane right', { tangentTo: 'plane', base: std('right') }], ['plane right 2nd', { tangentTo: 'plane', base: std('right'), second: true }], ['plane top', { tangentTo: 'plane', base: std('top') }], ['plane top flip', { tangentTo: 'plane', base: std('top'), flip: true }],
  ['line', { tangentTo: 'line', axis: sl('SS', 'sl') }], ['line 2nd', { tangentTo: 'line', axis: sl('SS', 'sl'), second: true }], ['line through', { tangentTo: 'line', axis: sl('SS', 'cl') }]])
  add('tangent sph ' + n, [...SPH, P({ mode: 'tangent', face: sf, ...o })]);
// flipped / offset cylinders, planar-face bases
const CYF = [{ ...CYL[0], id: 'CS2', plane: std('front') }, { type: 'extrude', id: 'CY2', op: 'add', sketch: 'CS2', end: 'blind', depth: 10, flip: true }];
for (const [n, o] of [['pt', { points: [sp('CS2', 'q')] }], ['pt axis', { points: [sp('CS2', 'c')] }], ['plane right', { tangentTo: 'plane', base: std('right') }], ['plane top', { tangentTo: 'plane', base: std('top') }], ['plane top 2nd', { tangentTo: 'plane', base: std('top'), second: true }]])
  add('tangent cyl flipped ' + n, [...CYF, P({ mode: 'tangent', face: 'CY2.S.e1', ...o })]);
add('axisfeat cyl flipped', [...CYF, A([{ kind: 'face', id: 'CY2.S.e1' }]), P({ mode: 'angle', base: std('top'), axis: ax, angle: 20 })]);
add('parallel face base', [...BOX, S1, P({ mode: 'parallel', base: { kind: 'face', id: 'BX.C1' }, points: [sp('S1', 'p3')] })]);
add('3pt on face sketch', [...BOX, { type: 'sketch', id: 'FS', plane: { kind: 'face', id: 'BX.C1' }, points: [{ id: 'p0', x: 1, y: 2 }, { id: 'p1', x: 8, y: 3 }, { id: 'p2', x: 3, y: 9 }], entities: [], constraints: [] }, P({ mode: 'threePoints', points: [sp('FS', 'p0'), sp('FS', 'p1'), sp('FS', 'p2')] })]);
add('normalCurve circ edge pt', [...CYL, P({ mode: 'normalCurve', curves: [{ kind: 'edge', faces: ['CY.C1', 'CY.S.e1'], at: [10, 10, 0] }], atEnd: true })]);
add('curve circle edge off', [...CYL, P({ mode: 'curve', offset: 3, flip: true, curves: [{ kind: 'edge', faces: ['CY.C0', 'CY.S.e1'], at: [10, 0, 0] }] })]);
add('curve line edge', [...BOX, P({ mode: 'curve', curves: [{ kind: 'edge', faces: ['BX.C0', 'BX.S.e1'], at: [10, 0, 0] }] })]);
add('curve 2 edges', [...BOX, P({ mode: 'curve', curves: [{ kind: 'edge', faces: ['BX.C0', 'BX.S.e1'], at: [10, 0, 0] }, { kind: 'edge', faces: ['BX.C0', 'BX.S.e2'], at: [20, 0, -5] }] })]);
add('curve 2 edges nonplanar', [...BOX, P({ mode: 'curve', curves: [{ kind: 'edge', faces: ['BX.C0', 'BX.S.e1'], at: [10, 0, 0] }, { kind: 'edge', faces: ['BX.C1', 'BX.S.e3'], at: [10, 10, -10] }] })]);
add('curve coplanar line+circle', [{ ...PC, id: 'PC2', points: [{ id: 'c', x: 3, y: 4 }, { id: 'p0', x: 0, y: 0 }, { id: 'p1', x: 5, y: 5 }], entities: [{ id: 'e1', type: 'circle', c: 'c', r: 5 }, { id: 'e2', type: 'line', p1: 'p0', p2: 'p1' }] }, P({ mode: 'curve', curves: [sc('PC2', 'e2'), sc('PC2', 'e1')] })]);
add('curve circle+line coplanar', [{ ...PC, id: 'PC2', points: [{ id: 'c', x: 3, y: 4 }, { id: 'p0', x: 0, y: 0 }, { id: 'p1', x: 5, y: 5 }], entities: [{ id: 'e1', type: 'circle', c: 'c', r: 5 }, { id: 'e2', type: 'line', p1: 'p0', p2: 'p1' }] }, P({ mode: 'curve', curves: [sc('PC2', 'e1'), sc('PC2', 'e2')] })]);
add('normal multi-seg arc chain', [{ id: 'MP', type: 'sketch', plane: std('front'), points: [{ id: 'a', x: 0, y: 0 }, { id: 'b', x: 10, y: 0 }, { id: 'c', x: 10, y: 10 }, { id: 'd', x: 20, y: 10 }], entities: [{ id: 'e1', type: 'line', p1: 'a', p2: 'b' }, { id: 'e2', type: 'arc', c: 'c', p1: 'b', p2: 'd' }], constraints: [] }, P({ mode: 'normal', path: 'MP', atEnd: true })]);
add('normal chain reversed order', [{ id: 'MP', type: 'sketch', plane: std('top'), points: [{ id: 'a', x: 0, y: 0 }, { id: 'b', x: 10, y: 0 }, { id: 'c', x: 10, y: 10 }], entities: [{ id: 'e2', type: 'line', p1: 'c', p2: 'b' }, { id: 'e1', type: 'line', p1: 'b', p2: 'a' }], constraints: [] }, P({ mode: 'normal', path: 'MP' })]);
// regression of the older modes
add('offset', [...base, P({ mode: 'offset', base: std('top'), offset: 7 })]);
add('angle x', [...base, P({ mode: 'angle', base: std('top'), axis: AX('x'), angle: 30 })]);
add('mid', [...base, P({ mode: 'mid', base: std('right'), base2: std('top') })]);

const reg = (k, feats) => JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'm', part: { format: 'ck-part', version: 1, features: feats } } })).json).result;
const sum = (r) => ({ errors: (r.errors || []).filter((e) => e[0] === 'P' || e[0] === 'A').map((e) => e[0] + ':' + e[1].code + (e[1].params && e[1].params.dev !== undefined ? ':' + e[1].params.dev : '')), plane: (r.planes || []).filter((p) => p[0] === 'P').map((p) => p[1])[0] });
const near = (a, b) => ['o', 'x', 'y', 'z'].every((k) => a[k].every((v, i) => Math.abs(v - b[k][i]) < 1e-6));
const KNOWN = new Set(JSON.parse(readFileSync(new URL('./plane_known.json', import.meta.url), 'utf8')));
let bad = 0, known = 0, n = 0;
for (const [name, feats] of Object.entries(cases)) {
  n++;
  const a = sum(reg(ref, feats)), b = sum(reg(mine, feats));
  const ok = JSON.stringify(a.errors) === JSON.stringify(b.errors) && (!a.plane === !b.plane) && (!a.plane || near(a.plane, b.plane));
  if (!ok) {
    if (KNOWN.has(name)) { known++; continue; }
    bad++;
    console.log('DIFF', name, '\n  oracle', JSON.stringify(a), '\n  ours  ', JSON.stringify(b));
  } else if (verbose) console.log('ok  ', name);
}
console.log(`${n} cases, ${bad} differ, ${known} known deviations`);
process.exit(bad ? 1 : 0);
