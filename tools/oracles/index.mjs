// Independent closed-form oracles for the B-rep cases of tools/brep_cases.mjs.
//   export const oracles = { caseName: { volume, area, com, bbox:{min,max}, bboxExact } }
// Every value is derived from the feature parameters (duplicated here, with
// the case's .ckpart numbers in comments) by textbook formulas -- never from
// the kernel. `area` may be null where no closed form is used (oblique cone
// lateral area). `bboxExact:false` marks curved parts: the kernel box of a
// curved body is mesh-sampled, so the gate only requires containment in the
// true box plus a small tessellation slack (see check.mjs).
//
// Conventions (from the plane definitions): top: sketch (x,y) -> world (x,*,-y),
// normal +y; front: (x,y,0), normal +z; right: (-z,y), normal +x. Revolves
// sweep from +x toward -z about the world Y axis; patterns rotate right-handed.
import {
  PI, poly, rectL, circL, halfDiscL, prism, boxS, cylY, csg, lump, revolvePoly, edgeCut, simpson,
} from './geom.mjs';

const o = {};
const done = (s, extra = {}) => ({
  volume: s.V, area: s.A, com: s.M.map((m) => m / s.V), bbox: s.bbox, bboxExact: s.exact, ...extra,
});
const R40x20 = rectL(0, 0, 40, 20); // rect(0,0,40,20) of brep_cases `box` & friends

// ---- extrudes -------------------------------------------------------------
const BOX = prism('top', [[R40x20, 1]], 0, 10); // 40x20x10, y 0..10, z -20..0
o.box = done(BOX);
o.flip = done(prism('front', [[R40x20, 1]], -10, 0)); // extrude flip:true
o.mid = done(prism('front', [[R40x20, 1]], -5, 5)); // end:'mid'
o.exprDepth = done(prism('right', [[R40x20, 1]], 0, 14)); // depth '=2*w', w=7
o.cyl = done(prism('top', [[circL(5, 3, 10), 1]], 0, 10)); // circle c(5,3) r10
o.dext = done(prism('front', [[halfDiscL(0, 0, 10), 1]], 0, 5)); // D: chord (-10,0)-(10,0) + arc, depth 5
o.box3 = done(prism('front', [[R40x20, 1], [rectL(5, 5, 15, 15), -1], [circL(30, 10, 4), -1]], 0, 10));
o.two = done(prism('front', [[rectL(0, 0, 10, 10), 1], [rectL(20, 0, 30, 10), 1]], 0, 5));
o.triPrism = done(prism('top', [[poly([[0, 0], [30, 0], [10, 20]]), 1]], 0, 10));

// draft 10 degrees (inward) on the 40x20 front-plane box, depth 10
{
  const k = Math.tan((10 * PI) / 180), h = 10, a = 40, b = 20;
  const Az = (z) => (a - 2 * k * z) * (b - 2 * k * z);
  const V = simpson(Az, h), Mz = simpson((z) => z * Az(z), h);
  const a2 = a - 2 * k * h, b2 = b - 2 * k * h, sl = h / Math.cos(Math.atan(k));
  const A = a * b + a2 * b2 + 2 * ((a + a2) / 2) * sl + 2 * ((b + b2) / 2) * sl;
  o.draft = done({ V, M: [V * 20, V * 10, Mz], A, bbox: { min: [0, 0, 0], max: [40, 20, 10] }, exact: true });
}

// ---- revolves (profile in the front plane about the +Y construction axis) ----
const rect2 = (x0, y0, x1, y1) => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
const D2R = PI / 180;
o.rev360 = done(revolvePoly(rect2(10, 0, 20, 5), 2 * PI));
o.rev90 = done(revolvePoly(rect2(10, 0, 20, 5), 90 * D2R));
o.rev90flip = done(revolvePoly(rect2(10, 0, 20, 5), 90 * D2R, 'flip'));
o.rev90mid = done(revolvePoly(rect2(10, 0, 20, 5), 90 * D2R, 'mid'));
for (const a of [30, 150, 200, 270]) o[`rev${a}`] = done(revolvePoly(rect2(10, 0, 20, 5), a * D2R));
o.rev200flip = done(revolvePoly(rect2(10, 0, 20, 5), 200 * D2R, 'flip'));
o.rev270mid = done(revolvePoly(rect2(10, 0, 20, 5), 270 * D2R, 'mid'));
o.revax360 = done(revolvePoly(rect2(0, 0, 20, 5), 2 * PI)); // rect touching the axis
o.revax90 = done(revolvePoly(rect2(0, 0, 20, 5), 90 * D2R));
o.cone = done(revolvePoly([[5, 0], [15, 0], [5, 10]], 120 * D2R)); // triangle, 120 deg
o.frustum = done(revolvePoly([[0, 0], [10, 0], [6, 10], [0, 10]], 2 * PI));
o.ring = done(revolvePoly([[4, 0], [10, 0], [6, 10], [4, 10]], 2 * PI));
// sphere r=10 at the origin (semicircle revolved) and torus R=20, r=5
o.sphere = done({ V: (4 / 3) * PI * 1000, M: [0, 0, 0], A: 4 * PI * 100, bbox: { min: [-10, -10, -10], max: [10, 10, 10] }, exact: false });
o.torus = done({ V: 2 * PI * PI * 20 * 25, M: [0, 0, 0], A: 4 * PI * PI * 20 * 5, bbox: { min: [-25, -5, -25], max: [25, 5, 25] }, exact: false });

// ---- cuts / bosses on the 40x20x10 box (hole circle r2 at sketch (5,5) -> world (5,*,-5)) ----
const hole = (cx, cz, r = 2) => cylY(cx, cz, r, 0, 10);
const thruA = (r) => 2 * PI * r * 10 - 2 * PI * r * r; // through-hole area change
o.cutHole = done(csg(BOX, [[-1, hole(5, -5)]], thruA(2)));
o.cutThroughFlip = done(csg(BOX, [[-1, hole(20, -10, 3)]], thruA(3))); // circle (20,10) r3
o.cutMiss = done(BOX); // circle at x=100: no intersection
o.pocket = done(csg(BOX, [[-1, cylY(10, -10, 3, 6, 10)]], 2 * PI * 3 * 4)); // r3 depth 4 from top
o.boss = done(csg(BOX, [[1, cylY(10, -10, 3, 10, 15)]], 2 * PI * 3 * 5)); // r3 height 5 on top
// linear pattern x3 spacing 10 along +x
o.lpatternCut = done(csg(BOX, [5, 15, 25].map((x) => [-1, hole(x, -5)]), 3 * thruA(2)));
// reverse z pattern x2 spacing 8: copy at z -5 -> -13
o.lpatternRev = done(csg(BOX, [-5, -13].map((z) => [-1, hole(5, z)]), 2 * thruA(2)));
// circular pattern (count 3, 90deg equal => 0,45,90): copy 2 centre (0,-5*sqrt2) is half outside (x<0),
// copy 3 centre (-5,-5) is fully outside
{
  const r = 2, h = 10, z2 = -5 * Math.SQRT2;
  const half = { V: (PI * r * r * h) / 2, M: [0, 0, 0] };
  half.M = [half.V * ((4 * r) / (3 * PI)), half.V * 5, half.V * z2];
  const dA = thruA(2) + (PI * r * h - PI * r * r - 2 * r * h);
  o.cpatternCut = done(csg(BOX, [[-1, hole(5, -5)], [-1, half]], dA));
}
// 4 bosses r3 h5 (circle (30,10) on the top face) rotated 90deg steps; only the first touches the box
{
  const b = (x, z) => cylY(x, z, 3, 10, 15);
  let s = csg(BOX, [[1, b(30, -10)]], 2 * PI * 3 * 5);
  for (const [x, z] of [[-10, -30], [-30, 10], [10, 30]]) s = lump(s, b(x, z));
  o.cpatternBoss = done(s);
}
// hole r2 at (5,-5), boss r2 x15 from y=0 at (5,-15), mirrored about x=0: only the boss is copied (hole copy misses)
{
  const boss = cylY(5, -15, 2, 10, 15), mirrored = cylY(-5, -15, 2, 0, 15);
  const s = csg(BOX, [[-1, hole(5, -5)], [1, boss]], thruA(2) + 2 * PI * 2 * 5);
  o.mirrorTwo = done(lump(s, mirrored));
}

// ---- straight-edge fillets/chamfers on the box (convex 90 deg edges) ----
{
  const E = (kind, r, len, mid, a, b) => edgeCut(kind, r, len, mid, a, b);
  const apply = (cuts) => done(csg(BOX, cuts.map((c) => [-1, c.piece]), cuts.reduce((s, c) => s + c.dA, 0)));
  const topE1 = (k, r) => E(k, r, 40, [20, 10, 0], [0, -1, 0], [0, 0, -1]);
  const topE3 = (k, r) => E(k, r, 40, [20, 10, -20], [0, -1, 0], [0, 0, 1]);
  o.filletOne = apply([topE1('fillet', 2)]);
  o.filletTwo = apply([topE1('fillet', 2), topE3('fillet', 2)]);
  o.chamferOne = apply([topE1('chamfer', 2)]);
  o.chamferTwo = apply([topE1('chamfer', 2), topE3('chamfer', 2)]);
  o.filletRail = apply([E('fillet', 3, 10, [40, 5, 0], [-1, 0, 0], [0, 0, -1])]);
  o.chamferRail = apply([E('chamfer', 3, 10, [40, 5, 0], [-1, 0, 0], [0, 0, -1])]);
  o.filletTwoParallel = o.filletTwo;
}

// ---- shells of the 40x20x10 box / r10 cylinder (analytic: outer minus cavity) ----
{
  // area = outer + cavity - 2 * (cavity area lying in the removed (open) outer faces)
  const shell = (outer, cav, openA) => done(csg(outer, [[-1, cav]], cav.A - 2 * openA));
  const cavBox = (x0, x1, y0, y1, z0, z1) => boxS([x0, y0, z0], [x1, y1, z1]);
  o.shellBoxClosed = shell(BOX, cavBox(1, 39, 1, 9, -19, -1), 0); // t=1
  o.shellBoxOpen = shell(BOX, cavBox(1, 39, 1, 10, -19, -1), 38 * 18); // top (F2.C1) open
  o.shellBoxOpenSide = shell(BOX, cavBox(1, 39, 1, 9, -19, 0), 38 * 8); // side e1 (z=0) open
  o.shellBoxOpenTwo = shell(BOX, cavBox(1, 39, 1, 10, -19, 0), 38 * 19 + 38 * 9); // side + top open
  o.shellBoxOpenThick = shell(BOX, cavBox(4.99, 35.01, 4.99, 10, -15.01, -4.99), 30.02 * 10.02); // t=4.99, top open
  // outward: cavity is the original box, walls grow outward by 1 (top stays open)
  o.shellBoxOutward = done(csg(boxS([-1, -1, -21], [41, 10, 1]), [[-1, BOX]], BOX.A - 2 * 800));
  const CYL = cylY(5, -3, 10, 0, 10);
  o.shellCylOpen = shell(CYL, cylY(5, -3, 9, 1, 10), PI * 81);
  o.shellCylClosed = shell(CYL, cylY(5, -3, 9, 1, 9), 0);
}

// ---- lofts with exact sections ----
{
  // square 20x20 at y=0 -> 10x10 at y=30 (centred): ruled frustum
  const h = 30, Az = (y) => (20 - (10 * y) / h) ** 2;
  const V = simpson(Az, h), My = simpson((y) => y * Az(y), h);
  const A = 400 + 100 + 4 * ((20 + 10) / 2) * Math.hypot(h, 5);
  o.loftSq2 = done({ V, M: [0, My, 0], A, bbox: { min: [-10, 0, -10], max: [10, 30, 10] }, exact: true });
  // circle r8 at (0,0) y=0 -> circle r4 at (2,0) y=30: oblique cone frustum (lateral area has no elementary form)
  const Ac = (y) => PI * (8 - (4 * y) / h) ** 2;
  const Vc = simpson(Ac, h);
  o.loftCircCirc = done({
    V: Vc, M: [simpson((y) => ((2 * y) / h) * Ac(y), h), simpson((y) => y * Ac(y), h), 0], A: null,
    bbox: { min: [-8, 0, -8], max: [8, 30, 8] }, exact: false,
  });
}

export const oracles = o;
export default oracles;
