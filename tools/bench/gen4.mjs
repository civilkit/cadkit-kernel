// Authoring helper for tools/bench/brepjs/*.ckpart.json: the 'mechanical' examples of the brepjs playground
// (Apache-2.0, https://github.com/andymai/brepjs) translated by hand into our feature tree. Parameters are
// paraphrased from the examples' stated dimensions; no example source is copied. Reference figures come from
// running each example with brepjs 20.5.0 (see manifest.json).
//
// Frame: the examples are Z-up (primitives grow along +Z). Authoring below is in those cq coordinates; lib4.mjs
// maps them onto our Y-up world (x,y,z)->(x,z,-y) and picks the sketch plane for each cq axis.
// usage: node tools/bench/gen4.mjs [name ...]     (rewrites tools/bench/brepjs/*.ckpart.json)
import { writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { B, Sk, rad, rrect } from './lib4.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const out = path.join(here, 'brepjs');
mkdirSync(out, { recursive: true });
const only = process.argv.slice(2);
const parts = {};
const def = (name, fn) => { if (only.length && !only.includes(name)) return; const b = new B(name); fn(b); parts[name] = b.json(); };
// predicates for blend edge selection (cq frame)
const bothSide = (e) => e.faces.slice(0, 2).every((f) => f.includes('.S.'));
const isCircle = (e) => !!e.circle;
const ring = (R, n, a0 = 0) => Array.from({ length: n }, (_, i) => [R * Math.cos(rad(a0 + (360 * i) / n)), R * Math.sin(rad(a0 + (360 * i) / n))]);
const rectC = (cx, cy, w, h) => (sk) => sk.rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2);

// 1. o-ring: torus of cord radius 1.5 on a 10.75 centreline radius
def('o-ring', (b) => {
  b.revZ((sk) => sk.circle(10.75, 0, 1.5));
});

// 2. rubber-foot: tapered post, rounded rims, washer recess from below, screw hole
def('rubber-foot', (b) => {
  const rTop = 12.5 - 12 * Math.tan(rad(10));
  b.revZ((sk) => sk.poly([[0, 0], [12.5, 0], [rTop, 12], [0, 12]]));
  b.blend('fillet', 2, isCircle);
  b.cut('z', -1, 9, (sk) => sk.circle(0, 0, 4.5));
  b.cut('z', -1, 13, (sk) => sk.circle(0, 0, 2.2));
});

// 8. nema-stepper: chamfered square can (shaft face at z=0), pilot boss, shaft, four tapped holes
def('nema-stepper', (b) => {
  b.prism('z', -40, 0, (sk) => sk.rect(-21.15, -21.15, 21.15, 21.15));
  b.blend('chamfer', 2, bothSide);
  b.prism('z', -1, 2, (sk) => sk.circle(0, 0, 11));
  b.prism('z', -1, 24, (sk) => sk.circle(0, 0, 2.5));
  b.cut('z', -4.5, 1, (sk) => { for (const [x, y] of [[-15.5, -15.5], [15.5, -15.5], [15.5, 15.5], [-15.5, 15.5]]) sk.circle(x, y, 1.5); });
});

// 10. tripod-rc2-plate: dovetail prism along Y, eight long edges chamfered, base corner reliefs, stepped screw hole
def('tripod-rc2-plate', (b) => {
  const xb = 21.2, xt = 18.7, flat = 3, T = 10.5, zMid = 7.5;
  b.prism('y', -26.25, 26.25, (sk) => sk.poly([[-xb, 0], [xb, 0], [xb, flat], [xt, zMid], [xt, T], [-xt, T], [-xt, zMid], [-xb, flat]]));
  b.blend('chamfer', 0.6, bothSide);
  b.cut('z', -0.5, 4.5, (sk) => { for (const sx of [-1, 1]) for (const sy of [-1, 1]) { const x0 = sx * 18.7 - 3.5, y0 = sy > 0 ? 0.75 : -26.75; sk.rect(x0, y0, x0 + 7, y0 + 26); } });
  b.cut('z', -1, 11.5, (sk) => sk.circle(0, 0, 3.4));
  b.cut('z', -1, 4, (sk) => sk.circle(0, 0, 6));
});

// 11. l-angle-gusset-bracket: two legs, triangular web, six holes. (The example's heel "chamfer" cutter touches the
// body in a single corner point only, volume unchanged, so it is not translated.)
def('l-angle-gusset-bracket', (b) => {
  b.prism('x', -20, 20, (sk) => sk.rect(0, 0, 60, 4));
  b.prism('x', -20, 20, (sk) => sk.rect(0, 0, 4, 60));
  b.prism('x', -2, 2, (sk) => sk.poly([[3.5, 3.5], [42, 3.5], [3.5, 42]]));
  const ds = [0, 1, 2].map((i) => 16.32 + (53.28 - 16.32) * (i / 2));
  b.cut('z', -1, 5, (sk) => { for (const d of ds) sk.circle(11.2, d, 2.6); });
  b.cut('y', -1, 5, (sk) => { for (const d of ds) sk.circle(11.2, d, 2.6); });
});

// 9. linear-bearing-block: barrel along X + base slab + ear block, bore, flex slot, counterbored cross bolt
def('linear-bearing-block', (b) => {
  b.prism('x', -12, 12, (sk) => sk.circle(0, 0, 10.5));
  b.prism('z', -14.5, -9.5, (sk) => sk.rect(-12, -10.5, 12, 10.5));
  b.prism('z', 0, 19.5, (sk) => sk.rect(-12, -7.5, 12, 7.5));
  b.cut('x', -13, 13, (sk) => sk.circle(0, 0, 7.5));
  b.cut('z', 0, 19.5, (sk) => sk.rect(-13, -2.5, 13, 2.5));
  b.cut('y', -8.5, 8.5, (sk) => sk.circle(0, 15, 1.6));
  b.cut('y', 2.5, 8.5, (sk) => sk.circle(0, 15, 3));
});

// 4. flanged-tee: run along X, branch up Z, three flanges, bores, 12 bolt holes
def('flanged-tee', (b) => {
  b.prism('x', -65, 65, (sk) => sk.circle(0, 0, 18));
  b.prism('z', 0, 60, (sk) => sk.circle(0, 0, 15));
  b.prism('x', 57, 65, (sk) => sk.circle(0, 0, 30));
  b.prism('x', -65, -57, (sk) => sk.circle(0, 0, 30));
  b.prism('z', 52, 60, (sk) => sk.circle(0, 0, 26));
  b.cut('x', -70, 70, (sk) => sk.circle(0, 0, 11));
  b.cut('z', -20, 70, (sk) => sk.circle(0, 0, 9));
  const r4 = ring(24, 4, 45);
  b.cut('x', 55, 67, (sk) => { for (const [u, v] of r4) sk.circle(u, v, 3.5); });
  b.cut('x', -67, -55, (sk) => { for (const [u, v] of r4) sk.circle(u, v, 3.5); });
  b.cut('z', 50, 62, (sk) => { for (const [u, v] of ring(21, 4, 45)) sk.circle(u, v, 3.5); });
});

// 6. fan-guard: rounded plate, air opening, hub, four rings, four spokes, corner holes
def('fan-guard', (b) => {
  b.prism('z', 0, 2.5, (sk) => sk.rect(-30, -30, 30, 30));
  b.blend('fillet', 3.75, bothSide);
  b.cut('z', -1, 3.5, (sk) => sk.circle(0, 0, 27.5));
  b.prism('z', 0, 2.5, (sk) => sk.circle(0, 0, 4.8));
  const pitch = (30 - 2.5 - 4.8) / 4;
  for (let i = 1; i <= 4; i++) { const ro = 4.8 + i * pitch; b.prism('z', 0, 2.5, (sk) => { sk.circle(0, 0, ro); sk.circle(0, 0, ro - 1.25); }); }
  for (const a of [0, 45, 90, 135]) {
    const c = Math.cos(rad(a)), s = Math.sin(rad(a));
    b.prism('z', 0, 2.5, (sk) => sk.poly([[-30, -1.25], [30, -1.25], [30, 1.25], [-30, 1.25]].map(([x, y]) => [x * c - y * s, x * s + y * c])));
  }
  b.cut('z', -1, 3.5, (sk) => { for (const [x, y] of [[-25, -25], [25, -25], [25, 25], [-25, 25]]) sk.circle(x, y, 2.4); });
});

// 5. gridfinity-bin (1x1x3): rounded block with pocket, socket foot (two ruled frusta), stacking lip (outer prism
// minus a five-piece inner profile). Our loft is smooth through its sections, so each ruled stretch is its own loft.
def('gridfinity-bin', (b) => {
  const rr = (inset) => (sk) => rrect(sk, 0, 0, 41.5 - 2 * inset, 41.5 - 2 * inset, 3.75 - inset);
  b.prism('z', 0, 21, rr(0));
  b.cut('z', 1, 22, rr(1.2));
  b.loft([['z', 0, rr(0)], ['z', -2.4, rr(2.15)]]);
  b.loft([['z', -2.4, rr(2.15)], ['z', -5, rr(2.95)]]);
  b.prism('z', 18.4, 25.4, rr(0));
  const zs = [[18.4, 1.2], [19.8, 2.6], [21, 2.6], [21.7, 1.9], [23.5, 1.9], [25.4, 0.05]];
  for (let i = 0; i + 1 < zs.length; i++) b.loft([['z', zs[i][0], rr(zs[i][1])], ['z', zs[i + 1][0], rr(zs[i + 1][1])]], 'cut');
});

// 3. fluted-knob: cone drum, 16 tilted cylindrical flutes (one cutter on an angled plane + circular pattern), shaft socket
def('fluted-knob', (b) => {
  b.revZ((sk) => sk.poly([[0, 0], [15, 0], [12, 18], [0, 18]]));
  const slant = Math.hypot(3, 18), nR = 18 / slant, nZ = 3 / slant, phi = Math.atan2(3, 18); // wall leans inward going up
  const off = 2.2 - 1.4, cR = 13.5 + nR * off, cZ = 9 + nZ * off, len = slant + 4;
  // cutter axis (-sin phi, cos phi) in (x, z): plane normal to it, tilted about the cq Y axis, then slid along the normal
  const dist = -cR * Math.sin(phi) + cZ * Math.cos(phi) - len / 2;
  const tilt = b.nid();
  b.add({ type: 'plane', id: tilt, mode: 'angle', base: { kind: 'std', id: 'top' }, axis: { kind: 'axis', axis: 'z' }, angle: (phi * 180) / Math.PI });
  const pl = b.nid();
  b.add({ type: 'plane', id: pl, mode: 'offset', base: { kind: 'feature', id: tilt }, offset: dist });
  const sk = new Sk(b.nid(), { kind: 'feature', id: pl }); sk.circle(0, cR * Math.cos(phi) + cZ * Math.sin(phi), 2.2); b.add(sk.json());
  const cutter = b.nid();
  b.add({ type: 'extrude', id: cutter, op: 'cut', sketch: sk.id, end: 'blind', depth: len, flip: true });
  b.add({ type: 'cpattern', id: b.nid(), seeds: [cutter], axis: { kind: 'axis', axis: 'y' }, count: 16, angle: 360, equal: true });
  b.cut('z', -1, 12, (s) => s.circle(0, 0, 3));
});

// 7. gt2-pulley: 20-groove toothed drum, two flanges, hub, bore, radial grub hole
def('gt2-pulley', (b) => {
  const bodyR = (20 * 2) / Math.PI / 2, flangeR = bodyR + 1.5;
  b.prism('z', 0, 7, (sk) => sk.circle(0, 0, bodyR));
  const cx = bodyR + 1.75 / 2 - 0.75;
  b.cut('z', -1, 8, (sk) => { for (let i = 0; i < 20; i++) { const a = rad((360 * i) / 20); rrect(sk, cx * Math.cos(a), cx * Math.sin(a), 1.75, 1.8, 0.9, (360 * i) / 20); } });
  b.prism('z', 0, 1, (sk) => sk.circle(0, 0, flangeR));
  b.prism('z', 6, 7, (sk) => sk.circle(0, 0, flangeR));
  b.prism('z', -6, 0, (sk) => sk.circle(0, 0, 6));
  b.cut('z', -7, 8, (sk) => sk.circle(0, 0, 2.5));
  b.cut('y', -7, 7, (sk) => sk.circle(0, -3, 1.5));
});

// 12. pipe-saddle-p-clamp: half-tube saddle, two feet, fillets where the feet meet the dome, two bolt holes
def('pipe-saddle-p-clamp', (b) => {
  b.prism('x', -7, 7, (sk) => { sk.circle(0, 0, 13); sk.circle(0, 0, 10); });
  b.cut('z', -17, 0, (sk) => sk.rect(-9, -31, 9, 31));
  b.prism('z', -2, 2, (sk) => sk.rect(-7, 9, 7, 29));
  b.prism('z', -2, 2, (sk) => sk.rect(-7, -29, 7, -9));
  b.blend('fillet', 2, (e) => Math.abs(Math.hypot(e.mid[1], e.mid[2]) - 13) < 0.01 && Math.abs(e.mid[2] - 2) < 0.01);
  b.cut('z', -3, 3, (sk) => { sk.circle(0, 21, 3.25); sk.circle(0, -21, 3.25); });
});

// 13. conduit-snap-clip: open C ring with filleted lips, foot, countersunk screw hole
def('conduit-snap-clip', (b) => {
  b.prism('x', -6, 6, (sk) => { sk.circle(0, 13.4, 10.4); sk.circle(0, 13.4, 8); });
  b.cut('z', 13.4, 34.2, (sk) => sk.rect(-7, -4.96, 7, 4.96));
  b.blend('fillet', 1.44, (e) => Math.abs(Math.hypot(e.mid[1], e.mid[2] - 13.4) - 8) < 0.01);
  b.prism('z', -0.5, 3.5, (sk) => sk.rect(-15, -6, 15, 6));
  b.cut('z', -1, 4.5, (sk) => sk.circle(-12.7, 0, 2.3));
  b.revZ((sk) => sk.poly([[-12.7, 1.6], [-12.7 + 2.3, 1.6], [-12.7 + 4.2, 3.9], [-12.7, 3.9]]), 'cut', 360, -12.7);
});

// 14. hobby-rc-servo: filleted case, ear flange, deck, gear boss, shaft, pivot, chamfered boss rim, four ear holes
def('hobby-rc-servo', (b) => {
  b.prism('z', 0, 37, (sk) => sk.rect(-20.25, -10, 20.25, 10));
  b.blend('fillet', 1.5, bothSide);
  b.prism('z', 29.4, 32, (sk) => sk.rect(-27.25, -10, 27.25, 10));
  b.prism('z', 36, 40.5, (sk) => sk.rect(-19.05, -8.8, 19.05, 8.8));
  b.prism('z', 39.5, 44.5, (sk) => sk.circle(9.5, 0, 6.5));
  b.prism('z', 43.5, 49, (sk) => sk.circle(9.5, 0, 2.8));
  b.prism('z', 39.5, 43, (sk) => sk.circle(-9.5, 0, 3));
  b.blend('chamfer', 0.8, (e) => e.circle && Math.abs(e.circle.r - 6.5) < 0.01 && Math.abs(e.mid[2] - 44.5) < 0.01);
  b.cut('z', 28.4, 33, (sk) => { for (const sx of [-1, 1]) for (const sy of [-1, 1]) sk.circle(sx * 24.65, sy * 7.4, 2); });
});

// 15. rotary-potentiometer: chamfered can, boss, bushing, locating pin, neck, D-flat shaft, three lugs, six thread grooves
def('rotary-potentiometer', (b) => {
  b.prism('z', 0, 14, (sk) => sk.circle(0, 0, 12));
  b.blend('chamfer', 1.2, isCircle);
  b.prism('z', 13, 16.5, (sk) => sk.circle(0, 0, 7));
  b.revZ((sk) => sk.poly([[0, 15.5], [4.4, 15.5], [5, 16.1], [5, 23.4], [4.4, 24], [0, 24]]));
  b.prism('z', 13, 17.7, (sk) => sk.rect(10.3, -1.5, 11.7, 1.5));
  b.prism('z', 23, 25, (sk) => sk.circle(0, 0, 3.6));
  b.prism('z', 23, 44, (sk) => sk.circle(0, 0, 3));
  for (const th of [55, 90, 125]) b.prism('z', -4, 1, (sk) => rrect(sk, -11.5 * Math.sin(rad(th)), 11.5 * Math.cos(rad(th)), 3.2, 0.6, 0, th));
  b.cut('z', 30, 46, (sk) => sk.rect(-4, 2, 4, 8));
  for (let z = 17.7; z < 23.2; z += 1) b.revZ((sk) => sk.circle(5, z, 0.28), 'cut');
});

// 16. button-top-battery-cell: AA can, crimp cone, domed nub, insulator groove, chamfered base rim
def('button-top-battery-cell', (b) => {
  b.prism('z', -25.25, 24.25, (sk) => sk.circle(0, 0, 7.25));
  b.revZ((sk) => sk.poly([[0, 22.65], [7.25, 22.65], [3.95, 25.25], [0, 25.25]]));
  const R = (2.75 * 2.75 + 0.25) / 1.0, c0 = 25.25 - R, ze = c0 + Math.sqrt(R * R - 2.75 * 2.75);
  b.revZ((sk) => {
    const c = sk.p(0, c0), p1 = sk.p(2.75, ze), p2 = sk.p(0, 25.25), a = sk.p(0, 23.25), d = sk.p(2.75, 23.25);
    sk.line(a, d); sk.line(d, p1); sk.arc(c, p1, p2); sk.line(p2, a);
  });
  b.revZ((sk) => sk.circle(3.5, -25.25, 0.4), 'cut');
  b.blend('chamfer', 0.6, (e) => e.circle && Math.abs(e.circle.r - 7.25) < 0.01 && Math.abs(e.mid[2] + 25.25) < 0.01);
});

// 17. conical-pour-funnel: spout + cone bowl, rolled rim bead, side hang loop, hollowed by an inner cone and a bore
def('conical-pour-funnel', (b) => {
  b.prism('z', 0, 41, (sk) => sk.circle(0, 0, 6));
  b.revZ((sk) => sk.poly([[0, 40], [6, 40], [37.5, 88], [0, 88]]));
  b.revZ((sk) => sk.circle(35.1, 86.8, 2.4));
  b.rev('x', 42.5, (sk) => { sk.circle(0, 86.8 + 7, 1.9); return [0, 86.8, 10, 86.8]; }); // loop ring: axis along cq Y
  b.revZ((sk) => sk.poly([[0, 40], [4.4, 40], [35.9, 92.4], [0, 92.4]]), 'cut');
  b.cut('z', -2, 90, (sk) => sk.circle(0, 0, 4.4));
});

// 18. axial-fan: filleted 57 mm frame with air bore and four holes, hub, nine blades lofted through three tilted
// sections (smooth loft, as in the example). Two lumps: the blades never reach the frame.
def('axial-fan', (b) => {
  b.prism('z', 0, 15, (sk) => sk.rect(-28.5, -28.5, 28.5, 28.5));
  b.blend('fillet', 4.25, bothSide);
  b.cut('z', -1, 16, (sk) => sk.circle(0, 0, 24.5));
  b.cut('z', -1, 16, (sk) => { for (const sx of [-1, 1]) for (const sy of [-1, 1]) sk.circle(sx * 24.25, sy * 24.25, 2.15); });
  b.prism('z', 0, 17, (sk) => sk.circle(0, 0, 14.5));
  const sec = (r, c, pitch) => ['x', r, (sk) => {
    const cs = Math.cos(rad(pitch)), sn = Math.sin(rad(pitch));
    sk.poly([[-c / 2, -0.6], [c / 2, -0.6], [c / 2, 0.6], [-c / 2, 0.6]].map(([y0, z0]) => [y0 * cs - z0 * sn, 11 + y0 * sn + z0 * cs]));
  }];
  const blade = b.loft([sec(13.5, 10, 40), sec(18.5, 8.5, 30), sec(23.5, 7, 22)]);
  b.add({ type: 'cpattern', id: b.nid(), seeds: [blade], axis: { kind: 'axis', axis: 'y' }, count: 9, angle: 360, equal: true });
});

// 19. knuckle-hinge: two leaves with interleaved knuckles, countersunk holes, pin bore, loose pin. The example's two
// leaves overlap (knuckle of one inside the plate of the other), we build their union (see manifest note).
def('knuckle-hinge', (b) => {
  const band = (50 - 4 * 0.4) / 5, cxs = (i) => -25 + band / 2 + i * (band + 0.4);
  for (const sign of [1, -1]) {
    b.prism('z', 0, 2, (sk) => sk.rect(-25, sign > 0 ? 1 : -22, 25, sign > 0 ? 22 : -1));
    for (let i = 0; i < 5; i++) {
      if ((i % 2 === 0) !== (sign > 0)) continue;
      b.prism('x', cxs(i) - band / 2, cxs(i) + band / 2, (sk) => sk.circle(0, 1, 3));
      b.prism('z', 0, 2, (sk) => sk.rect(cxs(i) - band / 2, sign > 0 ? 0 : -3, cxs(i) + band / 2, sign > 0 ? 3 : 0));
    }
  }
  for (const sign of [1, -1]) {
    const y = sign * 13.45;
    b.cut('z', -1, 3, (sk) => { sk.circle(-16, y, 1.8); sk.circle(16, y, 1.8); });
    for (const x of [-16, 16]) b.rev('y', y, (sk) => { sk.poly([[x, 0.4], [x + 1.8, 0.4], [x + 3.6, 2.2], [x, 2.2]]); return [x, 0, x, 10]; }, 'cut');
  }
  b.cut('x', -27, 27, (sk) => sk.circle(0, 1, 1.45));
  b.prism('x', -26, 26, (sk) => sk.circle(0, 1, 1.3));
});

// 20. dovetail-joint: tapered male tail on a plate (left) and a socket plate (right). The female cavity is the
// untapered flare only: the example's cavity is a prism minus two rotated slabs, a body-level boolean we cannot
// phrase as a feature (see manifest note).
def('dovetail-joint', (b) => {
  const hb = 8 + 8 / 6, ht = 8;
  const slab = (s, pivotX, cx0) => {
    const th = rad(-s * 3), c = Math.cos(th), sn = Math.sin(th);
    return [[s * 8, -6], [s * 24, -6], [s * 24, 36], [s * 8, 36]].map(([x, y]) => { const dx = x - s * pivotX, dy = y + 15; return [cx0 + s * pivotX + dx * c - dy * sn, -15 + dx * sn + dy * c]; });
  };
  b.prism('y', -15, 15, (sk) => sk.poly([[-29 - hb, 0], [-29 + hb, 0], [-29 + ht, 8], [-29 - ht, 8]]));
  for (const s of [1, -1]) b.cut('z', -8, 16, (sk) => sk.poly(slab(s, hb, -29)));
  b.prism('z', -6, 0, (sk) => sk.rect(-52, -15, -6, 15));
  b.prism('z', -12, 0, (sk) => sk.rect(6, -15, 52, 15));
  b.cut('y', -15, 15, (sk) => sk.poly([[29 - (hb + 0.2), -8], [29 + hb + 0.2, -8], [29 + 8.2, 0], [29 - 8.2, 0]]));
});

// 21. finger-tab-split-joint: the bar is split along a square-wave seam into two halves shown pulled apart.
// Built as the two halves directly (comb polygons), then the long top and bottom outer edges chamfered and the bore cut.
def('finger-tab-split-joint', (b) => {
  const comb = (g) => { const p = []; for (let i = 0; i < 8; i++) { const x0 = -45 + i * 11.25, x1 = x0 + 11.25; if (i % 2 === 0) p.push([x0, 7 - g], [x1, 7 - g]); else p.push([x0, -g], [x1, -g]); } return p; };
  b.prism('z', 0, 16, (sk) => sk.poly([...comb(0), [45, 17], [-45, 17]].map(([x, y]) => [x, y + 7])));
  b.prism('z', 0, 16, (sk) => sk.poly([[-45, -17], [45, -17], ...comb(0.25).reverse(), [-45, 7 - 0.25]].slice(0, -1).map(([x, y]) => [x, y - 7])));
  b.blend('chamfer', 1.4, (e) => Math.abs(Math.abs(e.mid[1]) - 24) < 1e-6 && (Math.abs(e.mid[2]) < 1e-6 || Math.abs(e.mid[2] - 16) < 1e-6) && Math.abs(e.mid[0]) < 1e-6);
  b.cut('x', -46, 46, (sk) => { sk.circle(7, 8, 5); sk.circle(-7, 8, 5); });
});

// 22. v-groove-slider-rail: rail with a V-groove along each side, and an inverted-U carriage riding on it (built as
// walls + top plate + ribs so it never overlaps the rail). The example's two lead-in "chamfer" slabs touch the rail in
// a single corner point only (rail volume stays 23040), so they are not translated.
def('v-groove-slider-rail', (b) => {
  b.prism('z', 0, 16, (sk) => sk.rect(-10, -40, 10, 40));
  for (const s of [1, -1]) b.cut('y', -41, 41, (sk) => sk.poly([[s * 10, 4], [s * 10, 12], [s * 6, 8]]));
  b.prism('z', 0.4, 20.8, (sk) => sk.rect(10.4, 3, 14.4, 37));
  b.prism('z', 0.4, 20.8, (sk) => sk.rect(-14.4, 3, -10.4, 37));
  b.prism('z', 17.3, 20.8, (sk) => sk.rect(-14.4, 3, 14.4, 37));
  for (const s of [1, -1]) b.prism('y', 2, 38, (sk) => sk.poly([[s * 11.4, 4], [s * 11.4, 13.2], [s * 6.8, 8.6]]));
});

// 23. modular-hose-segment: truncated ball end, waist, socket cup with spherical seat and flared mouth, bore, flex slots
def('modular-hose-segment', (b) => {
  const cz = 4.9 * 0.7, rz = Math.sqrt(4.9 * 4.9 - cz * cz);
  b.revZ((sk) => { const c = sk.p(0, cz), p0 = sk.p(rz, 0), top = sk.p(0, cz + 4.9), o = sk.p(0, 0); sk.line(o, p0); sk.arc(c, p0, top); sk.line(top, o); });
  const waistTop = 4.9 * 1.35, sock = waistTop - 1, seatZ = sock + 1.2 + 5.15;
  b.prism('z', 4.4, 4.4 + waistTop + 1.5, (sk) => sk.circle(0, 0, 2.65));
  b.prism('z', sock, 14, (sk) => sk.circle(0, 0, 6.3));
  b.revZ((sk) => { const bot = sk.p(0, seatZ - 5.15), top = sk.p(0, seatZ + 5.15), c = sk.p(0, seatZ); sk.arc(c, bot, top); sk.line(top, bot); }, 'cut');
  b.revZ((sk) => sk.poly([[0, seatZ], [4.3, seatZ], [6.8, seatZ + 5.39], [0, seatZ + 5.39]]), 'cut');
  b.cut('z', -2, 16, (sk) => sk.circle(0, 0, 1.5));
  const slotZ = (seatZ + 14) / 2;
  b.cut('x', -7.56, 7.56, (sk) => sk.circle(0, slotZ, 0.7));
  b.cut('y', -7.56, 7.56, (sk) => sk.circle(0, slotZ, 0.7));
});

// 24. vented-louvre-case: rounded base tray (ridge, 51 honeycomb hexes and 42 round vents in the floor) and a louvred lid
def('vented-louvre-case', (b) => {
  const rr = (w, d, r) => (sk) => rrect(sk, 0, 0, w, d, r);
  b.prism('z', 0, 28, rr(124.8, 84.8, 4));
  b.cut('z', 2, 29, rr(120, 80, 1.6));
  b.prism('z', 28, 33, rr(124.2, 84.2, 3.7));
  b.cut('z', 27.5, 33.5, rr(120, 80, 1.6));
  const R = 3.5 / Math.cos(rad(30)), colP = 1.5 * R + 1.6, rowP = 8.6;
  b.cut('z', -1, 3, (sk) => {
    for (let ci = -6; ci <= 6; ci++) {
      const hx = ci * colP; if (hx < -54.6 || hx > Math.min(54.6, -3)) continue;
      const st = ci % 2 === 0 ? 0 : rowP / 2;
      for (let ri = -6; ri <= 6; ri++) { const hy = ri * rowP + st; if (hy < -34.6 || hy > 34.6) continue; sk.loop(Array.from({ length: 6 }, (_, k) => [hx + R * Math.cos(rad(60 * k)), hy + R * Math.sin(rad(60 * k))])); }
    }
    for (let gx = 1; gx * 9 < 54.6; gx++) for (let gy = -5; gy <= 5; gy++) if (Math.abs(gy * 9) <= 34.6) sk.circle(gx * 9, gy * 9, 2);
  });
  const Y = 96.8;
  b.prism('z', 0, 12, (sk) => rrect(sk, 0, Y, 124.8, 84.8, 4));
  b.cut('z', -1, 10, (sk) => rrect(sk, 0, Y, 120, 80, 1.6));
  b.cut('z', 0, 5.3, (sk) => rrect(sk, 0, Y, 124.2, 84.2, 3.7));
  b.cut('z', 9, 13, (sk) => { for (let i = 0; i < 6; i++) rrect(sk, -37.44 + (74.88 * i) / 5, Y, 5, 67.2, 2.25); });
});

// 25. wall-mount-flanged-junction-box: base tray with ridge and four slotted mounting ears with gusset webs, and a lid
def('wall-mount-flanged-junction-box', (b) => {
  const rr = (w, d, r, y = 0) => (sk) => rrect(sk, 0, y, w, d, r);
  b.prism('z', 0, 23.8, rr(104.8, 74.8, 8));
  b.cut('z', 1.8, 24.8, rr(100, 70, 5.6));
  b.prism('z', 23.8, 26.8, rr(104.3, 74.3, 7.75));
  b.cut('z', 23.3, 27.3, rr(100, 70, 5.6));
  const slots = [];
  for (const s of [1, -1]) for (const x of [-43.4, 43.4]) {
    b.prism('z', 0, 4, (sk) => rrect(sk, x, s * 42.15, 10, 12.5, 3));
    b.prism('x', x - 5, x + 5, (sk) => sk.poly([[s * 35.9, 0], [s * 35.9, 7], [s * 42.9, 4]]));
    slots.push([x, s]);
  }
  b.cut('z', -1, 5, (sk) => { for (const [x, s] of slots) { const ym = s * 43.45; sk.loop([[x, ym - 6], [x + 2, ym - 4], [x + 2, ym + 4], [x, ym + 6], [x - 2, ym + 4], [x - 2, ym - 4]]); } });
  const Y = 90.8;
  b.prism('z', 0, 9.8, rr(104.8, 74.8, 8, Y));
  b.cut('z', -1, 8, rr(100, 70, 5.6, Y));
  b.cut('z', -0.25, 3.25, rr(104.8, 74.8, 7.75, Y));
});

//@@MORE

for (const [id, p] of Object.entries(parts)) writeFileSync(path.join(out, `${id}.ckpart.json`), JSON.stringify(p, null, 1) + '\n');
console.log(`wrote ${Object.keys(parts).length} parts`);
