// Builder for the brepjs "mechanical" translations (gen.mjs). Authoring is in the examples' own Z-up
// coordinates (call them cq); the part is built in our Y-up world via (x,y,z)_cq -> (x,z,-y)_cq, which maps
// cq planes onto ours: XY -> top (normal +Zcq), XZ -> front (normal -Ycq), YZ -> right (normal +Xcq).
import { readFileSync } from 'node:fs';
import { Kernel } from '../kernel.mjs';
import { Sk, STD, FEAT, plane, part } from './lib.mjs';

export const rad = (d) => (d * Math.PI) / 180;
export const W = (x, y, z) => [x, z, -y];
export const unW = ([x, y, z]) => [x, -z, y]; // our world -> cq

let kernel = null;
function regen(features) {
  kernel ??= new Kernel(new WebAssembly.Module(readFileSync(new URL('../../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url))));
  const r = kernel.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'probe', part: part('probe', features) } }));
  return r.trap ? { trap: r.trap } : JSON.parse(r.json).result;
}

export class B {
  constructor(name) { this.name = name; this.f = []; this.n = 0; this.planes = new Map(); }
  nid() { return `F${++this.n}`; }
  // sketch plane normal to cq axis `axis` at coordinate `off` (cq)
  planeFor(axis, off) {
    const base = { x: 'right', y: 'front', z: 'top' }[axis];
    const d = axis === 'y' ? -off : off;
    if (Math.abs(d) < 1e-12) return STD(base);
    const key = `${axis}${d}`;
    if (!this.planes.has(key)) { const id = this.nid(); this.f.push(plane(id, base, d)); this.planes.set(key, FEAT(id)); }
    return this.planes.get(key);
  }
  // sketch on a cq-axis plane, local coords: z-axis (x,y)cq; y-axis (x,z)cq; x-axis (y,z)cq
  sketch(axis, off, fn) { const sk = new Sk(this.nid(), this.planeFor(axis, off)); fn(sk); this.f.push(sk.json()); return sk; }
  // extrude from cq coordinate s0 to s1 along +axis
  prism(axis, s0, s1, fn, op = 'add') {
    const sk = this.sketch(axis, s0, fn);
    const id = this.nid();
    this.f.push({ type: 'extrude', id, op, sketch: sk.id, end: 'blind', depth: s1 - s0, ...((axis === 'y') !== (op === 'cut') ? { flip: true } : {}) });  // a cut runs against the plane normal by default
    return id;
  }
  cut(axis, s0, s1, fn) { return this.prism(axis, s0, s1, fn, 'cut'); }
  // revolve a profile (local coords (r, z) on the cq XZ plane = our front) about the cq Z axis
  revZ(fn, op = 'add', angle = 360, axisX = 0) {
    const sk = new Sk(this.nid(), STD('front')); fn(sk); sk.axis(axisX, 0, axisX, 10); this.f.push(sk.json());
    const id = this.nid();
    this.f.push({ type: 'revolve', id, op, sketch: sk.id, axis: `e${sk.ne}`, angle });
    return id;
  }
  // revolve about an in-plane axis; fn(sk) draws the profile and returns the axis [x0,y0,x1,y1] in sketch coordinates
  rev(axis, off, fn, op = 'add', angle = 360) {
    const sk = new Sk(this.nid(), this.planeFor(axis, off)); const a = fn(sk); sk.axis(...a); this.f.push(sk.json());
    const id = this.nid();
    this.f.push({ type: 'revolve', id, op, sketch: sk.id, axis: `e${sk.ne}`, angle });
    return id;
  }
  // loft through the given [axis, off, fn] sections (>= 2; each is a smooth segment, so use pairs for ruled lofts)
  loft(sections, op = 'add') {
    const ids = sections.map(([axis, off, fn]) => this.sketch(axis, off, fn).id);
    const id = this.nid();
    this.f.push({ type: 'loft', id, op, sketches: ids });
    return id;
  }
  add(f) { this.f.push(f); return f.id; }
  // fillet / chamfer the body edges (in cq frame) accepted by pred({faces, mid(cq), circle})
  blend(kind, dist, pred, o = {}) {
    const res = regen(this.f);
    if (!res?.body) throw new Error(`${this.name}: regen before blend failed ${JSON.stringify(res?.errors ?? res)}`);
    const edges = res.body.edges.filter((e) => pred({ faces: e.faces, mid: unW(e.mid), circle: e.circle }, e)).map((e) => ({ faces: e.faces.slice(0, 2), at: e.mid }));
    if (!edges.length) throw new Error(`${this.name}: blend selected no edges`);
    const id = this.nid();
    this.f.push(kind === 'fillet' ? { type: 'fillet', id, radius: dist, edges, ...o } : { type: 'chamfer', id, chamferType: 'equal', distance: dist, edges, ...o });
    return id;
  }
  json() { return part(this.name, this.f); }
}
export { Sk, STD, FEAT };
// regular helpers for 2D loops
export const circ = (u, v, r) => (sk) => sk.circle(u, v, r);

// rounded rectangle (w x h about cx,cy, corner radius r, optionally turned by ang degrees about cx,cy)
export function rrect(sk, cx, cy, w, h, r, ang = 0) {
  r = Math.min(r, w / 2, h / 2);
  const c = Math.cos(rad(ang)), s = Math.sin(rad(ang));
  const T = (x, y) => [cx + x * c - y * s, cy + x * s + y * c];
  const x1 = w / 2, y1 = h / 2;
  if (r < 1e-9) { sk.loop([T(-x1, -y1), T(x1, -y1), T(x1, y1), T(-x1, y1)]); return; }
  const P = (x, y) => sk.P(...T(x, y));
  const cs = [[x1 - r, -y1 + r], [x1 - r, y1 - r], [-x1 + r, y1 - r], [-x1 + r, -y1 + r]];
  const st = [[x1 - r, -y1], [x1, y1 - r], [-x1 + r, y1], [-x1, -y1 + r]];
  const en = [[x1, -y1 + r], [x1 - r, y1], [-x1, y1 - r], [-x1 + r, -y1]];
  for (let i = 0; i < 4; i++) {
    const cid = sk.p(...T(...cs[i]));
    sk.arc(cid, P(...st[i]), P(...en[i]));
    const nx = (i + 1) % 4;
    if (Math.hypot(en[i][0] - st[nx][0], en[i][1] - st[nx][1]) > 1e-9) sk.line(P(...en[i]), P(...st[nx]));
  }
}
