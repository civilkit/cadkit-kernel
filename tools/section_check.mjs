// section / sections / probe through the wasm (docs/agent-queries.md "section", "sections", "probe").
// usage: node tools/section_check.mjs [--wasm path] [--quick]     exit 1 on any mismatch
// Primitives with hand-computed areas, then the large sample part: sections-vs-massProps volume within 1e-3, finite
// closed section loops at several planes, probe consistency (centre of mass region, far point). --quick skips the
// large sample (a regen of minutes).
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const wi = args.indexOf('--wasm');
const wasm = wi >= 0 ? args[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const quick = args.includes('--quick');
const mod = new WebAssembly.Module(readFileSync(wasm));
const now = () => Date.now();

let bad = 0;
const ok = (cond, name, shown = '') => { if (!cond) bad++; console.log(`${cond ? 'ok  ' : 'FAIL'} ${name}${shown ? ': ' + shown : ''}`); };
const near = (a, b, rel) => Math.abs(a - b) <= rel * Math.max(Math.abs(b), 1e-12);

const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2) => ({ type: 'line', id, p1, p2 });
const sketch = (plane, points, entities) => ({ type: 'sketch', id: 'F1', plane: { kind: 'std', id: plane }, points, entities, constraints: [] });
const extrude = (depth) => ({ type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth });
const part = (features) => ({ format: 'ck-part', version: 1, features });
const rect = (x0, y0, x1, y1) => {
  const P = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([x, y], i) => pt(`p${i + 1}`, x, y));
  return [P, [0, 1, 2, 3].map((i) => line(`e${i + 1}`, `p${i + 1}`, `p${((i + 1) % 4) + 1}`))];
};

function open(p) {
  const k = new Kernel(mod, { now });
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: p } })).json);
  if (!r.ok || (r.result?.errors ?? []).length) throw new Error(`regen: ${JSON.stringify(r.error ?? r.result.errors)}`);
  const q = (msg) => { const t0 = now(); const j = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { tag: 'main', ...msg } })).json); j.wall = now() - t0; return j; };
  return q;
}

// --- primitives -------------------------------------------------------------------------------------------------
{
  const [P, E] = rect(0, 0, 40, 30);
  const q = open(part([sketch('top', P, E), extrude(20)]));
  const s = q({ type: 'section', plane: { origin: [20, 5, 0], normal: [1, 0, 0] } }).result;
  ok(near(s.area, 600, 1e-9) && s.loops.length === 1 && s.loops[0].kind === 'outer', 'box 40x30x20 mid-plane section area 600', s.area.toFixed(6));
  const m = q({ type: 'massProps' }).result;
  const st = q({ type: 'sections', axis: 'x', n: 41 }).result;
  ok(st.volumeError < 1e-3 && near(st.exactVolume, m.volume, 1e-12), 'box sections n=41 volume', `err ${st.volumeError.toExponential(2)}`);
  const pr = q({ type: 'probe', points: [[20, 10, -15], [200, 0, 0]], rays: [{ origin: [-5, 10, -15], dir: [1, 0, 0] }] }).result;
  ok(pr.points[0].inside === true && near(pr.points[0].distance, -10, 1e-9) && pr.points[1].inside === false, 'box probe centre / far point', JSON.stringify(pr.points.map((p) => p.distance)));
  ok(pr.rays[0].hits.length === 2 && pr.rays[0].hits[0].entering === true && near(pr.rays[0].hits[0].t, 5, 1e-9), 'box ray hits');
}
{
  const q = open(part([sketch('top', [pt('c', 0, 0)], [{ type: 'circle', id: 'e1', c: 'c', r: 10 }]), extrude(20)]));
  const s = q({ type: 'section', plane: { origin: [0, 10, 0], normal: [0, 1, 0] } }).result;
  ok(near(s.area, Math.PI * 100, 1e-9) && s.loops[0].edges[0].type === 'arc', 'cylinder r10 section area 314.159 (exact arc)', s.area.toFixed(6));
  const st = q({ type: 'sections', axis: 'y', n: 41 }).result;
  ok(st.volumeError < 1e-3, 'cylinder sections n=41 volume', `err ${st.volumeError.toExponential(2)}`);
}
{
  const ax = [pt('a1', 0, 0), pt('a2', 0, 10)];
  const axe = { type: 'line', id: 'ax', p1: 'a1', p2: 'a2', construction: true };
  const q = open(part([sketch('front', [pt('c', 0, 0), pt('s', 0, -10), pt('n', 0, 10), ...ax],
    [{ type: 'arc', id: 'e1', c: 'c', p1: 's', p2: 'n' }, line('e2', 'n', 's'), axe]),
  { type: 'revolve', id: 'F2', op: 'add', sketch: 'F1', axis: 'ax', angle: 360 }]));
  const s = q({ type: 'section', plane: { origin: [0, 0, 0], normal: [0, 1, 0] } }).result;
  ok(near(s.area, Math.PI * 100, 1e-6), 'sphere r10 equator section area 314.159', s.area.toFixed(6));
  const t = q({ type: 'section', plane: { origin: [0, 10, 0], normal: [0, 1, 0] } }).result;
  ok(Math.abs(t.area) < 1e-3, 'sphere tangent plane: no area', String(t.area));
  const st = q({ type: 'sections', axis: 'y', n: 41 }).result;
  ok(st.volumeError < 1e-3, 'sphere sections n=41 volume', `err ${st.volumeError.toExponential(2)}`);
}

// --- large sample -----------------------------------------------------------------------------------------------------
if (!quick) {
  const hd = JSON.parse(readFileSync(path.join(here, 'cases/largeSampleFull.ckpart'), 'utf8'));
  const t0 = now();
  const q = open(hd);
  console.log(`large sample regen ${((now() - t0) / 1000).toFixed(1)}s`);
  const m = q({ type: 'massProps' }).result;
  const bb = q({ type: 'sections', axis: 'x', n: 1 }).result.range;
  // the large sample area profile has steps (head / handle junction), so the midpoint rule only converges at ~800 sections
  for (const axis of ['x', 'y', 'z']) {
    const coarse = q({ type: 'sections', axis, n: 21 });
    ok(coarse.result.areas.every(Number.isFinite) && coarse.result.volumeError < 0.1, `large sample sections ${axis} n=21 (coarse)`, `err ${coarse.result.volumeError.toExponential(2)}, ${coarse.wall} ms`);
    const fine = q({ type: 'sections', axis, n: 801 });
    const r = fine.result;
    ok(r.volumeError < 1e-3 && near(r.exactVolume, m.volume, 1e-12), `large sample sections ${axis} n=801 vs massProps`, `err ${r.volumeError.toExponential(2)}, ${fine.wall} ms (incl. massProps)`);
  }
  for (const [name, plane] of [
    ['x mid', { origin: [(bb[0] + bb[1]) / 2, 0, 0], normal: [1, 0, 0] }],
    ['y mid', { origin: [0, 10, 0], normal: [0, 1, 0] }],
    ['z mid', { origin: [0, 0, 5], normal: [0, 0, 1] }],
    ['oblique', { origin: [0, 10, 5], normal: [0.3, 0.8, 0.52] }],
  ]) {
    const j = q({ type: 'section', plane });
    const r = j.result;
    const closed = r.loops.every((l) => l.closed && l.pts.length >= 3 && l.edges.length >= 1);
    ok(Number.isFinite(r.area) && closed && !r.openChains, `large sample section ${name}`, `${r.loops.length} loops, area ${r.area.toFixed(3)}, ${j.wall} ms`);
  }
  const pts = [[0, 0, 0], [1e4, 0, 0], [m.com[0], m.com[1], m.com[2]]];
  const pj = q({ type: 'probe', points: pts });
  ok(pj.ok && pj.result.points[1].inside === false, 'large sample probe far point outside', `${pj.wall} ms`);
}
console.log(bad ? `FAIL (${bad})` : 'section_check ok');
process.exit(bad ? 1 : 0);
