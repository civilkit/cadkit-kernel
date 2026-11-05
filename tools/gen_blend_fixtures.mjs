// Writes crates/ck-model/tests/fixtures/blend_cases.json: the synthetic fillet regression cases of
// crates/ck-model/tests/blend_table.rs (base features + the fillet feature with edge picks resolved + an optional
// reference volume per case).
//
//   node tools/gen_blend_fixtures.mjs --wasm kernel.wasm [--specs specs.json] [--reference ref.jsonl] [--out file]
//
// `--specs` is a comparison spec file {bases:{id:[features]}, runs:[{run, base, radius, picks:[{at, r?}], pre?}]}; without
// it only the bases built here (daisy petals) are used. `--reference` is JSON lines {run, ok, vol, vol0?, source?} of a
// reference engine; only its numbers (and the neutral source label) are embedded, under the case id.
// The picks of a run are 3D points; they are resolved to {faces, at} against the base body of this kernel, so the
// fixture file does not depend on how the picks were chosen.
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const mod = new WebAssembly.Module(readFileSync(opt('--wasm')));
const out = opt('--out', path.join(here, '../crates/ck-model/tests/fixtures/blend_cases.json'));
const specs = opt('--specs') ? JSON.parse(readFileSync(opt('--specs'), 'utf8')) : { bases: {}, runs: [] };
const refs = new Map();
if (opt('--reference')) for (const l of readFileSync(opt('--reference'), 'utf8').split('\n')) if (l.trim()) { const j = JSON.parse(l); refs.set(j.run, j); }

const req = (k, msg) => { const r = k.request(JSON.stringify({ id: 1, msg })); return r.json ? JSON.parse(r.json) : { result: null, trap: r.trap }; };
const regen = (k, features) => req(k, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features } });
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function dist(p, e) {
  const pts = e.pts?.length >= 2 ? e.pts : [e.mid, e.mid];
  let d = Math.hypot(...sub(p, e.mid));
  for (let i = 0; i + 1 < pts.length; i++) {
    const ab = sub(pts[i + 1], pts[i]), ap = sub(p, pts[i]), t = Math.max(0, Math.min(1, dot(ap, ab) / (dot(ab, ab) || 1)));
    d = Math.min(d, Math.hypot(...sub(p, pts[i].map((v, j) => v + ab[j] * t))));
  }
  return d;
}
const findEdge = (body, at) => { let best = null; for (const e of body.edges) { const d = dist(at, e); if (!best || d < best.d) best = { d, e }; } return best && best.d < 1e-3 ? best.e : null; };
const filletFeature = (id, body, picks, radius) => ({
  type: 'fillet', id, radius: Math.max(radius, ...picks.map((p) => p.r ?? 0)),
  edges: picks.map((p) => { const e = findEdge(body, p.at); if (!e) throw new Error('no edge near ' + p.at); const r = { faces: [...e.faces], at: p.at.map((v) => +v.toFixed(6)) }; if (p.r) r.radius = p.r; return r; }),
});

// ---- neutral case ids
const NAMES = [
  [/^A_daisyA1_N(\d+)_(V|T|VT)_r(.+)$/, (m) => `daisy-hub-n${m[1]}-${m[2]}-r${m[3]}`, 'daisy-hub'],
  [/^A_daisyA2_N(\d+)_(V|T|VT)_r(.+)$/, (m) => `daisy-ring-n${m[1]}-${m[2]}-r${m[3]}`, 'daisy-ring'],
  [/^X_daisyA1_N(\d+)_RW_r(.+)$/, (m) => `daisy-hub-n${m[1]}-rim-then-wall-r${m[2]}`, 'daisy-hub'],
  [/^X_daisyA2_N(\d+)_RW_r(.+)$/, (m) => `daisy-ring-n${m[1]}-rim-then-wall-r${m[2]}`, 'daisy-ring'],
  [/^X_lbr_RW_r(.+)$/, (m) => `l-bracket-rim-then-inner-r${m[1]}`, 'l-bracket'],
  [/^BH_pet144_(.+)_r(.+)$/, (m) => `feet-sector-${m[1]}-r${m[2]}`, 'feet-sector'],
  [/^C1_box_convex3_r(.+)$/, (m) => `box-corner-r${m[1]}`, 'box-corner'],
  [/^C2_box_convex3_unequal_(.+)$/, (m) => `box-corner-unequal-${m[1]}`, 'box-corner'],
  [/^C3a_lbracket_inner_r(.+)$/, (m) => `l-bracket-inner-r${m[1]}`, 'l-bracket'],
  [/^C3b_pocket_concave3_unequal_(.+)$/, (m) => `pocket-corner-unequal-${m[1]}`, 'pocket-corner'],
  [/^C3b_pocket_concave3_r(.+)$/, (m) => `pocket-corner-r${m[1]}`, 'pocket-corner'],
  [/^C4_cylcorner_r(.+)$/, (m) => `hole-corner-r${m[1]}`, 'hole-corner'],
  [/^E1_plate_edge_r(.+)$/, (m) => `plate-edge-r${m[1]}`, 'plate'],
  [/^E2_plate_ring_r(.+)$/, (m) => `plate-ring-r${m[1]}`, 'plate'],
  [/^D1_conepocket_rim_(oneside|all)_r(.+)$/, (m) => `cone-pocket-rim-${m[1]}-r${m[2]}`, 'cone-pocket'],
  [/^D2_pipe_tee_r(.+)$/, (m) => `pipe-tee-r${m[1]}`, 'pipe-tee'],
  [/^R_ring10_N(\d+)_(V|T|VT|VTB)_r(.+)$/, (m) => `daisy-ring-n${m[1]}-${m[2]}-r${m[3]}-ref`, 'daisy-ring'],
  [/^H_ring10_N(\d+)_hubrim_r(.+)$/, (m) => `daisy-ring-n${m[1]}-hubrim-r${m[2]}`, 'daisy-ring'],
  [/^S_ring10_N(\d+)_(.+)_r(.+)$/, (m) => `daisy-ring-n${m[1]}-${m[2]}-r${m[3]}`, 'daisy-ring'],
];
const BASE_NAMES = { ring10_5: 'daisy-ring10-5', box: 'box', lbr: 'l-bracket', pocket: 'pocket', cylcorner: 'hole-corner', plate: 'plate', conepocket: 'cone-pocket', pipe: 'pipe-tee', petH: 'feet-sector' };
for (const v of ['A1', 'A2']) for (const n of [5, 6, 7, 8]) BASE_NAMES[`daisy${v}_${n}`] = `daisy-${v === 'A1' ? 'hub' : 'ring'}-${n}`;

// synthetic sequence: the top rim rounded first (r 0.5), then the concave vertical edges next to the hub
for (const v of ['A1', 'A2']) for (const n of [5, 6, 8]) {
  const V = specs.runs.find((r) => r.run === `A_daisy${v}_N${n}_V_r0.5`), T = specs.runs.find((r) => r.run === `A_daisy${v}_N${n}_T_r0.5`);
  if (!V || !T) continue;
  for (const r of [0.5, 0.3, 0.2, 0.1]) specs.runs.push({ run: `X_daisy${v}_N${n}_RW_r${r}`, base: V.base, radius: r, picks: V.picks, pre: { radius: 0.5, picks: T.picks } });
}
// synthetic sequence on the bracket: the whole top outline rounded first (r 1), then the concave vertical edge
{
  const V = specs.runs.find((r) => r.run === 'C3a_lbracket_inner_r1');
  const rim = [[20, 10, 0], [40, 10, -5], [25, 10, -10], [10, 10, -20], [5, 10, -30], [0, 10, -15]].map((at) => ({ at }));
  if (V) for (const r of [1, 2, 3]) specs.runs.push({ run: `X_lbr_RW_r${r}`, base: V.base, radius: r, picks: [V.picks[2]], pre: { radius: 1, picks: rim }, altOrder: { cap: 'F2.C1', rim: 1 } });
}
// petal ring with a third-party reference: one outline of N petals (radial flank lines from a hub arc of radius Rc to
// the tangent points at distance tm, tip arc radius rho tangent to both), H high; the first petal's flank is on the
// sketch y axis. V = the 2N concave vertical edges at the hub, T = every top outer edge, VTB = V + T + the three bottom
// outer edges (two flanks and the tip arc) of the petal whose tip faces world direction (cos 142.4, 0, sin 142.4) deg.
function petalRing(N, { Rc, tm, rho, H }) {
  const alpha = Math.atan2(rho, tm), d = Math.hypot(tm, rho);
  const e = (a) => [Math.cos(a), Math.sin(a)], segs = [], th0 = Math.PI / 2 - alpha;
  const pts = [], ents = [], pid = new Map(), key = (p) => p.map((v) => v.toFixed(7)).join(',');
  const P = (p) => { const k = key(p); if (!pid.has(k)) { pid.set(k, 'p' + (pid.size + 1)); pts.push({ id: pid.get(k), x: p[0], y: p[1] }); } return pid.get(k); };
  for (let i = 0; i < N; i++) {
    const th = th0 + 2 * Math.PI * i / N, thn = th + 2 * Math.PI / N, c = [d * Math.cos(th), d * Math.sin(th)];
    const at = (a, r) => e(a).map((v) => v * r);
    segs.push({ l: [at(th - alpha, Rc), at(th - alpha, tm)] }, { a: [c, at(th - alpha, tm), at(th + alpha, tm)] }, { l: [at(th + alpha, tm), at(th + alpha, Rc)] }, { a: [[0, 0], at(th + alpha, Rc), at(thn - alpha, Rc)] });
  }
  let n = 0;
  for (const s of segs) {
    if (s.l) ents.push({ type: 'line', id: 'e' + ++n, p1: P(s.l[0]), p2: P(s.l[1]) });
    else { const [c, a, b] = s.a; const cid = 'c' + n; pts.push({ id: cid, x: c[0], y: c[1] }); ents.push({ type: 'arc', id: 'e' + ++n, c: cid, p1: P(a), p2: P(b) }); }
  }
  return [{ type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: pts, entities: ents, constraints: [] }, { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: H }];
}
{
  const P = { Rc: 10, tm: 30, rho: 8.83044266165705, H: 10 }, feats = petalRing(5, P);
  const body = regen(new Kernel(mod), feats).result?.body;
  if (body) {
    specs.bases.ring10_5 = feats;
    const near = (a, c, t = 1e-5) => Math.abs(a - c) < t, rad = (p) => Math.hypot(p[0], p[2]), ang = (p) => Math.atan2(p[2], p[0]);
    const flatY = (e, y) => e.pts.every((p) => near(p[1], y));
    const V = body.edges.filter((e) => e.pts.length >= 2 && e.pts.every((p) => near(p[0], e.pts[0][0]) && near(p[2], e.pts[0][2])) && near(rad(e.pts[0]), P.Rc, 1e-4));
    const T = body.edges.filter((e) => flatY(e, P.H));
    const bot = body.edges.filter((e) => flatY(e, 0) && rad(e.mid) > P.Rc + 0.5), w = 142.4 * Math.PI / 180, alpha = Math.atan2(P.rho, P.tm);
    const dang = (a, b) => Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
    const tip = bot.filter((e) => rad(e.mid) > P.tm).sort((p, q) => dang(ang(p.mid), w) - dang(ang(q.mid), w))[0];
    const B = [tip, ...bot.filter((e) => rad(e.mid) < P.tm && [-alpha, alpha].some((x) => dang(ang(e.mid), ang(tip.mid) + x) < 1e-3))];
    const picks = (es) => es.map((e) => ({ at: e.mid }));
    for (const [sn, es] of [['V', V], ['T', T], ['VT', [...V, ...T]], ['VTB', [...V, ...T, ...B]]]) specs.runs.push({ run: `R_ring10_N5_${sn}_r0.8`, base: 'ring10_5', radius: 0.8, picks: picks(es) });
    // the rim edge of one hub arc only: at both ends it runs into the wall of a petal whose rim is not rounded, so the
    // fillet ends flush on the (radial) flank plane. Reference: the rim section (r^2 (1 - pi/4), centroid r (10 - 3 pi) /
    // (3 (4 - pi)) in from the edge) swept over the arc's angle about the axis (Pappus), nothing more removed.
    const hub = T.filter((e) => e.pts.length > 2 && e.pts.every((p) => near(rad(p), P.Rc, 1e-4)))[0];
    const k0 = new Kernel(mod);
    regen(k0, feats);
    const vol0 = req(k0, { type: 'massProps', tag: 'main' }).result?.volume;
    if (hub && vol0) {
      const span = 2 * Math.PI / 5 - 2 * alpha;
      for (const r of [0.8, 1]) {
        const A = r * r * (1 - Math.PI / 4), cen = r * (10 - 3 * Math.PI) / (3 * (4 - Math.PI));
        specs.runs.push({ run: `H_ring10_N5_hubrim_r${r}`, base: 'ring10_5', radius: r, picks: picks([hub]), ref: { run: '', ok: true, vol: vol0 - A * (P.Rc - cen) * span, vol0, source: 'analytic' } });
      }
    }
    // partial selections round one petal: its rim is the tangent chain e1 (flank), e2 (tip arc), e3 (flank); the hub
    // arcs next to it are e20 and e4. `sharp`: the table also checks that no selected edge is left sharp.
    // References: `rim` / `flank1` / `flank1tip` analytic (each flank's rim section swept from the tangent point to
    // where it meets the hub cylinder, the tip arc's by Pappus; the tools meet flush at the tangent points); `rimhub`
    // from the reference engine (`--reference`; the reversed pick order gets the same shape's volume).
    const top = (i) => body.edges.find((e) => e.faces.includes('F2.C1') && e.faces.includes(`F2.S.e${i}`));
    const rimRef = (r, nFlank, tip) => {
      const N = 20000, dw = r / N, span2 = Math.PI + 2 * alpha;
      let fl = 0;
      for (let i = 0; i < N; i++) {
        const w = (i + 0.5) * dw, h = r - Math.sqrt(Math.max(0, r * r - (r - w) ** 2));
        fl += h * (P.tm - Math.sqrt(P.Rc * P.Rc - w * w)) * dw;
      }
      const A = r * r * (1 - Math.PI / 4), cen = r * (10 - 3 * Math.PI) / (3 * (4 - Math.PI));
      return vol0 - nFlank * fl - (tip ? A * (P.rho - cen) * span2 : 0);
    };
    const rim = [top(1), top(2), top(3)], hubs = [top(20), top(4)];
    const sets = [
      ['petal1-rim', rim, [0.8, 1], (r) => rimRef(r, 2, true)],
      ['petal1-rimhub', [...rim, ...hubs], [0.8], null],
      ['petal1-rimhub-rev', [...rim, ...hubs].reverse(), [0.8, 1], null],
      ['flank1', [top(1)], [0.8, 1], (r) => rimRef(r, 1, false)],
      ['flank1tip', [top(1), top(2)], [0.8], (r) => rimRef(r, 1, true)],
    ];
    if (vol0 && rim.every(Boolean) && hubs.every(Boolean)) {
      for (const [sn, es, rs, ref] of sets) {
        for (const r of rs) specs.runs.push({ run: `S_ring10_N5_${sn}_r${r}`, base: 'ring10_5', radius: r, picks: picks(es), sharp: true, ...(ref ? { ref: { run: '', ok: true, vol: ref(r), vol0, source: 'analytic' } } : {}) });
      }
    }
  }
}
const cases = [], bases = {};
for (const run of specs.runs) {
  const hit = NAMES.map(([re, f, fam]) => [re.exec(run.run), f, fam]).find(([m]) => m);
  if (!hit || !BASE_NAMES[run.base]) continue;
  const [m, f, family] = hit;
  const baseId = BASE_NAMES[run.base];
  bases[baseId] ??= specs.bases[run.base];
  let feats = [...bases[baseId]];
  let body = regen(new Kernel(mod), feats).result?.body;
  if (!body) { console.log('skip', run.run, ': the base gives no body'); continue; }
  const pre = run.pre ? filletFeature('F80', body, run.pre.picks, run.pre.radius) : null;
  const k = new Kernel(mod);
  if (pre) {
    feats.push(pre);
    body = regen(k, feats).result?.body;
    if (!body) { console.log('skip', run.run, ': the first fillet gives no body'); continue; }
  }
  const fillet = filletFeature('F90', body, run.picks, run.radius);
  let ref = refs.get(run.run) ?? run.ref;
  let reference = ref ? { ok: !!ref.ok, volume: ref.vol ?? null, base_volume: ref.vol0 ?? null, ...(ref.source ? { source: ref.source } : {}) } : null;
  if (!ref && run.altOrder) {
    // the same shape built the other way round: the corner edge first, then the whole top outline as one tangent chain
    const kk = new Kernel(mod), pure = [...bases[baseId]];
    regen(kk, pure);
    const v0 = req(kk, { type: 'massProps', tag: 'main' }).result?.volume;
    const f1 = [...pure, { ...fillet, id: 'F85' }];
    const b1 = regen(new Kernel(mod), f1).result?.body;
    const top = b1?.faces.find((x) => x.id === run.altOrder.cap);
    if (b1 && top) {
      const rimPicks = b1.edges.filter((e) => e.faces.includes(top.id)).map((e) => ({ at: e.mid }));
      const k2 = new Kernel(mod);
      const r2 = regen(k2, [...f1, filletFeature('F86', b1, rimPicks, run.altOrder.rim)]);
      if (!r2.result?.errors?.length) reference = { ok: true, volume: req(k2, { type: 'massProps', tag: 'main' }).result?.volume ?? null, base_volume: v0 ?? null, source: 'alternate order' };
    }
  }
  cases.push({ id: f(m), family, base: baseId, ...(pre ? { pre } : {}), fillet, reference, ...(run.sharp ? { sharp: true } : {}) });
}
const lines = (o) => Object.entries(o).map(([k, v]) => `  ${JSON.stringify(k)}: ${JSON.stringify(v)}`).join(',\n');
writeFileSync(out, `{\n "bases": {\n${lines(bases)}\n },\n "cases": [\n${cases.map((c) => '  ' + JSON.stringify(c)).join(',\n')}\n ]\n}\n`);
console.log('bases', Object.keys(bases).length, 'cases', cases.length, '->', out);
