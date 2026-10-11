// Compare a kernel massProps/box reply with an oracle entry.
//   checkOracle(oracle, { volume, area, com }, box) -> array of mismatch strings ([] = agrees)
// Volume/area/com: |ours - oracle| <= 1e-6 * (1 + |oracle|) (mass props are exact B-rep integrals).
// Box: exact parts 1e-6; curved parts (oracle.bboxExact === false) the kernel box is
// mesh-sampled, so it must lie inside the true box (+1e-6) and be within 2e-3 of the
// largest extent of it.
const TOL = 1e-6, BOX_SLACK = 2e-3;
const near = (a, b, tol = TOL) => typeof a === 'number' && Math.abs(a - b) <= tol * (1 + Math.abs(b));
export function checkOracle(or, mp, box) {
  const bad = [];
  if (!near(mp?.volume, or.volume)) bad.push(`volume ${mp?.volume} vs ${or.volume}`);
  if (or.area != null && !near(mp?.area, or.area)) bad.push(`area ${mp?.area} vs ${or.area}`);
  or.com.forEach((c, i) => { if (!near(mp?.com?.[i], c)) bad.push(`com[${i}] ${mp?.com?.[i]} vs ${c}`); });
  if (!box?.min || !box?.max) bad.push('no box');
  else {
    const ext = Math.max(...or.bbox.max.map((x, i) => x - or.bbox.min[i]));
    for (let i = 0; i < 3; i++) {
      const lo = box.min[i] - or.bbox.min[i], hi = or.bbox.max[i] - box.max[i];
      if (or.bboxExact) {
        if (Math.abs(lo) > TOL * (1 + Math.abs(or.bbox.min[i])) || Math.abs(hi) > TOL * (1 + Math.abs(or.bbox.max[i]))) bad.push(`box axis ${i} [${box.min[i]},${box.max[i]}] vs [${or.bbox.min[i]},${or.bbox.max[i]}]`);
      } else if (lo < -TOL || hi < -TOL || lo > BOX_SLACK * ext || hi > BOX_SLACK * ext) bad.push(`box(curved) axis ${i} [${box.min[i]},${box.max[i]}] vs [${or.bbox.min[i]},${or.bbox.max[i]}]`);
    }
  }
  return bad;
}
