// usage: node measure.mjs <id>   -> writes out/<id>.json and out/<id>.step
import { writeFileSync } from 'node:fs';
import { measureVolumeProps, measureSurfaceProps, getBounds, getSolids, getFaces, isValid, exportSTEP, unwrap } from 'brepjs/quick';
import * as B from 'brepjs/quick';
const id = process.argv[2];
const t0 = Date.now();
const mod = await import(`./ex/${id}.mjs`);
const t1 = Date.now();
let items = Array.isArray(mod.default) ? mod.default : [mod.default];
items = items.map((x) => (x && x.__color !== undefined ? x.shape : x));
const parts = items.map((s) => {
  const vp = unwrap(measureVolumeProps(s));
  const sp = unwrap(measureSurfaceProps(s));
  const b = getBounds(s);
  return { volume: vp.volume, area: sp.area, com: vp.centerOfMass, bbox: [b.xMin, b.yMin, b.zMin, b.xMax, b.yMax, b.zMax], solids: getSolids(s).length, faces: getFaces(s).length, valid: isValid(s) };
});
const V = parts.reduce((a, p) => a + p.volume, 0);
const agg = {
  volume: V,
  area: parts.reduce((a, p) => a + p.area, 0),
  com: [0, 1, 2].map((i) => parts.reduce((a, p) => a + p.volume * p.com[i], 0) / V),
  bbox: [0, 1, 2].map((i) => Math.min(...parts.map((p) => p.bbox[i]))).concat([0, 1, 2].map((i) => Math.max(...parts.map((p) => p.bbox[i + 3])))),
  solids: parts.reduce((a, p) => a + p.solids, 0),
  faces: parts.reduce((a, p) => a + p.faces, 0),
  valid: parts.every((p) => p.valid),
};
let step = null;
try {
  const shape = items.length === 1 ? items[0] : B.compound(items);
  const blob = unwrap(exportSTEP(shape));
  writeFileSync(`out/${id}.step`, Buffer.from(await blob.arrayBuffer()));
  step = `out/${id}.step`;
} catch (e) { step = 'ERR ' + String(e).slice(0, 200); }
writeFileSync(`out/${id}.json`, JSON.stringify({ id, nparts: items.length, ...agg, parts: items.length > 1 ? parts : undefined, buildMs: t1 - t0, step }, null, 1));
console.log(id, 'ok', JSON.stringify({ n: items.length, vol: agg.volume, area: agg.area, valid: agg.valid, solids: agg.solids, ms: t1 - t0, step }));
process.exit(0);
