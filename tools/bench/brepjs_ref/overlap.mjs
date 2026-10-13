// usage: node overlap.mjs <id>   -> volume of the sum of the example's solids vs the volume of their union
import { measureVolume, fuseAll, unwrap, getSolids } from 'brepjs/quick';
const id = process.argv[2];
const mod = await import(`./ex/${id}.mjs`);
let items = Array.isArray(mod.default) ? mod.default : [mod.default];
items = items.map((x) => (x && x.__color !== undefined ? x.shape : x));
const sum = items.reduce((a, s) => a + unwrap(measureVolume(s)), 0);
const u = unwrap(fuseAll(items));
console.log(id, 'sum', sum, 'union', unwrap(measureVolume(u)), 'solids', getSolids(u).length);
process.exit(0);
