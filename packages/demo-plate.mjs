// Demo: build the plate part with the compiled @ck-cad/builder and dump it.
import { writeFileSync } from 'node:fs';
import { Part } from './dist/builder/src/index.js';

const p = new Part('Plate with holes', 'aluminium');
const base = p.sketch('Outline', 'front', (s) => s.rect(0, 0, 100, 60));
p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 10, name: 'Plate' });
const holes = p.sketch('Holes', 'front', (s) => {
  for (const [x, y] of [[10, 10], [90, 10], [10, 50], [90, 50]]) s.circle(x, y, 4);
});
p.extrude({ sketch: holes, op: 'cut', end: 'throughBoth', name: 'Drill' });

const doc = p.toJSON();
writeFileSync(new URL('./plate.json', import.meta.url), JSON.stringify(doc, null, 2));
console.log(`features=${doc.features.length} seq=${doc.seq} material=${doc.material}`);
for (const f of doc.features) console.log(`- ${f.id} ${f.type} ${f.name ?? ''}`);
