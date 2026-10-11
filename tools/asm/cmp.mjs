// Assembly probes: node tools/asm/cmp.mjs ./asmcases.mjs [name..] (V=1 prints diffs)
// cmp.mjs <casesModule> [name...] : oracle vs rebuilt on small parts
import * as L from './lib.mjs';
const mod = await import(process.argv[2]);
const names = process.argv.slice(3);
const B = L.B();
let ok = 0, n = 0;
for (const [name, [f, o]] of Object.entries(mod.cases)) {
  if (names.length && !names.includes(name)) continue;
  n++;
  const msg = L.regen(L.part(f), { debug: true, ...(o ?? {}) });
  const a = L.run(L.A, msg), b = L.run(B, msg);
  if (a.trap || b.trap) { console.log(name, 'TRAP', a.trap, b.trap); continue; }
  const ra = a.o.result ?? a.o, rb = b.o.result ?? b.o;
  const diffs = [];
  for (const k of ['errors', 'components', 'planes', 'sketches', 'curves']) if (JSON.stringify(ra[k]) !== JSON.stringify(rb[k])) diffs.push(k);
  for (const k of ['box', 'faces', 'edges']) if (JSON.stringify(ra.body?.[k]) !== JSON.stringify(rb.body?.[k])) diffs.push('body.' + k);
  const strip = (t) => t.filter(l => !/tessellate/.test(l));
  if (JSON.stringify(strip(a.trace)) !== JSON.stringify(strip(b.trace))) diffs.push('trace');
  if (!diffs.length) { ok++; console.log(name.padEnd(14), 'ok'); continue; }
  console.log(name.padEnd(14), 'DIFF', diffs.join(','));
  if (process.env.V) for (const k of diffs) {
    const g = (r, k) => k === 'trace' ? (r === ra ? a.trace : b.trace) : k.startsWith('body.') ? r.body?.[k.slice(5)] : r[k];
    console.log('  orig', JSON.stringify(g(ra, k)).slice(0, 1500)); console.log('  ours', JSON.stringify(g(rb, k)).slice(0, 1500));
  }
}
console.log(`${ok}/${n} match`);
