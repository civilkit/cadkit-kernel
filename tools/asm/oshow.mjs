// oshow.mjs <cases> [name..]: oracle errors / faces / hole spans / massProps volume
import * as L from './lib.mjs';
const m = await import(process.argv[2]); const names = process.argv.slice(3);
for (const [n, [f, o]] of Object.entries(m.cases)) {
  if (names.length && !names.includes(n)) continue;
  const { o: r0, trace, ex } = L.run(L.A, L.regen(L.part(f), { debug: true, ...(o ?? {}) }), [{ type: 'massProps', tag: 'main' }]);
  const r = r0.result;
  console.log('==', n, JSON.stringify(r.errors), (r.body?.faces ?? []).length, 'faces', 'vol', ex[0]?.result?.volume);
  console.log('  ', (r.body?.faces ?? []).map((f) => f.id + ':' + f.kind).join(' '));
  console.log('  ', trace.filter((l) => /hole|boolean/.test(l)).map((s) => s.trim()).join(' | '));
  if (process.env.E) for (const e of r.body.edges) if (process.env.E === '1' || e.key.includes(process.env.E)) console.log('   ', JSON.stringify(e));
}
