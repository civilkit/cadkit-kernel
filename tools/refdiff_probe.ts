// Where does a cached refDiff spend its time? Fixed cost (identical documents), last-feature edit, middle edit. REPRO_DOC=path
import { readFileSync } from 'node:fs';
import { Session } from '@fw/session';
import type { PartDoc } from '@fw/part-model';
const doc = JSON.parse(readFileSync(process.env.REPRO_DOC!, 'utf8'));
const s = await Session.open(doc as PartDoc);
// eslint-disable-next-line @typescript-eslint/no-explicit-any
const k = (s as any).kernel;
const lap = async (tag: string, msg: Record<string, unknown>) => {
  const t = performance.now();
  const r = await k.request(msg);
  const res = r.result ?? {};
  console.log(`${tag.padEnd(40)} ${((performance.now() - t) / 1000).toFixed(2)} s  ${res.cacheStats ? `resumedAt ${res.cacheStats.resumedAt}` : ''} checked ${res.checked ?? ''} warnings ${res.warnings?.length ?? ''}`);
};
const edit = (id: string, key: string, v: number) => { const d = JSON.parse(JSON.stringify(doc)); d.features.find((f: { id: string }) => f.id === id)[key] = v; return d; };
const last = edit('F49', 'radius', 0.6), mid = edit('F42', 'angle', 359.9);
for (const d of [last, mid]) await k.request({ type: 'regen', tag: 'probe', part: d });
await lap('identical documents', { type: 'refDiff', base: doc, part: doc, cacheStats: true });
await lap('last feature edited', { type: 'refDiff', base: doc, part: last, cacheStats: true });
await lap('F42 edited', { type: 'refDiff', base: doc, part: mid, cacheStats: true });
await lap('regen last (full hit, reply only)', { type: 'regen', tag: 'probe', part: last });
await lap('regen last noView', { type: 'regen', tag: 'probe', part: last, opts: { noView: true } });
await s.close().catch(() => {});
