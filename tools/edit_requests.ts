// Per kernel request type counts and seconds for Session.edit on a live session (one kernel instance, as in the GUI).
// Run from the kit repo (vite-node harness): CK_REBUILD=<this worktree> CK_WASM=<wasm> REPRO_DOC=<doc.ckpart> \
//   REPRO_TS=<this worktree>/tools/edit_requests.ts node tools/repro_union_corner.mjs
// FULL_ONLY=1 runs the non-lean edits only (steady state: the before-mass is the previous after-mass).
// EDITS=last,f48,f42,early selects the cases (default last,f48,f42); LEAN_ONLY=1 skips the non-lean variants.
import { readFileSync } from 'node:fs';
import { Session } from '@fw/session';
import type { PartDoc } from '@fw/part-model';
const doc = JSON.parse(readFileSync(process.env.REPRO_DOC!, 'utf8'));
const s = await Session.open(doc as PartDoc);
// eslint-disable-next-line @typescript-eslint/no-explicit-any
const k = (s as any).kernel;
type Row = { type: string; n: number; s: number };
let rows = new Map<string, Row>();
const orig = k.request.bind(k);
k.request = async (m: { type: string; tag?: string }) => {
  const t = performance.now();
  try { return await orig(m); } finally {
    const key = `${m.type}${m.tag ? ` [${m.tag.replace(/^sess\d+/, 'sess')}]` : ''}`;
    const r = rows.get(key) ?? { type: key, n: 0, s: 0 };
    r.n++; r.s += (performance.now() - t) / 1000; rows.set(key, r);
  }
};
const dump = (title: string, wall: number, later?: number) => {
  console.log(`\n== ${title}: edit() returned after ${wall.toFixed(2)} s${later !== undefined ? `, deferred refDiff done ${later.toFixed(2)} s later` : ''}`);
  for (const r of rows.values()) console.log(`   ${r.type.padEnd(28)} x${r.n}  ${r.s.toFixed(2)} s`);
  rows = new Map();
};
const cases: Record<string, [string, string, string, number]> = {
  last: ['F49', 'radius', '', 0.6], f48: ['F48', 'radius', '', 1.9], f42: ['F42', 'angle', '', 359.9], early: ['F5', 'diameter', '', 3.2],
};
await s.regenReport(); rows = new Map();
let bump = 0;
for (const name of (process.env.EDITS ?? 'last,f48,f42').split(',')) {
  const [id, path, , v] = cases[name];
  for (const lean of process.env.LEAN_ONLY ? [true] : process.env.FULL_ONLY ? [false] : [true, false]) {
    bump += 0.01;
    const t0 = performance.now();
    const r = await s.edit([{ op: 'set', id, path, value: v + bump }], undefined, { lean });
    const wall = (performance.now() - t0) / 1000;
    const later = r.refWarningsLater ? await r.refWarningsLater.then(() => (performance.now() - t0) / 1000 - wall) : undefined;
    dump(`${name} ${lean ? 'lean' : 'full'} ok=${r.ok}${lean && !r.refWarningsLater ? ' (reference check skipped)' : ''}`, wall, later);
  }
}
await s.close().catch(() => {});
