// Time a document feature by feature: regen of each solid-feature prefix (the kernel keeps no regen cache, so each is a full build). REPRO_DOC=path.
import { readFileSync } from 'node:fs';
import { Session } from '@fw/session';
import type { PartDoc } from '@fw/part-model';
const doc = JSON.parse(readFileSync(process.env.REPRO_DOC!, 'utf8'));
const fs = doc.features as Array<{ id: string; type: string }>;
let prev = 0;
console.log('features', fs.length);
for (let k = 1; k <= fs.length; k++) {
  const t = fs[k - 1].type;
  if (t === 'sketch' || t === 'plane') continue;   // no geometry of their own
  const d = { ...doc, features: fs.slice(0, k) };
  const t0 = performance.now();
  const s = await Session.open(d as PartDoc);
  const rep = await s.regenReport();
  const ms = performance.now() - t0;
  const errs = rep.errors.map((e) => String((e as unknown as [string, { code: string }])[1]?.code));
  console.log(`${String(k).padStart(2)} ${fs[k - 1].id.padEnd(4)} ${t.padEnd(9)} cumulative ${(ms / 1000).toFixed(1).padStart(6)} s  delta ${((ms - prev) / 1000).toFixed(1).padStart(6)} s ${errs.length ? 'ERR ' + errs.join(',') : ''}`);
  prev = ms;
  await s.close().catch(() => {});
}
