import { spawnSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
const list = JSON.parse(readFileSync('/private/tmp/claude-501/brepjs-run/examples.json', 'utf8'));
for (const e of list) {
  const r = spawnSync('node', ['measure.mjs', e.id], { cwd: '/private/tmp/claude-501/brepjs-run', timeout: 240000, encoding: 'utf8' });
  const line = (r.stdout || '').split('\n').find((l) => l.startsWith(e.id + ' ok'));
  console.log(line ?? `${e.id} FAIL ${r.error ? r.error.code : ''} ${(r.stderr || '').split('\n').filter((l) => /rror/.test(l)).slice(0, 2).join(' | ').slice(0, 250)}`);
}
