// usage: node tools/kit_typecheck.mjs [--kit <kit dir>] [--out <scratch dir>]
// Type-checks the KIT against THIS kernel worktree's @ck/* packages (the kit imports ../rebuild/packages/*), so a kernel TS
// change that breaks a kit type (for example a field made required) shows up before the kit's CI does. Needs the kit's own
// node_modules (its tsc and @types). The kernel gate does not run the kit's tsc; run this before sending a batch that
// touches packages/*/src.
import { readFileSync, writeFileSync, mkdirSync, existsSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const args = process.argv.slice(2);
const opt = (f, d) => { const i = args.indexOf(f); return i < 0 ? d : args[i + 1]; };
const here = resolve(dirname(fileURLToPath(import.meta.url)), '..');
// the kit sits beside the kernel checkout, or two levels up when the kernel is a worktree under cadkit/worktrees/
const kit = resolve(opt('--kit', [resolve(here, '../kit'), resolve(here, '../../kit')].find((d) => existsSync(resolve(d, 'tsconfig.json'))) ?? resolve(here, '../kit')));
const out = resolve(opt('--out', process.env.TMPDIR || '/tmp'));
mkdirSync(out, { recursive: true });
const src = readFileSync(resolve(kit, 'tsconfig.json'), 'utf8').replace(/^\s*\/\/.*$/gm, '');
const base = JSON.parse(src).compilerOptions.paths ?? {};
const paths = {};
for (const [k, vs] of Object.entries(base)) {
  paths[k] = vs.map((v) => (v.startsWith('../rebuild/') ? resolve(here, v.slice('../rebuild/'.length)) : resolve(kit, v)));
}
const cfg = resolve(out, 'tsconfig.kit-vs-kernel.json');
writeFileSync(cfg, JSON.stringify({
  extends: resolve(kit, 'tsconfig.json'),
  compilerOptions: { paths, noEmit: true, typeRoots: [resolve(kit, 'node_modules/@types'), resolve(kit, 'node_modules')] },
}, null, 1));
try {
  execFileSync(resolve(kit, 'node_modules/.bin/tsc'), ['-p', cfg], { stdio: 'inherit' });
  console.log(`kit typecheck against ${here}: ok`);
} catch {
  console.log(`kit typecheck against ${here}: FAILED`);
  process.exit(1);
}

// The packages' own build config (tsconfig.build.json) also compiles files the kit never imports (the agent guide text in
// packages/session/src/guide.ts is one big template literal: a stray backtick in it breaks `npm run build` and leaves the
// kernel tests running against a stale dist). A syntax error (TS1xxx) there fails the check; errors that only come from
// packages not installed in this checkout (the MCP sdk, zod) are ignored.
try {
  const r = execFileSync(resolve(kit, 'node_modules/.bin/tsc'), ['-p', resolve(here, 'packages/tsconfig.build.json'), '--noEmit', '--typeRoots', resolve(kit, 'node_modules/@types')], { encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'] });
  if (r) console.log(r);
} catch (e) {
  const out = String(e.stdout || '') + String(e.stderr || '');
  const syntax = out.split('\n').filter((l) => /error TS1\d{3}:/.test(l));
  if (syntax.length) { console.log(syntax.join('\n')); console.log('packages build config: SYNTAX ERRORS'); process.exit(1); }
  console.log('packages build config: no syntax errors (other errors ignored: packages not installed here)');
}
