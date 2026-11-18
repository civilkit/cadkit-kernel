// Merges the stepped-disc corpus into the blend table fixtures: replaces every `stepped-disc*` base and case in
// crates/ck-model/tests/fixtures/blend_cases.json and every `stepped-disc*` entry of blend_expect.json by the ones of the
// document the kernel test writes (see docs/blend-table.md, "Stepped disc cases"):
//
//   STEP_EMIT=/tmp/corpus.json cargo test -p ck-model --profile fast --test stepped_disc_blend emit_corpus -- --nocapture
//   node tools/merge_stepped_corpus.mjs /tmp/corpus.json [prefix]
//
// The optional prefix (default `stepped-disc`) merges another kernel-written family the same way: `overhang` for the document
// `OH_EMIT=<file> cargo test -p ck-model --test overhang_fillet emit_corpus` writes (docs/blend-table.md, "Overhang cases").
//
// tools/gen_blend_fixtures.mjs keeps the `stepped-disc*` entries of the file it overwrites, so a regeneration does not drop them.
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const dir = path.join(here, '../crates/ck-model/tests/fixtures');
const PREFIX = process.argv[3] ?? 'stepped-disc';
const doc = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const casesPath = path.join(dir, 'blend_cases.json'), expectPath = path.join(dir, 'blend_expect.json');
const cur = JSON.parse(readFileSync(casesPath, 'utf8'));
const bases = Object.fromEntries(Object.entries(cur.bases).filter(([k]) => !k.startsWith(PREFIX)));
Object.assign(bases, doc.bases);
const cases = [...cur.cases.filter((c) => !c.id.startsWith(PREFIX)), ...doc.cases];
const lines = (o) => Object.entries(o).map(([k, v]) => `  ${JSON.stringify(k)}: ${JSON.stringify(v)}`).join(',\n');
writeFileSync(casesPath, `{\n "bases": {\n${lines(bases)}\n },\n "cases": [\n${cases.map((c) => '  ' + JSON.stringify(c)).join(',\n')}\n ]\n}\n`);
const exp = JSON.parse(readFileSync(expectPath, 'utf8'));
for (const k of Object.keys(exp)) if (k.startsWith(PREFIX)) delete exp[k];
Object.assign(exp, doc.expect);
const sorted = Object.keys(exp);
writeFileSync(expectPath, `{\n${sorted.map((k) => ` ${JSON.stringify(k)}: ${JSON.stringify(exp[k])}`).join(',\n')}\n}\n`);
console.log('cases', cases.length, 'expect', sorted.length);
