// Extract each example's code string from the (untrusted) playground source by text, then strip TS.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { transform } from 'sucrase';
mkdirSync('/private/tmp/claude-501/brepjs-run/ex', { recursive: true });
const src = readFileSync('/private/tmp/claude-501/brepjs-src/apps/playground/src/lib/examples/mechanical.ts', 'utf8');
const re = /\n    id: '([^']+)',\n    label: '((?:[^'\\]|\\.)*)',\n    description:\s*(?:'|")((?:[^'"\\]|\\.|["'])*?)(?:'|"),\n    code: `([\s\S]*?)`,\n  \},/g;
const list = [];
let m;
while ((m = re.exec(src))) {
  const code = m[4].replace(/\\`/g, '`').replace(/\\\$\{/g, '${').replace(/\\\\/g, '\\');
  list.push({ id: m[1], label: m[2], description: m[3], lines: code.split('\n').length });
  const dir = '/private/tmp/claude-501/brepjs-run/ex/';
  writeFileSync(`${dir}${m[1]}.ts`, code);
  let js = transform(code, { transforms: ['typescript'] }).code;
  js = js.replace(/import \{ color \} from 'brepjs\/playground';/, "const color = (shape, c) => ({ __color: c, shape });");
  writeFileSync(`${dir}${m[1]}.mjs`, js);
}
writeFileSync('/private/tmp/claude-501/brepjs-run/examples.json', JSON.stringify(list, null, 1));
console.log(list.length, 'examples');
console.log(list.map((e) => `${e.id} ${e.lines}`).join('\n'));
