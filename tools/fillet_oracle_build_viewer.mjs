// Build the self-contained oracle viewer: inline the oracle core (and optionally the kernel layer data) into the template.
//   node tools/fillet_oracle_build_viewer.mjs OUT.html [kernel_layer.json]
import { readFileSync, writeFileSync } from 'node:fs';
const here = (f) => new URL(f, import.meta.url);
const [out, data] = process.argv.slice(2);
const core = readFileSync(here('./fillet_oracle_core.mjs'), 'utf8').replace(/^export /gm, '');
let html = readFileSync(here('./fillet_oracle_viewer.template.html'), 'utf8');
const gpu = readFileSync(here('./fillet_oracle_gpu.js'), 'utf8');
html = html.replace('/*CORE*/', () => core).replace('/*GPU*/', () => gpu).replace('/*KDATA*/{}', () => (data ? readFileSync(data, 'utf8') : '{}'));
writeFileSync(out, html);
console.log(`wrote ${out} (${(html.length / 1024).toFixed(0)} KiB)`);
