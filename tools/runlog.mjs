import { readFileSync } from 'node:fs';
const mod = await WebAssembly.compile(readFileSync(process.argv[2]));
let inst; const logs=[];
const env = { ck_now: () => 0, ck_unix_ms: () => 0, ck_panic: (p, l) => console.log('PANIC', new TextDecoder().decode(new Uint8Array(inst.memory.buffer, p, l))),
  ck_log: (k,a,b,c,d) => logs.push([k,a,b,c,d]) };
inst = new WebAssembly.Instance(mod, { env }).exports;
const part = JSON.parse(readFileSync(process.argv[3],'utf8'));
const body = new TextEncoder().encode(JSON.stringify({id:1,msg:{type:'regen',tag:'m',part}}));
const p = inst.ck_alloc(body.length); new Uint8Array(inst.memory.buffer,p,body.length).set(body);
inst.ck_request(p, body.length);
for (const l of logs) console.log(l.join(' '));
