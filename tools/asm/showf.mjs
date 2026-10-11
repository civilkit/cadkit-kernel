import * as L from './lib.mjs';
const mod = await import(process.argv[2]);
const [f, o] = mod.cases[process.argv[3]];
const msg = L.regen(L.part(f), o ?? {});
const a = L.run(L.A, msg), b = L.run(L.B(), msg);
const k = process.argv[4] ?? 'faces';
const A = a.o.result.body[k], Bv = b.o.result.body[k];
for (let i = 0; i < Math.max(A.length, Bv.length); i++) { const x = JSON.stringify(A[i]), y = JSON.stringify(Bv[i]); if (x !== y) console.log(i, '\n o', x, '\n r', y); }
