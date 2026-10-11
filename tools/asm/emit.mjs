// emit.mjs <cases> <name> > part.json
const mod = await import(process.argv[2]);
const L = await import('./lib.mjs');
console.log(JSON.stringify(L.part(mod.cases[process.argv[3]][0])));
