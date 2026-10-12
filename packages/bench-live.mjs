// Live model benchmark: a real model gets a text description plus the guide
// and must return session edits; the built part is checked against the
// CadQuery ground-truth volume. First 10 bench parts, one shot per part,
// max_tokens caps the cost. No live call happens in `npm test`.
// usage: NANO_GPT_KEY=... node packages/bench-live.mjs [--first N] [--model id] [--max-tokens n] [--out dir]
// (build first: npm --prefix packages run build)
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { loadTasks } from './dist/session/src/bench.js';
import { getGuide } from './dist/session/src/guide.js';
import { Session } from './dist/session/src/session.js';
import { emptyPart } from './dist/part-model/src/index.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (f) => { const i = args.indexOf(f); return i < 0 ? null : args[i + 1]; };
const BASE = process.env.CK_BASE ?? 'https://nano-gpt.com/api/v1';
const MODEL = opt('--model') ?? process.env.CK_MODEL ?? 'anthropic/claude-haiku-4.5';
const MAX_TOKENS = +(opt('--max-tokens') ?? 2000);
const OUT = opt('--out') ?? null;
const KEY = process.env.NANO_GPT_KEY;
if (!KEY) { console.error('NANO_GPT_KEY is not set'); process.exit(2); }
if (OUT) mkdirSync(OUT, { recursive: true });

const OPS = `Edit ops (JSON array, applied atomically in order; ids are F1, F2, ... in creation order):
{"op":"add","feature":{...full feature JSON, without id (assigned) or with a fresh "Fn" id...}}
{"op":"set","id":"F2","path":"depth","value":5}
{"op":"remove","id":"F1"}  {"op":"move","id":"F2","to":0}  {"op":"replace","id":"F2","feature":{...}}
Sketch feature: {type:"sketch",name,plane:{kind:"std",id:"front"|"top"|"right"},
 points:[{id,x,y}],entities:[{id,type:"line",p1,p2}|{id,type:"circle",c,r}],constraints:[]}.
front=XY(+Z), top=XZ, right=ZY. Point/entity ids are p1,e5,... (any unique strings).
extrude: {type:"extrude",op:"add"|"cut",sketch:"F1",end:"blind"|"throughBoth",depth}. revolve:
{type:"revolve",op,sketch,axis:"<sketch line id>",angle:360}. hole: {type:"hole",sketch:"F3"(points
are centres),kind:"simple",diameter,end:"blind",depth}. fillet: {type:"fillet",radius,edges:[]}
(leave edges empty only if you have no edge names yet). A batch that regens with errors is
rejected whole: keep every feature valid. Respond with ONLY the JSON array.`;

async function ask(task, guide) {
  const body = {
    model: MODEL,
    max_tokens: MAX_TOKENS,
    temperature: 0,
    messages: [
      { role: 'system', content: `You build parametric CAD parts as JSON edit ops.\n\n${guide}\n\n${OPS}` },
      { role: 'user', content: `Build this part:\n${task.description}\n\nRespond with ONLY the JSON array of edit ops.` },
    ],
  };
  const r = await fetch(`${BASE}/chat/completions`, {
    method: 'POST',
    headers: { 'content-type': 'application/json', authorization: `Bearer ${KEY}` },
    body: JSON.stringify(body),
  });
  if (!r.ok) throw new Error(`model HTTP ${r.status}: ${(await r.text()).slice(0, 200)}`);
  const j = await r.json();
  return { text: j.choices?.[0]?.message?.content ?? '', usage: j.usage ?? null };
}

function parseOps(text) {
  const m = text.match(/```(?:json)?\s*([\s\S]*?)```/)?.[1] ?? text;
  return JSON.parse(m.trim());
}

function categorize(r, volumeWant) {
  if (!r.editOk) return r.schemaError ? 'schema-error' : 'regen-error';
  if (r.volumeGot === null) return 'no-body';
  const rel = Math.abs(r.volumeGot - volumeWant) / Math.abs(volumeWant);
  if (rel <= 1e-3) return 'ok';
  return rel <= 0.1 ? 'volume-close' : rel <= 1 ? 'volume-far' : 'volume-wrong-scale';
}

const tasks = loadTasks({ first: opt('--first') ? +opt('--first') : 10 });
const guide = getGuide().text;
const rows = [];
for (const task of tasks) {
  const t0 = Date.now();
  const s = await Session.open(emptyPart(task.id), { author: `live:${MODEL}` });
  try {
    let ops, raw = '', usage = null, detail = '', editOk = false, volumeGot = null;
    try {
      const ans = await ask(task, guide);
      raw = ans.text; usage = ans.usage;
      ops = parseOps(raw);
      if (!Array.isArray(ops)) throw new Error('response was not a JSON array');
      if (OUT) writeFileSync(path.join(OUT, `${task.id}.ops.json`), JSON.stringify({ task, raw, usage, ops }, null, 1));
    } catch (e) {
      detail = `model-output: ${String(e.message ?? e).slice(0, 160)}`;
      rows.push({ id: task.id, family: task.family, pass: false, category: 'unparseable', volumeGot, volumeWant: task.volume, detail, ms: Date.now() - t0, usage });
      console.log(`FAIL ${task.id} unparseable ${detail}`);
      continue;
    }
    const er = await s.edit(ops, `live attempt ${task.id}`);
    if (!er.ok) {
      detail = er.schemaError ?? er.errors?.map(([f, x]) => `${f}:${x.code}`).join(',') ?? 'edit failed';
      rows.push({ id: task.id, family: task.family, pass: false, category: categorize({ editOk: false, schemaError: er.schemaError, volumeGot }, task.volume), volumeGot, volumeWant: task.volume, detail, ms: Date.now() - t0, usage });
      console.log(`FAIL ${task.id} ${detail}`);
      continue;
    }
    editOk = true;
    const v = await s.verify([{ kind: 'volume', equals: task.volume, relTol: 1e-3 }]);
    volumeGot = (v.check?.results[0]?.measured) ?? null;
    const cat = categorize({ editOk, volumeGot }, task.volume);
    detail = cat === 'ok' ? `volume ${volumeGot}` : (v.hints.map((h) => h.hint).join(' | ') || `volume ${volumeGot} vs ${task.volume}`);
    rows.push({ id: task.id, family: task.family, pass: cat === 'ok', category: cat, volumeGot, volumeWant: task.volume, detail, ms: Date.now() - t0, usage });
    console.log(`${cat === 'ok' ? 'PASS' : 'FAIL'} ${task.id} ${cat} ${detail.slice(0, 160)}`);
    if (OUT) writeFileSync(path.join(OUT, `${task.id}.json`), JSON.stringify({ task, raw, usage, ops, row: rows.at(-1) }, null, 1));
  } finally {
    await s.close();
  }
}
const byFamily = {};
for (const r of rows) {
  byFamily[r.family] ??= { pass: 0, total: 0, categories: {} };
  byFamily[r.family].total++;
  if (r.pass) byFamily[r.family].pass++;
  byFamily[r.family].categories[r.category] = (byFamily[r.family].categories[r.category] ?? 0) + 1;
}
const cats = {};
for (const r of rows) cats[r.category] = (cats[r.category] ?? 0) + 1;
console.log('by family: ' + Object.entries(byFamily).map(([f, x]) => `${f} ${x.pass}/${x.total} [${Object.entries(x.categories).map(([c, n]) => `${c}:${n}`).join(',')}]`).join(' '));
console.log('modes: ' + Object.entries(cats).map(([c, n]) => `${c}:${n}`).join(' '));
console.log(`live-bench ${rows.filter((r) => r.pass).length}/${rows.length} (${MODEL})`);
if (OUT) writeFileSync(path.join(OUT, 'report.json'), JSON.stringify({ model: MODEL, rows, byFamily }, null, 1));
