// Request-body cases for the wire layer. Each case = a fresh kernel instance
// fed `bodies` in order (state carries across bodies within a case).
// Consumed by the reference-capture tool (not published) (writes the recorded reference data (not published) from the
// ORIGINAL kernel) and tools/diff.mjs (original vs rebuilt, byte-for-byte).
const T = ['comb', 'continuity', 'convert', 'curvatureAt', 'curvatureMap', 'drag', 'drawing', 'inspect',
  'interference', 'massProps', 'matchEdges', 'measure', 'refCurve', 'sketchOffset', 'step'];
const EMPTY = '{"type":"regen","part":{"format":"ck-part","features":[]}}';
const m = (msg) => `{"id":1,"msg":${msg}}`;
const cases = [];
const one = (name, ...bodies) => cases.push({ name, bodies });

// --- envelope ---
const env = ['', ' ', 'x', 'null', '1', '"s"', '[]', '{}', '{"id":1}', '{"msg":1}', '{"msg":null}', '{"msg":[]}',
  '{"msg":{}}', '{"id":"a","msg":{"type":"ping"}}', '{"id":null,"msg":{"type":"ping"}}', '{"msg":{"type":"ping"}}',
  '{"id":1.5,"msg":{"type":"ping"}}', '{"msg":{"type":1}}', '{"msg":{"type":null}}', '{"msg":{"type":""}}',
  '{"msg":{"type":"Ping"}}', '{"msg":{"type":"bogus"}}', '{"msg":{"type":"ping","x":1},"extra":2}',
  '{"msg":{"type":"ping"},"msg":{"type":"bogus"}}', '{"msg":{"type":"bogus"},"msg":{"type":"ping"}}',
  '{"msg":{"type":"ping","type":"bogus"}}',
  '{"msg":{"type":"ping"},}', '{"msg":{"type":"ping"}} ', '{"msg":{"type":"ping"}}x', ' {"msg":{"type":"ping"}}',
  '{"msg":{"type":"ping","a":01}}', '{"msg":{"type":"ping","a":[1,]}}', '{"msg":{"type":"ping","a":1e999}}',
  '{"msg":{"type":"ping","a":NaN}}', '{"msg":{"type":"ping","a":"\\x"}}', '{"msg":{"type":"ping","a":"\\ud800"}}',
  '{"msg":{"type":"ping"}/*c*/}', '{"msg":{"type":"p\\u0069ng"}}', '{"msg":{"type":"ping","a":-}}',
  '{"msg":{"type":"ping","a":.5}}', '{"msg":{"type":"ping","a":1.}}', '{"msg":{"type":"ping","a":+1}}',
  '{"msg":{"type":"ping","a":tru}}', "{'msg':1}", '{"msg":{"type":"ping","debug":true}}',
  '{"debug":true,"msg":{"type":"ping"}}', '{"session":"x","id":1,"msg":{"type":"ping"}}',
  '{"msg":{"type":"bogus\\"q\\u0001é"}}', '{"msg":{"type":"ping","a":"\t"}}', '{"msg":{"type":"ping","a":"\u0001"}}',
  '{"msg":{"type":"bogus\\b\\f\\n\\r\\t\\/\\u007f\\u2028\\u001f\\ud83d\\ude00\\ud800","debug":true}}',
  '{"msg":{"type":"x","debug":1}}', '{"msg":{"type":"x","debug":true}}', '{"msg":{"debug":true}}',
  '{"msg":{"type":"ping","debug":false}}', '{"msg":{"type":"ping","a":1e}}', '{"msg":{"type":"ping","a":1e+}}',
  '{"msg":{"type":"ping","a":-01.e5}}', '{"msg":{"type":"ping","a":"\\u00zz"}}', '{"msg":{"type":"ping","a":"\\uDC00"}}',
  '{"msg":{"type":"ping","a":Infinity}}', '{"msg":{"type":"ping","a":-Infinity}}', '{"msg":{"type":"ping","a":null}}',
  '{"msg":{"type":"ping"}}\n\t\r', '﻿{"msg":{"type":"ping"}}', '{"msg":{"type":"ping"}}{}',
  '{"msg":{"type":"ping","a":1E5}}', '{"msg":{"type":"ping","a":0x1}}', '{"msg" : { "type" : "ping" } }',
  '{"msg":\f{"type":"ping"}}', '{"msg":{"type":"a\\ud800\\u0041b"}}', '{"msg":{"type":"a\\ud800xb"}}',
  '{"msg":{"type":"a\\udc00\\ud800b"}}', '{"msg":{"type":"\\u0000"}}', '{"msg":{"type":"a\\ud800\\n"}}',
  `{"msg":{"type":"ping","a":${'['.repeat(510)}${']'.repeat(510)}}}`,
  `{"msg":{"type":"ping","a":${'['.repeat(511)}${']'.repeat(511)}}}`,
  `${'['.repeat(512)}${']'.repeat(512)}`, `${'['.repeat(513)}${']'.repeat(513)}`,
  `{"msg":{"type":"ping","a":${'{"a":'.repeat(510)}1${'}'.repeat(510)}}}`,
  `{"msg":{"type":"ping","a":${'{"a":'.repeat(511)}1${'}'.repeat(511)}}}`,
];
env.forEach((b, i) => one(`envelope#${i}`, b));

// --- every type with no state, then with/without debug ---
for (const t of [...T, 'regen', 'ping']) {
  one(`nostate:${t}`, m(`{"type":"${t}"}`));
  one(`nostate-debug:${t}`, m(`{"type":"${t}","debug":true,"tag":"none"}`));
}

// --- empty-part state (body null) ---
const after = (name, ...msgs) => one(name, m(EMPTY), ...msgs.map(m));
for (const t of T) {
  after(`empty:${t}`, `{"type":"${t}"}`, `{"type":"${t}","tag":"other"}`, `{"type":"${t}","tag":1}`,
    `{"type":"${t}","debug":true}`);
}
const F = '"frame":{"o":[0,0,0],"x":[1,0,0],"y":[0,1,0],"z":[0,0,1]}';
after('empty:params',
  '{"type":"sketchOffset","distance":1}', '{"type":"sketchOffset","distance":"1"}',
  '{"type":"sketchOffset","distance":1,"sketch":{}}', '{"type":"sketchOffset","distance":1,"sketch":{"points":[],"entities":[]},"ids":[]}',
  '{"type":"sketchOffset","distance":0,"sketch":{"points":[],"entities":[]},"ids":[]}',
  '{"type":"matchEdges","part":{"format":"ck-part","features":[]}}',
  '{"type":"matchEdges","part":{"format":"ck-part","features":[]},"seeds":[],"debug":true}',
  '{"type":"regen","part":5}', '{"type":"massProps"}',
  '{"type":"drag","face":"a","point":[0,0,0],"target":[1,0,0]}', '{"type":"drag","face":1,"point":[0,0,0],"target":[1,0,0]}',
  '{"type":"drag","face":"a","point":[0,0],"target":[1,0,0]}', '{"type":"drag","face":"a","point":[0,0,0]}',
  '{"type":"drag","face":"a","point":[0,0,0,1],"target":[1,0,0]}',
  '{"type":"drawing","views":[{}]}', '{"type":"drawing","views":[{"name":"front"}]}', '{"type":"drawing","views":5}',
  '{"type":"inspect","ref":{"kind":"face","id":"x"}}', '{"type":"inspect","ref":{}}', '{"type":"inspect","ref":"x"}',
  '{"type":"inspect","ref":{"kind":"edge"}}', '{"type":"inspect","ref":{"kind":"edge","faces":["a","b"],"at":[0,0,0]}}',
  '{"type":"inspect","ref":{"kind":"feature","id":"F1"}}', '{"type":"inspect","ref":{"kind":"feature"}}',
  '{"type":"inspect","ref":{"kind":"sketch","id":"F1"}}', '{"type":"inspect","ref":{"kind":"body"}}',
  '{"type":"inspect","ref":{"kind":"vertex","id":"x"}}', '{"type":"inspect","ref":{"kind":"vertex","at":[0,0,0]}}',
  '{"type":"inspect","ref":{"kind":"face"}}', '{"type":"inspect","ref":{"kind":"bogus","id":"x"}}',
  '{"type":"inspect","ref":{"kind":"curve","id":"x"}}', '{"type":"inspect","ref":{"kind":"plane","id":"x"}}',
  '{"type":"inspect","ref":{"kind":"FACE","id":"x"}}', '{"type":"inspect","ref":{"kind":5}}', '{"type":"inspect","ref":{"kind":""}}',
  '{"type":"inspect","ref":{"kind":"surface","id":"x"}}',
  '{"type":"convert","frame":{}}', '{"type":"convert","frame":{"o":[0,0,0]}}',
  '{"type":"convert","frame":{"o":[0,0,0],"x":[1,0,0],"y":[0,1,0]}}', '{"type":"convert","frame":{"o":[0,0],"x":[1,0,0],"y":[0,1,0],"z":[0,0,1]}}',
  '{"type":"convert","frame":[0,0,0]}', '{"type":"convert","frame":{"o":[0,0,0],"x":[0,0,0],"y":[0,0,0],"z":[0,0,0]}}',
  `{"type":"convert",${F}}`, `{"type":"convert",${F},"sketch":5}`,
  '{"type":"measure","refs":[]}', '{"type":"measure","refs":[{"kind":"face","id":"x"}]}',
  '{"type":"curvatureAt","points":[[0,0,0]]}', '{"type":"curvatureAt","points":5}', '{"type":"curvatureAt","points":[1,"x",{}]}',
  '{"type":"comb","refs":[{"kind":"face","id":"x"}]}', '{"type":"comb","refs":5}',
  '{"type":"comb","refs":[{"kind":"edge"},{"kind":"face"},{},{"kind":"curve","id":"c"},{"kind":"sketch","id":"S"},"x",null,{"kind":"vertex"}]}',
  '{"type":"continuity","refs":[],"required":"G1"}', '{"type":"continuity","required":5}', '{"type":"continuity","required":"G7"}',
  '{"type":"continuity","required":"G0"}', '{"type":"continuity","required":"g1"}', '{"type":"continuity","required":"G3"}',
  '{"type":"continuity","required":"G1 "}', '{"type":"continuity","tol":{"gap":1}}',
  '{"type":"continuity","tol":{"gap":"x","angle":2,"curvature":3,"rate":4,"z":1}}', '{"type":"continuity","tol":5}',
  '{"type":"refCurve","ref":{"kind":"edge"}}', '{"type":"refCurve","ref":5}', '{"type":"step","name":"x"}',
  '{"type":"interference","debug":true}');

// --- regen part validation & state retention ---
const P = (p) => m(`{"type":"regen","part":${p}}`);
one('regen:part-validation', ...['{"format":"ck-part"}', '{"format":"ck-part","version":2}',
  '{"format":"ck-part","version":1}', '{"format":"ck-part","version":1,"features":{}}',
  '{"format":"ck-part","features":[]}', '{"version":1,"features":[]}', '"x"',
  '{"format":"ck-part","version":0,"features":[]}', '{"format":"ck-part","version":1.5,"features":[]}',
  '{"format":"ck-part","features":null}', '{"format":"ck-part","features":[],"name":5,"params":5}'].map(P),
  m('{"type":"regen","part":{"format":"ck-part","features":[]},"debug":true}'),
  m('{"type":"regen","part":5,"debug":true}'));
one('regen:tags', m('{"type":"regen","tag":"x","part":{"format":"ck-part","features":[]}}'),
  m('{"type":"interference"}'), m('{"type":"interference","tag":"x"}'), m('{"type":"interference","tag":""}'),
  m('{"type":"regen","tag":"","part":{"format":"ck-part","features":[]}}'), m('{"type":"interference","tag":""}'),
  m('{"type":"regen","tag":5,"part":{"format":"ck-part","features":[]}}'), m('{"type":"interference"}'),
  m('{"type":"regen","tag":"x","part":5}'), m('{"type":"interference","tag":"x"}'));

export default cases;
