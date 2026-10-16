// Writes the BOSL2 "onion" profile (sphere arc + cone + axis, revolved 360 about the axis) as a part document.
// usage: node tools/onion_case.mjs r ang cap_h [out.json]   (defaults 15 45 0)
import { writeFileSync } from 'node:fs';
export function onionPart(r, angDeg, cap, plane = 'front') {
  const a = angDeg * Math.PI / 180, tipY = r / Math.sin(a), tr = [r * Math.cos(a), r * Math.sin(a)];
  const top = cap > 0 && cap < tipY ? cap : tipY, dx = cap > 0 && cap < tipY ? (tipY - cap) * Math.tan(a) : 0;
  const P = [['c', 0, 0], ['s', 0, -r], ['t', ...tr], ['k', dx, top], ['z', 0, top]];
  const E = [{ type: 'arc', id: 'e1', c: 'c', p1: 's', p2: 't' }, { type: 'line', id: 'e2', p1: 't', p2: 'k' }];
  if (dx > 0) E.push({ type: 'line', id: 'e3', p1: 'k', p2: 'z' });
  E.push({ type: 'line', id: 'ax', p1: dx > 0 ? 'z' : 'k', p2: 's' });
  const pts = P.filter(([id]) => dx > 0 || id !== 'z').map(([id, x, y]) => ({ id, x, y }));
  return { format: 'ck-part', version: 1, name: 'onion', features: [
    { type: 'sketch', id: 'F1', plane: { kind: 'std', id: plane }, points: pts, entities: E, constraints: [] },
    { type: 'revolve', id: 'F2', op: 'add', sketch: 'F1', axis: 'ax', angle: 360 }] };
}
if (process.argv[1].endsWith('onion_case.mjs')) {
  const [r = 15, ang = 45, cap = 0] = process.argv.slice(2, 5).map(Number);
  writeFileSync(process.argv[5] ?? '/dev/stdout', JSON.stringify(onionPart(r, ang, cap), null, 1));
}
