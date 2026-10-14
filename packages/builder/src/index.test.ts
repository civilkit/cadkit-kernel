import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { Part } from './index.js';
import { isSketchFeature } from '../../part-model/src/index.js';

describe('builder', () => {
  it('issues F-ids in creation order', () => {
    const p = new Part('T');
    const a = p.sketch('S', 'front', () => {});
    const b = p.extrude({ sketch: a, depth: 5 });
    assert.equal(a, 'F1');
    assert.equal(b, 'F2');
    assert.equal(p.seq, 3);
  });

  it('maps a string plane to {kind:"std",id} and rect() to 4 pts + 4 lines', () => {
    const p = new Part('T');
    p.sketch('S', 'front', (s) => s.rect(0, 0, 10, 5));
    const f = p.features[0];
    assert.equal(isSketchFeature(f), true);
    if (!isSketchFeature(f)) return;
    assert.deepEqual(f.plane, { kind: 'std', id: 'front' });
    assert.equal(f.points.length, 5); // O + 4 corners
    assert.equal(f.entities.length, 4);
    assert.ok(f.entities.every((e) => e.type === 'line'));
  });

  it('omits depth for throughBoth, keeps it for blind', () => {
    const p = new Part('T');
    const s = p.sketch('S', 'front', () => {});
    const cut = p.extrude({ sketch: s, op: 'cut', end: 'throughBoth' });
    const add = p.extrude({ sketch: s, end: 'blind', depth: 10 });
    assert.ok(!('depth' in (p.features[1] as Record<string, unknown>)));
    assert.equal((p.features[2] as Record<string, unknown>).depth, 10);
    assert.equal(cut, 'F2');
    assert.equal(add, 'F3');
  });

  it('offsetPlane builds a plane feature', () => {
    const p = new Part('T');
    const id = p.offsetPlane('top', 20);
    assert.equal(id, 'F1');
    assert.deepEqual(p.features[0], {
      type: 'plane', mode: 'offset',
      base: { kind: 'std', id: 'top' }, offset: 20, flip: false,
      name: 'Plane 20', visible: false, id: 'F1',
    });
  });

  it('plate golden: 4 features, 4 hole circles, cut has no depth', () => {    const p = new Part('Plate with holes', 'aluminium');
    const base = p.sketch('Outline', 'front', (s) => s.rect(0, 0, 100, 60));
    p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 10, name: 'Plate' });
    const holes = p.sketch('Holes', 'front', (s) => {
      for (const [x, y] of [[10, 10], [90, 10], [10, 50], [90, 50]] as Array<[number, number]>) s.circle(x, y, 4);
    });
    p.extrude({ sketch: holes, op: 'cut', end: 'throughBoth', name: 'Drill' });
    const doc = p.toJSON();
    assert.equal(doc.features.length, 4);
    assert.equal(doc.seq, 5);
    assert.equal(doc.material, 'aluminium');
    const holeSketch = doc.features[2];
    assert.equal(isSketchFeature(holeSketch), true);
    if (!isSketchFeature(holeSketch)) return;
    assert.equal(holeSketch.entities.filter((e) => e.type === 'circle').length, 4);
    assert.equal(holeSketch.points.length, 5); // O + 4 centres
    assert.deepEqual((doc.features[1] as Record<string, unknown>).depth, 10);
    assert.ok(!('depth' in (doc.features[3] as Record<string, unknown>)));
  });
});

describe('builder solid features', () => {
  const rec = (p: Part, i: number) => p.features[i] as unknown as Record<string, unknown>;
  it('emits defaults matching the kernel dispatch', () => {
    const p = new Part('T');
    const s = p.sketch('S', 'front', () => {});
    p.revolve({ sketch: s });
    p.fillet({ radius: 3, edges: [] });
    p.chamfer({ distance: 1, edges: [] });
    p.shell({ thickness: 2 });
    p.hole({ sketch: s });
    p.draft({ faces: [], angle: 3 });
    p.rib({ sketch: s, thickness: 2 });
    p.lip({ width: 1, height: 1 });
    p.helix({ sketch: s });
    p.sweep({ sketch: s });
    p.loft({ sketches: [s] });
    p.lpattern({ seeds: ['F2'], spacing: 20, count: 3 });
    p.cpattern({ seeds: ['F2'], count: 4 });
    p.mirror({ seeds: ['F2'] });
    p.scale({ factor: 2 });
    p.mate({});
    p.deleteBody({});
    p.deleteFace({ faces: [] });
    p.moveFace({ faces: [], distance: 1 });
    p.dome({ height: 5 });
    p.splitLine({ faces: [] });
    p.axis({ refs: [] });
    const types = p.features.map((f) => (f as { type: string }).type);
    assert.deepEqual(types, ['sketch', 'revolve', 'fillet', 'chamfer', 'shell', 'hole', 'draft', 'rib', 'lip', 'helix', 'sweep', 'loft', 'lpattern', 'cpattern', 'mirror', 'scale', 'mate', 'deleteBody', 'deleteFace', 'moveFace', 'dome', 'splitLine', 'axis']);
    assert.deepEqual(rec(p, 1), { type: 'revolve', name: 'add revolve', op: 'add', sketch: 'F1', axis: null, end: 'blind', angle: 360, id: 'F2' });
    assert.deepEqual(rec(p, 2), { type: 'fillet', name: 'fillet', radius: 3, edges: [], id: 'F3' });
    assert.deepEqual(rec(p, 3), { type: 'chamfer', name: 'chamfer', chamferType: 'equal', distance: 1, edges: [], id: 'F4' });
    assert.deepEqual(rec(p, 4), { type: 'shell', name: 'shell', faces: [], thickness: 2, id: 'F5' });
    assert.deepEqual(rec(p, 5), { type: 'hole', name: 'hole', sketch: 'F1', kind: 'simple', diameter: 6, end: 'blind', depth: 10, tip: true, id: 'F6' });
    assert.deepEqual(rec(p, 10), { type: 'sweep', name: 'add sweep', op: 'add', sketch: 'F1', path: null, id: 'F11' });
    assert.deepEqual(rec(p, 11), { type: 'loft', name: 'add loft', op: 'add', sketches: ['F1'], id: 'F12' });
    assert.deepEqual(rec(p, 12).dir, { kind: 'axis', axis: 'x' });
    assert.equal(rec(p, 13).equal, true);
    assert.deepEqual(rec(p, 15), { type: 'scale', name: 'scale', factor: 2, about: 'centroid', id: 'F16' });
  });

  it('moveFace / moveBody wire shapes', () => {
    const p = new Part('T');
    p.moveFace({ faces: ['F2.C1'], distance: 2.5 });
    p.moveFace({ faces: ['F2.S.e1'], mode: 'translate', translate: [3, 0, 0] });
    p.moveBody({ bodies: [{ face: 'F2.C1' }], translate: [10, 0, 0], rotate: { axis: [0, 0, 1], origin: [0, 0, 0], angle: 30 }, copy: true });
    assert.deepEqual(rec(p, 0), { type: 'moveFace', name: 'moveFace', faces: ['F2.C1'], distance: 2.5, id: 'F1' });
    assert.deepEqual(rec(p, 1), { type: 'moveFace', name: 'moveFace', faces: ['F2.S.e1'], mode: 'translate', translate: [3, 0, 0], id: 'F2' });
    assert.deepEqual(rec(p, 2), { type: 'moveBody', name: 'moveBody', bodies: [{ face: 'F2.C1' }], translate: [10, 0, 0], rotate: { axis: [0, 0, 1], origin: [0, 0, 0], angle: 30 }, copy: true, id: 'F3' });
  });

  it('point() makes a hole-centre point entity', () => {
    const p = new Part('T');
    let eid = '';
    p.sketch('C', 'front', (s) => { eid = s.point(50, 30); });
    const f = p.features[0];
    assert.equal(isSketchFeature(f), true);
    if (!isSketchFeature(f)) return;
    assert.equal(f.points.length, 2); // O + centre
    assert.deepEqual(f.entities, [{ id: eid, type: 'point', p: 'p1' }]);
  });
});

describe('builder document metadata', () => {
  it('fromJSON/toJSON preserve expectations', () => {
    const p = new Part('T');
    p.expect = [{ kind: 'volume', min: 10 }];
    const q = Part.fromJSON(p.toJSON());
    assert.deepEqual(q.toJSON().expect, [{ kind: 'volume', min: 10 }]);
    assert.equal(q.seq, p.seq);
  });
});
