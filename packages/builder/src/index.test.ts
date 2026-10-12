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

  it('plate golden: 4 features, 4 hole circles, cut has no depth', () => {
    const p = new Part('Plate with holes', 'aluminium');
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
