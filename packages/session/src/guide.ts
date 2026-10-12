// Versioned authoring guide served to the agent (spec §Authoring guidance).
// Error codes render from verify's fix table, so guidance and repair hints
// cannot drift apart. Bump GUIDE_VERSION when the guidance changes.
import { FEATURE_TYPES } from '../../part-model/src/index.js';
import { listFixes } from '../../driver/src/verify.js';

export const GUIDE_VERSION = 'ck-guide/1';

const FIELD_TABLE = `| type | key fields |
|---|---|
| sketch | plane(front=top XY, top=XZ, right=ZY), points[{id,x,y}], entities[line(p1,p2),circle(c,r),arc], constraints[] |
| extrude | op(add/cut), sketch, end(blind/mid/throughBoth), depth, flip |
| revolve | op, sketch, axis(sketch line id, defaults to first line), end, angle |
| fillet | radius, edges[{faces:[a,b],at:[x,y,z]}] |
| chamfer | chamferType(equal/angleDistance/distanceDistance/vertex), distance, distance2, angle, edges |
| shell | faces[] (empty = whole body), thickness, outward |
| hole | sketch(points = centres), kind(simple/cbore/csink), diameter, end, depth, tip |
| draft | neutral, faces[], angle |
| rib | sketch(open chain), thickness |
| lip | kind, faces[], edges[], width, height |
| helix | sketch, definedBy(pitchRevs), pitch, revolutions, height |
| sweep/loft | op, sketch(+path) / sketches[] |
| lpattern/cpattern | seeds[], dir/axis, spacing/count, angle, equal, reverse |
| mirror | seeds[], plane, body |
| plane | mode(offset), base(std plane), offset |
| dome/splitLine/moveFace/scale/axis/mate/deleteBody/deleteFace | see @ck-cad/part-model |
| sheet features | surfaceExtrude/Revolve/Planar/Offset, trimSurface, knit, fill, thicken, extend |
| freeform/editCurve/projectCurve/pipeCut/cutWithSurface | see @ck-cad/part-model |
| insertPart/moveBody/patterns | nested part, placement dx/dy/dz/axis/angle |`;

const RECIPES = `### plate with hole pattern
\`\`\`js
const s = p.sketch('Outline', 'front', (sk) => sk.rect(0, 0, 100, 60));
p.extrude({ sketch: s, op: 'add', end: 'blind', depth: 10 });
const h = p.sketch('Holes', 'front', (sk) => { for (const [x, y] of [[10,10],[90,10],[10,50],[90,50]]) sk.circle(x, y, 4); });
p.extrude({ sketch: h, op: 'cut', end: 'throughBoth' });
\`\`\`
### revolve profile
Revolve needs an axis: a sketch line id. The builder defaults to the sketch's
first line. Keep the profile on one side of the axis.
### shell
Shell faces[] empty hollows the whole body; list faces to keep them open.
Thickness must stay below the thinnest wall (else shellTooThick).`;

export function getGuide(): { version: string; text: string } {
  const fixes = listFixes()
    .map((f) => `- ${f.code}: ${f.sample.replace(/^Fn:? ?/, '')}`)
    .join('\n');
  const text = `# Agent authoring guide (${GUIDE_VERSION})

## Document
Units mm. Features F1, F2, … in creation order; seq is the next id. Every
change is an edit to this document (ADR 0001). Batches are atomic: a failing
batch leaves the document unchanged. Undo reverts whole batches.

## Features (${FEATURE_TYPES.length} wire types)
${FIELD_TABLE}

## Naming (ADR 0002)
Cap faces: F2.C0. Side faces from sketch entities: F2.S.e10. Edges:
{faces:[a,b], at:[x,y,z]}. Prefer referencing caps and sketch-entity sides
over positions; positions go stale, names persist. refDiff warns when an edit
re-points a reference.

## Recipes
${RECIPES}

## Error codes and usual fixes
${fixes}

## Loop
edit → read the diff (did anything unintended change?) → verify (regen +
validate + check). Fix the smallest responsible feature first; hints name it.
`;
  return { version: GUIDE_VERSION, text };
}
