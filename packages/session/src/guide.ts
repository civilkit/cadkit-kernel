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
| hole | sketch of point entities (sk.point(x,y) -> {type:"point",p}), kind(simple/cbore/csink), diameter, end, depth, tip |
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
### hole feature (not an extrude-cut)
Hole centres are point entities, not circles or bare points. The hole drills
away from the sketch plane by default; flip it to drill into the body:
\`\`\`js
const c = p.sketch('Centres', 'front', (sk) => { sk.point(50, 30); });
p.hole({ sketch: c, diameter: 8, flip: true });
\`\`\`
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
A document may carry intent: top-level "expect" with check expectations
([{kind:"volume",min:1000}]). verify checks them by default and edits flag
breaks automatically (ADR 0003 item 5).

## Features (${FEATURE_TYPES.length} wire types)
${FIELD_TABLE}

## Frames and traps (read before writing sketches)
- Sketch planes: front = XY (normal +Z), top = XZ, right = ZY. A sketch on
  **top** maps local y to **-z** in the body: a point at y=+15 lands at z=-15.
  A cut that misses the body fails with cutNoEffect; flip the sign of local y.
- Extrude and cut run along the plane normal; a cut sketched on a plane inside
  the body goes the other way from what a CadQuery reader expects, so check the
  floor and wall positions with query/describe or render.
- Revolve: axis is a sketch line id. Add a **construction** line for the axis
  (\`{type:"line", construction:true}\`); otherwise the first profile edge is
  used and the solid is wrong with no error.
- edit returns ok:false with errors/hints (it is not a protocol error). The
  document is unchanged then. Check ok and read hints before the next call.
- Omit "id" on an added feature and it is assigned; the reply lists them in
  "added". Reference features by those ids in later ops.
- A sketch's entities reference point ids; circle needs {c: pointId, r}.

## Naming (ADR 0002)
Cap faces: F2.C0. Side faces from sketch entities: F2.S.e10. Edges:
{faces:[a,b], at:[x,y,z]}. Prefer referencing caps and sketch-entity sides
over positions; positions go stale, names persist. refDiff warns when an edit
re-points a reference.

## Recipes
${RECIPES}

## Error codes and usual fixes
${fixes}

## Design rules (manufacturability)
checkProfile runs a named profile of rules on the body: fdm-print (3D printing),
cnc-3axis, injection. profiles lists them with each rule's parameters. Every
issue has rule, severity (error/warn/info), an actionable message with numbers,
faces / edges (persistent names, feed them to resolve / describe or highlight
them), value and limit. Pass an inline profile to change a limit:
{rules:[{id:"minWall", params:{min:2}, severity:"error"}]}. Sampled rules
(walls, gaps, undercut) are estimates; planes, cylinders and edges are exact.

## Loop
edit → read the diff (did anything unintended change?) → verify (regen +
validate + check). Fix the smallest responsible feature first; hints name it.
`;
  return { version: GUIDE_VERSION, text };
}
