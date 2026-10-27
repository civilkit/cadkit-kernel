# ck-io

File export. rs_paths.txt shows step/write.rs only — a STEP (AP214,
`AUTOMOTIVE_DESIGN`) writer. Every format string survives in
kernel.strings; layout/numbering/formatting are pinned by `step` replies
of the original kernel (the recorded reference data (not published)).

## API

```rust
use ck_io::step::{write_step, StepOpts, StepWriter};
let text: String = write_step(&body, name, &StepOpts { unix_ms: host_ck_unix_ms() });
```

- `step::write::real(f64)`, `string(&str)` (STEP string escaping),
  `timestamp(unix_ms)` are public helpers.
- `StepWriter` emits single entities (`point`, `dir`, `axis2`, `curve`,
  `surface`, ...) — used by the entity tests; `starting_at(n)` /
  `seed_dir` exist for those tests.

## `step` request (wire, pinned with tools/q.mjs)

- request `{type:"step", tag:"main", name:<partName>}` (client:
  app.pretty.js `exportSTEP` / `export({format:"step"})`).
- reply: `{"ok":true,"result":"<whole STEP file as a JSON string>","ms":0}`
  — no binary buffers. `result:null` when the regenerated body is null
  (empty part / surface-only part). `tag` missing / null / `"main"` → the
  regen state; any other tag → `{"ok":false,"error":"noState: nothing
  regenerated yet","ms":0}` (also before any regen). `name` missing or not
  a string → `"part"`.
- `FILE_NAME('<name>.step','<UTC of ck_unix_ms>',...)`, the same escaped
  name in `PRODUCT('<n>','<n>',...)`, `MANIFOLD_SOLID_BREP('<n>',..)` and
  `ADVANCED_BREP_SHAPE_REPRESENTATION('<n>',..)`.

## Pinned rules (see write.rs header for detail)

- reals: `0.`; plain shortest decimal + `.`; scientific `1.E-7`,
  `6.123233995736766E-17`, `1.E15` when |x|<1e-6 or ≥1e15.
- strings: `''`, `\\`, control chars dropped, non-ASCII → `\X2\hhhh\X0\`
  per char (UTF-16 units).
- DIRECTION sharing keyed by exact bits (sign of zero matters);
  CARTESIAN_POINT / VECTOR never shared. Decoded from kernel.wasm:
  f414 (DIRECTION) first normalises `v·(1/√(x²+y²+z²))` (zero vector if
  |v| ≤ 1e-300 or non-finite), then hashes the bits; f416
  (AXIS2_PLACEMENT_3D) writes axis = unit z and ref = unit
  (x − z(x·z)) (|·|² summed z²+(x²+y²); fallback unit z × e with e the
  world axis of z's smallest |component|, f3907), both through f414;
  f417 LINE = point o + t0·d, `VECTOR('',#dir,1.)` verbatim; CIRCLE and
  f418 PLANE axis = x × y of the stored frame; cylinder / sphere / torus
  axis = stored z; cone: negative semi-angle → negated axis.
- the sign of zero of every oracle DIRECTION was recovered by
  instrumenting f414 (log of the normalised components) —
  the recorded reference data (not published); `write_step_signed_zeros` (test support)
  prints `-0.` so the test can compare them.
- seam-split arcs (ck-topo `wire_chains`) are one EDGE_CURVE.
- order: context #1..#14, world placement #15..#18, faces in shell order
  (surface, then per coedge: edge on first use [v0, v1, curve,
  EDGE_CURVE] + ORIENTED_EDGE, EDGE_LOOP, bound, ADVANCED_FACE),
  CLOSED_SHELL per shell, then all MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS,
  ABSR(name,(breps..,#18),#14), SDR(#9,ABSR).
- seams dropped; loops cut at seams; closed sphere/torus split into two
  faces on one surface; FACE_OUTER_BOUND only on planar faces.
- voids: CLOSED_SHELL with flipped face sense / bound flags +
  `ORIENTED_CLOSED_SHELL('',*,#,.F.)` + `BREP_WITH_VOIDS`.
- negative cone semi-angle written as reversed axis + positive angle.

## Tests / status

`cargo test -p ck-io`:
- the recorded reference data (not published), 16 golden bodies (ck-topo make::* constructions):
  BYTE-identical AND every DIRECTION's sign of zero equal to the
  oracle's: box box3 cyl dext flip mid two rev360 rev90 revax360 revax90
  sphere torus cone void (15). `draft` is the one known diff (the oracle
  rebuilds drafted faces with a draft op: vertices ~2e-10 away, side
  plane x = −edge direction; see ck-topo README). Plus: a seam-split
  cylinder cap circle writes the unsplit cyl file.
- Debug: `OUT=<dir> cargo test -p ck-io --test dump write_ours -- --ignored`
  writes every golden body with signed zeros.
- `tests/entities.rs`: rational B-spline surface/curve, B-spline curve,
  ellipse + SURFACE_OF_LINEAR_EXTRUSION, SURFACE_OF_REVOLUTION verbatim
  against oracle lines.

Request bodies: `tests/golden/<n>.req.json` is the regen request; run
`node tools/q.mjs "$(cat <n>.req.json)" '{"id":2,"msg":{"type":"step","tag":"main","name":"<n>"}}'`
and take `result` of the second reply. ell/spl/cvs/loft/revspl `.step`
are reference-only (bodies not buildable with ck-topo yet).

## GUESS / open

- OFFSET_SURFACE self_intersect `.F.`; open shells (OPEN_SHELL,
  SHELL_BASED_SURFACE_MODEL, MANIFOLD_SURFACE_SHAPE_REPRESENTATION,
  SHAPE_REPRESENTATION_RELATIONSHIP) layout — no oracle body produced them.
- multi-lump + void ordering (shells of all lumps, then all breps).
- split of a sense=false sphere/torus; loop-run joining across loop start.
- B-spline `closed_curve` = first ctrl == last ctrl.

## STEP import (`step/parse.rs`, `step/read.rs`)

Adapted from brepkit v2.129.15 (MIT OR Apache-2.0, `third_party/brepkit/`):
entity walk, surface / curve entity set, B-spline knot / weight decoding. The
Part 21 parser is a real value-tree parser (nested lists, typed parameters,
complex entities, comments); the builder writes ck-topo bodies.

```rust
let imp = ck_io::step::read_step(&text)?;   // Imported { body, solids, length_scale, warnings }
```

- entities: MANIFOLD_SOLID_BREP, BREP_WITH_VOIDS (+ ORIENTED_CLOSED_SHELL),
  CLOSED_SHELL, ADVANCED_FACE / FACE_SURFACE, FACE_BOUND / FACE_OUTER_BOUND,
  EDGE_LOOP, ORIENTED_EDGE, EDGE_CURVE, VERTEX_POINT; PLANE, CYLINDRICAL /
  CONICAL / SPHERICAL / TOROIDAL_SURFACE, SURFACE_OF_REVOLUTION,
  SURFACE_OF_LINEAR_EXTRUSION, OFFSET_SURFACE, B_SPLINE_SURFACE_WITH_KNOTS
  (simple, complex, rational, Bezier / quasi-uniform); LINE, CIRCLE, ELLIPSE,
  B_SPLINE_CURVE (same forms), TRIMMED_CURVE (bounds come from the vertices),
  SURFACE_CURVE / SEAM_CURVE / INTERSECTION_CURVE (3D curve used).
- units: first LENGTH_UNIT / PLANE_ANGLE_UNIT (SI prefixes, CONVERSION_BASED_UNIT
  e.g. INCH, DEGREE) -> mm / rad.
- topology completion for periodic surfaces: STEP bounds a cylinder by two
  bare circles; the importer adds the seam edge (splitting a circle at the
  seam position when the two circles' vertices are not aligned), poles for
  single-boundary cone / sphere faces, and merges split full spheres / tori
  back into one face (`[S,S^-1]`, resp. both seams of the torus). Degenerate
  (pole / apex) edges are dropped. Closed B-spline profiles unrolled over
  several laps are handled (smallest period, first pass through a vertex).
- errors: `ImportError { code, entity, message }`, codes `parse`, `badEntity`,
  `unsupported`, `topology`, `noSolid`. Never panics on malformed input
  (fuzz-truncation test).
- not supported: assemblies / placements (SHAPE_REPRESENTATION_RELATIONSHIP),
  shell-based surface models / open shells as solids, POLY_LOOP / VERTEX_LOOP
  bounds, hyperbola / parabola / composite curves, faces whose boundary winds
  twice around a periodic surface.

`importStep` request (extension): `{type:"importStep", tag?, name?, data}`
-> regen-style result (`body`, `errors` as `["import",{code,params:{entity,message}}]`,
`import:{solids,faces,unitScale,warnings}`); the tag's state becomes the
imported body (massProps / inspect / validate / step / mesh work on it). A
failed import keeps the previous state.

Round trip (`tests/roundtrip.rs`, `tests/import_fixtures.rs`,
ck-wasm `tests/import_step.rs`, gate metric `import.roundtrip`): our `step`
export -> import: volume / area within 1e-6, same face and (non-seam) edge
counts, `validate` clean. Known exporter limit: a hole arc split at a seam
vertex but only half-used by a planar face (flangeHalf@9..11, flangeNut@12 in
brep_cases) is written as a closed circle; the import is then wrong.
## ACIS SAT import (`sat/`)

`ck_io::sat::import_sat(text) -> Result<SatImport, SatError>` builds a real
B-rep (ck-topo body, ck-geom surfaces/curves, mm). Origin: adapted from the
author's own SGRebuild `sat_import.rs` parser. Supported: plane, cone-surface
(cylinder/cone, circular), sphere, torus, spline-surface and intcurve-curve
with exact nubs/nurbs, straight/ellipse curves; face/coedge/edge sense,
single/double-sided (all-double = sheet), unit scale; ACIS seamless cylinder /
cone faces get a seam inserted. Unsupported (structured `SatError` with the
`$N` record): non-exact splines, periodic splines, elliptical cones,
torus faces whose loops wrap an axis, reflecting / sheared body transforms,
sphere bands whose loops share no azimuth vertex, mixed single/double faces.
Wire request `importSat {tag?, name?, data}` -> regen-shaped result (+
`import` summary); fixtures in `tests/fixtures/sat` (`expected.json` =
closed-form volumes, gated as `import.sat`).

Real-world dialects handled (checked on a 2002 ACIS 7.0 export and a
SolidWorks 2015 / ACIS 22 export): CR-only / CRLF line endings, `-N` record
index prefixes, the extra id integer of ACIS >= ~20 (dropped at parse time),
`{ ref N }` shared spline definitions (N-th `{` block in file order),
tolerant `tedge`/`tcoedge`/`tvertex`, the real `cone-surface` layout, control
nets stored u-fastest (long-in-v nets are transposed so mass is fast), body
`transform` (rotation rows, p' = (p.M)*s + t; uniform scale; translation),
many bodies (one lump each). Sphere faces cut by circles around the pole
axis get a meridian seam to an implicit pole (the `pole_jump` coedge keeps
the loop's u-winding so mass integrates). Attributes, pcurves and other
unread records are skipped and counted in `SatImport::summary.skipped`; a
file with zero faces is a `sat.empty` error. The `importSat` reply carries
`import.{bodies,lumps,shells,faces,records,version,skipped}`.

Real files are NOT in the repo (licences/size): `tests/sat_real_files.rs`
reads `$SAT_SAMPLES_DIR/{dice.sat,svalinn/ErrorBlocks.SAT}` and skips when
absent.
