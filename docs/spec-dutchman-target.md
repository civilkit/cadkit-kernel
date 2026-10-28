# Spec: a `.ckpart` target for Dutchman

Status: draft for the owner. Nothing here has been sent to the Dutchman project.

## Why

[Dutchman](https://github.com/dutchman-cad/dutchman) (Rust, Apache-2.0, early development) converts native CAD files into an
open, editable design: Fusion (`.f3d`, `.f3z`), Inventor (`.ipt`, `.iam`), SolidWorks parts, FreeCAD, CATIA, Rhino, ACIS and STEP
today, and it keeps the feature history when every feature maps. Its only real target so far is SolidCraft (`.solidcraft`); a
second, experimental target writes a FreeCAD macro. A `.ckpart` target would let all of those formats open in CadKit with an
editable history. Today the indirect route already works (Dutchman writes `.solidcraft`, our SolidCraft importer reads it), but it
depends on two lossy steps and on SolidCraft's format staying put.

## What Dutchman gives us

- Pipeline: native file, then cadmpeg decode into CADIR (a neutral model with exact B-rep, feature history, sketches, parameters,
  appearances, assemblies), then a `Writer`, plus a `Report` of losses.
- The seam is one trait: `Writer { fn extension(&self) -> &'static str; fn write(&self, model: &Model, out: &mut dyn Write, report: &mut Report) -> Result<(), Error>; }`.
- Crates: `dutchman-core` (formats, decode, `Writer`, `Report`), `dutchman-solidcraft` (the main writer), `dutchman-freecad`
  (a second writer, a good template), `dutchman` (CLI and MCP server), `dutchman-gui`.
- Contribution rules: fork and branch from `main`; one topic per pull request with tests; update the docs it touches; add a
  `changelog.d/` fragment; CI must pass (fmt, clippy `-D warnings`, tests on Linux, Windows and macOS). Test files must be
  synthetic (built in code), with their origin written in the fixtures README. A clean-room rule bars material from other CAD
  programs' SDKs or sources.

## The target

A new crate `dutchman-ckpart` implementing `Writer` with extension `ckpart`, selected with `--to ckpart` (as `--to freecad`
works today). It writes one JSON document: `{"format": "ck-part", "version": 1, "features": [...]}`, per the part-format book
(`book/src/format`).

Geometry that has no history (STEP, ACIS, Rhino, a Fusion file whose history does not fully map): a part cannot embed imported
B-rep as a feature (`importStep` is a request, not a feature), so the writer produces a STEP file beside the output (the
cadmpeg STEP encoder is already used by the SolidCraft writer) and reports that no history was kept. The app opens that STEP.

## Mapping (CADIR to ck-part)

| CADIR | ck-part | Notes |
|---|---|---|
| sketch on a plane or a face | `sketch` with `plane` std, `feature` or `face` | points, `line`, `arc` (CCW from p1 to p2), `circle`; constraints from `sketches.md` (coincident, horizontal, vertical, parallel, perpendicular, tangent, equal, distance, radius, angle) |
| extrude (blind, symmetric, through, to face) | `extrude` with `end` blind, mid, through, upToFace; `draft`, `thin`, `op` add, cut, newBody | cap and wall names `C0`, `C1`, `S.<entity>` |
| revolve | `revolve` with `axis` a line of the sketch (or `{sketch, id}`) | |
| sweep, loft, coil | `sweep`, `loft`, `helix` plus `sweep` | only what the book lists |
| hole (simple, counterbore, countersink, tapped) | `hole` | thread label to `thread` |
| fillet, chamfer (constant) | `fillet`, `chamfer` with edge picks `{"faces": [a, b], "at": [x, y, z]}` | variable fillet via `radii` |
| shell, draft | `shell`, `draft` | faces by name |
| patterns, mirror | linear and circular pattern, mirror features | features or bodies |
| parameters | part parameters and expressions | evaluated in CADIR order, forward references allowed |
| appearance | `appearance` | |
| components, joints | `insertPart` with the embedded part, `mate` | |
| anything else | a suppressed placeholder feature and a `Report` loss entry | never a failure of the whole file |

Face and edge names are derived from how geometry was made (the naming table in `features.md`), so the writer computes them by
following the table; an edge pick needs the two face names and a point on the edge from CADIR topology. If a name cannot be
derived, the feature is skipped and reported, like an unmapped feature.

## Verification

Same idea as Dutchman's SolidCraft check: after writing, regenerate the part in our kernel and compare volume, area and bounding
box with the CADIR (or STEP) properties within 1e-3; keep the history only if they agree, otherwise fall back to the STEP-only
output and say so. The kernel is Rust, but its crates are not published, so the check runs through the `ck_wasm` module with a
small script (as the book's `book/tools/run.mjs` does), or through the native service; CI for this job is informational, like
Dutchman's SolidCraft round trip.

## Phases

1. Spike (S to M): boxes, plates with holes, extrudes and cuts from synthetic Fusion and FreeCAD fixtures to `.ckpart` plus a
   STEP companion; volume check in a script.
2. Core history (M): sketches with constraints, revolve, patterns, mirror, fillets and chamfers, parameters.
3. More (L): sweeps, lofts, coils, assemblies with mates.
4. Upstream (S): `docs/ckpart.md`, README and `dutchman formats` row for the target, golden tests, a `changelog.d/` fragment,
   then the pull request.

## Decisions for the owner (nothing is sent before they are made)

1. Open a proposal issue on Dutchman first, then fork and send the pull request? A new target is bigger than the one-topic
   changes their guide describes, so an issue is the polite start. Both are public actions under the owner's account.
2. The public name and extension of the format in their README (`.ckpart`, format `ck-part`).
3. Licence: contributions there are Apache-2.0. Nothing derived from GPL material (for example the NopSCADlib-based motor
   library) may go into the crate or its fixtures.
4. Keep the SolidCraft route as the fallback until the direct target covers the cases the importer handles.

## Risks

Dutchman and cadmpeg are early and move fast (the cadmpeg commit is pinned). Our edge naming must match what the kernel
resolves. The CADIR to feature mapping for less common features (sheet metal, class-A surfaces) has no ck-part equivalent and
will always report a loss.
