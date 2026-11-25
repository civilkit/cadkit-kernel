# Fusion 360 import (`ck-asm`, request `importF3d`)

`crates/ck-asm` reads Fusion 360 `.f3d` documents and `.f3z` packages, or one bare `.smb` / `.smbh` ASM body blob, into
`ck-topo` bodies. The reader of the ASM B-rep is MitCad's (MIT, commit 806a421, see `crates/ck-asm/NOTICE` and
`THIRD_PARTY.md`); what is ours is the container, the Zstandard decoder, the builder into `ck-topo` and the report.
No Autodesk library is used and the crate has no dependency outside the workspace (it replaces MitCad's `ruzstd` dependency
by its own decoder, `src/zstd.rs`, so nothing has to be fetched, audited or compiled for wasm).

```
{type:"importF3d", tag:"t", name:"part", data:"<base64 of the .f3d / .f3z file, or of one .smb / .smbh blob>",
 opts:{history?:bool, allBodies?:bool}}
```

The reply is regen-shaped like `importSldprt`:

| Key | Meaning |
|---|---|
| `body` | the valid closed solids of the file, one lump each (the tag's body) |
| `surfaces` | `[["import.P<k>", body], ...]`: every other ASM body (a sheet, or a body with a face the reader could not make) as the surface body of the faces that came in |
| `errors` | `[["import", {code, params:{message}}]]` when the file cannot be read (`f3d.container`: not a zip or a bad entry; `f3d.asm`: not an ASM blob; `f3d.empty`: no body blob in the file); the tag is left alone |
| `warnings` | the `importStepParts` warning of the other imports when a body is not a solid |
| `import` | `format:"f3d"`, `name`, `container` (`zip` / `blob`), `unitScale` 10, `blobs[]` (`entry`, `guid`, `history`, `asmVersion`), `bodies`, `solids`, `surfaces`, `refused`, `faces`, `facesTotal`, `parts[]` (as `importStep`: `label`, `outcome`, `faces`, `facesTotal`, `surface?`, `reasons[]`), `skippedFaces[]` (at most 200: `body`, `face`, `code`, `message`) and `report[]` |

`import.report` is the report in plain English, one line each: the summary ("Read 2 body blob(s) holding 145 ASM
bodies: 63 valid solid(s), 82 surface bodies ..."), the faces left out by kind of surface or curve, every body that did
not come in as a solid with the reason, the tolerances and seam splits applied, and a note for a blob of the untested
`BinaryFile8` variant. Face names are `f3d.b<blob>r<record>f<n>` (`n` = the face's index in the reader's body).

## What is read

Surfaces: plane, cylinder, cone, sphere, torus, B-spline. Curves: line, circle, ellipse, B-spline (the `intcurve` of
the file, exact when it stores a B-spline). Units: the file is in centimetres, bodies come out in millimetres (x10). A face
on any other surface (extrusion, revolution, ruled, rolling-ball blend, offset, unknown) or with an edge on any other
curve (helix, interpolated, curve on a surface) is left out, never approximated, and the body is then reported as a
surface body with the reason, because it is not a closed solid. So are the faces that do not build consistently: a
vertex more than 0.01 mm off its surface or curve, loops that cannot be joined over a seam, a B-spline surface closed in
u or v (its seam is not made), an edge across the seam of a periodic B-spline curve. Nothing is ever kept as a solid that
is not one (`ck_heal::honest`).

ASM stores periodic faces without seam edges, so the seam is made here (as the SAT importer does): a line between the
two loops that wrap a cylinder or cone, a meridian between those of a sphere or torus, the seam to the apex or pole for a
cap. Where the two loops have no vertices at a common angle, an edge is split there (shared with the neighbouring faces).
Edges and vertices whose file geometry deviates from itself by more than a micrometre get a tolerance
(`ck_heal::tolerant_only`), as in any native import.

A `.smbh` (history) blob holds the bodies of the design as it stands; the plain `.smb` blobs of the same document keep the
bodies of earlier timeline steps. By default only the design's own bodies are read: the top-level bodies of the document's
history blob(s), or, for a document without one, the top-level bodies of its plain blobs (bodies that only own a face or an
edge the design refers to are not parts either). The report says how many bodies were not read; `opts.allBodies` reads every
body of every blob, and then a history body that has an identical twin (same counts and box) in a plain `.smb` blob is read
once, `opts.history` reading all of them. The ASM
history itself (undo states) and the design (timeline, sketches, parameters) are not read.

## The corpus

`crates/ck-asm/tests/corpus_all.rs` reads every `.f3d` / `.f3z` under `$CK_F3D_CORPUS` (62 third-party files, kept outside the
repository; the test passes when the variable is unset) with the app's defaults and holds the totals in
`tools/f3d_corpus_baseline.json` (`corpus`): solids built against the solids the reader finds, per read path
(`design` = the app's default, `all`, `history`), the faces built, the volume of the solids and of their mesh, the faces that do
not tessellate, and the design solids of each file as a floor. `CK_F3D_REPORT=<path>` writes the per-file report, with the
causes by surface or curve type of every reader solid that did not come out as a solid; `CK_F3D_LOST=1` lists them one by one;
`CK_F3D_UPDATE_BASELINE=1` rewrites the baseline.

What is left out, by cause, is in the report of each import. Still not imported: ruled and arc-sweep surfaces and helix
edges (threads), a self-intersecting torus (minor radius not below the major), a closed B-spline surface whose face crosses
or wraps its seam (other than a revolved cap, which is closed through its pole), and edges off their end vertices or faces
off their surface by more than 0.05 mm.

## Known gap: BinaryFile8

Only Windows-written files (`ASM BinaryFile4`, ASM 232.5) exist in the corpus. The macOS writer produces `BinaryFile8`
(8-byte integers); the reader has the code for it from MitCad but nothing here has been tested on such a file. The report
says so when it meets one.

## Corpus and gate

Three real files (Windows, `BinaryFile4`; third-party, so not in the repository): `1WireRGBWController.f3d` (Zstandard
entries, method 93) and the packages `f3d/F30a.f3z`, `f3d/F30b.f3z` (deflate), from the `cadmpeg` corpus. MitCad's
reader gives, over all blobs: 6 blobs, 153 bodies (67 solids), 1951 faces, 0 skipped, 0 issues. `ck-asm` gives the same
and builds every face (`crates/ck-asm/tests/corpus.rs`): every body builds, the 67 solids are valid with positive volume
(the whole file's solids: 652,276.5 mm^3, 10,000 mm^3 and 10,000 mm^3), a body is a solid exactly when MitCad's reader
says so, and the solids' mesh has no failed face and encloses the same volume to 0.2 %. The numbers, with the volume of
each solid, are in `tools/f3d_corpus_baseline.json` (`CK_F3D_UPDATE_BASELINE=1 cargo test -p ck-asm --test corpus`
rewrites it).

The test finds the files in `$CK_F3D_CORPUS`, else `crates/ck-asm/tests/corpus-local/` (same layout, gitignored, never
committed). With neither it prints `skipped: <file> not
found` and passes. **How the gate gets them:** `tools/remote_build.sh` syncs the untracked work tree to the box, so a copy
in `crates/ck-asm/tests/corpus-local/` reaches the CI box with the quick tier (and any other `remote_build.sh` run); a
worktree without it skips the corpus tests there. The full gate on `main` runs from a checkout that does not have the
directory: put the three files in a directory on the box and set `CK_F3D_CORPUS` in its `ci.env` (or pass
`--env CK_F3D_CORPUS=<dir>` to `remote_build.sh --cargo-test`), otherwise the corpus tests skip.

Tests of the crate that need no corpus: the vendored reader's own 90 tests (synthetic ASM blobs), the zip and
Zstandard decoders (fixtures made with the `zstd` tool, a flipped-byte fuzz that must never panic), and the builder on a
cube, a cylinder (seam), a cone with its apex and a body with a face of unknown geometry (`tests/synthetic.rs`); the wire
request in `crates/ck-wasm/tests/import_f3d.rs`.

## Phase B crates (design import)

`ck-f3d-dump` (the design dump IR, `Dump` and its records, JSON through serde; schema in `crates/ck-f3d-dump/SCHEMA.md`) and
`ck-f3d-file` (the document inside the `.f3d` / `.f3z` zip: `F3dFile::from_bytes`, segments, body blobs, display meshes
in `ogs`, appearance packages in `protein`, mesh bodies in `paramesh`) are MitCad's `dump` and `f3d-file` crates, adapted
(see their NOTICE files and THIRD_PARTY.md). They are the first crates with dependencies outside the workspace:
`serde` and `serde_json` through `[workspace.dependencies]` (`serde.workspace = true`), pinned by the committed
Cargo.lock. `ck-core`, `ck-geom`, `ck-topo`, `ck-wasm` and every other crate still have none, and ck-wasm links the new
crates only when it depends on one. `ck-f3d-file` has no zip dependency either: its `zip` module is MitCad's zip API on
ck-asm's decoders, and `F3dFile::open(path)` is not built on wasm32 (use `from_bytes`). The corpus tests of
`ck-f3d-file` (`tests/corpus.rs`) read the same untracked files as ck-asm's (`CK_F3D_CORPUS`) and skip when absent.

`ck-f3d-names` (names of the B-rep entities in the ASM body blobs and the entities that feature inputs name: `NamedState`, `EdgeTail`, `surface` point-in-face; MitCad's `f3d-names`, see its NOTICE) depends on `ck-asm` and `ck-f3d-dump` only.
`ck-f3d-decode` (MitCad's `f3d-decode`: `stream` for MetaStream.dat and the object framing of BulkStream.dat, `classes`,
`decode`, `sketch` and `unicode` for the records and sketches, `recipe` and `curve` for the entity recipes and stored
curves, `mesh`, `learn`, and `writer` for synthetic streams in tests) depends on ck-asm, ck-f3d-dump, ck-f3d-names and
serde_json; its tests are inline.
`ck-f3d-design` is MitCad's `f3d-design` crate (the design segment decoded into the dump IR). Its entry point from bytes is
`ck_f3d_design::decode_bytes(bytes, "name.f3d")` (an `.f3z` is told by the name): one `FileDesign` per design, each with
`dump: Option<ck_f3d_dump::Dump>`; `package_from_bytes`, `decode_document` and `resolve_inputs` are the pieces. The
path-based `decode_path`, `package` and `documents` are not built on wasm32. The inputs and appearance passes use
`std::thread::scope` with a fallback to the calling thread when a thread does not start, so wasm32 (no threads) runs them
serially. It depends on `ck-f3d-names` and `ck-f3d-decode`, whose `scan-count` feature its tests turn on.

## Feature history (`ck-f3d-features`)

`timeline_to_features(&Dump) -> FeatureImport` maps a design's timeline to a ck-part feature list (`part()` gives
`{"format":"ck-part","version":1,"features":[...]}`): sketches (points, lines, arcs, circles, ellipses, fitted splines;
on a standard plane or a `plane` feature given by its frame), construction planes, extrudes (blind, symmetric, through-all
cuts; join, cut, new body; profiles picked by `regions`, a hole's disc by an id of the hole), revolves (full turn or
symmetric), fillets of a constant radius and equal-distance chamfers. Everything else is a suppressed `placeholder` naming
the Fusion type and the reason; `items` carries one report per timeline item and `notes` the reasons.

- **Components.** A component placed once in the design is built where its occurrence is (the occurrence's `transform2`,
  rigid only); one placed several times (all placements rigid) is built at its first occurrence and its bodies are copied to the others with `moveBody` copies (the rotation and translation that take the first placement to the other, written after the last item of the design so the blends of other items regenerate without them); one with a scaled, sheared or mirrored occurrence, or with no usable placement, stays a placeholder. When bodies
  are made in more than one component each component's first body is named after it and its later joins and cuts act on it
  (`body` / `target`), so components do not merge (an add or cut of a component with several bodies targets all of them by name, so it cannot fuse with a body of a neighbouring component). A cut in a component that has no body of its own stays a placeholder.
- **Fillets and chamfers.** A Fusion edge is named by the file's own entity names, which a part cannot select, but it also
  carries its middle point and length. The history written so far is regenerated with the kernel, the edge with that point
  and length is found in the body, and the two faces that meet there (by the kernel's names) and the point are the pick.
  A fillet after an item that did not map stays a placeholder (the edges are not those of the file); so does one whose edge
  is not in the regenerated body. Variable-radius, full-round, asymmetric, two-distance and distance-and-angle blends are
  not mapped. The decoder gives no data for some chamfers (the item then says it names no edges).
- **Through-all cuts.** The decoder's `isPositiveDirection` flag 0 runs *against* the sketch normal (the schema says along
  it; the one design that verifies needs it that way, and the cuts that regenerated with `cutNoEffect` the other way were
  all this case). A distance given with a symmetric extent type is taken as the length to each side (Fusion's default
  measurement; not yet confirmed by a verified design, and the guard decides).
- **Face profiles.** An extrude whose profile is a planar face of the history (join out of the body with a positive
  distance, cut into it with a negative one, blind) is a `pressPull` of that face: the face is found in the history
  regenerated so far by the reference's point on the face and normal (so, like blends, only when every earlier item
  mapped). An extrude to a planar face (`ToEntity`) is an `upToFace` end with that face as `endRef`. Not mapped: new-body or
  intersect from a face, two-sided or symmetric, tapered, through-all from a face, to a non-planar entity or a chain.
  Extrudes that start away from the profile (an `OffsetStart`) stay placeholders: a part's extrude has no start offset,
  so it needs the sketch re-made on an offset plane.
- **Mirror and patterns.** Mapped since slice 6 where exact (see the slice 6 section); the part's mirror and pattern
  features take feature ids as seeds, while Fusion's inputs are bodies, faces or features by the file's own names.
- **Profiles.** A profile is picked by an id of its outer loop that only its region carries (`regions`); a region none of
  whose curves is its own (the lens of two overlapping outlines, a region whose every edge also bounds a neighbour) is
  picked by a point well inside it (`regionAt`, sketch-plane coordinates, an extrude / revolve field of the part: it picks
  the region or fill that has the point inside; a point in none is `extrudeRegionNone`). Regions that are all outlined by
  the same curve ids (a lens and its two crescents) cannot be told apart by the file's outline: when the file picks as many
  profiles with that outline as the kernel has regions with it, every one of them is picked; otherwise the item stays a
  placeholder (before slice 5 the last of them was taken, right or wrong).

`guard::verify(&import, reference)` regenerates the features with the kernel and compares volume and box with the ASM
reader's body (`guard::measure_import`): `Verdict::Unverified` with the deltas when they differ. A design is only
`Verified` when that check passes; a history with placeholders can still verify when the placeholders change nothing of the
final body, but none of the corpus does yet. 65 of the 124 designs are in `.f3z` packages whose bodies are read together
for all their designs, so they have no per-design reference and are never compared ("the imported body could not be built
to compare with"). Corpus numbers (aggregate, no names): `tools/f3d_features_baseline.json` (`CK_F3D_CORPUS`,
`CK_F3D_UPDATE_BASELINE=1`; `CK_F3D_DEBUG=1` prints why a profile outline is not found). Not wired into ck-wasm yet.

Corpus before and after slice 2 (6766 timeline items in 124 designs of 62 files; verified = regenerated volume within 0.2 %
and box within tolerance of the imported body):

| | slice 1 | slice 2 |
|---|---|---|
| items mapped | 1149 (17 %) | 3080 (46 %) |
| designs mapped completely | 4 | 10 |
| designs verified (of 43 comparable) | 0 | 1 |
| `.f3z` package designs (65, never compared): items mapped / fully mapped | 381 of 1273 / 4 | 552 of 1273 / 9 |
| single `.f3d` designs (43): items mapped / fully mapped / verified | 768 of 5493 / 0 / 0 | 2528 of 5493 / 1 / 1 |
| the five designs of 300+ items: items mapped | 332 of 3468 | 1641 of 3468 |
| Sketch / Extrude / Revolve / ConstructionPlane mapped | 720 / 204 / 10 / 215 | 1427 / 924 / 12 / 682 |
| Fillet / Chamfer mapped | 0 / 0 | 35 / 0 |

Slice 3 (same corpus; timeline items in 124 designs): multiply placed components, face profiles and extrudes to a face.
The `.f3z` / single `.f3d` split was not recomputed.

| | slice 2 | slice 3 |
|---|---|---|
| items mapped | 3080 (46 %) | 3381 (50 %) |
| designs mapped completely | 10 | 14 |
| designs verified (of 43 comparable) | 1 | 1 |
| Sketch mapped | 1427 | 1529 |
| Extrude mapped | 924 | 1025 |
| ConstructionPlane mapped | 682 | 780 |
| Fillet mapped | 35 | 35 |
| "placed N times" placeholders | 301 | 0 |
| "an earlier item did not map, so the faces ..." | 0 | 310 |

The face extrudes that would map (about 280) are mostly held back by the same rule as the blends: any earlier item that
did not map (390 + 310 items now wait on it). No new design verified: the multi-component designs that now map completely
differ from the imported body (the history builds fewer solids than the file, cuts that have no effect), which the guard
reports. What remains, by size: `BaseFeature` (439, an imported or direct-modelling body: the largest cause of "an
earlier item did not map") and Combine (187), Move (109) and CopyPaste (194), which change the body and so block every
later blend and face extrude; profiles that ids cannot select (118); `FixedSpline` and control-point splines; blends after an item that did not map
(400, they follow the rest); Move/Joint/Occurrence/CopyPaste items (they change the final placement), mirror and patterns,
combine, shell, hole, loft, sweep. Items that do not map keep their placeholder and note, and the app shows the history in
a folder next to the imported body, which stays the reference.

Slice 4 (same corpus): verification first. The numbers above said "1 verified" because the guard compared only a *single*
design file; the 65 designs of the `.f3z` packages were never compared ("the imported body could not be built to compare
with"). A package's designs are separate documents, each with body blobs of its own, so `guard::measure_documents` now reads
each document of a package on its own (in the order `decode_bytes` gives the designs) and every design has a reference of
its own. A reference with no closed solid is "no reference" (before, an empty body made every delta infinite).
The second cause was the profile mapping: a profile the file builds of several regions (a chord across a region
splits it in two) carries them as `children` beside the merged `loops`; the merged outline matched only one kernel region
(the best partial match), so the extrude lost the other, and the volume came out short. Each child is now a profile of
its own.

| | slice 3 | slice 4 |
|---|---|---|
| designs compared (a reference body exists) | 43 | 96 |
| designs verified | 1 | 13 |
| fully mapped designs | 14 | 12 |
| fully mapped designs verified | 1 | 10 |
| items mapped | 3381 | 3359 |
| Extrude mapped | 1025 | 1003 |

Why mapped fell by 22: those extrudes were mapped to one region of a profile of several, wrongly (the volume was short);
now each region is found on its own, and one with no curve that no other region has stays a placeholder ("shares all its
curves with other regions", 119 -> 144). Two fully mapped designs went with them.

Why the 14 fully mapped designs of slice 3 did not verify (13 of them were package designs, never compared): the 13
package designs were not compared at all; of the 12 fully mapped now, 10 verify. The 2 left are the same design (found in
two packages): its last item is a cut of a planar face with a negative distance that the kernel's press-pull cannot build
(`moveFaceFailed`), although the body before it already has the volume and box of the imported one (the file's cut removes
nothing there, or the face's direction is the other one). Designs with placeholders still do not verify because the
placeholders change the body.

Unverified reasons now (of 124): history builds no body 18 (nothing mapped builds one), `cutNoEffect` 14 (a cut that
misses the body: a body that is missing, or a cut targeting the wrong one in a multi-body design), volume short 28 and
volume over 14 (items left out), no reference 12 (design has no closed solid), `booleanFailed` 3, `booleanTangent` 2,
`moveFaceFailed` 2, `blendFailed` 1, `profileCrossesAxis` 1.

`lost`: items that only place or copy whole components (`ComponentFromBodies`, `CopyPasteOccurrence`, `DerivedInstance`,
`GeometricRelationship`) no longer mark the history as lost (they change no face or edge of a body); this unblocked nothing
in the corpus yet (the item before them is the cause). `BaseFeature` stays a placeholder and keeps `lost`: it holds a
direct-modelling body (the tool bodies of the `CombineFeature`s that follow are "not in the history state"), whose shape the
timeline does not give, so skipping it would build later joins and cuts on a body that is missing. `Move` changes where a
body is, and `Combine` joins, cuts or intersects bodies by the file's entity names, which a part cannot select: both also
stay.

Slice 5 (same corpus; branch `f3d-slice5`):

| | slice 4 | slice 5 |
|---|---|---|
| items mapped | 3359 | 3476 |
| Extrude mapped (of 1666) | 1003 | 1111 |
| Revolve mapped | 12 | 13 |
| Fillet mapped | 35 | 43 |
| designs compared | 96 | 96 |
| designs verified | 13 | 17 |
| fully mapped designs | 12 | 13 |
| fully mapped designs verified | 10 | 13 |

Unverified reasons (of the 91 designs with a timeline that do not verify, the 12 with no reference among them; slice 4 in brackets): history builds no body 15 (18),
`cutNoEffect` 16 (14), volume short 24 (28), volume over 14 (14), no reference 12 (12), `booleanFailed` 4 (3),
`booleanTangent` 2 (2), `blendFailed` 1 (1), `blendOverflow` 1 (0), `blendUnsupported` 1 (0), `profileCrossesAxis` 1 (1),
`moveFaceFailed` 0 (2). The "cutNoEffect" count rose and "history builds no body" fell because designs that built nothing now
build part of their body and stop on the next cut (items map that did not).

What changed:

- **A region with no curve of its own is picked by a point inside it.** The part's extrude and revolve take `regionAt`
  (`ck-model`, `extrude.rs`; `Region::contains` and `Region::interior_point` in `sketch/profile.rs`). The mapping gives a
  point only when exactly one region of the kernel contains it. This recovered the "shares all its curves with other
  regions" items (144 in slice 4: 0 now), mapped except where the outline is ambiguous (next point). A design that was 52 % short now verifies.
- **Ambiguous outlines are no longer guessed.** An outline equal to the outline of several kernel regions picked the last of
  them (right or wrong). It is now picked only when the file has as many such profiles as the kernel regions (all of them),
  else a placeholder (24 items now, they were mapped by luck before; none of the designs that verify depended on a guess).
- **The face-profile cut that failed `moveFaceFailed`** (the design that is in two packages). The file's cut is right as
  mapped, the kernel's press-pull is not: the face lies 1.4e-5 mm inside the opposite wall (the file's volume is the volume
  before the cut minus 1.2e-4 mm^3, 1.4e-5 mm x the face's 8.9 mm^2), so the cut removes a sliver the kernel cannot build
  (it refuses with a non-manifold edge). A cut of a face with no material 1e-3 mm behind it (the face's point shifted against
  its outward normal, tested with the kernel's containment) is kept as a suppressed `pressPull` with the reason in the note;
  the guard still compares the volume and box (both packages' designs verify). The direction was not the problem (reversing
  it would add 10 mm of material).
- **A sketch region the kernel lost** (`sketch/profile.rs`): a chord whose end overshoots the side it ends on by less than the
  vertex tolerance (here 6.6e-7 mm, a plate side a hair off vertical) left a stub shorter than a vertex past the crossing:
  a piece on a single vertex, which pruning does not remove and which broke the face next to it. The face was missing from
  the sketch's regions, so the extrude of it stayed a placeholder and the design came out 78 % over. Such a piece is now
  not made. The design verifies (a keyboard plate, 36 items, all mapped).

`cutNoEffect` (16 designs): traced for the five smallest, in each the cut is mapped as the file has it and misses because an
earlier item that makes or moves material did not map: a spline sketch and a Form / Thicken (the ghost keychain: items 3 to 6
and 9), a mesh base feature, a Move and an ambiguous profile (the keyboard plate), a fillet whose edge is not in the history, an
ambiguous profile and a Mirror (the robot), a sketch whose projected curves are not in the dump so its profile outline names
curves the sketch does not have (the nozzle brush: 47 items corpus-wide say "outline is not a region the kernel finds"),
an `ExternalDerived` item and a circular pattern (the hat). None is a mapping bug of an item that mapped; the next lever for
all of them is the item that did not.

Volume short / over: the designs with a single kind of placeholder left are a circular pattern (2 designs, 19 with others), a
Hole feature (1, +4.4 %: a simple hole mapped as a cut would settle it), a tapered extrude (the Choc keycaps: `draft` of the
part would take it, not mapped yet; +480 % because the tapered first extrude is a placeholder). The rest have several kinds
(Fillet 17, Occurrence 16, Group 14, Move 10, OffsetFaces 9, Chamfer 8 designs).

`BaseFeature` is not mapped, and is not a design-count lever: its 440 items are in 7 designs (418 of them in one), two of those
designs are a lone BaseFeature (the whole design is the direct-modelling body). What it needs: (1) a body feature for it in the
part (`importBody` reads only `.sldprt` now; an f3d format would have to carry the ASM blob and `ck-model` would depend on
`ck-asm`, which has no place in the part kernel's wasm today), (2) the ASM history state that holds the body the item made
(`ck_asm::history_states::StateWalk` walks the states of a `.smbh` back from the current one and says which bodies are the
same; the item to state link is only in the face names' `found ... before state N` strings, not on the item), (3) Combine and
CopyPasteBodies on bodies by the file's names, which a part cannot select. None can be done exactly in this slice, so
BaseFeature, Combine and CopyPasteBodies stay placeholders (and BaseFeature keeps `lost`).

Next lever: the items the five traced designs wait on, by count: circular pattern (36 items), fixed-spline sketches (85), the
projected curves a sketch has that the dump lacks (47 items of "outline is not a region"), Move (109), tapered extrude and Hole,
then a body source for `BaseFeature` as above.

Slice 6 (same corpus; branch `f3d-slice6`):

| | slice 5 | slice 6 |
|---|---|---|
| items mapped | 3476 | 3539 |
| Sketch mapped | 1529 | 1566 |
| Extrude mapped | 1111 | 1137 |
| designs verified (of 96 compared) | 17 | 17 (the same designs) |
| fully mapped designs / verified | 13 / 13 | 13 / 13 |
| `FixedSpline` curves not mapped (items) | 85 | 0 |
| `ControlPointSpline` curves not mapped (items) | 5 | 0 |
| rational splines not mapped (items) | 0 | 52 |

Unverified reasons moved by one design (history builds no body 15 -> 14, `cutNoEffect` 16 -> 17: a design now builds part of its
body and stops on its next cut). Nothing was marked verified without the guard; no guard threshold changed.

What was mapped, and what each is worth in the corpus:

- **FixedSpline and ControlPointSpline curves** (`sketch.rs`): the file gives each as a non-rational B-spline (control points
  in the sketch plane, degree, knots), which is the part sketch's `cvspline` with `degree` and `knots`; the control points are
  written as sketch points `<curve>_cv<k>`. A curve is refused when it is periodic or rational (52 items: the rational ones
  need weights, which `cvspline` has not), when its control points leave the sketch plane or its knots do not fit. This
  mapped 85 + 5 sketch items less the 52 now refused as rational, and the extrudes that wait on them (+26 extrudes). No
  design changed its verdict: the designs that gained splines still have other placeholders.
- **Tapered extrude** (`solid.rs`): the taper of the one blind side is the part's `draft` (degrees, inward) with `draftOut`
  for a negative angle; a second side, a symmetric or two-sided extrude, a to-face or through-all extent, and a tapered extrude
  of a face stay placeholders. 2 items in the corpus (the keycap design and its copy). The sign convention (a positive angle
  narrows, as in Fusion's dialog) cannot be checked by the corpus: the one design's reference body is a small part that the
  taper does not touch, so the mapped item says so in its note. Synthetic test: a frustum of known volume, both signs.
- **Circular and rectangular patterns, mirror of features and of the one body, Move of the one body** (`pattern.rs`): the
  mapping exists and its synthetic tests pass (a ring of four holes, a 3 x 2 grid by spacing and by extent, a mirrored hole, a
  mirrored body joined to the original, a body moved by a rotation and a translation), but **no corpus item maps through
  it yet**. Why, by count (items):
  - circular pattern 36: the gear-tooth pattern (an extrude and a fillet, 11 + 1) cannot be seeded, because the part's pattern
    repeats a feature's tool body and a fillet leaves none; 18 are in designs whose bodies are in several components (a
    pattern acts on the one body); 9 are patterns of bodies or faces by the file's names; 6 are symmetric.
  - rectangular pattern 13: bodies or faces (8), multi-component designs, and the rest wait on an item before them.
  - mirror 18: 12 are mirrors of bodies in designs with more than one body or after an item that did not map; the feature
    mirrors (5) are in multi-component designs or follow an unmapped item.
  - Move 109: every one is in a design with several bodies or after an item that did not map; a part's `moveBody` with no
    body named moves the whole body, which is exact only for a design with one body, so the others stay placeholders (the file
    names bodies by its own names, and a placement is not guessed).
  What each mapped item needs: a seed that is an extrude or revolve that mapped (found by the timeline index the file
  gives), the pattern axis from the decoder's `_f3d_axis` (origin and direction), a full turn, no suppressed or symmetric
  instances; a rectangular pattern's `directionOne` / `directionTwo` as vectors and its extent or spacing; a mirror plane's own
  geometry; a body mirror only when the file joins the image (`isCombine`) and the design has one body and nothing unmapped
  before it.

Not done, and why:

- **Projected curves missing from the sketch dump** (47 items): the dump's `counts` of a sketch agree with the curves it lists,
  so the missing curves are not in what the decoder reads from the sketch record at all (they are made by the projection
  and stored elsewhere); this is decoder work in `ck-f3d-decode`, not a mapping.
- **Hole feature** (12 items, 1 design +4.4 %): the dump gives a diameter, a tip angle, a distance and positions on a plane, but
  not whether the distance is measured to the tip or the end of the full diameter, nor whether `isDefaultDirection` runs
  into the body; guessing either would give a hole of the wrong depth. Left as a placeholder until a hole with a known
  result in the corpus settles both.
- **Rational splines** (52): need weights in the part's `cvspline`.

Next lever: the 11 gear-tooth patterns want a fillet-seeded or edge-pick pattern in the part; the multi-component patterns
(18 circular) want the pattern's `target` body naming in a design with several components; Move and body mirror wait on a body
list the part can select by the file's names (the same gap as `Combine` and `BaseFeature`).

Slice 7 (same corpus; branch `f3d-slice7`): bodies selected by the file's own names.

| | slice 6 | slice 7 |
|---|---|---|
| items mapped | 3539 | 3613 |
| CombineFeature mapped (of 187) | 0 | 6 |
| CopyPasteBodies mapped (of 66) | 0 | 16 |
| MoveFeature mapped (of 110) | 0 | 52 |
| designs verified (of 96 compared) | 17 | 17 (the same designs) |
| fully mapped designs / verified | 13 / 13 | 13 / 13 |
| "a move is mapped only for a design with one body ..." (items) | 109 | 1 |
| unverified, volume short / over (designs) | 24 / 14 | 22 / 16 |
| unverified, `cutNoEffect` (designs) | 17 | 17 |

No design newly verifies: every design that gained a Combine, a paste or a Move still has placeholders of other kinds (BaseFeature
above all). Nothing was marked verified without the guard and no threshold changed; the 17 verified designs are the same
17. (A first run of this slice had two designs stop on `missingReference`: a cut after a combine named the tool the combine
had consumed among its component's bodies. That was a bug of mine, found by the guard; the consumed names are now tracked and
the synthetic test `a_combine_inside_a_component...` covers it.)

**How a body is selected.** The decoder gives every body reference of a feature input `_f3d.producer` (the timeline index of
the item that made the body) and `_f3d.body_index` (its index among that item's bodies). `bodies.rs` uses that pair as the
file's name for a body. In a design that has a Combine, a CopyPasteBodies, or a Move whose inputs are bodies with a producer
(`Builder::named`), every body a mapped extrude or revolve makes is a named body of the part (`B<item>` in a design with bodies in
one component, the component's own names otherwise), a paste's copy is `B<item>` (`B<item>.<n>`), and `Builder::bodies` takes
(item, index) to the part's name and component. A feature selects through that table; a reference to a body the history did not make
(an item not mapped, a Loft / Split / BaseFeature / Box item whose bodies the part has no name for, a tool consumed by an earlier
combine, a suppressed one, one in another component) is a placeholder with that reason, never a guess. Before a combine, a
paste or a move is written the history so far is regenerated and every body it names must be exactly one lump: an extrude
of several separate profiles makes several lumps under one name, and (item, index) then selects none of them (7 items).
Designs without these features are unchanged (the naming is off), which is why the verified designs did not move.

- **Combine** (`bodies::combine`): join, cut and intersect (`union`, `cut`, `intersect` of the part's `combine`), the target
  selected by its (item, index), every tool likewise, `isKeepToolBodies` as `keepTools`; the result keeps the target's name and the
  consumed tools leave the table. A combine into a new component stays a placeholder. Synthetic tests: join / cut / intersect
  volumes of two overlapping plates, kept tools, a consumed tool named again, an unknown producer, a second index, a body
  with no producer, a body that is its own tool, a combine inside one component of a design with several.
  Corpus: 6 of 187 map. The rest, by cause (items): 71 + 68 + 15 + 8 + 4 name a body made by a CopyPasteBodies, BaseFeature,
  Extrude (not made by a mapped item), Split or Loft item; 4 name no target (the decoder gives none); at least 134 of the 187 are in
  the one design built on `BaseFeature` bodies, which cannot map until BaseFeature does.
- **Participant bodies of an extrude or revolve.** In a named design a join or cut acts on the file's participant bodies when the
  history names every one of them (`target`), else on whatever it touches as before.
- **CopyPasteBodies** (`bodies::copy_paste`): a `moveBody` with `copy` and no motion per source body (a paste is in place; the
  kernel documents that a copy with no motion still makes the copy), named `B<item>`. 16 of 66 map. 29 name no producer (the
  decoder has no ASM state for the item: "the item has no ASM state"), 8 are in another component than their source (the paste
  would need the occurrences' placement), the others copy a body of an unmapped item.
- **Move of bodies** (`bodies::move_named`): the matrix of the item (row-major, cm; rotation about the origin and then the
  translation, as for the one-body move of slice 6) as a `moveBody` on the bodies selected by name; a component placed away from the
  origin stays a placeholder. 52 of 110 map (slice 6: 0). The cadmpeg note (class `Move`, rigid-transform record: row-major
  4 x 4, centimetres, orthonormal basis, positive determinant = rotation about the origin then the translation) agrees with what the
  dump gives. No verified design depends on it (none of the designs with a mapped move verifies yet), so it is "consistent with the
  note" and not yet confirmed by a regenerate-and-compare pass.

**What the cadmpeg notes were worth** (`docs/formats/f3d.md` of the public cadmpeg project, CC BY 4.0, read as hypotheses; no
code or text copied; attribution in THIRD_PARTY.md):

- Combine operation codes: held in meaning (join / cut / intersect; the notes have two numberings by prologue, the dump already
  names them `JoinFeatureOperation` / `CutFeatureOperation` / `IntersectFeatureOperation`, so the codes themselves were not
  needed). The note that the target comes first and the tools in source order held (the dump lists them that way). The note
  that a combine of kept tools in the standard prologue is a Cut when no solid is deleted concerns decoding the record, not
  the dump (which already carries `operation` and `isKeepToolBodies` from the stored prologue). The note that a tool that is
  another component's body is not in the history state of the combine, so it is bound through the feature that generated
  it, matches the dump (`found: "the body is not in the history state"`, with the generating item as `producer`): the producer is
  the right key, and that is the key used.
- Move: row-major matrix in cm, rotation then translation: consistent, see above.
- BaseFeature result-body frames: **did not give a way in this slice**. The note says the result bodies of a Base Feature
  scope are a stored snapshot identified by the history-state transitions of the scope (its current and previous history-state
  identities are in the scope frame, three frame forms of 491, 707 and 711 bytes). Our decoder (`ck-f3d-decode`) reads none of
  that frame: the item's detail is `{}` for all 440 corpus items, so the item to ASM state link does not exist in our data
  (the only place a state number appears is the `found ... before state N` of a body reference, which is the state before a
  *later* item, not the one the base feature made). Reading those frames is decoder work, and even with the body the part has no
  feature that carries an ASM body (`importBody` is `.sldprt` only; `importStep` would round-trip the body through STEP text and
  could not be called exact). So `BaseFeature` stays a placeholder and keeps `lost`.
- Thicken: 0 items with that type in the corpus (the decoder gives no `ThickenFeature` item, so any Thicken is among the 61
  items of unknown type); the kernel has no thicken feature either. Coil: 1 item (a bottle cap's thread, a triangle section on a helix): the kernel's `coil`
  is a round-wire helix (`wireRadius`), not a triangular section, so it would be a different body; not mapped. Form (T-spline,
  among the 61 unknown-type items): not mapped, no data.

What remains, by size: `BaseFeature` (440 items, 7 designs; decoder frames + a body-carrying part feature), the bodies of
Loft / Split / Sweep / Box items (the part has no name for them: a feature that makes a body for the table), the components
(`Occurrence`, joints, `CopyPasteOccurrence`) and the multi-lump bodies of one feature.
