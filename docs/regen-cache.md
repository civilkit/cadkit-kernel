# Incremental regen (the regen cache)

The original kernel rebuilds the last edit of a 50-feature part in about a
second while the first load takes 18-25 s. It keeps the results of earlier
states and looks them up by content, so removing the last feature and
undoing that removal are both fast. Ours does the same.

Code: `crates/ck-wasm/src/regen_cache.rs` (cache, key, recorder),
`ck-model/src/regen.rs` (`regen_from`, `Resume`, `Checkpoint`),
`ck-model/src/features/mod.rs` (`BrepSnap`, `Brep::save/load`),
`ck-wasm/src/engine.rs` (the `regen` hook).
Checks: `node tools/regen_cache_check.mjs` and
`cargo test -p ck-wasm --test regen_cache`.

## What is cached

After every feature of a regen the cache stores the complete state a
continuation needs, under the key of the document prefix that produced it:

* the B-rep body with its persistent face names (names live in the body's
  faces), the tools of all body-making features (for mirror/patterns), the
  assembly state, helices, surface bodies (`BrepSnap`);
* the regen result so far: errors, planes, markers, sketches (with solved
  coordinates and profiles), sketches3, curves, id-keyed maps (`RegenResult`),
  plus the axis table.

Entries for failed features are stored like any other (the error list and
whatever the failing feature left in the body are part of the state), so a
replay reports exactly the same errors.

A regen looks up the longest cached prefix of the document (up to the
number of features it is going to run), restores that state and runs the
remaining features, storing a new entry after each. A full hit runs no
feature; the reply (tessellation, wire body) is then rebuilt from the
restored state.

`upTo` and numeric `rollback` just make the run shorter, i.e. a shorter
prefix. Suppressed features are skipped by the loop; they are still part of
the prefix JSON, so toggling `suppressed` changes the key from that feature
on. The regen tag (`main`, `preview`, ...) is NOT part of the key: a
`preview` regen of edited features reuses the states `main` built.

## The key

`keys[0]` is a SHA-256 over

* a format number and the kernel crate version,
* the effective regen options that change results: `sketchStatus`,
  `solveSketches` (which implies `sketchStatus`), `autoHeal` + `healTol`,
* the document-level fields the engine reads: `format`, `version`, `kind`,
  `material`, `variables` (raw JSON). Nothing else of the part object is read
  by regen (checked by grep), so `name`, `seq`, `counters`, `rollback` and UI
  keys are deliberately NOT in the key: renaming the part or bumping the
  feature counters keeps the cache warm.

`keys[i+1] = SHA256(keys[i] || json(feature i))` where `json` is the feature
object exactly as received (key order included) minus the cosmetic keys below.
Any other edit of a feature invalidates that feature and everything after it;
this is conservative on purpose (an unknown key stays in the hash). Hash equality therefore means an
identical prefix, and the hash is 256 bits, so collisions are not a concern.

### Cosmetic fields

`group`, `name`, `autoName`, `visible` and `userHidden` (top-level keys of a
feature object) are left out of the hash. They only label a feature in the
tree: putting rows in a folder (a folder is the `group` string), renaming,
hiding a plane. Editing them therefore keeps every key, a regen of the edited
document is a full hit (`cacheStats.fullHit`, `ranFeatures: 0`), and so is the
view snapshot (`viewHit`).

Evidence that nothing reads them (grep and compiler): no source file of
`ck-model`, `ck-ops` or `ck-wasm` names `"group"`, `"autoName"`, `"userHidden"`
or the key `"color"`; `"name"` and `"visible"` appear only in the parsers of
`Feature`, `Sketch`, `Sketch3` and the plane variant, and renaming those
fields makes no non-test code fail to compile (they are never read); the
expression evaluator only sees `variables`; refs are by feature id. Not
cosmetic: `suppressed`, `num`, `seq`, `rollback` and every unknown key (kept in
the hash on purpose; `tests/regen_cache.rs::other_keys_still_rerun`). Keys
nested inside a feature (`part`, `meta`, ...) are not touched, only the top
level of the feature object.

One place echoes them: the reply's `sketches[..].sketch` and
`sketches3[..].sketch` are the feature object (solved / normalised), so a
cached state carries the cosmetic keys of the document that built it. On every
hit `regen_cache::refresh_cosmetic` rewrites those echoes from the current
document (its key order, its cosmetic values, the normalisation's appended
arrays after them), so a hit replies byte for byte like a cold run
(`cosmetic_edits_keep_every_state_and_reply_like_cold`;
`cosmetic_keys_reach_the_reply_only_through_sketch_echoes` shows by diffing cold
replies that the echoes are the only place). Nothing else (errors, planes, body,
surfaces, threads, components) contains a feature label.

Keys of a document that has none of these keys are unchanged. For a document
that has them the keys differ from those of the previous build, which only
costs one cold run per kernel build (the key already contains the crate
version).

To add a cosmetic key: prove no code reads it, add it to `COSMETIC_KEYS`, and
if a reply could echo it extend `refresh_cosmetic`.

Not cached / bypassed:

* `opts.noCache: true` bypasses lookup and storage (the cache is untouched);
* `debug: true` requests (they want the real per-feature spans) bypass it;
* nested `insertPart` regens call the regen driver directly and do not use the
  cache (a nested part is memoised inside the assembly state as before).

`refDiff {base, part}` and `matchEdges {part}` do read the cache (they store
nothing). `matchEdges` continues from the longest cached prefix of its part.
`refDiff` finds the longest prefix the two documents share (the first changed
feature), takes the cached state just before it and rebuilds both documents
from there only, so a reference check after an edit of the last feature builds
one feature per side instead of two cold documents. The warnings are exactly
those of the full path; `checked` counts only the references from the first
changed feature on (an earlier reference is identical on both sides and cannot
warn). `noCache: true` on the request runs the reference path (both documents
from the start); `cacheStats: true` adds `cacheStats.resumedAt` (the number of
features that were not rebuilt). `tests/refdiff_cache.rs` compares the two
paths on edits, removals and moves. With a cold or evicted prefix it falls
back to the full path.

## Sharing and the cap

Snapshots share what a feature did not change: `BrepSnap` holds the body,
assembly, helices and surface bodies behind `Rc`, and a new snapshot reuses
the previous `Rc` when the piece compares equal (sketch/plane features do
not copy the body at all). Tools are `Rc<Tool>`. Cost per body-changing
feature is one body clone; see the measurements below.

The cache is bounded by an LRU over the *estimated* bytes of the unique
pieces held (`Body::est_bytes`, plus estimates for results, assembly and
tools; a piece shared by several entries counts once). Default cap:
`256 MiB` of estimate. Evicting an entry only drops that prefix; every entry
is self-contained, so any subset of entries is valid.

`opts.cacheCap: <bytes>` sets the cap of the instance (persists; `0`
disables storing; tests use tiny values to force eviction).

## Reuse statistics

`opts.cacheStats: true` appends a `cacheStats` object at the end of the
`regen` result (replies without the option are unchanged):

```
{"enabled": bool, "reusedFeatures": n, "ranFeatures": n, "fullHit": bool,
 "entries": n, "bytes": est, "cap": bytes,
 "lookups": n, "hits": n, "stored": n, "evicted": n}
```

`reusedFeatures + ranFeatures` is the number of features the regen runs
(`upTo`/`rollback` applied).

## Measurements

Release wasm, one `Kernel` instance, `node tools/regen_cache_check.mjs --bench`.

large sample (`tools/cases/largeSampleFull.ckpart`, 44 features, 23 MB reply):

| step | time | features reused |
|---|---|---|
| cold | 126 s | 0/44 |
| remove last feature | 0.87 s | 43/43 |
| undo (put it back) | 0.84 s | 44/44 |
| edit middle feature (22) | 16.6 s | 22/44 |
| edit feature 2 (errors early, little to rebuild) | 0.20 s | 1/44 |
| same edit again | 0.04 s | 44/44 |

The 0.85 s of a full hit is tessellation plus wire serialisation of the final
body (the features themselves run in 0 s). Mesh reuse for unchanged faces
could only trim that; it is not implemented.

50 BenchCAD parts, fresh instance each: cold with cache 15.7 s vs noCache
16.6 s (run-to-run noise is larger than the overhead; hashing and snapshots
cost well under 5 %), warm full hit 3.2 s (what remains is tessellation).

Memory: estimated cache state for the large sample's full chain is 30 MB
(wasm linear memory 167 MB after the cold run, 138 MB without the cache); 109
entries of various edits 40 MB. `--mem`-style loop on spur_gear with 80
different edits: cap 8 MB stays at 20 MB wasm memory with 109 evictions;
cap 256 MB grows with the estimate (wasm ~ estimate + 20 MB) until the cap
is reached, so worst case is roughly 300 MB of linear memory.

Why byte identity holds: a snapshot is the state after the feature, a restore
re-creates exactly it, and every later feature is a pure function of
(state, feature JSON, variables). Thread-local geometry memos (content-keyed)
persist across regens already; the check runs `noCache` regens in a used
instance against fresh-instance cold replies to guard against them.

## Display caches (view snapshot, face meshes)

The reply's `body` / `surfaces` snapshot (wire JSON plus display meshes) and the tessellation behind it are
cached too, because for a part like the dome cap they cost more than the features (4.5 s of 7.4 s cold).

* **Snapshot** (`view_cache.rs`): keyed by the final chain key, so an identical document, or one reverted to
  (undo / redo, slider back to a value seen before), replays the stored JSON and re-attaches the typed arrays to
  the new reply's buffer area in the pinned order (`positions, normals, mids, triFace, silhouette`, body before
  surfaces). The key covers everything the regen cache key covers, so the replay is byte-identical. Bypassed by
  `noCache` and by debug (trace) requests. Cap 96 MB.
* **Face meshes** (`ck_tess::mesh::tessellate_memo`): every face is keyed by the exact inputs of `mesh_face`
  (surface, sense, each live coedge's edge curve, pcurve, parameter range, final edge samples, seam flag, end
  vertices, periodic cut, tolerance) as a byte string compared exactly, so a hit is bit-identical to meshing.
  Adding or editing a hole re-meshes only the faces whose geometry or samples changed. Cap 128 MB, LRU.
  The body tolerance is rounded down to four significant figures (`body_tol`), because the box of a body with curved
  faces moves in the last bits with every unrelated edit and the periodic cuts (ties between equal gaps) are
  decided by that noise.
* `cacheStats` adds `viewHit`, `tessFaceLookups`, `tessFaceHits`, `tessEntries`, `tessBytes`, `viewEntries`,
  `viewBytes`. `cacheCap` scales the three caps together (regen states 1, faces 1/2, snapshots 3/8).
* `opts.noView: true` skips the snapshot altogether (`body` stays empty): for a client that already holds the view
  of this very document (a preview it is committing) and only needs the kernel's tag state and the errors.

Spline edges with a knot every fraction of a millimetre (interpolated intersection curves) used to be sampled once
per knot span; the sample list is now thinned to what the tolerance needs (`curvature::coalesce`), which cut the dome
cap from 92k to 22k triangles at the same chordal bound, and unconstrained edges whose midpoint fails the chord test
are flipped to the other diagonal before refinement (`Cdt::optimize_edges`).

## Where an edit's time goes (edit-latency bench)

`cargo test --profile fast -p ck-wasm --test edit_latency -- --ignored --nocapture` regens the fan shroud cold, then
edits the last fillet, F48 and F42 with the cache warm, printing wall time, instructions retired (macOS), reply size,
a digest of the reply without `cacheStats` (compare two builds with it) and the reuse counters. `CK_PART=<file>
CK_EDITS="F50:angle:1:0.9;..."` edits another document (feature id, numeric key, old value, new value);
`CK_LOOP=n` repeats the first edit with distinct values for a sampling profiler (`sample`).

The cache itself is not the problem: an edit of the last feature restores the previous state (one body clone) and runs one
feature. What an edit costs is the features after the first changed one (a prefix key cannot reuse a later feature, whose
input body changed) plus the reply. Before the tuning below, an F49 edit of the fan spent about 20 % outside the fillet
(face-cache keys built by formatting every surface and curve with `Debug`, the wire box and edge polylines recomputed for
every face, tessellation) and about 40 % of the fillet itself on validation of its own tool and result:

* `build_tool` integrated the tool's volume twice (the outward test of the orienting, then `mass_props` only to test
  `volume > 0`); it now takes the memoised coarse tier of `signed_volume_quick` once (exact to rounding on non-rational
  spline faces, exact fallback when the total cancels).
* the post-boolean volume check (`boolean::sanity`) integrated the operands first and then found that the result was not a
  closed solid (no verdict in both tiers); it now tests closedness first. Its coarse tier also accepts an operand much smaller
  than the other (a tool against the big trimming box), which used to fall through to the exact integrals.
* `boolean::apply` and the blend's validity test ran the full `check` (edge lengths, vertex and pcurve deviation of every
  edge of the whole result) and then kept only open / non-manifold / mispaired edges and broken loops; `check_topology`
  returns exactly those.
* the face-mesh cache key is a word list of f64 bits hashed once, the wire box of each curved face and the polyline of each
  free-form edge are memoised by content (`ck_topo::facekey`).

Results are unchanged (reply digests of the fan and large sample edits are identical before and after).
Native instructions (G) before -> after: fan edit F49 3.61 -> 1.83, F48 10.95 -> 6.94, F42 20.35 -> 16.08, cold 44.6 -> 38.2;
large sample edit F50 1.75 -> 0.82, F43 9.55 -> 8.15. What is left of an F42 edit is the features themselves
(curve-curve intersection in `imprint::trim`, edge splitting in `imprint`, pcurve fits of new cut edges).

Content keys instead of prefix keys: a feature after the first changed one is rerun even when its inputs are unchanged
only if the body it receives differs, and then it must run. The one case a prefix key misses is an edit that leaves the body
identical (renaming a feature, a UI-only field): every later key changes with the JSON although the states are equal. Chaining
keys from a state fingerprint would catch that, but the stored state also holds JSON-derived parts (errors, sketches, planes)
so it needs a full-state comparison; not done.
