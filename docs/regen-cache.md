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
object exactly as received (key order and all UI fields included, so any
edit of a feature, even a rename, invalidates that feature and everything
after it; this is conservative on purpose). Hash equality therefore means an
identical prefix, and the hash is 256 bits, so collisions are not a concern.

Not cached / bypassed:

* `opts.noCache: true` bypasses lookup and storage (the cache is untouched);
* `debug: true` requests (they want the real per-feature spans) bypass it;
* `matchEdges`, `refDiff` and nested `insertPart` regens call the regen driver
  directly and do not use the cache (a nested part is memoised inside the
  assembly state as before).

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
