# Regeneration and caching

Regeneration rebuilds a part from its feature list: it runs the features in order, collects errors, and produces the body and the data a viewer needs. This chapter covers the driver in `ck-model`, the options that shorten a run, how errors behave, and the cache in `ck-wasm` that makes an edit near the end of a long part fast. The request and reply format is in [regeneration (protocol)](../protocol/regen.md).

## The driver

`ck_model::regen::regen_from` walks the part's `features` array in order. For each feature that is not suppressed:

1. It opens a work-budget scope for the feature (`opts.featureTimeMs`, default 120000 ms) and checks that the whole-document budget has not run out.
2. It runs the feature. Sketches are solved and their profiles found; reference planes and axes are evaluated; every other feature is passed to the geometry executor (`ck_model::features::Brep`), which resolves the feature's references against the current body, evaluates its expressions with the part's `variables`, and calls `ck-topo` or `ck-ops`.
3. On success the feature's result replaces the current body (or is added to the sketch, plane and curve tables). On failure an error entry is recorded and the body stays as it was.

Features are independent records: there is no dependency graph. A feature depends on earlier ones only through the names it uses (`sketch`, face names, edge picks, feature ids) and through the body that earlier features left.

The result has these top-level keys, in this order: `errors`, `components`, `planes`, `markers`, `sketches`, `sketches3`, `curves`, `body`, `surfaces`, and, when non-empty, `threads`, `limits`, `assembly` and `warnings`. The maps keyed by feature id (`errors`, `components`, `planes`, `sketches`, `sketches3`, `threads`, `limits`, `warnings`) are sorted by the id as a *string*, so `F10` comes before `F3`, and a second entry for the same id replaces the first.

## Which features run

| mechanism | effect |
|---|---|
| `opts.upTo = n` | Run only `features[..n]`. A non-number or a negative value is ignored. |
| `part.rollback = n` | The same, stored in the document (the rollback bar). When both are given the smaller wins. |
| `suppressed: true` on a feature | The feature is skipped as if it were not there. It still takes part in the cache key. |
| `opts.featureTimeMs`, `opts.regenTimeMs` | Work budgets; `0` means unlimited. A feature that runs out is `featureBudgetExceeded`; the whole document running out gives `regenBudgetExceeded` for the feature in progress and every later one.  (A host clock that advances by more than `featureTimeMs` between reads makes every feature, sketches included, fail with `regenBudgetExceeded` and the feature-budget hint; see [errors](../protocol/errors.md).) |
| `opts.sketchStatus`, `opts.solveSketches` | Add a solver status to each sketch, or re-solve every sketch from its coordinates. |
| `opts.autoHeal`, `opts.healTol` | After each feature that leaves validator issues, try a repair pass and keep the repaired body only if it has fewer issues; the feature gets a `healed` entry. That entry is written to `errors`, not `warnings`, and as there is one entry per feature id it replaces any real error of the same feature. Default tolerance `1e-5`. |

Measured with a box, a fillet that fails (F3) and a chamfer that picks the fillet's face (F4):

| run | errors | faces |
|---|---|---|
| all features | F3 `blendOverflow`, F4 `missingEdge` | 7 (F5 still adds a face) |
| `upTo: 2` | none | 6 |
| `upTo: 3` | F3 `blendOverflow` | 6 |
| `rollback: 2` | none | 6 |

## Errors

An error entry is `["F3", {"code": "blendOverflow", "params": {...}}]`: the feature id, a code, and parameters such as `detail`, `hint`, `max` or `faces`. The codes are described in [feature errors](../validation/diagnostics.md).

- **A failing feature produces no output.** Its result is not applied, and the body is as it was before the feature.
- **The run continues.** The following features execute against the unchanged body. A feature that depended on the failed one then fails for its own reason, typically `missingReference`, `missingEdge` or `missingSketch`. The reply does not link the two errors; there is no `blockedBy` field. The way to find the root cause is to read the errors in the order of the features (not the order in the reply, which is sorted by the id string) and to take the first one in feature order.
- **Failures that never reach the geometry.** `unknownFeature` (the type is not one the kernel knows, with `params.type`), `missingSketch` (the sketch id is not an earlier sketch), `insertPartMissing`, `badExpression`, and `invalidValue`.
- **A time budget** turns whatever failure happened while the limit was running out into the budget error, with the code the feature would have reported in `params.cause`.

The `debug: true` flag on a regeneration request adds a `trace` array of spans with the feature labels, for example `F3 fillet: blendOverflow {...}` with its nested `F3: Fillet(30.0, Circular) of 1 edge`, and one span for each stage of a boolean.

## The regeneration cache

Without a cache, every edit would rebuild the part from the first feature. The kernel instead keeps the states of earlier runs and resumes from the longest prefix that is still valid.

### What is stored

After a feature (every feature in a short run, every `stride`-th in a long one) the cache stores the complete state a continuation needs: the body with its face names, the tools of earlier body-making features (needed by mirror and patterns), the assembly state, the helices, the surface bodies, the regeneration result so far (errors, planes, markers, sketches with their solved coordinates, curves) and the axis table. States share what a feature did not change: a sketch or plane feature does not copy the body, and a new snapshot reuses the previous one when the piece is equal. Entries for failed features are stored like the rest, so a replay reports the same errors.

### The key

The key of the state after feature `i` is a chain of SHA-256 hashes:

```text
key[0]   = SHA256( cache format number, kernel crate version,
                   sketchStatus, solveSketches, autoHeal + healTol,
                   the document fields format, version, kind, material, variables )
key[i+1] = SHA256( key[i] || the JSON text of feature i without its cosmetic keys )
```

So the key depends on the whole prefix of the document up to that feature, on the options that change results, and on the kernel version. It does not depend on the tag (a `preview` regeneration reuses the states that `main` built), on `name`, `seq`, `counters` or `rollback` of the part, or on `upTo`; those only change how long the run is. Fonts used by text entities are part of the key by content hash.

Five keys of a feature are *cosmetic* and left out of the hash: `group`, `name`, `autoName`, `visible` and `userHidden`. Renaming a feature, moving it into a folder or hiding a plane keeps every key. Any other change of a feature, including keys the kernel does not know, changes its key and the keys of every later feature, deliberately: an unknown key stays in the hash because the cache cannot know it is harmless. `suppressed` is not cosmetic.

### Lookup, resume and reuse

A regeneration computes the chain of keys for the features it is going to run, finds the longest prefix with a stored state, restores that state, and runs the remaining features, storing new entries as it goes. A full hit runs no feature; the reply is rebuilt from the restored state. Measured through the `cacheStats` option on a box with a fillet:

| step | `reusedFeatures` | `ranFeatures` | `fullHit` |
|---|---|---|---|
| first run of the 3 features, after the box was seen before | 2 | 1 | false |
| same document again | 3 | 0 | true |
| fillet radius 2 changed to 3 | 2 | 1 | false |
| fillet renamed | 3 | 0 | true |
| same edit with `noCache: true` | 0 | 3 | false |

An edit to feature `k` re-runs features `k` and after, because a later feature may use a body that now differs. The cache cannot reuse a later feature even when its inputs happen to be identical. In particular an edit that leaves the body identical (changing a UI-only field that is not in the cosmetic list) still invalidates every later key.

### Checkpoints and limits

Storing a state after every feature of a long run would copy the result and the body each time, which is quadratic in the number of features. A run of more than 256 features therefore stores every `stride`-th state, where `stride = ceil(n / 256)` and `n` is the number of features the run executes, plus always the last one. Positions are absolute (`(i + 1) % stride == 0`), so a later run of the same document stores at the same places, and an edit resumes from the nearest checkpoint before it.

The cache is bounded by an LRU over the *estimated* bytes of the unique pieces it holds (`Body::est_bytes` plus estimates for results and tools); a piece shared by several entries counts once. The default cap is 256 MiB of estimate; with the display caches the linear memory can reach roughly 300 MB in the worst case. Evicting an entry only drops that prefix, since every entry is self-contained.

### Known limitation: budget failures are cached

A failure caused by a time budget (`featureBudgetExceeded`, `regenBudgetExceeded`) is stored in the cache like any other failed feature. A later regeneration of the same document on the same kernel instance replays the stored error instead of running the feature again, even if the budget is raised or the machine is less busy. Pass `opts.noCache: true` (or use a fresh instance) to force a real run. The options `featureTimeMs` and `regenTimeMs` are not part of the cache key.

### Options

| option (in `opts`) | effect |
|---|---|
| `noCache: true` | Bypass lookup and storage. The cache is untouched. |
| `cacheCap: <bytes>` | Set the cap of this kernel instance (persists). `0` disables storing. The display caches scale with it. |
| `cacheStats: true` | Append a `cacheStats` object to the result: `enabled`, `reusedFeatures`, `ranFeatures`, `fullHit`, `entries`, `bytes`, `cap`, `lookups`, `hits`, `stored`, `evicted`, `viewHit`, and the face-mesh counters `tessFaceLookups`, `tessFaceHits`, `tessEntries`, `tessBytes`, `viewEntries`, `viewBytes`. |
| `noView: true` | Skip the display snapshot: `body` and `surfaces` stay empty. For a client that already holds the view of this document and only needs the kernel's state and the errors. |

Requests with `debug: true` bypass the cache because they want the real per-feature spans. A nested `insertPart` runs its own regeneration directly and does not use the cache; the assembly state memoises nested parts.

### Display caches

The wire body (face and edge JSON, display meshes) is cached too, because for a large part it costs as much as the features. The *view snapshot* is keyed by the final chain key, so an identical document, or one reverted to (undo, a slider moved back), replays the stored JSON and re-attaches the buffers; the default cap is 96 MB. The *face meshes* are cached per face by exact inputs (see [tessellation](tessellation.md)); the default cap is 128 MB. Both are bypassed by `noCache`. A cosmetic edit gives a full hit and a view hit, and the reply is byte for byte the one a cold run would give.

### Queries that use the cache

`refDiff {base, part}` regenerates two documents privately to find references that now point somewhere else (see [persistent names](naming.md)). It finds the longest prefix the two documents share, takes the cached state just before the first changed feature and rebuilds only the rest of each. `matchEdges {part}` continues from the longest cached prefix of its part. Neither stores anything.

### Why the replies are identical

A snapshot is the state after the feature, a restore re-creates exactly that state, and every later feature is a pure function of the state, the feature JSON and the variables. Thread-local geometry memos are keyed by content and persist across regenerations; the repository's check runs `noCache` regenerations in a used instance against the replies of a fresh instance to guard against them.

### Cost

The repository's measurements were made on a 44-feature part with a 23 MB reply that is not published, so they cannot be reproduced from this book: about two minutes cold, under a second after removing or restoring the last feature, and tens of seconds after editing a feature in the middle. The 0.85 s or so of a full hit is tessellation and serialisation of the final body. What an edit costs is the features after the changed one plus that reply. Hashing and snapshots cost well under 5 percent of a cold run.

## See also

- [Regeneration (protocol)](../protocol/regen.md)
- [Persistent names](naming.md)
- [Feature errors and diagnostics](../validation/diagnostics.md)
- [Architecture](architecture.md)
- [Tessellation](tessellation.md)
