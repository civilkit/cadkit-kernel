# D1: a panic is fatal on wasm, survivable natively

Status: proposal, nothing implemented. Sizes below marked "measured" come from builds in this repo; "estimate" is not measured.

## Today

- Native (`ck-native`, `--profile service`, `panic = "unwind"`): each job runs inside `catch_unwind`
  (`crates/ck-native/src/lib.rs`), so one bad request answers an error and the session and its caches live on.
- Wasm (`panic = "abort"` in `[profile.release]`): `ck-wasm/src/lib.rs` installs only a panic hook that reports the message
  through the `ck_panic` import. Then the module traps. `packages/kernel-client/src/wasm-host.ts` catches the trap,
  re-instantiates the module and returns `{ ok: false, trap }` for the request that died.
- What is lost on a trap: the whole linear memory. That is the regen cache (up to 256 MB of per-prefix states), the snapshot
  and face caches, the built-import cache, every sketch solve state. The document itself is held by the host
  (`packages/session`), not by the module.
- Stack overflow ("the kernel ran out of stack"), allocation failure and `unreachable` are traps too, not panics.

## Options

| | A. Host restart + replay | B. Checkpoint before each regen | C. Unwind wasm build |
|---|---|---|---|
| Idea | After a trap the host boots a new instance and replays what it needs: the document, then the last good request. The faulting request is not replayed. | Before each regen the module hands the host a serialised state, restored after a trap. | `panic = "unwind"` for wasm, `catch_unwind` around `ck_request`, state survives. |
| State kept | Nothing in memory; caches are rebuilt cold. Cost of the first regen after a crash is a cold regen of the document. | Bodies can be written (`ck_topo::codec`, with a SHA), but sketches, planes, the cache keys and `BrepSnap` sharing have no serialiser. Large (hundreds of MB worst case) copy out of wasm before every regen. | Everything, as native. |
| Wasm size | none | none (plus serialiser code) | estimate +5 to 15 percent for landing pads and unwind tables; not measured (see below) |
| Speed | none | a copy per regen, proportional to state size | estimate: about none on the non-panicking path |
| Toolchain | none | none | needs nightly `-Z build-std` and the wasm exception-handling feature; only stable 1.97 to 1.99 is installed here. Browsers need EH support (all current ones have it). |
| Covers | panics, stack overflow, OOM, any trap | panics, overflow, OOM | panics only. Stack overflow and OOM still trap, so A is needed anyway. |
| Risk | a poison request that re-traps on every replay | stale or inconsistent snapshot; large new surface area | `RefCell` and thread-local caches left half-updated after unwind (native already pays this and `catch_unwind` there is proven) |

Measured: the release wasm in the main worktree is 12.6 MB (unstripped, local build); the copy shipped with the app is 9.1 MB.
An unwind build could not be made here (no nightly), so option C's size is the one number in this page that is an estimate.

## Recommendation

1. Do A now and make it complete: the host keeps the last good document, reboots, replays that one cold regen, reports the
   faulting request as a `trap` with the feature id it was working on (the `ck_panic` text already carries the location),
   and marks that feature as the suspect so the next regen of the same document does not loop. Cost: zero bytes, zero
   speed. It is the only option that also covers stack overflow and OOM.
2. Keep native as is. Its `catch_unwind` is cheap and already works.
3. Reject B: it needs a state serialiser nobody has, to save the time of one cold regen after a rare event.
4. Revisit C only when wasm unwinding is stable in the pinned toolchain. Measure size and the regen gate time then; if the
   size is under 10 percent it is worth it for the speed of recovery (no cold regen), not for correctness.
5. Independent and cheap: K1 (no NaN panics in float sorts) removes the most likely trigger of traps in the first place.
