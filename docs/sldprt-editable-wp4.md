# WP4 design: keep the rebuilt prefix (`ck-swedit`)

Status: phases 1 and 2 implemented (`ck-model` `importBody`, `ck-swedit` `prefix.rs`); see "Implemented" at the end for
where the code departs from this design and the corpus numbers. Applies to `ck-swedit` after WP1 (revolve), WP2
(patterns) and WP3 (draft).

## Problem and numbers

Today the first feature that cannot be read back, or a regen error, or a failed verification anywhere throws the
whole history away: the file keeps its plain solid. On the local corpus after WP1-3 (45 importable files):
24 exact, 0 close, 21 fallback. Of the 21:

* 16 have a read-back prefix of at least one feature: 89 of their 358 features (25%) are read back and then
  discarded. 5 of them have half their tree or more (the longest: 20 of 35 steps).
* 5 have nothing to keep (the first modelling feature is an imported body (3) or a loft (2)).
* 1 reads back every step but regenerates with `missingEdge` on its last chamfers; a localised check keeps the
  steps before them.

So WP4 turns 16 of 21 fallbacks into "editable prefix + exact remainder", and every later WP5 fix lengthens a
prefix instead of only flipping whole files. The 24 exact files are unchanged.

## What a prefix is, and why there is no per-step reference

Each feature is read back from the faces it left on the *final* body, so the prefix body `P_k` (regen of the first
`k` features) is a history consistent with the surviving faces, not necessarily the body SolidWorks had after step
`k`: the corpus shows depths read back shorter than the listed `D1` where a later cut removed the top of a boss.
The file holds no intermediate states we can read: only the live partition is decoded, the `(deltas)` transmit is
not, the ghost partition is unused, and the stored mass properties are for the final body only. So there is
nothing to compare `P_k` with directly. Verification has two parts: local checks per step to choose `k`, and one
exact check on the composed final body.

## Decision: how the remainder is represented

Options considered:

| option | exact? | edits to the prefix reach the final body? | cost / risk |
|---|---|---|---|
| A. replace body at step `k+1` (the imported solid replaces the prefix body) | always: it is the import | no: the remainder is frozen | one feature; no boolean; never hangs |
| B. prefix + imported delta (`P_k` united with `F - P_k`, minus `P_k - F`) | exact only when the delta booleans succeed and verify | yes, locally | two booleans of near-coincident bodies at import and on every regen: the class of work that hung the boolean verify (the reason `fastverify.rs` exists); sliver solids where read-back depths differ |
| C. per-step verify | (not a representation: how `k` is chosen) | - | - |

**Decision: A is the representation that ships, chosen by C; B is a later, opt-in upgrade per file.**
Reasons: the project rule is "never ship a wrong rebuild", and A is exact by construction (the remainder is the
same reader's output for the same bytes). A gives the user the parameters and sketches of the prefix, roll-back to
any prefix step, and a final body to model on, at no robustness cost. B is real downstream editability but stakes
correctness on the hardest boolean case we have; it is worth doing only once A is in and we can measure on how many
files the delta booleans finish and verify within a budget.

Representation of A, using what the kernel already has:

* A new feature `importBody` in `ck-model`, the `importStep` pattern (blob key into the part's `blobs`, checked on
  every run, regen cache keyed on the feature JSON) with `format: "sldprt"` (blob: the part file, base64) and
  `replace: true` (the bodies so far are removed, as `deleteBody` does, then the import joins). Re-reading the
  original file with `ck-sldprt` keeps the remainder bit-identical to the plain import; a STEP round trip of the
  body would not be. Faces are named `<id>.I1F<n>` as `importStep` does, so features the user adds later can
  reference them.
* `ck-swedit` emits the prefix features, then `importBody` named "Imported remainder", carrying in its JSON the
  SolidWorks steps it stands for (`from`: the first unrebuilt step's name and kind, `reason`, `steps`: the names
  and kinds of the rest).
* `Outcome` gains `prefix: k`, `remainder: {mode: "replace", from, reason, steps}` and a per-step `check`. A
  file with `k = 0` stays the plain solid as today.

Shared crate touched (`ck-model`): the oracle gate must stay identical (a new feature type only).

## Verification per step (choosing `k`)

One regen of the candidate prefix with a body snapshot after each SolidWorks step (`regen_from`'s `Checkpoint`
hook; import plus read-back of the largest corpus file takes under 4 s today, so one extra regen is affordable).
For step `j`:

1. **Regen clean**: no feature error in step `j`'s features (localises `missingEdge` and similar).
2. **Settled faces**: every final-body face whose tags all are `<= j` (no later feature touched it) lies on
   `P_j`'s boundary: a face of `P_j` with the same surface (`fastverify::same_surface`) and the face's sample
   points (vertices, edge mid points, one interior point) inside that face (`point_in_face`). Faces tagged by `j`
   and by a later feature get the surface check only (the later feature may trim or extend them). No booleans.
3. **Volume sanity** where it is decidable: if every later step is a cut (or a fillet / chamfer that only removes
   material), `vol(P_j) >= vol(F)`; if every later step adds material, `<=`. Cheap and catches gross errors.

`k` = the last step before the first unsupported feature such that steps `1..k` all pass. A failing check is
reported on its step (`check: {regen, settled, volume}` with the first offending face), so a bad read-back is
localised instead of surfacing as a final-volume mismatch. With A the composed body is the import, so no final
verify is needed; the existing final verify still decides `exact` for files that rebuild completely.

For B (later): the delta solids are computed under a work budget (`ck_core::budget` with a clock installed in
`ck-swedit`; expiry or boolean error means "stay A"), the composed body is verified against `F` with
`fastverify`, and on failure the delta is retried at a bisected smaller `k` (at most `log2 k` tries, same budget);
no success keeps A at the full `k`.

## What the kit / UI should show (separate follow-up, per spec)

* Import summary: "Editable: k of n SolidWorks features", the reason the rest is imported (feature name, kind,
  note), and "exact" as now for full rebuilds.
* Feature tree: the prefix as normal features, then one "Imported remainder" node; expanding it lists the
  remaining SolidWorks steps greyed, the first one with its reason.
* Editing or suppressing a prefix feature with a `replace` remainder: a notice that the remainder is the imported
  solid and does not follow the change, with one action "remove remainder" (continue from the edited prefix).
  Roll-back above the remainder shows the prefix body as normal.
* Mode B, when it exists: the remainder regenerates like any feature; a failing delta boolean after an edit is an
  ordinary regen error on that node.

## Plan

| phase | work | effort |
|---|---|---|
| 1 | `importBody` (sldprt blob, `replace`), emit prefix + remainder, `Outcome` fields; synthetic tests: unsupported feature at step 3 keeps 2 steps and the exact body; `k = 0` stays plain; gate identical | 1-1.5 d |
| 2 | per-step snapshots and the three checks choosing `k`; tests: a regen error at step `j` and a deliberately wrong read-back depth are both localised to their step; corpus report gains prefix / remainder columns | 1-1.5 d |
| 3 | measure, then (optional) mode B with budget, composed verify and bisect; tests on a boss + pocket where the delta is known | 1 d, after phase 2 numbers |
| 4 | kit / UI tree and notices | separate follow-up |

Phases 1-2 deliver the 16-file gain on their own (the prefix lengths above are read-back counts; phase 2 may
shorten a few where a check fails, and the corpus report will show by how much).

Not in scope: rebuilding features after the first failure on top of the remainder (they would have to avoid every
face of the failed feature; the remainder already contains them).

## Implemented (phases 1 and 2)

* `importBody` (`crates/ck-model/src/features/import_body.rs`): `format: "sldprt"`, `blob` = key into `blobs` whose
  entry is the part file as base64 text (key: SHA-256 of that text, as `importStep`), `replace`, `body`; `from`,
  `reason`, `steps` are carried for the tree and not read. The file is read with `ck-sldprt` and sorted by the same
  function the `importSldprt` request now calls (`import_body::sort_sldprt`, moved out of `ck-wasm`), so the remainder
  is the plain import's body by construction. Faces `<id>.I<region>F<n>`. Error `importBodyInvalid`, warning
  `importBodyParts`. `ck-model` gains a dependency on `ck-sldprt` (which depends only on `ck-geom` / `ck-topo`, so no
  cycle; the wasm already linked it).
* `ck-swedit`: `Outcome` gains `prefix`, `remainder`, `checks` (one per step up to the failing one) and `ranges` (our
  features of each step); `Outcome::part_json(file)` returns the part document with the file in `blobs`;
  `Outcome::editable()`. A file that rebuilds to the end is unchanged (no snapshot regen, no checks). The features a
  failed read-back had already emitted are dropped. The prefix ends with its last rebuilt step (skipped / cosmetic
  steps after it go to the remainder; a prefix of such steps only is no prefix).

Departures from the design, found while implementing:

1. **Settled faces cannot require containment when a later step may add material.** The corpus has faces that a
   later coplanar boss grew while they kept only the earlier feature's tag, so "every sample point lies on `P_j`"
   rejected correct base extrudes on 6 files. Containment is required only when every later step removes material
   (or has no geometry); otherwise the face must overlap `P_j` (one sample point on a same-surface face). Wrong
   depths and wrong profiles are still caught by the surface match (a cap or side on another plane / cylinder).
   No interior sample point is used (vertices and edge mid points only); a face with no edges gets the surface check.
2. **Volume direction of blends.** A fillet or chamfer does not only remove material: a round in a concave corner
   adds. Fillets take their direction from the convexity of their strip faces (all convex: removes, all concave:
   adds); chamfers, patterns and unsupported kinds are unknown. The design's whole-remainder volume test rarely
   applies on real parts (the remainder holds a step of unknown kind), so a per-step test was added: a removing
   step must not grow the snapshot, an adding step must not shrink it.
3. **Every step passes but the whole does not verify** (no unsupported step, final verify fails): the design would
   keep every step with an empty remainder, i.e. a history known to be wrong somewhere. Implemented: no prefix
   (plain solid), noted in `notes`. Not hit on the corpus.
4. "Tags `<= j`" is tree order (position in the feature list), not id order; a tag of a feature the list does not
   name counts as later than every step.

Corpus (45 importable files): 24 exact (unchanged), 15 of the 21 fallback files now have an editable prefix and the
imported remainder (58 rebuilt steps kept of the 83 read back in those files; the design expected 16). The other 6:
3 imported bodies and 2 lofts (nothing to keep), 1 whose base extrude fails the settled-face check. Where a check
shortens a prefix it is a regen error of a read-back step (`noClosedProfile` 3, `missingEdge` 1) or a face of the
step missing from its snapshot (3). For every prefix the part document regenerates to the plain import (same faces,
volume and area to 1e-12; two files whose regions are no valid solid give the same surface bodies). Cost: one
regeneration of the prefix plus the checks, at most about 8 s on the largest file (the corpus test also regenerates
every composed document).
