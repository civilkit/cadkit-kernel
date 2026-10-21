# Spec: pulling the CadKit kernel into CivilKit

Audience: an agent working in `~/Development/civil_engineering/spacegass/SGRebuild/`
(CivilKit). Goal: give CivilKit's connection/detailing code real B-rep geometry
(solids, STEP out, clash checks, exact takeoff) by driving this repo's kernel
(the "CadKit kernel", a Rust/WASM CAD kernel built from a JSON feature tree).

Status of this document: written from a survey of both repos, not from running
an integration. Items marked **(confirm)** are assumptions to verify first.
Do not modify the kernel repo from CivilKit; ask for changes (section 7).

## 1. What the kernel is

- Repo: `<cadkit>/rebuild` (Rust workspace, ~45k lines).
  Local only, no remote. Same machine as CivilKit.
- Product: `ck_wasm.wasm`, a self-contained WebAssembly module with one
  request/reply interface. It is deterministic (no clock/random needed), runs
  in browser, Node or a worker, and has no dependencies at runtime.
- Direction since 2026-10-12: an **LLM-drivable** CAD kernel. The JSON feature
  tree, persistent face/edge names and structured per-feature errors are the
  stable interface. Geometry internals may change; this interface will not
  without notice.

## 2. Build and load

```
cd <cadkit>/rebuild
cargo build -p ck-wasm --target wasm32-unknown-unknown --release
# -> target/wasm32-unknown-unknown/release/ck_wasm.wasm   (use --profile fast for quick local builds)
```

Reference host (copy it, do not import from the kernel repo): `tools/kernel.mjs`
(about 30 lines). Exports used: `ck_alloc(len)`, `ck_request(ptr,len)`,
`ck_reply()`, `ck_commit()/ck_commit_len()`; imports `env.ck_now`,
`env.ck_unix_ms`, `env.ck_panic(ptr,len)`. After a trap, re-instantiate the
module. In CivilKit run it in a **worker**, like `packages/sections-engine`
runs `sectionprops.wasm` (`sectionsengineworker.js`, `loader.js`, `client.js`
are the pattern to follow). Vendor the built `.wasm` into a package, e.g.
`packages/kernel3d/` (**confirm** name), with a `VERSION` file recording the
kernel commit hash.

## 3. Protocol

Request: JSON `{"id": n, "msg": {"type": "...", ...}}`.
Reply: `[u32 LE json_len][json][pad to 8][binary buffers]`. JSON is
`{"id": n, "ok": true, "result": {...}, "ms": x}` or
`{"ok": false, "error": "..."}`. Large arrays (meshes) are returned as
`{"$buf": ...}` references into the trailing buffers. See `tools/kernel.mjs`
and `crates/ck-server/README.md`.

Message types you will use:

| type | purpose |
|---|---|
| `regen` | `{tag, part, opts?: {upTo?}}` rebuild a part from its feature list. Result has `body` (box, faces, edges, mesh), `errors[]` (per-feature, structured `{code, params}`), `components`, `markers`, etc. |
| `massProps` | `{tag}` volume, area, centre of mass, principal moments/axes of the current body. Exact B-rep integrals, not mesh estimates. Optional `bodies:true` lists per-lump volume/area/com; optional `densities:[per lump]` adds per-body `mass` and a `byMass` total (implies `bodies:true`; length must match the lump count). |
| `step` | `{tag, name}` STEP text of the body for fabrication/detailing hand-off. |
| `validate` | `{tag, tol?, mesh?}` independent closed-solid check (it also tessellates, unless `mesh:false`, and reports `TESSELLATION_OPEN` for faces without triangles (with the tessellator's reason), analytic faces (plane, cylinder, cone, sphere, torus) whose mesh area differs from the exact area by more than 10 percent and by more than 5e-4 of the squared part size, and mesh edges used by one triangle that no other face's triangles cover to within the chord deviation (a hairline crack between two faces' samples of a shared edge is not reported); `meshNotes` lists faces whose trim segments the triangulator could not recover; `tools/tess_open_report.mjs` classifies the open edges over the corpus): `{ok, issues[{code, entity, detail}]}`. Run it on every generated solid. |
| `inspect`, `measure`, `interference` | face/edge details, distances, solid-vs-solid clash. |
| `importStep`, `importSat`, `heal` | **in progress** (landing soon after this doc): bring in outside geometry; repair near-valid solids. Check the kernel `README.md` status section before relying on them. |
| `drawing` | 2D view extraction. **Not implemented yet.** |

`tag` names a body slot in the session; use one tag per solid you are building.

## 4. The part format

A part is `{"format": "ck-part", "version": 1, "features": [ ... ]}`. Each
feature has `type` and a unique `id` (`F1`, `F2`, ...). Features regenerate in
order; later features reference earlier ones by id. Worked examples to copy
from: `tools/brep_cases.mjs` (~230 small parts) and the larger real parts
`../coupling-sample.ckpart`, `../bracket.ckpart`. Field tables per
feature: `crates/ck-model/src/features/README.md`.

Features CivilKit connections will use most:

| need | feature |
|---|---|
| plate outline | `sketch` on a standard plane (`{kind:'std', id:'top'|'front'|'right'}`) with `points[]`, `entities[]` (line, arc, circle), then `extrude` (`op:'add'`, `end:'blind'`, `depth`) |
| bolt holes / slots | `sketch` of circles on a plate face (`plane:{kind:'face', id:'F2.C1'}`) then `extrude` with `op:'cut'`, `end:'throughBoth'`; or the `hole` feature (ISO sizes, cbore/csink) |
| bolt group | `lpattern` / `cpattern` over the hole feature (`seeds:['F4']`, `spacing`, `count`) |
| plate edge treatment | `chamfer`, `fillet` (`edges[]` by persistent name) |
| stiffeners, ribs | `rib` or a second extrude |
| mirrored halves | `mirror` |
| members | extrude a section profile sketch along a member length, then position the part (see assemblies below) |
| weld prep, copes | extrude `op:'cut'` with a profile sketch |

Dimensions are in the part's own length unit, typically **mm** **(confirm with
the `step` export header and CivilKit's `packages/units`)**. Parameters may be
expressions/strings where the field table says `(expr)`.

Assemblies: parts can be inserted and mated into an assembly document
(components + `mate` features). See the flange example. Use this to place a
bolt, nut and plates into one connection.

### Persistent names

Faces and edges carry stable ids that survive parameter changes, e.g.
`F2.S.e1` (side face from sketch entity e1 of feature F2), `F2.C0`/`F2.C1`
(caps), `F4.S.e1@F1.1` (derived through a later cut). Use these (not
positions) when a later feature or a check must refer to "the top face of the
plate" or "the edge for the chamfer". Generate features that reference caps
and sketch-entity sides, and the names will remain valid when depth or hole
spacing changes.

## 5. Mapping CivilKit connections to feature lists

Pattern, per connection type in `rust_solver/fem-core/src/conn_*.rs`:

1. A generator function (JS in a new package, or a Rust fn in a new
   `fem-geom` module compiled into `fem-wasm` **(decide: JS is simpler)**)
   takes the same input struct the capacity check takes
   (plate thickness, width, bolt layout from `boltlayout.rs`, edge distances,
   weld sizes) and returns a `features[]` array.
2. Call `regen`, then `validate`, then `massProps` (weight = volume x density)
   and `step`.
3. Keep geometry and design check on **one input struct** so they cannot
   diverge.

Suggested pilot (2-3 days of agent work, no changes needed on either side
beyond a vendored wasm):

- Connection: **conn_gusset** (plate with bolt group, chamfered corners).
  Output: one solid, 3 STEP/weight numbers.
- Acceptance: for 5 parameter sets, `validate.ok`, volume equals the
  closed-form `thickness x polygon_area - holes x pi r^2 x thickness` within
  1e-6, STEP opens in a CAD viewer.
- Then extend to: web cleat, base plate with anchors (use `hole` + `cpattern`),
  splice plates (mirror), seat angle.

## 6. Where it feeds back into CivilKit

- `jointsolid.js` / `connection3d.js`: replace hand-built Three.js solids with
  kernel `body.mesh` (tessellation arrives as binary buffers, ready for
  `BufferGeometry`). Face/edge ids give you picking and highlighting by name.
- `connection_takeoff.rs` and the estimating package: exact volume/weight and
  STEP per connection instead of formula weights.
- Clash checking: `interference` between member solids and plates.
- Detailing/drawings: `drawing` once implemented.
- LLM authoring: the same feature JSON is what an LLM edits; the structured
  `errors[]` (for example `booleanFailed`, `nothingToCut`, `missingReference`)
  tell it what to fix.

## 7. Boundaries and requests

- Do not read or edit anything under the kernel repo's `.claude/worktrees/` or
  other sibling `rebuild-*` directories (other sessions' work).
- Known weak spots, so you can avoid or report them: curved-surface trimming
  and some knit cases (surface modeling) are incomplete; very long helical
  sweeps (threads) are slower than flat-plate work; the kernel was not built
  for sheet-metal or weld bead geometry. Plate/bolt/hole/chamfer/fillet/pattern
  workflows are well covered and tested.
- If you need a new request type, feature or fix, write it as a short issue
  in `docs/civilkit-requests.md` in the kernel repo (append only, include a
  repro part JSON) instead of patching kernel code.
- Treat the kernel as pinned by commit: record the hash in your `VERSION`
  file, and re-test your 5 parameter sets when you bump it.

## 8. Reverse direction (CivilKit to kernel)

Already queued on the kernel side: ACIS SAT import (adapted from CivilKit's
`fem-core/src/sat_import.rs`, producing a real B-rep), geometry oracles in the
style of CivilKit's `test_suite/oracles/`, and a protocol-convention lint in
the style of ADR 0014. Nothing is needed from CivilKit's side for these.
