# Loose roadmap

Written 2026-10-13. Ideas and rough order, not commitments. Sizes are rough: S is days of agent time, M is about
a week, L is several weeks. "Needs" lists what has to exist first. Items move between sections as we learn.

## 1. In flight

- **CAD GUI, first slice** (kit repo, `docs/superpowers/specs/2026-10-13-shared-ui-kit-design.md`): the shell,
  command palette, drawer, history log, status bar and workspace settings are built. Next: the draft-and-confirm
  feature editor (needs `Session.preview`), sketch/curve/plane layers in the viewport, then the sketch editor.
- **Agent surface:** MCP server (`guide`, `schema`, `edit`, `verify`, `check`, `render`), benchmark parts
  (BenchCAD 50, brepjs 25).
- **Kernel breadth:** `ck-ops` stubs mostly done (pipeCut, loft guides, editCurve, sweep guides, fillet profiles,
  variable radius, freeform edits).

## 2. Kernel and correctness

| Item | Size | Notes |
|---|---|---|
| Post-boolean sanity check (catch a result that lost most of an operand) | S | Silent wrong results are the worst failure for an LLM driver: `j_hook`, `rect_frame`, the window fillet. A volume and bounding-box check after each boolean, reported as a warning or error |
| Remaining open `ck-ops` items | M | Adjacent conic or curvature fillets, curved loft centre line, loft `influence` modes, pipeCut on spline edges. See `docs/ck-ops-stubs.md` |
| Boolean robustness batch | M | Wall-tangent holes (BJ-8), fillet of plane/cylinder line edges (BJ-3), chamfer on slightly non-radial arcs, overlapping lofted blades |
| Body-level boolean (tool is "A minus B") and overlapping solids in one document | M | Needed for several brepjs examples |
| Missing outline (silhouette) lines in the `drawing` request | M | The original emits them, ours does not. Affects drawings and prints |
| STEP import gaps | M | HYPERBOLA and PARABOLA curves, torus-fillet shell orientation, sphere pole boundary |
| Ignored fields become errors or warnings | S | A field the kernel does not understand must say so, not be skipped |
| Regen speed | M | Small parts are 1.5 to 3.8 times slower than the original in wasm; the large sample is about 6 times. Levers: tessellation locate walk, allocator, helical-sweep closest point |
| `Session.preview(ops)` and speculative regen | S | Powers the draft editor and live handles |

## 3. Validation and manufacturability

A rule engine on top of today's `validate` / `check`: named **profiles** of rules with parameters and severity,
each issue naming the failing face or edge by persistent name. The drawer's Diagnostics tab, the viewport and an
agent all read the same list.

| Profile | Size | Idea |
|---|---|---|
| Rule framework (profiles, severities, per-entity issues, heat-map overlay) | M | First slice; everything below plugs in |
| 3D printing | M | Overhang by face normal against the build direction and angle limit; bridges; thin walls; minimum features; support volume estimate. Stretch: suggest a build orientation |
| Sheet metal and steel bending | L | Constant thickness, bend radius against material, flange length, hole-to-bend distance, self-collision when folded. The hard part is the flat pattern (unfold with a K-factor) and bend features, which we do not have yet. Validator alone is M |
| Injection moulding and CNC | M | Draft, undercuts, minimum wall, tool reach. Draft and shell features already exist |
| Design rules from the user | S | Min hole, min wall and similar as document-level expectations (extends the `expect` field) |

## 4. Authoring: ways to create parts

| Item | Size | Notes |
|---|---|---|
| Parametric generators as macro features (gears, threads, standard holes, fasteners) | M | Expand into ordinary features so they stay editable in the tree. Involute spur, helical, internal, bevel gears and racks; ISO metric threads, cosmetic or real. We already have twist-extrude screw surfaces and helix sweeps. Gear rules (undercut, contact ratio) become validation rules |
| CadQuery compatibility | M | Facade or shim that records a script into a feature tree. String selectors become `resolve` descriptions. Score it on the 50 BenchCAD programs (real CadQuery references exist) |
| build123d builder mode | M | Context-manager style maps to feature steps. Algebra mode maps poorly |
| Exporters to CadQuery, build123d, brepjs | M | Feature tree out as code; check by running it in the real library |
| brepjs compatibility | M | Functional, shape-valued style is a poor fit without a body-level boolean (section 2) |
| FeatureScript | L+ | Interpreter plus mapping of Onshape's operations and query system. Very large, subset at best, licence unchecked. Defer. Our own macro features cover the use case |
| Sketch editor (constraints, snapping) | L | The first thing that lets you build a part by hand in the GUI |
| Sketch/curve/plane layers in the viewport | S | Underway; makes display toggles real |

## 5. Collaboration (reuse what CivilKit built)

CivilKit already has the platform pieces, and they are generic over opaque data:
- `platform/version-store`: a content-addressed git DAG (commits, branches, write-once tags, merge-base).
- `platform/live`: one Cloudflare Durable Object per project holding presence, op relay and the authoritative live
  document, with version control serialised inside it so commits cannot race.
- The CivilKit program plan (`docs/superpowers/plans/2026-06-10-collaborative-platform-roadmap.md`) covers phases,
  presence, member-anchored comments and structural merge.

### 5a. Git-style histories

- **Our commit is an edit batch.** The session already records `{id, author, time, ops, reason}` and supports undo.
  A commit adds parent links and a content hash; a branch is a line of edit records; a tag is an issued revision
  (immutable, write-once). The drawer's History tab becomes the log, with branch and tag controls.
- **Merge is semantic, by feature.** Two branches that edit different features merge cleanly; edits to the same
  field conflict and are shown as a parameter diff. `refDiff` (reference changed or lost) and the `check`
  expectations are the conflict and sanity checks after a merge.
- **Compare view:** body facts (faces, volume, box) and named faces added or removed between two commits, the same
  diff the session already returns after an edit.
- **Needs:** unique, stable feature ids. Ours are sequential (`F1`, `F2`), and persistent names embed them
  (`F2.S.e1`), so two branches can both create `F7`. Options: random ids with display labels, or id remapping at
  merge time. This is the foundational decision; CivilKit had the same issue with sequential integers. Size: M for
  the model, M for the UI.
- **Reuse:** `version-store` as it is, with our snapshots as the blobs.

### 5b. Multi-user simultaneous editing

- **Authority model:** clients send edit batches to the project's Durable Object. It applies them in order, runs a
  regen with the kernel to validate, and broadcasts accepted batches. A rejected batch returns the structured error
  (code, parameters, hint), the same as today's atomic edit.
- **Optimistic clients:** apply locally for a fast response and roll back on rejection. Regen is slow for big parts,
  so the busy and cancel states matter; the draft-and-confirm editor already separates preview from commit.
- **Presence:** who is here, what they have selected (by persistent name) and which feature they are editing. A soft
  lock on the feature being edited avoids most conflicts; the hard rule stays "authority order wins".
- **Comments** anchored to persistent faces and features, review and approval states on tags.
- **Agents are users:** an MCP agent session is another editor with an author name, visible in the History log.
- **Open question:** op relay with server-side regen (simple, our fit) against a CRDT such as Yjs (offline edits, but
  geometry validity is not mergeable by construction). I lean to op relay.
- **Needs:** 5a ids, kernel running in the worker or on the edge (the wasm already runs in Workers-like hosts),
  `Session.preview`. Size: L.
- **Reuse:** `platform/live` as the starting point.

## 6. UI and platform

| Item | Size | Notes |
|---|---|---|
| Draft-and-confirm feature editor, tabbed left panel | M | Spec Step 4d |
| Command bar for the active tool, handles with typed values | M | Needs `Session.preview` |
| Multiple documents (tab strip), scopes (edit a part in the assembly with the rest faded) | M | State per document, small worker pool |
| Assemblies and mates in the new UI | L | Outliner, exploded view, interference check |
| Placement gizmo (IronCAD TriBall-style): one manipulator for move, rotate, re-anchor and arrays | L | Wanted. The TriBall patent (US5861889A) shows as expired (anticipated expiration 2016-04-19, Google Patents), so the idea is free to build; the name "TriBall" is a separate trademark, so call ours the placement gizmo. Not legal advice. Each gesture ends as a small edit op on a placement node (frames as first-class parameters), the gizmo can detach from its object and snap to references, dragging a handle can create a linear or circular array, and a typed value overrides a drag. See `docs/research-cad-ux-ironcad-plasticity.md` section 3. Needs `Session.preview` |
| Parity with the original UI's tools (105 ribbon tools) | L | Inventory in `docs/original-ui-tools-inventory.md`: each tool with its tooltip and kernel mapping. Build order: sketch tools and solver UI, the common features (extrude, revolve, fillet, hole, patterns), assemblies, surfaces, evaluate, drawing |
| Display layers and per-feature visibility | S | Spec Step 4b |
| Workspace presets and settings (done), theme (dark) in the real app | S | |
| Platform wrappers: desktop (Tauri, native kernel), mobile (Capacitor), touch tool flows | L | Native regen is 2 to 3 times faster than wasm |
| Drawings and BOM in the new UI | L | |
| Thumbnails and sample library, STEP/SAT import UI | S | |
| CivilKit adopts the shared kit (viewport, store, drawer, palette) | M each | Panel by panel, staged TypeScript and Preact move |
| Accessibility and keyboard map editor | M | |

## 7. Quality, speed and housekeeping

- Grow the benchmark sets (BenchCAD families not yet translated, more brepjs examples) and keep the gate's hard
  metrics (wire, validity, volume, oracles, imports, repairs) unchanged.
- Re-run `tools/bench/timing.mjs` after speed work.
- Keep `THIRD_PARTY.md` current whenever a reference is used; recheck licences before code crosses repos.
- Remove scratch tools (`tools/loft_guide_model.mjs`, `tools/probe_part.mjs`), refresh `tools/scoreboard.json`.

## 8. Open decisions

1. Feature id scheme (sequential or unique) before any branching or multi-user work.
2. Op relay or CRDT for live editing.
3. Where the shared kit and the version/live packages live (a third repo is the current plan).
4. Whether the in-app agent chat is part of this app or only its edits are shown.
5. Licence position for FeatureScript, and whether it is worth anything beyond macro features.
