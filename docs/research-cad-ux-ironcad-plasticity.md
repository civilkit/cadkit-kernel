# UX research: IronCAD and Plasticity as references for our interactive layer

Date: 2026-10-13. Read-only web research; short paraphrases only. "Unconfirmed" means I could not verify it from a primary or clearly attributed source. Evidence quality is uneven: Plasticity has good vendor docs but thin community data that I could reach (Reddit and HN were not returned by search); IronCAD has good legacy vendor help but mostly aggregator reviews.

Important correction to the brief: I found **no evidence that Plasticity has shipped a history or procedural feature tree**. Release notes 2024.2, 25.3 and 2026.1 do not mention one (see sources in section 2). The only relevant primary-ish statement is the developer saying parametric features were lower priority and a modifier system was planned (CG Channel, 2023). Treat "Plasticity history/procedural" as unconfirmed and possibly not existing. Plasticity does have live Instances (25.3) and a "step back to source objects" selection shortcut (Ctrl+T), which are the nearest things.

## 1. What users love (ranked)

**IronCAD**
1. TriBall positioning and drag-and-drop concepting; fast, forgiving for machinery concepts. Evidence: Software Advice/Capterra review summaries (https://www.softwareadvice.com/3d-cad/ironcad-profile/reviews/ via search summary; https://www.g2.com/products/ironcad/reviews). One reviewer praises forgiveness when shifting or deleting in the "constructing tree".
2. Hybrid parametric plus direct in one scene, no mandatory constraints to get control. Vendor explanation: position does not need a relation to be precise, the TriBall gives a third state between free and constrained (https://uploads-ssl.webflow.com/6229acf85804d0a1881c66f2/6229acf85804d02aba1c69e7_Managing%20the%20Anchor%20Point%20PDF.pdf).
3. Lower price than high-end CAD and a short learning curve per some reviewers (Software Advice, above); lots of training material (https://www.digitalengineering247.com/article/ironcad-2019-20-years-of-innovation, a favourable trade-press piece).
4. Catalog of reusable parametric content with attachment points that snap together (same anchor PDF; digital engineering article says nearly 30,000 components, vendor-adjacent claim).

**Plasticity**
1. Speed, clean context-sensitive UI, focus on one task at a time, "feels like sculpting" (https://blog.zsa.io/zsa-loves-plasticity/, a single happy user).
2. Fillets and booleans that work, from Parasolid (https://discourse.mcneel.com/t/plasticity-new-software/152284?page=2; vendor claim at https://plasticity.xyz/product).
3. Fair pricing and perpetual use (zsa blog; McNeel thread). Counter-note: tier splitting annoyed some (section 2).
4. Keyboard-first operation with Blender-familiar bindings (https://www.cgchannel.com/2023/04/check-out-promising-new-nurbs-modeller-plasticity).
5. Responsive developer, public voting on features (a Reddit comment surfaced via search summary; link not verified).

## 2. What users complain about

**IronCAD**
- Drawings and BOMs: clunky 2D drawings, structured BOMs on multi-level assemblies, weak 2D dimensioning (https://www.g2.com/products/ironcad/reviews; https://www.rfp.wiki/vendors/ironcad).
- Large-assembly instability, bugs lingering across releases, an auto mode switch on large assemblies some users disable (rfp.wiki; Software Advice).
- Interop: extra paid importers needed (Software Advice Australia summary).
- Thin or outdated docs, small community (G2, Software Advice).
- Learning curve because UI differs from SolidWorks/Autodesk norms (rfp.wiki).
- Fillet failure threads for IronCAD: none found. Unconfirmed.
- Sample sizes are small and some reviews are old.

**Plasticity**
- No history: edit a curve and the dependent surface does not update; "not parametric", weaker for mechanical CAD (Onshape forum, https://forum.onshape.com/discussion/comment/114717 search summary; fetch was redirected to login so not directly read).
- Missing parametric bulk edits, e.g. changing many hole diameters at once, plus dimensional control (r/macro_pads thread via https://redlib.hbubli.cc/r/macro_pads/comments/1jqh55p/spacemouse_keypad, summary only).
- Fillet depth: complex fillets less capable than advertised, same kernel does not guarantee robustness (McNeel forum thread above and https://discourse.mcneel.com/t/parasolid-kernel/157461, summary).
- Licensing: XNurbs only on Studio tier, felt like a mid-stream change (McNeel thread).
- Some bugs and "design logic" that needs learning (zsa blog).
- Basic draft tool, no parting-line draft (Core77, https://boards.core77.com/t/plasticity-a-true-cad-modeling-evolution/82604, summary).
- Drawings: only Export Hidden Line SVG arrived in 2026.1 (https://digitalproduction.com/2026/04/21/plasticity-2026-1/). Assemblies: STEP assembly import and groups only; no mates. Both inferred from release notes, so treat as likely rather than confirmed.

## 3. Interaction vocabulary

### IronCAD
Sources: TriBall help PDF (https://www.cs.cmu.edu/afs/cs.cmu.edu/Web/People/rapidproto2/labs/IronCAD/TriBallHelp.pdf, 2007, so menu names may have drifted), anchor PDF (link above), https://www.digitalengineering247.com/article/ironcad-2019-20-years-of-innovation.

- **Objects and levels**: Feature under Part under Assembly. Every object, including features, cameras and lights, owns an **anchor point** (position plus own XYZ). A catalog drop places the anchor; first object lands at origin, later ones at the cursor.
- **Selection quasi-mode**: repeated click on a shape cycles part, IntelliShape (size-box handles), then face/edge (digital engineering article). Right-click menus are contextual and are the main command surface; ribbon provides Tools tab and an Intelligence group.
- **Size box / IntelliShape handles**: drag handles to resize or type a dimension. Exact handle behaviour: unconfirmed beyond that.
- **TriBall** (the key idea). Activated from a toolbar/ribbon icon or the right-click menu of a shape, part or assembly. Two concentric zones:
  - Outer sphere: three 1D axis handles drag along an axis; three plane areas drag in 2D; clicking a 1D handle selects it as rotation axis, then dragging inside the ball rotates around it; dragging the circumference rotates around the ball centre; unconstrained rotation is off by default and enabled from a context menu.
  - Numeric entry: a live distance or angle readout appears while dragging; right-click it, choose Edit Value, type the number.
  - Inner sphere: orientation. Right-click an axis for To Point, To Centre Point, Point to Point, Parallel to Edge, Perpendicular to Face, Parallel to Axis, Reverse, Mirror (move/copy/link), Edit Direction. Right-click centre for To Point, To Midpoint, Edit Position.
  - Right-click-drag on a handle or plane offers Move, Copy, Link or pattern, i.e. one gesture yields several edit kinds.
  - Orientation is remembered (2015 notes, https://www.ironcad.com/blog/whats-new-2015/); catalog items can open with TriBall active on drop.
  - Moving the TriBall itself (not the part) moves the anchor, e.g. to put an origin at a corner.
- **Anchor modes** (right-click anchor): Disallow Drag (default, forces TriBall or dimension), Drag Freely, Drag Along Surfaces, Attach to Surface (feature-level; follows dimension changes). Shift snaps to other geometry.
- **Snapping**: green SmartSnap dots on hosts; Shift snaps hole depth to a face or edge (https://www.ironcad.com/?p=32175 summary). **Attachment points** are magnetic sockets that mate between catalog items. Smart Dimensions are a second route to positioning and can tie to the anchor via Ctrl-select.
- **Preview/commit**: drag-and-drop is live and direct; Quick Edit on drop shows parameters (2026 notes, https://www.ironcad.com/blog/whats-new-in-2026/). Undo, repeat and hotkey specifics: unconfirmed.
- **History**: Scene Browser tree on the left, Catalog Browser on the right; features under a part can be edited by click or "Edit Feature Options" (GrabCAD tutorial via search summary; page itself returned 403). Dual ACIS and Parasolid kernels, per the 2019 article.
- **Failures**: no IronCAD-specific fillet/boolean feedback found. Flexible Solving can reassign relationship levels when over-constrained (2015 notes).
- **Recent direction**: multi-window 3D and drawing, Smart Annotation, AI chatbot and Python scripting foundations (2026 notes). Shows the vendor itself is heading toward scripted and assistant-driven modelling.

### Plasticity
Sources: manual at https://doc.plasticity.xyz, with pages cited below.

- **Selection modes**: keys 1 to 4 for control point, edge, face, body; Tab or 5 for all; Shift+1..4 toggles (https://doc.plasticity.xyz/plasticity-essentials). Ctrl+1..5 converts the selection between types. Alt+click selects loops and related items; Ctrl+Alt+click selects rings and parallel faces; Ctrl+Shift+click opens a pick-list of what is under the cursor; Ctrl+T / Ctrl+Shift+T steps back and forward through the source objects of earlier operations (https://doc.plasticity.xyz/plasticity-essentials/selecting-objects). Box direction means touch versus contain, shown green versus purple.
- **Command-first, selection-aware**: select edges and Fillet starts automatically, or press B (https://doc.plasticity.xyz/solid/fillet-shell). Fillet on curves or vertices dispatches to different commands (https://doc.plasticity.xyz/solid/fillet). A command shows a floating Command Dialog with fields and OK / right-click to confirm.
- **Gizmos**: a yellow dot you drag (out = fillet, in = chamfer); axis arrows plus pivot circle for Move/Rotate/Scale; per-command gizmos such as Draft reference face, Offset Edge, Bridge continuity (https://doc.plasticity.xyz/release-notes/whats-new-2024.2).
- **Numeric entry**: Tab then type a distance from the gizmo, for Move, Rotate, Scale, Push Face, Extrude, Thicken (2026.1 notes). In dialogs, signed values encode fillet (positive) versus chamfer (negative). Single-letter keys inside a command switch sub-modes: D/C distance by cursor, A angle, V add variable point, L add limit, T toggle tangent-edge propagation, Ctrl add/remove edges.
- **Fillet options**: shape Conic, Chordal, G2, Full; tension; Y-blend attempt; chamfer Offset or Apex (fillet-shell page). Variable radius via points on the edge.
- **Boolean**: Q, pick target then tool, then Union/Difference/Intersect/Slice/Region; Keep Tools; target/tool "material" overrides; G/R/S move/rotate/scale the tool, X/Y/Z constrain (https://doc.plasticity.xyz/solid/boolean). Live preview is not described there (unconfirmed).
- **Snapping**: midpoints, edges, construction guides, tangent snaps (https://plasticity.xyz/product); incremental grid snapping; 2D snap projecting to the active construction plane (2026.1); sphere-centre snap (25.3).
- **Command invocation**: F opens the Command Palette (search); radial menus are JSON-defined, bound to a key, open at cursor, and work as click or press-hold-release-over-item; nestable (https://doc.plasticity.xyz/plasticity-essentials/radial-menu). Default radials: Selection mode and Viewport settings.
- **History view**: none as a tree. Outliner holds objects, groups and collections; instances are live references (25.3 notes, https://doc.plasticity.xyz/release-notes/whats-new-2025.3). Undo/redo shortcuts and stack UI: unconfirmed.
- **Failure feedback**: the fillet and boolean pages do not describe it; there is a "Fillet Order of Operations" page I could not retrieve (404). Community says it fails on hard cases like other Parasolid apps. XNurbs gets a warning indicator for conflicting inputs (2024.2).

## 4. Conflicts with a document-first, LLM-drivable design, and how to keep the feel

Our rules: feature tree is the model; every interaction ends as a small edit op; previews never change the document; selection is stored as persistent names.

| Product trait | Conflict | Resolution |
|---|---|---|
| Plasticity operates on the current B-rep, with no recorded history; selections are live topology | Direct edits have no tree node; selections are transient ids | Every gizmo commit becomes an op appended to the tree (e.g. "offset face X by d", "fillet edges A,B r"), referencing persistent face/edge names. Direct feel, recorded result. Needs name survival across ops. |
| Ctrl+T "go to the source objects" | Implies hidden lineage | Our tree already has it; expose as "select inputs of this feature" and "select what this feature produced". |
| IronCAD anchors and TriBall move objects freely with no constraint | Free position is state outside any rule | Store the anchor/frame as an explicit parameter set on a placement node; TriBall edits that node. The drag path is lost; only the final transform is kept. |
| IronCAD right-click-drag yields Move, Copy, or Link in one gesture | One gesture, multiple ops | Show the choice as a release-time menu or modifier; each result is one op (transform, copy, instance). |
| Edit Value on a live readout | Number typed against a transient state | Keep it: the readout is the op parameter. Typing replaces the parameter in the preview; commit writes it. |
| Plasticity single-letter sub-mode keys while a command runs | Hidden state inside a modal command | Reflect each key as an explicit, serialisable option on the op (so an LLM and the UI use the same schema); show options in a visible panel. |
| Drag-drop with the anchor following the cursor | Continuous stream of edits | Preview from a scratch evaluation (kernel call with throwaway params), then one commit op. Throttle by param, not by document revision. |
| Booleans/fillets that silently fail or leave result unchanged | Our ADR requires structured per-feature errors | Surface the error on the tree node and on the offending entities in the viewport (section 5, item 6). |
| IronCAD "edit later" can break references; Plasticity "can't edit later" | Both lose intent | Persistent names plus replay give both: edit any earlier op and the tail re-evaluates, with stale references flagged. |

## 5. Prioritised features (max 12)

Sizes: S small, M medium, L large. Kernel flags marked **K**.

1. **Selection modes with number keys, loop/ring/tangent-chain and "pick under cursor" list** (M, selection). Plasticity's Alt/Ctrl+Alt/Ctrl+Shift click vocabulary and 1..5 keys (selecting-objects page). Stores results as persistent names. Matters because fast selection is the most-cited speed source.
2. **Command palette plus bindable radial menus** (M, tools). F palette and hold-release radials from Plasticity; also friendly to LLM command naming. Same registry feeds ribbon, palette and LLM tool list.
3. **Selection-aware command dispatch** (S, tools). Selecting edges and pressing a key starts fillet; selecting curves does a different one (fillet page). Pre-fills inputs from selection.
4. **Fillet/chamfer gizmo: drag dot, signed Tab entry, sub-mode keys as visible options** (M, handles and parameters). Fillet-shell page. **K**: conic/G2/full shapes, variable radius and limit points, Y-blend.
5. **Live-readout numeric entry on every handle** (S, handles/parameters). Both products do it (TriBall Edit Value; Plasticity Tab). Ship one shared input component.
6. **Per-feature error display with offending-entity highlight and "why" text** (M, inspection and tree). Neither product documents this well, which is an opening. Our structured errors make it cheap. Fillet failure remains a top complaint class (McNeel threads).
7. **TriBall-style placement gizmo with orientation-by-reference** (L, handles and snapping). Outer sphere translate/rotate, inner sphere "align axis to edge/face/point". Each option a deterministic op on a placement node. Needs frames as first-class parameters.
8. **Drop-from-catalog with live parameter prompt on drop and attachment points** (M, tools and tree). IronCAD Quick Edit on drop (2026 notes) and magnetic attachment points (anchor PDF). Catalog entries are small feature subtrees; attachment points are named frames.
9. **Snapping engine with visible snap targets (SmartSnap-style dots), Shift to snap to other geometry, construction-plane 2D projection** (M, snapping). Both products.
10. **Direct face tools recorded as tree ops: push/pull face, offset face, move face** (L, handles). Plasticity "Direct Face Manipulation" (product page). **K**: face push/pull, move-face and healing of neighbouring faces; also imprint/split on faces for later targeting.
11. **Boolean flow with target/tool pick, Keep Tools, tool gizmo (G/R/S), preview of result** (M, tools and viewer). Boolean page. Preview must come from scratch evaluation. **K** if tangent/coincident cases are weak.
12. **Tree panel with step-back "inputs/outputs" selection and rollback bar** (M, tree). Replaces Plasticity's missing history and IronCAD's browser edit-feature flow; Ctrl+T idea turned into a tree action.

Not on the list but worth noting: live instances (Plasticity 25.3) and continuity/curvature combs (G0/G1/G2) need **K** surfacing support and are later.

## 6. Licence and trademark notes

Safe as inspiration (ideas and generic behaviours are not protectable by copyright): selection by number keys and loop/ring modifiers; a radial menu opening at the cursor with hold-release; gizmo drag with typed override; selection-aware command dispatch; live parameter prompt on catalog drop; anchor/attachment-point concepts; direct editing recorded as ops.

Must not copy: any icons, screenshots, catalog content, documentation text, UI layouts reproduced pixel-for-pixel, radial menu definition files, or code; product and feature names used as ours. "IronCAD", "TriBall", "IntelliShape", "SmartSnap", "Plasticity", "XNurbs" and "Parasolid" are names of those vendors or their partners and should not be used as our feature names. Use neutral terms ("placement gizmo", "attachment point", "command palette"). Patents (update 2026-10-13, from the Google Patents record supplied by the project owner): the TriBall patent is US5861889A ("Three dimensional computer graphics tool for manipulating objects", 3D/EYE Inc.), filed 1996-04-19, granted 1999-01-19, anticipated expiration 2016-04-19, status "Expired - Lifetime". The expiry is Google's computed date, not a legal determination, and no other patents were searched; the trademark "TriBall" is a separate matter and still applies, so we use neutral names. Earlier note: the sources describe TriBall as "patented" in a review snippet (rfp.wiki) but I did not verify any patent number or its expiry; the original TriSpectives dates to 1995, so a patent from that era is probably expired, but this is unconfirmed and a quick search by counsel is advisable before shipping a near-identical three-ring ball. I am not giving legal advice.

## Sources not fully read
- Onshape forum comment 114717 redirected to a login page; its content is from search summaries.
- GrabCAD IronCAD tutorial returned 403; G2 and Software Advice pages were read only through search summaries.
- Plasticity Fillet Order of Operations page 404; snap page 404; push face page 404.
