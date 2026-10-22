# Propeller foundations probe

Minimal feature documents in `tools/cases/prop_probe/`, run through the
original kernel (`867fff6c`) and ours (`67a1fee2`) by
`tools/prop_probe.mjs`.

| # | capability | ours | original | case | note |
|---:|---|---|---|---|---|
| 1 | loft 5..12 spline airfoils, twisted | ok | ok | `cap1_loft_spline12.json`, `cap1_loft_spline5.json` | volume 2172.875 / 2167.928; numeric section-area integral 2136.045 (1.5..1.7 % above, both kernels) |
| 1 | same with polygon sections | ok | ok | `cap1_loft_polygon8.json` | 3341.659; numeric 3344.458 (0.08 % low) |
| 1 | root section inside hub | ok | ok | `cap1_root_hub_8.json`, `cap1_root_hub_spline8.json` | r=8 inside hub r=12; both ok, **no `noClosedProfile` reproduced** |
| 2 | cylinder-wrapped sections | n/a | n/a | — | no wrap / project-to-cylinder feature in the schema |
| 3 | loft with leading/trailing guides | ok (valid guide) | ok | `cap3_loft_guides_valid.json` | 7000.0; the propeller-shaped guide attempt returns `loftGuideMiss` on both |
| 4 | helix curve + sweep | ok | ok | `cap4_helix_sweep.json` | 236.8705, identical |
| 5 | ruled helicoid sheet | ok | ok | `cap5_ruled_helicoid.json` | radial line sections (radius 20, pitch 20) rotated by the pitch angle and lofted `surface: true` |
| 6 | project closed outline + trim | ok | ok | `cap6_helicoid_project_trim.json` | `projectCurve` of a 12×12 outline onto the sheet (`faces: []` = every face), then `surfaceExtrude` of the outline as the prism tool and `trimSurface`; both kernels keep the trimmed sheet |
| 7 | thicken open free-form loft sheet | **ok, 3145.851** | **ok, 3145.863** | `cap7_thicken_open_loft.json` | was `thickenReach` (rim corner solved off the crease's miter image); fixed, rel 3.8e-6 |
| 7 | same sheet, variable thickness 4→1 mm along z | **ok, 3932.323** | `invalidValue` | `cap7_variable_thicken.json` | new `thicknessAt {along, stops}` field (original kernel rejects the unknown parameter); 1.25 × the constant t=2 volume — the ramp averages 2.5 mm, as the linear integral requires |
| 8 | blade-to-hub root fillet | `blendFailed` | `blendFailed` | `cap8_fillet_blade_hub.json` | the 12 blade-root edges (blade faces × hub cylinder) at r=2: both kernels fail the blend (ours: `detail` “foot 1”, the ball-centre foot solve; original: “the spring does not leave its face at the chain's end”). A single root edge is `blendUnsupported` on ours with `detail` “concave chain end is not on a planar face” (r=1 and r=2) and `blendFailed` on the original; at r=1 on the 12-edge loop the original kernel did not return within 3 min |

## Findings
- The kit script's `noClosedProfile` at r=8 inside a 12 mm hub is **not** a
  loft bug: the script's `chord(r) = 24 - 10*((r-R0)/(R-R0))**1.5` evaluates
  a fractional power of a negative number for `r < R0`, so every root-section
  point is NaN (`run_script_file` on the script: r=8 fails, r=9 works —
  `chord(9)` is finite). A NaN is not expressible in JSON, so across the
  kit's JSON boundary it arrives as `null`, the oracle-pinned `null -> 0`
  collapses the whole section onto the origin and the sketch has no region —
  the original kernel reports the same `noClosedProfile` for the same
  document. Both kernels also accept a root section inside a hub
  (`cap1_root_hub_8`). Where a NaN does reach the kernel in memory, the
  sketch's points are now refused as `invalidValue {point, detail, hint}`
  naming the point instead of the misleading `noClosedProfile` /
  `loftProfile` (sketch_nan tests).
- The only original-only gap in this probe was capability 7: open-section
  free-form loft thicken. Ours refused with `thickenReach`; the original
  builds 3145.863 mm3. Fixed: the rim image of an open edge now ends on the
  shared edge's miter corner (the same corner `rim_curve` produces in the
  creased path), so the rim closes onto the neighbouring layer.
- Capabilities 5 and 6 (ruled helicoid sheet, outline projected onto it and
  the sheet trimmed) work on both kernels with the feature documents above;
  there is no dedicated helicoid feature, a `surface: true` loft of rotating
  radial lines is the helicoid. Only capability 2 (sections wrapped onto a
  cylinder) has no feature in the schema.
- The blade-root fillet (capability 8) is the remaining gap: the blade root
  meets the hub in a closed chain of spline-intersection edges, which neither
  kernel blends. Our free-form blend now returns the chain / face-pair /
  end-condition reason as `params.detail` on `blendUnsupported` /
  `blendFailed` instead of a bare code.
- Variable thickness (root thick, tip thin) is **not** cheap: `thicken` takes
  one scalar, and the wall's rails, miters and layer naming all assume a
  constant offset along the whole sheet. A thickness function of v would need
  the schema (a per-station array or an expression of the surface parameter),
  the offset path (d varies along the rails, so the rim corners and the
  `offsetTooLarge` limit become per-station) and the freeform crease miters
  re-derived for unequal sides. Skipped for now.
- Guide lofts work when the guide actually hits the sections; the
  propeller-shaped guide was invalid (`loftGuideMiss` on both).

### Blade-root fillet: diagnosis (2026-10-22, branch `blade-root-blend`)

A smaller case than `cap8` (one lofted blade, 3 sections, on a cylinder;
root radius 3.5 inside a hub radius 6, chord 7, 0.2 mm blunt trailing edge)
pins the topology and the failure. See
`crates/ck-model/tests/blade_root_blend.rs` and the fixture
`blade_root_blunt.ckpart.json`.

- The root loop is a **closed ring** of spline-intersection edges. The small
  case has 5 edges sharing 5 vertices (each edge end is shared: as many
  distinct vertices as edges); a sharp trailing edge gives 2 edges.
- **`cap8`'s pick list is not the whole loop.** It names 12 of the loop's
  edges (the loop of a 19-sided polygon section has more), so the chain the
  blender builds is an **open sub-chain**. That is why `cap8` reaches the
  chain-end logic (`detail` "foot 1" on the 12-edge selection at r = 2, and
  "concave chain end is not on a planar face" for a single edge): the ends of
  an open sub-chain lie on the hub cylinder and a blade face, neither planar.
  The brief's "closed chain" only holds for the complete selection.
- Selecting the **whole** loop used to be refused at once:
  `blendUnsupported`, `detail` "closed ring of free-form edges"
  (`freeform::build_chain` had no ring path). The ring is now recognised
  (`build_chain` returns `closed`), the end logic is skipped and a ring tool
  is built (closed rail edges, band faces, no caps). The blend still fails,
  one step later: `blendFailed`, `detail` "centre did not converge", with the
  hint "try a smaller fillet radius…". The rolling-ball centre Newton diverges
  at the blade root's **shallow dihedral** (the third equation pins the centre
  to the plane normal to the edge, which is only a first-order model on a
  curved, near-tangent root). With `CK_BLEND_DEBUG=1` the failing station, ball
  centre and per-face distances print; e.g. `li=1 s=0.5556
  p=(5.7880,1.5810,-1.9143)`.
- It is the dihedral, not the 0.2 mm blunt width: a 2 mm blunt edge and a
  gentler twist (root section angle ≈ 20° instead of 54°) both still diverge
  at the same kind of station. So task 1 is blocked on a robust constrained
  centre solver (hypothesis 2 of the brief), not on the sharp-corner rule
  (task 2).
- `freeform.rs` now reports, behind `CK_BLEND_DEBUG`, the failing station
  (`li`, `s`, the edge point) and, on a `foot_set` miss, every face of the
  set with its closest-point distance, whether the ball centre lies in the
  padded face box and the outward normal there (`explain_foot`).

Open here: make the blade-root fillet work (task 1). The pieces in place are
closed-ring detection, the ring tool and the hint plumbing; the missing piece
is a centre solver that stays on the offset-surface intersection at a shallow,
curved dihedral instead of the plane-through-the-edge-point constraint.
