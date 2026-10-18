# Propeller foundations probe

Minimal feature documents in `tools/cases/prop_probe/`, run through the
original kernel (`867fff6c`) and ours (`96b5e2d8`) by
`tools/prop_probe.mjs`.

| # | capability | ours | original | case | note |
|---:|---|---|---|---|---|
| 1 | loft 5..12 spline airfoils, twisted | ok | ok | `cap1_loft_spline12.json`, `cap1_loft_spline5.json` | volume 2172.875 / 2167.928; numeric section-area integral 2136.045 (1.5..1.7 % above, both kernels) |
| 1 | same with polygon sections | ok | ok | `cap1_loft_polygon8.json` | 3341.659; numeric 3344.458 (0.08 % low) |
| 1 | root section inside hub | ok | ok | `cap1_root_hub_8.json`, `cap1_root_hub_spline8.json` | r=8 inside hub r=12; both ok, **no `noClosedProfile` reproduced** |
| 2 | cylinder-wrapped sections | n/a | n/a | — | no wrap / project-to-cylinder feature in the schema |
| 3 | loft with leading/trailing guides | ok (valid guide) | ok | `cap3_loft_guides_valid.json` | 7000.0; the propeller-shaped guide attempt returns `loftGuideMiss` on both |
| 4 | helix curve + sweep | ok | ok | `cap4_helix_sweep.json` | 236.8705, identical |
| 5 | ruled helicoid sheet | n/a | n/a | — | no helicoid-surface feature |
| 6 | project curve onto helicoid | n/a | n/a | — | depends on 5 |
| 7 | thicken open free-form loft sheet | **`thickenReach`** | **ok, 3145.863** | `cap7_thicken_open_loft.json` | **original-only gap** |
| 8 | blade-to-hub root fillet | `missingEdge` | `missingEdge` | `cap8_fillet_blade_hub.json` | edge pick not constructible from this feature document; not a capability verdict |

## Findings
- The kit script's `noClosedProfile` at r=8 inside a 12 mm hub does **not**
  reproduce in a minimal feature document; both kernels accept the root
  section inside the hub. The failure may depend on the kit script's
  hub/blade construction, not loft section handling alone.
- The only original-only gap in this probe is capability 7: open-section
  free-form loft thicken. Ours refuses with `thickenReach`; the original
  builds 3145.863 mm3. This is the same family as the boat-hull thicken gap.
- Capabilities 2, 5 and 6 have no corresponding feature in the current schema.
- Guide lofts work when the guide actually hits the sections; the
  propeller-shaped guide was invalid (`loftGuideMiss` on both).
