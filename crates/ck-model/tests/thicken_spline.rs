//! One-sided thicken of a closed spline sheet (a 10 mm extrusion, perimeter 220 mm, tightest bend radius about 3.6 mm)
//! rebuilt in 25 s native (75 s in the browser) and came back with a display mesh full of holes and no error. The wall's
//! original layer lost the sheet's pcurves, so the tessellator projected its edges onto the closed profile and landed
//! on the wrong turn of the seam.
#[path = "robust_common/mod.rs"]
mod common;

fn check(label: &str, src: String) {
    let t = std::time::Instant::now();
    let (issues, vol) = common::regen_validate(&src);
    let secs = t.elapsed().as_secs_f64();
    assert!(issues.is_empty(), "{label}: {issues:?}");
    // a wall of thickness t around a 220 mm x 10 mm sheet: perimeter x depth x t, give or take the curvature of the offset
    let t_mm: f64 = if label.contains("0.5") { 0.5 } else { 1.0 };
    let v = vol.unwrap();
    let want = 220.15 * 10.0 * t_mm;
    assert!((v / want - 1.0).abs() < 0.2, "{label}: volume {v} vs about {want}");
    assert!(secs < 20.0, "{label}: took {secs:.1}s");
}

#[test]
fn closed_spline_sheet_thickens_valid_and_fast() {
    let base = common::crash_case("spline_thicken_slow");
    assert!(base.contains(r#""side":"front""#));
    check("front 1", base.clone());
    check("back 1", base.replace(r#""side":"front""#, r#""side":"back""#));
    check("front 0.5", base.replace(r#""thickness":1"#, r#""thickness":0.5"#));
}
