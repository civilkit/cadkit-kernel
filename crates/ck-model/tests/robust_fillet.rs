//! A tangent-propagated fillet round a rounded-rectangle loop used to run for minutes and then trap (the curve fits of
//! the long propagated chain built a dense (n+1)^2 matrix per fit; fixed by the windowed solver in ck-geom).
#[path = "robust_common/mod.rs"]
mod common;

#[test]
fn rounded_rectangle_loop_fillet_succeeds() {
    let src = common::crash_case("rrect_loop_0");
    assert!(common::regen_codes_bounded(src.clone(), 60).is_empty());
    let (issues, vol) = common::regen_validate(&src);
    assert!(issues.is_empty(), "{issues:?}");
    // the sweep's OCCT reference volume for this case is 17696.4689447
    assert!((vol.unwrap() - 17696.4689447).abs() < 1e-3, "{vol:?}");
}
