//! Thicken of a seamed cylinder sheet used to panic on a missing edge-image key (fuzzer groups case_104 and kin).
#[path = "robust_common/mod.rs"]
mod common;

fn codes(id: &str) -> Vec<String> {
    common::regen_codes_bounded(common::crash_case(id), 20).into_iter().map(|e| e.1).collect()
}

#[test]
fn thicken_of_cylinder_sheet_succeeds() {
    // the offset layer needs an image of the cylinder's seam edge: previously `ls[li].edges[&seam]` panicked
    for id in ["case_104", "case_6", "case_28", "case_362", "case_835"] {
        assert!(codes(id).is_empty(), "{id}: {:?}", codes(id));
    }
}

#[test]
fn thicken_of_unrepresentable_sheet_is_a_structured_error() {
    // a cylinder of radius ~1e8: the offset boundary does not weld within tolerance
    for id in ["case_191", "case_892"] {
        assert_eq!(codes(id).first().map(String::as_str), Some("thickenFailed"), "{id}");
    }
    // the offset passes the centre of curvature
    assert_eq!(codes("case_498").first().map(String::as_str), Some("offsetTooLarge"));
}
