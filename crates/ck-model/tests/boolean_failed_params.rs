//! A `booleanFailed` error carries the faces and the point where the result could not be closed, and a hint.
use ck_model::regen::FeatureError;

#[test]
fn boolean_failed_params_name_faces_and_point() {
    let e = FeatureError::new("booleanFailed;edgeFaces=F8.S.e1~1@F9.1|F8.S.e1~1@F9.3;issue=nonManifoldEdge;at=14.2105|0.0000|8.2842");
    assert_eq!(e.code, "booleanFailed");
    let get = |k: &str| e.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()).unwrap_or_else(|| panic!("param {k}"));
    let s = ck_json::to_string(&get("faces"));
    assert_eq!(s, "[\"F8.S.e1~1@F9.1\",\"F8.S.e1~1@F9.3\"]");
    assert_eq!(ck_json::to_string(&get("issue")), "\"nonManifoldEdge\"");
    assert_eq!(ck_json::to_string(&get("at")), "[14.2105,0,8.2842]");
    assert!(ck_json::to_string(&get("hint")).contains("tangent"));
}

#[test]
fn plain_boolean_failed_carries_a_hint() {
    let e = FeatureError::new("booleanFailed");
    assert_eq!(e.code, "booleanFailed");
    let hint = e.params.iter().find(|(n, _)| n == "hint").map(|(_, v)| ck_json::to_string(v)).expect("hint");
    // the causes a bare failure can point at: tangency / coincidence / a sliver, and the knobs
    assert!(hint.contains("tangentially") && hint.contains("tolerance"), "{hint}");
}

#[test]
fn near_coincident_boolean_failed_names_the_faces_and_a_hint() {
    let e = FeatureError::new("booleanFailed;nearCoincident=F2.S.e1|F5.F1|2.5e-4");
    assert_eq!(e.code, "booleanFailed");
    let get = |k: &str| e.params.iter().find(|(n, _)| n == k).map(|(_, v)| ck_json::to_string(v)).unwrap_or_else(|| panic!("param {k}"));
    assert_eq!(get("hostFace"), "\"F2.S.e1\"");
    assert_eq!(get("toolFace"), "\"F5.F1\"");
    assert_eq!(get("gap"), "0.00025");
    assert!(get("hint").contains("nearly coincident"), "{}", get("hint"));
}
