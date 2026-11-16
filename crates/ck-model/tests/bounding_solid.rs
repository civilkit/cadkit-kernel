//! The `boundingSolid` feature (see features/solids.rs): the axis-aligned box around the selected bodies.
mod solids_common;
use solids_common::*;
use std::f64::consts::PI;

const BODIES: &str = r#"{"type":"box","id":"A","op":"newBody","body":"Left","center":[0,0,0],"size":[10,20,30]},
 {"type":"sphere","id":"B","op":"newBody","body":"Ball","center":[50,5,5],"radius":6}"#;

#[test]
fn the_box_around_every_body_spans_their_union() {
    let r = run(&format!("{BODIES},{}", r#"{"type":"boundingSolid","id":"F3"}"#));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(r.valid.is_empty(), "{:?}", r.valid);
    assert_eq!(r.lumps, 3);
    // x -5 .. 56, y -10 .. 11, z -15 .. 15 (the ball reaches y 11 and z 11)
    let want = 61.0 * 21.0 * 30.0;
    let sum = 10.0 * 20.0 * 30.0 + 4.0 / 3.0 * PI * 216.0;
    assert!(near(r.vol, sum + want, 1e-3), "{} vs {}", r.vol, sum + want);
    assert!(r.faces.iter().any(|(n, k)| n.starts_with("F3.") && *k == "plane"), "{:?}", r.faces);
}

#[test]
fn a_selection_and_a_padding_shape_the_box() {
    let r = run(&format!("{BODIES},{}", r#"{"type":"boundingSolid","id":"F3","bodies":[{"body":"Ball"}],"padding":2}"#));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    let side = 12.0 + 4.0;
    let sum = 10.0 * 20.0 * 30.0 + 4.0 / 3.0 * PI * 216.0;
    assert!(near(r.vol, sum + side * side * side, 2e-3), "{}", r.vol);
    let r = run(&format!("{BODIES},{}", r#"{"type":"boundingSolid","id":"F3","bodies":[{"face":"A.C1"}]}"#));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(near(r.vol, 6000.0 + 4.0 / 3.0 * PI * 216.0 + 6000.0, 1e-3), "{}", r.vol);
}

#[test]
fn bad_input_is_refused_with_stable_codes() {
    let r = run(r#"{"type":"boundingSolid","id":"F3"}"#);
    assert_eq!(r.codes, ["nothingToCut"], "{:?}", r.errors);
    for extra in [r#""padding":-1"#, r#""op":"cut""#, r#""op":"add""#] {
        let r = run(&format!("{BODIES},{}", format!(r#"{{"type":"boundingSolid","id":"F3",{extra}}}"#)));
        assert_eq!(r.codes, ["invalidValue"], "{extra}: {:?}", r.errors);
    }
    let r = run(&format!("{BODIES},{}", r#"{"type":"boundingSolid","id":"F3","bodies":[{"body":"Nope"}]}"#));
    assert_eq!(r.codes, ["missingReference"], "{:?}", r.errors);
}
