//! The `dome` feature (see features/solids.rs): a cap on a round planar face.
mod solids_common;
use solids_common::*;
use std::f64::consts::PI;

/// A cylinder of radius 10, height 20 about z (cap faces F1.C0 bottom, F1.C1 top).
const CYL: &str = r#"{"type":"sketch","id":"S","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0},{"id":"p","x":10,"y":0}],"entities":[{"type":"circle","id":"k","c":"c","r":10}],"constraints":[]},
 {"type":"extrude","id":"F1","op":"add","sketch":"S","end":"blind","depth":20}"#;

fn dome(extra: &str) -> String {
    format!(r#"{{"type":"dome","id":"F2",{extra}}}"#)
}

fn cap_volume(a: f64, h: f64) -> f64 {
    PI * h * (3.0 * a * a + h * h) / 6.0
}

#[test]
fn a_spherical_dome_adds_the_cap_volume() {
    let base = PI * 100.0 * 20.0;
    for h in [3.0, 10.0, 14.0] {
        let r = run(&format!("{CYL},{}", dome(&format!(r#""face":"F1.C1","height":{h}"#))));
        assert!(r.errors.is_empty(), "{h}: {:?}", r.errors);
        assert!(r.valid.is_empty(), "{h}: {:?}", r.valid);
        assert!(near(r.vol, base + cap_volume(10.0, h), 2e-3), "{h}: {} vs {}", r.vol, base + cap_volume(10.0, h));
        assert!(r.faces.iter().any(|(_, k)| *k == "sphere"), "{h}: {:?}", r.faces);
    }
}

#[test]
fn an_elliptical_dome_is_not_built_yet() {
    let r = run(&format!("{CYL},{}", dome(r#""face":"F1.C1","height":4,"elliptical":true"#)));
    assert_eq!(r.codes, ["notImplemented"], "{:?}", r.errors);
    assert!(r.errors[0].contains("elliptical dome"), "{:?}", r.errors);
}

#[test]
fn a_negative_height_cuts_a_dimple_and_the_face_may_be_in_faces() {
    let r = run(&format!("{CYL},{}", dome(r#""faces":["F1.C1"],"height":-3"#)));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    assert!(near(r.vol, PI * 100.0 * 20.0 - cap_volume(10.0, 3.0), 2e-3), "{}", r.vol);
}

#[test]
fn the_bottom_face_domes_outward_too() {
    let r = run(&format!("{CYL},{}", dome(r#""face":"F1.C0","height":5"#)));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    assert!(near(r.vol, PI * 100.0 * 20.0 + cap_volume(10.0, 5.0), 2e-3), "{}", r.vol);
    let (lo, hi) = r.bbox.unwrap();
    assert!(lo[2].min(lo[1]) < -4.0 || hi[2] > 24.0 || lo[1] < -4.0, "{lo:?} {hi:?}");
}

#[test]
fn bad_input_is_refused_with_stable_codes() {
    assert_eq!(run(r#"{"type":"dome","id":"F2"}"#).codes, ["nothingToCut"]);
    for extra in [r#""face":"F1.C1","height":0"#, r#""face":"F1.S.k","height":3"#, r#""faces":["F1.C0","F1.C1"],"height":3"#, r#""face":"F1.C1","height":"=0""#] {
        let r = run(&format!("{CYL},{}", dome(extra)));
        assert_eq!(r.codes, ["invalidValue"], "{extra}: {:?}", r.errors);
    }
    let r = run(&format!("{CYL},{}", dome(r#""face":"F1.Nope","height":3"#)));
    assert_eq!(r.codes, ["missingReference"], "{:?}", r.errors);
    // a rectangular face is not round
    let r = run(&format!("{BLOCK},{}", dome(r#""face":"F1.C1","height":3"#)));
    assert_eq!(r.codes, ["invalidValue"], "{:?}", r.errors);
}
