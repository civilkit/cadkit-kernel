//! The `torus` feature (see features/solids.rs).
mod solids_common;
use solids_common::*;
use std::f64::consts::PI;

fn torus(extra: &str) -> String {
    format!(r#"{{"type":"torus","id":"F1","op":"add",{extra}}}"#)
}

#[test]
fn a_torus_is_one_valid_toroidal_face_with_the_closed_form_volume() {
    for axis in ["x", "y", "z"] {
        let r = run(&torus(&format!(r#""center":[1,2,3],"axis":"{axis}","majorRadius":20,"minorRadius":4"#)));
        assert!(r.errors.is_empty(), "{axis}: {:?}", r.errors);
        assert!(r.valid.is_empty(), "{axis}: {:?}", r.valid);
        assert!(near(r.vol, 2.0 * PI * PI * 20.0 * 16.0, 1e-4), "{axis}: {}", r.vol);
        assert_eq!(r.faces, [("F1.S.e1".to_string(), "torus")], "{axis}");
    }
}

#[test]
fn the_axis_orients_the_ring() {
    let r = run(&torus(r#""axis":"z","majorRadius":20,"minorRadius":4"#));
    let (lo, hi) = r.bbox.unwrap();
    assert!(near(hi[0], 24.0, 1e-3) && near(hi[2], 4.0, 1e-3) && near(lo[2], -4.0, 1e-3), "{lo:?} {hi:?}");
    let r = run(&torus(r#""axis":[1,0,0],"majorRadius":20,"minorRadius":4"#));
    let (lo, hi) = r.bbox.unwrap();
    assert!(near(hi[0], 4.0, 1e-3) && near(hi[2], 24.0, 1e-3) || near(hi[1], 24.0, 1e-3) && near(hi[0], 4.0, 1e-3), "{lo:?} {hi:?}");
}

#[test]
fn radius_alias_and_expressions_work_and_cut_removes_the_ring() {
    let r = run(&format!("{BLOCK},{}", r#"{"type":"torus","id":"F2","op":"cut","center":[0,0,0],"radius":"=10+5","tubeRadius":3}"#));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    // a ring of centre radius 15 and tube 3 lies inside the 40 block only up to its half height 3
    assert!(near(r.vol, 64000.0 - 2.0 * PI * PI * 15.0 * 9.0, 2e-3), "{}", r.vol);
}

#[test]
fn bad_input_is_refused_with_stable_codes() {
    for extra in [
        r#""majorRadius":0,"minorRadius":1"#,
        r#""majorRadius":5,"minorRadius":-1"#,
        r#""majorRadius":5,"minorRadius":5"#,
        r#""majorRadius":5,"minorRadius":9"#,
        r#""majorRadius":5"#,
        r#""majorRadius":5,"minorRadius":1,"axis":[0,0,0]"#,
        r#""majorRadius":5,"minorRadius":1,"axis":"w""#,
        r#""majorRadius":5,"minorRadius":1,"center":[1,2]"#,
    ] {
        let r = run(&torus(extra));
        assert_eq!(r.codes, ["invalidValue"], "{extra}: {:?}", r.errors);
    }
    let r = run(r#"{"type":"torus","id":"F1","op":"cut","majorRadius":5,"minorRadius":1}"#);
    assert_eq!(r.codes, ["nothingToCut"]);
}

#[test]
fn as_a_new_body_it_stays_separate() {
    let r = run(&format!("{BLOCK},{}", r#"{"type":"torus","id":"F2","op":"newBody","body":"Ring","center":[60,0,0],"majorRadius":10,"minorRadius":2}"#));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    assert_eq!(r.lumps, 2);
    assert!(near(r.vol, 64000.0 + 2.0 * PI * PI * 10.0 * 4.0, 1e-4), "{}", r.vol);
}
