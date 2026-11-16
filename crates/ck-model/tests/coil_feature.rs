//! The `coil` feature (see features/solids.rs): a round wire wound as a helix.
mod solids_common;
use solids_common::*;
use std::f64::consts::PI;

fn coil(extra: &str) -> String {
    format!(r#"{{"type":"coil","id":"F1","op":"add",{extra}}}"#)
}

#[test]
fn a_coil_has_the_pappus_volume_and_is_valid() {
    // wire area * length of the centre line (revolutions * 2 pi R), the pitch only tilts the turns slightly
    let (r, w, pitch, revs) = (10.0f64, 1.0f64, 5.0f64, 8.0f64);
    let run1 = run(&coil(r#""center":[0,0,0],"axis":"z","radius":10,"wireRadius":1,"pitch":5,"revolutions":8"#));
    assert!(run1.errors.is_empty(), "{:?}", run1.errors);
    assert!(run1.valid.is_empty(), "{:?}", run1.valid);
    let want = PI * w * w * revs * (2.0 * PI * r);
    assert!(near(run1.vol, want, 2e-3), "{} vs {want}", run1.vol);
    let (lo, hi) = run1.bbox.unwrap();
    // the axis is z: the coil spans radius + wire in x and y, and about revolutions * pitch in z
    assert!(near(hi[0], 11.0, 2e-2) && near(lo[0], -11.0, 2e-2), "{lo:?} {hi:?}");
    assert!((hi[2] - lo[2] - (revs * pitch + 2.0 * w)).abs() < 1.0 + 2.0, "{lo:?} {hi:?}");
    assert!(run1.lumps == 1);
}

#[test]
fn height_gives_the_revolutions_and_hand_does_not_change_the_volume() {
    let a = run(&coil(r#""radius":10,"wireRadius":1,"pitch":5,"height":20"#));
    let b = run(&coil(r#""radius":10,"wireRadius":1,"pitch":5,"revolutions":4,"leftHand":true"#));
    assert!(a.errors.is_empty() && b.errors.is_empty(), "{:?} {:?}", a.errors, b.errors);
    assert!(a.valid.is_empty() && b.valid.is_empty(), "{:?} {:?}", a.valid, b.valid);
    assert!(near(a.vol, b.vol, 1e-6), "{} {}", a.vol, b.vol);
}

#[test]
fn an_axis_along_x_and_a_centre_place_the_coil() {
    let r = run(&coil(r#""center":[100,0,0],"axis":"x","radius":10,"wireRadius":1,"pitch":5,"revolutions":3"#));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    let (lo, hi) = r.bbox.unwrap();
    assert!(lo[0] > 98.0 && hi[0] < 120.0 && (hi[1] - lo[1] - 22.0).abs() < 0.5 && (hi[2] - lo[2] - 22.0).abs() < 0.5, "{lo:?} {hi:?}");
}

#[test]
fn bad_input_is_refused_with_stable_codes() {
    for extra in [
        r#""radius":10,"wireRadius":1,"pitch":5"#,                                   // no revolutions or height
        r#""radius":0,"wireRadius":1,"pitch":5,"revolutions":3"#,
        r#""radius":10,"wireRadius":0,"pitch":5,"revolutions":3"#,
        r#""radius":10,"wireRadius":1,"pitch":-5,"revolutions":3"#,
        r#""radius":10,"wireRadius":1,"pitch":5,"revolutions":0"#,
        r#""radius":10,"wireRadius":1,"pitch":5,"revolutions":100000"#,
        r#""radius":1,"wireRadius":1,"pitch":5,"revolutions":3"#,                   // the wire crosses the axis
        r#""radius":10,"wireRadius":1,"pitch":2,"revolutions":3"#,                   // turns overlap
        r#""radius":10,"wireRadius":1,"pitch":5,"revolutions":3,"axis":[0,0,0]"#,
    ] {
        let r = run(&coil(extra));
        assert_eq!(r.codes, ["invalidValue"], "{extra}: {:?}", r.errors);
    }
    let r = run(r#"{"type":"coil","id":"F1","op":"cut","radius":10,"wireRadius":1,"pitch":5,"revolutions":3}"#);
    assert_eq!(r.codes, ["nothingToCut"]);
}

#[test]
fn a_coil_cut_from_a_block_leaves_a_helical_groove() {
    let r = run(&format!("{BLOCK},{}", r#"{"type":"coil","id":"F2","op":"cut","radius":10,"wireRadius":1,"pitch":5,"revolutions":3}"#));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(r.valid.is_empty(), "{:?}", r.valid);
    // the wire lies fully inside the 40-high block (it spans z 0..15 + 1)
    assert!(near(r.vol, 64000.0 - PI * 3.0 * 2.0 * PI * 10.0, 5e-3), "{}", r.vol);
}
