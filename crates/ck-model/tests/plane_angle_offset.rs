//! Reference plane: `offset` in angle mode and the error hints.
//!
//! Std planes: `front` = XY (normal +Z, the kit's Top plane), `top` = XZ (normal +Y), `right` = ZY (normal +X).
use ck_core::linalg::Vec3;
use ck_model::regen::{regen, NoGeometry, RegenOpts};
use ck_model::Part;

fn run(feats: &str) -> (Vec<(String, ck_core::linalg::Frame)>, Vec<ck_model::regen::RegenError>) {
    let src = format!(r#"{{"format":"ck-part","version":1,"name":"t","features":[{feats}]}}"#);
    let r = regen(&Part::parse(&src).unwrap(), &RegenOpts::default(), &mut NoGeometry);
    (r.planes.iter().map(|(i, f)| (i.clone(), *f)).collect(), r.errors)
}
fn near(a: Vec3, b: [f64; 3]) -> bool {
    (a - Vec3::new(b[0], b[1], b[2])).len() < 1e-9
}
const TOP: &str = r#""base":{"kind":"std","id":"top"}"#;
const AX: &str = r#""axis":{"kind":"axis","axis":"x"}"#;

fn angle(extra: &str) -> ck_core::linalg::Frame {
    run(&format!(r#"{{"type":"plane","id":"P","mode":"angle",{TOP},{AX},"angle":30{extra}}}"#)).0.pop().unwrap().1
}

#[test]
fn angle_offset_moves_along_tilted_normal() {
    let c = 0.75f64.sqrt();
    let f0 = angle("");
    assert!(near(f0.o, [0., 0., 0.]));
    let f = angle(r#","offset":40"#);
    // same orientation as without offset, origin 40 along the tilted normal (0, c, 0.5)
    assert!(near(f.z, [0., c, 0.5]) && near(f.x, [1., 0., 0.]), "{f:?}");
    assert!(near(f.o, [0., 40. * c, 20.]), "{f:?}");
    // offset 0 equals no offset
    assert!(near(angle(r#","offset":0"#).o, [0., 0., 0.]));
    // expression offset ('=' prefix)
    assert!(near(angle(r#","offset":"=20+20""#).o, [0., 40. * c, 20.]));
}

/// Angle 30 about x on std `front` (XY, normal +Z; the kit's Top plane): z = (0, -0.5, 0.866).
#[test]
fn angle_offset_on_front_plane() {
    let front = r#""base":{"kind":"std","id":"front"}"#;
    let make = |extra: &str| {
        run(&format!(r#"{{"type":"plane","id":"P","mode":"angle",{front},{AX},"angle":30{extra}}}"#)).0.pop().unwrap().1
    };
    let c = 0.75f64.sqrt();
    let f0 = make("");
    assert!(near(f0.o, [0., 0., 0.]) && near(f0.z, [0., -0.5, c]), "{f0:?}");
    let f = make(r#","offset":0"#);
    assert!(near(f.o, [0., 0., 0.]) && near(f.z, [0., -0.5, c]), "{f:?}");
    let f = make(r#","offset":40"#);
    assert!(near(f.z, [0., -0.5, c]) && near(f.x, [1., 0., 0.]), "{f:?}");
    assert!(near(f.o, [0., -20., 40. * c]), "{f:?}"); // (0, -20, 34.641)
}

fn err_params(feats: &str) -> (String, String, String) {
    let e = run(feats).1.remove(0).error;
    let get = |k: &str| e.params.iter().find(|(n, _)| n == k).and_then(|(_, v)| v.as_str().map(|s| s.to_string())).unwrap_or_default();
    (e.code.clone(), get("detail"), get("hint"))
}

#[test]
fn plane_errors_carry_hints() {
    let (c, d, h) = err_params(&format!(r#"{{"type":"plane","id":"P","mode":"angle",{TOP},"angle":30}}"#));
    assert_eq!(c, "missingReference");
    assert!(!d.is_empty() && h.contains("angle needs an axis"), "{d} / {h}");
    let (c, _, h) = err_params(&format!(r#"{{"type":"plane","id":"P","mode":"mid",{TOP}}}"#));
    assert_eq!(c, "missingReference");
    assert!(h.contains("mid needs a second plane"), "{h}");
    let (c, d, h) = err_params(&format!(r#"{{"type":"plane","id":"P","mode":"perpendicular",{TOP},"axis":{{"kind":"axis","axis":"y"}}}}"#));
    assert_eq!(c, "planeLineNormal");
    assert!(!d.is_empty() && h.contains("axis lies along the base normal"), "{d} / {h}");
}
