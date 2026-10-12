//! Hand-written external-style STEP files (tests/fixtures/*.step, not made
//! by our writer): units, bare-circle and seam-style cylinders, trimmed
//! curves, rational B-spline surface + curves, several solids in one file.
use ck_io::step::read_step;
use std::f64::consts::PI;

fn load(name: &str) -> ck_io::step::Imported {
    let p = format!("{}/tests/fixtures/{name}.step", env!("CARGO_MANIFEST_DIR"));
    let t = std::fs::read_to_string(&p).unwrap();
    read_step(&t).unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn check(name: &str, vol: f64, faces: usize, solids: usize) -> ck_io::step::Imported {
    let imp = load(name);
    let m = ck_topo::mass::mass_props(&imp.body);
    assert!((m.volume - vol).abs() <= 1e-6 * vol, "{name}: volume {} vs {vol}", m.volume);
    assert_eq!(imp.body.num_faces(), faces, "{name}: faces");
    assert_eq!(imp.solids, solids, "{name}: solids");
    let issues = ck_check::validate(&imp.body, 1e-6);
    assert!(issues.is_empty(), "{name}: {issues:?}");
    let topo = ck_topo::check::check(&imp.body, 1e-6);
    assert!(topo.is_empty(), "{name}: {topo:?}");
    imp
}

#[test]
fn inch_block_with_bare_circle_hole() {
    let v = (2.0 * 1.0 * 0.5 - PI * 0.25 * 0.25 * 0.5) * 25.4f64.powi(3);
    let imp = check("inch_block_hole", v, 7, 1);
    assert!((imp.length_scale - 25.4).abs() < 1e-12);
}

#[test]
fn occ_cylinder_with_seam_and_second_solid() {
    let imp = check("occ_cylinder_and_box", PI * 25.0 * 8.0 + 60.0, 9, 2);
    assert_eq!(imp.body.num_lumps(), 2);
}

#[test]
fn rational_bspline_quarter_disc() {
    check("nurbs_quarter_disc", PI * 36.0 / 4.0 * 3.0, 5, 1);
}

#[test]
fn unsupported_entity_is_a_structured_error() {
    let t = std::fs::read_to_string(format!("{}/tests/fixtures/nurbs_quarter_disc.step", env!("CARGO_MANIFEST_DIR"))).unwrap();
    // swap the plane of the bottom face for a surface type we do not read
    let bad = t.replacen("PLANE(", "SWEPT_DISC_SURFACE(", 1);
    let e = read_step(&bad).unwrap_err();
    assert_eq!(e.code, "unsupported");
    assert!(e.entity.is_some() && e.message.contains("SWEPT_DISC_SURFACE"), "{e}");
}

#[test]
fn garbage_and_truncated_input_never_panic() {
    assert!(read_step("").is_err());
    assert!(read_step("hello").is_err());
    assert!(read_step("ISO-10303-21;\nDATA;\n#1=;\nENDSEC;").is_err());
    let t = std::fs::read_to_string(format!("{}/tests/fixtures/occ_cylinder_and_box.step", env!("CARGO_MANIFEST_DIR"))).unwrap();
    let step = (t.len() / 97).max(1);
    for cut in (0..t.len()).step_by(step) {
        if t.is_char_boundary(cut) {
            let _ = read_step(&t[..cut]); // Ok or Err, no panic
        }
    }
    // a curve that is its own basis must be an error, not a stack overflow
    let cyc = t.replacen("#15 = TRIMMED_CURVE('',#9,", "#15 = TRIMMED_CURVE('',#15,", 1);
    assert_eq!(read_step(&cyc).unwrap_err().code, "badEntity");
    // dangling references
    let dangling = t.replacen("CLOSED_SHELL('',(", "CLOSED_SHELL('',(#99999,", 1);
    let e = read_step(&dangling).unwrap_err();
    assert_eq!((e.code, e.entity), ("badEntity", Some(99999)));
}
