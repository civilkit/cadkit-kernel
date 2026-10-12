//! ACIS SAT import: hand-written fixtures (tests/fixtures/sat) must give a
//! valid closed body with the closed-form volume.

use ck_io::sat::import_sat;
use std::f64::consts::PI;

fn fixture(name: &str) -> String {
    let p = format!("{}/tests/fixtures/sat/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"))
}

fn check(name: &str, vol: f64, faces: usize) {
    let imp = import_sat(&fixture(name)).unwrap_or_else(|e| panic!("{name}: {e}"));
    assert!(!imp.sheet);
    assert_eq!(imp.body.num_faces(), faces, "{name} face count");
    let issues = ck_check::validate(&imp.body, 1e-6);
    assert!(issues.is_empty(), "{name}: {issues:?}");
    let m = ck_topo::mass::lump_moments(&imp.body);
    assert_eq!(m.len(), 1);
    let rel = (m[0].volume - vol).abs() / vol;
    assert!(rel < 1e-6, "{name}: volume {} vs {vol} (rel {rel:e})", m[0].volume);
}

#[test]
fn box_fixture() {
    check("box.sat", 10.0 * 20.0 * 30.0, 6);
}

#[test]
fn cylinder_with_hole_fixture() {
    check("cylinder_hole.sat", PI * (10.0 * 10.0 - 4.0 * 4.0) * 20.0, 4);
}

#[test]
fn plate_with_cylindrical_strip_fixture() {
    // 40 x 20 x 5 plate, radius-3 quarter-round on one long top edge
    let vol = 40.0 * (20.0 * 5.0 - 9.0 * (1.0 - PI / 4.0));
    check("plate_fillet.sat", vol, 7);
}

#[test]
fn box_with_nurbs_top_and_nubs_edge() {
    check("box_spline.sat", 6000.0, 6);
}

/// Loop-less closed sphere / torus faces import as the right surface kinds.
/// (`ck-topo::mass` does not integrate loop-less faces - same for a revolved
/// full sphere - so the only validator finding is the volume one.)
#[test]
fn closed_sphere_and_torus_import() {
    for (name, kind) in [("sphere.sat", "sphere"), ("torus.sat", "torus")] {
        let imp = import_sat(&fixture(name)).unwrap();
        assert_eq!(imp.body.num_faces(), 1);
        let f = imp.body.face_ids().next().unwrap();
        assert_eq!(imp.body.face(f).surface.kind(), kind);
        let issues = ck_check::validate(&imp.body, 1e-6);
        assert!(issues.iter().all(|i| i.code == "VOLUME_NONPOSITIVE"), "{name}: {issues:?}");
    }
}

#[test]
fn cone_frustum_fixture() {
    check("frustum.sat", PI * 9.0 / 3.0 * (36.0 + 18.0 + 9.0), 3);
}

#[test]
fn units_scale_is_applied() {
    let t = fixture("box.sat").replacen("\n1 1e-06 1e-10\n", "\n25.4 1e-06 1e-10\n", 1);
    let imp = import_sat(&t).unwrap();
    let m = ck_topo::mass::lump_moments(&imp.body);
    let want = 6000.0 * 25.4f64.powi(3);
    assert!((m[0].volume - want).abs() / want < 1e-9);
    assert!(ck_check::validate(&imp.body, 1e-6).is_empty());
}

#[test]
fn unsupported_surface_names_the_record() {
    let t = fixture("box.sat");
    let t = t.replacen("plane-surface", "mesh-surface", 1);
    let e = import_sat(&t).unwrap_err();
    assert_eq!(e.code, "sat.unsupported");
    assert!(e.entity.is_some());
    assert_eq!(e.kind, "mesh-surface");
}

#[test]
fn unsupported_spline_block_names_the_record() {
    let t = "700 0 1 0\n@1 x\n1 1e-06 1e-10\n\
        body $-1 -1 $-1 $1 $-1 $-1 #\nlump $-1 -1 $-1 $-1 $2 $0 #\nshell $-1 -1 $-1 $-1 $-1 $3 $-1 $1 #\n\
        face $-1 -1 $-1 $-1 $-1 $2 $-1 $4 forward single in #\n\
        spline-surface $-1 -1 $-1 forward { sumsur 1 2 3 } I I I I #\nEnd-of-ACIS-data\n";
    let e = import_sat(t).unwrap_err();
    assert_eq!(e.code, "sat.unsupported");
    assert_eq!(e.entity, Some(4));
}

#[test]
fn malformed_input_never_panics() {
    let good = fixture("cylinder_hole.sat");
    for cut in [0usize, 10, 40, 100, 300, 700, 1200, 1800] {
        let _ = import_sat(&good[..cut.min(good.len())]);
    }
    let _ = import_sat("");
    let _ = import_sat("garbage\n\n\n$$$ # # #");
    let _ = import_sat(&good.replace("$5", "$999"));
    let _ = import_sat(&good.replace("forward", "sideways"));
}

#[test]
fn nubs_curve_and_surface_blocks_parse() {
    // a bilinear nubs patch face is not closed, but the surface block itself must be accepted
    // (error, if any, must come from topology / vertices, not from the spline parser)
    let t = "700 0 1 0\n@1 x\n1 1e-06 1e-10\n\
        body $-1 -1 $-1 $1 $-1 $-1 #\nlump $-1 -1 $-1 $-1 $2 $0 #\nshell $-1 -1 $-1 $-1 $-1 $3 $-1 $1 #\n\
        face $-1 -1 $-1 $-1 $-1 $2 $-1 $4 forward single in #\n\
        spline-surface $-1 -1 $-1 forward { exactsur full nubs 1 1 open open none none 2 2\n 0 1 1 1\n 0 1 1 1\n\
        0 0 0  0 1 0  1 0 0  1 1 0.5\n 0 0 0 0 0 0 } I I I I #\nEnd-of-ACIS-data\n";
    let e = import_sat(t).unwrap_err();
    assert_ne!(e.code, "sat.parse", "{e}");
    assert_ne!(e.code, "sat.unsupported", "{e}");
}
