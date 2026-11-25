//! The builder on the synthetic ASM blobs of `ck_asm::testdata` (a cube, a cylinder, a cone with
//! its apex, bodies with faces the builder cannot make).

use ck_asm::import::{ImportOptions, import_smb};
use ck_asm::testdata;
use ck_heal::honest::Outcome;

fn import(blob: Vec<u8>) -> (ck_asm::import::Import, ck_heal::honest::Sorted) {
    let imp = import_smb(&blob, &ImportOptions::default()).unwrap();
    let sorted = imp.sort();
    (imp, sorted)
}

fn volume(imp: &ck_asm::import::Import) -> f64 {
    ck_topo::mass::mass_props(&imp.body).volume
}

#[test]
fn a_cube_is_a_solid_in_millimetres() {
    let (imp, s) = import(testdata::cube_blob());
    assert_eq!(imp.bodies.len(), 1);
    assert_eq!(s.parts[0].outcome, Outcome::Solid, "{:?}", s.parts[0].reasons);
    assert_eq!(imp.body.num_faces(), 6);
    let (lo, hi) = imp.body.bbox();
    assert!((hi.x - lo.x - 10.0).abs() < 1e-9 && (hi.y - lo.y - 10.0).abs() < 1e-9 && (hi.z - lo.z - 10.0).abs() < 1e-9, "{lo:?} {hi:?}");
    assert!((volume(&imp) - 1000.0).abs() < 1e-6);
}

#[test]
fn a_cylinder_gets_its_seam() {
    let (imp, s) = import(testdata::cylinder_blob());
    assert_eq!(s.parts[0].outcome, Outcome::Solid, "{:?}", s.parts[0].reasons);
    // two caps and the side, whose two loops are joined by an inserted seam
    assert_eq!(imp.body.num_faces(), 3);
    let side = imp.body.face_ids().find(|f| imp.body.face(*f).loops.len() == 1 && imp.body.face(*f).surface.kind() == "cylinder").unwrap();
    assert_eq!(imp.body.lp(imp.body.face(side).loops[0]).coedges.len(), 4);
    let v = volume(&imp);
    assert!(v > 0.0);
    let tess = ck_tess::mesh::tessellate(&imp.body);
    assert!(tess.mesh.failed.is_empty());
}

#[test]
fn a_cone_with_its_apex_is_closed_by_a_seam_to_the_apex() {
    let (imp, s) = import(testdata::cone_blob());
    assert_eq!(s.parts[0].outcome, Outcome::Solid, "{:?}", s.parts[0].reasons);
    assert_eq!(imp.body.num_faces(), 2);
    let tess = ck_tess::mesh::tessellate(&imp.body);
    assert!(tess.mesh.failed.is_empty(), "{:?}", tess.mesh.failed_why);
    // a cone of height 20 mm (apex at z = 20) on its base: V = pi r^2 h / 3, r the base's radius
    let (lo, hi) = imp.body.bbox();
    let r = (hi.x - lo.x) / 2.0;
    let expected = std::f64::consts::PI * r * r * 20.0 / 3.0;
    assert!((hi.z - lo.z - 20.0).abs() < 1e-9, "{lo:?} {hi:?}");
    assert!((volume(&imp) - expected).abs() < 1e-6 * expected, "{} vs {expected}", volume(&imp));
}

#[test]
fn a_face_on_a_surface_that_does_not_decode_is_left_out_and_the_body_is_not_a_solid() {
    let blob = testdata::cylinder_blob_with(testdata::CylinderVariant { unknown_side: true, ..Default::default() });
    let (imp, s) = import(blob);
    let b = &imp.bodies[0];
    assert_eq!(b.skipped.len(), 1);
    assert_eq!(b.skipped[0].code, "unknownGeometry");
    assert_eq!((b.faces_built, b.faces_total), (2, 3));
    assert_eq!(s.parts[0].outcome, Outcome::Surface);
    assert!(s.parts[0].reasons.iter().any(|r| r.code == "unknownGeometry" && r.message.contains("was left out")), "{:?}", s.parts[0].reasons);
    // never half a solid: the faces that came in are a surface body
    assert_eq!(s.solid.num_faces(), 0);
    assert_eq!(s.surfaces.len(), 1);
}

#[test]
fn bytes_that_are_not_a_file_are_refused_in_words() {
    assert!(import_smb(b"nothing like an ASM file", &ImportOptions::default()).is_err());
    let e = ck_asm::import::import_f3d(b"PK not really a zip", &ImportOptions::default()).unwrap_err();
    assert_eq!(e.code, "f3d.container");
    let zip = ck_asm::zip::stored_zip(&[("Manifest.dat", b"x")]);
    let e = ck_asm::import::import_f3d(&zip, &ImportOptions::default()).unwrap_err();
    assert_eq!(e.code, "f3d.empty");
}
