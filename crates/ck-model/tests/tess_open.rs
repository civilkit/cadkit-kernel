//! Display-mesh watertightness of bodies whose B-rep checks pass: every face meshes, and no mesh edge is left used by a
//! single triangle apart from hairline cracks between two faces' samples of a shared edge (`ck_check::tessellation`).
//! `CK_TESS_DBG=1` prints why a face dropped and which faces had unrecovered trim segments.
use ck_check::tessellation::check_mesh_why;
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

fn open_issues(json: &str) -> Vec<String> {
    std::env::set_var("CK_TESS_DBG", "1");
    let part = Part::parse(json).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let body = ex.0.body.clone().unwrap();
    let m = ck_tess::mesh::tessellate(&body).mesh;
    check_mesh_why(&body, &m.positions, &m.tri_face, &m.failed, &m.failed_why).into_iter().map(|i| format!("{}: {}", i.entity, i.detail)).collect()
}

/// A crescent-shaped cap (a circle and a surface intersection that touch at both ends) whose trim polylines cross by a few 1e-6.
#[test]
fn sliver_cap_meshes() {
    let issues = open_issues(include_str!("fixtures/tess_cap_sliver.json"));
    assert!(issues.is_empty(), "{issues:?}");
}

/// A bolt hole in a flange face: two long triangles bridged the hole.
#[test]
fn bolt_hole_is_not_bridged() {
    let issues = open_issues(include_str!("fixtures/tess_hole_sliver.json"));
    assert!(issues.is_empty(), "{issues:?}");
}
