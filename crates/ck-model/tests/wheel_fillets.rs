//! A wheel: a hub (radius 7.5, 6 deep), two opposite quarter-sector plates from radius 7.5 to 50 (5 deep) and a rim ring
//! from radius 50 to 55 (30 deep), the three extrudes fused in that order and then filleted.
//!
//! * `wheel_plates_r5_then_r10_vertical.json`: R5 where the plate tops meet the rim, then R10 on the eight vertical
//!   concave edges (plate side with hub, plate side with rim). The R10 step answers `blendUnsupported` ("concave chain
//!   end is not on a planar face", the end meets the torus of the R5 blend): open, see the ignored test.
//! * `wheel_plates_r10_hub_edge.json`: R10 on the arcs where the plate tops meet the hub, which stands 1 mm above the
//!   plates (radius larger than the face it rolls over). It builds as a pair of tori over the hub's top edge.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl FeatureExec for Exec {
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

/// (regen errors as text, the body, its validity issues)
fn build(src: &str) -> (Vec<String>, ck_topo::body::Body, usize) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let b = ex.0.body.take().expect("a body");
    let issues = ck_topo::check::check(&b, 1e-6).len();
    (errors, b, issues)
}

/// The hub fillet no longer answers `blendOverflow`: it builds, valid, as two tori (one per plate arc).
#[test]
fn r10_on_the_plate_to_short_hub_edge_builds_valid() {
    let (errors, b, issues) = build(include_str!("fixtures/wheel_plates_r10_hub_edge.json"));
    assert!(errors.is_empty(), "{errors:?}");
    assert_eq!(issues, 0);
    let tori = b.face_ids().filter(|&f| matches!(b.face(f).surface, ck_geom::surface::Surface::Torus { .. })).count();
    assert_eq!(tori, 2, "one torus per plate arc");
}

/// Open (issue 35): R10 on the vertical concave edges after R5 where the plates meet the rim.
#[test]
#[ignore = "open: blendUnsupported, a concave vertical blend ends on the torus of the R5 blend (blend/ ends only on planes)"]
fn r10_vertical_edges_after_r5_builds() {
    let (errors, _b, issues) = build(include_str!("fixtures/wheel_plates_r5_then_r10_vertical.json"));
    assert!(errors.is_empty(), "{errors:?}");
    assert_eq!(issues, 0);
}
