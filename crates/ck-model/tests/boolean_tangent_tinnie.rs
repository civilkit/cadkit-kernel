//! The kit's Tinnie hull sample (a lofted, shelled hull with floors, keel strips and bulkheads), cut down
//! to the features that made the later booleans fail.
//!
//! - `floor_bulkhead`: the last floor (F41) sits on the inner bottom where the loft dips 1 mm, so its side
//!   wall and its base meet the hull at one vertex. The edge of the floor wall along the hull ends a hair
//!   short of that vertex, so every later boolean split that edge at its own vertex: a piece with no length
//!   from the vertex to itself. The loop then ran through the vertex twice, the shell's Euler characteristic
//!   went odd and the bulkhead F48 (and every other later extrude of the sample) was refused as
//!   `booleanTangent`. The split is dropped now (imprint), and F48 builds a valid solid.
//! - `keel_floors`: the keel strip F38 meets the inner bottom along a cut that is nearly, but not exactly, an
//!   iso-line of the loft. Its pcurve was taken as a straight uv line because the uv samples were within 1e-6,
//!   which on a 3.5 m face is 1.3e-3 mm off the edge (EDGE_OFF_SURFACE). The line pcurve is now also checked
//!   in space.
//!
//! Run with `CK_BOOL_DBG=1` to print the arrangements.
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

/// Regen errors, the volume and the validation issues of a document.
fn build(src: &str) -> (Vec<String>, f64, Vec<String>) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{} {} {:?}", e.feature, e.error.code, e.error.describe())).collect();
    let b = ex.0.body.expect("a body");
    let issues = ck_check::validate(&b, 1e-6).iter().map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect();
    (errors, ck_topo::mass::mass_props(&b).volume, issues)
}

#[test]
fn bulkhead_on_a_hull_with_a_floor_in_the_dip_builds_valid() {
    let src = include_str!("../../../tools/cases/tinnie_tangent/floor_bulkhead.ckpart");
    let (errors, volume, issues) = build(src);
    assert!(errors.is_empty(), "{errors:?}");
    assert!(issues.is_empty(), "{issues:?}");
    // without the bulkhead (F46..F48) the body is 14,514,604 mm^3; the bulkhead plate is 2 mm thick with
    // ~373,000 mm^2 inside the hull
    let without = {
        let mut p = ck_json::parse(src).unwrap();
        if let Some(ck_json::Json::Arr(fs)) = p.get_mut("features") {
            fs.retain(|f| !matches!(f.get("id"), Some(ck_json::Json::Str(s)) if s == "F46" || s == "F47" || s == "F48"));
        }
        build(&p.to_string()).1
    };
    let added = volume - without;
    assert!((added - 2.0 * 373_000.0).abs() < 10_000.0, "bulkhead added {added} mm^3 (with {volume}, without {without})");
}

#[test]
fn keel_strip_on_a_lofted_bottom_has_edges_on_the_surface() {
    let (errors, volume, issues) = build(include_str!("../../../tools/cases/tinnie_tangent/keel_floors.ckpart"));
    assert!(errors.is_empty(), "{errors:?}");
    assert!(issues.is_empty(), "{issues:?}");
    assert!(volume > 1.0e7, "{volume}");
}
