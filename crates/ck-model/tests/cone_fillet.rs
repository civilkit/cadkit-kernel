//! A cone made by lofting two coaxial circles (an exact cone), thickened into a wall on a plate, with
//! fillets on its rims. The loft emits `Surface::Cone`, so thicken and the blends take the analytic paths.
use ck_geom::surface::Surface;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize { 0 }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec { brep: Brep, labels: Noop }
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> { Some(&mut self.labels) }
    fn face_count(&self) -> Option<usize> { self.brep.face_count() }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> { self.brep.face_frame(s, face) }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> { self.brep.exec(s, f) }
}

struct Built {
    errors: Vec<String>,
    volume: f64,
    issues: usize,
    cones: usize,
    freeform: usize,
}

/// Regen a fixture.
fn build(src: &str) -> Built {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let b = ex.brep.body.as_ref().expect("a body");
    let (mut cones, mut freeform) = (0, 0);
    for f in b.face_ids() {
        match b.face(f).surface {
            Surface::Cone { .. } => cones += 1,
            Surface::BSpline(_) => freeform += 1,
            _ => {}
        }
    }
    Built { errors, volume: ck_topo::mass::mass_props(b).volume, issues: ck_topo::check::check(b, 1e-6).len(), cones, freeform }
}

/// 55 x 55 x 3 plate with R6 corners.
fn plate_volume() -> f64 {
    55.0 * 55.0 * 3.0 - 4.0 * (1.0 - PI / 4.0) * 36.0 * 3.0
}

/// Volume of a conical wall of thickness `t` offset outward from the frustum sheet (`rb` -> `rt`, height `h`):
/// lateral area x t plus the curvature term pi t^2 h (the outer layer is longer than the sheet).
fn cone_wall_volume(rb: f64, rt: f64, h: f64, t: f64) -> f64 {
    let l = (h * h + (rb - rt) * (rb - rt)).sqrt();
    PI * (rb + rt) * l * t + PI * t * t * h
}

#[test]
fn loft_of_coaxial_circles_is_an_analytic_cone() {
    // solid loft R22 -> R17 on the plate: the two lateral halves fuse into the plate as cone faces
    let b = build(include_str!("../../../tools/cases/cone_fuse/loft_r22_17.json"));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_eq!(b.issues, 0);
    assert!(b.cones >= 1 && b.freeform == 0, "cones {} free-form {}", b.cones, b.freeform);
}

#[test]
fn thickened_cone_sheet_on_plate_matches_area_times_thickness() {
    let b = build(include_str!("../../../tools/cases/cone_fillet/thicken_cone.json"));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_eq!(b.issues, 0);
    assert_eq!(b.freeform, 0, "no free-form face is left");
    let want = plate_volume() + cone_wall_volume(18.5, 11.5, 20.0, 2.0);
    assert!((b.volume - want).abs() < 1e-3 * want, "{} vs {}", b.volume, want);
}

#[test]
fn thickened_wide_base_cone_r21_5_to_r13_5_matches_area_times_thickness() {
    let b = build(include_str!("../../../tools/cases/cone_fuse/thicken_a21.json"));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_eq!(b.issues, 0);
    assert_eq!(b.freeform, 0);
    let want = plate_volume() + cone_wall_volume(21.5, 13.5, 27.0, 1.0);
    assert!((b.volume - want).abs() < 1e-3 * want, "{} vs {}", b.volume, want);
}

/// A cone that opens upward (R12.5 -> R15, 14 high, 1 thick) fuses onto the plate too. Its offset wall dips into the
/// plate, and a loft in the other direction thickens to the other side: wall volume as the original kernel
/// (10247.5, and plate + lateral area x t - pi t^2 h for the reversed one).
#[test]
fn thickened_flaring_cone_fuses_in_both_loft_directions() {
    let b = build(include_str!("../../../tools/cases/cone_fillet/thicken_flare.json"));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_eq!((b.issues, b.freeform), (0, 0));
    assert!((b.volume - 10247.53).abs() < 1e-3 * 10247.53, "{}", b.volume);
    let r = build(include_str!("../../../tools/cases/cone_fillet/thicken_flare_reversed.json"));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert_eq!((r.issues, r.freeform), (0, 0));
    let l = (14.0f64 * 14.0 + 2.5 * 2.5).sqrt();
    let want = plate_volume() + PI * (12.5 + 15.0) * l - PI * 14.0;
    assert!((r.volume - want).abs() < 1e-3 * want, "{} vs {}", r.volume, want);
}

fn base_volume() -> f64 {
    let b = build(include_str!("../../../tools/cases/cone_fillet/thicken_cone.json"));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    b.volume
}

/// R1 on the top rim of the thickened cone: a convex round-over, removes material (the original kernel agrees to
/// 13211.8785).
#[test]
fn r1_fillet_on_the_top_rim_removes_material() {
    let base = base_volume();
    let b = build(include_str!("../../../tools/cases/cone_fillet/fillet_rim.json"));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_eq!((b.issues, b.freeform), (0, 0));
    assert!(b.volume < base && b.volume > base - 100.0, "{} vs base {}", b.volume, base);
}

/// R3 where the cone meets the plate: a concave fillet tangent to the cone and the plate top, ADDS material
/// (the original kernel rounds the wall base over instead and loses 180).
#[test]
fn r3_fillet_at_the_foot_adds_material() {
    let base = base_volume();
    let b = build(include_str!("../../../tools/cases/cone_fillet/fillet_foot.json"));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_eq!((b.issues, b.freeform), (0, 0));
    assert!(b.volume > base, "{} vs base {}", b.volume, base);
}
