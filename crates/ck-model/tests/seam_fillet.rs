//! A fillet on the edge between a slot's back wall and the flat top, which runs
//! from one slot side wall to the other (x = 3.5 .. 9.5) and continues beyond
//! each wall as a smooth tangent rim of the rounded end. The tool must stop
//! flush in the planes of the side walls: it removes exactly (1 - pi/4) r^2 L
//! (L = 6) and the end faces merge into the walls (no ledge, no extra faces).
use ck_geom::Vec3;
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

const BASE: &str = include_str!("../../../tools/cases/user_fillet/slot_seam_r2.ckpart");

/// (volume, area, face count) of the part, optionally without its last (fillet) feature
fn measure(src: &str, drop_fillet: bool) -> (f64, f64, usize) {
    let mut part = Part::parse(src).unwrap();
    if drop_fillet {
        part.features.pop();
    }
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "regen errors: {:?}", r.errors.len());
    let b = ex.0.body.clone().unwrap();
    assert!(ck_topo::check::check(&b, 1e-6).is_empty());
    let m = ck_topo::mass::mass_props(&b);
    (m.volume, m.area, b.face_ids().count())
}

fn variant(radius: f64, bottom: bool) -> String {
    let compact: String = BASE.chars().filter(|c| !c.is_whitespace()).collect();
    let mut s = compact.replace("\"radius\":2", &format!("\"radius\":{radius}"));
    if bottom {
        s = s.replace("\"F2.S.l2\"", "\"F2.S.l1\"").replace("\"at\":[6.5,0,9.5]", "\"at\":[6.5,0,-9.5]");
    }
    s
}

fn check(src: &str, r: f64) {
    let (v0, a0, n0) = measure(src, true);
    let (v1, a1, n1) = measure(src, false);
    let corner = (1.0 - std::f64::consts::FRAC_PI_4) * r * r;
    let len = 6.0;
    assert!((v0 - v1 - corner * len).abs() < 1e-6, "removed {} expected {}", v0 - v1, corner * len);
    // quarter cylinder replaces two r x L strips; each end wall gains the corner area
    let da = std::f64::consts::FRAC_PI_2 * r * len - 2.0 * r * len + 2.0 * corner;
    assert!((a1 - a0 - da).abs() < 1e-6, "area change {} expected {}", a1 - a0, da);
    assert_eq!(n1, n0 + 1, "one blend face, the ends merge into the slot walls");
    let _ = Vec3::ZERO;
}

#[test]
fn slot_seam_r2() {
    check(&variant(2.0, false), 2.0);
}
#[test]
fn slot_seam_r1() {
    check(&variant(1.0, false), 1.0);
}
#[test]
fn slot_seam_r3() {
    check(&variant(3.0, false), 3.0);
}
#[test]
fn slot_seam_mirrored_bottom_edge() {
    let s = variant(2.0, true);
    assert_ne!(s, variant(2.0, false));
    check(&s, 2.0);
}
#[test]
fn slot_seam_box_with_one_rounded_end() {
    // the far end is a flat wall instead of a second round
    let compact: String = BASE.chars().filter(|c| !c.is_whitespace()).collect();
    let flat = compact.replace("{\"id\":\"a1\",\"type\":\"arc\",\"c\":\"c1\",\"p1\":\"b\",\"p2\":\"d\"}", "{\"id\":\"a1\",\"type\":\"line\",\"p1\":\"b\",\"p2\":\"d\"}");
    assert_ne!(flat, compact);
    check(&flat, 2.0);
}
