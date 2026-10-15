//! BenchCAD propeller (hub + four lofted ellipse blades that overlap near the axis): crossed thin blades meet in
//! singular intersection points and touch tangentially at their root tips.
//!
//! Root causes fixed here:
//! * two intersection curves of ONE face pair crossing in their interiors (the branches of two equal tubes meeting
//!   symmetrically) were not split at the crossing, so the face walk met four edges on one edge (imprint.rs
//!   `cut_crossings`);
//! * a fragment that is a sliver lens between two near-tangent surfaces (blade tips of the fourth blade) has its
//!   sample point within ~1e-3 of the other solid's boundary, where single rays disagree; the first ray won and
//!   the lens was kept on both sides of a cut (non-manifold). `Solid::classify` now votes over all rays (and falls
//!   back to the winding number) for samples that close to the boundary.
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

/// The part with its `cpattern` changed to `count` copies, `step` degrees apart (`equal: false`), and optionally without the hub.
fn run(count: u32, step: f64, hub: bool) -> (Vec<(String, String)>, f64, usize) {
    let path = format!("{}/../../tools/bench/benchcad/propeller_000192.ckpart.json", env!("CARGO_MANIFEST_DIR"));
    let src = std::fs::read_to_string(path).unwrap();
    let src = if hub {
        src
    } else {
        let a = src.find("{\n   \"type\": \"sketch\",\n   \"id\": \"F1\"").unwrap();
        let b = src.find("{\n   \"type\": \"sketch\",\n   \"id\": \"F3\"").unwrap();
        format!("{}{}", &src[..a], &src[b..])
    };
    let src = src
        .replace("\"equal\": true", "\"equal\": false")
        .replace("\"count\": 4", &format!("\"count\": {count}"))
        .replace("\"angle\": 360,\n   \"equal\"", &format!("\"angle\": {step},\n   \"equal\""));
    let part = Part::parse(&src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect();
    let b = ex.0.body.as_ref().unwrap();
    (errs, ck_topo::mass::mass_props(b).volume, ck_check::validate(b, 1e-6).len())
}

/// One blade alone, no pattern copies.
const BLADE: f64 = 1468.7320728488512;

#[test]
fn two_crossed_blades_union() {
    // 90 degrees apart: their lateral faces cross along two curves that intersect each other (singular point)
    let (errs, vol, bad) = run(2, 90.0, false);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(bad, 0);
    assert!(vol > BLADE && vol < 2.0 * BLADE, "{vol}");
    assert!((vol - 2802.7676550025085).abs() < 1e-6 * vol, "{vol}");
}

#[test]
fn four_blades_without_hub() {
    // the fourth blade touches the second one tangentially at the root tips (lens-shaped sliver of 7e-4)
    let (errs, vol, bad) = run(4, 90.0, false);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(bad, 0);
    // same union in another order (0, 180 first), which never hit the tangent tip
    assert!((vol - 5336.454561997).abs() < 1e-6 * vol, "{vol}");
}

#[test]
fn propeller_with_hub_is_valid_and_close_to_occt() {
    let (errs, vol, bad) = run(4, 90.0, true);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(bad, 0);
    // reference (CadQuery/OCCT): 23868.815832807737
    assert!((vol - 23868.815832807737).abs() < 1e-3 * 23868.8, "volume {vol}");
}
