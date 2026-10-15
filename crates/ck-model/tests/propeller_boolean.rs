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

/// union + intersection of a blade with its copy turned by `ang` degrees: va + vb = vu + vi to 1e-6 (the pair
/// that overlaps by 134.7 mm3 at 90 degrees: OCCT's own union (145.3) and intersection (143.5) of the same
/// pair disagree by 1.8, a Monte Carlo of the solids with OCCT's point classifier is in the notes of FINDINGS.md).
#[test]
fn blade_pair_union_and_intersection_agree() {
    use ck_core::xform::Xform;
    use ck_geom::Vec3;
    use ck_ops::boolean::{apply, boolean, Op};
    let path = format!("{}/../../tools/bench/benchcad/propeller_000192.ckpart.json", env!("CARGO_MANIFEST_DIR"));
    let src = std::fs::read_to_string(path).unwrap();
    let a = src.find("{\n   \"type\": \"sketch\",\n   \"id\": \"F1\"").unwrap();
    let b = src.find("{\n   \"type\": \"sketch\",\n   \"id\": \"F3\"").unwrap();
    let src = format!("{}{}", &src[..a], &src[b..]).replace("\"count\": 4", "\"count\": 1");
    let part = Part::parse(&src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty());
    let a = ex.0.body.clone().unwrap();
    let va = ck_topo::mass::mass_props(&a).volume;
    for ang in [90.0f64, 180.0, 45.0] {
        let b = apply::instance(&a, &Xform::rotation(Vec3::ZERO, Vec3::X, ang.to_radians()), "X", 1);
        let u = boolean(&a, &b, Op::Union, 1e-6).unwrap().body;
        let i = boolean(&a, &b, Op::Intersect, 1e-6).unwrap().body;
        let (vu, vi) = (ck_topo::mass::mass_props(&u).volume, ck_topo::mass::mass_props(&i).volume);
        assert!((2.0 * va - vu - vi).abs() < 1e-6 * va, "angle {ang}: union {vu} intersection {vi}");
    }
}

/// A union that cannot be closed reports which faces border the open / non-manifold edges and where (welding
/// tolerance 3e-3 is far too coarse for these blades, which makes the second union fail).
#[test]
fn failed_union_reports_faces_and_point() {
    use ck_core::xform::Xform;
    use ck_geom::Vec3;
    use ck_ops::boolean::{apply, boolean, Op};
    let path = format!("{}/../../tools/bench/benchcad/propeller_000192.ckpart.json", env!("CARGO_MANIFEST_DIR"));
    let src = std::fs::read_to_string(path).unwrap();
    let a = src.find("{\n   \"type\": \"sketch\",\n   \"id\": \"F1\"").unwrap();
    let b = src.find("{\n   \"type\": \"sketch\",\n   \"id\": \"F3\"").unwrap();
    let src = format!("{}{}", &src[..a], &src[b..]).replace("\"count\": 4", "\"count\": 1");
    let part = Part::parse(&src).unwrap();
    let mut ex = Exec(Brep::new());
    let _ = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let a = ex.0.body.clone().unwrap();
    let inst = |ang: f64, k| apply::instance(&a, &Xform::rotation(Vec3::ZERO, Vec3::X, ang.to_radians()), "X", k);
    let t = boolean(&a, &inst(90.0, 1), Op::Union, 3e-3).unwrap().body;
    let e = boolean(&t, &inst(180.0, 2), Op::Union, 3e-3).err().expect("the coarse welding tolerance cannot close this union");
    assert!(e.starts_with("booleanFailed;edgeFaces="), "{e}");
    let fe = regen::FeatureError::new(&e);
    assert_eq!(fe.code, "booleanFailed");
    let get = |k: &str| fe.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()).unwrap_or_else(|| panic!("param {k}"));
    assert!(ck_json::to_string(&get("faces")).contains("@X."), "{:?}", fe.params);
    assert!(ck_json::to_string(&get("issue")).contains("Edge"));
    assert_eq!(get("at").as_arr().map(|v| v.len()), Some(3));
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
