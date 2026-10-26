//! `scale`: volumes (the determinant of the factors), validity (pcurves included), persistent names, the fixed point,
//! and the refusals that leave the body as it was.

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
use ck_core::linalg::Vec3;
use ck_model::Part;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
    volume: f64,
    com: Vec3,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn faces(&self) -> Vec<String> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        v.sort();
        v
    }
    fn bbox(&self) -> ([f64; 3], [f64; 3]) {
        let (a, b) = self.body.as_ref().unwrap().bbox();
        ([a.x, a.y, a.z], [b.x, b.y, b.z])
    }
    fn valid(&self) -> bool {
        ck_check::validate(self.body.as_ref().unwrap(), 1e-6).is_empty()
    }
    fn first_error(&self) -> Option<&ck_model::regen::FeatureError> {
        self.result.errors.first().map(|e| &e.error)
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let mp = ex.brep.body.as_ref().map(|b| ck_topo::mass::mass_props(b));
    Run { result, body: ex.brep.body.clone(), volume: mp.as_ref().map_or(0.0, |m| m.volume), com: mp.map_or(Vec3::ZERO, |m| m.com) }
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

const BLOCK: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// Cylinder of radius 5, height 30 along y.
const ROD: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"c1","c":"c","r":5}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}"#;

/// The same rod with a 10 degree draft: a cone.
const CONE: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"c1","c":"c","r":5}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30,"draft":10}"#;

/// The block with a 6 mm hole through it along y.
const HOLED: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
 {"type":"sketch","id":"F3","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":20,"y":10}],"entities":[{"type":"circle","id":"c1","c":"c","r":3}],"constraints":[]},
 {"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth","depth":0}"#;

const SPHERE: &str = r#"{"type":"sphere","id":"F1","op":"add","center":[1,2,3],"radius":10}"#;

fn scaled(base: &str, scale: &str) -> Run {
    run(&format!(r#"{base},{{"type":"scale","id":"F9",{scale}}}"#))
}

/// Scaled by `f` along the axes: no error, the volume times the product, a valid body, the same face names.
fn check_exact(base: &str, scale: &str, f: [f64; 3]) {
    let before = run(base);
    let r = scaled(base, scale);
    assert!(r.errors().is_empty(), "{scale}: {:?}", r.errors());
    assert!(close(r.volume, before.volume * f[0] * f[1] * f[2], 1e-9), "{scale}: volume {} vs {}", r.volume, before.volume * f[0] * f[1] * f[2]);
    assert!(r.valid(), "{scale}: {:?}", ck_check::validate(r.body.as_ref().unwrap(), 1e-6));
    assert_eq!(r.faces(), before.faces(), "{scale}: persistent names survive");
}

/// Refused with a detail and a hint, and the body is as it was.
fn check_refused(base: &str, scale: &str) {
    let before = run(base);
    let r = scaled(base, scale);
    let e = r.first_error().unwrap_or_else(|| panic!("{scale}: not refused"));
    assert_eq!(e.code, "invalidValue", "{scale}");
    let text = e.describe();
    assert!(text.contains("detail") && text.contains("hint"), "{scale}: {text}");
    assert!(close(r.volume, before.volume, 1e-12), "{scale}: the body changed");
    assert_eq!(r.faces(), before.faces());
}

#[test]
fn a_block_scales_along_each_axis_and_all_of_them() {
    for f in [[2.0, 1.0, 1.0], [1.0, 3.0, 1.0], [1.0, 1.0, 0.5], [1.5, 2.0, 0.75], [2.0, 2.0, 2.0]] {
        check_exact(BLOCK, &format!(r#""factor":[{},{},{}],"at":[0,0,0]"#, f[0], f[1], f[2]), f);
    }
}

#[test]
fn the_fixed_point_stays_put() {
    let before = run(BLOCK);
    let (lo, hi) = before.bbox();
    let r = scaled(BLOCK, r#""factor":2,"at":[10,0,0]"#);
    let (rlo, rhi) = r.bbox();
    assert!(close(rlo[0], 10.0 + 2.0 * (lo[0] - 10.0), 1e-9) && close(rhi[0], 10.0 + 2.0 * (hi[0] - 10.0), 1e-9), "{:?}", r.bbox());
    assert!(close(rlo[1], 2.0 * lo[1], 1e-9) && close(rhi[2], 2.0 * hi[2], 1e-9));
}

#[test]
fn without_a_point_it_scales_about_the_centre_of_mass() {
    let before = run(BLOCK);
    for s in [r#""factor":[2,1,3]"#, r#""factor":[2,1,3],"about":"centroid""#] {
        let r = scaled(BLOCK, s);
        assert!(r.errors().is_empty(), "{:?}", r.errors());
        assert!((r.com - before.com).len() < 1e-9, "{s}: {:?} vs {:?}", r.com, before.com);
    }
    let o = scaled(BLOCK, r#""factor":2,"about":"origin""#);
    assert!((o.com - before.com * 2.0).len() < 1e-9);
}

#[test]
fn a_cylinder_stretches_along_its_axis_and_scales_uniformly() {
    check_exact(ROD, r#""factor":[1,1.5,1],"at":[0,0,0]"#, [1.0, 1.5, 1.0]);
    check_exact(ROD, r#""factor":[1,0.4,1],"at":[3,7,1]"#, [1.0, 0.4, 1.0]);
    check_exact(ROD, r#""factor":2,"at":[0,0,0]"#, [2.0, 2.0, 2.0]);
    check_exact(ROD, r#""factor":[2,1,2],"at":[0,0,0]"#, [2.0, 1.0, 2.0]);
    let r = scaled(ROD, r#""factor":[1,1.5,1],"at":[0,0,0]"#);
    let (lo, hi) = r.bbox();
    assert!(close(hi[1] - lo[1], 45.0, 1e-9) && close(hi[0] - lo[0], 10.0, 1e-6), "{:?}", r.bbox());
}

#[test]
fn a_cone_keeps_its_shape_under_each_exact_scale() {
    check_exact(CONE, r#""factor":[1,1.5,1],"at":[0,0,0]"#, [1.0, 1.5, 1.0]);
    check_exact(CONE, r#""factor":[2,1,2],"at":[0,0,0]"#, [2.0, 1.0, 2.0]);
    check_exact(CONE, r#""factor":3,"at":[1,2,3]"#, [3.0, 3.0, 3.0]);
}

#[test]
fn a_cylinder_stretched_across_its_axis_is_refused() {
    check_refused(ROD, r#""factor":[2,1,1],"at":[0,0,0]"#);
    check_refused(ROD, r#""factor":[1,1,3],"at":[0,0,0]"#);
    check_refused(ROD, r#""factor":[2,1,3],"at":[0,0,0]"#);
    check_refused(CONE, r#""factor":[2,1,1],"at":[0,0,0]"#);
}

#[test]
fn a_block_with_a_hole_along_y() {
    check_exact(HOLED, r#""factor":[1,2,1],"at":[0,0,0]"#, [1.0, 2.0, 1.0]);
    check_exact(HOLED, r#""factor":[2,2,2],"at":[5,5,5]"#, [2.0, 2.0, 2.0]);
    check_exact(HOLED, r#""factor":[2,1,2],"at":[0,0,0]"#, [2.0, 1.0, 2.0]);
    check_refused(HOLED, r#""factor":[2,1,1],"at":[0,0,0]"#);
}

#[test]
fn a_sphere_scales_uniformly_only() {
    check_exact(SPHERE, r#""factor":2,"at":[0,0,0]"#, [2.0, 2.0, 2.0]);
    check_exact(SPHERE, r#""factor":0.3,"at":[1,2,3]"#, [0.3, 0.3, 0.3]);
    check_refused(SPHERE, r#""factor":[1,2,1],"at":[0,0,0]"#);
}

#[test]
fn bad_factors_and_no_body() {
    for s in [r#""factor":0"#, r#""factor":-1"#, r#""factor":[1,-2,1]"#, r#""factor":[1,2]"#, r#""factor":"x""#, r#""at":[0,0,0]"#] {
        let r = scaled(BLOCK, s);
        assert_eq!(r.first_error().map(|e| e.code.as_str()), Some("invalidValue"), "{s}: {:?}", r.errors());
    }
    let r = run(r#"{"type":"scale","id":"F1","factor":2}"#);
    assert_eq!(r.first_error().map(|e| e.code.as_str()), Some("nothingToCut"));
}

#[test]
fn factor_one_changes_nothing() {
    let before = run(BLOCK);
    let r = scaled(BLOCK, r#""factor":1"#);
    assert!(r.errors().is_empty());
    assert!(close(r.volume, before.volume, 1e-12));
}
