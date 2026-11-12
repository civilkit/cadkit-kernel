//! Prisms built from 74-sided polygons next to a disc: a `combine` intersect with `keepTools` keeps the tool's name
//! (the result piece and the kept tool both grow from the tool's faces, so a name given back by origin feature went to
//! one of them only), and a union of two identical coincident prisms merges them into one body.
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

/// Run the first `n` features; (errors, [(body name, volume)] in lump order).
fn run(json: &str, n: usize) -> (Vec<String>, Vec<(Option<String>, f64)>) {
    let mut part = Part::parse(json).unwrap();
    part.features.truncate(n);
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let b = ex.0.body.clone().unwrap();
    let mut out = vec![];
    for c in &ck_topo::region::components(&b) {
        let name = b.lump(b.face_lump(c[0])).name.clone();
        let sub = ck_ops::boolean::assemble::rebuild_in_order(&b, c);
        out.push((name, ck_topo::mass::mass_props(&sub).volume));
    }
    (errs, out)
}

fn check(bodies: &[(Option<String>, f64)], want: &[(&str, f64)]) {
    assert_eq!(bodies.len(), want.len(), "{bodies:?}");
    for (n, v) in want {
        let b = bodies.iter().find(|b| b.0.as_deref() == Some(n)).unwrap_or_else(|| panic!("no body {n}: {bodies:?}"));
        assert!((b.1 - v).abs() < 0.01 * v, "{n}: {bodies:?}");
    }
}

/// Features: three sketch + newBody extrudes (b8 disc, b9, b10), a union of b10 into b9, then b9 and b10 each
/// intersected with the disc, which stays as a tool.
#[test]
fn keep_tools_intersect_keeps_the_tool_name() {
    let json = include_str!("fixtures/union_identical_prisms.json");
    let (e, bodies) = run(json, 6);
    assert!(e.is_empty(), "{e:?}");
    check(&bodies, &[("b8", 1038.69), ("b9", 43.87), ("b10", 43.87)]);
    // the prisms are 5 mm apart, so the union leaves two bodies
    let (e, bodies) = run(json, 7);
    assert!(e.is_empty(), "{e:?}");
    check(&bodies, &[("b8", 1038.69), ("b9", 43.87), ("b10", 43.87)]);
    let (e, bodies) = run(json, 8);
    assert!(e.is_empty(), "{e:?}");
    check(&bodies, &[("b8", 1038.69), ("b9", 21.63), ("b10", 43.87)]);
    let (e, bodies) = run(json, 9);
    assert!(e.is_empty(), "{e:?}");
    check(&bodies, &[("b8", 1038.69), ("b9", 21.63), ("b10", 21.63)]);
}

#[test]
fn identical_coincident_prisms_union_into_one_body() {
    let (e, bodies) = run(include_str!("fixtures/union_identical_coincident.json"), 7);
    assert!(e.is_empty(), "{e:?}");
    check(&bodies, &[("b8", 1038.69), ("b9", 43.87)]);
}

/// A disc that is the tool of four keepTools intersects, then deleted: the four result bodies keep their names.
#[test]
fn delete_body_of_a_kept_tool_keeps_the_other_names() {
    let json = include_str!("fixtures/delete_tool_keeps_names.json");
    let (e, bodies) = run(json, 14);
    assert!(e.is_empty(), "{e:?}");
    assert_eq!(bodies.len(), 5, "{bodies:?}");
    let (e, bodies) = run(json, 15);
    assert!(e.is_empty(), "{e:?}");
    assert_eq!(bodies.len(), 4, "{bodies:?}");
    for n in ["b9", "b10", "b11", "b12"] {
        assert!(bodies.iter().any(|b| b.0.as_deref() == Some(n)), "no body {n}: {bodies:?}");
    }
    assert!(bodies.iter().all(|b| b.0.is_some()), "{bodies:?}");
}
