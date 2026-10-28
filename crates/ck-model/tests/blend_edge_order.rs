//! Property: the order of the `edges` list of a `fillet` or `chamfer` does not change the result. Every permutation of
//! a set of picks must give the same errors, the same face count and the same volume (and a valid body).
//! Block: x 0..40, y 0..10, z -20..0 (sketch on `top`: x, -z; extrude +y), as in blend_legs.rs.
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

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// (errors, face count, volume, check issues) of the box followed by `feature`.
fn run(feature: &str) -> (Vec<String>, usize, f64, usize) {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{BOX},{feature}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = res.errors.iter().map(|e| e.error.code.clone()).collect();
    let b = ex.0.body.as_ref().unwrap();
    (errs, b.num_faces(), ck_topo::mass::mass_props(b).volume, ck_topo::check::check(b, 1e-6).len())
}

fn permutations(items: &[&'static str]) -> Vec<Vec<&'static str>> {
    if items.len() <= 1 {
        return vec![items.to_vec()];
    }
    let mut out = vec![];
    for i in 0..items.len() {
        let mut rest = items.to_vec();
        let head = rest.remove(i);
        for mut p in permutations(&rest) {
            p.insert(0, head);
            out.push(p);
        }
    }
    out
}

/// The four edges of the top face: a closed ring.
const RING: [&str; 4] = [
    r#"{"faces":["F2.C1","F2.S.e1"],"at":[20,10,0]}"#,
    r#"{"faces":["F2.C1","F2.S.e2"],"at":[40,10,-10]}"#,
    r#"{"faces":["F2.C1","F2.S.e3"],"at":[20,10,-20]}"#,
    r#"{"faces":["F2.C1","F2.S.e4"],"at":[0,10,-10]}"#,
];
/// Three edges that meet at the corner (0, 10, 0): two top edges and the vertical edge between the side faces.
const CORNER: [&str; 3] = [
    r#"{"faces":["F2.C1","F2.S.e1"],"at":[20,10,0]}"#,
    r#"{"faces":["F2.C1","F2.S.e4"],"at":[0,10,-10]}"#,
    r#"{"faces":["F2.S.e1","F2.S.e4"],"at":[0,5,0]}"#,
];

fn order_independent(head: &str, picks: &[&'static str]) {
    let mut first: Option<(Vec<String>, usize, f64, usize)> = None;
    for p in permutations(picks) {
        let f = format!(r#"{{{head},"edges":[{}]}}"#, p.join(","));
        let r = run(&f);
        match &first {
            None => {
                assert!(r.0.is_empty(), "{head}: {:?}", r.0);
                assert_eq!(r.3, 0, "{head}: body issues");
                first = Some(r);
            }
            Some(a) => {
                assert_eq!(r.0, a.0, "{head}: errors differ for order {p:?}");
                assert_eq!(r.1, a.1, "{head}: face count differs for order {p:?}");
                assert!((r.2 - a.2).abs() < 1e-6, "{head}: volume {} vs {} for order {p:?}", r.2, a.2);
                assert_eq!(r.3, 0, "{head}: body issues for order {p:?}");
            }
        }
    }
}

#[test]
fn chamfer_ring_is_order_independent() {
    order_independent(r#""type":"chamfer","id":"F3","chamferType":"equal","distance":2"#, &RING);
}

#[test]
fn chamfer_two_distances_ring_is_order_independent() {
    order_independent(r#""type":"chamfer","id":"F3","chamferType":"distances","distance":2,"distance2":3,"face":"F2.C1""#, &RING);
}

#[test]
fn fillet_ring_is_order_independent() {
    order_independent(r#""type":"fillet","id":"F3","radius":2"#, &RING);
}

#[test]
fn chamfer_corner_is_order_independent() {
    order_independent(r#""type":"chamfer","id":"F3","chamferType":"equal","distance":2"#, &CORNER);
}

#[test]
fn fillet_corner_is_order_independent() {
    order_independent(r#""type":"fillet","id":"F3","radius":2"#, &CORNER);
}

#[test]
fn fillet_propagate_is_order_independent() {
    order_independent(r#""type":"fillet","id":"F3","radius":2,"propagate":true"#, &RING[..2]);
}
