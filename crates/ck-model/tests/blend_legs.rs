//! Chamfers with two distances / distance and angle (`distances` / `distanceDistance`,
//! `angle` / `angleDistance`), the `face` reference and `flip`, and variable-radius fillets that meet at
//! a corner. Volumes are analytic or the OCCT reference of tools/blend_reference.json.
//! Block: x 0..40, y 0..10, z -20..0 (sketch on `top`: x, -z; extrude +y).
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

fn run(feature: &str) -> (Vec<String>, Option<(f64, usize, Vec<[f64; 3]>)>) {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{BOX},{feature}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = res.errors.iter().map(|e| e.error.code.clone()).collect();
    let b = ex.0.body.as_ref().unwrap();
    let issues = ck_topo::check::check(b, 1e-6).len();
    let verts: Vec<[f64; 3]> = b.vertex_ids().map(|v| [b.vertex(v).p.x, b.vertex(v).p.y, b.vertex(v).p.z]).collect();
    (errs, Some((ck_topo::mass::mass_props(b).volume, issues, verts)))
}

fn has_vertex(vs: &[[f64; 3]], p: [f64; 3]) -> bool {
    vs.iter().any(|v| (0..3).all(|i| (v[i] - p[i]).abs() < 1e-6))
}

const TOP_FRONT: &str = r#"{"faces":["F2.C1","F2.S.e1"],"at":[20,10,0]}"#;
const FRONT_TOP: &str = r#"{"faces":["F2.S.e1","F2.C1"],"at":[20,10,0]}"#;

#[test]
fn two_distances_follow_the_named_face() {
    // top-front edge (y=10, z=0): d1 = 2 on top (z = -2), d2 = 3 on the front face (y = 7)
    for (ty, name) in [("distances", "a"), ("distanceDistance", "b")] {
        let f = format!(r#"{{"type":"chamfer","id":"F3","chamferType":"{ty}","distance":2,"distance2":3,"face":"F2.C1","edges":[{TOP_FRONT}]}}"#);
        let (errs, r) = run(&f);
        let (vol, issues, vs) = r.unwrap();
        assert!(errs.is_empty(), "{name}: {errs:?}");
        assert_eq!(issues, 0);
        assert!((vol - 7880.0).abs() < 1e-6, "{vol}");
        assert!(has_vertex(&vs, [0.0, 10.0, -2.0]) && has_vertex(&vs, [0.0, 7.0, 0.0]), "{vs:?}");
    }
    // the same edge named from the other side, face = front: d1 = 2 on the front face (y = 8), d2 = 3 on top (z = -3)
    let f = format!(r#"{{"type":"chamfer","id":"F3","chamferType":"distanceDistance","distance":2,"distance2":3,"face":"F2.S.e1","edges":[{TOP_FRONT}]}}"#);
    let (errs, r) = run(&f);
    let (vol, _, vs) = r.unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    assert!((vol - 7880.0).abs() < 1e-6);
    assert!(has_vertex(&vs, [0.0, 8.0, 0.0]) && has_vertex(&vs, [0.0, 10.0, -3.0]), "{vs:?}");
    // without `face` the first face of the pick gets `distance`, in either listing order
    for (pick, top_first) in [(TOP_FRONT, true), (FRONT_TOP, false)] {
        let f = format!(r#"{{"type":"chamfer","id":"F3","chamferType":"distanceDistance","distance":2,"distance2":3,"edges":[{pick}]}}"#);
        let (_, r) = run(&f);
        let (_, _, vs) = r.unwrap();
        let want = if top_first { [0.0, 10.0, -2.0] } else { [0.0, 10.0, -3.0] };
        assert!(has_vertex(&vs, want), "{pick}: {vs:?}");
    }
    // flip swaps the legs
    let f = format!(r#"{{"type":"chamfer","id":"F3","chamferType":"distanceDistance","distance":2,"distance2":3,"face":"F2.C1","flip":true,"edges":[{TOP_FRONT}]}}"#);
    let (_, r) = run(&f);
    assert!(has_vertex(&r.unwrap().2, [0.0, 10.0, -3.0]));
    // a face that does not border the pick is refused
    let f = format!(r#"{{"type":"chamfer","id":"F3","chamferType":"distanceDistance","distance":2,"distance2":3,"face":"F2.C0","edges":[{TOP_FRONT}]}}"#);
    assert_eq!(run(&f).0, vec!["missingEdge".to_string()]);
}

#[test]
fn distance_and_angle() {
    // d = 2 on top, angle 30 degrees to the top face: the other leg is 2 tan 30 on the front face
    let f = format!(r#"{{"type":"chamfer","id":"F3","chamferType":"angleDistance","distance":2,"angle":30,"face":"F2.C1","edges":[{TOP_FRONT}]}}"#);
    let (errs, r) = run(&f);
    let (vol, issues, vs) = r.unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0);
    let leg = 2.0 * 30f64.to_radians().tan();
    assert!((vol - (8000.0 - 0.5 * 2.0 * leg * 40.0)).abs() < 1e-6, "{vol}");
    assert!((vol - 7953.81197846483).abs() < 1e-6, "{vol}");
    assert!(has_vertex(&vs, [0.0, 10.0, -2.0]) && has_vertex(&vs, [0.0, 10.0 - leg, 0.0]), "{vs:?}");
}

#[test]
fn ring_of_asymmetric_chamfers() {
    // the four top edges, d1 = 2 on the top face, d2 = 3 on the sides: OCCT 7656
    let edges = r#"{"faces":["F2.C1","F2.S.e1"],"at":[20,10,0]},{"faces":["F2.C1","F2.S.e2"],"at":[40,10,-10]},{"faces":["F2.C1","F2.S.e3"],"at":[20,10,-20]},{"faces":["F2.C1","F2.S.e4"],"at":[0,10,-10]}"#;
    let f = format!(r#"{{"type":"chamfer","id":"F3","chamferType":"distanceDistance","distance":2,"distance2":3,"face":"F2.C1","edges":[{edges}]}}"#);
    let (errs, r) = run(&f);
    let (vol, issues, _) = r.unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0);
    assert!((vol - 7656.0).abs() < 1e-6, "{vol}");
}

#[test]
fn variable_radius_at_a_corner() {
    // two edges meeting at (40, 10, 0), radius 3 there and 1 at the far ends
    let two = r#"{"faces":["F2.C1","F2.S.e1"],"at":[38,10,0]},{"faces":["F2.C1","F2.S.e2"],"at":[40,10,-2]}"#;
    let f = format!(r#"{{"type":"fillet","id":"F3","filletType":"variable","radii":[3,1],"edges":[{two}]}}"#);
    let (errs, r) = run(&f);
    let (vol, issues, _) = r.unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0);
    assert!((vol - 7944.878041500547).abs() < 7944.88 * 2e-4, "{vol}");
    // conflicting radii at the shared vertex are refused by name
    let bad = r#"{"faces":["F2.C1","F2.S.e1"],"at":[38,10,0]},{"faces":["F2.C1","F2.S.e2"],"at":[40,10,-18]}"#;
    let f = format!(r#"{{"type":"fillet","id":"F3","filletType":"variable","radii":[3,1],"edges":[{bad}]}}"#);
    assert_eq!(run(&f).0, vec!["variableRadiusMismatch".to_string()]);
    // a per-pick pair makes the same two picks agree: 3 at the corner for the first, 1 -> 3 for the second
    let f = format!(r#"{{"type":"fillet","id":"F3","filletType":"variable","radii":[3,1],"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[38,10,0]}},{{"faces":["F2.C1","F2.S.e2"],"at":[40,10,-18],"radii":[1,3]}}]}}"#);
    let (errs, r) = run(&f);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(r.unwrap().1, 0);
}
