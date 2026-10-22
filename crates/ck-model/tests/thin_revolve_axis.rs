//! Thin-wall revolve on-axis segments (`thin.rs`): a profile edge that lies on the revolve axis
//! is ignored, so the catalog's arc-plus-diameter sphere profile hollows to the ideal shell
//! instead of leaving a solid rod (radius = the wall thickness) down the middle.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

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

struct Out {
    brep: Brep,
    errors: Vec<regen::RegenError>,
}

fn run(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-9)
}

fn check_valid(o: &Out, what: &str) {
    let b = o.brep.body.as_ref().unwrap();
    let iss = ck_check::validate(b, 1e-3);
    assert!(iss.is_empty(), "{what}: {:?}", iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
}

fn names(o: &Out) -> Vec<String> {
    let b = o.brep.body.as_ref().unwrap();
    b.face_ids().map(|f| b.face_id_str(f)).collect()
}

fn part(sk: &str, rest: &str) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{sk},{rest}]}}"#)
}

/// Upper half circle of radius `r` about the origin, counter-clockwise from (r,0) to (-r,0),
/// plus the diameter line on the x axis — the catalog's solid-sphere profile; the line doubles
/// as the revolve axis entity.
fn sphere_profile(r: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":0,"y":0}},{{"id":"p1","x":{r},"y":0}},{{"id":"p2","x":-{r},"y":0}}],
        "entities":[{{"type":"arc","id":"e1","c":"c","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p1","p2":"p2"}}],"constraints":[]}}"#
    )
}

fn rev(thin: &str) -> String {
    format!(r#"{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"e2","angle":360,"thin":{thin}}}"#)
}

#[test]
fn sphere_arc_plus_diameter_hollows_without_the_rod() {
    // R=37 wall 2 inside: ideal hollow 4/3 pi (R^3 - (R-t)^3); with the diameter offset as well
    // the same profile gave 33459.34 (the extra 879 is the rod)
    const R: f64 = 37.0;
    const T: f64 = 2.0;
    let o = run(&part(&sphere_profile(R), &rev(r#"{"thickness":2,"side":"inside","ends":"flat","cornerStyle":"mitre"}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let ideal = 4.0 / 3.0 * PI * (R.powi(3) - (R - T).powi(3));
    assert!(rel(vol(&o), ideal) < 1e-6, "{} vs {ideal}", vol(&o));
    check_valid(&o, "sphere inside");
}

#[test]
fn sphere_outside_is_a_shell() {
    const R: f64 = 37.0;
    const T: f64 = 2.0;
    let o = run(&part(&sphere_profile(R), &rev(r#"{"thickness":2,"side":"outside","ends":"flat","cornerStyle":"mitre"}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let ideal = 4.0 / 3.0 * PI * ((R + T).powi(3) - R.powi(3));
    assert!(rel(vol(&o), ideal) < 1e-6, "{} vs {ideal}", vol(&o));
    check_valid(&o, "sphere outside");
}

#[test]
fn a_rectangle_edge_on_the_axis_is_ignored() {
    // closed 12 x 8 rectangle with the left edge as the axis entity vs the open three-edge chain
    // plus a construction axis: the on-axis edge is dropped, the two give the same wall
    let pts = r#"[{"id":"a","x":0,"y":0},{"id":"b","x":12,"y":0},{"id":"c","x":12,"y":8},{"id":"d","x":0,"y":8}]"#;
    let closed = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":{pts},
        "entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},
        {{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}}"#
    );
    let open = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":{pts},
        "entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},
        {{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"ax","p1":"d","p2":"a","construction":true}}],"constraints":[]}}"#
    );
    let thin = r#"{"thickness":2,"side":"inside"}"#;
    let a = run(&part(&closed, &format!(r#"{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"e4","angle":360,"thin":{thin}}}"#)));
    let b = run(&part(&open, &format!(r#"{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360,"thin":{thin}}}"#)));
    assert!(a.errors.is_empty(), "{:?}", a.errors);
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert!(rel(vol(&a), vol(&b)) < 1e-9, "{} vs {}", vol(&a), vol(&b));
    check_valid(&a, "rect on axis");
    check_valid(&b, "rect open chain");
    // and it is a real wall, not nothing
    assert!(vol(&a) > 0.0);
}

#[test]
fn profiles_clear_of_the_axis_are_unchanged() {
    // the standing tube (line at x=10, axis construction at x=0): unchanged volume and face names
    let sk = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":10,"y":0},{"id":"b","x":10,"y":20},{"id":"o","x":0,"y":0},{"id":"q","x":0,"y":20}],
      "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"ax","p1":"o","p2":"q","construction":true}],"constraints":[]}"#;
    let o = run(&part(sk, &rev_tube()));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(rel(vol(&o), PI * (121.0 - 81.0) * 20.0) < 1e-6, "{}", vol(&o));
    let n = names(&o);
    let mut sorted = n.clone();
    sorted.sort();
    assert_eq!(sorted, ["F2.S.e1.E0", "F2.S.e1.E1", "F2.S.e1.W0", "F2.S.e1.W1"], "{n:?}");
    // extrudes are not affected: the rectangle with the left edge still gives the closed-loop
    // hollow wall (outer 12x8 minus inner 8x4), not the U-chain wall
    let rect = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":12,"y":0},{"id":"c","x":12,"y":8},{"id":"d","x":0,"y":8}],
      "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]}"#;
    let o = run(&part(rect, r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":5,"thin":{"thickness":2,"side":"inside"}}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(rel(vol(&o), (12.0 * 8.0 - 8.0 * 4.0) * 5.0) < 1e-6, "{}", vol(&o));
    check_valid(&o, "extrude hollow rect");
}

fn rev_tube() -> String {
    r#"{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360,"thin":{"thickness":2,"side":"mid"}}"#.to_string()
}
