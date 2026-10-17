//! Loft `centreLine` at the regen level (crates/ck-model/src/features/loft.rs, ck-ops `loft::centre`): the loft
//! follows a curved centre line; the marker keeps one loop / start per PROFILE (not per intermediate station);
//! a straight centre line is the plain loft; every refusal has a code with `detail` and `hint`.
//! (Engine level: STEP, validate with the mesh, cache on/off: crates/ck-wasm/tests/loft_centre.rs.)
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
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

struct Out {
    brep: Brep,
    errors: Vec<regen::RegenError>,
    markers: Vec<ck_json::Json>,
}

fn run(features: &str) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.brep, errors: r.errors, markers: r.markers }
}

fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}

fn square(id: &str, plane: &str, h: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{a},"y":{a}}},{{"id":"p2","x":{h},"y":{a}}},{{"id":"p3","x":{h},"y":{h}}},{{"id":"p4","x":{a},"y":{h}}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#,
        a = -h
    )
}

fn arc(id: &str, bow: f64) -> String {
    let r = (bow * bow + 225.0) / (2.0 * bow);
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"g0","x":{},"y":15}},{{"id":"g1","x":0,"y":0}},{{"id":"g2","x":0,"y":30}}],"entities":[{{"type":"arc","id":"e1","c":"g0","p1":"g1","p2":"g2"}}],"constraints":[]}}"#,
        bow - r
    )
}

const PLANE30: &str = r#"{"type":"plane","id":"P","mode":"offset","base":{"kind":"std","id":"top"},"offset":30}"#;

fn base() -> Vec<String> {
    vec![square("S1", r#"{"kind":"std","id":"top"}"#, 10.0), PLANE30.to_string(), square("S2", r#"{"kind":"feature","id":"P"}"#, 5.0)]
}

fn with(mut f: Vec<String>, cl: Option<String>, centre_json: &str) -> String {
    if let Some(c) = cl {
        f.push(c);
    }
    f.push(format!(r#"{{"type":"loft","id":"L","op":"add","sketches":["S1","S2"]{centre_json}}}"#));
    f.join(",")
}

#[test]
fn bent_loft_volume_and_markers() {
    let o = run(&with(base(), Some(arc("CL", 5.0)), r#","centreLine":{"sketch":"CL"}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    // oracle: 6006.0109
    assert!((vol(&o) - 6006.0109).abs() < 6.0, "{}", vol(&o));
    let m = o.markers.iter().find(|m| m.as_arr().and_then(|a| a.first()).and_then(|j| j.as_str()) == Some("L")).expect("loft marker");
    let mk = &m.as_arr().unwrap()[1];
    assert_eq!(mk.get_arr("loops").unwrap().len(), 2, "one loop per profile");
    assert_eq!(mk.get_arr("starts").unwrap().len(), 2, "one start per profile");
}

#[test]
fn straight_centre_line_is_the_plain_loft() {
    let plain = run(&with(base(), None, ""));
    let line = r#"{"type":"sketch","id":"CL","plane":{"kind":"std","id":"front"},"points":[{"id":"g1","x":0,"y":0},{"id":"g2","x":8,"y":30}],"entities":[{"type":"line","id":"e1","p1":"g1","p2":"g2"}],"constraints":[]}"#;
    let slanted = run(&with(base(), Some(line.to_string()), r#","centreLine":{"sketch":"CL"}"#));
    assert!(plain.errors.is_empty() && slanted.errors.is_empty());
    assert_eq!(vol(&plain), vol(&slanted));
    assert_eq!(plain.brep.face_count(), slanted.brep.face_count());
    assert_eq!(plain.markers, slanted.markers);
}

#[test]
fn refusals_carry_a_detail_and_a_hint() {
    // reach: the centre line stops at y = 15
    let short = r#"{"type":"sketch","id":"CL","plane":{"kind":"std","id":"front"},"points":[{"id":"g1","x":0,"y":0},{"id":"g2","x":0,"y":15}],"entities":[{"type":"line","id":"e1","p1":"g1","p2":"g2"}],"constraints":[]}"#;
    let o = run(&with(base(), Some(short.to_string()), r#","centreLine":{"sketch":"CL"}"#));
    assert_eq!(o.errors.len(), 1);
    let e = &o.errors[0].error;
    assert_eq!(e.code, "loftCentreLineReach");
    assert!(e.params.iter().any(|(k, v)| k == "detail" && v.as_str().is_some()) && e.params.iter().any(|(k, _)| k == "hint"), "{:?}", e.params);
    // a corner
    let corner = r#"{"type":"sketch","id":"CL","plane":{"kind":"std","id":"front"},"points":[{"id":"g1","x":0,"y":0},{"id":"g2","x":6,"y":15},{"id":"g3","x":0,"y":30}],"entities":[{"type":"line","id":"e1","p1":"g1","p2":"g2"},{"type":"line","id":"e2","p1":"g2","p2":"g3"}],"constraints":[]}"#;
    let o = run(&with(base(), Some(corner.to_string()), r#","centreLine":{"sketch":"CL"}"#));
    assert_eq!(o.errors[0].error.code, "loftCentreLineKink");
    // the centre line is not a sketch
    let o = run(&with(base(), None, r#","centreLine":{"edges":[]}"#));
    assert_eq!(o.errors[0].error.code, "loftCentreLineInvalid");
    assert!(o.errors[0].error.params.iter().any(|(k, _)| k == "hint"));
    // a null centre line is no centre line
    let o = run(&with(base(), None, r#","centreLine":null"#));
    assert!(o.errors.is_empty() && (vol(&o) - 7000.0).abs() < 1e-9);
}
