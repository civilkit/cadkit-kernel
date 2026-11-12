//! Two cylinders that touch along a line (centres exactly two radii apart) and both pierce the wall of a tube, then
//! a third tool joins the result. The loops the two cylinders cut in the wall touch at one point of the contact line; the
//! arrangement cut one of them a hair beyond that point, welded the resulting sliver edge into the loop through the same two
//! vertices, and the next union saw an edge with four faces (`booleanFailed`, `nonManifoldEdge`).

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn volume(&self) -> f64 {
        self.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume)
    }
    fn lumps(&self) -> usize {
        self.body.as_ref().map_or(0, |b| ck_topo::region::components(b).len())
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone() }
}


fn circle(id: &str, plane: &str, cx: f64, cy: f64, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"{plane}"}},"points":[{{"id":"p1","x":{cx},"y":{cy}}}],"entities":[{{"id":"e1","type":"circle","c":"p1","r":{r}}}],"constraints":[]}}"#)
}

/// A tube (axis z through (0,-7), radii 15.5 and 17.5, z 17..35) united with three wires of radius 0.65 along y at
/// z = 23.5 + `dz`: x = -0.65 and x = 0.65 (touching along a line) and a third one at x = 8, z = 26.
fn tube_and_wires(dz: f64) -> String {
    let mut f = vec![
        r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":-7}],"entities":[{"id":"e1","type":"circle","c":"p1","r":17.5},{"id":"e2","type":"circle","c":"p1","r":15.5}],"constraints":[]}"#.to_string(),
        r#"{"type":"extrude","id":"F1","op":"newBody","body":"tube","sketch":"S1","end":"blind","depth":18}"#.to_string(),
        r#"{"type":"moveBody","id":"M1","bodies":[{"body":"tube"}],"translate":[0,0,17]}"#.to_string(),
    ];
    let wires = [("w1", -0.65, 23.5 + dz), ("w2", 0.65, 23.5 + dz), ("w3", 8.0, 26.0)];
    for (i, (b, x, z)) in wires.iter().enumerate() {
        f.push(circle(&format!("SW{i}"), "front", 0.0, 0.0, 0.65));
        f.push(format!(r#"{{"type":"extrude","id":"FW{i}","op":"newBody","body":"{b}","sketch":"SW{i}","end":"blind","depth":35.5}}"#));
        f.push(format!(r#"{{"type":"moveBody","id":"MR{i}","bodies":[{{"body":"{b}"}}],"rotate":{{"axis":"x","origin":[0,0,0],"angle":90.0}}}}"#));
        f.push(format!(r#"{{"type":"moveBody","id":"MT{i}","bodies":[{{"body":"{b}"}}],"translate":[{x},0,{z}]}}"#));
    }
    f.push(r#"{"type":"combine","id":"C","op":"union","target":{"body":"tube"},"tools":[{"body":"w1"},{"body":"w2"},{"body":"w3"}],"body":"tube"}"#.to_string());
    f.join(",")
}

#[test]
fn line_tangent_wires_through_a_tube_wall_then_a_third_tool() {
    let mut volumes = vec![];
    // exactly tangent, and a hair either way (the noise of a transform decides where the loops are cut)
    for dz in [0.0, 1e-9, -1e-9, 1e-7, -1e-7] {
        let r = run(&tube_and_wires(dz));
        assert!(r.errors().is_empty(), "dz {dz}: {:?}", r.errors());
        assert_eq!(r.lumps(), 1, "dz {dz}");
        let b = r.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "dz {dz}");
        volumes.push(r.volume());
    }
    let tube = std::f64::consts::PI * (17.5f64 * 17.5 - 15.5 * 15.5) * 18.0;
    assert!(volumes[0] > tube && volumes[0] < tube + 3.0 * std::f64::consts::PI * 0.65 * 0.65 * 35.5, "{volumes:?}");
    for v in &volumes {
        assert!((v - volumes[0]).abs() < 1e-3, "{volumes:?}");
    }
}
