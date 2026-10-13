//! `lip` along a B-spline rim edge (large sample F46): the tool's path-side face
//! is the exact extrusion of the rim curve (not a chord polygon, which leaves
//! slivers against the host and fails the boolean), and the edge pick falls
//! back to the face family nearest the recorded point when the `~k` numbering
//! differs from the recording's.

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

fn part(pick: &str, lip: bool) -> String {
    let lip_feature = if lip {
        format!(
            r#",{{"type":"lip","id":"F3","kind":"lip","faces":["F2.C1"],"edges":[{{"faces":["F2.C1","{pick}"],"at":[46,10,10]}}],"width":1,"height":1}}"#
        )
    } else {
        String::new()
    };
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
 {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},
  "points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":40,"y":0}},{{"id":"c","x":46,"y":10}},{{"id":"p3","x":40,"y":20}},{{"id":"p4","x":0,"y":20}}],
  "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"spline","id":"e2","pts":["p2","c","p3"]}},
              {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}{lip_feature}
]}}"#
    )
}

fn run(src: &str) -> (Brep, Vec<regen::RegenError>) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (ex.brep, r.errors)
}

#[test]
fn lip_follows_a_spline_rim_edge() {
    let (base, e0) = run(&part("", false));
    assert!(e0.is_empty(), "{e0:?}");
    let v0 = ck_topo::mass::mass_props(base.body.as_ref().unwrap()).volume;
    // exact name, then a wrong `~k` the family fallback resolves by the pick point
    for pick in ["F2.S.e2", "F2.S.e2~7"] {
        let (br, errs) = run(&part(pick, true));
        assert!(errs.is_empty(), "{pick}: {errs:?}");
        let b = br.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "{pick}: {:?}", ck_check::validate(b, 1e-6));
        let v1 = ck_topo::mass::mass_props(b).volume;
        // band of width 1 and height 1 along the rim: its volume is the rim length (about 23)
        assert!(v1 > v0 + 15.0 && v1 < v0 + 30.0, "{pick}: volume {v0} -> {v1}");
        let exact = b
            .face_ids()
            .any(|f| b.face_id_str(f).starts_with("F3.S.F2.S.e2r") && b.face(f).surface.kind() == "extrusion");
        assert!(exact, "{pick}: path side must be one exact extrusion face");
    }
}
