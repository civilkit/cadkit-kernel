//! Damage valid part bodies (regenerated through ck-model) and heal them
//! back: validate-clean and volume within 1e-6.

use ck_heal::{damage, heal};
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use ck_topo::mass::mass_props;

struct Exec(Brep);
impl FeatureExec for Exec {
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

const RECT: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

fn parts() -> Vec<(&'static str, String)> {
    let circ = |plane: &str, x: i32, y: i32, r: i32| {
        format!(r#"{{"type":"sketch","id":"F3","plane":{plane},"points":[{{"id":"c","x":{x},"y":{y}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#)
    };
    let ex = |op: &str, end: &str| format!(r#"{{"type":"extrude","id":"F4","op":"{op}","sketch":"F3","end":"{end}","depth":4}}"#);
    let top = r#"{"kind":"std","id":"top"}"#;
    let cap = r#"{"kind":"face","id":"F2.C1"}"#;
    let wrap = |fs: String| format!(r#"{{"format":"ck-part","version":1,"features":[{fs}]}}"#);
    vec![
        ("box", wrap(RECT.to_string())),
        ("cutHole", wrap(format!("{RECT},{},{}", circ(top, 5, 5, 2), ex("cut", "throughBoth")))),
        ("pocket", wrap(format!("{RECT},{},{}", circ(cap, 10, 10, 3), ex("cut", "blind")))),
        ("boss", wrap(format!("{RECT},{},{}", circ(cap, 10, 10, 3), ex("add", "blind")))),
        (
            "cyl",
            wrap(format!(
                r#"{{"type":"sketch","id":"F1","plane":{top},"points":[{{"id":"c","x":5,"y":3}}],"entities":[{{"type":"circle","id":"e7","c":"c","r":10}}],"constraints":[]}},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}"#
            )),
        ),
    ]
}

fn body_of(src: &str) -> Body {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| e.error.code.clone()).collect::<Vec<_>>());
    ex.0.body.unwrap()
}

#[test]
fn damaged_parts_heal_back_to_valid_with_same_volume() {
    let mut cases = 0;
    for (name, src) in parts() {
        let good = body_of(&src);
        assert!(ck_check::validate(&good, 1e-6).is_empty(), "{name} is not valid to begin with");
        let v0 = mass_props(&good).volume;
        for kind in damage::KINDS {
            let mut b = good.clone();
            if !damage::apply(&mut b, kind, 2e-6) {
                continue;
            }
            let r = heal(&mut b, 1e-5);
            assert!(r.remaining.is_empty(), "{name}/{kind}: {:?} after {:?}", r.remaining, r.fixed);
            assert!(ck_check::validate(&b, 1e-6).is_empty(), "{name}/{kind}");
            let v = mass_props(&b).volume;
            assert!((v - v0).abs() <= 1e-6 * v0.abs(), "{name}/{kind}: volume {v} vs {v0}");
            assert!(!r.fixed.is_empty(), "{name}/{kind}: damage went unreported");
            cases += 1;
        }
    }
    assert!(cases >= 25, "only {cases} cases exercised");
}
