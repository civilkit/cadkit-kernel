//! Mirror and pattern of a join (add) feature on a hollow part: the volume must be the seed plus one gain per copy,
//! also when the tool straddles a wall. A copy that loses material silently is the failure under test.

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

fn run(features: &str) -> (f64, Vec<String>, bool) {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let mut errors: Vec<String> = result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect();
    errors.extend(result.warnings.iter().map(|w| format!("warning {}:{}", w.feature, w.error.code)));
    let b = ex.brep.body.as_ref().unwrap();
    let valid = ck_check::validate(b, 1e-6).is_empty() && ck_topo::region::components(b).len() == 1;
    (ck_topo::mass::mass_props(b).volume, errors, valid)
}

fn rect(id: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],
  "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

/// 40 x 40 x 20 block, shelled to 2 mm walls with the top open: 32000 - 36 * 36 * 18 = 8672 mm3.
fn hollow() -> String {
    format!(
        r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}},{{"type":"shell","id":"F3","faces":["F2.C1"],"thickness":2}}"#,
        rect("F1", 0.0, 0.0, 40.0, 40.0)
    )
}
const HOLLOW: f64 = 8672.0;

/// A 10 mm tall pillar standing on the base plane (inside the floor and the cavity), sketch x from `x0` to `x1`, y 10..14.
fn pillar(x0: f64, x1: f64) -> String {
    format!(r#"{},{{"type":"extrude","id":"F5","op":"add","sketch":"F4","end":"blind","depth":10}}"#, rect("F4", x0, 10.0, x1, 14.0))
}

fn check(features: &str, expect: f64) {
    let (v, e, valid) = run(features);
    assert!(e.is_empty(), "{e:?}");
    assert!(valid, "invalid or split body");
    assert!((v - expect).abs() < 1e-3, "volume {v}, expected {expect}");
}

#[test]
fn hollow_part_volume_is_known() {
    check(&hollow(), HOLLOW);
}

#[test]
fn join_in_the_cavity_mirrors_and_patterns_without_loss() {
    // pillar x 6..10: 4 x 4 x 8 above the 2 mm floor
    let gain = 4.0 * 4.0 * 8.0;
    let seed = format!("{},{}", hollow(), pillar(6.0, 10.0));
    check(&seed, HOLLOW + gain);
    let plane = r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"right"},"offset":20}"#;
    check(&format!(r#"{seed},{plane},{{"type":"mirror","id":"F7","seeds":["F5"],"plane":{{"kind":"feature","id":"P1"}}}}"#), HOLLOW + 2.0 * gain);
    check(
        &format!(r#"{seed},{{"type":"lpattern","id":"F7","seeds":["F5"],"dir":{{"kind":"axis","axis":"x"}},"spacing":10,"count":3}}"#),
        HOLLOW + 3.0 * gain,
    );
}

#[test]
fn join_straddling_the_wall_mirrors_and_patterns_without_loss() {
    // pillar x 0..6 overlaps the 2 mm side wall (x 0..2): only the part in the cavity adds material, 4 x 4 x 8 for x 2..6
    let gain = 4.0 * 4.0 * 8.0;
    let seed = format!("{},{}", hollow(), pillar(0.0, 6.0));
    check(&seed, HOLLOW + gain);
    let plane = r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"right"},"offset":20}"#;
    check(&format!(r#"{seed},{plane},{{"type":"mirror","id":"F7","seeds":["F5"],"plane":{{"kind":"feature","id":"P1"}}}}"#), HOLLOW + 2.0 * gain);
    check(
        &format!(r#"{seed},{{"type":"lpattern","id":"F7","seeds":["F5"],"dir":{{"kind":"axis","axis":"x"}},"spacing":14,"count":3}}"#),
        // the copies at x 14..20 and 28..34 are clear of the walls: 6 x 4 x 8 each
        HOLLOW + gain + 2.0 * 6.0 * 4.0 * 8.0,
    );
}
