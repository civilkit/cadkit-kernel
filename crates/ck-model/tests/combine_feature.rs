//! `combine` (body-level boolean between lumps), the fuzzy `tolerance` / `autoFuzzy` fields and the `booleanFuzzy`
//! warning (docs/spec-agent-interface.md, "Fuzzy tolerance and body-level booleans").

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn warnings(&self) -> Vec<String> {
        self.result.warnings.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn volume(&self) -> f64 {
        self.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume)
    }
    fn lumps(&self) -> usize {
        self.body.as_ref().map_or(0, |b| ck_topo::region::components(b).len())
    }
    fn faces(&self) -> Vec<String> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        v.sort();
        v
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone() }
}

/// A box [x0,x1] x [0,10] on the top plane, extruded 10, as sketch `S<id>` + extrude `F<id>`.
fn boxf(id: u32, x0: f64, x1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":0}},{{"id":"p2","x":{x1},"y":0}},{{"id":"p3","x":{x1},"y":10}},{{"id":"p4","x":{x0},"y":10}}],
          "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
         {{"type":"extrude","id":"F{id}","op":"add","sketch":"S{id}","end":"blind","depth":10}}"#
    )
}

/// Box A [0,10]^3 and box B (built at x 30..40, moved by `dx`): two lumps, overlapping when dx = -25.
fn two(dx: f64) -> String {
    format!(
        r#"{},{},{{"type":"moveBody","id":"M","bodies":[{{"face":"F0.C1"}}],"translate":[{dx},0,0]}}"#,
        boxf(1, 30.0, 40.0).replace("S1", "S0").replace("F1", "F0"),
        boxf(2, 0.0, 10.0)
    )
}

fn comb(op: &str, extra: &str) -> String {
    format!(r#"{{"type":"combine","id":"C","op":"{op}","target":{{"face":"F2.C1"}},"tools":[{{"face":"F0.C1"}}]{extra}}}"#)
}

#[test]
fn the_fixture_has_two_overlapping_lumps() {
    let r = run(&two(-25.0));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.lumps(), 2);
    assert!((r.volume() - 2000.0).abs() < 1e-6);
}

#[test]
fn union_cut_intersect_split() {
    let cases = [("union", 1500.0, 1), ("cut", 500.0, 1), ("intersect", 500.0, 1), ("split", 1000.0, 2)];
    for (op, vol, lumps) in cases {
        let r = run(&format!("{},{}", two(-25.0), comb(op, "")));
        assert!(r.errors().is_empty(), "{op}: {:?}", r.errors());
        assert!((r.volume() - vol).abs() < 1e-6, "{op}: volume {}", r.volume());
        assert_eq!(r.lumps(), lumps, "{op}");
        let b = r.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "{op}");
    }
}

#[test]
fn keep_tools_leaves_the_tool_lump() {
    let r = run(&format!("{},{}", two(-25.0), comb("cut", r#","keepTools":true"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!((r.volume() - 1500.0).abs() < 1e-6, "{}", r.volume());
    assert_eq!(r.lumps(), 2);
}

#[test]
fn result_faces_keep_their_names() {
    let r = run(&format!("{},{}", two(-25.0), comb("cut", "")));
    let names = r.faces();
    // the cut face of the tool is flipped but keeps the tool's feature id; the target's faces keep theirs
    assert!(names.iter().any(|n| n.starts_with("F2.")), "{names:?}");
    assert!(names.iter().any(|n| n.starts_with("F0.")), "{names:?}");
}

#[test]
fn errors_are_structured_and_leave_the_body_alone() {
    let disjoint = run(&format!("{},{}", two(0.0), comb("intersect", "")));
    assert_eq!(disjoint.errors(), vec!["C:combineEmpty"]);
    assert!((disjoint.volume() - 2000.0).abs() < 1e-6);
    let missing = run(&format!("{},{}", two(-25.0), r#"{"type":"combine","id":"C","op":"cut","target":{"face":"F9.C1"},"tools":[{"face":"F0.C1"}]}"#));
    assert_eq!(missing.errors(), vec!["C:missingReference"]);
    let same = run(&format!("{},{}", two(-25.0), r#"{"type":"combine","id":"C","op":"cut","target":{"face":"F2.C1"},"tools":[{"face":"F2.C0"}]}"#));
    assert_eq!(same.errors(), vec!["C:invalidValue"]);
    let bad_op = run(&format!("{},{}", two(-25.0), comb("explode", "")));
    assert_eq!(bad_op.errors(), vec!["C:invalidValue"]);
    let nothing = run(&comb("union", ""));
    assert_eq!(nothing.errors(), vec!["C:nothingToCut"]);
}

#[test]
fn lump_index_selects_too() {
    let r = run(&format!(
        "{},{}",
        two(-25.0),
        r#"{"type":"combine","id":"C","op":"union","target":{"lump":0},"tools":[{"lump":1}]}"#
    ));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!((r.volume() - 1500.0).abs() < 1e-6);
}

#[test]
fn n_ary_union_in_listed_order() {
    let src = format!(
        r#"{},{},{},{{"type":"moveBody","id":"M1","bodies":[{{"face":"F0.C1"}}],"translate":[-22,0,0]}},{{"type":"moveBody","id":"M2","bodies":[{{"face":"F4.C1"}}],"translate":[-44,0,0]}},{{"type":"combine","id":"C","op":"union","target":{{"face":"F2.C1"}},"tools":[{{"face":"F0.C1"}},{{"face":"F4.C1"}}]}}"#,
        boxf(1, 30.0, 40.0).replace("S1", "S0").replace("F1", "F0"),
        boxf(2, 0.0, 10.0),
        boxf(4, 60.0, 70.0)
    );
    let r = run(&src);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!((r.volume() - 2600.0).abs() < 1e-6, "{}", r.volume());
    assert_eq!(r.lumps(), 1);
}

/// Cylinder radius `r` along the top-plane normal from y0 to y0 + 10, as extrude `F<id>`.
fn cylf(id: u32, r: f64, y0: f64, extra: &str) -> String {
    let plane = if y0 == 0.0 {
        r#"{"kind":"std","id":"top"}"#.to_string()
    } else {
        format!(r#"{{"kind":"feature","id":"P{id}"}}"#)
    };
    let pl = if y0 == 0.0 { String::new() } else { format!(r#"{{"type":"plane","id":"P{id}","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":{y0}}},"#) };
    format!(
        r#"{pl}{{"type":"sketch","id":"S{id}","plane":{plane},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}},
         {{"type":"extrude","id":"F{id}","op":"add","sketch":"S{id}","end":"blind","depth":10{extra}}}"#
    )
}

#[test]
fn tolerance_snaps_a_wall_1e3_off_and_warns() {
    // r5 cylinder, then an r5.001 cylinder stacked on it: the rim circles are 1e-3 apart
    let exact = run(&format!("{},{}", cylf(1, 5.0, 0.0, ""), cylf(2, 5.001, 10.0, "")));
    assert!(exact.errors().iter().any(|e| e.ends_with("booleanFailed")), "{:?}", exact.errors());
    let fuzzy = run(&format!("{},{}", cylf(1, 5.0, 0.0, ""), cylf(2, 5.001, 10.0, r#","tolerance":0.002"#)));
    assert!(fuzzy.errors().is_empty(), "{:?}", fuzzy.errors());
    assert!((fuzzy.volume() - PI * 25.0 * 20.0).abs() < 0.01 * 20.0, "{}", fuzzy.volume());
    assert_eq!(fuzzy.warnings(), vec!["F2:booleanFuzzy"]);
    let auto = run(&format!("{},{}", cylf(1, 5.0, 0.0, ""), cylf(2, 5.001, 10.0, r#","autoFuzzy":true"#)));
    assert!(auto.errors().is_empty(), "{:?}", auto.errors());
    assert_eq!(auto.warnings(), vec!["F2:booleanFuzzy"]);
}

#[test]
fn no_tolerance_no_warning() {
    let r = run(&format!("{},{}", cylf(1, 5.0, 0.0, ""), cylf(2, 5.0, 10.0, r#","autoFuzzy":true,"tolerance":1e-4"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.warnings().is_empty(), "{:?}", r.warnings());
}
