//! `target` on a cut (or add) feature: the bodies the feature acts on, by name, so a cutter does not need a body of its own and a
//! `combine`. One selection, or a list; extrude, revolve, loft, sweep and hole take it. Volumes are analytic: boxes with integer overlaps.

#![allow(dead_code)]

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
    fn error_params(&self, code: &str) -> ck_json::Json {
        let e = self.result.errors.iter().find(|e| e.error.code == code).unwrap_or_else(|| panic!("no {code}: {:?}", self.errors()));
        ck_json::Json::Obj(e.error.params.clone())
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
    /// Body name of every live lump, in lump order.
    fn names(&self) -> Vec<Option<String>> {
        let b = self.body.as_ref().unwrap();
        b.lump_ids().map(|l| b.lump(l).name.clone()).collect()
    }
    /// The `massProps {bodies:true}` entries.
    fn mass_bodies(&self) -> Vec<ck_json::Json> {
        let j = ck_topo::mass::mass_props_json(self.body.as_ref().unwrap(), true, None);
        j.get("bodies").and_then(|b| b.as_arr()).unwrap().to_vec()
    }
    /// Volume of the lump with this name.
    fn volume_of(&self, name: &str) -> f64 {
        self.mass_bodies().iter().find(|b| b.get_str("name") == Some(name)).and_then(|b| b.get("volume")).and_then(|v| v.as_f64()).unwrap_or_else(|| panic!("no lump named {name}: {:?}", self.names()))
    }
    fn clean(&self) {
        assert!(self.errors().is_empty(), "{:?}", self.errors());
        let b = self.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "invalid result");
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone() }
}

fn rect_sketch(id: &str, x0: f64, x1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":0}},{{"id":"p2","x":{x1},"y":0}},{{"id":"p3","x":{x1},"y":10}},{{"id":"p4","x":{x0},"y":10}}],
          "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

/// Box [x0,x1] x [0,10] on the top plane, 10 high, as a body of its own with this name.
fn named_box(id: u32, x0: f64, x1: f64, name: &str) -> String {
    format!(
        r#"{},{{"type":"extrude","id":"F{id}","op":"newBody","sketch":"S{id}","end":"blind","depth":10,"body":"{name}"}}"#,
        rect_sketch(&format!("S{id}"), x0, x1)
    )
}

/// A: [0,10], B: [5,15], C: [20,30] (1000 each), then a cutter box [8,22] x [0,10] x 10 as an extrude cut with `extra`.
/// (a cut runs against the sketch normal, the boxes stand along it: flip). The cutter takes 200 from A, 700 from B, 200 from C.
fn abc_cut(extra: &str) -> Run {
    let f = format!(
        r#"{},{},{},{},{{"type":"extrude","id":"F9","op":"cut","sketch":"S9","end":"blind","depth":10,"flip":true{extra}}}"#,
        named_box(1, 0.0, 10.0, "A"),
        named_box(2, 5.0, 15.0, "B"),
        named_box(3, 20.0, 30.0, "C"),
        rect_sketch("S9", 8.0, 22.0)
    );
    run(&f)
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-3
}

#[test]
fn without_a_target_the_cut_reaches_every_body() {
    let r = abc_cut("");
    r.clean();
    assert!(near(r.volume_of("A"), 800.0) && near(r.volume_of("B"), 300.0) && near(r.volume_of("C"), 800.0));
}

#[test]
fn one_named_body_is_cut_and_the_others_stay() {
    let r = abc_cut(r#","target":{"body":"B"}"#);
    r.clean();
    assert!(near(r.volume_of("A"), 1000.0) && near(r.volume_of("B"), 300.0) && near(r.volume_of("C"), 1000.0), "{:?}", r.mass_bodies());
    assert_eq!(r.names().iter().filter(|n| n.is_some()).count(), 3, "names stay: {:?}", r.names());
}

#[test]
fn a_list_of_bodies_cuts_each_of_them() {
    let r = abc_cut(r#","target":[{"body":"A"},{"body":"C"}]"#);
    r.clean();
    assert!(near(r.volume_of("A"), 800.0) && near(r.volume_of("B"), 1000.0) && near(r.volume_of("C"), 800.0), "{:?}", r.mass_bodies());
}

#[test]
fn a_body_the_tool_misses_is_skipped_when_another_is_cut() {
    // cutter [8,12]: C is out of reach; the list still works
    let f = format!(
        r#"{},{},{},{},{{"type":"extrude","id":"F9","op":"cut","sketch":"S9","end":"blind","depth":10,"flip":true,"target":[{{"body":"A"}},{{"body":"C"}}]}}"#,
        named_box(1, 0.0, 10.0, "A"),
        named_box(2, 5.0, 15.0, "B"),
        named_box(3, 20.0, 30.0, "C"),
        rect_sketch("S9", 8.0, 12.0)
    );
    let r = run(&f);
    r.clean();
    assert!(near(r.volume_of("A"), 800.0) && near(r.volume_of("B"), 1000.0) && near(r.volume_of("C"), 1000.0), "{:?}", r.mass_bodies());
}

#[test]
fn a_target_the_tool_does_not_reach_is_cut_no_effect() {
    let f = format!(
        r#"{},{},{},{{"type":"extrude","id":"F9","op":"cut","sketch":"S9","end":"blind","depth":10,"flip":true,"target":{{"body":"C"}}}}"#,
        named_box(1, 0.0, 10.0, "A"),
        named_box(3, 20.0, 30.0, "C"),
        rect_sketch("S9", 8.0, 12.0)
    );
    let r = run(&f);
    assert!(r.errors().iter().any(|e| e.ends_with("cutNoEffect")), "{:?}", r.errors());
    assert!(near(r.volume_of("A"), 1000.0) && near(r.volume_of("C"), 1000.0));
}

#[test]
fn an_unknown_name_lists_the_bodies_that_exist() {
    let r = abc_cut(r#","target":{"body":"Cupp"}"#);
    assert!(r.errors().iter().any(|e| e.ends_with("missingReference")), "{:?}", r.errors());
    let p = r.error_params("missingReference").to_string();
    assert!(p.contains(r#""bodies":["A","B","C"]"#) && p.contains(r#""name":"Cupp""#), "{p}");
    // nothing was cut
    assert!(near(r.volume_of("A"), 1000.0) && near(r.volume_of("B"), 1000.0));
}

#[test]
fn an_unknown_name_in_a_list_is_refused_too() {
    let r = abc_cut(r#","target":[{"body":"A"},{"body":"Nope"}]"#);
    assert!(r.errors().iter().any(|e| e.ends_with("missingReference")), "{:?}", r.errors());
    assert!(near(r.volume_of("A"), 1000.0));
}

#[test]
fn an_empty_list_and_a_new_body_with_a_target_are_refused() {
    let r = abc_cut(r#","target":[]"#);
    assert!(r.errors().iter().any(|e| e.ends_with("invalidValue")), "{:?}", r.errors());
    let f = format!("{},{},{{\"type\":\"extrude\",\"id\":\"F9\",\"op\":\"newBody\",\"sketch\":\"S9\",\"end\":\"blind\",\"depth\":10,\"body\":\"N\",\"target\":{{\"body\":\"A\"}}}}", named_box(1, 0.0, 10.0, "A"), rect_sketch("S9", 20.0, 22.0));
    let r = run(&f);
    assert!(r.errors().iter().any(|e| e.ends_with("invalidValue")), "{:?}", r.errors());
}

#[test]
fn a_revolve_cut_takes_a_target() {
    // a cylinder r 3 about the axis x = 5 (front plane), overlapping A and B; target A leaves B whole
    let rev = |extra: &str| {
        format!(
            r#"{},{},{{"type":"sketch","id":"SR","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":5,"y":-3}},{{"id":"b","x":8,"y":-3}},{{"id":"c","x":8,"y":3}},{{"id":"d","x":5,"y":3}}],"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}},{{"type":"line","id":"ax","p1":"a","p2":"d","construction":true}}],"constraints":[]}},{{"type":"revolve","id":"FR","op":"cut","sketch":"SR","axis":"ax","angle":360{extra}}}"#,
            named_box(1, 0.0, 10.0, "A"),
            named_box(2, 5.0, 15.0, "B")
        )
    };
    let all = run(&rev(""));
    all.clean();
    assert!(all.volume_of("A") < 999.0 && all.volume_of("B") < 999.0, "{:?}", all.mass_bodies());
    let one = run(&rev(r#","target":{"body":"A"}"#));
    one.clean();
    assert!(one.volume_of("A") < 999.0 && near(one.volume_of("B"), 1000.0), "{:?}", one.mass_bodies());
}

#[test]
fn a_loft_cut_takes_a_target() {
    // a 6x6 prism lofted 10 high from the origin: 180 of A (x -4..4) and 150 of B (x -2..8) lie inside it
    let plane = r#"{"type":"plane","id":"P","mode":"offset","base":{"kind":"std","id":"top"},"offset":10}"#;
    let sq = |id: &str, pl: &str| format!(
        r#"{{"type":"sketch","id":"{id}","plane":{pl},"points":[{{"id":"a","x":-3,"y":-3}},{{"id":"b","x":3,"y":-3}},{{"id":"c","x":3,"y":3}},{{"id":"d","x":-3,"y":3}}],"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}}"#
    );
    let doc = |extra: &str| format!(
        r#"{},{},{plane},{},{},{{"type":"loft","id":"L","op":"cut","sketches":["S1","S2"]{extra}}}"#,
        named_box(1, -4.0, 4.0, "A"),
        named_box(2, -2.0, 8.0, "B"),
        sq("S1", r#"{"kind":"std","id":"top"}"#),
        sq("S2", r#"{"kind":"feature","id":"P"}"#)
    );
    let all = run(&doc(""));
    all.clean();
    assert!(near(all.volume_of("A"), 620.0) && near(all.volume_of("B"), 850.0), "{:?}", all.mass_bodies());
    let one = run(&doc(r#","target":{"body":"A"}"#));
    one.clean();
    assert!(near(one.volume_of("A"), 620.0) && near(one.volume_of("B"), 1000.0), "{:?}", one.mass_bodies());
}

#[test]
fn a_hole_takes_a_target() {
    // A: [0,10], B: [5,15] (x), both 10 high; a through hole of diameter 2 at x = 7.5 (inside both) drilled down from the top face (a plane at the box height).
    // Target A drills A only (A loses 10 pi, B stays whole), target B the other way round.
    let pi = std::f64::consts::PI;
    let doc = |extra: &str| format!(
        r#"{},{},{{"type":"plane","id":"PH","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":10}},{{"type":"sketch","id":"SH","plane":{{"kind":"feature","id":"PH"}},"points":[{{"id":"p1","x":7.5,"y":5}}],"entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}},{{"type":"hole","id":"H","sketch":"SH","kind":"simple","diameter":2,"end":"through"{extra}}}"#,
        named_box(1, 0.0, 10.0, "A"),
        named_box(2, 5.0, 15.0, "B")
    );
    // (without a target a hole through the overlap of two bodies is `booleanFailed` today: the multi-body cut of the whole tool
    // is the unchanged path, so it is not asserted here)
    let one = run(&doc(r#","target":{"body":"A"}"#));
    one.clean();
    assert!(near(one.volume_of("A"), 1000.0 - 10.0 * pi) && near(one.volume_of("B"), 1000.0), "{:?}", one.mass_bodies());
    let other = run(&doc(r#","target":{"body":"B"}"#));
    other.clean();
    assert!(near(other.volume_of("B"), 1000.0 - 10.0 * pi) && near(other.volume_of("A"), 1000.0), "{:?}", other.mass_bodies());
    let none = run(&doc(r#","target":{"body":"Nope"}"#));
    assert!(none.errors().iter().any(|e| e.starts_with("H:")), "{:?}", none.errors());
}

#[test]
fn a_sweep_cut_takes_a_target() {
    // A: [0,10], B: [5,15] (x). A straight sweep of the 2 x 2 square [6.5,8.5] x [4,6] (top plane) along the box height cuts
    // 4 x 10 = 40 from each body it meets; target A leaves B whole.
    let doc = |extra: &str| format!(
        r#"{},{},{{"type":"sketch","id":"SP","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":7.5,"y":0}},{{"id":"b","x":7.5,"y":10}}],"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}}],"constraints":[]}},{},{{"type":"sweep","id":"W","op":"cut","sketch":"SQ","path":{{"kind":"sketch","id":"SP"}}{extra}}}"#,
        named_box(1, 0.0, 10.0, "A"),
        named_box(2, 5.0, 15.0, "B"),
        r#"{"type":"sketch","id":"SQ","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":6.5,"y":4},{"id":"p2","x":8.5,"y":4},{"id":"p3","x":8.5,"y":6},{"id":"p4","x":6.5,"y":6}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#
    );
    let all = run(&doc(""));
    all.clean();
    assert!(near(all.volume_of("A"), 960.0) && near(all.volume_of("B"), 960.0), "{:?}", all.mass_bodies());
    let one = run(&doc(r#","target":{"body":"A"}"#));
    one.clean();
    assert!(near(one.volume_of("A"), 960.0) && near(one.volume_of("B"), 1000.0), "{:?}", one.mass_bodies());
}
