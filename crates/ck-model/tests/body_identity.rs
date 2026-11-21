//! Body identity: `combine` union of tools that do not touch the target, `deleteBody` by a shared name, and the
//! `faceBody` index (`ck_topo::mass::face_bodies`) that agrees with `massProps {bodies:true}`.

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

#[allow(dead_code)]
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


impl Run {
    fn warnings(&self) -> Vec<String> {
        self.result.warnings.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn warning_params(&self, code: &str) -> ck_json::Json {
        let e = self.result.warnings.iter().find(|e| e.error.code == code).unwrap_or_else(|| panic!("no {code}: {:?}", self.warnings()));
        ck_json::Json::Obj(e.error.params.clone())
    }
    /// Faces per body index, from `face_bodies`.
    fn faces_per_body(&self) -> Vec<usize> {
        let b = self.body.as_ref().unwrap();
        let mut n = vec![0; b.num_lumps()];
        for i in ck_topo::mass::face_bodies(b) {
            n[i as usize] += 1;
        }
        n
    }
}

fn cube(id: u32, x0: f64, name: &str) -> String {
    let (a, b) = (x0, x0 + 10.0);
    format!(
        r#"{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{a},"y":0}},{{"id":"p2","x":{b},"y":0}},{{"id":"p3","x":{b},"y":10}},{{"id":"p4","x":{a},"y":10}}],
          "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
         {{"type":"extrude","id":"F{id}","op":"newBody","body":"{name}","sketch":"S{id}","end":"blind","depth":10}}"#
    )
}

/// Disjoint 10 mm cubes A (x 0), B (x 30), C (x 60).
fn abc() -> String {
    format!("{},{},{}", cube(2, 0.0, "A"), cube(4, 30.0, "B"), cube(6, 60.0, "C"))
}

fn union(tools: &str, extra: &str) -> String {
    format!(r#"{{"type":"combine","id":"U","op":"union","target":{{"body":"A"}},"tools":[{tools}],"body":"Trio"{extra}}}"#)
}

/// The same with `disjoint: "join"`.
fn join(tools: &str, extra: &str) -> String {
    union(tools, &format!(r#","disjoint":"join"{extra}"#))
}

fn index_of(r: &Run) -> Vec<(i64, Option<String>)> {
    r.mass_bodies().iter().map(|b| (b.get_f64("index").unwrap() as i64, b.get_str("name").map(String::from))).collect()
}

#[test]
fn mass_props_bodies_carry_name_and_index() {
    let r = run(&format!("{},{}", abc(), cube(8, 90.0, "D").replace(r#""op":"newBody","body":"D","#, r#""op":"newBody","#)));
    r.clean();
    assert_eq!(index_of(&r), vec![(0, Some("A".into())), (1, Some("B".into())), (2, Some("C".into())), (3, None)]);
    // `name` is present (null) for an unnamed body
    assert_eq!(r.mass_bodies()[3].get("name"), Some(&ck_json::Json::Null));
    assert_eq!(r.faces_per_body(), vec![6, 6, 6, 6]);
}

#[test]
fn union_of_disjoint_tools_without_join_keeps_them_named_and_warns() {
    let r = run(&format!("{},{}", abc(), union(r#"{"body":"B"},{"body":"C"}"#, "")));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.names(), vec![Some("Trio".into()), Some("B".into()), Some("C".into())]);
    assert!((r.volume() - 3000.0).abs() < 1e-6);
    assert_eq!(ck_json::to_string(&r.warning_params("unionDisjoint")), r#"{"tools":["B","C"],"kept":false}"#);
}

#[test]
fn union_of_disjoint_tools_with_join_consumes_them_into_one_body() {
    let r = run(&format!("{},{}", abc(), join(r#"{"body":"B"},{"body":"C"}"#, "")));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.names(), vec![Some("Trio".into()); 3]);
    assert!((r.volume() - 3000.0).abs() < 1e-6);
    assert_eq!(r.faces_per_body(), vec![6, 6, 6]);
    assert!(r.warnings().contains(&"U:unionDisjoint".to_string()), "{:?}", r.warnings());
    let p = ck_json::to_string(&r.warning_params("unionDisjoint"));
    assert_eq!(p, r#"{"tools":["B","C"],"kept":false}"#);
    // the tools are gone as bodies: no lump called B or C
    assert!(r.names().iter().all(|n| n.as_deref() == Some("Trio")));
    // deleteBody keep by the shared name keeps every lump of the body
    let k = run(&format!("{},{},{}", abc(), join(r#"{"body":"B"},{"body":"C"}"#, ""), r#"{"type":"deleteBody","id":"K","mode":"keep","bodies":[{"body":"Trio"}]}"#));
    assert!(k.errors().is_empty(), "{:?}", k.errors());
    assert_eq!(k.lumps(), 3);
    assert!((k.volume() - 3000.0).abs() < 1e-6);
    let d = run(&format!("{},{},{}", abc(), join(r#"{"body":"B"},{"body":"C"}"#, ""), r#"{"type":"deleteBody","id":"K","mode":"delete","bodies":[{"body":"Trio"}]}"#));
    assert!(d.body.is_none(), "deleting Trio removes all three lumps");
}

#[test]
fn union_of_disjoint_tools_with_keep_tools_leaves_them_alone() {
    let r = run(&format!("{},{}", abc(), union(r#"{"body":"B"},{"body":"C"}"#, r#","keepTools":true"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.names(), vec![Some("Trio".into()), Some("B".into()), Some("C".into())]);
    assert!((r.volume() - 3000.0).abs() < 1e-6, "no tool is duplicated: {}", r.volume());
    assert_eq!(r.warning_params("unionDisjoint").get("kept"), Some(&ck_json::Json::Bool(true)));
    // keep only B by its own name
    let k = run(&format!("{},{},{}", abc(), union(r#"{"body":"B"},{"body":"C"}"#, r#","keepTools":true"#), r#"{"type":"deleteBody","id":"K","mode":"keep","bodies":[{"body":"B"}]}"#));
    assert_eq!(k.names(), vec![Some("B".into())]);
}

#[test]
fn union_of_touching_and_disjoint_tools() {
    // B at x 10..20 touches A (a shared face), C does not
    let r = run(&format!("{},{},{},{}", cube(2, 0.0, "A"), cube(4, 10.0, "B"), cube(6, 60.0, "C"), join(r#"{"body":"B"},{"body":"C"}"#, "")));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.lumps(), 2);
    assert_eq!(r.names(), vec![Some("Trio".into()); 2]);
    assert!((r.volume() - 3000.0).abs() < 1e-6);
    assert_eq!(ck_json::to_string(&r.warning_params("unionDisjoint")), r#"{"tools":["C"],"kept":false}"#);
    // all tools touching: no warning
    let t = run(&format!("{},{},{}", cube(2, 0.0, "A"), cube(4, 10.0, "B"), union(r#"{"body":"B"}"#, "")));
    assert!(t.warnings().is_empty(), "{:?}", t.warnings());
    assert_eq!(t.lumps(), 1);
}

#[test]
fn union_without_a_name_does_not_bring_tool_names_back() {
    let r = run(&format!(
        "{},{}",
        format!("{},{}", cube(2, 0.0, "A").replace(r#""op":"newBody","body":"A","#, r#""op":"newBody","#), cube(4, 30.0, "B")),
        r#"{"type":"combine","id":"U","op":"union","target":{"face":"F2.C1"},"tools":[{"body":"B"}],"disjoint":"join"}"#
    ));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.names(), vec![None, None], "{:?}", r.names());
}

#[test]
fn deleting_by_a_shared_name_acts_on_every_lump_with_it() {
    // two copies named alike are one body of two lumps: keep and delete take both
    let base = format!("{},{}", cube(2, 0.0, "A"), r#"{"type":"moveBody","id":"M","bodies":[{"body":"A"}],"copy":true,"body":"Z","translate":[0,30,0]}"#);
    let r = run(&format!("{},{}", base, cube(4, 60.0, "B")));
    let k = run(&format!("{},{},{}", base, cube(4, 60.0, "B"), r#"{"type":"deleteBody","id":"K","mode":"keep","bodies":[{"body":"Z"}]}"#));
    assert_eq!(r.lumps(), 3);
    assert_eq!(k.names(), vec![Some("Z".into())]);
}
