//! Patterns and mirrors carry the threads of their seeds (docs/spec-agent-interface.md, "Threads").

use ck_json::Json;
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

pub struct Run {
    pub result: RegenResult,
    pub body: Option<ck_topo::body::Body>,
    pub volume: f64,
    pub ms: f64,
}

impl Run {
    pub fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    pub fn warnings(&self) -> Vec<String> {
        self.result.warnings.iter().map(|w| format!("{}:{}", w.feature, w.error.describe())).collect()
    }
    pub fn warning(&self, code: &str) -> Option<&regen::FeatureError> {
        self.result.warnings.iter().find(|w| w.error.code == code).map(|w| &w.error)
    }
    pub fn thread(&self, id: &str) -> &Json {
        &self.result.threads.iter().find(|t| t.0 == id).unwrap_or_else(|| panic!("no thread entry {id}: {:?}", entry_ids(self))).1
    }
    pub fn faces(&self) -> Vec<String> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        v.sort();
        v
    }
}

pub fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let t = std::time::Instant::now();
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let ms = t.elapsed().as_secs_f64() * 1000.0;
    let volume = ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    Run { result, body: ex.brep.body.clone(), volume, ms }
}

fn num(j: &Json, k: &str) -> f64 {
    j.get_f64(k).unwrap_or_else(|| panic!("{k} in {}", ck_json::to_string(j)))
}
fn vec3(j: &Json, k: &str) -> [f64; 3] {
    j.get(k).and_then(|v| v.as_vec3()).unwrap_or_else(|| panic!("{k} in {}", ck_json::to_string(j)))
}

/// 80 x 80 x 10 plate centred on the origin, a through hole of diameter `d` at (20, 0) (F3 sketch, F4 cut).
fn plate(d: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":-40,"y":-40}},{{"id":"p2","x":40,"y":-40}},{{"id":"p3","x":40,"y":40}},{{"id":"p4","x":-40,"y":40}}],
  "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
 {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":20,"y":0}}],"entities":[{{"type":"circle","id":"c1","c":"c","r":{}}}],"constraints":[]}},
 {{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth","depth":0}}"#,
        d / 2.0
    )
}
fn thread(extra: &str) -> String {
    format!(r#",{{"type":"thread","id":"F5","face":"F4.S.c1","standard":"metric","size":"M5"{extra}}}"#)
}
fn cpat(extra: &str) -> String {
    format!(r#",{{"type":"cpattern","id":"F7","seeds":["F4"],"axis":{{"kind":"axis","axis":"y"}},"count":4,"angle":360,"equal":true{extra}}}"#)
}
const TAP: f64 = 4.2;

fn entry_ids(r: &Run) -> Vec<String> {
    r.result.threads.iter().map(|t| t.0.clone()).collect()
}

#[test]
fn circular_pattern_carries_a_cosmetic_thread() {
    let plain = run(&format!("{}{}", plate(TAP), cpat("")));
    let r = run(&format!("{}{}{}", plate(TAP), thread(""), cpat("")));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.result.warnings.is_empty(), "{:?}", r.warnings());
    assert_eq!(r.volume, plain.volume, "cosmetic: no geometry change");
    assert_eq!(entry_ids(&r), vec!["F5", "F5@F7.1", "F5@F7.2", "F5@F7.3"]);
    // instance k is the source turned by 90 k degrees about z
    let want = [[20.0, 0.0], [0.0, -20.0], [-20.0, 0.0], [0.0, 20.0]];
    let faces = r.faces();
    for (k, w) in want.iter().enumerate() {
        let id = if k == 0 { "F5".to_string() } else { format!("F5@F7.{k}") };
        let t = r.thread(&id);
        let s = vec3(t, "start");
        assert!((s[0] - w[0]).abs() < 1e-5 && (s[2] - w[1]).abs() < 1e-5, "{id} start {s:?}");
        let d = vec3(t, "dir");
        assert!(d[0].abs() < 1e-9 && d[2].abs() < 1e-9 && d[1].abs() > 0.999, "{id} dir {d:?}");
        assert_eq!(t.get_str("size"), Some("M5"));
        assert_eq!(t.get_str("class"), Some("6H"));
        assert_eq!(t.get_str("hand"), Some("right"));
        assert!((num(t, "length") - 10.0).abs() < 1e-6);
        assert_eq!(t.get_bool("internal"), Some(true));
        if k == 0 {
            assert!(t.get("instanceOf").is_none());
        } else {
            assert_eq!(t.get_str("face"), Some(format!("F4.S.c1@F7.{k}").as_str()));
            assert_eq!(t.get_str("instanceOf"), Some("F5"));
            assert!(faces.contains(&format!("F4.S.c1@F7.{k}")), "{faces:?}");
        }
    }
    // all four start at the same (mouth) end
    let z0 = vec3(r.thread("F5"), "start")[1];
    for k in 1..4 {
        assert!((vec3(r.thread(&format!("F5@F7.{k}")), "start")[1] - z0).abs() < 1e-6);
    }
    assert_eq!(r.result.to_json().get_arr("threads").unwrap().len(), 4);
}

#[test]
fn carry_threads_false_and_explicit_seeds() {
    let off = run(&format!("{}{}{}", plate(TAP), thread(""), cpat(r#","carryThreads":false"#)));
    assert_eq!(entry_ids(&off), vec!["F5"]);
    // explicit seeds name the thread too: same result as the automatic default
    let auto = run(&format!("{}{}{}", plate(TAP), thread(""), cpat("")));
    let both = run(&format!("{}{}{}", plate(TAP), thread(""), cpat("").replace(r#"["F4"]"#, r#"["F4","F5"]"#)));
    assert!(both.errors().is_empty(), "{:?}", both.errors());
    assert_eq!(entry_ids(&both), entry_ids(&auto));
    assert_eq!(both.volume, auto.volume);
    // an explicit thread seed wins over carryThreads:false
    let forced = run(&format!("{}{}{}", plate(TAP), thread(""), cpat(r#","carryThreads":false"#).replace(r#"["F4"]"#, r#"["F4","F5"]"#)));
    assert_eq!(entry_ids(&forced).len(), 4);
}

#[test]
fn a_thread_added_after_the_pattern_still_follows() {
    // hole, pattern, THEN the thread: same entries as thread first
    let first = run(&format!("{}{}{}", plate(TAP), thread(""), cpat("")));
    let after = run(&format!("{}{}{}", plate(TAP), cpat(""), thread("")));
    assert!(after.errors().is_empty(), "{:?}", after.errors());
    assert_eq!(entry_ids(&after), entry_ids(&first));
    for id in entry_ids(&first) {
        assert_eq!(ck_json::to_string(first.thread(&id)), ck_json::to_string(after.thread(&id)), "{id}");
    }
    // with carryThreads:false on the pattern the late thread stays alone
    let off = run(&format!("{}{}{}", plate(TAP), cpat(r#","carryThreads":false"#), thread("")));
    assert_eq!(entry_ids(&off), vec!["F5"]);
    // modeled, after the pattern: N grooves again
    let base = run(&format!("{}{}", plate(TAP), cpat("")));
    let m = run(&format!("{}{}{}", plate(TAP), cpat(""), thread(r#","modeled":true"#)));
    assert!(m.errors().is_empty() && m.warning("threadModelFallback").is_none(), "{:?} {:?}", m.errors(), m.warnings());
    let one = run(&format!("{}{}", plate(TAP), thread(r#","modeled":true"#)));
    let g1 = run(&plate(TAP)).volume - one.volume;
    assert!(((base.volume - m.volume) - 4.0 * g1).abs() <= 1e-3 * 4.0 * g1);
}

#[test]
fn editing_the_source_thread_updates_every_instance() {
    let a = run(&format!("{}{}{}", plate(TAP), thread(""), cpat("")));
    let b = run(&format!("{}{}{}", plate(5.0), thread("").replace(r#""size":"M5""#, r#""size":"M6","hand":"left","length":6"#), cpat("")));
    assert_eq!(a.thread("F5@F7.2").get_str("size"), Some("M5"));
    let t = b.thread("F5@F7.2");
    assert_eq!(t.get_str("size"), Some("M6"), "{}", ck_json::to_string(t));
    assert_eq!(t.get_str("hand"), Some("left"));
    assert!((num(t, "length") - 6.0).abs() < 1e-6);
}

#[test]
fn linear_pattern_carries_the_thread() {
    let lp = r#",{"type":"lpattern","id":"F7","seeds":["F4"],"dir":{"kind":"axis","axis":"z"},"spacing":15,"count":3}"#;
    let r = run(&format!("{}{}{}", plate(TAP), thread(""), lp));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(entry_ids(&r), vec!["F5", "F5@F7.1", "F5@F7.2"]);
    let s = vec3(r.thread("F5@F7.2"), "start");
    assert!((s[0] - 20.0).abs() < 1e-6 && (s[2] - 30.0).abs() < 1e-6, "{s:?}");
    assert_eq!(r.thread("F5@F7.2").get_str("face"), Some("F4.S.c1@F7.2"));
}

#[test]
fn mirror_carries_the_thread() {
    let m = r#",{"type":"mirror","id":"F7","seeds":["F4"],"plane":{"kind":"std","id":"right"}}"#;
    let r = run(&format!("{}{}{}", plate(TAP), thread(r#","hand":"left""#), m));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.result.warnings.is_empty(), "{:?}", r.warnings());
    assert_eq!(entry_ids(&r), vec!["F5", "F5@F7.1"]);
    let t = r.thread("F5@F7.1");
    let s = vec3(t, "start");
    assert!((s[0] + 20.0).abs() < 1e-6 && s[2].abs() < 1e-6, "{s:?}");
    assert_eq!(t.get_str("face"), Some("F4.S.c1@F7.1"));
    assert_eq!(t.get_str("hand"), Some("left"), "a mirrored thread keeps its hand");
    assert_eq!(t.get_str("instanceOf"), Some("F5"));
    // body mirror: the whole body and its thread
    let mb = r#",{"type":"mirror","id":"F7","seeds":[],"plane":{"kind":"std","id":"right"},"body":true}"#;
    let b = run(&format!("{}{}{}", plate(TAP), thread(""), mb));
    assert!(b.errors().is_empty(), "{:?}", b.errors());
    assert_eq!(entry_ids(&b), vec!["F5", "F5@F7.1"]);
}

/// Total removed volume of modeled threads: N instances remove N grooves.
#[test]
fn modeled_threads_remove_n_grooves() {
    let mt = thread(r#","modeled":true"#);
    let base1 = run(&plate(TAP));
    let one = run(&format!("{}{}", plate(TAP), mt));
    assert!(one.warning("threadModelFallback").is_none(), "{:?}", one.warnings());
    let g1 = base1.volume - one.volume;
    assert!(g1 > 1.0, "{g1}");
    let plain4 = run(&format!("{}{}", plate(TAP), cpat("")));
    let r4 = run(&format!("{}{}{}", plate(TAP), mt, cpat("")));
    eprintln!("4x modeled: {:.0} ms", r4.ms);
    assert!(r4.errors().is_empty(), "{:?}", r4.errors());
    assert!(r4.result.warnings.is_empty(), "{:?}", r4.warnings());
    let g4 = plain4.volume - r4.volume;
    assert!((g4 - 4.0 * g1).abs() <= 1e-3 * 4.0 * g1, "4 grooves {g4} vs 4 x {g1}");
    for k in 0..4 {
        let id = if k == 0 { "F5".to_string() } else { format!("F5@F7.{k}") };
        assert_eq!(r4.thread(&id).get_bool("modeled"), Some(true), "{id}");
    }
    let b = r4.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty());
    assert_eq!(b.num_lumps(), 1);
    // groove faces named consistently
    let faces = r4.faces();
    for k in 1..4 {
        assert!(faces.iter().any(|n| n.starts_with("F5.T") && n.ends_with(&format!("@F7.{k}"))), "{faces:?}");
    }
    // mirror: a right-hand groove again, same volume
    let m = r#",{"type":"mirror","id":"F7","seeds":["F4"],"plane":{"kind":"std","id":"right"}}"#;
    let plain_m = run(&format!("{}{}", plate(TAP), m));
    let rm = run(&format!("{}{}{}", plate(TAP), mt, m));
    assert!(rm.errors().is_empty() && rm.warning("threadModelFallback").is_none(), "{:?} {:?}", rm.errors(), rm.warnings());
    let gm = plain_m.volume - rm.volume;
    assert!((gm - 2.0 * g1).abs() <= 1e-3 * 2.0 * g1, "mirror {gm} vs 2 x {g1}");
    assert_eq!(rm.thread("F5@F7.1").get_bool("modeled"), Some(true));
}

#[test]
fn an_instance_wall_that_is_gone_warns() {
    // two big pockets where instances 1 and 3 land: its small hole vanishes into the void
    let pocket = r#",{"type":"sketch","id":"F6","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":20},{"id":"d","x":0,"y":-20}],"entities":[{"type":"circle","id":"c1","c":"c","r":8},{"type":"circle","id":"c2","c":"d","r":8}],"constraints":[]},
 {"type":"extrude","id":"F8","op":"cut","sketch":"F6","end":"throughBoth","depth":0}"#;
    let pat = cpat("").replace("F7", "F9");
    let r = run(&format!("{}{}{}{}", plate(TAP), thread(""), pocket, pat));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let w = r.warning("threadNotCarried").unwrap_or_else(|| panic!("{:?}", r.warnings()));
    assert!(w.params.iter().any(|p| p.0 == "face" && p.1.as_str() == Some("F4.S.c1@F9.1")));
    assert_eq!(entry_ids(&r), vec!["F5", "F5@F9.2"]);
}
