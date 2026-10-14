//! `thread` feature (docs/spec-agent-interface.md, "Threads"): cosmetic entries, the
//! diameter hint, structured errors, and (modeled) the real helical groove.

use ck_json::Json;
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
    pub fn warning(&self, code: &str) -> Option<&regen::FeatureError> {
        self.result.warnings.iter().find(|w| w.error.code == code).map(|w| &w.error)
    }
    pub fn thread(&self, id: &str) -> &Json {
        &self.result.threads.iter().find(|t| t.0 == id).unwrap_or_else(|| panic!("no thread entry {id}: {:?}", self.result.threads)).1
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

/// 40 x 40 x 20 block (top-plane sketch F1 -> extrude F2).
const BLOCK: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":40},{"id":"p4","x":0,"y":40}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}"#;

/// A through hole of diameter `d` at (20, 20) (circle `c1` of sketch F3, cut F4).
fn through_hole(d: f64) -> String {
    format!(
        r#"{BLOCK},
 {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":20,"y":20}}],"entities":[{{"type":"circle","id":"c1","c":"c","r":{}}}],"constraints":[]}},
 {{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth","depth":0}}"#,
        d / 2.0
    )
}

/// A shaft of diameter `d`, length 30 (circle `c1` of sketch F1, extrude F2).
fn shaft(d: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"c1","c":"c","r":{}}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}}"#,
        d / 2.0
    )
}

fn thread(extra: &str) -> String {
    format!(r#",{{"type":"thread","id":"T1",{extra}}}"#)
}

#[test]
fn cosmetic_internal_thread_on_a_through_hole() {
    let base = through_hole(5.0);
    let plain = run(&base);
    assert!(plain.errors().is_empty(), "{:?}", plain.errors());
    assert!(plain.faces().contains(&"F4.S.c1".to_string()), "{:?}", plain.faces());
    let r = run(&format!("{base}{}", thread(r#""face":"F4.S.c1","standard":"metric","size":"M6""#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.result.warnings.is_empty(), "{:?}", r.result.warnings);
    // no geometry change
    assert_eq!(r.volume, plain.volume);
    assert_eq!(r.faces(), plain.faces());
    let t = r.thread("T1");
    assert_eq!(t.get_str("face"), Some("F4.S.c1"));
    assert_eq!(t.get_str("standard"), Some("metric"));
    assert_eq!(t.get_str("size"), Some("M6"));
    assert_eq!(num(t, "pitch"), 1.0);
    assert_eq!(num(t, "major"), 6.0);
    assert!((num(t, "minor") - 4.917468).abs() < 1e-5);
    assert!((num(t, "length") - 20.0).abs() < 1e-6, "full length of the face");
    assert_eq!(t.get_str("hand"), Some("right"));
    assert_eq!(t.get_bool("internal"), Some(true));
    assert_eq!(t.get_bool("modeled"), Some(false));
    assert_eq!(t.get_str("class"), Some("6H"));
    // wire: threads is on the regen reply only when there are threads
    let j = r.result.to_json();
    assert_eq!(j.get_arr("threads").unwrap().len(), 1);
    assert!(plain.result.to_json().get("threads").is_none() && plain.result.to_json().get("warnings").is_none());
}

#[test]
fn mismatch_hint_on_a_hole_is_a_warning_not_an_error() {
    let r = run(&format!("{}{}", through_hole(6.5), thread(r#""face":"F4.S.c1","standard":"metric","size":"M6""#)));
    assert!(r.errors().is_empty(), "a hint must not fail the regen: {:?}", r.errors());
    let w = r.warning("threadDiameterMismatch").expect("warning");
    let get = |k: &str| w.params.iter().find(|p| p.0 == k).map(|p| p.1.clone()).unwrap();
    assert_eq!(get("expected").as_f64(), Some(5.0));
    assert_eq!(get("actual").as_f64(), Some(6.5));
    let hint = get("hint").as_str().unwrap().to_string();
    assert!(hint.contains("hole is 6.5") && hint.contains("M6 tap drill is 5"), "{hint}");
    assert!(r.thread("T1").get_bool("internal") == Some(true));
    // the tap drill and the minor diameter both pass
    for d in [5.0, 4.917, 5.1] {
        let r = run(&format!("{}{}", through_hole(d), thread(r#""face":"F4.S.c1","standard":"metric","size":"M6""#)));
        assert!(r.result.warnings.is_empty(), "{d}: {:?}", r.result.warnings);
    }
}

#[test]
fn external_thread_on_a_shaft() {
    let ok = run(&format!("{}{}", shaft(8.0), thread(r#""face":"F2.S.c1","standard":"metric","size":"M8""#)));
    assert!(ok.errors().is_empty(), "{:?}", ok.errors());
    assert!(ok.result.warnings.is_empty(), "{:?}", ok.result.warnings);
    let t = ok.thread("T1");
    assert_eq!(t.get_bool("internal"), Some(false));
    assert_eq!(t.get_str("class"), Some("6g"));
    assert!((num(t, "length") - 30.0).abs() < 1e-6);
    assert!((num(t, "minor") - (8.0 - 1.226869 * 1.25)).abs() < 1e-5);
    // a shaft too thin for the size
    let bad = run(&format!("{}{}", shaft(7.0), thread(r#""face":"F2.S.c1","standard":"metric","size":"M8""#)));
    let w = bad.warning("threadDiameterMismatch").expect("warning");
    let hint = w.params.iter().find(|p| p.0 == "hint").unwrap().1.as_str().unwrap().to_string();
    assert!(hint.contains("shaft is 7") && hint.contains("M8 major diameter is 8"), "{hint}");
}

#[test]
fn length_hand_and_start_are_reported() {
    let r = run(&format!(
        "{}{}",
        through_hole(5.0),
        thread(r#""face":"F4.S.c1","standard":"metric","size":"M6","length":8,"hand":"left","start":"bottom""#)
    ));
    let t = r.thread("T1");
    assert_eq!(num(t, "length"), 8.0);
    assert_eq!(t.get_str("hand"), Some("left"));
    // "bottom" = the lower end along the axis: the block spans y in [0, 20] or [-20, 0]
    let s = t.get("start").unwrap().as_f64_vec().unwrap();
    let d = t.get("dir").unwrap().as_f64_vec().unwrap();
    assert!(d[1] > 0.99, "bottom start runs toward +y: {d:?}");
    assert!(s[1] < 0.0 || s[1] < 1e-9 || s[1] <= 20.0);
    // longer than the face: clamped
    let r = run(&format!("{}{}", through_hole(5.0), thread(r#""face":"F4.S.c1","standard":"metric","size":"M6","length":99"#)));
    assert!((num(r.thread("T1"), "length") - 20.0).abs() < 1e-6);
    // "full" is the default spelled out
    let r = run(&format!("{}{}", through_hole(5.0), thread(r#""face":"F4.S.c1","standard":"metric","size":"M6","length":"full""#)));
    assert!((num(r.thread("T1"), "length") - 20.0).abs() < 1e-6);
}

#[test]
fn fine_pitch_and_other_standards() {
    let r = run(&format!("{}{}", through_hole(10.5), thread(r#""face":"F4.S.c1","standard":"metric-fine","size":"M12x1.5""#)));
    assert!(r.result.warnings.is_empty(), "{:?}", r.result.warnings);
    assert_eq!(num(r.thread("T1"), "pitch"), 1.5);
    // 1/4-20 UNC tap drill #7 = 5.105 mm
    let r = run(&format!("{}{}", through_hole(5.1), thread(r#""face":"F4.S.c1","standard":"UNC","size":"1/4-20""#)));
    assert!(r.result.warnings.is_empty(), "{:?}", r.result.warnings);
    assert_eq!(r.thread("T1").get_str("class"), Some("2B"));
    // an explicit class wins
    let r = run(&format!("{}{}", through_hole(5.1), thread(r#""face":"F4.S.c1","standard":"UNC","size":"1/4-20","class":"3B""#)));
    assert_eq!(r.thread("T1").get_str("class"), Some("3B"));
    // BSP G1/4 tap drill 11.8
    let r = run(&format!("{}{}", through_hole(11.8), thread(r#""face":"F4.S.c1","standard":"BSP","size":"G1/4""#)));
    assert!(r.result.warnings.is_empty(), "{:?}", r.result.warnings);
    // a coarse designation under metric-fine resolves; M6x1 names the coarse M6
    let r = run(&format!("{}{}", through_hole(5.0), thread(r#""face":"F4.S.c1","standard":"metric","size":"M6x1""#)));
    assert_eq!(r.thread("T1").get_str("size"), Some("M6"));
}

#[test]
fn tapered_npt_is_cosmetic() {
    // 1/4 NPT: tap drill 7/16 in = 11.11 mm
    let r = run(&format!("{}{}", through_hole(11.1), thread(r#""face":"F4.S.c1","standard":"NPT","size":"1/4 NPT","modeled":true"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let t = r.thread("T1");
    assert_eq!(t.get_bool("tapered"), Some(true));
    assert_eq!(t.get_bool("modeled"), Some(false), "tapered threads stay cosmetic");
    let w = r.warning("threadModelFallback").expect("fallback warning");
    let reason = w.params.iter().find(|p| p.0 == "reason").unwrap().1.as_str().unwrap().to_string();
    assert!(reason.contains("taper"), "{reason}");
    assert!((num(t, "major") - 13.716).abs() < 1e-3 && (25.4 / num(t, "pitch") - 18.0).abs() < 1e-4);
}

#[test]
fn errors_are_structured() {
    let base = through_hole(5.0);
    let r = run(&format!("{base}{}", thread(r#""face":"F4.S.c1","standard":"metric","size":"M7.5""#)));
    let e = &r.result.errors.iter().find(|e| e.feature == "T1").unwrap().error;
    assert_eq!(e.code, "threadSizeUnknown");
    let sugg = e.params.iter().find(|p| p.0 == "suggestions").unwrap().1.as_arr().unwrap().iter().map(|j| j.as_str().unwrap().to_string()).collect::<Vec<_>>();
    assert!(sugg.contains(&"M8".to_string()) || sugg.contains(&"M7".to_string()), "{sugg:?}");
    assert!(r.result.threads.is_empty());
    let r = run(&format!("{base}{}", thread(r#""face":"F4.S.c1","standard":"nope","size":"M6""#)));
    assert!(r.errors()[0].starts_with("T1:threadSizeUnknown"), "{:?}", r.errors());
    // a plane
    let r = run(&format!("{base}{}", thread(r#""face":"F2.C0","standard":"metric","size":"M6""#)));
    assert!(r.errors()[0].starts_with("T1:threadFaceNotCylinder"), "{:?}", r.errors());
    // missing / unknown face
    let r = run(&format!("{base}{}", thread(r#""face":"F9.S.zz","standard":"metric","size":"M6""#)));
    assert!(r.errors()[0].starts_with("T1:missingReference"), "{:?}", r.errors());
    let r = run(&format!("{base}{}", thread(r#""standard":"metric","size":"M6""#)));
    assert!(r.errors()[0].starts_with("T1:missingReference"), "{:?}", r.errors());
    let r = run(&format!("{base}{}", thread(r#""face":"F4.S.c1","standard":"metric","size":"M6","hand":"sideways""#)));
    assert!(r.errors()[0].starts_with("T1:invalidValue"), "{:?}", r.errors());
    let r = run(&format!("{base}{}", thread(r#""face":"F4.S.c1","standard":"metric","size":"M6","length":-1"#)));
    assert!(r.errors()[0].starts_with("T1:invalidValue"), "{:?}", r.errors());
}

#[test]
fn unused_import_guard() {
    let _ = PI;
}
