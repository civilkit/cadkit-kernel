//! An extrude with no depth: the reference clamps it to a 1e-6 sliver, so it still builds; the `extrudeZeroDepth`
//! warning says so (issue 49).


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


fn disc(id: u32, op: &str, extent: &str) -> String {
    format!(
        r#"{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"a","c":"c","r":10}}]}},
         {{"type":"extrude","id":"F{id}","op":"{op}","sketch":"S{id}","end":"blind",{extent}"body":"b{id}"}}"#
    )
}

fn hint_of(r: &Run) -> String {
    let w = &r.result.warnings[0].error;
    assert_eq!(w.code, "extrudeZeroDepth");
    match w.params.iter().find(|(k, _)| k == "hint") {
        Some((_, ck_json::Json::Str(h))) => h.clone(),
        _ => panic!("no hint"),
    }
}

#[test]
fn depth_zero_warns_with_a_hint_and_still_builds() {
    let r = run(&disc(1, "newBody", r#""depth":0,"#));
    assert_eq!(r.errors(), Vec::<String>::new());
    assert_eq!(r.warnings(), vec!["F1:extrudeZeroDepth"]);
    assert!(hint_of(&r).starts_with("depth is 0; give a non-zero depth"));
    assert!(r.volume() > 0.0 && r.volume() < 1e-3);
}

#[test]
fn depth_missing_negative_and_expression_zero_warn() {
    assert_eq!(run(&disc(1, "newBody", "")).warnings(), vec!["F1:extrudeZeroDepth"]);
    assert_eq!(run(&disc(1, "newBody", r#""depth":-3,"#)).warnings(), vec!["F1:extrudeZeroDepth"]);
    assert_eq!(run(&disc(1, "newBody", r#""depth":"2-2","#)).warnings(), vec!["F1:extrudeZeroDepth"]);
    let mid = disc(1, "newBody", r#""depth":0,"#).replace(r#""end":"blind""#, r#""end":"mid""#);
    assert_eq!(run(&mid).warnings(), vec!["F1:extrudeZeroDepth"]);
}

#[test]
fn a_real_depth_and_a_through_end_do_not_warn() {
    assert!(run(&disc(1, "newBody", r#""depth":5,"#)).warnings().is_empty());
    let through = disc(1, "newBody", "").replace(r#""end":"blind""#, r#""end":"through""#);
    assert!(run(&through).warnings().is_empty());
}

#[test]
fn a_union_with_the_sliver_keeps_the_warning_on_the_extrude() {
    let f = format!("{},{}", disc(1, "newBody", r#""depth":4,"#), disc(2, "add", r#""depth":0,"#).replace(r#","body":"b2""#, ""));
    let r = run(&f);
    assert!(r.warnings().contains(&"F2:extrudeZeroDepth".to_string()), "{:?} {:?}", r.warnings(), r.errors());
}
