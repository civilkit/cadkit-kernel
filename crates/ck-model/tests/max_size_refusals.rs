//! A refusal because a size is too large carries `maxSize {param, value, unit}`: the largest value of that
//! field that builds. Each case: the refusal names it, the feature builds at it, and a little above it is
//! refused again.
use ck_json::Json;
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

/// Regen `features`; the errors, and the body checked valid when there are none.
fn run(features: &str) -> Vec<regen::RegenError> {
    ck_model::features::max_size::set_blend_search(true);
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    if r.errors.is_empty() {
        let b = ex.brep.body.as_ref().expect("a body");
        let iss = ck_check::validate(b, 1e-6);
        assert!(iss.is_empty(), "{:?}", iss.iter().take(3).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
    }
    r.errors
}

/// The one error's `maxSize` as (param, value, unit), and its hint.
fn max_size(errors: &[regen::RegenError]) -> (String, f64, String, String) {
    assert_eq!(errors.len(), 1, "{:?}", errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let e = &errors[0].error;
    let get = |k: &str| e.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
    let m = get("maxSize").unwrap_or_else(|| panic!("no maxSize: {}", e.describe()));
    let param = m.get("param").and_then(|v| v.as_str()).expect("param").to_string();
    let value = m.get("value").and_then(|v| v.as_f64()).expect("value");
    let unit = m.get("unit").and_then(|v| v.as_str()).expect("unit").to_string();
    let hint = match get("hint") {
        Some(Json::Str(h)) => h,
        _ => panic!("no hint: {}", e.describe()),
    };
    assert!(hint.contains(&format!("{value} {unit}")), "the hint names the size: {hint}");
    (param, value, unit, hint)
}

fn no_max_size(errors: &[regen::RegenError]) {
    assert!(!errors.is_empty(), "expected a refusal");
    for e in errors {
        assert!(e.error.params.iter().all(|(k, _)| k != "maxSize"), "{}", e.error.describe());
    }
}

/// 40 x 20 x 10 box on the top plane (z 0..10; F2.C1 the top face, F2.S.e1 / e3 the long sides).
const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[
  {"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
 "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
  {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

fn with_box(f: &str) -> String {
    format!("{BOX},{f}")
}

/// 40 x 30 x 2 plate on the top plane (the blend table's `plate` base).
const PLATE: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[
  {"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":30},{"id":"p4","x":0,"y":30}],
 "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
  {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":2}"#;

fn with_plate(f: &str) -> String {
    format!("{PLATE},{f}")
}

#[test]
fn shell_thickness() {
    let shell = |t: f64| run(&with_box(&format!(r#"{{"type":"shell","id":"F3","faces":["F2.C1"],"thickness":{t}}}"#)));
    let (param, v, unit, _) = max_size(&shell(12.0));
    assert_eq!((param.as_str(), unit.as_str()), ("thickness", "mm"));
    // the floor (10 high, the top opened) is the limit
    assert!(v > 9.0 && v < 10.0, "{v}");
    assert!(shell(v).is_empty(), "builds at {v}");
    assert!(!shell(v * 1.03 + 0.04).is_empty(), "refused above {v}");
}

#[test]
fn move_face_distance() {
    let mv = |d: f64| run(&with_box(&format!(r#"{{"type":"moveFace","id":"F3","faces":["F2.C1"],"distance":{d}}}"#)));
    let (param, v, unit, _) = max_size(&mv(-12.0));
    assert_eq!((param.as_str(), unit.as_str()), ("distance", "mm"));
    // the sign of the request is kept: the top pushed down by less than the 10 mm height
    assert!(v < -9.0 && v > -10.0, "{v}");
    assert!(mv(v).is_empty(), "builds at {v}");
    assert!(!mv(v * 1.03 - 0.04).is_empty(), "refused past {v}");
}

#[test]
fn draft_angle() {
    // the two long sides 20 apart, 10 high: drafted inward they meet at 45 degrees
    let draft = |a: f64, flip: bool| {
        run(&with_box(&format!(r#"{{"type":"draft","id":"F3","neutral":{{"kind":"std","id":"top"}},"faces":["F2.S.e1","F2.S.e3"],"angle":{a},"flip":{flip}}}"#)))
    };
    let flip = [false, true].into_iter().find(|&fl| draft(60.0, fl).iter().any(|e| e.error.code == "draftTooLarge")).expect("one pull direction narrows the box");
    let (param, v, unit, _) = max_size(&draft(60.0, flip));
    assert_eq!((param.as_str(), unit.as_str()), ("angle", "deg"));
    // (at 45 exactly the sides meet in a ridge with zero-length edges, which is refused)
    assert!(v > 40.0 && v < 45.0, "{v}");
    assert!(draft(v, flip).is_empty(), "builds at {v}");
    assert!(!draft(v * 1.03 + 0.04, flip).is_empty(), "refused above {v}");
}

#[test]
fn extrude_draft_angle() {
    // a 20 x 20 square, 20 deep: the taper closes at atan(10 / 20) = 26.57 degrees
    let sq = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[
      {"id":"p1","x":0,"y":0},{"id":"p2","x":20,"y":0},{"id":"p3","x":20,"y":20},{"id":"p4","x":0,"y":20}],
     "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
      {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#;
    let ex = |a: f64| run(&format!(r#"{sq},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20,"draft":{a}}}"#));
    let (param, v, unit, _) = max_size(&ex(40.0));
    assert_eq!((param.as_str(), unit.as_str()), ("draft", "deg"));
    assert!(v > 25.0 && v < 26.57, "{v}");
    assert!(ex(v).is_empty(), "builds at {v}");
    assert!(!ex(v * 1.03 + 0.04).is_empty(), "refused above {v}");
}

/// The box with one M8 hole at (10, 10) on its top face; `extra` = more hole fields.
fn hole(extra: &str) -> Vec<regen::RegenError> {
    run(&with_box(&format!(
        r#"{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{{"id":"p0","x":10,"y":10}}],
         "entities":[{{"type":"point","id":"e2","p":"p0"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","standard":"iso","size":"M8","fit":"normal"{extra}}}"#
    )))
}

#[test]
fn hole_counterbore_depth() {
    let cb = |d: f64| hole(&format!(r#","kind":"cbore","end":"blind","depth":6,"cboreDiameter":15,"cboreDepth":{d}"#));
    let (param, v, unit, _) = max_size(&cb(8.0));
    assert_eq!((param.as_str(), unit.as_str()), ("cboreDepth", "mm"));
    assert_eq!(v, 5.99);
    assert!(cb(v).is_empty(), "builds at {v}");
    assert!(!cb(6.01).is_empty());
    // a counterbore narrower than the hole is a lower bound, not a size too large: no maxSize
    no_max_size(&hole(r#","kind":"cbore","end":"blind","depth":6,"cboreDiameter":5,"cboreDepth":2"#));
}

#[test]
fn hole_countersink_diameter() {
    // M8 normal is 9 mm; a 90 degree cone in a 3 mm deep hole reaches 2 * (4.5 + 3) = 15 mm
    let cs = |d: f64| hole(&format!(r#","kind":"countersink","end":"blind","depth":3,"cskDiameter":{d},"cskAngle":90"#));
    let (param, v, unit, _) = max_size(&cs(20.0));
    assert_eq!((param.as_str(), unit.as_str()), ("cskDiameter", "mm"));
    assert_eq!(v, 14.99);
    assert!(cs(v).is_empty(), "builds at {v}");
    assert!(!cs(15.01).is_empty());
}

#[test]
fn knurl_depth() {
    // radius 10, 24 grooves: pitch 2.618, a 90 degree V leaves a land up to depth 0.45 pitch = 1.178
    let cyl = r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":10}],"constraints":[]},
        {"type":"extrude","id":"F1","op":"add","sketch":"S1","end":"blind","depth":12}"#;
    let kn = |d: f64| run(&format!(r#"{cyl},{{"type":"knurlBand","id":"K","axis":"z","center":[0,0],"radius":10,"from":0,"to":12,"pattern":"pyramid","count":24,"depth":{d}}}"#));
    let (param, v, unit, _) = max_size(&kn(2.5));
    assert_eq!((param.as_str(), unit.as_str()), ("depth", "mm"));
    assert_eq!(v, 1.17);
    assert!(kn(v).is_empty(), "builds at {v}");
    assert!(!kn(1.19).is_empty());
}

#[test]
fn thin_wall_thickness() {
    // a half circle of radius 10, the wall inside: past the centre of curvature
    let half = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0},{"id":"p1","x":10,"y":0},{"id":"p2","x":-10,"y":0}],
        "entities":[{"type":"arc","id":"e1","c":"c","p1":"p1","p2":"p2"}],"constraints":[]}"#;
    let th = |t: f64| run(&format!(r#"{half},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":5,"thin":{{"thickness":{t},"side":"inside"}}}}"#));
    let (param, v, unit, _) = max_size(&th(15.0));
    assert_eq!((param.as_str(), unit.as_str()), ("thin.thickness", "mm"));
    assert!(v > 9.0 && v < 10.0, "{v}");
    assert!(th(v).is_empty(), "builds at {v}");
    assert!(!th(v * 1.03 + 0.04).is_empty(), "refused above {v}");
}

#[test]
fn fillet_radius() {
    // the blend table's plate-edge-r3 case: a 2 mm plate whose top edge takes a 3 mm round-over, refused
    // `blendOverflow`; the largest radius that builds is named in `maxSize`.
    let f = |r: f64| {
        run(&with_plate(&format!(
            r#"{{"type":"fillet","id":"F90","radius":{r},"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[20,2,0]}}]}}"#
        )))
    };
    let (param, v, unit, _) = max_size(&f(3.0));
    assert_eq!((param.as_str(), unit.as_str()), ("radius", "mm"));
    assert!(v > 1.0 && v < 3.0, "{v}");
    assert!(f(v).is_empty(), "builds at {v}");
    assert!(!f(v * 1.03 + 0.04).is_empty(), "refused above {v}");
}
