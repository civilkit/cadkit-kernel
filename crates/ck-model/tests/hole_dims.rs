//! Custom counterbore / countersink dimensions of `hole` (docs/spec-agent-interface.md, "Hole").

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
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

/// Run a part; (errors as "F3:code {params}", volume, sorted face ids).
fn run(src: &str) -> (Vec<String>, f64, Vec<String>) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    match ex.brep.body.as_ref() {
        Some(b) => {
            let iss = ck_check::validate(b, 1e-6);
            assert!(iss.is_empty(), "{:?}", iss.iter().take(3).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
            let mut ids: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
            ids.sort();
            (errs, ck_topo::mass::mass_props(b).volume, ids)
        }
        None => (errs, 0.0, vec![]),
    }
}

/// Box 40x20x10 with one M8 normal hole at (10,10); `extra` = more hole fields.
fn hole(extra: &str) -> (Vec<String>, f64, Vec<String>) {
    let end = if extra.contains("\"end\"") { "" } else { r#","end":"through""# };
    let src = format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[
          {{"id":"p1","x":0,"y":0}},{{"id":"p2","x":40,"y":0}},{{"id":"p3","x":40,"y":20}},{{"id":"p4","x":0,"y":20}}],
         "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
          {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
        {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{{"id":"p0","x":10,"y":10}}],
         "entities":[{{"type":"point","id":"e2","p":"p0"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","standard":"iso","size":"M8","fit":"normal"{end}{extra}}}]}}"#
    );
    run(&src)
}
const BOX: f64 = 8000.0;
fn removed(extra: &str) -> f64 {
    let (e, v, _) = hole(extra);
    assert!(e.is_empty(), "{e:?}");
    BOX - v
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-3 * b.max(1.0), "{a} vs {b}");
}
fn frustum(h: f64, big: f64, small: f64) -> f64 {
    PI * h / 3.0 * (big * big + big * small + small * small)
}
const R: f64 = 4.5; // M8 normal

#[test]
fn cbore_table_unchanged() {
    // M8 table counterbore: diameter 15, depth 8.6
    close(removed(r#","kind":"cbore""#), PI * (7.5 * 7.5 * 8.6 + R * R * 1.4));
}

#[test]
fn cbore_custom() {
    close(removed(r#","kind":"cbore","cboreDiameter":14,"cboreDepth":4"#), PI * (49.0 * 4.0 + R * R * 6.0));
    // either field alone overrides only itself
    close(removed(r#","kind":"cbore","cboreDepth":4"#), PI * (7.5 * 7.5 * 4.0 + R * R * 6.0));
    close(removed(r#","kind":"cbore","cboreDiameter":14"#), PI * (49.0 * 8.6 + R * R * 1.4));
    let (_, _, ids) = hole(r#","kind":"cbore","cboreDiameter":14,"cboreDepth":4"#);
    for n in ["F4.S.h1", "F4.S.h2", "F4.S.h3"] {
        assert!(ids.iter().any(|i| i == n), "{n} in {ids:?}");
    }
}

#[test]
fn cbore_errors() {
    let code = |x: &str| hole(x).0.into_iter().next().unwrap_or_default();
    assert!(code(r#","kind":"cbore","cboreDiameter":9"#).starts_with("F4:holeCboreInvalid"));
    assert!(code(r#","kind":"cbore","cboreDiameter":6"#).starts_with("F4:holeCboreInvalid"));
    assert!(code(r#","kind":"cbore","cboreDepth":0"#).starts_with("F4:holeCboreInvalid"));
    assert!(code(r#","kind":"cbore","end":"blind","depth":5,"cboreDepth":5"#).starts_with("F4:holeCboreInvalid"));
    assert!(code(r#","kind":"cbore","cboreDepth":4"#).is_empty());
    assert!(code(r#","kind":"cbore","cboreDiameter":9"#).contains("hint"));
}

/// `kind:"cbore"` with a free `diameter` and no `size` (found while converting
/// issue #1691) is `holeInvalid` with a detail and a hint naming size vs
/// diameter — not a bare code.
#[test]
fn cbore_free_diameter_says_what_to_change() {
    let src = r#"{"format":"ck-part","version":1,"features":[
        {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[
          {"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
         "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
          {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
        {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
        {"type":"sketch","id":"F3","plane":{"kind":"face","id":"F2.C1"},"points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"p0","x":10,"y":10}],
         "entities":[{"type":"point","id":"e2","p":"p0"}],"constraints":[]},
        {"type":"hole","id":"F4","sketch":"F3","standard":"iso","kind":"cbore","diameter":8,"end":"through"}]}"#;
    let (errs, _, _) = run(src);
    assert_eq!(errs.len(), 1, "{errs:?}");
    let e = &errs[0];
    assert!(e.starts_with("F4:holeInvalid"), "{e}");
    assert!(e.contains("detail"), "{e}");
    assert!(e.contains("hint"), "{e}");
    assert!(e.contains("size"), "{e}");
    assert!(e.contains("diameter"), "{e}");
}

#[test]
fn csink_legacy_is_plain() {
    let plain = removed("");
    close(plain, PI * R * R * 10.0);
    close(removed(r#","kind":"csink""#), plain);
    close(removed(r#","kind":"csink","cskAngle":60"#), plain); // angle alone does not switch the cone on
}

#[test]
fn csk_custom() {
    // head 16, 90 degrees: cone depth (8 - 4.5) = 3.5
    close(removed(r#","kind":"csink","cskDiameter":16"#), frustum(3.5, 8.0, R) + PI * R * R * 6.5);
    close(removed(r#","kind":"countersink","cskDiameter":16"#), frustum(3.5, 8.0, R) + PI * R * R * 6.5);
    close(removed(r#","kind":"csk","cskDiameter":16,"cskAngle":90"#), frustum(3.5, 8.0, R) + PI * R * R * 6.5);
    // 82 degrees included
    let h = 3.5 / (41.0f64).to_radians().tan();
    close(removed(r#","kind":"csink","cskDiameter":16,"cskAngle":82"#), frustum(h, 8.0, R) + PI * R * R * (10.0 - h));
    // blind with drill point: cone + blind cylinder + tip cone
    let tip = PI * R * R * (R / 59.0f64.to_radians().tan()) / 3.0;
    close(removed(r#","kind":"csink","cskDiameter":16,"end":"blind","depth":6,"tip":true"#), frustum(3.5, 8.0, R) + PI * R * R * 2.5 + tip);
    let (_, _, ids) = hole(r#","kind":"csink","cskDiameter":16"#);
    for n in ["F4.S.K0", "F4.S.h1"] {
        assert!(ids.iter().any(|i| i == n), "{n} in {ids:?}");
    }
}

#[test]
fn countersink_table_default() {
    // M8 ISO 10642 head 16.4, 90 degrees: cone depth 3.7
    close(removed(r#","kind":"countersink""#), frustum(3.7, 8.2, R) + PI * R * R * 6.3);
}

#[test]
fn csk_errors() {
    let code = |x: &str| hole(x).0.into_iter().next().unwrap_or_default();
    assert!(code(r#","kind":"csink","cskDiameter":9"#).starts_with("F4:holeCskDiameter"));
    assert!(code(r#","kind":"countersink","cskDiameter":5"#).starts_with("F4:holeCskDiameter"));
    assert!(code(r#","kind":"countersink","cskAngle":0"#).starts_with("F4:holeCskDiameter"));
    assert!(code(r#","kind":"countersink","cskAngle":180"#).starts_with("F4:holeCskDiameter"));
    // cone deeper than a blind hole
    assert!(code(r#","kind":"csink","cskDiameter":16,"end":"blind","depth":3"#).starts_with("F4:holeCskDiameter"));
    assert!(code(r#","kind":"csink","cskDiameter":16"#).is_empty());
    assert!(code(r#","kind":"csink","cskDiameter":9"#).contains("hint"));
}
