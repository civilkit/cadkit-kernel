//! A sketch point with a non-finite coordinate (NaN, as a kit script yields for
//! `chord(r) = 24 - 10*((r-R0)/(R-R0))**1.5` at `r < R0`) is refused as
//! `invalidValue {point, detail, hint}` naming the point. A loop of NaN points
//! finds no region, which used to surface as a misleading `noClosedProfile`
//! (propeller probe task 2: the kit propeller script failed its root section at
//! r=8 and worked at r=9).
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
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

fn run(doc: &Json) -> Vec<(String, String, Vec<(String, Json)>)> {
    let part = Part::from_json(doc).unwrap();
    let mut ex = Exec(Brep::new());
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone(), e.error.params.clone())).collect()
}

/// Sets the `axis` coordinate of sketch point `id` to NaN (JSON text cannot
/// carry NaN; the kit delivers the value in memory).
fn nan_axis(j: &mut Json, id: &str, axis: &str) -> bool {
    match j {
        Json::Arr(a) => a.iter_mut().any(|v| nan_axis(v, id, axis)),
        Json::Obj(m) => {
            let mine = m.iter().any(|(k, v)| k == "id" && v.as_str() == Some(id));
            for (k, v) in m.iter_mut() {
                if mine && k == axis && matches!(v, Json::Num(_)) {
                    *v = Json::Num(f64::NAN);
                    return true;
                }
                if nan_axis(v, id, axis) {
                    return true;
                }
            }
            false
        }
        _ => false,
    }
}

/// A closed 10×10 square whose point `a` sits at `[ax, 0]`.
fn square(sk: &str, ax: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{sk}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":{ax},"y":0}},{{"id":"b","x":10,"y":0}},{{"id":"c","x":10,"y":10}},{{"id":"d","x":0,"y":10}}],
"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}}"#
    )
}

/// Moves the sketch `sk` onto the reference plane `plane`.
fn on_plane(doc: &mut Json, sk: &str, plane: &str) {
    match doc {
        Json::Arr(a) => a.iter_mut().for_each(|v| on_plane(v, sk, plane)),
        Json::Obj(m) => {
            let is = m.iter().any(|(k, v)| k == "type" && v.as_str() == Some("sketch") && m.iter().any(|(n, w)| n == "id" && w.as_str() == Some(sk)));
            if is {
                if let Some(p) = m.iter_mut().find(|(n, _)| n == "plane") {
                    p.1 = ck_json::parse(plane).unwrap();
                }
            }
            m.iter_mut().for_each(|(_, v)| on_plane(v, sk, plane));
        }
        _ => {}
    }
}

fn extrude_doc(ax: f64) -> Json {
    ck_json::parse(&format!(
        r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":5}}]}}"#,
        square("F1", ax)
    ))
    .unwrap()
}

fn loft_doc(ax: f64) -> Json {
    let mut d = ck_json::parse(&format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"plane","id":"P0","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":0}},{},
{{"type":"plane","id":"P1","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":20}},{},
{{"type":"loft","id":"F3","op":"add","sketches":["S1","S2"],"ends":{{"first":"cap","last":"cap"}}}}]}}"#,
        square("S1", ax),
        square("S2", ax)
    ))
    .unwrap();
    on_plane(&mut d, "S1", r#"{"kind":"feature","id":"P0"}"#);
    on_plane(&mut d, "S2", r#"{"kind":"feature","id":"P1"}"#);
    d
}

#[test]
fn nan_point_extrude_names_the_point() {
    let mut doc = extrude_doc(0.0);
    assert!(nan_axis(&mut doc, "a", "x"));
    let errs = run(&doc);
    assert_eq!(errs.len(), 1, "{errs:?}");
    let (feat, code, params) = &errs[0];
    assert_eq!((feat.as_str(), code.as_str()), ("F2", "invalidValue"));
    let get = |k: &str| params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
    assert_eq!(get("point"), Some(Json::Str("a".into())));
    let detail = get("detail").and_then(|d| d.as_str().map(str::to_string)).expect("detail");
    assert!(detail.contains("point a") && detail.contains("NaN"), "{detail}");
    assert!(get("hint").is_some(), "a refusal carries a hint");
}

#[test]
fn finite_square_extrudes() {
    assert!(run(&extrude_doc(0.0)).is_empty());
}

#[test]
fn nan_point_loft_is_invalid_value_not_no_closed_profile() {
    let mut doc = loft_doc(0.0);
    assert!(nan_axis(&mut doc, "a", "x"));
    let errs = run(&doc);
    assert_eq!(errs.len(), 1, "{errs:?}");
    let (feat, code, params) = &errs[0];
    assert_eq!((feat.as_str(), code.as_str()), ("F3", "invalidValue"), "a NaN section is not noClosedProfile");
    assert!(params.iter().any(|(n, _)| n == "point"));
}

#[test]
fn finite_loft_builds() {
    assert!(run(&loft_doc(0.0)).is_empty());
}
