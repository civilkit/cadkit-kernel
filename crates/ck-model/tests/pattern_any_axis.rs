//! Patterns about / along any axis: a circular pattern about a free axis (point + direction), a feature axis and a
//! cylindrical face; a linear pattern along a free vector. Hole centres are closed form.
use ck_geom::surface::Surface;
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
    fn face_surf(&mut self, s: &regen::RegenState, face: &str) -> Option<regen::FaceSurf> {
        self.0.face_surf(s, face)
    }
    fn edge_line(&mut self, s: &regen::RegenState, faces: &[String], at: Option<[f64; 3]>) -> Option<(ck_core::linalg::Vec3, ck_core::linalg::Vec3)> {
        self.0.edge_line(s, faces, at)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

/// 100 x 100 x 10 plate (z 0..10), one through hole of diameter 4 at (hx, hy), then `tail`.
fn plate_at(hx: f64, hy: f64, tail: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[
     {{"id":"p1","x":0,"y":0}},{{"id":"p2","x":100,"y":0}},{{"id":"p3","x":100,"y":100}},{{"id":"p4","x":0,"y":100}}],
   "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
     {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10}},
  {{"type":"plane","id":"P3","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":10,"name":"top"}},
  {{"type":"sketch","id":"S4","plane":{{"kind":"feature","id":"P3"}},"points":[{{"id":"p1","x":{hx},"y":{hy}}}],
   "entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}},
  {{"type":"hole","id":"H5","sketch":"S4","kind":"simple","diameter":4,"end":"through"}}{tail}
 ]}}"#
    )
}

fn round(x: f64) -> f64 {
    let r = (x * 1e6).round() / 1e6;
    if r == 0.0 { 0.0 } else { r }
}

/// Regenerate: (errors, hole centres (x, y) sorted).
fn holes(src: &str) -> (Vec<String>, Vec<(f64, f64)>) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let body = ex.0.body.clone().expect("body");
    let mut v: Vec<(f64, f64)> = body
        .face_ids()
        .filter_map(|f| match &body.face(f).surface {
            Surface::Cylinder { frame, r } if (*r - 2.0).abs() < 1e-9 => Some((round(frame.o.x), round(frame.o.y))),
            _ => None,
        })
        .collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v.dedup();
    (errs, v)
}

fn same(a: &[(f64, f64)], b: &[(f64, f64)]) {
    assert_eq!(a.len(), b.len(), "{a:?} vs {b:?}");
    for (p, q) in a.iter().zip(b) {
        assert!((p.0 - q.0).abs() < 1e-6 && (p.1 - q.1).abs() < 1e-6, "{a:?} vs {b:?}");
    }
}

#[test]
fn cpattern_about_a_point_and_direction() {
    // hole at (70, 50), axis through (50, 50) along z, 4 copies of 90 degrees
    let tail = r#",{"type":"cpattern","id":"C6","seeds":["H5"],"axis":{"kind":"axis","point":[50,50,0],"dir":[0,0,1]},"count":4,"angle":90,"equal":false}"#;
    let (errs, h) = holes(&plate_at(70.0, 50.0, tail));
    assert!(errs.is_empty(), "{errs:?}");
    same(&h, &[(30.0, 50.0), (50.0, 30.0), (50.0, 70.0), (70.0, 50.0)]);
}

#[test]
fn cpattern_about_a_skew_direction_matches_a_world_axis_rotation() {
    // dir [0,0,-2] is the same axis reversed (not normalised): the copies go the other way round
    let tail = r#",{"type":"cpattern","id":"C6","seeds":["H5"],"axis":{"kind":"axis","point":[50,50,3],"dir":[0,0,-2]},"count":2,"angle":90}"#;
    let (errs, h) = holes(&plate_at(70.0, 50.0, tail));
    assert!(errs.is_empty(), "{errs:?}");
    same(&h, &[(50.0, 30.0), (70.0, 50.0)]);
}

#[test]
fn cpattern_about_an_axis_feature() {
    // the axis feature is the line where x = 50 meets y = 50 (two offset planes)
    let tail = r#",{"type":"plane","id":"P6","mode":"offset","base":{"kind":"std","id":"right"},"offset":50},
        {"type":"plane","id":"P7","mode":"offset","base":{"kind":"std","id":"top"},"offset":50},
        {"type":"axis","id":"A8","refs":[{"kind":"feature","id":"P6"},{"kind":"feature","id":"P7"}]},
        {"type":"cpattern","id":"C9","seeds":["H5"],"axis":{"kind":"feature","id":"A8"},"count":4,"angle":360,"equal":true}"#;
    let (errs, h) = holes(&plate_at(70.0, 50.0, tail));
    assert!(errs.is_empty(), "{errs:?}");
    same(&h, &[(30.0, 50.0), (50.0, 30.0), (50.0, 70.0), (70.0, 50.0)]);
}

#[test]
fn cpattern_about_the_axis_of_a_cylindrical_face() {
    // a 30 mm hole centred at (50, 50), the 4 mm hole at (70, 50) patterned about its wall's axis
    let tail = r#",{"type":"sketch","id":"S6","plane":{"kind":"feature","id":"P3"},"points":[{"id":"c","x":50,"y":50}],
          "entities":[{"type":"circle","id":"e1","c":"c","r":15}],"constraints":[]},
        {"type":"extrude","id":"E7","op":"cut","sketch":"S6","end":"through"},
        {"type":"cpattern","id":"C8","seeds":["H5"],"axis":{"kind":"face","id":"E7.S.e1"},"count":3,"angle":120,"equal":false}"#;
    let (errs, h) = holes(&plate_at(70.0, 50.0, tail));
    // the 4 mm hole at (70, 50) overlaps the big hole's edge region only if it is within 15: it is at 20 -> separate
    assert!(errs.is_empty(), "{errs:?}");
    let c = |a: f64| (round(50.0 + 20.0 * a.to_radians().cos()), round(50.0 + 20.0 * a.to_radians().sin()));
    let mut want = vec![c(0.0), c(120.0), c(240.0)];
    want.sort_by(|a, b| a.partial_cmp(b).unwrap());
    same(&h, &want);
}

#[test]
fn lpattern_along_a_free_vector() {
    // (3, 4) is 5 long: spacing 10 along the unit vector steps (6, 8)
    let tail = r#",{"type":"lpattern","id":"L6","seeds":["H5"],"dir":{"kind":"vector","dir":[3,4,0]},"count":3,"spacing":10}"#;
    let (errs, h) = holes(&plate_at(10.0, 10.0, tail));
    assert!(errs.is_empty(), "{errs:?}");
    same(&h, &[(10.0, 10.0), (16.0, 18.0), (22.0, 26.0)]);
}

#[test]
fn lpattern_along_the_axis_feature_and_reversed() {
    let tail = r#",{"type":"axis","id":"A8","refs":[{"kind":"vector","dir":[1,1,0]}]},
        {"type":"lpattern","id":"L9","seeds":["H5"],"dir":{"kind":"feature","id":"A8"},"count":2,"spacing":14.142135623730951,"reverse":true}"#;
    let (errs, h) = holes(&plate_at(30.0, 30.0, tail));
    assert!(errs.is_empty(), "{errs:?}");
    same(&h, &[(20.0, 20.0), (30.0, 30.0)]);
}

#[test]
fn world_axes_unchanged_and_bad_references_still_error() {
    let ok = r#",{"type":"lpattern","id":"L6","seeds":["H5"],"dir":{"kind":"axis","axis":"x"},"count":2,"spacing":15}"#;
    let (errs, h) = holes(&plate_at(10.0, 10.0, ok));
    assert!(errs.is_empty());
    same(&h, &[(10.0, 10.0), (25.0, 10.0)]);
    for bad in [r#"{"kind":"axis","dir":[0,0,0]}"#, r#"{"kind":"vector"}"#, r#"{"kind":"axis","axis":"w"}"#] {
        let tail = format!(r#",{{"type":"cpattern","id":"C6","seeds":["H5"],"axis":{bad},"count":3,"angle":90}}"#);
        let (errs, _) = holes(&plate_at(10.0, 10.0, &tail));
        assert!(errs.iter().any(|e| e.contains("missingReference")), "{bad}: {errs:?}");
    }
}
