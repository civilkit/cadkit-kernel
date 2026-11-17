//! A round too small to change the part (solidcraft `a_round_too_small_to_change_the_part_is_refused`, Apache-2.0 for solidcore/, a crease that
//! turns 5 degrees): the refusal is about a size that is too SMALL, so it names a bigger radius that builds (`minSize`)
//! and never "the largest radius that builds" (which would send the user the wrong way: building is not monotone in the
//! radius, a narrow band of small sizes can build between sizes that fail).
//! Plate profile (0,0) (10,0) (20,0.875) (20,10) (0,10), 5 thick; the crease is the edge x = 10 of the two lowest side faces.
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

const PLATE: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":20,"y":0.875},{"id":"p4","x":20,"y":10},{"id":"p5","x":0,"y":10}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p5"},{"type":"line","id":"e5","p1":"p5","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":5}"#;

/// (error code, error params as JSON text, volume) of the plate with its crease rounded `r`.
fn round(r: f64) -> (Vec<(String, String)>, f64) {
    let f = format!(r#"{{"type":"fillet","id":"F3","radius":{r},"edges":[{{"faces":["F2.S.e1","F2.S.e2"],"at":[10,0,2.5]}}]}}"#);
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{PLATE},{f}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| (e.error.code.clone(), ck_json::to_string(&ck_json::Json::Obj(e.error.params.clone())))).collect();
    (errs, ex.0.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume))
}

fn min_size(params: &str) -> f64 {
    let j = ck_json::parse(params).unwrap();
    j.get("minSize").and_then(|m| m.get("value")).and_then(|v| v.as_f64()).unwrap_or(f64::NAN)
}

#[test]
fn a_round_too_small_to_change_the_part_names_a_size_that_builds() {
    // control: a round that is large enough builds and takes material off
    let (errs, v) = round(0.5);
    assert!(errs.is_empty(), "{errs:?}");
    // the plate is 5 x 195.625 = 978.125; a 5 degree crease rounded 0.5 takes a sliver off
    assert!(v < 978.125 && v > 978.125 - 1.0, "{v}");
    // radii that fail: named as too small, with a bigger size that builds (building it is the proof)
    for r in [0.001, 0.05] {
        let (errs, _) = round(r);
        assert_eq!(errs.len(), 1, "r {r}: {errs:?}");
        let (code, params) = &errs[0];
        assert_eq!(code, "blendFailed", "r {r}: {params}");
        assert!(!params.contains("largest"), "r {r}: {params}");
        assert!(!params.contains("maxSize"), "r {r}: {params}");
        let m = min_size(params);
        assert!(m > r, "r {r}: minSize {m} in {params}");
        let (errs, v) = round(m);
        assert!(errs.is_empty(), "r {r}: minSize {m} does not build: {errs:?}");
        assert!(v > 0.0);
    }
}
