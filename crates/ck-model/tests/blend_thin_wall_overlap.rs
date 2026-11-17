//! Two rounds in one fillet whose second edge the first round swallows whole (solidcraft kernel case
//! `1668-fillet-both-sides-of-thin-wall-r7`; the tray as STEP in fixtures/solidcraft, see THIRD_PARTY.md): a tray 104 x 64 x 42 with a 100 x 60 pocket 2 deep, the outer top edge (convex) and the pocket's floor edge beside it
//! (concave) picked in that order. Past the wall's height the convex round runs over the wall and takes the concave edge with it.
//! The second pick used to be dropped without a word (one blend face for two picks, 1051 mm3 too much off); now the fillet is
//! refused as overlapping, naming the radius that builds, and that radius builds BOTH rounds (the closed form: the convex
//! round takes (1 - pi/4) r^2 along its length, the concave one gives it back along its own).
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

const TRAY: &str = include_str!("fixtures/solidcraft/1668_thin_wall_tray.step");

/// (errors as `code params`, volume, face count, blend faces) of the tray with `picks` rounded `r`.
fn run(r: f64, picks: &str) -> (Vec<String>, f64, usize, usize) {
    let step = ck_json::to_string(&ck_json::Json::Str(TRAY.to_string()));
    let text = format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"importStep","id":"F1","step":{step},"heal":true}},{{"type":"fillet","id":"F2","radius":{r},"edges":[{picks}]}}]}}"#
    );
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = res.errors.iter().map(|e| format!("{} {}", e.error.code, ck_json::to_string(&ck_json::Json::Obj(e.error.params.clone())))).collect();
    let b = ex.0.body.as_ref().unwrap_or_else(|| panic!("no body: {errs:?}"));
    let blends = b.face_ids().filter(|&f| matches!(b.face(f).name.role, ck_topo::body::Role::Blend(_))).count();
    (errs, ck_topo::mass::mass_props(b).volume, b.num_faces(), blends)
}

const OUTER: &str = r#"{"faces":["F1.I1F1","F1.I1F2"],"at":[0,32,42]}"#;
const INNER: &str = r#"{"faces":["F1.I1F10","F1.I1F11"],"at":[0,30,40]}"#;

#[test]
fn a_round_that_swallows_the_next_pick_is_refused_not_dropped() {
    let a = std::f64::consts::FRAC_PI_4;
    let v0 = 39552.0; // the tray: its outer top edge is 104 long, the pocket's floor edge 100
    // radius 1.9: both rounds build, inside the wall's 2 mm
    let (errs, v, _, blends) = run(1.9, &format!("{OUTER},{INNER}"));
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(blends, 2);
    // outer convex round takes A over its 104, the inner concave one gives A back over its 100
    let a_r = (1.0 - a) * 1.9 * 1.9;
    assert!((v - (v0 - 104.0 * a_r + 100.0 * a_r)).abs() < 1e-6 * v0, "{v}");
    // radius 7: the convex round runs over the wall and takes the concave edge; one blend face for two picks is a wrong part
    let (errs, v, _, blends) = run(7.0, &format!("{OUTER},{INNER}"));
    assert_eq!(errs.len(), 1, "built with {blends} blend face(s), volume {v}");
    assert!(errs[0].starts_with("blendOverflow") && errs[0].contains("blends overlap"), "{}", errs[0]);
    // the radius it names builds both rounds
    let j = ck_json::parse(errs[0].split_once(' ').unwrap().1).unwrap();
    let m = j.get("maxSize").and_then(|m| m.get("value")).and_then(|v| v.as_f64()).expect("maxSize");
    let (errs, v, _, blends) = run(m, &format!("{OUTER},{INNER}"));
    assert!(errs.is_empty() && blends == 2, "{m}: {errs:?} {blends}");
    let a_r = (1.0 - a) * m * m;
    assert!((v - (v0 - 104.0 * a_r + 100.0 * a_r)).abs() < 1e-5 * v0, "{m}: {v}");
}
