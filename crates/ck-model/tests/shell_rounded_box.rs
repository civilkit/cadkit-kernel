//! Shell of a box whose every edge is rounded, opened at the top (solidcore #2532 / #2484). 40 mm cube:
//! x 0..40, y 0..40, z -40..0 (sketch on `top`: x, -z; extrude +y), the open face is the cap F2.C1 at y = 40.
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

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":40},{"id":"p4","x":0,"y":40}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

fn run(feature: &str) -> (Vec<(String, String)>, Option<(f64, usize, Vec<[f64; 3]>)>) {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{BOX},{feature}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<(String, String)> = res.errors.iter().map(|e| (e.error.code.clone(), format!("{:?}", e.error.params))).collect();
    let Some(b) = ex.0.body.as_ref() else { return (errs, None) };
    let issues = ck_topo::check::check(b, 1e-6).len();
    let verts: Vec<[f64; 3]> = b.vertex_ids().map(|v| [b.vertex(v).p.x, b.vertex(v).p.y, b.vertex(v).p.z]).collect();
    (errs, Some((ck_topo::mass::mass_props(b).volume, issues, verts)))
}

fn has_vertex(vs: &[[f64; 3]], p: [f64; 3]) -> bool {
    vs.iter().any(|v| (0..3).all(|i| (v[i] - p[i]).abs() < 1e-6))
}


const VERTICAL: &str = r#"{"faces":["F2.S.e1","F2.S.e2"],"at":[40,20,0]},{"faces":["F2.S.e2","F2.S.e3"],"at":[40,20,-40]},{"faces":["F2.S.e3","F2.S.e4"],"at":[0,20,-40]},{"faces":["F2.S.e4","F2.S.e1"],"at":[0,20,0]}"#;
const TOP_RIM: &str = r#"{"faces":["F2.C1","F2.S.e1"],"at":[20,40,0]},{"faces":["F2.C1","F2.S.e2"],"at":[40,40,-20]},{"faces":["F2.C1","F2.S.e3"],"at":[20,40,-40]},{"faces":["F2.C1","F2.S.e4"],"at":[0,40,-20]}"#;
const BOTTOM_RIM: &str = r#"{"faces":["F2.C0","F2.S.e1"],"at":[20,0,0]},{"faces":["F2.C0","F2.S.e2"],"at":[40,0,-20]},{"faces":["F2.C0","F2.S.e3"],"at":[20,0,-40]},{"faces":["F2.C0","F2.S.e4"],"at":[0,0,-20]}"#;

type Outcome = (Vec<(String, String)>, Option<(f64, usize, Vec<[f64; 3]>)>);

/// Fillet `edges` (radius `r`), then shell the cap F2.C1 (y = 40) with wall `t`.
fn rounded_box_shell(edges: &[&str], r: f64, t: f64) -> Outcome {
    let list = edges.join(",");
    let fillet = format!(r#"{{"type":"fillet","id":"F3","radius":{r},"edges":[{list}]}}"#);
    let shell = format!(r#"{{"type":"shell","id":"F4","faces":["F2.C1"],"thickness":{t}}}"#);
    run(&format!("{fillet},{shell}"))
}

/// Volume of an a x b x h box with every edge rounded at r.
fn rounded(a: f64, b: f64, h: f64, r: f64) -> f64 {
    let (x, y, z) = (a - 2.0 * r, b - 2.0 * r, h - 2.0 * r);
    let pi = std::f64::consts::PI;
    x * y * z + 2.0 * r * (x * y + x * z + y * z) + pi * r * r * (x + y + z) + 4.0 * pi * r * r * r / 3.0
}

/// The opened face is bordered by the round of its own edges. The cavity is the box offset inward (rounds R5 - t) plus the
/// prism of the opening (the 30 x 30 flat of the top) from there up through the face: closed-form volume.
#[test]
fn a_rounded_rim_around_the_opened_face_shells() {
    for t in [2.0, 1.0] {
        let (errs, r) = rounded_box_shell(&[VERTICAL, TOP_RIM, BOTTOM_RIM], 5.0, t);
        assert!(errs.is_empty(), "t {t}: {errs:?}");
        let (vol, issues, _) = r.unwrap();
        assert_eq!(issues, 0, "t {t}");
        let want = rounded(40.0, 40.0, 10.0, 5.0) - rounded(40.0 - 2.0 * t, 40.0 - 2.0 * t, 10.0 - 2.0 * t, 5.0 - t) - 30.0 * 30.0 * t;
        assert!((vol - want).abs() < 1e-4 * want, "t {t}: volume {vol} want {want}");
    }
}

#[test]
fn rounding_everything_but_the_rim_still_shells() {
    let (errs, r) = rounded_box_shell(&[VERTICAL, BOTTOM_RIM], 5.0, 2.0);
    assert!(errs.is_empty(), "{errs:?}");
    let (vol, issues, _) = r.unwrap();
    assert_eq!(issues, 0);
    assert!((vol - 4964.5).abs() < 1.0, "{vol}");
}
