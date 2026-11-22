//! A 40 x 30 x 10 box whose bottom ring of edges is rounded (mitred corners: the vertical edges stay sharp), shelled open at
//! the top. Two equal rounds meeting at a corner with the third edge sharp intersect in an ellipse; the shell must offset it.
//! Volumes are the closed form (cross-section integral over the mitre regions): the shelled body is the rounded box minus the
//! cavity, the cavity being the box moved in by the wall with the round radius reduced by the wall (a sharp inner corner when
//! the wall is at least the radius).
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

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":30},{"id":"p4","x":0,"y":30}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// Bottom face F2.C0 (y = 0) against the four side faces.
const BOTTOM_RING: &str = r#"{"faces":["F2.C0","F2.S.e1"],"at":[20,0,0]},{"faces":["F2.C0","F2.S.e2"],"at":[40,0,-15]},{"faces":["F2.C0","F2.S.e3"],"at":[20,0,-30]},{"faces":["F2.C0","F2.S.e4"],"at":[0,0,-15]}"#;

/// (errors, volume, check issues, lumps)
fn run(r: f64, t: f64) -> (Vec<String>, f64, usize, usize) {
    let fillet = format!(r#"{{"type":"fillet","id":"F3","radius":{r},"edges":[{BOTTOM_RING}]}}"#);
    let shell = format!(r#"{{"type":"shell","id":"F4","faces":["F2.C1"],"thickness":{t}}}"#);
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{BOX},{fillet},{shell}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = res.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect();
    let Some(b) = ex.0.body.as_ref() else { return (errs, 0.0, 0, 0) };
    (errs, ck_topo::mass::mass_props(b).volume, ck_topo::check::check(b, 1e-6).len(), b.lump_ids().count())
}

fn check(r: f64, t: f64, want: f64) {
    let (errs, vol, issues, lumps) = run(r, t);
    assert!(errs.is_empty(), "r {r} t {t}: {errs:?}");
    assert_eq!(issues, 0, "r {r} t {t}");
    assert_eq!(lumps, 1, "r {r} t {t}");
    assert!((vol - want).abs() < 1e-6 * want, "r {r} t {t}: volume {vol} want {want}");
}

#[test]
fn ring_round_r1_5_shells() {
    check(1.5, 0.5, 1217.997039);
    check(1.5, 1.0, 2364.728596);
    check(1.5, 1.4, 3232.146695);
}

#[test]
fn ring_round_r3_shells() {
    check(3.0, 1.0, 2274.197601);
    check(3.0, 2.0, 4278.182829);
}

/// Other radii and walls, down to a wall just under the radius (the inner round nearly vanishes).
#[test]
fn ring_round_other_walls() {
    check(2.0, 0.7, 1663.964117);
    check(2.0, 1.0, 2334.834783);
    check(2.5, 1.9, 4145.410089);
    check(1.5, 0.2, 495.878674);
    check(3.0, 0.1, 240.035317);
    check(3.0, 2.9, 5863.961954);
    check(4.0, 3.5, 6616.295609);
    check(1.5, 1.49, 3421.311215);
}

/// A wall at least the radius: the inward offset of the round vanishes and the cavity has a sharp inner corner (the
/// solidcore 316 case is R1.5 with wall 2: 4445.69). The cavity keeps a round of 1e-3 mm there, far under the tolerance.
#[test]
fn ring_round_wall_at_least_the_radius() {
    check(1.5, 1.5, 3442.194671);
    check(1.5, 2.0, 4445.694671);
    check(3.0, 3.0, 6027.955683);
    check(3.0, 4.0, 7515.955683);
}

/// The vertical edges rounded instead (solidcore ladder 09-006): a 60 x 40 x 20 box with R1 on the four vertical edges,
/// shelled 2 mm open at the top; the inward offset of the rounds vanishes, so the cavity is a sharp 56 x 36 x 18 box.
#[test]
fn vertical_rounds_thinner_than_the_wall() {
    const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":60,"y":0},{"id":"p3","x":60,"y":40},{"id":"p4","x":0,"y":40}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}"#;
    const VERT: &str = r#"{"faces":["F2.S.e1","F2.S.e2"],"at":[60,10,0]},{"faces":["F2.S.e2","F2.S.e3"],"at":[60,10,-40]},{"faces":["F2.S.e3","F2.S.e4"],"at":[0,10,-40]},{"faces":["F2.S.e4","F2.S.e1"],"at":[0,10,0]}"#;
    let text = format!(
        r#"{{"format":"ck-part","version":1,"features":[{BOX},{{"type":"fillet","id":"F3","radius":1,"edges":[{VERT}]}},{{"type":"shell","id":"F4","faces":["F2.C1"],"thickness":2}}]}}"#
    );
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let b = ex.0.body.as_ref().unwrap();
    assert_eq!(ck_topo::check::check(b, 1e-6).len(), 0);
    let want = 11694.831853;
    let vol = ck_topo::mass::mass_props(b).volume;
    assert!((vol - want).abs() < 1e-6 * want, "volume {vol} want {want}");
}
