//! Cases ported from the solidcraft kernel tests (tangency-labs/solidcraft, Apache-2.0 for solidcore/; see THIRD_PARTY.md), written as
//! feature trees. The volumes are the closed forms their tests give or integrals of the drafted profile.

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

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
}
impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn volume(&self) -> f64 {
        self.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume)
    }
    fn lumps(&self) -> usize {
        self.body.as_ref().map_or(0, |b| b.num_lumps())
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone() }
}

fn integrate(f: impl Fn(f64) -> f64, h: f64) -> f64 {
    let n = 20000;
    (0..n).map(|i| f((i as f64 + 0.5) * h / n as f64) * h / n as f64).sum()
}

// ---- a lump with a sealed void (solidcraft #3175, kernel case booleans/3175)

/// A 20 mm cube named `A` with a sealed r3 ball void at its centre, and a 10 mm cube `T` whose corner overlaps it at
/// (15..20)^3.
const VOID_LUMP: &str = r#"{"type":"plane","id":"b110P","mode":"frame","origin":[0,0,0],"xDir":[1,0,0],"yDir":[0,1,0]},{"type":"sketch","id":"b110S","plane":{"kind":"feature","id":"b110P"},"points":[{"id":"p0","x":0,"y":0},{"id":"p1","x":20,"y":0},{"id":"p2","x":20,"y":20},{"id":"p3","x":0,"y":20}],"entities":[{"type":"line","id":"e0","p1":"p0","p2":"p1"},{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p0"}],"constraints":[]},{"type":"extrude","id":"b110E","sketch":"b110S","end":"blind","depth":20,"op":"newBody","body":"A"},{"type":"sphere","id":"sv","op":"cut","center":[10,10,10],"radius":3},{"type":"plane","id":"b111P","mode":"frame","origin":[0,0,15],"xDir":[1,0,0],"yDir":[0,1,0]},{"type":"sketch","id":"b111S","plane":{"kind":"feature","id":"b111P"},"points":[{"id":"p0","x":15,"y":15},{"id":"p1","x":25,"y":15},{"id":"p2","x":25,"y":25},{"id":"p3","x":15,"y":25}],"entities":[{"type":"line","id":"e0","p1":"p0","p2":"p1"},{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p0"}],"constraints":[]},{"type":"extrude","id":"b111E","sketch":"b111S","end":"blind","depth":10,"op":"newBody","body":"T"}"#;

fn void_lump(op: &str) -> Run {
    run(&format!(r#"{VOID_LUMP},{{"type":"combine","id":"c","op":"{op}","target":{{"body":"A"}},"tools":[{{"body":"T"}}],"body":"R"}}"#))
}

/// A `{body}` selector on a lump that holds a void read the lump twice (the void's shell is a component of its own):
/// `ambiguousBody`.
#[test]
fn a_lump_with_a_sealed_void_is_one_combine_target() {
    let ball = 4.0 / 3.0 * PI * 27.0;
    for (op, want) in [("cut", 8000.0 - ball - 125.0), ("union", 8000.0 - ball + 1000.0 - 125.0), ("intersect", 125.0)] {
        let r = void_lump(op);
        assert!(r.errors().is_empty(), "{op}: {:?}", r.errors());
        assert!((r.volume() - want).abs() < 1e-6, "{op}: {} vs {want}", r.volume());
        assert_eq!(r.lumps(), 1, "{op}");
        let b = r.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "{op}");
    }
}

// ---- a taper that folds the cap over itself (kernel-robustness brief, section A)

/// Two 10 x 10 blocks joined by a 4 x 1 web, drafted inward by 5 degrees: the web's two long sides meet at
/// 0.5 / tan 5 = 5.71 mm.
const DUMB_BASE: &str = r#"{"type":"plane","id":"tP","mode":"frame","origin":[0,0,0],"xDir":[1,0,0],"yDir":[0,1,0]},{"type":"sketch","id":"tS","plane":{"kind":"feature","id":"tP"},"points":[{"id":"dp0","x":0,"y":0},{"id":"dp1","x":10,"y":0},{"id":"dp2","x":10,"y":4.5},{"id":"dp3","x":14,"y":4.5},{"id":"dp4","x":14,"y":0},{"id":"dp5","x":24,"y":0},{"id":"dp6","x":24,"y":10},{"id":"dp7","x":14,"y":10},{"id":"dp8","x":14,"y":5.5},{"id":"dp9","x":10,"y":5.5},{"id":"dp10","x":10,"y":10},{"id":"dp11","x":0,"y":10}],"entities":[{"type":"line","id":"de0","p1":"dp0","p2":"dp1"},{"type":"line","id":"de1","p1":"dp1","p2":"dp2"},{"type":"line","id":"de2","p1":"dp2","p2":"dp3"},{"type":"line","id":"de3","p1":"dp3","p2":"dp4"},{"type":"line","id":"de4","p1":"dp4","p2":"dp5"},{"type":"line","id":"de5","p1":"dp5","p2":"dp6"},{"type":"line","id":"de6","p1":"dp6","p2":"dp7"},{"type":"line","id":"de7","p1":"dp7","p2":"dp8"},{"type":"line","id":"de8","p1":"dp8","p2":"dp9"},{"type":"line","id":"de9","p1":"dp9","p2":"dp10"},{"type":"line","id":"de10","p1":"dp10","p2":"dp11"},{"type":"line","id":"de11","p1":"dp11","p2":"dp0"}],"constraints":[]}"#;

fn dumbbell(depth: f64) -> Run {
    run(&format!(r#"{DUMB_BASE},{{"type":"extrude","id":"tE","sketch":"tS","end":"blind","depth":{depth},"op":"newBody","body":"T","draft":5}}"#))
}

fn dumbbell_volume(h: f64) -> f64 {
    let t5 = (5.0_f64).to_radians().tan();
    integrate(
        |y| {
            let t = y * t5;
            2.0 * (10.0 - 2.0 * t).max(0.0).powi(2) + if t < 0.5 { (4.0 + 2.0 * t) * (1.0 - 2.0 * t) } else { 0.0 }
        },
        h,
    )
}

#[test]
fn a_taper_that_stops_short_of_closing_a_thin_web_builds() {
    for h in [2.0, 4.0, 5.5] {
        let r = dumbbell(h);
        assert!(r.errors().is_empty(), "{h}: {:?}", r.errors());
        assert!((r.volume() - dumbbell_volume(h)).abs() < 1e-6 * dumbbell_volume(h), "{h}: {} vs {}", r.volume(), dumbbell_volume(h));
    }
}

/// Past 5.71 mm the web's sides have crossed. The cap used to fold over itself: the body was built with the web as a
/// negative sliver, passed the validity check and had the wrong volume (1396.46 against 1398.86 at 8 mm), with no
/// error. It is refused now, as the other tapers that cannot be built are.
#[test]
fn a_taper_that_carries_a_thin_webs_sides_through_each_other_is_refused() {
    for h in [6.0, 8.0] {
        let r = dumbbell(h);
        assert_eq!(r.errors(), vec!["tE:draftTooLarge"], "{h}: volume {}", r.volume());
    }
}

/// A 20 x 20 plate with a 10 x 1 slot, drafted outward by 5 degrees: the slot closes at 5.71 mm.
const SLOT_BASE: &str = r#"{"type":"plane","id":"tP","mode":"frame","origin":[0,0,0],"xDir":[1,0,0],"yDir":[0,1,0]},{"type":"sketch","id":"tS","plane":{"kind":"feature","id":"tP"},"points":[{"id":"op0","x":0,"y":0},{"id":"op1","x":20,"y":0},{"id":"op2","x":20,"y":20},{"id":"op3","x":0,"y":20},{"id":"ip0","x":5,"y":9.5},{"id":"ip1","x":15,"y":9.5},{"id":"ip2","x":15,"y":10.5},{"id":"ip3","x":5,"y":10.5}],"entities":[{"type":"line","id":"oe0","p1":"op0","p2":"op1"},{"type":"line","id":"oe1","p1":"op1","p2":"op2"},{"type":"line","id":"oe2","p1":"op2","p2":"op3"},{"type":"line","id":"oe3","p1":"op3","p2":"op0"},{"type":"line","id":"ie0","p1":"ip0","p2":"ip1"},{"type":"line","id":"ie1","p1":"ip1","p2":"ip2"},{"type":"line","id":"ie2","p1":"ip2","p2":"ip3"},{"type":"line","id":"ie3","p1":"ip3","p2":"ip0"}],"constraints":[]}"#;

fn slotted(depth: f64) -> Run {
    run(&format!(r#"{SLOT_BASE},{{"type":"extrude","id":"tE","sketch":"tS","end":"blind","depth":{depth},"op":"newBody","body":"T","draft":5,"draftOut":true}}"#))
}

#[test]
fn a_slot_a_taper_has_not_yet_closed_builds_and_one_it_closes_is_refused() {
    let t5 = (5.0_f64).to_radians().tan();
    let want = |h: f64| integrate(|y| (20.0 + 2.0 * y * t5).powi(2) - (10.0 - 2.0 * y * t5) * (1.0 - 2.0 * y * t5), h);
    let r = slotted(4.0);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!((r.volume() - want(4.0)).abs() < 1e-6 * want(4.0), "{} vs {}", r.volume(), want(4.0));
    // closing partway: MitCad drops the slot there; the kernel refuses (draftTooLarge) rather than build it wrong
    assert_eq!(slotted(8.0).errors(), vec!["tE:draftTooLarge"]);
}

// ---- a tapered extrude To a curved face (solidcraft #15820, engine test `tapered_extrude_to_a_curved_face_ends_on_it`)

const BALL: f64 = 4188.790204786391;

/// A ball R10 centred at z 20 and a circle of radius `r` (with a hole r1.5 when `hole`) on z = 0, extruded To the ball
/// with a taper of `deg` degrees (their sign: positive grows, negative narrows) and joined to it.
fn to_ball(r: f64, deg: f64, hole: bool) -> Run {
    let inner = if hole { r#",{"type":"circle","id":"i","c":"c","r":1.5}"# } else { "" };
    let out = if deg > 0.0 { r#","draftOut":true"# } else { "" };
    run(&format!(
        r#"{{"type":"sphere","id":"sp","op":"newBody","center":[0,0,20],"radius":10,"body":"Ball"}},
{{"type":"plane","id":"tP","mode":"frame","origin":[0,0,0],"xDir":[1,0,0],"yDir":[0,1,0]}},
{{"type":"sketch","id":"tS","plane":{{"kind":"feature","id":"tP"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"o","c":"c","r":{r}}}{inner}],"constraints":[]}},
{{"type":"extrude","id":"tE","sketch":"tS","end":"toObject","endRef":{{"kind":"body","name":"Ball"}},"draft":{},"op":"add","target":{{"body":"Ball"}}{out}}}"#,
        deg.abs()
    ))
}

/// The column's volume to the ball (the frustum to where its wall meets the ball, less the ball's cap below that), from
/// the solidcraft test. The tool is built long and trimmed at the ball, and a narrowing taper closes before the long
/// tool ends (at z 45.7 for r4 at -5 deg, the ball at 10.5): it used to be refused `draftTooLarge`.
#[test]
fn a_narrowing_taper_to_a_curved_face_ends_on_it() {
    for (r, deg, post) in [(4.0, 5.0, 669.1892253335005), (4.0, -5.0, 408.1381918962213), (4.0, -15.0, 241.3566646562573), (12.0, -10.0, 4765.092818474139), (12.0, -20.0, 3612.7398353298513), (12.0, -21.0, 3520.22820600382)] {
        let run = to_ball(r, deg, false);
        assert!(run.errors().is_empty(), "r {r}, {deg} deg: {:?}", run.errors());
        let want = BALL + post;
        assert!((run.volume() - want).abs() < 1e-6 * want, "r {r}, {deg} deg: {} vs {want}", run.volume());
        assert_eq!(run.lumps(), 1, "r {r}, {deg} deg");
    }
}

/// With a hole r1.5 the hole's wall narrows to nothing at z 17.1 when the outside grows (+5 deg), after the ball.
#[test]
fn a_taper_with_a_hole_that_closes_after_the_curved_face_ends_on_it() {
    let run = to_ball(4.0, 5.0, true);
    assert!(run.errors().is_empty(), "{:?}", run.errors());
    let want = BALL + 631.703987706579;
    assert!((run.volume() - want).abs() < 1e-6 * want, "{} vs {want}", run.volume());
}

/// The same with the taper narrowing: the hole grows into the outside wall at z 14.3, the ball is at 10.3. Still
/// refused (`booleanVolumeMismatch` from the trim); the closed form is 285.63157703482165 for the post.
#[test]
#[ignore = "known gap: the shortened tool meets the ball only 4 mm below where its hole breaks into the wall; the trim is refused"]
fn a_narrowing_taper_with_a_hole_to_a_curved_face() {
    let run = to_ball(4.0, -5.0, true);
    assert!(run.errors().is_empty(), "{:?}", run.errors());
    let want = BALL + 285.63157703482165;
    assert!((run.volume() - want).abs() < 1e-6 * want, "{} vs {want}", run.volume());
}
