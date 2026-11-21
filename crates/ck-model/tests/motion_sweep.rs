//! Motion studies: finding where a driver stops (`motion::sweep`): mate limit, contact, clearance; exact numbers.

use ck_model::features::mate::Xf;
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

fn boxp(w: f64, d: f64, h: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":{w},"y":0}},{{"id":"p3","x":{w},"y":{d}}},{{"id":"p4","x":0,"y":{d}}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}]}}"#
    )
}
fn cylp(r: f64, h: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}]}}"#
    )
}
fn ins(id: &str, part: &str, extra: &str) -> String {
    format!(r#"{{"type":"insertPart","id":"{id}","name":"{id}","part":{part}{extra}}}"#)
}
fn mate(id: &str, kind: &str, a: &str, b: &str, extra: &str) -> String {
    format!(r#"{{"type":"mate","id":"{id}","kind":"{kind}","a":"{a}","b":"{b}"{extra}}}"#)
}


use ck_core::linalg::Vec3;
use ck_model::features::mate_kinds::{geo, G};


use ck_model::features::motion::{self, StopReason, SweepOpts};

struct Run {
    body: ck_topo::body::Body,
    asm: ck_model::features::mate::Assembly,
}

fn run_text(text: &str) -> Run {
    let part = Part::parse(text).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    Run { body: ex.brep.body.clone().unwrap(), asm: ex.brep.asm.clone() }
}
fn run(features: &[String]) -> Run {
    run_text(&format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(",")))
}
fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}
fn opts() -> SweepOpts {
    SweepOpts { tol: 0.005, ..SweepOpts::default() }
}

#[test]
fn hinge_door_stops_at_its_limit_with_the_mate_named() {
    let r = run_text(include_str!("fixtures/assembly-hinge-door.ckpart"));
    let s = motion::sweep(&r.body, &r.asm, "F3", 0.0, 180.0, &opts()).unwrap();
    match &s.stop {
        StopReason::Limit(h) => {
            assert_eq!(h.mate, "F4");
            assert_eq!(h.bound, "max");
            assert!(near(h.value, 110.0, 1e-9));
        }
        other => panic!("expected the swing limit, got {other:?}"),
    }
    assert!(near(s.reached, 110.0, 0.01), "reached {}", s.reached);
    // inside the limit the sweep reaches the end of its range
    let ok = motion::sweep(&r.body, &r.asm, "F3", 0.0, 100.0, &opts()).unwrap();
    assert!(matches!(ok.stop, StopReason::Range), "{:?}", ok.stop);
    assert_eq!(ok.reached, 100.0);
}

/// The `limits` mate measures the angle between two faces as an unsigned angle (0 to 180), so a door turned the other way reads the same
/// number: it swings backwards as freely as forwards, and stops at the same 110 degrees on that side.
#[test]
fn limits_angle_is_unsigned_so_the_door_also_swings_backwards() {
    let r = run_text(include_str!("fixtures/assembly-hinge-door.ckpart"));
    let s = motion::sweep(&r.body, &r.asm, "F3", 0.0, -90.0, &opts()).unwrap();
    assert!(matches!(s.stop, StopReason::Range), "{:?}", s.stop);
    let s = motion::sweep(&r.body, &r.asm, "F3", 0.0, -150.0, &opts()).unwrap();
    assert!(matches!(&s.stop, StopReason::Limit(h) if h.bound == "max"), "{:?}", s.stop);
    assert!(near(s.reached, -110.0, 0.01), "reached {}", s.reached);
}

/// A rail with a slot (x 25..35, z -25..-5 in the world: a top-plane sketch's y runs along -z; 10 mm deep) and a 4 mm pin through it: the pin travels 20 - 4 = 16 mm in the slot.
fn slot_slider() -> Vec<String> {
    let rail = r#"{"format":"ck-part","version":1,"features":[
{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":60,"y":0},{"id":"p3","x":60,"y":30},{"id":"p4","x":0,"y":30}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
{"type":"sketch","id":"F3","plane":{"kind":"std","id":"top"},"points":[{"id":"q1","x":25,"y":5},{"id":"q2","x":35,"y":5},{"id":"q3","x":35,"y":25},{"id":"q4","x":25,"y":25}],
"entities":[{"type":"line","id":"e1","p1":"q1","p2":"q2"},{"type":"line","id":"e2","p1":"q2","p2":"q3"},{"type":"line","id":"e3","p1":"q3","p2":"q4"},{"type":"line","id":"e4","p1":"q4","p2":"q1"}],"constraints":[]},
{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth","depth":10}]}"#;
    let pin = cylp(2.0, 10.0);
    vec![
        ins("F1", rail, r#","fixed":true"#),
        ins("F2", &pin, r#","dx":30,"dy":0,"dz":-15"#),
        mate("F3", "slot", "F2.S.e1@F2.1", "F4.S.e4@F1.1", r#","b2":"F4.S.e2@F1.1""#),
    ]
}

#[test]
fn slot_slider_travels_slot_length_minus_pin_diameter() {
    let r = run(&slot_slider());
    for (to, sign) in [(30.0, 1.0), (-30.0, -1.0)] {
        let s = motion::sweep(&r.body, &r.asm, "F3", 0.0, to, &opts()).unwrap();
        match &s.stop {
            StopReason::Contact(c) => {
                let pair = [c.a.as_str(), c.b.as_str()];
                assert!(pair.contains(&"F1") && pair.contains(&"F2"), "{pair:?}");
                assert!(c.distance == 0.0);
                // the contact is at a slot end wall (z = -5 or -25), 10 mm from the slot's middle
                assert!(near((c.at[2] + 15.0).abs(), 10.0, 0.05), "contact at {:?}", c.at);
            }
            other => panic!("expected contact, got {other:?}"),
        }
        // 20 mm slot - 4 mm pin = 16 mm: 8 mm each way from the middle
        assert!(near(s.reached, sign * 8.0, 0.02), "reached {} (want {})", s.reached, sign * 8.0);
    }
}

#[test]
fn clearance_shortens_the_travel_by_the_clearance_per_end() {
    let r = run(&slot_slider());
    let mut o = opts();
    o.contact.clearance = 0.5;
    for (to, sign) in [(30.0, 1.0), (-30.0, -1.0)] {
        let s = motion::sweep(&r.body, &r.asm, "F3", 0.0, to, &o).unwrap();
        assert!(matches!(&s.stop, StopReason::Contact(c) if c.distance > 0.0 && c.distance < 0.5), "{:?}", s.stop);
        assert!(near(s.reached, sign * 7.5, 0.02), "reached {} (want {})", s.reached, sign * 7.5);
    }
}

#[test]
fn contact_can_be_turned_off_and_a_range_end_is_reported() {
    let r = run(&slot_slider());
    let mut o = opts();
    o.contact.check = false;
    let s = motion::sweep(&r.body, &r.asm, "F3", 0.0, 12.0, &o).unwrap();
    assert!(matches!(s.stop, StopReason::Range), "{:?}", s.stop);
    assert_eq!(s.reached, 12.0);
    // within the free travel the range ends first even with contact on
    let s = motion::sweep(&r.body, &r.asm, "F3", 0.0, 6.0, &opts()).unwrap();
    assert!(matches!(s.stop, StopReason::Range), "{:?}", s.stop);
}

#[test]
fn sweep_is_deterministic_and_does_not_write_the_assembly() {
    let r = run(&slot_slider());
    let before: Vec<_> = r.asm.components.iter().map(|c| (c.place.r, c.place.t)).collect();
    let a = motion::sweep(&r.body, &r.asm, "F3", 0.0, 30.0, &opts()).unwrap();
    let b = motion::sweep(&r.body, &r.asm, "F3", 0.0, 30.0, &opts()).unwrap();
    assert_eq!(a.reached, b.reached);
    assert_eq!(a.poses_solved, b.poses_solved);
    let after: Vec<_> = r.asm.components.iter().map(|c| (c.place.r, c.place.t)).collect();
    assert_eq!(before, after);
}

#[test]
fn bad_drivers_are_named() {
    let r = run(&slot_slider());
    let e = motion::sweep(&r.body, &r.asm, "Nope", 0.0, 1.0, &opts()).err().unwrap();
    assert_eq!(e, (motion::UNKNOWN_DRIVER.to_string(), "Nope".to_string()));
}

/// The 20:40 gear train sample: drive the 20-tooth gear a full turn; the 40-tooth follower turns half a turn the other way.
#[test]
fn gear_train_turns_the_follower_half_a_turn() {
    let r = run_text(include_str!("fixtures/assembly-gear-train.ckpart"));
    let ps = motion::solve_poses(&r.body, &r.asm, &[vec![("F6".to_string(), 90.0)], vec![("F6".to_string(), 360.0)]]).unwrap();
    assert!(ps.iter().all(|p| p.error.is_none()), "{:?}", ps.iter().map(|p| p.error.clone()).collect::<Vec<_>>());
    let ang = |p: &motion::Pose, id: &str| {
        let q = &p.places.iter().find(|(c, _)| c == id).unwrap().1;
        q.r[2].atan2(q.r[0]).to_degrees()
    };
    // the drive angle is the driver gear's (F2), the follower (F3) turns the opposite way at half the angle
    assert!(near(ang(&ps[0], "F2"), 90.0, 1e-6), "driver {}", ang(&ps[0], "F2"));
    assert!(near(ang(&ps[0], "F3"), -45.0, 1e-6), "follower {}", ang(&ps[0], "F3"));
    assert!(near(ang(&ps[1], "F3").abs(), 180.0, 1e-6), "follower {}", ang(&ps[1], "F3"));
    let s = motion::sweep(&r.body, &r.asm, "F6", 0.0, 360.0, &SweepOpts { steps: 36, ..opts() }).unwrap();
    assert!(matches!(s.stop, StopReason::Range), "{:?}", s.stop);
}

// ---------------------------------------------------------------- slider-crank (closed loop)

fn rect_part(x0: f64, x1: f64, z0: f64, z1: f64, depth: f64, holes: &[(f64, f64)], hole_r: f64) -> String {
    let mut f = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":{z0}}},{{"id":"p2","x":{x1},"y":{z0}}},{{"id":"p3","x":{x1},"y":{z1}}},{{"id":"p4","x":{x0},"y":{z1}}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth}}}"#
    );
    for (k, (hx, hy)) in holes.iter().enumerate() {
        let (sk, ex) = (3 + 2 * k, 4 + 2 * k);
        f += &format!(
            r#",{{"type":"sketch","id":"F{sk}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":{hx},"y":{hy}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{hole_r}}}],"constraints":[]}},
{{"type":"extrude","id":"F{ex}","op":"cut","sketch":"F{sk}","end":"throughBoth","depth":{depth}}}"#
        );
    }
    format!(r#"{{"format":"ck-part","version":1,"features":[{f}]}}"#)
}

/// Base plate with a pivot pin at the origin and a slot along x (x 8..95, 6 wide); crank (r = 20), rod (50 between holes), slider pin.
fn slider_crank(rod: f64) -> Vec<String> {
    let base = format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":-30,"y":-20}},{{"id":"p2","x":110,"y":-20}},{{"id":"p3","x":110,"y":20}},{{"id":"p4","x":-30,"y":20}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":5}},
{{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":3}}],"constraints":[]}},
{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":10}},
{{"type":"sketch","id":"F5","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"q1","x":8,"y":-3}},{{"id":"q2","x":95,"y":-3}},{{"id":"q3","x":95,"y":3}},{{"id":"q4","x":8,"y":3}}],
"entities":[{{"type":"line","id":"e1","p1":"q1","p2":"q2"}},{{"type":"line","id":"e2","p1":"q2","p2":"q3"}},{{"type":"line","id":"e3","p1":"q3","p2":"q4"}},{{"type":"line","id":"e4","p1":"q4","p2":"q1"}}],"constraints":[]}},
{{"type":"extrude","id":"F6","op":"cut","sketch":"F5","end":"throughBoth","depth":5}}]}}"#
    );
    let crank = rect_part(-5.0, 25.0, -5.0, 5.0, 5.0, &[(0.0, 0.0), (20.0, 0.0)], 3.2);
    let rodp = rect_part(-5.0, rod + 5.0, -5.0, 5.0, 5.0, &[(0.0, 0.0), (rod, 0.0)], 2.2);
    let slider = cylp(2.0, 15.0);
    vec![
        ins("F1", &base, r#","fixed":true"#),
        ins("F2", &crank, r#","dy":5"#),
        ins("F3", &rodp, r#","dx":20,"dy":10"#),
        ins("F4", &slider, &format!(r#","dx":{}"#, 20.0 + rod)),
        mate("F5", "hinge", "F4.S.e1@F2.1", "F4.S.e1@F1.1", r#","a2":"F2.C0@F2.1","b2":"F2.C1@F1.1""#),
        mate("F6", "hinge", "F4.S.e1@F3.1", "F6.S.e1@F2.1", r#","a2":"F2.C0@F3.1","b2":"F2.C1@F2.1""#),
        mate("F7", "hinge", "F6.S.e1@F3.1", "F2.S.e1@F4.1", r#","a2":"F2.C1@F3.1","b2":"F2.C1@F4.1""#),
        mate("F8", "slot", "F2.S.e1@F4.1", "F6.S.e1@F1.1", r#","b2":"F6.S.e3@F1.1""#),
    ]
}

/// Slider-crank, crank radius 20, rod 50: the slider's travel is twice the crank radius (30 to 70), the sweep goes the full turn without a
/// branch jump or a contact, and the slider follows x = r cos(a) + sqrt(L^2 - (r sin a)^2).
#[test]
fn slider_crank_travels_twice_the_crank_radius_and_never_flips() {
    let r = run(&slider_crank(50.0));
    let s = motion::sweep(&r.body, &r.asm, "F5", 0.0, 360.0, &SweepOpts { steps: 72, ..opts() }).unwrap();
    assert!(matches!(s.stop, StopReason::Range), "{:?}", s.stop);
    assert_eq!(s.reached, 360.0);
    // the poses of the sweep's own 5 degree steps, each solved from the one before
    let mut all = vec![];
    for k in 1..=72 {
        all.push(vec![("F5".to_string(), 5.0 * k as f64)]);
    }
    let ps = motion::solve_poses(&r.body, &r.asm, &all).unwrap();
    assert!(ps.iter().all(|p| p.error.is_none()));
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for (k, p) in (1..=72).zip(&ps) {
        let a = (5.0 * k as f64).to_radians();
        let want = 20.0 * a.cos() + (2500.0 - (20.0 * a.sin()).powi(2)).sqrt();
        let got = p.places[3].1.t[0];
        assert!(near(got, want, 1e-5), "angle {} slider {got} want {want}", 5 * k);
        // the slider stays on the rail line, at its height
        assert!(p.places[3].1.t[2].abs() < 1e-5 && p.places[3].1.t[1].abs() < 1e-5, "{:?}", p.places[3].1.t);
        lo = lo.min(got);
        hi = hi.max(got);
    }
    assert!(near(hi, 70.0, 1e-5) && near(lo, 30.0, 1e-5), "travel {lo}..{hi}");
}

/// A rod shorter than the crank radius cannot follow the crank round: the sweep stops with the solver where the rod runs out
/// (sin a = 15/20, a = 48.59 degrees).
#[test]
fn a_rod_that_is_too_short_stops_the_solver_at_the_right_angle() {
    let r = run(&slider_crank(15.0));
    let s = motion::sweep(&r.body, &r.asm, "F5", 0.0, 180.0, &SweepOpts { steps: 30, tol: 0.02, ..opts() }).unwrap();
    assert!(matches!(&s.stop, StopReason::Solver { code } if code == "mateConflict"), "{:?}", s.stop);
    let limit = (15.0f64 / 20.0).asin().to_degrees();
    // (a loop solved by relaxation converges slowly near its singular pose and gives up a fraction of a degree early)
    assert!(near(s.reached, limit, 1.0) && s.reached <= limit, "stopped at {} want {limit}", s.reached);
}

/// A pose that carries the pin right through the slot end and into the rail's material has no surface crossing, only a part wholly
/// inside another: it is still a contact.
#[test]
fn a_part_pushed_wholly_inside_another_is_a_contact() {
    let r = run(&slot_slider());
    let ps = motion::solve_poses_with(&r.body, &r.asm, &[vec![("F3".to_string(), 5.0)], vec![("F3".to_string(), 12.0)]], &motion::ContactOpts::default()).unwrap();
    assert!(ps[0].contact.is_none());
    let c = ps[1].contact.as_ref().expect("12 mm is past the slot end (8 mm)");
    assert!(c.a == "F1" && c.b == "F2" || c.a == "F2" && c.b == "F1");
}
