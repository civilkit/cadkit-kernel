//! Motion studies: driving mates through poses (`motion::solve_poses`), exact numbers.

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
use ck_model::features::motion;

struct Run {
    body: ck_topo::body::Body,
    asm: ck_model::features::mate::Assembly,
}

fn run(features: &[String]) -> Run {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","));
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    Run { body: ex.brep.body.clone().unwrap(), asm: ex.brep.asm.clone() }
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}
fn pose(d: &[(&str, f64)]) -> Vec<(String, f64)> {
    d.iter().map(|(k, v)| (k.to_string(), *v)).collect()
}
/// Rotation of a placement about +y, degrees in (-180, 180].
fn ang_y(p: &Xf) -> f64 {
    p.r[2].atan2(p.r[0]).to_degrees()
}
fn two_cyls() -> Vec<String> {
    let cyl = cylp(5.0, 20.0);
    vec![ins("F1", &cyl, r#","fixed":true"#), ins("F2", &cyl, r#","dx":100,"dy":7,"dz":3"#)]
}
fn hinge_doc() -> Vec<String> {
    let mut f = two_cyls();
    f.push(mate("F3", "hinge", "F2.S.e1@F2.1", "F2.S.e1@F1.1", r#","a2":"F2.C0@F2.1","b2":"F2.C1@F1.1""#));
    f
}
fn place_of<'a>(p: &'a motion::Pose, id: &str) -> &'a Xf {
    &p.places.iter().find(|(c, _)| c == id).unwrap().1
}

#[test]
fn hinge_driver_turns_the_door_exactly() {
    let r = run(&hinge_doc());
    let ps = motion::solve_poses(&r.body, &r.asm, &[pose(&[("F3", 0.0)]), pose(&[("F3", 30.0)]), pose(&[("F3", 110.0)]), pose(&[("F3", -45.0)])]).unwrap();
    for p in &ps {
        assert!(p.error.is_none(), "{:?}", p.error);
    }
    let want = [0.0, 30.0, 110.0, -45.0];
    for (p, w) in ps.iter().zip(want) {
        let pl = place_of(p, "F2");
        assert!(near(ang_y(pl), w, 1e-6), "angle {} want {w}", ang_y(pl));
        // the hinge stays on its axis: the cap stop (y = 20) and the axis position hold
        assert!(near(pl.t[1], 20.0, 1e-6), "{:?}", pl.t);
    }
    // the fixed component never moves
    assert!(ps.iter().all(|p| near(place_of(p, "F1").t[0], 0.0, 1e-12)));
}

#[test]
fn hinge_driver_is_continuous_past_180() {
    let r = run(&hinge_doc());
    let ps = motion::solve_poses(&r.body, &r.asm, &[pose(&[("F3", 170.0)]), pose(&[("F3", 200.0)]), pose(&[("F3", 360.0)])]).unwrap();
    assert!(ps.iter().all(|p| p.error.is_none()));
    assert!(near(ang_y(place_of(&ps[1], "F2")), -160.0, 1e-6));
    assert!(near(ang_y(place_of(&ps[2], "F2")), 0.0, 1e-6));
}

#[test]
fn unlisted_drivers_keep_their_value_and_nothing_is_written() {
    let r = run(&hinge_doc());
    let before = r.asm.components[1].place;
    let ps = motion::solve_poses(&r.body, &r.asm, &[pose(&[("F3", 80.0)]), pose(&[])]).unwrap();
    assert!(near(ang_y(place_of(&ps[0], "F2")), 80.0, 1e-6));
    assert!(near(ang_y(place_of(&ps[1], "F2")), 80.0, 1e-6), "a driver the pose does not list stays where it was");
    // a fresh request starts from the assembled pose
    let fresh = motion::solve_poses(&r.body, &r.asm, &[pose(&[])]).unwrap();
    assert!(near(ang_y(place_of(&fresh[0], "F2")), 0.0, 1e-6));
    // the inputs are untouched (display only)
    assert_eq!(before.t, r.asm.components[1].place.t);
    assert_eq!(before.r, r.asm.components[1].place.r);
}

#[test]
fn same_input_same_poses() {
    let r = run(&hinge_doc());
    let poses: Vec<_> = (0..12).map(|i| pose(&[("F3", 17.0 * i as f64)])).collect();
    let a = motion::solve_poses(&r.body, &r.asm, &poses).unwrap();
    let b = motion::solve_poses(&r.body, &r.asm, &poses).unwrap();
    for (x, y) in a.iter().zip(&b) {
        for ((_, p), (_, q)) in x.places.iter().zip(&y.places) {
            assert_eq!(p.t, q.t);
            assert_eq!(p.r, q.r);
        }
    }
}

fn slot_doc() -> Vec<String> {
    let b = boxp(40.0, 20.0, 10.0);
    let cyl = cylp(2.0, 10.0);
    vec![
        ins("F1", &b, r#","fixed":true"#),
        ins("F2", &cyl, r#","dx":5,"dy":3,"dz":4"#),
        mate("F3", "slot", "F2.S.e1@F2.1", "F2.S.e4@F1.1", r#","b2":"F2.S.e2@F1.1""#),
    ]
}
fn origin(r: &Run, id: &str) -> Vec3 {
    match geo(&r.body, id).unwrap() {
        G::Plane { o, .. } | G::Cyl { o, .. } => o,
        _ => panic!(),
    }
}

#[test]
fn slot_driver_slides_the_pin_along_the_slot() {
    let r = run(&slot_doc());
    let p0 = r.asm.components[1].place;
    let ps = motion::solve_poses(&r.body, &r.asm, &[pose(&[("F3", 0.0)]), pose(&[("F3", 7.5)]), pose(&[("F3", -3.0)])]).unwrap();
    assert!(ps.iter().all(|p| p.error.is_none()), "{:?}", ps.iter().map(|p| p.error.clone()).collect::<Vec<_>>());
    let dist = |p: &motion::Pose| {
        let q = place_of(p, "F2");
        Vec3::new(q.t[0] - p0.t[0], q.t[1] - p0.t[1], q.t[2] - p0.t[2])
    };
    // the pin stays between the walls (x) and at its height (y); it moves only along the slot (z)
    let d = dist(&ps[1]);
    assert!(near(d.x, 0.0, 1e-6) && near(d.y, 0.0, 1e-6) && near(d.z.abs(), 7.5, 1e-6), "{d:?}");
    let d2 = dist(&ps[2]);
    assert!(near(d2.z.abs(), 3.0, 1e-6) && d.z * d2.z < 0.0, "opposite sides: {d:?} {d2:?}");
    assert!(near(dist(&ps[0]).z, 0.0, 1e-9));
    let _ = origin(&r, "F2.S.e1@F2.1");
}

#[test]
fn bad_drivers_are_named() {
    let r = run(&hinge_doc());
    let e = motion::solve_poses(&r.body, &r.asm, &[pose(&[("Nope", 1.0)])]).err().unwrap();
    assert_eq!(e, (motion::UNKNOWN_DRIVER.to_string(), "Nope".to_string()));
    let e = motion::solve_poses(&r.body, &r.asm, &[pose(&[("F1", 1.0)])]).err().unwrap();
    assert_eq!(e.0, motion::UNKNOWN_DRIVER, "an insertPart is not a mate");
}
