//! `hole` end conditions other than blind / through (docs/spec-agent-interface.md, "Hole: end conditions"):
//! `upToNext` stops each centre at the first wall it drills out of, `upToFace` / `offsetFromFace` at a face's plane.
//! Volumes are derived by hand from a 100 x 40 x 30 block on the front plane (z = 0..30), holes d18 drilled from the
//! front plane into the block (`flip`).

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

const D: f64 = 18.0;
const T: f64 = 3.0;
fn disc() -> f64 {
    PI * (D / 2.0) * (D / 2.0)
}

/// (errors "F:code {..}", volume, sorted face ids) of the features `feats` (JSON objects, comma separated).
fn run(feats: &str) -> (Vec<String>, f64, Vec<String>) {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{feats}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let b = ex.brep.body.as_ref().expect("a body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{:?}", iss.iter().take(3).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
    let mut ids: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    ids.sort();
    (errs, ck_topo::mass::mass_props(b).volume, ids)
}

fn rect(id: &str, x0: f64, x1: f64, w: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[
          {{"id":"p1","x":{x0},"y":-{w}}},{{"id":"p2","x":{x1},"y":-{w}}},{{"id":"p3","x":{x1},"y":{w}}},{{"id":"p4","x":{x0},"y":{w}}}],
         "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
          {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}
/// The 100 x 40 x 30 block (F1, F2).
fn block() -> String {
    format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}}"#, rect("F1", -50.0, 50.0, 20.0))
}
/// The block with its bottom raised by 10 over x > 0 (F3, F4: the step floor is at z = 10).
fn stepped() -> String {
    format!(r#"{},{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":10,"flip":true}}"#, block(), rect("F3", 0.0, 60.0, 25.0))
}
fn shell(open: &str) -> String {
    format!(r#"{{"type":"shell","id":"SH","faces":[{open}],"thickness":{T}}}"#)
}
/// Hole sketch HS on the front plane with centres at (x, 0), and the hole H with `fields`.
fn hole(xs: &[f64], fields: &str) -> String {
    let pts: Vec<String> = xs.iter().enumerate().map(|(i, x)| format!(r#"{{"id":"c{i}","x":{x},"y":0}}"#)).collect();
    let ents: Vec<String> = (0..xs.len()).map(|i| format!(r#"{{"type":"point","id":"q{i}","p":"c{i}"}}"#)).collect();
    format!(
        r#"{{"type":"sketch","id":"HS","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}},
        {{"type":"hole","id":"H","sketch":"HS","kind":"simple","diameter":{D},"flip":true{fields}}}"#,
        pts.join(","),
        ents.join(",")
    )
}
/// Volume removed by the hole from `base` (which must regenerate cleanly with and without it).
fn removed(base: &str, xs: &[f64], fields: &str) -> f64 {
    let (e0, v0, _) = run(base);
    assert!(e0.is_empty(), "base: {e0:?}");
    let (e, v, ids) = run(&format!("{base},{}", hole(xs, fields)));
    assert!(e.is_empty(), "{e:?} faces {ids:?}");
    v0 - v
}
fn error(base: &str, xs: &[f64], fields: &str) -> String {
    run(&format!("{base},{}", hole(xs, fields))).0.into_iter().next().unwrap_or_default()
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-3 * b.max(1.0), "{a} vs {b}");
}

#[test]
fn up_to_next_closed_shell_cuts_the_floor_only() {
    let base = format!("{},{}", block(), shell(""));
    close(removed(&base, &[30.0], r#","end":"upToNext""#), disc() * T);
    // through all still drills both walls
    close(removed(&base, &[30.0], r#","end":"through""#), 2.0 * disc() * T);
}

#[test]
fn up_to_next_open_shell_and_solid_block() {
    let open = format!("{},{}", block(), shell(r#""F2.C1""#));
    close(removed(&open, &[30.0], r#","end":"upToNext""#), disc() * T);
    // a solid block: the first exit is the far side, the same as through
    close(removed(&block(), &[30.0], r#","end":"upToNext""#), disc() * 30.0);
    close(removed(&block(), &[30.0], r#","end":"through""#), disc() * 30.0);
}

#[test]
fn up_to_next_each_centre_its_own_length() {
    // solid stepped block: x = -30 drills 30, x = 30 crosses the 10 mm gap first and drills 20
    close(removed(&stepped(), &[-30.0, 30.0], r#","end":"upToNext""#), disc() * (30.0 + 20.0));
    // shelled: each centre only cuts its own floor (one 3 mm wall at z 0..3, the other at z 10..13), not the roof
    let sh = format!("{},{}", stepped(), shell(""));
    close(removed(&sh, &[-30.0, 30.0], r#","end":"upToNext""#), 2.0 * disc() * T);
    close(removed(&sh, &[-30.0, 30.0], r#","end":"through""#), 4.0 * disc() * T);
}

#[test]
fn up_to_next_missing_the_body_is_refused() {
    // drilled away from the block (no flip: out of the front plane face)
    let e = run(&format!("{},{}", block(), hole(&[30.0], r#","end":"upToNext""#).replace(r#","flip":true"#, ""))).0;
    assert!(e.first().is_some_and(|e| e.starts_with("H:upToNextNone") && e.contains("hint")), "{e:?}");
}

#[test]
fn up_to_face_and_offset_from_face() {
    // the step floor (z = 10) of the cut F4, reached from x = -30 inside the solid half: a blind 10 there
    let (_, _, ids) = run(&stepped());
    assert!(ids.iter().any(|i| i == "F4.C1"), "{ids:?}");
    close(removed(&stepped(), &[-30.0], r#","end":"upToFace","endRef":"F4.C1""#), disc() * 10.0);
    // the far face, 10 back into the material, from two centres (each its own length, 20)
    close(removed(&block(), &[-30.0, 30.0], r#","end":"offsetFromFace","endRef":"F2.C1","offset":-10"#), 2.0 * disc() * 20.0);
    // the far face itself is a hole all the way
    close(removed(&block(), &[30.0], r#","end":"upToFace","endRef":"F2.C1""#), disc() * 30.0);
    // `depth` is the blind depth only: ignored here
    close(removed(&block(), &[30.0], r#","end":"offsetFromFace","endRef":"F2.C1","offset":-10,"depth":3"#), disc() * 20.0);
    // a drill point below the end
    let r = D / 2.0;
    let h = r / 59f64.to_radians().tan();
    close(removed(&block(), &[30.0], r#","end":"offsetFromFace","endRef":"F2.C1","offset":-10,"tip":true"#), disc() * 20.0 + disc() * h / 3.0);
}

#[test]
fn hole_end_face_refusals() {
    let code = |fields: &str| error(&block(), &[30.0], fields);
    // a face parallel to the hole axis (a side wall)
    let e = code(r#","end":"upToFace","endRef":"F2.S.e1""#);
    assert!(e.starts_with("H:holeEndFace") && e.contains("hint") && e.contains("parallel"), "{e}");
    // the face behind the sketch plane (the plane is the front face itself, or 5 outside it)
    let e = code(r#","end":"offsetFromFace","endRef":"F2.C0","offset":5"#);
    assert!(e.starts_with("H:holeEndFace") && e.contains("behind"), "{e}");
    assert!(code(r#","end":"upToFace","endRef":"F2.C0""#).starts_with("H:holeEndFace"));
    // a curved face: the wall of an earlier hole
    let base = format!("{},{}", block(), hole(&[0.0], r#","end":"through""#).replace(r#""id":"H""#, r#""id":"H0""#).replace(r#""id":"HS""#, r#""id":"HS0""#).replace(r#""sketch":"HS""#, r#""sketch":"HS0""#));
    let e = error(&base, &[30.0], r#","end":"upToFace","endRef":"H0.S.h1""#);
    assert!(e.starts_with("H:holeEndFace") && e.contains("planar"), "{e}");
    // no endRef / a face that does not exist
    assert!(code(r#","end":"upToFace""#).starts_with("H:missingReference"), "{}", code(r#","end":"upToFace""#));
    assert!(code(r#","end":"upToFace","endRef":"F9.C1""#).starts_with("H:missingReference"));
}

#[test]
fn blind_and_through_unchanged() {
    close(removed(&block(), &[30.0], r#","end":"blind","depth":10"#), disc() * 10.0);
    let r = D / 2.0;
    let h = r / 59f64.to_radians().tan();
    close(removed(&block(), &[30.0], r#","end":"blind","depth":10,"tip":true"#), disc() * 10.0 + disc() * h / 3.0);
    close(removed(&block(), &[30.0], r#","end":"through""#), disc() * 30.0);
    close(removed(&block(), &[30.0], r#","end":"throughBoth""#), disc() * 30.0);
}
