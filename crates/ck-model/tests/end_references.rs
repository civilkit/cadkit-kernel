//! Extrude / cut end conditions `upToFace`, `offsetFromFace` and `upToPoint` (docs/spec-agent-interface.md,
//! "End references"): exact end planes, slanted target faces, persistent face names across edits,
//! structured errors and the warning fallback.

use ck_core::linalg::Vec3;
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
    volume: f64,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn warning(&self, code: &str) -> Option<&regen::FeatureError> {
        self.result.warnings.iter().find(|w| w.error.code == code).map(|w| &w.error)
    }
    fn param<'a>(e: &'a regen::FeatureError, k: &str) -> Option<&'a Json> {
        e.params.iter().find(|p| p.0 == k).map(|p| &p.1)
    }
    /// Id of the planar face with outward normal `n` through the point `p`.
    fn plane_at(&self, n: Vec3, p: Vec3) -> String {
        let b = self.body.as_ref().unwrap();
        for f in b.face_ids() {
            if let ck_geom::surface::Surface::Plane { frame } = &b.face(f).surface {
                let out = if b.face(f).sense { frame.z } else { -frame.z };
                if out.dot(n) > 0.999999 && (p - frame.o).dot(out).abs() < 1e-6 {
                    return b.face_id_str(f);
                }
            }
        }
        panic!("no plane face with normal {n:?} through {p:?}: {:?}", self.faces());
    }
    fn faces(&self) -> Vec<String> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        v.sort();
        v
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let volume = ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    Run { result, body: ex.brep.body.clone(), volume }
}

fn rect(id: &str, plane: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],
  "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}
const TOP: &str = r#"{"kind":"std","id":"top"}"#;

/// 40 x 40 x `h` block on the top plane (z from 0 up).
fn block(h: f64) -> String {
    format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}"#, rect("F1", TOP, 0.0, 0.0, 40.0, 40.0))
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn assert_clean(r: &Run) {
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let b = r.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty(), "invalid body: {:?}", ck_check::validate(b, 1e-6));
    assert_eq!(ck_topo::region::components(b).len(), 1, "one lump");
}

/// Boss on the top plane at x 35..45, y 10..20 (5 mm over the block's edge) with an end condition.
fn boss(end: &str) -> String {
    format!(r#"{},{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3",{end}}}"#, block(20.0), rect("F3", TOP, 35.0, 10.0, 45.0, 20.0))
}

#[test]
fn offset_from_face_moves_the_end_plane_along_the_outward_normal() {
    let base = run(&block(20.0));
    let top = base.plane_at(Vec3::Y, Vec3::new(0.0, 20.0, 0.0));
    let r = run(&boss(&format!(r#""end":"offsetFromFace","endRef":{{"kind":"face","id":"{top}"}},"offset":5"#)));
    assert_clean(&r);
    // outside part 5*10*25 + the 5 mm above the block 5*10*5
    assert!(near(r.volume - base.volume, 1250.0 + 250.0, 1e-3), "{}", r.volume - base.volume);
    assert!(r.result.warnings.is_empty());
    // negative = into the material
    let r = run(&boss(&format!(r#""end":"offsetFromFace","endRef":"{top}","offset":-5"#)));
    assert_clean(&r);
    assert!(near(r.volume - base.volume, 5.0 * 10.0 * 15.0, 1e-3), "{}", r.volume - base.volume);
    // the offset is an expression
    let r = run(&boss(&format!(r#""end":"offsetFromFace","endRef":"{top}","offset":"=2+3""#)));
    assert!(near(r.volume - base.volume, 1500.0, 1e-3), "{:?} {}", r.errors(), r.volume - base.volume);
}

#[test]
fn up_to_face_stops_at_a_covered_face() {
    // block with a 10x10 boss on top (z 20..30); a 4x4 cut from the bottom up to the boss top
    let b = format!(
        r#"{},{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":10}}"#,
        block(20.0),
        rect("F3", r#"{"kind":"face","id":"F2.C1"}"#, 15.0, 15.0, 25.0, 25.0)
    );
    let base = run(&b);
    assert!(base.errors().is_empty(), "{:?}", base.errors());
    let top = base.plane_at(Vec3::Y, Vec3::new(0.0, 30.0, 0.0));
    let cut = format!(
        r#"{b},{},{{"type":"extrude","id":"F6","op":"cut","sketch":"F5","end":"upToFace","endRef":{{"kind":"face","id":"{top}"}},"flip":true}}"#,
        rect("F5", TOP, 18.0, 18.0, 22.0, 22.0)
    );
    let r = run(&cut);
    assert_clean(&r);
    assert!(r.result.warnings.is_empty(), "{:?}", r.result.warnings);
    assert!(near(base.volume - r.volume, 16.0 * 30.0, 1e-3), "{}", base.volume - r.volume);
}

#[test]
fn up_to_face_that_does_not_cover_the_profile_falls_back_with_a_warning() {
    let base = run(&block(20.0));
    let top = base.plane_at(Vec3::Y, Vec3::new(0.0, 20.0, 0.0));
    // the boss sticks out of the block's top face
    let r = run(&boss(&format!(r#""end":"upToFace","endRef":"{top}","depth":3"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let w = r.warning("endConditionFallback").expect("fallback warning");
    assert_eq!(Run::param(w, "reason").and_then(|v| v.as_str()), Some("upToFaceOutside"));
    assert_eq!(Run::param(w, "end").and_then(|v| v.as_str()), Some("upToFace"));
    assert_eq!(Run::param(w, "depth").and_then(|v| v.as_f64()), Some(3.0));
    assert!(near(r.volume - base.volume, 5.0 * 10.0 * 3.0, 1e-3), "blind 3 mm instead: {}", r.volume - base.volume);
    // strictEnd turns it into the oracle's error
    let r = run(&boss(&format!(r#""end":"upToFace","endRef":"{top}","depth":3,"strictEnd":true"#)));
    assert!(r.errors()[0].starts_with("F4:upToFaceOutside"), "{:?}", r.errors());
}

#[test]
fn up_to_point_plane_and_vertex_references() {
    let base = run(&block(20.0));
    let r = run(&boss(r#""end":"upToPoint","endRef":[0,12,0]"#));
    assert_clean(&r);
    assert!(near(r.volume - base.volume, 600.0, 1e-3), "{}", r.volume - base.volume);
    let r = run(&boss(r#""end":"upToPoint","endRef":{"kind":"point","at":[1,25,2]}"#));
    assert_clean(&r);
    assert!(near(r.volume - base.volume, 1500.0, 1e-3), "{}", r.volume - base.volume);
    // a vertex named by the faces that meet there: the block corner (0, 0, 20)
    let top = base.plane_at(Vec3::Y, Vec3::new(0.0, 20.0, 0.0));
    let left = base.plane_at(-Vec3::X, Vec3::ZERO);
    let front = base.plane_at(Vec3::Z, Vec3::ZERO);
    let r = run(&boss(&format!(r#""end":"upToPoint","endRef":{{"kind":"vertex","faces":["{top}","{left}","{front}"],"at":[0,20,0]}}"#)));
    assert_clean(&r);
    assert!(near(r.volume - base.volume, 1000.0, 1e-3), "{}", r.volume - base.volume);
}

#[test]
fn up_to_point_behind_the_sketch_falls_back() {
    let base = run(&block(20.0));
    let r = run(&boss(r#""end":"upToPoint","endRef":[0,-5,0],"depth":3"#));
    let w = r.warning("endConditionFallback").expect("fallback warning");
    assert_eq!(Run::param(w, "reason").and_then(|v| v.as_str()), Some("upToPointBehind"));
    assert!(near(r.volume - base.volume, 150.0, 1e-3));
    let r = run(&boss(r#""end":"upToPoint","endRef":[0,-5,0],"depth":3,"strictEnd":true"#));
    assert!(r.errors()[0].starts_with("F4:upToPointBehind"), "{:?}", r.errors());
}

#[test]
fn missing_references_are_errors_not_fallbacks() {
    for end in ["upToFace", "offsetFromFace", "upToPoint"] {
        let r = run(&boss(&format!(r#""end":"{end}""#)));
        assert!(r.errors()[0].starts_with("F4:missingReference"), "{end}: {:?}", r.errors());
    }
    let r = run(&boss(r#""end":"upToFace","endRef":"F9.C9""#));
    assert!(r.errors()[0].starts_with("F4:missingReference"), "{:?}", r.errors());
    let r = run(&boss(r#""end":"upToPoint","endRef":{"kind":"vertex","faces":["F9.C9"],"at":[0,0,0]}"#));
    assert!(r.errors()[0].starts_with("F4:missingReference"), "{:?}", r.errors());
    // junk in endRef never panics
    for j in ["7", "true", "{}", "[1,2]", "{\"kind\":\"face\"}", "\"\""] {
        let r = run(&boss(&format!(r#""end":"upToFace","endRef":{j}"#)));
        assert!(r.errors()[0].starts_with("F4:missingReference"), "{j}: {:?}", r.errors());
    }
    // no body at all: a face is missing, a point is not
    let sk = rect("F1", TOP, 0.0, 0.0, 10.0, 10.0);
    let r = run(&format!(r#"{sk},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"upToFace","endRef":"F1.C0"}}"#));
    assert!(r.errors()[0].starts_with("F2:missingReference"), "{:?}", r.errors());
    let r = run(&format!(r#"{sk},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"upToPoint","endRef":[0,7,0]}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(near(r.volume, 700.0, 1e-6));
}

#[test]
fn slanted_target_face_is_trimmed_exactly() {
    // wedge: profile (0,0) (40,0) (40,10) (0,20) on the front plane, extruded 40 along +z;
    // the sloped top y = 20 - x/4 is the target
    let front = r#"{"kind":"std","id":"front"}"#;
    let sk = format!(
        r#"{{"type":"sketch","id":"F1","plane":{front},"points":[{{"id":"a","x":0,"y":0}},{{"id":"b","x":40,"y":0}},{{"id":"c","x":40,"y":10}},{{"id":"d","x":0,"y":20}}],
 "entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}}"#
    );
    let wedge = format!(r#"{sk},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":40}}"#);
    let base = run(&wedge);
    assert!(base.errors().is_empty(), "{:?}", base.errors());
    let slope = {
        let b = base.body.as_ref().unwrap();
        b.face_ids()
            .find(|&f| match &b.face(f).surface {
                ck_geom::surface::Surface::Plane { frame } => frame.z.y.abs() > 0.9 && frame.z.y.abs() < 0.99,
                _ => false,
            })
            .map(|f| b.face_id_str(f))
            .expect("sloped face")
    };
    // cut a 5 x 5 pocket up from the wedge's bottom (y = 0) to the slope: sketch x 5..10, z 5..10
    let cut = |end: &str| format!(r#"{wedge},{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","flip":true,{end}}}"#, rect("F3", TOP, 5.0, -10.0, 10.0, -5.0));
    let r = run(&cut(&format!(r#""end":"upToFace","endRef":"{slope}""#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.result.warnings.is_empty(), "{:?}", r.result.warnings);
    let b = r.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty(), "{:?}", ck_check::validate(b, 1e-6));
    let expect = 25.0 * (20.0 - 7.5 / 4.0);
    assert!(near(base.volume - r.volume, expect, 1e-3), "{} vs {expect}", base.volume - r.volume);
    assert!(r.faces().iter().all(|f| !f.contains(".E.")), "trim face renamed: {:?}", r.faces());
    // offset along the slope's outward normal: the pocket stops 2 mm below the slope plane, measured normal to it
    let r2 = run(&cut(&format!(r#""end":"offsetFromFace","endRef":"{slope}","offset":-2"#)));
    assert!(r2.errors().is_empty(), "{:?}", r2.errors());
    assert!(r2.volume > r.volume && r2.volume < base.volume);
    let cos = 40.0 / (40.0f64 * 40.0 + 10.0 * 10.0).sqrt();
    let expect2 = 25.0 * (20.0 - 7.5 / 4.0 - 2.0 / cos);
    assert!(near(base.volume - r2.volume, expect2, 1e-3), "{} vs {expect2}", base.volume - r2.volume);
    // the end condition is part of the feature: the same regen is deterministic
    assert_eq!(run(&cut(&format!(r#""end":"upToFace","endRef":"{slope}""#))).faces(), r.faces());
}

#[test]
fn persistent_face_names_survive_edits() {
    let boss_to = |h: f64, r: &str, extra: &str| {
        format!(r#"{},{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"offsetFromFace","endRef":{r},"offset":4{extra}}}"#, block(h), rect("F3", TOP, 35.0, 10.0, 45.0, 20.0))
    };
    // the target keeps its name when the block is edited (depth 20 -> 30): the end follows the face
    for h in [20.0, 30.0, 12.5] {
        let r = run(&boss_to(h, r#""F2.C1""#, ""));
        assert!(r.errors().is_empty(), "{:?}", r.errors());
        let top = h + 4.0;
        let outside = 5.0 * 10.0 * top;
        let above = 5.0 * 10.0 * 4.0;
        assert!(near(r.volume - run(&block(h)).volume, outside + above, 1e-3), "h={h}: {}", r.volume - run(&block(h)).volume);
        assert!(r.warning("endRefRelinked").is_none());
    }
    // a `~k` region suffix that is no longer there: the face family is found again, with a notice
    let r = run(&boss_to(20.0, r#""F2.C1~2""#, ""));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let w = r.warning("endRefRelinked").expect("relink notice");
    assert_eq!(Run::param(w, "to").and_then(|v| v.as_str()), Some("F2.C1"));
    assert!(near(r.volume - run(&block(20.0)).volume, 1400.0, 1e-3));
    // a renamed face with an `at` anchor on it: found by position
    let r = run(&boss_to(20.0, r#"{"kind":"face","id":"Gone.C1","at":[10,20,-10]}"#, ""));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.warning("endRefRelinked").is_some());
    assert!(near(r.volume - run(&block(20.0)).volume, 1400.0, 1e-3));
    // without the anchor it is a missingReference
    let r = run(&boss_to(20.0, r#""Gone.C1""#, ""));
    assert!(r.errors()[0].starts_with("F4:missingReference"), "{:?}", r.errors());
    // an anchor on an edge is ambiguous
    let r = run(&boss_to(20.0, r#"{"kind":"face","id":"Gone.C1","at":[0,20,0]}"#, ""));
    assert!(r.errors()[0].starts_with("F4:missingReference"), "{:?}", r.errors());
}

#[test]
fn non_planar_target_falls_back() {
    // a cylinder block: its side face is no plane
    let cyl = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"c1","c":"c","r":10}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}"#;
    let pocket = rect("F3", TOP, -2.0, -2.0, 2.0, 2.0);
    let r = run(&format!(
        r#"{cyl},{pocket},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","flip":true,"end":"upToFace","endRef":"F2.S.c1","depth":5}}"#
    ));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let w = r.warning("endConditionFallback").expect("fallback warning");
    assert_eq!(Run::param(w, "reason").and_then(|v| v.as_str()), Some("upToFaceUnsupported"));
}

#[test]
fn warnings_reach_the_wire_and_survive_json() {
    let r = run(&boss(r#""end":"upToPoint","endRef":[0,-5,0],"depth":3"#));
    let j = r.result.to_json();
    let w = j.get_arr("warnings").expect("warnings on the wire");
    assert_eq!(w.len(), 1);
    assert!(ck_json::to_string(&w[0]).contains("endConditionFallback"));
}

#[test]
fn up_to_face_parallel_to_the_extrusion_is_unreachable_not_run_through() {
    // the block's right side (x = 40) lies along the boss's extrusion (+y) and covers only part of the profile (x 35..45)
    let base = run(&block(20.0));
    let side = base.plane_at(Vec3::X, Vec3::new(40.0, 0.0, 0.0));
    for end in ["upToFace", "offsetFromFace"] {
        let r = run(&boss(&format!(r#""end":"{end}","endRef":"{side}","offset":2,"depth":3"#)));
        assert!(r.errors().is_empty(), "{end}: {:?}", r.errors());
        let w = r.warning("endConditionFallback").expect("fallback warning");
        assert_eq!(Run::param(w, "reason").and_then(|v| v.as_str()), Some("upToFaceBehind"));
        assert!(near(r.volume - base.volume, 5.0 * 10.0 * 3.0, 1e-3), "{end}: blind 3 mm instead: {}", r.volume - base.volume);
        // strictEnd turns it into a clear error
        let r = run(&boss(&format!(r#""end":"{end}","endRef":"{side}","offset":2,"depth":3,"strictEnd":true"#)));
        assert!(r.errors()[0].starts_with("F4:upToFaceBehind"), "{end}: {:?}", r.errors());
    }
    // the same for a cut: nothing is removed beyond the blind depth
    let cut = format!(
        r#"{},{},{{"type":"extrude","id":"F6","op":"cut","sketch":"F5","flip":true,"end":"upToFace","endRef":"{side}","depth":4}}"#,
        block(20.0),
        rect("F5", TOP, 35.0, 10.0, 45.0, 20.0)
    );
    let r = run(&cut);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.warning("endConditionFallback").is_some());
    assert!(near(base.volume - r.volume, 5.0 * 10.0 * 4.0, 1e-3), "{}", base.volume - r.volume);
}
