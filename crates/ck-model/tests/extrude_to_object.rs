//! Extrude / cut "To object" and "From object" (`end:"toObject"` + `endRef`, `start:"fromObject"` + `startRef`;
//! docs/spec-agent-interface.md, "To object / From object"): the end (or start) is any reference (a face, a standard or
//! Plane-feature plane, a sketch point or vertex, a named body) and follows it when it moves. A lost or unusable
//! reference is a refusal with a hint, never a blind fallback or a re-find by position. Also: a cut whose sketch has many
//! disjoint profiles is one feature.

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
    fn error(&self, feature: &str) -> &regen::FeatureError {
        &self.result.errors.iter().find(|e| e.feature == feature).unwrap_or_else(|| panic!("no error on {feature}: {:?}", self.errors())).error
    }
    fn param<'a>(e: &'a regen::FeatureError, k: &str) -> Option<&'a Json> {
        e.params.iter().find(|p| p.0 == k).map(|p| &p.1)
    }
    /// (min, max) world y of the body's vertices.
    fn y_span(&self) -> (f64, f64) {
        let b = self.body.as_ref().unwrap();
        b.vertex_ids().map(|v| b.vertex(v).p.y).fold((f64::INFINITY, f64::NEG_INFINITY), |a, y| (a.0.min(y), a.1.max(y)))
    }
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
        panic!("no plane face with normal {n:?} through {p:?}");
    }
}

fn run_with(vars: &str, features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"variables":[{vars}],"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let volume = ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    Run { result, body: ex.brep.body.clone(), volume }
}
fn run(features: &str) -> Run {
    run_with("", features)
}

fn rect(id: &str, plane: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],
  "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}
/// The xz plane: sketch x is world x, sketch y runs along -z, the normal is +y.
const TOP: &str = r#"{"kind":"std","id":"top"}"#;

/// 40 x 40 x `h` block on the top plane (y from 0 up).
fn block(h: f64) -> String {
    format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}"#, rect("F1", TOP, 0.0, 0.0, 40.0, 40.0))
}
/// A 10 x 10 profile on the top plane at x 50..60 (beside the block), extruded with `end`.
fn post(end: &str) -> String {
    format!(r#"{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3",{end}}}"#, rect("F3", TOP, 50.0, 0.0, 60.0, 10.0))
}
/// A Plane feature parallel to the top plane at height `off` (a number or an expression).
fn lid_plane(off: &str) -> String {
    format!(r#"{{"type":"plane","id":"P1","mode":"offset","base":{TOP},"offset":{off}}}"#)
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn span_is(r: &Run, lo: f64, hi: f64) {
    let (a, b) = r.y_span();
    assert!(near(a, lo, 1e-9) && near(b, hi, 1e-9), "y {a}..{b}, expected {lo}..{hi}: {:?}", r.errors());
}

fn assert_clean(r: &Run) {
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.result.warnings.is_empty(), "{:?}", r.result.warnings);
    let b = r.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty(), "invalid body: {:?}", ck_check::validate(b, 1e-6));
}

#[test]
fn to_a_face_ends_at_its_plane_and_follows_it() {
    // the post stands beside the block: the block's top face does not cover it, its plane still ends it
    for h in [20.0, 30.0] {
        let base = run(&block(h));
        let top = base.plane_at(Vec3::Y, Vec3::new(0.0, h, 0.0));
        assert_eq!(top, "F2.C1");
        let r = run(&format!("{},{}", block(h), post(r#""end":"toObject","endRef":{"kind":"face","id":"F2.C1"}"#)));
        assert_clean(&r);
        assert!(near(r.volume - base.volume, 100.0 * h, 1e-6), "h={h}: {}", r.volume - base.volume);
    }
    // the bare face id works too
    let r = run(&format!("{},{}", block(20.0), post(r#""end":"toObject","endRef":"F2.C1""#)));
    assert_clean(&r);
    assert!(near(r.y_span().1, 20.0, 1e-9));
}

#[test]
fn to_a_plane_follows_the_plane_feature_and_the_standard_planes() {
    // a Plane feature driven by a variable: the post ends on it at 25, then at 40
    for (lid, expect) in [("25", 25.0), ("40", 40.0)] {
        let r = run_with(&format!(r#"{{"name":"lid","expr":"{lid}"}}"#), &format!(r#"{},{}"#, lid_plane(r#""=lid""#), post(r#""end":"toObject","endRef":{"kind":"feature","id":"P1"}"#)));
        assert_clean(&r);
        assert!(near(r.volume, 100.0 * expect, 1e-6), "lid={lid}: {}", r.volume);
        assert!(near(r.y_span().1, expect, 1e-9));
    }
    // a sketch on the Plane feature extruded back down to the standard top plane
    let r = run(&format!(
        r#"{},{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3","flip":true,"end":"toObject","endRef":{TOP}}}"#,
        lid_plane("30"),
        rect("F3", r#"{"kind":"feature","id":"P1"}"#, 0.0, 0.0, 10.0, 10.0)
    ));
    assert_clean(&r);
    assert!(near(r.volume, 3000.0, 1e-6), "{}", r.volume);
    span_is(&r, 0.0, 30.0);
}

#[test]
fn to_a_slanted_plane_is_cut_along_it() {
    let tilt = r#"{"type":"plane","id":"P1","mode":"frame","origin":[0,20,0],"xDir":[0,0,1],"yDir":[4,1,0]}"#;
    // the plane holds (0,20,0), z along +z and (4,1,0): y = 20 + x / 4
    let r = run(&format!(r#"{tilt},{}"#, post(r#""end":"toObject","endRef":{"kind":"feature","id":"P1"}"#)));
    assert_clean(&r);
    // x 50..60: mean height 20 + 55 / 4
    assert!(near(r.volume, 100.0 * (20.0 + 55.0 / 4.0), 1e-6), "{}", r.volume);
    assert!(near(r.y_span().1, 35.0, 1e-9));
}

#[test]
fn to_a_point_ends_on_the_plane_through_it() {
    // a sketch point on the front plane at height `tip`: the post follows it
    let marks = |tip: f64| format!(r#"{{"type":"sketch","id":"M","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"tip","x":3,"y":{tip}}}],"entities":[],"constraints":[]}}"#);
    for tip in [15.0, 30.0] {
        let r = run(&format!(r#"{},{}"#, marks(tip), post(r#""end":"toObject","endRef":{"kind":"sketchPoint","sketch":"M","point":"tip"}"#)));
        assert_clean(&r);
        assert!(near(r.volume, 100.0 * tip, 1e-6), "tip={tip}: {}", r.volume);
    }
    // a body vertex named by its faces: the block corner (0, 20, 0) follows the block's height
    let base = run(&block(20.0));
    let top = base.plane_at(Vec3::Y, Vec3::new(0.0, 20.0, 0.0));
    let left = base.plane_at(-Vec3::X, Vec3::ZERO);
    let front = base.plane_at(Vec3::Z, Vec3::ZERO);
    let v = format!(r#""end":"toObject","endRef":{{"kind":"vertex","faces":["{top}","{left}","{front}"],"at":[0,20,0]}}"#);
    for h in [20.0, 35.0] {
        let r = run(&format!("{},{}", block(h), post(&v)));
        assert_clean(&r);
        assert!(near(r.y_span().1, h, 1e-9), "h={h}: {:?}", r.y_span());
    }
}

#[test]
fn to_a_body_stops_at_its_surface() {
    // a separate body "Lid" (y 30..35 over x 45..65) and a post that runs up to it
    let lid = format!(
        r#"{},{},{{"type":"extrude","id":"L2","op":"newBody","body":"Lid","sketch":"L1","end":"blind","depth":5}}"#,
        lid_plane("30"),
        rect("L1", r#"{"kind":"feature","id":"P1"}"#, 45.0, -5.0, 65.0, 15.0)
    );
    let base = run(&format!("{},{lid}", block(20.0)));
    assert!(base.errors().is_empty(), "{:?}", base.errors());
    let r = run(&format!("{},{lid},{}", block(20.0), post(r#""end":"toObject","endRef":{"kind":"body","name":"Lid"}"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(near(r.volume - base.volume, 100.0 * 30.0, 1e-6), "{}", r.volume - base.volume);
    // a body the post never reaches is refused, not run through
    let r = run(&format!("{},{lid},{}", block(20.0), rect("F3", TOP, 100.0, 0.0, 110.0, 10.0) + r#",{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"toObject","endRef":{"kind":"body","name":"Lid"}}"#));
    assert_eq!(r.error("F4").code, "endRefMissed", "{:?}", r.errors());
    assert!(Run::param(r.error("F4"), "hint").is_some());
}

#[test]
fn cut_to_a_body_inside_the_part() {
    // a block, a separate body "Core" floating above it, and a cut from the block's top up to the core
    let core = format!(
        r#"{},{},{{"type":"extrude","id":"L2","op":"newBody","body":"Core","sketch":"L1","end":"blind","depth":5}}"#,
        lid_plane("30"),
        rect("L1", r#"{"kind":"feature","id":"P1"}"#, 10.0, 10.0, 30.0, 30.0)
    );
    let base = run(&format!("{},{core}", block(20.0)));
    // the cut starts at the bottom (y = 0) and goes up through the block: it stops at the core, so it goes through the whole block
    let cut = format!(
        r#"{},{core},{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","flip":true,"end":"toObject","endRef":{{"kind":"body","name":"Core"}}}}"#,
        block(20.0),
        rect("F3", TOP, 18.0, 18.0, 22.0, 22.0)
    );
    let r = run(&cut);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(near(base.volume - r.volume, 16.0 * 20.0, 1e-6), "{}", base.volume - r.volume);
}

#[test]
fn end_offset_is_measured_along_the_extrusion() {
    for (off, expect) in [("5", 30.0), ("-5", 20.0), (r#""=2+3""#, 30.0)] {
        let r = run(&format!(r#"{},{}"#, lid_plane("25"), post(&format!(r#""end":"toObject","endRef":{{"kind":"feature","id":"P1"}},"endOffset":{off}"#))));
        assert_clean(&r);
        assert!(near(r.y_span().1, expect, 1e-9), "offset {off}: {:?}", r.y_span());
    }
    // the same on a face: 3 mm short of the block top
    let r = run(&format!("{},{}", block(20.0), post(r#""end":"toObject","endRef":"F2.C1","endOffset":-3"#)));
    assert_clean(&r);
    let b = r.body.as_ref().unwrap();
    let top = b.vertex_ids().map(|v| b.vertex(v).p).filter(|p| p.x > 45.0).map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
    assert!(near(top, 17.0, 1e-9), "{top}");
    // a body end takes no offset: refused, with a hint
    let r = run(&format!("{},{}", block(20.0), post(r#""end":"toObject","endRef":{"kind":"body","name":"X"},"endOffset":2"#)));
    let e = r.error("F4");
    assert_eq!(e.code, "invalidValue", "{:?}", r.errors());
    assert!(Run::param(e, "hint").is_some());
}

#[test]
fn from_an_object_starts_the_extrusion_there() {
    // from the Plane feature at 10, blind 5: y 10..15
    let r = run(&format!(r#"{},{}"#, lid_plane("10"), post(r#""start":"fromObject","startRef":{"kind":"feature","id":"P1"},"end":"blind","depth":5"#)));
    assert_clean(&r);
    span_is(&r, 10.0, 15.0);
    assert!(near(r.volume, 500.0, 1e-6));
    // from a face of the block (its top at 20) to a plane at 32, with a start offset of 2: y 22..32
    let r = run(&format!(
        r#"{},{},{}"#,
        block(20.0),
        lid_plane("32"),
        post(r#""start":"fromObject","startRef":"F2.C1","startOffset":2,"end":"toObject","endRef":{"kind":"feature","id":"P1"}"#)
    ));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let b = r.body.as_ref().unwrap();
    let ys: Vec<f64> = b.vertex_ids().map(|v| b.vertex(v).p).filter(|p| p.x > 45.0).map(|p| p.y).collect();
    assert!(ys.iter().all(|y| near(*y, 22.0, 1e-9) || near(*y, 32.0, 1e-9)), "{ys:?}");
    // the start follows the plane: the plane at 10 -> 18
    let r = run_with(r#"{"name":"s","expr":"18"}"#, &format!(r#"{},{}"#, lid_plane(r#""=s""#), post(r#""start":"fromObject","startRef":{"kind":"feature","id":"P1"},"end":"blind","depth":5"#)));
    span_is(&r, 18.0, 23.0);
    // a slanted start plane: y = 20 + x / 4 up to a flat plane at 50
    let tilt = r#"{"type":"plane","id":"P2","mode":"frame","origin":[0,20,0],"xDir":[0,0,1],"yDir":[4,1,0]}"#;
    let r = run(&format!(
        r#"{tilt},{},{}"#,
        lid_plane("50"),
        post(r#""start":"fromObject","startRef":{"kind":"feature","id":"P2"},"end":"toObject","endRef":{"kind":"feature","id":"P1"}"#)
    ));
    assert_clean(&r);
    assert!(near(r.volume, 100.0 * (50.0 - 20.0 - 55.0 / 4.0), 1e-6), "{}", r.volume);
}

#[test]
fn a_lost_reference_is_refused_with_a_hint_never_guessed() {
    let hint = |r: &Run, f: &str| Run::param(r.error(f), "hint").and_then(|h| h.as_str()).map(str::to_string).unwrap_or_default();
    // a face id that no longer exists, with a picked point on a live face: refused, not re-found by position
    let r = run(&format!("{},{}", block(20.0), post(r#""end":"toObject","endRef":{"kind":"face","id":"F9.C1","at":[10,20,-10]}"#)));
    let e = r.error("F4");
    assert_eq!(e.code, "missingReference", "{:?}", r.errors());
    assert_eq!(Run::param(e, "field").and_then(|v| v.as_str()), Some("endRef"));
    assert!(!hint(&r, "F4").is_empty());
    assert!(r.result.warnings.is_empty(), "no relink: {:?}", r.result.warnings);
    // no body at all: a face reference is lost
    let r = run(&post(r#""end":"toObject","endRef":"F2.C1""#));
    assert_eq!(r.error("F4").code, "missingReference");
    // a Plane feature that is gone, or that comes later in the timeline
    let r = run(&post(r#""end":"toObject","endRef":{"kind":"feature","id":"P1"}"#));
    assert_eq!(r.error("F4").code, "missingReference");
    let r = run(&format!(r#"{},{}"#, post(r#""end":"toObject","endRef":{"kind":"feature","id":"P1"}"#), lid_plane("25")));
    assert_eq!(r.error("F4").code, "missingReference");
    assert!(hint(&r, "F4").contains("earlier"), "{}", hint(&r, "F4"));
    // a sketch point that was deleted, a body that was renamed
    let r = run(&format!(
        r#"{{"type":"sketch","id":"M","plane":{TOP},"points":[],"entities":[],"constraints":[]}},{}"#,
        post(r#""end":"toObject","endRef":{"kind":"sketchPoint","sketch":"M","point":"tip"}"#)
    ));
    assert_eq!(r.error("F4").code, "missingReference");
    let r = run(&format!("{},{}", block(20.0), post(r#""end":"toObject","endRef":{"kind":"body","name":"Lid"}"#)));
    assert_eq!(r.error("F4").code, "missingReference");
    // the start reference the same way
    let r = run(&format!("{},{}", block(20.0), post(r#""start":"fromObject","startRef":"F9.C1","end":"blind","depth":5"#)));
    assert_eq!(r.error("F4").code, "missingReference");
    assert_eq!(Run::param(r.error("F4"), "field").and_then(|v| v.as_str()), Some("startRef"));
    // absent or junk references never panic and are refused
    for j in ["", r#","endRef":7"#, r#","endRef":{}"#, r#","endRef":[1,2]"#, r#","endRef":{"kind":"face"}"#, r#","endRef":"""#, r#","endRef":{"kind":"nope","id":"x"}"#] {
        let r = run(&format!("{},{}", block(20.0), post(&format!(r#""end":"toObject"{j}"#))));
        assert_eq!(r.error("F4").code, "missingReference", "{j}: {:?}", r.errors());
    }
}

#[test]
fn an_end_that_cannot_be_reached_is_refused_not_run_blind() {
    // a plane behind the profile
    let r = run(&format!(r#"{},{}"#, lid_plane("-10"), post(r#""end":"toObject","endRef":{"kind":"feature","id":"P1"},"depth":5"#)));
    assert_eq!(r.error("F4").code, "endRefBehind", "{:?}", r.errors());
    assert!(Run::param(r.error("F4"), "hint").is_some());
    assert!(r.result.warnings.is_empty());
    // a plane that holds the extrusion direction gives no end
    let r = run(&post(r#""end":"toObject","endRef":{"kind":"std","id":"right"},"depth":5"#));
    assert_eq!(r.error("F4").code, "endRefNoEnd", "{:?}", r.errors());
    // a start beyond the end
    let r = run(&format!(
        r#"{},{},{}"#,
        lid_plane("30"),
        r#"{"type":"plane","id":"P2","mode":"offset","base":{"kind":"std","id":"top"},"offset":10}"#,
        post(r#""start":"fromObject","startRef":{"kind":"feature","id":"P1"},"end":"toObject","endRef":{"kind":"feature","id":"P2"}"#)
    ));
    assert_eq!(r.error("F4").code, "endRefBehind", "{:?}", r.errors());
    // From object takes blind and To object ends only
    let r = run(&format!(r#"{},{}"#, lid_plane("10"), post(r#""start":"fromObject","startRef":{"kind":"feature","id":"P1"},"end":"mid","depth":5"#)));
    assert_eq!(r.error("F4").code, "invalidValue", "{:?}", r.errors());
    // an unknown start is refused like an unknown end
    let r = run(&post(r#""start":"sideways","end":"blind","depth":5"#));
    assert_eq!(r.error("F4").code, "invalidValue", "{:?}", r.errors());
}

#[test]
fn a_cut_of_many_holes_apart_is_one_feature() {
    // a 210 x 210 x 10 plate and one cut sketch of 10 x 10 = 100 circles r 4 on a 20 mm pitch
    let mut pts = vec![];
    let mut ents = vec![];
    for i in 0..10 {
        for j in 0..10 {
            let k = i * 10 + j;
            pts.push(format!(r#"{{"id":"c{k}","x":{},"y":{}}}"#, 15 + 20 * i, 15 + 20 * j));
            ents.push(format!(r#"{{"type":"circle","id":"e{k}","c":"c{k}","r":4}}"#));
        }
    }
    let holes = format!(r#"{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), ents.join(","));
    let plate = format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}"#, rect("F1", TOP, 0.0, 0.0, 210.0, 210.0));
    let t0 = std::time::Instant::now();
    let r = run(&format!(r#"{plate},{holes},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"through"}}"#));
    let ms = t0.elapsed().as_millis();
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let b = r.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty(), "{:?}", ck_check::validate(b, 1e-6));
    let expect = 210.0 * 210.0 * 10.0 - 100.0 * std::f64::consts::PI * 16.0 * 10.0;
    assert!(near(r.volume, expect, 1e-3 * expect), "{} vs {expect}", r.volume);
    // one lump, 100 hole walls
    assert_eq!(ck_topo::region::components(b).len(), 1);
    let walls = b.face_ids().filter(|&f| matches!(b.face(f).surface, ck_geom::surface::Surface::Cylinder { .. })).count();
    assert!(walls >= 100, "{walls}");
    eprintln!("100 holes in one cut: {ms} ms");
    // a blind cut of the same holes (the tool ends inside the plate) works the same way
    let r = run(&format!(r#"{plate},{holes},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":4}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let expect = 210.0 * 210.0 * 10.0 - 100.0 * std::f64::consts::PI * 16.0 * 4.0;
    assert!(near(r.volume, expect, 1e-3 * expect), "{} vs {expect}", r.volume);
}
