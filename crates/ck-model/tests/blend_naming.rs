//! Persistent-naming failure classes for blend edge picks (issue 7): a suppressed maker, a pick that must never move
//! to an edge of another body, and picks that must survive upstream edits. Edge picks are two face names plus a point;
//! this file pins what a pick does when its names go away or when other features change around it.
//! Body A (F1 sketch, F2 extrude): x 0..40, z -20..0, y 0..10. Body B (F3 sketch, F4 newBody extrude): the same box moved by 100 in x.
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

fn sketch(id: &str, x0: f64) -> String {
    let (x1, y1) = (x0 + 40.0, 20.0);
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":0}},{{"id":"p2","x":{x1},"y":0}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

/// Body A, then (when `two`) body B, then `rest`.
fn base(two: bool, a_extra: &str) -> String {
    let mut s = format!(r#"{}, {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10{a_extra}}}"#, sketch("F1", 0.0));
    if two {
        s += &format!(r#", {}, {{"type":"extrude","id":"F4","op":"newBody","body":"B","sketch":"F3","end":"blind","depth":10}}"#, sketch("F3", 100.0));
    }
    s
}

struct R {
    errs: Vec<String>,
    volume: f64,
    faces: usize,
}

fn run(features: &str) -> R {
    let part = Part::parse(&format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#)).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| e.error.code.clone()).collect();
    let b = ex.0.body.as_ref();
    let volume = b.map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    let faces = b.map_or(0, |b| b.num_faces());
    R { errs, volume, faces }
}

const TOP_FRONT_A: &str = r#"{"faces":["F2.C1","F2.S.e1"],"at":[20,10,0]}"#;
const TOP_FRONT_B: &str = r#"{"faces":["F2.C1","F2.S.e1"],"at":[120,10,0]}"#;
const BOX_VOL: f64 = 8000.0;
/// Volume removed by a radius 2 fillet along a 40 mm edge.
const FILLET_2: f64 = 40.0 * (4.0 - std::f64::consts::PI * 4.0 / 4.0);

fn fillet(edge: &str) -> String {
    format!(r#", {{"type":"fillet","id":"F5","radius":2,"edges":[{edge}]}}"#)
}

// ---- baseline: two bodies, the pick names body A

#[test]
fn baseline_pick_rounds_body_a_only() {
    let r = run(&(base(true, "") + &fillet(TOP_FRONT_A)));
    assert!(r.errs.is_empty(), "{:?}", r.errs);
    assert!((r.volume - (2.0 * BOX_VOL - FILLET_2)).abs() < 1e-6, "{}", r.volume);
}

// ---- class: a blend edge falling back to a point on another body

/// The face names say body A but the stored point lies on the same edge of body B: names win, body B stays a box.
#[test]
fn names_beat_the_picked_point_on_another_body() {
    let r = run(&(base(true, "") + &fillet(TOP_FRONT_B)));
    assert!(r.errs.is_empty(), "{:?}", r.errs);
    assert!((r.volume - (2.0 * BOX_VOL - FILLET_2)).abs() < 1e-6, "{}", r.volume);
    // body B still has its 6 faces: A gained one face (7), B has 6, total 13
    assert_eq!(r.faces, 13, "only body A's edge is rounded");
}

/// The face names are gone (typo of the maker) and the point lies exactly on an edge of body B: the pick is lost, it is not re-pointed.
#[test]
fn lost_names_do_not_fall_back_to_a_point_on_another_body() {
    let r = run(&(base(true, "") + &fillet(r#"{"faces":["F9.C1","F9.S.e1"],"at":[120,10,0]}"#)));
    assert_eq!(r.errs, vec!["missingEdge".to_string()]);
    assert!((r.volume - 2.0 * BOX_VOL).abs() < 1e-6, "nothing may be rounded: {}", r.volume);
}

/// One name still exists (body A's top), the other is gone: still lost, even with the point on body B's edge.
#[test]
fn half_lost_names_do_not_fall_back_to_a_point_on_another_body() {
    let r = run(&(base(true, "") + &fillet(r#"{"faces":["F2.C1","F9.S.e1"],"at":[120,10,0]}"#)));
    assert_eq!(r.errs, vec!["missingEdge".to_string()]);
    assert!((r.volume - 2.0 * BOX_VOL).abs() < 1e-6, "{}", r.volume);
}

// ---- class: a suppressed maker

/// Suppress the maker of the picked faces: the blend is in error (no faces to pick), nothing else is rounded; body B is
/// the only body left and stays a box. Unsuppress: the same document rounds exactly as before.
#[test]
fn suppressed_maker_is_not_repointed_and_recovers() {
    let feats = base(true, "") + &fillet(TOP_FRONT_A);
    let sup = feats.replace(r#""type":"extrude","id":"F2","#, r#""type":"extrude","suppressed":true,"id":"F2","#);
    let r = run(&sup);
    assert_eq!(r.errs, vec!["missingEdge".to_string()], "the dependent blend reports, it is not silently moved");
    assert!((r.volume - BOX_VOL).abs() < 1e-6, "body B unchanged: {}", r.volume);
    let back = run(&feats);
    assert!(back.errs.is_empty());
    assert!((back.volume - (2.0 * BOX_VOL - FILLET_2)).abs() < 1e-6);
}

/// The maker of a face the blend needs is suppressed but the other body holds a face of the same shape: not re-pointed.
#[test]
fn suppressed_maker_with_a_lookalike_body_never_rounds_it() {
    let sup = (base(true, "") + &fillet(TOP_FRONT_B)).replace(r#""type":"extrude","id":"F2","#, r#""type":"extrude","suppressed":true,"id":"F2","#);
    let r = run(&sup);
    assert!((r.volume - BOX_VOL).abs() < 1e-6, "body B must stay a box: {}", r.volume);
}

// ---- class: picks across upstream edits

/// Changing the numbers of an upstream feature keeps the names.
#[test]
fn upstream_number_edit_keeps_the_pick() {
    for depth in [6.0, 10.0, 14.0] {
        let f = format!(r#"{}, {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth}}}{}"#, sketch("F1", 0.0), fillet(TOP_FRONT_A).replace("[20,10,0]", &format!("[20,{depth},0]")));
        let r = run(&f);
        assert!(r.errs.is_empty(), "depth {depth}: {:?}", r.errs);
        assert!((r.volume - (40.0 * 20.0 * depth - FILLET_2)).abs() < 1e-6, "depth {depth}: {}", r.volume);
    }
}

/// A later-inserted feature upstream of the blend that splits the picked side face (a pocket through the front wall's
/// top edge region) leaves the pick resolving to the surviving piece; and a pocket elsewhere does not matter.
#[test]
fn upstream_cut_elsewhere_keeps_the_pick() {
    let cut = r#", {"type":"sketch","id":"F6","plane":{"kind":"face","id":"F2.C1"},"points":[{"id":"c","x":30,"y":8}],"entities":[{"type":"circle","id":"e1","c":"c","r":3}],"constraints":[]},
{"type":"extrude","id":"F7","op":"cut","sketch":"F6","end":"blind","depth":4}"#;
    let plain = run(&(base(false, "") + &fillet(TOP_FRONT_A)));
    let with = run(&(base(false, "") + cut + &fillet(TOP_FRONT_A)));
    assert!(with.errs.is_empty(), "{:?}", with.errs);
    assert!(with.volume < plain.volume - 100.0, "the pocket is cut: {} vs {}", with.volume, plain.volume);
    // the same fillet removes the same material with and without the pocket (volume difference of the pocket alone)
    let pocket = run(&(base(false, "") + cut));
    assert!(((pocket.volume - with.volume) - FILLET_2).abs() < 1e-6, "fillet size changed: {}", pocket.volume - with.volume);
}

/// Renaming a sketch entity renames the faces made from it (documented): a pick that names the old id is lost, not
/// silently re-pointed to the neighbouring face of the same feature.
#[test]
fn renamed_entity_loses_the_pick_instead_of_repointing() {
    let renamed = base(false, "").replace(r#""id":"e1""#, r#""id":"e9""#);
    let r = run(&(renamed + &fillet(TOP_FRONT_A)));
    assert_eq!(r.errs, vec!["missingEdge".to_string()], "{:?}", r.errs);
    assert!((r.volume - BOX_VOL).abs() < 1e-6, "nothing rounded: {}", r.volume);
}

/// Deleting the picked face's maker upstream and replacing it with a feature of another id: lost, not re-pointed.
#[test]
fn replaced_maker_id_loses_the_pick() {
    let f = base(false, "").replace(r#""id":"F2""#, r#""id":"F8""#);
    let r = run(&(f + &fillet(TOP_FRONT_A)));
    assert_eq!(r.errs, vec!["missingEdge".to_string()]);
    assert!((r.volume - BOX_VOL).abs() < 1e-6);
}
