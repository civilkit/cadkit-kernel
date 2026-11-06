//! Domes, sweeps and lofts read back as the revolves and extrudes they are geometrically, on bodies made in-test: a
//! part document is regenerated, its faces are tagged with the SolidWorks feature that would have made them (a dome,
//! a sweep cut, a loft: several of our features may stand for one of theirs), the history is read back from the
//! tagged body and the rebuilt body must match the original.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::SwFeature;
use ck_swedit::{rebuild_history, Status, Tagged};

fn feats(src: &str) -> Vec<Json> {
    match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    }
}

/// `as_sw`: (our feature id, SolidWorks feature id, kind, name, face kind code of a planar cap or 0): the features
/// tagged as a SolidWorks feature of another type (several of ours may map to one of theirs). The others keep the
/// usual names (sketches, extrudes).
fn run(src: &str, as_sw: &[(&str, i64, &str, &str, i64)]) -> ck_swedit::Outcome {
    let fs = feats(src);
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let map = |id: &str| as_sw.iter().find(|m| m.0 == id);
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        let nm = &body.face(fid).name;
        let t = match map(&nm.feature) {
            Some(m) => (m.1, if matches!(nm.role, ck_topo::body::Role::Cap(_)) && m.4 != 0 { m.4 } else { 74 }),
            None => match nm.feature[1..].parse::<i64>() {
                Ok(t) => (t, 0),
                Err(_) => continue,
            },
        };
        tags[fid.idx()] = vec![t];
    }
    let mut sw: Vec<SwFeature> = vec![];
    for f in &fs {
        let fid = f.get_str("id").unwrap();
        if let Some(m) = map(fid) {
            if !sw.iter().any(|s| s.id == m.1) {
                sw.push(SwFeature { id: m.1, name: m.3.into(), kind: m.2.into(), dims: vec![] });
            }
            continue;
        }
        let id: i64 = fid[1..].parse().unwrap();
        let cut = f.get_str("op") == Some("cut");
        let (kind, name) = match f.get_str("type").unwrap() {
            "sketch" => ("Sketch", format!("Sketch{id}")),
            "plane" => ("Plane", format!("Plane{id}")),
            "extrude" if cut => ("Extrusion", format!("Cut-Extrude{id}")),
            "extrude" => ("Boss-Extrude", format!("Boss-Extrude{id}")),
            t => panic!("{t}"),
        };
        sw.push(SwFeature { id, name, kind: kind.into(), dims: vec![] });
    }
    let t = Tagged { body: &body, tags, features: &sw, unit_mm: 1.0, start: None };
    rebuild_history(&t)
}

fn assert_exact(o: &ck_swedit::Outcome) {
    let steps: Vec<String> = o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect();
    assert!(o.fallback.is_none(), "fallback: {:?}\nsteps: {steps:#?}\nfeatures: {}", o.fallback, ck_json::to_string(&Json::Arr(o.features.clone())));
    let v = o.verify.as_ref().unwrap();
    assert!(v.exact, "not exact: rel {} area {} bbox {}\nsteps: {steps:#?}\nfeatures: {}", v.rel, v.area_rel, v.bbox_dev, ck_json::to_string(&Json::Arr(o.features.clone())));
    assert!(o.steps.iter().all(|s| s.status != Status::Unsupported), "{steps:#?}");
}

fn note<'a>(o: &'a ck_swedit::Outcome, kind: &str) -> &'a str {
    &o.steps.iter().find(|s| s.kind == kind).unwrap().note
}

#[test]
fn dome_on_a_disc() {
    // a disc extruded along +Z (front plane) and a spherical cap 3.5 high on its top face, made here by a revolve
    // about the disc's axis (the dome's sphere alone does not say which way the axis runs)
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":12.7}],"constraints":[]},
    {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":2},
    {"type":"sketch","id":"F3","plane":{"kind":"std","id":"top"},"points":[{"id":"o","x":0,"y":-2},{"id":"r","x":12.7,"y":-2},{"id":"p","x":0,"y":-5.5},{"id":"c","x":0,"y":19.291428571428572}],
    "entities":[{"type":"line","id":"e1","p1":"o","p2":"r"},{"type":"arc","id":"e2","c":"c","p1":"p","p2":"r"},{"type":"line","id":"e3","p1":"p","p2":"o"},{"type":"line","id":"ax","p1":"p","p2":"o","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F4","op":"add","sketch":"F3","axis":"ax","angle":360}]"#;
    let o = run(src, &[("F3", 9, "Sketch", "Sketch9", 0), ("F4", 10, "Dome", "Dome1", 0)]);
    assert_exact(&o);
    assert!(note(&o, "Dome").contains("dome as a revolve") && note(&o, "Dome").contains("(0.000,0.000,1.000)"), "{}", note(&o, "Dome"));
}

/// A block (top plane: x -20..50, z -50..50, y 0..20) and a V groove cut along the path: straight along +X from the
/// right plane (x = 0) out through the end of the block, and round a vertical axis (x = 0, z = -30) towards -X out
/// through the other end, made here as an extrude and a revolve of one profile.
const BLOCK: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-20,"y":-50},{"id":"p2","x":50,"y":-50},{"id":"p3","x":50,"y":50},{"id":"p4","x":-20,"y":50}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
    {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}"#;
const ARC: &str = r#"{"type":"sketch","id":"F3","plane":{"kind":"std","id":"right"},"points":[{"id":"q1","x":-3,"y":25},{"id":"q2","x":0,"y":14},{"id":"q3","x":3,"y":25},{"id":"a1","x":30,"y":30},{"id":"a2","x":30,"y":0}],
    "entities":[{"id":"e1","type":"line","p1":"q1","p2":"q2"},{"id":"e2","type":"line","p1":"q2","p2":"q3"},{"id":"e3","type":"line","p1":"q3","p2":"q1"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F4","op":"cut","sketch":"F3","axis":"ax","angle":90}"#;
const LINE: &str = r#"{"type":"sketch","id":"F5","plane":{"kind":"std","id":"right"},"points":[{"id":"q1","x":-3,"y":25},{"id":"q2","x":0,"y":14},{"id":"q3","x":3,"y":25}],
    "entities":[{"id":"e1","type":"line","p1":"q1","p2":"q2"},{"id":"e2","type":"line","p1":"q2","p2":"q3"},{"id":"e3","type":"line","p1":"q3","p2":"q1"}],"constraints":[]},
    {"type":"extrude","id":"F6","op":"cut","sketch":"F5","end":"blind","depth":60,"flip":true}"#;

#[test]
fn sweep_cut_along_a_line_and_an_arc() {
    let sweep = [("F3", 7, "Sketch", "Sketch7", 0), ("F4", 8, "SweepCut", "Cut-Sweep1", 0), ("F5", 7, "Sketch", "Sketch7", 0), ("F6", 8, "SweepCut", "Cut-Sweep1", 0)];
    let o = run(&format!("[{BLOCK}, {ARC}, {LINE}]"), &sweep);
    assert_exact(&o);
    let n = note(&o, "SweepCut");
    assert!(n.contains("along a line and an arc"), "{n}");
    let types: Vec<&str> = o.features.iter().filter_map(|f| f.get_str("type")).collect();
    assert_eq!(types.iter().filter(|t| **t == "revolve").count(), 1, "{types:?}");
}

#[test]
fn sweep_cut_along_a_straight_path() {
    // the straight piece alone: a sweep that is an extrude
    let src = format!("[{BLOCK}, {LINE}]");
    let o = run(&src, &[("F5", 7, "Sketch", "Sketch7", 0), ("F6", 8, "SweepCut", "Cut-Sweep1", 0)]);
    assert_exact(&o);
    assert!(note(&o, "SweepCut").contains("straight path as an extrude"), "{}", note(&o, "SweepCut"));
}

#[test]
fn loft_cone_notched_by_a_later_cut() {
    // a frustum between two coaxial circles on a block (a loft in SolidWorks, a cone: made here by a revolve), then a
    // slot cut across its side: the loft's boundary is no pair of planar sections any more, its faces are a revolve
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-20,"y":-20},{"id":"p2","x":20,"y":-20},{"id":"p3","x":20,"y":20},{"id":"p4","x":-20,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
    {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":6},
    {"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":6},{"id":"b","x":9,"y":6},{"id":"c","x":5,"y":16},{"id":"d","x":0,"y":16}],
    "entities":[{"id":"e1","type":"line","p1":"a","p2":"b"},{"id":"e2","type":"line","p1":"b","p2":"c"},{"id":"e3","type":"line","p1":"c","p2":"d"},{"id":"e4","type":"line","p1":"d","p2":"a"},{"id":"ax","type":"line","p1":"d","p2":"a","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F4","op":"add","sketch":"F3","axis":"ax","angle":360},
    {"type":"sketch","id":"F5","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-1,"y":10},{"id":"p2","x":1,"y":10},{"id":"p3","x":1,"y":20},{"id":"p4","x":-1,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
    {"type":"extrude","id":"F6","op":"cut","sketch":"F5","end":"throughBoth"}]"#;
    let o = run(src, &[("F3", 3, "Sketch", "Sketch3", 0), ("F4", 4, "Loft", "Loft1", 77)]);
    assert_exact(&o);
    assert!(note(&o, "Loft").contains("loft as a revolve"), "{}", note(&o, "Loft"));
}
