//! Thin-wall cuts and free-form extrude profiles: round trips on bodies made in-test. The history is read back from
//! the faces alone and must rebuild the same body.
//!
//! * a thin cut that left only the slanted faces where its wall ran along two edges of the body (the rest of the wall
//!   ran through air): one floor slab per plane;
//! * a boss whose profile has a spline piece (the side a B-spline extrusion): the spline is drawn as a `cvspline`;
//! * a drafted spline boss (its side a ruled B-spline between two rows): read back as a ruled loft between the rows.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::{classify, rebuild_history, Class, Outcome, Status, Tagged};

fn run(src: &str) -> Outcome {
    let fs = match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    };
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        // (the feature id: the digits after `F`, whatever follows them)
        let digits: String = body.face(fid).name.feature[1..].chars().take_while(|c| c.is_ascii_digit()).collect();
        if let Ok(t) = digits.parse::<i64>() {
            tags[fid.idx()] = vec![(t, 0)];
        }
    }
    let sw: Vec<SwFeature> = fs
        .iter()
        .map(|f| {
            let id: i64 = f.get_str("id").unwrap()[1..].parse().unwrap();
            let cut = f.get_str("op") == Some("cut");
            let (kind, name) = match f.get_str("type").unwrap() {
                "sketch" => ("Sketch".to_string(), format!("Sketch{id}")),
                "extrude" if cut => ("Extrusion".to_string(), format!("Cut-Extrude{id}")),
                "extrude" => ("Boss-Extrude".to_string(), format!("Boss-Extrude{id}")),
                // (a loft standing in for a drafted extrude: the file calls it an extrusion)
                "loft" => ("Extrusion".to_string(), format!("Boss-Extrude{id}")),
                "plane" => ("RefPlane".to_string(), format!("Plane{id}")),
                t => panic!("{t}"),
            };
            // (the SolidWorks type and name, when the test gives them)
            let kind = f.get_str("swKind").map(str::to_string).unwrap_or(kind);
            let name = f.get_str("swName").map(str::to_string).unwrap_or(name);
            let dims = f.get_f64("depth").map(|d| vec![SwDim { name: "D1".into(), text: d.to_string(), value: Some(d) }]).unwrap_or_default();
            SwFeature { id, name, kind, dims }
        })
        .collect();
    rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0, start: None })
}

fn assert_whole(o: &Outcome, exact: bool) -> Vec<String> {
    let steps: Vec<String> = o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect();
    let feats = ck_json::to_string(&Json::Arr(o.features.clone()));
    assert!(o.fallback.is_none(), "fallback: {:?}\nsteps: {steps:#?}\nfeatures: {feats}", o.fallback);
    let v = o.verify.as_ref().unwrap();
    assert!(if exact { v.exact } else { v.close }, "not {}: rel {} area {} bbox {}\nsteps: {steps:#?}\nfeatures: {feats}", if exact { "exact" } else { "close" }, v.rel, v.area_rel, v.bbox_dev);
    assert!(o.steps.iter().all(|s| s.status == Status::Rebuilt), "{steps:#?}");
    steps
}

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

#[test]
fn thin_extrudes_are_extrudes() {
    for (kind, name, cut) in [("Extrusion", "Cut-Extrude-Thin1", true), ("Cut-Extrude-Thin", "Cut-Extrude-Thin3", true), ("Boss-Extrude-Thin", "Boss-Extrude-Thin2", false)] {
        let f = SwFeature { id: 9, name: name.into(), kind: kind.into(), dims: vec![] };
        assert_eq!(classify(&f), Class::Extrude { cut }, "{kind} {name}");
    }
}

/// A thin cut along y whose two wall pieces bevel the box's two long top edges: the cut leaves two slanted faces that
/// touch nowhere, on two planes; each is a floor with nothing of the body above it.
#[test]
fn thin_cut_bevelling_two_edges_reads_as_two_slabs() {
    let src = format!(
        r#"[{BOX},
{{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a1","x":-32,"y":4}},{{"id":"a2","x":-32,"y":12}},{{"id":"a3","x":-24,"y":12}},{{"id":"b1","x":32,"y":4}},{{"id":"b2","x":32,"y":12}},{{"id":"b3","x":24,"y":12}}],"entities":[{{"id":"e1","type":"line","p1":"a1","p2":"a2"}},{{"id":"e2","type":"line","p1":"a2","p2":"a3"}},{{"id":"e3","type":"line","p1":"a3","p2":"a1"}},{{"id":"e4","type":"line","p1":"b1","p2":"b2"}},{{"id":"e5","type":"line","p1":"b2","p2":"b3"}},{{"id":"e6","type":"line","p1":"b3","p2":"b1"}}],"constraints":[]}},
{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"mid","depth":100,"swName":"Cut-Extrude-Thin1"}}]"#
    );
    let o = run(&src);
    let steps = assert_whole(&o, true);
    assert!(steps.iter().any(|s| s.contains("Thin1") && s.contains("one slab each")), "{steps:#?}");
}

/// A boss on the box whose profile closes with a cubic spline: the cap outline gives the spline piece.
#[test]
fn spline_profile_boss_round_trips() {
    let src = format!(
        r#"[{BOX},
{{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":-10,"y":-10}},{{"id":"p2","x":10,"y":-10}},{{"id":"c1","x":17,"y":-4}},{{"id":"c2","x":15,"y":5}},{{"id":"p3","x":10,"y":10}},{{"id":"p4","x":-10,"y":10}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"cvspline","pts":["p2","c1","c2","p3"],"degree":3,"knots":[0,0,0,0,1,1,1,1]}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":15}}]"#
    );
    let o = run(&src);
    let steps = assert_whole(&o, false);
    assert!(steps.iter().any(|s| s.contains("Boss-Extrude4 Rebuilt") && s.contains("cap outline")), "{steps:#?}");
    let feats = ck_json::to_string(&Json::Arr(o.features.clone()));
    assert!(feats.contains("cvspline"), "{feats}");
}

/// A closed spline boss whose side tapers (ruled B-splines between a section inside the box and a smaller one above
/// it, as SolidWorks leaves a drafted spline extrude): read back as a ruled loft between the sides' rows. The sections
/// are two cubic halves each, so the rows of two sides chain into each loop.
#[test]
fn drafted_spline_boss_reads_as_ruled_loft() {
    // (the box stands on the top plane: its height runs along +y; a section's frame there has its normal along +y)
    let ring = |id: &str, k: f64| -> (String, String) {
        let c = [(10.0, 0.0), (10.0, -10.0), (-10.0, -10.0), (-10.0, 0.0), (-10.0, 10.0), (10.0, 10.0)];
        let pts: Vec<String> = c.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"{id}{i}","x":{},"y":{}}}"#, x * k, y * k)).collect();
        let knots = "[0,0,0,0,1,1,1,1]";
        let e = format!(
            r#"{{"id":"e1","type":"cvspline","pts":["{id}0","{id}1","{id}2","{id}3"],"degree":3,"knots":{knots}}},{{"id":"e2","type":"cvspline","pts":["{id}3","{id}4","{id}5","{id}0"],"degree":3,"knots":{knots}}}"#
        );
        (pts.join(","), e)
    };
    let (p0, e0) = ring("a", 1.0);
    let (p1, e1) = ring("b", 0.9);
    let src = format!(
        r#"[{BOX},
{{"type":"plane","id":"F3","mode":"frame","origin":[0,5,0],"xDir":[1,0,0],"yDir":[0,0,-1]}},
{{"type":"sketch","id":"F4","plane":{{"kind":"feature","id":"F3"}},"points":[{p0}],"entities":[{e0}],"constraints":[]}},
{{"type":"plane","id":"F5","mode":"frame","origin":[0,15,0],"xDir":[1,0,0],"yDir":[0,0,-1]}},
{{"type":"sketch","id":"F6","plane":{{"kind":"feature","id":"F5"}},"points":[{p1}],"entities":[{e1}],"constraints":[]}},
{{"type":"loft","id":"F7","op":"add","sketches":["F4","F6"]}}]"#
    );
    let o = run(&src);
    let steps = assert_whole(&o, false);
    assert!(steps.iter().any(|s| s.contains("Boss-Extrude7 Rebuilt") && s.contains("ruled loft")), "{steps:#?}");
}

/// A boss on the box whose profile is a whole ellipse (its side an elliptic extrusion): drawn as an `ellipse`.
#[test]
fn ellipse_profile_boss_round_trips() {
    let src = format!(
        r#"[{BOX},
{{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":2,"y":-3}},{{"id":"p","x":14,"y":2}}],"entities":[{{"id":"e1","type":"ellipse","c":"c","p":"p","r":6}}],"constraints":[]}},
{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":15}}]"#
    );
    let o = run(&src);
    let steps = assert_whole(&o, false);
    assert!(steps.iter().any(|s| s.contains("Boss-Extrude4 Rebuilt") && s.contains("cap outline")), "{steps:#?}");
    let feats = ck_json::to_string(&Json::Arr(o.features.clone()));
    assert!(feats.contains(r#""type":"ellipse""#), "{feats}");
}
