//! Profiles and blend supports later features consumed, on parts made in-test: the faces are tagged with the
//! feature that made them (as a SolidWorks file tags them), the history is read back and the rebuilt body must
//! match the original.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::{rebuild_history, Outcome, Status, Tagged};

fn feats(src: &str) -> Vec<Json> {
    match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    }
}

fn sw_features(fs: &[Json]) -> Vec<SwFeature> {
    let mut out = vec![];
    for f in fs {
        let id: i64 = f.get_str("id").unwrap()[1..].parse().unwrap();
        let ty = f.get_str("type").unwrap();
        let cut = f.get_str("op") == Some("cut");
        let (kind, name, dims): (&str, String, Vec<(&str, f64)>) = match ty {
            "sketch" => ("Sketch", format!("Sketch{id}"), vec![]),
            "plane" => ("Plane", format!("Plane{id}"), vec![]),
            "extrude" if cut => ("Extrusion", format!("Cut-Extrude{id}"), f.get_f64("depth").map(|d| vec![("D1", d)]).unwrap_or_default()),
            "extrude" => ("Boss-Extrude", format!("Boss-Extrude{id}"), f.get_f64("depth").map(|d| vec![("D1", d)]).unwrap_or_default()),
            "fillet" => ("Fillet", format!("Fillet{id}"), vec![("D1", f.get_f64("radius").unwrap())]),
            "chamfer" => ("Chamfer", format!("Chamfer{id}"), vec![("D1", f.get_f64("distance").unwrap())]),
            t => panic!("{t}"),
        };
        out.push(SwFeature { id, name, kind: kind.into(), dims: dims.into_iter().map(|(n, v)| SwDim { name: n.into(), text: v.to_string(), value: Some(v) }).collect() });
    }
    out
}

fn run(src: &str) -> Outcome {
    let fs = feats(src);
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        if let Ok(t) = body.face(fid).name.feature[1..].parse::<i64>() {
            tags[fid.idx()] = vec![(t, 0)];
        }
    }
    let sw = sw_features(&fs);
    rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 })
}

fn steps(o: &Outcome) -> Vec<String> {
    o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect()
}

fn assert_exact(o: &Outcome) {
    let st = steps(o);
    assert!(o.fallback.is_none(), "fallback: {:?}\nsteps: {st:#?}\nfeatures: {}", o.fallback, ck_json::to_string(&Json::Arr(o.features.clone())));
    let v = o.verify.as_ref().unwrap();
    assert!(v.exact, "not exact: rel {} area {}\nsteps: {st:#?}\nfeatures: {}", v.rel, v.area_rel, ck_json::to_string(&Json::Arr(o.features.clone())));
    assert!(o.steps.iter().all(|s| s.status == Status::Rebuilt), "{st:#?}");
}

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// A sketch of axis-parallel rectangles `(x0, y0, x1, y1)` on `plane`.
fn rects(id: &str, plane: &str, rs: &[(f64, f64, f64, f64)]) -> String {
    let (mut pts, mut ents) = (vec![], vec![]);
    for (k, &(x0, y0, x1, y1)) in rs.iter().enumerate() {
        let c = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
        for (i, (x, y)) in c.iter().enumerate() {
            pts.push(format!(r#"{{"id":"p{k}_{i}","x":{x},"y":{y}}}"#));
        }
        for i in 0..4 {
            ents.push(format!(r#"{{"id":"e{}","type":"line","p1":"p{k}_{i}","p2":"p{k}_{}"}}"#, 4 * k + i + 1, (i + 1) % 4));
        }
    }
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), ents.join(","))
}

#[test]
fn bosses_whose_inner_ends_sit_in_earlier_material() {
    // two tabs from z 2 to 8 reach into the box's side: their inner ends and their caps' inner edges left no face
    // of theirs, so each tab's outline closes through the box (four ends nothing meets)
    let src = format!(
        r#"[{BOX},
        {{"type":"plane","id":"F3","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":2}},
        {sk},
        {{"type":"extrude","id":"F5","op":"add","sketch":"F4","end":"blind","depth":6}}]"#,
        sk = rects("F4", r#"{"kind":"feature","id":"F3"}"#, &[(20.0, -15.0, 40.0, -5.0), (20.0, 5.0, 40.0, 15.0)])
    );
    let o = run(&src);
    assert_exact(&o);
    let note = &o.steps.iter().find(|s| s.name == "Boss-Extrude5").unwrap().note;
    assert!(note.contains("2 loop(s)"), "{note}");
}

#[test]
fn notches_cut_out_of_an_edge() {
    // two notches through the box at its +y edge: each notch's walls end at the edge, where the cut breaks out
    let src = format!(
        r#"[{BOX},
        {sk},
        {{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"through","flip":true}}]"#,
        sk = rects("F3", r#"{"kind":"std","id":"top"}"#, &[(-15.0, 15.0, -5.0, 25.0), (5.0, 15.0, 15.0, 25.0)])
    );
    let o = run(&src);
    assert_exact(&o);
    let note = &o.steps.iter().find(|s| s.name == "Cut-Extrude4").unwrap().note;
    assert!(note.contains("2 loop(s)"), "{note}");
}

#[test]
fn chamfer_on_a_vertical_edge_between_caps() {
    // the strip of a chamfer on a short vertical edge meets the caps along edges longer than its rails: the
    // supports are the two walls whose intersection runs in the strip's plane, not the parallel caps
    let src = format!(
        r#"[{BOX},
        {{"type":"chamfer","id":"F3","chamferType":"equal","distance":8,"edges":[{{"faces":["F2.S.e2","F2.S.e3"],"at":[30,20,5]}}]}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
}

#[test]
fn chamfer_whose_rail_a_later_chamfer_cut_back() {
    // a vertical chamfer, then a chamfer round the top that trims the first strip: the later strip is longer than
    // the first strip's rails but is no support of it
    let src = format!(
        r#"[{BOX},
        {{"type":"chamfer","id":"F3","chamferType":"equal","distance":8,"edges":[{{"faces":["F2.S.e2","F2.S.e3"],"at":[30,20,5]}}]}},
        {{"type":"chamfer","id":"F4","chamferType":"equal","distance":2,"edges":[{{"faces":["F2.C1","F2.S.e2"],"at":[30,0,10]}},{{"faces":["F2.C1","F3.B0"],"at":[26,16,10]}},{{"faces":["F2.C1","F2.S.e3"],"at":[0,20,10]}}]}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
}

#[test]
fn concave_fillet_round_a_boss_foot() {
    // a round at the foot of a boss: the ball rolls outside the material, its sharp edge is where the boss's walls
    // meet the box's top
    let src = format!(
        r#"[{BOX},
        {sk},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":8}},
        {{"type":"fillet","id":"F5","radius":2,"edges":[{{"faces":["F2.C1","F4.S.e1"],"at":[0,-5,10]}},{{"faces":["F2.C1","F4.S.e3"],"at":[0,5,10]}}]}}]"#,
        sk = rects("F3", r#"{"kind":"face","id":"F2.C1"}"#, &[(-10.0, -5.0, 10.0, 5.0)])
    );
    let o = run(&src);
    assert_exact(&o);
}
