//! Loft round trips on bodies made in-test: a part document with a loft is regenerated, its faces are tagged with
//! the feature that made them (end caps with the loft cap kind, as a SolidWorks file tags them), the history is read
//! back from the tagged body and the rebuilt body must match the original.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::{rebuild_history, Status, Tagged};
use ck_topo::body::Role;

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
            "plane" => ("Plane", format!("Plane{id}"), f.get_f64("offset").map(|d| vec![("D1", d)]).unwrap_or_default()),
            "extrude" => ("Boss-Extrude", format!("Boss-Extrude{id}"), f.get_f64("depth").map(|d| vec![("D1", d)]).unwrap_or_default()),
            "loft" if cut => ("Loft", format!("Cut-Loft{id}"), vec![]),
            "loft" => ("Loft", format!("Loft{id}"), vec![]),
            t => panic!("{t}"),
        };
        out.push(SwFeature { id, name, kind: kind.into(), dims: dims.into_iter().map(|(n, v)| SwDim { name: n.into(), text: v.to_string(), value: Some(v) }).collect() });
    }
    out
}

fn run(src: &str) -> ck_swedit::Outcome {
    let fs = feats(src);
    let lofts: Vec<String> = fs.iter().filter(|f| f.get_str("type") == Some("loft")).map(|f| f.get_str("id").unwrap().to_string()).collect();
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        let name = &body.face(fid).name;
        if let Ok(t) = name.feature[1..].parse::<i64>() {
            let loft = lofts.contains(&name.feature);
            let kind = match name.role {
                Role::Cap(_) if loft => 77,
                Role::Cap(_) => 40,
                _ => 74,
            };
            tags[fid.idx()] = vec![(t, kind)];
        }
    }
    let sw = sw_features(&fs);
    let t = Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 };
    rebuild_history(&t)
}

fn steps(o: &ck_swedit::Outcome) -> Vec<String> {
    o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect()
}

fn assert_exact(o: &ck_swedit::Outcome) {
    let st = steps(o);
    let v = o.verify.as_ref().map(|v| ck_json::to_string(&v.json())).unwrap_or_default();
    assert!(o.fallback.is_none(), "fallback: {:?}\nverify: {v}\nsteps: {st:#?}\nfeatures: {}", o.fallback, ck_json::to_string(&Json::Arr(o.features.clone())));
    let v = o.verify.as_ref().unwrap();
    assert!(v.exact, "not exact: rel {} area {} bbox {}\nsteps: {st:#?}\nfeatures: {}", v.rel, v.area_rel, v.bbox_dev, ck_json::to_string(&Json::Arr(o.features.clone())));
}

fn loft_of(o: &ck_swedit::Outcome) -> Json {
    o.features.iter().find(|f| f.get_str("type") == Some("loft")).cloned().expect("a loft in the rebuilt document")
}

fn square(id: &str, plane: &str, h: f64, c: (f64, f64)) -> String {
    let (x0, x1, y0, y1) = (c.0 - h, c.0 + h, c.1 - h, c.1 + h);
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

fn circle(id: &str, plane: &str, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#)
}

fn offset(id: &str, d: f64) -> String {
    format!(r#"{{"type":"plane","id":"{id}","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":{d}}}"#)
}

const TOP: &str = r#"{"kind":"std","id":"top"}"#;

fn on(id: &str) -> String {
    format!(r#"{{"kind":"feature","id":"{id}"}}"#)
}

#[test]
fn loft_square_to_circle() {
    // the shape of the usual test part: a square at the base, a circle on an offset plane, straight rulings
    let src = format!(
        r#"[{}, {}, {}, {{"type":"loft","id":"F4","op":"add","sketches":["F1","F3"]}}]"#,
        square("F1", TOP, 5.0, (0.0, 0.0)),
        offset("F2", 10.0),
        circle("F3", &on("F2"), 5.0)
    );
    let o = run(&src);
    assert_exact(&o);
    let l = loft_of(&o);
    assert!(l.get("startNormal").is_none() && l.get("endNormal").is_none(), "{}", ck_json::to_string(&l));
    let note = &o.steps.iter().find(|s| s.kind == "Loft").unwrap().note;
    assert!(note.contains("boss loft") && note.contains("4 -> 4") && note.contains("ruled"), "{note}");
}

#[test]
fn loft_normal_to_the_sections() {
    // end constraints: the loft leaves both sections along their normals (unequal tensions)
    let src = format!(
        r#"[{}, {}, {}, {{"type":"loft","id":"F4","op":"add","sketches":["F1","F3"],"startNormal":1.5,"endNormal":0.75}}]"#,
        square("F1", TOP, 6.0, (0.0, 0.0)),
        offset("F2", 12.0),
        circle("F3", &on("F2"), 4.0)
    );
    let o = run(&src);
    assert_exact(&o);
    let l = loft_of(&o);
    let (t0, t1) = (l.get_f64("startNormal").unwrap_or(0.0), l.get_f64("endNormal").unwrap_or(0.0));
    assert!((t0 - 1.5).abs() < 1e-6 && (t1 - 0.75).abs() < 1e-6, "tensions {t0} {t1}");
}

fn block() -> String {
    format!(r#"{}, {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":6}}"#, square("F1", TOP, 15.0, (0.0, 0.0)))
}

#[test]
fn loft_boss_on_a_block() {
    // a loft standing on the top of a block: no cap of its own there
    let src = format!(
        r#"[{}, {}, {}, {}, {}, {{"type":"loft","id":"F7","op":"add","sketches":["F4","F6"]}}]"#,
        block(),
        offset("F3", 6.0),
        square("F4", &on("F3"), 6.0, (2.0, 1.0)),
        offset("F5", 16.0),
        circle("F6", &on("F5"), 3.0)
    );
    let o = run(&src);
    assert_exact(&o);
    assert!(o.steps.iter().any(|s| s.note.starts_with("boss loft") && s.note.contains("1 cap(s)")), "{:#?}", steps(&o));
}

#[test]
fn loft_cut_into_a_block() {
    // a loft cut from the block's base into it
    let src = format!(
        r#"[{}, {}, {}, {}, {{"type":"loft","id":"F6","op":"cut","sketches":["F3","F5"]}}]"#,
        block(),
        square("F3", TOP, 4.0, (-7.0, -7.0)),
        offset("F4", 4.0),
        square("F5", &on("F4"), 2.0, (-7.0, -6.0))
    );
    let o = run(&src);
    assert_exact(&o);
    assert!(o.steps.iter().any(|s| s.note.starts_with("cut loft")), "{:#?}", steps(&o));
}

#[test]
fn loft_to_a_point() {
    let apex = r#"{"type":"sketch","id":"F3","plane":{"kind":"feature","id":"F2"},"points":[{"id":"a","x":1,"y":2}],"entities":[{"type":"point","id":"e1","p":"a"}],"constraints":[]}"#;
    let src = format!(r#"[{}, {}, {apex}, {{"type":"loft","id":"F4","op":"add","sketches":["F1","F3"]}}]"#, square("F1", TOP, 5.0, (0.0, 0.0)), offset("F2", 9.0));
    let o = run(&src);
    assert_exact(&o);
    assert!(o.steps.iter().any(|s| s.note.contains("to a point")), "{:#?}", steps(&o));
}

#[test]
fn loft_hollow() {
    // a square with a round hole to a smaller square with a smaller hole
    let holed = |id: &str, plane: &str, h: f64, r: f64| {
        square(id, plane, h, (0.0, 0.0)).replace(
            r#"{"type":"line","id":"e4","p1":"p4","p2":"p1"}]"#,
            &format!(r#"{{"type":"line","id":"e4","p1":"p4","p2":"p1"}},{{"type":"circle","id":"e5","c":"c","r":{r}}}]"#),
        ).replace(r#""points":["#, r#""points":[{"id":"c","x":0,"y":0},"#)
    };
    let src = format!(r#"[{}, {}, {}, {{"type":"loft","id":"F4","op":"add","sketches":["F1","F3"]}}]"#, holed("F1", TOP, 8.0, 3.0), offset("F2", 10.0), holed("F3", &on("F2"), 5.0, 2.0));
    let o = run(&src);
    assert_exact(&o);
    assert!(o.steps.iter().any(|s| s.note.contains("1 inner loop")), "{:#?}", steps(&o));
}

#[test]
fn loft_three_sections_falls_back() {
    // a loft through a middle section is not read back: the file keeps its plain solid, with the reason
    let src = format!(
        r#"[{}, {}, {}, {}, {}, {{"type":"loft","id":"F6","op":"add","sketches":["F1","F3","F5"]}}]"#,
        square("F1", TOP, 5.0, (0.0, 0.0)),
        offset("F2", 8.0),
        circle("F3", &on("F2"), 3.0),
        offset("F4", 16.0),
        square("F5", &on("F4"), 4.0, (0.0, 0.0))
    );
    let o = run(&src);
    assert!(o.fallback.is_some(), "{:#?}", steps(&o));
    let s = o.steps.iter().find(|s| s.kind == "Loft").unwrap();
    assert_eq!(s.status, Status::Unsupported, "{:#?}", steps(&o));
}
