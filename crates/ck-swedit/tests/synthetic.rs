//! Round trips on bodies made in-test: a part document is regenerated, its faces are tagged with the feature that
//! made them (as a SolidWorks file tags them), the history is read back from the tagged body and the rebuilt body
//! must match the original.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::{rebuild_history, Status, Tagged};

fn feats(src: &str) -> Vec<Json> {
    match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    }
}

/// A SolidWorks-style feature list for our document: id = the number of `F<n>`, type from ours.
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
            "hole" => ("HoleWizard", format!("Hole{id}"), vec![("Hole Dia.", f.get_f64("diameter").unwrap())]),
            "shell" => ("Shell", format!("Shell{id}"), vec![("D1", f.get_f64("thickness").unwrap())]),
            t => panic!("{t}"),
        };
        out.push(SwFeature { id, name, kind: kind.into(), dims: dims.into_iter().map(|(n, v)| SwDim { name: n.into(), text: v.to_string(), value: Some(v) }).collect() });
    }
    out
}

struct Res {
    out: ck_swedit::Outcome,
    types: Vec<String>,
}

fn run(src: &str) -> Res {
    let fs = feats(src);
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    // our shell keeps the original names on its inner faces; SolidWorks tags them with the shell feature: here a
    // face with a vertex strictly inside the body's box is an inner face
    let shell_id: Option<i64> = fs.iter().find(|f| f.get_str("type") == Some("shell")).map(|f| f.get_str("id").unwrap()[1..].parse().unwrap());
    let (lo, hi) = body.bbox();
    let on_box = |p: ck_geom::Vec3| [(p.x, lo.x, hi.x), (p.y, lo.y, hi.y), (p.z, lo.z, hi.z)].iter().any(|(v, a, b)| (v - a).abs() < 1e-6 || (v - b).abs() < 1e-6);
    let tag: Vec<Option<i64>> = body
        .face_ids()
        .map(|fid| {
            let f = body.face(fid);
            let inner = body.face_edges(fid).iter().any(|&e| !on_box(body.vertex(body.edge(e).v0).p) || !on_box(body.vertex(body.edge(e).v1).p));
            if shell_id.is_some() && inner {
                shell_id
            } else {
                f.name.feature[1..].parse().ok()
            }
        })
        .collect();
    let mut tags = vec![vec![]; body.faces.len()];
    for (i, fid) in body.face_ids().enumerate() {
        if let Some(t) = tag[i] {
            tags[fid.idx()] = vec![(t, 0)];
        }
    }
    let sw = sw_features(&fs);
    let t = Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 };
    let out = rebuild_history(&t);
    let types = out.features.iter().map(|f| f.get_str("type").unwrap().to_string()).collect();
    Res { out, types }
}

fn assert_exact(r: &Res) {
    let o = &r.out;
    let steps: Vec<String> = o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect();
    assert!(o.fallback.is_none(), "fallback: {:?}\nsteps: {steps:#?}\nfeatures: {}", o.fallback, ck_json::to_string(&Json::Arr(o.features.clone())));
    let v = o.verify.as_ref().unwrap();
    assert!(v.exact, "not exact: rel {} area {} bbox {}\nsteps: {steps:#?}\nfeatures: {}", v.rel, v.area_rel, v.bbox_dev, ck_json::to_string(&Json::Arr(o.features.clone())));
    assert!(o.steps.iter().all(|s| s.status == Status::Rebuilt), "{steps:#?}");
}

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

#[test]
fn plain_box() {
    let r = run(&format!("[{BOX}]"));
    assert_exact(&r);
    assert_eq!(r.types, ["sketch", "extrude"]);
}

#[test]
fn boss_and_pocket_on_a_box() {
    // a round boss on the top, a rectangular blind pocket into the boss, a through slot in the base
    let src = format!(
        r#"[{BOX},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":5,"y":3}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":12}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":8}},
        {{"type":"sketch","id":"F5","plane":{{"kind":"face","id":"F4.C1"}},"points":[{{"id":"p1","x":2,"y":0}},{{"id":"p2","x":8,"y":0}},{{"id":"p3","x":8,"y":4}},{{"id":"p4","x":2,"y":4}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"line","p1":"p2","p2":"p3"}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}},
        {{"type":"extrude","id":"F6","op":"cut","sketch":"F5","end":"blind","depth":5}},
        {{"type":"sketch","id":"F7","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":-25,"y":-15}},{{"id":"p2","x":-15,"y":-15}},{{"id":"p3","x":-15,"y":15}},{{"id":"p4","x":-25,"y":15}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"line","p1":"p2","p2":"p3"}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}},
        {{"type":"extrude","id":"F8","op":"cut","sketch":"F7","end":"through","flip":true}}]"#
    );
    let r = run(&src);
    assert_exact(&r);
    assert_eq!(r.out.steps.len(), 4);
}

#[test]
fn fillet_chamfer_and_hole() {
    let src = format!(
        r#"[{BOX},
        {{"type":"fillet","id":"F3","radius":3,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[0,-20,10]}},{{"faces":["F2.C1","F2.S.e3"],"at":[0,20,10]}}]}},
        {{"type":"chamfer","id":"F4","chamferType":"equal","distance":2,"edges":[{{"faces":["F2.C0","F2.S.e2"],"at":[30,0,0]}}]}},
        {{"type":"sketch","id":"F5","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":5,"y":3}},{{"id":"k2","x":-10,"y":3}}],"entities":[{{"id":"pt1","type":"point","p":"k"}},{{"id":"pt2","type":"point","p":"k2"}}],"constraints":[]}},
        {{"type":"hole","id":"F6","sketch":"F5","kind":"simple","diameter":6,"end":"through"}},
        {{"type":"sketch","id":"F7","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":20,"y":-5}}],"entities":[{{"id":"pt1","type":"point","p":"k"}}],"constraints":[]}},
        {{"type":"hole","id":"F8","sketch":"F7","kind":"simple","diameter":4,"end":"blind","depth":6,"tip":true}}]"#
    );
    let r = run(&src);
    assert_exact(&r);
    assert!(r.types.iter().filter(|t| *t == "hole").count() == 2, "{:?}", r.types);
}

#[test]
fn shelled_box() {
    let src = format!(r#"[{BOX},{{"type":"shell","id":"F3","faces":["F2.C1"],"thickness":1.5}}]"#);
    let r = run(&src);
    assert_exact(&r);
    assert!(r.types.contains(&"shell".to_string()));
}

#[test]
fn oblique_extrusion() {
    // a boss on a frame plane that is no standard plane
    let src = r#"[{"type":"plane","id":"F1","mode":"frame","origin":[0,0,0],"xDir":[1,0,0],"yDir":[0,0.8,0.6]},
    {"type":"sketch","id":"F2","plane":{"kind":"feature","id":"F1"},"points":[{"id":"p1","x":-10,"y":-10},{"id":"p2","x":10,"y":-10},{"id":"p3","x":10,"y":10},{"id":"p4","x":-10,"y":10}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
    {"type":"extrude","id":"F3","op":"add","sketch":"F2","end":"blind","depth":7}]"#;
    let r = run(src);
    assert_exact(&r);
    assert_eq!(r.types, ["plane", "sketch", "extrude"]);
}

#[test]
fn unsupported_feature_falls_back() {
    // a loft is not read back: the file keeps its plain solid and says why
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":5},{"id":"p4","x":0,"y":5}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"},{"id":"e5","type":"line","p1":"p4","p2":"p1","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"e5","angle":360}]"#;
    let fs = feats(src);
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    let body = body.unwrap();
    let tags: Vec<Vec<(i64, i64)>> = body.faces.iter().map(|_| vec![(2, 0)]).collect();
    let sw = vec![SwFeature { id: 1, name: "Sketch1".into(), kind: "Sketch".into(), dims: vec![] }, SwFeature { id: 2, name: "Loft1".into(), kind: "Loft".into(), dims: vec![] }];
    let out = rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 });
    assert!(out.fallback.as_deref().unwrap().contains("Loft"), "{:?}", out.fallback);
    assert_eq!(out.steps[0].status, Status::Unsupported);
    assert!(out.body.is_none());
}

#[test]
fn through_cut_whose_mouth_a_later_chamfer_trimmed() {
    // the cut's walls end where the chamfer starts, short of the plate's top face: the cut must still start at the top
    // face (not at the wall's end, which would leave a ceiling over the hole and no edge for the chamfer to find)
    let src = format!(
        r#"[{BOX},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":5}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"through"}},
        {{"type":"chamfer","id":"F5","chamferType":"equal","distance":1.5,"edges":[{{"faces":["F2.C1","F4.S.e1"],"at":[5,0,10]}}]}}]"#
    );
    let r = run(&src);
    assert_exact(&r);
    let cut = r.out.features.iter().find(|f| f.get_str("op") == Some("cut")).expect("cut");
    assert_eq!(cut.get_str("end"), Some("through"));
}
