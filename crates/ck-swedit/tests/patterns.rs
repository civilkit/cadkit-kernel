//! Patterns and mirrors: round trips on parts made in-test. A part document with a pattern / mirror feature is
//! regenerated, its faces are tagged as a SolidWorks file tags them (a copy's faces carry the pattern's id, the
//! seed's faces keep the seed's), and the history read back must reproduce the body.

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

fn dim(name: &str, v: f64) -> SwDim {
    SwDim { name: name.into(), text: v.to_string(), value: Some(v) }
}

/// SolidWorks-style feature list for our document (id = the number of `F<n>`). Pattern spacings are listed rounded
/// (as the file does), counts exactly.
fn sw_features(fs: &[Json]) -> Vec<SwFeature> {
    let mut out = vec![];
    for f in fs {
        let id: i64 = f.get_str("id").unwrap()[1..].parse().unwrap();
        let ty = f.get_str("type").unwrap();
        let cut = f.get_str("op") == Some("cut");
        let num = |k: &str| f.get_f64(k).unwrap();
        let (kind, name, dims): (&str, String, Vec<SwDim>) = match ty {
            "sketch" => ("Sketch", format!("Sketch{id}"), vec![]),
            "plane" => ("Plane", format!("Plane{id}"), vec![]),
            "extrude" if cut => ("Extrusion", format!("Cut-Extrude{id}"), f.get_f64("depth").map(|d| vec![dim("D1", d)]).unwrap_or_default()),
            "extrude" => ("Boss-Extrude", format!("Boss-Extrude{id}"), f.get_f64("depth").map(|d| vec![dim("D1", d)]).unwrap_or_default()),
            "hole" => ("HoleWizard", format!("Hole{id}"), vec![dim("Hole Dia.", num("diameter"))]),
            "lpattern" if f.get_f64("count2").is_some() => ("LPattern", format!("LPattern{id}"), vec![dim("D1", num("count")), dim("D2", num("count2")), dim("D3", num("spacing").round()), dim("D4", num("spacing2").round())]),
            "lpattern" => ("LPattern", format!("LPattern{id}"), vec![dim("D1", num("count")), dim("D3", num("spacing").round())]),
            "cpattern" => ("CirPattern", format!("CirPattern{id}"), vec![dim("D1", num("count")), dim("D2", num("angle"))]),
            "mirror" => ("Mirror", format!("Mirror{id}"), vec![]),
            t => panic!("{t}"),
        };
        out.push(SwFeature { id, name, kind: kind.into(), dims });
    }
    out
}

fn run_with(src: &str, edit: impl Fn(&mut Vec<SwFeature>)) -> Outcome {
    let fs = feats(src);
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        let name = &body.face(fid).name;
        // a copy (`@F7.2`) belongs to the pattern, the rest to the feature that made it
        let owner = match name.suffix.rsplit('@').next() {
            Some(s) if !s.is_empty() => s.split('.').next().unwrap().to_string(),
            _ => name.feature.clone(),
        };
        if let Ok(t) = owner[1..].parse::<i64>() {
            tags[fid.idx()] = vec![(t, 0)];
        }
    }
    let mut sw = sw_features(&fs);
    edit(&mut sw);
    let t = Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 };
    rebuild_history(&t)
}

fn run(src: &str) -> Outcome {
    run_with(src, |_| {})
}

fn steps(o: &Outcome) -> Vec<String> {
    o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect()
}

fn assert_exact(o: &Outcome) {
    assert!(o.fallback.is_none(), "fallback: {:?}\nsteps: {:#?}\nfeatures: {}", o.fallback, steps(o), ck_json::to_string(&Json::Arr(o.features.clone())));
    let v = o.verify.as_ref().unwrap();
    assert!(v.exact, "not exact: rel {} area {}\nsteps: {:#?}", v.rel, v.area_rel, steps(o));
    assert!(o.steps.iter().all(|s| s.status == Status::Rebuilt), "{:#?}", steps(o));
}

fn types(o: &Outcome) -> Vec<String> {
    o.features.iter().map(|f| f.get_str("type").unwrap().to_string()).collect()
}

fn pattern_of<'a>(o: &'a Outcome, ty: &str) -> &'a Json {
    o.features.iter().find(|f| f.get_str("type") == Some(ty)).unwrap_or_else(|| panic!("no {ty} in {:?}", types(o)))
}

/// 80 x 80 x 10 plate on the top plane.
const PLATE: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-40,"y":-40},{"id":"p2","x":40,"y":-40},{"id":"p3","x":40,"y":40},{"id":"p4","x":-40,"y":40}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

#[test]
fn hole_array() {
    // a through hole, four in a row along +x
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":-24,"y":-10}}],"entities":[{{"id":"pt1","type":"point","p":"k"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}},
        {{"type":"lpattern","id":"F5","seeds":["F4"],"dir":{{"kind":"axis","axis":"x"}},"count":4,"spacing":16}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
    assert_eq!(types(&o).iter().filter(|t| *t == "lpattern").count(), 1, "{:?}", types(&o));
    let p = pattern_of(&o, "lpattern");
    assert_eq!(p.get_f64("count"), Some(4.0));
    assert!((p.get_f64("spacing").unwrap() - 16.0).abs() < 1e-6);
}

#[test]
fn boss_pattern_reversed_direction() {
    // a round boss, three copies marching along -z (the plate is extruded along +y: the top plane is y up)
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":10,"y":-30}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":4}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":6}},
        {{"type":"lpattern","id":"F5","seeds":["F4"],"dir":{{"kind":"axis","axis":"z"}},"count":3,"spacing":20,"reverse":true}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
    let p = pattern_of(&o, "lpattern");
    assert_eq!(p.get("reverse").and_then(|v| v.as_bool()), Some(true));
}

#[test]
fn grid_of_pockets_two_directions() {
    // a square pocket, 3 x 2 copies (counts differ, so the assignment of the counts to the directions matters)
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"p1","x":-32,"y":-30}},{{"id":"p2","x":-26,"y":-30}},{{"id":"p3","x":-26,"y":-24}},{{"id":"p4","x":-32,"y":-24}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"line","p1":"p2","p2":"p3"}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":4}},
        {{"type":"lpattern","id":"F5","seeds":["F4"],"dir":{{"kind":"axis","axis":"x"}},"count":3,"spacing":25,"dir2":{{"kind":"axis","axis":"z"}},"count2":2,"spacing2":30,"reverse2":true}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
    let p = pattern_of(&o, "lpattern");
    assert_eq!(p.get_f64("count"), Some(3.0));
    assert_eq!(p.get_f64("count2"), Some(2.0));
}

#[test]
fn circular_pattern_of_a_boss() {
    // four copies of a boss about the vertical (extrusion) axis
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":20,"y":0}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":3}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":5}},
        {{"type":"cpattern","id":"F5","seeds":["F4"],"axis":{{"kind":"axis","axis":"y"}},"count":4,"angle":360}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
    let p = pattern_of(&o, "cpattern");
    assert_eq!(p.get_f64("count"), Some(4.0));
    assert!((p.get_f64("angle").unwrap() - 90.0).abs() < 1e-6, "{}", ck_json::to_string(p));
}

#[test]
fn mirrored_pocket() {
    // a rectangular pocket mirrored in the plane x = 0
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"p1","x":10,"y":-8}},{{"id":"p2","x":22,"y":-8}},{{"id":"p3","x":22,"y":5}},{{"id":"p4","x":10,"y":5}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"line","p1":"p2","p2":"p3"}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":4}},
        {{"type":"mirror","id":"F5","seeds":["F4"],"plane":{{"kind":"std","id":"right"}},"body":false}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
    assert!(types(&o).contains(&"mirror".to_string()), "{:?}", types(&o));
}

#[test]
fn mirrored_boss_in_an_offset_plane() {
    // a boss mirrored in the plane x = 10: the plane is read back as an offset plane
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":14,"y":3}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":3}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":5}},
        {{"type":"plane","id":"F5","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":10}},
        {{"type":"mirror","id":"F6","seeds":["F4"],"plane":{{"kind":"feature","id":"F5"}},"body":false}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
}

#[test]
fn wrong_count_falls_back() {
    // the file says 3 copies but the body has 4: no rebuild may be offered
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":-20,"y":20}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":4}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":6}},
        {{"type":"lpattern","id":"F5","seeds":["F4"],"dir":{{"kind":"axis","axis":"x"}},"count":4,"spacing":15}}]"#
    );
    let o = run_with(&src, |sw| {
        for f in sw.iter_mut().filter(|f| f.kind == "LPattern") {
            f.dims[0] = dim("D1", 3.0);
        }
    });
    assert!(o.fallback.is_some(), "a wrong count must not rebuild");
    assert!(!o.exact());
}

#[test]
fn pattern_faces_that_are_no_copy_fall_back() {
    // a pattern whose faces do not come from any earlier feature: claim a pocket's id is a pattern
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":-20,"y":20}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":4}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":6}},
        {{"type":"sketch","id":"F5","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":20,"y":-20}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":7}}],"constraints":[]}},
        {{"type":"extrude","id":"F6","op":"add","sketch":"F5","end":"blind","depth":3}}]"#
    );
    let o = run_with(&src, |sw| {
        let last = sw.last_mut().unwrap();
        last.kind = "LPattern".into();
        last.dims = vec![dim("D1", 2.0), dim("D3", 40.0)];
    });
    assert!(o.fallback.is_some());
    assert!(o.steps.iter().any(|s| s.status == Status::Unsupported && s.kind == "LPattern"), "{:#?}", steps(&o));
}

#[test]
fn circular_pattern_about_an_axis_off_the_origin() {
    // four copies of a boss about a parallel axis through (10, ., 0): not a world axis through the origin
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":20,"y":0}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":3}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":5}},
        {{"type":"cpattern","id":"F5","seeds":["F4"],"axis":{{"kind":"axis","point":[10,0,0],"dir":[0,1,0]}},"count":4,"angle":90,"equal":false}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
    let p = pattern_of(&o, "cpattern");
    assert_eq!(p.get_f64("count"), Some(4.0));
    assert!((p.get_f64("angle").unwrap() - 90.0).abs() < 1e-6, "{}", ck_json::to_string(p));
    let ax = p.get("axis").unwrap();
    assert_eq!(ax.get_str("kind"), Some("axis"));
    let pt: Vec<f64> = ax.get("point").unwrap().as_arr().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    assert!((pt[0] - 10.0).abs() < 1e-6 && pt[2].abs() < 1e-6, "{pt:?}");
}

#[test]
fn linear_pattern_along_a_diagonal() {
    // three holes marching (6, 0, 8) apart: a direction that is no world axis
    let src = format!(
        r#"[{PLATE},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":-24,"y":-10}}],"entities":[{{"id":"pt1","type":"point","p":"k"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}},
        {{"type":"lpattern","id":"F5","seeds":["F4"],"dir":{{"kind":"vector","dir":[3,0,4]}},"count":3,"spacing":10}}]"#
    );
    let o = run(&src);
    assert_exact(&o);
    let p = pattern_of(&o, "lpattern");
    assert_eq!(p.get_f64("count"), Some(3.0));
    assert!((p.get_f64("spacing").unwrap() - 10.0).abs() < 1e-6, "{}", ck_json::to_string(p));
    assert_eq!(p.get("dir").unwrap().get_str("kind"), Some("vector"));
}
