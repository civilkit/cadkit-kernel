//! Recognisers on hand-built bodies: regenerate a feature program, recognise the result as a history-free
//! body, rebuild the recognised features and compare.
use ck_json::Json;
use ck_recog::{rebuild, recognize};

fn feats(src: &str) -> Vec<Json> {
    match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    }
}

const RECT: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

struct Res {
    types: Vec<String>,
    strategy: &'static str,
    exact: bool,
    close: bool,
    rel: f64,
    unexplained: usize,
    debug: String,
}

fn run(src: &str) -> Res {
    let orig_feats = feats(src);
    let (body, errs) = rebuild::rebuild(&orig_feats).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let rec = recognize(&body);
    let types: Vec<String> = rec.features.iter().map(|f| f.get_str("type").unwrap().to_string()).collect();
    let (nb, errs) = rebuild::rebuild(&rec.features).unwrap();
    assert!(errs.is_empty(), "rebuild errors {errs:?} in {}", ck_json::to_string(&Json::Arr(rec.features.clone())));
    let nb = nb.expect("rebuilt body");
    let v = rebuild::verify(&body, &nb);
    let debug = ck_json::to_string(&ck_recog::report(&body, &rec));
    Res { debug, types, strategy: rec.strategy, exact: v.exact, close: v.close, rel: v.rel, unexplained: rec.unexplained.len() }
}

fn has(r: &Res, t: &str) -> bool {
    r.types.iter().any(|x| x == t)
}

#[test]
fn plain_box() {
    let r = run(&format!("[{RECT}]"));
    assert_eq!(r.strategy, "prism");
    assert!(r.exact, "rel {}", r.rel);
    assert_eq!(r.unexplained, 0);
}

#[test]
fn box_with_through_hole() {
    let r = run(&format!(
        r#"[{RECT},{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":5,"y":3}}],"entities":[{{"id":"pt1","type":"point","p":"k"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}}]"#
    ));
    assert!(has(&r, "hole"));
    assert!(r.exact, "rel {}", r.rel);
    assert_eq!(r.unexplained, 0);
}

#[test]
fn box_with_blind_hole() {
    let r = run(&format!(
        r#"[{RECT},{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":5,"y":3}}],"entities":[{{"id":"pt1","type":"point","p":"k"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"blind","depth":4}}]"#
    ));
    assert!(has(&r, "hole"));
    assert!(r.close, "rel {} {}", r.rel, r.debug);
}

#[test]
fn box_with_counterbore() {
    let r = run(&format!(
        r#"[{RECT},{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":0,"y":0}}],"entities":[{{"id":"pt1","type":"point","p":"k"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}},
        {{"type":"sketch","id":"F5","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":0,"y":0}}],"entities":[{{"id":"c1","type":"circle","c":"k","r":6}}],"constraints":[]}},
        {{"type":"extrude","id":"F6","op":"cut","sketch":"F5","end":"blind","depth":4}}]"#
    ));
    assert!(has(&r, "hole"));
    assert!(r.close, "rel {} {}", r.rel, r.debug);
}

#[test]
fn plate_with_fillets() {
    let r = run(&format!(
        r#"[{RECT},{{"type":"fillet","id":"F3","radius":2,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[0,-20,10]}},{{"faces":["F2.C1","F2.S.e3"],"at":[0,20,10]}}]}}]"#
    ));
    assert!(has(&r, "fillet"), "{}", r.debug);
    assert!(r.close, "rel {} {}", r.rel, r.debug);
}

#[test]
fn plate_with_chamfer() {
    let r = run(&format!(
        r#"[{RECT},{{"type":"chamfer","id":"F3","chamferType":"equal","distance":2,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[0,-20,10]}}]}}]"#
    ));
    assert!(has(&r, "chamfer"), "{}", r.debug);
    assert!(r.close, "rel {} {}", r.rel, r.debug);
}

#[test]
fn l_bracket() {
    let r = run(
        r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":10},{"id":"p4","x":10,"y":10},{"id":"p5","x":10,"y":30},{"id":"p6","x":0,"y":30}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p5"},{"id":"e5","type":"line","p1":"p5","p2":"p6"},{"id":"e6","type":"line","p1":"p6","p2":"p1"}],"constraints":[]},
        {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":15}]"#,
    );
    assert_eq!(r.strategy, "prism");
    assert!(r.exact, "rel {}", r.rel);
}

#[test]
fn cylinder_with_groove_is_a_revolve() {
    let r = run(
        r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":12},{"id":"p4","x":8,"y":12},{"id":"p5","x":8,"y":18},{"id":"p6","x":10,"y":18},{"id":"p7","x":10,"y":30},{"id":"p8","x":0,"y":30},{"id":"a1","x":0,"y":0},{"id":"a2","x":0,"y":30}],
        "entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p5"},{"id":"e5","type":"line","p1":"p5","p2":"p6"},{"id":"e6","type":"line","p1":"p6","p2":"p7"},{"id":"e7","type":"line","p1":"p7","p2":"p8"},{"id":"e8","type":"line","p1":"p8","p2":"p1"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
        {"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}]"#,
    );
    assert_eq!(r.strategy, "revolve");
    assert!(r.exact, "rel {}", r.rel);
}

#[test]
fn bolt_circle_of_six_holes_is_a_pattern() {
    let r = run(
        r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-40,"y":-40},{"id":"p2","x":40,"y":-40},{"id":"p3","x":40,"y":40},{"id":"p4","x":-40,"y":40}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
        {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
        {"type":"sketch","id":"F3","plane":{"kind":"face","id":"F2.C1"},"points":[{"id":"k","x":25,"y":0}],"entities":[{"id":"pt1","type":"point","p":"k"}],"constraints":[]},
        {"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":5,"end":"through"},
        {"type":"cpattern","id":"F7","seeds":["F4"],"axis":{"kind":"axis","axis":"y"},"count":6,"angle":360,"equal":true}]"#,
    );
    assert!(has(&r, "cpattern"), "{:?}", r.types);
    assert!(r.close, "rel {} {}", r.rel, r.debug);
}
