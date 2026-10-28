//! `drawing` on a small multi-body part (no oracle: the checks are geometric facts of the views).
//!
//! `fixtures/drawing_multibody.json`, by feature id: E1 a 24 mm plate (z 0..3), EH an off-centre hole, E2 a boss on
//! it, E3 a 3.1 mm bore, E4/E5 a second bored plate (z 8.1..11.1), R1 a 2 mm shaft (a revolve, z 0.3..32) through
//! the bores as a separate body, R2 a chamfered ring (z 11.2..15.2, 0.1 mm clear of the second plate).
//! The `top` view looks along the part's Y axis; its v axis is -z.

use ck_json::Json;

fn call(msg: &Json) -> Json {
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg.clone())]));
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn regen(part: Json) {
    ck_wasm::engine::install();
    let r = call(&Json::Obj(vec![
        ("type".into(), Json::Str("regen".into())),
        ("tag".into(), Json::Str("main".into())),
        ("part".into(), part),
    ]));
    let errs = r.get("result").and_then(|r| r.get("errors")).and_then(|e| e.as_arr().map(|a| a.len()));
    assert_eq!(errs, Some(0), "regen failed: {}", ck_json::to_string(&r));
}

/// (kind, visible, points) of every line of the first view of a `drawing` reply, or the view's error.
fn view(v: Json) -> Result<Vec<(String, bool, Vec<f64>)>, String> {
    let r = call(&Json::Obj(vec![
        ("type".into(), Json::Str("drawing".into())),
        ("tag".into(), Json::Str("main".into())),
        ("views".into(), Json::Arr(vec![v])),
        ("hidden".into(), Json::Bool(true)),
    ]));
    let vw = r.get("result").and_then(|r| r.get_arr("views")).and_then(|a| a.first()).cloned().expect("a view");
    if let Some(e) = vw.get_str("error") {
        return Err(e.to_string());
    }
    Ok(vw
        .get_arr("lines")
        .unwrap()
        .iter()
        .map(|l| {
            let p = l.get_arr("p").unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            (l.get_str("k").unwrap().to_string(), matches!(l.get("v"), Some(Json::Bool(true))), p)
        })
        .collect())
}

fn named(n: &str) -> Json {
    Json::Obj(vec![("name".into(), Json::Str(n.into()))])
}

fn fixture() -> Json {
    ck_json::parse(include_str!("fixtures/drawing_multibody.json")).unwrap()
}

/// The silhouette line x = +1 of the shaft: the shaft is hidden behind the second plate, the ring and the boss
/// and plainly visible in between and beyond; the line must be cut into those stretches, not drawn as one piece
/// with the class of its midpoint (which left the whole shaft invisible in the motor models).
#[test]
fn shaft_silhouette_is_cut_where_other_bodies_cover_it() {
    regen(fixture());
    let lines = view(named("top")).unwrap();
    let shaft: Vec<_> = lines.iter().filter(|(k, _, p)| k == "o" && p.len() == 4 && (p[0] - 1.0).abs() < 1e-6 && (p[2] - 1.0).abs() < 1e-6).collect();
    // the visible stretch out of the free end: from v = -31.8 up to the ring
    let tip = shaft.iter().find(|(_, v, p)| *v && p[1].min(p[3]) < -31.79).expect("the shaft's free end stretch");
    assert!(p_len(&tip.2) > 15.0, "{:?}", tip);
    // covered stretches are present as hidden pieces, and nothing spans from the tip to the plates
    assert!(shaft.iter().any(|(_, v, p)| !*v && p[1].max(p[3]) <= -8.0 && p[1].min(p[3]) >= -15.3), "{shaft:?}");
    assert!(!shaft.iter().any(|(_, _, p)| p[1].min(p[3]) < -31.0 && p[1].max(p[3]) > -10.0), "{shaft:?}");
}

fn p_len(p: &[f64]) -> f64 {
    (p[3] - p[1]).hypot(p[2] - p[0])
}

/// The far end of the chamfered ring (circle r 7 at v = -15.2) and its two other rims, seen edge-on, are plain
/// visible lines: one piece each, spanning the diameter. A circle edge-on was tested at its centre's depth, so
/// the ring's own inner wall (in front of that point) hid it.
#[test]
fn edge_on_ring_circles_are_single_visible_lines() {
    regen(fixture());
    let lines = view(named("top")).unwrap();
    for (v, r) in [(-15.2, 7.0), (-14.2, 8.0), (-11.2, 8.0)] {
        let at: Vec<_> = lines.iter().filter(|(k, _, p)| k == "e" && p.len() == 4 && (p[1] - v).abs() < 1e-6 && (p[3] - v).abs() < 1e-6 && p[0].abs() > 3.0).collect();
        assert!(!at.is_empty(), "no line at v = {v}");
        assert!(at.iter().all(|(_, vis, _)| *vis), "hidden piece at v = {v}: {at:?}");
        assert!(at.iter().any(|(_, _, p)| (p[0].abs() - r).abs() < 1e-6 && (p[2].abs() - r).abs() < 1e-6), "no full-diameter piece at v = {v}: {at:?}");
    }
}

/// A section whose plane point lies far from a revolved body (the kit sends a point on the axis through the
/// origin) was refused as cutNoEffect: the cutting tool was sized from the flat vertex box.
#[test]
fn section_of_a_revolve_far_from_the_plane_point() {
    let ring = |y0: f64| {
        let p = [[3.0, y0], [12.0, y0], [12.0, y0 - 3.5], [3.0, y0 - 3.5], [0.0, 0.0], [0.0, -1.0]];
        let pts: Vec<String> = p.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, q[0], q[1])).collect();
        let ents = r#"{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"},{"id":"ax","type":"line","p1":"p5","p2":"p6","construction":true}"#;
        ck_json::parse(&format!(
            r#"{{"format":"ck-part","version":1,"features":[{{"type":"plane","id":"Rp","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":0}},{{"type":"sketch","id":"Rs","plane":{{"kind":"feature","id":"Rp"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{}],"entities":[{}],"constraints":[]}},{{"type":"revolve","id":"R","op":"add","sketch":"Rs","axis":"ax","angle":360}}]}}"#,
            pts.join(","),
            ents
        ))
        .unwrap()
    };
    let sect = || {
        let f = |a: [f64; 3]| Json::Arr(a.iter().map(|&x| Json::Num(x)).collect());
        Json::Obj(vec![
            ("x".into(), f([0.0, 0.0, -1.0])),
            ("y".into(), f([0.0, 1.0, 0.0])),
            ("section".into(), Json::Obj(vec![("point".into(), f([0.0, 0.0, 0.0])), ("hatch".into(), Json::Num(3.0))])),
        ])
    };
    for y0 in [-5.0, -20.0] {
        regen(ring(y0));
        let lines = view(sect()).unwrap_or_else(|e| panic!("section of the ring at y {y0}: {e}"));
        assert!(lines.iter().any(|(k, _, _)| k == "h"), "no hatch for the ring at y {y0}");
    }
}
