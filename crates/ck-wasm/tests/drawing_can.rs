//! `drawing` of an open can (a body of revolution with a chamfered closed end), side view: the silhouette generators run the
//! full length of the can up to the end, and the chamfer's cone gives its own silhouette piece (K20, "silhouette pieces are
//! missing at a can end").

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
    ck_json::parse(include_str!("fixtures/drawing_can.json")).unwrap()
}

fn spans(p: &[f64], u: f64, v0: f64, v1: f64) -> bool {
    p.len() == 4 && (p[0] - u).abs() < 1e-6 && (p[2] - u).abs() < 1e-6 && (p[1].min(p[3]) - v0).abs() < 1e-6 && (p[1].max(p[3]) - v1).abs() < 1e-6
}

/// The outer wall of the can (r 12, v 0 .. -18.5) gives its two silhouette generators as single visible lines over the
/// whole length, and the chamfer at the closed end (r 12 -> 10.5 over 1.5) gives its two slanted outline pieces and the
/// edge-on end circle: no piece is missing at the can end.
#[test]
fn can_outline_is_complete_up_to_the_end() {
    regen(fixture());
    let lines = view(named("top")).unwrap();
    for u in [-12.0, 12.0] {
        assert!(lines.iter().any(|(k, vis, p)| k == "o" && *vis && spans(p, u, -18.5, 0.0)), "wall generator at x = {u}: {lines:?}");
        let (e, f): (f64, f64) = if u > 0.0 { (10.5, 12.0) } else { (-10.5, -12.0) };
        assert!(
            lines.iter().any(|(k, vis, p)| k == "o" && *vis && p.len() == 4 && (p[0].min(p[2]) - e.min(f)).abs() < 1e-6 && (p[0].max(p[2]) - e.max(f)).abs() < 1e-6 && (p[1].min(p[3]) + 20.0).abs() < 1e-6 && (p[1].max(p[3]) + 18.5).abs() < 1e-6),
            "chamfer outline at x = {u}: {lines:?}"
        );
    }
    // the closed end's own circle (r 10.5, v = -20) and the chamfer's start circle (r 12, v = -18.5) are visible diameters
    for (v, r) in [(-20.0, 10.5), (-18.5, 12.0)] {
        assert!(lines.iter().any(|(k, vis, p)| k == "e" && *vis && p.len() == 4 && (p[1] - v).abs() < 1e-6 && (p[3] - v).abs() < 1e-6 && (p[0].abs() - r).abs() < 1e-6), "circle at v = {v}");
    }
}

/// The can's inner floor edge (circle r 11 at v = -19, inside the wall) is drawn as a visible line in the side view.
/// It lies on a seam of the chamfer cone's display mesh (a mesh edge exactly along the line, which the covering test
/// treats as "not strictly inside either triangle"; `covered_seam` tests both sides of it), so neither the front nor the back half of the circle counts as
/// covered although the outer wall is in front of both.
#[test]
fn can_inner_floor_circle_is_hidden_in_the_side_view() {
    regen(fixture());
    let lines = view(named("top")).unwrap();
    let floor: Vec<_> = lines.iter().filter(|(k, _, p)| k == "e" && p.len() == 4 && (p[1] + 19.0).abs() < 1e-6 && (p[3] + 19.0).abs() < 1e-6).collect();
    assert!(!floor.is_empty());
    assert!(floor.iter().all(|(_, vis, _)| !*vis), "{floor:?}");
}
