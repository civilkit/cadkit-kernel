//! `section` / `sections` / `probe` through the full server path (hand-computed areas).

use ck_json::Json;
use std::f64::consts::PI;

fn raw(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn call(msg: &str) -> Json {
    let j = raw(msg);
    assert_eq!(j.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&j));
    j.get("result").unwrap().clone()
}

fn call_err(msg: &str) -> String {
    let j = raw(msg);
    assert_eq!(j.get("ok"), Some(&Json::Bool(false)));
    j.get_str("error").unwrap().to_string()
}

fn rect_sketch(x0: f64, y0: f64, x1: f64, y1: f64, pfx: &str) -> (String, String) {
    let p = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    let pts: Vec<String> = p.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"{pfx}{}","x":{x},"y":{y}}}"#, i + 1)).collect();
    let ents: Vec<String> = (0..4)
        .map(|i| format!(r#"{{"type":"line","id":"{pfx}e{}","p1":"{pfx}{}","p2":"{pfx}{}"}}"#, i + 1, i + 1, (i + 1) % 4 + 1))
        .collect();
    (pts.join(","), ents.join(","))
}

fn sketch(plane: &str, pts: &str, ents: &str) -> String {
    format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"{plane}"}},"points":[{pts}],"entities":[{ents}],"constraints":[]}}"#)
}

fn extrude(depth: f64) -> String {
    format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth}}}"#)
}

fn part(features: &[String]) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","))
}

fn load(tag: &str, p: &str) {
    call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{p}}}"#));
}

fn box_part() -> String {
    let (p, e) = rect_sketch(0.0, 0.0, 40.0, 30.0, "p");
    part(&[sketch("top", &p, &e), extrude(20.0)])
}

fn cyl_part() -> String {
    part(&[sketch("top", r#"{"id":"c","x":0,"y":0}"#, r#"{"type":"circle","id":"e1","c":"c","r":10}"#), extrude(20.0)])
}

fn revolve_part(circle: bool) -> String {
    let ax_p = r#"{"id":"a1","x":0,"y":0},{"id":"a2","x":0,"y":10}"#;
    let ax_e = r#"{"type":"line","id":"ax","p1":"a1","p2":"a2","construction":true}"#;
    let (pts, ents) = if circle {
        (format!(r#"{{"id":"c","x":20,"y":0}},{ax_p}"#), format!(r#"{{"type":"circle","id":"e1","c":"c","r":5}},{ax_e}"#))
    } else {
        (
            format!(r#"{{"id":"c","x":0,"y":0}},{{"id":"s","x":0,"y":-10}},{{"id":"n","x":0,"y":10}},{ax_p}"#),
            format!(r#"{{"type":"arc","id":"e1","c":"c","p1":"s","p2":"n"}},{{"type":"line","id":"e2","p1":"n","p2":"s"}},{ax_e}"#),
        )
    };
    part(&[
        sketch("front", &pts, &ents),
        r#"{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}"#.to_string(),
    ])
}

fn plane_msg(tag: &str, o: [f64; 3], n: [f64; 3]) -> String {
    format!(r#"{{"type":"section","tag":"{tag}","plane":{{"origin":[{},{},{}],"normal":[{},{},{}]}}}}"#, o[0], o[1], o[2], n[0], n[1], n[2])
}

fn area(r: &Json) -> f64 {
    r.get_f64("area").unwrap()
}

fn bounds(tag: &str, axis: &str) -> (f64, f64) {
    let r = call(&format!(r#"{{"type":"sections","tag":"{tag}","axis":"{axis}","n":1}}"#));
    let a = r.get_arr("range").unwrap();
    (a[0].as_f64().unwrap(), a[1].as_f64().unwrap())
}

/// Mid-height along the extrusion axis: a top-plane sketch extrudes along world +Y.
fn mid_y(tag: &str) -> f64 {
    let (a, b) = bounds(tag, "y");
    (a + b) / 2.0
}

fn count(r: &Json, kind: &str) -> usize {
    r.get_arr("loops").unwrap().iter().filter(|l| l.get_str("kind") == Some(kind)).count()
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-12)
}

#[test]
fn box_section_area() {
    load("b", &box_part());
    let y = mid_y("b");
    let r = call(&plane_msg("b", [0.0, y, 0.0], [0.0, 1.0, 0.0]));
    assert!(rel(area(&r), 1200.0) < 1e-9, "{}", area(&r));
    assert_eq!(count(&r, "outer"), 1);
    let edges = r.get_arr("loops").unwrap()[0].get_arr("edges").unwrap();
    assert_eq!(edges.len(), 4);
    assert!(edges.iter().all(|e| e.get_str("type") == Some("line") && e.get_str("face").is_some()));
    // the x-normal section through the middle is 30 x 20
    let r = call(&plane_msg("b", [20.0, y, 0.0], [1.0, 0.0, 0.0]));
    assert!(rel(area(&r), 600.0) < 1e-9, "{}", area(&r));
}

#[test]
fn box_plane_through_a_face_and_outside() {
    load("b2", &box_part());
    let (lo, hi) = bounds("b2", "y");
    for y in [lo, hi] {
        let r = call(&plane_msg("b2", [0.0, y, 0.0], [0.0, 1.0, 0.0]));
        let a = area(&r);
        assert!(a.abs() < 1e-9 || rel(a, 1200.0) < 1e-9, "y={y} area {a}");
    }
    let r = call(&plane_msg("b2", [0.0, hi + 5.0, 0.0], [0.0, 1.0, 0.0]));
    assert_eq!(area(&r), 0.0);
    assert_eq!(r.get_arr("loops").unwrap().len(), 0);
    let r = call(&plane_msg("b2", [100.0, 0.0, 0.0], [1.0, 0.0, 0.0]));
    assert_eq!(r.get_arr("loops").unwrap().len(), 0);
}

#[test]
fn cylinder_section_is_an_exact_arc() {
    load("c", &cyl_part());
    let y = mid_y("c");
    let r = call(&plane_msg("c", [0.0, y, 0.0], [0.0, 1.0, 0.0]));
    assert!(rel(area(&r), 100.0 * PI) < 1e-9, "{}", area(&r));
    let l = &r.get_arr("loops").unwrap()[0];
    let e = &l.get_arr("edges").unwrap()[0];
    assert_eq!(e.get_str("type"), Some("arc"));
    assert!((e.get_f64("radius").unwrap() - 10.0).abs() < 1e-9);
    // the axial section is 20 x 20
    let r = call(&plane_msg("c", [0.0, y, 0.0], [0.0, 0.0, 1.0]));
    assert!(rel(area(&r), 400.0) < 1e-4, "{}", area(&r));
    // an oblique cut is an ellipse: pi * a * b, b = 10, a = 10 / cos(tilt) with the normal 30 degrees off the axis
    let n = [0.5, 0.75f64.sqrt(), 0.0];
    let r = call(&plane_msg("c", [0.0, y, 0.0], n));
    let want = PI * 10.0 * 10.0 / 0.75f64.sqrt();
    assert!(rel(area(&r), want) < 1e-4, "{} vs {want}", area(&r));
}

#[test]
fn sphere_and_torus() {
    load("s", &revolve_part(false));
    let r = call(&plane_msg("s", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    assert!(rel(area(&r), 100.0 * PI) < 1e-6, "{}", area(&r));
    let r = call(&plane_msg("s", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]));
    assert!(rel(area(&r), 100.0 * PI) < 1e-6, "{}", area(&r));
    // off-centre: circle of radius sqrt(100 - 36)
    let r = call(&plane_msg("s", [0.0, 6.0, 0.0], [0.0, 1.0, 0.0]));
    assert!(rel(area(&r), 64.0 * PI) < 1e-6, "{}", area(&r));
    // tangent plane: nothing (or a vanishing sliver)
    let r = call(&plane_msg("s", [0.0, 10.0, 0.0], [0.0, 1.0, 0.0]));
    assert!(area(&r).abs() < 1e-3, "{}", area(&r));
    load("t", &revolve_part(true));
    // torus R=20 r=5 about +Y: the plane z=0 through the axis gives two discs
    let r = call(&plane_msg("t", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    assert!(rel(area(&r), 2.0 * PI * 25.0) < 1e-6, "{}", area(&r));
    assert_eq!(count(&r, "outer"), 2);
    // the equatorial plane perpendicular to the axis gives an annulus: outer R+r, inner R-r
    let r = call(&plane_msg("t", [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]));
    assert!(rel(area(&r), PI * (625.0 - 225.0)) < 1e-6, "{}", area(&r));
    assert_eq!((count(&r, "outer"), count(&r, "inner")), (1, 1));
}

#[test]
fn plate_with_hole_and_two_lumps() {
    let (p, e) = rect_sketch(0.0, 0.0, 40.0, 20.0, "p");
    let pts = format!(r#"{p},{{"id":"c5","x":30,"y":10}}"#);
    let ents = format!(r#"{e},{{"type":"circle","id":"e5","c":"c5","r":4}}"#);
    load("h", &part(&[sketch("top", &pts, &ents), extrude(10.0)]));
    let y = mid_y("h");
    let r = call(&plane_msg("h", [0.0, y, 0.0], [0.0, 1.0, 0.0]));
    assert!(rel(area(&r), 800.0 - 16.0 * PI) < 1e-9, "{}", area(&r));
    assert_eq!((count(&r, "outer"), count(&r, "inner")), (1, 1));
    let (pa, ea) = rect_sketch(0.0, 0.0, 10.0, 10.0, "a");
    let (pb, eb) = rect_sketch(20.0, 0.0, 30.0, 10.0, "b");
    load("two", &part(&[sketch("top", &format!("{pa},{pb}"), &format!("{ea},{eb}")), extrude(5.0)]));
    let y = mid_y("two");
    let r = call(&plane_msg("two", [0.0, y, 0.0], [0.0, 1.0, 0.0]));
    assert!(rel(area(&r), 200.0) < 1e-9);
    assert_eq!(count(&r, "outer"), 2);
}

#[test]
fn sections_volume_matches_mass_props() {
    for (tag, p) in [("vb", box_part()), ("vc", cyl_part()), ("vs", revolve_part(false)), ("vt", revolve_part(true))] {
        load(tag, &p);
        for axis in ["x", "y", "z"] {
            let r = call(&format!(r#"{{"type":"sections","tag":"{tag}","axis":"{axis}","n":41}}"#));
            let err = r.get_f64("volumeError").unwrap();
            // along the extrusion / revolution axis (y) 1e-3; across a curved profile the midpoint rule is coarser
            let tol = if axis == "y" && tag != "vt" { 1e-3 } else { 5e-3 };
            assert!(err < tol, "{tag} {axis}: volumeError {err}");
            assert_eq!(r.get_arr("positions").unwrap().len(), 41);
        }
    }
    load("vo", &box_part());
    let r = call(r#"{"type":"sections","tag":"vo","axis":[1,1,0],"n":21}"#);
    assert!(r.get_f64("volumeError").unwrap() < 2e-3, "{}", ck_json::to_string(&r));
    let r = call(r#"{"type":"sections","tag":"vo","axis":"x","positions":[5,10,15,20,25,30]}"#);
    assert_eq!(r.get_str("rule"), Some("trapezoid"));
    let r = call(r#"{"type":"sections","tag":"vo"}"#);
    assert_eq!(r.get_arr("areas").unwrap().len(), 21);
    // exact:false skips massProps
    let r = call(r#"{"type":"sections","tag":"vo","exact":false}"#);
    assert!(r.get("volumeError").is_none() && r.get_f64("volume").unwrap() > 0.0);
}

#[test]
fn probe_points_and_rays() {
    load("p", &box_part());
    let (lo, hi) = bounds("p", "y");
    let yc = (lo + hi) / 2.0;
    let r = call(&format!(r#"{{"type":"probe","tag":"p","points":[[20,{yc},-15],[20,{},-15],[100,{yc},-15],[0,{yc},-15],[20,{},-15]]}}"#, hi + 3.0, lo + 1.0));
    let pts = r.get_arr("points").unwrap();
    // centre of 40 x 30 x 20: nearest face is a 10 mm z face
    assert_eq!(pts[0].get("inside"), Some(&Json::Bool(true)));
    assert!((pts[0].get_f64("distance").unwrap() + 10.0).abs() < 1e-6);
    assert_eq!(pts[1].get("inside"), Some(&Json::Bool(false)));
    assert!((pts[1].get_f64("distance").unwrap() - 3.0).abs() < 1e-6);
    assert!((pts[2].get_f64("distance").unwrap() - 60.0).abs() < 1e-6);
    assert_eq!(pts[3].get_str("inside"), Some("boundary"));
    assert!((pts[4].get_f64("distance").unwrap() + 1.0).abs() < 1e-6);
    assert!(pts[0].get_str("face").is_some());
    let r = call(&format!(r#"{{"type":"probe","tag":"p","rays":[{{"origin":[-10,{yc},-15],"dir":[1,0,0]}},{{"origin":[20,{yc},-15],"dir":[0,0,1]}},{{"origin":[-10,100,-15],"dir":[1,0,0]}}]}}"#));
    let rays = r.get_arr("rays").unwrap();
    let h = rays[0].get_arr("hits").unwrap();
    assert_eq!(h.len(), 2, "{}", ck_json::to_string(&r));
    assert!((h[0].get_f64("t").unwrap() - 10.0).abs() < 1e-9 && h[0].get("entering") == Some(&Json::Bool(true)));
    assert!((h[1].get_f64("t").unwrap() - 50.0).abs() < 1e-9 && h[1].get("entering") == Some(&Json::Bool(false)));
    let h = rays[1].get_arr("hits").unwrap();
    assert_eq!(h.len(), 1);
    assert_eq!(h[0].get("entering"), Some(&Json::Bool(false)));
    assert_eq!(rays[2].get_arr("hits").unwrap().len(), 0);
}

#[test]
fn probe_cylinder() {
    load("pc", &cyl_part());
    let yc = mid_y("pc");
    let r = call(&format!(r#"{{"type":"probe","tag":"pc","points":[[0,{yc},0],[7,{yc},0],[13,{yc},0]]}}"#));
    let p = r.get_arr("points").unwrap();
    assert!((p[0].get_f64("distance").unwrap() + 10.0).abs() < 1e-6);
    assert!((p[1].get_f64("distance").unwrap() + 3.0).abs() < 1e-6);
    assert!((p[2].get_f64("distance").unwrap() - 3.0).abs() < 1e-6);
}

#[test]
fn errors_and_limits() {
    load("e", &box_part());
    assert!(call_err(r#"{"type":"section","tag":"e"}"#).contains("plane"));
    assert!(call_err(r#"{"type":"section","tag":"e","plane":{"origin":[0,0,0],"normal":[0,0,0]}}"#).contains("zero"));
    assert!(call_err(r#"{"type":"sections","tag":"e","axis":"w"}"#).contains("axis"));
    assert!(call_err(r#"{"type":"probe","tag":"e"}"#).contains("points"));
    let many: Vec<String> = (0..2001).map(|_| "[0,0,0]".to_string()).collect();
    assert!(call_err(&format!(r#"{{"type":"probe","tag":"e","points":[{}]}}"#, many.join(","))).contains("2000"));
}
