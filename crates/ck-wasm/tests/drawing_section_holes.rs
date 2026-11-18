//! `drawing` of a plate with two `hole` features of different size on one face (K21): a section through the axes of
//! one hole feature (what the kit's script layer asks for when the cut sits at the view's middle), and the circle edges
//! of the holes, which must name the feature that made each hole (the app's callouts follow that feature, not the
//! radius). A sketch circle plus a cut names its own cut feature, so it is never taken for a hole feature.

use ck_json::Json;

fn call(msg: &Json) -> Json {
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg.clone())]));
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn regen(src: &str) {
    ck_wasm::engine::install();
    let r = call(&Json::Obj(vec![
        ("type".into(), Json::Str("regen".into())),
        ("tag".into(), Json::Str("main".into())),
        ("part".into(), ck_json::parse(src).unwrap()),
    ]));
    let errs = r.get("result").and_then(|r| r.get("errors")).and_then(|e| e.as_arr().map(|a| a.len()));
    assert_eq!(errs, Some(0), "regen failed: {}", ck_json::to_string(&r));
}

fn drawing_view(v: Json) -> Json {
    let r = call(&Json::Obj(vec![
        ("type".into(), Json::Str("drawing".into())),
        ("tag".into(), Json::Str("main".into())),
        ("views".into(), Json::Arr(vec![v])),
        ("hidden".into(), Json::Bool(true)),
    ]));
    r.get("result").and_then(|r| r.get_arr("views")).and_then(|a| a.first()).cloned().expect("a view")
}

fn vec3(a: [f64; 3]) -> Json {
    Json::Arr(a.iter().map(|&x| Json::Num(x)).collect())
}

/// Plate 60 x 40 x 10; feature H1 = 2 holes of diameter 3 at x = 30, feature H2 = 4 holes of diameter 2.5 at x = 10 and 50.
const TAIL: &str = r#"{"type":"plane","id":"P3","mode":"offset","base":{"kind":"std","id":"front"},"offset":10,"name":"top"}"#;

fn plate(cut_instead_of_h2: bool) -> String {
    plate_with(cut_instead_of_h2, r#""kind":"simple","end":"through""#)
}

/// `h5_kind`: the kind fields of the H5 holes (simple, counterbore, countersink).
fn plate_with(cut_instead_of_h2: bool, h5_kind: &str) -> String {
    plate_full(cut_instead_of_h2, h5_kind, &[(30.0, 10.0), (30.0, 30.0)])
}

fn plate_full(cut_instead_of_h2: bool, h5_kind: &str, h5: &[(f64, f64)]) -> String {
    let head = r#"{"format":"ck-part","version":1,"features":[
  {"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[
     {"id":"p1","x":0,"y":0},{"id":"p2","x":60,"y":0},{"id":"p3","x":60,"y":40},{"id":"p4","x":0,"y":40}],
   "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
     {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
  {"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10},"#;
    let pts = |l: &[(f64, f64)], tag: &str| -> (String, String) {
        let p: Vec<String> = l.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"{tag}q{i}","x":{},"y":{}}}"#, q.0, q.1)).collect();
        let e: Vec<String> = l.iter().enumerate().map(|(i, _)| format!(r#"{{"type":"point","id":"{tag}d{i}","p":"{tag}q{i}"}}"#)).collect();
        (p.join(","), e.join(","))
    };
    let (p1, e1) = pts(h5, "a");
    let (p2, e2) = pts(&[(10.0, 10.0), (10.0, 30.0), (50.0, 10.0), (50.0, 30.0)], "b");
    let h2 = if cut_instead_of_h2 {
        // the same four holes as sketch circles plus a cut
        let mut pp = vec![];
        let mut ee = vec![];
        for (i, q) in [(10.0, 10.0), (10.0, 30.0), (50.0, 10.0), (50.0, 30.0)].iter().enumerate() {
            pp.push(format!(r#"{{"id":"c{i}","x":{},"y":{}}}"#, q.0, q.1));
            ee.push(format!(r#"{{"type":"circle","id":"k{i}","c":"c{i}","r":1.25}}"#));
        }
        format!(
            r#"{{"type":"sketch","id":"S6","plane":{{"kind":"feature","id":"P3"}},"points":[{}],"entities":[{}],"constraints":[]}},
  {{"type":"extrude","id":"X7","op":"cut","sketch":"S6","end":"through"}}"#,
            pp.join(","),
            ee.join(",")
        )
    } else {
        format!(
            r#"{{"type":"sketch","id":"S6","plane":{{"kind":"feature","id":"P3"}},"points":[{p2}],"entities":[{e2}],"constraints":[]}},
  {{"type":"hole","id":"H7","sketch":"S6","kind":"simple","diameter":2.5,"end":"through"}}"#
        )
    };
    format!(
        r#"{head}
  {TAIL},
  {{"type":"sketch","id":"S4","plane":{{"kind":"feature","id":"P3"}},"points":[{p1}],"entities":[{e1}],"constraints":[]}},
  {{"type":"hole","id":"H5","sketch":"S4",{h5_kind},"diameter":3}},
  {h2}
 ]}}"#
    )
}

/// The circle edges of a face-on view: (radius, feature prefixes of the faces the edge borders).
fn circles(view: &Json) -> Vec<(f64, Vec<String>)> {
    view.get_arr("edges")
        .unwrap()
        .iter()
        .filter_map(|e| {
            let r = e.get("circle")?.get_f64("r")?;
            let feats = e.get_arr("faces")?.iter().filter_map(|f| f.as_str()).map(|f| f.split('.').next().unwrap().to_string()).collect();
            Some((r, feats))
        })
        .collect()
}

fn front() -> Json {
    Json::Obj(vec![("name".into(), Json::Str("front".into()))])
}

/// Every hole circle names the feature that made it: the 3 mm circles H5, the 2.5 mm circles H7, whatever the order.
#[test]
fn hole_circles_name_their_own_feature() {
    regen(&plate(false));
    let c = circles(&drawing_view(front()));
    let of = |r: f64| -> Vec<&Vec<String>> { c.iter().filter(|(x, _)| (x - r).abs() < 1e-9).map(|(_, f)| f).collect() };
    assert!(!of(1.5).is_empty() && !of(1.25).is_empty(), "{c:?}");
    assert!(of(1.5).iter().all(|f| f.iter().any(|x| x == "H5") && !f.iter().any(|x| x == "H7")), "{c:?}");
    assert!(of(1.25).iter().all(|f| f.iter().any(|x| x == "H7") && !f.iter().any(|x| x == "H5")), "{c:?}");
}

/// A sketch circle plus a cut is the feature X7 (an extrude), not a hole: no circle names a hole feature for it.
#[test]
fn circle_plus_cut_names_the_cut_feature() {
    regen(&plate(true));
    let c = circles(&drawing_view(front()));
    let small: Vec<_> = c.iter().filter(|(r, _)| (r - 1.25).abs() < 1e-9).collect();
    assert!(!small.is_empty(), "{c:?}");
    assert!(small.iter().all(|(_, f)| f.iter().any(|x| x == "X7") && !f.iter().any(|x| x == "H5" || x == "H7")), "{c:?}");
}

/// The script layer's vertical section (cut at the middle of the front view, x = 30) goes through the axes of the
/// two H5 holes; the section view must come back with hatching and the hole walls, not an error.
#[test]
fn section_through_hole_axes_is_not_dropped() {
    for cut in [false, true] {
        regen(&plate(cut));
        for x in [30.0, 10.0, 30.0 + 1.5, 29.0] {
            let v = drawing_view(Json::Obj(vec![
                ("x".into(), vec3([0.0, 0.0, -1.0])),
                ("y".into(), vec3([0.0, 1.0, 0.0])),
                ("section".into(), Json::Obj(vec![("point".into(), vec3([x, 0.0, 0.0])), ("hatch".into(), Json::Num(3.0))])),
            ]));
            assert!(v.get_str("error").is_none(), "section at x {x} (cut {cut}): {}", ck_json::to_string(&v));
            let lines = v.get_arr("lines").expect("lines");
            assert!(lines.iter().any(|l| l.get_str("k") == Some("h")), "no hatch at x {x} (cut {cut})");
        }
        // the horizontal cut (view looks along y) at y = 10 passes through the axes of the holes at y = 10
        let v = drawing_view(Json::Obj(vec![
            ("x".into(), vec3([1.0, 0.0, 0.0])),
            ("y".into(), vec3([0.0, 0.0, -1.0])),
            ("section".into(), Json::Obj(vec![("point".into(), vec3([0.0, 10.0, 0.0])), ("hatch".into(), Json::Num(3.0))])),
        ]));
        assert!(v.get_str("error").is_none(), "horizontal section (cut {cut}): {}", ck_json::to_string(&v));
    }
}

fn section_view_at_30(kind: &str, h5: &[(f64, f64)]) -> Json {
    regen(&plate_full(false, kind, h5));
    drawing_view(Json::Obj(vec![
        ("x".into(), vec3([0.0, 0.0, -1.0])),
        ("y".into(), vec3([0.0, 1.0, 0.0])),
        ("section".into(), Json::Obj(vec![("point".into(), vec3([30.0, 0.0, 0.0])), ("hatch".into(), Json::Num(3.0))])),
    ]))
}

/// A countersunk hole: the section through its axis comes back with hatching.
#[test]
fn section_through_csk_holes() {
    let v = section_view_at_30(r#""kind":"countersink","cskDiameter":6,"cskAngle":90,"end":"through""#, &[(30.0, 10.0), (30.0, 30.0)]);
    assert!(v.get_str("error").is_none(), "{}", ck_json::to_string(&v));
    assert!(v.get_arr("lines").unwrap().iter().any(|l| l.get_str("k") == Some("h")), "no hatch");
}

const CBORE: &str = r#""kind":"cbore","cboreDiameter":6,"cboreDepth":3,"end":"through""#;

/// A counterbored hole (bore r 3, floor at z 7, hole r 1.5): a plane through its axis used to fail in the boolean
/// (`nonManifoldEdge` on the bore floor). The section's seam relocation turned the bore wall's start vertex without
/// the slit that joins the floor's two circles, so the floor and the wall no longer paired along it.
#[test]
fn section_through_cbore_holes_draws() {
    let v = section_view_at_30(CBORE, &[(30.0, 20.0)]);
    assert!(v.get_str("error").is_none(), "{}", ck_json::to_string(&v));
    assert!(v.get_arr("lines").unwrap().iter().any(|l| l.get_str("k") == Some("h")), "no hatch");
}

fn vertical(at: f64) -> Json {
    Json::Obj(vec![
        ("x".into(), vec3([0.0, 0.0, -1.0])),
        ("y".into(), vec3([0.0, 1.0, 0.0])),
        ("section".into(), Json::Obj(vec![("point".into(), vec3([at, 0.0, 0.0])), ("hatch".into(), Json::Num(3.0))])),
    ])
}

fn horizontal(at: f64) -> Json {
    Json::Obj(vec![
        ("x".into(), vec3([1.0, 0.0, 0.0])),
        ("y".into(), vec3([0.0, 0.0, -1.0])),
        ("section".into(), Json::Obj(vec![("point".into(), vec3([0.0, at, 0.0])), ("hatch".into(), Json::Num(3.0))])),
    ])
}

fn drew(v: &Json, what: &str) {
    assert!(v.get_str("error").is_none(), "{what}: {}", ck_json::to_string(v));
    assert!(v.get_arr("lines").unwrap().iter().any(|l| l.get_str("k") == Some("h")), "{what}: no hatch");
}

/// Vertical and horizontal cuts through the axes of counterbored holes, through the cylinder seams (the horizontal
/// cut at y = 20 runs along the seam of the bore wall, at angle 0 from the hole centre), and offsets that graze the
/// bore (x = 33 touches the bore wall r 3, x = 31.5 the hole r 1.5) or cut just inside / outside it.
#[test]
fn cbore_sections_vertical_horizontal_seam_and_grazing() {
    regen(&plate_full(false, CBORE, &[(30.0, 20.0)]));
    for x in [30.0, 29.0, 31.0, 31.5, 28.5, 32.9, 33.0, 33.1, 27.0, 26.9] {
        drew(&drawing_view(vertical(x)), &format!("vertical at x {x}"));
    }
    for y in [20.0, 19.0, 21.5, 18.5, 17.0, 23.0, 22.9, 17.1] {
        drew(&drawing_view(horizontal(y)), &format!("horizontal at y {y}"));
    }
}

/// Two counterbored holes (feature H5) beside the simple holes of H7, cut through the axes of either.
#[test]
fn cbore_sections_with_other_hole_features() {
    regen(&plate_full(false, CBORE, &[(30.0, 10.0), (30.0, 30.0)]));
    drew(&drawing_view(vertical(30.0)), "vertical through both bores");
    drew(&drawing_view(horizontal(10.0)), "horizontal through the first bore and two H7 holes");
    drew(&drawing_view(horizontal(30.0)), "horizontal through the second bore and two H7 holes");
}

/// A blind counterbored hole (bore 3 deep, hole to depth 6 from the top).
#[test]
fn blind_cbore_sections() {
    regen(&plate_full(false, r#""kind":"cbore","cboreDiameter":6,"cboreDepth":3,"end":"blind","depth":6"#, &[(30.0, 20.0)]));
    for x in [30.0, 29.0, 31.5] {
        drew(&drawing_view(vertical(x)), &format!("blind cbore, vertical at x {x}"));
    }
    drew(&drawing_view(horizontal(20.0)), "blind cbore, horizontal through the axis");
}

/// A counterbore in a round stock: a disc of radius 15 (a cylinder with its own seam) holds the bore at its axis,
/// so the bore floor sits inside a cylindrical body.
#[test]
fn cbore_in_a_cylinder_sections() {
    regen(&format!(
        r#"{{"format":"ck-part","version":1,"features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"k","c":"c","r":15}}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10}},
  {TAIL},
  {{"type":"sketch","id":"S4","plane":{{"kind":"feature","id":"P3"}},"points":[{{"id":"h","x":0,"y":0}}],"entities":[{{"type":"point","id":"d","p":"h"}}],"constraints":[]}},
  {{"type":"hole","id":"H5","sketch":"S4",{CBORE},"diameter":3}}
 ]}}"#
    ));
    for x in [0.0, 1.0, 3.0, 2.9] {
        drew(&drawing_view(vertical(x)), &format!("disc, vertical at x {x}"));
    }
    drew(&drawing_view(horizontal(0.0)), "disc, horizontal through the axis");
}

/// A plane that misses the body is still `cutNoEffect` (pinned) and carries no `detail`: only a failed boolean does.
#[test]
fn section_plane_missing_the_body_has_no_detail() {
    regen(&plate(false));
    let miss = drawing_view(Json::Obj(vec![
        ("x".into(), vec3([0.0, 0.0, -1.0])),
        ("y".into(), vec3([0.0, 1.0, 0.0])),
        ("section".into(), Json::Obj(vec![("point".into(), vec3([-50.0, 0.0, 0.0])), ("hatch".into(), Json::Num(3.0))])),
    ]));
    assert_eq!(miss.get_str("error"), Some("cutNoEffect"), "{}", ck_json::to_string(&miss));
    assert!(miss.get("detail").is_none(), "{}", ck_json::to_string(&miss));
}
