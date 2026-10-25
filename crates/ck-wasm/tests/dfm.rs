//! Design-for-manufacture queries (`dfmMinWall`, `dfmMinInternalRadius`, `dfmPocketRatio`, `dfmOverhang`,
//! `dfmConstantSection`) through the full server path, on small fixtures with hand-computed answers.
//!
//! Orientation: a sketch on the `top` plane maps sketch (x, y) to world (x, ., -y) and `add` extrudes along +Y.

use ck_json::Json;

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

fn err(msg: &str) -> String {
    let j = raw(msg);
    assert_eq!(j.get("ok"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&j));
    j.get_str("error").unwrap().to_string()
}

const TOP: &str = r#"{"kind":"std","id":"top"}"#;

fn poly_sketch(id: &str, plane: &str, pts: &[(f64, f64)]) -> String {
    let n = pts.len();
    let p: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{}","x":{x},"y":{y}}}"#, i + 1)).collect();
    let e: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % n + 1)).collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, p.join(","), e.join(","))
}

fn rect(id: &str, plane: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    poly_sketch(id, plane, &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
}

fn extrude(id: &str, sk: &str, op: &str, depth: f64) -> String {
    format!(r#"{{"type":"extrude","id":"{id}","op":"{op}","sketch":"{sk}","end":"blind","depth":{depth}}}"#)
}

fn load(tag: &str, feats: &[String]) {
    let r = call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{{"format":"ck-part","version":1,"features":[{}]}}}}"#, feats.join(",")));
    let errs = r.get_arr("errors").map_or(0, |e| e.len());
    assert_eq!(errs, 0, "{}", ck_json::to_string(&r));
}

fn prism(tag: &str, pts: &[(f64, f64)], depth: f64) {
    load(tag, &[poly_sketch("F1", TOP, pts), extrude("F2", "F1", "add", depth)]);
}

fn num(j: &Json, k: &str) -> f64 {
    j.get_f64(k).unwrap_or_else(|| panic!("no {k}: {}", ck_json::to_string(j)))
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn arr<'a>(j: &'a Json, k: &str) -> &'a [Json] {
    j.get_arr(k).unwrap_or_else(|| panic!("no {k}: {}", ck_json::to_string(j)))
}

fn strs(j: &Json, k: &str) -> Vec<String> {
    arr(j, k).iter().map(|s| s.as_str().unwrap().to_string()).collect()
}

fn pt3(j: &Json, k: &str) -> [f64; 3] {
    j.get(k).and_then(|p| p.as_vec3()).unwrap_or_else(|| panic!("no {k}: {}", ck_json::to_string(j)))
}

/// 40 x 20 plate, 10 deep, d = 8 through hole at (30, 10).
fn plate(tag: &str) {
    let pts = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20},{"id":"c5","x":30,"y":10}"#;
    let ents = r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"},{"type":"circle","id":"e5","c":"c5","r":4}"#;
    load(tag, &[format!(r#"{{"type":"sketch","id":"F1","plane":{TOP},"points":[{pts}],"entities":[{ents}],"constraints":[]}}"#), extrude("F2", "F1", "add", 10.0)]);
}

/// Sketch (x, y) = (0..40, 0..5) wide step, (0..20, 5..10) tall part; 10 deep. Steps along sketch y = world -Z.
fn stepped(tag: &str) {
    prism(tag, &[(0.0, 0.0), (40.0, 0.0), (40.0, 5.0), (20.0, 5.0), (20.0, 10.0), (0.0, 10.0)], 10.0);
}

/// L bracket: 3 thick legs, 40 and 30 long, 10 deep.
fn bracket(tag: &str) {
    prism(tag, &[(0.0, 0.0), (40.0, 0.0), (40.0, 3.0), (3.0, 3.0), (3.0, 30.0), (0.0, 30.0)], 10.0);
}

/// 40 x 40 x 20 block with a 20 x 10 x `depth` pocket in the top face (sketch x 10..30, y 15..25); `fillet` rounds the four
/// vertical pocket corners.
fn pocketed(tag: &str, depth: f64, fillet: Option<f64>) {
    let mut f = vec![
        rect("F1", TOP, 0.0, 0.0, 40.0, 40.0),
        extrude("F2", "F1", "add", 20.0),
        rect("F3", r#"{"kind":"face","id":"F2.C1"}"#, 10.0, 15.0, 30.0, 25.0),
        extrude("F4", "F3", "cut", depth),
    ];
    if let Some(r) = fillet {
        let ymid = 20.0 - depth / 2.0;
        // corners p1..p4 of the pocket sketch; e4|e1 at p1, e1|e2 at p2, e2|e3 at p3, e3|e4 at p4
        let corners = [("e4", "e1", 10.0, -15.0), ("e1", "e2", 30.0, -15.0), ("e2", "e3", 30.0, -25.0), ("e3", "e4", 10.0, -25.0)];
        let edges: Vec<String> =
            corners.iter().map(|(a, b, x, z)| format!(r#"{{"faces":["F4.S.{a}","F4.S.{b}"],"at":[{x},{ymid},{z}]}}"#)).collect();
        f.push(format!(r#"{{"type":"fillet","id":"F5","radius":{r},"edges":[{}]}}"#, edges.join(",")));
    }
    load(tag, &f);
}

/// 40 x 30 x 20 box shelled to `t` with the top face open.
fn shell_box(tag: &str, t: f64) {
    load(tag, &[rect("F1", TOP, 0.0, 0.0, 40.0, 30.0), extrude("F2", "F1", "add", 20.0), format!(r#"{{"type":"shell","id":"F3","faces":["F2.C1"],"thickness":{t}}}"#)]);
}

// ---------------------------------------------------------------- dfmMinWall

#[test]
fn min_wall_plate_with_hole() {
    plate("w1");
    // thinnest wall: hole edge (x = 34) to the right side (x = 40): 6 (sampled, so a hair above)
    let r = call(r#"{"type":"dfmMinWall","tag":"w1","threshold":7}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("supported"), Some(&Json::Bool(true)));
    assert_eq!(r.get("rule").unwrap().get_str("id"), Some("minWall"));
    let min = r.get("minimum").unwrap();
    assert!(close(num(min, "thickness"), 6.0, 0.05), "{}", ck_json::to_string(&r));
    let v = arr(&r, "violations");
    assert!(!v.is_empty());
    assert_eq!(v[0].get_str("rule"), Some("minWall"));
    assert!(num(&v[0], "thickness") < 7.0);
    assert_eq!(strs(&v[0], "faces").len(), 2);
    let p = pt3(&v[0], "point");
    assert!(p.iter().all(|x| x.is_finite()));
    let r = call(r#"{"type":"dfmMinWall","tag":"w1","threshold":5}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)));
    assert!(arr(&r, "violations").is_empty());
    // deterministic
    let a = ck_json::to_string(&call(r#"{"type":"dfmMinWall","tag":"w1","threshold":7}"#));
    let b = ck_json::to_string(&call(r#"{"type":"dfmMinWall","tag":"w1","threshold":7}"#));
    assert_eq!(a, b);
}

#[test]
fn min_wall_bracket_stepped_and_plain() {
    bracket("w2");
    let r = call(r#"{"type":"dfmMinWall","tag":"w2","threshold":4}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)));
    assert!(close(num(r.get("minimum").unwrap(), "thickness"), 3.0, 1e-6), "{}", ck_json::to_string(&r));
    assert!(close(num(&arr(&r, "violations")[0], "thickness"), 3.0, 1e-6));
    assert_eq!(call(r#"{"type":"dfmMinWall","tag":"w2","threshold":3}"#).get("pass"), Some(&Json::Bool(true)));
    stepped("w3");
    let r = call(r#"{"type":"dfmMinWall","tag":"w3","threshold":6}"#);
    assert!(close(num(r.get("minimum").unwrap(), "thickness"), 5.0, 1e-6), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)));
    prism("w4", &[(0.0, 0.0), (40.0, 0.0), (40.0, 30.0), (0.0, 30.0)], 20.0);
    let r = call(r#"{"type":"dfmMinWall","tag":"w4","threshold":20}"#);
    assert!(close(num(r.get("minimum").unwrap(), "thickness"), 20.0, 1e-6), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)));
}

#[test]
fn min_wall_thin_shell_fails() {
    shell_box("w5", 0.8);
    let r = call(r#"{"type":"dfmMinWall","tag":"w5","threshold":1.2}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    assert!(close(num(r.get("minimum").unwrap(), "thickness"), 0.8, 1e-6));
    assert!(!arr(&r, "violations").is_empty());
    assert_eq!(call(r#"{"type":"dfmMinWall","tag":"w5","threshold":0.8}"#).get("pass"), Some(&Json::Bool(true)));
}

// ---------------------------------------------------------------- dfmMinInternalRadius

#[test]
fn internal_radius_sharp_and_filleted() {
    bracket("r1");
    // the inside corner of the L is a sharp concave edge along Y at sketch (3, 3) = world (3, ., -3)
    let r = call(r#"{"type":"dfmMinInternalRadius","tag":"r1","toolRadius":1,"direction":[0,1,0]}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    let m = r.get("minimum").unwrap();
    assert_eq!(num(m, "radius"), 0.0);
    assert_eq!(m.get_str("kind"), Some("sharp"));
    assert_eq!(strs(m, "faces").len(), 2);
    assert!(m.get_str("edge").is_some());
    let p = pt3(m, "point");
    assert!(close(p[0], 3.0, 1e-6) && close(p[2], -3.0, 1e-6), "{p:?}");
    assert_eq!(arr(&r, "violations").len(), 1);
    // a tool axis across the corner sees no vertical corner
    let r = call(r#"{"type":"dfmMinInternalRadius","tag":"r1","toolRadius":1,"direction":[1,0,0]}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)));
    assert_eq!(r.get("minimum"), Some(&Json::Null));

    pocketed("r2", 12.0, Some(2.0));
    let r = call(r#"{"type":"dfmMinInternalRadius","tag":"r2","toolRadius":3,"direction":[0,1,0]}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    assert!(close(num(r.get("minimum").unwrap(), "radius"), 2.0, 1e-9));
    assert_eq!(r.get("minimum").unwrap().get_str("kind"), Some("fillet"));
    assert_eq!(arr(&r, "violations").len(), 4);
    let r = call(r#"{"type":"dfmMinInternalRadius","tag":"r2","toolRadius":2,"direction":[0,1,0]}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    // unfilleted pocket: four sharp corners
    pocketed("r3", 12.0, None);
    let r = call(r#"{"type":"dfmMinInternalRadius","tag":"r3","toolRadius":0.5,"direction":[0,1,0]}"#);
    assert_eq!(arr(&r, "violations").len(), 4, "{}", ck_json::to_string(&r));
}

#[test]
fn internal_radius_hole_is_not_a_corner() {
    plate("r4");
    let r = call(r#"{"type":"dfmMinInternalRadius","tag":"r4","toolRadius":5,"direction":[0,1,0]}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("minimum"), Some(&Json::Null));
}

// ---------------------------------------------------------------- dfmPocketRatio

#[test]
fn pocket_ratio_block_and_shell() {
    pocketed("p1", 12.0, None);
    let r = call(r#"{"type":"dfmPocketRatio","tag":"p1","maxRatio":1}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    let ps = arr(&r, "pockets");
    assert_eq!(ps.len(), 1);
    assert!(close(num(&ps[0], "depth"), 12.0, 1e-6) && close(num(&ps[0], "width"), 10.0, 1e-6) && close(num(&ps[0], "ratio"), 1.2, 1e-6));
    assert_eq!(ps[0].get_str("floor"), Some("F4.C0"));
    assert_eq!(strs(&ps[0], "walls").len(), 4);
    assert_eq!(arr(&r, "violations").len(), 1);
    assert!(close(pt3(&ps[0], "point")[1], 8.0, 1e-6));
    assert_eq!(call(r#"{"type":"dfmPocketRatio","tag":"p1","maxRatio":1.2}"#).get("pass"), Some(&Json::Bool(true)));
    // filleted corners keep the same depth and width
    pocketed("p2", 12.0, Some(2.0));
    let r = call(r#"{"type":"dfmPocketRatio","tag":"p2","maxRatio":1}"#);
    let ps = arr(&r, "pockets");
    assert_eq!(ps.len(), 1, "{}", ck_json::to_string(&r));
    assert!(close(num(&ps[0], "depth"), 12.0, 1e-6) && close(num(&ps[0], "width"), 10.0, 1e-6), "{}", ck_json::to_string(&r));
    // the inside of a shelled box is a pocket: 19.2 deep, 28.4 wide
    shell_box("p3", 0.8);
    let r = call(r#"{"type":"dfmPocketRatio","tag":"p3","maxRatio":0.5}"#);
    let ps = arr(&r, "pockets");
    assert_eq!(ps.len(), 1, "{}", ck_json::to_string(&r));
    assert!(close(num(&ps[0], "depth"), 19.2, 1e-6) && close(num(&ps[0], "width"), 28.4, 1e-6), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)));
}

#[test]
fn pocket_ratio_counts_no_pockets_in_steps_and_through_holes() {
    for (tag, make) in [("p4", stepped as fn(&str)), ("p5", plate), ("p6", bracket)] {
        make(tag);
        let r = call(&format!(r#"{{"type":"dfmPocketRatio","tag":"{tag}","maxRatio":0.1}}"#));
        assert!(arr(&r, "pockets").is_empty(), "{tag}: {}", ck_json::to_string(&r));
        assert_eq!(r.get("pass"), Some(&Json::Bool(true)));
        assert_eq!(r.get("maximum"), Some(&Json::Null));
    }
}

// ---------------------------------------------------------------- dfmOverhang

/// T section: stem sketch x 10..30, y 0..5; head x 0..40, y 5..10; 10 deep. Sketch +y is world -Z.
fn tee(tag: &str) {
    prism(tag, &[(10.0, 0.0), (30.0, 0.0), (30.0, 5.0), (40.0, 5.0), (40.0, 10.0), (0.0, 10.0), (0.0, 5.0), (10.0, 5.0)], 10.0);
}

#[test]
fn overhang_tee_wings() {
    tee("o1");
    // building towards sketch +y (world -Z): both wing undersides (10 x 10 each) are flat ceilings 5 above the plate
    let r = call(r#"{"type":"dfmOverhang","tag":"o1","direction":[0,0,-1],"angle":45}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    assert!(close(num(&r, "overhangArea"), 200.0, 1e-6), "{}", ck_json::to_string(&r));
    assert!(close(num(&r, "supportVolume"), 1000.0, 1e-6), "{}", ck_json::to_string(&r));
    let fs = arr(&r, "faces");
    assert_eq!(fs.len(), 2);
    for f in fs {
        assert!(close(num(f, "overhangAngle"), 90.0, 1e-6) && close(num(f, "overhangArea"), 100.0, 1e-6) && close(num(f, "supportVolume"), 500.0, 1e-6));
        assert_eq!(f.get_str("rule"), Some("overhang"));
        assert!(f.get_str("face").is_some());
    }
    // a 90 degree limit allows flat ceilings
    assert_eq!(call(r#"{"type":"dfmOverhang","tag":"o1","direction":[0,0,-1],"angle":90}"#).get("pass"), Some(&Json::Bool(true)));
    // building along the extrusion: every face is a wall or a cap
    let r = call(r#"{"type":"dfmOverhang","tag":"o1","direction":[0,1,0],"angle":45}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    assert_eq!(num(&r, "supportVolume"), 0.0);
    // upside down the flat top of the head lies on the plate and nothing else faces down
    let r = call(r#"{"type":"dfmOverhang","tag":"o1","direction":[0,0,1],"angle":45}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
}

#[test]
fn overhang_stepped_block_and_input_errors() {
    stepped("o2");
    // up = world -Z (sketch +y): the step ledge (sketch y = 5, x 20..40) faces away from the plate? it faces up -> no overhang
    let r = call(r#"{"type":"dfmOverhang","tag":"o2","direction":[0,0,-1],"angle":45}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    // the other way up the ledge is a ceiling 20 x 10 at height 5 above the plate
    let r = call(r#"{"type":"dfmOverhang","tag":"o2","direction":[0,0,1],"angle":45}"#);
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    assert!(close(num(&r, "overhangArea"), 200.0, 1e-6), "{}", ck_json::to_string(&r));
    assert!(close(num(&r, "supportVolume"), 1000.0, 1e-6), "{}", ck_json::to_string(&r));
    assert!(err(r#"{"type":"dfmOverhang","tag":"o2","angle":45}"#).contains("direction"));
    assert!(err(r#"{"type":"dfmOverhang","tag":"o2","direction":[0,0,0],"angle":45}"#).contains("direction"));
    assert!(err(r#"{"type":"dfmOverhang","tag":"o2","direction":[0,1,0]}"#).contains("angle"));
    assert!(err(r#"{"type":"dfmOverhang","tag":"o2","direction":[0,1,0],"angle":91}"#).contains("angle"));
}

// ---------------------------------------------------------------- dfmConstantSection

#[test]
fn constant_section_extrusions() {
    plate("c1");
    // the plate with its through hole is one section along the extrusion axis (Y)
    let r = call(r#"{"type":"dfmConstantSection","tag":"c1","axis":[0,1,0]}"#);
    assert_eq!(r.get("constant"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("pass"), Some(&Json::Bool(true)));
    assert!(close(num(&r, "length"), 10.0, 1e-6));
    assert!(close(num(&r, "sectionArea"), 40.0 * 20.0 - std::f64::consts::PI * 16.0, 1e-3), "{}", ck_json::to_string(&r));
    assert_eq!(num(&r, "loops"), 2.0);
    // across the hole (axis X) the cylinder is not parallel: the section changes where the hole starts, x = 26
    let r = call(r#"{"type":"dfmConstantSection","tag":"c1","axis":[1,0,0]}"#);
    assert_eq!(r.get("constant"), Some(&Json::Bool(false)));
    let st = r.get("station").unwrap();
    assert!(close(num(st, "position"), 26.0, 1e-3) && close(num(st, "offset"), 26.0, 1e-3), "{}", ck_json::to_string(&r));
    assert_eq!(strs(st, "faces").len(), 1);
    // a plain box is constant along every edge direction
    prism("c2", &[(0.0, 0.0), (40.0, 0.0), (40.0, 30.0), (0.0, 30.0)], 20.0);
    for ax in ["[1,0,0]", "[0,1,0]", "[0,0,1]"] {
        let r = call(&format!(r#"{{"type":"dfmConstantSection","tag":"c2","axis":{ax}}}"#));
        assert_eq!(r.get("constant"), Some(&Json::Bool(true)), "{ax}: {}", ck_json::to_string(&r));
    }
    // ... and not along a diagonal: the section changes at once
    let r = call(r#"{"type":"dfmConstantSection","tag":"c2","axis":[1,1,0]}"#);
    assert_eq!(r.get("constant"), Some(&Json::Bool(false)));
    assert!(close(num(r.get("station").unwrap(), "offset"), 0.0, 1e-6), "{}", ck_json::to_string(&r));
    // the L bracket is constant along its extrusion only
    bracket("c3");
    assert_eq!(call(r#"{"type":"dfmConstantSection","tag":"c3","axis":[0,1,0]}"#).get("constant"), Some(&Json::Bool(true)));
}

#[test]
fn constant_section_first_station_of_a_step() {
    stepped("c4");
    assert_eq!(call(r#"{"type":"dfmConstantSection","tag":"c4","axis":[0,1,0]}"#).get("constant"), Some(&Json::Bool(true)));
    // along world Z the ledge (sketch y = 5 -> z = -5) splits the body: 20 x 10 for z in [-10, -5], 40 x 10 for [-5, 0]
    let r = call(r#"{"type":"dfmConstantSection","tag":"c4","axis":[0,0,1]}"#);
    assert_eq!(r.get("constant"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("pass"), Some(&Json::Bool(false)));
    let st = r.get("station").unwrap();
    assert!(close(num(st, "position"), -5.0, 1e-6) && close(num(st, "offset"), 5.0, 1e-6), "{}", ck_json::to_string(&r));
    assert!(close(num(st, "sectionAreaBefore"), 200.0, 1e-3) && close(num(st, "sectionAreaAfter"), 400.0, 1e-3), "{}", ck_json::to_string(&r));
    assert_eq!(strs(st, "faces").len(), 1);
    assert!(close(pt3(st, "point")[2], -5.0, 1e-6));
    // a blind pocket floor is a station too: the pocket (12 deep from y = 20) starts at y = 8
    pocketed("c5", 12.0, None);
    let r = call(r#"{"type":"dfmConstantSection","tag":"c5","axis":[0,1,0]}"#);
    let st = r.get("station").unwrap();
    assert_eq!(r.get("constant"), Some(&Json::Bool(false)));
    assert!(close(num(st, "position"), 8.0, 1e-6), "{}", ck_json::to_string(&r));
    assert_eq!(strs(st, "faces"), vec!["F4.C0".to_string()]);
}

// ---------------------------------------------------------------- refusals and errors

#[test]
fn free_form_bodies_are_refused_not_guessed() {
    // a loft through two closed splines has B-spline walls
    let sk = |id: &str, plane: &str, r: f64| {
        let n = 8;
        let pts: Vec<String> = (0..n).map(|k| format!(r#"{{"id":"p{k}","x":{},"y":{}}}"#, r * (std::f64::consts::TAU * k as f64 / n as f64).cos(), r * (std::f64::consts::TAU * k as f64 / n as f64).sin())).collect();
        let ids: Vec<String> = (0..n).map(|k| format!(r#""p{k}""#)).collect();
        format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{{"id":"s","type":"spline","pts":[{}],"closed":true}}],"constraints":[]}}"#, pts.join(","), ids.join(","))
    };
    load(
        "f1",
        &[
            sk("S0", r#"{"kind":"std","id":"front"}"#, 10.0),
            r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"front"},"offset":30}"#.to_string(),
            sk("S1", r#"{"kind":"feature","id":"P1"}"#, 14.0),
            r#"{"type":"loft","id":"L","op":"add","sketches":["S0","S1"]}"#.to_string(),
        ],
    );
    for q in [
        r#"{"type":"dfmMinInternalRadius","tag":"f1","toolRadius":1}"#,
        r#"{"type":"dfmPocketRatio","tag":"f1","maxRatio":2}"#,
        r#"{"type":"dfmConstantSection","tag":"f1","axis":[0,0,1]}"#,
    ] {
        let r = call(q);
        assert_eq!(r.get("supported"), Some(&Json::Bool(false)), "{q}: {}", ck_json::to_string(&r));
        assert_eq!(r.get("pass"), Some(&Json::Null));
        let rf = r.get("refusal").unwrap();
        assert_eq!(rf.get_str("code"), Some("nonAnalyticFaces"));
        assert!(!strs(rf, "faces").is_empty());
        assert!(rf.get_str("reason").unwrap().contains("free-form"));
    }
    // the ray / mesh based queries still answer on it
    let r = call(r#"{"type":"dfmMinWall","tag":"f1","threshold":1}"#);
    assert_eq!(r.get("supported"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    let r = call(r#"{"type":"dfmOverhang","tag":"f1","direction":[0,0,1],"angle":45}"#);
    assert_eq!(r.get("supported"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
}

#[test]
fn no_body_is_null_and_bad_inputs_are_errors() {
    call(r#"{"type":"regen","tag":"e1","part":{"format":"ck-part","features":[]}}"#);
    for q in [
        r#"{"type":"dfmMinWall","tag":"e1","threshold":1}"#,
        r#"{"type":"dfmMinInternalRadius","tag":"e1","toolRadius":1}"#,
        r#"{"type":"dfmPocketRatio","tag":"e1","maxRatio":1}"#,
        r#"{"type":"dfmOverhang","tag":"e1","direction":[0,1,0],"angle":45}"#,
        r#"{"type":"dfmConstantSection","tag":"e1","axis":[0,1,0]}"#,
    ] {
        assert_eq!(call(q), Json::Null, "{q}");
    }
    plate("e2");
    assert!(err(r#"{"type":"dfmMinWall","tag":"e2"}"#).contains("threshold"));
    assert!(err(r#"{"type":"dfmMinWall","tag":"e2","threshold":-1}"#).contains("threshold"));
    assert!(err(r#"{"type":"dfmMinWall","tag":"e2","threshold":"x"}"#).contains("threshold"));
    assert!(err(r#"{"type":"dfmMinInternalRadius","tag":"e2"}"#).contains("toolRadius"));
    assert!(err(r#"{"type":"dfmMinInternalRadius","tag":"e2","toolRadius":1,"direction":[0,0,0]}"#).contains("direction"));
    assert!(err(r#"{"type":"dfmPocketRatio","tag":"e2"}"#).contains("maxRatio"));
    assert!(err(r#"{"type":"dfmConstantSection","tag":"e2"}"#).contains("axis"));
    assert!(err(r#"{"type":"dfmConstantSection","tag":"e2","axis":[0,0,0]}"#).contains("axis"));
}
