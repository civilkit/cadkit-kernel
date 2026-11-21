//! The "peg": a bar 40 x 16 x 10 whose far end is a half disc R8 about (40, 8), a disc R8 about the same centre
//! extruded 10 up on its top and joined (the bar's front face y = 0 runs tangent into the cylinder along x = 40).
//! A blend of the front top edge (y = 0, z = 10) must stop at the cylinder's wall: the cylinder overhangs the
//! strip near x = 40 and is not cut. A result that is valid but cuts the cylinder (the full 40 mm prism) is wrong.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

fn peg(blend: &str) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let mut feats = vec![
        r#"{"type":"sketch","id":"E1","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"d","x":40,"y":16},{"id":"e","x":0,"y":16},{"id":"k","x":40,"y":8}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"arc","id":"e2","c":"k","p1":"b","p2":"d"},{"type":"line","id":"e3","p1":"d","p2":"e"},{"type":"line","id":"e4","p1":"e","p2":"a"}],"constraints":[]}"#.to_string(),
        r#"{"type":"extrude","id":"E2","op":"add","sketch":"E1","end":"blind","depth":10}"#.to_string(),
        r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"front"},"offset":10}"#.to_string(),
        r#"{"type":"sketch","id":"S3","plane":{"kind":"feature","id":"P1"},"points":[{"id":"c","x":40,"y":8}],"entities":[{"type":"circle","id":"c1","c":"c","r":8}],"constraints":[]}"#.to_string(),
        r#"{"type":"extrude","id":"E4","op":"add","sketch":"S3","end":"blind","depth":10}"#.to_string(),
    ];
    if !blend.is_empty() {
        feats.push(blend.to_string());
    }
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, r.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

fn vol(b: &ck_topo::body::Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

const FRONT_TOP: &str = r#""edges":[{"faces":["E2.C1","E2.S.e1"],"at":[20,0,10]}]"#;

#[test]
fn peg_front_round_stops_at_the_cylinder() {
    let (base, e) = peg("");
    assert!(e.is_empty(), "{e:?}");
    let v0 = vol(&base.unwrap());
    assert!((v0 - (6400.0 + 960.0 * std::f64::consts::PI)).abs() < 1e-6 * v0, "{v0}");
    let (b, e) = peg(&format!(r#"{{"type":"fillet","id":"B1","radius":2,{FRONT_TOP}}}"#));
    assert!(e.is_empty(), "{e:?}");
    let b = b.unwrap();
    assert!(ck_check::validate(&b, 1e-6).is_empty());
    // removed = integral over y in [0, 2] of (2 - sqrt(4 - (2 - y)^2)) (40 - sqrt(64 - (8 - y)^2)) dy
    let want = 32.3064750439;
    let got = v0 - vol(&b);
    assert!((got - want).abs() < 2e-3, "removed {got}, want {want}");
}

/// The strip a front-top-edge blend takes at distance `y` from the front face: the depth below the top.
fn fillet_depth(r: f64) -> impl Fn(f64) -> f64 {
    move |y| if y >= r { 0.0 } else { r - (r * r - (r - y) * (r - y)).max(0.0).sqrt() }
}
fn chamfer_depth(d: f64) -> impl Fn(f64) -> f64 {
    move |y| (d - y).max(0.0)
}

/// Independent reference: the volume a blend of the front top edge takes when it ends on the walls that rise over the
/// strip: `integral over y in [0, w] of depth(y) * length(y) dy` (the length of the line at distance y from the front face
/// that lies on material of the plate and outside the columns), by Simpson's rule on a graded grid (the integrand has
/// square-root ends).
fn removed(w: f64, depth: impl Fn(f64) -> f64, length: impl Fn(f64) -> f64) -> f64 {
    // substitute y = w (1 - cos(t))/2 so that the ends are resolved
    let n = 20000;
    let f = |t: f64| {
        let y = 0.5 * w * (1.0 - t.cos());
        depth(y) * length(y) * 0.5 * w * t.sin()
    };
    let h = std::f64::consts::PI / n as f64;
    let mut s = f(0.0) + f(std::f64::consts::PI);
    for i in 1..n {
        s += f(i as f64 * h) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    s * h / 3.0
}

/// Block 40 x 16 x 10 (front face y = 0), an optional boss `(cx, cy, R)` of height 10 on its top and an optional vertical
/// hole `(cx, cy, R)` through it.
fn block(boss: Option<(f64, f64, f64)>, hole: Option<(f64, f64, f64)>, blend: &str) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let mut feats = vec![
        r#"{"type":"sketch","id":"E1","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"d","x":40,"y":16},{"id":"e","x":0,"y":16}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"d"},{"type":"line","id":"e3","p1":"d","p2":"e"},{"type":"line","id":"e4","p1":"e","p2":"a"}],"constraints":[]}"#.to_string(),
        r#"{"type":"extrude","id":"E2","op":"add","sketch":"E1","end":"blind","depth":10}"#.to_string(),
    ];
    if let Some((x, y, r)) = boss {
        feats.push(r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"front"},"offset":10}"#.to_string());
        feats.push(format!(r#"{{"type":"sketch","id":"S3","plane":{{"kind":"feature","id":"P1"}},"points":[{{"id":"c","x":{x},"y":{y}}}],"entities":[{{"type":"circle","id":"c1","c":"c","r":{r}}}],"constraints":[]}}"#));
        feats.push(r#"{"type":"extrude","id":"E4","op":"add","sketch":"S3","end":"blind","depth":10}"#.to_string());
    }
    if let Some((x, y, r)) = hole {
        feats.push(format!(r#"{{"type":"sketch","id":"S5","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":{x},"y":{y}}}],"entities":[{{"type":"circle","id":"c1","c":"c","r":{r}}}],"constraints":[]}}"#));
        feats.push(r#"{"type":"extrude","id":"E6","op":"cut","sketch":"S5","end":"blind","depth":12,"flip":true}"#.to_string());
    }
    if !blend.is_empty() {
        feats.push(blend.to_string());
    }
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, r.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

fn blend_of(kind: &str, size: f64, ats: &[f64], top: &str) -> String {
    let edges: Vec<String> = ats.iter().map(|x| format!(r#"{{"faces":["{top}","E2.S.e1"],"at":[{x},0,10]}}"#)).collect();
    let key = if kind == "fillet" { "radius" } else { "distance" };
    let ch = if kind == "fillet" { "" } else { r#""chamferType":"equal","# };
    format!(r#"{{"type":"{kind}","id":"B1",{ch}"{key}":{size},"edges":[{}]}}"#, edges.join(","))
}

fn check(v0: f64, b: Option<ck_topo::body::Body>, errs: Vec<String>, want: f64, what: &str) {
    // a refusal is fine (never a valid-looking wrong body); a body must be right
    if !errs.is_empty() {
        eprintln!("{what}: refused: {errs:?}");
        return;
    }
    let b = b.unwrap();
    assert!(ck_check::validate(&b, 1e-6).is_empty(), "{what}: invalid");
    let got = v0 - vol(&b);
    assert!((got - want).abs() < 2e-3, "{what}: removed {got}, want {want}");
}

#[test]
fn reference_integral_matches_the_case_formula() {
    let r = removed(2.0, fillet_depth(2.0), |y| 40.0 - (64.0 - (8.0 - y) * (8.0 - y)).sqrt());
    assert!((r - 32.3064750439).abs() < 1e-6, "{r}");
}

#[test]
fn peg_front_chamfer_stops_at_the_cylinder() {
    let (base, _) = peg("");
    let v0 = vol(&base.unwrap());
    let (b, e) = peg(&format!(r#"{{"type":"chamfer","id":"B1","chamferType":"equal","distance":2,{FRONT_TOP}}}"#));
    let want = removed(2.0, chamfer_depth(2.0), |y| 40.0 - (64.0 - (8.0 - y) * (8.0 - y)).sqrt());
    check(v0, b, e, want, "peg chamfer d2");
}

/// A boss on the plate that reaches over the strip (its footprint crosses the strip but not the edge): the blend must not
/// cut under it.
#[test]
fn front_round_under_a_boss_footprint() {
    let boss = (20.0, 8.0, 7.0);
    let (base, _) = block(Some(boss), None, "");
    let v0 = vol(&base.unwrap());
    let chord = |y: f64| 2.0 * (49.0 - (8.0 - y) * (8.0 - y)).max(0.0).sqrt();
    for (kind, size) in [("fillet", 2.0), ("chamfer", 2.0)] {
        let (b, e) = block(Some(boss), None, &blend_of(kind, size, &[10.0], "E2.C1"));
        let want = if kind == "fillet" { removed(2.0, fillet_depth(2.0), |y| 40.0 - chord(y)) } else { removed(2.0, chamfer_depth(2.0), |y| 40.0 - chord(y)) };
        check(v0, b, e, want, &format!("boss footprint over the strip, {kind}"));
    }
}

/// A boss tangent to the front face: the edge is split at the tangent point; both halves rounded.
#[test]
fn front_round_past_a_tangent_boss() {
    let boss = (20.0, 8.0, 8.0);
    let (base, _) = block(Some(boss), None, "");
    let v0 = vol(&base.unwrap());
    let chord = |y: f64| 2.0 * (64.0 - (8.0 - y) * (8.0 - y)).max(0.0).sqrt();
    for (kind, size) in [("fillet", 2.0), ("chamfer", 2.0)] {
        let (b, e) = block(Some(boss), None, &blend_of(kind, size, &[10.0, 30.0], "E2.C1"));
        let want = if kind == "fillet" { removed(2.0, fillet_depth(2.0), |y| 40.0 - chord(y)) } else { removed(2.0, chamfer_depth(2.0), |y| 40.0 - chord(y)) };
        check(v0, b, e, want, &format!("tangent boss, {kind}"));
    }
}

/// A vertical hole through the plate whose wall cuts the edge (a notch in it).
#[test]
fn front_round_across_a_hole_notch() {
    let hole = (20.0, 2.0, 3.0);
    let (base, _) = block(None, Some(hole), "");
    let v0 = vol(&base.unwrap());
    let chord = |y: f64| 2.0 * (9.0 - (2.0 - y) * (2.0 - y)).max(0.0).sqrt();
    for (kind, size) in [("fillet", 2.0), ("chamfer", 2.0)] {
        let (b, e) = block(None, Some(hole), &blend_of(kind, size, &[5.0, 35.0], "E2.C1"));
        let want = if kind == "fillet" { removed(2.0, fillet_depth(2.0), |y| 40.0 - chord(y)) } else { removed(2.0, chamfer_depth(2.0), |y| 40.0 - chord(y)) };
        check(v0, b, e, want, &format!("hole notch, {kind}"));
    }
}
