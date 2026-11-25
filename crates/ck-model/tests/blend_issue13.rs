//! Issue 13: (1) the order of the `edges` list of a chamfer or fillet does not change the result on several bodies,
//! (2) the variable chamfer (`chamferType: "variable"`, `startDistance` / `endDistance`), (3) a fillet on an edge that
//! ends on a tangent seam. Blocks are built with the sketch on `front` (x, y; extrude +z), so `F2.C1` is the top cap.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

struct Out {
    body: Option<Body>,
    errors: Vec<String>,
    warnings: Vec<String>,
}

fn run(feats: &[String]) -> Out {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    let f = |v: &Vec<regen::RegenError>| v.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect();
    Out { body: br.body, errors: f(&r.errors), warnings: f(&r.warnings) }
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn valid(b: &Body) -> bool {
    ck_check::validate(b, 1e-6).is_empty()
}

fn rect(w: f64, d: f64, extra_pts: &str, extra_ents: &str) -> String {
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":{w},"y":0}},{{"id":"p3","x":{w},"y":{d}}},{{"id":"p4","x":0,"y":{d}}}{extra_pts}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}{extra_ents}],"constraints":[]}}"#
    )
}

fn extrude(depth: f64) -> String {
    format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth}}}"#)
}

/// A pick (faces + a point on the edge) for every edge of face `face`, read off the body.
fn picks_of_faces(b: &Body, faces: &[&str]) -> Vec<String> {
    let mut out = vec![];
    let mut seen = vec![];
    for name in faces {
        let f = b.find_face(name).unwrap_or_else(|| panic!("no face {name}"));
        for e in b.edge_ids() {
            let fs = b.edge_faces(e);
            if fs.len() != 2 || !fs.contains(&f) || seen.contains(&e) {
                continue;
            }
            seen.push(e);
            let ed = b.edge(e);
            let (p0, p1) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
            // a line: its midpoint; a closed or curved edge: an end vertex lies on it
            let p = if ed.v0 == ed.v1 { p0 } else if matches!(ed.curve, ck_geom::curve::Curve::Line { .. }) { (p0 + p1) * 0.5 } else { p0 };
            out.push(format!(r#"{{"faces":["{}","{}"],"at":[{:.9},{:.9},{:.9}]}}"#, b.face_id_str(fs[0]), b.face_id_str(fs[1]), p.x, p.y, p.z));
        }
    }
    out
}

/// Deterministic shuffles (xorshift): all permutations up to 4 picks, else `n` pseudo-random ones.
fn orders(len: usize, n: usize) -> Vec<Vec<usize>> {
    let mut seed = 0x9E3779B97F4A7C15u64 ^ len as u64;
    let mut out: Vec<Vec<usize>> = vec![(0..len).collect(), (0..len).rev().collect()];
    for _ in 0..n {
        let mut v: Vec<usize> = (0..len).collect();
        for i in (1..len).rev() {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            v.swap(i, (seed % (i as u64 + 1)) as usize);
        }
        out.push(v);
    }
    out
}

/// Every order of the picks of `head` (a chamfer or fillet) on `base` gives the same errors, volume and validity.
fn order_independent(name: &str, base: &[String], face_names: &[&str], head: &str) {
    let b0 = run(base);
    assert!(b0.errors.is_empty(), "{name}: base {:?}", b0.errors);
    let picks = picks_of_faces(b0.body.as_ref().unwrap(), face_names);
    assert!(picks.len() >= 2, "{name}: only {} picks", picks.len());
    let mut first: Option<(Vec<String>, f64, bool, usize)> = None;
    for ord in orders(picks.len(), 12) {
        let list: Vec<&str> = ord.iter().map(|&i| picks[i].as_str()).collect();
        let mut feats = base.to_vec();
        feats.push(format!(r#"{{{head},"edges":[{}]}}"#, list.join(",")));
        let o = run(&feats);
        let r = (o.errors.clone(), o.body.as_ref().map(vol).unwrap_or(f64::NAN), o.body.as_ref().is_some_and(valid), o.body.as_ref().map(|b| b.num_faces()).unwrap_or(0));
        match &first {
            None => first = Some(r),
            Some(a) => {
                assert_eq!(r.0, a.0, "{name}: errors differ for order {ord:?}");
                assert!((r.1 - a.1).abs() <= 1e-6 * a.1.abs().max(1.0), "{name}: volume {} vs {} for order {ord:?}", r.1, a.1);
                assert_eq!(r.2, a.2, "{name}: validity differs for order {ord:?}");
                assert_eq!(r.3, a.3, "{name}: face count differs for order {ord:?}");
            }
        }
    }
    let a = first.unwrap();
    assert!(a.0.is_empty() && a.2, "{name}: the common result is an error or invalid: {:?} valid={}", a.0, a.2);
}

fn plate_with_holes() -> Vec<String> {
    let pts = r#",{"id":"c1","x":10,"y":10},{"id":"c2","x":30,"y":10}"#;
    let ents = r#",{"type":"circle","id":"e5","c":"c1","r":3},{"type":"circle","id":"e6","c":"c2","r":4}"#;
    vec![rect(40.0, 20.0, pts, ents), extrude(6.0)]
}

fn box_with_round() -> Vec<String> {
    vec![
        rect(40.0, 20.0, "", ""),
        extrude(10.0),
        r#"{"type":"fillet","id":"F3","radius":4,"edges":[{"faces":["F2.S.e2","F2.S.e3"],"at":[40,20,5]}]}"#.to_string(),
    ]
}

fn cylinder() -> Vec<String> {
    vec![
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":8}],"constraints":[]}"#.to_string(),
        extrude(15.0),
    ]
}

#[test]
fn shuffled_chamfer_edges_give_the_same_part() {
    let h = r#""type":"chamfer","id":"F9","chamferType":"equal","distance":1"#;
    order_independent("chamfer box+round", &box_with_round(), &["F2.C1"], h);
    order_independent("chamfer plate+holes", &plate_with_holes(), &["F2.C1"], h);
    order_independent("chamfer cylinder", &cylinder(), &["F2.C1", "F2.C0"], h);
}

#[test]
fn shuffled_fillet_edges_give_the_same_part() {
    let h = r#""type":"fillet","id":"F9","radius":1"#;
    order_independent("fillet box+round", &box_with_round(), &["F2.C1"], h);
    order_independent("fillet plate+holes", &plate_with_holes(), &["F2.C1"], h);
    order_independent("fillet cylinder", &cylinder(), &["F2.C1", "F2.C0"], h);
}

// ---- variable chamfer: block 40 x 10 x 20, top-front edge F2.C1 | F2.S.e1 (y = 0, z = 20, x 0..40)

fn block() -> Vec<String> {
    vec![rect(40.0, 10.0, "", ""), extrude(20.0)]
}

fn has_vertex(b: &Body, p: [f64; 3]) -> bool {
    b.vertex_ids().any(|v| {
        let q = b.vertex(v).p;
        (q.x - p[0]).abs() < 1e-6 && (q.y - p[1]).abs() < 1e-6 && (q.z - p[2]).abs() < 1e-6
    })
}

fn variable(at_x: f64, start: &str, end: &str) -> Out {
    let mut f = block();
    f.push(format!(
        r#"{{"type":"chamfer","id":"F3","chamferType":"variable","startDistance":{start},"endDistance":{end},"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[{at_x},0,20]}}]}}"#
    ));
    run(&f)
}

#[test]
fn variable_chamfer_runs_from_start_distance_at_the_pick_end_to_end_distance() {
    // removed volume = L (d0^2 + d0 d1 + d1^2) / 6 for a distance that is linear along the edge
    let want = 8000.0 - 40.0 * (1.0 + 3.0 + 9.0) / 6.0;
    // pick near x = 0: start (1) at x = 0, end (3) at x = 40
    let o = variable(5.0, "1", "3");
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    assert!(valid(b));
    assert!((vol(b) - want).abs() < 1e-6, "{} vs {want}", vol(b));
    assert!(has_vertex(b, [0.0, 1.0, 20.0]) && has_vertex(b, [0.0, 0.0, 19.0]), "start end");
    assert!(has_vertex(b, [40.0, 3.0, 20.0]) && has_vertex(b, [40.0, 0.0, 17.0]), "far end");
    // the pick near x = 40 swaps the ends: same volume, mirrored
    let o = variable(35.0, "1", "3");
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    assert!(valid(b) && (vol(b) - want).abs() < 1e-6);
    assert!(has_vertex(b, [40.0, 1.0, 20.0]) && has_vertex(b, [0.0, 3.0, 20.0]));
}

#[test]
fn variable_chamfer_with_equal_ends_is_the_equal_chamfer() {
    let o = variable(5.0, "2", "2");
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    assert!(valid(b) && (vol(b) - (8000.0 - 40.0 * 2.0)).abs() < 1e-6, "{}", vol(b));
}

#[test]
fn variable_chamfer_with_partial_edge_offsets() {
    // 5 mm left off at the start, 10 at the far end: the distance still runs 1 -> 3 over the edge, the blend covers x 5..30
    let mut f = block();
    f.push(r#"{"type":"chamfer","id":"F3","chamferType":"variable","startDistance":2,"endDistance":2,"startOffset":5,"endOffset":10,"edges":[{"faces":["F2.C1","F2.S.e1"],"at":[5,0,20]}]}"#.to_string());
    let o = run(&f);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    assert!(valid(b) && (vol(b) - (8000.0 - 25.0 * 2.0)).abs() < 1e-6, "{}", vol(b));
}

#[test]
fn variable_chamfer_refusals_name_the_field() {
    for (s, e, field) in [("1", "0", "endDistance"), ("-1", "2", "startDistance")] {
        let o = variable(5.0, s, e);
        assert!(o.errors.iter().any(|x| x.contains("invalidValue") && x.contains(field)), "{field}: {:?}", o.errors);
    }
    let mut f = block();
    f.push(r#"{"type":"chamfer","id":"F3","chamferType":"variable","startDistance":2,"edges":[{"faces":["F2.C1","F2.S.e1"],"at":[5,0,20]}]}"#.to_string());
    let o = run(&f);
    assert!(o.errors.iter().any(|x| x.contains("invalidValue") && x.contains("endDistance")), "{:?}", o.errors);
    let mut f = block();
    f.push(r#"{"type":"chamfer","id":"F3","chamferType":"variable","startDistance":2,"endDistance":3,"propagate":true,"edges":[{"faces":["F2.C1","F2.S.e1"],"at":[5,0,20]}]}"#.to_string());
    let o = run(&f);
    assert!(o.errors.iter().any(|x| x.contains("invalidValue") && x.contains("propagate")), "{:?}", o.errors);
}

#[test]
fn variable_chamfer_too_large_for_the_face_is_refused() {
    let o = variable(5.0, "1", "30"); // the front face is 20 tall
    assert!(!o.errors.is_empty(), "built a chamfer larger than its face");
}

// ---- fillet on an edge that ends on a tangent seam: block 40 x 10 x 20 with the vertical edge at (40, 0) rounded r 3.
// The top-front line (x 0..37) ends where the top plane meets the round tangentially.

#[test]
fn fillet_of_an_edge_ending_on_a_tangent_seam() {
    let mut base = block();
    base.push(r#"{"type":"fillet","id":"F3","radius":3,"edges":[{"faces":["F2.S.e1","F2.S.e2"],"at":[40,0,10]}]}"#.to_string());
    let rounded = run(&base);
    assert!(rounded.errors.is_empty(), "{:?}", rounded.errors);
    let v0 = vol(rounded.body.as_ref().unwrap());
    let r = 2.0;
    let section = (1.0 - std::f64::consts::FRAC_PI_4) * r * r;
    let mut f = base.clone();
    f.push(format!(r#"{{"type":"fillet","id":"F4","radius":{r},"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[15,0,20]}}]}}"#));
    let o = run(&f);
    assert!(o.errors.is_empty(), "refused: {:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    assert!(valid(b), "{:?}", ck_check::validate(b, 1e-6).iter().map(|i| i.code).collect::<Vec<_>>());
    // material removed: the straight run (37 mm) of the section, plus at most the part of the ball tool that overruns the seam
    let removed = v0 - vol(b);
    assert!(removed > 37.0 * section - 1e-6 && removed < (37.0 + 2.0 * r) * section, "removed {removed}, straight run {}", 37.0 * section);
    // with propagate the chain follows the seam round: more is removed
    let mut g = base.clone();
    g.push(format!(r#"{{"type":"fillet","id":"F4","radius":{r},"propagate":true,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[15,0,20]}}]}}"#));
    let p = run(&g);
    assert!(p.errors.is_empty(), "propagate refused: {:?}", p.errors);
    let pb = p.body.as_ref().unwrap();
    assert!(valid(pb));
    assert!(v0 - vol(pb) > removed - 1e-6);
}
