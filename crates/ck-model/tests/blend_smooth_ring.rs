//! A round on the rim of an extruded closed smooth free-form profile (a closed spline, an ellipse-like spline, a bean):
//! refused at once with a named reason; a profile with a corner (a spline closed by a line) still builds.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use std::time::Instant;

const TAU: f64 = std::f64::consts::TAU;

fn closed_spline(arr: &[(f64, f64)]) -> String {
    let pts: Vec<String> = arr.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x:.6},"y":{y:.6}}}"#)).collect();
    let ids: Vec<String> = (0..arr.len()).map(|i| format!(r#""p{i}""#)).collect();
    format!(
        r#"{{"type":"sketch","id":"F1","name":"S","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{}],"entities":[{{"id":"e1","type":"spline","pts":[{}],"closed":true}}],"constraints":[]}}"#,
        pts.join(","),
        ids.join(",")
    )
}

fn ellipse(s: f64, n: usize) -> String {
    let a: Vec<_> = (0..n).map(|i| TAU * i as f64 / n as f64).map(|t| (s * 0.5 * t.cos(), s * 0.3 * t.sin())).collect();
    closed_spline(&a)
}

fn bean(s: f64, n: usize) -> String {
    let (d, w) = (0.55, 0.55);
    let a: Vec<_> = (0..n)
        .map(|i| TAU * i as f64 / n as f64)
        .map(|t| (s * 0.5 * t.cos(), s * 0.3 * t.sin() - d * s * 0.3 * (-((t.cos() / w).powi(2))).exp() * t.sin().max(0.0)))
        .collect();
    closed_spline(&a)
}

fn wavy(s: f64, lobes: f64, a: f64, n: usize) -> String {
    let v: Vec<_> = (0..n)
        .map(|i| TAU * i as f64 / n as f64)
        .map(|t| {
            let r = s * 0.5 * (1.0 + a * (lobes * t).cos());
            (r * t.cos(), r * t.sin())
        })
        .collect();
    closed_spline(&v)
}

/// An open spline over the top, closed by a line (a profile with two corners).
fn spline_line(s: f64, n: usize) -> String {
    let a: Vec<(f64, f64)> = (0..n)
        .map(|i| {
            let t = std::f64::consts::PI * i as f64 / (n - 1) as f64;
            (s * 0.5 * t.cos(), s * 0.3 * t.sin() + if i % 2 == 1 { 0.02 * s } else { 0.0 })
        })
        .collect();
    let pts: Vec<String> = a.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x:.6},"y":{y:.6}}}"#)).collect();
    let ids: Vec<String> = (0..n).map(|i| format!(r#""p{i}""#)).collect();
    format!(
        r#"{{"type":"sketch","id":"F1","name":"S","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{}],"entities":[{{"id":"e1","type":"spline","pts":[{}]}},{{"id":"e2","type":"line","p1":"p{}","p2":"p0"}}],"constraints":[]}}"#,
        pts.join(","),
        ids.join(","),
        n - 1
    )
}

struct Out {
    body: Option<Body>,
    errors: Vec<String>,
    secs: f64,
}

/// A pick (faces + a point on the edge) for every edge of the cap `face`, read off the plain extrusion.
fn rim_picks(sketch: &str, depth: f64, face: &str) -> Vec<String> {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{sketch},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":{depth},"id":"F2"}}]}}"#);
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    regen::regen(&part, &RegenOpts::default(), &mut br);
    let b = br.body.expect("base body");
    let f = b.find_face(face).unwrap_or_else(|| panic!("no face {face}"));
    let mut out = vec![];
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        if fs.len() != 2 || !fs.contains(&f) {
            continue;
        }
        let ed = b.edge(e);
        let (p0, p1) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
        let p = if ed.v0 == ed.v1 { p0 } else if matches!(ed.curve, ck_geom::curve::Curve::Line { .. }) { (p0 + p1) * 0.5 } else { p0 };
        out.push(format!(r#"{{"faces":["{}","{}"],"at":[{:.9},{:.9},{:.9}]}}"#, b.face_id_str(fs[0]), b.face_id_str(fs[1]), p.x, p.y, p.z));
    }
    out
}

fn run(sketch: &str, depth: f64, face: &str, r: f64) -> Out {
    let edges = rim_picks(sketch, depth, face).join(",");
    let doc = format!(
        r#"{{"format":"ck-part","version":1,"features":[{sketch},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":{depth},"id":"F2"}},{{"id":"F3","type":"fillet","radius":{r},"edges":[{edges}]}}]}}"#
    );
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let t = Instant::now();
    let res = regen::regen(&part, &RegenOpts::default(), &mut br);
    let secs = t.elapsed().as_secs_f64();
    let errors = res.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect();
    Out { body: br.body, errors, secs }
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn plain_vol(sketch: &str, depth: f64) -> f64 {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{sketch},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":{depth},"id":"F2"}}]}}"#);
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    regen::regen(&part, &RegenOpts::default(), &mut br);
    vol(br.body.as_ref().unwrap())
}

fn assert_refused_named(name: &str, sketch: &str, edges: &str, r: f64) {
    let o = run(sketch, 10.0, edges, r);
    assert!(o.body.is_none() || !o.errors.is_empty(), "{name}: expected a refusal");
    let all = format!("{:?}", o.errors);
    assert!(all.contains("blendUnsupported"), "{name}: code: {all}");
    assert!(all.contains("closed smooth free-form curve"), "{name}: detail: {all}");
    assert!(all.contains("corner"), "{name}: hint: {all}");
    let limit = if cfg!(debug_assertions) { 20.0 } else { 3.0 };
    assert!(o.secs < limit, "{name}: refusal took {:.1}s", o.secs);
}

#[test]
fn closed_spline_rim_refused_named_and_fast() {
    assert_refused_named("ellipse top", &ellipse(50.0, 12), "F2.C1", 1.0);
    assert_refused_named("ellipse top r5", &ellipse(50.0, 12), "F2.C1", 5.0);
    assert_refused_named("bean top", &bean(50.0, 16), "F2.C1", 1.0);
    assert_refused_named("wavy top", &wavy(40.0, 5.0, 0.1, 30), "F2.C1", 1.0);
    assert_refused_named("ellipse bottom", &ellipse(50.0, 12), "F2.C0", 1.0);
}

#[test]
fn spline_closed_by_line_still_builds() {
    for (s, n, r) in [(50.0, 8, 1.0), (20.0, 8, 0.5), (100.0, 10, 2.0)] {
        let sk = spline_line(s, n);
        let o = run(&sk, 10.0, "F2.C1", r);
        assert!(o.errors.is_empty(), "spline+line s{s} r{r}: {:?}", o.errors);
        let b = o.body.expect("body");
        assert!(ck_check::validate(&b, 1e-6).is_empty(), "spline+line s{s} r{r} invalid");
        let removed = plain_vol(&sk, 10.0) - vol(&b);
        assert!(removed > 0.0, "spline+line s{s} r{r}: nothing removed");
    }
}
