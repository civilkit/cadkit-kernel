//! The closed-spline families of the bean matrix (bean, ellipse, soft heart, wavy lobes, teardrop, few and many control points)
//! rounded and chamfered on the top, bottom and both rims. Every case either builds a valid solid whose removed volume is the
//! Pappus estimate of the rim (within 1 %), or, where a bend of the rim is tighter than the size (the Pappus estimate no longer
//! holds there), the distance-field estimate `swept_removal` (within 2.5 %: the swept-cut model is the lower bound of what a tight tip removes, the fan patch removes a little more), or is refused with a named error: never a broken solid. `FAMILIES=1` prints the table.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use std::collections::HashMap;
use std::f64::consts::PI;

const TAU: f64 = std::f64::consts::TAU;

fn sample(f: impl Fn(f64) -> (f64, f64), n: usize) -> Vec<(f64, f64)> {
    (0..n).map(|i| f(TAU * i as f64 / n as f64)).collect()
}
fn bean(s: f64, n: usize, d: f64, w: f64) -> Vec<(f64, f64)> {
    sample(|t| (s * 0.5 * t.cos(), s * 0.3 * t.sin() - d * s * 0.3 * (-((t.cos() / w).powi(2))).exp() * t.sin().max(0.0)), n)
}
fn ellipse(s: f64, n: usize) -> Vec<(f64, f64)> {
    sample(|t| (s * 0.5 * t.cos(), s * 0.3 * t.sin()), n)
}
fn heart(s: f64, n: usize, mix: f64) -> Vec<(f64, f64)> {
    sample(
        |t| {
            let x = s * 0.5 * t.sin().powi(3);
            let y = s * 0.0325 * (13.0 * t.cos() - 5.0 * (2.0 * t).cos() - 2.0 * (3.0 * t).cos() - (4.0 * t).cos());
            ((1.0 - mix) * x + mix * s * 0.5 * t.sin(), (1.0 - mix) * y + mix * s * 0.4 * t.cos())
        },
        n,
    )
}
fn wavy(s: f64, lobes: f64, a: f64, n: usize) -> Vec<(f64, f64)> {
    sample(|t| { let r = s * 0.5 * (1.0 + a * (lobes * t).cos()); (r * t.cos(), r * t.sin()) }, n)
}
fn teardrop(s: f64, n: usize, tip: f64) -> Vec<(f64, f64)> {
    sample(|t| (s * 0.5 * t.cos(), s * 0.32 * t.sin() * ((1.0 - t.cos()) / 2.0 + tip)), n)
}

fn families() -> Vec<(&'static str, Vec<(f64, f64)>)> {
    vec![
        ("bean50", bean(50.0, 16, 0.55, 0.55)),
        ("bean100", bean(100.0, 16, 0.55, 0.55)),
        ("bean-deep", bean(60.0, 20, 0.9, 0.5)),
        ("bean-tight", bean(30.0, 24, 1.1, 0.25)),
        ("bean-tight2", bean(50.0, 32, 1.2, 0.18)),
        ("bean-ctl8", bean(50.0, 8, 0.4, 0.55)),
        ("bean-ctl24", bean(50.0, 24, 0.55, 0.55)),
        ("ellipse20", ellipse(20.0, 12)),
        ("ellipse50", ellipse(50.0, 12)),
        ("ellipse-ctl4", ellipse(50.0, 4)),
        ("ellipse-ctl6", ellipse(50.0, 6)),
        ("heart30", heart(30.0, 16, 0.35)),
        ("heart60", heart(60.0, 20, 0.35)),
        ("heart100", heart(100.0, 24, 0.5)),
        ("wavy3", wavy(40.0, 3.0, 0.15, 24)),
        ("wavy3-100", wavy(100.0, 3.0, 0.12, 24)),
        ("wavy5", wavy(40.0, 5.0, 0.1, 30)),
        ("wavy5-100", wavy(100.0, 5.0, 0.1, 30)),
        ("teardrop30", teardrop(30.0, 16, 0.04)),
        ("teardrop60", teardrop(60.0, 20, 0.02)),
        ("teardrop100", teardrop(100.0, 24, 0.06)),
    ]
}

fn doc(pts: &[(f64, f64)], kind: &str, extra: &str) -> String {
    let p: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x:.6},"y":{y:.6}}}"#)).collect();
    let ids: Vec<String> = (0..pts.len()).map(|i| format!(r#""p{i}""#)).collect();
    let deg = if kind == "cvspline" { r#","degree":3"# } else { "" };
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"sketch","id":"F1","name":"S","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{}],"entities":[{{"id":"e1","type":"{kind}","pts":[{}],"closed":true{deg}}}],"constraints":[]}},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":10,"id":"F2"}}{extra}]}}"#,
        p.join(","),
        ids.join(",")
    )
}

fn regen_doc(d: &str) -> (Option<Body>, Vec<String>) {
    let part = Part::parse(d).unwrap();
    let mut br = Brep::new();
    let res = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, res.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect())
}

fn picks(b: &Body, face: &str) -> Vec<String> {
    let f = b.find_face(face).unwrap();
    b.edge_ids()
        .filter(|&e| b.edge_faces(e).len() == 2 && b.edge_faces(e).contains(&f))
        .map(|e| {
            let fs = b.edge_faces(e);
            let p = b.vertex(b.edge(e).v0).p;
            format!(r#"{{"faces":["{}","{}"],"at":[{:.9},{:.9},{:.9}]}}"#, b.face_id_str(fs[0]), b.face_id_str(fs[1]), p.x, p.y, p.z)
        })
        .collect()
}

fn rim_length(b: &Body, face: &str) -> f64 {
    let f = b.find_face(face).unwrap();
    b.edge_ids().filter(|&e| b.edge_faces(e).len() == 2 && b.edge_faces(e).contains(&f)).map(|e| b.edge_length(e)).sum()
}

fn removed(chamfer: bool, s: f64, len: f64) -> f64 {
    if chamfer {
        s * s / 2.0 * (len - 2.0 * PI * s / 3.0)
    } else {
        let xr = s * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0);
        (1.0 - PI / 4.0) * s * s * (len - 2.0 * PI * xr)
    }
}

/// Independent estimate of the volume a round of size `r` removes from the rim `face` of the straight extrusion: a point of the
/// cap at distance `d` from the rim (inside) loses the depth `r - sqrt(r^2 - (r - d)^2)` (the ball's lower arc) when `d < r`,
/// which is the union of the swept per-section cut-outs whether or not they overlap at tight bends. Midpoint rule on a grid of
/// `h` mm, nearest rim segment from a bucket grid; the error is O(h) of the rim length times the local slope. A chamfer loses `s - d`.
fn rim_edges(b: &Body, face: &str) -> Vec<ck_topo::body::EdgeId> {
    let f = b.find_face(face).unwrap();
    b.edge_ids().filter(|&e| b.edge_faces(e).len() == 2 && b.edge_faces(e).contains(&f)).collect()
}

fn swept_removal(b: &Body, face: &str, r: f64, chamfer: bool, h: f64) -> f64 {
    let ax = match &b.face(b.find_face(face).unwrap()).surface {
        ck_geom::surface::Surface::Plane { frame } => *frame,
        _ => panic!(),
    };
    let to2 = |p: ck_geom::Vec3| ((p - ax.o).dot(ax.x), (p - ax.o).dot(ax.y));
    let mut pts: Vec<(f64, f64)> = vec![];
    let mut segs: Vec<((f64, f64), (f64, f64))> = vec![];
    for e in rim_edges(b, face) {
        let ed = b.edge(e);
        let len = ed.curve.length_between(ed.t0, ed.t1);
        let n = ((len / 0.1).ceil() as usize).max(8);
        let q: Vec<(f64, f64)> = (0..=n).map(|k| to2(ed.curve.derivs2_arr(ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64)[0])).collect();
        for w in q.windows(2) {
            segs.push((w[0], w[1]));
        }
        pts.extend(q);
    }
    let (mut x0, mut x1, mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for p in &pts {
        x0 = x0.min(p.0);
        x1 = x1.max(p.0);
        y0 = y0.min(p.1);
        y1 = y1.max(p.1);
    }
    let cell = r.max(1.0);
    let key = |x: f64, y: f64| (((x - x0) / cell).floor() as i64, ((y - y0) / cell).floor() as i64);
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, (a, c)) in segs.iter().enumerate() {
        let (ka, kc) = (key(a.0.min(c.0), a.1.min(c.1)), key(a.0.max(c.0), a.1.max(c.1)));
        for kx in ka.0..=kc.0 {
            for ky in ka.1..=kc.1 {
                grid.entry((kx, ky)).or_default().push(i);
            }
        }
    }
    let mut total = 0.0;
    let ny = ((y1 - y0) / h).ceil() as usize;
    let nx = ((x1 - x0) / h).ceil() as usize;
    for j in 0..ny {
        let y = y0 + (j as f64 + 0.5) * h;
        // crossings of the scanline with the rim: inside between pairs
        let mut xs: Vec<f64> = vec![];
        for (a, c) in &segs {
            if (a.1 <= y) != (c.1 <= y) {
                xs.push(a.0 + (y - a.1) / (c.1 - a.1) * (c.0 - a.0));
            }
        }
        xs.sort_by(|p, q| p.partial_cmp(q).unwrap());
        for i in 0..nx {
            let x = x0 + (i as f64 + 0.5) * h;
            let inside = xs.iter().filter(|&&v| v < x).count() % 2 == 1;
            if !inside {
                continue;
            }
            let (kx, ky) = key(x, y);
            let mut d2 = f64::INFINITY;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if let Some(v) = grid.get(&(kx + dx, ky + dy)) {
                        for &si in v {
                            let (a, c) = segs[si];
                            let (ex, ey) = (c.0 - a.0, c.1 - a.1);
                            let t = (((x - a.0) * ex + (y - a.1) * ey) / (ex * ex + ey * ey)).clamp(0.0, 1.0);
                            let (px, py) = (a.0 + t * ex - x, a.1 + t * ey - y);
                            d2 = d2.min(px * px + py * py);
                        }
                    }
                }
            }
            let d = d2.sqrt();
            if d < r {
                total += if chamfer { r - d } else { r - (r * r - (r - d) * (r - d)).sqrt() } * h * h;
            }
        }
    }
    total
}

#[test]
fn closed_spline_families_build_or_refuse_by_name() {
    let (mut built, mut refused, mut total) = (0, 0, 0);
    let mut broken: Vec<String> = vec![];
    let mut unnamed: Vec<String> = vec![];
    let show = std::env::var_os("FAMILIES").is_some();
    for (name, pts) in families() {
        for kind in ["spline", "cvspline"] {
            let (b0, e0) = regen_doc(&doc(&pts, kind, ""));
            let Some(b0) = b0 else { assert!(!e0.is_empty()); continue };
            if !e0.is_empty() {
                continue;
            }
            let v0 = ck_topo::mass::mass_props(&b0).volume;
            for (rims, rl) in [(&["F2.C1"][..], "top"), (&["F2.C0"][..], "bottom"), (&["F2.C1", "F2.C0"][..], "both")] {
                for (chamfer, s) in [(false, 0.5), (false, 1.0), (false, 2.0), (false, 5.0), (true, 1.0), (true, 2.0)] {
                    total += 1;
                    let edges: Vec<String> = rims.iter().flat_map(|f| picks(&b0, f)).collect();
                    let f = if chamfer { format!(r#",{{"id":"F3","type":"chamfer","distance":{s},"edges":[{}]}}"#, edges.join(",")) } else { format!(r#",{{"id":"F3","type":"fillet","radius":{s},"edges":[{}]}}"#, edges.join(",")) };
                    let t = std::time::Instant::now();
                    let (b, errs) = regen_doc(&doc(&pts, kind, &f));
                    let secs = t.elapsed().as_secs_f64();
                    let tag = format!("{name}/{kind}/{rl}/{}{s}", if chamfer { "c" } else { "r" });
                    match (b, errs.is_empty()) {
                        (Some(b), true) => {
                            let bad = ck_check::validate(&b, 1e-6);
                            let want: f64 = rims.iter().map(|f| removed(chamfer, s, rim_length(&b0, f))).sum();
                            let got = v0 - ck_topo::mass::mass_props(&b).volume;
                            // beyond the smooth-rim formula (a bend tighter than the size): the distance-field estimate
                            let off = (got - want).abs() > 0.01 * want.abs() + 1e-3;
                            let want2: f64 = if off && bad.is_empty() { rims.iter().map(|f| swept_removal(&b0, f, s, chamfer, (s / 8.0).clamp(0.03, 0.2))).sum() } else { want };
                            if !bad.is_empty() || (off && (got - want2).abs() > 0.025 * want2.abs() + 1e-3) {
                                broken.push(format!("{tag}: {} issues, removed {got:.3} against {want:.3} (swept estimate {want2:.3})", bad.len()));
                            } else {
                                built += 1;
                            }
                            if show { println!("{tag}: built {secs:.2}s"); }
                        }
                        (_, _) => {
                            refused += 1;
                            if !errs.iter().all(|e| e.contains("blend") || e.contains("Overflow") || e.contains("invalidValue")) {
                                unnamed.push(format!("{tag}: {errs:?}"));
                            }
                            if show { println!("{tag}: refused {errs:?} {secs:.2}s"); }
                        }
                    }
                    assert!(secs < 30.0, "{tag}: took {secs:.1}s");
                }
            }
        }
    }
    println!("closed-spline families: {total} cases, {built} built, {refused} refused");
    assert!(broken.is_empty(), "broken results: {broken:#?}");
    assert!(unnamed.is_empty(), "unnamed refusals: {unnamed:#?}");
    assert!(built * 10 >= total * 6, "only {built} of {total} build");
}
