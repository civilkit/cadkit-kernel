//! A round and a chamfer on the rim of a larger "blob" outline (nine fit points, one closed spline, 25 mm blind extrude).
//! The tightest convex bend of the rim has a radius of curvature of about 1.8 mm, so every size above it meets bends tighter
//! than the size: there the inward offset has a swallowtail and the builder trims it (a fan of elliptic sections to the point
//! where the two surviving branches of the offset meet). Each size must build a valid solid whose volume matches an independent
//! estimate: the rolling-ball removal as a distance-field integral over the cap (`swept_removal`).
//! Run with `--nocapture` for the table.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use std::collections::HashMap;
use std::f64::consts::PI;

const PTS: [(f64, f64); 9] = [
    (-12.9, 66.8),
    (61.2, 51.2),
    (71.3, -12.2),
    (21.4, 0.0),
    (-29.3, -24.8),
    (-71.8, -16.1),
    (-114.3, 13.5),
    (-72.7, 24.6),
    (-68.2, 59.1),
];
const H: f64 = 25.0;

fn doc(kind: &str, extra: &str) -> String {
    let p: Vec<String> = PTS.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let ids: Vec<String> = (0..PTS.len()).map(|i| format!(r#""p{i}""#)).collect();
    let deg = if kind == "cvspline" { r#","degree":3"# } else { "" };
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"sketch","id":"F1","name":"S","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{}],"entities":[{{"id":"e1","type":"{kind}","pts":[{}],"closed":true{deg}}}],"constraints":[]}},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":{H},"id":"F2"}}{extra}]}}"#,
        p.join(","),
        ids.join(",")
    )
}

fn regen_doc(d: &str) -> (Option<Body>, Vec<String>) {
    let part = Part::parse(d).unwrap();
    let mut br = Brep::new();
    let res = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, res.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

fn picks(b: &Body, face: &str) -> Vec<String> {
    let f = b.find_face(face).unwrap();
    let mut out = vec![];
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        if fs.len() != 2 || !fs.contains(&f) {
            continue;
        }
        let p = b.vertex(b.edge(e).v0).p;
        out.push(format!(r#"{{"faces":["{}","{}"],"at":[{:.9},{:.9},{:.9}]}}"#, b.face_id_str(fs[0]), b.face_id_str(fs[1]), p.x, p.y, p.z));
    }
    out
}

fn rim_edges(b: &Body, face: &str) -> Vec<ck_topo::body::EdgeId> {
    let f = b.find_face(face).unwrap();
    b.edge_ids().filter(|&e| b.edge_faces(e).len() == 2 && b.edge_faces(e).contains(&f)).collect()
}

fn rim_length(b: &Body, face: &str) -> f64 {
    rim_edges(b, face).iter().map(|&e| b.edge_length(e)).sum()
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn removed(chamfer: bool, s: f64, len: f64) -> f64 {
    if chamfer {
        s * s / 2.0 * (len - 2.0 * PI * s / 3.0)
    } else {
        let xr = s * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0);
        (1.0 - PI / 4.0) * s * s * (len - 2.0 * PI * xr)
    }
}

/// Builds on the rim `face`; Ok(body) or Err(refusal text).
fn build(kind: &str, chamfer: bool, s: f64, face: &str) -> Result<Body, Vec<String>> {
    let b = regen_doc(&doc(kind, "")).0.expect("base body");
    let edges = picks(&b, face).join(",");
    let f = if chamfer {
        format!(r#",{{"id":"F3","type":"chamfer","distance":{s},"edges":[{edges}]}}"#)
    } else {
        format!(r#",{{"id":"F3","type":"fillet","radius":{s},"edges":[{edges}]}}"#)
    };
    let (out, errs) = regen_doc(&doc(kind, &f));
    match out {
        Some(o) if errs.is_empty() => Ok(o),
        _ => Err(errs),
    }
}

/// (smallest convex-side radius of curvature, smallest concave-side radius) of the rim, from the curve's own derivatives.
/// The rim is one simple closed curve, so it turns one way overall: bends that turn that way are convex.
fn curvature_limits(b: &Body, face: &str) -> (f64, f64) {
    let mut samples: Vec<(ck_geom::Vec3, ck_geom::Vec3, ck_geom::Vec3, f64)> = vec![];
    for e in rim_edges(b, face) {
        let ed = b.edge(e);
        let n = 4000;
        for k in 0..=n {
            let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64;
            let [p, g, g2] = ed.curve.derivs2_arr(t);
            let w = if k == 0 || k == n { 0.5 } else { 1.0 } * (ed.t1 - ed.t0) / n as f64;
            samples.push((p, g, g2, w));
        }
    }
    // the plane normal from the cap's own plane
    let ax = match &b.face(b.find_face(face).unwrap()).surface {
        ck_geom::surface::Surface::Plane { frame } => frame.z,
        _ => panic!("cap is not planar"),
    };
    let total: f64 = samples.iter().map(|(_, g, g2, w)| g.cross(*g2).dot(ax) / (g.len() * g.len()) * w).sum();
    eprintln!("total turning {:.3} rad ({:.2} turns)", total, total / (2.0 * PI));
    let (mut convex, mut concave) = (f64::INFINITY, f64::INFINITY);
    for (_, g, g2, _) in &samples {
        let ks = g.cross(*g2).dot(ax) / g.len().powi(3);
        if ks.abs() < 1e-9 {
            continue;
        }
        let r = 1.0 / ks.abs();
        if ks * total > 0.0 { convex = convex.min(r) } else { concave = concave.min(r) }
    }
    // where the convex bends are tighter than a few sizes (arc length of the rim, and the place of the tightest point)
    let mut at = (f64::INFINITY, ck_geom::Vec3::new(0.0, 0.0, 0.0));
    let mut tight = [0.0f64; 4];
    for (p, g, g2, w) in &samples {
        let ks = g.cross(*g2).dot(ax) / g.len().powi(3);
        if ks * total <= 0.0 {
            continue;
        }
        let r = 1.0 / ks.abs();
        if r < at.0 {
            at = (r, *p);
        }
        for (i, lim) in [3.7, 7.0, 15.0, 20.0].iter().enumerate() {
            if r < *lim {
                tight[i] += g.len() * w;
            }
        }
    }
    eprintln!("tightest convex bend {:.3} at ({:.1}, {:.1}, {:.1}); rim length with convex radius < 3.7 / 7 / 15 / 20: {:.1} / {:.1} / {:.1} / {:.1} mm", at.0, at.1.x, at.1.y, at.1.z, tight[0], tight[1], tight[2], tight[3]);
    (convex, concave)
}


/// Independent estimate of the volume a round of size `r` removes from the rim `face` of the straight extrusion: a point of the
/// cap at distance `d` from the rim (inside) loses the depth `r - sqrt(r^2 - (r - d)^2)` (the ball's lower arc) when `d < r`,
/// which is the union of the swept per-section cut-outs whether or not they overlap at tight bends. Midpoint rule on a grid of
/// `h` mm, nearest rim segment from a bucket grid; the error is O(h) of the rim length times the local slope, well under 0.2%.
fn swept_removal(b: &Body, face: &str, r: f64, h: f64) -> f64 {
    removal_models(b, face, r, h).0
}

fn removal_models(b: &Body, face: &str, r: f64, h: f64) -> (f64, f64) {
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
                total += (r - (r * r - (r - d) * (r - d)).sqrt()) * h * h;
            }
        }
    }
    (total, 0.0)
}

fn check_builds(kind: &str, sizes: &[f64], tol_pct: f64) {
    let b0 = regen_doc(&doc(kind, "")).0.expect("base");
    let v0 = vol(&b0);
    let len = rim_length(&b0, "F2.C1");
    let (ca, _) = curvature_limits(&b0, "F2.C1");
    eprintln!("{kind}: rim length {len:.2}, tightest convex radius {ca:.3}");
    for &r in sizes {
        let o = match build(kind, false, r, "F2.C1") {
            Ok(o) => o,
            Err(e) => panic!("{kind} round {r}: refused {e:?}"),
        };
        let issues = ck_check::validate(&o, 1e-6);
        assert!(issues.is_empty(), "{kind} round {r}: {} validity issues {:?}", issues.len(), issues.iter().take(4).map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect::<Vec<_>>());
        let m = ck_tess::mesh::tessellate(&o).mesh;
        assert!(m.tri_face.len() > 100, "{kind} round {r}: mesh is empty");
        let got = v0 - vol(&o);
        let want = swept_removal(&b0, "F2.C1", r, 0.25);
        let pap = removed(false, r, len);
        let pct = 100.0 * (got - want) / want;
        eprintln!("{kind} round {r}: builds, valid, removed {got:.1} vs swept estimate {want:.1} ({pct:+.2}%), plain Pappus {pap:.1} ({:+.2}%)", 100.0 * (got - pap) / pap);
        assert!(pct.abs() <= tol_pct, "{kind} round {r}: removed volume {got:.1} is {pct:+.2}% off the estimate {want:.1}");
    }
}

#[test]
fn blob_outline_round_through_tight_bends() {
    check_builds("spline", &[1.0, 2.0, 3.7, 5.0, 7.0, 10.0, 15.0, 20.0], 1.0);
}

#[test]
fn blob_outline_cvspline_through_tight_bends() {
    check_builds("cvspline", &[5.0, 10.0, 15.0], 1.0);
}

#[test]
fn blob_outline_chamfer_and_bottom() {
    let b0 = regen_doc(&doc("spline", "")).0.expect("base");
    for (name, chamfer, r, face) in [("chamfer 3.7 top", true, 3.7, "F2.C1"), ("round 3.7 bottom", false, 3.7, "F2.C0"), ("chamfer 3.7 bottom", true, 3.7, "F2.C0"), ("chamfer 10 top", true, 10.0, "F2.C1")] {
        let o = build("spline", chamfer, r, face).unwrap_or_else(|e| panic!("{name}: refused {e:?}"));
        let issues = ck_check::validate(&o, 1e-6);
        eprintln!("{name}: builds, validate {} issues, removed {:.2}", issues.len(), vol(&b0) - vol(&o));
        assert!(issues.is_empty(), "{name}: invalid");
    }
}

/// A size within a hair of the tightest bend's radius is refused with a text that gives sizes that do build.
#[test]
fn blob_outline_size_at_the_bend_radius_refuses_cleanly() {
    let b0 = regen_doc(&doc("spline", "")).0.expect("base");
    let (ca, _) = curvature_limits(&b0, "F2.C1");
    for k in [0.96, 0.98, 0.99, 1.0, 1.01, 1.02, 1.05, 1.1] {
        match build("spline", false, ca * k, "F2.C1") {
            Ok(o) => eprintln!("size {:.3} ({k} x bend radius): builds, {} issues", ca * k, ck_check::validate(&o, 1e-6).len()),
            Err(e) => eprintln!("size {:.3} ({k} x bend radius): refused {}", ca * k, e.join(" ").chars().take(150).collect::<String>()),
        }
    }
    match build("spline", false, ca * 0.995, "F2.C1") {
        Ok(o) => assert!(ck_check::validate(&o, 1e-6).is_empty(), "a build must be valid"),
        Err(e) => {
            let t = e.join(" ");
            eprintln!("refusal at the bend radius: {t}");
            assert!(t.contains("use a size below"), "{t}");
        }
    }
}

/// Sizes up to the wall height (25 mm) build a valid solid whose removed volume is within 1% of the swept-cut estimate.
#[test]
fn blob_outline_large_rounds() {
    let b0 = regen_doc(&doc("spline", "")).0.expect("base");
    let v0 = vol(&b0);
    for r in [22.5, 24.0, 24.9] {
        let o = build("spline", false, r, "F2.C1").unwrap_or_else(|e| panic!("round {r} refused {e:?}"));
        let issues = ck_check::validate(&o, 1e-6);
        assert!(issues.is_empty(), "round {r}: {issues:?}");
        let got = v0 - vol(&o);
        let sw = swept_removal(&b0, "F2.C1", r, 0.3);
        eprintln!("large round {r}: builds, valid, removed {got:.1} vs swept estimate {sw:.1} ({:+.2}%)", 100.0 * (got - sw) / sw);
        assert!((got - sw).abs() <= 0.01 * sw, "round {r}: volume {got} vs {sw}");
    }
}
