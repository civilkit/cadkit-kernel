//! A plain extruded quadrilateral with every one of its twelve edges rounded (no overhang): the exact result is
//! the Minkowski sum of the prism pulled in by `r` and a ball of radius `r`, so its volume is the Steiner sum
//! `V' + S' r + M' r^2 + 4/3 pi r^3` of the inset prism (`M' = pi H' + pi p'/2`) and a point is inside the body
//! exactly where its distance to the inset prism is at most `r`. Both are checked for acute and obtuse vertical
//! wall angles, over radii, mirrored and rotated: a notch or a blob at a corner shows in either.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::region::{classify, Containment};

struct Exec(Brep);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

/// The reported trapezoid (sketch points), counter-clockwise.
const TRAP: [(f64, f64); 4] = [(-31.492154, 14.863514), (37.493198, 14.863514), (46.485156, 37.5), (-31.492154, 37.5)];
const H: f64 = 10.0;

fn doc(poly: &[(f64, f64)], r: f64) -> String {
    let n = poly.len();
    let pts: Vec<String> = poly.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, p.0, p.1)).collect();
    let ents: Vec<String> = (0..n).map(|i| format!(r#"{{"id":"e{}","type":"line","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % n + 1)).collect();
    // sketch (x, y) on the top plane is world (x, ., -y); the prism spans world y 0..H
    let w = |p: (f64, f64), y: f64| format!("[{},{},{}]", p.0, y, -p.1);
    let mut picks = vec![];
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        let m = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
        for (cap, y) in [("C1", H), ("C0", 0.0)] {
            picks.push(format!(r#"{{"faces":["F2.{cap}","F2.S.e{}"],"at":{}}}"#, i + 1, w(m, y)));
        }
        // vertical edge at the start of side i: between side i and side i - 1
        picks.push(format!(r#"{{"faces":["F2.S.e{}","F2.S.e{}"],"at":{}}}"#, (i + n - 1) % n + 1, i + 1, w(a, H / 2.0)));
    }
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{}],"entities":[{}],"constraints":[]}},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":{H},"id":"F2"}},{{"type":"fillet","radius":{r},"id":"F3","edges":[{}]}}]}}"#,
        pts.join(","),
        ents.join(","),
        picks.join(",")
    )
}

fn regen_body(src: &str) -> Result<ck_topo::body::Body, String> {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    if let Some(e) = res.errors.first() {
        let d = e.error.params.iter().find(|(k, _)| k == "detail").and_then(|(_, v)| v.as_str()).unwrap_or("").to_string();
        return Err(format!("{} {d}", e.error.code));
    }
    Ok(ex.0.body.clone().unwrap())
}

/// The polygon pulled in by `r` (convex, counter-clockwise).
fn inset(poly: &[(f64, f64)], r: f64) -> Vec<(f64, f64)> {
    let n = poly.len();
    let line = |i: usize| {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        let l = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
        let (dx, dy) = ((b.0 - a.0) / l, (b.1 - a.1) / l);
        // inward normal of a counter-clockwise polygon is the left of the direction
        let (nx, ny) = (-dy, dx);
        ((a.0 + nx * r, a.1 + ny * r), (dx, dy))
    };
    (0..n)
        .map(|i| {
            let ((p, d), (q, e)) = (line((i + n - 1) % n), line(i));
            let t = ((q.0 - p.0) * e.1 - (q.1 - p.1) * e.0) / (d.0 * e.1 - d.1 * e.0);
            (p.0 + d.0 * t, p.1 + d.1 * t)
        })
        .collect()
}

fn area_perimeter(p: &[(f64, f64)]) -> (f64, f64) {
    let n = p.len();
    let (mut a, mut l) = (0.0, 0.0);
    for i in 0..n {
        let (u, v) = (p[i], p[(i + 1) % n]);
        a += u.0 * v.1 - v.0 * u.1;
        l += ((v.0 - u.0).powi(2) + (v.1 - u.1).powi(2)).sqrt();
    }
    (a / 2.0, l)
}

fn dist_to_polygon(p: &[(f64, f64)], q: (f64, f64)) -> f64 {
    let n = p.len();
    let inside = (0..n).all(|i| {
        let (a, b) = (p[i], p[(i + 1) % n]);
        (b.0 - a.0) * (q.1 - a.1) - (b.1 - a.1) * (q.0 - a.0) >= 0.0
    });
    if inside {
        return 0.0;
    }
    (0..n)
        .map(|i| {
            let (a, b) = (p[i], p[(i + 1) % n]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let t = (((q.0 - a.0) * dx + (q.1 - a.1) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            ((q.0 - a.0 - dx * t).powi(2) + (q.1 - a.1 - dy * t).powi(2)).sqrt()
        })
        .fold(f64::MAX, f64::min)
}

fn check(poly: &[(f64, f64)], r: f64, label: &str) {
    let src = doc(poly, r);
    let b = regen_body(&src).unwrap_or_else(|e| panic!("{label} r={r}: {e}"));
    let issues = ck_topo::check::check(&b, 1e-6);
    assert!(issues.is_empty(), "{label} r={r}: invalid body {issues:?}");
    let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    // 6 planes, 12 edge rounds, 8 corner patches
    assert_eq!(names.len(), 26, "{label} r={r}: faces {names:?}");
    let spheres = b.face_ids().filter(|&f| matches!(b.face(f).surface, ck_geom::surface::Surface::Sphere { .. })).count();
    assert_eq!(spheres, 8, "{label} r={r}: sphere corner patches in {names:?}");
    let ip = inset(poly, r);
    let (a, p) = area_perimeter(&ip);
    let hh = H - 2.0 * r;
    let want = a * hh + (2.0 * a + p * hh) * r + (std::f64::consts::PI * hh + std::f64::consts::PI * p / 2.0) * r * r + 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
    let v = ck_topo::mass::mass_props(&b).volume;
    assert!((v - want).abs() < 1e-4 * want, "{label} r={r}: volume {v} vs analytic {want}");
    // membership just inside / outside the analytic rounded surface, densest around the corners
    let bad = std::cell::RefCell::new(vec![]);
    let probe = |q: (f64, f64), y: f64| {
        let dxy = dist_to_polygon(&ip, q);
        let dy = if y < r { r - y } else if y > H - r { y - (H - r) } else { 0.0 };
        let d = (dxy * dxy + dy * dy).sqrt();
        let m = 0.02 * r.max(0.5);
        let want = if d < r - m { Containment::Inside } else if d > r + m { Containment::Outside } else { return };
        let got = classify(&b, ck_geom::Vec3::new(q.0, y, -q.1), 1e-6);
        if got != want {
            bad.borrow_mut().push(format!("({:.3},{:.3},{:.3}) want {want:?} got {got:?}", q.0, q.1, y));
        }
    };
    let n = poly.len();
    for (i, &c) in poly.iter().enumerate() {
        let (pa, pb) = (poly[(i + n - 1) % n], poly[(i + 1) % n]);
        let dir = |t: (f64, f64)| {
            let l = ((t.0 - c.0).powi(2) + (t.1 - c.1).powi(2)).sqrt();
            ((t.0 - c.0) / l, (t.1 - c.1) / l)
        };
        let (da, db) = (dir(pa), dir(pb));
        let bis = (da.0 + db.0, da.1 + db.1);
        let bl = (bis.0 * bis.0 + bis.1 * bis.1).sqrt();
        let bis = (bis.0 / bl, bis.1 / bl);
        // points along the bisector and fans around it, at several heights and distances from the corner
        for k in 0..=12 {
            let s = r * 3.0 * k as f64 / 12.0;
            for sw in [-0.6f64, 0.0, 0.6] {
                let t = (bis.0 * (1.0 - sw.abs()) + da.0 * sw.max(0.0) + db.0 * (-sw).max(0.0), bis.1 * (1.0 - sw.abs()) + da.1 * sw.max(0.0) + db.1 * (-sw).max(0.0));
                for y in [0.1 * r, 0.5 * r, r, H / 2.0, H - r, H - 0.5 * r, H - 0.1 * r] {
                    probe((c.0 + t.0 * s, c.1 + t.1 * s), y);
                }
            }
        }
    }
    let bad = bad.into_inner();
    assert!(bad.is_empty(), "{label} r={r}: {} membership errors, e.g. {:?}", bad.len(), &bad[..bad.len().min(6)]);
}

fn rotated(poly: &[(f64, f64)], deg: f64) -> Vec<(f64, f64)> {
    let (s, c) = deg.to_radians().sin_cos();
    poly.iter().map(|p| (p.0 * c - p.1 * s + 5.0, p.0 * s + p.1 * c - 3.0)).collect()
}

fn mirrored(poly: &[(f64, f64)]) -> Vec<(f64, f64)> {
    poly.iter().rev().map(|p| (-p.0, p.1)).collect()
}

#[test]
fn trapezoid_prism_all_edges_rounded_at_four_radii() {
    for r in [0.5, 1.0, 2.0, 4.0] {
        check(&TRAP, r, "trapezoid");
    }
}

#[test]
fn mirrored_and_rotated_trapezoids_round_the_same_way() {
    for r in [1.0, 2.0] {
        check(&mirrored(&TRAP), r, "mirrored");
        check(&rotated(&TRAP, 37.0), r, "rotated 37");
        check(&rotated(&mirrored(&TRAP), 200.0), r, "mirrored+rotated");
    }
}

#[test]
fn very_acute_and_very_obtuse_vertical_wall_angles() {
    // 45 and 135 degree corners, and a 25 degree sliver-ish wedge corner
    let p45 = [(0.0, 0.0), (40.0, 0.0), (25.0, 15.0), (0.0, 15.0)];
    let p25 = [(0.0, 0.0), (40.0, 0.0), (22.0, 8.0), (0.0, 8.0)];
    for r in [0.5, 1.0, 2.0] {
        check(&p45, r, "45deg");
        check(&p25, r, "25deg");
    }
}
