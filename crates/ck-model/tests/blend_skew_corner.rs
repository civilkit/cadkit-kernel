//! All edges of a convex prism / drafted prism rounded with one radius: the corners where the three rounds meet at
//! non-right angles (parallelogram, triangle and hexagon bases, drafted sides). The analytic volume is the Steiner
//! formula of the inner parallel body (the body eroded by r, then dilated by r): V(Q) + S(Q) r + r^2 sum(l (pi - theta) / 2)
//! + 4 pi r^3 / 3, computed from the half-space description of the part (the constants below came from a numeric
//! evaluation of that formula, not from the kernel).
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
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

fn run_all(features: &str) -> (Vec<String>, f64, usize) {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = res.errors.iter().map(|e| e.error.code.clone()).collect();
    let b = ex.0.body.as_ref().unwrap();
    (errs, ck_topo::mass::mass_props(b).volume, ck_topo::check::check(b, 1e-6).len())
}

/// Inward offset of a counter-clockwise polygon by `t` (adjacent offset lines intersected).
fn inset(p: &[(f64, f64)], t: f64) -> Vec<(f64, f64)> {
    let n = p.len();
    let line = |i: usize| {
        let (a, b) = (p[i], p[(i + 1) % n]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l = (dx * dx + dy * dy).sqrt();
        let (nx, ny) = (-dy / l, dx / l); // inward normal of a CCW polygon
        ((a.0 + nx * t, a.1 + ny * t), (dx, dy))
    };
    (0..n)
        .map(|i| {
            let ((p0, d0), (p1, d1)) = (line((i + n - 1) % n), line(i));
            let den = d0.0 * d1.1 - d0.1 * d1.0;
            let s = ((p1.0 - p0.0) * d1.1 - (p1.1 - p0.1) * d1.0) / den;
            (p0.0 + d0.0 * s, p0.1 + d0.1 * s)
        })
        .collect()
}

/// (errors, volume, validity issues) of the prism of the CCW polygon `p` (sketch plane TOP, extruded `h`, side draft
/// `draft_deg` inwards) with every edge rounded by `r`.
fn all_edges(p: &[(f64, f64)], h: f64, draft_deg: f64, r: f64) -> (Vec<String>, f64, usize) {
    let n = p.len();
    let top = if draft_deg == 0.0 { p.to_vec() } else { inset(p, h * draft_deg.to_radians().tan()) };
    let pts: Vec<String> = p.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, q.0, q.1)).collect();
    let ents: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % n + 1)).collect();
    let sk = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{}],"entities":[{}],"constraints":[]}}"#,
        pts.join(","),
        ents.join(",")
    );
    let draft = if draft_deg == 0.0 { String::new() } else { format!(r#","draft":{draft_deg}"#) };
    let ex = format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}{draft}}}"#);
    // world = (x, height, -y)
    let at = |a: (f64, f64, f64), b: (f64, f64, f64)| format!("[{},{},{}]", (a.0 + b.0) / 2.0, (a.2 + b.2) / 2.0, -(a.1 + b.1) / 2.0);
    let mut edges = vec![];
    for i in 0..n {
        let j = (i + 1) % n;
        let s = |k: usize| format!("F2.S.e{}", k + 1);
        edges.push(format!(r#"{{"faces":["{}","F2.C0"],"at":{}}}"#, s(i), at((p[i].0, p[i].1, 0.0), (p[j].0, p[j].1, 0.0))));
        edges.push(format!(r#"{{"faces":["{}","F2.C1"],"at":{}}}"#, s(i), at((top[i].0, top[i].1, h), (top[j].0, top[j].1, h))));
        // vertical edge at vertex j: faces e(i) and e(j)
        edges.push(format!(r#"{{"faces":["{}","{}"],"at":{}}}"#, s(i), s(j), at((p[j].0, p[j].1, 0.0), (top[j].0, top[j].1, h))));
    }
    run_all(&format!(r#"{sk},{ex},{{"type":"fillet","id":"F3","radius":{r},"edges":[{}]}}"#, edges.join(",")))
}

fn hexagon() -> Vec<(f64, f64)> {
    (0..6).map(|i| (20.0 * (60.0 * i as f64).to_radians().cos(), 20.0 * (60.0 * i as f64).to_radians().sin())).collect()
}
fn parallelogram() -> Vec<(f64, f64)> {
    let k = 30.0 * 3f64.sqrt() / 2.0;
    vec![(0.0, 0.0), (30.0, 0.0), (45.0, k), (15.0, k)]
}
fn square() -> Vec<(f64, f64)> {
    vec![(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]
}
fn triangle() -> Vec<(f64, f64)> {
    vec![(0.0, 0.0), (40.0, 0.0), (20.0, 40.0 * 3f64.sqrt() / 2.0)]
}

fn check(name: &str, p: Vec<(f64, f64)>, h: f64, d: f64, want: f64) {
    let (errs, vol, issues) = all_edges(&p, h, d, 2.0);
    let rel = (vol - want) / want;
    eprintln!("CASE {name}: vol {vol:.6} analytic {want:.6} rel {rel:.3e} errs {errs:?} issues {issues}");
    assert!(errs.is_empty(), "{name}: {errs:?}");
    assert_eq!(issues, 0, "{name}");
    assert!(rel.abs() < 1e-5, "{name}: {vol} vs {want} (rel {rel:.3e})");
}

#[test]
fn skew_parallelogram_prism() {
    check("para60", parallelogram(), 20.0, 0.0, 15279.22609376968);
}
#[test]
fn triangular_prism() {
    check("tri", triangle(), 20.0, 0.0, 13504.952078365826);
}
#[test]
fn hexagonal_prism() {
    check("hex", hexagon(), 15.0, 0.0, 15370.122688670588);
}
#[test]
fn drafted_box_5() {
    check("box_d5", square(), 20.0, 5.0, 28955.231356798322);
}
#[test]
fn drafted_box_15() {
    check("box_d15", square(), 20.0, 15.0, 23841.99692805085);
}
#[test]
fn drafted_hexagon() {
    check("hex_d10", hexagon(), 15.0, 10.0, 13108.42291942367);
}
#[test]
fn drafted_parallelogram() {
    check("para_d10", parallelogram(), 20.0, 10.0, 11440.773551373612);
}
#[test]
fn near_right_parallelogram_prism() {
    let (c, k) = (86f64.to_radians().cos() * 40.0, 86f64.to_radians().sin() * 40.0);
    check("quad86", vec![(0.0, 0.0), (40.0, 0.0), (40.0 + c, k), (c, k)], 12.0, 0.0, 18847.647967928136);
}
#[test]
fn triangular_prism_radius_4() {
    let (errs, vol, issues) = all_edges(&triangle(), 20.0, 0.0, 4.0);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0);
    assert!((vol - 12526.30158174065).abs() < 12526.3 * 1e-5, "{vol}");
}
