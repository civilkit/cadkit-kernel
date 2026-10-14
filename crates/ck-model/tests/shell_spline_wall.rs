//! Shell (and moveFace) of an extruded closed free-form outline: the kidney / bean shaped spline
//! wall. The wall is an extrusion of a B-spline (a closed sketch spline is a two-lap curve, split
//! into two wall faces), its offset the extrusion of the offset curve. Checks analytic wall
//! volumes against an independently offset polygon, validity, and the structured too-thick hints
//! (smallest radius found, thickness that would work).
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

/// Bean outline: a dent on top, rounded ends, a full belly.
const KIDNEY: [(f64, f64); 7] = [(-30.0, 0.0), (-20.0, 15.0), (0.0, 9.0), (20.0, 15.0), (30.0, 0.0), (15.0, -15.0), (-15.0, -15.0)];
const H: f64 = 5.0;

fn part(kind: &str, extra: &str) -> String {
    part_h(kind, H, extra)
}

fn part_h(kind: &str, h: f64, extra: &str) -> String {
    let p: Vec<String> = KIDNEY.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let ids: Vec<String> = (0..KIDNEY.len()).map(|i| format!("\"p{i}\"")).collect();
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
 {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{}],
  "entities":[{{"type":"{kind}","id":"e1","pts":[{}],"closed":true{}}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}{extra}
]}}"#,
        p.join(","),
        ids.join(","),
        if kind == "cvspline" { r#","degree":3"# } else { "" }
    )
}

struct Out {
    brep: Brep,
    errors: Vec<regen::RegenError>,
}

fn run(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}

fn shell_feat(faces: &[&str], t: f64, outward: bool) -> String {
    let f: Vec<String> = faces.iter().map(|s| format!("\"{s}\"")).collect();
    format!(r#",{{"type":"shell","id":"F3","faces":[{}],"thickness":{t},"outward":{outward}}}"#, f.join(","))
}

/// Outline of the bottom cap as a closed polygon (dense samples of its spline edges) and its area.
fn outline(o: &Out) -> (Vec<(f64, f64)>, f64) {
    let b = o.brep.body.as_ref().unwrap();
    let cap = b.face_ids().find(|&f| b.face_id_str(f) == "F2.C0").unwrap();
    let mut pts = vec![];
    for &l in &b.face(cap).loops {
        for &c in &b.lp(l).coedges {
            let (ta, tb) = b.coedge_params(c);
            let ed = b.edge(b.coedge(c).edge);
            for k in 0..4000 {
                let p = ed.curve.eval(ta + (tb - ta) * k as f64 / 4000.0);
                pts.push((p.x, p.y));
            }
        }
    }
    let a = area(&pts);
    (pts, a)
}

fn area(p: &[(f64, f64)]) -> f64 {
    let n = p.len();
    0.5 * (0..n).map(|i| p[i].0 * p[(i + 1) % n].1 - p[(i + 1) % n].0 * p[i].1).sum::<f64>()
}

/// The polygon moved by `d` along its outward normal (positive grows), signed area independent of winding.
fn grown(p: &[(f64, f64)], d: f64) -> f64 {
    let n = p.len();
    let ccw = area(p) > 0.0;
    let q: Vec<(f64, f64)> = (0..n)
        .map(|i| {
            let (a, c) = (p[(i + n - 1) % n], p[(i + 1) % n]);
            let (tx, ty) = (c.0 - a.0, c.1 - a.1);
            let l = tx.hypot(ty);
            // outward = right of travel for ccw
            let (nx, ny) = if ccw { (ty / l, -tx / l) } else { (-ty / l, tx / l) };
            (p[i].0 + d * nx, p[i].1 + d * ny)
        })
        .collect();
    area(&q).abs()
}

/// Smallest radius of curvature of the polygon on the inner side (the centre of curvature inside).
fn inner_min_radius(p: &[(f64, f64)]) -> f64 {
    let n = p.len();
    let ccw = area(p) > 0.0;
    let step = 40;
    let mut best = f64::INFINITY;
    for i in (0..n).step_by(5) {
        let (a, b, c) = (p[(i + n - step) % n], p[i], p[(i + step) % n]);
        let cross = (b.0 - a.0) * (c.1 - b.1) - (b.1 - a.1) * (c.0 - b.0);
        // the inside is the left of a ccw loop: a left turn has its centre inside
        if (cross > 0.0) != ccw {
            continue;
        }
        let (ab, bc, ca) = ((b.0 - a.0).hypot(b.1 - a.1), (c.0 - b.0).hypot(c.1 - b.1), (a.0 - c.0).hypot(a.1 - c.1));
        let r = ab * bc * ca / (2.0 * cross.abs());
        best = best.min(r);
    }
    best
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

fn param(e: &regen::FeatureError, k: &str) -> Option<Json> {
    e.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone())
}

fn check_valid(o: &Out, what: &str) {
    let b = o.brep.body.as_ref().unwrap();
    let iss = ck_check::validate(b, 1e-3);
    assert!(iss.is_empty(), "{what}: {:?}", iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
}

fn shell_variants(kind: &str) {
    let base = run(&part(kind, ""));
    assert!(base.errors.is_empty(), "{kind}: {:?}", base.errors);
    let v0 = vol(&base);
    let (poly, a0) = outline(&base);
    assert!(rel(v0, a0.abs() * H) < 2e-3, "{kind}: base volume {v0} vs {}", a0.abs() * H);
    let a0 = a0.abs();
    let t = 1.0;
    let a_in = grown(&poly, -t);
    let a_out = grown(&poly, t);
    // (faces to open, outward, expected wall volume)
    let cases: [(&[&str], bool, f64, &str); 4] = [
        (&["F2.C1"], false, v0 - a_in * (H - t), "inward, top open"),
        (&[], false, v0 - a_in * (H - 2.0 * t), "inward, closed"),
        (&["F2.C1"], true, a_out * (H + t) - v0, "outward, top open"),
        (&[], true, a_out * (H + 2.0 * t) - v0, "outward, closed"),
    ];
    for (open, outward, expect, what) in cases {
        let r = run(&part(kind, &shell_feat(open, t, outward)));
        assert!(r.errors.is_empty(), "{kind} {what}: {:?}", r.errors);
        check_valid(&r, &format!("{kind} {what}"));
        let v = vol(&r);
        assert!(rel(v, expect) < 4e-3, "{kind} {what}: volume {v} vs analytic {expect} (base {v0}, area {a0}, inner {a_in}, outer {a_out})");
        // wall volume = (outer area - inner area) x height, to first order
        if !outward && open.is_empty() {
            let ring = (a0 - a_in) * H;
            assert!(v > ring && v < ring + 2.0 * a_in * t * 1.0 + 1.0, "{kind} {what}: {v} vs ring {ring}");
        }
    }
}

#[test]
fn spline_kidney_shells_in_all_variants() {
    shell_variants("spline");
}

#[test]
fn cvspline_kidney_shells_in_all_variants() {
    shell_variants("cvspline");
}

#[test]
fn too_thick_reports_radius_and_working_thickness() {
    let tall = 20.0;
    let base = run(&part_h("spline", tall, ""));
    let (poly, _) = outline(&base);
    let r_min = inner_min_radius(&poly);
    assert!(r_min.is_finite() && r_min > 3.0 && r_min < 30.0, "radius {r_min}");
    // well past the tightest radius
    let t = (1.6 * r_min).min(9.0);
    let r = run(&part_h("spline", tall, &shell_feat(&["F2.C1"], t, false)));
    assert_eq!(r.errors.len(), 1, "{:?}", r.errors);
    let e = &r.errors[0].error;
    assert_eq!(e.code, "shellTooThick", "{}", e.describe());
    let detail = param(e, "detail").and_then(|j| j.as_str().map(str::to_string)).unwrap_or_default();
    assert!(detail.contains("use at most"), "detail: {detail}");
    assert!(!detail.contains("simplify"), "generic text left: {detail}");
    let min_radius = param(e, "minRadius").and_then(|j| j.as_f64()).expect("minRadius");
    let max_t = param(e, "maxThickness").and_then(|j| j.as_f64()).expect("maxThickness");
    assert!(param(e, "face").is_some());
    assert!(max_t > 0.0 && max_t < t && max_t <= min_radius, "max {max_t} min radius {min_radius} thickness {t}");
    // the reported radius is the real one (curvature limit)
    if param(e, "limit").and_then(|j| j.as_str().map(str::to_string)).as_deref() == Some("curvature") {
        assert!(rel(min_radius, r_min) < 0.05, "min radius {min_radius} vs independent {r_min}");
    }
    // and the suggested thickness works
    let ok = run(&part_h("spline", tall, &shell_feat(&["F2.C1"], max_t, false)));
    assert!(ok.errors.is_empty(), "thickness {max_t} (suggested): {:?}", ok.errors);
    check_valid(&ok, "suggested thickness");
}

#[test]
fn move_face_of_the_spline_wall() {
    let walls = |o: &Out| -> Vec<String> {
        let b = o.brep.body.as_ref().unwrap();
        b.face_ids().filter(|&f| b.face(f).surface.kind() == "extrusion").map(|f| b.face_id_str(f)).collect()
    };
    let base = run(&part("spline", ""));
    let v0 = vol(&base);
    let (poly, a0) = outline(&base);
    let a0 = a0.abs();
    let names = walls(&base);
    assert_eq!(names.len(), 2);
    let faces = names.iter().map(|n| format!("\"{n}\"")).collect::<Vec<_>>().join(",");
    for d in [1.0, -1.0] {
        let mf = format!(r#",{{"type":"moveFace","id":"F3","faces":[{faces}],"distance":{d}}}"#);
        let r = run(&part("spline", &mf));
        assert!(r.errors.is_empty(), "moveFace {d}: {:?}", r.errors);
        check_valid(&r, "moveFace");
        let expect = grown(&poly, d) * H;
        let v = vol(&r);
        assert!(rel(v, expect) < 4e-3, "moveFace {d}: volume {v} vs {expect} (base {v0}, area {a0})");
    }
    // far past the tightest radius: structured hint, no panic
    let r_min = inner_min_radius(&poly);
    let mf = format!(r#",{{"type":"moveFace","id":"F3","faces":[{faces}],"distance":{}}}"#, -(2.0 * r_min));
    let r = run(&part("spline", &mf));
    assert_eq!(r.errors.len(), 1, "{:?}", r.errors);
    let e = &r.errors[0].error;
    assert!(e.code == "moveFaceFailed" || e.code == "moveFaceUnsupported", "{}", e.describe());
    assert!(param(e, "maxThickness").is_some() && param(e, "minRadius").is_some(), "{}", e.describe());
}

/// Manual probe: `FWT=5.6 FWK=spline cargo test ... probe_thickness -- --ignored --nocapture`.
#[test]
#[ignore]
fn probe_thickness() {
    let t: f64 = std::env::var("FWT").ok().and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let kind = std::env::var("FWK").unwrap_or("spline".into());
    let r = run(&part(&kind, &shell_feat(&["F2.C1"], t, std::env::var("FWO").is_ok())));
    eprintln!("errors {:?} vol {:?}", r.errors, r.brep.body.as_ref().map(|_| vol(&r)));
}
