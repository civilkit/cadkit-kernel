//! Analytic geometry written as B-splines (straight edges, circles, flat faces) is read as lines, circles and planes,
//! without changing the body: valid, same volume, same faces.
mod spline_step;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_io::step::{read_step, read_step_plain};
use ck_topo::body::Body;

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn kinds(b: &Body) -> (usize, usize, usize, usize) {
    let lines = b.edges.iter().filter(|e| matches!(e.curve, Curve::Line { .. })).count();
    let circles = b.edges.iter().filter(|e| matches!(e.curve, Curve::Circle { .. })).count();
    let splines = b.edges.iter().filter(|e| matches!(e.curve, Curve::BSpline(_))).count();
    let planes = b.faces.iter().filter(|f| matches!(f.surface, Surface::Plane { .. })).count();
    (lines, circles, splines, planes)
}

#[test]
fn box_with_spline_edges_and_faces_reads_analytic() {
    let text = spline_step::box_step(20.0, 10.0, 6.0, true);
    let plain = read_step_plain(&text).unwrap();
    let rec = read_step(&text).unwrap();
    assert_eq!(kinds(&plain.body), (0, 0, 12, 0), "the plain read keeps the splines");
    assert_eq!(kinds(&rec.body), (12, 0, 0, 6), "recognised: 12 lines, 6 planes");
    assert!(rec.parts[0].refused.is_empty());
    assert_eq!(rec.parts[0].faces.len(), 6);
    assert!(ck_check::validate(&rec.body, 1e-6).is_empty(), "{:?}", ck_check::validate(&rec.body, 1e-6));
    assert!((vol(&rec.body) - 1200.0).abs() < 1e-9 * 1200.0, "{}", vol(&rec.body));
    assert!((vol(&rec.body) - vol(&plain.body)).abs() < 1e-9 * 1200.0);
    // the analytic version is the same body as one written with LINE and PLANE
    let exact = read_step(&spline_step::box_step(20.0, 10.0, 6.0, false)).unwrap();
    assert_eq!(kinds(&exact.body), kinds(&rec.body));
    assert!((vol(&exact.body) - vol(&rec.body)).abs() < 1e-9 * 1200.0);
}

#[test]
fn cylinder_with_rational_circles_and_flat_caps_reads_analytic() {
    let text = spline_step::cylinder_step(5.0, 10.0, true);
    let plain = read_step_plain(&text).unwrap();
    let rec = read_step(&text).unwrap();
    let want = std::f64::consts::PI * 25.0 * 10.0;
    assert_eq!(kinds(&rec.body).1, 2, "both rim circles recognised");
    assert_eq!(kinds(&rec.body).2, 0);
    assert_eq!(kinds(&rec.body).3, 2, "both caps planes");
    assert!(rec.parts[0].refused.is_empty(), "{:?}", rec.parts[0].refused);
    assert!(ck_check::validate(&rec.body, 1e-6).is_empty(), "{:?}", ck_check::validate(&rec.body, 1e-6));
    assert!((vol(&rec.body) - want).abs() < 1e-9 * want, "{} vs {want}", vol(&rec.body));
    // the spline circles' own volume is quadrature-accurate to ~5e-7 only; the recognised one is exact
    assert!((vol(&rec.body) - vol(&plain.body)).abs() < 1e-5 * want, "{} vs plain {}", vol(&rec.body), vol(&plain.body));
}

#[test]
fn near_miss_stays_a_spline() {
    // one control point of the last face's net moved out of its plane by 1e-4 (the net points come after the vertices): the face is
    // not flat, the read keeps the spline surface and the other five become planes
    let text = spline_step::box_step(20.0, 10.0, 6.0, true);
    let at = text.rfind("(20.0,10.0,6.0)").unwrap();
    let bent = format!("{}(20.0,10.0001,6.0){}", &text[..at], &text[at + "(20.0,10.0,6.0)".len()..]);
    let r = read_step(&bent).unwrap();
    let (_, _, _, planes) = kinds(&r.body);
    let splines = r.body.faces.iter().filter(|f| matches!(f.surface, Surface::BSpline(_))).count();
    assert_eq!((planes, splines), (5, 1), "a warped face must not become a plane");
}
