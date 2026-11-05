//! Equation-driven sketch curves (`spline` entity with `equation: {x, y, t, tol}`): the fitted spline
//! against the exact curve (circle, ellipse, Lissajous, a helix seen from the side), the fewest points
//! for the tolerance, part variables in the formulas, the regen echo and the structured errors.
use ck_json::Json;
use ck_model::expr::{resolve_variables, Variable, Vars};
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::sketch::equation::{self, EqError, Spec};
use ck_model::sketch::spline::InterpSpline;
use ck_model::Part;
use std::f64::consts::PI;

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

fn spec(x: &str, y: &str, t0: f64, t1: f64, tol: f64) -> Spec {
    Spec::from_json(&ck_json::parse(&format!(r#"{{"x":{x:?},"y":{y:?},"t":[{t0},{t1}],"tol":{tol}}}"#)).unwrap())
}

fn vars(l: &[(&str, &str)]) -> Vars {
    resolve_variables(&l.iter().map(|(n, e)| Variable { name: n.to_string(), expr: e.to_string() }).collect::<Vec<_>>()).values
}

/// Largest distance from the fitted spline (densely sampled) to a dense polyline of the exact curve, and back.
fn hausdorff(f: &equation::Fit, exact: impl Fn(f64) -> [f64; 2], t0: f64, t1: f64) -> f64 {
    let sp = InterpSpline::new(&f.pts, f.closed).unwrap();
    let n = 20000;
    let ex: Vec<[f64; 2]> = (0..=n).map(|i| exact(t0 + (t1 - t0) * i as f64 / n as f64)).collect();
    let near = |q: [f64; 2], poly: &[[f64; 2]]| -> f64 {
        poly.windows(2)
            .map(|w| {
                let (a, b) = (w[0], w[1]);
                let d = [b[0] - a[0], b[1] - a[1]];
                let l2 = d[0] * d[0] + d[1] * d[1];
                let u = if l2 > 0.0 { (((q[0] - a[0]) * d[0] + (q[1] - a[1]) * d[1]) / l2).clamp(0.0, 1.0) } else { 0.0 };
                (q[0] - a[0] - u * d[0]).hypot(q[1] - a[1] - u * d[1])
            })
            .fold(0.0f64 / 0.0, f64::min)
    };
    let m = 2000;
    let sp_pts: Vec<[f64; 2]> = (0..=m).map(|i| sp.eval(sp.length() * i as f64 / m as f64)).collect();
    let a = sp_pts.iter().map(|q| near(*q, &ex)).fold(0.0, f64::max);
    let b = ex.iter().step_by(10).map(|q| near(*q, &sp_pts)).fold(0.0, f64::max);
    a.max(b)
}

#[test]
fn circle_by_formula_is_within_tolerance_of_the_exact_circle() {
    let f = equation::fit(&spec("20*cos(t)", "20*sin(t)", 0.0, 360.0, 0.01), &Vars::new()).unwrap();
    assert!(f.closed, "a curve that ends where it starts is closed");
    let mut worst = 0.0f64;
    let sp = InterpSpline::new(&f.pts, true).unwrap();
    for i in 0..4000 {
        let p = sp.eval(sp.length() * i as f64 / 4000.0);
        worst = worst.max((p[0].hypot(p[1]) - 20.0).abs());
    }
    assert!(worst <= 0.01 + 1e-9, "radial error {worst}");
    assert!(f.deviation <= 0.01 && f.deviation > 0.0, "reported deviation {}", f.deviation);
    assert!(f.pts.len() <= 24, "a circle of radius 20 at 0.01 mm needs few points, got {}", f.pts.len());
}

#[test]
fn the_fewest_even_points_for_the_tolerance() {
    // no smaller even count meets the tolerance (and the split placement may beat even spacing)
    for (x, y, t1) in [("20*cos(t)", "20*sin(t)", 360.0), ("t/10", "8*sin(t*2)", 360.0), ("t", "t*t/40", 30.0)] {
        let s = spec(x, y, 0.0, t1, 0.01);
        let f = equation::fit(&s, &Vars::new()).unwrap();
        let n = f.pts.len();
        // the first even count within the tolerance is n itself, or above n (the split placement did better)
        if let Some(even) = (2..=n).find(|&k| equation::even_deviation(&s, &Vars::new(), k).unwrap() <= 0.01) {
            assert_eq!(even, n, "{x},{y}: {n} points but {even} even points suffice");
        }
    }
    // a straight line needs only its two ends
    let f = equation::fit(&spec("t", "2*t+1", 0.0, 10.0, 0.01), &Vars::new()).unwrap();
    assert_eq!(f.pts.len(), 2);
    assert!(!f.closed);
}

#[test]
fn ellipse_by_formula_matches_the_exact_ellipse() {
    let f = equation::fit(&spec("30*cos(t)", "12*sin(t)", 0.0, 360.0, 0.005), &Vars::new()).unwrap();
    assert!(f.closed);
    let d = hausdorff(&f, |t| [30.0 * (t * PI / 180.0).cos(), 12.0 * (t * PI / 180.0).sin()], 0.0, 360.0);
    assert!(d <= 0.005 + 1e-4, "ellipse deviation {d}");
}

#[test]
fn lissajous_is_within_tolerance_and_warns_that_it_crosses_itself() {
    let f = equation::fit(&spec("20*sin(3*t + 90)", "15*sin(2*t)", 0.0, 360.0, 0.02), &Vars::new()).unwrap();
    assert!(f.closed);
    let d = hausdorff(&f, |t| [20.0 * ((3.0 * t + 90.0) * PI / 180.0).sin(), 15.0 * ((2.0 * t) * PI / 180.0).sin()], 0.0, 360.0);
    assert!(d <= 0.02 + 2e-4, "lissajous deviation {d}");
    assert!(f.warnings.iter().any(|w| w.code == "equationSelfIntersects"), "{:?}", f.warnings);
    // a circle does not cross itself
    let c = equation::fit(&spec("20*cos(t)", "20*sin(t)", 0.0, 360.0, 0.01), &Vars::new()).unwrap();
    assert!(c.warnings.is_empty(), "{:?}", c.warnings);
}

#[test]
fn helix_seen_from_the_side_is_a_sine_wave_within_tolerance() {
    // helix radius 10, pitch 6 projected on its side plane: x = r cos(t), y = pitch * t / 360, two turns
    let f = equation::fit(&spec("10*cos(t)", "6*t/360", 0.0, 720.0, 0.01), &Vars::new()).unwrap();
    assert!(!f.closed);
    let d = hausdorff(&f, |t| [10.0 * (t * PI / 180.0).cos(), 6.0 * t / 360.0], 0.0, 720.0);
    assert!(d <= 0.01 + 1e-4, "helix projection deviation {d}");
    assert!((f.pts[0][0] - 10.0).abs() < 1e-9 && f.pts[0][1].abs() < 1e-9);
    let last = f.pts[f.pts.len() - 1];
    assert!((last[0] - 10.0).abs() < 1e-9 && (last[1] - 12.0).abs() < 1e-9, "{last:?}");
}

#[test]
fn formulas_and_range_use_part_variables() {
    let v = vars(&[("r", "15"), ("turns", "2"), ("t", "999")]);
    let s = Spec::from_json(&ck_json::parse(r#"{"x":"r*cos(t)","y":"=r*sin(t)","t":[0,"=360*turns/2"]}"#).unwrap());
    let f = equation::fit(&s, &v).unwrap();
    assert!(f.closed, "the curve variable t hides a part variable of the same name");
    assert!(f.pts.iter().all(|p| (p[0].hypot(p[1]) - 15.0).abs() < 1e-9));
    assert_eq!(f.tol, equation::DEFAULT_TOL);
}

fn code(e: &EqError) -> &'static str {
    e.code()
}

#[test]
fn errors_are_structured_with_plain_messages() {
    let none = Vars::new();
    let err = |x: &str, y: &str, t: &str| equation::fit(&Spec::from_json(&ck_json::parse(&format!(r#"{{"x":{x:?},"y":{y:?},"t":{t}}}"#)).unwrap()), &none).unwrap_err();
    let e = err("20*cos(t", "0", "[0,360]");
    assert_eq!(code(&e), "equationFormula");
    assert!(e.message().contains("x(t)"), "{}", e.message());
    let e = err("t", "q*t", "[0,10]");
    assert_eq!(code(&e), "equationFormula");
    assert!(e.message().contains("'q'"), "names the unknown name: {}", e.message());
    let e = err("", "t", "[0,10]");
    assert_eq!(code(&e), "equationFormula");
    let e = err("t", "sqrt(5 - t)", "[0,10]");
    assert_eq!(code(&e), "equationNotFinite");
    assert!(e.message().contains("y(t)"), "{}", e.message());
    let e = err("t", "1/(t-5)", "[0,10]");
    assert_eq!(code(&e), "equationNotFinite", "{}", e.message());
    let e = err("t", "t", "[5,5]");
    assert_eq!(code(&e), "equationRange");
    let e = err("t", "t", "[0]");
    assert_eq!(code(&e), "equationRange");
    let e = err("t", "t", "[0,\"=abc\"]");
    assert_eq!(code(&e), "equationRange");
    let e = err("3", "4", "[0,10]");
    assert_eq!(code(&e), "equationDegenerate");
    let tol = |t: &str| equation::fit(&Spec::from_json(&ck_json::parse(&format!(r#"{{"x":"t","y":"t","t":[0,1],"tol":{t}}}"#)).unwrap()), &none).unwrap_err();
    assert_eq!(code(&tol("0")), "equationTolerance");
    assert_eq!(code(&tol("-1")), "equationTolerance");
    // a curve that cannot meet a tight tolerance in the point budget is refused, saying so
    let e = equation::fit(&spec("40*sin(t*40)", "t/9", 0.0, 3600.0, 0.00001), &none).unwrap_err();
    assert_eq!(code(&e), "equationTolerance");
    assert!(e.message().contains("points"), "{}", e.message());
}

const DISC: &str = r#"{"format":"ck-part","version":1,"name":"eq-disc","variables":[{"name":"R","expr":"RADIUS"}],
 "features":[
  {"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[{"id":"O","x":0,"y":0,"fixed":true}],
   "entities":[{"type":"spline","id":"e1","pts":[],"equation":{"x":"R*cos(t)","y":"R*sin(t)","t":[0,360],"tol":0.001}}],"constraints":[]},
  {"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":10}
 ]}"#;

fn regen_disc(radius: &str) -> (regen::RegenResult, Brep) {
    let part = Part::parse(&DISC.replace("RADIUS", radius)).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (r, ex.0)
}

#[test]
fn regen_builds_the_points_from_the_formula_and_follows_a_variable() {
    for r in [10.0, 17.5] {
        let (res, brep) = regen_disc(&r.to_string());
        assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| &e.error.code).collect::<Vec<_>>());
        let sk = &res.sketches[0].sketch;
        let Some(ck_model::sketch::SketchEntity::Spline { pts, closed, .. }) = sk.entity("e1") else { panic!("spline e1") };
        assert!(*closed && pts.len() >= 8, "{pts:?}");
        assert_eq!(pts[0], "e1_start");
        assert!(pts.iter().all(|id| sk.point(id).map_or(false, |p| p.fixed && (p.x.hypot(p.y) - r).abs() < 1e-9)));
        let vol = ck_topo::mass::mass_props(brep.body.as_ref().unwrap()).volume;
        let exact = PI * r * r * 10.0;
        assert!((vol - exact).abs() / exact < 3e-4, "volume {vol} vs {exact}");
        // the echo keeps the formula and writes the generated points as fixed
        let echo = ck_json::to_string(&sk.to_json());
        assert!(echo.contains("\"equation\"") && echo.contains("\"e1_start\"") && echo.contains("\"fixed\":true"), "{echo}");
    }
}

#[test]
fn regen_replaces_stale_generated_points_and_keeps_other_points() {
    // a document saved with the points of an earlier radius: they are rebuilt from the formula, not trusted
    let stale = DISC.replace("RADIUS", "12").replace(
        r#""points":[{"id":"O","x":0,"y":0,"fixed":true}]"#,
        r#""points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"e1_start","x":99,"y":0,"fixed":true},{"id":"e1_p1","x":0,"y":99,"fixed":true},{"id":"e1_p77","x":5,"y":5,"fixed":true},{"id":"q","x":3,"y":4}]"#,
    );
    let part = Part::parse(&stale).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty());
    let sk = &res.sketches[0].sketch;
    assert!(sk.point("e1_p77").is_none(), "a generated id that is no longer used is dropped");
    assert!(sk.point("q").is_some() && sk.point("O").is_some());
    let p = sk.point("e1_start").unwrap();
    assert!((p.x - 12.0).abs() < 1e-9 && p.y.abs() < 1e-9);
}

#[test]
fn an_open_curve_names_its_ends_and_a_line_can_join_them() {
    // a wave from x=0 to x=40 closed by three lines into a profile; the lines end on the curve's named end points
    let src = r#"{"format":"ck-part","version":1,"name":"eq-wave","variables":[{"name":"A","expr":"3"}],
     "features":[
      {"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},
       "points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"a","x":40,"y":-10},{"id":"b","x":0,"y":-10}],
       "entities":[{"type":"spline","id":"w","pts":[],"equation":{"x":"t/9","y":"A*sin(t)","t":[0,360]}},
         {"type":"line","id":"l1","p1":"w_end","p2":"a"},{"type":"line","id":"l2","p1":"a","p2":"b"},{"type":"line","id":"l3","p1":"b","p2":"w_start"}],
       "constraints":[]},
      {"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":5}
     ]}"#;
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| &e.error.code).collect::<Vec<_>>());
    let vol = ck_topo::mass::mass_props(ex.0.body.as_ref().unwrap()).volume;
    // area under the wave over a whole period is that of the 40 x 10 rectangle
    assert!((vol - 40.0 * 10.0 * 5.0).abs() < 0.5, "volume {vol}");
}

#[test]
fn regen_errors_name_the_entity() {
    let part = Part::parse(&DISC.replace("RADIUS", "10").replace("R*sin(t)", "R*sin(t")).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let e = res.errors.iter().find(|e| e.feature == "S1").expect("sketch error");
    assert_eq!(e.error.code, "equationFormula");
    let p = |k: &str| e.error.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
    assert_eq!(p("entity"), Some(Json::Str("e1".into())));
    assert!(matches!(p("detail"), Some(Json::Str(s)) if s.contains("y(t)")));
}

#[test]
fn the_query_returns_points_or_a_coded_error() {
    let ok = equation::reply(&ck_json::parse(r#"{"type":"equationCurve","x":"r*cos(t)","y":"r*sin(t)","t":[0,360],"tol":0.01,"variables":[{"name":"r","expr":"20"}]}"#).unwrap()).unwrap();
    assert_eq!(ok.get("closed"), Some(&Json::Bool(true)));
    let n = ok.get("pts").and_then(|p| p.as_arr()).map(|a| a.len()).unwrap();
    assert_eq!(ok.get("count").and_then(|c| c.as_f64()), Some(n as f64));
    let bad = equation::reply(&ck_json::parse(r#"{"type":"equationCurve","x":"t","y":"t","t":[1,1]}"#).unwrap()).unwrap_err();
    assert!(bad.starts_with("equationRange: "), "{bad}");
}
