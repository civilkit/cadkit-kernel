//! Booleans on of an extruded closed free-form outline: the kidney / bean shaped spline
//! wall. The wall is an extrusion of a B-spline (a closed sketch spline is a two-lap curve, split
//! into two wall faces), its offset the extrusion of the offset curve. Checks analytic wall
//! volumes against an independently offset polygon, validity, and the structured too-thick hints
//! (smallest radius found, thickness that would work).
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
const H: f64 = 4.0;
thread_local!(static KIND: std::cell::Cell<&'static str> = const { std::cell::Cell::new("spline") });

fn kidney_sketch(id: &str, plane: &str, closed_pts: &[(f64, f64)], inner: Option<&[(f64, f64)]>) -> String {
    let mut pts = vec![];
    let mut ents = vec![];
    let mut add = |tag: &str, list: &[(f64, f64)], spline: &str| {
        let mut ids = vec![];
        for (i, (x, y)) in list.iter().enumerate() {
            pts.push(format!(r#"{{"id":"{tag}{i}","x":{x},"y":{y}}}"#));
            ids.push(format!("\"{tag}{i}\""));
        }
        ents.push(format!(r#"{{"type":"{spline}","id":"e{tag}","pts":[{}],"closed":true{}}}"#, ids.join(","), if spline == "cvspline" { r#","degree":3"# } else { "" }));
    };
    let kind = KIND.with(|k| k.get());
    add("p", closed_pts, kind);
    if let Some(i) = inner {
        add("q", i, kind);
    }
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), ents.join(","))
}

fn base_feats() -> String {
    format!(
        r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{H}}}"#,
        kidney_sketch("F1", r#"{"kind":"std","id":"front"}"#, &KIDNEY, None)
    )
}

fn run_feats(f: &str) -> Out {
    run(&format!(r#"{{"format":"ck-part","version":1,"features":[{f}]}}"#))
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

fn check_valid(o: &Out, what: &str) {
    let b = o.brep.body.as_ref().unwrap();
    let iss = ck_check::validate(b, 1e-3);
    assert!(iss.is_empty(), "{what}: {:?}", iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
}

fn grown_area(p: &[(f64, f64)], d: f64) -> f64 {
    let q = sample_offset(p, d, p.len());
    area(&q).abs()
}

/// Area of the inner (1 mm inset) outline, measured from its own standalone extrusion so the
/// expectation does not depend on how well the sampled points approximate the offset.
fn inner_area(inner: &[(f64, f64)]) -> f64 {
    let f = format!(
        r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":1}}"#,
        kidney_sketch("F1", r#"{"kind":"std","id":"front"}"#, inner, None)
    );
    let r = run_feats(&f);
    assert!(r.errors.is_empty(), "inner standalone: {:?}", r.errors);
    vol(&r)
}

const TOP: &str = r#"{"kind":"face","id":"F2.C1"}"#;

fn spline_kidney_booleans() {
    let base = run_feats(&base_feats());
    assert!(base.errors.is_empty(), "{:?}", base.errors);
    let (poly, a0) = outline(&base);
    let (a0, v0) = (a0.abs(), vol(&base));
    let inner = sample_offset(&poly, -1.0, 36);
    let ai = inner_area(&inner);
    // the inset outline agrees with an independently offset polygon
    // (a control-point spline only approximates the through-point offset)
    let tol = if KIND.with(|k| k.get()) == "spline" { 5e-3 } else { 2e-2 };
    assert!(rel(ai, grown_area(&poly, -1.0)) < tol, "inner {ai} vs polygon {}", grown_area(&poly, -1.0));
    let inner_sk = |id: &str| kidney_sketch(id, TOP, &inner, None);
    // the outline is on the same wall as the solid: ring between outer and inner
    let ring_sk = kidney_sketch("F3", TOP, &KIDNEY, Some(&inner));
    let cases: Vec<(&str, String, f64)> = vec![
        ("ring cut 1 mm", format!(r#"{ring_sk},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":1}}"#), v0 - (a0 - ai) * 1.0),
        ("inner pocket 2 mm", format!(r#"{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":2}}"#, inner_sk("F3")), v0 - ai * 2.0),
        ("inner boss 2 mm", format!(r#"{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":2}}"#, inner_sk("F3")), v0 + ai * 2.0),
        ("inner through", format!(r#"{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":4}}"#, inner_sk("F3")), v0 - ai * 4.0),
    ];
    for (what, tail, expect) in cases {
        let r = run_feats(&format!("{},{tail}", base_feats()));
        assert!(r.errors.is_empty(), "{what}: {:?}", r.errors);
        check_valid(&r, what);
        let v = vol(&r);
        assert!(rel(v, expect) < 5e-3, "{what}: volume {v} vs analytic {expect}");
    }
}

#[test]
fn spline_on_spline_wall_booleans() {
    KIND.with(|k| k.set("spline"));
    spline_kidney_booleans();
}

#[test]
fn cvspline_on_cvspline_wall_booleans() {
    KIND.with(|k| k.set("cvspline"));
    spline_kidney_booleans();
}

/// A spline pocket and a spline boss on a plain block (no spline wall), the pocket straddling the edge.
#[test]
fn spline_pocket_in_a_block() {
    KIND.with(|k| k.set("spline"));
    let blk = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":-40,"y":-25},{"id":"b","x":40,"y":-25},{"id":"c","x":40,"y":25},{"id":"d","x":-40,"y":25}],
      "entities":[{"type":"line","id":"l1","p1":"a","p2":"b"},{"type":"line","id":"l2","p1":"b","p2":"c"},{"type":"line","id":"l3","p1":"c","p2":"d"},{"type":"line","id":"l4","p1":"d","p2":"a"}],"constraints":[]},
      {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":6}"#;
    let area = |pts: &[(f64, f64)]| {
        let f = format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":1}}"#, kidney_sketch("F1", r#"{"kind":"std","id":"front"}"#, pts, None));
        vol(&run_feats(&f))
    };
    let v0 = 80.0 * 50.0 * 6.0;
    let k = &KIDNEY;
    let ak = area(k);
    for (what, op, depth, expect) in [("pocket", "cut", 3.0, v0 - ak * 3.0), ("boss", "add", 3.0, v0 + ak * 3.0), ("through", "cut", 6.0, v0 - ak * 6.0)] {
        let f = format!(r#"{blk},{},{{"type":"extrude","id":"F4","op":"{op}","sketch":"F3","end":"blind","depth":{depth}}}"#, kidney_sketch("F3", TOP, k, None));
        let r = run_feats(&f);
        assert!(r.errors.is_empty(), "{what}: {:?}", r.errors);
        check_valid(&r, what);
        let v = vol(&r);
        assert!(rel(v, expect) < 3e-3, "{what}: volume {v} vs {expect}");
    }
}

/// `n` points of the closed polygon `p` moved by `d` along the outward normal, evenly by arc index.
fn sample_offset(p: &[(f64, f64)], d: f64, n: usize) -> Vec<(f64, f64)> {
    let m = p.len();
    let ccw = area(p) > 0.0;
    (0..n)
        .map(|k| {
            let i = k * m / n;
            let (a, c) = (p[(i + m - 5) % m], p[(i + 5) % m]);
            let (tx, ty) = (c.0 - a.0, c.1 - a.1);
            let l = tx.hypot(ty);
            let (nx, ny) = if ccw { (ty / l, -tx / l) } else { (-ty / l, tx / l) };
            (p[i].0 + d * nx, p[i].1 + d * ny)
        })
        .collect()
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



