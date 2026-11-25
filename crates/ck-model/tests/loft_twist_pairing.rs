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

pub struct Out {
    pub brep: Brep,
    pub errors: Vec<regen::RegenError>,
    pub warnings: Vec<regen::RegenError>,
}

pub fn run_part(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.brep, errors: r.errors, warnings: r.warnings }
}

// A plain circle / ellipse / polygon lofted to a plain polygon (the owner's circle r25 to a 50x30 rectangle 40 mm above, loft
// defaults): the automatic start pairing must not twist. The twist is read from the seams (the edges of the sides that run
// along the loft): the polar angle of each seam's two ends about the section centres must agree (a twisted pairing gives a
// seam running diagonally round the loft, 100+ degrees off).

const H: f64 = 40.0;

enum Sec {
    Circle(f64),
    /// semi axis along x, semi axis along y
    Ellipse(f64, f64),
    /// corners in drawing order (the first one is the start of the section)
    Poly(Vec<(f64, f64)>),
}

fn sketch(id: &str, plane: &str, c: (f64, f64), s: &Sec) -> String {
    match s {
        Sec::Circle(r) => format!(
            r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"O","x":{},"y":{}}}],"entities":[{{"id":"e1","type":"circle","c":"O","r":{r}}}],"constraints":[]}}"#,
            c.0, c.1
        ),
        Sec::Ellipse(a, b) => format!(
            r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"O","x":{},"y":{}}},{{"id":"Q","x":{},"y":{}}}],"entities":[{{"id":"e1","type":"ellipse","c":"O","p":"Q","r":{b}}}],"constraints":[]}}"#,
            c.0, c.1, c.0 + a, c.1
        ),
        Sec::Poly(p) => {
            let n = p.len();
            let pts: Vec<String> = p.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"p{i}","x":{},"y":{}}}"#, c.0 + q.0, c.1 + q.1)).collect();
            let es: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"l{i}","p1":"p{i}","p2":"p{}"}}"#, (i + 1) % n)).collect();
            format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), es.join(","))
        }
    }
}

fn loft2(a: &Sec, ca: (f64, f64), b: &Sec, cb: (f64, f64), extra: &str) -> Out {
    let front = r#"{"kind":"std","id":"front"}"#;
    let f = [
        format!(r#"{{"type":"plane","id":"P1","mode":"offset","base":{front},"offset":{H}}}"#),
        sketch("S0", front, ca, a),
        sketch("S1", r#"{"kind":"feature","id":"P1"}"#, cb, b),
        format!(r#"{{"type":"loft","id":"F8","op":"add","sketches":["S0","S1"]{extra}}}"#),
    ];
    run_part(&format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(",")))
}

fn rect(hx: f64, hy: f64, start: usize) -> Sec {
    let c = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)];
    Sec::Poly((0..4).map(|i| c[(i + start) % 4]).collect())
}

fn ngon(n: usize, r: f64, deg0: f64) -> Sec {
    Sec::Poly((0..n).map(|i| { let a = (deg0 + 360.0 * i as f64 / n as f64).to_radians(); (r * a.cos(), r * a.sin()) }).collect())
}

/// The worst difference (degrees, mod 360) between the polar angles of a seam's two ends, each about its own section centre.
fn twist(o: &Out, ca: (f64, f64), cb: (f64, f64)) -> (f64, usize) {
    let b = o.brep.body.as_ref().expect("a body");
    let (mut worst, mut seams) = (0.0f64, 0);
    for e in b.edge_ids() {
        let ed = b.edge(e);
        let (mut p0, mut p1) = (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1));
        if (p0.z - p1.z).abs() < 1.0 {
            continue;
        }
        if p0.z > p1.z {
            std::mem::swap(&mut p0, &mut p1);
        }
        seams += 1;
        let a0 = (p0.y - ca.1).atan2(p0.x - ca.0).to_degrees();
        let a1 = (p1.y - cb.1).atan2(p1.x - cb.0).to_degrees();
        worst = worst.max(((a1 - a0 + 540.0).rem_euclid(360.0) - 180.0).abs());
    }
    (worst, seams)
}

/// Inside-out triangles of the mesh: a triangle whose normal points at the section axis rather than away from it.
fn assert_valid(name: &str, o: &Out) {
    assert!(o.errors.is_empty(), "{name}: {:?}", o.errors);
    let b = o.brep.body.as_ref().expect("a body");
    let v = ck_check::validate(b, 1e-6);
    assert!(v.is_empty(), "{name}: {v:?}");
}

#[test]
fn plain_sections_pair_without_a_twist() {
    let z = (0.0, 0.0);
    let off = (10.0, 5.0);
    // the last field: the most twist (degrees) the pairing may have. Two sections that are the same shape turned by half their corner
    // spacing cannot do better than that turn (every pairing twists by it).
    type Case = (&'static str, Sec, (f64, f64), Sec, (f64, f64), f64);
    let cases: Vec<Case> = vec![
        ("circle r25 -> rectangle 50x30", Sec::Circle(25.0), z, rect(25.0, 15.0, 0), z, 20.0),
        ("rectangle 50x30 -> circle", rect(25.0, 15.0, 0), z, Sec::Circle(25.0), z, 20.0),
        ("circle -> rectangle starting at the third corner", Sec::Circle(25.0), z, rect(25.0, 15.0, 2), z, 20.0),
        ("circle -> tall rectangle 30x50", Sec::Circle(25.0), z, rect(15.0, 25.0, 1), z, 20.0),
        ("circle -> hexagon", Sec::Circle(25.0), z, ngon(6, 25.0, 0.0), z, 20.0),
        ("circle -> hexagon starting at 20 degrees", Sec::Circle(25.0), z, ngon(6, 25.0, 20.0), z, 20.0),
        ("circle -> triangle", Sec::Circle(25.0), z, ngon(3, 25.0, 90.0), z, 20.0),
        ("circle -> rectangle, offset centre", Sec::Circle(25.0), z, rect(25.0, 15.0, 0), off, 20.0),
        ("ellipse -> rectangle", Sec::Ellipse(25.0, 15.0), z, rect(25.0, 15.0, 0), z, 20.0),
        ("ellipse -> hexagon", Sec::Ellipse(25.0, 15.0), z, ngon(6, 25.0, 0.0), z, 20.0),
        ("square -> square rotated 45", rect(20.0, 20.0, 0), z, ngon(4, 20.0 * 2f64.sqrt(), 90.0), z, 45.5),
        ("square -> square rotated 45, offset", rect(20.0, 20.0, 0), z, ngon(4, 20.0 * 2f64.sqrt(), 90.0), off, 45.5),
        ("rectangle -> rectangle, other start corner", rect(25.0, 15.0, 0), z, rect(20.0, 12.0, 2), z, 20.0),
        ("triangle -> hexagon", ngon(3, 25.0, 90.0), z, ngon(6, 25.0, 0.0), z, 30.5),
    ];
    let mut bad = vec![];
    for (name, a, ca, b, cb, limit) in &cases {
        let o = loft2(a, *ca, b, *cb, "");
        let (t, seams) = twist(&o, *ca, *cb);
        eprintln!("TWIST {name}: {t:.1} degrees over {seams} seams, errors {}", o.errors.len());
        if !o.errors.is_empty() || seams < 3 || t > *limit {
            bad.push(format!("{name}: twist {t:.1} degrees, {seams} seams, {:?}", o.errors.iter().map(|e| e.error.code.clone()).collect::<Vec<_>>()));
        } else {
            assert_valid(name, &o);
        }
    }
    assert!(bad.is_empty(), "twisted or failed pairings:\n{}", bad.join("\n"));
}

/// A connector set by the user still wins over the automatic start.
#[test]
fn connectors_override_the_automatic_start() {
    let z = (0.0, 0.0);
    let auto = loft2(&Sec::Circle(25.0), z, &rect(25.0, 15.0, 0), z, "");
    let fixed = loft2(&Sec::Circle(25.0), z, &rect(25.0, 15.0, 0), z, r#","connectors":[0.25,null]"#);
    let (ta, _) = twist(&auto, z, z);
    assert!(ta < 20.0, "{ta}");
    if fixed.errors.is_empty() {
        let (tf, _) = twist(&fixed, z, z);
        assert!(tf > 20.0, "a circle start fixed a quarter of the way round must not be undone: {tf}");
    }
}

/// The circle is cut at the corner directions of the polygon: the four cut angles of the circle equal the four corner angles
/// (the square's 45/135/225/315, a 50x30 rectangle's unequal ones).
#[test]
fn circle_is_cut_at_the_corner_directions() {
    let z = (0.0, 0.0);
    for (name, hx, hy) in [("50x50", 25.0, 25.0), ("49.98x50", 24.99, 25.0), ("50x30", 25.0, 15.0)] {
        // the app's rectangle: corner (-w/2,-h/2) first, counter clockwise
        let sq = Sec::Poly(vec![(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]);
        let o = loft2(&Sec::Circle(25.0), z, &sq, z, "");
        assert!(o.errors.is_empty(), "{name}: {:?}", o.errors);
        let b = o.brep.body.as_ref().unwrap();
        let mut got = vec![];
        for e in b.edge_ids() {
            let ed = b.edge(e);
            let (p0, p1) = (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1));
            if (p0.z - p1.z).abs() < 1.0 {
                continue;
            }
            let p = if p0.z < p1.z { p0 } else { p1 };
            got.push(p.y.atan2(p.x).rem_euclid(std::f64::consts::TAU));
        }
        let mut want: Vec<f64> = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)].iter().map(|c: &(f64, f64)| c.1.atan2(c.0).rem_euclid(std::f64::consts::TAU)).collect();
        got.sort_by(|a, b| a.partial_cmp(b).unwrap());
        want.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(got.len(), 4, "{name}: {got:?}");
        for (g, w) in got.iter().zip(&want) {
            assert!((g - w).abs() < 1e-6, "{name}: circle cut at {g} rad, corner at {w}");
        }
    }
}
