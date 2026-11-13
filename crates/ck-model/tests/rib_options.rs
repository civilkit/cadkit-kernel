//! The rib's options (`side`, `draft`, `fillet`, `target`) against hand-computed volumes, and the extension of the
//! rib line never filling a bore it runs through (the bore cut before or after the rib gives one part).
//!
//! Parts are 40 mm deep in z (extrude mid). The bracket is an L (plate 100 x 10, wall 10 x 60) and the rib a line from the
//! wall face to the plate top: a right triangle of legs 50 and 30 (area 750), thickness 6.

use ck_core::linalg::Vec3;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
    volume: f64,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn clean(&self) {
        assert!(self.errors().is_empty(), "{:?}", self.errors());
        let b = self.body.as_ref().unwrap();
        let bad = ck_check::validate(b, 1e-6);
        assert!(bad.is_empty(), "invalid body: {bad:?}");
    }
    /// The z of the plane face `name` (a plane parallel to the sketch plane).
    fn plane_z(&self, name: &str) -> f64 {
        let b = self.body.as_ref().unwrap();
        let f = b.find_face(name).unwrap_or_else(|| panic!("no face {name}"));
        match &b.face(f).surface {
            ck_geom::surface::Surface::Plane { frame } => {
                assert!(frame.z.dot(Vec3::Z).abs() > 0.999999, "{name} is not parallel to the sketch plane");
                frame.o.z
            }
            s => panic!("{name} is not a plane: {s:?}"),
        }
    }
    fn has_face(&self, name: &str) -> bool {
        self.body.as_ref().unwrap().find_face(name).is_some()
    }
    fn error_code(&self, feature: &str) -> String {
        self.result.errors.iter().find(|e| e.feature == feature).unwrap_or_else(|| panic!("no error on {feature}: {:?}", self.errors())).error.code.clone()
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let volume = ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    Run { result, body: ex.brep.body.clone(), volume }
}

const FRONT: &str = r#"{"kind":"std","id":"front"}"#;

fn poly(id: &str, pts: &[(f64, f64)]) -> String {
    let points: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{i}","x":{},"y":{}}}"#, p.0, p.1)).collect();
    let n = pts.len();
    let ents: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{i}","p2":"p{}"}}"#, i + 1, (i + 1) % n)).collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{FRONT},"points":[{}],"entities":[{}],"constraints":[]}}"#, points.join(","), ents.join(","))
}
fn chain(id: &str, pts: &[(f64, f64)]) -> String {
    let points: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{i}","x":{},"y":{}}}"#, p.0, p.1)).collect();
    let ents: Vec<String> = (0..pts.len() - 1).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{i}","p2":"p{}"}}"#, i + 1, i + 1)).collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{FRONT},"points":[{}],"entities":[{}],"constraints":[]}}"#, points.join(","), ents.join(","))
}
fn circle(id: &str, cx: f64, cy: f64, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{FRONT},"points":[{{"id":"c","x":{cx},"y":{cy}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#)
}
fn extrude(id: &str, sk: &str, op: &str, end: &str, extra: &str) -> String {
    format!(r#"{{"type":"extrude","id":"{id}","op":"{op}","sketch":"{sk}","end":"{end}","depth":40{extra}}}"#)
}

/// The L bracket (volume 60000) and the line of the rib (wall face to plate top).
fn bracket() -> String {
    format!(
        "{},{},{}",
        poly("F1", &[(0.0, 0.0), (100.0, 0.0), (100.0, 10.0), (10.0, 10.0), (10.0, 60.0), (0.0, 60.0)]),
        extrude("F2", "F1", "add", "mid", ""),
        chain("F3", &[(10.0, 40.0), (60.0, 10.0)])
    )
}
fn rib(extra: &str) -> String {
    format!(r#"{{"type":"rib","id":"F4","sketch":"F3","thickness":6{extra}}}"#)
}
fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

const BASE: f64 = 60000.0;
const AREA: f64 = 750.0;
const LEN: f64 = 58.309518948453004; // the free edge: sqrt(50^2 + 30^2)

#[test]
fn defaults_are_unchanged() {
    let r = run(&format!("{},{}", bracket(), rib("")));
    r.clean();
    assert!(near(r.volume, BASE + 6.0 * AREA, 1e-3), "{}", r.volume);
    for n in ["F4.C0", "F4.C1", "F4.S.e1r"] {
        assert!(r.has_face(n), "{n}");
    }
    assert!(near(r.plane_z("F4.C0"), -3.0, 1e-9) && near(r.plane_z("F4.C1"), 3.0, 1e-9));
    // `both` is the default spelled out
    let r2 = run(&format!("{},{}", bracket(), rib(r#","side":"both""#)));
    r2.clean();
    assert!(near(r2.volume, r.volume, 1e-6));
}

#[test]
fn one_sided_thickness_stands_on_one_side_of_the_sketch_plane() {
    let right = run(&format!("{},{}", bracket(), rib(r#","side":"right""#)));
    right.clean();
    assert!(near(right.volume, BASE + 6.0 * AREA, 1e-3), "{}", right.volume);
    assert!(near(right.plane_z("F4.C0"), 0.0, 1e-9) && near(right.plane_z("F4.C1"), 6.0, 1e-9));
    let left = run(&format!("{},{}", bracket(), rib(r#","side":"left""#)));
    left.clean();
    assert!(near(left.volume, BASE + 6.0 * AREA, 1e-3), "{}", left.volume);
    assert!(near(left.plane_z("F4.C0"), -6.0, 1e-9) && near(left.plane_z("F4.C1"), 0.0, 1e-9));
}

#[test]
fn draft_widens_the_rib_toward_the_body() {
    // the wall is `thickness` thick at the free edge and 2 tan(draft) wider per mm of distance from it
    let h = 2.0 * AREA / LEN; // the triangle's height over the free edge
    let moment = AREA * h / 3.0; // integral of the distance from the free edge over the triangle
    for deg in [3.0f64, 8.0] {
        let r = run(&format!("{},{}", bracket(), rib(&format!(r#","draft":{deg}"#))));
        r.clean();
        let want = BASE + 6.0 * AREA + 2.0 * deg.to_radians().tan() * moment;
        assert!(near(r.volume, want, 0.5), "draft {deg}: {} want {want}", r.volume);
        for n in ["F4.C0", "F4.C1", "F4.S.e1r"] {
            assert!(r.has_face(n), "{n}");
        }
    }
    // one-sided and drafted: still one rib, the plane side tilts too
    let r = run(&format!("{},{}", bracket(), rib(r#","draft":5,"side":"right""#)));
    r.clean();
    let want = BASE + 6.0 * AREA + 2.0 * 5f64.to_radians().tan() * moment;
    assert!(near(r.volume, want, 0.5), "{} want {want}", r.volume);
}

#[test]
fn fillet_rounds_the_free_edge() {
    // two rounds of the free edge: each takes (1 - pi/4) r^2 per mm of edge, less the end clipping by the two faces the
    // rib meets (the strip is shorter by (cot A + cot B) per mm of depth into the rib)
    let r0 = 1.5f64;
    let (ca, cb) = (50.0 / 30.0, 30.0 / 50.0);
    let a_c = (1.0 - std::f64::consts::FRAC_PI_4) * r0 * r0;
    let m = r0.powi(3) * (5.0 / 6.0 - std::f64::consts::FRAC_PI_4);
    let want = BASE + 6.0 * AREA - 2.0 * (LEN * a_c - (ca + cb) * m);
    let r = run(&format!("{},{}", bracket(), rib(&format!(r#","fillet":{r0}"#))));
    r.clean();
    assert!(near(r.volume, want, 0.5), "{} want {want}", r.volume);
    // a bigger round takes more
    let big = run(&format!("{},{}", bracket(), rib(r#","fillet":2.5"#)));
    big.clean();
    assert!(big.volume < r.volume - 10.0, "{} vs {}", big.volume, r.volume);
    // with a draft too
    let d = run(&format!("{},{}", bracket(), rib(r#","fillet":1,"draft":4"#)));
    d.clean();
    let plain = run(&format!("{},{}", bracket(), rib(r#","draft":4"#)));
    assert!(d.volume < plain.volume && plain.volume - d.volume < 40.0, "{} vs {}", d.volume, plain.volume);
}

#[test]
fn bad_values_are_refused() {
    for (extra, code) in [
        (r#","side":"up""#, "invalidValue"),
        (r#","draft":-2"#, "invalidValue"),
        (r#","draft":60"#, "invalidValue"),
        (r#","fillet":3"#, "invalidValue"),
        (r#","fillet":-1"#, "invalidValue"),
    ] {
        let r = run(&format!("{},{}", bracket(), rib(extra)));
        assert_eq!(r.error_code("F4"), code, "{extra}");
        assert!(near(r.volume, BASE, 1e-6), "{extra}: the body is untouched");
    }
    // a drafted rib is one straight line
    let two = format!(
        "{},{},{},{}",
        poly("F1", &[(0.0, 0.0), (100.0, 0.0), (100.0, 10.0), (10.0, 10.0), (10.0, 60.0), (0.0, 60.0)]),
        extrude("F2", "F1", "add", "mid", ""),
        chain("F3", &[(10.0, 40.0), (35.0, 30.0), (60.0, 10.0)]),
        rib(r#","draft":5"#)
    );
    assert_eq!(run(&two).error_code("F4"), "draftUnsupported");
}

/// The bracket as body "A" and a block "B" far to the side, both across the sketch plane.
fn two_bodies() -> String {
    format!(
        "{},{},{},{}",
        poly("F1", &[(0.0, 0.0), (100.0, 0.0), (100.0, 10.0), (10.0, 10.0), (10.0, 60.0), (0.0, 60.0)]),
        extrude("F2", "F1", "newBody", "mid", r#","body":"A""#),
        poly("F5", &[(200.0, 0.0), (300.0, 0.0), (300.0, 10.0), (200.0, 10.0)]),
        extrude("F6", "F5", "newBody", "mid", r#","body":"B""#),
    ) + &format!(",{}", chain("F3", &[(10.0, 40.0), (60.0, 10.0)]))
}

#[test]
fn target_limits_the_rib_to_the_named_bodies() {
    let b_vol = 100.0 * 10.0 * 40.0;
    let one = run(&format!("{},{}", two_bodies(), rib("")));
    one.clean();
    assert!(near(one.volume, BASE + b_vol + 6.0 * AREA, 1e-3), "{}", one.volume);
    let a = run(&format!("{},{}", two_bodies(), rib(r#","target":{"body":"A"}"#)));
    a.clean();
    assert!(near(a.volume, one.volume, 1e-6), "{}", a.volume);
    let both = run(&format!("{},{}", two_bodies(), rib(r#","target":[{"body":"A"},{"body":"B"}]"#)));
    assert!(both.errors().is_empty() || both.errors().iter().all(|e| !e.contains("F4")), "{:?}", both.errors());
    assert!(near(both.volume, one.volume, 1e-6), "{}", both.volume);
    // the rib line does not touch B: restricted to B there is nothing to extend to
    let b = run(&format!("{},{}", two_bodies(), rib(r#","target":{"body":"B"}"#)));
    assert_eq!(b.error_code("F4"), "ribNoContact");
    assert!(near(b.volume, BASE + b_vol, 1e-6), "the bodies are untouched: {}", b.volume);
    // an unknown name lists the bodies there are
    let z = run(&format!("{},{}", two_bodies(), rib(r#","target":{"body":"Z"}"#)));
    assert_eq!(z.error_code("F4"), "missingReference");
}

/// Plate, block and a boss with a bore through it; the rib line runs from the plate over the block and on into the boss, so its
/// extension crosses the bore. The bore is cut `before` or after the rib.
fn clevis(bore_first: bool, with_rib: bool) -> Run {
    let body = format!(
        "{},{},{},{},{},{}",
        poly("S1", &[(0.0, 0.0), (100.0, 0.0), (100.0, 10.0), (0.0, 10.0)]),
        extrude("F1", "S1", "add", "mid", ""),
        poly("S2", &[(60.0, 10.0), (100.0, 10.0), (100.0, 40.0), (60.0, 40.0)]),
        extrude("F2", "S2", "add", "mid", ""),
        circle("S3", 80.0, 40.0, 20.0),
        extrude("F3", "S3", "add", "mid", ""),
    );
    let bore = format!("{},{}", circle("S4", 80.0, 40.0, 10.0), extrude("F4", "S4", "cut", "throughBoth", ""));
    let line = chain("S5", &[(20.0, 10.0), (68.0, 44.0)]);
    let rib = r#"{"type":"rib","id":"F5","sketch":"S5","thickness":6}"#;
    let tail = match (bore_first, with_rib) {
        (_, false) => bore,
        (true, true) => format!("{bore},{line},{rib}"),
        (false, true) => format!("{line},{rib},{bore}"),
    };
    run(&format!("{body},{tail}"))
}

#[test]
fn the_rib_never_fills_a_bore_it_runs_through() {
    let none = clevis(true, false);
    none.clean();
    // plate 100 x 10, block 40 x 30, the upper half of the boss disc (r 20), less the bore (r 10), 40 deep
    let pi = std::f64::consts::PI;
    let want_none = (1000.0 + 1200.0 + 0.5 * pi * 400.0 - pi * 100.0) * 40.0;
    assert!(near(none.volume, want_none, 0.5), "{} want {want_none}", none.volume);
    // the rib: the triangle between the line, the plate top and the block face (legs 40 and 28.33), 6 thick
    let rib_vol = 0.5 * 40.0 * (40.0 * 34.0 / 48.0) * 6.0;
    let before = clevis(false, true);
    before.clean();
    assert!(near(before.volume, want_none + rib_vol, 0.5), "rib before the bore: {} want {}", before.volume, want_none + rib_vol);
    let after = clevis(true, true);
    after.clean();
    assert!(near(after.volume, want_none + rib_vol, 0.5), "rib after the bore: {} want {}", after.volume, want_none + rib_vol);
    assert!(near(after.volume, before.volume, 1e-6));
}
