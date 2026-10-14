//! A sphere (revolved semicircle) centred on a block face, added and cut: the boolean trims the sphere with
//! the face plane and must keep the hemisphere face (area 2 pi r^2) in the result.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

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

const R: f64 = 6.0;
const S: f64 = 40.0;

/// 40 mm cube, x and z in [-20, 20], y in [0, 40] (top plane sketch, +Y extrusion).
const BLOCK: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":-20,"y":-20},{"id":"b","x":20,"y":-20},{"id":"c","x":20,"y":20},{"id":"d","x":-20,"y":20}],
  "entities":[{"type":"line","id":"l1","p1":"a","p2":"b"},{"type":"line","id":"l2","p1":"b","p2":"c"},{"type":"line","id":"l3","p1":"c","p2":"d"},{"type":"line","id":"l4","p1":"d","p2":"a"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":40}"#;

/// Sphere of radius R about `(u, v)` of std plane `plane`, axis along the sketch's x or y direction.
fn sphere(plane: &str, u: f64, v: f64, along_x: bool, op: &str) -> String {
    sphere_on(&format!(r#"{{"kind":"std","id":"{plane}"}}"#), u, v, along_x, op)
}

fn sphere_on(plane: &str, u: f64, v: f64, along_x: bool, op: &str) -> String {
    // the arc runs counter-clockwise from p1 to p2
    let (p1, p2) = if along_x { ((u + R, v), (u - R, v)) } else { ((u, v - R), (u, v + R)) };
    format!(
        r#"{{"type":"sketch","id":"F3","plane":{plane},"points":[{{"id":"c","x":{u},"y":{v}}},{{"id":"p1","x":{},"y":{}}},{{"id":"p2","x":{},"y":{}}}],
  "entities":[{{"type":"arc","id":"arc","c":"c","p1":"p1","p2":"p2"}},{{"type":"line","id":"cl","p1":"p2","p2":"p1"}},{{"type":"line","id":"ax","p1":"p1","p2":"p2","construction":true}}],"constraints":[]}},
 {{"type":"revolve","id":"F4","op":"{op}","sketch":"F3","axis":"ax","angle":360}}"#,
        p1.0, p1.1, p2.0, p2.1
    )
}

struct Res {
    errors: Vec<String>,
    vol: f64,
    sphere_area: f64,
    nsph: usize,
    valid: Vec<String>,
}

fn run(feats: &str) -> Res {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{feats}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let Some(b) = ex.brep.body.as_ref() else { return Res { errors, vol: 0.0, sphere_area: 0.0, nsph: 0, valid: vec![] } };
    let mut area = 0.0;
    let mut n = 0;
    for f in b.face_ids().filter(|&f| b.face(f).surface.kind() == "sphere") {
        n += 1;
        let msg = ck_json::parse(&format!(r#"{{"type":"measure","refs":[{{"kind":"face","id":"{}"}}]}}"#, b.face_id_str(f))).unwrap();
        let m = ck_ops::measure::reply(b, &msg);
        area += m.get_arr("items").and_then(|i| i.first()).and_then(|i| i.get_f64("area")).unwrap_or(f64::NAN);
    }
    let valid = ck_check::validate(b, 1e-3).iter().map(|i| format!("{} {}", i.code, i.detail)).collect();
    Res { errors, vol: ck_topo::mass::mass_props(b).volume, sphere_area: area, nsph: n, valid }
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

#[test]
fn hemisphere_face_survives_on_every_block_face() {
    let v0 = S * S * S;
    let hemi = 2.0 * PI * R * R;
    let dv = 2.0 / 3.0 * PI * R * R * R;
    // (sketch plane, u, v, label): centre on a face plane, any axis through it
    let cases: [(&str, f64, f64, &str); 8] = [
        ("front", 0.0, 40.0, "top face, front sketch"),
        ("right", 0.0, 40.0, "top face, right sketch"),
        ("front", 0.0, 0.0, "bottom face, front sketch"),
        ("right", 0.0, 0.0, "bottom face, right sketch"),
        ("front", 20.0, 20.0, "+x face, front sketch"),
        ("front", -20.0, 20.0, "-x face, front sketch"),
        ("right", 20.0, 20.0, "+z face, right sketch"),
        ("right", -20.0, 20.0, "-z face, right sketch"),
    ];
    let mut bad = vec![];
    for (plane, u, v, label) in cases {
        for along_x in [false, true] {
            for (op, expect) in [("add", v0 + dv), ("cut", v0 - dv)] {
                let r = run(&format!("{BLOCK},{}", sphere(plane, u, v, along_x, op)));
                let what = format!("{label}, axis {}, {op}", if along_x { "x" } else { "y" });
                if !r.errors.is_empty() {
                    bad.push(format!("{what}: {:?}", r.errors));
                } else if !r.valid.is_empty() {
                    bad.push(format!("{what}: invalid {:?}", r.valid));
                } else if !near(r.vol, expect, 1e-3) {
                    bad.push(format!("{what}: volume {} vs {expect}", r.vol));
                } else if r.nsph == 0 || !near(r.sphere_area, hemi, 2e-3) {
                    bad.push(format!("{what}: {} sphere faces, area {} vs {hemi}", r.nsph, r.sphere_area));
                }
            }
        }
    }
    assert!(bad.is_empty(), "{} failures:\n{}", bad.len(), bad.join("\n"));
}

/// Manual probe: `FWP=front FWU=0 FWV=40 FWX=0 FWO=add cargo test ... probe -- --ignored --nocapture`.
#[test]
#[ignore]
fn probe() {
    let g = |k: &str, d: &str| std::env::var(k).unwrap_or(d.into());
    let f = format!("{BLOCK},{}", sphere(&g("FWP", "front"), g("FWU", "0").parse().unwrap(), g("FWV", "40").parse().unwrap(), g("FWX", "0") == "1", &g("FWO", "add")));
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{f}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    eprintln!("errors {:?}", r.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    if let Some(b) = ex.brep.body.as_ref() {
        for fc in b.face_ids() {
            eprintln!("face {} {}", b.face_id_str(fc), b.face(fc).surface.kind());
        }
    }
}

/// The sketch on the face itself: the semicircle lies in the face plane, so the sphere's axis does too and
/// both poles sit on the face. Off-centre positions as well.
#[test]
fn sphere_sketched_on_the_face() {
    let dv = 2.0 / 3.0 * PI * R * R * R;
    let hemi = 2.0 * PI * R * R;
    let face = r#"{"kind":"face","id":"F2.C1"}"#;
    for (u, v) in [(0.0, 0.0), (7.0, -3.0), (-9.0, 8.0)] {
        for along_x in [false, true] {
            for op in ["add", "cut"] {
                let r = run(&format!("{BLOCK},{}", sphere_on(face, u, v, along_x, op)));
                let what = format!("({u},{v}) axis {} {op}", if along_x { "x" } else { "y" });
                assert!(r.errors.is_empty(), "{what}: {:?}", r.errors);
                assert!(r.valid.is_empty(), "{what}: {:?}", r.valid);
                let expect = if op == "add" { S * S * S + dv } else { S * S * S - dv };
                assert!(near(r.vol, expect, 1e-3), "{what}: volume {} vs {expect}", r.vol);
                assert!(r.nsph >= 1 && near(r.sphere_area, hemi, 2e-3), "{what}: {} sphere faces, area {} vs {hemi}", r.nsph, r.sphere_area);
            }
        }
    }
}
