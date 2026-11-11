//! A rod meeting a ball: the circle edge between the cylinder and the sphere takes a fillet (the rolling ball's section is
//! the meridian corner of a line and a circle, solved in 2D).
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

struct Res {
    errors: Vec<String>,
    vol: f64,
    faces: Vec<(String, &'static str)>,
    valid: Vec<String>,
}

fn run(feats: &str) -> Res {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{feats}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let Some(b) = ex.brep.body.as_ref() else { return Res { errors, vol: 0.0, faces: vec![], valid: vec![] } };
    let faces = b.face_ids().map(|f| (b.face_id_str(f), b.face(f).surface.kind())).collect();
    let valid = ck_check::validate(b, 1e-3).iter().map(|i| format!("{} {}", i.code, i.detail)).collect();
    Res { errors, vol: ck_topo::mass::mass_props(b).volume, faces, valid }
}

/// ball of radius `big` at the origin, a rod of radius `rod` from its centre out along +X (length 30)
fn base(big: f64, rod: f64) -> String {
    format!(
        r#"{{"type":"sphere","id":"F1","center":[0,0,0],"radius":{big}}},
        {{"type":"sketch","id":"F2","plane":{{"kind":"std","id":"right"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{rod}}}],"constraints":[]}},
        {{"type":"extrude","id":"F3","op":"add","sketch":"F2","end":"blind","depth":30}}"#
    )
}

fn with_fillet(big: f64, rod: f64, r: f64) -> Res {
    let x = (big * big - rod * rod).sqrt();
    run(&format!(
        r#"{},{{"type":"fillet","id":"F4","radius":{r},"edges":[{{"faces":["F1.S.e1","F3.S.e1"],"at":[{x},{rod},0]}}]}}"#,
        base(big, rod)
    ))
}

#[test]
fn faces_of_a_rod_in_a_ball() {
    let r = run(&base(7.5, 2.0));
    eprintln!("FACES {:?} errors {:?} vol {}", r.faces, r.errors, r.vol);
    assert!(r.errors.is_empty());
}

#[test]
fn a_rod_meeting_a_ball_takes_a_fillet() {
    for (big, rod, r) in [(7.5, 2.0, 1.5), (7.5, 2.0, 2.0), (10.0, 2.0, 1.0), (15.0, 4.0, 1.5)] {
        let plain = run(&base(big, rod));
        let f = with_fillet(big, rod, r);
        eprintln!("FIL {big} {rod} {r}: errors {:?} valid {:?} vol {} plain {}", f.errors, f.valid, f.vol, plain.vol);
        assert!(f.errors.is_empty(), "{big} {rod} {r}: {:?}", f.errors);
        assert!(f.valid.is_empty(), "{:?}", f.valid);
        // a concave fillet adds material: a ring of the corner between the rod, the ball and the rolling ball
        assert!(f.vol > plain.vol, "{} vs {}", f.vol, plain.vol);
        assert!(f.vol < plain.vol * 1.1);
        // the added ring is the solid of revolution of the corner between the rod wall, the ball and the rolling ball
        let (zq, rq) = (((big + r) * (big + r) - (rod + r) * (rod + r)).sqrt(), rod + r);
        let z_tb = zq * big / (big + r);
        let n = 200_000;
        let mut exact = 0.0;
        for i in 0..n {
            let z = z_tb + (zq - z_tb) * (i as f64 + 0.5) / n as f64;
            let top = rq - (r * r - (z - zq) * (z - zq)).max(0.0).sqrt();
            let low = (big * big - z * z).max(0.0).sqrt().max(rod);
            exact += std::f64::consts::PI * (top * top - low * low) * (zq - z_tb) / n as f64;
        }
        let got = f.vol - plain.vol;
        eprintln!("RING {big} {rod} {r}: exact {exact:.4} got {got:.4}");
        assert!((got - exact).abs() < 0.01 * exact, "{got} vs {exact}");
    }
}

#[test]
fn the_rim_of_a_hole_in_a_ball_is_not_the_analytic_builders() {
    // a convex circle against a ball still goes to the free-form strips as before (the analytic section handles a concave edge);
    // whatever it gives, it must not hand back a model the sanity check refuses silently: errors name a blend code
    let (big, rod) = (10.0f64, 2.0f64);
    let ball_hole = format!(
        r#"{{"type":"sphere","id":"F1","center":[0,0,0],"radius":{big}}},
        {{"type":"sketch","id":"F2","plane":{{"kind":"std","id":"right"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{rod}}}],"constraints":[]}},
        {{"type":"extrude","id":"F3","op":"cut","sketch":"F2","end":"blind","depth":30}}"#
    );
    let x = (big * big - rod * rod).sqrt();
    let f = run(&format!(r#"{ball_hole},{{"type":"fillet","id":"F4","radius":1,"edges":[{{"faces":["F1.S.e1","F3.S.e1"],"at":[{x},{rod},0]}}]}}"#));
    eprintln!("HOLE errors {:?} valid {:?}", f.errors, f.valid);
    assert!(f.errors.is_empty() || f.errors.iter().all(|e| e.contains("blend")), "{:?}", f.errors);
}

#[test]
fn an_oblique_rod_in_a_ball_takes_a_fillet() {
    // the rod's axis is not along a world axis (the sphere's pole axis), so the circle edge is a small loop off the seam and the
    // poles: the sphere's seam loop is dropped as bounding nothing and the containment checks must read the loop as a hole
    let feats = r#"{"type": "sphere", "center": [0, 0, 0], "radius": 10, "id": "F1"}, {"type": "sketch", "id": "F2", "plane": {"kind": "std", "id": "front"}, "points": [{"id": "O", "x": 0, "y": 0, "fixed": true}, {"id": "q0", "x": 0, "y": 0}], "entities": [{"id": "e0", "type": "point", "p": "q0"}], "constraints": []}, {"type": "plane", "id": "F4", "mode": "offset", "base": {"kind": "std", "id": "front"}, "offset": -7.071067812, "visible": false}, {"type": "sketch", "id": "F3", "plane": {"kind": "feature", "id": "F4"}, "points": [{"id": "O", "x": 0, "y": 0, "fixed": true}, {"id": "q1", "x": 0, "y": 7.071067812}], "entities": [{"id": "e1", "type": "point", "p": "q1"}], "constraints": []}, {"type": "plane", "id": "F6", "mode": "offset", "base": {"kind": "std", "id": "front"}, "offset": 4.082482905, "visible": false}, {"type": "sketch", "id": "F5", "plane": {"kind": "feature", "id": "F6"}, "points": [{"id": "O", "x": 0, "y": 0, "fixed": true}, {"id": "q2", "x": -8.164965809, "y": 4.082482905}], "entities": [{"id": "e2", "type": "point", "p": "q2"}], "constraints": []}, {"type": "plane", "id": "F7", "mode": "threePoints", "points": [{"kind": "sketchPoint", "sketch": "F2", "point": "q0"}, {"kind": "sketchPoint", "sketch": "F3", "point": "q1"}, {"kind": "sketchPoint", "sketch": "F5", "point": "q2"}]}, {"type": "sketch", "id": "F8", "plane": {"kind": "feature", "id": "F7"}, "points": [{"id": "O", "x": 0, "y": 0, "fixed": true}, {"id": "p1", "x": 0, "y": 0}], "entities": [{"id": "e1", "type": "circle", "c": "p1", "r": 2}], "constraints": []}, {"type": "extrude", "id": "F9", "op": "add", "sketch": "F8", "end": "blind", "depth": 30}"#;
    let (big, rod, r) = (10.0f64, 2.0f64, 1.0f64);
    let plain = run(feats);
    assert!(plain.errors.is_empty(), "{:?}", plain.errors);
    let f = run(&format!(r#"{},{{"type":"fillet","id":"F10","radius":{r},"edges":[{{"faces":["F1.S.e1","F9.S.e1"],"at":[7.071,4.243,5.657]}}]}}"#, feats));
    assert!(f.errors.is_empty(), "{:?}", f.errors);
    assert!(f.valid.is_empty(), "{:?}", f.valid);
    // the added ring is the same solid of revolution as for a rod along an axis
    let (zq, rq) = (((big + r) * (big + r) - (rod + r) * (rod + r)).sqrt(), rod + r);
    let z_tb = zq * big / (big + r);
    let n = 200_000;
    let mut exact = 0.0;
    for i in 0..n {
        let z = z_tb + (zq - z_tb) * (i as f64 + 0.5) / n as f64;
        let top = rq - (r * r - (z - zq) * (z - zq)).max(0.0).sqrt();
        let low = (big * big - z * z).max(0.0).sqrt().max(rod);
        exact += std::f64::consts::PI * (top * top - low * low) * (zq - z_tb) / n as f64;
    }
    let got = f.vol - plain.vol;
    eprintln!("OBL ring: exact {exact:.4} got {got:.4}");
    assert!((got - exact).abs() < 0.01 * exact, "{got} vs {exact}");
}
