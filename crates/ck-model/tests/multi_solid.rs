//! Multi-solid results: a body holds one lump per solid. Disjoint and
//! point-contact unions keep every solid (torus_link used to lose the torus
//! to a broken seam pcurve of the untouched torus fragment).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

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

fn regen_text(text: &str) -> (ck_topo::body::Body, String) {
    let part = Part::parse(text).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r.to_json().to_string();
    (ex.0.body.clone().expect("a body"), errs)
}

fn rect(id: &str, x0: f64, x1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":0}},{{"id":"p2","x":{x1},"y":0}},{{"id":"p3","x":{x1},"y":10}},{{"id":"p4","x":{x0},"y":10}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},{{"type":"extrude","id":"E{id}","op":"add","sketch":"S{id}","end":"blind","depth":5}}"#
    )
}

#[test]
fn disjoint_extrudes_are_two_solids() {
    let text = format!(r#"{{"format":"ck-part","version":1,"name":"two","features":[{},{}]}}"#, rect("1", 0.0, 10.0), rect("2", 30.0, 40.0));
    let (b, errs) = regen_text(&text);
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    assert_eq!(b.num_lumps(), 2);
    let mp = ck_topo::mass::mass_props(&b);
    assert!((mp.volume - 1000.0).abs() < 1e-6, "{}", mp.volume);
    assert!(ck_check::validate(&b, 1e-6).is_empty());
    // per-solid moments add up
    let ms = ck_topo::mass::lump_moments(&b);
    assert_eq!(ms.len(), 2);
    assert!((ms[0].volume - 500.0).abs() < 1e-6 && (ms[1].volume - 500.0).abs() < 1e-6);
}

#[test]
fn torus_with_point_contact_lugs_keeps_the_torus() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/bench/benchcad/torus_link_000095.ckpart.json");
    let (b, errs) = regen_text(&std::fs::read_to_string(path).unwrap());
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    // torus (2 pi^2 R r^2 = 100942) + 3 lugs (849 each) - 3 holes; the reference gives 102954
    let mp = ck_topo::mass::mass_props(&b);
    assert!((mp.volume - 102954.0).abs() / 102954.0 < 1e-3, "volume {}", mp.volume);
    assert!(b.num_lumps() >= 2, "lumps {}", b.num_lumps());
    // the untouched torus fragment keeps its full area
    let torus = b.face_ids().find(|&f| b.face(f).surface.kind() == "torus").expect("torus face");
    let a = ck_topo::mass::face_integral::<1>(&b, torus, 1, &|_, n| [n.len()])[0];
    assert!((a - 20558.57).abs() < 1.0, "torus area {a}");
    assert!(ck_check::validate(&b, 1e-6).is_empty());
}
