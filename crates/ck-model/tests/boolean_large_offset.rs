//! Booleans on a part far from the origin (SolidCraft #1706): a block with a hole and a
//! boss at (o, o, o). The welding/intersection tolerances are absolute, so at 1e6 mm the
//! f64 resolution of squared distances (~1e-4 at 1e12) breaks them; the boolean moves such
//! operands to a local origin and moves the result back.
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

/// 30 x 20 x 10 block at (o, o, o), r4 through hole, r3 boss on top: volume
/// 6000 - pi*16*10 + pi*9*6 = 5666.9911787...
fn part(o: f64) -> String {
    let (x0, y0, x1, y1) = (o, o, o + 30.0, o + 20.0);
    let (hx, hy, bx, by) = (o + 15.0, o + 10.0, o + 5.0, o + 5.0);
    let (zt, zb) = (o + 10.0, o + 16.0);
    format!(
        r#"{{"format":"ck-part","version":1,"name":"offset",
 "features":[
  {{"type":"plane","id":"F1","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{o},"name":"base"}},
  {{"type":"sketch","id":"S1","plane":{{"kind":"feature","id":"F1"}},"points":[
     {{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},
     {{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],
   "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
     {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
  {{"type":"extrude","id":"E3","op":"add","sketch":"S1","end":"blind","depth":10}},
  {{"type":"plane","id":"F4","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{zt},"name":"top"}},
  {{"type":"sketch","id":"S5","plane":{{"kind":"feature","id":"F4"}},"points":[{{"id":"c","x":{hx},"y":{hy}}}],
   "entities":[{{"type":"circle","id":"e1","c":"c","r":4}}],"constraints":[]}},
  {{"type":"extrude","id":"E6","op":"cut","sketch":"S5","end":"blind","depth":10}},
  {{"type":"plane","id":"F7","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{zt},"name":"top2"}},
  {{"type":"sketch","id":"S8","plane":{{"kind":"feature","id":"F7"}},"points":[{{"id":"c","x":{bx},"y":{by}}}],
   "entities":[{{"type":"circle","id":"e1","c":"c","r":3}}],"constraints":[]}},
  {{"type":"extrude","id":"E9","op":"add","sketch":"S8","end":"blind","depth":6}}
 ]}}"#
    )
}

#[test]
fn booleans_at_large_offsets() {
    const V: f64 = 5666.9911787194815;
    for o in [1e3, 1e4, 1e5, 1e6, 1e7, 1e8] {
        let p = Part::parse(&part(o)).unwrap_or_else(|e| panic!("offset {o}: {e}"));
        let mut ex = Exec(Brep::new());
        let res = regen::regen(&p, &RegenOpts::default(), &mut ex);
        assert!(res.errors.is_empty(), "offset {o}: {:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
        let b = ex.0.body.as_ref().unwrap_or_else(|| panic!("offset {o}: no body"));
        let mp = ck_topo::mass::mass_props(b);
        assert!((mp.volume - V).abs() < 1e-3, "offset {o}: volume {} vs {V}", mp.volume);
        assert!(ck_check::validate(b, 1e-6).is_empty(), "offset {o}: {:?}", ck_check::validate(b, 1e-6));
        // the result stays at the offset, not at the local origin
        let (lo, _) = b.bbox();
        assert!((lo.x - o).abs() < 0.05, "offset {o}: bbox min x {}", lo.x);
    }
}
