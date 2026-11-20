//! The fan shroud (tools/cases/perf/fan_shroud.ckpart, also the app sample `fan-shroud-filleted`): fillets F48 (r 2) and F49
//! (r 0.5) round edges where a blade slot meets the hub and shroud cones. The free-form blend cleaned up the welding slivers
//! of its boolean by collapsing every edge of the whole result shorter than 6e-3, which also took a 7.5e-4 mm edge of the
//! grille ring, 20 mm away from the blend: the neighbours' ends moved by its length, their edges were re-fitted without
//! pcurves and lay 7.5e-4 off their faces (8 `EDGE_OFF_SURFACE`, no feature error), and F49 then failed (`blendFailed`, an
//! edge used by four faces). Only slivers between new vertices are collapsed now.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

struct Exec(Brep, Vec<(String, f64)>);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let r = self.0.exec(s, f);
        if let Some(b) = &self.0.body {
            self.1.push((f.id.clone(), ck_topo::mass::mass_props(b).volume));
        }
        r
    }
}

const DOC: &str = include_str!("../../../tools/cases/perf/fan_shroud.ckpart");

#[test]
fn slot_fillets_leave_the_grille_alone() {
    let part = Part::parse(DOC).unwrap();
    let mut ex = Exec(Brep::new(), vec![]);
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    assert!(errs.is_empty(), "{errs:?}");
    let b = ex.0.body.as_ref().expect("body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{:?}", iss.iter().map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect::<Vec<_>>());
    let vol = |id: &str| ex.1.iter().rev().find(|x| x.0 == id).map(|x| x.1).unwrap();
    // both are convex rounds of short edges: each removes a little material
    for (f, prev) in [("F48", "F47"), ("F49", "F48")] {
        let dv = vol(prev) - vol(f);
        assert!(dv > 1e-3 && dv < 20.0, "{f} removed {dv}");
    }
}
