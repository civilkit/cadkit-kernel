//! A cut whose tool side face meets a wall left by an earlier cut along a rim must close (it failed with
//! `booleanFailed;issue=openEdge`).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

const DOC: &str = include_str!("fixtures/boolean_rim_graze.json");

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

fn regen_part(src: &str) -> Result<ck_topo::body::Body, Vec<regen::RegenError>> {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    if res.errors.is_empty() {
        Ok(ex.0.body.unwrap())
    } else {
        Err(res.errors)
    }
}

/// The part with every point of sketch F13 (the F14 tool) moved by `dx` along the sketch x axis.
fn shifted(dx: f64) -> String {
    let mut j: ck_json::Json = ck_json::parse(DOC).unwrap();
    let ck_json::Json::Obj(kvs) = &mut j else { panic!() };
    let ck_json::Json::Arr(feats) = &mut kvs.iter_mut().find(|(k, _)| k == "features").unwrap().1 else { panic!() };
    for f in feats.iter_mut() {
        let ck_json::Json::Obj(fk) = f else { continue };
        if !fk.iter().any(|(k, v)| k == "id" && matches!(v, ck_json::Json::Str(s) if s == "F13")) {
            continue;
        }
        let ck_json::Json::Arr(pts) = &mut fk.iter_mut().find(|(k, _)| k == "points").unwrap().1 else { panic!() };
        for p in pts.iter_mut() {
            let ck_json::Json::Obj(pk) = p else { continue };
            for (k, v) in pk.iter_mut() {
                if let (true, ck_json::Json::Num(x)) = (k == "x", &mut *v) {
                    *x += dx;
                }
            }
        }
    }
    ck_json::to_string(&j)
}

fn volume(src: &str) -> f64 {
    let body = regen_part(src).expect("regen errors");
    let issues: Vec<String> = ck_check::validate(&body, 1e-3).iter().map(|i| format!("{} {}", i.code, i.detail)).collect();
    assert!(issues.is_empty(), "validate issues: {issues:?}");
    ck_topo::mass::mass_props(&body).volume
}

/// F14 cuts with a two-region tool whose shared wall lies in the plane of the F10 wall (x = 0): the wall piece
/// inside the tool must go. The volume must equal, to the size of the graze, that of the part with the tool
/// moved by a small step either way.
#[test]
fn rim_graze_cut_closes() {
    let v0 = volume(DOC);
    assert!(v0 > 0.0);
    let d = 1e-4;
    for dx in [d, -d] {
        let v = volume(&shifted(dx));
        assert!((v0 - v).abs() <= d * 60.0, "volume {v0} vs {v} for a tool moved by {dx}");
    }
}
