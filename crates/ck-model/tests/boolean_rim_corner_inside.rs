//! A plate whose outline is a polygon inscribed in the rim of a disc (corners on the rim) and ends in a tab that
//! crosses the rim, with one corner 1.2e-4 inside the rim, stands on that disc. The stretch of the tab edge between
//! that corner and the rim is shorter than the chord sag of the rim's sampled polygon (1.1e-4 at 720 chords), so
//! the face test put it outside the disc and the sliver of disc top left between the plate's chord and the rim had no
//! face: `booleanFailed;issue=openEdge` at the rim (civilkit/cadkit-kernel #57).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

const DOC: &str = include_str!("fixtures/boolean_rim_corner_inside.json");

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

fn regen_part(src: &str) -> Result<ck_topo::body::Body, Vec<String>> {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    if res.errors.is_empty() {
        Ok(ex.0.body.unwrap())
    } else {
        Err(res.errors.iter().map(|e| format!("{e:?}")).collect())
    }
}

/// The part with the plate outline turned half a turn about the disc axis (the tab on the other side) and/or
/// without the final union.
fn variant(turn: bool, union: bool) -> String {
    let mut j = ck_json::parse(DOC).unwrap();
    let ck_json::Json::Obj(kvs) = &mut j else { panic!() };
    let ck_json::Json::Arr(feats) = &mut kvs.iter_mut().find(|(k, _)| k == "features").unwrap().1 else { panic!() };
    if !union {
        feats.pop();
    }
    for f in feats.iter_mut() {
        let ck_json::Json::Obj(fk) = f else { continue };
        if !turn || !fk.iter().any(|(k, v)| k == "id" && matches!(v, ck_json::Json::Str(s) if s == "F17")) {
            continue;
        }
        let ck_json::Json::Arr(pts) = &mut fk.iter_mut().find(|(k, _)| k == "points").unwrap().1 else { panic!() };
        for p in pts.iter_mut() {
            let ck_json::Json::Obj(pk) = p else { continue };
            for (k, v) in pk.iter_mut() {
                if let (true, ck_json::Json::Num(x)) = (k == "x" || k == "y", &mut *v) {
                    *x = -*x;
                }
            }
        }
    }
    ck_json::to_string(&j)
}

/// (checked at 1e-5: the tab edge left inside the rim is 1.8e-4 long, which a 1e-3 check would call degenerate)
fn volume_valid(src: &str, what: &str) -> f64 {
    let body = regen_part(src).unwrap_or_else(|e| panic!("{what}: regen errors: {e:?}"));
    let issues: Vec<String> = ck_check::validate(&body, 1e-5).iter().map(|i| format!("{} {}", i.code, i.detail)).collect();
    assert!(issues.is_empty(), "{what}: validate issues: {issues:?}");
    ck_topo::mass::mass_props(&body).volume
}

/// The union closes, on both sides of the part, and the two solids that only touch along a face keep all their
/// volume.
#[test]
fn plate_with_tab_over_disc_rim_unites() {
    for turn in [false, true] {
        let before = volume_valid(&variant(turn, false), &format!("operands, turn={turn}"));
        let after = volume_valid(&variant(turn, true), &format!("union, turn={turn}"));
        assert!(before > 0.0);
        assert!((after - before).abs() <= 1e-6 * before, "turn={turn}: volume {after} vs {before} of the operands");
    }
}
