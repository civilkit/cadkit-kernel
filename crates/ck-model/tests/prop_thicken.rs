//! Thicken of the propeller probe's open blade sheet: a five-station open-section surface loft
//! whose planar faces meet in creases whose rails are not perpendicular to the sections. Ours
//! refused with `thickenReach` (the rim's corner solved off the crease's miter image); the
//! original builds 3145.863 mm3 (`tools/prop_probe.mjs`, case `cap7_thicken_open_loft`).
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

fn case_json(id: &str) -> ck_json::Json {
    let p = format!("{}/../../tools/cases/prop_probe/{id}.json", env!("CARGO_MANIFEST_DIR"));
    ck_json::parse(&std::fs::read_to_string(&p).unwrap()).unwrap()
}

/// The case's part document, with the thicken feature's thickness set to `t`.
fn case_part_thick(id: &str, t: f64) -> String {
    let mut j = case_json(id);
    set_thickness(&mut j, t);
    format!(
        r#"{{"format":"ck-part","version":1,"features":{}}}"#,
        ck_json::to_string(j.get("features").expect("features"))
    )
}

fn set_thickness(j: &mut ck_json::Json, t: f64) {
    match j {
        ck_json::Json::Arr(a) => a.iter_mut().for_each(|v| set_thickness(v, t)),
        ck_json::Json::Obj(m) => {
            let is = m.iter().any(|(k, v)| k == "type" && v.as_str() == Some("thicken"));
            if is {
                if let Some(v) = m.iter_mut().find(|(k, _)| k == "thickness") {
                    v.1 = ck_json::Json::Num(t);
                }
            }
            m.iter_mut().for_each(|(_, v)| set_thickness(v, t));
        }
        _ => {}
    }
}

fn case_part(id: &str) -> String {
    let j = case_json(id);
    format!(
        r#"{{"format":"ck-part","version":1,"features":{}}}"#,
        ck_json::to_string(j.get("features").expect("features"))
    )
}

fn run(src: &str) -> (Vec<(String, String)>, Option<f64>, usize) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    let errs = r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect();
    let body = ex.0.body.as_ref();
    let vol = body.map(|b| ck_topo::mass::mass_props(b).volume);
    let issues = body.map(|b| ck_check::validate(b, 1e-6).len()).unwrap_or(0);
    (errs, vol, issues)
}

/// The original kernel's volume for `cap7_thicken_open_loft` (recorded by `tools/prop_probe.mjs`).
const ORIG: f64 = 3145.8631579932917;

#[test]
fn open_sheet_thicken_matches_original() {
    let (errs, vol, issues) = run(&case_part("cap7_thicken_open_loft"));
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0, "validate issues");
    let v = vol.expect("a body");
    eprintln!("cap7: volume {v:.3} original {ORIG:.3} ({:+.4} %)", 100.0 * (v - ORIG) / ORIG);
    assert!((v - ORIG).abs() <= 1e-3 * ORIG, "volume {v} against the original's {ORIG}");
}

/// A double-sided wall of a planar sheet is the sheet's area times the thickness: doubling the
/// thickness must double the volume (the rim miters only correct the corners).
#[test]
fn open_sheet_thicken_volume_scales_with_thickness() {
    let (a, va, ia) = run(&case_part_thick("cap7_thicken_open_loft", 2.0));
    let (b, vb, ib) = run(&case_part_thick("cap7_thicken_open_loft", 4.0));
    assert!(a.is_empty() && b.is_empty(), "{a:?} {b:?}");
    assert_eq!(ia + ib, 0, "validate issues");
    let (x, y) = (va.expect("body"), vb.expect("body"));
    eprintln!("cap7 t=2 {x:.3} t=4 {y:.3} ratio {:.4}", y / x);
    assert!((y / x - 2.0).abs() < 0.02, "t=4 volume {y} against twice t=2's {x}");
}
