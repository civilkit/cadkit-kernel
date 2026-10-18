//! Shell and thicken of polygon-section (creased) multi-station lofts: the original-kernel gap cases.
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
}

pub fn run_part(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

pub fn case_json(file: &str) -> ck_json::Json {
    let p = format!("{}/../../tools/cases/original-gaps/{file}", env!("CARGO_MANIFEST_DIR"));
    ck_json::parse(&std::fs::read_to_string(&p).unwrap()).unwrap()
}

fn case_part(id: &str) -> String {
    let j = case_json(&format!("{id}.json"));
    format!(r#"{{"format":"ck-part","version":1,"features":{}}}"#, ck_json::to_string(j.get("features").expect("features")))
}

/// The original kernel's volume of case `id` (recorded by `tools/orig_diff_sweep.mjs --seed 7`).
fn orig_volume(id: &str) -> f64 {
    case_json("orig_volumes.json").get(id).and_then(|c| c.get_f64("volume")).unwrap_or_else(|| panic!("no recorded volume for {id}"))
}

fn volume(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().expect("a body")).volume
}

fn assert_valid_solid(name: &str, o: &Out) {
    let b = o.brep.body.as_ref().expect("a body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{name}: {iss:?}");
}

fn check_against_original(id: &str) {
    let o = run_part(&case_part(id));
    assert!(o.errors.is_empty(), "{id}: {:?}", o.errors);
    assert_valid_solid(id, &o);
    let (v, want) = (volume(&o), orig_volume(id));
    eprintln!("{id}: volume {v:.1} original {want:.1} ({:+.4} %)", 100.0 * (v - want) / want);
    assert!((v - want).abs() <= 1e-3 * want, "{id}: volume {v} against the original's {want}");
}

/// The flared 7-station boat hull of the hand test, deck removed, 3 mm wall: a polygon-section loft whose opened
/// top is a free-form (B-spline) face. The original's volume is 20,643,366.8 mm3.
#[test]
fn seed_hull_shell_matches_original() {
    check_against_original("hull_seed_shell3");
}

/// Random 5 to 8 station hulls of `node tools/orig_diff_sweep.mjs --samples 10 --seed 7` (walls 1 to 5 mm).
#[test]
fn random_hull_shells_match_original() {
    for id in ["hull_shell_0", "hull_shell_1", "hull_shell_2", "hull_shell_3"] {
        check_against_original(id);
    }
}

#[test]
fn more_random_hull_shells_match_original() {
    for id in ["hull_shell_5", "hull_shell_6", "hull_shell_7", "hull_shell_8", "hull_shell_9"] {
        check_against_original(id);
    }
}

/// The shell of a convex body opened on one face is `area x t` less the curvature term: the wall volume must lie
/// between 90 % and 100 % of the kept area times the thickness.
#[test]
fn seed_hull_wall_is_area_times_thickness() {
    let base = {
        let src = case_part("hull_seed_shell3").replace(r#",{"type":"shell","id":"F9","faces":["F8.S.e0_2"],"thickness":3}"#, "");
        run_part(&src)
    };
    assert!(base.errors.is_empty(), "{:?}", base.errors);
    let b = base.brep.body.as_ref().unwrap();
    let total = ck_topo::mass::mass_props(b).area;
    let top = b.face_ids().find(|&f| b.face_id_str(f) == "F8.S.e0_2").unwrap();
    let top_area = ck_topo::mass::faces_moments(b, &[top], ck_geom::Vec3::ZERO).area;
    let wall = volume(&run_part(&case_part("hull_seed_shell3")));
    let at = (total - top_area) * 3.0;
    assert!(wall < at && wall > 0.9 * at, "wall {wall} against area x t {at}");
}
