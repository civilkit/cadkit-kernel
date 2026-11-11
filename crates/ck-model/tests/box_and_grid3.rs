//! The `box` primitive and the third direction (`dir3`) of the linear pattern.
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

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-6 * b.abs().max(1.0)
}

#[test]
fn a_box_has_the_volume_and_the_faces_of_an_extruded_rectangle() {
    let r = run(r#"{"type":"box","id":"F1","center":[1,2,3],"size":[10,20,30]}"#);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(r.valid.is_empty(), "{:?}", r.valid);
    assert!(near(r.vol, 6000.0), "{}", r.vol);
    let names: Vec<&str> = r.faces.iter().map(|f| f.0.as_str()).collect();
    for n in ["F1.S.e1", "F1.S.e2", "F1.S.e3", "F1.S.e4", "F1.C0", "F1.C1"] {
        assert!(names.contains(&n), "{n} in {names:?}");
    }
    // width / depth / height as well, and the sides must be above 0
    assert!(near(run(r#"{"type":"box","id":"F1","width":4,"depth":5,"height":6}"#).vol, 120.0));
    assert!(!run(r#"{"type":"box","id":"F1","size":[4,0,6]}"#).errors.is_empty());
}

#[test]
fn a_box_cuts_and_unites() {
    let r = run(r#"{"type":"box","id":"F1","size":[40,40,40]},{"type":"box","id":"F2","op":"cut","center":[20,0,0],"size":[20,10,10]}"#);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(near(r.vol, 63000.0), "{}", r.vol);
}

fn grid(extra: &str) -> Res {
    run(&format!(
        r#"{{"type":"box","id":"F1","size":[10,10,10]}},
        {{"type":"lpattern","id":"F2","seeds":["F1"],"dir":{{"kind":"axis","axis":"x"}},"spacing":20,"count":3{extra}}}"#
    ))
}

#[test]
fn the_third_direction_makes_a_block_of_copies() {
    // 3 x 2 x 4 = 24 separate cubes of 1000 mm3
    let r = grid(r#","dir2":{"kind":"axis","axis":"y"},"spacing2":20,"count2":2,"dir3":{"kind":"axis","axis":"z"},"spacing3":20,"count3":4"#);
    eprintln!("GRID3 errors {:?} vol {} valid {:?}", r.errors, r.vol, r.valid);
    assert!(r.errors.iter().all(|e| !e.contains("invalid")), "{:?}", r.errors);
    assert!(near(r.vol, 24000.0), "{}", r.vol);
    // without dir3 it is the 3 x 2 grid as before
    let two = grid(r#","dir2":{"kind":"axis","axis":"y"},"spacing2":20,"count2":2"#);
    assert!(near(two.vol, 6000.0), "{}", two.vol);
    // dir3 alone is a second direction
    let alt = grid(r#","dir3":{"kind":"axis","axis":"z"},"spacing3":20,"count3":2"#);
    assert!(near(alt.vol, 6000.0), "{}", alt.vol);
}

#[test]
fn the_third_direction_must_leave_the_plane_of_the_first_two() {
    let r = grid(r#","dir2":{"kind":"axis","axis":"y"},"spacing2":20,"count2":2,"dir3":{"kind":"vector","dir":[1,1,0]},"spacing3":20,"count3":2"#);
    assert!(r.errors.iter().any(|e| e.contains("dir3") || e.contains("plane")), "{:?}", r.errors);
    let r = grid(r#","dir3":{"kind":"axis","axis":"x"},"spacing3":20,"count3":2"#);
    assert!(r.errors.iter().any(|e| e.contains("dir3") || e.contains("along")), "{:?}", r.errors);
}
