//! Shell on curved and stepped bodies (tools/cases/shell_bodies): each fixture is a body plus a `shell` as its last
//! feature. The volumes are the kernel's own, checked against the closed form where there is one (the sphere).
//! The fixtures that do not shell yet are kept beside them and listed in the ignored test at the end.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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

fn shell_body(src: &str) -> (Vec<String>, f64, bool) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let b = ex.brep.body.as_ref().unwrap();
    (errors, ck_topo::mass::mass_props(b).volume, ck_check::validate(b, 1e-6).is_empty())
}

macro_rules! fixture {
    ($name:literal) => {
        include_str!(concat!("../../../tools/cases/shell_bodies/", $name, ".json"))
    };
}

fn passes(src: &str, volume: f64) {
    let (errors, v, valid) = shell_body(src);
    assert!(errors.is_empty(), "{errors:?}");
    assert!(valid, "invalid body");
    assert!((v - volume).abs() < 0.05, "volume {v}, expected {volume}");
}

#[test]
fn truncated_cone_shelled_open_at_the_top_and_closed() {
    passes(fixture!("cone_open_top"), 7479.214);
    passes(fixture!("cone_closed"), 7904.524);
}

#[test]
fn cylinder_with_a_box_on_top() {
    passes(fixture!("cyl_box"), 2861.024);
    passes(fixture!("cyl_box_open_cyl_top"), 2648.429);
}

#[test]
fn stepped_tube_inward() {
    passes(fixture!("stepped_tube"), 3597.124);
}

#[test]
fn closed_spheres_have_the_closed_form_volume() {
    // 4/3 pi (15^3 - 13^3), a ball and a revolved half disc
    let expect = 4.0 / 3.0 * std::f64::consts::PI * (15.0f64.powi(3) - 13.0f64.powi(3));
    passes(fixture!("sphere_closed"), expect);
    passes(fixture!("rev_sphere_closed"), expect);
}

#[test]
fn swept_bar_with_a_bend() {
    passes(fixture!("swept_bar_bend"), 1518.478);
    passes(fixture!("swept_open_both_ends"), 1502.478);
}

#[test]
fn block_with_a_through_hole_shelled_open_at_the_top() {
    passes(fixture!("through_hole_open_top"), 9702.442);
}

#[test]
#[ignore = "known failures: shell of a body opened at a curved face or a hemisphere's flat face"]
fn shell_cases_that_do_not_work_yet() {
    for src in [fixture!("cone_side_wall_opened"), fixture!("halfsphere_flat_opened"), fixture!("hole_wall_opened")] {
        let (errors, _, _) = shell_body(src);
        assert!(errors.is_empty(), "{errors:?}");
    }
}
