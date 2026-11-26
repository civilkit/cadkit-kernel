//! Shell on curved and stepped bodies (tools/cases/shell_bodies): each fixture is a body plus a `shell` as its last
//! feature. The volumes are the kernel's own, checked against the closed form where there is one (the sphere).
//! The fixtures that do not shell yet are kept beside them: an opening at a curved face is refused (`shellUnsupported`), a
//! hemisphere opened at its flat face is pinned as passing with its closed form.
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

/// A cylinder with a rectangular notch cut across the seam: the notch walls meet the cylinder in saddle-shaped corners.
#[test]
fn cylinder_with_a_notch_across_the_seam() {
    passes(fixture!("saddle_notch_cyl_open_top"), 2178.260);
    passes(fixture!("saddle_notch_cyl_closed"), 2518.730);
}

/// A thickened free-form surface shells: its walls used to be built with their loops against the outward normal, which made the
/// offset of the rim fail. Volume checked against the body (565.24) less the cavity (a 2 mm slab inside the 4 mm one).
#[test]
fn thickened_free_form_surface_shells() {
    passes(fixture!("thickened_freeform_closed"), 369.726);
}

/// A drafted block with its four oblique edges rounded (the round's ends are conics) shells: the offset rebuilds the ellipse
/// where two offset faces meet. Volume against the cross-section integral (3231.1: rounded-rectangle section less the section
/// moved in by the wall over cos 8 degrees, a floor of one wall), the rolling-ball corner not being a circle in a horizontal cut.
#[test]
fn drafted_block_with_rounded_edges_shells() {
    let (errors, v, valid) = shell_body(fixture!("tapered_fillet_open_top"));
    assert!(errors.is_empty(), "{errors:?}");
    assert!(valid);
    assert!((v - 3231.1).abs() < 2e-3 * 3231.1, "volume {v}");
}

/// Opening a body at a cone wall is not built yet: the refusal names it (`shellUnsupported`) and the body stays as it was.
/// (The wall of a through hole is built: see `shell_hole_wall.rs`.)
#[test]
fn shell_opened_at_a_curved_face_is_refused() {
    let (errors, _, valid) = shell_body(fixture!("cone_side_wall_opened"));
    assert_eq!(errors.len(), 1, "{errors:?}");
    assert!(errors[0].starts_with("S:") && errors[0].contains("shellUnsupported"), "{errors:?}");
    assert!(valid, "the body before the shell stays valid");
}

/// The 40 x 40 x 20 block with the 8 mm through hole, shelled 2 mm and open at the hole wall: body (32000 - 16 pi 20) less
/// the cavity inside it (36 x 36 x 16 less the hole's cylinder over 16) = 11062.94.
#[test]
fn shell_opened_at_the_wall_of_a_through_hole() {
    let pi = std::f64::consts::PI;
    passes(fixture!("hole_wall_opened"), 32000.0 - 16.0 * pi * 20.0 - (36.0 * 36.0 * 16.0 - 16.0 * pi * 16.0));
}

/// A hemisphere (R 15) shelled 2 mm inward and open at its flat face: half of the closed-sphere wall, 2/3 pi (15^3 - 13^3).
#[test]
fn shell_of_a_hemisphere_opened_at_the_flat_face() {
    let expect = 2.0 / 3.0 * std::f64::consts::PI * (15.0f64.powi(3) - 13.0f64.powi(3));
    passes(fixture!("halfsphere_flat_opened"), expect);
}
