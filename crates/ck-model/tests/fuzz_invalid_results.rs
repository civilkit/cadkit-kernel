//! Crash-fuzz cases (seed 7, `docs/crash-fuzz.md`) that answered `ok` with a body that fails validation: a feature that cannot give a
//! valid solid must say so with a structured error, and one that succeeds must give a valid, closed mesh.
use ck_check::tessellation::check_mesh_why;
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

/// Error codes of the regen, and the validation and mesh issues of the body it left (empty when it left an error).
fn outcome(json: &str) -> (Vec<String>, Vec<String>) {
    let part = Part::parse(json).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors: Vec<String> = res.errors.iter().map(|e| e.error.code.clone()).collect();
    let mut issues = vec![];
    if let Some(body) = ex.0.body.clone() {
        issues.extend(ck_check::validate(&body, 1e-6).into_iter().map(|i| format!("{}: {}", i.code, i.detail)));
        let m = ck_tess::mesh::tessellate(&body).mesh;
        issues.extend(check_mesh_why(&body, &m.positions, &m.tri_face, &m.failed, &m.failed_why).into_iter().map(|i| format!("{}: {}", i.entity, i.detail)));
    }
    (errors, issues)
}

fn ok_or_valid(name: &str, json: &str) {
    let (errors, issues) = outcome(json);
    assert!(issues.is_empty(), "{name}: errors {errors:?} but the body is invalid: {issues:?}");
}

/// A pattern or mirror of a 1e-6 mm slab (the clamp of a zero or negative depth) welded the two faces of the slab together.
#[test]
fn patterns_and_mirrors_of_a_slab_refuse_or_stay_valid() {
    for (name, json) in [
        ("case_642", include_str!("fixtures/fuzz_case_642.json")),
        ("case_834", include_str!("fixtures/fuzz_case_834.json")),
        ("case_1114", include_str!("fixtures/fuzz_case_1114.json")),
        ("case_3369", include_str!("fixtures/fuzz_case_3369.json")),
        ("case_4392", include_str!("fixtures/fuzz_case_4392.json")),
    ] {
        ok_or_valid(name, json);
    }
}

/// The slab itself is a valid solid (K22); only combining it with a second operand is refused.
#[test]
fn single_slab_stays_valid() {
    let (errors, issues) = outcome(include_str!("fixtures/fuzz_slab_only.json"));
    assert!(errors.is_empty() && issues.is_empty(), "{errors:?} {issues:?}");
}

/// A circular surface extrude thickened by 1e-5.
#[test]
fn thin_thickened_surface() {
    ok_or_valid("case_3670", include_str!("fixtures/fuzz_case_3670.json"));
}

/// A slab a little thicker than the weld tolerance (1.2 micrometres, the size of the sliver that line-tangent cylinders leave,
/// `tangent_wires_tube_wall.rs`) is a valid operand: patterning it must not be refused as a `thinFeature`. Only a slab at or
/// below the weld distance (the 1e-6 clamp of a zero depth) is refused.
#[test]
fn slab_above_the_weld_distance_is_not_refused() {
    for depth in [1.2e-6, 1.5e-6, 1.9e-6, 3e-6] {
        let json = format!(
            r#"{{"format":"ck-part","version":1,"features":[{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"p0","x":0,"y":0}},{{"id":"p1","x":10,"y":0}},{{"id":"p2","x":10,"y":-9}},{{"id":"p3","x":0,"y":-9}}],"entities":[{{"type":"line","id":"e1","p1":"p0","p2":"p1"}},{{"type":"line","id":"e2","p1":"p1","p2":"p2"}},{{"type":"line","id":"e3","p1":"p2","p2":"p3"}},{{"type":"line","id":"e4","p1":"p3","p2":"p0"}}],"constraints":[]}},{{"type":"extrude","id":"F2","op":"add","sketch":"S1","end":"blind","depth":{depth}}},{{"type":"lpattern","id":"F5","seeds":["F2"],"dir":{{"kind":"axis","axis":"x"}},"spacing":15,"count":2}}]}}"#
        );
        let part = Part::parse(&json).unwrap();
        let mut ex = Exec(Brep::new());
        let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
        for e in &res.errors {
            assert!(!e.error.params.iter().any(|(k, _)| k == "thickness"), "depth {depth}: refused as a thin feature: {}: {}", e.error.code, e.feature);
        }
    }
}
