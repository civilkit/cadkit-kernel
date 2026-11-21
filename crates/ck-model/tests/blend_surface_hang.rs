//! A `blendSurface` on a plate that is not at the origin must finish. The fixture is the document that left the 0.8.5
//! release "Busy" with no error: the blend's edge was measured by an adaptive integral that could not reach its tolerance
//! and bisected to 2^30 panels (see `proc_length`, which now has a work budget counted in panels, no clock needed).
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

fn run(src: &str) -> (regen::RegenResult, Brep) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (r, ex.brep)
}

/// Regen, then tessellate the sheet and measure it the way the app's view does.
fn build(src: &str, radius: f64) -> (f64, usize) {
    let (r, b) = run(src);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect::<Vec<_>>());
    let (_, sheet) = b.surfs.iter().find(|(i, _)| i == "B1").expect("the blend sheet");
    let t = ck_tess::mesh::tessellate(sheet);
    assert!(t.mesh.failed.is_empty(), "failed faces {:?}", t.mesh.failed);
    let tri_area: f64 = t
        .mesh
        .positions
        .chunks(9)
        .map(|c| {
            let p = |k: usize| ck_core::linalg::Vec3::new(c[3 * k] as f64, c[3 * k + 1] as f64, c[3 * k + 2] as f64);
            (p(1) - p(0)).cross(p(2) - p(0)).len() * 0.5
        })
        .sum();
    // a quarter cylinder of the radius along the 25 mm edge
    let want = std::f64::consts::PI * radius * 25.0 / 2.0;
    assert!((tri_area - want).abs() < 0.01 * want, "mesh area {tri_area} vs {want}");
    (tri_area, t.mesh.tri_face.len())
}

#[test]
fn offorigin_plate_blend_builds_and_tessellates() {
    build(include_str!("fixtures/blend_plate_offorigin.ckpart.json"), 3.0);
}

#[test]
fn offorigin_plate_blend_other_radii() {
    let src = include_str!("fixtures/blend_plate_offorigin.ckpart.json");
    for r in [1.0, 2.0, 5.0, 8.0] {
        build(&src.replace("\"radius\":3", &format!("\"radius\":{r}")), r);
    }
}

/// The second case found by the fuzz: bottom face + the far side face of a plate AT the origin, radius 2, automatic sides.
/// It hung the 0.8.5 wasm for the same reason as the off-origin plate.
#[test]
fn origin_plate_bottom_and_far_side_blend_builds_and_tessellates() {
    let src = r#"{"format":"ck-part","version":1,"name":"P","seq":4,"rollback":null,"material":"steel","counters":{},"features":[
{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":25},{"id":"p4","x":0,"y":25}],"entities":[{"id":"e1","type":"line","p1":"O","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"O"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
{"type":"blendSurface","id":"B1","faces":["F2.C0","F2.S.e3"],"radius":2,"tolerance":0.001}]}"#;
    let (r, b) = run(src);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect::<Vec<_>>());
    let (_, sheet) = b.surfs.iter().find(|(i, _)| i == "B1").expect("the blend sheet");
    let t = ck_tess::mesh::tessellate(sheet);
    assert!(t.mesh.failed.is_empty(), "failed faces {:?}", t.mesh.failed);
    assert!(!t.mesh.tri_face.is_empty());
}

/// The sibling path: a `sweepSurface` tube (r 2, 25 long) away from the origin. Its edges are iso-curves of the sweep, measured by
/// the same `proc_length`.
#[test]
fn sweep_tube_off_the_origin_builds_and_tessellates() {
    let src = include_str!("fixtures/sweep_tube_offorigin.ckpart.json");
    for (x, y) in [(0.0, 0.0), (15.0, 40.0), (4000.0, 4000.0)] {
        let doc = src.replace("@OX@", &format!("{x}")).replace("@OY@", &format!("{y}"));
        let (r, b) = run(&doc);
        assert!(r.errors.is_empty(), "offset ({x},{y}): {:?}", r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect::<Vec<_>>());
        let (_, sheet) = b.surfs.iter().find(|(i, _)| i == "W1").expect("the sweep sheet");
        let t = ck_tess::mesh::tessellate(sheet);
        assert!(t.mesh.failed.is_empty(), "offset ({x},{y}): failed faces {:?}", t.mesh.failed);
        let area: f64 = t
            .mesh
            .positions
            .chunks(9)
            .map(|c| {
                let p = |k: usize| ck_core::linalg::Vec3::new(c[3 * k] as f64, c[3 * k + 1] as f64, c[3 * k + 2] as f64);
                (p(1) - p(0)).cross(p(2) - p(0)).len() * 0.5
            })
            .sum();
        let want = 2.0 * std::f64::consts::PI * 2.0 * 25.0;
        assert!((area - want).abs() < 0.01 * want, "offset ({x},{y}): mesh area {area} vs {want}");
    }
}
