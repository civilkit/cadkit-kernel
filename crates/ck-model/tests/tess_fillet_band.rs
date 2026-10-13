//! Regression: a torus fillet band (v range ending exactly on the torus' period edge, or running
//! past it) must be tessellated as the band, not folded over itself. Before the fix the strip
//! mapping shifted the boundary segment lying on the window edge a period down, so the band's
//! mesh covered the wrong part of the tube (area 2.4x the true value, mesh volume 16% off, body
//! box too small) and the corner spheres of a big three-edge fillet came out empty.

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};

struct Ex(Brep);
impl FeatureExec for Ex {
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

/// (mesh area per face, mesh signed volume, per-face surface is torus/sphere)
fn mesh_stats(src: &str) -> (Vec<(f64, bool)>, f64, Vec<usize>) {
    let part = ck_model::Part::parse(src).unwrap();
    let mut ex = Ex(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "regen errors");
    let body = ex.0.body.as_ref().expect("body");
    let t = ck_tess::mesh::tessellate(body);
    let m = &t.mesh;
    let nf = body.face_ids().count();
    let mut area = vec![0.0f64; nf];
    let mut tris = vec![0usize; nf];
    let mut vol = 0.0;
    for i in 0..m.tri_face.len() {
        let p = |k: usize| [m.positions[i * 9 + k * 3] as f64, m.positions[i * 9 + k * 3 + 1] as f64, m.positions[i * 9 + k * 3 + 2] as f64];
        let (a, b, c) = (p(0), p(1), p(2));
        let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
        let cr = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        let f = m.tri_face[i] as usize;
        area[f] += 0.5 * (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
        tris[f] += 1;
        vol += (a[0] * cr[0] + a[1] * cr[1] + a[2] * cr[2]) / 6.0;
    }
    use ck_geom::surface::Surface;
    let curved: Vec<(f64, bool)> = body.face_ids().enumerate().map(|(i, f)| (area[i], matches!(body.face(f).surface, Surface::Torus { .. } | Surface::Sphere { .. }))).collect();
    (curved, vol, tris)
}

// cylinder r=10, h=10 with a bottom edge fillet r=2: the fillet is a quarter torus of centre-circle radius 8, tube radius 2
// (area = 2 pi r (Rc pi/2 + r) = 183.05), band v = 3pi/2 .. 2pi ends on the period edge.
const CYL_BOT: &str = r#"{"format":"ck-part","version":1,"features":[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":5,"y":3}],"entities":[{"type":"circle","id":"e7","c":"c","r":10}],"constraints":[]},{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},{"type":"fillet","id":"F3","radius":2,"edges":[{"faces":["F2.C0","F2.S.e7"],"at":[15,0,-3]}]}]}"#;

// revolved frustum (r 10 -> 6, h 10) with a bottom rim fillet r=1: the band straddles/ends at the torus seam.
const FRUSTUM_BOT: &str = r#"{"format":"ck-part","version":1,"features":[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":6,"y":10},{"id":"p4","x":0,"y":10},{"id":"a1","x":0,"y":0},{"id":"a2","x":0,"y":10}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"},{"type":"line","id":"ax","p1":"a1","p2":"a2","construction":true}],"constraints":[]},{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360},{"type":"fillet","id":"F3","radius":1,"edges":[{"faces":["F2.S.e1","F2.S.e2"],"at":[-10,0,0]}]}]}"#;

#[test]
fn cylinder_bottom_fillet_torus_band_has_the_closed_form_area() {
    let (faces, vol, _) = mesh_stats(CYL_BOT);
    let torus = faces.iter().find(|f| f.1).expect("torus face").0;
    let exact = 2.0 * std::f64::consts::PI * 2.0 * (8.0 * std::f64::consts::FRAC_PI_2 + 2.0);
    assert!((torus - exact).abs() < 2e-3 * exact, "torus band area {torus} vs {exact}");
    // body volume (massProps, shared with the oracle) 3090.0668137679
    assert!((vol - 3090.0668137679).abs() < 3e-3 * 3090.0668137679, "mesh volume {vol}");
}

#[test]
fn frustum_bottom_fillet_torus_band_mesh_volume_matches() {
    let (faces, vol, _) = mesh_stats(FRUSTUM_BOT);
    // body volume (massProps, shared with the oracle) 2022.5244519897235; before the fix the mesh gave 1690
    assert!((vol - 2022.5244519897235).abs() < 3e-3 * 2022.5244519897235, "mesh volume {vol}");
    let torus = faces.iter().find(|f| f.1).expect("torus face").0;
    assert!((torus - 113.38).abs() < 0.01 * 113.38, "torus band area {torus}");
}
