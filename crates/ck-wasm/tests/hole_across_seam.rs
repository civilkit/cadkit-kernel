//! Regression: the set-screw hole of flangeHalf straddles the hub cylinder's seam. The two halves of its loop end a few
//! ulps apart (each edge evaluates its own pcurve), so the strip boundary was left unstitched and the tessellation of the
//! hub wall filled the hole in (area 5150 instead of 5130, a disc of triangles over the hole).
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};

struct Ex(Brep);
impl regen::FeatureExec for Ex {
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

#[test]
fn set_screw_hole_is_cut_out_of_hub_wall() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/cases/flangeHalf.ckpart")).unwrap();
    let part = ck_model::Part::parse(&src).unwrap();
    let mut ex = Ex(Brep::new());
    regen::regen(&part, &RegenOpts::default(), &mut ex);
    let body = ex.0.body.take().unwrap();
    let m = ck_tess::mesh::tessellate(&body).mesh;
    let tri = |i: usize| -> [[f64; 3]; 3] {
        let o = i * 9;
        let p = |k: usize| [m.positions[o + 3 * k] as f64, m.positions[o + 3 * k + 1] as f64, m.positions[o + 3 * k + 2] as f64];
        [p(0), p(1), p(2)]
    };
    let area = |t: &[[f64; 3]; 3]| -> f64 {
        let a = [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]];
        let b = [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]];
        0.5 * ((a[1] * b[2] - a[2] * b[1]).powi(2) + (a[2] * b[0] - a[0] * b[2]).powi(2) + (a[0] * b[1] - a[1] * b[0]).powi(2)).sqrt()
    };
    let (hub, wall) = (19, 20);
    // hole axis from the wall's bounding box: the longest extent is the axis
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for i in (0..m.tri_face.len()).filter(|&i| m.tri_face[i] == wall) {
        for p in tri(i) {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
    let ext: Vec<f64> = (0..3).map(|k| hi[k] - lo[k]).collect();
    let ax = (0..3).max_by(|&a, &b| ext[a].total_cmp(&ext[b])).unwrap();
    let mid: Vec<f64> = (0..3).map(|k| 0.5 * (lo[k] + hi[k])).collect();
    let mut total = 0.0;
    for i in (0..m.tri_face.len()).filter(|&i| m.tri_face[i] == hub) {
        let t = tri(i);
        total += area(&t);
        let c: Vec<f64> = (0..3).map(|k| (t[0][k] + t[1][k] + t[2][k]) / 3.0).collect();
        let radial = (0..3).filter(|&k| k != ax).map(|k| (c[k] - mid[k]).powi(2)).sum::<f64>().sqrt();
        let same_side = c[ax] * mid[ax] > 0.0;
        assert!(!(same_side && radial < 2.0), "hub wall triangle covers the set-screw hole: centroid {:?}", c);
    }
    assert!((total - 5130.11).abs() < 5.13, "hub wall area {total}");
}
