//! Display-mesh accuracy against the exact geometry: the largest chordal deviation of the edge polylines (from their
//! curves) and of the face triangles (triangle edge midpoints against the surface at the uv midpoint) stays within the
//! chord tolerance, and the dome cap (cylinder holes cut into a sphere: intersection curves with ~300 knots each) is
//! meshed with a few ten-thousand triangles, not a hundred thousand.
use ck_geom::surface::Surface;
use ck_geom::Vec2;
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

fn body_of(text: &str) -> ck_topo::body::Body {
    let part = ck_model::Part::parse(text).unwrap();
    let mut ex = Ex(Brep::new());
    regen::regen(&part, &RegenOpts::default(), &mut ex);
    ex.0.body.take().unwrap()
}

/// (max edge-polyline sag, max triangle-edge midpoint deviation on curved faces, triangles)
fn deviations(body: &ck_topo::body::Body) -> (f64, f64, usize, f64) {
    let tess = ck_tess::mesh::tessellate(body);
    let tol = tess.tol.unwrap();
    let mut edge_dev = 0.0f64;
    for e in body.edge_ids() {
        let ed = body.edge(e);
        let ps = &tess.edges.params[e.idx()];
        for w in ps.windows(2) {
            let (a, b) = (ed.curve.eval(w[0]), ed.curve.eval(w[1]));
            let m = ed.curve.eval(0.5 * (w[0] + w[1]));
            let d = b - a;
            let l = d.len();
            let sag = if l < 1e-300 { m.dist(a) } else { d.cross(m - a).len() / l };
            edge_dev = edge_dev.max(sag);
        }
    }
    let mut face_dev = 0.0f64;
    let mut tris = 0;
    for (wi, fid) in body.face_ids().enumerate() {
        let Some(fm) = &tess.faces[wi] else { continue };
        tris += fm.tris.len();
        let surf = &body.face(fid).surface;
        if matches!(surf, Surface::Plane { .. }) {
            continue;
        }
        for t in &fm.tris {
            for k in 0..3 {
                let (i, j) = (t[k] as usize, t[(k + 1) % 3] as usize);
                let mut uv: Vec2 = (fm.uv[i] + fm.uv[j]) * 0.5;
                if let Some((c, p)) = fm.win[0] {
                    if uv.x >= c + p {
                        uv.x -= p;
                    }
                }
                if let Some((c, p)) = fm.win[1] {
                    if uv.y >= c + p {
                        uv.y -= p;
                    }
                }
                let s = surf.eval(uv.x, uv.y);
                face_dev = face_dev.max(s.dist((fm.pts[i] + fm.pts[j]) * 0.5));
            }
        }
    }
    (edge_dev, face_dev, tris, tol.chord)
}

#[test]
fn dome_cap_deviation_within_chord_tolerance_and_lean() {
    let body = body_of(include_str!("fixtures/dome_cap.ckpart"));
    let (edge_dev, face_dev, tris, chord) = deviations(&body);
    eprintln!("dome_cap: chord {chord:.4} edge dev {edge_dev:.4} face dev {face_dev:.4} tris {tris}");
    assert!(edge_dev <= chord * (1.0 + 1e-9), "edge polyline deviates {edge_dev} > {chord}");
    assert!(face_dev <= chord * 1.001, "triangle edges deviate {face_dev} > {chord}");
    assert!(tris < 30000, "{tris} triangles");
}
