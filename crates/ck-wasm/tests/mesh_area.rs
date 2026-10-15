//! Display-mesh area against the exact B-rep area, per face. The tessellation is a view of the part, not a
//! measurement, but a face whose mesh drops area (a trimmed spline face, a periodic face, a face with many holes)
//! shows up as a hole in the picture and breaks every client that integrates the mesh. Fixtures: a revolved dome, a
//! propeller blade (lofted / swept spline faces) and a cam (spline profile extrusion).
//!
//! `CK_MESH_AREA_PARTS=<file>,<file>..` (ignored test `audit`) prints the worst faces of arbitrary parts.
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

fn try_body_of(text: &str) -> Option<ck_topo::body::Body> {
    let part = ck_model::Part::parse(text).ok()?;
    let mut ex = Ex(Brep::new());
    regen::regen(&part, &RegenOpts::default(), &mut ex);
    ex.0.body.take()
}

fn body_of(text: &str) -> ck_topo::body::Body {
    try_body_of(text).unwrap()
}

struct FaceRow {
    name: String,
    id: usize,
    kind: &'static str,
    exact: f64,
    mesh: f64,
}

/// (total exact area, total mesh area, per-face rows)
fn audit_body(body: &ck_topo::body::Body) -> (f64, f64, Vec<FaceRow>) {
    let tess = ck_tess::mesh::tessellate(body);
    let mut rows = vec![];
    let (mut te, mut tm) = (0.0, 0.0);
    for (wi, fid) in body.face_ids().enumerate() {
        let exact = ck_topo::mass::face_integral::<1>(body, fid, 1, &|_, n| [n.len()])[0].abs();
        let mesh: f64 = tess.faces[wi]
            .as_ref()
            .map(|fm| fm.tris.iter().map(|t| 0.5 * (fm.pts[t[1] as usize] - fm.pts[t[0] as usize]).cross(fm.pts[t[2] as usize] - fm.pts[t[0] as usize]).len()).sum())
            .unwrap_or(0.0);
        te += exact;
        tm += mesh;
        rows.push(FaceRow { id: fid.idx(), name: body.face_id_str(fid), kind: body.face(fid).surface.kind(), exact, mesh });
    }
    (te, tm, rows)
}

fn check(text: &str, total_tol: f64, face_tol: f64, what: &str) {
    let body = body_of(text);
    let (te, tm, rows) = audit_body(&body);
    let mut bad = vec![];
    for r in &rows {
        // (tiny faces, a few mm2 bounded by short arcs: an absolute slack of 0.5 mm2)
        if (r.mesh - r.exact).abs() > face_tol * r.exact + 0.5 {
            bad.push(format!("{} {} exact {:.4} mesh {:.4} ({:+.2e})", r.name, r.kind, r.exact, r.mesh, (r.mesh - r.exact) / r.exact));
        }
    }
    assert!(((tm - te) / te).abs() <= total_tol, "{what}: total mesh area {tm:.3} vs exact {te:.3} ({:+.2e})", (tm - te) / te);
    assert!(bad.is_empty(), "{what}: faces off by more than {face_tol:e}:\n{}", bad.join("\n"));
}

#[test]
fn revolved_dome_mesh_area() {
    check(include_str!("fixtures/dome_cap.ckpart"), 3e-3, 3e-2, "dome cap");
}

#[test]
fn propeller_mesh_area() {
    check(include_str!("../../../tools/bench/benchcad/propeller_000192.ckpart.json"), 3e-3, 3e-2, "propeller");
}

#[test]
fn cam_mesh_area() {
    check(include_str!("../../../tools/bench/benchcad/cam_012392.ckpart.json"), 3e-3, 3e-2, "cam");
}

#[test]
#[ignore]
fn audit() {
    let Ok(list) = std::env::var("CK_MESH_AREA_PARTS") else { return };
    for p in list.split(',') {
        let text = std::fs::read_to_string(p).unwrap();
        let Some(body) = try_body_of(&text) else {
            eprintln!("PART {p} no body");
            continue;
        };
        let (te, tm, mut rows) = audit_body(&body);
        eprintln!("PART {p} exact {te:.3} mesh {tm:.3} rel {:+.3e} faces {}", (tm - te) / te, rows.len());
        rows.sort_by(|a, b| ((a.mesh - a.exact)).partial_cmp(&(b.mesh - b.exact)).unwrap());
        for r in rows.iter().take(6).chain(rows.iter().rev().take(3)) {
            if (r.mesh - r.exact).abs() > 1e-3 * r.exact + 1e-3 {
                eprintln!("   #{} {} {} exact {:.4} mesh {:.4} ({:+.2e})", r.id, r.name, r.kind, r.exact, r.mesh, (r.mesh - r.exact) / r.exact);
            }
        }
    }
}
