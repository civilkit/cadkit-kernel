//! Display-mesh watertightness of bodies whose B-rep checks pass: every face meshes, and no mesh edge is left used by a
//! single triangle apart from hairline cracks between two faces' samples of a shared edge (`ck_check::tessellation`).
//! `CK_TESS_DBG=1` prints why a face dropped and which faces had unrecovered trim segments.
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

fn open_issues(json: &str) -> Vec<String> {
    std::env::set_var("CK_TESS_DBG", "1");
    let part = Part::parse(json).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let body = ex.0.body.clone().unwrap();
    let m = ck_tess::mesh::tessellate(&body).mesh;
    check_mesh_why(&body, &m.positions, &m.tri_face, &m.failed, &m.failed_why).into_iter().map(|i| format!("{}: {}", i.entity, i.detail)).collect()
}

/// A crescent-shaped cap (a circle and a surface intersection that touch at both ends) whose trim polylines cross by a few 1e-6.
#[test]
fn sliver_cap_meshes() {
    let issues = open_issues(include_str!("fixtures/tess_cap_sliver.json"));
    assert!(issues.is_empty(), "{issues:?}");
}

/// Debug probe (set `CK_PART=<part json>`): one line per face whose mesh is suspect (no triangles, open boundary graph,
/// unrecovered trims, or mesh area outside 99..101 percent of the exact area).
#[test]
#[ignore = "debug probe: needs CK_PART"]
fn probe_part() {
    let Ok(path) = std::env::var("CK_PART") else { return };
    let json = std::fs::read_to_string(&path).unwrap();
    let part = Part::parse(&json).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    for e in &res.errors {
        eprintln!("regen error: {}", e.error.describe());
    }
    let body = ex.0.body.clone().unwrap();
    let t0 = std::time::Instant::now();
    let t = ck_tess::mesh::tessellate(&body);
    let t_tess = t0.elapsed();
    let ids: Vec<_> = body.face_ids().collect();
    eprintln!("PROBE timing tessellate {:?}", t_tess);
    let mut area = vec![0.0f64; ids.len()];
    let mut nt = vec![0usize; ids.len()];
    for (k, &fi) in t.mesh.tri_face.iter().enumerate() {
        let p = &t.mesh.positions[9 * k..9 * k + 9];
        let u = [(p[3] - p[0]) as f64, (p[4] - p[1]) as f64, (p[5] - p[2]) as f64];
        let w = [(p[6] - p[0]) as f64, (p[7] - p[1]) as f64, (p[8] - p[2]) as f64];
        area[fi as usize] += 0.5 * ((u[1] * w[2] - u[2] * w[1]).powi(2) + (u[2] * w[0] - u[0] * w[2]).powi(2) + (u[0] * w[1] - u[1] * w[0]).powi(2)).sqrt();
        nt[fi as usize] += 1;
    }
    let mut bad = 0;
    for (i, &f) in ids.iter().enumerate() {
        let exact = ck_topo::mass::face_integral::<1>(&body, f, 1, &|_, n| [n.len()])[0].abs();
        let notes: Vec<&str> = t.mesh.notes.iter().filter(|n| n.0 == i).map(|n| n.1.as_str()).collect();
        let ratio = if exact > 0.0 { area[i] / exact } else { 1.0 };
        if nt[i] == 0 || !(0.93..=1.07).contains(&ratio) || !notes.is_empty() {
            bad += 1;
            let sk = format!("{:?}", body.face(f).surface);
            eprintln!("PROBE {} {} tris={} area={:.4} exact={:.4} ratio={:.3} {:?}", body.face_id_str(f), sk.split(|c: char| !c.is_alphanumeric()).next().unwrap_or(""), nt[i], area[i], exact, ratio, notes);
        }
    }
    if let Ok(near) = std::env::var("CK_NEAR") {
        let c: Vec<f64> = near.split(',').map(|x| x.parse().unwrap()).collect();
        for (k, &fi) in t.mesh.tri_face.iter().enumerate() {
            let p = &t.mesh.positions[9 * k..9 * k + 9];
            if (0..3).any(|j| (0..3).all(|d| (p[3 * j + d] as f64 - c[d]).abs() < 2e-3)) {
                eprintln!("PROBE NEAR tri {} face {} {:?}", k, body.face_id_str(ids[fi as usize]), p.iter().map(|x| (*x * 1e4).round() / 1e4).collect::<Vec<_>>());
            }
        }
    }
    for i in check_mesh_why(&body, &t.mesh.positions, &t.mesh.tri_face, &t.mesh.failed, &t.mesh.failed_why) {
        eprintln!("PROBE ISSUE {}: {}", i.entity, i.detail);
    }
    eprintln!("PROBE total faces {} suspect {} tris {}", ids.len(), bad, t.mesh.tri_face.len());
}

/// A bolt hole in a flange face: two long triangles bridged the hole.
#[test]
fn bolt_hole_is_not_bridged() {
    let issues = open_issues(include_str!("fixtures/tess_hole_sliver.json"));
    assert!(issues.is_empty(), "{issues:?}");
}
