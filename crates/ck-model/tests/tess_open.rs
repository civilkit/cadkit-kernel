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

/// A bore wall cut by a branch hole (a tee): the rim of the wall is walked backwards after the strip is opened, which
/// used to leave one segment a period away and the wall 9 percent covered.
#[test]
fn bore_wall_with_a_branch_hole_is_covered() {
    let issues = open_issues(include_str!("fixtures/tess_tee_bore.json"));
    assert!(issues.is_empty(), "{issues:?}");
}

/// Torus bands that span a full period in v (a funnel rim): the band needs cut lines in both directions, and an edge on
/// the lower window edge belongs to the top of the window.
#[test]
fn torus_band_spanning_a_period_is_covered() {
    let issues = open_issues(include_str!("fixtures/tess_funnel_torus.json"));
    assert!(issues.is_empty(), "{issues:?}");
}

/// A sphere zone and a half torus whose circle junction sits on the cut line.
#[test]
fn ring_junction_on_the_cut_line_is_kept() {
    for json in [include_str!("fixtures/tess_hose_sphere.json"), include_str!("fixtures/tess_battery_torus.json")] {
        let issues = open_issues(json);
        assert!(issues.is_empty(), "{issues:?}");
    }
}

/// Flange rings with slits and bolt holes on the slit line: the chains either side of the slit connect over the dropped junction.
#[test]
fn slit_ring_with_bolt_holes_on_the_slit_is_closed() {
    let issues = open_issues(include_str!("fixtures/tess_coupling_slit.json"));
    assert!(issues.is_empty(), "{issues:?}");
}

/// Spline faces meeting at a cusp whose trim tail crosses the shared edge: no empty face, and the one-edge hairline is not a hole.
#[test]
fn spline_cusp_faces_mesh_without_holes() {
    let issues = open_issues(include_str!("fixtures/tess_large sample_cusp.json"));
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

/// A teardrop (BOSL2 onion): a sphere cap whose pole is at the low v end, and a cone to an apex on the axis. The strip's side
/// lines were left out when the pole lay below the loop.
#[test]
fn onion_sphere_and_cone_mesh() {
    let issues = open_issues(include_str!("fixtures/tess_onion.json"));
    assert!(issues.is_empty(), "{issues:?}");
}

/// Lofts through closed splines with a near-duplicate point (loft_fuzz hull mode, seeds 196 and 223): the section spline has a
/// micro-kink, and the lattice put a fan of nodes a few 1e-4 apart there, which a welded copy of the mesh collapsed unevenly.
#[test]
fn hull_lofts_mesh_without_holes() {
    for json in [include_str!("fixtures/tess_loft_hull196.json"), include_str!("fixtures/tess_loft_hull223.json")] {
        let issues = open_issues(json);
        assert!(issues.is_empty(), "{issues:?}");
    }
}

/// A circle revolved a full turn (one torus face): the mesh used to cover 84% of the face for R/r = 10 (an O-ring) and
/// less above it, and the horn torus (R = r) left mesh edges used by one triangle (solidcore #1491, #1710). Not covered:
/// a torus a hair short of a horn (R = 1.001 r, a 0.001 mm inner ring) still leaves a few open mesh edges.
#[test]
fn full_torus_meshes_closed_at_any_proportion() {
    use ck_geom::{Frame, Vec2, Vec3};
    use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    let mut bad = vec![];
    for (r, big) in [(3.0, 30.0), (1.0, 10.0), (1.0, 12.0), (1.0, 20.0), (1.0, 50.0), (1.5, 15.0), (1.0, 9.0), (4.0, 20.0), (3.0, 3.0), (5.0, 5.0)] {
        let fr = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
        let prof = Profile::new(fr, vec![ProfileRegion { outer: ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(big, 0.0), r)]), holes: vec![] }]);
        let body = ck_topo::make::revolve::revolve("F2", &prof, Vec3::ZERO, Vec3::Y, 0.0, 2.0 * std::f64::consts::PI).unwrap();
        let m = ck_tess::mesh::tessellate(&body).mesh;
        let issues: Vec<String> = check_mesh_why(&body, &m.positions, &m.tri_face, &m.failed, &m.failed_why).into_iter().map(|i| i.detail).collect();
        if !issues.is_empty() {
            bad.push(format!("r {r} R {big}: {issues:?}"));
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// The marine propeller default (a lofted blade fused to the hub, patterned, bored): every face meshes and the display
/// mesh is watertight (K17 reported `TESSELLATION_OPEN` here).
#[test]
fn marine_propeller_default_meshes_closed() {
    let issues = open_issues(include_str!("../../../tools/cases/marine_fuse/default_s5.ckpart"));
    assert!(issues.is_empty(), "{issues:?}");
}
