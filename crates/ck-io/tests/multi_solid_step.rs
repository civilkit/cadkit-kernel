//! Multi-solid STEP import and the faces the BenchCAD ground-truth files
//! exercise: a sweep face closed on itself (B-spline patch with a seam), a
//! torus fragment bounded by a seam on one side and a real edge on the other.
use ck_io::step::read_step;

fn gt(name: &str) -> String {
    std::fs::read_to_string(format!("{}/../../tools/bench/benchcad/gt/{name}.step", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

#[test]
fn torsion_spring_two_solids_import_with_full_volume() {
    let r = read_step(&gt("torsion_spring_000054")).unwrap();
    assert_eq!(r.solids, 2);
    assert_eq!(r.body.num_lumps(), 2);
    let mp = ck_topo::mass::mass_props(&r.body);
    assert!((mp.volume - 180176.517).abs() / 180176.517 < 1e-4, "volume {}", mp.volume);
    assert!((mp.area - 102528.72).abs() / 102528.72 < 1e-4, "area {}", mp.area);
    let lumps = ck_topo::mass::lump_moments(&r.body);
    assert!(lumps.iter().all(|m| m.volume > 0.0));
    assert!(ck_check::validate(&r.body, 1e-6).is_empty());
    // the closed sweep face tessellates (bbox reaches the coil radius)
    let t = ck_tess::mesh::tessellate(&r.body);
    let (lo, hi) = t.mesh.bbox().unwrap();
    assert!(lo.x < -59.0 && hi.x > 59.0, "{lo:?} {hi:?}");
}

#[test]
fn eyebolt_torus_fragments_tessellate() {
    let r = read_step(&gt("eyebolt_000265")).unwrap();
    let t = ck_tess::mesh::tessellate(&r.body);
    let (lo, hi) = t.mesh.bbox().unwrap();
    // the torus eye spans x = +-63
    assert!(lo.x < -62.9 && hi.x > 62.9, "{lo:?} {hi:?}");
    assert!(t.faces.iter().all(|f| f.is_some()), "a face failed to tessellate");
}
