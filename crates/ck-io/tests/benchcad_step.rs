//! BenchCAD ground-truth STEP files (OpenCASCADE output) that used to fail on
//! import: a dome revolved from an arc whose centre is ~1e-4 off the axis
//! (sphere pole boundary), a tee of two cylinders whose spline intersection
//! edges were fitted by a 32-sample cubic pcurve (EDGE_OFF_SURFACE), and a
//! coil spring whose 100k-triangle rational sweep face took minutes to mesh.
use ck_io::step::read::read_step;

fn gt(name: &str) -> String {
    let p = format!("{}/../../tools/bench/benchcad/gt/{name}.step", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"))
}

fn check(name: &str, vol: f64, area: f64) {
    let r = read_step(&gt(name)).unwrap_or_else(|e| panic!("{name}: {e}"));
    let m = ck_topo::mass::mass_props(&r.body);
    assert!((m.volume - vol).abs() <= 1e-3 * vol, "{name}: volume {} vs {vol}", m.volume);
    assert!((m.area - area).abs() <= 1e-3 * area, "{name}: area {} vs {area}", m.area);
    let issues = ck_check::validate(&r.body, 1e-6);
    assert!(issues.is_empty(), "{name}: {issues:?}");
}

#[test]
fn dome_cap_sphere_pole_boundary() {
    check("dome_cap_000212", 328738.724369601, 33426.47866758815);
}

#[test]
fn t_pipe_fitting_cylinder_intersection_edges() {
    check("t_pipe_fitting_000075", 104294.41712280322, 52635.72238704952);
}
