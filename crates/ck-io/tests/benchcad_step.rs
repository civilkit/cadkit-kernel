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
    check_file(&gt(name), name, vol, area)
}

fn check_brepjs(name: &str, vol: f64, area: f64) {
    let p = format!("{}/../../tools/bench/brepjs/gt/{name}.step", env!("CARGO_MANIFEST_DIR"));
    check_file(&std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}")), name, vol, area)
}

fn check_file(text: &str, name: &str, vol: f64, area: f64) {
    let r = read_step(text).unwrap_or_else(|e| panic!("{name}: {e}"));
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

/// OCCT writes FACE_BOUND(.F.) on torus fillet faces whose loop already runs
/// against its neighbours: SHELL_ORIENTATION and a volume 9-44% low.
#[test]
fn torus_fillet_face_bound_orientation() {
    check("motor_end_cap_000242", 245873.66139676693, 43117.68510201314);
    check("rect_frame_008694", 43446.94836285351, 28324.399303470338);
    check_brepjs("rubber-foot", 4200.65768740757, 1830.9416713149517);
}

/// STEP PARABOLA / HYPERBOLA edges (OCCT cone / plane intersections): focus-form
/// Conic with the near branch, edges whose carrier runs against the vertex order.
#[test]
fn parabola_and_hyperbola_edges() {
    check("snap_clip_000052", 4.315531069529865, 29.576804605002973);
    check_brepjs("rotary-potentiometer", 7752.984457404009, 2753.2684415038934);
}

/// The conduit clip reference is itself non-manifold (one EDGE_CURVE used by four
/// oriented edges): volume and area must still match, validate reports it.
#[test]
fn hyperbola_edges_non_manifold_reference() {
    let p = format!("{}/../../tools/bench/brepjs/gt/conduit-snap-clip.step", env!("CARGO_MANIFEST_DIR"));
    let r = read_step(&std::fs::read_to_string(&p).unwrap()).unwrap();
    let m = ck_topo::mass::mass_props(&r.body);
    assert!((m.volume - 2687.6807671338415).abs() <= 1e-3 * 2687.68, "volume {}", m.volume);
    assert!((m.area - 2329.112149254925).abs() <= 1e-3 * 2329.11, "area {}", m.area);
    let issues = ck_check::validate(&r.body, 1e-6);
    assert!(issues.iter().all(|i| i.code == "SHELL_NONMANIFOLD"), "{issues:?}");
}

/// Import -> export -> import of files with PARABOLA / HYPERBOLA edges: the
/// writer emits the analytic conics (not B-spline approximations) and the
/// second import reproduces volume, area and a clean validate.
#[test]
fn conic_edges_round_trip() {
    for (path, vol, area) in [
        ("benchcad/gt/snap_clip_000052.step", 4.315531069529865, 29.576804605002973),
        ("brepjs/gt/rotary-potentiometer.step", 7752.984457404009, 2753.2684415038934),
    ] {
        let p = format!("{}/../../tools/bench/{path}", env!("CARGO_MANIFEST_DIR"));
        let r = read_step(&std::fs::read_to_string(&p).unwrap()).unwrap();
        let out = ck_io::step::write_step(&r.body, "rt", &ck_io::step::StepOpts { unix_ms: 0.0 });
        
        if let Ok(d) = std::env::var("DUMP_RT") {
            std::fs::write(format!("{d}/{}", path.rsplit('/').next().unwrap()), &out).unwrap();
        }
        let r2 = read_step(&out).unwrap_or_else(|e| panic!("{path}: re-import {e}"));
        let m = ck_topo::mass::mass_props(&r2.body);
        assert!((m.volume - vol).abs() <= 1e-3 * vol, "{path}: volume {} vs {vol}", m.volume);
        assert!((m.area - area).abs() <= 1e-3 * area, "{path}: area {} vs {area}", m.area);
        let issues = ck_check::validate(&r2.body, 1e-6);
        assert!(issues.is_empty(), "{path}: {issues:?}");
    }
}
