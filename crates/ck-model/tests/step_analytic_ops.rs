//! A STEP whose straight edges, circles and flat faces are B-splines (what exporters write) is read as lines, circles
//! and planes, so the analytic shell / fillet / chamfer paths take it. The plain (spline) read of the same text is what
//! these used to run on.
#[path = "../../ck-io/tests/spline_step/mod.rs"]
mod spline_step;
use ck_core::linalg::Vec3;
use ck_io::step::{read_step, read_step_plain};
use ck_topo::body::Body;

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn pick(b: &Body, a: Vec3, c: Vec3) -> (String, String, Vec3) {
    let e = b.edge_ids().find(|&e| { let ed = b.edge(e); let m = ed.curve.eval(0.5 * (ed.t0 + ed.t1)); m.dist((a + c) * 0.5) < 1e-6 }).expect("edge");
    let fs = b.edge_faces(e);
    (b.face_id_str(fs[0]), b.face_id_str(fs[1]), (a + c) * 0.5)
}

fn top_face(b: &Body) -> ck_topo::body::FaceId {
    b.face_ids().find(|&f| matches!(&b.face(f).surface, ck_geom::surface::Surface::Plane { frame } if frame.z.z > 0.99 && frame.o.z > 5.0)).expect("top face is a plane")
}

#[test]
fn chamfer_fillet_and_shell_work_on_the_recognised_box() {
    let text = spline_step::box_step(20.0, 10.0, 6.0, true);
    let plain = read_step_plain(&text).unwrap().body;
    let rec = read_step(&text).unwrap().body;
    let edge = |b: &Body| pick(b, Vec3::new(0.0, 0.0, 6.0), Vec3::new(20.0, 0.0, 6.0));
    // the spline version refuses the chamfer; the recognised one gives the closed-form volume
    let e = ck_ops::blend::build::chamfer(&plain, &[edge(&plain)], 1.0, "F9", 1e-6).err().expect("a chamfer on a spline edge was refused");
    assert_eq!(e.code, "blendUnsupported");
    let c = ck_ops::blend::build::chamfer(&rec, &[edge(&rec)], 1.0, "F9", 1e-6).unwrap();
    assert!((vol(&c) - (1200.0 - 0.5 * 20.0)).abs() < 1e-9 * 1200.0, "{}", vol(&c));
    assert!(ck_check::validate(&c, 1e-6).is_empty());
    let f = ck_ops::blend::build::fillet(&rec, &[edge(&rec)], 1.0, "F9", 1e-6).unwrap();
    let want = 1200.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 20.0;
    assert!((vol(&f) - want).abs() < 1e-9 * 1200.0, "{} vs {want}", vol(&f));
    assert!(ck_check::validate(&f, 1e-6).is_empty());
    // shell with the top open, 1 mm wall: 20 x 10 x 6 minus the 18 x 8 x 5 cavity
    let s = ck_ops::shell::shell(&rec, &[top_face(&rec)], 1.0, false, 1e-6).unwrap_or_else(|e| panic!("shell: {} {:?}", e.code, e.detail));
    assert!((vol(&s) - (1200.0 - 18.0 * 8.0 * 5.0)).abs() < 1e-9 * 1200.0, "{}", vol(&s));
    assert!(ck_check::validate(&s, 1e-6).is_empty());
}
