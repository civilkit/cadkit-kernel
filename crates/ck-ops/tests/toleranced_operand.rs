//! A body that carries edge / vertex tolerances (set by ck_heal on imports from other CAD systems) must work
//! downstream: tessellation, mass properties and booleans neither panic nor change their results.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use ck_topo::mass::mass_props;

fn boxb(feature: &str, x0: f64, y0: f64, x1: f64, y1: f64, s1: f64) -> Body {
    let v = Vec2::new;
    let p = [v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y1)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    let fr = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y };
    extrude(feature, &Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]), ExtrudeOpts { s0: 0.0, s1, dir: None, draft: 0.0 }).unwrap()
}

fn toleranced(mut b: Body, t: f64) -> Body {
    for e in &mut b.edges {
        e.tol = t;
    }
    for v in &mut b.vertices {
        v.tol = t;
    }
    b
}

fn vol(b: &Body) -> f64 {
    mass_props(b).volume
}

#[test]
fn tolerances_do_not_change_mass_or_mesh() {
    let plain = boxb("A", 0.0, 0.0, 10.0, 10.0, 10.0);
    let tol = toleranced(plain.clone(), 4e-3);
    assert_eq!(vol(&plain), vol(&tol));
    let (mp, mt) = (ck_tess::mesh::tessellate(&plain), ck_tess::mesh::tessellate(&tol));
    assert_eq!(mp.mesh.positions.len(), mt.mesh.positions.len());
    assert!(ck_check::validate(&tol, 1e-6).is_empty());
}

#[test]
fn toleranced_operand_in_boolean() {
    let a = toleranced(boxb("A", 0.0, 0.0, 10.0, 10.0, 10.0), 4e-3);
    let b = boxb("B", 5.0, 5.0, 15.0, 15.0, 10.0);
    // overlap is 5 x 5 x (10 - 0): the boxes extrude 10 along the frame normal from the same plane
    let overlap = 5.0 * 5.0 * 10.0;
    let u = boolean(&a, &b, Op::Union, 1e-6).expect("union");
    assert!((vol(&u.body) - (2000.0 - overlap)).abs() < 1e-6 * 2000.0, "union {}", vol(&u.body));
    let c = boolean(&a, &b, Op::Cut, 1e-6).expect("cut");
    assert!((vol(&c.body) - (1000.0 - overlap)).abs() < 1e-6 * 1000.0, "cut {}", vol(&c.body));
    // as the tool
    let c2 = boolean(&b, &a, Op::Cut, 1e-6).expect("cut");
    assert!((vol(&c2.body) - (1000.0 - overlap)).abs() < 1e-6 * 1000.0, "cut {}", vol(&c2.body));
    assert!(ck_check::validate(&u.body, 1e-6).is_empty());
}
