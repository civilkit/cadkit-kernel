use super::*;
use crate::damage;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::FaceId;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use ck_topo::mass::mass_props;

const TOL: f64 = 1e-5;

pub fn box_body() -> Body {
    let fr = Frame {
        o: Vec3::ZERO,
        x: Vec3::X,
        y: Vec3::new(0.0, 0.0, -1.0),
        z: Vec3::Y,
    };
    let p = [
        Vec2::new(0.0, 0.0),
        Vec2::new(40.0, 0.0),
        Vec2::new(40.0, 20.0),
        Vec2::new(0.0, 20.0),
    ];
    let tags = ["e1", "e2", "e3", "e4"];
    let lp = ProfileLoop::new(
        (0..4)
            .map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4]))
            .collect(),
    );
    let prof = Profile::new(
        fr,
        vec![ProfileRegion {
            outer: lp,
            holes: vec![],
        }],
    );
    extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap()
}

pub fn cyl_body() -> Body {
    let fr = Frame {
        o: Vec3::ZERO,
        x: Vec3::X,
        y: Vec3::new(0.0, 0.0, -1.0),
        z: Vec3::Y,
    };
    let lp = ProfileLoop::new(vec![ProfileSeg::circle("c", Vec2::new(0.0, 0.0), 5.0)]);
    let prof = Profile::new(
        fr,
        vec![ProfileRegion {
            outer: lp,
            holes: vec![],
        }],
    );
    extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap()
}

fn vol(b: &Body) -> f64 {
    mass_props(b).volume
}

fn healed_ok(b: &mut Body, v0: f64) -> HealReport {
    let r = heal(b, TOL);
    assert!(
        r.remaining.is_empty(),
        "remaining {:?} after {:?}",
        r.remaining,
        r.fixed
    );
    assert!(ck_check::validate(b, 1e-6).is_empty());
    assert!(
        (vol(b) - v0).abs() <= 1e-6 * v0.abs(),
        "volume {} vs {v0}",
        vol(b)
    );
    r
}

fn has(r: &HealReport, code: &str) -> bool {
    r.fixed.iter().any(|f| f.code == code)
}

#[test]
fn valid_body_is_a_fixed_point() {
    for mut b in [box_body(), cyl_body()] {
        let before = b.clone();
        let r = heal(&mut b, TOL);
        assert!(r.fixed.is_empty() && r.remaining.is_empty());
        assert_eq!(b, before);
    }
}

#[test]
fn gap_merges_vertices_and_repairs_edges() {
    let mut b = box_body();
    let v0 = vol(&b);
    assert!(damage::detach_faces(&mut b, &[FaceId(1)], 2e-6, false));
    assert!(ck_check::validate(&b, TOL)
        .iter()
        .any(|i| i.code == "SHELL_OPEN"));
    let r = healed_ok(&mut b, v0);
    assert!(has(&r, "VERTEX_MERGED") && has(&r, "EDGE_SEWN"));
}

#[test]
fn gap_on_cylinder_with_seam() {
    for k in 0..3 {
        let mut b = cyl_body();
        let v0 = vol(&b);
        damage::detach_faces(&mut b, &[FaceId(k)], 2e-6, false);
        healed_ok(&mut b, v0);
    }
}

#[test]
fn flipped_face() {
    for i in 0..6 {
        let mut b = box_body();
        let v0 = vol(&b);
        ck_topo::orient::flip_face(&mut b, FaceId(i));
        let r = healed_ok(&mut b, v0);
        assert!(has(&r, "FACE_FLIPPED"));
    }
}

#[test]
fn inside_out_solid() {
    let mut b = box_body();
    let v0 = vol(&b);
    assert!(damage::apply(&mut b, "invert", 0.0));
    assert!(vol(&b) < 0.0);
    let r = healed_ok(&mut b, v0);
    assert!(has(&r, "SOLID_FLIPPED"));
}

#[test]
fn reversed_loop_and_coedge() {
    let mut b = box_body();
    let v0 = vol(&b);
    let l = b.face(FaceId(3)).loops[0];
    ck_topo::orient::reverse_loop(&mut b, l);
    healed_ok(&mut b, v0);
    // a single coedge running against its loop
    let mut b = box_body();
    assert!(damage::apply(&mut b, "reverseCoedge", 0.0));
    let r = healed_ok(&mut b, v0);
    assert!(has(&r, "LOOP_CHAINED"));
}

#[test]
fn open_misordered_wire() {
    let mut b = box_body();
    let v0 = vol(&b);
    assert!(damage::apply(&mut b, "shuffle", 0.0));
    assert!(ck_check::validate(&b, TOL)
        .iter()
        .any(|i| i.code == "LOOP_OPEN"));
    let r = healed_ok(&mut b, v0);
    assert!(has(&r, "LOOP_CHAINED"));
}

#[test]
fn tiny_edge_is_removed() {
    let mut b = box_body();
    let v0 = vol(&b);
    assert!(damage::tiny_edge(&mut b));
    assert!(ck_check::validate(&b, TOL)
        .iter()
        .any(|i| i.code == "EDGE_DEGENERATE"));
    let ne = b.edge_ids().count();
    let r = healed_ok(&mut b, v0);
    assert!(has(&r, "EDGE_REMOVED"));
    assert_eq!(b.edge_ids().count(), ne - 1);
}

#[test]
fn tiny_faces_are_removed() {
    // a slab thinner than the tolerance: its four side faces collapse to
    // zero-area bigons and are dropped, the caps are sewn together
    let fr = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y };
    let p = [Vec2::new(0.0, 0.0), Vec2::new(40.0, 0.0), Vec2::new(40.0, 20.0), Vec2::new(0.0, 20.0)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line("e", p[i], p[(i + 1) % 4])).collect());
    let prof = Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]);
    let mut b = extrude("F2", &prof, ExtrudeOpts::blind(1e-7)).unwrap();
    let r = heal(&mut b, TOL);
    assert_eq!(r.fixed.iter().filter(|f| f.code == "FACE_REMOVED").count(), 4);
    assert_eq!(b.num_faces(), 2);
    assert!(r.remaining.is_empty(), "{:?}", r.remaining);
}

#[test]
fn two_open_shells_sew_into_one() {
    let mut b = box_body();
    let v0 = vol(&b);
    assert!(damage::apply(&mut b, "shells", 2e-6));
    assert_eq!(b.num_lumps(), 2);
    let r = healed_ok(&mut b, v0);
    assert!(has(&r, "SHELL_MERGED") && has(&r, "LUMP_MERGED"));
    assert_eq!(b.num_lumps(), 1);
    assert_eq!(b.shell_ids().count(), 1);
}

#[test]
fn every_damage_on_every_shape() {
    for (name, mk) in [("box", box_body as fn() -> Body), ("cyl", cyl_body)] {
        for k in damage::KINDS {
            let mut b = mk();
            let v0 = vol(&b);
            if !damage::apply(&mut b, k, 2e-6) {
                continue;
            }
            let r = heal(&mut b, TOL);
            assert!(
                r.remaining.is_empty(),
                "{name}/{k}: {:?} {:?}",
                r.remaining,
                r.fixed
            );
            assert!((vol(&b) - v0).abs() <= 1e-6 * v0, "{name}/{k}");
        }
    }
}

// ------------------------------------------------------------ tolerant geometry

fn tolerant(tol: f64) -> HealOpts {
    let mut o = HealOpts::new(tol);
    o.tolerant = true;
    o
}

/// Face with outward normal along +Y (the box extrudes along +Y).
fn top_face(b: &Body) -> FaceId {
    b.face_ids().find(|&f| b.outward_normal(f, 0.0, 0.0).dist(Vec3::Y) < 1e-9 && matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. })).unwrap()
}

fn shift_plane(b: &mut Body, f: FaceId, d: f64) {
    if let ck_geom::surface::Surface::Plane { frame } = &mut b.faces[f.idx()].surface {
        frame.o = frame.o + Vec3::Y * d;
    } else {
        panic!("not a plane");
    }
}

#[test]
fn face_shifted_gets_edge_tolerance() {
    let mut b = box_body();
    let v0 = vol(&b);
    let f = top_face(&b);
    shift_plane(&mut b, f, 3e-3);
    let issues = ck_check::validate(&b, 1e-6);
    assert!(issues.iter().any(|i| i.code == "EDGE_OFF_SURFACE"), "{issues:?}");
    // without the tolerant pass nothing changes
    let mut c = b.clone();
    assert!(!heal_with(&mut c, &HealOpts::new(1e-5)).remaining.is_empty());
    let r = heal_with(&mut b, &tolerant(1e-5));
    assert!(r.remaining.is_empty(), "{:?}", r.remaining);
    assert!(has(&r, "EDGE_TOLERANCE"));
    let used = ck_check::tolerance_used(&b);
    assert!(used >= 3e-3 && used < 4e-3, "{used}");
    assert!((vol(&b) - v0).abs() <= 1e-3 * v0);
    // a tight cap turns the same deviation into an error
    let mut o = tolerant(1e-5);
    o.cap = 1e-3;
    let r = heal_with(&mut c, &o);
    assert!(r.remaining.iter().any(|i| i.code == "EDGE_OFF_SURFACE"));
    assert_eq!(ck_check::tolerance_used(&c), 0.0);
}

#[test]
fn cylinder_edge_off_its_surface() {
    let mut b = cyl_body();
    let e = b.edge_ids().find(|&e| matches!(b.edge(e).curve, ck_geom::curve::Curve::Circle { .. })).unwrap();
    if let ck_geom::curve::Curve::Circle { r, .. } = &mut b.edges[e.idx()].curve {
        *r += 4e-3;
    }
    assert!(!ck_check::validate(&b, 1e-6).is_empty());
    let r = heal_with(&mut b, &tolerant(1e-5));
    assert!(r.remaining.is_empty(), "{:?}", r.remaining);
    assert!(b.edge(e).tol >= 4e-3 && b.edge(e).tol < 5e-3);
}

#[test]
fn gap_of_1e4_is_sewn_with_tolerance() {
    let mut b = box_body();
    let v0 = vol(&b);
    assert!(damage::detach_faces(&mut b, &[FaceId(1)], 1e-4, false));
    assert!(ck_check::validate(&b, 1e-6).iter().any(|i| i.code == "SHELL_OPEN"));
    let r = heal_with(&mut b, &tolerant(1e-3));
    assert!(r.remaining.is_empty(), "{:?} after {:?}", r.remaining, r.fixed);
    assert!(has(&r, "EDGE_SEWN"));
    assert!((vol(&b) - v0).abs() <= 1e-4 * v0);
}

#[test]
fn switches_disable_fixers() {
    let mut b = box_body();
    ck_topo::orient::flip_face(&mut b, FaceId(2));
    let mut o = HealOpts::new(TOL);
    o.fix_orientation = false;
    assert!(!heal_with(&mut b.clone(), &o).remaining.is_empty());
    assert!(heal_with(&mut b, &HealOpts::new(TOL)).remaining.is_empty());
}

#[test]
fn tolerant_pass_is_a_noop_on_valid_bodies() {
    for mut b in [box_body(), cyl_body()] {
        let before = b.clone();
        let r = heal_with(&mut b, &tolerant(1e-4));
        assert!(r.fixed.is_empty() && r.remaining.is_empty());
        assert_eq!(b, before);
    }
}
