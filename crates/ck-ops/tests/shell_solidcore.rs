//! Shell edge cases found by replaying solidcore's bug repros: walls that touch, offset
//! surfaces that cross by a hair, a spherical dome.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_ops::shell::shell;
use ck_topo::body::Body;
use ck_topo::body::FaceId;
use ck_topo::mass::mass_props;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use ck_geom::surface::Surface;
use std::f64::consts::PI;

fn xy() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }
}
fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}
fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> ProfileLoop {
    let p = [v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y1)];
    ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("r{}", i + 1), p[i], p[(i + 1) % 4])).collect())
}
fn hole(tag: &str, cx: f64, cy: f64, r: f64) -> ProfileLoop {
    ProfileLoop::new(vec![ProfileSeg::circle(tag, v(cx, cy), r)])
}
/// A box with through holes, the holes cut by boolean as the cut feature does.
fn plate(w: f64, d: f64, h: f64, holes: Vec<ProfileLoop>) -> Body {
    let prof = |l: ProfileLoop| Profile::new(xy(), vec![ProfileRegion { outer: l, holes: vec![] }]);
    let mut b = extrude("E1", &prof(rect(0.0, 0.0, w, d)), ExtrudeOpts { s0: 0.0, s1: h, dir: None, draft: 0.0 }).unwrap();
    for (i, l) in holes.into_iter().enumerate() {
        let tool = extrude(&format!("H{i}"), &prof(l), ExtrudeOpts { s0: -1.0, s1: h + 1.0, dir: None, draft: 0.0 }).unwrap();
        b = boolean(&b, &tool, Op::Cut, 1e-6).unwrap().body;
    }
    b
}
/// The planar face whose outward normal is `n`.
fn face_facing(b: &Body, n: Vec3) -> FaceId {
    b.face_ids()
        .find(|&f| match &b.face(f).surface {
            Surface::Plane { frame } => {
                let o = if b.face(f).sense { frame.z } else { -frame.z };
                o.dot(n) > 1.0 - 1e-9
            }
            _ => false,
        })
        .expect("face")
}
fn vol(b: &Body) -> f64 {
    mass_props(b).volume
}
fn report(what: &str, r: &Result<Body, ck_ops::shell::ShellError>) {
    match r {
        Ok(b) => eprintln!("DIAG {what}: ok vol {:.6} faces {} issues {:?}", vol(b), b.num_faces(), ck_check::validate(b, 1e-6).iter().take(3).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>()),
        Err(e) => eprintln!("DIAG {what}: err {} {:?} {:?}", e.code, e.detail, e.limit),
    }
}

fn reduced_1457() -> (Body, FaceId) {
    let b = plate(60.0, 40.0, 10.0, vec![hole("h", 22.5, 7.0, 3.0)]);
    let open = face_facing(&b, -Vec3::Z);
    (b, open)
}

/// A hole's offset (r 3 + 2) is tangent to the front wall's offset (y = 2): the cavity would be
/// pinched to zero width along a line. Refused, with the thickness that works; a thinner wall builds.
#[test]
fn walls_that_touch_are_refused() {
    let (b, open) = reduced_1457();
    let e = shell(&b, &[open], 2.0, false, 1e-6).err().expect("pinched cavity must be refused");
    assert_eq!(e.code, "shellTooThick", "{e:?}");
    let d = e.detail.clone().unwrap_or_default();
    assert!(d.contains("walls touch") && d.contains("use at most"), "{d}");
    let max = e.limit.as_ref().expect("limit").max_thickness;
    assert!(max > 0.5 && max < 2.0, "{max}");
    for t in [1.5, max] {
        let ok = shell(&b, &[open], t, false, 1e-6).unwrap_or_else(|e| panic!("t={t}: {e:?}"));
        assert!(ck_check::validate(&ok, 1e-6).is_empty(), "t={t}");
    }
    let ok = shell(&b, &[open], 1.5, false, 1e-6).unwrap();
    assert!((vol(&ok) - 6331.503297).abs() < 1e-3, "{}", vol(&ok));
}

/// A hole whose offset crosses the side wall's offset by 0.01 mm: the offset cylinder and the wall
/// plane overlap in a sliver. Refused (not a body with overlapping faces), the suggested wall builds.
#[test]
fn offsets_that_cross_are_refused() {
    let b = plate(40.0, 40.0, 10.0, vec![hole("h", 4.99, 20.0, 2.0)]);
    let open = face_facing(&b, Vec3::Z);
    let e = shell(&b, &[open], 1.5, false, 1e-6).err().expect("crossing offsets must be refused");
    assert_eq!(e.code, "shellTooThick", "{e:?}");
    assert!(e.detail.clone().unwrap_or_default().contains("walls touch"), "{e:?}");
    let max = e.limit.as_ref().expect("limit").max_thickness;
    assert!(max > 0.5 && max < 1.49, "{max}");
    for t in [1.4, max] {
        let ok = shell(&b, &[open], t, false, 1e-6).unwrap_or_else(|e| panic!("t={t}: {e:?}"));
        assert!(ck_check::validate(&ok, 1e-6).is_empty(), "t={t}");
    }
}
/// A hemisphere of radius 10 (a quarter disc revolved), shelled open at the flat face: the offset
/// sphere is concentric. The end of its meridian arc slides along the sphere when the flat face
/// moves, so the arc's range has to follow the moved vertex.
#[test]
fn hemisphere_shells_open_at_the_flat_face() {
    let prof = Profile::new(
        xy(),
        vec![ProfileRegion {
            outer: ProfileLoop::new(vec![
                ProfileSeg::line("a", v(0.0, 0.0), v(10.0, 0.0)),
                ProfileSeg::arc("b", v(0.0, 0.0), v(10.0, 0.0), v(0.0, 10.0)),
                ProfileSeg::line("c", v(0.0, 10.0), v(0.0, 0.0)),
            ]),
            holes: vec![],
        }],
    );
    let h = revolve("R1", &prof, Vec3::ZERO, Vec3::Y, 0.0, 2.0 * PI).unwrap();
    let open = face_facing(&h, -Vec3::Y);
    for t in [2.0, 1.0, 0.5, 0.1] {
        let b = shell(&h, &[open], t, false, 1e-6).unwrap_or_else(|e| panic!("t={t}: {e:?}"));
        let want = 2.0 * PI / 3.0 * (1000.0 - (10.0 - t).powi(3));
        assert!((vol(&b) - want).abs() < 1e-5 * want, "t={t}: {} vs {want}", vol(&b));
        assert!(ck_check::validate(&b, 1e-6).is_empty(), "t={t}");
        assert_eq!(b.num_faces(), 3, "t={t}");
    }
}

/// The full #1457 input: a 0.2 mm web between a notch corner and the hole. A wall of 2 (or 1.5, 2.5)
/// cannot be built by offsetting (the web would have to become solid); the refusal names a thickness
/// that does build and is a correct body, and no thickness that builds crosses the web.
#[test]
fn thin_web_limit_is_a_working_thickness() {
    let mut b = plate(60.0, 40.0, 10.0, vec![hole("h", 22.5, 7.0, 3.0)]);
    let tool = extrude("N", &Profile::new(xy(), vec![ProfileRegion { outer: rect(25.0, -5.0, 35.0, 5.0), holes: vec![] }]), ExtrudeOpts { s0: -1.0, s1: 11.0, dir: None, draft: 0.0 }).unwrap();
    b = boolean(&b, &tool, Op::Cut, 1e-6).unwrap().body;
    let open = face_facing(&b, -Vec3::Z);
    for t in [1.5, 2.0, 2.5] {
        let e = shell(&b, &[open], t, false, 1e-6).err().unwrap_or_else(|| panic!("t={t} must be refused"));
        let max = e.limit.as_ref().unwrap_or_else(|| panic!("t={t}: no limit in {e:?}")).max_thickness;
        // the web is 0.2 mm: no wall above 0.1 can fit
        assert!(max > 0.0 && max <= 0.1, "t={t}: suggested {max}");
        let ok = shell(&b, &[open], max, false, 1e-6).unwrap_or_else(|e| panic!("suggested {max}: {e:?}"));
        assert!(ck_check::validate(&ok, 1e-6).is_empty());
    }
    assert!(shell(&b, &[open], 0.15, false, 1e-6).is_err(), "0.15 mm wall crosses a 0.2 mm web");
}

/// A half ball made by revolving the half disc 180 degrees: its flat side is two coplanar faces (the
/// start and end caps) that share the axis edge. Opening both (solidcore #1735) must either build the
/// half ball shell or refuse saying why and what to do; it was refused as a generic cavity failure that
/// blamed nothing in particular.
#[test]
fn half_ball_open_at_two_coplanar_caps() {
    let prof = Profile::new(
        xy(),
        vec![ProfileRegion {
            outer: ProfileLoop::new(vec![
                ProfileSeg::arc("a", v(0.0, 0.0), v(0.0, -10.0), v(0.0, 10.0)),
                ProfileSeg::line("b", v(0.0, 10.0), v(0.0, -10.0)),
            ]),
            holes: vec![],
        }],
    );
    let h = revolve("R1", &prof, Vec3::ZERO, Vec3::Y, 0.0, PI).unwrap();
    let open: Vec<FaceId> = h.face_ids().filter(|&f| matches!(h.face(f).surface, Surface::Plane { .. })).collect();
    assert_eq!(open.len(), 2);
    for t in [1.0, 2.0] {
        match shell(&h, &open, t, false, 1e-6) {
            Ok(b) => {
                let want = 2.0 * PI / 3.0 * (1000.0 - (10.0 - t).powi(3));
                assert!((vol(&b) - want).abs() < 1e-6 * want, "t={t}: {} vs {want}", vol(&b));
                assert!(ck_check::validate(&b, 1e-6).is_empty(), "t={t}: {:?}", ck_check::validate(&b, 1e-6));
            }
            Err(e) => {
                assert_eq!(e.code, "shellUnsupported", "t={t}: {e:?}");
                let d = e.detail.unwrap_or_default();
                assert!(d.contains("one plane") && d.contains("full turn"), "{d}");
            }
        }
    }
}

