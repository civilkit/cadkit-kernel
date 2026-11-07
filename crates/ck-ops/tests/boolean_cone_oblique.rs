//! A cylinder cut across a cone with its axis normal to the cone's side (solidcore fixture 10415, "cut from a
//! plane tangent to the cone"). Cone: base radius 10 at y = 0, apex (0, 20, 0). The tool starts on the plane tangent
//! to the cone's side at half height (centre (5, 10, 0)) and runs through the cone, so its start disc touches the
//! cone along a line and its side meets the cone in a general (non-axial, non-parallel) curve.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::PI;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

fn cone() -> Body {
    let xy = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
    let prof = Profile::new(
        xy,
        vec![ProfileRegion {
            outer: ProfileLoop::new(vec![
                ProfileSeg::line("a", v(0.0, 0.0), v(10.0, 0.0)),
                ProfileSeg::line("b", v(10.0, 0.0), v(0.0, 20.0)),
                ProfileSeg::line("c", v(0.0, 20.0), v(0.0, 0.0)),
            ]),
            holes: vec![],
        }],
    );
    revolve("R1", &prof, Vec3::ZERO, Vec3::Y, 0.0, 2.0 * PI).unwrap()
}

/// Cylinder of radius `r` from `start` along `dir` (unit), `len` long, its start disc `back` behind `start`.
fn tool(start: Vec3, dir: Vec3, r: f64, back: f64, len: f64) -> Body {
    let x = dir.cross(Vec3::Z).normalize();
    let y = dir.cross(x);
    let fr = Frame { o: start, x, y, z: dir };
    let prof = Profile::new(fr, vec![ProfileRegion { outer: ProfileLoop::new(vec![ProfileSeg::circle("t", v(0.0, 0.0), r)]), holes: vec![] }]);
    extrude("T1", &prof, ExtrudeOpts { s0: -back, s1: len, dir: None, draft: 0.0 }).unwrap()
}

fn side_normal_inward() -> Vec3 {
    Vec3::new(-20.0, -10.0, 0.0).normalize()
}

/// Turn `p` about the cone's axis by `deg`.
fn about_axis(p: Vec3, deg: f64) -> Vec3 {
    let (s, c) = deg.to_radians().sin_cos();
    Vec3::new(p.x * c + p.z * s, p.y, -p.x * s + p.z * c)
}

fn cut(shift: f64, back: f64, len: f64) -> Result<f64, String> {
    cut_at(0.0, shift, back, len)
}

fn cut_at(deg: f64, shift: f64, back: f64, len: f64) -> Result<f64, String> {
    let n = about_axis(side_normal_inward(), deg);
    let start = about_axis(Vec3::new(5.0, 10.0, 0.0), deg) - n * shift;
    let r = boolean(&cone(), &tool(start, n, 2.0, back, len), Op::Cut, 1e-6).map_err(|e| format!("{e:?}"))?;
    let issues = ck_topo::check::check(&r.body, 1e-6);
    if !issues.is_empty() {
        return Err(format!("invalid: {issues:?}"));
    }
    Ok(ck_topo::mass::mass_props(&r.body).volume)
}

const AZIMUTHS: [f64; 9] = [0.0, 10.0, 20.0, 45.0, 90.0, 180.0, 270.0, 300.0, 350.0];

/// The upstream (quadrature) reference volumes: a tool 1 clear of the side, 9 long (it stops 8 inside): 1995.00843891;
/// a tool starting on the tangent plane, 30 long (through the far side): 1909.355312428. The solid is a body of
/// revolution, so the answer cannot depend on the azimuth the tool is cut from: the cone's seam, the tool's seam and
/// the apex pole all sit at different places relative to the cut in this sweep.
#[test]
fn a_blind_oblique_cylinder_cut_does_not_depend_on_where_it_is_on_the_cone() {
    for deg in AZIMUTHS {
        let v = cut_at(deg, 1.0, 0.0, 9.0).unwrap_or_else(|e| panic!("azimuth {deg}: {e}"));
        assert!((v - 1995.00843891).abs() < 2e-3, "azimuth {deg}: {v}");
    }
}

#[test]
fn an_oblique_cylinder_cut_through_the_cone_does_not_depend_on_where_it_is_on_the_cone() {
    for deg in AZIMUTHS {
        let v = cut_at(deg, 0.0, 0.0, 30.0).unwrap_or_else(|e| panic!("azimuth {deg}: {e}"));
        assert!((v - 1909.355312428).abs() < 2e-2, "azimuth {deg}: {v}");
    }
}

#[test]
fn a_pointed_cone_face_covers_the_whole_turn() {
    use ck_geom::surface::Surface;
    use ck_ops::boolean::classify::{point_in_face, Where};
    let c = cone();
    let f = c.face_ids().find(|&f| matches!(c.face(f).surface, Surface::Cone { .. })).unwrap();
    for deg in (15..360).step_by(15) {
        let a = (deg as f64).to_radians();
        let (y, r) = (10.63, 10.0 - 10.63 / 2.0);
        let p = Vec3::new(r * a.cos(), y, -r * a.sin());
        assert_eq!(point_in_face(&c, f, p, 1e-6), Where::In, "azimuth {deg}");
    }
}
