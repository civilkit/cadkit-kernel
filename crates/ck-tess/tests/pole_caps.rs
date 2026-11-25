//! Faces whose boundary runs through a pole must tessellate and cover the right area, built by hand with the loop topology
//! the Fusion builder (`ck-asm`) makes, so the cases need no corpus file:
//!   - a cap on a revolved B-spline surface that closes in v without a periodic flag and has a pole at one end of u: the loop is the rim
//!     circle, then the meridian to the pole and the same meridian back, the second walk placed one period over by its pcurve;
//!   - a sphere face whose meridian edge runs through a pole (its pcurve continues past the pole, v beyond +-pi/2).
use ck_geom::curve::Curve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::pcurve::pcurve_of;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_tess::mesh::tessellate;
use ck_topo::body::Body;
use ck_topo::body::FaceName;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// The disc of radius `r` as a rational B-spline: u runs across the radius (a straight line from `u = -1` to `u = 0`), v once round
/// the circle (closed, no periodic flag). `pole_at_hi`: the centre is the u = 0 row (else the u = -1 one).
fn disc_surface(r: f64, pole_at_hi: bool) -> BSplineSurface {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let ring: Vec<(f64, f64, f64)> = vec![(1.0, 0.0, 1.0), (1.0, 1.0, s), (0.0, 1.0, 1.0), (-1.0, 1.0, s), (-1.0, 0.0, 1.0), (-1.0, -1.0, s), (0.0, -1.0, 1.0), (1.0, -1.0, s), (1.0, 0.0, 1.0)];
    let mut ctrl = vec![];
    let mut ws = vec![];
    for i in 0..2 {
        let at_pole = (i == 1) == pole_at_hi;
        for &(x, y, w) in &ring {
            ctrl.push(if at_pole { Vec3::ZERO } else { Vec3::new(r * x, r * y, 0.0) });
            ws.push(w);
        }
    }
    BSplineSurface {
        deg_u: 1,
        deg_v: 2,
        knots_u: vec![-1.0, -1.0, 0.0, 0.0],
        knots_v: vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0].iter().map(|k| k * TAU).collect(),
        nu: 2,
        nv: 9,
        ctrl,
        weights: Some(ws),
    }
}

fn mesh_area(b: &Body) -> (f64, usize) {
    let t = tessellate(b);
    let area: f64 = t
        .mesh
        .positions
        .chunks_exact(9)
        .map(|p| {
            let q: Vec<Vec3> = (0..3).map(|k| Vec3::new(p[3 * k] as f64, p[3 * k + 1] as f64, p[3 * k + 2] as f64)).collect();
            0.5 * (q[1] - q[0]).cross(q[2] - q[0]).len()
        })
        .sum();
    assert!(t.mesh.failed.is_empty(), "faces that did not tessellate: {:?}", t.mesh.failed_why);
    (area, t.mesh.positions.len() / 9)
}

/// The cap as `ck-asm` builds it: the rim, the meridian to the pole, the meridian back with its pcurve one period over.
fn disc_cap(pole_at_hi: bool) -> Body {
    let r = 5.0;
    let bs = disc_surface(r, pole_at_hi);
    let surface = Surface::BSpline(Box::new(bs.clone()));
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let rim_u = if pole_at_hi { -1.0 } else { 0.0 };
    let pole_u = if pole_at_hi { 0.0 } else { -1.0 };
    let va = b.add_vertex(Vec3::new(r, 0.0, 0.0));
    let pole = b.add_vertex(Vec3::ZERO);
    let rim_curve = Curve::Circle { frame: Frame::identity(), r };
    let rim = b.add_edge(rim_curve.clone(), 0.0, TAU, va, va);
    let rim_pc = pcurve_of(&surface, &rim_curve, 0.0, TAU, 1e-6).unwrap();
    // the meridian, as `iso_v`: parameter = u
    let (t0, t1, ev0, ev1) = if pole_at_hi { (rim_u, pole_u, va, pole) } else { (pole_u, rim_u, pole, va) };
    let seam = b.add_edge(Curve::BSpline(bs.iso_v(0.0)), t0, t1, ev0, ev1);
    let seam_pc = Curve2Line::at_v(0.0);
    let f = b.add_face(shell, surface, true, FaceName::other("test", "cap"));
    let lp = b.add_loop(f);
    // the loop runs from the rim's end (v = 2 pi) back to v = 0, out along the meridian and back to the pole
    b.add_coedge(lp, rim, true, Some(rim_pc));
    let pole_first = !pole_at_hi;
    b.add_coedge(lp, seam, pole_first, Some(seam_pc.clone()));
    // the second walk leaves the pole on the sheet one period over: it ends at the loop's start, v = 2 pi
    b.add_coedge(lp, seam, !pole_first, Some(Curve2Line::at_v(TAU)));
    b
}

struct Curve2Line;
impl Curve2Line {
    fn at_v(v: f64) -> ck_geom::curve2::Curve2 {
        ck_geom::curve2::Curve2::Line { o: Vec2::new(0.0, v), d: Vec2::new(1.0, 0.0) }
    }
}

#[test]
fn a_revolved_bspline_cap_closed_through_its_pole_tessellates() {
    for pole_at_hi in [true, false] {
        let b = disc_cap(pole_at_hi);
        let (area, tris) = mesh_area(&b);
        let want = PI * 25.0;
        assert!(tris > 20, "pole_at_hi {pole_at_hi}: {tris} triangles");
        assert!((area / want - 1.0).abs() < 0.02, "pole_at_hi {pole_at_hi}: the cap's mesh has area {area}, the disc {want}");
    }
}

/// The part of a sphere bounded by half the equator and one half great circle through the south pole (a single meridian edge from the
/// equator at u = pi to the equator at u = 0, its pcurve u = pi with v running from 0 to -pi, past the pole): a quarter of the sphere.
fn quarter_sphere_area(sense: bool) -> f64 {
    let r = 4.0;
    let surface = Surface::Sphere { frame: Frame::identity(), r };
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let e0 = b.add_vertex(Vec3::new(r, 0.0, 0.0));
    let e1 = b.add_vertex(Vec3::new(-r, 0.0, 0.0));
    // half the equator, counter-clockwise seen from +z
    let eq = b.add_edge(Curve::Circle { frame: Frame::identity(), r }, 0.0, PI, e0, e1);
    // the great circle in the plane y = 0 that passes through the south pole, from (-r, 0, 0) to (r, 0, 0)
    let gx = Vec3::new(-1.0, 0.0, 0.0);
    let gy = Vec3::new(0.0, 0.0, -1.0);
    let gc = b.add_edge(Curve::Circle { frame: Frame { o: Vec3::ZERO, x: gx, y: gy, z: gx.cross(gy) }, r }, 0.0, PI, e1, e0);
    let f = b.add_face(shell, surface.clone(), sense, FaceName::other("test", "quarter"));
    let lp = b.add_loop(f);
    let eq_pc = pcurve_of(&surface, &b.edge(eq).curve.clone(), 0.0, PI, 1e-9).unwrap();
    let gc_pc = pcurve_of(&surface, &b.edge(gc).curve.clone(), 0.0, PI, 1e-9).unwrap();
    // the meridian's pcurve runs past the pole: v beyond +-pi/2 at one end
    assert!(gc_pc.eval(PI).y.abs() > FRAC_PI_2 + 0.1 || gc_pc.eval(0.0).y.abs() > FRAC_PI_2 + 0.1, "the case needs a pcurve that runs past the pole: {:?} .. {:?}", gc_pc.eval(0.0), gc_pc.eval(PI));
    b.add_coedge(lp, eq, false, Some(eq_pc));
    b.add_coedge(lp, gc, false, Some(gc_pc));
    mesh_area(&b).0
}

#[test]
fn a_sphere_face_bounded_by_a_meridian_through_its_pole_tessellates() {
    let quarter = PI * 4.0 * 4.0;
    for sense in [true, false] {
        let a = quarter_sphere_area(sense);
        assert!((a / quarter - 1.0).abs() < 0.03, "sense {sense}: the mesh covers {a}, a quarter of the sphere is {quarter}");
    }
}
