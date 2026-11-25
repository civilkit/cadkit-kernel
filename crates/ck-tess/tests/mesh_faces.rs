//! Faces of periodic surfaces the Fusion builder (`ck-asm`) makes, built by hand with the loop topology and pcurve sheets it
//! gives them, so the cases need no corpus file. Each must tessellate closed and cover its exact area.
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::pcurve::pcurve_of;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_tess::mesh::tessellate;
use ck_topo::body::{Body, FaceName};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn mesh_area(b: &Body) -> f64 {
    mesh_area_with(b, None)
}

/// The mesh area at the body's own tolerance, or at an explicit one (a small part inside a big assembly has its chord).
fn mesh_area_with(b: &Body, tol: Option<ck_tess::curvature::Tol>) -> f64 {
    let t = match tol {
        Some(t) => ck_tess::mesh::tessellate_with(b, &t),
        None => tessellate(b),
    };
    assert!(t.mesh.failed.is_empty(), "faces that did not tessellate: {:?}", t.mesh.failed_why);
    t.mesh
        .positions
        .chunks_exact(9)
        .map(|p| {
            let q: Vec<Vec3> = (0..3).map(|k| Vec3::new(p[3 * k] as f64, p[3 * k + 1] as f64, p[3 * k + 2] as f64)).collect();
            0.5 * (q[1] - q[0]).cross(q[2] - q[0]).len()
        })
        .sum()
}

/// Every mesh edge of a face of a periodic surface is used by two triangles, except along the cut the strip is opened at, where the
/// two sides of the cut are one curve in space: matched by position, no edge is left over.
fn assert_closed_but_at_the_cut(b: &Body, what: &str) {
    use std::collections::HashMap;
    let t = tessellate(b);
    let q = |p: &[f32]| ((p[0] as f64 * 1e3).round() as i64, (p[1] as f64 * 1e3).round() as i64, (p[2] as f64 * 1e3).round() as i64);
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), i32> = HashMap::new();
    for tri in t.mesh.positions.chunks_exact(9) {
        let v = [q(&tri[0..3]), q(&tri[3..6]), q(&tri[6..9])];
        for k in 0..3 {
            let (a, c) = (v[k], v[(k + 1) % 3]);
            if a == c {
                continue;
            }
            *edges.entry(if a < c { (a, c) } else { (c, a) }).or_insert(0) += 1;
        }
    }
    // the face is a band or a wedge, not a closed shell: its boundary edges are used once, the rest twice
    let bad = edges.values().filter(|&&n| n > 2).count();
    assert_eq!(bad, 0, "{what}: {bad} mesh edges used by more than two triangles");
}

fn radial(a: f64) -> Vec3 {
    Vec3::new(a.cos(), a.sin(), 0.0)
}

fn line2(o: (f64, f64), d: (f64, f64)) -> Curve2 {
    Curve2::Line { o: Vec2::new(o.0, o.1), d: Vec2::new(d.0, d.1) }
}

/// A sphere lune from the equator to the north pole between azimuths `a0 < a1`, the way Fusion writes it: both meridian edges end at
/// the pole vertex; the first one's pcurve starts on the far side of the sphere (v = pi, running past the pole), the second one's
/// sheet is a period over (u + 2 pi).
fn lune(a0: f64, a1: f64, sense: bool) -> Body {
    let r = 0.5;
    let surface = Surface::Sphere { frame: Frame::identity(), r };
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let v0 = b.add_vertex(radial(a0) * r);
    let v1 = b.add_vertex(radial(a1) * r);
    let vp = b.add_vertex(Vec3::new(0.0, 0.0, r));
    let w = a1 - a0;
    // equator from a1 down to a0 (t = a1 - u)
    let eq = b.add_edge(Curve::Circle { frame: Frame { o: Vec3::ZERO, x: radial(a1), y: radial(a1 + FRAC_PI_2) * -1.0, z: Vec3::new(0.0, 0.0, 1.0) }, r }, 0.0, w, v1, v0);
    // meridian at a1, from the equator up to the pole, with the far-side pcurve (u + pi, v = pi - t)
    let m1 = b.add_edge(Curve::Circle { frame: Frame { o: Vec3::ZERO, x: radial(a1), y: Vec3::new(0.0, 0.0, 1.0), z: radial(a1).cross(Vec3::new(0.0, 0.0, 1.0)) }, r }, 0.0, FRAC_PI_2, v1, vp);
    // meridian at a0, from the equator up to the pole
    let m0 = b.add_edge(Curve::Circle { frame: Frame { o: Vec3::ZERO, x: radial(a0), y: Vec3::new(0.0, 0.0, 1.0), z: radial(a0).cross(Vec3::new(0.0, 0.0, 1.0)) }, r }, 0.0, FRAC_PI_2, v0, vp);
    let f = b.add_face(shell, surface, sense, FaceName::other("test", "lune"));
    let lp = b.add_loop(f);
    // counter-clockwise in (u, v) when the face's sense is true; the loop turns about the outward normal, so clockwise when it is not
    let walks = [(eq, true, line2((a1, 0.0), (-1.0, 0.0))), (m1, false, line2((a1 + PI, PI), (0.0, -1.0))), (m0, true, line2((a0 + TAU, 0.0), (0.0, 1.0)))];
    if sense {
        for (e, rev, pc) in walks {
            b.add_coedge(lp, e, rev, Some(pc));
        }
    } else {
        for (e, rev, pc) in walks.into_iter().rev() {
            b.add_coedge(lp, e, !rev, Some(pc));
        }
    }
    b
}

#[test]
fn a_sphere_lune_with_both_meridians_through_the_pole_tessellates() {
    for (a0, a1) in [(0.3, 1.9), (3.159, 4.712), (-1.0, 0.5), (5.0, 7.0)] {
        for sense in [true, false] {
            let b = lune(a0, a1, sense);
            let a = mesh_area(&b);
            let want = 0.25 * (a1 - a0);
            assert!((a / want - 1.0).abs() < 0.03, "lune {a0}..{a1} sense {sense}: mesh {a}, lune {want}");
        }
    }
}

/// A torus band that goes once round the tube axis (u over a whole period, from `u0`) and covers v from `v0` to `v1`, bounded by
/// the two rims (each split in two arcs) and the seam, which both sides of the face's loop walk (as Fusion writes a closed band).
fn torus_band(u0: f64, v0: f64, v1: f64, sense: bool, big: f64, small: f64) -> (Body, f64) {
    let surface = Surface::Torus { frame: Frame::identity(), major: big, minor: small };
    let at = |u: f64, v: f64| radial(u) * (big + small * v.cos()) + Vec3::new(0.0, 0.0, small * v.sin());
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let rim = |v: f64| Curve::Circle { frame: Frame { o: Vec3::new(0.0, 0.0, small * v.sin()), x: Vec3::new(1.0, 0.0, 0.0), y: Vec3::new(0.0, 1.0, 0.0), z: Vec3::new(0.0, 0.0, 1.0) }, r: big + small * v.cos() };
    let (um, ue) = (u0 + PI, u0 + TAU);
    let (p0, pm, p1, q0, qm, q1) = (b.add_vertex(at(u0, v0)), b.add_vertex(at(um, v0)), b.add_vertex(at(ue, v0)), b.add_vertex(at(u0, v1)), b.add_vertex(at(um, v1)), b.add_vertex(at(ue, v1)));
    let _ = (p1, q1);
    let (ba, bb) = (b.add_edge(rim(v0), u0, um, p0, pm), b.add_edge(rim(v0), um, ue, pm, p0));
    let (ta, tb) = (b.add_edge(rim(v1), u0, um, q0, qm), b.add_edge(rim(v1), um, ue, qm, q0));
    let m = Curve::Circle { frame: Frame { o: radial(u0) * big, x: radial(u0), y: Vec3::new(0.0, 0.0, 1.0), z: radial(u0).cross(Vec3::new(0.0, 0.0, 1.0)) }, r: small };
    let seam = b.add_edge(m, v0, v1, p0, q0);
    let f = b.add_face(shell, surface, sense, FaceName::other("test", "band"));
    let lp = b.add_loop(f);
    let h = |v: f64| line2((0.0, v), (1.0, 0.0));
    let up = |u: f64| line2((u, 0.0), (0.0, 1.0));
    // counter-clockwise in (u, v): the bottom rim, the seam up on the right (a period over), the top rim back, the seam down on the left
    let walks = vec![(ba, false, h(v0)), (bb, false, h(v0)), (seam, false, up(ue)), (tb, true, h(v1)), (ta, true, h(v1)), (seam, true, up(u0))];
    if sense {
        for (e, rev, pc) in walks {
            b.add_coedge(lp, e, rev, Some(pc));
        }
    } else {
        for (e, rev, pc) in walks.into_iter().rev() {
            b.add_coedge(lp, e, !rev, Some(pc));
        }
    }
    // exact area of the band: R * 2 pi * r (v1 - v0) + r^2 2 pi (sin v1 - sin v0)
    let area = TAU * (big * small * (v1 - v0) + small * small * (v1.sin() - v0.sin()));
    (b, area)
}

#[test]
fn a_torus_band_once_round_with_the_seam_on_both_sides_tessellates() {
    for (u0, v0, v1) in [(PI, PI, 1.5 * PI), (0.0, 0.0, FRAC_PI_2), (1.0, 4.0, 5.5), (PI, 1.5 * PI, 2.0 * PI)] {
        for sense in [true, false] {
            let (b, want) = torus_band(u0, v0, v1, sense, 10.0, 3.0);
            let a = mesh_area(&b);
            assert!((a / want - 1.0).abs() < 0.05, "band {u0} {v0}..{v1} sense {sense}: mesh {a}, band {want}");
        }
    }
}

/// A latitude / longitude patch of a sphere, `u0 < u1`, `v0 < v1`, whose meridian at `u0` has the pcurve Fusion writes for a
/// great circle that runs the other way round the sphere: `(u0 - pi, -pi - v)`, v beyond the pole.
fn sphere_patch(r: f64, (u0, u1): (f64, f64), (v0, v1): (f64, f64), far_side: bool, sense: bool) -> (Body, f64) {
    let surface = Surface::Sphere { frame: Frame::identity(), r };
    let at = |u: f64, v: f64| radial(u) * (r * v.cos()) + Vec3::new(0.0, 0.0, r * v.sin());
    let z = Vec3::new(0.0, 0.0, 1.0);
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let (p00, p10, p11, p01) = (b.add_vertex(at(u0, v0)), b.add_vertex(at(u1, v0)), b.add_vertex(at(u1, v1)), b.add_vertex(at(u0, v1)));
    let lat = |v: f64| Curve::Circle { frame: Frame { o: Vec3::new(0.0, 0.0, r * v.sin()), x: Vec3::new(1.0, 0.0, 0.0), y: Vec3::new(0.0, 1.0, 0.0), z }, r: r * v.cos() };
    let mer = |u: f64| Curve::Circle { frame: Frame { o: Vec3::ZERO, x: radial(u), y: z, z: radial(u).cross(z) }, r };
    let bottom = b.add_edge(lat(v0), u0, u1, p00, p10);
    let right = b.add_edge(mer(u1), v0, v1, p10, p11);
    let top = b.add_edge(lat(v1), u0, u1, p01, p11);
    let left = b.add_edge(mer(u0), v0, v1, p00, p01);
    let f = b.add_face(shell, surface, sense, FaceName::other("test", "patch"));
    let lp = b.add_loop(f);
    let left_pc = if far_side { line2((u0 - PI, -PI), (0.0, -1.0)) } else { line2((u0, 0.0), (0.0, 1.0)) };
    let walks = vec![(bottom, false, line2((0.0, v0), (1.0, 0.0))), (right, false, line2((u1, 0.0), (0.0, 1.0))), (top, true, line2((0.0, v1), (1.0, 0.0))), (left, true, left_pc)];
    if sense {
        for (e, rev, pc) in walks {
            b.add_coedge(lp, e, rev, Some(pc));
        }
    } else {
        for (e, rev, pc) in walks.into_iter().rev() {
            b.add_coedge(lp, e, !rev, Some(pc));
        }
    }
    (b, r * r * (u1 - u0) * (v1.sin() - v0.sin()))
}

#[test]
fn a_sphere_patch_with_a_far_side_meridian_pcurve_tessellates() {
    for (u, v) in [((1.816, 2.85), (-0.344, 1.22)), ((0.2, 1.0), (0.1, 0.9)), ((-2.0, -1.0), (-1.2, 0.3))] {
        for far in [false, true] {
            for sense in [true, false] {
                let (b, want) = sphere_patch(0.1, u, v, far, sense);
                let a = mesh_area_with(&b, Some(ck_tess::curvature::Tol::new(0.01, 0.3)));
                assert!((a / want - 1.0).abs() < 0.15, "patch {u:?} {v:?} far {far} sense {sense}: mesh {a}, patch {want}");
            }
        }
    }
}

/// A torus wedge: the part of the torus between two whole meridian circles, at `u0 < u1`. Two loops of one closed edge each; the
/// pcurves run once round v (here downwards, as Fusion writes them), the loop at `u1` walked the other way round from the one at `u0`.
fn torus_wedge(u0: f64, u1: f64, sense: bool, big: f64, small: f64, down: bool) -> (Body, f64) {
    let surface = Surface::Torus { frame: Frame::identity(), major: big, minor: small };
    let z = Vec3::new(0.0, 0.0, 1.0);
    // (v = t, or v = -t when the circle's frame is turned the other way round, as Fusion's are)
    let dir = if down { -1.0 } else { 1.0 };
    let mer = |u: f64| Curve::Circle { frame: Frame { o: radial(u) * big, x: radial(u), y: z * dir, z: radial(u).cross(z * dir) }, r: small };
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let f = b.add_face(shell, surface, sense, FaceName::other("test", "wedge"));
    let mut mk = |b: &mut Body, u: f64, rev: bool| {
        let vtx = b.add_vertex(radial(u) * (big + small));
        let e = b.add_edge(mer(u), 0.0, TAU, vtx, vtx);
        let lp = b.add_loop(f);
        let pc = if down { line2((u, 0.0), (0.0, -1.0)) } else { line2((u, 0.0), (0.0, 1.0)) };
        b.add_coedge(lp, e, rev, Some(pc));
    };
    // (the outer loop turns counter-clockwise about the outward normal: up the right side u1 ... the one at u0 the other way)
    mk(&mut b, u1, sense == down);
    mk(&mut b, u0, sense != down);
    (b, big * small * TAU * (u1 - u0) + small * small * 0.0)
}

#[test]
fn a_torus_wedge_between_two_whole_meridian_circles_tessellates() {
    for (u0, u1) in [(0.0, 0.832), (5.451, TAU), (1.0, 4.0), (-0.5, 0.5)] {
        for sense in [true, false] {
            for down in [true, false] {
                let (b, want) = torus_wedge(u0, u1, sense, 10.0, 0.75, down);
                let a = mesh_area(&b);
                assert!((a / want - 1.0).abs() < 0.05, "wedge {u0}..{u1} sense {sense} down {down}: mesh {a}, wedge {want}");
                assert_closed_but_at_the_cut(&b, "wedge");
            }
        }
    }
}

/// A ball of radius 0.5 mm in a part that sets a chord of 0.4 mm: the patch's mesh is a handful of flat triangles that cover a
/// quarter less than the surface, which is the tolerance and not a region of the wrong size.
#[test]
fn a_small_sphere_patch_under_a_coarse_chord_is_not_refused_for_its_size() {
    for sense in [true, false] {
        let (b, want) = sphere_patch(0.5, (4.73, 7.24), (-1.1, 1.2), false, sense);
        let a = mesh_area_with(&b, Some(ck_tess::curvature::Tol::new(0.4, 0.3)));
        assert!(a > 0.5 * want && a < 1.2 * want, "sense {sense}: mesh {a}, patch {want}");
    }
}

/// A rectangle of the torus in (u, v), four edges, no seam: the fillet patches of a rounded corner.
fn torus_patch(u: (f64, f64), v: (f64, f64), sense: bool, big: f64, small: f64) -> (Body, f64) {
    let surface = Surface::Torus { frame: Frame::identity(), major: big, minor: small };
    let at = |u: f64, v: f64| radial(u) * (big + small * v.cos()) + Vec3::new(0.0, 0.0, small * v.sin());
    let z = Vec3::new(0.0, 0.0, 1.0);
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let (p00, p10, p11, p01) = (b.add_vertex(at(u.0, v.0)), b.add_vertex(at(u.1, v.0)), b.add_vertex(at(u.1, v.1)), b.add_vertex(at(u.0, v.1)));
    let rim = |v: f64| Curve::Circle { frame: Frame { o: Vec3::new(0.0, 0.0, small * v.sin()), x: Vec3::new(1.0, 0.0, 0.0), y: Vec3::new(0.0, 1.0, 0.0), z }, r: big + small * v.cos() };
    let mer = |u: f64| Curve::Circle { frame: Frame { o: radial(u) * big, x: radial(u), y: z, z: radial(u).cross(z) }, r: small };
    let bottom = b.add_edge(rim(v.0), u.0, u.1, p00, p10);
    let right = b.add_edge(mer(u.1), v.0, v.1, p10, p11);
    let top = b.add_edge(rim(v.1), u.0, u.1, p01, p11);
    let left = b.add_edge(mer(u.0), v.0, v.1, p00, p01);
    let f = b.add_face(shell, surface, sense, FaceName::other("test", "tpatch"));
    let lp = b.add_loop(f);
    let walks = vec![(bottom, false, line2((0.0, v.0), (1.0, 0.0))), (right, false, line2((u.1, 0.0), (0.0, 1.0))), (top, true, line2((0.0, v.1), (1.0, 0.0))), (left, true, line2((u.0, 0.0), (0.0, 1.0)))];
    if sense {
        for (e, rev, pc) in walks {
            b.add_coedge(lp, e, rev, Some(pc));
        }
    } else {
        for (e, rev, pc) in walks.into_iter().rev() {
            b.add_coedge(lp, e, !rev, Some(pc));
        }
    }
    (b, big * small * (u.1 - u.0) * (v.1 - v.0) + small * small * (u.1 - u.0) * (v.1.sin() - v.0.sin()))
}

#[test]
fn a_torus_rectangle_meshes_closed() {
    for (u, v) in [((-FRAC_PI_2, FRAC_PI_2), (0.0, FRAC_PI_2)), ((0.0, PI), (0.0, FRAC_PI_2)), ((PI, 1.5 * PI), (0.0, FRAC_PI_2)), ((1.0, 2.0), (4.0, 5.0)), ((0.0, PI), (4.712, TAU))] {
        for sense in [true, false] {
            let (b, want) = torus_patch(u, v, sense, 0.6, 0.1);
            let t = tessellate(&b);
            assert!(t.mesh.failed.is_empty(), "failed {:?}", t.mesh.failed_why);
            assert!(t.mesh.notes.is_empty(), "patch {u:?} {v:?} sense {sense}: {:?}", t.mesh.notes);
            let a = mesh_area(&b);
            assert!((a / want - 1.0).abs() < 0.1, "patch {u:?} {v:?} sense {sense}: mesh {a}, patch {want}");
        }
    }
}

/// A barrel as a rational B-spline: u runs up the axis (`z = u`, 0 to 1, the radius bulging from 1 to 1.3 in the middle), v once round
/// (0 to 2 pi, closed, no periodic flag), the way a revolved free-form surface is stored.
fn closed_barrel_spline() -> ck_geom::nurbs::bsurf::BSplineSurface {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let ring: [(f64, f64, f64); 9] = [(1.0, 0.0, 1.0), (1.0, 1.0, s), (0.0, 1.0, 1.0), (-1.0, 1.0, s), (-1.0, 0.0, 1.0), (-1.0, -1.0, s), (0.0, -1.0, 1.0), (1.0, -1.0, s), (1.0, 0.0, 1.0)];
    let (mut ctrl, mut ws) = (vec![], vec![]);
    for (z, r) in [(0.0, 1.0), (0.5, 1.6), (1.0, 1.0)] {
        for &(x, y, w) in &ring {
            ctrl.push(Vec3::new(r * x, r * y, z));
            ws.push(w);
        }
    }
    ck_geom::nurbs::bsurf::BSplineSurface {
        deg_u: 2,
        deg_v: 2,
        knots_u: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        knots_v: vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0].iter().map(|k| k * TAU).collect(),
        nu: 3,
        nv: 9,
        ctrl,
        weights: Some(ws),
    }
}

/// A rectangle of the closed B-spline barrel in the middle of its domain. Its two rims are circle arcs whose parameter starts below
/// zero (the mesher's heuristic for the first circle of an SSI pair), which used to open the strip at `[2 pi, 4 pi]`, outside the
/// spline's domain: the interior points were evaluated on the clamped surface and the mesh did not follow the face.
fn closed_spline_patch(u: (f64, f64), v: (f64, f64), sense: bool) -> (Body, f64) {
    let bs = closed_barrel_spline();
    let surface = Surface::BSpline(Box::new(bs.clone()));
    let z = Vec3::new(0.0, 0.0, 1.0);
    let vm = 0.5 * (v.0 + v.1);
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let (p00, p01, p11, p10) = (b.add_vertex(bs.eval(u.0, v.0)), b.add_vertex(bs.eval(u.0, v.1)), b.add_vertex(bs.eval(u.1, v.1)), b.add_vertex(bs.eval(u.1, v.0)));
    let rim = |uu: f64| {
        let p = bs.eval(uu, vm);
        let a = Vec3::new(p.x, p.y, 0.0);
        Curve::Circle { frame: Frame { o: Vec3::new(0.0, 0.0, p.z), x: a * (1.0 / a.len()), y: z.cross(a * (1.0 / a.len())), z }, r: a.len() }
    };
    let (c0, c1) = (rim(u.0), rim(u.1));
    let (t0, t1) = (v.0 - vm, v.1 - vm);
    let bottom = b.add_edge(c0.clone(), t0, t1, p00, p01);
    let right = b.add_edge(Curve::BSpline(bs.iso_v(v.1)), u.0, u.1, p01, p11);
    let top = b.add_edge(c1.clone(), t0, t1, p10, p11);
    let left = b.add_edge(Curve::BSpline(bs.iso_v(v.0)), u.0, u.1, p00, p10);
    let pc_bottom = pcurve_of(&surface, &c0, t0, t1, 1e-6).unwrap();
    let pc_top = pcurve_of(&surface, &c1, t0, t1, 1e-6).unwrap();
    let f = b.add_face(shell, surface, sense, FaceName::other("test", "spatch"));
    let lp = b.add_loop(f);
    let walks = vec![(bottom, false, pc_bottom), (right, false, line2((0.0, v.1), (1.0, 0.0))), (top, true, pc_top), (left, true, line2((0.0, v.0), (1.0, 0.0)))];
    if sense {
        for (e, rev, pc) in walks {
            b.add_coedge(lp, e, rev, Some(pc));
        }
    } else {
        for (e, rev, pc) in walks.into_iter().rev() {
            b.add_coedge(lp, e, !rev, Some(pc));
        }
    }
    // the exact area: the integral of the surface's area element over the rectangle
    let n = 200;
    let mut area = 0.0;
    for i in 0..n {
        for j in 0..n {
            let (uu, vv) = (u.0 + (u.1 - u.0) * (i as f64 + 0.5) / n as f64, v.0 + (v.1 - v.0) * (j as f64 + 0.5) / n as f64);
            let d = bs.derivs(uu, vv, 1);
            area += d[1][0].cross(d[0][1]).len() * (u.1 - u.0) * (v.1 - v.0) / (n * n) as f64;
        }
    }
    (b, area)
}

#[test]
fn a_patch_in_the_middle_of_a_closed_spline_strip_covers_its_own_area() {
    for (u, v) in [((0.2, 0.8), (1.0, 2.0)), ((0.1, 0.9), (3.0, 4.2)), ((0.3, 0.6), (5.0, 6.0))] {
        for sense in [true, false] {
            let (b, want) = closed_spline_patch(u, v, sense);
            let a = mesh_area(&b);
            assert!((a / want - 1.0).abs() < 0.05, "patch {u:?} {v:?} sense {sense}: mesh {a}, patch {want}");
        }
    }
}
