//! Volume, area, centroid and inertia (the `massProps` message), by the
//! divergence theorem evaluated exactly on the trimmed B-rep faces:
//! each face integral ∫∫_D F(σ, σu×σv) du dv is turned into a boundary
//! integral ∮ G dv (Green's theorem, G = ∫_{u*}^{u} F du) over the face's
//! coedge pcurves, both integrals by composite Gauss-Legendre. Analytic
//! faces with exact pcurves reach machine precision (the oracle's numbers
//! are exact too: cylinder volume = π r² h to the last digit).
//!
//! Wire shape (pinned by oracle probes, `{type:"massProps", tag, bodies:
//! true, densities:[..]}`):
//! `{volume, area, com, principal, axes, firstMoment, inertia, inertiaOrigin
//!   [, bodies:[{volume,area,com[, mass]}]]
//!   [, byMass:{mass, com, principal, axes, firstMoment, inertia, inertiaOrigin}]}` — principal moments
//! ascending (volume-weighted, density 1), `axes[i]` = unit eigenvector of
//! `principal[i]`; `firstMoment` = ∫x dV about the origin (volume × com; mass × com
//! in `byMass`), `inertia` / `inertiaOrigin` = the full 3×3 tensor (rows, world axes)
//! about the centre of mass / the origin, the same integrals the principal moments
//! come from (additive to the oracle shape); `bodies` per lump (wire lump order), `byMass` uses
//! `densities[lump]` (g/mm³, client table Xt). With `densities` every
//! `bodies[]` entry also carries `mass` (= volume × density); without, the
//! reply is the oracle-pinned shape (no `mass`). The request-level rule
//! (`densities` implies `bodies:true`, length must match the lump count)
//! lives in ck-wasm `query_hook`.
//!
//! A body of sheets only (open edges) answers `volume: 0`, `com` = the area centroid, `kind: "surface"`, with the
//! other moments weighted by area (see [`mass_props_json`]).

use crate::body::{Body, FaceId, LumpId};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_json::{Json, Obj};

/// 8-point Gauss-Legendre on [-1, 1].
pub const GL: [(f64, f64); 8] = [
    (-0.960_289_856_497_536_2, 0.101_228_536_290_376_26),
    (-0.796_666_477_413_626_7, 0.222_381_034_453_374_47),
    (-0.525_532_409_916_329, 0.313_706_645_877_887_3),
    (-0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.525_532_409_916_329, 0.313_706_645_877_887_3),
    (0.796_666_477_413_626_7, 0.222_381_034_453_374_47),
    (0.960_289_856_497_536_2, 0.101_228_536_290_376_26),
];

thread_local! {
    static COARSE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Run `f` with the cheap tier of the keyed (post-boolean sanity) integrals: for a non-rational spline
/// face, only as many Gauss points across a span and as many table nodes across v as the polynomial
/// integrand's degree needs (exact to rounding, a third to a half fewer evaluations); every other face
/// and the boundary quadrature are untouched.
pub fn with_coarse<R>(f: impl FnOnce() -> R) -> R {
    let prev = COARSE.with(|c| c.replace(true));
    let r = f();
    COARSE.with(|c| c.set(prev));
    r
}

/// One boundary piece in uv: either an exact pcurve over [ta,tb] or a
/// polyline (fallback when a coedge has no pcurve).
enum Piece<'a> {
    Exact(&'a Curve2, f64, f64),
    Poly(Vec<Vec2>),
}

/// Pcurves for the coedges of `f` that carry none, where the 3D edge has an exact planar image: a line,
/// circle, ellipse or B-spline lying in a planar face is a line, circle, ellipse or B-spline of the plane's
/// own coordinates (the orthographic projection is linear, so a spline keeps its knots and weights and its
/// control points are the projected ones; the parameter is the edge's). The 48-chord polyline used otherwise
/// cuts 0.3 percent off a full circle and as much off a long spline edge. One entry per coedge in loop order.
fn derived_pcurves(b: &Body, f: FaceId) -> Vec<Option<Curve2>> {
    let surf = &b.face(f).surface;
    let periods = surf.periods();
    let mut out = vec![];
    for &l in &b.face(f).loops {
        // end of the previous coedge in the loop (None where unknown)
        let mut prev: Option<Vec2> = None;
        for &c in &b.lp(l).coedges {
            let co = b.coedge(c);
            let (ta, tb) = b.coedge_params(c);
            if let Some(pc) = &co.pcurve {
                out.push(None);
                prev = Some(pc.eval(tb));
                continue;
            }
            let ed = b.edge(co.edge);
            let planar = matches!(surf, Surface::Plane { .. });
            let mut pc = ck_geom::pcurve::exact_pcurve(surf, &ed.curve, 1e-7);
            if pc.is_none() {
                if let (Surface::Plane { frame }, ck_geom::curve::Curve::BSpline(bs)) = (surf, &ed.curve) {
                    let scale = 1.0 + bs.ctrl.iter().map(|p| p.len()).fold(0.0, f64::max);
                    if bs.ctrl.iter().all(|p| (*p - frame.o).dot(frame.z).abs() <= 1e-7 * scale) {
                        let ctrl = bs
                            .ctrl
                            .iter()
                            .map(|p| {
                                let q = frame.to_local(*p);
                                Vec2::new(q.x, q.y)
                            })
                            .collect();
                        pc = Some(Curve2::BSpline(ck_geom::nurbs::bspline::BSplineCurve { degree: bs.degree, knots: bs.knots.clone(), ctrl, weights: bs.weights.clone() }));
                    }
                }
            }
            // a periodic surface: the exact line must start where the previous coedge ended (modulo a period)
            if !planar {
                if let Some(Curve2::Line { o, d }) = &mut pc {
                    match prev {
                        Some(pv) => {
                            let st = *o + *d * ta;
                            if let Some(per) = periods.0 {
                                o.x += per * ((pv.x - st.x) / per).round();
                            }
                            if let Some(per) = periods.1 {
                                o.y += per * ((pv.y - st.y) / per).round();
                            }
                        }
                        None if periods.0.is_some() || periods.1.is_some() => pc = None,
                        None => {}
                    }
                }
            }
            prev = pc.as_ref().map(|q| q.eval(tb));
            out.push(pc);
        }
    }
    out
}

/// Segments for the polyline image of a coedge without a pcurve: 48, doubled until the 3D curve's chord
/// sagitta over every segment is under 1e-5 of the edge's extent (a 48-chord circle is 0.3 percent short of
/// its area; a long spline edge 1e-4). `fine` off (the post-boolean sanity tier, 1e-3 of the volume) keeps 48.
fn poly_segments(b: &Body, c: crate::body::CoedgeId, fine: bool) -> usize {
    let (ta, tb) = b.coedge_params(c);
    if !fine {
        return 48;
    }
    let cv = &b.edge(b.coedge(c).edge).curve;
    let ext = (0..=8).map(|k| cv.eval(ta + (tb - ta) * k as f64 / 8.0)).collect::<Vec<_>>();
    let len: f64 = ext.windows(2).map(|w| w[0].dist(w[1])).sum();
    let tol = 1e-5 * len.max(1e-9);
    // the same edge is asked for again by its other coedge and by every later mass call: memoise on the samples
    let key = {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        for p in &ext {
            p.x.to_bits().hash(&mut h);
            p.y.to_bits().hash(&mut h);
            p.z.to_bits().hash(&mut h);
        }
        h.finish()
    };
    if let Some(n) = POLY_SEGS.with(|m| m.borrow().get(&key).copied()) {
        return n;
    }
    let worst_at = |n: usize| -> f64 {
        let pts: Vec<Vec3> = (0..=n).map(|k| cv.eval(ta + (tb - ta) * k as f64 / n as f64)).collect();
        (0..n).map(|k| cv.eval(ta + (tb - ta) * (k as f64 + 0.5) / n as f64).dist((pts[k] + pts[k + 1]) * 0.5)).fold(0.0, f64::max)
    };
    // the sagitta falls as 1/n^2: from the 48-chord value jump to the level just below the expected answer
    // (the doubling below still decides, so the result is the first passing power-of-two multiple of 48)
    let mut n = 48;
    let w48 = worst_at(48);
    if w48 > tol {
        let want = 48.0 * (w48 / tol).sqrt();
        while n < 384 && (n * 2) as f64 * 1.0 < want * 0.5 {
            n *= 2;
        }
    }
    while n < 384 {
        if worst_at(n) <= tol {
            break;
        }
        n *= 2;
    }
    POLY_SEGS.with(|m| {
        let mut m = m.borrow_mut();
        if m.len() >= 8192 {
            m.clear();
        }
        m.insert(key, n);
    });
    n
}

thread_local! {
    static POLY_SEGS: std::cell::RefCell<std::collections::HashMap<u64, usize>> = std::cell::RefCell::new(Default::default());
}

fn face_pieces<'a>(b: &'a Body, f: FaceId, derived: &'a [Option<Curve2>], fine: bool) -> Vec<Piece<'a>> {
    let mut out = vec![];
    let mut k_co = 0;
    // (the spline's content hash once per face, not once per sample)
    let fp = match &b.face(f).surface {
        Surface::BSpline(bs) if b.face(f).loops.iter().any(|&l| b.lp(l).coedges.iter().any(|&c| b.coedge(c).pcurve.is_none())) => Some(bs.fingerprint()),
        _ => None,
    };
    for &l in &b.face(f).loops {
        let mut prev: Option<Vec2> = None;
        let ring = &b.lp(l).coedges;
        // A sphere loop whose pcurves run past a pole closes in the continued parameters when the meridian is continued
        // (no fold needed: the signed integral over the continuation is the area); one that ends on the far side of the
        // pole from where it started (a meridian that comes in to the pole and leaves it on the opposite one) is folded.
        let fold_ring = matches!(b.face(f).surface, Surface::Sphere { .. }) && {
            let ends = |c: crate::body::CoedgeId, first: bool| {
                let (ta, tb) = b.coedge_params(c);
                b.coedge(c).pcurve.as_ref().map(|q| q.eval(if first { ta } else { tb }))
            };
            let apart = |a: Vec2, z: Vec2| (a.y - z.y).abs() > 1e-9 || ((a.x - z.x) / std::f64::consts::TAU - ((a.x - z.x) / std::f64::consts::TAU).round()).abs() > 1e-9;
            // (or a ring that is cut inside: a meridian that comes in to the pole from the far side, v = pi down to pi/2, starts
            // where the previous coedge ended in space but not in the parameter plane, however the whole ring closes)
            let cut_inside = ring.windows(2).any(|w| matches!((ends(w[0], false), ends(w[1], true)), (Some(z), Some(a)) if apart(a, z)));
            cut_inside
                || match (ring.first().and_then(|&c| ends(c, true)), ring.last().and_then(|&c| ends(c, false))) {
                    (Some(a), Some(z)) => apart(a, z),
                    _ => false,
                }
        };
        let ring_start = out.len();
        for (ci, &c) in ring.iter().enumerate() {
            let (ta, tb) = b.coedge_params(c);
            let pcurve = b.coedge(c).pcurve.as_ref().or(derived[k_co].as_ref());
            k_co += 1;
            // where the next coedge of the ring starts (a fold of this one over a pole closes against it)
            let next_start = {
                let nc = ring[(ci + 1) % ring.len()];
                let (na, _) = b.coedge_params(nc);
                b.coedge(nc).pcurve.as_ref().map(|q| q.eval(na))
            };
            if let (true, Some(pc @ Curve2::Line { .. })) = (fold_ring, pcurve) {
                if let Some(pieces) = sphere_pole_fold(pc, ta, tb, prev, next_start) {
                    prev = pieces.last().and_then(|p| match p {
                        Piece::Poly(v) => v.last().copied(),
                        _ => None,
                    });
                    out.extend(pieces);
                    continue;
                }
            }
            match pcurve {
                Some(pc) => {
                    // A coedge that leaves a pole starts anywhere along the pole line (the pole itself has no edge: a cone's
                    // apex, a B-spline cap's centre): the loop's parameter-plane image jumps there, and the jump is part of the
                    // boundary, a straight piece along the pole line.
                    let start = pc.eval(ta);
                    if let (Some(q), Surface::BSpline(_)) = (prev, &b.face(f).surface) {
                        if q.dist(start) > 1e-9 * (1.0 + q.x.abs() + q.y.abs()) && on_pole_line(&b.face(f).surface, q, start) {
                            out.push(Piece::Poly(vec![q, start]));
                        }
                    }
                    prev = Some(pc.eval(tb));
                    out.push(Piece::Exact(pc, ta, tb));
                }
                None => {
                    let n = poly_segments(b, c, fine);
                    let mut pts = vec![];
                    for k in 0..=n {
                        let t = ta + (tb - ta) * k as f64 / n as f64;
                        let mut uv = b.coedge_uv_fp(c, t, prev, fp);
                        // a B-spline strip that closes on itself in u (the free-form fillet tool and blend of a closed rim): the
                        // closest point at the end of a rail that went all the way round is u = 0 again, not the end of the
                        // domain, which would make the loop close without advancing and integrate to nothing
                        if let (Some(w), Some(q)) = (closed_u_width(&b.face(f).surface), prev) {
                            if uv.x - q.x < -0.5 * w {
                                uv.x += w;
                            } else if uv.x - q.x > 0.5 * w {
                                uv.x -= w;
                            }
                        }
                        // a B-spline that closes on itself in v (the converted sweep of a closed profile): the same unwrapping, so
                        // a rim that goes round it and a seam walked up and back are continuous in the parameter plane
                        if let (Some(w), Some(q)) = (closed_v_width(&b.face(f).surface), prev) {
                            if uv.y - q.y < -0.5 * w {
                                uv.y += w;
                            } else if uv.y - q.y > 0.5 * w {
                                uv.y -= w;
                            }
                        }
                        prev = Some(uv);
                        pts.push(uv);
                    }
                    // (a rail that goes all the way round starts at the closing point, which reads as either end of the
                    // domain: start it at the end its direction leaves from)
                    if let (Some(w), true) = (closed_u_width(&b.face(f).surface), pts.len() > 1) {
                        let ((u0, _), _) = b.face(f).surface.domain();
                        let d = pts[pts.len() - 1].x - pts[0].x;
                        if d < -0.5 * w && pts[0].x < u0 + 0.5 * w {
                            pts.iter_mut().for_each(|q| q.x += w);
                        } else if d > 0.5 * w && pts[0].x > u0 + 0.5 * w {
                            pts.iter_mut().for_each(|q| q.x -= w);
                        }
                    }
                    out.push(Piece::Poly(pts));
                }
            }
        }
            if fold_ring {
            close_folded_ring(&mut out, ring_start);
        }
}
    // A strip that closes on itself in u (see `closed_u_width`) is bounded by two rails that each go once round the u period at
    // a constant v. The integral reads the boundary of the face in the parameter plane, where the two seam sides of that
    // rectangle (the first station and the last, one curve in space) are the part that carries the volume: add them, as
    // the straight pieces joining the end of each rail to the start of the other.
    if b.face(f).loops.len() == 2 && closed_u_width(&b.face(f).surface).is_some() {
        let w = closed_u_width(&b.face(f).surface).unwrap_or(0.0);
        let ends = |p: &Piece| match p {
            Piece::Poly(v) if v.len() > 1 => Some((v[0], v[v.len() - 1])),
            _ => None,
        };
        let rails: Vec<(Vec2, Vec2)> = out.iter().filter_map(ends).collect();
        let is_rail = |(a, z): &(Vec2, Vec2)| (z.x - a.x).abs() > 0.99 * w && (z.y - a.y).abs() < 1e-6 * w;
        if rails.len() == 2 && out.len() == 2 && rails.iter().all(is_rail) && (rails[0].0.y - rails[1].0.y).abs() > 1e-6 * w {
            out.push(Piece::Poly(vec![rails[0].1, rails[1].0]));
            out.push(Piece::Poly(vec![rails[1].1, rails[0].0]));
        }
    }
    out
}

/// A folded ring (see [`sphere_pole_fold`]) that goes through one pole: the run after the pole is put a whole number of periods
/// over so that the ring closes (the pole is where u may jump, and the net u of the ring must be that of a closed loop).
fn close_folded_ring(out: &mut [Piece<'_>], from: usize) {
    use std::f64::consts::TAU;
    let ends = |p: &Piece| -> Option<(Vec2, Vec2)> {
        match p {
            Piece::Poly(v) if v.len() > 1 => Some((v[0], v[v.len() - 1])),
            Piece::Exact(pc, ta, tb) => Some((pc.eval(*ta), pc.eval(*tb))),
            _ => None,
        }
    };
    let n = out.len() - from;
    if n < 2 {
        return;
    }
    let es: Vec<(Vec2, Vec2)> = match out[from..].iter().map(ends).collect::<Option<Vec<_>>>() {
        Some(e) => e,
        None => return,
    };
    let big = |a: Vec2, b: Vec2| (a.y - b.y).abs() > 1e-9 || (a.x - b.x).abs() > 1e-9;
    let jumps: Vec<usize> = (1..n).filter(|&i| big(es[i - 1].1, es[i].0)).collect();
    if jumps.len() != 1 {
        return;
    }
    let (s0, e1) = (es[0].0, es[n - 1].1);
    if (s0.y - e1.y).abs() > 1e-9 {
        return;
    }
    let k = ((s0.x - e1.x) / TAU).round();
    if k == 0.0 || (s0.x - e1.x - k * TAU).abs() > 1e-9 {
        return;
    }
    for i in jumps[0]..n {
        let piece = &mut out[from + i];
        let pts: Vec<Vec2> = match piece {
            Piece::Poly(v) => v.clone(),
            Piece::Exact(pc, ta, tb) => (0..=64).map(|j| pc.eval(*ta + (*tb - *ta) * j as f64 / 64.0)).collect(),
        };
        *piece = Piece::Poly(pts.into_iter().map(|q| Vec2::new(q.x + k * TAU, q.y)).collect());
    }
}

/// A meridian pcurve of a sphere that runs past a pole (v beyond +-pi/2: the evaluation continues onto the opposite meridian) as
/// straight pieces inside the domain: in to the pole, across the pole line, out again on the opposite meridian (u + pi, or
/// u - pi when that one meets the next coedge of the ring). None when the pcurve stays inside the domain.
fn sphere_pole_fold(pc: &Curve2, ta: f64, tb: f64, prev: Option<Vec2>, next_start: Option<Vec2>) -> Option<Vec<Piece<'static>>> {
    use std::f64::consts::{FRAC_PI_2, PI};
    let Curve2::Line { d, .. } = pc else { return None };
    if d.x != 0.0 {
        return None;
    }
    let (a, z) = (pc.eval(ta), pc.eval(tb));
    // (a point on the pole itself belongs to either side)
    let out = |v: f64| v.abs() > FRAC_PI_2 + 1e-9;
    let inside = |v: f64| v.abs() < FRAC_PI_2 - 1e-9;
    let fold = |q: Vec2, du: f64| Vec2::new(q.x + du, q.y.signum() * PI - q.y);
    let pick = |target: Option<Vec2>, base_x: f64| -> f64 {
        match target {
            Some(t) if (base_x - PI - t.x).abs() < (base_x + PI - t.x).abs() => -PI,
            _ => PI,
        }
    };
    let pts: Vec<Vec2> = if inside(a.y) && out(z.y) {
        // in, then out past the pole: [a .. pole] jump [folded end]
        let zf = fold(z, pick(next_start, z.x));
        let vp = FRAC_PI_2 * z.y.signum();
        vec![a, Vec2::new(a.x, vp), Vec2::new(zf.x, vp), zf]
    } else if out(a.y) && inside(z.y) {
        // starts past the pole, ends inside: [folded start] pole jump [.. z]
        let af = fold(a, pick(prev, a.x));
        let vp = FRAC_PI_2 * a.y.signum();
        vec![af, Vec2::new(af.x, vp), Vec2::new(z.x, vp), z]
    } else if (out(a.y) && !inside(z.y)) || (!inside(a.y) && out(z.y)) {
        // wholly past the pole (it may end on the pole): the folded copy
        let du = if out(a.y) { pick(prev, a.x) } else { pick(next_start, z.x) };
        vec![fold(a, du), fold(z, du)]
    } else {
        return None;
    };
    Some(vec![Piece::Poly(pts)])
}

/// Whether the segment `a`..`b` of the parameter plane lies on a pole line of `surf`: its ends and its middle are one point in space.
fn on_pole_line(surf: &Surface, a: Vec2, b: Vec2) -> bool {
    let p = surf.eval(a.x, a.y);
    let tol = 1e-9 * (1.0 + p.len());
    [b, a.lerp(b, 0.5), a.lerp(b, 0.25)].iter().all(|q| surf.eval(q.x, q.y).dist(p) <= tol)
}

/// The width of the v domain of a B-spline surface whose first and last v isolines coincide (it closes on itself in v without
/// being flagged periodic), else `None`.
fn closed_v_width(s: &Surface) -> Option<f64> {
    let Surface::BSpline(_) = s else { return None };
    let ((u0, u1), (v0, v1)) = s.domain();
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()) || s.periods().1.is_some() {
        return None;
    }
    let scale = 1e-6 * (1.0 + s.eval(u0, v0).len());
    [0.0, 0.3, 0.7, 1.0].iter().all(|&k| {
        let u = u0 + (u1 - u0) * k;
        s.eval(u, v0).dist(s.eval(u, v1)) <= scale
    }).then_some(v1 - v0)
}

/// The width of the u domain of a B-spline surface whose first and last u isolines coincide (it closes on itself in u without
/// being flagged periodic), else `None`.
fn closed_u_width(s: &Surface) -> Option<f64> {
    let Surface::BSpline(_) = s else { return None };
    let ((u0, u1), (v0, v1)) = s.domain();
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()) || s.periods().0.is_some() {
        return None;
    }
    let scale = 1e-9 * (1.0 + s.eval(u0, v0).len());
    [0.0, 0.3, 0.7, 1.0].iter().all(|&k| {
        let v = v0 + (v1 - v0) * k;
        s.eval(u0, v).dist(s.eval(u1, v)) <= scale.max(1e-9)
    }).then_some(u1 - u0)
}

/// B-spline faces are piecewise polynomial: cut straight boundary pcurves
/// where they cross an interior knot line, so that every Gauss panel lies
/// in one polynomial span (loft side faces have knots at the stations).
/// Interior knots of the profile curve of a screw patch, in the patch's v range (the profile parameter). A profile that is a
/// fitted spline (a gear flank through its fillet and root: tens of cubic spans) changes its shape from span to span; the
/// integrals along v are cut there, as those of a B-spline face are cut at its knots.
fn screw_profile_knots(surf: &Surface) -> Vec<f64> {
    let Surface::Screw { profile, vrange, .. } = surf else { return vec![] };
    let Curve::BSpline(c) = &**profile else { return vec![] };
    let (lo, hi) = (vrange.0.min(vrange.1), vrange.0.max(vrange.1));
    let mut k: Vec<f64> = c.knots.iter().copied().filter(|&x| x > lo + 1e-12 && x < hi - 1e-12).collect();
    k.dedup();
    k
}

fn split_at_knots<'a>(surf: &Surface, pieces: Vec<Piece<'a>>) -> Vec<Piece<'a>> {
    let inner = |k: &[f64], lo: f64, hi: f64| -> Vec<f64> {
        let mut out: Vec<f64> = k.iter().copied().filter(|&x| x > lo && x < hi).collect();
        out.dedup();
        out
    };
    let (ku, kv) = match surf {
        Surface::BSpline(bs) => {
            let ((u0, u1), (v0, v1)) = bs.domain();
            (inner(&bs.knots_u, u0, u1), inner(&bs.knots_v, v0, v1))
        }
        Surface::Screw { .. } => (vec![], screw_profile_knots(surf)),
        _ => return pieces,
    };
    if ku.is_empty() && kv.is_empty() {
        return pieces;
    }
    let mut out = vec![];
    for p in pieces {
        match p {
            Piece::Exact(pc @ Curve2::Line { o, d }, ta, tb) => {
                let mut ts: Vec<f64> = vec![];
                for (ks, oo, dd) in [(&ku, o.x, d.x), (&kv, o.y, d.y)] {
                    if dd == 0.0 {
                        continue;
                    }
                    for &k in ks.iter() {
                        let t = (k - oo) / dd;
                        if t > ta.min(*&tb) && t < ta.max(tb) {
                            ts.push(t);
                        }
                    }
                }
                ts.sort_by(|a, b| a.total_cmp(b));
                if tb < ta {
                    ts.reverse();
                }
                let mut a = ta;
                for t in ts {
                    out.push(Piece::Exact(pc, a, t));
                    a = t;
                }
                out.push(Piece::Exact(pc, a, tb));
            }
            p => out.push(p),
        }
    }
    out
}

/// Outer panel count: angular parameters get ≤ π/8 panels (GL8 is then
/// exact to ~1e-16 for the trig-polynomial integrands of analytic faces).
fn panels_for(pc: &Curve2, ta: f64, tb: f64, ang: (bool, bool), curved: bool, spline: bool, poly: bool) -> usize {
    match pc {
        Curve2::Line { d, .. } => {
            let step = std::f64::consts::PI / 8.0;
            let mut n: f64 = if spline && (poly || d.y == 0.0) { 1.0 } else if curved { 4.0 } else { 2.0 };
            if ang.0 {
                n = n.max((d.x * (tb - ta)).abs() / step);
            }
            if ang.1 {
                n = n.max((d.y * (tb - ta)).abs() / step);
            }
            n.ceil() as usize
        }
        Curve2::Circle { .. } | Curve2::Ellipse { .. } => ((tb - ta).abs() / (std::f64::consts::PI / 8.0)).ceil().max(2.0) as usize,
        Curve2::BSpline(bs) => (bs.ctrl.len() * 2).clamp(4, 4096),
    }
}

/// Which parameters are angles (trig dependence).
fn angular(s: &Surface) -> (bool, bool) {
    match s {
        Surface::Plane { .. } => (false, false),
        Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Revolution { .. } => (true, false),
        Surface::Sphere { .. } | Surface::Torus { .. } => (true, true),
        Surface::Extrusion { profile, .. } => (profile.period().is_some(), false),
        Surface::Offset { base, .. } => angular(base),
        Surface::BSpline(_) => (false, false),
        Surface::Screw { .. } => (true, false),
        Surface::Sweep(w) => (w.path.period().is_some(), w.profile.period().is_some()),
        // the section across a blend is a circular arc, angle-proportional in v
        Surface::Blend(_) => (false, true),
    }

}

/// ∮ G dv over the face boundary = sense · ∫∫_D F(σ, σu×σv) du dv, i.e.
/// the integral of F against the OUTWARD area element. `res` ≥ 1 scales
/// panel counts (region.rs refines near-singular integrands with it).
pub fn face_integral<const K: usize>(b: &Body, f: FaceId, res: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    face_integral_with(b, f, res, integrand, None, false, None)
}

/// [`face_integral`] for an integrand that is a polynomial in the surface parameter `v` on cylinders and
/// cones (the area element and the moments of a cylinder / cone are): a face with a long dense boundary
/// (cut by helical flutes) then tabulates its inner integrals (see `analytic_tab`).
pub fn face_integral_vpoly<const K: usize>(b: &Body, f: FaceId, res: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    face_integral_with(b, f, res, integrand, None, true, None)
}

/// n-point Gauss-Legendre rule on [-1, 1] (Newton on the Legendre polynomial).
fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
        let mut dp = 1.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n {
                let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            if n == 1 {
                p0 = 1.0;
                p1 = x;
            }
            dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let dx = p1 / dp;
            x -= dx;
            if dx.abs() < 1e-16 {
                break;
            }
        }
        out.push((x, 2.0 / ((1.0 - x * x) * dp * dp)));
    }
    out.reverse();
    out
}

/// [`face_integral`] for an integrand that is a polynomial of degree `ord`
/// in the position times the (bilinear) area element, at `res` = 1. On a
/// non-rational B-spline face every Gauss panel is then exact with just the
/// points the degree needs (one panel per u span, fewer points across v),
/// which matters on long screw sweeps (hundreds of spans).
pub fn face_integral_poly<const K: usize>(b: &Body, f: FaceId, ord: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    face_integral_with(b, f, 1, integrand, Some(ord), true, None)
}

thread_local! {
    /// Cumulative-integral tables of long spline faces (see `build_vtab`), by caller-chosen key.
    static VTAB_CACHE: std::cell::RefCell<std::collections::HashMap<u64, std::rc::Rc<dyn std::any::Any>>> = std::cell::RefCell::new(std::collections::HashMap::new());
}

/// [`face_integral_poly`] for an integrand that is the same function on every call with the same
/// `key` (the key must identify the integrand and the surface): the tables a long spline face
/// builds depend on the surface and the integrand, not on the face's trimming, so a surface that
/// comes back with other trimming (a cut through a shell wall) reuses them.
pub fn face_integral_poly_keyed<const K: usize>(b: &Body, f: FaceId, ord: usize, key: u64, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    face_integral_with(b, f, 1, integrand, Some(ord), true, Some(key ^ (K as u64).wrapping_mul(0x9e3779b97f4a7c15) ^ (ord as u64) << 56))
}

fn face_integral_with<const K: usize>(b: &Body, f: FaceId, res: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K], poly_ord: Option<usize>, vpoly: bool, tab_key: Option<u64>) -> [f64; K] {
    let surf = &b.face(f).surface;
    let coarse = tab_key.is_some() && COARSE.with(|c| c.get());
    let tab_key = tab_key.map(|k| if coarse { k ^ 0xc0a2_5e00_0000_0001 } else { k });
    let derived = derived_pcurves(b, f);
    let pieces = split_at_knots(surf, face_pieces(b, f, &derived, tab_key.is_none()));
    let mut acc = [0.0; K];
    if pieces.is_empty() {
        return acc;
    }
    // reference u* and u extent
    let first = match &pieces[0] {
        Piece::Exact(pc, ta, _) => pc.eval(*ta),
        Piece::Poly(p) => p[0],
    };
    let ustar = first.x;
    let (mut umin, mut umax) = (ustar, ustar);
    let (mut vmin, mut vmax) = (first.y, first.y);
    for p in &pieces {
        match p {
            Piece::Exact(pc, ta, tb) => {
                for k in 0..=8 {
                    let q = pc.eval(ta + (tb - ta) * k as f64 / 8.0);
                    umin = umin.min(q.x);
                    umax = umax.max(q.x);
                    vmin = vmin.min(q.y);
                    vmax = vmax.max(q.y);
                }
            }
            Piece::Poly(pts) => pts.iter().for_each(|q| {
                umin = umin.min(q.x);
                umax = umax.max(q.x);
                vmin = vmin.min(q.y);
                vmax = vmax.max(q.y);
            }),
        }
    }
    let width = (umax - umin).max(1e-12);
    let ang = angular(surf);
    // (many boundary samples: a face cut by helical flutes)
    let dense_boundary = vpoly && {
        let dense: usize = pieces
            .iter()
            .map(|p| match p {
                Piece::Exact(pc @ Curve2::BSpline(_), ta, tb) => panels_for(pc, *ta, *tb, ang, true, false, false) * 8,
                Piece::Poly(pts) => pts.len() * 8,
                _ => 0,
            })
            .sum();
        dense > 1500
    };
    let mut hu = width / (4.0 * res as f64);
    if ang.0 {
        hu = hu.min(std::f64::consts::PI / (8.0 * res as f64));
    }
    // (the keyed caller is the post-boolean volume check at 1e-3 of the volume: the original kernel
    // integrates with one panel per knot span there; `width / 4` is enough for it)
    if tab_key.is_none() && matches!(surf, Surface::BSpline(_) | Surface::Revolution { .. } | Surface::Extrusion { .. } | Surface::Screw { .. }) {
        hu = hu.min(width / (16.0 * res as f64));
    }
    // a plane's polynomial integrands are exact in one GL8 panel: with a dense boundary do not subdivide
    if dense_boundary && matches!(surf, Surface::Plane { .. }) {
        hu = width * 2.0;
    }
    // polynomial integrand on a non-rational spline: exact panel counts
    // (u: degree (ord+2) pu, cumulated; v: degree (ord+2) pv - 1, GL points)
    let poly_deg = match (surf, poly_ord) {
        (Surface::BSpline(bs), Some(ord)) if bs.weights.is_none() => Some(((ord + 2) * bs.deg_u, (ord + 2) * bs.deg_v)),
        _ => None,
    };
    let fast_panels = match poly_deg {
        Some((du, _)) if du <= 15 => 1,
        _ => 2,
    };
    // coarse tier: across a polynomial span the Gauss rule with just the points the degree needs
    // (exact to rounding, like the 8 points, which are more than a cubic needs)
    let coarse_rule: Option<Vec<(f64, f64)>> = if coarse { poly_deg.filter(|&(du, _)| du <= 15).map(|(du, _)| gauss_legendre(((du + 2) / 2).clamp(2, 8))) } else { None };
    let v_rule: Option<Vec<(f64, f64)>> = poly_deg.map(|(_, dv)| gauss_legendre(((dv + 1) / 2).clamp(2, 8)));
    // a cylinder / cone cut by helical flutes: thousands of boundary samples, each needing an inner integral
    // over up to a turn. The integrands are polynomial in v, so tabulate the cumulative integral at the
    // pi/8 cuts for a few v nodes (as for the screw patches) and interpolate
    let analytic_tab = dense_boundary && matches!(surf, Surface::Cylinder { .. } | Surface::Cone { .. }) && umax - umin > 0.8 && vmax > vmin;
    let curved = !matches!(surf, Surface::Plane { .. });
    // non-rational B-spline boundary lines are cut at the knots: pieces are short
    // (the one-panel rule is exact for polynomial integrands only; an area element is not, e.g. a ruled twisted
    // face. A piece along u (v constant) contributes nothing, so it keeps the single panel)
    let spline = matches!(surf, Surface::BSpline(b) if b.weights.is_none());

    // B-spline faces: cut the u-integral at the interior knots (rational
    // quarter-turn arcs have C0 knots; a panel across one loses ~1e-4)
    let knots_u: Vec<f64> = match surf {
        Surface::BSpline(bs) => {
            let (a, z) = bs.domain().0;
            let mut k: Vec<f64> = bs.knots_u.iter().copied().filter(|&x| x > a && x < z).collect();
            k.dedup();
            k
        }
        // a screw patch: cut every pi/8 (a hair less, so a cut segment never needs a second panel), where one
        // GL8 panel is exact to rounding for the trigonometric integrands
        Surface::Screw { urange: (a, z), .. } => {
            let step = std::f64::consts::PI / 8.0 * 0.9999;
            (1..).map(|i| a + step * i as f64).take_while(|&x| x < z - 1e-12).collect()
        }
        _ if analytic_tab => {
            let step = std::f64::consts::PI / 8.0 * 0.9999;
            (1..).map(|i| umin + step * i as f64).take_while(|&x| x < umax - 1e-12).collect()
        }
        _ => vec![],
    };
    // ∫ from u = a to u = c at fixed v, cut at the interior knots
    // (the iso-v evaluator of the last v asked for: the table build and `h_diff` ask for many segments at the
    // same v; and the u basis of a cut segment, which does not depend on v: a table node of every v span
    // integrates the same knot spans at the same Gauss points. Both give the very numbers recomputing would.)
    type BasisPts = std::rc::Rc<Vec<[[f64; 12]; 3]>>;
    let vline_last: std::cell::RefCell<Option<(u64, Option<ck_geom::nurbs::bsurf::VLine<'_>>)>> = Default::default();
    let basis_memo: std::cell::RefCell<std::collections::HashMap<(u64, u64), BasisPts>> = Default::default();
    let rational = matches!(surf, Surface::BSpline(bs) if bs.weights.is_some());
    let memo_rows = std::cell::Cell::new(0usize);
    let cone_t = match surf {
        Surface::Cone { semi_angle, .. } => semi_angle.tan(),
        _ => 0.0,
    };
    let seg_integral = |a0: f64, u: f64, v: f64, memo: bool| -> [f64; K] {
        let mut g = [0.0; K];
        if a0 == u {
            return g;
        }
        // cut points: a0, the interior knots strictly between a0 and u (in
        // travel order), u -- indexed, no allocation
        let (lo, hi) = (a0.min(u), a0.max(u));
        let (i0, i1) = (knots_u.partition_point(|&k| k <= lo), knots_u.partition_point(|&k| k < hi));
        let m_in = i1.saturating_sub(i0);
        let cut = |j: usize| -> f64 {
            if j == 0 {
                a0
            } else if j == m_in + 1 {
                u
            } else if u >= a0 {
                knots_u[i0 + j - 1]
            } else {
                knots_u[i1 - j]
            }
        };
        // non-rational B-spline: fixed-v evaluator (v basis once; v direction
        // collapsed per knot span); each cut segment lies in one polynomial
        // span, where two GL8 panels are exact for the area-element
        // polynomials and ~1e-13 for |Su x Sv|
        let vl = match surf {
            Surface::BSpline(bs) => {
                let mut last = vline_last.borrow_mut();
                match &*last {
                    Some((k, l)) if *k == v.to_bits() => *l,
                    _ => {
                        let l = bs.vline(v);
                        *last = Some((v.to_bits(), l));
                        l
                    }
                }
            }
            _ => None,
        };
        let sl = surf.screw_vline(v);
        for j in 0..=m_in {
            let (a, c) = (cut(j), cut(j + 1));
            let span = c - a;
            if span == 0.0 {
                continue;
            }
            let fast = vl.as_ref().and_then(|l| l.segment(0.5 * (a + c))).filter(|s| s.covers(a, c));
            // (a rational span keeps the panel count of the generic path: only the evaluator is the fast one)
            let n = if fast.is_some() && !rational { fast_panels * res } else { ((span.abs() / hu).ceil() as usize).clamp(1, 512 * res) };
            let h = span / n as f64;
            let rule: &[(f64, f64)] = match &coarse_rule {
                Some(r) if fast.is_some() => r,
                _ => &GL,
            };
            let bas: Option<BasisPts> = match (&fast, memo) {
                (Some(sg), true) => {
                    let mut m = basis_memo.borrow_mut();
                    // (entries and basis rows: a rational span has many panels, and the rows are 288 bytes each)
                    if m.len() >= 8192 || memo_rows.get() >= 131_072 {
                        m.clear();
                        memo_rows.set(0);
                    }
                    Some(
                        m.entry((a.to_bits(), c.to_bits()))
                            .or_insert_with(|| {
                                memo_rows.set(memo_rows.get() + n * rule.len());
                                let mut v = Vec::with_capacity(n * rule.len());
                                for i in 0..n {
                                    let mid = a + h * (i as f64 + 0.5);
                                    for &(x, _) in rule.iter() {
                                        v.push(sg.basis(mid + 0.5 * h * x));
                                    }
                                }
                                std::rc::Rc::new(v)
                            })
                            .clone(),
                    )
                }
                _ => None,
            };
            let mut at = 0usize;
            for i in 0..n {
                let m = a + h * (i as f64 + 0.5);
                for &(x, w) in rule.iter() {
                    let s = m + 0.5 * h * x;
                    let (pt, nrm) = match &fast {
                        Some(sg) => {
                            let (p, du, dv) = match &bas {
                                Some(b) => {
                                    at += 1;
                                    sg.eval1_basis(&b[at - 1])
                                }
                                None => sg.eval1(s),
                            };
                            (p, du.cross(dv))
                        }
                        None if sl.is_some() => {
                            let (p, du, dv) = sl.as_ref().unwrap().eval1(s);
                            (p, du.cross(dv))
                        }
                        // cylinder / cone / sphere / torus: the first derivatives only, one sin_cos (the very expressions of
                        // `Surface::derivs2`, which also builds the second derivatives from two sin_cos)
                        None => match surf {
                            Surface::Cylinder { frame, r } => {
                                let (sn, cs) = s.sin_cos();
                                let (rad, tan) = (frame.x * cs + frame.y * sn, frame.x * (-sn) + frame.y * cs);
                                (frame.o + rad * *r + frame.z * v, (tan * *r).cross(frame.z))
                            }
                            Surface::Cone { frame, r, .. } => {
                                let (sn, cs) = s.sin_cos();
                                let (rad, tan) = (frame.x * cs + frame.y * sn, frame.x * (-sn) + frame.y * cs);
                                let rho = r + v * cone_t;
                                (frame.o + rad * rho + frame.z * v, (tan * rho).cross(rad * cone_t + frame.z))
                            }
                            Surface::Sphere { frame, r } => {
                                let (sn, cs) = s.sin_cos();
                                let (rad, tan) = (frame.x * cs + frame.y * sn, frame.x * (-sn) + frame.y * cs);
                                let (sv, cv) = v.sin_cos();
                                (frame.o + rad * (r * cv) + frame.z * (r * sv), (tan * (r * cv)).cross(rad * (-r * sv) + frame.z * (r * cv)))
                            }
                            Surface::Torus { frame, major, minor } => {
                                let (sn, cs) = s.sin_cos();
                                let (rad, tan) = (frame.x * cs + frame.y * sn, frame.x * (-sn) + frame.y * cs);
                                let (sv, cv) = v.sin_cos();
                                let rho = major + minor * cv;
                                (frame.o + rad * rho + frame.z * (minor * sv), (tan * rho).cross(rad * (-minor * sv) + frame.z * (minor * cv)))
                            }
                            _ => {
                                let d = surf.derivs2(s, v);
                                (d.p, d.du.cross(d.dv))
                            }
                        },
                    };
                    let val = integrand(pt, nrm);
                    for k in 0..K {
                        g[k] += 0.5 * h * w * val[k];
                    }
                }
            }
        }
        g
    };
    // long multi-knot B-spline faces (screw sweeps): the inner integral over
    // u from u* would cost O(spans) per boundary sample. The integrand is a
    // low-degree polynomial in v on a span, so keep the cumulative integral
    // at every u knot for a few v nodes and Lagrange-interpolate in v
    let long_spline = analytic_tab || knots_u.len() >= if matches!(surf, Surface::Screw { .. }) { 6 } else { 32 };
    // per v span: Chebyshev-Lobatto nodes (exact for the polynomial integrands, ~1e-12 for the smooth
    // area element |Su x Sv|) and the cumulative integral at every u knot for each node
    type VSpanTab<const K: usize> = (f64, f64, Vec<f64>, Vec<Vec<[f64; K]>>);
    let vspans: Vec<(f64, f64)> = match surf {
        Surface::BSpline(bs) if long_spline => {
            let ((_, _), (dv0, dv1)) = bs.domain();
            let mut ks: Vec<f64> = vec![dv0];
            ks.extend(bs.knots_v.iter().copied().filter(|&x| x > dv0 && x < dv1));
            ks.push(dv1);
            ks.dedup();
            ks.windows(2).map(|w| (w[0], w[1])).collect()
        }
        Surface::Screw { vrange, .. } if long_spline && vrange.1 > vrange.0 => {
            let mut ks = vec![vrange.0];
            ks.extend(screw_profile_knots(surf));
            ks.push(vrange.1);
            ks.windows(2).map(|w| (w[0], w[1])).collect()
        }
        _ if analytic_tab => vec![(vmin, vmax)],
        _ => vec![],
    };
    let build_vtab_raw = || -> Option<(Vec<f64>, Vec<VSpanTab<K>>)> {
        let ((du0, du1), nvn) = match surf {
            Surface::BSpline(bs) => (bs.domain().0, match poly_deg {
                    // (the integrand is a polynomial of degree dv in v: dv + 1 nodes interpolate it exactly)
                    Some((_, dv)) if coarse => dv + 1,
                    _ => 3 * bs.deg_v + 6,
                }),
            // (volume-type integrands are trigonometric polynomials in v: 24 nodes are spectral; the area
            // element of a strongly twisted face has singularities near the real axis: 56)
            Surface::Screw { urange, .. } => (*urange, if poly_ord.is_some() { 24 } else { 56 }),
            _ if analytic_tab => ((umin, umax), 10),
            _ => return None,
        };
        if !long_spline || vspans.is_empty() {
            return None;
        }
        let mut ks = vec![du0];
        ks.extend(knots_u.iter().copied());
        ks.push(du1);
        let tabs = vspans
            .iter()
            .map(|&(a, b)| {
                let vn: Vec<f64> = (0..nvn).map(|j| a + (b - a) * 0.5 * (1.0 - (std::f64::consts::PI * j as f64 / (nvn - 1) as f64).cos())).collect();
                let cum: Vec<Vec<[f64; K]>> = vn
                    .iter()
                    .map(|&v| {
                        let mut c = vec![[0.0; K]];
                        for w in ks.windows(2) {
                            let d = seg_integral(w[0], w[1], v, true);
                            let mut g = *c.last().unwrap();
                            for k in 0..K {
                                g[k] += d[k];
                            }
                            c.push(g);
                        }
                        c
                    })
                    .collect();
                (a, b, vn, cum)
            })
            .collect();
        Some((ks, tabs))
    };
    type VTab<const K: usize> = Option<(Vec<f64>, Vec<VSpanTab<K>>)>;
    let cache_get = || -> Option<std::rc::Rc<VTab<K>>> {
        let key = tab_key?;
        VTAB_CACHE.with(|c| c.borrow().get(&key).cloned()).and_then(|r| r.downcast::<VTab<K>>().ok())
    };
    let build_vtab = || -> std::rc::Rc<VTab<K>> {
        if let Some(hit) = cache_get() {
            return hit;
        }
        let t = std::rc::Rc::new(build_vtab_raw());
        if let (Some(key), true) = (tab_key, t.is_some()) {
            VTAB_CACHE.with(|c| {
                let mut c = c.borrow_mut();
                if c.len() >= 16 {
                    c.clear();
                }
                c.insert(key, t.clone() as std::rc::Rc<dyn std::any::Any>);
            });
        }
        t
    };
    // a ruled screw face (one v span): the table is always cheaper; a face with several v spans builds it
    // only once as many boundary samples as table nodes sit at distinct v (a cut curve across the sweep)
    let vtab: std::cell::OnceCell<std::rc::Rc<VTab<K>>> = std::cell::OnceCell::new();
    if vspans.len() == 1 {
        let _ = vtab.set(build_vtab());
    }
    let new_lists = std::cell::Cell::new(0usize);
    let table_ready = || -> bool {
        if vtab.get().is_none() && vspans.len() > 1 {
            if let Some(hit) = cache_get() {
                let _ = vtab.set(hit);
            }
        }
        if vspans.len() == 1 || vtab.get().is_some() {
            return vtab.get().map_or(false, |t| t.is_some());
        }
        // (building costs one inner integral per table node: worth it once that many boundary samples
        // sit at distinct v)
        let nodes = vspans.len() * match surf {
            Surface::BSpline(bs) => 3 * bs.deg_v + 6,
            Surface::Screw { .. } => 56,
            _ => 0,
        };
        new_lists.get() >= nodes && vtab.get_or_init(build_vtab).is_some()
    };
    // h(u, v) - h(u*, v) from the table: the v interpolation (barycentric form on the Chebyshev-Lobatto
    // nodes) is shared by both ends; the partial spans are integrated directly at the true v (one
    // seg_integral instead of one per interpolation node)
    let h_diff = |u: f64, v: f64| -> [f64; K] {
        let (ks, tabs) = (**vtab.get().unwrap()).as_ref().unwrap();
        let kof = |x: f64| (ks.partition_point(|&y| y <= x).max(1) - 1).min(ks.len() - 2);
        let (k, k0) = (kof(u), kof(ustar));
        let si = (tabs.partition_point(|t| t.0 <= v).max(1) - 1).min(tabs.len() - 1);
        let (_, _, vn, cum) = &tabs[si];
        let n = vn.len();
        let mut w = [0.0f64; 128];
        let (mut sum, mut exact) = (0.0, None);
        for j in 0..n {
            let d = v - vn[j];
            if d == 0.0 {
                exact = Some(j);
                break;
            }
            let lam = if j % 2 == 0 { 1.0 } else { -1.0 } * if j == 0 || j + 1 == n { 0.5 } else { 1.0 };
            w[j] = lam / d;
            sum += w[j];
        }
        if let Some(j) = exact {
            w[..n].fill(0.0);
            w[j] = 1.0;
            sum = 1.0;
        }
        let mut g = [0.0; K];
        for j in 0..n {
            let wj = w[j] / sum;
            for i in 0..K {
                g[i] += wj * (cum[j][k][i] - cum[j][k0][i]);
            }
        }
        let (part, part0) = (seg_integral(ks[k], u, v, false), seg_integral(ks[k0], ustar, v, true));
        for i in 0..K {
            g[i] += part[i] - part0[i];
        }
        g
    };
    let period_tab: Option<(Vec<f64>, Vec<[f64; K]>)> = match (surf, surf.periods().0) {
        (Surface::Cylinder { .. } | Surface::Cone { .. }, Some(per)) if umax - umin > 1.5 * per && vmax > vmin => {
            let vn: Vec<f64> = (0..6).map(|j| vmin + (vmax - vmin) * j as f64 / 5.0).collect();
            let tab = vn.iter().map(|&v| seg_integral(ustar, ustar + per, v, false)).collect();
            Some((vn, tab))
        }
        _ => None,
    };
    // (a long face with several v spans: integrals checkpointed per v, the
    // v of a constant-v edge being equal to rounding)
    let ckpt: std::cell::RefCell<std::collections::HashMap<u64, Vec<(f64, [f64; K])>>> = Default::default();
    let inner = |u: f64, v: f64| -> [f64; K] {
        if u - ustar == 0.0 {
            return [0.0; K];
        }
        if long_spline && table_ready() {
            return h_diff(u, v);
        }
        if long_spline {
            let v = (v * 1e9).round() / 1e9;
            let mut map = ckpt.borrow_mut();
            let list = map.entry(v.to_bits()).or_insert_with(|| {
                new_lists.set(new_lists.get() + 1);
                vec![(ustar, [0.0; K])]
            });
            let idx = list.partition_point(|e| e.0 < u);
            let cand = [idx.checked_sub(1), (idx < list.len()).then_some(idx)];
            let best = cand.iter().flatten().copied().min_by(|&x, &y| (list[x].0 - u).abs().total_cmp(&(list[y].0 - u).abs())).unwrap();
            let (uc, gc) = list[best];
            if uc == u {
                return gc;
            }
            let d = seg_integral(uc, u, v, false);
            let mut g = gc;
            for k in 0..K {
                g[k] += d[k];
            }
            list.insert(idx, (u, g));
            return g;
        }
        // a face wound many times round its axis (cylinder cut by a
        // helical groove): whole periods integrate to the same value, a
        // polynomial in v for cylinders / cones (tabulated at a few nodes)
        if let (Some(per), Some((vn, tab))) = (surf.periods().0, period_tab.as_ref()) {
            if (u - ustar).abs() > 1.5 * per {
                let k = ((u - ustar) / per).trunc();
                let mut full = [0.0; K];
                for j in 0..vn.len() {
                    let mut w = 1.0;
                    for m in 0..vn.len() {
                        if m != j {
                            w *= (v - vn[m]) / (vn[j] - vn[m]);
                        }
                    }
                    for i in 0..K {
                        full[i] += w * tab[j][i];
                    }
                }
                let mut g = seg_integral(ustar, u - k * per, v, false);
                for i in 0..K {
                    g[i] += k * full[i];
                }
                return g;
            }
        }
        seg_integral(ustar, u, v, false)
    };
    let mut add = |uv: Vec2, dvdt: f64, wt: f64| {
        if dvdt == 0.0 {
            return;
        }
        let g = inner(uv.x, uv.y);
        for k in 0..K {
            acc[k] += wt * dvdt * g[k];
        }
    };
    // a generic (non-polynomial) integrand on a screw patch -- the area element of a strongly twisted face
    // has complex singularities close to the real axis -- is integrated along the boundary adaptively
    let adaptive = matches!(surf, Surface::Screw { .. }) && poly_ord.is_none();
    let mut acc_ad = [0.0; K];
    let panel = |pc: &Curve2, a: f64, b: f64| -> [f64; K] {
        let mut s = [0.0; K];
        let (m, hh) = (0.5 * (a + b), 0.5 * (b - a));
        for &(x, w) in GL.iter() {
            let d = pc.derivs(m + hh * x, 1);
            if d[1].y == 0.0 {
                continue;
            }
            let g = inner(d[0].x, d[0].y);
            for k in 0..K {
                s[k] += hh * w * d[1].y * g[k];
            }
        }
        s
    };
    for p in &pieces {
        match p {
            // (the dense panels of a fitted boundary curve already follow it; the straight cap edges have 4)
            Piece::Exact(pc, ta, tb) if adaptive && matches!(pc, Curve2::Line { d, .. } if d.y != 0.0) => {
                let n = panels_for(pc, *ta, *tb, ang, curved, spline, false) * res;
                let h = (tb - ta) / n as f64;
                for i in 0..n {
                    let (a, b) = (ta + h * i as f64, ta + h * (i + 1) as f64);
                    let g = adapt_panel(&|x, y| panel(pc, x, y), a, b, panel(pc, a, b), 0);
                    for k in 0..K {
                        acc_ad[k] += g[k];
                    }
                }
            }
            Piece::Exact(pc, ta, tb) => {
                let mut n = panels_for(pc, *ta, *tb, ang, curved, spline, poly_ord.is_some()) * res;
                // (the keyed caller is the post-boolean volume check, which needs ~1e-4 of the volume: a quarter
                // of the usual two panels per control point of a fitted trim curve is good to ~5e-5 there)
                if tab_key.is_some() && matches!(pc, Curve2::BSpline(_)) {
                    n = (n / 16).max(2).min(n);
                }
                let h = (tb - ta) / n as f64;
                // a piece along v at constant u (a cap of a long sweep): the
                // cumulated integrand is a polynomial in v of the degree above
                let rule: &[(f64, f64)] = match (&v_rule, pc) {
                    (Some(r), Curve2::Line { d, .. }) if d.x == 0.0 && n == 1 => r,
                    _ => &GL,
                };
                for i in 0..n {
                    let m = ta + h * (i as f64 + 0.5);
                    for &(x, w) in rule.iter() {
                        let t = m + 0.5 * h * x;
                        let d = pc.derivs(t, 1);
                        add(d[0], d[1].y, 0.5 * h * w);
                    }
                }
            }
            Piece::Poly(pts) => {
                for s in pts.windows(2) {
                    let (a, c) = (s[0], s[1]);
                    let dv = c.y - a.y;
                    for &(x, w) in GL.iter() {
                        let t = 0.5 * (x + 1.0);
                        add(a.lerp(c, t), dv, 0.5 * w);
                    }
                }
            }
        }
    }
    for k in 0..K {
        acc[k] += acc_ad[k];
    }
    acc
}

/// Panel `[a, b]` of a boundary integral refined (by halves, to depth 6) until the two halves agree with
/// the whole to 1e-10 in every component.
fn adapt_panel<const K: usize>(panel: &dyn Fn(f64, f64) -> [f64; K], a: f64, b: f64, whole: [f64; K], depth: u32) -> [f64; K] {
    let m = 0.5 * (a + b);
    let (l, r) = (panel(a, m), panel(m, b));
    let mut ok = true;
    let mut sum = [0.0; K];
    for k in 0..K {
        sum[k] = l[k] + r[k];
        if (sum[k] - whole[k]).abs() > 1e-10 * (l[k].abs() + r[k].abs()) + 1e-12 {
            ok = false;
        }
    }
    if ok || depth >= 6 {
        return sum;
    }
    let (gl, gr) = (adapt_panel(panel, a, m, l, depth + 1), adapt_panel(panel, m, b, r, depth + 1));
    for k in 0..K {
        sum[k] = gl[k] + gr[k];
    }
    sum
}

/// Raw moments of a set of faces about `r` (density 1).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Moments {
    pub volume: f64,
    pub area: f64,
    /// ∫ (p - r) dV
    pub first: Vec3,
    /// ∫ (p-r)_i (p-r)_j dV as [xx, yy, zz, xy, yz, xz]
    pub second: [f64; 6],
    pub r: Vec3,
}

impl Moments {
    /// Add one face's integrals (see [`faces_moments`]).
    fn add_face(&mut self, v: &[f64; 11]) {
        self.volume += v[0];
        // v[1] is sense * (orientation of the face's loops in its parameter domain) * area. The product of the two
        // signs is +1 on a consistently built face, but a revolved face can carry clockwise loops together with
        // sense = true (a torus whose centre lies on the far side of the axis does): the volume terms cancel that
        // pair of signs, an area does not. An area is never negative, so take its magnitude.
        self.area += v[1].abs();
        self.first = self.first + Vec3::new(v[2], v[3], v[4]);
        for i in 0..6 {
            self.second[i] += v[5 + i];
        }
    }

    pub fn com(&self) -> Vec3 {
        self.r + self.first / self.volume
    }
    /// Re-express about another reference point.
    pub fn about(&self, r: Vec3) -> Moments {
        let d = self.r - r; // p - r = (p - self.r) + d
        let (f, v, s) = (self.first, self.volume, self.second);
        let sec = [
            s[0] + 2.0 * d.x * f.x + v * d.x * d.x,
            s[1] + 2.0 * d.y * f.y + v * d.y * d.y,
            s[2] + 2.0 * d.z * f.z + v * d.z * d.z,
            s[3] + d.x * f.y + d.y * f.x + v * d.x * d.y,
            s[4] + d.y * f.z + d.z * f.y + v * d.y * d.z,
            s[5] + d.x * f.z + d.z * f.x + v * d.x * d.z,
        ];
        Moments { volume: v, area: self.area, first: f + d * v, second: sec, r }
    }
    pub fn scaled(&self, k: f64) -> Moments {
        let mut m = *self;
        m.volume *= k;
        m.area *= k;
        m.first = m.first * k;
        m.second.iter_mut().for_each(|x| *x *= k);
        m
    }
    pub fn add(&self, o: &Moments) -> Moments {
        let o = o.about(self.r);
        let mut s = self.second;
        for i in 0..6 {
            s[i] += o.second[i];
        }
        Moments { volume: self.volume + o.volume, area: self.area + o.area, first: self.first + o.first, second: s, r: self.r }
    }
    /// Inertia tensor about the centroid (rows).
    pub fn inertia(&self) -> [[f64; 3]; 3] {
        self.inertia_about(self.com())
    }
    /// Inertia tensor about an arbitrary point `p` (rows, world axes, density 1): J = ∫ (|q|² E − q qᵀ) dV, q = x − p.
    /// Products of inertia carry the usual minus sign, so a body's tensor stays positive definite whatever holes it has.
    pub fn inertia_about(&self, p: Vec3) -> [[f64; 3]; 3] {
        let s = self.about(p).second;
        [[s[1] + s[2], -s[3], -s[5]], [-s[3], s[0] + s[2], -s[4]], [-s[5], -s[4], s[0] + s[1]]]
    }
}

/// Entries the table of face moments holds before it is cleared.
const MOMENTS_MEMO_CAP: usize = 1 << 15;

thread_local! {
    /// The integrals of [`faces_moments`] of a face by [`moments_key`]: the mass properties of a body are asked again
    /// for the same faces (the check after a boolean, the verification of a rebuild against the import, a later
    /// snapshot of the same history), and an integral over a free-form face is the cost of a measurement.
    static MOMENTS_MEMO: std::cell::RefCell<std::collections::HashMap<u64, [f64; 11]>> = std::cell::RefCell::new(Default::default());
}

/// Hash of everything a face's moments about `r` depend on: its surface, sense, trimming (edge curves at five
/// points each, with the coedge directions) and the reference point.
fn moments_key(b: &Body, f: FaceId, r: Vec3) -> u64 {
    let face = b.face(f);
    let mut h = 0xcbf29ce484222325u64;
    let mut mix = |x: u64| {
        h = (h ^ x).wrapping_mul(0x100000001b3);
        h ^= h >> 29;
    };
    mix(face.surface.fingerprint());
    mix(face.sense as u64);
    mix(r.x.to_bits());
    mix(r.y.to_bits());
    mix(r.z.to_bits());
    for &lp in &face.loops {
        mix(0xa5a5);
        for &c in &b.lp(lp).coedges {
            let ed = b.edge(b.coedge(c).edge);
            mix(b.coedge(c).rev as u64);
            // five points, not three: a full circle (or any closed edge) read forwards or backwards has the same
            // start, middle and end, and only a point between them tells its direction
            for t in [ed.t0, 0.75 * ed.t0 + 0.25 * ed.t1, 0.5 * (ed.t0 + ed.t1), 0.25 * ed.t0 + 0.75 * ed.t1, ed.t1] {
                let p = ed.curve.eval(t);
                mix(p.x.to_bits());
                mix(p.y.to_bits());
                mix(p.z.to_bits());
            }
        }
    }
    h
}

/// Moments of the given faces (normally one lump / the whole body).
pub fn faces_moments(b: &Body, faces: &[FaceId], r: Vec3) -> Moments {
    let mut m = Moments { r, ..Default::default() };
    for &f in faces {
        let key = moments_key(b, f, r);
        if let Some(v) = MOMENTS_MEMO.with(|t| t.borrow().get(&key).copied()) {
            m.add_face(&v);
            continue;
        }
        let v = face_integral_vpoly::<11>(b, f, 1, &|p, n| {
            let q = p - r;
            [
                q.dot(n) / 3.0,
                n.len(),
                0.5 * q.x * q.x * n.x,
                0.5 * q.y * q.y * n.y,
                0.5 * q.z * q.z * n.z,
                q.x * q.x * q.x * n.x / 3.0,
                q.y * q.y * q.y * n.y / 3.0,
                q.z * q.z * q.z * n.z / 3.0,
                0.5 * q.x * q.x * q.y * n.x,
                0.5 * q.y * q.y * q.z * n.y,
                0.5 * q.z * q.z * q.x * n.z,
            ]
        });
        MOMENTS_MEMO.with(|t| {
            let mut t = t.borrow_mut();
            if t.len() >= MOMENTS_MEMO_CAP {
                t.clear();
            }
            t.insert(key, v);
        });
        m.add_face(&v);
    }
    m
}

/// True when some edge of these faces is used by other than two coedges: the faces do not enclose a volume (a sheet).
pub fn faces_are_open(b: &Body, faces: &[FaceId]) -> bool {
    let mut uses: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
    for &f in faces {
        for &l in &b.face(f).loops {
            for &c in &b.lp(l).coedges {
                *uses.entry(b.coedge(c).edge.idx() as u32).or_insert(0) += 1;
            }
        }
    }
    uses.values().any(|&n| n != 2)
}

/// Area-weighted moments of a sheet (surface density 1): `volume` holds the area, `first` = ∫ (p - r) dA and `second`
/// = ∫ (p-r)_i (p-r)_j dA, so [`Moments::com`] is the area centroid and the inertia is that of a zero-thickness shell.
pub fn sheet_moments(b: &Body, faces: &[FaceId], r: Vec3) -> Moments {
    let mut m = Moments { r, ..Default::default() };
    for &f in faces {
        let v = face_integral_vpoly::<10>(b, f, 1, &|p, n| {
            let (q, a) = (p - r, n.len());
            [a, q.x * a, q.y * a, q.z * a, q.x * q.x * a, q.y * q.y * a, q.z * q.z * a, q.x * q.y * a, q.y * q.z * a, q.x * q.z * a]
        });
        // an area is never negative (see `add_face`): the sign of the area carries the face's orientation
        let k = if v[0] < 0.0 { -1.0 } else { 1.0 };
        m.volume += k * v[0];
        m.area += k * v[0];
        m.first = m.first + Vec3::new(k * v[1], k * v[2], k * v[3]);
        for i in 0..6 {
            m.second[i] += k * v[4 + i];
        }
    }
    m
}

pub fn lump_faces(b: &Body, l: LumpId) -> Vec<FaceId> {
    let mut out = vec![];
    for &s in &b.lump(l).shells {
        if b.shell(s).alive {
            out.extend(b.shell(s).faces.iter().copied().filter(|&f| b.face(f).alive));
        }
    }
    out
}

/// Body index of every live face, parallel to `face_ids()` (the order of the wire `faces` list and of `triFace`
/// values): the face's lump rank among the live lumps, the same `index` `massProps {bodies:true}` gives each entry.
pub fn face_bodies(b: &Body) -> Vec<i32> {
    let mut rank = vec![-1i32; b.lumps.len()];
    for (i, l) in b.lump_ids().enumerate() {
        rank[l.idx()] = i as i32;
    }
    b.face_ids().map(|f| rank[b.face_lump(f).idx()]).collect()
}

/// Name of every live lump (body), in the order of `face_bodies` indexes and of `massProps {bodies:true}` `index`.
pub fn body_names(b: &Body) -> Vec<Option<String>> {
    b.lump_ids().map(|l| b.lump(l).name.clone()).collect()
}

/// Reference point (bbox centre) the moments are taken about.
pub fn ref_point_of(b: &Body) -> Vec3 {
    ref_point(b)
}

fn ref_point(b: &Body) -> Vec3 {
    let (lo, hi) = b.bbox();
    if lo.is_finite() && hi.is_finite() {
        (lo + hi) * 0.5
    } else {
        Vec3::ZERO
    }
}

/// Per-lump moments (wire lump order) about a common reference point.
pub fn lump_moments(b: &Body) -> Vec<Moments> {
    let r = ref_point(b);
    b.lump_ids().map(|l| faces_moments(b, &lump_faces(b, l), r)).collect()
}

/// Signed volume of a set of faces (negative = inside-out / void shell).
pub fn signed_volume(b: &Body, faces: &[FaceId]) -> f64 {
    let r = ref_point(b);
    faces
        .iter()
        .map(|&f| face_integral_poly::<1>(b, f, 1, &|p, n| [(p - r).dot(n) / 3.0])[0])
        .sum()
}

/// Signed volume of a set of faces to about 1e-3, for sign tests (orientation, `VOLUME_NONPOSITIVE`):
/// the coarse quadrature tier with per-face memoisation, which is several times cheaper than
/// [`signed_volume`] on free-form faces and shared between calls on the same faces.
pub fn signed_volume_quick(b: &Body, faces: &[FaceId]) -> f64 {
    let parts: Vec<f64> = faces.iter().map(|&f| face_volume_memo(b, f, true).0).collect();
    let (total, scale) = (parts.iter().sum::<f64>(), parts.iter().map(|x| x.abs()).sum::<f64>());
    // a total lost in cancellation (or a non-finite share, e.g. a pole-closed cone) is not a sign to trust
    if total.is_finite() && total.abs() > 1e-6 * scale && scale > 0.0 {
        total
    } else {
        signed_volume(b, faces)
    }
}

/// The quick signed volume of a set of faces when it is far from zero against the faces' shares (its sign is
/// then that of the exact volume of a closed shell: the quadrature is good to about 1e-6 of the shares), else
/// `None` (the caller measures exactly).
pub fn signed_volume_quick_margin(b: &Body, faces: &[FaceId]) -> Option<f64> {
    let parts: Vec<f64> = faces.iter().map(|&f| face_volume_memo(b, f, true).0).collect();
    let (total, scale) = (parts.iter().sum::<f64>(), parts.iter().map(|x| x.abs()).sum::<f64>());
    (total.is_finite() && scale > 0.0 && total.abs() > 1e-3 * scale).then_some(total)
}

/// One face's share of the signed volume, measured from the origin (so it depends on the face
/// alone and can be cached per face; the sum over a closed shell does not depend on the origin).
pub fn face_signed_volume(b: &Body, f: FaceId) -> f64 {
    face_integral_poly_keyed::<1>(b, f, 1, b.face(f).surface.fingerprint() ^ 0x5eed_0001, &|p, n| [p.dot(n) / 3.0])[0]
}


/// Entries one tier of the face-volume table holds before it is cleared. It has to hold the faces of the biggest model that
/// is rebuilt over and over: `rebuild_lumps` visits every face on each boolean result, and a table smaller than the model
/// (it was 8192, a model of 11,500 faces) is cleared during every pass and so never hits, which made every cut of a large
/// multi-body model cost seconds. 1 << 18 entries is a few MB.
const FACE_VOL_MEMO_CAP: usize = 1 << 18;

thread_local! {
    /// Per-face signed volumes by orientation-free face key (see `face_key`), one table per tier.
    static FACE_VOL_MEMO: std::cell::RefCell<[std::collections::HashMap<u64, f64>; 2]> = std::cell::RefCell::new([Default::default(), Default::default()]);
}

/// One face's signed volume share (see `face_signed_volume`), memoised by a hash of the face's surface
/// and trimming; a reversed traversal of the same face reuses the negated value. Boolean chains keep
/// most faces from one result to the next, so the sum over a result is mostly lookups. `coarse` selects
/// the cheap quadrature tier. Returns the value and whether it was a table hit.
pub fn face_volume_memo(b: &Body, f: FaceId, coarse: bool) -> (f64, bool) {
    let tier = coarse as usize;
    let (key, sgn) = face_key(b, f);
    if let Some(x) = FACE_VOL_MEMO.with(|m| m.borrow()[tier].get(&key).copied()) {
        return (sgn * x, true);
    }
    let x = if coarse { with_coarse(|| face_signed_volume(b, f)) } else { face_signed_volume(b, f) };
    if x.is_finite() {
        FACE_VOL_MEMO.with(|m| {
            let mut m = m.borrow_mut();
            if m[tier].len() >= FACE_VOL_MEMO_CAP {
                m[tier].clear();
            }
            m[tier].insert(key, sgn * x);
        });
    }
    (x, false)
}

/// The volume of `a` minus the volume of `b`, summed over the faces that have no twin in the other body (a twin = the same
/// `face_key`: same surface, same trimming): a twin has the same share in both, so it cancels. Two bodies that differ in a few
/// faces cost only those faces, not the whole free-form model. Faces are measured from the origin as in [`face_signed_volume`],
/// so the difference is the difference of the volumes for closed shells.
pub fn volume_difference_unshared(a: &Body, b: &Body) -> f64 {
    let keys = |x: &Body| -> std::collections::HashSet<u64> { x.face_ids().map(|f| face_key(x, f).0).collect() };
    let (ka, kb) = (keys(a), keys(b));
    let side = |x: &Body, other: &std::collections::HashSet<u64>| -> f64 {
        x.face_ids().filter(|&f| !other.contains(&face_key(x, f).0)).map(|f| face_volume_memo(x, f, false).0).sum()
    };
    side(a, &kb) - side(b, &ka)
}

/// Orientation-free key of everything one face's volume integral depends on, and the sign that maps
/// this face's integral to the key's canonical traversal. The integral runs along the face's boundary
/// as traversed (the face `sense` does not enter it), so the same face traversed the other way round
/// (the tool's face of a cut, reversed) has the negated integral: both traversals get the same key,
/// `+1` for the one whose forward hash is the smaller.
fn face_key(b: &Body, f: FaceId) -> (u64, f64) {
    let face = b.face(f);
    // (rev flag, [start, quarter, middle, three-quarter, end] of the edge curve) of every coedge, loop by loop
    let loops: Vec<Vec<(bool, [Vec3; 5])>> = face
        .loops
        .iter()
        .map(|&l| {
            b.lp(l)
                .coedges
                .iter()
                .map(|&c| {
                    let ed = b.edge(b.coedge(c).edge);
                    (b.coedge(c).rev, [ed.t0, 0.75 * ed.t0 + 0.25 * ed.t1, 0.5 * (ed.t0 + ed.t1), 0.25 * ed.t0 + 0.75 * ed.t1, ed.t1].map(|t| ed.curve.eval(t)))
                })
                .collect()
        })
        .collect();
    let sfp = face.surface.fingerprint();
    let hash = |reversed: bool| -> u64 {
        let mut h = 0xcbf29ce484222325u64;
        let mut mix = |x: u64| {
            h = (h ^ x).wrapping_mul(0x100000001b3);
            h ^= h >> 29;
        };
        mix(sfp);
        for cs in &loops {
            mix(0xa5a5);
            for i in 0..cs.len() {
                let (rev, pts) = &cs[if reversed { cs.len() - 1 - i } else { i }];
                mix((*rev != reversed) as u64);
                for k in 0..5 {
                    let p = pts[if reversed { 4 - k } else { k }];
                    mix(p.x.to_bits());
                    mix(p.y.to_bits());
                    mix(p.z.to_bits());
                }
            }
        }
        h
    };
    let (fw, rv) = (hash(false), hash(true));
    // a face bounded only by seam twins (a whole sphere): the loop reads the same flipped or not, the orientation
    // lives in the pcurve placement, which follows the sense: the sense is part of the key
    let seam_only = face.loops.iter().all(|&l| {
        let cs = &b.lp(l).coedges;
        !cs.is_empty() && cs.iter().all(|&c| cs.iter().any(|&d| b.coedge(d).edge == b.coedge(c).edge && b.coedge(d).rev != b.coedge(c).rev))
    });
    if seam_only {
        return (if face.sense { fw.min(rv) } else { fw.min(rv) ^ 0x9e37_79b9_7f4a_7c15 }, 1.0);
    }
    if fw <= rv {
        (fw, 1.0)
    } else {
        (rv, -1.0)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct MassProps {
    pub volume: f64,
    pub area: f64,
    pub com: Vec3,
    pub principal: [f64; 3],
    pub axes: [Vec3; 3],
    /// ∫ x dV about the world origin (= volume × com)
    pub first_moment: Vec3,
    /// inertia tensor about the centre of mass, world axes (rows)
    pub inertia: [[f64; 3]; 3],
    /// inertia tensor about the world origin, world axes (rows)
    pub inertia_origin: [[f64; 3]; 3],
}

impl MassProps {
    pub fn from_moments(m: &Moments) -> MassProps {
        let inertia = m.inertia();
        let (principal, axes) = eigen_sym(inertia);
        MassProps {
            volume: m.volume,
            area: m.area,
            com: m.com(),
            principal,
            axes,
            first_moment: m.about(Vec3::ZERO).first,
            inertia,
            inertia_origin: m.inertia_about(Vec3::ZERO),
        }
    }
}

pub fn mass_props(b: &Body) -> MassProps {
    let ms = lump_moments(b);
    let total = ms.iter().skip(1).fold(ms.first().copied().unwrap_or_default(), |a, m| a.add(m));
    MassProps::from_moments(&total)
}

/// The `massProps` reply `result` object. With `densities`, every `bodies[]`
/// entry also carries `mass` (= volume × density); without, the reply is the
/// oracle-pinned shape (no `mass` field). `byMass` totals are unchanged.
///
/// A body whose every lump is a sheet (open edges: no enclosed volume) answers `volume: 0`, its `area`, `com` = the
/// area centroid, and `kind: "surface"`; `principal`, `axes`, `firstMoment`, `inertia` and `inertiaOrigin` are then
/// area-weighted (surface density 1) instead of volume-weighted, and `mass` is 0. Callers that read only `volume`
/// or `area` are unaffected. (Before, `com` and `principal` were null.)
pub fn mass_props_json(b: &Body, bodies: bool, densities: Option<&[f64]>) -> Json {
    let lumps: Vec<LumpId> = b.lump_ids().collect();
    let surface = !lumps.is_empty() && lumps.iter().all(|&l| faces_are_open(b, &lump_faces(b, l)));
    let ms = if surface {
        let r = ref_point(b);
        lumps.iter().map(|&l| sheet_moments(b, &lump_faces(b, l), r)).collect()
    } else {
        lump_moments(b)
    };
    let vol = |m: &Moments| if surface { 0.0 } else { m.volume };
    let total = ms.iter().skip(1).fold(ms.first().copied().unwrap_or_default(), |a, m| a.add(m));
    let mp = MassProps::from_moments(&total);
    let v3 = crate::body::v3;
    let axes = |a: &[Vec3; 3]| Json::Arr(a.iter().map(|&x| v3(x)).collect());
    let mat = |m: &[[f64; 3]; 3]| Json::Arr(m.iter().map(|r| Json::Arr(r.iter().map(|&x| Json::Num(x)).collect())).collect());
    let mut o = Obj::new()
        .put("volume", if surface { 0.0 } else { mp.volume })
        .put("area", mp.area)
        .put("com", v3(mp.com))
        .put("principal", Json::Arr(mp.principal.iter().map(|&x| Json::Num(x)).collect()))
        .put("axes", axes(&mp.axes))
        .put("firstMoment", v3(mp.first_moment))
        .put("inertia", mat(&mp.inertia))
        .put("inertiaOrigin", mat(&mp.inertia_origin));
    if surface {
        o = o.put("kind", "surface");
    }
    if bodies {
        let names: Vec<Option<String>> = b.lump_ids().map(|l| b.lump(l).name.clone()).collect();
        let arr = ms
            .iter()
            .enumerate()
            .map(|(i, m)| {
                let mut e = Obj::new().put("volume", vol(m)).put("area", m.area).put("com", v3(m.com()));
                // `name` is the document's body name (null when the lump has none), `index` the lump order of this
                // reply (and of the mesh replies' `faceBody` / `triBody`)
                e = match names.get(i) {
                    Some(Some(n)) => e.put("name", n.as_str()),
                    _ => e.put("name", Json::Null),
                };
                e = e.put("index", i as f64);
                if let Some(ds) = densities {
                    e = e.put("mass", vol(m) * ds.get(i).copied().unwrap_or(1.0));
                }
                e.build()
            })
            .collect();
        o = o.put("bodies", Json::Arr(arr));
    }
    if let Some(ds) = densities {
        // GUESS: missing density entries count as 1.
        let mut acc: Option<Moments> = None;
        for (i, m) in ms.iter().enumerate() {
            let w = m.scaled(ds.get(i).copied().unwrap_or(1.0));
            acc = Some(match acc {
                None => w,
                Some(a) => a.add(&w),
            });
        }
        if let Some(a) = acc {
            let p = MassProps::from_moments(&a);
            o = o.put(
                "byMass",
                Obj::new()
                    .put("mass", vol(&a))
                    .put("com", v3(p.com))
                    .put("principal", Json::Arr(p.principal.iter().map(|&x| Json::Num(x)).collect()))
                    .put("axes", axes(&p.axes))
                    .put("firstMoment", v3(p.first_moment))
                    .put("inertia", mat(&p.inertia))
                    .put("inertiaOrigin", mat(&p.inertia_origin)),
            );
        }
    }
    o.build()
}

/// Symmetric 3×3 eigen-decomposition, decoded from the oracle (WAT f3939,
/// called by the massProps builder f3561): cyclic Jacobi over the pairs
/// (0,1), (0,2), (1,2), up to 64 sweeps, stopping only when
/// |a01|+|a02|+|a12| < 1e-300; a pair is skipped when |apq| < 1e-300;
/// θ = (aqq − app)/(apq + apq), t = sign(θ)/(|θ| + √(θ²+1)) (1 when θ = 0),
/// c = 1/√(t²+1), s = t·c; A ← JᵀAJ (columns, then rows), V ← VJ.
/// Eigenvalues sorted ascending (total order, stable); eigenvectors are the
/// columns of V scaled by 1/|v| — NO sign convention: the signs are those
/// the rotations produce (oracle rev90 / rev90flip axes
/// [.707,0,.707],[.707,0,−.707] / [−.707,0,.707],[.707,0,.707]). For
/// repeated moments (cylinder, sphere, torus) the result depends on the
/// rounding noise of the tensor (e.g. a 45° pair when two diagonal entries
/// are bit-equal), so it matches the oracle only as far as the integrated
/// tensor does.
pub fn eigen_sym(m: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
    let mut a = m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..64 {
        if a[0][1].abs() + a[0][2].abs() + a[1][2].abs() < 1e-300 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let apq = a[p][q];
            let theta = (a[q][q] - a[p][p]) / (apq + apq);
            let sg = if theta.is_nan() { f64::NAN } else { 1f64.copysign(theta) };
            let t = if theta == 0.0 { 1.0 } else { sg / (theta.abs() + (theta * theta + 1.0).sqrt()) };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for row in v.iter_mut() {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]));
    let mut vals = [0.0; 3];
    let mut vecs = [Vec3::ZERO; 3];
    for (k, &i) in idx.iter().enumerate() {
        vals[k] = a[i][i];
        vecs[k] = Vec3::new(v[0][i], v[1][i], v[2][i]).normalize();
    }
    (vals, vecs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::make::tests::{box_body, rect, top_frame};
    use crate::make::{extrude, revolve::revolve, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    use ck_geom::Frame;
    use std::f64::consts::{PI, TAU};

    /// A sheet (a box with one face deleted) has no volume: `volume` 0, the area centroid for `com`, no nulls.
    #[test]
    fn sheet_body_answers_zero_volume_and_the_area_centroid() {
        let mut b = box_body();
        let r = ref_point_of(&b);
        let full = sheet_moments(&b, &b.face_ids().collect::<Vec<_>>(), r);
        let f = b.face_ids().next().unwrap();
        let gone = sheet_moments(&b, &[f], r);
        crate::edit::delete_face(&mut b, f);
        let j = mass_props_json(&b, true, Some(&[2.0]));
        assert_eq!(j.get_f64("volume"), Some(0.0));
        assert_eq!(j.get_str("kind"), Some("surface"));
        let area = j.get_f64("area").unwrap();
        assert!(close(area, full.area - gone.area, 1e-9), "{area}");
        // area-weighted: open sheet + removed face = the closed surface, whose centroid is the box centre
        let c: Vec<f64> = j.get_arr("com").unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
        let want = (full.com() * full.area - gone.com() * gone.area) * (1.0 / area);
        assert!((c[0] - want.x).abs() < 1e-9 && (c[1] - want.y).abs() < 1e-9 && (c[2] - want.z).abs() < 1e-9, "{c:?} vs {want:?}");
        for k in ["principal", "axes", "firstMoment", "inertia", "inertiaOrigin"] {
            assert!(!j.get(k).unwrap().is_null(), "{k}");
        }
        let p: Vec<f64> = j.get_arr("principal").unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
        assert!(p.iter().all(|x| x.is_finite() && *x > 0.0), "{p:?}");
        let bodies = j.get_arr("bodies").unwrap();
        assert_eq!(bodies[0].get_f64("volume"), Some(0.0));
        assert_eq!(bodies[0].get_f64("mass"), Some(0.0));
        assert_eq!(j.get("byMass").unwrap().get_f64("mass"), Some(0.0));
        // a closed body is unchanged: no `kind`, positive volume
        let jc = mass_props_json(&box_body(), false, None);
        assert!(jc.get("kind").is_none() && jc.get_f64("volume").unwrap() > 0.0);
    }


    fn close(a: f64, b: f64, rel: f64) -> bool {
        (a - b).abs() <= rel * b.abs().max(1.0)
    }
    fn vclose(a: Vec3, b: [f64; 3], tol: f64) -> bool {
        (a.x - b[0]).abs() < tol && (a.y - b[1]).abs() < tol && (a.z - b[2]).abs() < tol
    }

    #[test]
    fn box_mass_matches_oracle() {
        // oracle: volume 8000, area 2800, com [20,5,-10],
        // principal [333333.33, 1133333.33, 1333333.33], axes [[1,0,0],[0,0,1],[0,1,0]]
        let b = box_body();
        let m = mass_props(&b);
        assert!(close(m.volume, 8000.0, 1e-13), "{}", m.volume);
        assert!(close(m.area, 2800.0, 1e-13));
        assert!(vclose(m.com, [20.0, 5.0, -10.0], 1e-12));
        assert!(close(m.principal[0], 333333.3333333342, 1e-12));
        assert!(close(m.principal[1], 1133333.333333336, 1e-12));
        assert!(close(m.principal[2], 1333333.3333333367, 1e-12));
        assert!(vclose(m.axes[0], [1.0, 0.0, 0.0], 1e-12));
        assert!(vclose(m.axes[1], [0.0, 0.0, 1.0], 1e-12));
        assert!(vclose(m.axes[2], [0.0, 1.0, 0.0], 1e-12));
        // byMass with aluminium 0.0027 -> mass 21.6, principal[0] 900
        let j = mass_props_json(&b, true, Some(&[0.0027]));
        let bm = j.get("byMass").unwrap();
        assert!(close(bm.get_f64("mass").unwrap(), 21.6, 1e-13));
        assert!(close(bm.get_arr("principal").unwrap()[0].as_f64().unwrap(), 900.0, 1e-12));
        let keys: Vec<_> = j.as_obj().unwrap().iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(keys, ["volume", "area", "com", "principal", "axes", "firstMoment", "inertia", "inertiaOrigin", "bodies", "byMass"]);
    }

    #[test]
    fn cylinder_and_holes_mass() {
        let prof = Profile::new(
            top_frame(),
            vec![ProfileRegion {
                outer: ProfileLoop::new(vec![ProfileSeg::circle("e7", Vec2::new(5.0, 3.0), 10.0)]),
                holes: vec![],
            }],
        );
        let b = extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap();
        let m = mass_props(&b);
        // oracle: 3141.592653589795, area 1256.6370614359173, com [5,5,-3], principal [104719.755.., .., 157079.63..]
        assert!(close(m.volume, 3141.592653589795, 1e-13), "{}", m.volume);
        assert!(close(m.area, 1256.6370614359173, 1e-13));
        assert!(vclose(m.com, [5.0, 5.0, -3.0], 1e-12));
        assert!(close(m.principal[0], 104719.75511965966, 1e-12), "{:?} {:?}", m.principal, m.axes);
        assert!(close(m.principal[2], 157079.63267948956, 1e-12));

        // box3: outer 40x20 with circle r4 @ (30,10) and 10x10 square hole, depth 10
        let fr = Frame::identity();
        let hole = rect(["h1", "h2", "h3", "h4"], 5.0, 5.0, 15.0, 15.0).reversed();
        let circ = ProfileLoop::new(vec![ProfileSeg::circle("e5", Vec2::new(30.0, 10.0), 4.0).reversed()]);
        let prof = Profile::new(fr, vec![ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 0.0, 0.0, 40.0, 20.0), holes: vec![circ, hole] }]);
        let b = extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap();
        let m = mass_props(&b);
        // oracle: 6497.345175425632, area 3150.79644737231, com [20.765459063659883,10,5],
        // principal [310467.2571635832, 956394.8009831763, 1158572.9718896654]
        assert!(close(m.volume, 6497.345175425632, 1e-13));
        assert!(close(m.area, 3150.79644737231, 1e-13));
        assert!(vclose(m.com, [20.765459063659883, 10.0, 5.0], 1e-11));
        assert!(close(m.principal[0], 310467.2571635832, 1e-12));
        assert!(close(m.principal[1], 956394.8009831763, 1e-12));
        assert!(close(m.principal[2], 1158572.9718896654, 1e-12));
    }

    #[test]
    fn draft_mass() {
        let prof = Profile::new(fr_id(), vec![ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 0.0, 0.0, 40.0, 20.0), holes: vec![] }]);
        let mut o = ExtrudeOpts::blind(10.0);
        o.draft = 10f64.to_radians();
        let b = extrude("F2", &prof, o).unwrap();
        let m = mass_props(&b);
        // oracle: volume 6983.493054573724, area 2547.7371956219913, com z 4.762349236064046
        assert!(close(m.volume, 6983.493054573724, 1e-10), "{}", m.volume);
        assert!(close(m.area, 2547.7371956219913, 1e-10), "{}", m.area);
        assert!((m.com.z - 4.762349236064046).abs() < 1e-9);
        assert!(close(m.principal[0], 253799.75274091944, 1e-9));
    }
    fn fr_id() -> Frame {
        Frame::identity()
    }

    #[test]
    fn revolve_mass_matches_oracle() {
        let rect_p = |x0: f64| {
            let p = [Vec2::new(x0, 0.0), Vec2::new(20.0, 0.0), Vec2::new(20.0, 5.0), Vec2::new(x0, 5.0)];
            let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
            Profile::new(Frame::identity(), vec![ProfileRegion { outer, holes: vec![] }])
        };
        // rev360 (10..20): 4712.3889803846905 area 2827.433388230816 principal [598866.09.., .., 1178097.245..]
        let m = mass_props(&revolve("F2", &rect_p(10.0), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        assert!(close(m.volume, 4712.3889803846905, 1e-13), "{}", m.volume);
        assert!(close(m.area, 2827.433388230816, 1e-13));
        assert!(close(m.principal[0], 598866.0995905546, 1e-12));
        assert!(close(m.principal[2], 1178097.2450961727, 1e-12));
        // rev90: 1178.0972450961717 area 806.8583470577036 com [9.902974236829046,2.5,-9.902974236829046]
        let m = mass_props(&revolve("F2", &rect_p(10.0), Vec3::ZERO, Vec3::Y, 0.0, PI / 2.0).unwrap());
        assert!(close(m.volume, 1178.0972450961717, 1e-13));
        assert!(close(m.area, 806.8583470577036, 1e-13));
        assert!(vclose(m.com, [9.902974236829046, 2.5, -9.902974236829046], 1e-11));
        // revax360 (solid cylinder): 6283.185307179589 area 3141.5926535897943
        let m = mass_props(&revolve("F2", &rect_p(0.0), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        assert!(close(m.volume, 6283.185307179589, 1e-13), "{}", m.volume);
        assert!(close(m.area, 3141.5926535897943, 1e-13));
        // revax90: 1570.7963267948962 area 985.3981633974479 principal [33998.42818517255, 60352.12.., 87805.56..]
        let m = mass_props(&revolve("F2", &rect_p(0.0), Vec3::ZERO, Vec3::Y, 0.0, PI / 2.0).unwrap());
        assert!(close(m.volume, 1570.7963267948962, 1e-13));
        assert!(close(m.area, 985.3981633974479, 1e-13));
        assert!(close(m.principal[0], 33998.42818517255, 1e-12));
        assert!(close(m.principal[2], 87805.56851717288, 1e-12));
    }

    #[test]
    fn sphere_torus_cone_mass() {
        let fr = Frame::identity();
        let semi = ProfileLoop::new(vec![
            ProfileSeg::arc("e1", Vec2::ZERO, Vec2::new(0.0, -10.0), Vec2::new(0.0, 10.0)),
            ProfileSeg::line("e2", Vec2::new(0.0, 10.0), Vec2::new(0.0, -10.0)),
        ]);
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![ProfileRegion { outer: semi, holes: vec![] }]), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        // oracle 4188.79020478639, 1256.6370614359178, principal 167551.6081914555
        assert!(close(m.volume, 4188.79020478639, 1e-13), "{}", m.volume);
        assert!(close(m.area, 1256.6370614359178, 1e-13));
        assert!(close(m.principal[0], 167551.60819145554, 1e-12));
        let tor = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(20.0, 0.0), 5.0)]);
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![ProfileRegion { outer: tor, holes: vec![] }]), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        // oracle 9869.604401089362, 3947.841760435745, principal [2128133.4489848926, .., 4132896.8429561704]
        assert!(close(m.volume, 9869.604401089362, 1e-13), "{}", m.volume);
        assert!(close(m.area, 3947.841760435745, 1e-13));
        assert!(close(m.principal[2], 4132896.8429561704, 1e-12));
        let tri = ProfileLoop::new(vec![
            ProfileSeg::line("e1", Vec2::new(5.0, 0.0), Vec2::new(15.0, 0.0)),
            ProfileSeg::line("e2", Vec2::new(15.0, 0.0), Vec2::new(5.0, 10.0)),
            ProfileSeg::line("e3", Vec2::new(5.0, 10.0), Vec2::new(5.0, 0.0)),
        ]);
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![ProfileRegion { outer: tri, holes: vec![] }]), Vec3::ZERO, Vec3::Y, 0.0, 120f64.to_radians()).unwrap());
        // oracle 872.664625997165, 710.3514612362036, com [3.721470044097097,3,-6.445775195221761]
        assert!(close(m.volume, 872.664625997165, 1e-13), "{}", m.volume);
        assert!(close(m.area, 710.3514612362036, 1e-13));
        assert!(vclose(m.com, [3.721470044097097, 3.0, -6.445775195221761], 1e-11));
        assert!(close(m.principal[1], 24886.35364447574, 1e-12));
        // oracle axes (raw Jacobi signs, brep probe `cone`)
        for (a, o) in m.axes.iter().zip([
            [0.8660254037844387, 1.6315768633458208e-16, 0.49999999999999983],
            [-0.3896325508076662, 0.6266944242646857, 0.6748633742815399],
            [0.3133472121323425, 0.7792651016153322, -0.542733291823281],
        ]) {
            assert!(vclose(*a, o, 1e-9), "{a:?} {o:?}");
        }
    }

    #[test]
    fn jacobi_signs_are_the_rotations() {
        // oracle rev90 / rev90flip (front rect (10,0)-(20,5) about +Y, ±90°):
        // the 45° pair comes out of the (0,2) rotation unsigned-normalised
        let fr = Frame::identity();
        let r = || ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 10.0, 0.0, 20.0, 5.0), holes: vec![] };
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![r()]), Vec3::ZERO, Vec3::Y, 0.0, PI / 2.0).unwrap());
        let h = std::f64::consts::FRAC_1_SQRT_2;
        assert!(vclose(m.axes[0], [h, 0.0, h], 1e-9) && vclose(m.axes[1], [h, 0.0, -h], 1e-9), "{:?}", m.axes);
        // a hand-made tensor: off-diagonal sign decides the rotation sign
        let (_, v) = eigen_sym([[2.0, 0.0, -0.5], [0.0, 5.0, 0.0], [-0.5, 0.0, 3.0]]);
        assert!(v[0].x > 0.0 && v[0].z > 0.0 && v[1].x < 0.0 && v[1].z > 0.0, "{v:?}");
    }

    #[test]
    fn d_shape_and_two_lumps() {
        let fr = Frame::identity();
        let d = ProfileLoop::new(vec![
            ProfileSeg::line("e1", Vec2::new(-10.0, 0.0), Vec2::new(10.0, 0.0)),
            ProfileSeg::arc("e2", Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(-10.0, 0.0)),
        ]);
        let m = mass_props(&extrude("F2", &Profile::new(fr, vec![ProfileRegion { outer: d, holes: vec![] }]), ExtrudeOpts::blind(5.0)).unwrap());
        // oracle 785.3981633974481, 571.238898038469, com y 4.244131815783879
        assert!(close(m.volume, 785.3981633974481, 1e-13));
        assert!(close(m.area, 571.238898038469, 1e-13));
        assert!((m.com.y - 4.244131815783879).abs() < 1e-12);
        let prof2 = Profile::new(
            fr,
            vec![
                ProfileRegion { outer: rect(["a0", "a1", "a2", "a3"], 0.0, 0.0, 10.0, 10.0), holes: vec![] },
                ProfileRegion { outer: rect(["b0", "b1", "b2", "b3"], 20.0, 0.0, 30.0, 10.0), holes: vec![] },
            ],
        );
        let b = extrude("F2", &prof2, ExtrudeOpts::blind(5.0)).unwrap();
        // no densities: oracle-pinned shape, no `mass` on bodies[]
        let j0 = mass_props_json(&b, true, None);
        let bodies0 = j0.get_arr("bodies").unwrap();
        assert_eq!(bodies0.len(), 2);
        assert!(bodies0[0].get("mass").is_none());
        assert!(j0.get("byMass").is_none());
        let j = mass_props_json(&b, true, Some(&[1.0, 2.0]));
        // oracle byMass mass 1500, com x 18.33333333333334, principal [15625, 148958.33, 158333.33]
        let bm = j.get("byMass").unwrap();
        assert!(close(bm.get_f64("mass").unwrap(), 1500.0, 1e-13));
        assert!((bm.get("com").unwrap().as_vec3().unwrap()[0] - 18.33333333333334).abs() < 1e-11);
        assert!(close(bm.get_arr("principal").unwrap()[1].as_f64().unwrap(), 148958.33333333308, 1e-12));
        let bodies = j.get_arr("bodies").unwrap();
        assert_eq!(bodies.len(), 2);
        assert!(close(bodies[1].get_f64("volume").unwrap(), 500.0, 1e-13));
        // per-body mass = volume × density (hand: 500×1 and 500×2)
        assert!(close(bodies[0].get_f64("mass").unwrap(), 500.0, 1e-13));
        assert!(close(bodies[1].get_f64("mass").unwrap(), 1000.0, 1e-13));
        // mass-weighted CG x: (500·5 + 1000·25) / 1500 = 55/3
        let cx = bm.get("com").unwrap().as_vec3().unwrap()[0];
        assert!((cx - 55.0 / 3.0).abs() < 1e-11, "{cx}");
    }

    // ---- inertia tensor, first moments (density 1) ----

    fn mat_close(a: &[[f64; 3]; 3], b: [[f64; 3]; 3], rel: f64, scale: f64) -> bool {
        (0..3).all(|i| (0..3).all(|j| (a[i][j] - b[i][j]).abs() <= rel * scale))
    }
    fn solid_circle(c: Vec2, r: f64) -> ProfileRegion {
        ProfileRegion { outer: ProfileLoop::new(vec![ProfileSeg::circle("e1", c, r)]), holes: vec![] }
    }

    #[test]
    fn box_tensor_first_moments_and_parallel_axis() {
        // box_body: 40 x 10 x 20, volume 8000, com [20, 5, -10]
        let m = mass_props(&box_body());
        let v = 8000.0;
        let d = [v * 500.0 / 12.0, v * 2000.0 / 12.0, v * 1700.0 / 12.0];
        assert!(mat_close(&m.inertia, [[d[0], 0.0, 0.0], [0.0, d[1], 0.0], [0.0, 0.0, d[2]]], 1e-12, d[1]), "{:?}", m.inertia);
        assert!(vclose(m.first_moment, [160000.0, 40000.0, -80000.0], 1e-8), "{:?}", m.first_moment);
        // parallel axis: I_o = I_com + V (|c|^2 E - c c^T), c = (20, 5, -10)
        let (cx, cy, cz) = (20.0, 5.0, -10.0);
        let c2 = cx * cx + cy * cy + cz * cz;
        let o = [
            [d[0] + v * (c2 - cx * cx), -v * cx * cy, -v * cx * cz],
            [-v * cx * cy, d[1] + v * (c2 - cy * cy), -v * cy * cz],
            [-v * cx * cz, -v * cy * cz, d[2] + v * (c2 - cz * cz)],
        ];
        assert!(mat_close(&m.inertia_origin, o, 1e-12, o[0][0]), "{:?}", m.inertia_origin);
        // the tensor and the eigen-pairs describe the same thing: I a_k = principal_k a_k
        for k in 0..3 {
            let a = [m.axes[k].x, m.axes[k].y, m.axes[k].z];
            for i in 0..3 {
                let ia: f64 = (0..3).map(|j| m.inertia[i][j] * a[j]).sum();
                assert!((ia - m.principal[k] * a[i]).abs() < 1e-6 * m.principal[2], "axis {k} row {i}");
            }
        }
    }

    #[test]
    fn cylinder_sphere_hollow_closed_forms() {
        // cylinder r 10, h 10 (axis along y in this frame)
        let prof = Profile::new(top_frame(), vec![solid_circle(Vec2::new(5.0, 3.0), 10.0)]);
        let m = mass_props(&extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap());
        let v = PI * 100.0 * 10.0;
        let (iax, itr) = (v * 100.0 / 2.0, v * (3.0 * 100.0 + 100.0) / 12.0);
        assert!(mat_close(&m.inertia, [[itr, 0.0, 0.0], [0.0, iax, 0.0], [0.0, 0.0, itr]], 1e-12, iax), "{:?}", m.inertia);
        // sphere r 10: 2/5 V r^2 on every diagonal entry, no products; centred on the origin, so both tensors agree
        let fr = Frame::identity();
        let semi = ProfileLoop::new(vec![
            ProfileSeg::arc("e1", Vec2::ZERO, Vec2::new(0.0, -10.0), Vec2::new(0.0, 10.0)),
            ProfileSeg::line("e2", Vec2::new(0.0, 10.0), Vec2::new(0.0, -10.0)),
        ]);
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![ProfileRegion { outer: semi, holes: vec![] }]), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        let i = 0.4 * (4.0 / 3.0 * PI * 1000.0) * 100.0;
        assert!(mat_close(&m.inertia, [[i, 0.0, 0.0], [0.0, i, 0.0], [0.0, 0.0, i]], 1e-12, i), "{:?}", m.inertia);
        assert!(mat_close(&m.inertia_origin, m.inertia, 1e-9, i), "{:?}", m.inertia_origin);
        assert!(vclose(m.first_moment, [0.0, 0.0, 0.0], 1e-8), "{:?}", m.first_moment);
        // hollow cylinder (through hole r 4): a hole removes mass, it never flips a sign
        let hole = ProfileLoop::new(vec![ProfileSeg::circle("e2", Vec2::new(5.0, 3.0), 4.0).reversed()]);
        let prof = Profile::new(top_frame(), vec![ProfileRegion { outer: ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(5.0, 3.0), 10.0)]), holes: vec![hole] }]);
        let m = mass_props(&extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap());
        let v = PI * (100.0 - 16.0) * 10.0;
        let (iax, itr) = (v * (100.0 + 16.0) / 2.0, v * (3.0 * (100.0 + 16.0) + 100.0) / 12.0);
        assert!(close(m.volume, v, 1e-13));
        assert!(mat_close(&m.inertia, [[itr, 0.0, 0.0], [0.0, iax, 0.0], [0.0, 0.0, itr]], 1e-12, iax), "{:?}", m.inertia);
        assert!(m.inertia[0][0] > 0.0 && m.inertia[1][1] > 0.0 && m.inertia[2][2] > 0.0);
        assert!(vclose(m.com, [5.0, 5.0, -3.0], 1e-12));
    }

    #[test]
    fn rotated_translated_box_principal_axes() {
        // 30 x 10 rectangle turned 30 degrees about z, moved off the origin, extruded 6
        let (a, b, c, th) = (30.0, 10.0, 6.0, 30f64.to_radians());
        let (ct, st) = (th.cos(), th.sin());
        let (ox, oy) = (17.0, -9.0);
        let p = |u: f64, w: f64| Vec2::new(ox + u * ct - w * st, oy + u * st + w * ct);
        let q = [p(-a / 2.0, -b / 2.0), p(a / 2.0, -b / 2.0), p(a / 2.0, b / 2.0), p(-a / 2.0, b / 2.0)];
        let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), q[i], q[(i + 1) % 4])).collect());
        let m = mass_props(&extrude("F2", &Profile::new(Frame::identity(), vec![ProfileRegion { outer, holes: vec![] }]), ExtrudeOpts::blind(c)).unwrap());
        let v = a * b * c;
        assert!(close(m.volume, v, 1e-12));
        // covariance of the box in world axes: S = R diag(a^2, b^2, c^2)/12 R^T
        let (sxx, syy, szz, sxy) = ((a * a * ct * ct + b * b * st * st) / 12.0, (a * a * st * st + b * b * ct * ct) / 12.0, c * c / 12.0, (a * a - b * b) * ct * st / 12.0);
        let want = [[v * (syy + szz), -v * sxy, 0.0], [-v * sxy, v * (sxx + szz), 0.0], [0.0, 0.0, v * (sxx + syy)]];
        assert!(mat_close(&m.inertia, want, 1e-11, want[2][2]), "{:?}\n{:?}", m.inertia, want);
        // about the origin: translation by the centre of mass adds V (|c|^2 E - c c^T)
        let cm = m.com;
        let c2 = cm.x * cm.x + cm.y * cm.y + cm.z * cm.z;
        let cc = [cm.x, cm.y, cm.z];
        let mut o = want;
        for i in 0..3 {
            for j in 0..3 {
                o[i][j] += v * ((if i == j { c2 } else { 0.0 }) - cc[i] * cc[j]);
            }
        }
        assert!(mat_close(&m.inertia_origin, o, 1e-11, o[0][0]), "{:?}\n{:?}", m.inertia_origin, o);
        // principal moments about the a, b, c axes of the box, ascending; axes are the turned frame
        let want_p = [v * (b * b + c * c) / 12.0, v * (a * a + c * c) / 12.0, v * (a * a + b * b) / 12.0];
        for k in 0..3 {
            assert!(close(m.principal[k], want_p[k], 1e-11), "{k}: {:?}", m.principal);
        }
        let dots = [m.axes[0].x * ct + m.axes[0].y * st, -m.axes[1].x * st + m.axes[1].y * ct, m.axes[2].z];
        for (k, d) in dots.iter().enumerate() {
            assert!((d.abs() - 1.0).abs() < 1e-9, "axis {k}: {:?}", m.axes);
        }
    }

    #[test]
    fn mass_json_carries_tensor_and_first_moment() {
        let b = box_body();
        let j = mass_props_json(&b, false, Some(&[0.0027]));
        let row = |j: &Json, k: &str, i: usize| -> Vec<f64> { j.get_arr(k).unwrap()[i].as_arr().unwrap().iter().map(|x| x.as_f64().unwrap()).collect() };
        let ixx = 8000.0 * 500.0 / 12.0;
        assert!(close(row(&j, "inertia", 0)[0], ixx, 1e-12));
        assert!(close(j.get_arr("firstMoment").unwrap()[0].as_f64().unwrap(), 160000.0, 1e-12));
        let bm = j.get("byMass").unwrap();
        // by mass: density 0.0027 scales volume-weighted numbers; first moment = mass * com
        assert!(close(row(bm, "inertia", 0)[0], ixx * 0.0027, 1e-12));
        assert!(close(bm.get_arr("firstMoment").unwrap()[0].as_f64().unwrap(), 160000.0 * 0.0027, 1e-12));
        assert!(close(row(bm, "inertiaOrigin", 0)[0], 8000.0 * 0.0027 * (500.0 / 12.0 + 125.0), 1e-12));
    }

    #[test]
    fn gauss_legendre_rules() {
        for (g, r) in GL.iter().zip(gauss_legendre(8)) {
            assert!((g.0 - r.0).abs() < 1e-14 && (g.1 - r.1).abs() < 1e-14, "{g:?} {r:?}");
        }
        // 5 points integrate x^8 exactly: 2/9
        let s: f64 = gauss_legendre(5).iter().map(|&(x, w)| w * x.powi(8)).sum();
        assert!((s - 2.0 / 9.0).abs() < 1e-15, "{s}");
        let s: f64 = gauss_legendre(2).iter().map(|&(x, w)| w * x * x).sum();
        assert!((s - 2.0 / 3.0).abs() < 1e-15, "{s}");
    }
}
