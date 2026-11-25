//! The tool body of a `wrap` feature (see wrap.rs): a slab laid on a cylinder, built by hand as faces.
//!
//! The profile is given in the development of the cylinder, coordinates `(X, Z)`: `X` the arc length along the
//! circumferential direction (increasing angle about the axis, measured at the face radius), `Z` the position along the axis.
//! Every loop arrives in canonical order: outer loops counter-clockwise in `(X, Z)`, holes clockwise, so the loop turns
//! counter-clockwise about the radial direction `rho` (`(theta, axis, rho)` is right handed).
//!
//! The slab of one region lies between two cylinders about the axis, `r_base` and `r_top`, and its walls are ruled surfaces
//! between a base rail (at `r_base`) and a top rail (at `r_top`) that share their parameterisation, so corresponding points lie
//! on one ruling:
//! * an edge parallel to the axis (untapered): a ruling of the cylinder; the wall is a radial plane;
//! * an edge of constant `Z` (untapered): an arc of a circle; the wall is the plane `Z = const`;
//! * any other edge: both rails are cubic B-splines through the same number of points of the mapped curve (the number grows
//!   until the rails stay within a quarter of the tolerance of the exact map between the points, at most [`MAX_POINTS`]); the wall
//!   is the ruled surface of degree (3, 1) between them.
//!
//! The caps are the two cylinders, trimmed by the loops. One lump per region.

use crate::regen::FeatureError;
use ck_core::linalg::{Frame, Vec3};
use ck_geom::curve::Curve;
use ck_geom::fit;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_json::Json;
use ck_topo::body::{Body, EdgeId, FaceName, VertexId};
use std::f64::consts::PI;

/// Most points a fitted rail interpolates (the proposal's 400 control point cap).
pub const MAX_POINTS: usize = 400;

pub const WRAP_TOOL_INVALID: &str = "wrapToolInvalid";

fn invalid(detail: String) -> FeatureError {
    FeatureError::new(WRAP_TOOL_INVALID).with("detail", Json::Str(detail))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Parallel to the axis (`X` constant).
    Axial,
    /// Constant `Z`.
    Circ,
    Fit,
}

/// `lambda in [0, 1]` -> (base rail point, top rail point) in the development `(X, Z)`.
pub type DevFn = Box<dyn Fn(f64) -> ([f64; 2], [f64; 2])>;

pub struct EdgeIn {
    pub tag: String,
    pub kind: Kind,
    pub f: DevFn,
}

pub struct LoopIn {
    pub edges: Vec<EdgeIn>,
}

pub struct RegionIn {
    pub outer: LoopIn,
    pub holes: Vec<LoopIn>,
}

/// The cylinder the tool is laid on. Angles `theta` are measured from `ex` about `a` (`ey = a x ex`).
#[derive(Debug, Clone, Copy)]
pub struct Geo {
    pub o: Vec3,
    pub a: Vec3,
    pub ex: Vec3,
    pub ey: Vec3,
    /// Face radius: the development is isometric at this radius.
    pub r: f64,
    /// Angle (about the axis) of the middle of the tool: the seam of the tool's cylinders sits opposite.
    pub th_c: f64,
}

impl Geo {
    pub fn dir(&self, th: f64) -> Vec3 {
        self.ex * th.cos() + self.ey * th.sin()
    }
    pub fn pt(&self, rad: f64, d: [f64; 2]) -> Vec3 {
        self.o + self.a * d[1] + self.dir(d[0] / self.r) * rad
    }
    /// Angle in the tool's cylinder frame, in (0, 2 pi) for a point within a half turn of the middle.
    pub fn u(&self, x: f64) -> f64 {
        x / self.r - self.th_c + PI
    }
    pub fn frame(&self) -> Frame {
        let x = self.dir(self.th_c - PI);
        Frame { o: self.o, x, y: self.a.cross(x), z: self.a }
    }
}

pub struct Out {
    pub body: Body,
    /// Worst distance between a rail and the exact map, over all fitted edges.
    pub deviation: f64,
    /// Set when a fit stopped at [`MAX_POINTS`] above a quarter of the tolerance: the worst deviation seen.
    pub approximated: Option<f64>,
}

/// One edge of the tool's base rail and top rail, with the wall between them.
struct Built {
    base: EdgeId,
    base_rev: bool,
    top: EdgeId,
    top_rev: bool,
    wall: Surface,
    /// Corners of the wall quad: the base start and end, the top end and start.
    quad: [Vec3; 4],
    n_out: Vec3,
    /// The wall surface's own normal is the outward one (a plane is built so; a ruled surface is measured).
    planar: bool,
}

fn newell(p: &[Vec3]) -> Vec3 {
    let mut a = Vec3::new(0.0, 0.0, 0.0);
    for i in 0..p.len() {
        let (u, v) = (p[i], p[(i + 1) % p.len()]);
        a = a + Vec3::new((u.y - v.y) * (u.z + v.z), (u.z - v.z) * (u.x + v.x), (u.x - v.x) * (u.y + v.y));
    }
    a
}

struct Stats {
    deviation: f64,
    approximated: Option<f64>,
}

/// Interpolating rails through `n` points of the exact map; the deviation between the points measured at three places per span.
fn fit_rails(geo: &Geo, f: &DevFn, r_base: f64, r_top: f64, n: usize) -> Result<(BSplineCurve, BSplineCurve, f64), String> {
    let lams: Vec<f64> = (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
    let (mut pb, mut pt) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for &l in &lams {
        let (b, t) = f(l);
        pb.push(geo.pt(r_base, b));
        pt.push(geo.pt(r_top, t));
    }
    let u = fit::params(&pb, fit::Param::ChordLength);
    if !(u[n - 1] > 0.0) || u.windows(2).any(|w| !(w[1] > w[0])) {
        return Err("a rail has coincident points".into());
    }
    let knots = fit::averaged_knots(&u, 3);
    let ctrl = fit::interpolate_with_many(&[pb, pt], 3, &u, &knots)?;
    let cb = BSplineCurve::new(3, knots.clone(), ctrl[0].clone())?;
    let ct = BSplineCurve::new(3, knots, ctrl[1].clone())?;
    let mut dev = 0.0f64;
    for w in lams.windows(2) {
        for q in [0.25, 0.5, 0.75] {
            let l = w[0] + (w[1] - w[0]) * q;
            let (b, t) = f(l);
            let (qb, qt) = (geo.pt(r_base, b), geo.pt(r_top, t));
            dev = dev.max(cb.closest_point(qb).1.dist(qb)).max(ct.closest_point(qt).1.dist(qt));
        }
    }
    Ok((cb, ct, dev))
}

fn fit_edge(geo: &Geo, e: &EdgeIn, r_base: f64, r_top: f64, tol: f64, st: &mut Stats) -> Result<(BSplineCurve, BSplineCurve, BSplineSurface), FeatureError> {
    let mut n = 8usize;
    loop {
        let (cb, ct, dev) = fit_rails(geo, &e.f, r_base, r_top, n).map_err(|m| invalid(format!("edge {}: {m}", e.tag)))?;
        if dev <= tol / 4.0 || n >= MAX_POINTS {
            st.deviation = st.deviation.max(dev);
            if dev > tol / 4.0 {
                st.approximated = Some(st.approximated.unwrap_or(0.0).max(dev));
            }
            // control net: index i * 2 + j, i along the rail, j from the base (0) to the top (1)
            let mut ctrl = Vec::with_capacity(2 * n);
            for i in 0..n {
                ctrl.push(cb.ctrl[i]);
                ctrl.push(ct.ctrl[i]);
            }
            let surf = BSplineSurface::new(3, 1, cb.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], n, 2, ctrl, None).map_err(|m| invalid(format!("edge {}: {m}", e.tag)))?;
            return Ok((cb, ct, surf));
        }
        n = (n * 2).min(MAX_POINTS);
    }
}

/// Build the tool. `base_exposed`: the base cap is the one that stays in the result (a cut's floor, a bore's add top).
pub fn build(feature: &str, geo: &Geo, regions: &[RegionIn], r_base: f64, r_top: f64, tol: f64) -> Result<Out, FeatureError> {
    let mut body = Body::new();
    let mut st = Stats { deviation: 0.0, approximated: None };
    let multi = regions.len() > 1;
    let hsign = if r_top > r_base { 1.0 } else { -1.0 };
    // the base cap is the low-radius one when the top is further out
    let base_low = hsign > 0.0;
    let fr = geo.frame();
    for (ri, reg) in regions.iter().enumerate() {
        let lump = body.add_lump();
        let shell = body.add_shell(lump);
        let rtag = if multi { Some(ri as u32 + 1) } else { None };
        // exposed cap: the one `depth` away from the face; named C0, the other C1
        let (name_base, name_top) = (FaceName::cap(feature, 0, rtag), FaceName::cap(feature, 1, rtag));
        let cyl = |r: f64| Surface::Cylinder { frame: fr, r };
        // the high-radius cap looks outward (+rho), the low one inwards
        let f_base = body.add_face(shell, cyl(r_base), !base_low, name_base);
        let f_top = body.add_face(shell, cyl(r_top), base_low, name_top);
        for (li, lp_in) in std::iter::once(&reg.outer).chain(reg.holes.iter()).enumerate() {
            let m = lp_in.edges.len();
            if m < 2 {
                return Err(invalid("a loop with fewer than two edges".into()));
            }
            let (mut bv, mut tv): (Vec<VertexId>, Vec<VertexId>) = (vec![], vec![]);
            let (mut bp, mut tp): (Vec<Vec3>, Vec<Vec3>) = (vec![], vec![]);
            for e in &lp_in.edges {
                let (b, t) = (e.f)(0.0);
                let (pb, pt) = (geo.pt(r_base, b), geo.pt(r_top, t));
                bp.push(pb);
                tp.push(pt);
                bv.push(body.add_vertex(pb));
                tv.push(body.add_vertex(pt));
            }
            let rails: Vec<EdgeId> = (0..m).map(|k| body.add_edge(Curve::line_through(bp[k], tp[k]), 0.0, bp[k].dist(tp[k]), bv[k], tv[k])).collect();
            let mut built: Vec<Built> = vec![];
            for (k, e) in lp_in.edges.iter().enumerate() {
                let j = (k + 1) % m;
                let (db0, dt0) = (e.f)(0.0);
                let (db1, dt1) = (e.f)(1.0);
                // outward direction of the wall: right of the direction of travel, in (theta, axis)
                let right = |tx: f64, tz: f64, x: f64| -> Vec3 {
                    let th = x / geo.r;
                    let (tx, tz) = (tx, tz);
                    let l = tx.hypot(tz).max(1e-300);
                    geo.dir(th + PI / 2.0) * (tz / l) - geo.a * (tx / l)
                };
                let quad = [bp[k], bp[j], tp[j], tp[k]];
                let b = match e.kind {
                    Kind::Axial => {
                        let n_out = right(0.0, db1[1] - db0[1], db0[0]);
                        let x = geo.a;
                        let z = n_out;
                        let wall = Surface::Plane { frame: Frame { o: bp[k], x, y: z.cross(x), z } };
                        let base = body.add_edge(Curve::line_through(bp[k], bp[j]), 0.0, bp[k].dist(bp[j]), bv[k], bv[j]);
                        let top = body.add_edge(Curve::line_through(tp[k], tp[j]), 0.0, tp[k].dist(tp[j]), tv[k], tv[j]);
                        Built { base, base_rev: false, top, top_rev: false, wall, quad, n_out, planar: true }
                    }
                    Kind::Circ => {
                        let n_out = right(db1[0] - db0[0], 0.0, db0[0]);
                        let (ua, ub) = (geo.u(db0[0]), geo.u(db1[0]));
                        let radial = geo.dir(db0[0] / geo.r);
                        let wall = Surface::Plane { frame: Frame { o: bp[k], x: radial, y: n_out.cross(radial), z: n_out } };
                        let ctr = |r: f64, z: f64| Curve::circle_x(geo.o + geo.a * z, geo.a, fr.x, r);
                        let (lo, hi, rev) = if ub > ua { (ua, ub, false) } else { (ub, ua, true) };
                        let (v0b, v1b, v0t, v1t) = if rev { (bv[j], bv[k], tv[j], tv[k]) } else { (bv[k], bv[j], tv[k], tv[j]) };
                        let base = body.add_edge(ctr(r_base, db0[1]), lo, hi, v0b, v1b);
                        let top = body.add_edge(ctr(r_top, dt0[1]), lo, hi, v0t, v1t);
                        let _ = (dt0, dt1);
                        Built { base, base_rev: rev, top, top_rev: rev, wall, quad, n_out, planar: true }
                    }
                    Kind::Fit => {
                        let (cb, ct, surf) = fit_edge(geo, e, r_base, r_top, tol, &mut st)?;
                        // the middle of the wall: direction of travel from the exact map
                        let (a, b) = ((e.f)(0.49).0, (e.f)(0.51).0);
                        let n_out = right(b[0] - a[0], b[1] - a[1], (e.f)(0.5).0[0]);
                        let base = body.add_edge(Curve::BSpline(cb), 0.0, 1.0, bv[k], bv[j]);
                        let top = body.add_edge(Curve::BSpline(ct), 0.0, 1.0, tv[k], tv[j]);
                        Built { base, base_rev: false, top, top_rev: false, wall: Surface::BSpline(Box::new(surf)), quad, n_out, planar: false }
                    }
                };
                built.push(b);
            }
            // caps: the canonical loop (outer counter-clockwise, holes clockwise in (X, Z)) turns counter-clockwise about +rho
            let cap_loop = |face, edges: Vec<(EdgeId, bool)>, forward: bool, body: &mut Body| {
                let lp = body.add_loop(face);
                if forward {
                    for (e, r) in edges {
                        body.add_coedge(lp, e, r, None);
                    }
                } else {
                    for (e, r) in edges.into_iter().rev() {
                        body.add_coedge(lp, e, !r, None);
                    }
                }
            };
            cap_loop(f_base, built.iter().map(|b| (b.base, b.base_rev)).collect(), !base_low, &mut body);
            cap_loop(f_top, built.iter().map(|b| (b.top, b.top_rev)).collect(), base_low, &mut body);
            // walls
            for (k, b) in built.iter().enumerate() {
                let j = (k + 1) % m;
                let e = &lp_in.edges[k];
                // [base forward, rail up at the end, top backward, rail down at the start]: counter-clockwise about the
                // surface's own normal (a plane's normal is built outward)
                let mut list = vec![(b.base, b.base_rev), (rails[j], false), (b.top, !b.top_rev), (rails[k], true)];
                let (sense, forward) = if b.planar {
                    (true, newell(&b.quad).dot(b.n_out) > 0.0)
                } else {
                    let Surface::BSpline(s) = &b.wall else { unreachable!() };
                    let nat = s.normal(0.5, 0.5);
                    let sense = nat.dot(b.n_out) > 0.0;
                    (sense, true)
                };
                if !forward {
                    list = list.into_iter().rev().map(|(e, r)| (e, !r)).collect();
                }
                if !b.planar && !sense {
                    list = list.into_iter().rev().map(|(e, r)| (e, !r)).collect();
                }
                let face = body.add_face(shell, b.wall.clone(), sense, FaceName::side(feature, &e.tag));
                let lp = body.add_loop(face);
                for (e, r) in list {
                    body.add_coedge(lp, e, r, None);
                }
            }
            let _ = li;
        }
    }
    body.number_split_faces();
    let (_, vol) = ck_topo::orient::orient_body_volume(&mut body);
    if !(vol > 0.0) {
        return Err(invalid(format!("the tool has volume {vol}")));
    }
    if let Some(i) = ck_topo::check::check_topology(&body, 1e-6).iter().find(|i| !matches!(i, ck_topo::check::Issue::VertexOffCurve(..))) {
        return Err(invalid(format!("the tool does not close into a manifold solid ({i:?})")));
    }
    Ok(Out { body, deviation: st.deviation, approximated: st.approximated })
}
