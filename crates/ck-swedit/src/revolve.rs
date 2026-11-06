//! Revolve features (boss and cut), read back from the faces the feature left on the final body.
//!
//! Every face of a revolve is a surface of revolution about one axis (plane normal to it, cylinder, cone, sphere,
//! torus) or, for a partial angle, one of the two planar end caps that contain the axis. The axis comes from the
//! curved faces; the profile is the chain of the faces' generatrices in the half plane (s along the axis, rho the
//! distance from it). Each generatrix runs between the extremes of the face's boundary points, so a face a later
//! feature notched keeps its full extent. Where the chain is open (a cut that ran through the body's surface, a
//! boss that stands on earlier material) it is closed the way the kit's own revolve needs it: a cut is extended
//! past the surface along its end tangents, then the profile is closed either directly or through the axis,
//! whichever puts the region on the side the faces' outward normals point to (cut) or away from (boss). A partial
//! angle takes the sketch plane from the start cap and the sweep from the angle between the two caps.
//!
//! A face on a stored surface of revolution (a spline profile spun about an axis) gives its generatrix from the
//! profile itself, cut down to the face's boundary and read as a line, an arc or a `cvspline`. Faces that later
//! features split around the axis (a pattern of cuts, a channel across a face) lie on one generatrix and are read
//! as one piece. Where a later feature took a corner off (a chamfer, a step), two lines whose carriers meet just
//! past both open ends are run on to their corner.
//!
//! This reads the faces itself rather than going through `ck_recog::revolve::analyze`: that one needs every face
//! of the body, circular edges only and a closed chain, none of which holds for a feature in the middle of a
//! history. The rebuild is still checked against the import by the caller.

use crate::{face_points, Cx, TOL};
use ck_geom::curve::Curve;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_json::Obj;
use ck_recog::emit::{arr, n, s as js, Sk};
use ck_recog::model::{dist_to_line, par, principal, Kind, Model, PlaneSpec};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::{EdgeId, FaceId};
use std::f64::consts::{PI, TAU};

type P = (f64, f64);

/// A generatrix in the (s, rho) half plane.
#[derive(Debug, Clone)]
enum Gen {
    Line { a: P, b: P },
    /// Counter-clockwise (s towards rho) arc about `c`: `p(chi) = c + r (cos chi, sin chi)`.
    Arc { c: P, r: f64, a0: f64, span: f64 },
    Circle { c: P, r: f64 },
    /// A spline generatrix (the profile of a surface of revolution), in (s, rho, 0).
    Spline(BSplineCurve),
}

struct Prim {
    g: Gen,
    /// The faces the generatrix belongs to (several where later features split one face of the revolve around the
    /// axis); empty for a piece that bridges a gap a later feature left.
    faces: Vec<FaceId>,
    /// Outward normal of the face at the generatrix' middle, in (s, rho).
    nrm: P,
}

/// A profile segment in travel order.
#[derive(Debug, Clone)]
struct Seg {
    a: P,
    b: P,
    /// Arc about `c` with radius `r`, from angle `a0` over the signed `span`.
    arc: Option<(P, f64, f64, f64)>,
    /// A spline in (s, rho, 0) from `a` to `b`.
    spl: Option<BSplineCurve>,
    faces: Vec<FaceId>,
}

fn v2(p: Vec3) -> P {
    (p.x, p.y)
}

fn v3(p: P) -> Vec3 {
    Vec3::new(p.0, p.1, 0.0)
}

fn arc_pt(c: P, r: f64, chi: f64) -> P {
    (c.0 + r * chi.cos(), c.1 + r * chi.sin())
}

fn d2(a: P, b: P) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

impl Seg {
    fn line(a: P, b: P, face: Option<FaceId>) -> Seg {
        Seg { a, b, arc: None, spl: None, faces: face.into_iter().collect() }
    }
    fn mid(&self) -> P {
        if let Some(c) = &self.spl {
            let (u0, u1) = c.domain();
            return v2(c.eval((u0 + u1) / 2.0));
        }
        match self.arc {
            Some((c, r, a0, sp)) => arc_pt(c, r, a0 + sp / 2.0),
            None => ((self.a.0 + self.b.0) / 2.0, (self.a.1 + self.b.1) / 2.0),
        }
    }
    fn samples(&self) -> Vec<P> {
        if let Some(c) = &self.spl {
            return c.sample(48).into_iter().map(v2).collect();
        }
        match self.arc {
            Some((c, r, a0, sp)) => (0..=24).map(|i| arc_pt(c, r, a0 + sp * i as f64 / 24.0)).collect(),
            None => vec![self.a, self.b],
        }
    }
    /// Unit tangent at the end of travel.
    fn tangent_end(&self) -> P {
        if let Some(c) = &self.spl {
            return v2(c.tangent(c.domain().1));
        }
        match self.arc {
            Some((_, _, a0, sp)) => {
                let t = a0 + sp;
                let s = sp.signum();
                (-t.sin() * s, t.cos() * s)
            }
            None => unit((self.b.0 - self.a.0, self.b.1 - self.a.1)),
        }
    }
    fn tangent_start(&self) -> P {
        if let Some(c) = &self.spl {
            return v2(c.tangent(c.domain().0));
        }
        match self.arc {
            Some((_, _, a0, sp)) => {
                let s = sp.signum();
                (-a0.sin() * s, a0.cos() * s)
            }
            None => self.tangent_end(),
        }
    }
    fn reversed(&self) -> Seg {
        Seg { a: self.b, b: self.a, arc: self.arc.map(|(c, r, a0, sp)| (c, r, a0 + sp, -sp)), spl: self.spl.as_ref().map(|c| c.reversed()), faces: self.faces.clone() }
    }
}

fn unit(v: P) -> P {
    let l = (v.0 * v.0 + v.1 * v.1).sqrt().max(1e-300);
    (v.0 / l, v.1 / l)
}

/// Even-odd point in polygon.
fn inside(poly: &[P], p: P) -> bool {
    let mut c = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            c = !c;
        }
    }
    c
}

fn area(poly: &[P]) -> f64 {
    let mut s = 0.0;
    for i in 0..poly.len() {
        let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
        s += p.0 * q.1 - q.0 * p.1;
    }
    s / 2.0
}

fn wrap(t: f64) -> f64 {
    t.rem_euclid(TAU)
}

/// The axis: a point and a unit direction shared by every face of the feature.
fn find_axis(m: &Model, faces: &[FaceId], l: f64, sphere_axis: Option<Vec3>) -> Result<(Vec3, Vec3, Vec<FaceId>), String> {
    let mut ax: Option<(Vec3, Vec3)> = None;
    for &f in faces {
        match m.kind(f) {
            Kind::Cyl { o, a, .. } | Kind::Cone { o, a, .. } => ax = ax.or(Some((*o, *a))),
            Kind::Torus { c, a, .. } => ax = ax.or(Some((*c, *a))),
            _ => {}
        }
    }
    // a surface of revolution about its own axis (a spline profile: the file stores the swept curve)
    for &f in faces {
        if let Some((o, d)) = rev_axis(m, f) {
            ax = ax.or(Some((o, d)));
        }
    }
    if ax.is_none() {
        // spheres only (and planes): the poles go along the caller's axis (a dome: its base face's normal), else
        // along +Y like the kit's own ball
        if let Some(c) = faces.iter().find_map(|&f| if let Kind::Sphere { c, .. } = m.kind(f) { Some(*c) } else { None }) {
            ax = Some((c, sphere_axis.unwrap_or(Vec3::Y)));
        }
    }
    let (ao, mut a) = ax.ok_or("no surface of revolution among the feature's faces")?;
    a = a.normalize();
    if let Some((_, sg)) = principal(a) {
        if sg < 0.0 {
            a = -a;
        }
    }
    let tol = TOL * (1.0 + l);
    let mut caps = vec![];
    for &f in faces {
        let ok = match m.kind(f) {
            Kind::Cyl { o, a: ca, .. } | Kind::Cone { o, a: ca, .. } => par(*ca, a) && dist_to_line(*o, ao, a) < tol,
            Kind::Torus { c, a: ta, .. } => par(*ta, a) && dist_to_line(*c, ao, a) < tol,
            Kind::Sphere { c, .. } => dist_to_line(*c, ao, a) < tol,
            Kind::Plane { n, d } => {
                if par(*n, a) {
                    true
                } else if n.dot(a).abs() < 1e-7 && (n.dot(ao) - d).abs() < tol {
                    caps.push(f);
                    true
                } else {
                    false
                }
            }
            Kind::Other(_) => rev_axis(m, f).map_or(false, |(o, d)| par(d, a) && dist_to_line(o, ao, a) < tol),
        };
        if !ok {
            return Err(format!("face {} ({}) is not about the axis", m.b.face_id_str(f), m.kind(f).name()));
        }
    }
    Ok((ao, a, caps))
}

/// Axis of a face on a surface of revolution (a profile curve spun about an axis).
fn rev_axis(m: &Model, f: FaceId) -> Option<(Vec3, Vec3)> {
    match &m.b.face(f).surface {
        Surface::Revolution { axis_o, axis, .. } if matches!(m.kind(f), Kind::Other(_)) => Some((*axis_o, axis.normalize())),
        _ => None,
    }
}

struct Ax {
    ao: Vec3,
    a: Vec3,
}

impl Ax {
    /// (s, rho, unit radial direction) of a point.
    fn sr(&self, p: Vec3) -> (f64, f64, Vec3) {
        let v = p - self.ao;
        let s = v.dot(self.a);
        let r = v - self.a * s;
        let rho = r.len();
        (s, rho, if rho > 1e-12 { r * (1.0 / rho) } else { Vec3::ZERO })
    }
}

fn edge_points(m: &Model, e: EdgeId, k: usize) -> Vec<Vec3> {
    let ed = m.b.edge(e);
    ed.curve.sample(ed.t0, ed.t1, k)
}

/// Boundary points of a face as (s, rho), with the 3D points.
fn boundary(m: &Model, ax: &Ax, f: FaceId) -> Vec<(P, Vec3)> {
    let mut out = vec![];
    for e in m.edges(f) {
        for p in edge_points(m, e, 16) {
            let (s, rho, _) = ax.sr(p);
            out.push(((s, rho), p));
        }
    }
    out
}

/// Radial interval of a planar face normal to the axis.
fn disc_interval(m: &Model, ax: &Ax, f: FaceId, tol: f64) -> (f64, f64) {
    struct L {
        rmin: f64,
        rmax: f64,
        axis_circle: bool,
    }
    let mut loops: Vec<L> = vec![];
    for &l in &m.b.face(f).loops {
        let es: Vec<EdgeId> = m.b.lp(l).coedges.iter().map(|&c| m.b.coedge(c).edge).filter(|&e| !m.b.is_seam(e)).collect();
        let (mut rmin, mut rmax) = (f64::INFINITY, 0.0f64);
        for &e in &es {
            for p in edge_points(m, e, 16) {
                let (_, rho, _) = ax.sr(p);
                rmin = rmin.min(rho);
                rmax = rmax.max(rho);
            }
        }
        // (a circle about the axis, whole or split into arcs)
        let about = |e: EdgeId| matches!(m.curve(e), Curve::Circle { frame, .. } if par(frame.z, ax.a) && dist_to_line(frame.o, ax.ao, ax.a) < tol);
        let axis_circle = !es.is_empty() && es.iter().all(|&e| about(e)) && rmax - rmin < tol;
        if !es.is_empty() {
            loops.push(L { rmin, rmax, axis_circle });
        }
    }
    let Some(outer) = loops.iter().max_by(|x, y| x.rmax.total_cmp(&y.rmax)) else { return (0.0, 0.0) };
    if outer.axis_circle {
        let inner = loops.iter().filter(|l| l.axis_circle && l.rmax < outer.rmax - tol).map(|l| l.rmax).fold(f64::INFINITY, f64::min);
        (if inner.is_finite() { inner } else { 0.0 }, outer.rmax)
    } else {
        (outer.rmin, outer.rmax)
    }
}

/// The outward normal of face `f` at the 3D point `p` as (n.axis, n.radial).
fn normal_sr(m: &Model, ax: &Ax, f: FaceId, p: Vec3) -> P {
    let (u, v, q) = m.b.face(f).surface.closest_point(p);
    let nrm = m.b.outward_normal(f, u, v);
    let (_, _, dir) = ax.sr(q);
    (nrm.dot(ax.a), nrm.dot(dir))
}

/// The generatrix of lateral faces on one surface (one face, or the pieces later features split it into around the
/// axis). `frac` = sweep / 2 pi (1 for a full revolve).
fn generatrix(m: &Model, ax: &Ax, fs: &[FaceId], frac: f64, tol: f64) -> Result<Prim, String> {
    let f = fs[0];
    let mut pts = vec![];
    let mut owner = vec![];
    for &g in fs {
        let b = boundary(m, ax, g);
        owner.extend(std::iter::repeat(g).take(b.len()));
        pts.extend(b);
    }
    // a boundary point off the axis, for the sense of the normal
    let probe = pts.iter().zip(&owner).find(|((q, _), _)| q.1 > 10.0 * tol).map(|((_, p), g)| (*p, *g));
    let sense = |mid: P, c: P| -> P {
        // normal at `mid` of an arc about `c`: radial in the generatrix plane, outward = the side the normal at a
        // boundary point faces
        let r = unit((mid.0 - c.0, mid.1 - c.1));
        let sg = match probe {
            Some((p, g)) => {
                let nn = normal_sr(m, ax, g, p);
                let (ps, prho, _) = ax.sr(p);
                let rq = (ps - c.0, prho - c.1);
                if nn.0 * rq.0 + nn.1 * rq.1 >= 0.0 {
                    1.0
                } else {
                    -1.0
                }
            }
            None => 1.0,
        };
        (r.0 * sg, r.1 * sg)
    };
    let line_normal = || -> Result<P, String> {
        let (p, g) = probe.ok_or("no boundary point off the axis")?;
        Ok(normal_sr(m, ax, g, p))
    };
    let name = m.b.face_id_str(f);
    match m.kind(f).clone() {
        Kind::Plane { n: pn, d } => {
            let s = (d - pn.dot(ax.ao)) / pn.dot(ax.a);
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for &g in fs {
                let (l, h) = disc_interval(m, ax, g, tol);
                lo = lo.min(l);
                hi = hi.max(h);
            }
            if hi - lo < tol {
                return Err(format!("disc {name} has no radial extent"));
            }
            Ok(Prim { g: Gen::Line { a: (s, lo), b: (s, hi) }, faces: fs.to_vec(), nrm: (pn.dot(ax.a), 0.0) })
        }
        Kind::Cyl { r, .. } => {
            let (lo, hi) = s_range(&pts).ok_or_else(|| format!("cylinder {name} has no boundary"))?;
            if hi - lo < tol {
                return Err(format!("cylinder {name} has no axial extent"));
            }
            Ok(Prim { g: Gen::Line { a: (lo, r), b: (hi, r) }, faces: fs.to_vec(), nrm: line_normal()? })
        }
        Kind::Cone { o, a: ca, r0, half, .. } => {
            let (lo, hi) = s_range(&pts).ok_or_else(|| format!("cone {name} has no boundary"))?;
            let rho_at = |s: f64| -> f64 {
                let v: Vec<f64> = pts.iter().filter(|(q, _)| (q.0 - s).abs() < tol).map(|(q, _)| q.1).collect();
                v.iter().sum::<f64>() / v.len() as f64
            };
            let (rlo, rhi) = (pts.iter().map(|(q, _)| q.1).fold(f64::INFINITY, f64::min), pts.iter().map(|(q, _)| q.1).fold(f64::NEG_INFINITY, f64::max));
            let (a, b) = if hi - lo < tol && rhi - rlo > tol {
                // a cone nearly normal to the axis (a slight draft on a disc): the ends are told by the radius
                let at = |rho: f64| pts.iter().min_by(|x, y| (x.0 .1 - rho).abs().total_cmp(&(y.0 .1 - rho).abs())).unwrap().0;
                (at(rlo), at(rhi))
            } else if hi - lo < tol {
                let v = -r0 / half.tan();
                let sa = (o + ca * v - ax.ao).dot(ax.a);
                ((lo, rho_at(lo)), (sa, 0.0))
            } else {
                ((lo, rho_at(lo)), (hi, rho_at(hi)))
            };
            let nrm = line_normal()?;
            Ok(Prim { g: Gen::Line { a, b }, faces: fs.to_vec(), nrm })
        }
        Kind::Sphere { c, r, .. } => {
            let sc = (c - ax.ao).dot(ax.a);
            let psi = |q: P| q.1.atan2(q.0 - sc);
            let (lo, hi) = if pts.is_empty() {
                (0.0, PI)
            } else {
                let (slo, shi) = s_range(&pts).unwrap();
                if shi - slo < tol {
                    // one circle: the cap on one side of it, told by the area
                    let p1 = psi(pts[0].0);
                    let cap = TAU * r * r * (1.0 - p1.cos()) * frac;
                    let a: f64 = fs.iter().map(|&g| m.area(g)).sum();
                    let rest = 2.0 * TAU * r * r * frac - cap;
                    // (a hemisphere: both sides have the same area; the face's own points tell the side)
                    let side = if (cap - rest).abs() < 1e-6 * cap {
                        let (mut above, mut below) = (0usize, 0usize);
                        for q in fs.iter().flat_map(|&g| face_points(m, g)) {
                            let (qs, qr, _) = ax.sr(q);
                            let d = psi((qs, qr)) - p1;
                            if d > 1e-6 {
                                above += 1;
                            } else if d < -1e-6 {
                                below += 1;
                            }
                        }
                        Some(below >= above)
                    } else {
                        None
                    };
                    if side.unwrap_or_else(|| (a - cap).abs() <= (a - rest).abs()) {
                        (0.0, p1)
                    } else {
                        (p1, PI)
                    }
                } else {
                    let v: Vec<f64> = pts.iter().map(|(q, _)| psi(*q)).collect();
                    (v.iter().cloned().fold(f64::INFINITY, f64::min), v.iter().cloned().fold(f64::NEG_INFINITY, f64::max))
                }
            };
            let cc = (sc, 0.0);
            let mid = arc_pt(cc, r, (lo + hi) / 2.0);
            Ok(Prim { g: Gen::Arc { c: cc, r, a0: lo, span: hi - lo }, faces: fs.to_vec(), nrm: sense(mid, cc) })
        }
        Kind::Torus { c, major, minor, .. } => {
            let cc = ((c - ax.ao).dot(ax.a), major);
            if pts.is_empty() {
                return Ok(Prim { g: Gen::Circle { c: cc, r: minor }, faces: fs.to_vec(), nrm: sense((cc.0 + minor, cc.1), cc) });
            }
            let chi = |q: P| (q.1 - cc.1).atan2(q.0 - cc.0);
            let mut v: Vec<f64> = pts.iter().map(|(q, _)| wrap(chi(*q))).collect();
            v.sort_by(|a, b| a.total_cmp(b));
            v.dedup_by(|a, b| (*a - *b).abs() < 1e-7);
            let (a0, span) = if v.len() == 1 {
                return Err(format!("torus {name} bounded by a single circle"));
            } else if v.len() == 2 {
                // two circles: the short or the long way round, told by the area
                let (c1, c2) = (v[0], v[1]);
                let full = |a0: f64, sp: f64| TAU * minor * (major * sp - minor * ((a0 + sp).cos() - a0.cos())) * frac;
                let (s1, s2) = (c2 - c1, TAU - (c2 - c1));
                let a: f64 = fs.iter().map(|&g| m.area(g)).sum();
                if (full(c1, s1) - a).abs() <= (full(c2, s2) - a).abs() {
                    (c1, s1)
                } else {
                    (c2, s2)
                }
            } else {
                // the largest gap is the part of the circle the face does not cover
                let mut best = (0.0, 0usize);
                for i in 0..v.len() {
                    let g = wrap(v[(i + 1) % v.len()] - v[i]);
                    if g > best.0 {
                        best = (g, i);
                    }
                }
                if best.0 < 1e-3 {
                    return Ok(Prim { g: Gen::Circle { c: cc, r: minor }, faces: fs.to_vec(), nrm: (1.0, 0.0) });
                }
                let start = v[(best.1 + 1) % v.len()];
                (start, TAU - best.0)
            };
            let mid = arc_pt(cc, minor, a0 + span / 2.0);
            Ok(Prim { g: Gen::Arc { c: cc, r: minor, a0, span }, faces: fs.to_vec(), nrm: sense(mid, cc) })
        }
        Kind::Other(k) => match &m.b.face(f).surface {
            Surface::Revolution { profile, .. } => {
                let (p, g) = probe.ok_or("no boundary point off the axis")?;
                let (_, _, dir) = ax.sr(p);
                let gen = spun(ax, profile, &pts, tol).map_err(|e| format!("face {name}: {e}"))?;
                let nrm = match &gen {
                    Gen::Line { .. } => normal_sr(m, ax, g, p),
                    Gen::Arc { c, r, a0, span } => sense(arc_pt(*c, *r, a0 + span / 2.0), *c),
                    Gen::Spline(c) => {
                        let (u0, u1) = c.domain();
                        let q = v2(c.eval((u0 + u1) / 2.0));
                        normal_sr(m, ax, g, ax.ao + ax.a * q.0 + dir * q.1)
                    }
                    Gen::Circle { .. } => unreachable!(),
                };
                Ok(Prim { g: gen, faces: fs.to_vec(), nrm })
            }
            _ => Err(format!("face {name} is a {k}")),
        },
    }
}

/// The generatrix of a face on a surface of revolution: the spun profile curve in its own half plane, cut down to the
/// extent of the face's boundary points, and read as a line or an arc where it is one (a spline otherwise).
fn spun(ax: &Ax, profile: &Curve, pts: &[(P, Vec3)], tol: f64) -> Result<Gen, String> {
    let Curve::BSpline(c) = profile else { return Err("a surface of revolution with a non-spline profile".into()) };
    // the profile's half plane: through the axis and its control point farthest from it
    let off = |p: Vec3| {
        let v = p - ax.ao;
        v - ax.a * v.dot(ax.a)
    };
    let far = c.ctrl.iter().map(|&p| off(p)).max_by(|x, y| x.len().total_cmp(&y.len())).ok_or("empty profile")?;
    if far.len() < tol {
        return Err("the profile lies on the axis".into());
    }
    let dp = far.normalize();
    let side = ax.a.cross(dp);
    let mut ctrl = vec![];
    for &p in &c.ctrl {
        let r = off(p);
        // (a control point off the half plane: no planar profile through the axis)
        if r.dot(side).abs() > tol || r.dot(dp) < -tol {
            return Err("the profile is not in a half plane through the axis".into());
        }
        ctrl.push(Vec3::new((p - ax.ao).dot(ax.a), r.dot(dp).max(0.0), 0.0));
    }
    let c2 = BSplineCurve { degree: c.degree, knots: c.knots.clone(), ctrl, weights: c.weights.clone() };
    let (d0, d1) = c2.domain();
    let (mut u0, mut u1) = (f64::INFINITY, f64::NEG_INFINITY);
    for (q, _) in pts {
        let u = c2.closest_param(v3(*q));
        if v2(c2.eval(u)).0.is_nan() || d2(v2(c2.eval(u)), *q) > 100.0 * tol {
            return Err("a boundary point off the profile".into());
        }
        u0 = u0.min(u);
        u1 = u1.max(u);
    }
    if !(u1 > u0) {
        return Err("no extent along the profile".into());
    }
    let eps = 1e-9 * (d1 - d0);
    let cut = c2.segment(if u0 - d0 < eps { d0 } else { u0 }, if d1 - u1 < eps { d1 } else { u1 });
    let smp: Vec<P> = cut.sample(32).into_iter().map(v2).collect();
    let (a, b, mid) = (smp[0], smp[32], smp[16]);
    if d2(a, b) < tol {
        return Err("the profile piece is closed or has no length".into());
    }
    // a line?
    let t = unit((b.0 - a.0, b.1 - a.1));
    if smp.iter().all(|q| ((q.0 - a.0) * t.1 - (q.1 - a.1) * t.0).abs() < tol) {
        return Ok(Gen::Line { a, b });
    }
    // an arc? (circumcircle of the ends and the middle)
    let (ax_, ay, bx, by, cx_, cy) = (a.0, a.1, mid.0, mid.1, b.0, b.1);
    let dd = 2.0 * (ax_ * (by - cy) + bx * (cy - ay) + cx_ * (ay - by));
    if dd.abs() > 1e-12 {
        let sq = |x: f64, y: f64| x * x + y * y;
        let ux = (sq(ax_, ay) * (by - cy) + sq(bx, by) * (cy - ay) + sq(cx_, cy) * (ay - by)) / dd;
        let uy = (sq(ax_, ay) * (cx_ - bx) + sq(bx, by) * (ax_ - cx_) + sq(cx_, cy) * (bx - ax_)) / dd;
        let cc = (ux, uy);
        let r = d2(cc, a);
        if r < 1e6 * (1.0 + d2(a, b)) && smp.iter().all(|q| (d2(*q, cc) - r).abs() < tol) {
            let ang = |q: P| (q.1 - cc.1).atan2(q.0 - cc.0);
            let (ca, cm, cb) = (ang(a), ang(mid), ang(b));
            let ccw = wrap(cb - ca);
            let span = if wrap(cm - ca) < ccw { ccw } else { ccw - TAU };
            return Ok(Gen::Arc { c: cc, r, a0: ca, span });
        }
    }
    if cut.weights.as_ref().map_or(false, |w| w.iter().any(|x| (x - 1.0).abs() > 1e-12)) {
        return Err("a rational spline profile that is no line or arc".into());
    }
    Ok(Gen::Spline(cut))
}

fn s_range(pts: &[(P, Vec3)]) -> Option<(f64, f64)> {
    if pts.is_empty() {
        return None;
    }
    Some((pts.iter().map(|(q, _)| q.0).fold(f64::INFINITY, f64::min), pts.iter().map(|(q, _)| q.0).fold(f64::NEG_INFINITY, f64::max)))
}

/// Whether `q` lies on the generatrix piece `g`.
fn on_gen(g: &Gen, q: P, tol: f64) -> bool {
    match g {
        Gen::Line { a, b } => {
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let l2 = dx * dx + dy * dy;
            let t = (((q.0 - a.0) * dx + (q.1 - a.1) * dy) / l2.max(1e-300)).clamp(0.0, 1.0);
            d2(q, (a.0 + t * dx, a.1 + t * dy)) < tol
        }
        Gen::Arc { c, r, a0, span } => {
            if (d2(q, *c) - r).abs() > tol {
                return false;
            }
            let chi = (q.1 - c.1).atan2(q.0 - c.0);
            let slack = tol / r.max(1e-300);
            let rel = if *span >= 0.0 { wrap(chi - a0 + slack) } else { wrap(a0 - chi + slack) };
            rel <= span.abs() + 2.0 * slack
        }
        Gen::Spline(c) => d2(v2(c.eval(c.closest_param(v3(q)))), q) < tol,
        Gen::Circle { .. } => false,
    }
}

/// Later features (a pattern of cuts around the axis, a split line) cut one face of the revolve into several, each
/// on the same surface: their generatrices overlap. Every set of overlapping generatrices on one surface is read
/// again as one, over all the pieces' boundaries.
fn join_split(m: &Model, ax: &Ax, prims: Vec<Prim>, frac: f64, tol: f64) -> Result<Vec<Prim>, String> {
    let n = prims.len();
    let inner = |p: &Prim| -> Vec<P> {
        let sm = seg_of(p, false).samples();
        let k = sm.len() - 1;
        if k < 2 {
            let (a, b) = (sm[0], sm[k]);
            return [0.25, 0.5, 0.75].iter().map(|t| (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1))).collect();
        }
        vec![sm[k / 4], sm[k / 2], sm[3 * k / 4]]
    };
    let same_kind = |x: &Gen, y: &Gen| std::mem::discriminant(x) == std::mem::discriminant(y);
    let mut root: Vec<usize> = (0..n).collect();
    fn find(r: &mut Vec<usize>, i: usize) -> usize {
        let mut i = i;
        while r[i] != i {
            r[i] = r[r[i]];
            i = r[i];
        }
        i
    }
    let pts: Vec<Vec<P>> = prims.iter().map(|p| if matches!(p.g, Gen::Circle { .. }) { vec![] } else { inner(p) }).collect();
    let mut any = false;
    for i in 0..n {
        for j in i + 1..n {
            if !same_kind(&prims[i].g, &prims[j].g) || pts[i].is_empty() || pts[j].is_empty() {
                continue;
            }
            // the same carrier: every interior sample of one on the other, either way round (one piece notched
            // more than the other), or overlapping ends
            let ij = pts[j].iter().filter(|q| on_gen(&prims[i].g, **q, tol)).count();
            let ji = pts[i].iter().filter(|q| on_gen(&prims[j].g, **q, tol)).count();
            if ij + ji >= 2 {
                let (ri, rj) = (find(&mut root, i), find(&mut root, j));
                if ri != rj {
                    root[rj] = ri;
                    any = true;
                }
            }
        }
    }
    if !any {
        return Ok(prims);
    }
    let mut groups: Vec<(usize, Vec<FaceId>)> = vec![];
    for i in 0..n {
        let r = find(&mut root, i);
        match groups.iter_mut().find(|g| g.0 == r) {
            Some(g) => g.1.extend(prims[i].faces.iter().copied()),
            None => groups.push((r, prims[i].faces.clone())),
        }
    }
    let mut out = vec![];
    for (r, fs) in groups {
        if fs.len() == prims[r].faces.len() {
            out.push(Prim { g: prims[r].g.clone(), faces: fs, nrm: prims[r].nrm });
        } else {
            out.push(generatrix(m, ax, &fs, frac, tol)?);
        }
    }
    Ok(out)
}

fn ends_of(g: &Gen) -> (P, P) {
    match g {
        Gen::Line { a, b } => (*a, *b),
        Gen::Arc { c, r, a0, span } => (arc_pt(*c, *r, *a0), arc_pt(*c, *r, a0 + span)),
        Gen::Circle { c, r } => ((c.0 + r, c.1), (c.0 + r, c.1)),
        Gen::Spline(c) => (v2(c.start()), v2(c.end())),
    }
}

fn seg_of(p: &Prim, rev: bool) -> Seg {
    let s = match &p.g {
        Gen::Line { a, b } => Seg { a: *a, b: *b, arc: None, spl: None, faces: p.faces.clone() },
        Gen::Arc { c, r, a0, span } => {
            let (a, b) = ends_of(&p.g);
            Seg { a, b, arc: Some((*c, *r, *a0, *span)), spl: None, faces: p.faces.clone() }
        }
        Gen::Spline(c) => {
            let (a, b) = ends_of(&p.g);
            Seg { a, b, arc: None, spl: Some(c.clone()), faces: p.faces.clone() }
        }
        Gen::Circle { .. } => unreachable!(),
    };
    if rev {
        s.reversed()
    } else {
        s
    }
}

struct Graph {
    nodes: Vec<P>,
    links: Vec<(usize, usize)>,
    deg: Vec<usize>,
}

fn graph(prims: &[Prim], tol: f64) -> Graph {
    let mut nodes: Vec<P> = vec![];
    let mut links = vec![];
    for p in prims {
        let (a, b) = ends_of(&p.g);
        let mut node = |q: P| -> usize {
            if let Some(i) = nodes.iter().position(|x| d2(*x, q) < tol) {
                return i;
            }
            nodes.push(q);
            nodes.len() - 1
        };
        let (i, j) = (node(a), node(b));
        links.push((i, j));
    }
    let mut deg = vec![0usize; nodes.len()];
    for l in &links {
        deg[l.0] += 1;
        deg[l.1] += 1;
    }
    Graph { nodes, links, deg }
}

/// Unit tangent of a generatrix at its start (`at_end` false) or end, pointing away from the generatrix.
fn tangent_out(g: &Gen, at_end: bool) -> P {
    let s = match g {
        Gen::Line { a, b } => {
            let t = unit((b.0 - a.0, b.1 - a.1));
            if at_end {
                t
            } else {
                (-t.0, -t.1)
            }
        }
        Gen::Arc { a0, span, .. } => {
            let sg = span.signum();
            let chi = if at_end { a0 + span } else { *a0 };
            let t = (-chi.sin() * sg, chi.cos() * sg);
            if at_end {
                t
            } else {
                (-t.0, -t.1)
            }
        }
        Gen::Circle { .. } => (0.0, 0.0),
        Gen::Spline(c) => {
            let (u0, u1) = c.domain();
            let t = v2(c.tangent(if at_end { u1 } else { u0 }));
            if at_end {
                t
            } else {
                (-t.0, -t.1)
            }
        }
    };
    s
}

/// A later feature can leave a gap in the profile (a groove cut across a cylinder, a bore through a disc). Where
/// more than two ends are open: join two ends of one underlying surface (same line, same circle) across the gap,
/// then run what is left to the axis along its tangent.
fn fill_gaps(prims: &mut Vec<Prim>, tol: f64) {
    let ends_now = |prims: &[Prim]| -> Vec<(usize, bool, P)> {
        // (prim, at_end, point) of every open end
        let g = graph(prims, tol);
        let mut out = vec![];
        for (i, p) in prims.iter().enumerate() {
            let (a, b) = ends_of(&p.g);
            if g.deg[g.links[i].0] == 1 {
                out.push((i, false, a));
            }
            if g.deg[g.links[i].1] == 1 {
                out.push((i, true, b));
            }
        }
        out
    };
    let mut ends = ends_now(prims);
    if ends.len() <= 2 {
        return;
    }
    // 1: bridges between ends of one surface, nearest first
    let mut cand: Vec<(f64, usize, usize)> = vec![];
    for i in 0..ends.len() {
        for j in i + 1..ends.len() {
            let (ei, ej) = (&ends[i], &ends[j]);
            if ei.0 == ej.0 || prims[ei.0].faces.is_empty() || prims[ej.0].faces.is_empty() {
                continue;
            }
            let same = match (&prims[ei.0].g, &prims[ej.0].g) {
                (Gen::Line { a, b }, Gen::Line { a: c, b: d }) => {
                    let t = unit((b.0 - a.0, b.1 - a.1));
                    let u = unit((d.0 - c.0, d.1 - c.1));
                    let off = |q: P| ((q.0 - a.0) * t.1 - (q.1 - a.1) * t.0).abs();
                    (t.0 * u.1 - t.1 * u.0).abs() < 1e-7 && off(ej.2) < tol && off(*c) < tol && off(*d) < tol
                }
                (Gen::Arc { c, r, .. }, Gen::Arc { c: c2, r: r2, .. }) => d2(*c, *c2) < tol && (r - r2).abs() < tol,
                _ => false,
            };
            if same && d2(ei.2, ej.2) > tol {
                cand.push((d2(ei.2, ej.2), i, j));
            }
        }
    }
    cand.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut taken = vec![false; ends.len()];
    let mut add: Vec<Prim> = vec![];
    for (_, i, j) in cand {
        if taken[i] || taken[j] {
            continue;
        }
        taken[i] = true;
        taken[j] = true;
        let (ei, ej) = (ends[i].clone(), ends[j].clone());
        let g = match &prims[ei.0].g {
            Gen::Line { .. } => Gen::Line { a: ei.2, b: ej.2 },
            Gen::Arc { c, r, .. } => {
                let (ci, cj) = ((ei.2 .1 - c.1).atan2(ei.2 .0 - c.0), (ej.2 .1 - c.1).atan2(ej.2 .0 - c.0));
                let sg = match &prims[ei.0].g {
                    Gen::Arc { span, .. } => span.signum(),
                    _ => 1.0,
                };
                // continue past the end of the piece in its own sense of travel (or backwards from its start)
                let ccw = (sg > 0.0) == ei.1;
                let span = if ccw { wrap(cj - ci) } else { -wrap(ci - cj) };
                Gen::Arc { c: *c, r: *r, a0: ci, span }
            }
            Gen::Circle { .. } | Gen::Spline(_) => continue,
        };
        add.push(Prim { g, faces: vec![], nrm: (0.0, 0.0) });
    }
    prims.extend(add);
    ends = ends_now(prims);
    if ends.len() <= 2 {
        return;
    }
    // 1b: a line whose carrier runs on to another open end (a slight draft that meets a round tangentially, the rest of
    // the face replaced by a later feature); 1c: two lines whose carriers meet just past both ends (a corner a later
    // chamfer or fillet took off). Nearest first; each end once.
    let is_line = |i: usize| matches!(prims[i].g, Gen::Line { .. }) && !prims[i].faces.is_empty();
    let mut cand: Vec<(f64, usize, usize, P)> = vec![];
    for i in 0..ends.len() {
        for j in 0..ends.len() {
            let (ei, ej) = (&ends[i], &ends[j]);
            if i == j || ei.0 == ej.0 || !is_line(ei.0) {
                continue;
            }
            let t = tangent_out(&prims[ei.0].g, ei.1);
            let w = (ej.2 .0 - ei.2 .0, ej.2 .1 - ei.2 .1);
            let along = w.0 * t.0 + w.1 * t.1;
            if along > tol && (w.0 * t.1 - w.1 * t.0).abs() < tol {
                cand.push((d2(ei.2, ej.2), i, j, ej.2));
                continue;
            }
            if j < i || !is_line(ej.0) {
                continue;
            }
            let u = tangent_out(&prims[ej.0].g, ej.1);
            let den = t.0 * u.1 - t.1 * u.0;
            if den.abs() < 1e-6 {
                continue;
            }
            // ei + a t = ej + b u
            let a_ = (w.0 * u.1 - w.1 * u.0) / den;
            let b_ = (w.0 * t.1 - w.1 * t.0) / den;
            let gap = d2(ei.2, ej.2);
            if a_ > tol && b_ > tol && a_ < 2.0 * gap && b_ < 2.0 * gap {
                cand.push((gap, i, j, (ei.2 .0 + a_ * t.0, ei.2 .1 + a_ * t.1)));
            }
        }
    }
    cand.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut taken = vec![false; ends.len()];
    let mut add: Vec<Prim> = vec![];
    for (_, i, j, x) in cand {
        if taken[i] || taken[j] {
            continue;
        }
        taken[i] = true;
        taken[j] = true;
        let (pi, pj) = (ends[i].2, ends[j].2);
        add.push(Prim { g: Gen::Line { a: pi, b: x }, faces: vec![], nrm: (0.0, 0.0) });
        if d2(x, pj) > tol {
            add.push(Prim { g: Gen::Line { a: x, b: pj }, faces: vec![], nrm: (0.0, 0.0) });
        }
    }
    prims.extend(add);
    ends = ends_now(prims);
    if ends.len() <= 2 {
        return;
    }
    // 2: lines running towards the axis are extended to it
    let mut add: Vec<Prim> = vec![];
    for (i, at_end, pt) in ends {
        if pt.1 < tol {
            continue;
        }
        let Gen::Line { .. } = prims[i].g else { continue };
        let t = tangent_out(&prims[i].g, at_end);
        if t.1 < -1e-9 {
            let k = pt.1 / -t.1;
            add.push(Prim { g: Gen::Line { a: pt, b: (pt.0 + t.0 * k, 0.0) }, faces: vec![], nrm: (0.0, 0.0) });
        }
    }
    prims.extend(add);
}

/// Order the generatrices into one chain. Returns the segments in travel order and whether the chain is closed.
fn chain(prims: &[Prim], tol: f64) -> Result<(Vec<Seg>, bool), String> {
    let Graph { nodes, links, deg } = graph(prims, tol);
    let _ = &nodes;
    if deg.iter().any(|&d| d > 2) {
        return Err("a profile point joins more than two faces".into());
    }
    let ends: Vec<usize> = (0..nodes.len()).filter(|&i| deg[i] == 1).collect();
    if !(ends.is_empty() || ends.len() == 2) {
        return Err(format!("the faces' generatrices form {} open ends", ends.len()));
    }
    let start = ends.first().copied().unwrap_or(links[0].0);
    let mut used = vec![false; prims.len()];
    let mut out = vec![];
    let mut cur = start;
    while let Some(i) = (0..prims.len()).find(|&i| !used[i] && (links[i].0 == cur || links[i].1 == cur)) {
        used[i] = true;
        let rev = links[i].1 == cur && links[i].0 != cur;
        out.push(seg_of(&prims[i], rev));
        cur = if rev { links[i].0 } else { links[i].1 };
    }
    if used.iter().any(|u| !u) {
        return Err("the faces' generatrices form several pieces".into());
    }
    Ok((out, ends.is_empty()))
}

/// Join consecutive segments that continue one another (one line, one circle): a bridge across a gap and the pieces
/// it joins are one sketch entity, as the original profile had it.
fn merge(segs: &mut Vec<Seg>, tol: f64) {
    let cont = |x: &Seg, y: &Seg| -> bool {
        if d2(x.b, y.a) > tol || x.spl.is_some() || y.spl.is_some() {
            return false;
        }
        match (x.arc, y.arc) {
            (None, None) => {
                let (t, u) = (unit((x.b.0 - x.a.0, x.b.1 - x.a.1)), unit((y.b.0 - y.a.0, y.b.1 - y.a.1)));
                (t.0 * u.1 - t.1 * u.0).abs() < 1e-9 && t.0 * u.0 + t.1 * u.1 > 0.0
            }
            (Some((c, r, _, sp)), Some((c2, r2, _, sp2))) => d2(c, c2) < tol && (r - r2).abs() < tol && sp * sp2 > 0.0,
            _ => false,
        }
    };
    loop {
        let n = segs.len();
        if n < 3 {
            return;
        }
        let Some(i) = (0..n).find(|&i| cont(&segs[i], &segs[(i + 1) % n])) else { return };
        let j = (i + 1) % n;
        let (x, y) = (segs[i].clone(), segs[j].clone());
        let arc = match (x.arc, y.arc) {
            (Some((c, r, a0, sp)), Some((_, _, _, sp2))) => Some((c, r, a0, sp + sp2)),
            _ => None,
        };
        let mut faces = x.faces.clone();
        faces.extend(y.faces.iter().copied());
        segs[i] = Seg { a: x.a, b: y.b, arc, spl: None, faces };
        segs.remove(j);
    }
}

/// Consecutive segments meet where the chain joined them (within `tol`); a spline's end is moved onto its neighbour's
/// so the sketch's pieces share their points (lines and arcs keep the points they were read with).
fn snap(segs: &mut [Seg], tol: f64) {
    let k = segs.len();
    for i in 0..k {
        let j = (i + 1) % k;
        if i == j || d2(segs[i].b, segs[j].a) > tol || d2(segs[i].b, segs[j].a) < 1e-12 {
            continue;
        }
        if let Some(c) = segs[j].spl.as_mut() {
            c.ctrl[0] = v3(segs[i].b);
            segs[j].a = segs[i].b;
        } else if let Some(c) = segs[i].spl.as_mut() {
            *c.ctrl.last_mut().unwrap() = v3(segs[j].a);
            segs[i].b = segs[j].a;
        }
    }
}

/// Read a revolve back from its faces and emit plane, sketch and revolve.
pub fn revolve(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool) -> Result<String, String> {
    revolve_with(cx, f, faces, cut, &RevOpts::default())
}

/// What a caller knows of a revolve that its faces do not say (`sweep.rs`).
#[derive(Debug, Clone, Default)]
pub struct RevOpts {
    /// The axis direction of a revolve made of spheres only (a dome: the normal of its base face).
    pub sphere_axis: Option<Vec3>,
    /// A revolve without end caps that spans less than a turn (an arc piece of a sweep path): a point of the half
    /// plane it starts from (where the next piece of the path takes over) and the angle it runs on past the faces'
    /// angular extent (where the tool leaves the material).
    pub start: Option<(Vec3, f64)>,
    /// Out: the normal of the sketch plane the profile was drawn in.
    pub normal: std::cell::Cell<Option<Vec3>>,
}

pub fn revolve_with(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool, opts: &RevOpts) -> Result<String, String> {
    let m = cx.m;
    let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    for v in m.b.vertex_ids() {
        let p = m.b.vertex(v).p;
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let l = (hi - lo).len().max(1.0);
    let tol = TOL * (1.0 + l);
    let (ao, mut a, caps) = find_axis(m, faces, l, opts.sphere_axis)?;
    let mut ax = Ax { ao, a };
    let lateral: Vec<FaceId> = faces.iter().copied().filter(|g| !caps.contains(g)).collect();
    if lateral.is_empty() {
        return Err("no lateral face".into());
    }
    // the half plane the sketch is drawn in, and the sweep
    let (u0, sweep, c0, c1): (Vec3, f64, Vec<FaceId>, Vec<FaceId>) = match caps.len() {
        0 if opts.start.is_some() => {
            let (p0, extra) = opts.start.unwrap();
            let (_, rho0, u) = ax.sr(p0);
            if rho0 < tol {
                return Err("the start of the partial revolve is on its axis".into());
            }
            // the faces' angular extent from the start half plane, on the side they lie (flip the axis for the other)
            let phis: Vec<f64> = lateral.iter().flat_map(|&g| face_points(m, g)).filter(|p| ax.sr(*p).1 > tol).map(|p| {
                let (_, _, d) = ax.sr(p);
                wrap(a.cross(u).dot(d).atan2(u.dot(d)))
            }).collect();
            let near0 = |t: f64| t < 1e-6 || t > TAU - 1e-6;
            let pos = phis.iter().filter(|&&t| !near0(t)).all(|&t| t < PI);
            let neg = phis.iter().filter(|&&t| !near0(t)).all(|&t| t > PI);
            if pos == neg {
                return Err("the faces of the partial revolve do not lie on one side of its start".into());
            }
            if neg {
                a = -a;
                ax = Ax { ao, a };
            }
            let span = phis.iter().map(|&t| if near0(t) { 0.0 } else if neg { TAU - t } else { t }).fold(0.0, f64::max);
            (u, (span + extra).min(TAU), vec![], vec![])
        }
        0 => {
            let u = match principal(a) {
                Some((ai, _)) => {
                    let spec = PlaneSpec::for_axis(if ai == 2 { 1 } else { 2 });
                    if par(spec.xs, a) {
                        spec.ys
                    } else {
                        spec.xs
                    }
                }
                None => {
                    let w = [Vec3::X, Vec3::Y, Vec3::Z];
                    let k = (0..3).min_by(|&i, &j| w[i].dot(a).abs().total_cmp(&w[j].dot(a).abs())).unwrap();
                    (w[k] - a * w[k].dot(a)).normalize()
                }
            };
            (u, TAU, vec![], vec![])
        }
        2 => {
            let dir = |g: FaceId| -> Result<Vec3, String> {
                let mut best: Option<(f64, Vec3)> = None;
                for p in face_points(m, g) {
                    let (_, rho, d) = ax.sr(p);
                    if best.map_or(true, |b| rho > b.0) {
                        best = Some((rho, d));
                    }
                }
                match best {
                    Some((rho, d)) if rho > tol => Ok(d),
                    _ => Err("a cap face has no extent off the axis".into()),
                }
            };
            let (ua, ub) = (dir(caps[0])?, dir(caps[1])?);
            let phi = |p: Vec3, u: Vec3| -> f64 {
                let (_, _, d) = ax.sr(p);
                wrap((a.cross(u)).dot(d).atan2(u.dot(d)))
            };
            let th = wrap((a.cross(ua)).dot(ub).atan2(ua.dot(ub)));
            if th < 1e-6 || th > TAU - 1e-6 {
                return Err("the two end caps coincide".into());
            }
            // which way round the material lies: lateral boundary points inside (0, sweep) from the start cap
            let mut score = [0i32; 2];
            for &g in &lateral {
                for p in face_points(m, g) {
                    let (_, rho, _) = ax.sr(p);
                    if rho < tol {
                        continue;
                    }
                    let e = 1e-6;
                    let (p0, p1) = (phi(p, ua), phi(p, ub));
                    if p0 > e && p0 < th - e {
                        score[0] += 1;
                    } else if p0 > th + e && p0 < TAU - e {
                        score[0] -= 1;
                    }
                    let t1 = TAU - th;
                    if p1 > e && p1 < t1 - e {
                        score[1] += 1;
                    } else if p1 > t1 + e && p1 < TAU - e {
                        score[1] -= 1;
                    }
                }
            }
            if score[0] == score[1] {
                return Err("cannot tell which way round the revolve sweeps".into());
            }
            if score[0] > score[1] {
                (ua, th, vec![caps[0]], vec![caps[1]])
            } else {
                (ub, TAU - th, vec![caps[1]], vec![caps[0]])
            }
        }
        k => return Err(format!("{k} planar faces contain the axis (expected 0 or 2 end caps)")),
    };
    let frac = sweep / TAU;
    let mut prims = vec![];
    for &g in &lateral {
        prims.push(generatrix(m, &ax, &[g], frac, tol)?);
    }
    let prims0 = prims.len();
    let mut prims = join_split(m, &ax, prims, frac, tol)?;
    let nprims = prims.len();
    // sketch frame: the plane through the axis and the start half plane
    let z = a.cross(u0).normalize();
    let fr = cx.frame(z, ao);
    opts.normal.set(Some(fr.z));
    let world = |p: P| ao + a * p.0 + u0 * p.1;
    let sk_xy = |p: P| fr.xy(world(p));

    // profile: the chain, closed
    let mut segs: Vec<Seg>;
    let mut circle: Option<(P, f64, FaceId)> = None;
    if let [Prim { g: Gen::Circle { c, r }, faces, .. }] = prims.as_slice() {
        circle = Some((*c, *r, faces[0]));
        segs = vec![];
    } else {
        if prims.iter().any(|p| matches!(p.g, Gen::Circle { .. })) {
            return Err("a full circle among several faces".into());
        }
        fill_gaps(&mut prims, tol);
        let (cs, closed) = chain(&prims, tol)?;
        segs = cs;
        if !closed {
            close_open(&mut segs, &prims, cut, l, tol)?;
        }
    }
    merge(&mut segs, tol);
    snap(&mut segs, tol);
    // the sketch: counter-clockwise in the sketch plane
    if circle.is_none() {
        let mut poly: Vec<P> = vec![];
        for s in &segs {
            poly.extend(s.samples().into_iter().map(sk_xy));
        }
        if area(&poly) < 0.0 {
            segs = segs.iter().rev().map(|s| s.reversed()).collect();
        }
    }
    let sid = cx.tree.ids.next();
    let mut sk = Sk::new(sid.clone(), fr.plane.clone());
    let mut pts: Vec<(P, String)> = vec![];
    let mut point = |sk: &mut Sk, p: P| -> String {
        let q = sk_xy(p);
        if let Some((_, id)) = pts.iter().find(|(r, _)| d2(*r, q) < 1e-9) {
            return id.clone();
        }
        let id = sk.point(q.0, q.1);
        pts.push((q, id.clone()));
        id
    };
    let mut named: Vec<(FaceId, String)> = vec![];
    let (mut smin, mut smax) = (f64::INFINITY, f64::NEG_INFINITY);
    if let Some((c, r, face)) = circle {
        let pc = point(&mut sk, c);
        let e = sk.circle(&pc, r);
        named.push((face, e));
        smin = c.0 - r;
        smax = c.0 + r;
    }
    for s in &segs {
        for q in s.samples() {
            smin = smin.min(q.0);
            smax = smax.max(q.0);
        }
        let e = match s.arc {
            None if s.spl.is_some() => {
                let c = s.spl.as_ref().unwrap();
                let k = c.ctrl.len();
                let ids: Vec<String> = (0..k)
                    .map(|i| {
                        if i == 0 {
                            point(&mut sk, s.a)
                        } else if i == k - 1 {
                            point(&mut sk, s.b)
                        } else {
                            let q = sk_xy(v2(c.ctrl[i]));
                            sk.point(q.0, q.1)
                        }
                    })
                    .collect();
                let id = format!("cv{}", sk.entities.len() + 1);
                sk.entities.push(
                    Obj::new()
                        .put("type", "cvspline")
                        .put("id", id.as_str())
                        .put("pts", arr(ids.iter().map(|x| js(x)).collect()))
                        .put("degree", n(c.degree as f64))
                        .put("knots", arr(c.knots.iter().map(|&k| n(k)).collect()))
                        .build(),
                );
                id
            }
            None => {
                let (pa, pb) = (point(&mut sk, s.a), point(&mut sk, s.b));
                sk.line(&pa, &pb, false)
            }
            Some((c, _, _, _)) => {
                let (pa, pb, pc) = (point(&mut sk, s.a), point(&mut sk, s.b), point(&mut sk, c));
                let (qa, qc, qm) = (sk_xy(s.a), sk_xy(c), sk_xy(s.mid()));
                let ccw = (qa.0 - qc.0) * (qm.1 - qc.1) - (qa.1 - qc.1) * (qm.0 - qc.0) > 0.0;
                if ccw {
                    sk.arc(&pc, &pa, &pb)
                } else {
                    sk.arc(&pc, &pb, &pa)
                }
            }
        };
        for &fc in &s.faces {
            named.push((fc, e.clone()));
        }
    }
    if smax - smin < 1e-6 {
        smin -= 1.0;
        smax += 1.0;
    }
    let a0 = point(&mut sk, (smin, 0.0));
    let a1 = point(&mut sk, (smax, 0.0));
    let axis = sk.line(&a0, &a1, true);
    cx.tree.push(sk.json());
    let rid = cx.tree.ids.next();
    let angle = sweep.to_degrees();
    let o = Obj::new().put("type", "revolve").put("id", rid.as_str()).put("op", if cut { "cut" } else { "add" }).put("sketch", sid.as_str()).put("axis", axis.as_str()).put("angle", n(angle));
    cx.tree.push(o.build());
    for (fc, e) in named {
        cx.names.insert(fc, format!("{rid}.S.{e}"));
    }
    for g in c0 {
        cx.names.insert(g, format!("{rid}.C0"));
    }
    for g in c1 {
        cx.names.insert(g, format!("{rid}.C1"));
    }
    let dims: Vec<String> = f.dims.iter().map(|d| format!("{} = {}", d.name, d.text)).collect();
    let joined = if nprims < prims0 { format!(" ({prims0} pieces on {nprims} surfaces)") } else { String::new() };
    Ok(format!("{} about axis ({:.3},{:.3},{:.3}); {} face(s){joined}, angle {:.4}{}{}", if cut { "cut" } else { "boss" }, a.x, a.y, a.z, lateral.len(), angle, if caps.is_empty() { "" } else { ", end caps" }, if dims.is_empty() { String::new() } else { format!(" (file lists {})", dims.join(", ")) }))
}

/// Close an open chain: extend a cut past the surface it meets, then join the ends directly or through the axis,
/// on the side the faces' normals call for.
fn close_open(segs: &mut Vec<Seg>, prims: &[Prim], cut: bool, l: f64, tol: f64) -> Result<(), String> {
    let first = segs.first().unwrap().clone();
    let last = segs.last().unwrap().clone();
    let mg = if cut { 0.05 * l + 1.0 } else { 0.0 };
    let (a_pt, b_pt) = (first.a, last.b);
    let mut head: Vec<Seg> = vec![];
    let mut tail: Vec<Seg> = vec![];
    let mut qa = a_pt;
    let mut pb = b_pt;
    if mg > 0.0 {
        if b_pt.1 > tol {
            let t = last.tangent_end();
            pb = (b_pt.0 + t.0 * mg, b_pt.1 + t.1 * mg);
            tail.push(Seg::line(b_pt, pb, None));
        }
        if a_pt.1 > tol {
            let t = first.tangent_start();
            qa = (a_pt.0 - t.0 * mg, a_pt.1 - t.1 * mg);
            head.push(Seg::line(qa, a_pt, None));
        }
    }
    // candidate closures from `pb` back to `qa`
    let direct: Vec<Seg> = if d2(pb, qa) > tol { vec![Seg::line(pb, qa, None)] } else { vec![] };
    let mut via: Vec<Seg> = vec![];
    let mut cur = pb;
    if cur.1 > tol {
        via.push(Seg::line(cur, (cur.0, 0.0), None));
        cur = (cur.0, 0.0);
    }
    if (cur.0 - qa.0).abs() > tol {
        via.push(Seg::line(cur, (qa.0, 0.0), None));
        cur = (qa.0, 0.0);
    }
    if qa.1 > tol {
        via.push(Seg::line(cur, qa, None));
    }
    // the probes: next to the middle of every face, on the side the feature's material lies (inside the region)
    // and on the other side (outside it)
    let side = if cut { 1.0 } else { -1.0 };
    let eps = 1e-3 * (1.0 + l);
    let probes: Vec<(P, P)> = prims
        .iter()
        .filter(|p| !p.faces.is_empty())
        .map(|p| {
            let mid = seg_of(p, false).mid();
            ((mid.0 + side * eps * p.nrm.0, mid.1 + side * eps * p.nrm.1), (mid.0 - side * eps * p.nrm.0, mid.1 - side * eps * p.nrm.1))
        })
        .collect();
    // through the axis first (the region between the chain and the axis); the direct line only where that would
    // double back on itself (both ends at one level) or the probe rejects it
    let cands = if (pb.0 - qa.0).abs() > tol { vec![via, direct] } else { vec![direct] };
    for cand in cands {
        let mut all: Vec<Seg> = head.clone();
        all.extend(segs.iter().cloned());
        all.extend(tail.iter().cloned());
        all.extend(cand);
        let poly: Vec<P> = all.iter().flat_map(|s| s.samples()).collect();
        if poly.len() >= 3 && area(&poly).abs() > 1e-9 && probes.iter().all(|(i, o)| inside(&poly, *i) && !inside(&poly, *o)) {
            *segs = all;
            return Ok(());
        }
    }
    Err("no closure of the profile puts the region on the faces' material side".into())
}
