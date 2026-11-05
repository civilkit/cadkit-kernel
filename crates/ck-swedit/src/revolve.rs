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
//! This reads the faces itself rather than going through `ck_recog::revolve::analyze`: that one needs every face
//! of the body, circular edges only and a closed chain, none of which holds for a feature in the middle of a
//! history. The rebuild is still checked against the import by the caller.

use crate::{face_points, Cx, TOL};
use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_json::Obj;
use ck_recog::emit::{n, Sk};
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
}

struct Prim {
    g: Gen,
    /// The face the generatrix belongs to; `None` for a piece that bridges a gap a later feature left.
    face: Option<FaceId>,
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
    faces: Vec<FaceId>,
}

fn arc_pt(c: P, r: f64, chi: f64) -> P {
    (c.0 + r * chi.cos(), c.1 + r * chi.sin())
}

fn d2(a: P, b: P) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

impl Seg {
    fn line(a: P, b: P, face: Option<FaceId>) -> Seg {
        Seg { a, b, arc: None, faces: face.into_iter().collect() }
    }
    fn mid(&self) -> P {
        match self.arc {
            Some((c, r, a0, sp)) => arc_pt(c, r, a0 + sp / 2.0),
            None => ((self.a.0 + self.b.0) / 2.0, (self.a.1 + self.b.1) / 2.0),
        }
    }
    fn samples(&self) -> Vec<P> {
        match self.arc {
            Some((c, r, a0, sp)) => (0..=24).map(|i| arc_pt(c, r, a0 + sp * i as f64 / 24.0)).collect(),
            None => vec![self.a, self.b],
        }
    }
    /// Unit tangent at the end of travel.
    fn tangent_end(&self) -> P {
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
        match self.arc {
            Some((_, _, a0, sp)) => {
                let s = sp.signum();
                (-a0.sin() * s, a0.cos() * s)
            }
            None => self.tangent_end(),
        }
    }
    fn reversed(&self) -> Seg {
        Seg { a: self.b, b: self.a, arc: self.arc.map(|(c, r, a0, sp)| (c, r, a0 + sp, -sp)), faces: self.faces.clone() }
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
fn find_axis(m: &Model, faces: &[FaceId], l: f64) -> Result<(Vec3, Vec3, Vec<FaceId>), String> {
    let mut ax: Option<(Vec3, Vec3)> = None;
    for &f in faces {
        match m.kind(f) {
            Kind::Cyl { o, a, .. } | Kind::Cone { o, a, .. } => ax = ax.or(Some((*o, *a))),
            Kind::Torus { c, a, .. } => ax = ax.or(Some((*c, *a))),
            _ => {}
        }
    }
    if ax.is_none() {
        // spheres only (and planes): the poles go along +Y like the kit's own ball
        if let Some(c) = faces.iter().find_map(|&f| if let Kind::Sphere { c, .. } = m.kind(f) { Some(*c) } else { None }) {
            ax = Some((c, Vec3::Y));
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
            Kind::Other(_) => false,
        };
        if !ok {
            return Err(format!("face {} ({}) is not about the axis", m.b.face_id_str(f), m.kind(f).name()));
        }
    }
    Ok((ao, a, caps))
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
        let axis_circle = es.len() == 1 && matches!(m.curve(es[0]), Curve::Circle { frame, .. } if par(frame.z, ax.a) && dist_to_line(frame.o, ax.ao, ax.a) < tol);
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

/// The generatrix of one lateral face. `frac` = sweep / 2 pi (1 for a full revolve).
fn generatrix(m: &Model, ax: &Ax, f: FaceId, frac: f64, tol: f64) -> Result<Prim, String> {
    let pts = boundary(m, ax, f);
    // a boundary point off the axis, for the sense of the normal
    let probe = pts.iter().find(|(q, _)| q.1 > 10.0 * tol).map(|(_, p)| *p);
    let sense = |mid: P, c: P| -> P {
        // normal at `mid` of an arc about `c`: radial in the generatrix plane, outward = the side the normal at a
        // boundary point faces
        let r = unit((mid.0 - c.0, mid.1 - c.1));
        let sg = match probe {
            Some(p) => {
                let nn = normal_sr(m, ax, f, p);
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
        let p = probe.ok_or("no boundary point off the axis")?;
        Ok(normal_sr(m, ax, f, p))
    };
    let name = m.b.face_id_str(f);
    match m.kind(f).clone() {
        Kind::Plane { n: pn, d } => {
            let s = (d - pn.dot(ax.ao)) / pn.dot(ax.a);
            let (lo, hi) = disc_interval(m, ax, f, tol);
            if hi - lo < tol {
                return Err(format!("disc {name} has no radial extent"));
            }
            Ok(Prim { g: Gen::Line { a: (s, lo), b: (s, hi) }, face: Some(f), nrm: (pn.dot(ax.a), 0.0) })
        }
        Kind::Cyl { r, .. } => {
            let (lo, hi) = s_range(&pts).ok_or_else(|| format!("cylinder {name} has no boundary"))?;
            if hi - lo < tol {
                return Err(format!("cylinder {name} has no axial extent"));
            }
            Ok(Prim { g: Gen::Line { a: (lo, r), b: (hi, r) }, face: Some(f), nrm: line_normal()? })
        }
        Kind::Cone { o, a: ca, r0, half, .. } => {
            let (lo, hi) = s_range(&pts).ok_or_else(|| format!("cone {name} has no boundary"))?;
            let rho_at = |s: f64| -> f64 {
                let v: Vec<f64> = pts.iter().filter(|(q, _)| (q.0 - s).abs() < tol).map(|(q, _)| q.1).collect();
                v.iter().sum::<f64>() / v.len() as f64
            };
            let (a, b) = if hi - lo < tol {
                let v = -r0 / half.tan();
                let sa = (o + ca * v - ax.ao).dot(ax.a);
                ((lo, rho_at(lo)), (sa, 0.0))
            } else {
                ((lo, rho_at(lo)), (hi, rho_at(hi)))
            };
            let nrm = line_normal()?;
            Ok(Prim { g: Gen::Line { a, b }, face: Some(f), nrm })
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
                    let a = m.area(f);
                    let rest = 2.0 * TAU * r * r * frac - cap;
                    if (a - cap).abs() <= (a - rest).abs() {
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
            Ok(Prim { g: Gen::Arc { c: cc, r, a0: lo, span: hi - lo }, face: Some(f), nrm: sense(mid, cc) })
        }
        Kind::Torus { c, major, minor, .. } => {
            let cc = ((c - ax.ao).dot(ax.a), major);
            if pts.is_empty() {
                return Ok(Prim { g: Gen::Circle { c: cc, r: minor }, face: Some(f), nrm: sense((cc.0 + minor, cc.1), cc) });
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
                let a = m.area(f);
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
                    return Ok(Prim { g: Gen::Circle { c: cc, r: minor }, face: Some(f), nrm: (1.0, 0.0) });
                }
                let start = v[(best.1 + 1) % v.len()];
                (start, TAU - best.0)
            };
            let mid = arc_pt(cc, minor, a0 + span / 2.0);
            Ok(Prim { g: Gen::Arc { c: cc, r: minor, a0, span }, face: Some(f), nrm: sense(mid, cc) })
        }
        Kind::Other(k) => Err(format!("face {name} is a {k}")),
    }
}

fn s_range(pts: &[(P, Vec3)]) -> Option<(f64, f64)> {
    if pts.is_empty() {
        return None;
    }
    Some((pts.iter().map(|(q, _)| q.0).fold(f64::INFINITY, f64::min), pts.iter().map(|(q, _)| q.0).fold(f64::NEG_INFINITY, f64::max)))
}

fn ends_of(g: &Gen) -> (P, P) {
    match g {
        Gen::Line { a, b } => (*a, *b),
        Gen::Arc { c, r, a0, span } => (arc_pt(*c, *r, *a0), arc_pt(*c, *r, a0 + span)),
        Gen::Circle { c, r } => ((c.0 + r, c.1), (c.0 + r, c.1)),
    }
}

fn seg_of(p: &Prim, rev: bool) -> Seg {
    let s = match &p.g {
        Gen::Line { a, b } => Seg::line(*a, *b, p.face),
        Gen::Arc { c, r, a0, span } => {
            let (a, b) = ends_of(&p.g);
            Seg { a, b, arc: Some((*c, *r, *a0, *span)), faces: p.face.into_iter().collect() }
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
            if ei.0 == ej.0 || prims[ei.0].face.is_none() || prims[ej.0].face.is_none() {
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
            Gen::Circle { .. } => continue,
        };
        add.push(Prim { g, face: None, nrm: (0.0, 0.0) });
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
            add.push(Prim { g: Gen::Line { a: pt, b: (pt.0 + t.0 * k, 0.0) }, face: None, nrm: (0.0, 0.0) });
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
        if d2(x.b, y.a) > tol {
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
        segs[i] = Seg { a: x.a, b: y.b, arc, faces };
        segs.remove(j);
    }
}

/// Read a revolve back from its faces and emit plane, sketch and revolve.
pub fn revolve(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool) -> Result<String, String> {
    let m = cx.m;
    let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    for v in m.b.vertex_ids() {
        let p = m.b.vertex(v).p;
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let l = (hi - lo).len().max(1.0);
    let tol = TOL * (1.0 + l);
    let (ao, a, caps) = find_axis(m, faces, l)?;
    let ax = Ax { ao, a };
    let lateral: Vec<FaceId> = faces.iter().copied().filter(|g| !caps.contains(g)).collect();
    if lateral.is_empty() {
        return Err("no lateral face".into());
    }
    // the half plane the sketch is drawn in, and the sweep
    let (u0, sweep, c0, c1): (Vec3, f64, Vec<FaceId>, Vec<FaceId>) = match caps.len() {
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
        prims.push(generatrix(m, &ax, g, frac, tol)?);
    }
    // sketch frame: the plane through the axis and the start half plane
    let z = a.cross(u0).normalize();
    let fr = cx.frame(z, ao);
    let world = |p: P| ao + a * p.0 + u0 * p.1;
    let sk_xy = |p: P| fr.xy(world(p));

    // profile: the chain, closed
    let mut segs: Vec<Seg>;
    let mut circle: Option<(P, f64, FaceId)> = None;
    if let [Prim { g: Gen::Circle { c, r }, face, .. }] = prims.as_slice() {
        circle = Some((*c, *r, face.expect("a lateral face")));
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
    Ok(format!("{} about axis ({:.3},{:.3},{:.3}); {} face(s), angle {:.4}{}{}", if cut { "cut" } else { "boss" }, a.x, a.y, a.z, lateral.len(), angle, if caps.is_empty() { "" } else { ", end caps" }, if dims.is_empty() { String::new() } else { format!(" (file lists {})", dims.join(", ")) }))
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
        .filter(|p| p.face.is_some())
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
