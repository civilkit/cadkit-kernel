//! Phase C: IR -> ck-topo [`Body`] (millimetres).
//!
//! Follows the structure of `ck-io`'s ACIS importer (seam insertion for periodic faces, loop ordering, pcurves
//! from the shared helpers): the XT half-edge graph is read by vertex geometry, so edge parameter ranges are
//! derived from the vertices on the carrier curve and the fin senses are mapped to coedge reversals.
//!
//! Extra passes: void-region skip (done in the IR: only solid regions' shells are read), shell orientation (outer
//! shell outward, cavity shells inward), unit scaling (file metres -> mm), edge splitting where a periodic face
//! needs a seam between two loops that have no vertex at a common angle (a first pass records the split points,
//! the second pass builds with them), full sphere / torus faces, cone apex seams, approximated carriers reported
//! with their maximum deviation.

use crate::error::{Limits, SldprtError, R};
use crate::ir::*;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::fit::{self, Param};
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::pcurve::pcurve_of;
use ck_geom::surface::Surface;
use ck_geom::{frame_from, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, ShellId, VertexId};
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::rc::Rc;

/// Metres -> millimetres.
pub const SCALE: f64 = 1000.0;

/// A carrier that is an approximation of the exact curve in the file.
#[derive(Debug, Clone)]
pub struct Approx {
    pub what: String,
    pub max_err_mm: f64,
}

#[derive(Debug)]
pub struct Built {
    pub body: Body,
    pub solids: usize,
    pub approx: Vec<Approx>,
    pub notes: Vec<String>,
}

fn v3(a: V3) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}
fn pt(a: V3) -> Vec3 {
    v3(a) * SCALE
}
fn gerr(msg: impl Into<String>) -> SldprtError {
    SldprtError::new("geometry.invalid", msg)
}
fn uerr(msg: impl Into<String>) -> SldprtError {
    SldprtError::new("unsupported.geometry", msg)
}
fn terr(msg: impl Into<String>) -> SldprtError {
    SldprtError::new("topology.invalid", msg)
}

/// Distance tolerance (mm) between a vertex and its curve / face surface.
fn geo_tol(p: Vec3) -> f64 {
    2e-3 + 1e-5 * p.len()
}

struct FwCurve {
    curve: Curve,
    /// Parameter period of circles / ellipses.
    period: Option<f64>,
    /// The edge runs along increasing curve parameter (before looking at the vertices).
    dir_inc: bool,
    /// A closed B-spline (start == end).
    closed_spline: bool,
}

#[derive(Clone)]
struct Ce {
    edge: EdgeId,
    /// IR edge this coedge came from (`usize::MAX` for synthesised seams).
    ir_edge: usize,
    rev: bool,
    pole_jump: bool,
    /// Explicit pcurve (synthesised periodic loops).
    pc: Option<Curve2>,
}

impl Ce {
    fn new(edge: EdgeId, rev: bool) -> Ce {
        Ce { edge, ir_edge: usize::MAX, rev, pole_jump: false, pc: None }
    }
    fn jump(edge: EdgeId, rev: bool) -> Ce {
        Ce { edge, ir_edge: usize::MAX, rev, pole_jump: true, pc: None }
    }
}

/// An IR edge as built: kernel edges in increasing parameter order (several when split).
#[derive(Clone)]
struct EdgeInfo {
    segs: Vec<EdgeId>,
    /// The XT edge direction is opposite to our increasing parameter.
    flip: bool,
}

struct B<'a> {
    ir: &'a Ir,
    body: Body,
    curves: HashMap<usize, Rc<FwCurve>>,
    surfaces: HashMap<usize, Rc<Surface>>,
    verts: HashMap<usize, (VertexId, Vec3)>,
    edges: HashMap<usize, EdgeInfo>,
    approx: Vec<Approx>,
    notes: Vec<String>,
    /// Split points (mm) per IR edge, from the previous pass.
    splits: HashMap<usize, Vec<Vec3>>,
    /// Split points this pass needed.
    need_split: Vec<(usize, Vec3)>,
}

/// Build the kernel body of one or more IRs (all solid bodies end up as lumps of one body).
pub fn build(irs: &[&Ir], limits: &Limits) -> R<Built> {
    let _ = limits;
    let mut splits: HashMap<usize, Vec<Vec3>> = HashMap::new();
    let mut last_err = None;
    for pass in 0..3 {
        // every IR keeps its own split table (edge indices are per IR)
        let mut need_all: Vec<Vec<(usize, Vec3)>> = vec![];
        let mut body = Body::new();
        let (mut solids, mut approx, mut notes) = (0usize, vec![], vec![]);
        let mut err: Option<SldprtError> = None;
        for (k, ir) in irs.iter().enumerate() {
            let mut b = B { ir, body, curves: HashMap::new(), surfaces: HashMap::new(), verts: HashMap::new(), edges: HashMap::new(), approx: vec![], notes: vec![], splits: filter_splits(&splits, k), need_split: vec![] };
            let r = b.run(&mut solids);
            need_all.push(std::mem::take(&mut b.need_split));
            approx.extend(b.approx);
            notes.extend(b.notes);
            body = b.body;
            if let Err(e) = r {
                err = Some(e);
                break;
            }
        }
        let any_need = need_all.iter().any(|n| !n.is_empty());
        if !any_need {
            if let Some(e) = err {
                return Err(e);
            }
            if body.num_faces() == 0 {
                return Err(SldprtError::new("unsupported.body", format!("no solid faces imported ({})", notes.join("; "))));
            }
            return Ok(Built { body, solids, approx, notes });
        }
        for (k, need) in need_all.into_iter().enumerate() {
            for (e, p) in need {
                let v = splits.entry(k * 1_000_000_000 + e).or_default();
                if !v.iter().any(|q| q.dist(p) < 1e-6) {
                    v.push(p);
                }
            }
        }
        last_err = err;
        let _ = pass;
    }
    Err(last_err.unwrap_or_else(|| uerr("edge splitting for periodic faces did not converge")))
}

fn filter_splits(all: &HashMap<usize, Vec<Vec3>>, k: usize) -> HashMap<usize, Vec<Vec3>> {
    all.iter().filter(|(e, _)| **e / 1_000_000_000 == k).map(|(e, v)| (*e % 1_000_000_000, v.clone())).collect()
}

fn bbox_of(pts: &[Vec3]) -> (Vec3, Vec3) {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in pts {
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (lo, hi)
}

impl<'a> B<'a> {
    fn run(&mut self, solids: &mut usize) -> R<()> {
        let ir = self.ir;
        for body in &ir.bodies {
            if body.kind != BodyKind::Solid {
                self.notes.push(format!("body #{} is a {:?} body (not imported)", body.id, body.kind));
                continue;
            }
            for reg in &body.regions {
                if !reg.solid || reg.shells.iter().all(|s| s.faces.is_empty()) {
                    continue; // void region: the infinite outside
                }
                let lump = self.body.add_lump();
                let mut shells: Vec<(ShellId, f64)> = vec![];
                for sh in &reg.shells {
                    if sh.faces.is_empty() {
                        continue;
                    }
                    let s = self.body.add_shell(lump);
                    let mut all: Vec<Vec3> = vec![];
                    for f in &sh.faces {
                        all.extend(self.face(s, f)?);
                    }
                    let (lo, hi) = bbox_of(&all);
                    shells.push((s, if lo.x <= hi.x { lo.dist(hi) } else { 0.0 }));
                }
                self.orient_shells(&shells);
                *solids += 1;
            }
        }
        Ok(())
    }

    // ------------------------------------------------------------ orientation
    /// Largest shell outward (positive volume), the others (cavities) inward.
    fn orient_shells(&mut self, shells: &[(ShellId, f64)]) {
        let mut order: Vec<(ShellId, f64)> = shells.to_vec();
        order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        for (i, (s, _)) in order.iter().enumerate() {
            let faces: Vec<FaceId> = self.body.shell(*s).faces.clone();
            let vol = ck_topo::mass::signed_volume(&self.body, &faces);
            let want_pos = i == 0;
            if vol.is_finite() && (vol > 0.0) != want_pos && vol.abs() > 1e-12 {
                for &f in &faces {
                    ck_topo::orient::flip_face(&mut self.body, f);
                }
            }
        }
    }

    // ------------------------------------------------------------ curves
    fn bspline_curve(&self, nc: &NurbsCurveIr) -> R<BSplineCurve> {
        if nc.dim != 3 {
            return Err(uerr("2D B-curve used as a 3D carrier"));
        }
        let ctrl: Vec<Vec3> = nc.ctrl.iter().map(|c| Vec3::new(c[0], c[1], c[2]) * SCALE).collect();
        let r = match &nc.weights {
            Some(w) => BSplineCurve::new_rational(nc.degree, nc.knots.clone(), ctrl, w.clone()),
            None => BSplineCurve::new(nc.degree, nc.knots.clone(), ctrl),
        };
        r.map_err(|e| gerr(format!("bad B-curve: {e}")))
    }

    fn fit_points(&self, pts: &[Vec3]) -> R<BSplineCurve> {
        if pts.len() < 2 {
            return Err(gerr("fewer than two points to fit"));
        }
        let deg = 3.min(pts.len() - 1);
        fit::interpolate(pts, deg, Param::ChordLength).map_err(|e| gerr(format!("curve fit failed: {e}")))
    }

    fn is_closed(bs: &BSplineCurve) -> bool {
        bs.start().dist(bs.end()) < 1e-6 * (1.0 + bs.start().len())
    }

    fn curve(&mut self, ci: usize) -> R<Rc<FwCurve>> {
        if let Some(c) = self.curves.get(&ci) {
            return Ok(c.clone());
        }
        let node = &self.ir.curves[ci];
        let sense = node.sense_plus;
        let fc = match &node.geom {
            IrCurve::Line { o, d } => {
                if v3(*d).len() < 1e-12 {
                    return Err(gerr("line with zero direction"));
                }
                FwCurve { curve: Curve::line(pt(*o), v3(*d)), period: None, dir_inc: sense, closed_spline: false }
            }
            IrCurve::Circle { c, n, x, r } => {
                if v3(*n).len() < 1e-12 || v3(*x).len() < 1e-12 || !(*r > 0.0) {
                    return Err(gerr("degenerate circle"));
                }
                FwCurve { curve: Curve::circle_x(pt(*c), v3(*n), v3(*x), r * SCALE), period: Some(TAU), dir_inc: sense, closed_spline: false }
            }
            IrCurve::Ellipse { c, n, x, a, b } => {
                if v3(*n).len() < 1e-12 || v3(*x).len() < 1e-12 || !(*a > 0.0) || !(*b > 0.0) {
                    return Err(gerr("degenerate ellipse"));
                }
                let curve = if (a - b).abs() < 1e-12 * a { Curve::circle_x(pt(*c), v3(*n), v3(*x), a * SCALE) } else { Curve::ellipse(pt(*c), v3(*n), v3(*x), a * SCALE, b * SCALE) };
                FwCurve { curve, period: Some(TAU), dir_inc: sense, closed_spline: false }
            }
            IrCurve::BCurve(nc) => {
                let bs = self.bspline_curve(nc)?;
                let closed = Self::is_closed(&bs);
                FwCurve { curve: Curve::BSpline(bs), period: None, dir_inc: sense, closed_spline: closed }
            }
            IrCurve::Trimmed { basis, t1, t2, .. } => {
                let bi = basis.ok_or_else(|| gerr("trimmed curve without basis"))?;
                let b = self.curve(bi)?;
                FwCurve { curve: b.curve.clone(), period: b.period, dir_inc: sense == (t2 > t1), closed_spline: b.closed_spline }
            }
            IrCurve::Intersection { pts, surfaces } => {
                let p: Vec<Vec3> = pts.iter().map(|q| pt(*q)).collect();
                let bs = self.fit_points(&p)?;
                let closed = Self::is_closed(&bs);
                let curve = Curve::BSpline(bs);
                // deviation from the support surfaces at 48 samples
                let mut worst = 0.0f64;
                let mut checked = 0;
                for s in surfaces.iter().flatten() {
                    if let Ok(sf) = self.surface(*s) {
                        let (a, b) = curve.range();
                        for k in 0..48 {
                            let t = a + (b - a) * (k as f64 + 0.37) / 48.0;
                            worst = worst.max(sf.distance(curve.eval(t)));
                        }
                        checked += 1;
                    }
                }
                self.approx.push(Approx { what: format!("intersection curve #{} ({} chart points, {} support surfaces checked)", node.id, pts.len(), checked), max_err_mm: worst });
                FwCurve { curve, period: None, dir_inc: true, closed_spline: closed }
            }
            IrCurve::Sp { surface, uv } => {
                let (Some(si), Some(uv)) = (surface, uv) else { return Err(gerr("SP-curve without surface or parameter curve")) };
                let sf = self.surface(*si)?;
                if uv.dim != 2 {
                    return Err(gerr("SP-curve parameter curve is not 2D"));
                }
                let ctrl: Vec<Vec2> = uv.ctrl.iter().map(|c| Vec2::new(c[0], c[1])).collect();
                let c2 = match &uv.weights {
                    Some(w) => BSplineCurve::<Vec2>::new_rational(uv.degree, uv.knots.clone(), ctrl, w.clone()),
                    None => BSplineCurve::<Vec2>::new(uv.degree, uv.knots.clone(), ctrl),
                }
                .map_err(|e| gerr(format!("bad SP-curve: {e}")))?;
                let (t0, t1) = c2.domain();
                let n = (8 * uv.ctrl.len()).clamp(32, 400);
                let pts: Vec<Vec3> = (0..=n)
                    .map(|i| {
                        let q = c2.eval(t0 + (t1 - t0) * i as f64 / n as f64);
                        sf.eval(q.x, q.y)
                    })
                    .collect();
                let mut clean: Vec<Vec3> = vec![];
                for p in pts {
                    if clean.last().map_or(true, |l| l.dist(p) > 1e-9) {
                        clean.push(p);
                    }
                }
                let bs = self.fit_points(&clean)?;
                let closed = Self::is_closed(&bs);
                let curve = Curve::BSpline(bs);
                let mut worst = 0.0f64;
                let (a, b) = curve.range();
                for k in 0..48 {
                    let t = a + (b - a) * (k as f64 + 0.37) / 48.0;
                    worst = worst.max(sf.distance(curve.eval(t)));
                }
                self.approx.push(Approx { what: format!("SP-curve #{} sampled through {} points", node.id, clean.len()), max_err_mm: worst });
                FwCurve { curve, period: None, dir_inc: true, closed_spline: closed }
            }
            IrCurve::Unsupported(m) => return Err(uerr(format!("curve #{}: {m}", node.id))),
        };
        let rc = Rc::new(fc);
        self.curves.insert(ci, rc.clone());
        Ok(rc)
    }

    // ------------------------------------------------------------ surfaces
    fn surface(&mut self, si: usize) -> R<Rc<Surface>> {
        if let Some(s) = self.surfaces.get(&si) {
            return Ok(s.clone());
        }
        let node = &self.ir.surfaces[si];
        let s = match &node.geom {
            IrSurface::Plane { o, n, x } => {
                if v3(*n).len() < 1e-12 {
                    return Err(gerr("plane with zero normal"));
                }
                Surface::Plane { frame: frame_from(pt(*o), v3(*n), v3(*x)) }
            }
            IrSurface::Cylinder { o, axis, r, x } => {
                if !(*r > 0.0) || v3(*axis).len() < 1e-12 {
                    return Err(gerr("degenerate cylinder"));
                }
                Surface::Cylinder { frame: frame_from(pt(*o), v3(*axis), v3(*x)), r: r * SCALE }
            }
            IrSurface::Cone { o, axis, r, sin, cos, x } => {
                if v3(*axis).len() < 1e-12 {
                    return Err(gerr("cone with zero axis"));
                }
                let half = sin.atan2(*cos);
                Surface::Cone { frame: frame_from(pt(*o), v3(*axis), v3(*x)), r: r * SCALE, semi_angle: half }
            }
            IrSurface::Sphere { c, r, axis, x } => {
                if !(*r > 0.0) {
                    return Err(gerr("degenerate sphere"));
                }
                Surface::Sphere { frame: frame_from(pt(*c), v3(*axis), v3(*x)), r: r * SCALE }
            }
            IrSurface::Torus { c, axis, major, minor, x } => {
                if !(*major > 0.0) || !(*minor > 0.0) || minor >= major {
                    return Err(uerr(format!("self-intersecting (apple/lemon) torus, major {major}, minor {minor}")));
                }
                Surface::Torus { frame: frame_from(pt(*c), v3(*axis), v3(*x)), major: major * SCALE, minor: minor * SCALE }
            }
            IrSurface::Nurbs(ns) => {
                let ctrl: Vec<Vec3> = ns.ctrl.iter().map(|c| pt(*c)).collect();
                let s = BSplineSurface::new(ns.du, ns.dv, ns.uknots.clone(), ns.vknots.clone(), ns.nu, ns.nv, ctrl, ns.weights.clone()).map_err(|e| gerr(format!("bad B-surface #{}: {e}", node.id)))?;
                Surface::BSpline(Box::new(s))
            }
            IrSurface::Swept { section, dir } => {
                let ci = section.ok_or_else(|| gerr("swept surface without section"))?;
                let c = self.curve(ci)?;
                if v3(*dir).len() < 1e-12 {
                    return Err(gerr("swept surface with zero direction"));
                }
                Surface::Extrusion { profile: Box::new(c.curve.clone()), dir: v3(*dir).normalize() }
            }
            IrSurface::Spun { profile, base, axis } => {
                let ci = profile.ok_or_else(|| gerr("spun surface without profile"))?;
                let c = self.curve(ci)?;
                if v3(*axis).len() < 1e-12 {
                    return Err(gerr("spun surface with zero axis"));
                }
                Surface::Revolution { profile: Box::new(c.curve.clone()), axis_o: pt(*base), axis: v3(*axis).normalize() }
            }
            IrSurface::Offset { base, dist } => {
                let bi = base.ok_or_else(|| gerr("offset surface without base"))?;
                let b = self.surface(bi)?;
                Surface::Offset { base: Box::new((*b).clone()), dist: dist * SCALE }
            }
            IrSurface::Blend => return Err(uerr(format!("rolling-ball blend surface #{} (not evaluated)", node.id))),
            IrSurface::Unsupported(m) => return Err(uerr(format!("surface #{}: {m}", node.id))),
        };
        let rc = Rc::new(s);
        self.surfaces.insert(si, rc.clone());
        Ok(rc)
    }

    // ------------------------------------------------------------ vertices / edges
    fn vertex(&mut self, vi: usize) -> (VertexId, Vec3) {
        if let Some(v) = self.verts.get(&vi) {
            return *v;
        }
        let p = pt(self.ir.vertices[vi].p);
        let id = self.body.add_vertex(p);
        self.verts.insert(vi, (id, p));
        (id, p)
    }

    /// Build an IR edge (split where the previous pass asked for it).
    fn edge(&mut self, ei: usize) -> R<EdgeInfo> {
        if let Some(e) = self.edges.get(&ei) {
            return Ok(e.clone());
        }
        let e = &self.ir.edges[ei];
        let eid = e.id;
        let ci = e.curve.or(e.fin_curve).ok_or_else(|| terr(format!("edge #{eid} has neither a curve nor fin curves")))?;
        let fc = self.curve(ci)?;
        let curve = &fc.curve;
        // (lo, hi, vertex at lo, vertex at hi, flip)
        let (lo, hi, v_lo, v_hi, flip): (f64, f64, VertexId, VertexId, bool) = match (e.start, e.end) {
            (Some(sv), Some(ev)) => {
                let (vs, ps) = self.vertex(sv);
                let (ve, pe) = self.vertex(ev);
                let ta = curve.closest_param(ps);
                let tb = curve.closest_param(pe);
                let miss = curve.eval(ta).dist(ps).max(curve.eval(tb).dist(pe));
                if miss > geo_tol(ps).max(geo_tol(pe)) {
                    return Err(gerr(format!("edge #{eid}: curve misses its vertices by {miss:.3e} mm")));
                }
                let tie = vs == ve || ps.dist(pe) < 1e-9;
                if let Some(per) = fc.period {
                    let norm = |t: f64| t.rem_euclid(per);
                    let (a, b) = (norm(ta), norm(tb));
                    if fc.dir_inc {
                        let mut end = b;
                        while end <= a + 1e-9 {
                            end += per;
                        }
                        (a, end, vs, ve, false)
                    } else {
                        let mut end = a;
                        while end <= b + 1e-9 {
                            end += per;
                        }
                        (b, end, ve, vs, true)
                    }
                } else if tie {
                    if !fc.closed_spline {
                        return Err(terr(format!("edge #{eid}: both ends at one vertex on an open curve")));
                    }
                    let (d0, d1) = curve.range();
                    (d0, d1, vs, ve, !fc.dir_inc)
                } else if ta <= tb {
                    (ta, tb, vs, ve, false)
                } else {
                    (tb, ta, ve, vs, true)
                }
            }
            (None, None) => {
                // ring edge on a closed curve: synthesise a vertex at the start of the parameter range
                let (d0, d1) = match fc.period {
                    Some(p) => (0.0, p),
                    None => {
                        if !fc.closed_spline {
                            return Err(terr(format!("edge #{eid} without vertices on an open curve")));
                        }
                        curve.range()
                    }
                };
                let v = self.body.add_vertex(curve.eval(d0));
                (d0, d1, v, v, !fc.dir_inc)
            }
            _ => return Err(terr(format!("edge #{eid} has only one vertex"))),
        };
        if !(hi - lo > 1e-12) {
            return Err(terr(format!("edge #{eid} has zero extent")));
        }
        // split points inside (lo, hi)
        let mut ts: Vec<f64> = vec![];
        if let Some(pts) = self.splits.get(&ei) {
            for p in pts {
                let mut t = curve.closest_param(*p);
                if let Some(per) = fc.period {
                    while t < lo - 1e-9 {
                        t += per;
                    }
                    while t > hi + 1e-9 {
                        t -= per;
                    }
                }
                if curve.eval(t).dist(*p) < 10.0 * geo_tol(*p) && t > lo + 1e-7 && t < hi - 1e-7 {
                    ts.push(t);
                }
            }
        }
        ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        ts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        let mut segs = vec![];
        let mut bounds = vec![lo];
        bounds.extend(ts.iter().copied());
        bounds.push(hi);
        let mut vprev = v_lo;
        for k in 0..bounds.len() - 1 {
            let vnext = if k + 2 == bounds.len() { v_hi } else { self.body.add_vertex(curve.eval(bounds[k + 1])) };
            segs.push(self.body.add_edge(curve.clone(), bounds[k], bounds[k + 1], vprev, vnext));
            vprev = vnext;
        }
        let info = EdgeInfo { segs, flip };
        self.edges.insert(ei, info.clone());
        Ok(info)
    }

    // ------------------------------------------------------------ faces
    fn samples(&self, ce: &Ce) -> usize {
        match &self.body.edge(ce.edge).curve {
            Curve::BSpline(_) => 512,
            _ => 16,
        }
    }

    fn ce_points(&self, ce: &Ce, n: usize) -> Vec<Vec3> {
        let e = self.body.edge(ce.edge);
        let (a, b) = if ce.rev { (e.t1, e.t0) } else { (e.t0, e.t1) };
        (0..=n).map(|i| e.curve.eval(a + (b - a) * i as f64 / n as f64)).collect()
    }

    fn ce_start(&self, ce: &Ce) -> VertexId {
        let e = self.body.edge(ce.edge);
        if ce.rev {
            e.v1
        } else {
            e.v0
        }
    }

    fn vpos(&self, v: VertexId) -> Vec3 {
        self.body.vertices[v.idx()].p
    }

    fn vertex_at(&mut self, p: Vec3) -> VertexId {
        for (i, v) in self.body.vertices.iter().enumerate() {
            if v.p.dist(p) < 1e-7 {
                return VertexId(i as u32);
            }
        }
        self.body.add_vertex(p)
    }

    /// Build one face; returns the vertex positions seen (shell extent).
    fn face(&mut self, shell: ShellId, f: &IrFace) -> R<Vec<Vec3>> {
        let fid = f.id;
        let si = f.surface.ok_or_else(|| terr(format!("face #{fid} has no surface")))?;
        let surface = (*self.surface(si)?).clone();
        let mut loops: Vec<Vec<Ce>> = vec![];
        let mut pts: Vec<Vec3> = vec![];
        for lp in &f.loops {
            if lp.fins.is_empty() {
                continue; // isolated vertex loop (pole): implicit
            }
            let mut ring = vec![];
            for fin in &lp.fins {
                let info = self.edge(fin.edge)?;
                let rev = (!fin.forward) ^ info.flip;
                let order: Vec<EdgeId> = if rev { info.segs.iter().rev().copied().collect() } else { info.segs.clone() };
                for id in order {
                    ring.push(Ce { edge: id, ir_edge: fin.edge, rev, pole_jump: false, pc: None });
                    let e = self.body.edge(id);
                    pts.push(self.vpos(e.v0));
                    pts.push(self.vpos(e.v1));
                }
            }
            if f.back {
                ring.reverse();
                for c in ring.iter_mut() {
                    c.rev = !c.rev;
                }
            }
            loops.push(ring);
        }
        let sense = f.same_sense ^ f.back;

        // every vertex must lie on the surface
        let is_spline = matches!(surface, Surface::BSpline(_));
        for p in &pts {
            let d = surface.distance(*p);
            let tol = if is_spline { 20.0 * geo_tol(*p) } else { 5.0 * geo_tol(*p) };
            if d > tol {
                return Err(gerr(format!("face #{fid}: vertex ({:.4}, {:.4}, {:.4}) is {d:.3e} mm off its {} surface", p.x, p.y, p.z, surface.kind())));
            }
        }
        if loops.is_empty() {
            match &surface {
                Surface::Sphere { .. } | Surface::Torus { .. } => {
                    loops.push(self.full_periodic_loop(&surface, sense).ok_or_else(|| gerr(format!("face #{fid}: cannot build the loop of a full {}", surface.kind())))?);
                }
                _ => return Err(terr(format!("face #{fid} has no loop on a non-closed {} surface", surface.kind()))),
            }
        }

        let loops = self.seam_wrapping(fid, &surface, sense, loops)?;
        let loops = self.order_loops(fid, &surface, sense, loops)?;

        let name = FaceName::other("sldprt", &format!("F{fid}"));
        let face = self.body.add_face(shell, surface.clone(), sense, name);
        let (pu, pv) = surface.periods();
        for ring in loops {
            let lp = self.body.add_loop(face);
            let mut prev_end: Option<Vec2> = None;
            let mut loop_start: Option<Vec2> = None;
            for ce in ring {
                let e = self.body.edge(ce.edge);
                let (t0, t1, curve) = (e.t0, e.t1, e.curve.clone());
                let (ts, te) = if ce.rev { (t1, t0) } else { (t0, t1) };
                let explicit = ce.pc.is_some();
                let mut pc = match ce.pc.clone() {
                    Some(p) => Some(p),
                    None => pcurve_of(&surface, &curve, t0, t1, 1e-6).ok(),
                };
                if let (Some(c), false) = (pc.as_mut(), explicit) {
                    if ce.pole_jump {
                        if let Some(ls) = loop_start {
                            let s = c.eval(te);
                            let du = pu.map_or(0.0, |p| ((ls.x - s.x) / p).round() * p);
                            if du != 0.0 {
                                *c = shift_pc(c, du, 0.0);
                            }
                        }
                    } else if let Some(pe) = prev_end {
                        let s = c.eval(ts);
                        let du = pu.map_or(0.0, |p| ((pe.x - s.x) / p).round() * p);
                        let dv = pv.map_or(0.0, |p| ((pe.y - s.y) / p).round() * p);
                        if du != 0.0 || dv != 0.0 {
                            *c = shift_pc(c, du, dv);
                        }
                    }
                }
                if let Some(c) = pc.as_ref() {
                    if loop_start.is_none() {
                        loop_start = Some(c.eval(ts));
                    }
                    prev_end = Some(c.eval(te));
                }
                self.body.add_coedge(lp, ce.edge, ce.rev, pc);
            }
        }
        Ok(pts)
    }

    /// Loop of a full sphere (`[S, S^-1]`, pole-to-pole meridian) or torus (`[R, S, R^-1, S^-1]`, both seams) with
    /// explicit pcurves, counter-clockwise in uv for `sense`.
    fn full_periodic_loop(&mut self, surf: &Surface, sense: bool) -> Option<Vec<Ce>> {
        let line = |o: Vec2, d: Vec2| Curve2::Line { o, d };
        let mut l = match surf {
            Surface::Sphere { frame, r } => {
                let c = Curve::Circle { frame: ck_geom::Frame { o: frame.o, x: frame.x, y: frame.z, z: frame.x.cross(frame.z) }, r: *r };
                let (s, n) = (frame.o - frame.z * *r, frame.o + frame.z * *r);
                let (vs, vn) = (self.vertex_at(s), self.vertex_at(n));
                let e = self.body.add_edge(c, -FRAC_PI_2, FRAC_PI_2, vs, vn);
                vec![
                    Ce { pc: Some(line(Vec2::new(TAU, 0.0), Vec2::new(0.0, 1.0))), ..Ce::new(e, false) },
                    Ce { pc: Some(line(Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.0))), ..Ce::new(e, true) },
                ]
            }
            Surface::Torus { frame, major, minor } => {
                // seam S: meridian circle at u = 0; ring R: circle of radius major + minor at v = 0
                let sc = Curve::Circle { frame: ck_geom::Frame { o: frame.o + frame.x * *major, x: frame.x, y: frame.z, z: frame.x.cross(frame.z) }, r: *minor };
                let rc = Curve::Circle { frame: ck_geom::Frame { o: frame.o, x: frame.x, y: frame.y, z: frame.z }, r: major + minor };
                let v = self.vertex_at(sc.eval(0.0));
                let es = self.body.add_edge(sc, 0.0, TAU, v, v);
                let er = self.body.add_edge(rc, 0.0, TAU, v, v);
                vec![
                    Ce { pc: Some(line(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0))), ..Ce::new(er, false) },
                    Ce { pc: Some(line(Vec2::new(TAU, 0.0), Vec2::new(0.0, 1.0))), ..Ce::new(es, false) },
                    Ce { pc: Some(line(Vec2::new(0.0, TAU), Vec2::new(1.0, 0.0))), ..Ce::new(er, true) },
                    Ce { pc: Some(line(Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.0))), ..Ce::new(es, true) },
                ]
            }
            _ => return None,
        };
        if !sense {
            l.reverse();
            for c in &mut l {
                c.rev = !c.rev;
            }
        }
        Some(l)
    }

    /// Total u-winding (in turns) of each loop on a u-periodic surface.
    fn windings(&self, surface: &Surface, loops: &[Vec<Ce>]) -> Vec<i32> {
        let frame = match surface {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Sphere { frame, .. } | Surface::Torus { frame, .. } => frame,
            _ => return vec![0; loops.len()],
        };
        loops
            .iter()
            .map(|ring| {
                if ring.iter().any(|c| c.pc.is_some()) {
                    return 0;
                }
                let mut total = 0.0;
                let mut last: Option<f64> = None;
                for ce in ring {
                    for p in self.ce_points(ce, self.samples(ce)) {
                        let d = p - frame.o;
                        if d.dot(frame.x).hypot(d.dot(frame.y)) < 1e-9 * (1.0 + d.len()) {
                            continue;
                        }
                        let u = d.dot(frame.y).atan2(d.dot(frame.x));
                        if let Some(l) = last {
                            let mut du = u - l;
                            while du > PI {
                                du -= TAU;
                            }
                            while du < -PI {
                                du += TAU;
                            }
                            total += du;
                        }
                        last = Some(u);
                    }
                }
                (total / TAU).round() as i32
            })
            .collect()
    }

    /// Find a vertex of `ring` at azimuth `ua` (about `frame`); if none, record a split of the coedge that passes
    /// through that azimuth and return `None`.
    fn vertex_at_azimuth(&mut self, ring: &[Ce], frame: &ck_geom::Frame, ua: f64) -> Option<usize> {
        let ang = |p: Vec3| {
            let d = p - frame.o;
            d.dot(frame.y).atan2(d.dot(frame.x))
        };
        let wrap = |x: f64| {
            let mut d = x % TAU;
            if d > PI {
                d -= TAU;
            }
            if d < -PI {
                d += TAU;
            }
            d
        };
        for (k, ce) in ring.iter().enumerate() {
            if wrap(ang(self.vpos(self.ce_start(ce))) - ua).abs() < 1e-6 {
                return Some(k);
            }
        }
        // no vertex there: look for the coedge whose interior passes through the azimuth
        for ce in ring {
            if ce.ir_edge == usize::MAX {
                continue;
            }
            let e = self.body.edge(ce.edge);
            let n = 720;
            let at = |i: usize| e.curve.eval(e.t0 + (e.t1 - e.t0) * i as f64 / n as f64);
            let mut prev = wrap(ang(at(0)) - ua);
            for i in 1..=n {
                let cur = wrap(ang(at(i)) - ua);
                if prev.signum() != cur.signum() && prev.abs() + cur.abs() < 1.0 {
                    // bisect
                    let (mut a, mut b) = (e.t0 + (e.t1 - e.t0) * (i - 1) as f64 / n as f64, e.t0 + (e.t1 - e.t0) * i as f64 / n as f64);
                    let sa = wrap(ang(e.curve.eval(a)) - ua).signum();
                    for _ in 0..60 {
                        let m = 0.5 * (a + b);
                        if wrap(ang(e.curve.eval(m)) - ua).signum() == sa {
                            a = m;
                        } else {
                            b = m;
                        }
                    }
                    let p = e.curve.eval(0.5 * (a + b));
                    self.need_split.push((ce.ir_edge, p));
                    return None;
                }
                prev = cur;
            }
        }
        None
    }

    /// Parasolid periodic faces have no seam edge: the boundary loops of a cylinder / cone / torus each wrap the axis.
    /// Join two of them with a seam. A cone or sphere cap with a single wrapping loop gets a seam to the apex / pole.
    fn seam_wrapping(&mut self, fid: i64, surface: &Surface, sense: bool, mut loops: Vec<Vec<Ce>>) -> R<Vec<Vec<Ce>>> {
        if surface.periods().0.is_none() {
            return Ok(loops);
        }
        let w = self.windings(surface, &loops);
        let wrap: Vec<usize> = (0..loops.len()).filter(|&i| w[i] != 0).collect();
        if wrap.is_empty() {
            return Ok(loops);
        }
        if let Surface::Sphere { .. } = surface {
            return self.sphere_seam(fid, surface, sense, loops, &wrap, &w);
        }
        let frame = match surface {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Torus { frame, .. } => frame.clone(),
            _ => return Err(uerr(format!("face #{fid}: {} face with a loop wrapping its axis (needs poles/seams)", surface.kind()))),
        };
        if wrap.len() == 1 {
            if let Surface::Cone { r, semi_angle, .. } = surface {
                return self.cone_cap_seam(fid, surface, loops, wrap[0], *r, *semi_angle);
            }
        }
        if wrap.len() != 2 || w[wrap[0]].abs() != 1 || w[wrap[1]].abs() != 1 {
            return Err(uerr(format!("face #{fid}: {} face with {} wrapping loops (a seam needs exactly two)", surface.kind(), wrap.len())));
        }
        if w[wrap[0]] == w[wrap[1]] {
            return Err(SldprtError::new("topology.orientation", format!("face #{fid}: both wrapping loops run the same way round the axis")));
        }
        let ang = |p: Vec3| {
            let d = p - frame.o;
            d.dot(frame.y).atan2(d.dot(frame.x))
        };
        let a = loops[wrap[0]].clone();
        let mut bl = loops[wrap[1]].clone();
        let va = self.ce_start(&a[0]);
        let pa = self.vpos(va);
        let ua = ang(pa);
        let Some(k) = self.vertex_at_azimuth(&bl, &frame, ua) else {
            if self.need_split.is_empty() {
                return Err(uerr(format!("face #{fid}: periodic face whose two loops have no vertices at a common angle (seam needs an edge split)")));
            }
            // first pass: the split is recorded, the face is built in the next pass
            return Ok(loops);
        };
        bl.rotate_left(k);
        let vb = self.ce_start(&bl[0]);
        let pb = self.vpos(vb);
        let len = pa.dist(pb);
        if !(len > 1e-9) {
            return Err(gerr(format!("face #{fid}: zero-length seam")));
        }
        let seam = if let Surface::Torus { major, minor, .. } = surface {
            // meridian circle at azimuth ua; the shorter arc between the two loops
            let radial = frame.x * ua.cos() + frame.y * ua.sin();
            let cc = frame.o + radial * *major;
            let n = radial.cross(frame.z);
            let _ = minor;
            let tv = |p: Vec3| (p - cc).dot(frame.z).atan2((p - cc).dot(radial));
            let (ta, mut tb) = (tv(pa), tv(pb));
            while tb - ta > PI {
                tb -= TAU;
            }
            while ta - tb > PI {
                tb += TAU;
            }
            let curve = Curve::circle_x(cc, n, radial, (pa - cc).len());
            let (lo, hi, vlo, vhi) = if ta < tb { (ta, tb, va, vb) } else { (tb, ta, vb, va) };
            let e = self.body.add_edge(curve, lo, hi, vlo, vhi);
            let fwd_rev = ta > tb;
            let mut joined = a;
            joined.push(Ce::new(e, fwd_rev));
            joined.extend(bl);
            joined.push(Ce::new(e, !fwd_rev));
            let mut out = vec![joined];
            for (i, l) in loops.drain(..).enumerate() {
                if i != wrap[0] && i != wrap[1] {
                    out.push(l);
                }
            }
            return Ok(out);
        } else {
            self.body.add_edge(Curve::line_through(pa, pb), 0.0, len, va, vb)
        };
        let mut joined = a;
        joined.push(Ce::new(seam, false));
        joined.extend(bl);
        joined.push(Ce::new(seam, true));
        let mut out = vec![joined];
        for (i, l) in loops.drain(..).enumerate() {
            if i != wrap[0] && i != wrap[1] {
                out.push(l);
            }
        }
        Ok(out)
    }

    /// A cone face bounded by one loop around the axis reaches the apex: seam out to the apex and back.
    fn cone_cap_seam(&mut self, fid: i64, surface: &Surface, loops: Vec<Vec<Ce>>, wi: usize, r: f64, semi: f64) -> R<Vec<Vec<Ce>>> {
        let Surface::Cone { frame, .. } = surface else { unreachable!() };
        let tan = semi.tan();
        if tan.abs() < 1e-9 {
            return Err(gerr(format!("face #{fid}: cone without taper")));
        }
        let apex = frame.o + frame.z * (-r / tan);
        let ring = loops[wi].clone();
        let va = self.ce_start(&ring[0]);
        let pa = self.vpos(va);
        let len = pa.dist(apex);
        if !(len > 1e-9) {
            return Err(gerr(format!("face #{fid}: cone loop lies on the apex")));
        }
        let vapex = self.vertex_at(apex);
        let seam = self.body.add_edge(Curve::line_through(apex, pa), 0.0, len, vapex, va);
        let mut joined = ring;
        joined.push(Ce::new(seam, true));
        joined.push(Ce::jump(seam, false));
        let mut out = vec![joined];
        for (i, l) in loops.into_iter().enumerate() {
            if i != wi {
                out.push(l);
            }
        }
        Ok(out)
    }

    fn sphere_seam(&mut self, fid: i64, surface: &Surface, sense: bool, loops: Vec<Vec<Ce>>, wrap: &[usize], w: &[i32]) -> R<Vec<Vec<Ce>>> {
        let Surface::Sphere { frame, r } = surface else { unreachable!() };
        let (c, axis) = (frame.o, frame.z);
        if wrap.iter().any(|&i| w[i].abs() != 1) || wrap.len() > 2 {
            return Err(uerr(format!("face #{fid}: sphere face with {} wrapping loop(s)", wrap.len())));
        }
        let azimuth = |p: Vec3| -> R<Vec3> {
            let d = p - c;
            let h = d - axis * d.dot(axis);
            if h.len() < 1e-9 * r.max(1.0) {
                return Err(gerr(format!("face #{fid}: sphere loop vertex sits on the pole axis")));
            }
            Ok(h.normalize())
        };
        let mut out: Vec<Vec<Ce>> = vec![];
        if wrap.len() == 1 {
            let ring = loops[wrap[0]].clone();
            let va = self.ce_start(&ring[0]);
            let pa = self.vpos(va);
            let pts = self.ce_points(&ring[0], 8);
            let tan = pts[1] - pts[0];
            let nrm = if sense { (pa - c).normalize() } else { -(pa - c).normalize() };
            let north = nrm.cross(tan).dot(axis) > 0.0;
            let xdir = if north { axis } else { -axis };
            let e = azimuth(pa)?;
            let t0 = (pa - c).dot(e).atan2((pa - c).dot(xdir));
            if !(t0 > 1e-9) {
                return Err(gerr(format!("face #{fid}: sphere cap loop lies on the pole")));
            }
            let pole = self.body.add_vertex(c + xdir * *r);
            let seam = self.body.add_edge(Curve::circle_x(c, xdir.cross(e), xdir, *r), 0.0, t0, pole, va);
            let mut joined = ring;
            joined.push(Ce::new(seam, true));
            joined.push(Ce::jump(seam, false));
            out.push(joined);
        } else {
            if w[wrap[0]] == w[wrap[1]] {
                return Err(SldprtError::new("topology.orientation", format!("face #{fid}: both wrapping loops run the same way round the axis")));
            }
            let a = loops[wrap[0]].clone();
            let mut bl = loops[wrap[1]].clone();
            let va = self.ce_start(&a[0]);
            let pa = self.vpos(va);
            let ea = azimuth(pa)?;
            let mut k = None;
            for i in 0..bl.len() {
                let p = self.vpos(self.ce_start(&bl[i]));
                if azimuth(p)?.dist(ea) < 1e-6 {
                    k = Some(i);
                    break;
                }
            }
            let k = k.ok_or_else(|| uerr(format!("face #{fid}: sphere band whose two loops have no vertices at a common azimuth")))?;
            bl.rotate_left(k);
            let vb = self.ce_start(&bl[0]);
            let pb = self.vpos(vb);
            let ang = |p: Vec3| (p - c).dot(ea).atan2((p - c).dot(axis));
            let (ta, tb) = (ang(pa), ang(pb));
            if (ta - tb).abs() < 1e-9 {
                return Err(gerr(format!("face #{fid}: zero-length seam")));
            }
            let curve = Curve::circle_x(c, axis.cross(ea), axis, *r);
            let (lo, hi, vlo, vhi) = if ta < tb { (ta, tb, va, vb) } else { (tb, ta, vb, va) };
            let seam = self.body.add_edge(curve, lo, hi, vlo, vhi);
            let fwd_rev = ta > tb;
            let mut joined = a;
            joined.push(Ce::new(seam, fwd_rev));
            joined.extend(bl);
            joined.push(Ce::new(seam, !fwd_rev));
            out.push(joined);
        }
        for (i, l) in loops.into_iter().enumerate() {
            if !wrap.contains(&i) {
                out.push(l);
            }
        }
        Ok(out)
    }

    /// Outer loop first. Planar faces: by signed area about the outward normal (a negative outer area is an
    /// orientation error); other faces: by extent of the loop's bounding box.
    fn order_loops(&self, fid: i64, surface: &Surface, sense: bool, loops: Vec<Vec<Ce>>) -> R<Vec<Vec<Ce>>> {
        if let Surface::Plane { frame } = surface {
            let n = if sense { frame.z } else { -frame.z };
            let mut areas: Vec<(f64, Vec<Ce>)> = loops
                .into_iter()
                .map(|ring| {
                    let mut poly: Vec<Vec3> = vec![];
                    for ce in &ring {
                        let mut p = self.ce_points(ce, self.samples(ce));
                        p.pop();
                        poly.extend(p);
                    }
                    let mut acc = Vec3::ZERO;
                    for i in 0..poly.len() {
                        acc = acc + poly[i].cross(poly[(i + 1) % poly.len()]);
                    }
                    (0.5 * acc.dot(n), ring)
                })
                .collect();
            areas.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
            if let Some((a, _)) = areas.first() {
                if *a < 0.0 && a.abs() > 1e-9 {
                    return Err(SldprtError::new("topology.orientation", format!("face #{fid}: planar face loop runs clockwise about the face's outward normal")));
                }
            }
            return Ok(areas.into_iter().map(|(_, r)| r).collect());
        }
        if loops.len() < 2 {
            return Ok(loops);
        }
        let mut ext: Vec<(f64, Vec<Ce>)> = loops
            .into_iter()
            .map(|ring| {
                let pts: Vec<Vec3> = ring.iter().flat_map(|ce| self.ce_points(ce, 8)).collect();
                let (lo, hi) = bbox_of(&pts);
                (lo.dist(hi), ring)
            })
            .collect();
        ext.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        Ok(ext.into_iter().map(|(_, r)| r).collect())
    }
}

fn shift_pc(pc: &Curve2, du: f64, dv: f64) -> Curve2 {
    let d = Vec2::new(du, dv);
    match pc {
        Curve2::Line { o, d: dd } => Curve2::Line { o: *o + d, d: *dd },
        Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: *c + d, x: *x, r: *r, ccw: *ccw },
        Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: *c + d, x: *x, a: *a, b: *b, ccw: *ccw },
        Curve2::BSpline(b) => Curve2::BSpline(b.map(|p| p + d)),
    }
}
