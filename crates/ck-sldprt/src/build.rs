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

/// An approximated carrier that stays further than this from the exact geometry is refused (an error, not a silently wrong
/// body).
pub const MAX_APPROX_MM: f64 = 0.05;

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
    /// What became of each solid region, in file order.
    pub parts: Vec<PartRead>,
}

/// One solid region of the file as built. A face that cannot be built is left out (named in `refused`) instead of failing
/// the whole file; whether the faces that came in make a valid solid is judged afterwards (`ck_heal::honest`).
#[derive(Debug, Clone)]
pub struct PartRead {
    /// XT body id.
    pub body: i64,
    /// Faces the file gives the region.
    pub faces_total: usize,
    /// Faces built (ids in `Built::body`).
    pub faces: Vec<FaceId>,
    /// The XT face each entry of `faces` came from (its colour and feature tag are on the IR face).
    pub face_xt: Vec<i64>,
    /// Faces left out: (XT face id, reason).
    pub refused: Vec<(i64, SldprtError)>,
    /// The surface and sense of each refused face whose surface could be built: what a repair rebuilds the face on.
    pub refused_surfaces: Vec<(i64, Surface, bool)>,
    /// The faces make a sheet body of the file (not a solid region): an open surface, whatever it encloses.
    pub sheet: bool,
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

/// A point this close (mm) to a curve is on it for the validator at model tolerance 1e-6 (which accepts 1e-5 (1 + |p|)),
/// with room for our own fitted carriers.
fn on_curve(p: Vec3) -> f64 {
    2e-6 * (1.0 + p.len())
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
    curves: HashMap<(usize, Option<(u64, u64)>), Rc<FwCurve>>,
    surfaces: HashMap<usize, Rc<Surface>>,
    /// Surfaces as blend supports ([`B::support`]).
    supports: HashMap<usize, Rc<Surface>>,
    verts: HashMap<usize, (VertexId, Vec3)>,
    edges: HashMap<usize, EdgeInfo>,
    approx: Vec<Approx>,
    notes: Vec<String>,
    /// Split points (mm) per IR edge, from the previous pass.
    splits: HashMap<usize, Vec<Vec3>>,
    /// Split points this pass needed.
    need_split: Vec<(usize, Vec3)>,
    parts: Vec<PartRead>,
    /// IR edges at each IR vertex (filled on first use).
    vedges: Option<HashMap<usize, Vec<usize>>>,
}

/// Build the kernel body of one or more IRs (all solid bodies end up as lumps of one body).
pub fn build(irs: &[&Ir], limits: &Limits) -> R<Built> {
    let _ = limits;
    let mut splits: HashMap<usize, Vec<Vec3>> = HashMap::new();
    let mut last_err = None;
    for pass in 0..4 {
        // every IR keeps its own split table (edge indices are per IR)
        let mut need_all: Vec<Vec<(usize, Vec3)>> = vec![];
        let mut body = Body::new();
        let (mut solids, mut approx, mut notes, mut parts) = (0usize, vec![], vec![], vec![]);
        let mut err: Option<SldprtError> = None;
        for (k, ir) in irs.iter().enumerate() {
            let mut b = B { ir, body, curves: HashMap::new(), surfaces: HashMap::new(), supports: HashMap::new(), verts: HashMap::new(), edges: HashMap::new(), approx: vec![], notes: vec![], splits: filter_splits(&splits, k), need_split: vec![], parts: vec![], vedges: None };
            let r = b.run(&mut solids);
            need_all.push(std::mem::take(&mut b.need_split));
            approx.extend(b.approx);
            notes.extend(b.notes);
            parts.extend(b.parts);
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
                // nothing came in: the first face's reason when a face was refused
                if let Some((_, e)) = parts.iter().find_map(|p: &PartRead| p.refused.first()) {
                    return Err(e.clone());
                }
                return Err(SldprtError::new("unsupported.body", format!("no solid faces imported ({})", notes.join("; "))));
            }
            return Ok(Built { body, solids, approx, notes, parts });
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
            let sheet = body.kind == BodyKind::Sheet && ir.import_sheets;
            if body.kind != BodyKind::Solid && !sheet {
                self.notes.push(format!("body #{} is a {:?} body (not imported)", body.id, body.kind));
                continue;
            }
            for reg in &body.regions {
                if (!reg.solid && !sheet) || reg.shells.iter().all(|s| s.faces.is_empty()) {
                    continue; // void region: the infinite outside
                }
                let lump = self.body.add_lump();
                let mut shells: Vec<(ShellId, f64)> = vec![];
                let mut part = PartRead { body: body.id, faces_total: 0, faces: vec![], face_xt: vec![], refused: vec![], refused_surfaces: vec![], sheet: body.kind == BodyKind::Sheet };
                for sh in &reg.shells {
                    if sh.faces.is_empty() {
                        continue;
                    }
                    let s = self.body.add_shell(lump);
                    let mut all: Vec<Vec3> = vec![];
                    for f in &sh.faces {
                        part.faces_total += 1;
                        let n0 = self.body.faces.len();
                        match self.face(s, f) {
                            Ok(pts) => {
                                all.extend(pts);
                                part.faces.extend((n0..self.body.faces.len()).map(|i| FaceId(i as u32)));
                                part.face_xt.resize(part.faces.len(), f.id);
                            }
                            Err(e) => {
                                // a face whose surface can be built keeps it, for a repair to rebuild the face on
                                if let Some(si) = f.surface {
                                    if let Ok(sf) = self.surface(si) {
                                        part.refused_surfaces.push((f.id, (*sf).clone(), f.same_sense ^ f.back));
                                    }
                                }
                                part.refused.push((f.id, e));
                            }
                        }
                    }
                    let (lo, hi) = bbox_of(&all);
                    shells.push((s, if lo.x <= hi.x { lo.dist(hi) } else { 0.0 }));
                }
                if !sheet {
                    self.orient_shells(&shells);
                    *solids += 1;
                }
                self.parts.push(part);
            }
        }
        self.tolerant_edges();
        Ok(())
    }

    // ------------------------------------------------------------ orientation
    /// Largest shell outward (positive volume), the others (cavities) inward.
    /// A tolerant edge of the file (one with a stored tolerance, often a curve on one of its faces only) may be off its
    /// faces by up to that tolerance, and any edge may be off them by up to the tolerance of a tolerant vertex it ends at
    /// (the seam of a periodic face run to such a vertex, say): when a built edge is off its faces by more than the
    /// validator accepts at model tolerance, and by no more than that file tolerance, it is given its deviation (with a
    /// margin, at most the file's tolerance) as its own tolerance.
    fn tolerant_edges(&mut self) {
        let ir = self.ir;
        let mut file_tol: HashMap<EdgeId, f64> = HashMap::new();
        for (ei, info) in &self.edges {
            if let Some(t) = ir.edges[*ei].tol {
                for &e in &info.segs {
                    file_tol.insert(e, t * SCALE);
                }
            }
        }
        for e in self.body.edge_ids().collect::<Vec<_>>() {
            let ed = self.body.edge(e);
            let t = file_tol.get(&e).copied().unwrap_or(0.0).max(self.body.vertex(ed.v0).tol).max(self.body.vertex(ed.v1).tol);
            if !(t > 0.0) || ed.tol > 0.0 {
                continue;
            }
            let (t0, t1) = (ed.t0, ed.t1);
            let mut dev = 0.0f64;
            let mut lim = f64::INFINITY;
            for c in self.body.edge_coedges(e) {
                let surf = &self.body.face(self.body.coedge_face(c)).surface;
                let mut prev = None;
                for k in 0..=16 {
                    let s = t0 + (t1 - t0) * k as f64 / 16.0;
                    let uv = self.body.coedge_uv(c, s, prev);
                    prev = Some(uv);
                    let q = ed.curve.eval(s);
                    dev = dev.max(surf.eval(uv.x, uv.y).dist(q));
                    lim = lim.min(on_curve(q));
                }
            }
            if dev > lim && dev <= t {
                self.body.edges[e.idx()].tol = (dev * 1.5).min(t);
            }
        }
    }

    fn orient_shells(&mut self, shells: &[(ShellId, f64)]) {
        let mut order: Vec<(ShellId, f64)> = shells.to_vec();
        order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        for (i, (s, _)) in order.iter().enumerate() {
            let faces: Vec<FaceId> = self.body.shell(*s).faces.clone();
            // only the sign is used: the coarse memoised quadrature (good to about 1e-6 of the faces' shares)
            // decides it unless the total is lost among the shares (then the exact integral does)
            let vol = ck_topo::mass::signed_volume_quick(&self.body, &faces);
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
        self.curve_t(ci, None)
    }

    /// The carrier of IR curve `ci`, restricted to the parameter range `trim` (of its own parameterisation) when the
    /// curve is a B-spline or an SP-curve; analytic curves are bounded by their edge's vertices instead.
    fn curve_t(&mut self, ci: usize, trim: Option<(f64, f64)>) -> R<Rc<FwCurve>> {
        let key = (ci, trim.map(|(a, b)| (a.to_bits(), b.to_bits())));
        if let Some(c) = self.curves.get(&key) {
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
                let mut bs = self.bspline_curve(nc)?;
                if let Some((a, b)) = trim {
                    let (d0, d1) = bs.domain();
                    let (a, b) = (a.max(d0), b.min(d1));
                    if b > a && (a > d0 + 1e-12 || b < d1 - 1e-12) {
                        bs = bs.segment(a, b);
                    }
                }
                let closed = Self::is_closed(&bs);
                FwCurve { curve: Curve::BSpline(bs), period: None, dir_inc: sense, closed_spline: closed }
            }
            IrCurve::Trimmed { basis, t1, t2, .. } => {
                let bi = basis.ok_or_else(|| gerr("trimmed curve without basis"))?;
                let b = self.curve_t(bi, Some((t1.min(*t2), t1.max(*t2))))?;
                FwCurve { curve: b.curve.clone(), period: b.period, dir_inc: sense == (t2 > t1), closed_spline: b.closed_spline }
            }
            IrCurve::Intersection { pts, surfaces, bounds, chordal } => {
                let mut p: Vec<Vec3> = pts.iter().map(|q| pt(*q)).collect();
                // chart points are put exactly on both supports and the fit is refined until it stays on them
                let sups = match (surfaces[0], surfaces[1]) {
                    (Some(a), Some(b)) => match (self.surface(a), self.surface(b)) {
                        (Ok(sa), Ok(sb)) => Some([sa, sb]),
                        _ => None,
                    },
                    _ => None,
                };
                // one support and the boundary of a blend on it: the curve is where the blend's cross-sections end on
                // that support (the end of the blend's cross-section nearer the support, put onto the support)
                let bound = match (*surfaces, *bounds) {
                    ([Some(a), None], [None, Some((b, _))]) | ([None, Some(a)], [Some((b, _)), None]) => match (self.surface(a), self.surface(b)) {
                        (Ok(sa), Ok(sb)) if matches!(&*sb, Surface::BSpline(_)) => Some((sa, sb)),
                        _ => None,
                    },
                    _ => None,
                };
                let onto = |q: Vec3| -> Option<Vec3> {
                    if let Some(sup) = &sups {
                        return Self::project_offsets(sup, [0.0, 0.0], q).or_else(|| Self::project_meet(sup, [0.0, 0.0], q));
                    }
                    let (sa, sb) = bound.as_ref()?;
                    let Surface::BSpline(bb) = &**sb else { return None };
                    let (u, _) = bb.closest_param(q);
                    // past the end of the blend's cross-sections the nearest one is not where the curve is
                    let (u0, u1) = bb.domain().0;
                    if u <= u0 + 1e-9 * (u1 - u0) || u >= u1 - 1e-9 * (u1 - u0) {
                        return None;
                    }
                    let (v0, v1) = bb.domain().1;
                    let (e0, e1) = (bb.eval(u, v0), bb.eval(u, v1));
                    let e = if sa.distance(e0) <= sa.distance(e1) { e0 } else { e1 };
                    Some(sa.closest_point(e).2)
                };
                let can = sups.is_some() || bound.is_some();
                let mut bs = self.fit_points(&p)?;
                // the fit is refined until it is on the supports to a tenth of what the validator accepts at model
                // tolerance 1e-6 (10 * 1e-6 * (1 + |p|) mm): a curve that stays further off puts its edges off their faces
                let fine = |q: Vec3| 1e-6 * (1.0 + q.len());
                let strays = can && {
                    let (a, b) = bs.domain();
                    (0..48).any(|k| {
                        let q = bs.eval(a + (b - a) * (k as f64 + 0.37) / 48.0);
                        onto(q).map_or(false, |c| c.dist(q) > fine(q))
                    })
                };
                if strays {
                    // a chart point may be off the curve by the chart's chordal error
                    let snap = 1e-2f64.max(1.5 * chordal * SCALE);
                    for q in p.iter_mut() {
                        if let Some(c) = onto(*q) {
                            if c.dist(*q) < snap {
                                *q = c;
                            }
                        }
                    }
                    p.dedup_by(|a, b| a.dist(*b) <= 1e-9);
                    bs = self.fit_points(&p)?;
                    for _ in 0..6 {
                        if p.len() > 2000 {
                            break;
                        }
                        let params = fit::params(&p, Param::ChordLength);
                        let (d0, d1) = bs.domain();
                        let mut ins: Vec<(usize, Vec3)> = vec![];
                        for i in 0..p.len() - 1 {
                            let m = bs.eval(d0 + (d1 - d0) * 0.5 * (params[i] + params[i + 1]));
                            if let Some(q) = onto(m) {
                                let e = q.dist(m);
                                if e > 0.5 * fine(m) && e < 0.5 * p[i].dist(p[i + 1]) && q.dist(p[i]) > 1e-9 && q.dist(p[i + 1]) > 1e-9 {
                                    ins.push((i, q));
                                }
                            }
                        }
                        if ins.is_empty() {
                            break;
                        }
                        for (i, q) in ins.into_iter().rev() {
                            p.insert(i + 1, q);
                        }
                        bs = self.fit_points(&p)?;
                    }
                }
                let closed = Self::is_closed(&bs);
                let curve = Curve::BSpline(bs);
                // deviation from the support surfaces at 48 samples
                let mut worst = 0.0f64;
                let mut checked = 0;
                let refined_bound = strays && bound.is_some();
                for s in surfaces.iter().flatten().chain(bounds.iter().flatten().map(|b| &b.0).filter(|_| refined_bound)) {
                    if let Ok(sf) = self.surface(*s) {
                        let (a, b) = curve.range();
                        for k in 0..48 {
                            let t = a + (b - a) * (k as f64 + 0.37) / 48.0;
                            worst = worst.max(sf.distance(curve.eval(t)));
                        }
                        checked += 1;
                    }
                }
                if worst > MAX_APPROX_MM {
                    return Err(gerr(format!("intersection curve #{}: the fit stays {worst:.3e} mm from its support surfaces", node.id)));
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
                let (mut t0, mut t1) = c2.domain();
                if let Some((a, b)) = trim {
                    if b.min(t1) > a.max(t0) {
                        t0 = a.max(t0);
                        t1 = b.min(t1);
                    }
                }
                let n = (8 * uv.ctrl.len()).clamp(32, 400);
                // the parameter curve is in the file's unit: lengths in (u, v) scale to mm, angles and spline parameters do not
                let (su, sv) = match &*sf {
                    Surface::Plane { .. } => (SCALE, SCALE),
                    Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Extrusion { .. } => (1.0, SCALE),
                    _ => (1.0, 1.0),
                };
                // a spun surface's parameters are (profile, angle about the file's axis): ours are (angle about the
                // opposite axis, profile)
                let spun = matches!(self.ir.surfaces[*si].geom, IrSurface::Spun { .. });
                let eval_t = |t: f64| {
                    let q = c2.eval(t);
                    if spun {
                        sf.eval(-q.y, q.x)
                    } else {
                        sf.eval(q.x * su, q.y * sv)
                    }
                };
                // sample, fit, and add samples where the fit strays from the surface curve
                let mut tp: Vec<(f64, Vec3)> = vec![];
                for i in 0..=n {
                    let t = t0 + (t1 - t0) * i as f64 / n as f64;
                    let p = eval_t(t);
                    if tp.last().map_or(true, |l| l.1.dist(p) > 1e-9) {
                        tp.push((t, p));
                    }
                }
                let mut clean: Vec<Vec3> = tp.iter().map(|x| x.1).collect();
                let mut bs = self.fit_points(&clean)?;
                for _ in 0..5 {
                    if tp.len() > 3000 {
                        break;
                    }
                    let mut ins: Vec<(usize, (f64, Vec3))> = vec![];
                    for i in 0..tp.len() - 1 {
                        if tp[i].1.dist(tp[i + 1].1) < 1e-7 {
                            continue;
                        }
                        let tm = 0.5 * (tp[i].0 + tp[i + 1].0);
                        let pm = eval_t(tm);
                        if bs.closest_point(pm).1.dist(pm) > 1e-4 && pm.dist(tp[i].1) > 1e-9 && pm.dist(tp[i + 1].1) > 1e-9 {
                            ins.push((i, (tm, pm)));
                        }
                    }
                    if ins.is_empty() {
                        break;
                    }
                    for (i, e) in ins.into_iter().rev() {
                        tp.insert(i + 1, e);
                    }
                    clean = tp.iter().map(|x| x.1).collect();
                    bs = self.fit_points(&clean)?;
                }
                let closed = Self::is_closed(&bs);
                let curve = Curve::BSpline(bs);
                let mut worst = 0.0f64;
                let (a, b) = curve.range();
                for k in 0..48 {
                    let t = a + (b - a) * (k as f64 + 0.37) / 48.0;
                    worst = worst.max(sf.distance(curve.eval(t)));
                }
                if worst > MAX_APPROX_MM {
                    return Err(gerr(format!("SP-curve #{}: the sampled curve stays {worst:.3e} mm from its surface", node.id)));
                }
                self.approx.push(Approx { what: format!("SP-curve #{} sampled through {} points", node.id, clean.len()), max_err_mm: worst });
                FwCurve { curve, period: None, dir_inc: true, closed_spline: closed }
            }
            IrCurve::Unsupported(m) => return Err(uerr(format!("curve #{}: {m}", node.id))),
        };
        let rc = Rc::new(fc);
        self.curves.insert(key, rc.clone());
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
                // a negative major radius (smaller than the minor) is a lemon: the tube circle's centre is across the axis
                // and the face is the arc on this side of it, revolved (the same build as the apple sheet below)
                if !(*minor > 0.0) || -*major >= *minor || v3(*axis).len() < 1e-12 {
                    return Err(uerr(format!("degenerate torus, major {major}, minor {minor}")));
                }
                if minor >= major {
                    // self-intersecting (apple / lemon) torus: no kernel torus, but the same surface is the revolution
                    // of the part of the tube circle that is on one side of the axis (a rational quadratic arc of two
                    // spans: the surface the faces lie on is this outer sheet, and a profile that crosses the axis
                    // would defeat the surface-of-revolution foot search)
                    let fr = frame_from(pt(*c), v3(*axis), v3(*x));
                    let (xr, ax) = (fr.x, fr.z);
                    let (m, r) = (major * SCALE, minor * SCALE);
                    let t0 = (-m / r).acos(); // the circle meets the axis at t = +-t0 from the outer direction
                    let ctr = fr.o + xr * m;
                    let make = |dir: Vec3| -> R<Surface> {
                        let th = t0; // half-sweep of each span is t0 (two spans over -t0..t0)
                        let w = (0.5 * th).cos();
                        let at = |t: f64| ctr + (xr * t.cos() + dir * t.sin()) * r;
                        // middle control point of a span from a to b: the tangent intersection
                        let mid = |a: f64, b: f64| {
                            let h = 0.5 * (b - a);
                            let c = 0.5 * (a + b);
                            ctr + (xr * c.cos() + dir * c.sin()) * (r / h.cos())
                        };
                        let ctrl = vec![at(-t0), mid(-t0, 0.0), at(0.0), mid(0.0, t0), at(t0)];
                        let prof = BSplineCurve::new_rational(2, vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0], ctrl, vec![1.0, w, 1.0, w, 1.0]).map_err(|e| gerr(format!("apple torus arc: {e}")))?;
                        Ok(Surface::Revolution { profile: Box::new(Curve::BSpline(prof)), axis_o: fr.o, axis: ax })
                    };
                    let mut rev = make(ax)?;
                    // the natural normal of a torus points out of the tube
                    let (u, v) = (0.3f64, 0.4f64);
                    let tube = fr.o + (xr * u.cos() + fr.y * u.sin()) * m;
                    if rev.normal(u, v).dot(rev.eval(u, v) - tube) < 0.0 {
                        rev = make(-ax)?;
                    }
                    let rc = Rc::new(rev);
                    self.surfaces.insert(si, rc.clone());
                    return Ok(rc);
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
                // the file's spun surface is parameterised (profile, angle), ours (angle, profile): about the opposite axis
                // the natural normal is the file's (the side face senses, offsets and blend ranges refer to)
                Surface::Revolution { profile: Box::new(c.curve.clone()), axis_o: pt(*base), axis: -v3(*axis).normalize() }
            }
            IrSurface::Offset { base, dist } => {
                let bi = base.ok_or_else(|| gerr("offset surface without base"))?;
                let b = self.surface(bi)?;
                Surface::Offset { base: Box::new((*b).clone()), dist: dist * SCALE }
            }
            IrSurface::Blend { .. } => self.blend_surface(si, 0.0)?,
            IrSurface::Unsupported(m) => return Err(uerr(format!("surface #{}: {m}", node.id))),
        };
        let rc = Rc::new(s);
        self.surfaces.insert(si, rc.clone());
        Ok(rc)
    }

    // ------------------------------------------------------------ rolling-ball blends
    /// Surface `si` as a blend's support: spline surfaces and profiles continued past their domain ([`extended_support`]),
    /// a blend (a blend on a blend) with its cross-section arcs continued past its contact lines, where the ball of the
    /// other blend can touch it.
    fn support(&mut self, si: usize) -> R<Rc<Surface>> {
        if let Some(s) = self.supports.get(&si) {
            return Ok(s.clone());
        }
        if let IrSurface::Blend { range: [0.0, 0.0], spine: Some(sp), .. } = self.ir.surfaces[si].geom {
            // a zero-radius blend is the sharp edge along its spine: the ball touches the edge itself, so the support is
            // a pipe of negligible radius round it (the ball's centre is at the blend radius from the edge)
            let s = match self.curve(sp)?.curve {
                Curve::Line { o, d } => Surface::Cylinder { frame: frame_from(o, d, d.cross(Vec3::new(0.0, 0.0, 1.0)) + d.cross(Vec3::new(1.0, 0.0, 0.0))), r: EDGE_PIPE },
                Curve::Circle { ref frame, r } => Surface::Torus { frame: frame.clone(), major: r, minor: EDGE_PIPE },
                _ => return Err(uerr(format!("blend #{}: a support is a sharp edge along a {} (only lines and circles)", self.ir.surfaces[si].id, self.curve(sp)?.curve.kind()))),
            };
            let s = Rc::new(s);
            self.supports.insert(si, s.clone());
            return Ok(s);
        }
        let s = if matches!(self.ir.surfaces[si].geom, IrSurface::Blend { .. }) {
            // and continued a little past the ends of its spine (the other ball can touch it just past them)
            let b = self.blend_surface(si, 0.5)?;
            let e = match &b {
                Surface::BSpline(bs) => extend_surface_u(bs, SUPPORT_EXTENSION).map(|e| Surface::BSpline(Box::new(e))),
                _ => None,
            };
            Rc::new(e.unwrap_or(b))
        } else {
            let s = self.surface(si)?;
            extended_support(&s).map_or(s, Rc::new)
        };
        self.supports.insert(si, s.clone());
        Ok(s)
    }

    /// Alternating projection of `p` onto the two support surfaces offset by `off` along their natural normals
    /// (the locus of the rolling ball's centre).
    fn project_offsets(sup: &[Rc<Surface>; 2], off: [f64; 2], p: Vec3) -> Option<Vec3> {
        let mut c = p;
        // the first foot on each support is the global closest point; the iteration moves the centre a little each
        // round, so the later feet are found by Newton from the previous one (`closest_point_near` takes the global
        // search again whenever that foot is not a stationary point of the distance)
        let mut hint: [Option<(f64, f64)>; 2] = [None, None];
        let foot_of = |k: usize, q: Vec3, hint: &mut [Option<(f64, f64)>; 2]| -> (f64, f64, Vec3) {
            let r = match hint[k] {
                Some(h) => sup[k].closest_point_near(q, h),
                None => sup[k].closest_point(q),
            };
            hint[k] = Some((r.0, r.1));
            r
        };
        for _ in 0..60 {
            let c0 = c;
            for k in 0..2 {
                let (u, v, foot) = foot_of(k, c, &mut hint);
                let n = sup[k].normal(u, v);
                if n.len() < 1e-12 {
                    return None;
                }
                c = foot + n.normalize() * off[k];
            }
            if c.dist(c0) < 1e-10 {
                break;
            }
        }
        for k in 0..2 {
            let (u, v, foot) = foot_of(k, c, &mut hint);
            let n = sup[k].normal(u, v).normalize();
            if ((c - foot).dot(n) - off[k]).abs() > 1e-5 * (1.0 + off[k].abs()) || (c - foot).len() > off[k].abs() * 1.05 + 1e-6 {
                return None;
            }
        }
        Some(c)
    }

    /// [`Self::project_offsets`] by Newton steps on the two (offset) tangent planes: quadratic where the supports cross
    /// at a shallow angle, which alternating projection approaches too slowly to settle.
    fn project_meet(sup: &[Rc<Surface>; 2], off: [f64; 2], p: Vec3) -> Option<Vec3> {
        let mut c = p;
        for _ in 0..40 {
            let mut f = [Vec3::new(0.0, 0.0, 0.0); 2];
            let mut n = f;
            for k in 0..2 {
                let (u, v, foot) = sup[k].closest_point(c);
                let nn = sup[k].normal(u, v);
                if nn.len() < 1e-12 {
                    return None;
                }
                f[k] = foot;
                n[k] = nn.normalize();
            }
            let (d0, d1) = (n[0].dot(c - f[0]) - off[0], n[1].dot(c - f[1]) - off[1]);
            if d0.abs().max(d1.abs()) < 1e-10 * (1.0 + c.len()) {
                break;
            }
            let g = n[0].dot(n[1]);
            let det = 1.0 - g * g;
            if det < 1e-10 {
                return None; // tangent supports: no transversal meeting point
            }
            let a = (-d0 + g * d1) / det;
            let b = (-d1 + g * d0) / det;
            c = c + n[0] * a + n[1] * b;
            if !(c.dist(p) < 1e3) {
                return None;
            }
        }
        for k in 0..2 {
            let (u, v, foot) = sup[k].closest_point(c);
            let n = sup[k].normal(u, v).normalize();
            if ((c - foot).dot(n) - off[k]).abs() > 1e-6 * (1.0 + c.len() + off[k].abs()) || (c - foot).len() > off[k].abs() * 1.05 + 1e-6 * (1.0 + c.len()) {
                return None;
            }
        }
        Some(c)
    }

    /// Cross-section arc points from `f0` to `f1` about `c` (n points, equal angle steps); `widen` > 0 continues the arc
    /// past both feet by that fraction of its angle (the canal surface beyond its contact lines).
    fn blend_row(c: Vec3, f0: Vec3, f1: Vec3, n: usize, widen: f64) -> Vec<Vec3> {
        let r = 0.5 * (c.dist(f0) + c.dist(f1));
        let xv = (f0 - c).normalize();
        let d1 = f1 - c;
        let yv = d1 - xv * d1.dot(xv);
        if yv.len() < 1e-7 * r {
            return vec![c + xv * r; n];
        }
        let yv = yv.normalize();
        let ang = d1.dot(yv).atan2(d1.dot(xv));
        if ang <= 1e-9 {
            return vec![c + xv * r; n];
        }
        let w = widen.min(0.5 * (5.0 - ang / PI)).max(0.0); // the whole arc stays under 300 degrees
        (0..n).map(|k| {
            let a = ang * (-w + (1.0 + 2.0 * w) * k as f64 / (n - 1) as f64);
            c + xv * (r * a.cos()) + yv * (r * a.sin())
        }).collect()
    }

    /// A rolling-ball blend as an interpolated B-spline surface: u along the spine, v along the circular
    /// cross-section from the foot on support 0 to the foot on support 1 (swapped when the spine is reversed).
    /// The maximum deviation from the exact cross-sections between the sections is recorded.
    fn blend_surface(&mut self, si: usize, widen: f64) -> R<Surface> {
        let narc: usize = if widen > 0.0 { 21 } else { 13 };
        let node = &self.ir.surfaces[si];
        let IrSurface::Blend { supports, support_plus, spine, spine_plus, range, limits } = &node.geom else { unreachable!() };
        let nid = node.id;
        let (Some(a), Some(b)) = (supports[0], supports[1]) else { return Err(uerr(format!("blend #{nid}: a support is a blend boundary or a cliff edge"))) };
        let sup = [self.support(a)?, self.support(b)?];
        let sg = |p: bool| if p { 1.0 } else { -1.0 };
        let mut off = [range[0] * SCALE * sg(support_plus[0]), range[1] * SCALE * sg(support_plus[1])];
        let expect = range[0].abs().max(range[1].abs()) * SCALE;
        for (k, s) in [a, b].into_iter().enumerate() {
            if matches!(self.ir.surfaces[s].geom, IrSurface::Blend { range: [0.0, 0.0], .. }) {
                off[k] = expect - EDGE_PIPE; // out from the edge
            }
        }
        let sci = spine.ok_or_else(|| gerr(format!("blend #{nid} without spine")))?;
        let swap = !*spine_plus;
        // spine centres
        let mut centres: Vec<Vec3> = vec![];
        let mut analytic: Option<(Curve, f64, f64)> = None;
        match &self.ir.curves[sci].geom {
            IrCurve::Intersection { pts, .. } => {
                for q in pts {
                    if let Some(c) = Self::project_offsets(&sup, off, pt(*q)) {
                        if centres.last().map_or(true, |l| l.dist(c) > 1e-9) {
                            centres.push(c);
                        }
                    }
                }
            }
            _ => {
                let fc = self.curve(sci)?;
                let curve = fc.curve.clone();
                let (mut t0, mut t1) = curve.range();
                if let (Some(l0), Some(l1)) = (limits[0], limits[1]) {
                    let (mut ta, mut tb) = (curve.closest_param(pt(l0)), curve.closest_param(pt(l1)));
                    if let Some(per) = fc.period {
                        // the limits run along the spine's own direction: against the carrier's parameter when the
                        // curve is stored reversed, so the arc is the one from l1 round to l0 (not the complement)
                        if !fc.dir_inc {
                            std::mem::swap(&mut ta, &mut tb);
                        }
                        if tb <= ta {
                            tb += per;
                        }
                    }
                    t0 = ta;
                    t1 = tb;
                } else if !t0.is_finite() || !t1.is_finite() {
                    t0 = -1000.0;
                    t1 = 1000.0;
                }
                for i in 0..=64 {
                    centres.push(curve.eval(t0 + (t1 - t0) * i as f64 / 64.0));
                }
                analytic = Some((curve, t0, t1));
            }
        }
        if centres.len() < 2 {
            return Err(gerr(format!("blend #{nid}: spine too short")));
        }
        let feet_of = |c: Vec3| -> Option<(Vec3, Vec3)> {
            let f = [sup[0].closest_point(c).2, sup[1].closest_point(c).2];
            let (f0, f1) = if swap { (f[1], f[0]) } else { (f[0], f[1]) };
            let (r0, r1) = (c.dist(f0), c.dist(f1));
            let ok = (r0 - r1).abs() <= 0.02 * r0.max(r1) + 1e-9 && (expect == 0.0 || ((r0 - expect).abs() <= 0.02 * expect && (r1 - expect).abs() <= 0.02 * expect));
            ok.then_some((f0, f1))
        };
        // longest run of valid cross-sections
        let flags: Vec<Option<(Vec3, Vec3)>> = centres.iter().map(|&c| feet_of(c)).collect();
        let (mut best, mut i) = ((0usize, 0usize), 0usize);
        while i < flags.len() {
            if flags[i].is_some() {
                let mut j = i;
                while j < flags.len() && flags[j].is_some() {
                    j += 1;
                }
                if j - i > best.1 - best.0 {
                    best = (i, j);
                }
                i = j;
            } else {
                i += 1;
            }
        }
        let nvalid = best.1 - best.0;
        if nvalid < 2 || (nvalid as f64) < 0.5 * flags.len() as f64 {
            return Err(gerr(format!("blend #{nid}: support feet are not at the blend radius along the spine ({nvalid} of {} cross-sections valid)", flags.len())));
        }
        // entries: (centre, row, spine fraction)
        let nsamp = flags.len();
        let mut entries: Vec<(Vec3, Vec<Vec3>, f64)> = (best.0..best.1).map(|i| {
            let (f0, f1) = flags[i].unwrap();
            (centres[i], Self::blend_row(centres[i], f0, f1, narc, widen), i as f64 / (nsamp - 1) as f64)
        }).collect();
        entries.dedup_by(|b, a| a.0.dist(b.0) <= 1e-10);
        // the chart of an intersection spine is within its chordal error of the curve, and its end points are put back
        // onto the ball-centre locus, so the faces on the blend can run on a little past the last cross-section:
        // continue the spine past both ends (a quarter radius at a time, up to one radius) while the ball still
        // touches both supports
        if analytic.is_none() && entries.len() >= 2 && expect > 0.0 {
            for front in [false, true] {
                let n = entries.len();
                let (mut last, prev) = if front { (entries[0].0, entries[1].0) } else { (entries[n - 1].0, entries[n - 2].0) };
                let d = (last - prev).normalize();
                let step = 0.25 * expect;
                let mut more = vec![];
                for _ in 0..4 {
                    let Some(c) = Self::project_offsets(&sup, off, last + d * step) else { break };
                    let Some((f0, f1)) = feet_of(c) else { break };
                    if (c - last).dot(d) < 0.1 * step {
                        break;
                    }
                    more.push((c, Self::blend_row(c, f0, f1, narc, widen), if front { 0.0 } else { 1.0 }));
                    last = c;
                }
                if front {
                    more.reverse();
                    entries.splice(0..0, more);
                } else {
                    entries.extend(more);
                }
            }
        }
        // the natural normal of a rolling-ball blend points away from the ball's centre: v runs so that du x dv does
        let outward = |ents: &[(Vec3, Vec<Vec3>, f64)]| -> Option<bool> {
            if ents.len() < 2 {
                return None;
            }
            let i = ents.len() / 2;
            let (a, b) = (i.saturating_sub(1), (i + 1).min(ents.len() - 1));
            let du = ents[b].0 - ents[a].0;
            let m = narc / 2;
            let dv = ents[i].1[m + 1] - ents[i].1[m - 1];
            let d = du.cross(dv).dot(ents[i].1[m] - ents[i].0);
            (d.abs() > 0.0).then_some(d > 0.0)
        };
        let flip_v = outward(&entries) == Some(false);
        if flip_v {
            for e in entries.iter_mut() {
                e.1.reverse();
            }
        }
        let build = |ents: &[(Vec3, Vec<Vec3>, f64)]| -> R<(BSplineSurface, Vec<f64>)> {
            let mut u = vec![0.0];
            for k in 1..ents.len() {
                u.push(u[k - 1] + ents[k - 1].0.dist(ents[k].0));
            }
            let grid: Vec<Vec<Vec3>> = ents.iter().map(|e| e.1.clone()).collect();
            let v: Vec<f64> = (0..narc).map(|k| k as f64 / (narc - 1) as f64).collect();
            let s = fit::grid_surface(&grid, &u, &v, 3, 3).map_err(|e| gerr(format!("blend #{nid}: surface fit failed: {e}")))?;
            Ok((s, u))
        };
        let (mut surf, mut params) = build(&entries)?;
        let mut worst_final = 0.0f64;
        let mut prev_worst = f64::INFINITY;
        for round in 0..6 {
            if entries.len() >= 400 {
                break;
            }
            let mut inserts: Vec<(usize, f64, (Vec3, Vec<Vec3>, f64))> = vec![];
            let mut round_worst = 0.0f64;
            for i in 0..entries.len() - 1 {
                let (ca, cb) = (entries[i].0, entries[i + 1].0);
                if ca.dist(cb) < 1e-7 {
                    continue;
                }
                let m = ca.lerp(cb, 0.5);
                let tm = 0.5 * (entries[i].2 + entries[i + 1].2);
                let q = match &analytic {
                    Some((c, t0, t1)) => Some(c.eval(t0 + (t1 - t0) * tm)),
                    None => Self::project_offsets(&sup, off, m),
                };
                let Some(q) = q else { continue };
                if q.dist(m) > 0.5 * ca.dist(cb) || q.dist(ca) < 1e-9 || q.dist(cb) < 1e-9 {
                    continue;
                }
                let Some((f0, f1)) = feet_of(q) else { continue };
                let mut row = Self::blend_row(q, f0, f1, narc, widen);
                if flip_v {
                    row.reverse();
                }
                let um = 0.5 * (params[i] + params[i + 1]);
                let worst = (0..narc).map(|k| surf.eval(um, k as f64 / (narc - 1) as f64).dist(row[k])).fold(0.0f64, f64::max);
                round_worst = round_worst.max(worst);
                if worst > 2e-6 {
                    inserts.push((i, worst, (q, row, tm)));
                }
            }
            worst_final = round_worst;
            if inserts.is_empty() || (round > 0 && round_worst > 0.75 * prev_worst) {
                break;
            }
            prev_worst = round_worst;
            inserts.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
            let room = 400usize.saturating_sub(entries.len());
            let mut chosen: Vec<_> = inserts.into_iter().take(room).collect();
            chosen.sort_by_key(|c| c.0);
            for (i, _, e) in chosen.into_iter().rev() {
                entries.insert(i + 1, e);
            }
            let (s, p) = build(&entries)?;
            surf = s;
            params = p;
        }
        if worst_final > MAX_APPROX_MM {
            return Err(gerr(format!("blend #{nid}: the rolling-ball surface could not be reproduced within {MAX_APPROX_MM} mm (best {worst_final:.3e} mm)")));
        }
        self.approx.push(Approx { what: format!("rolling-ball blend #{nid}{} ({} sections x {narc} arc points, cubic interpolation)", if widen > 0.0 { " as a support" } else { "" }, entries.len()), max_err_mm: worst_final });
        Ok(Surface::BSpline(Box::new(surf)))
    }

    // ------------------------------------------------------------ vertices / edges
    fn vertex(&mut self, vi: usize) -> (VertexId, Vec3) {
        if let Some(v) = self.verts.get(&vi) {
            return *v;
        }
        let (p, tol) = self.vertex_position(vi);
        let id = self.body.add_vertex(p);
        self.body.vertices[id.idx()].tol = tol;
        self.verts.insert(vi, (id, p));
        (id, p)
    }

    /// Where a vertex is put, and its tolerance (mm, 0 = exact). A tolerant vertex (one with a stored tolerance) can be off
    /// the curves that meet there by up to its tolerance. When those curves come together at one point (their nearest
    /// points to the stored position agree to within [`on_curve`]), the vertex goes to that point and is exact; otherwise
    /// it stays where the file puts it and keeps the file's tolerance, as long as every curve passes within it.
    fn vertex_position(&mut self, vi: usize) -> (Vec3, f64) {
        let v = &self.ir.vertices[vi];
        let p = pt(v.p);
        let Some(tol) = v.tol.map(|t| t * SCALE) else { return (p, 0.0) };
        if self.vedges.is_none() {
            let mut m: HashMap<usize, Vec<usize>> = HashMap::new();
            for (k, e) in self.ir.edges.iter().enumerate() {
                for w in [e.start, e.end].into_iter().flatten() {
                    let l = m.entry(w).or_default();
                    if !l.contains(&k) {
                        l.push(k);
                    }
                }
            }
            self.vedges = Some(m);
        }
        let es = self.vedges.as_ref().and_then(|m| m.get(&vi)).cloned().unwrap_or_default();
        let mut feet: Vec<Vec3> = vec![];
        for ei in es {
            let e = &self.ir.edges[ei];
            let Some(ci) = e.curve.or(e.fin_curve) else { return (p, 0.0) };
            let Ok(fc) = self.curve(ci) else { return (p, 0.0) };
            let t = fc.curve.closest_param(p);
            feet.push(fc.curve.eval(t));
        }
        let worst = feet.iter().map(|f| f.dist(p)).fold(0.0, f64::max);
        // on its curves to well within what the validator accepts at model tolerance 1e-6 (1e-5 (1 + |p|) mm): exact;
        // our own carriers (fitted intersection curves) may add up to `geo_tol` to the file's gap
        if feet.is_empty() || worst <= on_curve(p) || worst > tol + geo_tol(p) {
            return (p, 0.0);
        }
        let m = feet.iter().fold(Vec3::new(0.0, 0.0, 0.0), |a, f| a + *f) * (1.0 / feet.len() as f64);
        if feet.iter().all(|f| f.dist(m) <= on_curve(m)) {
            (m, 0.0)
        } else {
            (p, tol.max(worst * (1.0 + 1e-9)))
        }
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
                let (mut ta, mut tb) = (curve.closest_param(ps), curve.closest_param(pe));
                if fc.closed_spline {
                    // a vertex at the closing point (within the vertex tolerance, not exactly) projects a little inside
                    // either end of the range: put it on the end it is nearer, for `closing_ends` to see it there
                    let (d0, d1) = curve.range();
                    let close = curve.eval(d0);
                    let near = [close.dist(ps) <= geo_tol(ps), close.dist(pe) <= geo_tol(pe)];
                    for (k, t) in [&mut ta, &mut tb].into_iter().enumerate() {
                        if near[k] && !near[1 - k] {
                            *t = if *t - d0 <= d1 - *t { d0 } else { d1 };
                        }
                    }
                }
                let miss = curve.eval(ta).dist(ps).max(curve.eval(tb).dist(pe));
                let vtol = self.body.vertex(vs).tol.max(self.body.vertex(ve).tol);
                if miss > geo_tol(ps).max(geo_tol(pe)).max(vtol) {
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
                } else if let Some((a, b)) = fc.closed_spline.then(|| closing_ends(curve.range(), ta, tb, fc.dir_inc)).flatten() {
                    // an end at the closing point of a closed spline: the end of the range the edge runs towards
                    if fc.dir_inc {
                        (a, b, vs, ve, false)
                    } else {
                        (b, a, ve, vs, true)
                    }
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
        let mut surface = (*self.surface(si)?).clone();
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
        let mut checked: Vec<Vec3> = pts.clone();
        checked.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)).then(a.z.total_cmp(&b.z)));
        checked.dedup_by(|a, b| a.x == b.x && a.y == b.y && a.z == b.z);
        for p in &checked {
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

        self.reaxis_sphere(&mut surface, &loops);
        let loops = self.closed_spline_face(&mut surface, loops);
        let loops = self.seam_wrapping(fid, &mut surface, sense, loops)?;
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

    /// A spherical patch that does not wrap the sphere's axis but touches (or nearly touches) a pole gets a new axis away
    /// from its points: the pole of the file's frame would make the uv boundary jump. The surface is the same sphere (the
    /// outward normal does not depend on the axis).
    fn reaxis_sphere(&self, surface: &mut Surface, loops: &[Vec<Ce>]) {
        if !matches!(surface, Surface::Sphere { .. }) {
            return;
        }
        if loops.is_empty() || loops.iter().flatten().any(|c| c.pc.is_some()) {
            return; // nothing to do, or a synthesised loop whose pcurves belong to this frame
        }
        let wraps = self.windings(&*surface, loops).iter().any(|&w| w != 0);
        let Surface::Sphere { frame, r } = &*surface else { return };
        let dirs: Vec<Vec3> = loops.iter().flatten().flat_map(|ce| self.ce_points(ce, 16)).map(|p| (p - frame.o).normalize()).collect();
        // normals of the great circles among the boundary arcs: an axis in such a plane makes the arc a meridian whose
        // pcurve may run over the pole
        let normals: Vec<Vec3> = loops
            .iter()
            .flatten()
            .filter_map(|ce| match &self.body.edge(ce.edge).curve {
                Curve::Circle { frame: cf, r: cr } if (cr - r).abs() < 1e-6 * r && cf.o.dist(frame.o) < 1e-6 * r => Some(cf.z),
                _ => None,
            })
            .collect();
        let near_pole = |a: Vec3| dirs.iter().map(|d| d.dot(a).abs()).fold(0.0f64, f64::max);
        let meridian = |a: Vec3| normals.iter().map(|n| n.dot(a).abs()).fold(f64::INFINITY, f64::min);
        let need = (!wraps && near_pole(frame.z) >= 0.9) || meridian(frame.z) <= 0.05;
        if dirs.is_empty() || !need {
            return;
        }
        // best of a fixed spread of candidate axes: far from the patch's points and from every great-circle plane
        let n = 96;
        let best = (0..n)
            .map(|i| {
                let z = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
                let rr = (1.0 - z * z).sqrt();
                let ph = i as f64 * 2.399_963_229_728_653;
                Vec3::new(rr * ph.cos(), rr * ph.sin(), z)
            })
            .map(|a| ((if wraps { 1.0 } else { 0.97 - near_pole(a) }).min(meridian(a).min(1.0) - 0.1), a))
            .fold((f64::NEG_INFINITY, Vec3::ZERO), |b, c| if c.0 > b.0 { c } else { b });
        if best.0 > 0.0 {
            let (o, r) = (frame.o, *r);
            *surface = Surface::Sphere { frame: frame_from(o, best.1, ck_geom::any_perp(best.1)), r };
        }
    }

    /// B-spline faces closed in one parameter (a blend round a closed spine, a spun spline) have no seam edge in
    /// the file either, and the surface's closing isoline is wherever its knot vector starts. A face that does not
    /// wrap the closed parameter but crosses that isoline gets the same surface with the isoline moved into a gap of
    /// the face (its pcurves would otherwise run off the domain). A band between two loops that each wrap once gets a
    /// seam edge along the closing isoline, moved through a vertex of the first loop; the other loop is split where
    /// it crosses (recorded for the next pass), and the loops get explicit pcurves on the two sides of the seam.
    fn closed_spline_face(&mut self, surface: &mut Surface, loops: Vec<Vec<Ce>>) -> Vec<Vec<Ce>> {
        if !matches!(surface, Surface::BSpline(_)) || loops.iter().flatten().any(|c| c.pc.is_some()) {
            return loops;
        }
        let (d, per) = match ck_geom::pcurve::closed_periods(surface) {
            (Some(p), None) => (0usize, p),
            (None, Some(p)) => (1usize, p),
            _ => return loops,
        };
        let lo_of = |s: &Surface| match s {
            Surface::BSpline(b) => if d == 0 { b.domain().0 .0 } else { b.domain().1 .0 },
            _ => 0.0,
        };
        let comp = move |q: Vec2| if d == 0 { q.x } else { q.y };
        let mut winds: Vec<i32> = vec![];
        let mut xs: Vec<f64> = vec![];
        for ring in &loops {
            let mut pts = vec![];
            for ce in ring {
                let mut p = self.ce_points(ce, 64);
                p.pop();
                pts.extend(p);
            }
            if pts.is_empty() {
                return loops;
            }
            pts.push(pts[0]);
            let uv = ck_geom::pcurve::project_points(surface, &pts);
            let w = (comp(uv[uv.len() - 1]) - comp(uv[0])) / per;
            if (w - w.round()).abs() > 0.1 {
                return loops;
            }
            winds.push(w.round() as i32);
            xs.extend(uv.iter().map(|q| comp(*q)));
        }
        let lo = lo_of(surface);
        if winds.iter().all(|&w| w == 0) {
            // the widest gap between the face's parameters, folded into one period
            let mut f: Vec<f64> = xs.iter().map(|x| (x - lo).rem_euclid(per)).collect();
            f.sort_by(f64::total_cmp);
            let mut best = (0.0f64, 0.0f64, false);
            for i in 0..f.len() {
                let last = i + 1 == f.len();
                let (a, b) = (f[i], if last { f[0] + per } else { f[i + 1] });
                if b - a > best.0 {
                    best = (b - a, a + 0.5 * (b - a), last);
                }
            }
            // (the widest gap already holds the closing isoline: nothing to move)
            if best.2 || best.0 < 1e-3 * per {
                return loops;
            }
            if let Some(s) = rotate_closed(surface, d, lo + best.1) {
                *surface = s;
            }
            return loops;
        }
        let wrap: Vec<usize> = (0..loops.len()).filter(|&i| winds[i] != 0).collect();
        if wrap.len() != 2 || winds[wrap[0]].abs() != 1 || winds[wrap[0]] != -winds[wrap[1]] {
            return loops;
        }
        // closing isoline through a vertex of the first loop: one with a partner at the same parameter on the other
        // loop when there is one, else the first (and the other loop is split where it crosses)
        let xof = |p: Vec3| {
            let (u, v) = surface.closest_param(p);
            comp(Vec2::new(u, v))
        };
        let xa_all: Vec<f64> = loops[wrap[0]].iter().map(|c| xof(self.vpos(self.ce_start(c)))).collect();
        let xb_all: Vec<f64> = loops[wrap[1]].iter().map(|c| xof(self.vpos(self.ce_start(c)))).collect();
        let near = |x: f64, y: f64| {
            let r = (x - y).rem_euclid(per);
            r.min(per - r) < 1e-7 * per
        };
        let ka = (0..xa_all.len()).find(|&i| xb_all.iter().any(|&y| near(xa_all[i], y))).unwrap_or(0);
        let mut a = loops[wrap[0]].clone();
        a.rotate_left(ka);
        let xa = xa_all[ka];
        let mut surf = surface.clone();
        if !near(xa, lo) {
            match rotate_closed(&surf, d, lo + (xa - lo).rem_euclid(per)) {
                Some(s) => surf = s,
                None => return loops,
            }
        }
        let lo = lo_of(&surf);
        let on_iso = |p: Vec3| {
            let (u, v) = surf.closest_param(p);
            let x = (comp(Vec2::new(u, v)) - lo).rem_euclid(per);
            x.min(per - x) < 1e-7 * per && surf.eval(u, v).dist(p) < 1e-3
        };
        // the second loop: a vertex on the isoline, else a split where it crosses it
        let mut b = loops[wrap[1]].clone();
        match (0..b.len()).find(|&k| on_iso(self.vpos(self.ce_start(&b[k])))) {
            Some(k) => b.rotate_left(k),
            None => {
                for ce in &b {
                    if ce.ir_edge == usize::MAX {
                        continue;
                    }
                    if let Some(p) = self.iso_crossing(&surf, d, per, lo, ce) {
                        self.need_split.push((ce.ir_edge, p));
                        break;
                    }
                }
                return loops;
            }
        }
        if !on_iso(self.vpos(self.ce_start(&a[0]))) {
            return loops;
        }
        let Some(mut pa_c) = self.chain_pcurves(&surf, d, per, &a, lo) else { return loops };
        let (ta0, _) = self.ce_ends(&a[0]);
        let (_, ta1) = self.ce_ends(&a[a.len() - 1]);
        let mut end_a = comp(pa_c[pa_c.len() - 1].eval(ta1));
        if (end_a - (lo - per)).abs() < 1e-6 * per {
            pa_c = pa_c.iter().map(|c| shift_pc(c, if d == 0 { per } else { 0.0 }, if d == 1 { per } else { 0.0 })).collect();
            end_a += per;
        }
        let start_a = comp(pa_c[0].eval(ta0));
        if (end_a - start_a).abs() < 0.5 * per {
            return loops;
        }
        let Some(pb_c) = self.chain_pcurves(&surf, d, per, &b, end_a) else { return loops };
        let (tb0, _) = self.ce_ends(&b[0]);
        let (_, tb1) = self.ce_ends(&b[b.len() - 1]);
        if (comp(pb_c[pb_c.len() - 1].eval(tb1)) - start_a).abs() > 1e-6 * per {
            return loops;
        }
        let other = |q: Vec2| if d == 0 { q.y } else { q.x };
        let (ya, yb) = (other(pa_c[0].eval(ta0)), other(pb_c[0].eval(tb0)));
        if (ya - yb).abs() < 1e-12 {
            return loops;
        }
        let Surface::BSpline(bs) = &surf else { return loops };
        let iso = if d == 0 { bs.iso_u(lo) } else { bs.iso_v(lo) };
        let (va, vb) = (self.ce_start(&a[0]), self.ce_start(&b[0]));
        if iso.eval(ya).dist(self.vpos(va)) > 1e-3 || iso.eval(yb).dist(self.vpos(vb)) > 1e-3 {
            return loops;
        }
        let (y0, y1, v0, v1) = if ya < yb { (ya, yb, va, vb) } else { (yb, ya, vb, va) };
        let seam = self.body.add_edge(Curve::BSpline(iso), y0, y1, v0, v1);
        let line = |x: f64| if d == 0 { Curve2::Line { o: Vec2::new(x, 0.0), d: Vec2::new(0.0, 1.0) } } else { Curve2::Line { o: Vec2::new(0.0, x), d: Vec2::new(1.0, 0.0) } };
        let up = ya < yb;
        let mut joined: Vec<Ce> = a.into_iter().zip(pa_c).map(|(c, p)| Ce { pc: Some(p), ..c }).collect();
        joined.push(Ce { pc: Some(line(end_a)), ..Ce::new(seam, !up) });
        joined.extend(b.into_iter().zip(pb_c).map(|(c, p)| Ce { pc: Some(p), ..c }));
        joined.push(Ce { pc: Some(line(start_a)), ..Ce::new(seam, up) });
        *surface = surf;
        let mut out = vec![joined];
        for (i, l) in loops.into_iter().enumerate() {
            if !wrap.contains(&i) {
                out.push(l);
            }
        }
        out
    }

    /// Start and end parameter of a coedge along its own direction.
    fn ce_ends(&self, ce: &Ce) -> (f64, f64) {
        let e = self.body.edge(ce.edge);
        if ce.rev {
            (e.t1, e.t0)
        } else {
            (e.t0, e.t1)
        }
    }

    /// Pcurves of a chain of coedges on a surface closed in parameter `d`, joined up by whole periods, the first
    /// starting nearest `start` in that parameter. Each is fitted through projections taken along the coedge, every one
    /// on the branch nearest the one before (an end on the closing isoline projects to either side of it).
    /// `None` when a pcurve cannot be made or strays from its edge.
    fn chain_pcurves(&self, surf: &Surface, d: usize, per: f64, ring: &[Ce], start: f64) -> Option<Vec<Curve2>> {
        let mut out = vec![];
        let mut at = start;
        let x = |q: Vec2| if d == 0 { q.x } else { q.y };
        for ce in ring {
            let e = self.body.edge(ce.edge);
            let n = if matches!(e.curve, Curve::BSpline(_)) { 160 } else { 48 };
            let (ts, te) = self.ce_ends(ce);
            let mut tv: Vec<(f64, Vec2)> = Vec::with_capacity(n + 1);
            for i in 0..=n {
                let t = ts + (te - ts) * i as f64 / n as f64;
                let (u, v) = surf.closest_param(e.curve.eval(t));
                let mut q = Vec2::new(u, v);
                let near = tv.last().map_or(at, |l| x(l.1));
                let k = ((near - x(q)) / per).round() * per;
                if d == 0 {
                    q.x += k;
                } else {
                    q.y += k;
                }
                tv.push((t, q));
            }
            at = x(tv[n].1);
            if ts > te {
                tv.reverse();
            }
            let (t0, t1) = (e.t0, e.t1);
            let u: Vec<f64> = tv.iter().map(|p| (p.0 - t0) / (t1 - t0)).collect();
            let uv: Vec<Vec2> = tv.iter().map(|p| p.1).collect();
            let knots = fit::averaged_knots(&u, 3);
            let bc = fit::interpolate_with(&uv, 3, &u, knots).ok()?;
            let pc = Curve2::BSpline(bc.reparam(t0, t1));
            let lo = match surf {
                Surface::BSpline(b) => if d == 0 { b.domain().0 .0 } else { b.domain().1 .0 },
                _ => 0.0,
            };
            for i in 0..=32 {
                let t = t0 + (t1 - t0) * (i as f64 + 0.5) / 33.0;
                let mut q = pc.eval(t);
                if d == 0 {
                    q.x = lo + (q.x - lo).rem_euclid(per);
                } else {
                    q.y = lo + (q.y - lo).rem_euclid(per);
                }
                if surf.eval(q.x, q.y).dist(e.curve.eval(t)) > MAX_APPROX_MM {
                    return None;
                }
            }
            out.push(pc);
        }
        Some(out)
    }

    /// Where a coedge crosses the closing isoline (`lo` + whole periods) of a surface closed in parameter `d`.
    fn iso_crossing(&self, surf: &Surface, d: usize, per: f64, lo: f64, ce: &Ce) -> Option<Vec3> {
        let e = self.body.edge(ce.edge);
        let n = 256;
        let ts: Vec<f64> = (0..=n).map(|i| e.t0 + (e.t1 - e.t0) * i as f64 / n as f64).collect();
        let pts: Vec<Vec3> = ts.iter().map(|&t| e.curve.eval(t)).collect();
        let uv = ck_geom::pcurve::project_points(surf, &pts);
        let x = |q: Vec2| if d == 0 { q.x } else { q.y };
        let level = |v: f64| ((v - lo) / per).floor();
        for i in 0..n {
            let (xa, xb) = (x(uv[i]), x(uv[i + 1]));
            if level(xa) == level(xb) {
                continue;
            }
            let target = lo + level(xa.max(xb)) * per;
            let (mut a, mut b, mut fa) = (ts[i], ts[i + 1], xa);
            for _ in 0..60 {
                let m = 0.5 * (a + b);
                let (pu, pv) = surf.closest_param(e.curve.eval(m));
                let xm = ck_geom::pcurve::unwrap_near(x(Vec2::new(pu, pv)), 0.5 * (fa + xb), per);
                if (xm - target).signum() == (fa - target).signum() {
                    a = m;
                    fa = xm;
                } else {
                    b = m;
                }
            }
            return Some(e.curve.eval(0.5 * (a + b)));
        }
        None
    }

    /// Total u-winding (in turns) of each loop on a u-periodic surface.
    fn windings(&self, surface: &Surface, loops: &[Vec<Ce>]) -> Vec<i32> {
        let Some(frame) = axis_frame(surface) else { return vec![0; loops.len()] };
        let frame = &frame;
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
        let on_axis = |p: Vec3| {
            let d = p - frame.o;
            d.dot(frame.x).hypot(d.dot(frame.y)) < 1e-9 * (1.0 + d.len())
        };
        for (k, ce) in ring.iter().enumerate() {
            let p = self.vpos(self.ce_start(ce));
            if !on_axis(p) && wrap(ang(p) - ua).abs() < 1e-6 {
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
    fn seam_wrapping(&mut self, fid: i64, surface: &mut Surface, sense: bool, mut loops: Vec<Vec<Ce>>) -> R<Vec<Vec<Ce>>> {
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
            Surface::Revolution { .. } if wrap.len() == 2 => axis_frame(surface).expect("revolution frame"),
            _ => return Err(uerr(format!("face #{fid}: {} face with a loop wrapping its axis (needs poles/seams)", surface.kind()))),
        };
        if wrap.len() == 1 {
            if let Surface::Cone { r, semi_angle, .. } = surface.clone() {
                return self.cone_cap_seam(fid, surface, loops, wrap[0], r, semi_angle);
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
        if let Surface::Revolution { profile, axis_o, axis } = &*surface {
            // the profile turned to the seam's azimuth, between the two loops' points on it
            let (u, ta) = surface.closest_param(pa);
            let (_, tb) = surface.closest_param(pb);
            let (o, z) = (*axis_o, *axis);
            let (su, cu) = u.sin_cos();
            let rot = move |w: Vec3| w * cu + z * (z.dot(w) * (1.0 - cu)) + z.cross(w) * su;
            let curve = profile.transform(|p| o + rot(p - o), rot);
            if curve.eval(ta).dist(pa) > 10.0 * geo_tol(pa) || curve.eval(tb).dist(pb) > 10.0 * geo_tol(pb) || (ta - tb).abs() < 1e-12 {
                return Err(gerr(format!("face #{fid}: revolution seam does not meet its loops")));
            }
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
    fn cone_cap_seam(&mut self, fid: i64, surface: &mut Surface, loops: Vec<Vec<Ce>>, wi: usize, r: f64, semi: f64) -> R<Vec<Vec<Ce>>> {
        // Same cone, expressed with the axis pointing towards the apex (negative semi-angle, apex at v > 0): the
        // layout the kernel's own revolved cones have (u and v both reverse, so the natural normal is unchanged).
        let (r, semi) = if semi > 0.0 {
            if let Surface::Cone { frame, semi_angle, .. } = surface {
                *frame = ck_geom::Frame { o: frame.o, x: frame.x, y: -frame.y, z: -frame.z };
                *semi_angle = -*semi_angle;
            }
            (r, -semi)
        } else {
            (r, semi)
        };
        let Surface::Cone { frame, .. } = &*surface else { unreachable!() };
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
                return Err(uerr(format!("face #{fid}: sphere face bounded by a great circle through the poles")));
            }
            Ok(h.normalize())
        };
        let mut out: Vec<Vec<Ce>> = vec![];
        let azimuth_ok = |p: Vec3| {
            let d = p - c;
            (d - axis * d.dot(axis)).len() >= 1e-9 * r.max(1.0)
        };
        let off_axis = |this: &Self, ring: &mut Vec<Ce>| {
            if let Some(k) = (0..ring.len()).find(|&k| azimuth_ok(this.vpos(this.ce_start(&ring[k])))) {
                ring.rotate_left(k);
            }
        };
        if wrap.len() == 1 {
            let mut ring = loops[wrap[0]].clone();
            off_axis(self, &mut ring);
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
            let mut a = loops[wrap[0]].clone();
            off_axis(self, &mut a);
            let mut bl = loops[wrap[1]].clone();
            let va = self.ce_start(&a[0]);
            let pa = self.vpos(va);
            let ea = azimuth(pa)?;
            let ua = (pa - c).dot(frame.y).atan2((pa - c).dot(frame.x));
            let Some(k) = self.vertex_at_azimuth(&bl, frame, ua) else {
                if self.need_split.is_empty() {
                    return Err(uerr(format!("face #{fid}: sphere band whose two loops have no vertices at a common azimuth")));
                }
                return Ok(loops);
            };
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

/// Start and end parameter (in travel order) of an edge from `ta` to `tb` on a closed spline over `range`, running
/// with increasing parameter when `inc`, when one end sits at the closing point and must be read as the other end of the
/// range for the edge to run its way (`None`: the plain order already does, or neither end is at the closing point).
fn closing_ends((d0, d1): (f64, f64), ta: f64, tb: f64, inc: bool) -> Option<(f64, f64)> {
    let at = |t: f64, d: f64| (t - d).abs() <= 1e-7 * (d1 - d0);
    match inc {
        true if ta > tb && at(tb, d0) => Some((ta, d1)),
        true if ta > tb && at(ta, d1) => Some((d0, tb)),
        false if ta < tb && at(ta, d0) => Some((d1, tb)),
        false if ta < tb && at(tb, d1) => Some((ta, d0)),
        _ => None,
    }
}

/// Frame about the axis of a surface periodic in u (z along the axis): azimuths for loop windings and seams.
fn axis_frame(surface: &Surface) -> Option<ck_geom::Frame> {
    match surface {
        Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Sphere { frame, .. } | Surface::Torus { frame, .. } => Some(frame.clone()),
        Surface::Revolution { profile, axis_o, axis } => {
            let (a, b) = profile.range();
            let t = if a.is_finite() && b.is_finite() { 0.5 * (a + b) } else { 0.0 };
            let off = profile.eval(t) - *axis_o;
            Some(frame_from(*axis_o, *axis, off))
        }
        _ => None,
    }
}

/// The same surface, closed in parameter `d`, with its closing isoline moved to `s` (the parameter range becomes
/// `[s, s + period]`): the knot `s` is raised to multiplicity `degree` and the two halves of the net swap places. Only for a
/// clamped net whose end rows coincide; `None` when the result does not reproduce the surface.
fn rotate_closed(surface: &Surface, d: usize, s: f64) -> Option<Surface> {
    let Surface::BSpline(orig) = surface else { return None };
    let bs: BSplineSurface = if d == 0 { (**orig).clone() } else { orig.transposed() };
    let bs = clamp_u(&bs)?;
    let (p, n) = (bs.deg_u, bs.nu);
    let k = &bs.knots_u;
    let (u0, u1) = (k[p], k[n]);
    let per = u1 - u0;
    if !(s > u0 + 1e-9 * per && s < u1 - 1e-9 * per) || k[..=p].iter().any(|&x| x != u0) || k[n..].iter().any(|&x| x != u1) {
        return None;
    }
    let m = k.iter().filter(|&&x| (x - s).abs() <= 1e-12 * per).count();
    if m > p {
        return None;
    }
    let ins = if m < p { bs.insert_knot_u(s, p - m) } else { bs.clone() };
    let kk = &ins.knots_u;
    let j = kk.iter().position(|&x| (x - s).abs() <= 1e-12 * per)?;
    let n2 = ins.nu;
    let nv = ins.nv;
    let mut knots = vec![s; p + 1];
    knots.extend(kk[j + p..n2].iter().copied());
    knots.extend(std::iter::repeat(u1).take(p));
    knots.extend(kk[p + 1..j].iter().map(|x| x + per));
    knots.extend(std::iter::repeat(s + per).take(p + 1));
    let rows: Vec<usize> = (j - 1..n2).chain(1..j).collect();
    if rows.len() != n2 || knots.len() != n2 + p + 1 {
        return None;
    }
    let ctrl: Vec<Vec3> = rows.iter().flat_map(|&i| (0..nv).map(move |c| (i, c))).map(|(i, c)| ins.cp(i, c)).collect();
    let weights = ins.weights.as_ref().map(|_| rows.iter().flat_map(|&i| (0..nv).map(move |c| (i, c))).map(|(i, c)| ins.weight(i, c)).collect());
    let out = BSplineSurface::new(p, ins.deg_v, knots, ins.knots_v.clone(), n2, nv, ctrl, weights).ok()?;
    // the same surface: u maps to u (or u + period below s)
    let ((_, _), (v0, v1)) = out.domain();
    let scale = 1.0 + bs.eval(u0, v0).len();
    for a in 0..12 {
        for b in 0..5 {
            let u = u0 + per * (a as f64 + 0.31) / 12.0;
            let v = v0 + (v1 - v0) * (b as f64 + 0.43) / 5.0;
            let un = if u < s { u + per } else { u };
            if out.eval(un, v).dist(bs.eval(u, v)) > 1e-9 * scale {
                return None;
            }
        }
    }
    let out = if d == 0 { out } else { out.transposed() };
    Some(Surface::BSpline(Box::new(out)))
}

/// The same surface with a clamped knot vector in u (an unclamped, periodic-style net is cut to its domain).
fn clamp_u(bs: &BSplineSurface) -> Option<BSplineSurface> {
    let (u0, u1) = bs.domain().0;
    let s = reverse_u(&clamp_start_u(&reverse_u(&clamp_start_u(bs)?))?);
    // the same surface over the same domain
    let ((_, _), (v0, v1)) = s.domain();
    let scale = 1.0 + bs.eval(u0, v0).len();
    for a in 0..=10 {
        for b in 0..5 {
            let (u, v) = (u0 + (u1 - u0) * a as f64 / 10.0, v0 + (v1 - v0) * (b as f64 + 0.43) / 5.0);
            if s.eval(u, v).dist(bs.eval(u, v)) > 1e-9 * scale {
                return None;
            }
        }
    }
    Some(s)
}

/// Knots at the start of the u domain raised to multiplicity `degree + 1` (the net cut there).
fn clamp_start_u(bs: &BSplineSurface) -> Option<BSplineSurface> {
    let p = bs.deg_u;
    let u0 = bs.domain().0 .0;
    let tol = 1e-12 * (1.0 + u0.abs() + (bs.domain().0 .1 - u0).abs());
    let mult = |k: &[f64]| k.iter().filter(|&&y| (y - u0).abs() <= tol).count();
    if bs.knots_u[..=p].iter().all(|&x| (x - u0).abs() <= tol) {
        return Some(bs.clone());
    }
    let m = mult(&bs.knots_u);
    let s = if m < p { bs.insert_knot_u(u0, p - m) } else { bs.clone() };
    let k = &s.knots_u;
    let j0 = k.iter().position(|&y| (y - u0).abs() <= tol)?;
    if mult(k) != p || j0 == 0 {
        return None;
    }
    let i0 = j0 - 1;
    let mut knots = vec![u0; p + 1];
    knots.extend(k[j0 + p..].iter().copied());
    let nu = s.nu - i0;
    let nv = s.nv;
    let ctrl: Vec<Vec3> = s.ctrl[i0 * nv..].to_vec();
    let weights = s.weights.as_ref().map(|w| w[i0 * nv..].to_vec());
    BSplineSurface::new(p, s.deg_v, knots, s.knots_v.clone(), nu, nv, ctrl, weights).ok()
}

/// The same surface with u running the other way (u -> -u).
fn reverse_u(bs: &BSplineSurface) -> BSplineSurface {
    let nv = bs.nv;
    let rows: Vec<usize> = (0..bs.nu).rev().collect();
    BSplineSurface {
        knots_u: bs.knots_u.iter().rev().map(|x| -x).collect(),
        ctrl: rows.iter().flat_map(|&i| bs.ctrl[i * nv..(i + 1) * nv].iter().copied()).collect(),
        weights: bs.weights.as_ref().map(|w| rows.iter().flat_map(|&i| w[i * nv..(i + 1) * nv].iter().copied()).collect()),
        ..bs.clone()
    }
}

/// Radius (mm) of the pipe that stands for a sharp edge a blend's ball runs along.
const EDGE_PIPE: f64 = 1e-7;

/// Fraction of a B-spline's parameter range added at each end when it is extended as a blend support.
const SUPPORT_EXTENSION: f64 = 0.5;

/// `c` with its end span continued as the same polynomial (rational: in homogeneous coordinates) by `len` past the end of
/// the domain, at most by the end span's own length (further out the polynomial wanders off); unchanged on the domain.
fn extend_curve_hi(c: &BSplineCurve, len: f64) -> Option<BSplineCurve> {
    let p = c.degree;
    let c = c.clamp_ends();
    let (a, b) = c.domain();
    let k = c.knots.iter().copied().filter(|&k| k < b && k > a).fold(a, f64::max);
    let len = len.min(b - k);
    if p == 0 || !(len > 0.0) || !(b > a) {
        return None;
    }
    let (left, right) = if k > a { let (l, r) = c.split(k); (Some(l), r) } else { (None, c.clone()) };
    if right.ctrl.len() != p + 1 {
        return None;
    }
    let mut q: Vec<[f64; 4]> = (0..=p).map(|i| {
        let w = right.weight(i);
        let x = right.ctrl[i];
        [x.x * w, x.y * w, x.z * w, w]
    }).collect();
    let t = (b + len - k) / (b - k);
    let mut ext = vec![q[0]];
    for r in 1..=p {
        for i in 0..=p - r {
            for d in 0..4 {
                q[i][d] = (1.0 - t) * q[i][d] + t * q[i + 1][d];
            }
        }
        ext.push(q[0]);
    }
    if ext.iter().any(|h| !(h[3] > 1e-9)) {
        return None; // a weight would vanish: no extension
    }
    let (mut ctrl, mut wts, mut knots) = (vec![], vec![], vec![]);
    if let Some(l) = &left {
        for i in 0..l.ctrl.len() - 1 {
            ctrl.push(l.ctrl[i]);
            wts.push(l.weight(i));
        }
        knots.extend_from_slice(&l.knots[..l.ctrl.len()]);
        knots.extend(std::iter::repeat(k).take(p));
    } else {
        knots.extend(std::iter::repeat(k).take(p + 1));
    }
    for h in &ext {
        ctrl.push(Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]));
        wts.push(h[3]);
    }
    knots.extend(std::iter::repeat(b + len).take(p + 1));
    if c.weights.is_some() { BSplineCurve::new_rational(p, knots, ctrl, wts).ok() } else { BSplineCurve::new(p, knots, ctrl).ok() }
}

/// `c` continued past both ends of its domain by `frac` of its parameter range (the end spans' polynomials).
fn extend_curve(c: &BSplineCurve, frac: f64) -> Option<BSplineCurve> {
    let (a, b) = c.domain();
    let len = (b - a) * frac;
    let hi = extend_curve_hi(c, len)?;
    let (_, b2) = hi.domain();
    let lo = extend_curve_hi(&hi.reversed(), len)?.reversed();
    // the two reversals move every parameter up by the low end's extension: shift back so that the old domain keeps its
    // parameters
    let shift = lo.domain().1 - b2;
    let mut out = lo;
    for k in out.knots.iter_mut() {
        *k -= shift;
    }
    Some(out)
}

/// `s` continued past both ends of its u domain (each row of the control net as a curve in u).
fn extend_surface_u(s: &BSplineSurface, frac: f64) -> Option<BSplineSurface> {
    let mut rows = vec![];
    for j in 0..s.nv {
        let ctrl: Vec<Vec3> = (0..s.nu).map(|i| s.cp(i, j)).collect();
        let c = match &s.weights {
            Some(_) => BSplineCurve::new_rational(s.deg_u, s.knots_u.clone(), ctrl, (0..s.nu).map(|i| s.weight(i, j)).collect()).ok()?,
            None => BSplineCurve::new(s.deg_u, s.knots_u.clone(), ctrl).ok()?,
        };
        rows.push(extend_curve(&c, frac)?);
    }
    let (nu, knots) = (rows[0].ctrl.len(), rows[0].knots.clone());
    if rows.iter().any(|r| r.ctrl.len() != nu || r.knots != knots) {
        return None;
    }
    let mut ctrl = vec![Vec3::new(0.0, 0.0, 0.0); nu * s.nv];
    let mut w = vec![1.0; nu * s.nv];
    for (j, r) in rows.iter().enumerate() {
        for i in 0..nu {
            ctrl[i * s.nv + j] = r.ctrl[i];
            w[i * s.nv + j] = r.weight(i);
        }
    }
    BSplineSurface::new(s.deg_u, s.deg_v, knots, s.knots_v.clone(), nu, s.nv, ctrl, s.weights.as_ref().map(|_| w)).ok()
}

/// A blend support as the blend sees it: B-spline profiles and surfaces continued past their domain (a blend's spine can
/// run on past the end of a support's spline, where its contact is on the support's natural extension). `None` when
/// nothing needs extending (analytic surfaces) or the extension cannot be made.
fn extended_support(s: &Surface) -> Option<Surface> {
    let ext_c = |c: &Curve| match c {
        Curve::BSpline(b) if b.start().dist(b.end()) > 1e-6 => extend_curve(b, SUPPORT_EXTENSION).map(Curve::BSpline),
        _ => None,
    };
    match s {
        Surface::Extrusion { profile, dir } => Some(Surface::Extrusion { profile: Box::new(ext_c(profile)?), dir: *dir }),
        Surface::Revolution { profile, axis_o, axis } => Some(Surface::Revolution { profile: Box::new(ext_c(profile)?), axis_o: *axis_o, axis: *axis }),
        Surface::BSpline(b) => {
            // a direction in which the surface closes up is not extended (the continuation would leave the surface)
            let ((u0, u1), (v0, v1)) = b.domain();
            let closed_u = (0..=8).all(|k| {
                let v = v0 + (v1 - v0) * k as f64 / 8.0;
                b.eval(u0, v).dist(b.eval(u1, v)) < 1e-6
            });
            let closed_v = (0..=8).all(|k| {
                let u = u0 + (u1 - u0) * k as f64 / 8.0;
                b.eval(u, v0).dist(b.eval(u, v1)) < 1e-6
            });
            let mut e = (**b).clone();
            if !closed_u {
                e = extend_surface_u(&e, SUPPORT_EXTENSION)?;
            }
            if !closed_v {
                e = extend_surface_u(&e.transposed(), SUPPORT_EXTENSION)?.transposed();
            }
            (!closed_u || !closed_v).then(|| Surface::BSpline(Box::new(e)))
        }
        Surface::Offset { base, dist } => Some(Surface::Offset { base: Box::new(extended_support(base)?), dist: *dist }),
        _ => None,
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A closed cubic tube with uniform (unclamped) knots in u: rows wrap round, the last three repeat the first.
    fn periodic_tube() -> BSplineSurface {
        let m = 8; // distinct rows
        let p = 3;
        let nu = m + p;
        let mut ctrl = vec![];
        for i in 0..nu {
            let a = TAU * (i % m) as f64 / m as f64;
            for j in 0..2 {
                ctrl.push(Vec3::new(10.0 * a.cos(), 7.0 * a.sin(), 5.0 * j as f64));
            }
        }
        let knots_u: Vec<f64> = (0..nu + p + 1).map(|k| k as f64).collect();
        BSplineSurface::new(p, 1, knots_u, vec![0.0, 0.0, 1.0, 1.0], nu, 2, ctrl, None).unwrap()
    }

    #[test]
    fn closing_isoline_moves_and_the_surface_stays() {
        let bs = periodic_tube();
        let s0 = Surface::BSpline(Box::new(bs.clone()));
        let (cu, cv) = ck_geom::pcurve::closed_periods(&s0);
        assert!(cu.is_some() && cv.is_none());
        let (u0, u1) = bs.domain().0;
        let c = clamp_u(&bs).unwrap();
        assert_eq!(c.domain().0, (u0, u1));
        for s in [u0 + 0.37 * (u1 - u0), u0 + 0.5 * (u1 - u0), u0 + 1.0] {
            let Some(Surface::BSpline(r)) = rotate_closed(&s0, 0, s) else { panic!("rotation at {s}") };
            assert!((r.domain().0 .0 - s).abs() < 1e-12 && (r.domain().0 .1 - s - (u1 - u0)).abs() < 1e-12);
            for k in 0..40 {
                let u = u0 + (u1 - u0) * (k as f64 + 0.5) / 40.0;
                let un = if u < s { u + (u1 - u0) } else { u };
                assert!(r.eval(un, 0.3).dist(bs.eval(u, 0.3)) < 1e-9);
            }
            // still closed, now at s
            assert!(ck_geom::pcurve::closed_periods(&Surface::BSpline(r.clone())).0.is_some());
        }
        // the same in v
        let t = Surface::BSpline(Box::new(bs.transposed()));
        let Some(Surface::BSpline(r)) = rotate_closed(&t, 1, u0 + 1.5) else { panic!("rotation in v") };
        assert!(r.eval(0.3, u0 + 2.0).dist(bs.eval(u0 + 2.0, 0.3)) < 1e-9);
    }

    /// A barrel: a lemon torus (negative major radius, the tube centre across the axis) between two planar caps, all
    /// in one shell. Every face is built (the lemon read as a revolution with a seam between its two rims) and the
    /// volume is the integral of the barrel's cross-sections.
    #[test]
    fn lemon_torus_barrel_is_a_closed_solid_with_its_volume() {
        let (m, rr, h) = (-0.03f64, 0.04f64, 0.02f64);
        let rad = |z: f64| m + (rr * rr - z * z).sqrt();
        let sn = |id, geom| IrSurfaceNode { id, geom, sense_plus: true };
        let surfaces = vec![
            sn(1, IrSurface::Torus { c: [0.0; 3], axis: [0.0, 0.0, 1.0], major: m, minor: rr, x: [1.0, 0.0, 0.0] }),
            sn(2, IrSurface::Plane { o: [0.0, 0.0, h], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0] }),
            sn(3, IrSurface::Plane { o: [0.0, 0.0, -h], n: [0.0, 0.0, -1.0], x: [1.0, 0.0, 0.0] }),
        ];
        let circle = |id, z: f64| IrCurveNode { id, geom: IrCurve::Circle { c: [0.0, 0.0, z], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0], r: rad(z) }, sense_plus: true };
        let curves = vec![circle(10, h), circle(11, -h)];
        let vertices = vec![IrVertex { id: 20, p: [rad(h), 0.0, h], tol: None }, IrVertex { id: 21, p: [rad(-h), 0.0, -h], tol: None }];
        let edges = vec![IrEdge { id: 30, curve: Some(0), start: Some(0), end: Some(0), fin_curve: None, tol: None }, IrEdge { id: 31, curve: Some(1), start: Some(1), end: Some(1), fin_curve: None, tol: None }];
        let lp = |id, edge, forward| IrLoop { id, fins: vec![IrFin { edge, forward }], vertex: None };
        let face = |id, surface, loops| IrFace { id, surface: Some(surface), same_sense: true, loops, back: false, colour: None, sw_id: None, sw_ids: vec![] };
        let faces = vec![face(40, 0, vec![lp(50, 0, false), lp(51, 1, true)]), face(41, 1, vec![lp(52, 0, true)]), face(42, 2, vec![lp(53, 1, false)])];
        let shell = IrShell { id: 60, faces };
        let body = IrBody { id: 70, kind: BodyKind::Solid, regions: vec![IrRegion { solid: true, shells: vec![shell] }], density: None, last_feature: None };
        let ir = Ir { schema: "test".into(), bodies: vec![body], surfaces, curves, vertices, edges, notes: vec![], import_sheets: false };
        let built = build(&[&ir], &Limits::default()).unwrap();
        assert!(built.parts[0].refused.is_empty(), "{:?}", built.parts[0].refused);
        assert_eq!(built.body.num_faces(), 3);
        let mut body = built.body;
        assert!(ck_check::validate(&body, 1e-6).is_empty(), "{:?}", ck_check::validate(&body, 1e-6));
        // Simpson over the cross-section areas (mm)
        let n = 2000;
        let dz = 2.0 * h / n as f64;
        let mut want = 0.0;
        for k in 0..=n {
            let z = -h + dz * k as f64;
            let w = if k == 0 || k == n { 1.0 } else if k % 2 == 1 { 4.0 } else { 2.0 };
            want += w * PI * rad(z).powi(2);
        }
        want *= dz / 3.0 * SCALE.powi(3);
        let got = ck_topo::mass::mass_props(&body).volume;
        assert!((got - want).abs() < 1e-6 * want, "volume {got} vs {want}");
        ck_topo::orient::flip_face(&mut body, FaceId(0));
        assert!(!ck_check::validate(&body, 1e-6).is_empty(), "a flipped face is caught");
    }

    /// A point near where a plane cuts a torus just below its crown (the two meet at a shallow angle): the meeting
    /// point found by Newton steps lies on both.
    #[test]
    fn shallow_meeting_point_lies_on_both_supports() {
        let fr = frame_from(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0));
        let torus = Rc::new(Surface::Torus { frame: fr, major: 100.0, minor: 15.0 });
        let plane = Rc::new(Surface::Plane { frame: frame_from(Vec3::new(0.0, 0.0, 14.99), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)) });
        let sup = [torus.clone(), plane.clone()];
        for p in [Vec3::new(100.6, 3.0, 14.99), Vec3::new(99.0, -2.0, 14.98), Vec3::new(70.0, 70.0, 14.995)] {
            let c = B::project_meet(&sup, [0.0, 0.0], p).expect("meeting point");
            assert!(torus.distance(c) < 1e-6 && plane.distance(c) < 1e-6, "{c:?}");
            assert!(c.dist(p) < 2.0);
            // the rolling-ball centre 2 mm off both along their normals
            let c = B::project_meet(&sup, [2.0, 2.0], p + Vec3::new(0.0, 0.0, 2.0)).expect("offset meeting point");
            assert!((torus.distance(c) - 2.0).abs() < 1e-6 && (plane.distance(c) - 2.0).abs() < 1e-6, "{c:?}");
        }
    }


    #[test]
    fn an_edge_ending_at_the_closing_point_runs_its_own_way() {
        // forward from 0.79 to the closing point (found at 0): the short arc up to 1, not back down to 0
        assert_eq!(closing_ends((0.0, 1.0), 0.79, 1e-16, true), Some((0.79, 1.0)));
        assert_eq!(closing_ends((0.0, 1.0), 1.0, 0.2, true), Some((0.0, 0.2)));
        assert_eq!(closing_ends((0.0, 1.0), 0.0, 0.3, false), Some((1.0, 0.3)));
        assert_eq!(closing_ends((0.0, 1.0), 0.3, 1.0, false), Some((0.3, 0.0)));
        // already in order, or no end at the closing point
        assert_eq!(closing_ends((0.0, 1.0), 0.0, 0.21, true), None);
        assert_eq!(closing_ends((0.0, 1.0), 0.6, 0.2, true), None);
    }

    /// A builder over `ir` with nothing built yet.
    fn builder(ir: &Ir) -> B<'_> {
        B { ir, body: Body::new(), curves: HashMap::new(), surfaces: HashMap::new(), supports: HashMap::new(), verts: HashMap::new(), edges: HashMap::new(), approx: vec![], notes: vec![], splits: HashMap::new(), need_split: vec![], parts: vec![], vedges: None }
    }

    fn sn(id: i64, geom: IrSurface) -> IrSurfaceNode {
        IrSurfaceNode { id, geom, sense_plus: true }
    }

    fn cn(id: i64, geom: IrCurve) -> IrCurveNode {
        IrCurveNode { id, geom, sense_plus: true }
    }

    #[test]
    fn an_extended_spline_is_its_end_spans_polynomial() {
        // a cubic Bezier: the extension is the same cubic (Bernstein form evaluated past 1)
        let ctrl = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 2.0, 0.0), Vec3::new(3.0, 2.0, 1.0), Vec3::new(4.0, 0.0, 0.0)];
        let bez = BSplineCurve::bezier(ctrl.clone());
        let cubic = |t: f64| {
            let s = 1.0 - t;
            ctrl[0] * (s * s * s) + ctrl[1] * (3.0 * s * s * t) + ctrl[2] * (3.0 * s * t * t) + ctrl[3] * (t * t * t)
        };
        let e = extend_curve(&bez, 0.25).unwrap();
        assert_eq!(e.domain(), (-0.25, 1.25));
        for k in 0..=30 {
            let t = -0.25 + 1.5 * k as f64 / 30.0;
            assert!(e.eval(t).dist(cubic(t)) < 1e-12, "t {t}");
        }
        // several spans: unchanged on the domain, continued by at most the end span's length
        let knots = vec![0.0, 0.0, 0.0, 0.0, 0.25, 0.5, 1.0, 1.0, 1.0, 1.0];
        let ctrl: Vec<Vec3> = (0..6).map(|i| Vec3::new(i as f64, (i as f64 * 0.7).sin() * 3.0, (i * i) as f64 * 0.1)).collect();
        let c = BSplineCurve::new(3, knots, ctrl).unwrap();
        let e = extend_curve(&c, 0.5).unwrap();
        assert_eq!(e.domain(), (-0.25, 1.5));
        for k in 0..=40 {
            let t = k as f64 / 40.0;
            assert!(e.eval(t).dist(c.eval(t)) < 1e-12, "t {t}");
        }
        // smooth across the old ends
        for t in [0.0, 1.0] {
            let (a, b) = (e.eval(t - 1e-6), e.eval(t + 1e-6));
            let d = c.derivs(t, 1)[1];
            assert!(((b - a) * (1.0 / 2e-6) - d).len() < 1e-4 * d.len(), "tangent at {t}");
        }
    }

    #[test]
    fn a_spun_surface_has_the_files_natural_normal() {
        // the file's spun surface is (profile t, angle): its natural normal dP/dt x dP/dangle points into the axis for a
        // profile running up the axis, whatever our own parameter order
        let r = 0.02;
        let surfaces = vec![sn(1, IrSurface::Spun { profile: Some(0), base: [0.0; 3], axis: [0.0, 0.0, 1.0] })];
        let curves = vec![cn(2, IrCurve::Line { o: [r, 0.0, 0.0], d: [0.0, 0.0, 1.0] })];
        let ir = Ir { schema: "test".into(), surfaces, curves, ..Default::default() };
        let mut b = builder(&ir);
        let s = b.surface(0).unwrap();
        for p in [Vec3::new(20.0, 0.0, 5.0), Vec3::new(0.0, -20.0, -3.0), Vec3::new(-12.0, 16.0, 1.0)] {
            let (u, v, q) = s.closest_point(p);
            assert!(q.dist(p) < 1e-3, "{q:?}");
            let n = s.normal(u, v).normalize();
            let inward = Vec3::new(-p.x, -p.y, 0.0).normalize();
            assert!(n.dot(inward) > 0.999, "normal {n:?} at {p:?}");
        }
    }

    /// Two planes at right angles (z = 0 facing +z, x = 0 facing +x) and a 1 mm round between them along y.
    fn round_ir(support1: IrSurface) -> Ir {
        let r = 0.001;
        let surfaces = vec![
            sn(1, IrSurface::Plane { o: [0.0; 3], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0] }),
            sn(2, support1),
            sn(3, IrSurface::Blend { supports: [Some(0), Some(1)], support_plus: [true, true], spine: Some(0), spine_plus: true, range: [r, r], limits: [Some([r, -0.01, r]), Some([r, 0.01, r])] }),
        ];
        let curves = vec![cn(4, IrCurve::Line { o: [r, 0.0, r], d: [0.0, 1.0, 0.0] }), cn(5, IrCurve::Line { o: [0.0, 0.0, 0.0], d: [0.0, 1.0, 0.0] })];
        Ir { schema: "test".into(), surfaces, curves, ..Default::default() }
    }

    #[test]
    fn a_rolling_ball_blend_faces_away_from_its_ball() {
        let ir = round_ir(IrSurface::Plane { o: [0.0; 3], n: [1.0, 0.0, 0.0], x: [0.0, 1.0, 0.0] });
        let mut b = builder(&ir);
        let s = b.surface(2).unwrap();
        let ((u0, u1), (v0, v1)) = match &*s {
            Surface::BSpline(bs) => bs.domain(),
            _ => panic!("blend as a spline"),
        };
        for (fu, fv) in [(0.5, 0.5), (0.2, 0.1), (0.8, 0.9)] {
            let (u, v) = (u0 + (u1 - u0) * fu, v0 + (v1 - v0) * fv);
            let p = s.eval(u, v);
            let centre = Vec3::new(1.0, p.y, 1.0);
            assert!((p.dist(centre) - 1.0).abs() < 1e-5, "{p:?} on the ball");
            assert!(s.normal(u, v).normalize().dot((p - centre).normalize()) > 0.999, "normal at {p:?}");
        }
        // the same blend with its feet swapped (the spine reversed) still faces away from the ball
        let mut ir2 = ir.clone();
        if let IrSurface::Blend { spine_plus, .. } = &mut ir2.surfaces[2].geom {
            *spine_plus = false;
        }
        let mut b = builder(&ir2);
        let s = b.surface(2).unwrap();
        let Surface::BSpline(bs) = &*s else { panic!() };
        let ((u0, u1), (v0, v1)) = bs.domain();
        let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
        let p = s.eval(u, v);
        assert!(s.normal(u, v).normalize().dot((p - Vec3::new(1.0, p.y, 1.0)).normalize()) > 0.999);
    }

    #[test]
    fn a_ball_along_a_sharp_edge_touches_the_edge() {
        // support 1 is a zero-radius blend: the sharp edge along y through (1, 0, 1) mm; the 1 mm ball on the plane
        // z = 0 that touches the edge has its centre at (0, y, 1) and its arc runs from the plane's foot to the edge
        let mut ir = round_ir(IrSurface::Blend { supports: [None, None], support_plus: [true, true], spine: Some(1), spine_plus: true, range: [0.0, 0.0], limits: [None, None] });
        if let IrSurface::Blend { limits, .. } = &mut ir.surfaces[2].geom {
            *limits = [Some([0.0, -0.01, 0.001]), Some([0.0, 0.01, 0.001])];
        }
        ir.curves[0].geom = IrCurve::Line { o: [0.0, 0.0, 0.001], d: [0.0, 1.0, 0.0] };
        ir.curves[1].geom = IrCurve::Line { o: [0.001, 0.0, 0.001], d: [0.0, 1.0, 0.0] };
        let mut b = builder(&ir);
        let pipe = b.support(1).unwrap();
        assert!((pipe.distance(Vec3::new(0.0, 3.0, 1.0)) - 1.0).abs() < 1e-6);
        let s = b.surface(2).unwrap();
        let Surface::BSpline(bs) = &*s else { panic!("blend as a spline") };
        let ((u0, u1), (v0, v1)) = bs.domain();
        let um = 0.5 * (u0 + u1);
        let ends = [s.eval(um, v0), s.eval(um, v1)];
        assert!(ends.iter().any(|e| (e.x - 1.0).hypot(e.z - 1.0) < 1e-5), "an arc end on the edge: {ends:?}");
        assert!(ends.iter().any(|e| e.x.hypot(e.z) < 1e-5), "an arc end on the plane: {ends:?}");
    }

    #[test]
    fn a_tolerant_vertex_goes_where_its_curves_meet() {
        // two lines meet at (10, 0, 0) mm; the file puts their common vertex 3 um above it, within its 10 um tolerance
        let curves = vec![cn(1, IrCurve::Line { o: [0.0; 3], d: [1.0, 0.0, 0.0] }), cn(2, IrCurve::Line { o: [0.01, 0.0, 0.0], d: [0.0, 1.0, 0.0] }), cn(3, IrCurve::Line { o: [0.01, 0.0, 0.0], d: [0.0, 0.0, 1.0] })];
        let off = 3e-6;
        let vertices = vec![IrVertex { id: 10, p: [0.0; 3], tol: None }, IrVertex { id: 11, p: [0.01, 0.0, off], tol: Some(1e-5) }, IrVertex { id: 12, p: [0.01, 0.01, 0.0], tol: None }, IrVertex { id: 13, p: [0.01, 4e-6, 0.01], tol: None }];
        let edge = |id, curve, start, end| IrEdge { id, curve: Some(curve), start: Some(start), end: Some(end), fin_curve: None, tol: None };
        let ir = Ir { schema: "test".into(), curves: curves.clone(), vertices: vertices.clone(), edges: vec![edge(20, 0, 0, 1), edge(21, 1, 1, 2)], ..Default::default() };
        let mut b = builder(&ir);
        let e = b.edge(0).unwrap();
        let v = b.body.edge(e.segs[0]).v1;
        assert!(b.body.vertex(v).p.dist(Vec3::new(10.0, 0.0, 0.0)) < 1e-9 && b.body.vertex(v).tol == 0.0);
        // a third line that does not pass through that point: the vertex stays and keeps the file's tolerance
        let mut curves3 = curves;
        curves3[2].geom = IrCurve::Line { o: [0.01, 4e-6, 0.0], d: [0.0, 0.0, 1.0] };
        let ir = Ir { schema: "test".into(), curves: curves3, vertices, edges: vec![edge(20, 0, 0, 1), edge(21, 1, 1, 2), edge(22, 2, 1, 3)], ..Default::default() };
        let mut b = builder(&ir);
        let e = b.edge(0).unwrap();
        let v = b.body.edge(e.segs[0]).v1;
        assert!(b.body.vertex(v).p.dist(Vec3::new(10.0, 0.0, off * SCALE)) < 1e-9);
        assert!((b.body.vertex(v).tol - 1e-2).abs() < 1e-12);
        assert!(ck_check::validate_edges(&b.body, 1e-6, ck_check::DEFAULT_TOL_CAP).is_empty());
    }

    /// A tolerant vertex a fraction of a micrometre off the curves that meet there (well inside the file's
    /// tolerance, but more than the validator accepts of an exact vertex) goes where they meet, or keeps the tolerance.
    #[test]
    fn a_tolerant_vertex_just_off_its_curves_is_on_them_or_tolerant() {
        let curves = vec![cn(1, IrCurve::Line { o: [0.0; 3], d: [1.0, 0.0, 0.0] }), cn(2, IrCurve::Line { o: [0.01, 0.0, 0.0], d: [0.0, 1.0, 0.0] }), cn(3, IrCurve::Line { o: [0.01, 3e-7, 0.0], d: [0.0, 0.0, 1.0] })];
        let off = 2e-7; // 0.2 um
        let vertices = vec![IrVertex { id: 10, p: [0.0; 3], tol: None }, IrVertex { id: 11, p: [0.01, 0.0, off], tol: Some(1e-6) }, IrVertex { id: 12, p: [0.01, 0.01, 0.0], tol: None }, IrVertex { id: 13, p: [0.01, 3e-7, 0.01], tol: None }];
        let edge = |id, curve, start, end| IrEdge { id, curve: Some(curve), start: Some(start), end: Some(end), fin_curve: None, tol: None };
        let ir = Ir { schema: "test".into(), curves: curves.clone(), vertices: vertices.clone(), edges: vec![edge(20, 0, 0, 1), edge(21, 1, 1, 2)], ..Default::default() };
        let mut b = builder(&ir);
        let e = b.edge(0).unwrap();
        let v = b.body.edge(e.segs[0]).v1;
        assert!(b.body.vertex(v).p.dist(Vec3::new(10.0, 0.0, 0.0)) < 1e-9 && b.body.vertex(v).tol == 0.0);
        b.edge(1).unwrap();
        assert!(ck_check::validate_edges(&b.body, 1e-6, ck_check::DEFAULT_TOL_CAP).is_empty());
        // with a third line 0.3 um away the curves do not meet in one point: the vertex keeps a tolerance
        let ir = Ir { schema: "test".into(), curves, vertices, edges: vec![edge(20, 0, 0, 1), edge(21, 1, 1, 2), edge(22, 2, 1, 3)], ..Default::default() };
        let mut b = builder(&ir);
        for k in 0..3 {
            b.edge(k).unwrap();
        }
        let v = b.body.edge(b.edges[&0].segs[0]).v1;
        assert!(b.body.vertex(v).tol > 0.0 && b.body.vertex(v).tol <= 1e-3);
        assert!(ck_check::validate_edges(&b.body, 1e-6, ck_check::DEFAULT_TOL_CAP).is_empty());
    }

    /// The fit through a few chart points of an intersection (here a plane at 30 degrees through a cylinder) is refined
    /// until it lies on both supports to well within what the validator accepts at model tolerance 1e-6.
    #[test]
    fn an_intersection_fit_lies_on_both_supports() {
        let (r, a) = (0.002f64, 30f64.to_radians());
        let surfaces = vec![
            sn(1, IrSurface::Cylinder { o: [0.0; 3], axis: [0.0, 0.0, 1.0], r, x: [1.0, 0.0, 0.0] }),
            sn(2, IrSurface::Plane { o: [0.0; 3], n: [0.0, a.sin(), a.cos()], x: [1.0, 0.0, 0.0] }),
        ];
        let at = |th: f64| [r * th.cos(), r * th.sin(), -a.tan() * r * th.sin()];
        let pts: Vec<V3> = (0..5).map(|k| at(0.4 * k as f64)).collect();
        let curves = vec![cn(3, IrCurve::Intersection { pts, surfaces: [Some(0), Some(1)], bounds: [None, None], chordal: 0.0 })];
        let ir = Ir { schema: "test".into(), surfaces, curves, ..Default::default() };
        let mut b = builder(&ir);
        let c = b.curve(0).unwrap();
        let (s0, s1) = (b.surface(0).unwrap(), b.surface(1).unwrap());
        let (t0, t1) = c.curve.range();
        let worst = (0..=400).map(|k| {
            let p = c.curve.eval(t0 + (t1 - t0) * k as f64 / 400.0);
            s0.distance(p).max(s1.distance(p)) / (1.0 + p.len())
        }).fold(0.0, f64::max);
        assert!(worst < 2e-6, "the fit is {worst:.3e} (1 + |p|) mm off its supports");
    }

    /// An edge whose file tolerance covers its distance from a face (a tolerant edge: here a cap circle 0.3 um below the
    /// cap's plane) takes that deviation as its tolerance and the solid is valid; an exact edge that far off is not.
    #[test]
    fn a_tolerant_edge_may_be_off_its_face_within_the_files_tolerance() {
        let (r, h, d) = (0.01f64, 0.005f64, 3e-7f64);
        let mk = |tol: Option<f64>| {
            let surfaces = vec![
                sn(1, IrSurface::Cylinder { o: [0.0; 3], axis: [0.0, 0.0, 1.0], r, x: [1.0, 0.0, 0.0] }),
                sn(2, IrSurface::Plane { o: [0.0, 0.0, h + d], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0] }),
                sn(3, IrSurface::Plane { o: [0.0, 0.0, -h], n: [0.0, 0.0, -1.0], x: [1.0, 0.0, 0.0] }),
            ];
            let circle = |id, z: f64| cn(id, IrCurve::Circle { c: [0.0, 0.0, z], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0], r });
            let curves = vec![circle(10, h), circle(11, -h)];
            let vertices = vec![IrVertex { id: 20, p: [r, 0.0, h], tol: None }, IrVertex { id: 21, p: [r, 0.0, -h], tol: None }];
            let edges = vec![IrEdge { id: 30, curve: Some(0), start: Some(0), end: Some(0), fin_curve: None, tol }, IrEdge { id: 31, curve: Some(1), start: Some(1), end: Some(1), fin_curve: None, tol: None }];
            let lp = |id, edge, forward| IrLoop { id, fins: vec![IrFin { edge, forward }], vertex: None };
            let face = |id, surface, loops| IrFace { id, surface: Some(surface), same_sense: true, loops, back: false, colour: None, sw_id: None, sw_ids: vec![] };
            let faces = vec![face(40, 0, vec![lp(50, 0, false), lp(51, 1, true)]), face(41, 1, vec![lp(52, 0, true)]), face(42, 2, vec![lp(53, 1, false)])];
            let body = IrBody { id: 70, kind: BodyKind::Solid, regions: vec![IrRegion { solid: true, shells: vec![IrShell { id: 60, faces }] }], density: None, last_feature: None };
            let ir = Ir { schema: "test".into(), bodies: vec![body], surfaces, curves, vertices, edges, notes: vec![], import_sheets: false };
            build(&[&ir], &Limits::default()).unwrap().body
        };
        let exact = mk(None);
        assert!(ck_check::validate(&exact, 1e-6).iter().any(|i| i.code == "EDGE_OFF_SURFACE"));
        let tolerant = mk(Some(1e-6));
        assert!(ck_check::validate(&tolerant, 1e-6).is_empty(), "{:?}", ck_check::validate(&tolerant, 1e-6));
        let t = tolerant.edge_ids().map(|e| tolerant.edge(e).tol).fold(0.0, f64::max);
        assert!(t >= d * SCALE && t <= 1e-3, "edge tolerance {t}");
    }

    #[test]
    fn a_vertex_just_off_the_closing_point_ends_the_short_way() {
        // a closed cubic loop; the edge runs forward from just past the closing point (found a little inside the end of
        // the range) to a point a little further on: the short piece, not the rest of the loop
        let n = 8;
        let ctrl: Vec<Vec<f64>> = (0..=n).map(|i| {
            let a = TAU * (i % n) as f64 / n as f64;
            vec![0.01 * a.cos(), 0.01 * a.sin(), 0.0]
        }).collect();
        let mut knots = vec![0.0; 4];
        knots.extend((1..n - 2).map(|k| k as f64 / (n - 2) as f64));
        knots.extend([1.0; 4]);
        let nc = NurbsCurveIr { degree: 3, dim: 3, ctrl, weights: None, knots, periodic: false, closed: true };
        let curves = vec![cn(1, IrCurve::BCurve(nc))];
        let ir0 = Ir { schema: "test".into(), curves: curves.clone(), ..Default::default() };
        let c = builder(&ir0).curve(0).unwrap();
        let (d0, d1) = c.curve.range();
        let start = c.curve.eval(d1 - 1e-6); // 0.1 um before the closing point
        let end = c.curve.eval(d0 + 0.05 * (d1 - d0));
        let m = |p: Vec3| [p.x / SCALE, p.y / SCALE, p.z / SCALE];
        let vertices = vec![IrVertex { id: 10, p: m(start), tol: None }, IrVertex { id: 11, p: m(end), tol: None }];
        let ir = Ir { schema: "test".into(), curves, vertices, edges: vec![IrEdge { id: 20, curve: Some(0), start: Some(0), end: Some(1), fin_curve: None, tol: None }], ..Default::default() };
        let mut b = builder(&ir);
        let e = b.edge(0).unwrap();
        let len: f64 = e.segs.iter().map(|&s| b.body.edge_length(s)).sum();
        let short = c.curve.length_between(d0, d0 + 0.05 * (d1 - d0));
        assert!((len - short).abs() < 1e-3 * short, "edge length {len}, the short way {short}");
    }

    /// A sheet body (one disc, no solid) is built as an open surface only when asked, and then it is a part marked as a
    /// sheet that counts no solid.
    #[test]
    fn a_sheet_body_is_a_surface_part_when_asked() {
        let sn = |id, geom| IrSurfaceNode { id, geom, sense_plus: true };
        let surfaces = vec![sn(1, IrSurface::Plane { o: [0.0, 0.0, 0.0], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0] })];
        let curves = vec![IrCurveNode { id: 10, geom: IrCurve::Circle { c: [0.0; 3], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0], r: 0.01 }, sense_plus: true }];
        let vertices = vec![IrVertex { id: 20, p: [0.01, 0.0, 0.0], tol: None }];
        let edges = vec![IrEdge { id: 30, curve: Some(0), start: Some(0), end: Some(0), fin_curve: None, tol: None }];
        let lp = IrLoop { id: 50, fins: vec![IrFin { edge: 0, forward: true }], vertex: None };
        let face = IrFace { id: 40, surface: Some(0), same_sense: true, loops: vec![lp], back: false, colour: None, sw_id: None, sw_ids: vec![] };
        let body = IrBody { id: 70, kind: BodyKind::Sheet, regions: vec![IrRegion { solid: false, shells: vec![IrShell { id: 60, faces: vec![face] }] }], density: None, last_feature: None };
        let mut ir = Ir { schema: "test".into(), bodies: vec![body], surfaces, curves, vertices, edges, notes: vec![], import_sheets: false };
        assert_eq!(ir.sheet_faces(), 1);
        assert_eq!(ir.sheet_bodies(), 1);
        assert_eq!(ir.solid_faces(), 0);
        assert!(build(&[&ir], &Limits::default()).is_err(), "sheets are not built unless asked");
        ir.import_sheets = true;
        let built = build(&[&ir], &Limits::default()).unwrap();
        assert_eq!(built.solids, 0);
        assert_eq!(built.body.num_faces(), 1);
        assert!(built.parts[0].sheet && built.parts[0].refused.is_empty());
        let m = ck_topo::mass::mass_props(&built.body);
        assert!((m.area - PI * 100.0).abs() < 1e-6 * PI * 100.0, "area {}", m.area);
    }

}
