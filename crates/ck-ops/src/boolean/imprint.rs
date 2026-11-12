//! Imprint (ck-ops/src/boolean/imprint.rs): intersect every face of body A
//! with every face of body B, trim the intersection curves to both faces,
//! and build a shared edge arrangement:
//! - one welded vertex pool for both bodies (tolerance `tol`),
//! - original edges split at every pool point lying on them,
//! - cut segments (trimmed SSI curves) split the same way,
//! - coincident pieces de-duplicated (a cut lying on an existing edge, or
//!   coplanar boundary edges of A and B, become ONE arrangement edge).
//! Each face then lists its boundary pieces (in original loop order) and
//! the cut edges strictly inside it ([`ArrFace`]). Fragment loops are
//! walked in assemble.rs.
//!
//! Coincident face pairs (same plane / same analytic carrier) imprint each
//! other's boundary edges instead of an SSI curve.

use super::classify::{point_in_face_cached, LoopCache, Where};
use crate::bvh::{curve_hull_box, curve_range_box, curve_span_boxes, face_box, Aabb, Bvh};
use crate::intersect::ss::{self, Bounds, SsResult};
use crate::intersect::{cc, cs, fit_param, param_slack};
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName};

/// Arrangement edge.
#[derive(Debug, Clone, PartialEq)]
pub struct ArrEdge {
    pub curve: Curve,
    pub t0: f64,
    pub t1: f64,
    pub v0: usize,
    pub v1: usize,
    /// (side, original edge) for pieces of an existing edge (first source
    /// wins when A and B pieces coincide).
    pub src: Option<(usize, EdgeId)>,
}

/// A face of A (side 0) or B (side 1) with its arrangement pieces.
#[derive(Debug, Clone, PartialEq)]
pub struct ArrFace {
    pub side: usize,
    pub face: FaceId,
    pub surface: Surface,
    pub sense: bool,
    pub name: FaceName,
    /// [`Face::absorbed`] of the source face (carried through the arrangement).
    pub absorbed: Vec<String>,
    /// Per original loop: boundary pieces in traversal order (edge, rev).
    pub bound: Vec<Vec<(usize, bool)>>,
    /// Cut edges strictly inside the face (both directions are used).
    pub inner: Vec<usize>,
    /// Original coedge count (to detect untouched faces).
    pub n_coedges: usize,
}

impl ArrFace {
    pub fn touched(&self) -> bool {
        !self.inner.is_empty() || self.bound.iter().map(|l| l.len()).sum::<usize>() != self.n_coedges
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Arrangement {
    pub verts: Vec<Vec3>,
    pub edges: Vec<ArrEdge>,
    pub faces: Vec<ArrFace>,
    pub tol: f64,
    /// Number of face pairs that produced cut segments or coincidences.
    pub interacting_pairs: usize,
    /// Welding grid over `verts` (cell = tol) and `edges` by end-vertex pair:
    /// lookups instead of linear scans (private, filled by `vertex` / `edge`).
    vgrid: ck_core::fxhash::FxHashMap<(i64, i64, i64), Vec<usize>>,
    eidx: ck_core::fxhash::FxHashMap<(usize, usize), Vec<usize>>,
}

/// A trimmed intersection curve before splitting.
#[derive(Debug, Clone)]
struct CutSeg {
    curve: Curve,
    t0: f64,
    t1: f64,
    /// faces (side index into `faces` of `Arrangement` via (side, FaceId))
    faces: Vec<(usize, FaceId)>,
}

/// Two edge pieces between the same two vertices that stay within `SLIVER * tol` of each other are one edge: the
/// lens between them is a sliver the arrangement cannot resolve anyway (a cut curve running 7e-4 beside a rim where
/// two blade tips touch tangentially left a two-faced lens whose twin edges never paired up).
const SLIVER: f64 = 100.0;

impl Arrangement {
    /// Weld `p` into the pool.
    fn vertex(&mut self, p: Vec3) -> usize {
        let tol = self.tol;
        let usable = tol > 0.0 && p.x.is_finite() && p.y.is_finite() && p.z.is_finite() && p.x.abs().max(p.y.abs()).max(p.z.abs()) / tol < 1e15;
        if !usable {
            if let Some(i) = self.verts.iter().position(|q| q.dist(p) <= tol) {
                return i;
            }
            self.verts.push(p);
            return self.verts.len() - 1;
        }
        let cell = |p: Vec3| ((p.x / tol).floor() as i64, (p.y / tol).floor() as i64, (p.z / tol).floor() as i64);
        let (cx, cy, cz) = cell(p);
        let mut best: Option<usize> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(v) = self.vgrid.get(&(cx + dx, cy + dy, cz + dz)) {
                        for &i in v {
                            if best.map_or(true, |b| i < b) && self.verts[i].dist(p) <= tol {
                                best = Some(i);
                            }
                        }
                    }
                }
            }
        }
        if let Some(i) = best {
            return i;
        }
        self.verts.push(p);
        let i = self.verts.len() - 1;
        self.vgrid.entry((cx, cy, cz)).or_default().push(i);
        i
    }

    /// Add an edge piece or return the coincident existing one. Returns
    /// (edge index, same direction as given).
    fn edge(&mut self, curve: &Curve, t0: f64, t1: f64, v0: usize, v1: usize, src: Option<(usize, EdgeId)>) -> (usize, bool) {
        let tol = self.tol;
        let key = (v0.min(v1), v0.max(v1));
        let cands: Vec<usize> = self.eidx.get(&key).cloned().unwrap_or_default();
        for i in cands {
            let e = &self.edges[i];
            // samples of the new piece on the existing one, and of the existing one on the new piece: a sliver
            // between two vertices a hair apart lies on the long piece through them (a closed loop whose two ends
            // were cut at slightly different points) without being that piece
            let reach = tol * if super::robust() { SLIVER } else { 10.0 };
            let on = [0.31, 0.5, 0.73].iter().all(|&f| {
                let p = curve.eval(t0 + (t1 - t0) * f);
                near_in_range(&e.curve, (e.t0, e.t1), p, reach)
            }) && [0.31, 0.5, 0.73].iter().all(|&f| {
                let p = e.curve.eval(e.t0 + (e.t1 - e.t0) * f);
                near_in_range(curve, (t0, t1), p, reach)
            });
            if !on {
                super::bool_dbg!(
                    "edge pieces share vertices {key:?} but differ: deviations {:?}",
                    [0.31, 0.5, 0.73].iter().map(|&f| dist_in_range(&e.curve, (e.t0, e.t1), curve.eval(t0 + (t1 - t0) * f)).1).collect::<Vec<_>>()
                );
                continue;
            }
            let same = if v0 != v1 {
                e.v0 == v0
            } else {
                // closed: compare tangents
                let a = curve.deriv(t0 + (t1 - t0) * 0.25);
                let p = curve.eval(t0 + (t1 - t0) * 0.25);
                let (t, _) = dist_in_range(&e.curve, (e.t0, e.t1), p);
                a.dot(e.curve.deriv(t)) > 0.0
            };
            return (i, same);
        }
        self.edges.push(ArrEdge { curve: curve.clone(), t0, t1, v0, v1, src });
        self.eidx.entry(key).or_default().push(self.edges.len() - 1);
        (self.edges.len() - 1, true)
    }

    /// Human-readable dump (CK_BOOL_DBG diagnostics).
    pub fn dump(&self) -> String {
        use std::fmt::Write;
        let mut s = String::new();
        for (i, e) in self.edges.iter().enumerate() {
            let kind = match &e.curve {
                Curve::Line { .. } => "line",
                Curve::Circle { .. } => "circle",
                Curve::Ellipse { .. } => "ellipse",
                _ => "other",
            };
            let _ = writeln!(s, "  edge {i}: {kind} v{}->v{} t[{:.5},{:.5}] src={:?} p0={:?} p1={:?}", e.v0, e.v1, e.t0, e.t1, e.src, e.curve.eval(e.t0), e.curve.eval(e.t1));
        }
        for (i, f) in self.faces.iter().enumerate() {
            let _ = writeln!(s, "  face {i}: side {} {} {} bound={:?} inner={:?}", f.side, f.name.id(), ck_topo::body::wire_kind(&f.surface), f.bound, f.inner);
        }
        s
    }

    pub fn face_index(&self, side: usize, f: FaceId) -> Option<usize> {
        self.faces.iter().position(|af| af.side == side && af.face == f)
    }
}

/// Closest parameter (within range, periodic-aware) and distance.
pub fn dist_in_range(c: &Curve, r: (f64, f64), p: Vec3) -> (f64, f64) {
    let t = c.closest_param(p);
    if let Some(tt) = fit_param(c, t, r, 0.0) {
        return (tt, c.eval(tt).dist(p));
    }
    let (d0, d1) = (c.eval(r.0).dist(p), c.eval(r.1).dist(p));
    if d0 <= d1 {
        (r.0, d0)
    } else {
        (r.1, d1)
    }
}

/// `dist_in_range(..).1 <= tol`, skipping the closest-point search when a
/// control-polygon bound already puts `p` farther than `tol` from a spline.
pub fn near_in_range(c: &Curve, r: (f64, f64), p: Vec3, tol: f64) -> bool {
    c.may_be_within(p, tol) && dist_in_range(c, r, p).1 <= tol
}

/// Interior crossings of two cut curves: pushes the crossing point (midpoint of the closest pair) to `out` when the curves
/// meet within `2 tol` at a parameter strictly inside both ranges. Polyline pre-selection, then alternating closest-point
/// refinement; only transversal-ish crossings converge, which is what a branch point is.
fn cut_crossings(a: &CutSeg, b: &CutSeg, tol: f64, out: &mut Vec<Vec3>) {
    const N: usize = 64;
    let sample = |c: &CutSeg| -> Vec<(f64, Vec3)> {
        (0..=N)
            .map(|k| {
                let t = c.t0 + (c.t1 - c.t0) * k as f64 / N as f64;
                (t, c.curve.eval(t))
            })
            .collect()
    };
    let (pa, pb) = (sample(a), sample(b));
    let box_of = |p: &[(f64, Vec3)]| p.iter().fold(Aabb::new(p[0].1, p[0].1), |bx, q| bx.union(&Aabb::new(q.1, q.1)));
    let slack = 0.05 * (box_of(&pa).diagonal().min(box_of(&pb).diagonal())) + 4.0 * tol;
    if !box_of(&pa).padded(slack).overlaps(&box_of(&pb)) {
        return;
    }
    let mut found: Vec<Vec3> = vec![];
    for wa in pa.windows(2) {
        // two curves a hair apart pass the pre-selection in nearly all of the 64 x 64 window pairs, each refined by up to 30
        // closest-point searches: seconds for one pair of cuts. Out of time, the crossings found so far are dropped with
        // the arrangement the caller discards.
        if ck_core::budget::expired() {
            return;
        }
        for wb in pb.windows(2) {
            let (a0, a1, b0, b1) = (wa[0].1, wa[1].1, wb[0].1, wb[1].1);
            let (da, db) = (a1 - a0, b1 - b0);
            // closest points of two segments (clamped), good enough for a pre-selection
            let w = a0 - b0;
            let (aa, bb, cc, dd, ee) = (da.dot(da), da.dot(db), db.dot(db), da.dot(w), db.dot(w));
            let den = aa * cc - bb * bb;
            let (mut s, mut u) = if den > 1e-18 { ((bb * ee - cc * dd) / den, (aa * ee - bb * dd) / den) } else { (0.5, 0.5) };
            s = s.clamp(0.0, 1.0);
            u = u.clamp(0.0, 1.0);
            let (qa, qb) = (a0 + da * s, b0 + db * u);
            if qa.dist(qb) > slack.max(0.05 * (da.len() + db.len())) {
                continue;
            }
            // segments running side by side are a stretch where the cuts coincide or touch, not a crossing (which
            // needs the curves transversal); refining every pair of such a stretch costs a closest-point search per
            // step and finds no branch point
            if da.cross(db).len() < 0.02 * da.len() * db.len() {
                continue;
            }
            // refine on the true curves
            let (mut ta, mut tb) = (wa[0].0 + (wa[1].0 - wa[0].0) * s, wb[0].0 + (wb[1].0 - wb[0].0) * u);
            let mut ok = false;
            for _ in 0..30 {
                let pa_ = a.curve.eval(ta);
                let (nb, _) = dist_in_range(&b.curve, (b.t0, b.t1), pa_);
                let pb_ = b.curve.eval(nb);
                let (na, _) = dist_in_range(&a.curve, (a.t0, a.t1), pb_);
                let moved = (na - ta).abs() + (nb - tb).abs();
                ta = na;
                tb = nb;
                if moved < 1e-13 {
                    break;
                }
            }
            let (ca, cb) = (a.curve.eval(ta), b.curve.eval(tb));
            if ca.dist(cb) <= 2.0 * tol {
                ok = true;
            }
            let inside = |c: &CutSeg, t: f64| {
                let s = param_slack(&c.curve, t, tol);
                t > c.t0 + s && t < c.t1 - s
            };
            if ok && (inside(a, ta) || inside(b, tb)) {
                let m = (ca + cb) * 0.5;
                if !found.iter().any(|f| f.dist(m) <= 10.0 * tol) {
                    found.push(m);
                }
            }
        }
    }
    out.extend(found);
}

/// Edge tolerance the original stores on every edge (face boxes grow by it:
/// a cap disk's box is 2e-5 + 2·1e-6 thick in the oracle's BVH dump).
const EDGE_TOL: f64 = 1e-5;

/// Box padding (in units of the welding tolerance) of the lossless hull-box rejections of [`trim`] and the
/// edge splitting: well past the 2 tol (plus parameter slack) within which a crossing or a pool point counts.
const REJECT_PAD: f64 = 16.0;

/// Parameter range of a CLOSED intersection conic that no boundary cuts:
/// [−0.38·2π − 1e-9, +2π] of its frame, i.e. it starts at 0.62·2π (oracle:
/// box C0 ∩ through hole r2 at (5,5) starts at (3.542062743788083, 0,
/// −3.6309057896005594) = angle −0.38·τ − 1e-9 of the cylinder frame bit
/// for bit, mid at 0.7539822358615504; same for r3 / r11.25 / r1.7 circles
/// on top / front sketches). Our periodic faces keep a seam, so
/// the edge is split there too; ck-topo `wire_chains` rotates the merged
/// ring to start at this vertex.
pub fn closed_start(period: f64) -> (f64, f64) {
    let t0 = -0.38 * std::f64::consts::TAU - 1e-9;
    (t0, t0 + period)
}

/// Per-face-pair override of the surface/surface intersection: given a face
/// of `a` and a face of `b`, `Some(curves)` replaces the SSI result (empty =
/// the pair does not interact, e.g. a tangent blend patch; otherwise the
/// known intersection curves, trimmed to both faces as usual); `None` runs
/// the normal SSI. Used by free-form blends, whose tool faces touch the body
/// tangentially (SSI cannot find that) or along an exactly known curve.
pub type PairHook<'a> = &'a dyn Fn(FaceId, FaceId) -> Option<Vec<Curve>>;

thread_local! {
    static SKIP_PAIRS: std::cell::RefCell<Vec<(FaceId, FaceId)>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// Guard returned by [`skip_pairs`]; clears the list when dropped.
pub struct SkipGuard;

impl Drop for SkipGuard {
    fn drop(&mut self) {
        SKIP_PAIRS.with(|s| s.borrow_mut().clear());
    }
}

/// Declare face pairs `(face of a, face of b)` that never intersect (e.g.
/// a shell's cavity face and the parallel face it was offset from) for the
/// booleans run while the guard lives: their surface/surface intersection
/// is skipped (it only burns time marching between near-parallel sheets).
pub fn skip_pairs(pairs: Vec<(FaceId, FaceId)>) -> SkipGuard {
    SKIP_PAIRS.with(|s| *s.borrow_mut() = pairs);
    SkipGuard
}

/// True when a plane face and a cone face surely share no point (so their intersection curves would all be trimmed
/// away): every point of the plane face inside the cone face's axial slab stays radially farther than a margin from
/// the cone face's radius band. In the cone's frame (z = v along the axis, rho = distance from the axis):
/// - the cone face's v range is that of its boundary edges' hull boxes (v has no interior extremum on a cone face;
///   one reaching the apex has its seam ending there), its rho is |r + v tan a| over that range;
/// - a point of the plane face at height z lies on a chord of constant z whose ends are on the face's boundary, and
///   rho along that chord is convex: over the face's part inside the slab, rho is bounded by its values at boundary
///   points inside the slab (lines clipped exactly, other edges by their span hull boxes) and, where a chord's foot
///   point (closest to the axis) may lie inside the face, by the foot segment's rho. The foot segment is left out
///   only when no boundary box comes near it and its midpoint classifies Out.
/// A face with no boundary inside the slab does not reach it. The margin covers the trim's tolerance and the chord
/// sag of the uv polygons the trim classifies with (1e-4 for circles): a kept curve piece's midpoint lies within it
/// of both faces. Plane x cone pairs whose section is a hyperbola or parabola are marched otherwise, and on the
/// fan shroud's cavity cut nearly all of them (blade walls against the cavity's thin bands) keep nothing.
fn plane_cone_apart(pb: &Body, pf: FaceId, cb: &Body, cf: FaceId, tol: f64, cache: &mut LoopCache) -> bool {
    plane_cone_apart_opt(pb, pf, cb, cf, tol, cache) == Some(true)
}

fn plane_cone_apart_opt(pb: &Body, pf: FaceId, cb: &Body, cf: FaceId, tol: f64, cache: &mut LoopCache) -> Option<bool> {
    let Surface::Plane { frame: pfr } = &pb.face(pf).surface else { return None };
    let Surface::Cone { frame, r, semi_angle } = &cb.face(cf).surface else { return None };
    let k = semi_angle.tan();
    let loc_box = |bb: &Aabb| -> Aabb {
        let mut out = Aabb::EMPTY;
        for i in 0..8 {
            let p = Vec3::new(if i & 1 == 0 { bb.min.x } else { bb.max.x }, if i & 2 == 0 { bb.min.y } else { bb.max.y }, if i & 4 == 0 { bb.min.z } else { bb.max.z });
            out.add_point(frame.to_local(p));
        }
        out
    };
    // cone face v range
    let (mut v0, mut v1) = (f64::INFINITY, f64::NEG_INFINITY);
    for e in cb.face_edges(cf) {
        let ed = cb.edge(e);
        let lb = loc_box(&curve_hull_box(&ed.curve, ed.t0, ed.t1)?);
        v0 = v0.min(lb.min.z);
        v1 = v1.max(lb.max.z);
    }
    if !(v0.is_finite() && v1.is_finite()) {
        return None;
    }
    let (ra, rb) = (r + k * v0, r + k * v1);
    let (rc0, rc1) = if ra * rb <= 0.0 { (0.0, ra.abs().max(rb.abs())) } else { (ra.abs().min(rb.abs()), ra.abs().max(rb.abs())) };
    let m = REJECT_PAD * tol + 1e-9 + 1e-3 * (1.0 + rc1);
    let (z0, z1) = (v0 - m, v1 + m);
    // plane face rho bounds over the slab
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut boxes: Vec<Aabb> = vec![];
    let rho_rect = |b: &Aabb| -> (f64, f64) {
        let dx = if b.min.x > 0.0 { b.min.x } else if b.max.x < 0.0 { -b.max.x } else { 0.0 };
        let dy = if b.min.y > 0.0 { b.min.y } else if b.max.y < 0.0 { -b.max.y } else { 0.0 };
        let mx = b.min.x.abs().max(b.max.x.abs());
        let my = b.min.y.abs().max(b.max.y.abs());
        ((dx * dx + dy * dy).sqrt(), (mx * mx + my * my).sqrt())
    };
    for e in pb.face_edges(pf) {
        let ed = pb.edge(e);
        if let Curve::Line { .. } = ed.curve {
            let (a, b) = (frame.to_local(ed.curve.eval(ed.t0)), frame.to_local(ed.curve.eval(ed.t1)));
            boxes.push(Aabb::from_points(&[a, b]));
            // clip to the slab
            let (mut s0, mut s1) = (0.0f64, 1.0f64);
            let dz = b.z - a.z;
            if dz.abs() < 1e-300 {
                if a.z < z0 || a.z > z1 {
                    continue;
                }
            } else {
                let (ta, tb) = ((z0 - a.z) / dz, (z1 - a.z) / dz);
                s0 = s0.max(ta.min(tb));
                s1 = s1.min(ta.max(tb));
                if s0 > s1 {
                    continue;
                }
            }
            let (p, q) = (a + (b - a) * s0, a + (b - a) * s1);
            let (p2, q2) = (Vec3::new(p.x, p.y, 0.0), Vec3::new(q.x, q.y, 0.0));
            let d = q2 - p2;
            let l2 = d.dot(d);
            let t = if l2 > 1e-300 { (-p2.dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
            lo = lo.min((p2 + d * t).len());
            hi = hi.max(p2.len()).max(q2.len());
        } else {
            for bb in curve_span_boxes(&ed.curve, ed.t0, ed.t1)? {
                let lb = loc_box(&bb);
                boxes.push(lb);
                if lb.max.z < z0 || lb.min.z > z1 {
                    continue;
                }
                let (a, b) = rho_rect(&lb);
                lo = lo.min(a);
                hi = hi.max(b);
            }
        }
    }
    if !hi.is_finite() {
        return Some(true); // no boundary in the slab: the face does not reach it
    }
    // foot segment: on the line of constant z, the point closest to the axis
    let nl = frame.to_local(pfr.o + pfr.z) - frame.to_local(pfr.o);
    let dl = nl.dot(frame.to_local(pfr.o));
    let h2 = nl.x * nl.x + nl.y * nl.y;
    if h2 < 1e-24 {
        return None; // plane across the axis (circle case)
    }
    let foot = |z: f64| {
        let s = (dl - nl.z * z) / h2;
        Vec3::new(nl.x * s, nl.y * s, z)
    };
    let (fa, fb) = (foot(z0), foot(z1));
    let fbox = Aabb::from_points(&[fa, fb]).padded(m);
    let mut use_foot = boxes.iter().any(|b| b.overlaps(&fbox));
    if !use_foot {
        let mid = frame.o + frame.vec_to_world((fa + fb) * 0.5);
        use_foot = point_in_face_cached(pb, pf, mid, m, cache) != Where::Out;
    }
    if use_foot {
        let (ra, rb) = (Vec3::new(fa.x, fa.y, 0.0).len(), Vec3::new(fb.x, fb.y, 0.0).len());
        let crosses = (dl - nl.z * z0) * (dl - nl.z * z1) <= 0.0;
        lo = lo.min(if crosses { 0.0 } else { ra.min(rb) });
    }
    Some(lo - rc1 > m || rc0 - hi > m)
}

/// Build the arrangement of `a` (side 0) and `b` (side 1).
/// Region the SSI marches in. The union of the two face boxes, except when one
/// face is a long multi-span B-spline (a screw sweep): the other face then
/// meets it only inside the overlap of the boxes, and marching the whole
/// sweep (every turn crossing a small cap's plane) costs minutes.
fn ssi_bounds(sf: &Surface, sg: &Surface, bi: &Aabb, bj: &Aabb, bx: &Aabb, pad: f64) -> Bounds {
    let long = |s: &Surface| s.is_long_sweep();
    // a huge companion face (the rib's wall-to-wall tool) makes the union box
    // so large that the marcher misses a branch of a small spline face
    let lopsided = bi.diagonal().min(bj.diagonal()) * 8.0 < bx.diagonal();
    if long(sf) || long(sg) || lopsided {
        let inter = Aabb::new(bi.min.max(bj.min), bi.max.min(bj.max));
        if !inter.is_empty() {
            let tight = inter.padded(0.01 * bi.diagonal().max(bj.diagonal()) + pad);
            if tight.diagonal() < 0.5 * bx.diagonal() {
                return Bounds { lo: tight.min, hi: tight.max };
            }
        }
    }
    Bounds { lo: bx.min, hi: bx.max }
}

pub fn imprint(a: &Body, b: &Body, tol: f64) -> Arrangement {
    imprint_hooked(a, b, tol, None)
}

/// [`imprint`] with an optional per-pair SSI override.
pub fn imprint_hooked(a: &Body, b: &Body, tol: f64, hook: Option<PairHook>) -> Arrangement {
    let bodies = [a, b];
    let mut arr = Arrangement { tol, ..Default::default() };
    // ---- 1. face pairs and cut segments
    let fa: Vec<FaceId> = a.face_ids().collect();
    let fb: Vec<FaceId> = b.face_ids().collect();
    // oracle (f2331): face boxes padded by max(tol, 1e-6), pairs in the
    // original BVH traversal order (decides cut-edge / hole-loop order)
    // (face boxes = edge boxes grown by the edge tolerance 1e-5, f_cee)
    let pad = tol.max(1e-6);
    let boxes_a: Vec<Aabb> = fa.iter().map(|&f| face_box(a, f).padded(EDGE_TOL + pad)).collect();
    let boxes_b: Vec<Aabb> = fb.iter().map(|&f| face_box(b, f).padded(EDGE_TOL + pad)).collect();
    let (ta, tb) = (Bvh::build(&boxes_a), Bvh::build(&boxes_b));
    let mut cuts: Vec<CutSeg> = vec![];
    let mut loop_cache = LoopCache::new();
    let mut extra: Vec<Vec3> = vec![];
    // plane x cone pairs that surely share no point (see `plane_cone_apart`)
    let apart = |f: FaceId, g: FaceId, cache: &mut LoopCache| match (&a.face(f).surface, &b.face(g).surface) {
        (Surface::Plane { .. }, Surface::Cone { .. }) => plane_cone_apart(a, f, b, g, tol, cache),
        (Surface::Cone { .. }, Surface::Plane { .. }) => plane_cone_apart(b, g, a, f, tol, cache),
        _ => false,
    };
    for (i, j) in ta.overlap_pairs_ordered(&tb, pad) {
        // out of time: the caller (boolean_at) sees the expiry and discards this partial arrangement
        if ck_core::budget::poll() {
            break;
        }
        // the SSI / trim bounds keep the old, wider slack
        let (bi, bj) = (boxes_a[i], boxes_b[j]);
        let (f, g) = (fa[i], fb[j]);
        let (sf, sg) = (&a.face(f).surface, &b.face(g).surface);
        let bx = bi.union(&bj);
        let bounds = ssi_bounds(sf, sg, &bi, &bj, &bx, pad);
        // the march's chord length (the same expression as `ss::intersect_uncached`):
        // an SSI curve may stop this far short of a face's boundary edge
        let step = (bounds.hi - bounds.lo).len() / 64.0;
        let max_gap = 2.0 * step;
        let before = cuts.len();
        let skipped = SKIP_PAIRS.with(|sp| {
            let sp = sp.borrow();
            !sp.is_empty() && sp.contains(&(f, g))
        });
        let ssi = match hook.and_then(|h| h(f, g)) {
            _ if skipped => SsResult::Disjoint,
            Some(cs) if cs.is_empty() => SsResult::Disjoint,
            Some(cs) => SsResult::Curves(cs.into_iter().map(ss::SsCurve::Exact).collect()),
            None if apart(f, g, &mut loop_cache) => SsResult::Disjoint,
            None => ss::intersect(sf, sg, Some(bounds), tol),
        };
        // a surface pair that needed marching can take long on its own, and `poll` reads the clock only every few
        // calls: read it after each real intersection
        if ck_core::budget::expired() {
            break;
        }
        match ssi {
            SsResult::Disjoint => {}
            SsResult::Coincident => {
                // imprint each face's boundary onto the other
                for (src, sf_id, dst, df_id) in [(1usize, g, 0usize, f), (0usize, f, 1usize, g)] {
                    let sb = bodies[src];
                    for e in sb.face_edges(sf_id) {
                        let ed = sb.edge(e);
                        for (t0, t1) in trim(&ed.curve, (ed.t0, ed.t1), &[(bodies[dst], df_id)], &bx, tol, true, &mut loop_cache) {
                            cuts.push(CutSeg { curve: ed.curve.clone(), t0, t1, faces: vec![(dst, df_id)] });
                        }
                    }
                }
                arr.interacting_pairs += 1;
            }
            SsResult::Curves(cs) => {
                for c in cs {
                    // each curve is trimmed against both faces (hundreds of ms for a long fitted one): out of time, stop here
                    if ck_core::budget::expired() {
                        break;
                    }
                    let mut c = c;
                    extend_to_face_boundary(&mut c, &[(a, f), (b, g)], tol, max_gap);
                    let curve = c.curve().clone();
                    let r = curve.range();
                    let kept = trim(&curve, r, &[(a, f), (b, g)], &bx, tol, false, &mut loop_cache);
                    super::bool_dbg!("ssi A{} x B{}: curve range {:?} p0 {:?} pm {:?} kept {:?}", f.idx(), g.idx(), r, curve.eval(r.0), curve.eval(0.5 * (r.0 + r.1)), kept);
                    // whole closed curve: kept pieces contiguous over one
                    // period (a relocated seam crossing splits the range)
                    let mut kept = kept;
                    let contiguous = !kept.is_empty() && kept.windows(2).all(|w| (w[0].1 - w[1].0).abs() <= 1e-12);
                    let whole = contiguous
                        && curve.period().map_or(false, |p| (kept[kept.len() - 1].1 - kept[0].0 - p).abs() <= 1e-9);
                    if whole && kept.len() > 1 && matches!(curve, Curve::Circle { .. } | Curve::Ellipse { .. }) {
                        // an interior break on a boundary edge (a seam that
                        // was not relocated) stays a split point
                        let start_pt = curve.eval(closed_start(kept[kept.len() - 1].1 - kept[0].0).0);
                        // a cut that is an existing edge (a tool's rim circle lying in the
                        // interior of the other face) keeps that edge's own start, so it
                        // never becomes the relocated-seam start: the seam crossing must
                        // stay a split point or the circle is not tied to the seam
                        let (w0, w1) = (kept[0].0, kept[kept.len() - 1].1);
                        let is_existing = [(a, f), (b, g)].iter().any(|&(bd, fc)| {
                            bd.face_edges(fc).into_iter().any(|e| {
                                let ed = bd.edge(e);
                                [0.13, 0.41, 0.77].iter().all(|&k| near_in_range(&ed.curve, (ed.t0, ed.t1), curve.eval(w0 + (w1 - w0) * k), tol))
                            })
                        });
                        for w in kept.windows(2) {
                            let p = curve.eval(w[0].1);
                            let on_edge = [(a, f), (b, g)].iter().any(|&(bd, fc)| {
                                bd.face_edges(fc).into_iter().any(|e| {
                                    let ed = bd.edge(e);
                                    near_in_range(&ed.curve, (ed.t0, ed.t1), p, tol)
                                })
                            });
                            if on_edge && (is_existing || p.dist(start_pt) > tol) {
                                extra.push(p);
                            }
                        }
                        kept = vec![(kept[0].0, kept[kept.len() - 1].1)];
                    }
                    let whole = whole && kept.len() == 1;
                    for (t0, t1) in kept {
                        // (a cut that is an existing edge keeps that edge's start)
                        let on_existing = whole
                            && [(a, f), (b, g)].iter().any(|&(bd, fc)| {
                                bd.face_edges(fc).into_iter().any(|e| {
                                    let ed = bd.edge(e);
                                    [0.13, 0.41, 0.77].iter().all(|&k| {
                                        let q = curve.eval(t0 + (t1 - t0) * k);
                                        near_in_range(&ed.curve, (ed.t0, ed.t1), q, tol)
                                    })
                                })
                            });
                        let (t0, t1) = if whole && !on_existing && matches!(curve, Curve::Circle { .. } | Curve::Ellipse { .. }) {
                            // keep the old start as a split point where a
                            // boundary edge (our seam) crosses it
                            let p = curve.eval(t0);
                            let on_edge = [(a, f), (b, g)].iter().any(|&(bd, fc)| {
                                bd.face_edges(fc).into_iter().any(|e| {
                                    let ed = bd.edge(e);
                                    near_in_range(&ed.curve, (ed.t0, ed.t1), p, tol)
                                })
                            });
                            if on_edge {
                                extra.push(p);
                            }
                            closed_start(t1 - t0)
                        } else {
                            (t0, t1)
                        };
                        cuts.push(CutSeg { curve: curve.clone(), t0, t1, faces: vec![(0, f), (1, g)] });
                    }
                }
            }
        }
        if cuts.len() > before {
            arr.interacting_pairs += 1;
        }
    }
    // ---- 1b. crossings between two cut curves of the same face pair: two branches of one intersection
    // (equal-size tubes meeting symmetrically, e.g. crossed blades) cross each other in their interiors
    // at a singular point; both must be split there or the face walk sees one edge in four directions.
    // (each crossing search can take long on near-coincident cuts: seconds for 12 copies of a revolved body a hair apart,
    // all of it here in the robust retry; out of time, the caller sees the expiry and discards the arrangement)
    'crossings: for i in 0..if super::robust() { cuts.len() } else { 0 } {
        for j in i + 1..cuts.len() {
            if cuts[i].faces == cuts[j].faces {
                if ck_core::budget::expired() {
                    break 'crossings;
                }
                cut_crossings(&cuts[i], &cuts[j], tol, &mut extra);
            }
        }
    }
    // ---- 2. vertex pool: original vertices, then cut ends
    let mut pts: Vec<Vec3> = vec![];
    for body in bodies {
        for v in body.vertex_ids() {
            pts.push(body.vertex(v).p);
        }
    }
    for c in &cuts {
        pts.push(c.curve.eval(c.t0));
        pts.push(c.curve.eval(c.t1));
    }
    pts.extend(extra.iter().copied());
    for &p in &pts {
        arr.vertex(p);
    }
    let pool = arr.verts.clone();
    // ---- 3. split original edges and cut segments at pool points on them
    // spatial index over the pool: a point can only split a curve piece
    // when it lies within `tol` of the piece, so lines / conics query the
    // pool by the piece's box instead of testing every point
    let pool_boxes: Vec<Aabb> = pool.iter().map(|&p| Aabb::new(p, p)).collect();
    let pool_bvh = Bvh::build(&pool_boxes);
    let split_params = |c: &Curve, t0: f64, t1: f64| -> Vec<f64> {
        let mut ts = vec![t0, t1];
        let cand: Option<Vec<usize>> = match c {
            Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. } => {
                let bb = curve_range_box(c, t0, t1);
                if bb.is_empty() || !(bb.diagonal().is_finite()) {
                    None
                } else {
                    Some(pool_bvh.query_box(&bb.padded(4.0 * tol + 1e-9)))
                }
            }
            // splines: the pool points within reach of the control-point hull of a span in range (a point
            // farther than `tol` from the curve can never split it)
            _ => curve_span_boxes(c, t0, t1).map(|bs| {
                let mut v: Vec<usize> = bs.iter().flat_map(|bb| pool_bvh.query_box(&bb.padded(REJECT_PAD * tol + 1e-9))).collect();
                v.sort_unstable();
                v.dedup();
                v
            }),
        };
        let n_cand = cand.as_ref().map_or(pool.len(), |c| c.len());
        for k in 0..n_cand {
            let p = pool[cand.as_ref().map_or(k, |c| c[k])];
            if !c.may_be_within(p, tol) {
                continue; // farther than `tol` from the whole curve: skip the closest-point search
            }
            let (t, d) = dist_in_range(c, (t0, t1), p);
            if d <= tol {
                let s = param_slack(c, t, tol);
                if t > t0 + s && t < t1 - s {
                    ts.push(t);
                }
            }
        }
        ts.retain(|t| t.is_finite()); // robustness: huge coordinates can yield NaN params
        ts.sort_by(f64::total_cmp);
        ts.dedup_by(|x, y| (*x - *y).abs() <= 1e-12);
        ts
    };
    // original edge pieces: (side, edge) -> [(arr edge, same_dir)]
    let mut pieces: Vec<Vec<Vec<(usize, bool)>>> = vec![vec![], vec![]];
    for (s, body) in bodies.iter().enumerate() {
        pieces[s] = vec![vec![]; body.edges.len()];
        for e in body.edge_ids() {
            let ed = body.edge(e);
            let ts = split_params(&ed.curve, ed.t0, ed.t1);
            let mut out = vec![];
            for w in ts.windows(2) {
                let v0 = if w[0] == ed.t0 { arr.vertex(body.vertex(ed.v0).p) } else { arr.vertex(ed.curve.eval(w[0])) };
                let v1 = if w[1] == ed.t1 { arr.vertex(body.vertex(ed.v1).p) } else { arr.vertex(ed.curve.eval(w[1])) };
                out.push(arr.edge(&ed.curve, w[0], w[1], v0, v1, Some((s, e))));
            }
            pieces[s][e.idx()] = out;
        }
    }
    // cut pieces
    let mut cut_edges: Vec<(usize, Vec<(usize, FaceId)>)> = vec![];
    for c in &cuts {
        let ts = split_params(&c.curve, c.t0, c.t1);
        for w in ts.windows(2) {
            let v0 = arr.vertex(c.curve.eval(w[0]));
            let v1 = arr.vertex(c.curve.eval(w[1]));
            if v0 == v1 && c.curve.eval(w[0]).dist(c.curve.eval(0.5 * (w[0] + w[1]))) <= tol {
                continue; // degenerate
            }
            let (ei, _) = arr.edge(&c.curve, w[0], w[1], v0, v1, None);
            cut_edges.push((ei, c.faces.clone()));
        }
    }
    // ---- 4. faces
    for (s, body) in bodies.iter().enumerate() {
        for f in body.face_ids() {
            let face = body.face(f);
            let mut bound = vec![];
            let mut n_co = 0;
            for &l in &face.loops {
                let mut lp = vec![];
                for &c in &body.lp(l).coedges {
                    n_co += 1;
                    let ce = body.coedge(c);
                    let ps = &pieces[s][ce.edge.idx()];
                    if ce.rev {
                        for &(ei, same) in ps.iter().rev() {
                            lp.push((ei, same)); // rev XOR !same == same
                        }
                    } else {
                        for &(ei, same) in ps {
                            lp.push((ei, !same));
                        }
                    }
                }
                bound.push(lp);
            }
            let mut inner = vec![];
            for (ei, fs) in &cut_edges {
                if !fs.contains(&(s, f)) {
                    continue;
                }
                if bound.iter().flatten().any(|&(x, _)| x == *ei) || inner.contains(ei) {
                    continue;
                }
                inner.push(*ei);
            }
            arr.faces.push(ArrFace {
                side: s,
                face: f,
                surface: face.surface.clone(),
                sense: face.sense,
                name: face.name.clone(),
                absorbed: face.absorbed.clone(),
                bound,
                inner,
                n_coedges: n_co,
            });
        }
    }
    arr
}

/// Clip a free-form SSI curve to the trimmed face boundary: the face's edge
/// meets the other face's surface at the point the curve should reach. If the
/// march stopped short of it, the point is appended; if it overshot (the blade
/// rail is nearly tangent to the cylinder, so the SSI marches a fraction of a
/// step past the rail), the overshooting samples are dropped and the endpoint
/// is replaced. Either way the curve is refitted so it really ends on the
/// shared rail. Unlike clustering endpoints in the vertex pool, this changes
/// the curve geometry, so the fragment walk sees the edge on the face boundary.
fn extend_to_face_boundary(c: &mut ss::SsCurve, faces: &[(&Body, FaceId)], tol: f64, max_gap: f64) {
    let (pts, curve, closed) = match c {
        ss::SsCurve::Exact(_) => return,
        ss::SsCurve::Fitted { pts, curve, closed } => (pts, curve, closed),
    };
    if *closed {
        super::bool_dbg!("extend_to_face_boundary: closed curve skipped (p0 {:?})", curve.eval(curve.range().0));
        return;
    }
    // Clip the overshoot only for the near-tangent free-form SSI of a periodic cylinder with a spline
    // (the blade hub fuse): an ordinary boolean curve that runs past a boundary edge is handled by
    // `trim` (or kept as-is), and clamping it here changes the result of unrelated booleans.
    let trim_ok = faces.len() == 2 && {
        let s = |i: usize| &faces[i].0.face(faces[i].1).surface;
        let cyl = |x: &Surface| matches!(x, Surface::Cylinder { .. }) && x.periods().0.is_some();
        let spl = |x: &Surface| matches!(x, Surface::BSpline(_));
        (cyl(s(0)) && spl(s(1))) || (cyl(s(1)) && spl(s(0)))
    };
    let r = curve.range();
    let p0 = curve.eval(r.0);
    let p1 = curve.eval(r.1);
    super::bool_dbg!("extend_to_face_boundary: open curve r {:?} p0 {:?} p1 {:?} max_gap {:.4} faces {}", r, p0, p1, max_gap, faces.len());
    let mut start: Option<(f64, Vec3)> = None;
    let mut end: Option<(f64, Vec3)> = None;
    // an endpoint already lying on a boundary edge (an exact hit within tol) is not dangling: leave it
    let (mut p0_on, mut p1_on) = (false, false);
    for &(body, f) in faces {
        for &(other_body, other_f) in faces {
            if std::ptr::eq(body, other_body) && f == other_f {
                continue;
            }
            let sg = &other_body.face(other_f).surface;
            for e in body.face_edges(f) {
                let ed = body.edge(e);
                let eb = crate::bvh::curve_range_box(&ed.curve, ed.t0, ed.t1).padded(max_gap + tol);
                if !eb.contains(p0) && !eb.contains(p1) {
                    continue;
                }
                super::bool_dbg!("extend: edge {:?} kind {:?} contains p0={} p1={}", e, ed.curve.kind(), eb.contains(p0), eb.contains(p1));
                let res = cs::intersect(&ed.curve, (ed.t0, ed.t1), sg, tol);
                for h in res.hits {
                    let d0 = h.p.dist(p0);
                    let d1 = h.p.dist(p1);
                    super::bool_dbg!("extend: hit d0={:.4} d1={:.4} max_gap={:.4}", d0, d1, max_gap);
                    p0_on |= d0 <= tol;
                    p1_on |= d1 <= tol;
                    if d0 <= max_gap && d0 > tol && start.map_or(true, |(bd, _)| d0 < bd) {
                        start = Some((d0, h.p));
                    }
                    if d1 <= max_gap && d1 > tol && end.map_or(true, |(bd, _)| d1 < bd) {
                        end = Some((d1, h.p));
                    }
                }
            }
        }
    }
    super::bool_dbg!("extend: start {:?} end {:?} max_gap {:.4}", start, end, max_gap);
    // Nearest sample of the marched polyline to the boundary hit: the curve may have marched a step
    // *past* the boundary (the blade rail is nearly tangent to the cylinder, so the SSI overshoots by a
    // fraction of a step). In that case the endpoint sample is replaced (and any samples beyond it are
    // dropped), rather than rejected on the tangent sign, so the curve really ends on the rail.
    let nearest = |pts: &[Vec3], p: Vec3| {
        pts.iter().enumerate().min_by(|a, b| a.1.dist(p).partial_cmp(&b.1.dist(p)).unwrap()).map_or(0, |(i, _)| i)
    };
    let mut ext = false;
    if let Some((_, p)) = start {
        let t0 = curve.tangent(r.0);
        if t0.len() > 1e-12 && (p0 - p).dot(t0) > 0.0 {
            pts.insert(0, p);
            ext = true;
        } else if trim_ok && !p0_on {
            let i = nearest(pts, p);
            if i > 0 {
                pts.drain(..i);
            }
            if let Some(first) = pts.first_mut() {
                *first = p;
                ext = true;
            }
        }
    }
    if let Some((_, p)) = end {
        let t1 = curve.tangent(r.1);
        if t1.len() > 1e-12 && (p - p1).dot(t1) > 0.0 {
            pts.push(p);
            ext = true;
        } else if trim_ok && !p1_on {
            let i = nearest(pts, p);
            if i + 1 < pts.len() {
                pts.truncate(i + 1);
            }
            if let Some(last) = pts.last_mut() {
                *last = p;
                ext = true;
            }
        }
    }
    super::bool_dbg!("extend: applied={} pts={}", ext, pts.len());
    if ext && pts.len() >= 2 {
        let deg = 3.min(pts.len() - 1);
        if let Ok(bs) = ck_geom::fit::interpolate(pts, deg, ck_geom::fit::Param::ChordLength) {
            *curve = Curve::BSpline(bs);
        }
    }
}

/// Trim curve `c` over `r` to the parts inside all `faces`. Breakpoints:
/// crossings with every boundary edge of every face (+ overlap ends).
/// Kept: intervals whose midpoint is not Out of any face and (unless
/// `allow_all_on`) not On the boundary of all of them.
/// Relative distance within which a tangent-contact breakpoint yields to an exact crossing (see [`trim`]).
const SOFT_SNAP: f64 = 2e-5; // x the size of the pair's box

/// The distance within which a tangent-contact breakpoint yields to an exact crossing: [`SOFT_SNAP`] of the pair's box,
/// or (a tangent hit is off by about sqrt(2 e R) for a curve fitted to e with curvature radius R near the contact: a
/// millimetre-sized round fitted to 1e-6 puts it 1e-3 away) 3 sqrt(tol), held under a twentieth of the box.
fn soft_window(tol: f64, bx: &Aabb) -> f64 {
    let diag = (bx.max - bx.min).len();
    (SOFT_SNAP * diag).max((3.0 * tol.sqrt()).min(0.05 * diag))
}

fn trim(c: &Curve, r: (f64, f64), faces: &[(&Body, FaceId)], bx: &Aabb, tol: f64, allow_all_on: bool, cache: &mut LoopCache) -> Vec<(f64, f64)> {
    let r = crate::intersect::finite_range(c, r, bx.min, bx.max);
    if !(r.0.is_finite() && r.1.is_finite()) || r.1 - r.0 <= 0.0 {
        return vec![];
    }
    let mut ts = vec![r.0, r.1];
    let mut soft: Vec<f64> = vec![];
    // tangent touches of a fitted curve: hits and the ends of short overlaps (see below), which yield to an exact
    // crossing within the wider `soft_window`
    let mut soft_ov: Vec<f64> = vec![];
    // where the curve starts or stops running along a boundary edge (the ends of a long overlap)
    let mut anchors: Vec<Vec3> = vec![];
    // the vertices of the boundary edges met
    let mut verts: Vec<Vec3> = vec![];
    // an edge whose hull box stays clear of the curve's cannot meet it (every hit and overlap of cc::intersect lies
    // within 2 tol of both curves, a parameter slack past the range ends included): skip the intersection
    let hull = curve_hull_box(c, r.0, r.1).map(|b| b.padded(REJECT_PAD * tol + 1e-9));
    // (then span by span: the boxes of the curve's knot spans / arcs, built on first use, against the edge's)
    let mut spans: Option<Option<Bvh>> = None;
    for &(body, f) in faces {
        for e in body.face_edges(f) {
            let ed = body.edge(e);
            if let (Some(h), Some(eb)) = (&hull, curve_hull_box(&ed.curve, ed.t0, ed.t1)) {
                if !h.overlaps(&eb) {
                    continue;
                }
                let sb = spans.get_or_insert_with(|| curve_span_boxes(c, r.0, r.1).map(|v| Bvh::build(&v.iter().map(|b| b.padded(REJECT_PAD * tol + 1e-9)).collect::<Vec<_>>())));
                if let (Some(sb), Some(es)) = (sb.as_ref(), curve_span_boxes(&ed.curve, ed.t0, ed.t1)) {
                    if !es.iter().any(|b| sb.any_overlap(b)) {
                        continue;
                    }
                }
            }
            verts.push(body.vertex(ed.v0).p);
            verts.push(body.vertex(ed.v1).p);
            let res = cc::intersect(c, r, &ed.curve, (ed.t0, ed.t1), tol);
            // a fitted (spline) curve on either side: its tangent touches are off by up to sqrt(2 e R), see `soft_window`
            let fitted = matches!(c, Curve::BSpline(_)) || matches!(ed.curve, Curve::BSpline(_));
            for h in &res.hits {
                // a hit where the two curves run (nearly) tangent is located only to ~sqrt(tol)
                let (ta, tb) = (c.tangent(h.t1), ed.curve.tangent(h.t2));
                if ta.len() > 1e-12 && tb.len() > 1e-12 && ta.cross(tb).len() < 0.1 * ta.len() * tb.len() {
                    if fitted {
                        soft_ov.push(h.t1);
                    }
                    soft.push(h.t1);
                }
            }
            ts.extend(res.hits.iter().map(|h| h.t1));
            for o in res.overlaps {
                ts.push(o.a.0);
                ts.push(o.a.1);
                // a curve that touches an edge tangentially runs within tol of it over a short stretch (about
                // sqrt(2 tol R) each side for curvature radius R): a short overlap is that touch, as unsure as a
                // tangent hit (its ends are soft)
                if fitted && c.eval(o.a.0).dist(c.eval(o.a.1)) < soft_window(tol, bx) {
                    soft_ov.push(o.a.0);
                    soft_ov.push(o.a.1);
                } else {
                    anchors.push(c.eval(o.a.0));
                    anchors.push(c.eval(o.a.1));
                }
            }
        }
    }
    ts.sort_by(|x, y| x.partial_cmp(y).unwrap());
    ts.dedup_by(|x, y| (*x - *y).abs() <= 1e-12 || (*x - *y).abs() <= param_slack(c, *y, tol * 0.5));
    // a tangent contact with a boundary edge (a curve running along the line a face touches tangentially, e.g. a
    // blend face meeting a wall that bounds a joined boss) is found off by up to ~sqrt(tol), next to the exact
    // transversal crossing of the tool's own edge at the same place: only the exact one stays
    if !soft.is_empty() || !soft_ov.is_empty() {
        // (the curve's own ends are exact whatever touches them there)
        let inner = |t: f64| (t - r.0).abs() > 1e-12 && (t - r.1).abs() > 1e-12;
        let near = |v: &[f64], t: f64| v.iter().any(|&s| (s - t).abs() <= 1e-12);
        let is_soft = |t: f64| inner(t) && (near(&soft, t) || near(&soft_ov, t));
        let snap = SOFT_SNAP * (bx.max - bx.min).len();
        let snap_ov = soft_window(tol, bx);
        let hard: Vec<Vec3> = ts.iter().filter(|&&t| !is_soft(t)).map(|&t| c.eval(t)).collect();
        // (a fitted curve's touch yields within the wider window only to the curve's own ends and to where it starts or
        // stops running along an edge: the touch is then at a corner where that run ends, as a blend tool's section
        // curve ends at the tool's corner; next to a transversal crossing it keeps the narrow window)
        anchors.push(c.eval(r.0));
        anchors.push(c.eval(r.1));
        ts.retain(|&t| {
            if !is_soft(t) {
                return true;
            }
            let p = c.eval(t);
            // (a touch on a vertex of the faces' boundary is exact: the vertex is where the curves meet)
            let wide = near(&soft_ov, t) && !verts.iter().any(|q| q.dist(p) <= 2.0 * tol) && anchors.iter().any(|q| q.dist(p) > tol && q.dist(p) < snap_ov);
            !wide && !hard.iter().any(|q| q.dist(p) > tol && q.dist(p) < snap)
        });
    }
    let mut out: Vec<(f64, f64)> = vec![];
    super::bool_dbg!("  trim breakpoints {:?}", ts);
    for w in ts.windows(2) {
        if ck_core::budget::expired() {
            break;
        }
        let (t0, t1) = (w[0], w[1]);
        if c.eval(t0).dist(c.eval(t1)) <= tol && c.eval(t0).dist(c.eval(0.5 * (t0 + t1))) <= tol {
            continue;
        }
        let m = c.eval(0.5 * (t0 + t1));
        let mut all_on = true;
        let mut out_any = false;
        for &(body, f) in faces {
            let w = point_in_face_cached(body, f, m, tol, cache);
            super::bool_dbg!("  trim interval ({t0:.4},{t1:.4}) m={m:?} face {} -> {w:?}", f.idx());
            match w {
                Where::Out => out_any = true,
                Where::In => all_on = false,
                Where::On => {}
            }
        }
        if out_any || (all_on && !allow_all_on) {
            continue;
        }
        out.push((t0, t1));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body};

    /// Two cuts of one face pair that run along each other (here the same line twice) have no crossing between them;
    /// two that cross at an angle are split at their crossing.
    #[test]
    fn cuts_side_by_side_have_no_crossing() {
        let seg = |o: Vec3, d: Vec3| CutSeg { curve: Curve::line(o, d), t0: 0.0, t1: 10.0, faces: vec![(0, FaceId(0)), (1, FaceId(0))] };
        let along = seg(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let mut out = vec![];
        cut_crossings(&along, &seg(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)), 1e-6, &mut out);
        assert!(out.is_empty(), "{out:?}");
        cut_crossings(&along, &seg(Vec3::new(5.0, -5.0, 0.0), Vec3::new(0.0, 1.0, 0.0)), 1e-6, &mut out);
        assert_eq!(out.len(), 1);
        assert!(out[0].dist(Vec3::new(5.0, 0.0, 0.0)) < 1e-9, "{out:?}");
    }

    #[test]
    fn box_cylinder_arrangement() {
        let a = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let b = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let arr = imprint(&a, &b, 1e-6);
        // C0 and C1 of the box get one inner circle each (two pieces: it
        // starts at 0.62·2π and our tool seam splits it at 0); the tool
        // cylinder gets both circles inside
        let c0 = arr.faces.iter().find(|f| f.side == 0 && f.name.id() == "F2.C0").unwrap();
        assert_eq!(c0.inner.len(), 2);
        let (t0, _) = closed_start(std::f64::consts::TAU);
        assert!(c0.inner.iter().any(|&e| arr.edges[e].t0 == t0));
        let side = arr.faces.iter().find(|f| f.side == 1 && f.name.id() == "F4.S.e1").unwrap();
        assert_eq!(side.inner.len(), 4);
        assert!(side.touched());
        let s1 = arr.faces.iter().find(|f| f.side == 0 && f.name.id() == "F2.S.e1").unwrap();
        assert!(!s1.touched());
    }

    /// `plane_cone_apart` against the SSI + trim it stands in for: every pair it calls apart keeps no curve, and the
    /// walls that do cross the cone band are not called apart (a wall across the band whose boundary lies far
    /// outside it, with the band's section in the middle of the face, needs the foot segment).
    #[test]
    fn plane_cone_apart_is_lossless() {
        use crate::boolean::tests::top;
        use ck_geom::Vec2;
        use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
        let tol = 1e-6;
        // drafted circle: a cone band about the y axis, radius 5 at y = 0, height 2
        let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(0.0, 0.0), 5.0)]);
        let prof = Profile::new(top(), vec![ProfileRegion { outer: lp, holes: vec![] }]);
        let cone = extrude("K", &prof, ExtrudeOpts { s0: 0.0, s1: 2.0, dir: None, draft: 0.2 }).unwrap();
        let cf = cone.face_ids().find(|&f| matches!(cone.face(f).surface, Surface::Cone { .. })).expect("cone face");
        let cases: [(&str, f64, f64, f64, f64, bool); 4] = [
            // (name, x0, y0, x1, y1, some wall crosses the band)
            ("far", 9.0, -1.0, 10.0, 1.0, false),  // radially outside the band
            ("core", -1.0, -1.0, 1.0, 1.0, false), // around the axis, inside the band
            ("cross", 4.0, -1.0, 6.0, 1.0, true),  // walls across the band
            ("chord", 3.0, -10.0, 4.0, 10.0, true), // x = 3 and 4 cut the band away from their boundary
        ];
        for (name, x0, y0, x1, y1, crosses) in cases {
            let bx = box_body(name, x0, y0, x1, y1, -1.0, 3.0);
            let mut cache = LoopCache::new();
            let mut any_kept = false;
            for f in bx.face_ids() {
                if !matches!(bx.face(f).surface, Surface::Plane { .. }) {
                    continue;
                }
                let apart = plane_cone_apart(&bx, f, &cone, cf, tol, &mut cache);
                let (b0, b1) = (face_box(&bx, f).padded(1e-3), face_box(&cone, cf).padded(1e-3));
                let un = b0.union(&b1);
                let mut kept = 0;
                if let SsResult::Curves(cs) = ss::intersect(&bx.face(f).surface, &cone.face(cf).surface, Some(Bounds { lo: un.min, hi: un.max }), tol) {
                    for c in cs {
                        let cu = c.curve();
                        kept += trim(cu, cu.range(), &[(&bx, f), (&cone, cf)], &un, tol, false, &mut cache).len();
                    }
                }
                assert!(!(apart && kept > 0), "{name}: face {} called apart but keeps {kept} pieces", bx.face(f).name.id());
                any_kept |= kept > 0;
                if !crosses && !matches!(bx.face(f).name.id().as_str(), s if s.ends_with("C0") || s.ends_with("C1")) {
                    assert!(apart, "{name}: wall {} not called apart", bx.face(f).name.id());
                }
            }
            assert_eq!(any_kept, crosses, "{name}");
        }
    }
}

