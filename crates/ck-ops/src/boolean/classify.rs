//! Classification (ck-ops/src/boolean/classify.rs): point-in-face (uv
//! polygon test, periodic-aware), fragment sample points, and fragment
//! vs. solid classification (In / Out / On-same / On-opposite).
//!
//! Solid containment uses ck-topo's generalised winding number
//! (`region::winding_number`); "on" is decided first by an explicit
//! point-on-face test so coplanar overlaps are resolved by normals.

use ck_geom::curve::Curve;
use ck_geom::pcurve::unwrap_near;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, FaceId};
use ck_topo::region::winding_number;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Where {
    In,
    On,
    Out,
}

/// Fragment (or face) relative to the other solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Class {
    In,
    Out,
    /// On the other solid's boundary, outward normals agree.
    OnSame,
    /// On the other solid's boundary, outward normals opposite.
    OnOpp,
}

/// Samples per curve piece for uv polygons.
pub fn samples_for(c: &Curve) -> usize {
    match c {
        Curve::Line { .. } => 1,
        Curve::Circle { .. } | Curve::Ellipse { .. } => 24,
        Curve::BSpline(b) => (b.ctrl.len() * 4).clamp(8, 128),
        Curve::Conic { .. } | Curve::Sinusoid { .. } => 32,
    }
}

/// Content hash of a B-spline surface for [`uv_near_fp`] (`None` for any other surface): one pass over
/// the control net, so a caller that projects a polygon of points takes it once, not once per point.
pub fn spline_fp(s: &Surface) -> Option<u64> {
    match s {
        Surface::BSpline(bs) => Some(bs.fingerprint()),
        _ => None,
    }
}

/// uv of a 3D point, unwrapped near `prev`.
pub fn uv_near(s: &Surface, p: Vec3, prev: Option<Vec2>) -> Vec2 {
    uv_near_fp(s, None, p, prev)
}

/// [`uv_near`] with the surface's [`spline_fp`] taken by the caller (`None` = not yet known: the
/// per-point hash of `closest_param`). Same answers, bit for bit.
pub fn uv_near_fp(s: &Surface, fp: Option<u64>, p: Vec3, prev: Option<Vec2>) -> Vec2 {
    // long B-spline faces (screw sweeps): the global closest-point search is
    // O(spans); a point on the surface next to the previous sample is found
    // by a local Gauss-Newton projection from it
    if let (Surface::BSpline(bs), Some(q)) = (s, prev) {
        if let Some(uv) = local_project(bs, p, q) {
            return uv;
        }
    }
    if let (Surface::Screw { .. }, Some(q)) = (s, prev) {
        let (u, v, foot) = s.closest_point_near(p, (q.x, q.y));
        if foot.dist(p) < LOCAL_TOL {
            return Vec2::new(u, v);
        }
    }
    let (mut u, mut v) = match (s, fp) {
        (Surface::BSpline(bs), Some(fp)) => bs.closest_param_fp(fp, p),
        _ => s.closest_param(p),
    };
    if let Some(q) = prev {
        let (pu, pv) = s.periods();
        if let Some(per) = pu {
            u = unwrap_near(u, q.x, per);
        }
        if let Some(per) = pv {
            v = unwrap_near(v, q.y, per);
        }
    }
    Vec2::new(u, v)
}

const LOCAL_TOL: f64 = 2e-4;

/// Gauss-Newton projection of `p` onto `bs` from the parameter `q`; Some
/// only when it lands on the surface (distance < 1e-7).
fn local_project(bs: &ck_geom::nurbs::bsurf::BSplineSurface, p: Vec3, q: Vec2) -> Option<Vec2> {
    let ((u0, u1), (v0, v1)) = bs.domain();
    let (mut u, mut v) = (q.x.clamp(u0, u1), q.y.clamp(v0, v1));
    let mut converged = false;
    for _ in 0..12 {
        let d = bs.derivs2(u, v);
        let r = d.p - p;
        if r.len() < 1e-9 {
            return Some(Vec2::new(u, v));
        }
        let (su, sv) = (d.du, d.dv);
        let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let (e, f) = (su.dot(r), sv.dot(r));
        let det = a * c - b * b;
        if det.abs() < 1e-300 {
            return None;
        }
        let du = -(c * e - b * f) / det;
        let dv = -(a * f - b * e) / det;
        u = (u + du).clamp(u0, u1);
        v = (v + dv).clamp(v0, v1);
        if du.abs() < 1e-13 && dv.abs() < 1e-13 {
            converged = true;
            break;
        }
        converged = du.abs() < 1e-9 * (u1 - u0) && dv.abs() < 1e-9 * (v1 - v0);
    }
    // interpolated screw sweeps deviate ~1e-5 from the exact surface, so the
    // arc / rail edges of their faces are that far off it: a converged
    // projection that close is the right foot point, no need for the O(spans)
    // global search
    // (a converged projection is the continuous foot point even when the edge
    // sample is off the face by its fit error -- shelled / offset-fitted
    // surfaces carry ~1e-3 -- and the global search would cost a full scan)
    let dist = bs.eval(u, v).dist(p);
    (dist < LOCAL_TOL || (converged && dist < 2e-4 * (1.0 + p.len()))).then(|| Vec2::new(u, v))
}

/// Samples per curve piece for the polygons of [`Solid`] (point
/// classification): circles / ellipses get enough chords for a sag of
/// ~1e-4 (24 chords sag 0.9% of the radius: a ray hit 0.4 inside the
/// r = 49 hole of an annulus read as inside the face), capped at 720.
fn samples_fine(c: &Curve, ta: f64, tb: f64) -> usize {
    let r = match c {
        Curve::Circle { r, .. } => *r,
        Curve::Ellipse { a, b, .. } => a.max(*b),
        _ => return samples_for(c),
    };
    let alpha = (2.0e-4 / r.max(1e-6)).sqrt().max(1e-6);
    let full = ((std::f64::consts::PI / alpha).ceil() as usize).clamp(samples_for(c), 720);
    // pieces shorter than a full turn need proportionally fewer
    let frac = ((tb - ta).abs() / std::f64::consts::TAU).min(1.0);
    ((full as f64 * frac).ceil() as usize).max(8)
}

/// uv polygon of a loop given as directed curve pieces `(curve, ta, tb)`.
/// `sense` is the face's sense: the loop bounds the face region on its left in
/// uv when `sense` (right-handed parameterization), on its right otherwise.
pub fn loop_polygon(s: &Surface, pieces: &[(&Curve, f64, f64)], sense: bool) -> Vec<Vec2> {
    // fine circle sampling: a chord sag of 0.9% of the radius (24 chords)
    // misclassifies points near a large rim (flange annulus holes)
    loop_polygon_with(s, pieces, sense, samples_fine)
}

/// `c` is a circle on the axis of the revolution surface `s` (cylinder, cone, torus): its uv image is a
/// line of constant v.
fn coaxial_circle(s: &Surface, c: &Curve) -> bool {
    let Curve::Circle { frame: cf, .. } = c else { return false };
    let sf = match s {
        Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Torus { frame, .. } => frame,
        _ => return false,
    };
    let d = cf.o - sf.o;
    cf.z.cross(sf.z).len() < 1e-12 && d.cross(sf.z).len() < 1e-9 * (1.0 + d.len())
}

/// For each piece of a loop that is a seam edge walked both ways ("a seam
/// spike"), the u its samples must be pinned to: one period for the traversal
/// whose region side is the far copy, zero for the other. `None` for every
/// other piece.
///
/// A seam edge is an iso-u line of a periodic surface (`u = 0 mod period`). A
/// spike on it means the loop bounds a region that wraps the seam, and its two
/// traversals must be one period apart or the uv polygon is a zero-width spike
/// (see [`loop_polygon_with`]).
pub(crate) fn seam_spike_u(s: &Surface, pieces: &[(&Curve, f64, f64)], sense: bool) -> Vec<Option<f64>> {
    let mut forced = vec![None; pieces.len()];
    // a surface periodic in both directions (a torus) may wrap either seam, and the
    // copy choice differs per direction: leave those to the existing trim handling
    if s.periods().1.is_some() {
        return forced;
    }
    let Some(per) = s.periods().0 else { return forced };
    // v delta of the traversal when the whole piece is on the seam line
    let seam_dv = |c: &Curve, ta: f64, tb: f64| -> Option<f64> {
        let mut checked = false;
        for t in [ta, 0.5 * (ta + tb), tb] {
            let (u, v) = s.closest_param(c.eval(t));
            // u is arbitrary at a pole: there the seam test says nothing
            if s.is_pole(u, v) {
                continue;
            }
            checked = true;
            let m = u.rem_euclid(per);
            if m.min(per - m) > 1e-6 * per {
                return None;
            }
        }
        if !checked {
            return None;
        }
        Some(s.closest_param(c.eval(ta)).1 - s.closest_param(c.eval(tb)).1)
    };
    for (i, &(c, ta, tb)) in pieces.iter().enumerate() {
        let Some(dv) = seam_dv(c, ta, tb) else { continue };
        if dv.abs() < 1e-12 {
            continue;
        }
        // the edge walked out and straight back: only a consecutive pair of half-edges is a
        // spike (a seam whose two traversals are apart in the loop bounds the whole parameter
        // strip and the continuity chain already places them one period apart)
        let n = pieces.len();
        let twin = [((i + 1) % n), ((i + n - 1) % n)].iter().any(|&j| {
            j != i && {
                let (d, a, b) = pieces[j];
                std::ptr::eq(c as *const Curve, d as *const Curve) && (ta - b).abs() < 1e-9 && (tb - a).abs() < 1e-9
            }
        });
        if twin {
            // up the seam with the region on its left puts the region at -u, so
            // the piece belongs on the far copy; a CW loop (sense false) swaps
            forced[i] = Some(if (dv < 0.0) == sense { per } else { 0.0 });
        }
    }
    forced
}

fn loop_polygon_with(s: &Surface, pieces: &[(&Curve, f64, f64)], sense: bool, nfun: impl Fn(&Curve, f64, f64) -> usize) -> Vec<Vec2> {
    let mut out = vec![];
    let mut prev: Option<Vec2> = None;
    let fp = spline_fp(s);
    // A seam edge walked both ways in this loop (a seam spike) bounds a region
    // that wraps the periodic seam: sampled naively both traversals sit on
    // u = 0, the polygon is a zero-width spike, and its shoelace area is the
    // complement of the region, negated. `group` then reads the whole
    // complement as a hole of one small fragment and `sample_point` finds no
    // interior point. Place the two traversals one period apart instead, each
    // on the copy that keeps the face's region on its left (see
    // `seam_spike_u`).
    let forced = seam_spike_u(s, pieces, sense);
    for (i, &(c, ta, tb)) in pieces.iter().enumerate() {
        let mut n = nfun(c, ta, tb);
        if coaxial_circle(s, c) {
            // a circle coaxial with a cylinder / cone / torus is a straight iso-line in uv: the fine
            // chords are collinear, a quarter-turn step keeps the u unwrapping unambiguous
            let frac = ((tb - ta).abs() / std::f64::consts::TAU).min(1.0);
            n = n.min(((16.0 * frac).ceil() as usize).max(2));
        }
        let start = out.len();
        for k in 0..n {
            let mut uv = uv_near_fp(s, fp, c.eval(ta + (tb - ta) * k as f64 / n as f64), prev);
            // u is arbitrary at a pole: carry the previous u through it so the
            // samples after the pole are not unwrapped against junk
            if let Some(q) = prev {
                if s.is_pole(uv.x, uv.y) {
                    uv.x = q.x;
                }
            }
            prev = Some(uv);
            out.push(uv);
        }
        if let Some(target) = forced[i] {
            // snap the piece to the chosen copy by whole periods (the seam's own
            // u is 0 mod period; poles have no u of their own)
            if let Some(q) = out[start..].iter().find(|q| !s.is_pole(q.x, q.y)).copied() {
                let d = target - q.x;
                for p in &mut out[start..] {
                    p.x += d;
                }
                prev = out.last().copied();
            }
        }
    }
    // the loop's true end point (the samples stop one chord short of it): the
    // winding in u is read from it, so a loop that ends on a circle piece
    // (seam up, seam down, circle) is recognised like one that ends on a seam
    let end_u = match (pieces.last(), out.first()) {
        (Some(&(c, _, tb)), Some(_)) => Some(uv_near_fp(s, fp, c.eval(tb), prev).x),
        _ => None,
    };
    close_pole_spikes(s, &mut out, end_u);
    out
}

/// A loop that reaches a pole of a periodic surface along a seam and comes
/// back along the same seam (a hemisphere cap bounded by one circle:
/// circle, seam up, seam down) encloses nothing in uv as sampled: the up
/// and down seams coincide (a spike). The loop did advance one full period
/// in u (first and last sample differ by it), so the pole is really a
/// line: insert the pole line (u runs one period back to the loop's start)
/// and carry the descent along the start seam. The polygon becomes the
/// strip `[u0, u0 + period] x [circle, pole]`.
fn close_pole_spikes(s: &Surface, poly: &mut Vec<Vec2>, end_u: Option<f64>) {
    let Some(per) = s.periods().0 else { return };
    let n = poly.len();
    if n < 4 {
        return;
    }
    // a loop that starts on a pole and returns to it a full period around
    // (seam down, circle, seam up): the samples stop one chord short of the
    // pole, so without the pole line the closing edge cuts a diagonal across
    // the strip and hides its corner — points near the pole then classify
    // by the sampling (In / Out flips with the circle's chord count)
    if s.is_pole(poly[0].x, poly[0].y) {
        poly.push(Vec2::new(poly[n - 1].x, poly[0].y));
        return;
    }
    let adv = end_u.unwrap_or(poly[n - 1].x) - poly[0].x;
    if (adv.abs() - per).abs() > 1e-6 * per {
        return;
    }
    for i in 1..n - 1 {
        let (prev, p, next) = (poly[i - 1], poly[i], poly[i + 1]);
        if !s.is_pole(p.x, p.y) {
            continue;
        }
        if (prev.x - next.x).abs() > 1e-6 * per {
            continue; // a pole crossing, not a spike
        }
        let shift = -adv.signum() * per;
        for q in poly[i + 1..].iter_mut() {
            q.x += shift;
        }
        poly.insert(i + 1, Vec2::new(p.x + shift, p.y));
        return;
    }
}

/// Shoelace area (positive = counter-clockwise in uv).
pub fn poly_area(p: &[Vec2]) -> f64 {
    let n = p.len();
    let mut a = 0.0;
    for i in 0..n {
        let (x, y) = (p[i], p[(i + 1) % n]);
        a += x.x * y.y - y.x * x.y;
    }
    0.5 * a
}

fn crossings(p: Vec2, poly: &[Vec2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, c) = (poly[i], poly[(i + 1) % n]);
        if (a.y > p.y) != (c.y > p.y) && p.x < a.x + (p.y - a.y) * (c.x - a.x) / (c.y - a.y) {
            inside = !inside;
        }
    }
    inside
}

/// Point-in-polygon with periodic shifts (true if inside for some shift).
pub fn in_polygon(s: &Surface, uv: Vec2, poly: &[Vec2]) -> bool {
    let (pu, pv) = s.periods();
    // shifts that can bring the point into the polygon's extent (a helical
    // boundary on a cylinder spans many periods)
    let shifts = |per: Option<f64>, pick: fn(&Vec2) -> f64, x: f64| -> Vec<f64> {
        match per {
            Some(p) => {
                let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
                for q in poly {
                    lo = lo.min(pick(q));
                    hi = hi.max(pick(q));
                }
                if !lo.is_finite() {
                    return vec![0.0];
                }
                let (k0, k1) = (((lo - x) / p).floor().min(0.0).max(-1e6) as i64 - 1, ((hi - x) / p).ceil().max(0.0).min(1e6) as i64 + 1);
                let mut out = vec![0.0];
                for k in 1..=k0.abs().max(k1.abs()).max(2) {
                    if -k >= k0.min(-2) {
                        out.push(-(k as f64) * p);
                    }
                    if k <= k1.max(2) {
                        out.push(k as f64 * p);
                    }
                }
                out
            }
            None => vec![0.0],
        }
    };
    let su = shifts(pu, |q| q.x, uv.x);
    let sv = shifts(pv, |q| q.y, uv.y);
    su.iter().any(|&a| sv.iter().any(|&b| crossings(Vec2::new(uv.x + a, uv.y + b), poly)))
}

/// Even-odd over loops (each loop tested with periodic shifts).
pub fn in_loops(s: &Surface, uv: Vec2, loops: &[Vec<Vec2>]) -> bool {
    loops.iter().filter(|l| in_polygon(s, uv, l)).count() % 2 == 1
}

/// Directed pieces of the loops of a body face.
pub fn face_loops_uv(b: &Body, f: FaceId) -> Vec<Vec<Vec2>> {
    face_loops_uv_with(b, f, |c, _, _| samples_for(c))
}

/// As [`face_loops_uv`] with the finer circle sampling of [`Solid`].
fn face_loops_uv_fine(b: &Body, f: FaceId) -> Vec<Vec<Vec2>> {
    face_loops_uv_with(b, f, samples_fine)
}

fn face_loops_uv_with(b: &Body, f: FaceId, nfun: impl Fn(&Curve, f64, f64) -> usize + Copy) -> Vec<Vec<Vec2>> {
    let face = b.face(f);
    face.loops
        .iter()
        // a loop that only walks seams (every edge both ways: a revolved
        // sphere's meridian) bounds nothing: the face spans its period
        .filter(|&&l| {
            let cs = &b.lp(l).coedges;
            !cs.iter().all(|&c| cs.iter().any(|&d| b.coedge(d).edge == b.coedge(c).edge && b.coedge(d).rev != b.coedge(c).rev))
        })
        .map(|&l| {
            let pieces: Vec<(&Curve, f64, f64)> = b
                .lp(l)
                .coedges
                .iter()
                .map(|&c| {
                    let (ta, tb) = b.coedge_params(c);
                    (&b.edge(b.coedge(c).edge).curve, ta, tb)
                })
                .collect();
            loop_polygon_with(&face.surface, &pieces, face.sense, nfun)
        })
        .collect()
}

/// False only when `p` is surely farther than `tol` from the spline `c` over `r` (outside the control-point hull
/// box of the spans in range): the closest-point search of [`super::imprint::near_in_range`] can be skipped. Other
/// curves (cheap closed forms) always answer true.
fn near_hull(c: &Curve, r: (f64, f64), p: Vec3, tol: f64) -> bool {
    !matches!(c, Curve::BSpline(_)) || crate::bvh::curve_hull_box(c, r.0, r.1).map_or(true, |bb| bb.padded(tol + 1e-9).contains(p))
}

/// Is `p` (assumed near the face's surface) inside / on the boundary of /
/// outside face `f`?
pub fn point_in_face(b: &Body, f: FaceId, p: Vec3, tol: f64) -> Where {
    point_in_face_cached(b, f, p, tol, &mut std::collections::HashMap::new())
}

/// uv-loop cache of [`point_in_face_cached`], keyed by (body address, face).
pub type LoopCache = std::collections::HashMap<(usize, u32), Vec<Vec<Vec2>>>;

/// As [`point_in_face`], reusing the face's uv polygons across calls (they
/// cost thousands of closest-point searches on a long helical face).
pub fn point_in_face_cached(b: &Body, f: FaceId, p: Vec3, tol: f64, cache: &mut LoopCache) -> Where {
    for e in b.face_edges(f) {
        let ed = b.edge(e);
        if !near_hull(&ed.curve, (ed.t0, ed.t1), p, tol) {
            continue;
        }
        if super::imprint::near_in_range(&ed.curve, (ed.t0, ed.t1), p, tol) {
            return Where::On;
        }
    }
    let face = b.face(f);
    if face.surface.distance(p) > tol * 10.0 {
        return Where::Out;
    }
    let loops = cache.entry((b as *const Body as usize, f.idx() as u32)).or_insert_with(|| face_loops_uv_fine(b, f));
    if loops.iter().all(|l| l.len() < 3) {
        return Where::In; // closed surface without boundary (full sphere/torus)
    }
    let (u, v) = face.surface.closest_param(p);
    if in_loops(&face.surface, Vec2::new(u, v), loops) {
        Where::In
    } else {
        Where::Out
    }
}

/// Classify point `q` (on a fragment with outward normal `n`) against
/// solid `other` (one-shot; prefer [`Solid`] for repeated queries).
pub fn classify_point(other: &Body, q: Vec3, n: Vec3, tol: f64) -> Class {
    Solid::new(other, tol).classify(q, n)
}

/// A body prepared for repeated point classification: face uv polygons,
/// face boxes and a BVH, so each query is an "on" test against the few
/// faces near the point plus ray parity against the faces the ray's box
/// path meets.
pub struct Solid<'a> {
    pub body: &'a Body,
    tol: f64,
    faces: Vec<FaceId>,
    /// uv polygons per face, built on first use (most faces are never near a probe)
    loops: Vec<std::cell::OnceCell<Vec<Vec<Vec2>>>>,
    edges: Vec<Vec<ck_topo::body::EdgeId>>,
    bvh: crate::bvh::Bvh,
}

/// Ray directions tried in turn (irrational-ish, away from the axes so a
/// ray seldom grazes an axis-aligned edge or face).
const RAYS: [[f64; 3]; 6] = [
    [0.5773502691896258, 0.6123724356957945, 0.5400617248673217],
    [-0.3511234415883917, 0.8164965809277260, 0.4580392070833400],
    [0.7071067811865475, -0.2236067977499790, 0.6708203932499369],
    [-0.6, -0.48, 0.64],
    [0.1414213562373095, 0.4242640687119285, -0.8944271909999159],
    [-0.8017837257372732, 0.2672612419124244, -0.5345224838248488],
];

impl<'a> Solid<'a> {
    pub fn new(body: &'a Body, tol: f64) -> Solid<'a> {
        let faces: Vec<FaceId> = body.face_ids().collect();
        let loops = faces.iter().map(|_| std::cell::OnceCell::new()).collect();
        let edges = faces.iter().map(|&f| body.face_edges(f)).collect();
        let boxes: Vec<crate::bvh::Aabb> = faces
            .iter()
            .map(|&f| {
                let bb = crate::bvh::face_box(body, f);
                // sampled boxes under-cover curved faces by < 0.5% of their size
                bb.padded(tol * 10.0 + 0.02 * bb.diagonal())
            })
            .collect();
        Solid { body, tol, faces, loops, edges, bvh: crate::bvh::Bvh::build(&boxes) }
    }

    /// Point `p` (near face `i`'s surface) inside / on / outside face `i`.
    fn in_face(&self, i: usize, p: Vec3) -> Where {
        self.in_face_within(i, p, self.tol * 10.0)
    }

    /// [`Self::in_face`] for a point up to `lim` off the face's surface (its foot point decides).
    fn in_face_within(&self, i: usize, p: Vec3, lim: f64) -> Where {
        let b = self.body;
        for &e in &self.edges[i] {
            let ed = b.edge(e);
            if !near_hull(&ed.curve, (ed.t0, ed.t1), p, self.tol) {
                continue;
            }
            if super::imprint::near_in_range(&ed.curve, (ed.t0, ed.t1), p, self.tol) {
                return Where::On;
            }
        }
        let s = &b.face(self.faces[i]).surface;
        if s.distance(p) > lim {
            return Where::Out;
        }
        let loops = self.loops[i].get_or_init(|| face_loops_uv_fine(b, self.faces[i]));
        if loops.iter().all(|l| l.len() < 3) {
            return Where::In;
        }
        let (u, v) = s.closest_param(p);
        if in_loops(s, Vec2::new(u, v), loops) {
            Where::In
        } else {
            Where::Out
        }
    }

    /// Classify point `q` (with outward normal `n` of the fragment it
    /// samples).
    pub fn classify(&self, q: Vec3, n: Vec3) -> Class {
        let (b, tol) = (self.body, self.tol);
        if self.faces.is_empty() {
            return Class::Out;
        }
        let near = crate::bvh::Aabb::new(q, q).padded(tol * 10.0);
        let robust = super::robust();
        for i in self.bvh.query_box(&near) {
            let g = self.faces[i];
            let (u, v, cp) = b.face(g).surface.closest_point(q);
            let d = cp.dist(q);
            if d > tol * 10.0 {
                continue;
            }
            if self.in_face(i, q) == Where::Out {
                continue;
            }
            let ng = b.outward_normal(g, u, v);
            return if ng.dot(n) > 0.0 { Class::OnSame } else { Class::OnOpp };
        }
        // Not on any face: In/Out by ray parity. In a robust retry take the majority of all rays and the
        // winding number on a tie, rather than trusting the first decisive ray: a single ray can miss a
        // crossing on a spline face (iterative `line_bspline`), which flips the parity of a point far from
        // the body — the thickened wide cone onto a plate drops the plate caps because the inner cone
        // wall's second crossing is missed and a cap far outside the tool is read as In. Sliver fragments
        // (a thin lens between near-tangent surfaces) hit the same vote because their single rays disagree.
        if robust {
            let (mut ins, mut outs) = (0usize, 0usize);
            for d in RAYS {
                match self.ray_parity(q, Vec3::new(d[0], d[1], d[2])) {
                    Some(true) => ins += 1,
                    Some(false) => outs += 1,
                    None => {}
                }
            }
            if ins != outs && (ins == 0 || outs == 0 || ins.abs_diff(outs) >= 2) {
                return if ins > outs { Class::In } else { Class::Out };
            }
            return if winding_number(b, &self.faces, q) > 0.5 { Class::In } else { Class::Out };
        }
        for d in RAYS {
            if let Some(inside) = self.ray_parity(q, Vec3::new(d[0], d[1], d[2])) {
                return if inside { Class::In } else { Class::Out };
            }
        }
        // every ray ambiguous (or a face without a closed-form line
        // intersection): generalised winding number
        if winding_number(b, &self.faces, q) > 0.5 {
            Class::In
        } else {
            Class::Out
        }
    }

    /// Parity of the crossings of the ray `q + t d` (t > 0); None when the
    /// ray grazes a face, passes through an edge, or meets a surface
    /// without a closed-form line intersection.
    fn ray_parity(&self, q: Vec3, d: Vec3) -> Option<bool> {
        let b = self.body;
        let mut count = 0usize;
        let mut seen = vec![];
        for (i, _) in self.bvh.query_ray(q, d, 0.0, f64::INFINITY) {
            if seen.contains(&i) {
                continue;
            }
            seen.push(i);
            let s = &b.face(self.faces[i]).surface;
            let mut ts = crate::intersect::cs::line_params(q, d, s, self.tol)?;
            ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let mut prev: Option<f64> = None;
            for t in ts {
                if t.abs() <= self.tol * 10.0 {
                    // q on this surface: outside the face there is no crossing
                    // at the start of the ray (the fallback to the winding
                    // number costs seconds); on its boundary / inside: ambiguous
                    if self.in_face(i, q) == Where::Out {
                        continue;
                    }
                    return None;
                }
                if t < 0.0 {
                    continue;
                }
                if prev.map_or(false, |p| (t - p).abs() <= self.tol * 10.0) {
                    return None; // double root: tangent
                }
                prev = Some(t);
                let p = q + d * t;
                match self.in_face(i, p) {
                    Where::On => return None,
                    Where::Out => {}
                    Where::In => {
                        let (u, v) = s.closest_param(p);
                        let nn = s.normal(u, v).normalize();
                        if nn.dot(d).abs() < 1e-3 {
                            return None; // grazing
                        }
                        count += 1;
                    }
                }
            }
        }
        Some(count % 2 == 1)
    }
}

/// Outward normal of a (surface, sense) pair at the closest point to `p`.
pub fn outward(s: &Surface, sense: bool, p: Vec3) -> Vec3 {
    let (u, v) = s.closest_param(p);
    let n = s.normal(u, v);
    if sense {
        n
    } else {
        -n
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body};

    #[test]
    fn faces_and_solids() {
        let a = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let c0 = a.find_face("F2.C0").unwrap();
        assert_eq!(point_in_face(&a, c0, Vec3::new(5., 0., -5.), 1e-6), Where::In);
        assert_eq!(point_in_face(&a, c0, Vec3::new(0., 0., -5.), 1e-6), Where::On);
        assert_eq!(point_in_face(&a, c0, Vec3::new(45., 0., -5.), 1e-6), Where::Out);
        assert_eq!(classify_point(&a, Vec3::new(5., 5., -5.), Vec3::X, 1e-6), Class::In);
        assert_eq!(classify_point(&a, Vec3::new(50., 5., -5.), Vec3::X, 1e-6), Class::Out);
        assert_eq!(classify_point(&a, Vec3::new(5., 0., -5.), -Vec3::Y, 1e-6), Class::OnSame);
        assert_eq!(classify_point(&a, Vec3::new(5., 0., -5.), Vec3::Y, 1e-6), Class::OnOpp);
        // periodic face
        let c = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let s = c.find_face("F4.S.e1").unwrap();
        for ang in [0.1f64, 1.0, 3.0, 6.2] {
            let p = Vec3::new(20.0 + 5.0 * ang.cos(), 4.0, -10.0 - 5.0 * ang.sin());
            assert_eq!(point_in_face(&c, s, p, 1e-6), Where::In, "{ang}");
        }
        assert_eq!(point_in_face(&c, s, Vec3::new(25.0, 12.0, -10.0), 1e-6), Where::Out);
    }

    /// `uv_near_fp` with the surface hash taken once answers exactly what `uv_near` does per point.
    #[test]
    fn uv_near_fp_is_bit_identical() {
        use ck_geom::nurbs::bsurf::BSplineSurface;
        let n = 6;
        let knots = vec![0.0, 0.0, 0.0, 0.0, 0.3, 0.7, 1.0, 1.0, 1.0, 1.0];
        let ctrl: Vec<Vec3> = (0..n * n).map(|k| Vec3::new((k / n) as f64 * 3.0, (k % n) as f64 * 3.0, ((k * 7919 % 13) as f64 - 6.0) * 0.4)).collect();
        let s = Surface::BSpline(Box::new(BSplineSurface::new(3, 3, knots.clone(), knots, n, n, ctrl, None).unwrap()));
        let fp = spline_fp(&s);
        assert!(fp.is_some());
        let mut prev = None;
        for k in 0..60 {
            let t = k as f64 / 59.0;
            let p = Vec3::new(2.0 + 11.0 * t, 1.5 + 9.0 * (6.0 * t).sin().abs(), 3.0 * (t * 9.0).cos());
            let a = uv_near(&s, p, prev);
            let b = uv_near_fp(&s, fp, p, prev);
            assert_eq!((a.x.to_bits(), a.y.to_bits()), (b.x.to_bits(), b.y.to_bits()), "{k}");
            prev = Some(a);
        }
    }
}
