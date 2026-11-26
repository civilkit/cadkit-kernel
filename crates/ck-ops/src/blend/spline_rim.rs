//! A round or chamfer on the rim of an extruded closed profile made of free-form curves (closed or open splines, ellipse-like
//! blobs, a D, a circle with a flat) and straight lines and arcs: built in place, with no tool and no boolean.
//!
//! The rim between a plane cap and a straight extruded wall is a plain geometry. With `C(t)` the rim (a curve on the cap plane),
//! `n(t)` its outward normal in that plane and `u` the unit direction from the rim into the wall (away from the cap):
//!
//! ```text
//!   wall contact   W(t) = C(t) + r u        (the rim moved down the wall: exact)
//!   corner         M(t) = C(t)
//!   cap contact    T(t) = C(t) - r n(t)     (the rim offset inward on the cap)
//!   round          S(t, w) = the quarter arc W - M - T  (rational quadratic, weights 1, 1/sqrt 2, 1), one arc per t
//!   chamfer        S(t, w) = the segment W - T
//! ```
//!
//! so the blend face is a tensor-product B-spline whose three control rows are curves of the profile: two of them exact, the
//! third (the offset) a piecewise cubic Hermite fit to a few 1e-7 mm. The topology is edited where it stands: the rim edge
//! becomes the wall contact edge, a new edge carries the cap contact, the seam edges between the wall's faces are shortened,
//! and each seam vertex is split into the wall vertex and the cap vertex.
//!
//! * At a tangent-continuous vertex the two are joined by the quarter-circle edge (the chamfer's straight edge) shared by the two
//!   neighbouring blend faces.
//! * At a sharp convex corner (a spline meeting a line at an angle, a mitre) the two blend faces cross: the edge between them is
//!   the intersection of the two strips, found by Newton's method on the strips' own formulas (the same function the surfaces are
//!   fitted to), and carried as an interpolating B-spline with its two pcurves; both cap contact edges end at the point where the
//!   two offset curves meet.
//! * A link that is an arc of radius equal to the size (a vertical edge rounded with the same radius as the rim) collapses its cap
//!   contact to the arc's centre: its blend face is a spherical triangle.
//!
//! The pattern is recognised first (`None` hands the selection to the other builders): every edge of the cap's outer loop a
//! B-spline on a straight extruded wall perpendicular to the cap, a straight line on a plane wall, or an arc on a cylinder wall
//! (at least one B-spline), every joint tangent-continuous or a sharp convex corner, the seam edges plain lines. One pick of a
//! smooth ring rounds the whole ring; a ring with a sharp corner is built when every edge of it is picked.
//!
//! The vertical corner edges of such a profile (between the walls of two links, with or without the rim in the same selection) are
//! rounded or chamfered here too (`spline_vertical.rs`), which leaves a loop of lines and arcs the rim builder then rounds.

use super::diag::{dbg, unsupported_hint};
use super::find::find_edge;
use super::model::{ChamferLeg, FilletProfile, Spec};
use super::BlendError;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::fit::{interpolate, Param};
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, FaceName, VertexId};
use std::collections::{HashMap, HashSet};

/// Accuracy of the fitted cap contact curve (mm).
const FIT_TOL: f64 = 1e-6;
/// Accuracy of the cubic pieces of an arc (mm).
const ARC_TOL: f64 = 5e-8;
/// Samples along a mitre edge.
const MITRE_N: usize = 24;

pub(super) enum Miss {
    /// Not this pattern: the other builders take the selection.
    NotMine,
    /// This pattern, and the round cannot be made as asked.
    Refuse(BlendError),
}
pub(super) type R<T> = Result<T, Miss>;

pub(super) fn nm<T>(line: u32) -> R<T> {
    dbg(&format!("spline rim: not the pattern (spline_rim.rs:{line})"));
    Err(Miss::NotMine)
}

pub(super) fn refuse<T>(detail: &str, hint: &str) -> R<T> {
    Err(Miss::Refuse(unsupported_hint(detail, hint)))
}

/// `Some` when the selection is the rim of an extruded free-form profile (a ring) or rings of such profiles, with or without the
/// vertical corner edges of the profile.
pub(super) fn try_rim(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Option<Result<Body, BlendError>> {
    let (r, chamfer) = match spec {
        Spec::Fillet(r, FilletProfile::Circular) => (r, false),
        Spec::Chamfer(d, ChamferLeg::Dist(d2)) if (d - d2).abs() <= 1e-12 * d.abs().max(1.0) => (d, true),
        _ => return None,
    };
    if !(r > 0.0) || !r.is_finite() {
        return None;
    }
    let picks: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    if picks.is_empty() || picks.len() != edges.len() {
        return None;
    }
    run(b, &picks, &vec![r; picks.len()], chamfer, feature, tol)
}

/// [`try_rim`] with a size for each pick (a vertical edge rounded with another radius than the rim).
pub fn try_mixed(b: &Body, edges: &[(String, String, Vec3)], sizes: &[f64], chamfer: bool, feature: &str, tol: f64) -> Option<Result<Body, BlendError>> {
    if sizes.iter().any(|r| !(*r > 0.0) || !r.is_finite()) {
        return None;
    }
    let picks: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    if picks.is_empty() || picks.len() != edges.len() || sizes.len() != picks.len() {
        return None;
    }
    run(b, &picks, sizes, chamfer, feature, tol)
}

fn run(b: &Body, picks: &[EdgeId], sizes: &[f64], chamfer: bool, feature: &str, tol: f64) -> Option<Result<Body, BlendError>> {
    match build(b, picks, sizes, chamfer, feature, tol) {
        Ok(out) => Some(Ok(out)),
        Err(Miss::NotMine) => None,
        Err(Miss::Refuse(e)) => Some(Err(e)),
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub(super) enum Wall {
    /// extruded B-spline
    Ext,
    Plane,
    Cyl,
}

/// One edge of a ring and what the build needs of it.
pub(super) struct Link {
    pub e: EdgeId,
    /// coedge of the edge on the cap, and on the wall
    pub c_cap: CoedgeId,
    pub c_wall: CoedgeId,
    /// sign of the outward normal: n = sig * (C' x dir) / |C' x dir|
    pub sig: f64,
    pub wall: Wall,
    /// the wall pcurve moves by `(ax[0] . u_in, ax[1] . u_in) r` when the contact moves down the wall
    pub ax: [Vec3; 2],
}

pub(super) struct Ring {
    pub cap: FaceId,
    pub links: Vec<Link>,
    /// unit direction along the extrusion (the cap normal, up to sign)
    pub dir: Vec3,
    /// into the wall, away from the cap
    pub u_in: Vec3,
    /// cap outward normal
    pub ncap: Vec3,
    /// seam edge at each ring vertex
    pub seam: HashMap<VertexId, EdgeId>,
}

pub(super) fn find_ring(b: &Body, e0: EdgeId) -> R<Ring> {
    let fs = b.edge_faces(e0);
    if fs.len() != 2 {
        return nm(line!());
    }
    for &cap in &fs {
        if !matches!(b.face(cap).surface, Surface::Plane { .. }) {
            continue;
        }
        match find_ring_cap(b, e0, cap) {
            Err(Miss::NotMine) => continue,
            other => return other,
        }
    }
    nm(line!())
}

fn find_ring_cap(b: &Body, e0: EdgeId, cap: FaceId) -> R<Ring> {
    let ncap = b.outward_normal(cap, 0.0, 0.0);
    // the cap's loop that holds the edge
    let lp = b.face(cap).loops.iter().copied().find(|&l| b.lp(l).alive && b.lp(l).coedges.iter().any(|&c| b.coedge(c).edge == e0));
    let Some(lp) = lp else { return nm(line!()) };
    let dir = ncap;
    let mut links: Vec<Link> = vec![];
    for &c_cap in &b.lp(lp).coedges {
        let e = b.coedge(c_cap).edge;
        let ed = b.edge(e);
        let faces = b.edge_faces(e);
        if faces.len() != 2 || !faces.contains(&cap) {
            return nm(line!());
        }
        let wall_f = if faces[0] == cap { faces[1] } else { faces[0] };
        let Some(c_wall) = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == wall_f) else { return nm(line!()) };
        if b.edge_coedges(e).iter().filter(|&&c| b.coedge_face(c) == wall_f).count() != 1 {
            return nm(line!());
        }
        let Some(pc @ Curve2::Line { o, d }) = &b.coedge(c_wall).pcurve else { return nm(line!()) };
        let (wall, ax) = match (&ed.curve, &b.face(wall_f).surface) {
            (Curve::BSpline(_), Surface::Extrusion { dir: wd, profile }) => {
                if !matches!(**profile, Curve::BSpline(_)) {
                    return nm(line!());
                }
                let wdir = wd.normalize();
                if (wd.len() - 1.0).abs() > 1e-9 || wdir.dot(ncap).abs() < 1.0 - 1e-9 {
                    return nm(line!());
                }
                if (d.x - 1.0).abs() > 1e-9 || d.y.abs() > 1e-9 || o.x.abs() > 1e-9 {
                    return nm(line!());
                }
                (Wall::Ext, [Vec3::ZERO, wdir])
            }
            (Curve::Line { .. }, Surface::Plane { frame }) => {
                if frame.z.dot(ncap).abs() > 1e-9 {
                    return nm(line!());
                }
                (Wall::Plane, [frame.x, frame.y])
            }
            (Curve::Circle { frame: cf, .. }, Surface::Cylinder { frame, .. }) => {
                if frame.z.dot(ncap).abs() < 1.0 - 1e-9 || cf.z.dot(ncap).abs() < 1.0 - 1e-9 {
                    return nm(line!());
                }
                if (d.x.abs() - 1.0).abs() > 1e-9 || d.y.abs() > 1e-9 {
                    return nm(line!());
                }
                (Wall::Cyl, [Vec3::ZERO, frame.z])
            }
            _ => return nm(line!()),
        };
        // the sign of n: toward the wall's outward normal at the middle of the edge
        let tm = 0.5 * (ed.t0 + ed.t1);
        let [_, g] = ed.curve.derivs1_arr(tm);
        let gx = g.cross(dir);
        if gx.len() < 1e-12 {
            return nm(line!());
        }
        let uv = pc.eval(tm);
        let on = b.outward_normal(wall_f, uv.x, uv.y);
        let sig = if gx.dot(on) >= 0.0 { 1.0 } else { -1.0 };
        links.push(Link { e, c_cap, c_wall, sig, wall, ax });
    }
    if links.is_empty() || !links.iter().any(|l| l.wall == Wall::Ext) {
        return nm(line!());
    }
    // the seam edge at each ring vertex: one straight edge along the extrusion, 3-valent vertices
    let ring_edges: Vec<EdgeId> = links.iter().map(|l| l.e).collect();
    let mut seam: HashMap<VertexId, EdgeId> = HashMap::new();
    let mut u_in = Vec3::ZERO;
    for l in &links {
        for v in [b.edge(l.e).v0, b.edge(l.e).v1] {
            if seam.contains_key(&v) {
                continue;
            }
            let inc: Vec<EdgeId> = b.edge_ids().filter(|&x| b.edge_coedges(x).len() > 0 && (b.edge(x).v0 == v || b.edge(x).v1 == v)).collect();
            let others: Vec<EdgeId> = inc.iter().copied().filter(|x| !ring_edges.contains(x)).collect();
            if others.len() != 1 || inc.len() - others.len() > 2 {
                return nm(line!());
            }
            let s = others[0];
            let sd = b.edge(s);
            let Curve::Line { d, .. } = &sd.curve else { return nm(line!()) };
            if (d.len() - 1.0).abs() > 1e-9 || d.dot(dir).abs() < 1.0 - 1e-9 || sd.v0 == sd.v1 {
                return nm(line!());
            }
            // into the wall: from this vertex to the seam's other end; a convex rim has the wall below the cap
            let other = if sd.v0 == v { sd.v1 } else { sd.v0 };
            let dv = b.vertex(other).p - b.vertex(v).p;
            let len = dv.len();
            if len < 1e-9 {
                return nm(line!());
            }
            let ui = dv * (1.0 / len);
            if ui.dot(ncap) > -1.0 + 1e-9 {
                return nm(line!());
            }
            if u_in == Vec3::ZERO {
                u_in = ui;
            } else if u_in.dot(ui) < 1.0 - 1e-9 {
                return nm(line!());
            }
            seam.insert(v, s);
        }
    }
    Ok(Ring { cap, links, dir, u_in, ncap, seam })
}

/// Outward normal in the cap plane and the speed of `C` at `t` (unit `n`, `|C'|`), with the derivative of `n`.
pub(super) fn offset_terms(c: &Curve, t: f64, dir: Vec3, sig: f64) -> (Vec3, Vec3, Vec3, Vec3, f64) {
    let [p, g, g2] = c.derivs2_arr(t);
    let gx = g.cross(dir);
    let m = gx.len();
    let n = gx * (sig / m);
    let gx2 = g2.cross(dir);
    let dn = (gx2 * (1.0 / m) - gx * (g.dot(g2) / (m * m * m))) * sig;
    (p, n, dn, g, m)
}

/// Bezier pieces of one row, on common breakpoints.
struct Rows {
    breaks: Vec<f64>,
    w: Vec<[Vec3; 4]>,
    m: Vec<[Vec3; 4]>,
    t: Vec<[Vec3; 4]>,
    /// smallest `1 - r kappa` (the speed of the offset over the speed of the rim) met
    fmin: f64,
    /// stretches where the offset is trimmed (a bend tighter than the size)
    fans: Vec<Fan>,
}

/// Polar form of a cubic Bezier at three parameters (the control points of a sub-interval are its values).
fn blossom(p: &[Vec3; 4], a: f64, b: f64, c: f64) -> Vec3 {
    let l = |x: Vec3, y: Vec3, t: f64| x * (1.0 - t) + y * t;
    let q0 = [l(p[0], p[1], a), l(p[1], p[2], a), l(p[2], p[3], a)];
    let q1 = [l(q0[0], q0[1], b), l(q0[1], q0[2], b)];
    l(q1[0], q1[1], c)
}

fn bez(p: &[Vec3; 4], s: f64) -> Vec3 {
    let (a, b) = (1.0 - s, s);
    p[0] * (a * a * a) + p[1] * (3.0 * a * a * b) + p[2] * (3.0 * a * b * b) + p[3] * (b * b * b)
}

/// Offset `T(t) = C - r n`, its velocity and the speed ratio `1 - r kappa` (negative where the offset runs backwards).
fn offset_at(c: &Curve, t: f64, r: f64, dir: Vec3, sig: f64) -> (Vec3, Vec3, f64) {
    let (p, n, dn, g, _) = offset_terms(c, t, dir, sig);
    let td = g - dn * r;
    (p - n * r, td, td.dot(g) / g.dot(g))
}

/// A stretch `[ta, tb]` of an edge where the inward offset has a swallowtail (the rim bends tighter than the size), with the
/// point `p` where the two surviving branches of the offset meet.
#[derive(Clone, Copy)]
struct Fan {
    ta: f64,
    tb: f64,
    p: Vec3,
}

fn seg_cross(a0: (f64, f64), a1: (f64, f64), b0: (f64, f64), b1: (f64, f64)) -> bool {
    let o = |p: (f64, f64), q: (f64, f64), s: (f64, f64)| (q.0 - p.0) * (s.1 - p.1) - (q.1 - p.1) * (s.0 - p.0);
    let (d1, d2, d3, d4) = (o(b0, b1, a0), o(b0, b1, a1), o(a0, a1, b0), o(a0, a1, b1));
    d1 * d2 <= 0.0 && d3 * d4 <= 0.0 && (d1 != d2 || d3 != d4)
}

/// The rim of one ring as one closed curve `u` in `[0, K)` (link `k` covers `[k, k+1]`, in loop order), so that a swallowtail of the
/// inward offset may run across the vertex between two edges, or across the seam of a closed one.
struct Comp<'a> {
    links: Vec<(&'a Curve, f64, f64, bool, f64)>,
    r: f64,
    dir: Vec3,
}

impl Comp<'_> {
    fn k(&self) -> f64 {
        self.links.len() as f64
    }
    /// (offset point, its derivative in `u`, `1 - r kappa`)
    fn at(&self, u: f64) -> (Vec3, Vec3, f64) {
        let kk = self.k();
        let u = u.rem_euclid(kk);
        let k = (u.floor() as usize).min(self.links.len() - 1);
        let (c, t0, t1, rev, sig) = self.links[k];
        let frac = u - k as f64;
        let (t, dt) = if rev { (t0 + (t1 - t0) * (1.0 - frac), -(t1 - t0)) } else { (t0 + (t1 - t0) * frac, t1 - t0) };
        let (p, td, f) = offset_at(c, t, self.r, self.dir, sig);
        (p, td * dt, f)
    }
}

/// Where the rim's convex bend is tighter than `r` the offset runs backwards between two cusps and crosses itself; the true
/// offset (the boundary of the rim's erosion by `r`) skips the loop. Returns the stretches `[ua, ub]` of `u` between the two
/// crossing branches (`ub - ua < K`, `ua` may be negative) and the crossing point. A loop that does not close is refused.
fn find_fans(cm: &Comp) -> R<Vec<(f64, f64, Vec3)>> {
    const PER: usize = 2000;
    let (r, dir) = (cm.r, cm.dir);
    let kk = cm.links.len();
    let m = PER * kk;
    let uu = |i: usize| i as f64 / PER as f64;
    let neg: Vec<bool> = (0..m).map(|i| cm.at(uu(i)).2 < 0.0).collect();
    let Some(s0) = neg.iter().position(|&x| !x) else {
        return refuse(&format!("the round {r} does not fit: the whole rim bends tighter than the size"), "use a smaller size");
    };
    let e1 = if dir.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let e1 = (e1 - dir * e1.dot(dir)).normalize();
    let e2 = dir.cross(e1);
    let xy = |p: Vec3| (p.dot(e1), p.dot(e2));
    let mut fans: Vec<(f64, f64, Vec3)> = vec![];
    let mut k = 1;
    while k <= m {
        let i = (s0 + k) % m;
        if !neg[i] {
            k += 1;
            continue;
        }
        // a run of negative samples s0+ka ..= s0+kb (unwrapped indices)
        let ka = k;
        while k <= m && neg[(s0 + k) % m] {
            k += 1;
        }
        let kb = k - 1;
        let (ia, ib) = (s0 + ka, s0 + kb);
        let f = |u: f64| cm.at(u).2;
        let bis = |mut lo: f64, mut hi: f64, lo_neg: bool| {
            for _ in 0..70 {
                let mid = 0.5 * (lo + hi);
                if (f(mid) < 0.0) == lo_neg { lo = mid } else { hi = mid }
            }
            0.5 * (lo + hi)
        };
        let tc1 = bis(uu(ia - 1), uu(ia), false);
        let tc2 = bis(uu(ib), uu(ib + 1), true);
        let mut delta = (tc2 - tc1).max(1.0 / PER as f64);
        let mut found: Option<(f64, f64)> = None;
        let kf = kk as f64;
        for _ in 0..20 {
            let dl = delta.min(0.5 * kf - 0.5 * (tc2 - tc1)).max(0.0);
            let (alo, bhi) = (tc1 - dl, tc2 + dl);
            let n = 1500usize;
            let sa: Vec<f64> = (0..=n).map(|i| alo + (tc1 - alo) * i as f64 / n as f64).collect();
            let sb: Vec<f64> = (0..=n).map(|i| tc2 + (bhi - tc2) * i as f64 / n as f64).collect();
            let pa: Vec<(f64, f64)> = sa.iter().map(|&t| xy(cm.at(t).0)).collect();
            let pb: Vec<(f64, f64)> = sb.iter().map(|&t| xy(cm.at(t).0)).collect();
            let mut best: Option<(f64, usize, usize)> = None;
            for i in 0..n {
                let (a0, a1) = (pa[i], pa[i + 1]);
                let (ax0, ax1, ay0, ay1) = (a0.0.min(a1.0), a0.0.max(a1.0), a0.1.min(a1.1), a0.1.max(a1.1));
                for j in 0..n {
                    let (b0, b1) = (pb[j], pb[j + 1]);
                    if b0.0.max(b1.0) < ax0 || b0.0.min(b1.0) > ax1 || b0.1.max(b1.1) < ay0 || b0.1.min(b1.1) > ay1 {
                        continue;
                    }
                    if seg_cross(a0, a1, b0, b1) {
                        let span = sb[j] - sa[i];
                        if best.map_or(true, |(s, _, _)| span < s) {
                            best = Some((span, i, j));
                        }
                    }
                }
            }
            if let Some((_, i, j)) = best {
                found = Some((0.5 * (sa[i] + sa[i + 1]), 0.5 * (sb[j] + sb[j + 1])));
                break;
            }
            if dl >= 0.5 * kf - 0.5 * (tc2 - tc1) - 1e-12 {
                break;
            }
            delta *= 2.0;
        }
        let Some((mut ta, mut tb)) = found else {
            dbg(&format!("spline rim: no crossing for cusps {tc1} {tc2} size {r}"));
            return refuse(
                &format!("the round {r} does not fit: the inside offset of a tight bend of the rim does not close its loop (the rim is too short round the bend)"),
                "use a smaller size",
            );
        };
        for _ in 0..40 {
            let (pa, da, _) = cm.at(ta);
            let (pb, db, _) = cm.at(tb);
            let res = pa - pb;
            let (a11, a12, a22) = (da.dot(da), -da.dot(db), db.dot(db));
            let (b1, b2) = (-da.dot(res), db.dot(res));
            let det = a11 * a22 - a12 * a12;
            if det.abs() < 1e-300 {
                break;
            }
            let (dta, dtb) = ((b1 * a22 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det);
            ta += dta;
            tb += dtb;
            if dta.abs() + dtb.abs() < 1e-14 {
                break;
            }
        }
        let (pa, _, fa) = cm.at(ta);
        let (pb, _, fb) = cm.at(tb);
        if pa.dist(pb) > 1e-7 || !(ta < tc1) || !(tb > tc2) || fa <= 0.0 || fb <= 0.0 || tb - ta >= cm.k() {
            dbg(&format!("spline rim: swallowtail crossing not resolved ta {ta} tb {tb} cusps {tc1} {tc2} gap {}", pa.dist(pb)));
            return refuse(&format!("the round {r} does not fit: the inside offset of a tight bend of the rim could not be trimmed"), "use a smaller size");
        }
        dbg(&format!("spline rim: tight bend, offset loop cusps {tc1:.6} {tc2:.6} trimmed at {ta:.6} {tb:.6} (of {kk} edges), meeting point {pa:?}"));
        fans.push((ta, tb, (pa + pb) * 0.5));
    }
    Ok(fans)
}

/// Per link the stretches of its parameter inside a fan, and the vertices that lie inside one (their cap point is the meeting
/// point). A fan that swallows a whole edge is refused.
fn ring_fans(b: &Body, ring: &Ring, r: f64, chamfer: bool, sharp: &HashSet<VertexId>) -> R<(Vec<Vec<Fan>>, HashMap<VertexId, Vec3>)> {
    let links: Vec<(&Curve, f64, f64, bool, f64)> = ring.links.iter().map(|l| (&b.edge(l.e).curve, b.edge(l.e).t0, b.edge(l.e).t1, b.coedge(l.c_cap).rev, l.sig)).collect();
    let cm = Comp { links, r, dir: ring.dir };
    let found = find_fans(&cm)?;
    let kk = ring.links.len();
    let mut per: Vec<Vec<Fan>> = vec![vec![]; kk];
    let mut vp: HashMap<VertexId, Vec3> = HashMap::new();
    let what = if chamfer { "chamfer" } else { "round" };
    for &(ua, ub, p) in &found {
        for shift in [-(kk as f64), 0.0, kk as f64] {
            let (a, bb) = (ua + shift, ub + shift);
            for (k, l) in ring.links.iter().enumerate() {
                let (lo, hi) = (a.max(k as f64), bb.min(k as f64 + 1.0));
                if hi - lo < 1e-9 {
                    continue;
                }
                let ed = b.edge(l.e);
                let rev = b.coedge(l.c_cap).rev;
                let to_t = |u: f64| {
                    let fr = u - k as f64;
                    ed.t0 + (ed.t1 - ed.t0) * if rev { 1.0 - fr } else { fr }
                };
                let (x, y) = (to_t(lo), to_t(hi));
                let (ta, tb) = (x.min(y), x.max(y));
                if !matches!(ed.curve, Curve::BSpline(_)) {
                    return refuse(
                        &format!("the {what} {r} does not fit: a tight bend of the rim reaches into a straight or circular edge of the profile"),
                        "use a smaller size",
                    );
                }
                if ta <= ed.t0 + 1e-9 && tb >= ed.t1 - 1e-9 {
                    return refuse(
                        &format!("the {what} {r} does not fit: a tight bend of the rim lies inside one short edge of the profile"),
                        "use a smaller size",
                    );
                }
                per[k].push(Fan { ta, tb, p });
            }
            // the vertices strictly inside the fan: the start vertex of link m is at u = m
            for (m, l) in ring.links.iter().enumerate() {
                let u = m as f64;
                let sv = {
                    let ed = b.edge(l.e);
                    if b.coedge(l.c_cap).rev { ed.v1 } else { ed.v0 }
                };
                if sharp.contains(&sv) && u > a - 1e-9 && u < bb + 1e-9 {
                    return refuse(
                        &format!("the {what} {r} does not fit: a tight bend of the rim reaches a sharp corner of the profile"),
                        "use a smaller size",
                    );
                }
                if u > a + 1e-9 && u < bb - 1e-9 {
                    let ed = b.edge(l.e);
                    let v = if b.coedge(l.c_cap).rev { ed.v1 } else { ed.v0 };
                    vp.insert(v, p);
                }
            }
        }
    }
    Ok((per, vp))
}

/// The rim edge's curve over `[t0, t1]` as clamped cubic Bezier pieces (parameter `t`): a B-spline's own pieces, a line as one
/// piece, an arc as Hermite pieces fine enough for `ARC_TOL`.
fn base_pieces(c: &Curve, t0: f64, t1: f64) -> R<Vec<BSplineCurve>> {
    let piece = |x0: f64, x1: f64| -> R<BSplineCurve> {
        let [pa, da] = c.derivs1_arr(x0);
        let [pb, db] = c.derivs1_arr(x1);
        let h = x1 - x0;
        BSplineCurve::new(3, vec![x0, x0, x0, x0, x1, x1, x1, x1], vec![pa, pa + da * (h / 3.0), pb - db * (h / 3.0), pb]).map_err(|_| Miss::NotMine)
    };
    match c {
        Curve::BSpline(bs) => {
            let cl = bs.clamp_ends();
            let (da, db) = cl.domain();
            let eps = 1e-9 * (db - da).abs().max(1.0);
            if t0 < da - eps || t1 > db + eps || t1 - t0 < 1e-9 {
                dbg(&format!("spline rim: edge range ({t0}, {t1}) outside the curve's domain ({da}, {db})"));
                return nm(line!());
            }
            let (t0, t1) = (t0.max(da), t1.min(db));
            let seg = if t0 - da < eps && db - t1 < eps { cl } else if t0 - da < eps { cl.split(t1).0 } else if db - t1 < eps { cl.split(t0).1 } else { cl.segment(t0, t1) };
            Ok(seg.bezier_segments().into_iter().filter(|q| q.domain().1 - q.domain().0 > 1e-9).collect())
        }
        Curve::Line { .. } => Ok(vec![piece(t0, t1)?]),
        Curve::Circle { r, .. } => {
            let per_rad = (r.abs().max(1.0) / (384.0 * ARC_TOL)).powf(0.25);
            let n = (((t1 - t0).abs() * per_rad).ceil() as usize).clamp(1, 4000);
            (0..n).map(|k| piece(t0 + (t1 - t0) * k as f64 / n as f64, t0 + (t1 - t0) * (k + 1) as f64 / n as f64)).collect()
        }
        _ => nm(line!()),
    }
}

fn rows(c: &Curve, t0: f64, t1: f64, r: f64, dir: Vec3, sig: f64, u_in: Vec3, fans: Vec<Fan>) -> R<Rows> {
    let base = base_pieces(c, t0, t1)?;
    let in_fan = |t: f64| fans.iter().any(|f| t > f.ta && t < f.tb);
    dbg(&format!("spline rim: pieces {} range ({t0}, {t1})", base.len()));
    let mut best: Option<Rows> = None;
    for sub in [1usize, 2, 4, 8, 16, 32, 64] {
        let mut breaks = vec![];
        let mut pm: Vec<[Vec3; 4]> = vec![];
        let mut pt: Vec<[Vec3; 4]> = vec![];
        let mut worst = 0.0f64;
        let mut fmin = f64::INFINITY;
        for piece in &base {
            let (a, bb) = piece.domain();
            let pc = [piece.ctrl[0], piece.ctrl[1], piece.ctrl[2], piece.ctrl[3]];
            for j in 0..sub {
                let (xa, xb) = (a + (bb - a) * j as f64 / sub as f64, a + (bb - a) * (j + 1) as f64 / sub as f64);
                // the stretch, cut where a fan starts or ends
                let mut cuts = vec![xa];
                for f in &fans {
                    for q in [f.ta, f.tb] {
                        if q > xa + 1e-9 && q < xb - 1e-9 {
                            cuts.push(q);
                        }
                    }
                }
                cuts.push(xb);
                cuts.sort_by(|p, q| p.partial_cmp(q).unwrap());
                for w in cuts.windows(2) {
                    let (x0, x1) = (w[0], w[1]);
                    if x1 - x0 < 1e-12 {
                        continue;
                    }
                    let (s0, s1) = ((x0 - a) / (bb - a), (x1 - a) / (bb - a));
                    let cp = if x0 == a && x1 == bb { pc } else { [blossom(&pc, s0, s0, s0), blossom(&pc, s0, s0, s1), blossom(&pc, s0, s1, s1), blossom(&pc, s1, s1, s1)] };
                    if breaks.is_empty() {
                        breaks.push(x0);
                    }
                    breaks.push(x1);
                    let h = x1 - x0;
                    let fan = fans.iter().find(|f| 0.5 * (x0 + x1) > f.ta && 0.5 * (x0 + x1) < f.tb);
                    if let Some(f) = fan {
                        // the cap contact stands still at the meeting point; the section is an elliptic quarter arc to it
                        pm.push(cp);
                        pt.push([f.p; 4]);
                        continue;
                    }
                    // one-sided derivatives from this piece's own control polygon
                    let d0 = (cp[1] - cp[0]) * (3.0 / h);
                    let d1 = (cp[3] - cp[2]) * (3.0 / h);
                    let dd0 = (cp[2] - cp[1] * 2.0 + cp[0]) * (6.0 / (h * h));
                    let dd1 = (cp[3] - cp[2] * 2.0 + cp[1]) * (6.0 / (h * h));
                    let end = |p: Vec3, g: Vec3, g2: Vec3| {
                        let gx = g.cross(dir);
                        let m = gx.len();
                        let n = gx * (sig / m);
                        let dn = (g2.cross(dir) * (1.0 / m) - gx * (g.dot(g2) / (m * m * m))) * sig;
                        (p - n * r, g - dn * r)
                    };
                    let (ta, tda) = end(cp[0], d0, dd0);
                    let (tb, tdb) = end(cp[3], d1, dd1);
                    for (g, td) in [(d0, tda), (d1, tdb)] {
                        fmin = fmin.min(td.dot(g) / g.dot(g));
                    }
                    let tp = [ta, ta + tda * (h / 3.0), tb - tdb * (h / 3.0), tb];
                    for s in [0.25, 0.5, 0.75] {
                        let tt = x0 + h * s;
                        let (p, n, _, _, _) = offset_terms(c, tt, dir, sig);
                        worst = worst.max(bez(&tp, s).dist(p - n * r));
                    }
                    pm.push(cp);
                    pt.push(tp);
                }
            }
        }
        // the curvature factor at the interior samples (outside the trimmed stretches)
        for piece in &base {
            let (a, bb) = piece.domain();
            for k in 0..=32 {
                let tt = a + (bb - a) * k as f64 / 32.0;
                if in_fan(tt) {
                    continue;
                }
                let (_, _, dn, g, _) = offset_terms(c, tt, dir, sig);
                let td = g - dn * r;
                fmin = fmin.min(td.dot(g) / g.dot(g));
            }
        }
        let pw: Vec<[Vec3; 4]> = pm.iter().map(|q| [q[0] + u_in * r, q[1] + u_in * r, q[2] + u_in * r, q[3] + u_in * r]).collect();
        best = Some(Rows { breaks, w: pw, m: pm, t: pt, fmin, fans: fans.clone() });
        if worst <= FIT_TOL {
            break;
        }
    }
    best.ok_or(Miss::NotMine)
}

fn curve_of(breaks: &[f64], pieces: &[[Vec3; 4]]) -> R<BSplineCurve> {
    let s = pieces.len();
    let mut knots = vec![breaks[0]; 4];
    for b in &breaks[1..s] {
        knots.extend([*b, *b, *b]);
    }
    knots.extend([breaks[s]; 4]);
    let mut ctrl = vec![pieces[0][0]];
    for p in pieces {
        ctrl.extend([p[1], p[2], p[3]]);
    }
    BSplineCurve::new(3, knots, ctrl).map_err(|_| Miss::NotMine)
}

fn project(frame: &Frame, c: &BSplineCurve) -> Curve2 {
    Curve2::BSpline(c.map(|p| {
        let q = p - frame.o;
        Vec2::new(q.dot(frame.x), q.dot(frame.y))
    }))
}

// ---------------------------------------------------------------------------------------------------------------- the mitre

/// The strip of one link as a function: `q(t, w)` the section point, with the derivatives the mitre's Newton needs.
pub(super) struct Sec<'a> {
    pub c: &'a Curve,
    pub dir: Vec3,
    pub sig: f64,
    pub u_in: Vec3,
    pub r: f64,
    pub chamfer: bool,
}

impl Sec<'_> {
    /// (section point, d/dt, d/dw)
    fn q(&self, t: f64, w: f64) -> (Vec3, Vec3, Vec3) {
        let (p, n, dn, g, _) = offset_terms(self.c, t, self.dir, self.sig);
        let wv = p + self.u_in * self.r;
        let tv = p - n * self.r;
        let dtv = g - dn * self.r;
        if self.chamfer {
            (wv * (1.0 - w) + tv * w, g * (1.0 - w) + dtv * w, tv - wv)
        } else {
            let c = std::f64::consts::FRAC_1_SQRT_2;
            let (n0, n1, n2) = ((1.0 - w) * (1.0 - w), 2.0 * w * (1.0 - w), w * w);
            let (d0, d1, d2) = (-2.0 * (1.0 - w), 2.0 * (1.0 - 2.0 * w), 2.0 * w);
            let den = n0 + c * n1 + n2;
            let num = wv * n0 + p * (c * n1) + tv * n2;
            let q = num / den;
            let dnum = wv * d0 + p * (c * d1) + tv * d2;
            let dden = d0 + c * d1 + d2;
            let dqw = (dnum - q * dden) / den;
            let dqt = (g * (n0 + c * n1) + dtv * n2) / den;
            (q, dqt, dqw)
        }
    }
    /// The cap contact point and its derivative.
    pub(super) fn top(&self, t: f64) -> (Vec3, Vec3) {
        let (p, n, dn, g, _) = offset_terms(self.c, t, self.dir, self.sig);
        (p - n * self.r, g - dn * self.r)
    }
}

fn solve3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = |m: &[[f64; 3]; 3]| m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    let d = det(&a);
    if d.abs() < 1e-300 || !d.is_finite() {
        return None;
    }
    let mut out = [0.0; 3];
    for k in 0..3 {
        let mut m = a;
        for i in 0..3 {
            m[i][k] = b[i];
        }
        out[k] = det(&m) / d;
    }
    Some(out)
}

/// One end of a link at a ring vertex.
#[derive(Clone, Copy, Debug)]
struct End {
    link: usize,
    /// the vertex is the link's `v1` (parameter `t1`)
    hi: bool,
}

/// A sharp convex corner between two links: where the two offset curves meet, and the intersection of the two strips.
struct Miter {
    ends: [End; 2],
    /// the cap contact point both links' offsets pass through
    p: Vec3,
    /// parameter of each link at that point
    tc: [f64; 2],
    /// points of the intersection edge from the wall vertex (w = 0 on both strips) to the cap vertex (w = 1)
    xs: Vec<Vec3>,
    /// (t, w) of the intersection on each link's strip, same samples
    uv: [Vec<(f64, f64)>; 2],
}

/// Where the two offsets of the cap contact (the centres of the balls rolling on the cap and the two walls) of two links meet: the
/// parameters (t on `a`, s on `b`). `ta0` / `tb0` are the parameters at the corner, `ia` / `ib` the signs that go into each edge.
pub(super) fn offset_meet(a: &Sec, ta0: f64, ia: f64, b: &Sec, tb0: f64, ib: f64, cos_th: f64) -> R<(f64, f64)> {
    let fail = |why: &str| -> R<(f64, f64)> {
        dbg(&format!("spline rim: mitre failed: {why}"));
        refuse(&format!("the corner of this rim cannot be rounded at this size: {why}"), "use a smaller size, or round the vertical corner edge first")
    };
    let tan_half = ((1.0 - cos_th) / (1.0 + cos_th)).max(0.0).sqrt();
    let delta = a.r * tan_half;
    let (_, ga) = a.top(ta0);
    let (_, gb) = b.top(tb0);
    let mut t = ta0 + ia * delta / ga.len().max(1e-12);
    let mut s = tb0 + ib * delta / gb.len().max(1e-12);
    let (_, e1, _, _, _) = offset_terms(a.c, ta0, a.dir, a.sig);
    let e2 = a.dir.cross(e1);
    for _ in 0..60 {
        let (pa, da) = a.top(t);
        let (pb, db) = b.top(s);
        let f = pa - pb;
        let fx = [f.dot(e1), f.dot(e2)];
        if fx[0].abs() + fx[1].abs() < 1e-13 {
            return Ok((t, s));
        }
        let j = [[da.dot(e1), -db.dot(e1)], [da.dot(e2), -db.dot(e2)]];
        let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
        if det.abs() < 1e-14 {
            return fail("the two offset curves are tangent there");
        }
        t -= (fx[0] * j[1][1] - fx[1] * j[0][1]) / det;
        s -= (j[0][0] * fx[1] - j[1][0] * fx[0]) / det;
    }
    fail("the offset curves of the two edges do not meet")
}

fn miter(a: &Sec, ta0: f64, ia: f64, b: &Sec, tb0: f64, ib: f64, cos_th: f64) -> R<([f64; 2], Vec<Vec3>, [Vec<(f64, f64)>; 2])> {
    let fail = |why: &str| -> R<_> {
        dbg(&format!("spline rim: mitre failed: {why}"));
        refuse(&format!("the corner of this rim cannot be rounded at this size: {why}"), "use a smaller size, or round the vertical corner edge first")
    };
    // 1. where the two offsets of the cap contact meet (2D Newton in the cap plane)
    let (t, s) = offset_meet(a, ta0, ia, b, tb0, ib, cos_th)?;
    let p = a.top(t).0;
    let tc = [t, s];
    // 2. the intersection of the strips, from the wall vertex up to the cap contact point
    let n = MITRE_N;
    let mut xs = vec![a.q(ta0, 0.0).0];
    let mut ua = vec![(ta0, 0.0)];
    let mut ub = vec![(tb0, 0.0)];
    let (mut ca, mut cb, mut cw) = (ta0, tb0, 0.0f64);
    for k in 1..n {
        let wa = k as f64 / n as f64;
        let (mut tt, mut ss, mut ww) = (ca, cb, cw.max(1e-3).min(1.0 - 1e-3));
        // a guess along the straight line between the two ends of the intersection
        tt += (tc[0] - ta0) / n as f64;
        ss += (tc[1] - tb0) / n as f64;
        ww = (ww + 1.0 / n as f64).min(1.0);
        let mut conv = false;
        for _ in 0..60 {
            let (qa, dta, _) = a.q(tt, wa);
            let (qb, dsb, dwb) = b.q(ss, ww);
            let f = qa - qb;
            if f.len() < 1e-12 {
                conv = true;
                break;
            }
            let Some(d) = solve3([[dta.x, -dsb.x, -dwb.x], [dta.y, -dsb.y, -dwb.y], [dta.z, -dsb.z, -dwb.z]], [f.x, f.y, f.z]) else {
                return fail("the strips are tangent there");
            };
            // damped step: the parameters stay in range
            let mut lam = 1.0;
            while lam > 1e-3 && (ww - lam * d[2] < -0.05 || ww - lam * d[2] > 1.05) {
                lam *= 0.5;
            }
            tt -= lam * d[0];
            ss -= lam * d[1];
            ww -= lam * d[2];
        }
        if !conv {
            return fail("the two blend faces do not meet in a clean curve there");
        }
        xs.push(a.q(tt, wa).0);
        ua.push((tt, wa));
        ub.push((ss, ww));
        ca = tt;
        cb = ss;
        cw = ww;
    }
    xs.push(p);
    ua.push((tc[0], 1.0));
    ub.push((tc[1], 1.0));
    Ok((tc, xs, [ua, ub]))
}

fn build(b: &Body, picks: &[EdgeId], sizes: &[f64], chamfer: bool, feature: &str, tol: f64) -> R<Body> {
    let mut out = b.clone();
    // rim edges (a ring of the pattern passes through them) and vertical corner edges (a seam of such a ring)
    let mut rim: Vec<usize> = vec![];
    let mut vert: Vec<usize> = vec![];
    for (i, &e) in picks.iter().enumerate() {
        match find_ring(&out, e) {
            Ok(_) => rim.push(i),
            Err(Miss::Refuse(x)) => return Err(Miss::Refuse(x)),
            Err(Miss::NotMine) => {
                if super::spline_vertical::is_vertical(&out, e) {
                    vert.push(i);
                } else {
                    return nm(line!());
                }
            }
        }
    }
    if rim.is_empty() && vert.is_empty() {
        return nm(line!());
    }
    let mut picked: Vec<EdgeId> = rim.iter().map(|&i| picks[i]).collect();
    let mut spare = picks.len() as u32;
    if !vert.is_empty() {
        let list: Vec<(EdgeId, f64, u32)> = vert.iter().map(|&i| (picks[i], sizes[i], i as u32)).collect();
        let made = super::spline_vertical::round(&mut out, &list, chamfer, feature)?;
        picked.extend(made);
    }
    let seeds: Vec<EdgeId> = rim.iter().map(|&i| picks[i]).collect();
    let r = match seeds.first() {
        Some(_) => sizes[rim[0]],
        None => 0.0,
    };
    if rim.iter().any(|&i| (sizes[i] - r).abs() > 1e-12 * (1.0 + r)) {
        return nm(line!());
    }
    let mut left: Vec<EdgeId> = seeds;
    while let Some(&e0) = left.first() {
        let ring = find_ring(&out, e0)?;
        let ids: Vec<EdgeId> = ring.links.iter().map(|l| l.e).collect();
        let numbering: Vec<u32> = ring
            .links
            .iter()
            .map(|l| match picks.iter().position(|&p| p == l.e) {
                Some(i) => i as u32,
                None => {
                    spare += 1;
                    spare - 1
                }
            })
            .collect();
        apply_ring(&mut out, &ring, &numbering, r, chamfer, feature, &picked)?;
        left.retain(|e| !ids.contains(e));
    }
    // the result must be a closed, valid solid
    let issues = ck_check::validate(&out, tol.max(1e-6));
    if !issues.is_empty() {
        let codes: Vec<&str> = issues.iter().map(|i| i.code).collect();
        dbg(&format!("spline rim: result invalid {:?}", issues.iter().map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect::<Vec<_>>()));
        return Err(Miss::Refuse(BlendError::detail("blendFailed", &format!("the rounded rim of the free-form profile did not pass the validity check: {codes:?}"))));
    }
    Ok(out)
}

/// What a vertex of the ring is: tangent-continuous (one normal) or a sharp convex corner.
enum Join {
    Smooth(Vec3),
    Sharp(Miter),
}

fn apply_ring(b: &mut Body, ring: &Ring, numbering: &[u32], r: f64, chamfer: bool, feature: &str, picked: &[EdgeId]) -> R<()> {
    let dir = ring.dir;
    let u_in = ring.u_in;
    let what = if chamfer { "chamfer" } else { "round" };
    let cap_frame = match &b.face(ring.cap).surface {
        Surface::Plane { frame } => *frame,
        _ => return nm(line!()),
    };
    // the wall must be taller than the round (the seam edge keeps a part of its length)
    for (&v, &s) in &ring.seam {
        let sd = b.edge(s);
        let len = sd.t1 - sd.t0;
        if len <= r * (1.0 + 1e-6) {
            return refuse(
                &format!("the {what} {r} does not fit: the wall under this rim is only {len:.3} high (at ({:.3}, {:.3}, {:.3}))", b.vertex(v).p.x, b.vertex(v).p.y, b.vertex(v).p.z),
                "use a smaller size, or make the extrusion taller",
            );
        }
    }
    let nl = ring.links.len();
    // an arc whose radius equals the size: its cap contact collapses to the centre
    let mut collapsed = vec![false; nl];
    for (k, l) in ring.links.iter().enumerate() {
        let ed = b.edge(l.e);
        if let Curve::Circle { frame, r: rho } = &ed.curve {
            let tm = 0.5 * (ed.t0 + ed.t1);
            let (p, n, _, _, _) = offset_terms(&ed.curve, tm, dir, l.sig);
            let s = if n.dot(p - frame.o) >= 0.0 { 1.0 } else { -1.0 };
            collapsed[k] = (rho - s * r).abs() <= 1e-7 * rho.max(1.0);
        }
    }
    // the sharp corners of the ring (the two outward normals differ): the offset is not one curve across them
    let mut sharp: HashSet<VertexId> = HashSet::new();
    {
        let mut nv: HashMap<VertexId, Vec3> = HashMap::new();
        for l in &ring.links {
            let ed = b.edge(l.e);
            for (v, t) in [(ed.v0, ed.t0), (ed.v1, ed.t1)] {
                let (_, n, _, _, _) = offset_terms(&ed.curve, t, dir, l.sig);
                match nv.get(&v) {
                    Some(m) if (*m - n).len() * r >= 3e-7 => {
                        sharp.insert(v);
                    }
                    Some(_) => {}
                    None => {
                        nv.insert(v, n);
                    }
                }
            }
        }
    }
    // A convex bend tighter than the size (the inward offset has a swallowtail) is trimmed where it lies on B-spline links of a
    // ring with no collapsed arc (see `find_fans`); a fan that would reach into a line or an arc, or across a sharp corner, is refused.
    // In every other ring a bend tighter than the size is refused below, as before.
    let fans_on = !collapsed.iter().any(|&c| c);
    let (link_fans, vert_fan) = if fans_on { ring_fans(b, ring, r, chamfer, &sharp)? } else { (vec![vec![]; nl], HashMap::new()) };
    // pieces of every link
    let mut all: Vec<Rows> = vec![];
    for (l, fans) in ring.links.iter().zip(link_fans) {
        let ed = b.edge(l.e);
        all.push(rows(&ed.curve, ed.t0, ed.t1, r, dir, l.sig, u_in, fans)?);
    }
    // the inward offset turns inside out where the rim bends tighter than the size: 1 - r kappa <= 0 (outside the trimmed stretches)
    let fmin = all.iter().zip(&collapsed).filter(|(_, &c)| !c).map(|(x, _)| x.fmin).fold(f64::INFINITY, f64::min);
    if fmin < 0.02 {
        // tightest bend radius met: 1/k = r / (1 - fmin)
        let rho = r / (1.0 - fmin);
        if fans_on {
            // a bend tighter than the size was trimmed; what is left is a size within a hair of a bend's radius, where the
            // offset's speed is nearly zero and its loop too small to trim
            return refuse(
                &format!("the {what} {r} is almost exactly the radius of the tightest bend of the rim (about {rho:.3}), where the inside offset degenerates"),
                &format!("use a size below {:.3} or above {:.3}", rho * 0.97, rho * 1.03),
            );
        }
        return refuse(
            &format!("the {what} {r} does not fit the rim: it bends with a radius of about {rho:.3} on the inside of the round"),
            &format!("use a size below {:.3}, or soften the tightest bend of the profile", (rho * 0.95).max(0.0)),
        );
    }
    // ---- the vertices of the ring: smooth or a sharp convex corner
    let mut inc: Vec<(VertexId, Vec<End>)> = vec![];
    for (k, l) in ring.links.iter().enumerate() {
        let ed = b.edge(l.e);
        for (v, hi) in [(ed.v0, false), (ed.v1, true)] {
            match inc.iter_mut().find(|(x, _)| *x == v) {
                Some((_, list)) => list.push(End { link: k, hi }),
                None => inc.push((v, vec![End { link: k, hi }])),
            }
        }
    }
    let secs: Vec<Sec> = ring.links.iter().map(|l| Sec { c: &b.edge(l.e).curve, dir, sig: l.sig, u_in, r, chamfer }).collect();
    let mut joins: Vec<Join> = vec![];
    let mut lo: Vec<f64> = ring.links.iter().map(|l| b.edge(l.e).t0).collect();
    let mut hi: Vec<f64> = ring.links.iter().map(|l| b.edge(l.e).t1).collect();
    for (v, list) in &inc {
        if list.len() != 2 {
            return nm(line!());
        }
        let at = |e: End| {
            let ed = b.edge(ring.links[e.link].e);
            let t = if e.hi { ed.t1 } else { ed.t0 };
            let (_, n, _, g, _) = offset_terms(&ed.curve, t, dir, ring.links[e.link].sig);
            (t, n, g.normalize() * if e.hi { -1.0 } else { 1.0 })
        };
        let ((ta, na, tga), (tb, nb, tgb)) = (at(list[0]), at(list[1]));
        if (na - nb).len() * r < 3e-7 {
            joins.push(Join::Smooth(na));
            continue;
        }
        // a sharp corner: convex when each edge leaves on the inner side of the other's wall
        if tgb.dot(na) > -1e-9 || tga.dot(nb) > -1e-9 {
            dbg("spline rim: reflex corner");
            return nm(line!());
        }
        if !list.iter().all(|e| picked.contains(&ring.links[e.link].e)) {
            return nm(line!());
        }
        let cos_th = na.dot(nb).clamp(-1.0, 1.0);
        let ia = if list[0].hi { -1.0 } else { 1.0 };
        let ib = if list[1].hi { -1.0 } else { 1.0 };
        let (tc, xs, uv) = miter(&secs[list[0].link], ta, ia, &secs[list[1].link], tb, ib, cos_th)?;
        for (e, t) in list.iter().zip(tc) {
            if e.hi {
                hi[e.link] = hi[e.link].min(t);
            } else {
                lo[e.link] = lo[e.link].max(t);
            }
        }
        let p = xs[xs.len() - 1];
        joins.push(Join::Sharp(Miter { ends: [list[0], list[1]], p, tc, xs, uv }));
        let _ = v;
    }
    for (k, l) in ring.links.iter().enumerate() {
        let ed = b.edge(l.e);
        if hi[k] - lo[k] < 1e-6 * (ed.t1 - ed.t0).abs().max(1e-3) || hi[k] > ed.t1 + 1e-9 || lo[k] < ed.t0 - 1e-9 {
            return refuse(&format!("the {what} {r} does not fit: an edge of this rim is shorter than the mitres at its two ends need"), "use a smaller size");
        }
    }
    // the offset must not run into another part of the rim or into a neighbour on the cap
    let mut rim_pts: Vec<Vec3> = vec![];
    let step = (r / 6.0).max(0.05);
    for l in &ring.links {
        rim_pts.extend(b.sample_edge(l.e, step * step / 8.0));
    }
    let mut t_pts: Vec<Vec3> = vec![];
    for (k, rw) in all.iter().enumerate() {
        for (j, q) in rw.t.iter().enumerate() {
            if rw.breaks[j + 1] <= lo[k] + 1e-9 || rw.breaks[j] >= hi[k] - 1e-9 {
                continue;
            }
            for s in [0.0, 0.5] {
                let par = rw.breaks[j] + s * (rw.breaks[j + 1] - rw.breaks[j]);
                if par >= lo[k] - 1e-9 && par <= hi[k] + 1e-9 {
                    t_pts.push(bez(q, s));
                }
            }
        }
    }
    let limit = r * 0.97;
    // a spatial grid keeps this near-linear
    let cell = r.max(0.05);
    let key = |p: Vec3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64);
    let mut grid: HashMap<(i64, i64, i64), Vec<Vec3>> = HashMap::new();
    for &p in &rim_pts {
        grid.entry(key(p)).or_default().push(p);
    }
    for &tp in &t_pts {
        let (kx, ky, kz) = key(tp);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(v) = grid.get(&(kx + dx, ky + dy, kz + dz)) {
                        if v.iter().any(|p| p.dist(tp) < limit) {
                            dbg(&format!("spline rim: offset point {tp:?} is within {:.4} of the rim (size {r})", v.iter().map(|p| p.dist(tp)).fold(f64::INFINITY, f64::min)));
                            return refuse(
                                &format!("the {what} {r} does not fit: the inside of the round runs into another part of the same rim (a neck narrower than twice the size)"),
                                "use a smaller size",
                            );
                        }
                    }
                }
            }
        }
    }
    // the cap contact curves of two far apart parts of the ring must not touch (the cap face between them would have no width: a
    // neck of exactly twice the size)
    {
        let mut ts: Vec<(Vec3, f64)> = vec![];
        let mut acc = 0.0;
        for (k, rw) in all.iter().enumerate() {
            // the samples of this link in the order the cap's loop runs along it
            let mut mine: Vec<Vec3> = vec![];
            for (j, q) in rw.t.iter().enumerate() {
                // fine enough that two curves closer than the width are caught: a sample every half size or so
                let chord = q[0].dist(q[1]) + q[1].dist(q[2]) + q[2].dist(q[3]);
                let m = ((chord / (0.25 * r)).ceil() as usize).clamp(2, 200);
                for i in 0..m {
                    let s = i as f64 / m as f64;
                    let par = rw.breaks[j] + s * (rw.breaks[j + 1] - rw.breaks[j]);
                    if par >= lo[k] - 1e-9 && par <= hi[k] + 1e-9 {
                        mine.push(bez(q, s));
                    }
                }
            }
            if b.coedge(ring.links[k].c_cap).rev {
                mine.reverse();
            }
            for p in mine {
                if let Some((pp, _)) = ts.last() {
                    acc += pp.dist(p);
                }
                ts.push((p, acc));
            }
        }
        let total = acc;
        let width = 0.05 * r + 1e-4;
        let cellw = (2.0 * r).max(0.05);
        let keyw = |p: Vec3| ((p.x / cellw).floor() as i64, (p.y / cellw).floor() as i64, (p.z / cellw).floor() as i64);
        let mut gridw: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
        for (i, (p, _)) in ts.iter().enumerate() {
            gridw.entry(keyw(*p)).or_default().push(i);
        }
        for (i, (p, si)) in ts.iter().enumerate() {
            let (kx, ky, kz) = keyw(*p);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        for &j in gridw.get(&(kx + dx, ky + dy, kz + dz)).into_iter().flatten() {
                            let (q, sj) = ts[j];
                            let gap = (si - sj).abs();
                            // far apart along the curve, both ways round the ring
                            if gap > 3.0 * r && total - gap > 3.0 * r && p.dist(q) < width && j != i {
                                return refuse(
                                    &format!("the {what} {r} does not fit: two parts of this rim are less than twice the size apart, the cap face between them would be reduced to nothing"),
                                    "use a smaller size",
                                );
                            }
                        }
                    }
                }
            }
        }
    }
    // other loops of the cap must stay clear of the round's cap contact
    for &lp in &b.face(ring.cap).loops {
        if b.lp(lp).coedges.iter().any(|&c| ring.links.iter().any(|l| l.e == b.coedge(c).edge)) {
            continue;
        }
        for &c in &b.lp(lp).coedges {
            for p in b.sample_edge(b.coedge(c).edge, 1e-3) {
                if t_pts.iter().any(|q| q.dist(p) < 1.0e-3 + 0.0) || rim_pts.iter().any(|q| q.dist(p) < r * 1.0) {
                    return refuse("the round reaches another edge of the same face", "use a smaller size, or round the other edges first");
                }
            }
        }
    }
    // ---- vertices: the old one becomes the wall vertex; a new cap vertex and the edge between them
    // (arcs that collapse put the cap vertices of their two ends together)
    let verts: Vec<VertexId> = inc.iter().map(|(v, _)| *v).collect();
    let mut leader: HashMap<VertexId, VertexId> = verts.iter().map(|&v| (v, v)).collect();
    fn root(m: &HashMap<VertexId, VertexId>, mut v: VertexId) -> VertexId {
        while m[&v] != v {
            v = m[&v];
        }
        v
    }
    for (k, l) in ring.links.iter().enumerate() {
        if collapsed[k] {
            let (a, c) = (root(&leader, b.edge(l.e).v0), root(&leader, b.edge(l.e).v1));
            if a != c {
                leader.insert(c, a);
            }
        }
    }
    let mut cap_of_leader: HashMap<VertexId, VertexId> = HashMap::new();
    let mut cap_v: HashMap<VertexId, VertexId> = HashMap::new();
    let mut join_e: HashMap<VertexId, EdgeId> = HashMap::new();
    for ((v, _), j) in inc.iter().zip(&joins) {
        let p = b.vertex(*v).p;
        let wp = p + u_in * r;
        let tp = match j {
            Join::Smooth(n) => vert_fan.get(v).copied().unwrap_or(p - *n * r),
            Join::Sharp(m) => m.p,
        };
        let ld = root(&leader, *v);
        let tv = match cap_of_leader.get(&ld) {
            Some(&t) => t,
            None => {
                let t = b.add_vertex(tp);
                cap_of_leader.insert(ld, t);
                t
            }
        };
        b.vertices[v.idx()].p = wp;
        cap_v.insert(*v, tv);
        let ae = match j {
            Join::Sharp(m) => {
                let curve = interpolate(&m.xs, 3, Param::Uniform).map_err(|_| Miss::NotMine)?;
                b.add_edge(Curve::BSpline(curve), 0.0, 1.0, *v, tv)
            }
            Join::Smooth(n) => {
                if chamfer {
                    let dv = tp - wp;
                    b.add_edge(Curve::Line { o: wp, d: dv.normalize() }, 0.0, dv.len(), *v, tv)
                } else {
                    // the quarter circle as the rational quadratic of the blend face's own section (parameter w in [0, 1]), so that the
                    // edge and the face's boundary line u = const agree to the last bit
                    let _ = n;
                    let q = BSplineCurve::new_rational(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![wp, p, tp], vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0]).map_err(|_| Miss::NotMine)?;
                    b.add_edge(Curve::BSpline(q), 0.0, 1.0, *v, tv)
                }
            }
        };
        join_e.insert(*v, ae);
        // the seam edge loses the first r of its length at this end
        let s = ring.seam[v];
        if b.edge(s).v1 == *v {
            b.edges[s.idx()].t1 -= r;
        } else {
            b.edges[s.idx()].t0 += r;
        }
    }
    // ---- links
    let sh = cap_frame;
    for (k, (l, rw)) in ring.links.iter().zip(&all).enumerate() {
        let ed = b.edge(l.e).clone();
        let (t0, t1) = (ed.t0, ed.t1);
        let wc = curve_of(&rw.breaks, &rw.w)?;
        let mc = curve_of(&rw.breaks, &rw.m)?;
        let tc = curve_of(&rw.breaks, &rw.t)?;
        // the surface: u = t, v = 0 at the first row
        let nu = mc.ctrl.len();
        let (rows3, wts): (Vec<&BSplineCurve>, Vec<f64>) = if chamfer { (vec![&wc, &tc], vec![]) } else { (vec![&wc, &mc, &tc], vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0]) };
        let build_surface = |flip: bool| -> R<BSplineSurface> {
            let mut order: Vec<usize> = (0..rows3.len()).collect();
            if flip {
                order.reverse();
            }
            let nv = rows3.len();
            let mut ctrl = Vec::with_capacity(nu * nv);
            let mut weights = Vec::with_capacity(nu * nv);
            for i in 0..nu {
                for &j in &order {
                    ctrl.push(rows3[j].ctrl[i]);
                    weights.push(if chamfer { 1.0 } else { wts[j] });
                }
            }
            let (deg_v, kv) = if chamfer { (1, vec![0.0, 0.0, 1.0, 1.0]) } else { (2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]) };
            BSplineSurface::new(3, deg_v, mc.knots.clone(), kv, nu, nv, ctrl, if chamfer { None } else { Some(weights) }).map_err(|_| Miss::NotMine)
        };
        // natural normal against the outward one at the middle of the face
        let tm = 0.5 * (lo[k] + hi[k]);
        let (_, n_mid, _, _, _) = offset_terms(&ed.curve, tm, dir, l.sig);
        let want = (n_mid + ring.ncap).normalize();
        let mut surf = build_surface(false)?;
        let mut flipped = false;
        let nat = Surface::BSpline(Box::new(surf.clone())).normal(tm, 0.5);
        if nat.dot(want) < 0.0 {
            surf = build_surface(true)?;
            flipped = true;
        }
        // the old edge becomes the wall contact, the new one the cap contact (exact where the rim is a line or an arc)
        let cap_curve: Curve;
        {
            let e = &mut b.edges[l.e.idx()];
            match &ed.curve {
                Curve::Line { o, d } => {
                    e.curve = Curve::Line { o: *o + u_in * r, d: *d };
                    cap_curve = Curve::Line { o: *o - n_mid * r, d: *d };
                }
                Curve::Circle { frame, r: rho } => {
                    let (p, n, _, _, _) = offset_terms(&ed.curve, tm, dir, l.sig);
                    let s = if n.dot(p - frame.o) >= 0.0 { 1.0 } else { -1.0 };
                    e.curve = Curve::Circle { frame: Frame { o: frame.o + u_in * r, ..*frame }, r: *rho };
                    cap_curve = Curve::Circle { frame: *frame, r: (rho - s * r).max(0.0) };
                }
                _ => {
                    e.curve = Curve::BSpline(wc.clone());
                    cap_curve = Curve::BSpline(tc.clone());
                }
            }
        }
        // the wall coedge: the line at the new v
        if let Some(Curve2::Line { o, d }) = &b.coedge(l.c_wall).pcurve.clone() {
            let sh2 = Vec2::new(l.ax[0].dot(u_in) * r, l.ax[1].dot(u_in) * r);
            b.coedges[l.c_wall.idx()].pcurve = Some(Curve2::Line { o: *o + sh2, d: *d });
        }
        // the cap contact edge
        let (v0, v1) = (ed.v0, ed.v1);
        let ce: Option<EdgeId> = if collapsed[k] {
            // no edge: the coedge leaves the cap's loop
            let c = l.c_cap;
            let old = b.coedge(c).edge;
            b.edges[old.idx()].coedges.retain(|&x| x != c);
            let lpid = b.coedge(c).lp;
            b.loops[lpid.idx()].coedges.retain(|&x| x != c);
            b.coedges[c.idx()].alive = false;
            None
        } else {
            let ce = b.add_edge(cap_curve.clone(), lo[k], hi[k], cap_v[&v0], cap_v[&v1]);
            let old = b.coedge(l.c_cap).edge;
            b.edges[old.idx()].coedges.retain(|&c| c != l.c_cap);
            b.coedges[l.c_cap.idx()].edge = ce;
            b.coedges[l.c_cap.idx()].pcurve = match &cap_curve {
                Curve::BSpline(c) => Some(project(&sh, c)),
                other => ck_topo::make::plane_pcurve(&sh, other),
            };
            b.edges[ce.idx()].coedges.push(l.c_cap);
            Some(ce)
        };
        // the blend face
        let shell = b.face(ring.cap).shell;
        let fid = b.add_face(shell, Surface::BSpline(Box::new(surf)), true, FaceName::blend(feature, numbering[k]));
        let lp = b.add_loop(fid);
        let line_u = |w: f64| Curve2::Line { o: Vec2::new(0.0, w), d: Vec2::new(1.0, 0.0) };
        // bottom (w = 0) forward, right up, top (w = 1) backward, left down
        let (bottom_e, top_e) = if flipped { (ce, Some(l.e)) } else { (Some(l.e), ce) };
        let (vb, vt) = if flipped { (1.0, 0.0) } else { (0.0, 1.0) };
        let _ = (vb, vt);
        if let Some(e) = bottom_e {
            b.add_coedge(lp, e, false, Some(line_u(0.0)));
        }
        // the edge at one end of the face: the shared arc or the mitre
        let end_pc = |b: &Body, hi_end: bool| -> R<Curve2> {
            let v = if hi_end { v1 } else { v0 };
            let ix = inc.iter().position(|(x, _)| *x == v).unwrap();
            match &joins[ix] {
                Join::Smooth(_) => {
                    let u = if hi_end { t1 } else { t0 };
                    let span = b.edge(join_e[&v]).t1;
                    Ok(if flipped { Curve2::Line { o: Vec2::new(u, 1.0), d: Vec2::new(0.0, -1.0 / span) } } else { Curve2::Line { o: Vec2::new(u, 0.0), d: Vec2::new(0.0, 1.0 / span) } })
                }
                Join::Sharp(m) => {
                    // which of the corner's two ends is this link's (a link meeting itself takes the matching end)
                    let side = if m.ends[0].link == k && m.ends[0].hi == hi_end { 0 } else { 1 };
                    let pts: Vec<Vec2> = m.uv[side].iter().map(|&(t, w)| Vec2::new(t, if flipped { 1.0 - w } else { w })).collect();
                    interpolate(&pts, 3, Param::Uniform).map(Curve2::BSpline).map_err(|_| Miss::NotMine)
                }
            }
        };
        let (pc1, pc0) = (end_pc(b, true)?, end_pc(b, false)?);
        b.add_coedge(lp, join_e[&v1], flipped, Some(pc1));
        if let Some(e) = top_e {
            b.add_coedge(lp, e, true, Some(line_u(1.0)));
        }
        b.add_coedge(lp, join_e[&v0], !flipped, Some(pc0));
    }
    Ok(())
}
