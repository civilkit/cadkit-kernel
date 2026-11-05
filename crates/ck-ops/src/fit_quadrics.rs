//! Refit facet fans as analytic faces: a ring (or arc) of planar strips that all run parallel to one axis is a faceted
//! cylinder; strips whose edges all meet in one apex are a faceted cone. Each such group is replaced by ONE cylinder or cone
//! face, its end edges by circles, so a mesh import (SketchUp, STL-like polygon soups) gets the exact faces a CAD model has:
//! holes can be drilled to their axis, fillets and chamfers find circular edges, STEP export writes cylinders.
//!
//! What counts as a fan (all of it is checked, a group failing any step is left as facets and reported in
//! [`FitReport::declined`]):
//! - strips are planar faces with one outer loop, joined along rails (edges shared by two strips) that are parallel
//!   (cylinder) or pass through one apex (cone) within `tol_fit`;
//! - the cross-section of the rails lies on a circle within `tol_fit`, in equal angular steps no larger than
//!   `max_step_deg` (a hexagonal or square prism is a prism, not a cylinder) and at least `min_strips` strips long
//!   (an arc may stop at a flat face: a D-cut shaft keeps its flat);
//! - the chord between neighbouring rails deviates from the circle by at most `tol` (the sag of the facets: the fit
//!   changes the volume by about that much, no more);
//! - every other edge of a strip is an end chord at a constant height, and the face across it is a plane normal to the
//!   axis (the circle then lies in it) or a strip of another accepted group on the same circle;
//! - coaxial groups that meet at a crease smaller than `smooth_deg` form a tessellated sphere, torus or other curved
//!   profile, not cones: they are all left alone.
//!
//! Not recognised: caps that cut the axis obliquely (their edge would be an ellipse), strips with holes, fans split by
//! extra vertices, spheres and tori. The module only edits the body it is handed; the caller (`fitQuadrics` in the
//! server) validates the result and keeps the original when it is worse.

use ck_geom::{Frame, Vec2, Vec3};
use ck_json::{Json, Obj};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, LoopId};
use ck_topo::edit::{gc, merge_coplanar};
use ck_topo::make::plane_pcurve;
use std::collections::{HashMap, HashSet};
use std::f64::consts::{PI, TAU};

#[derive(Debug, Clone)]
pub struct FitOpts {
    /// Largest deviation (mm) between a facet and the fitted surface: the chord sag of the strips.
    pub tol: f64,
    /// Largest deviation (mm) of a facet corner from the fitted surface (the fit residual).
    pub tol_fit: f64,
    /// Largest angular step (degrees) between neighbouring rails.
    pub max_step_deg: f64,
    /// Fewest strips of a group.
    pub min_strips: usize,
    /// Groups of coaxial strips meeting at a crease below this angle (degrees) are a curved profile, not cones.
    pub smooth_deg: f64,
    /// Recognise cones too.
    pub cones: bool,
    /// Merge coplanar neighbours first (bodies of up to `merge_max` faces; triangulated strips become quads).
    pub merge: bool,
    pub merge_max: usize,
}

impl FitOpts {
    /// Defaults for a body of bounding-box diagonal `diag` (mm): `tol` 0.5 % of it, `tol_fit` a tenth of that.
    pub fn for_size(diag: f64) -> FitOpts {
        let tol = (0.005 * diag).max(1e-6);
        FitOpts { tol, tol_fit: tol * 0.1, max_step_deg: 30.0, min_strips: 3, smooth_deg: 20.0, cones: true, merge: true, merge_max: 3000 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Cylinder,
    Cone,
}

/// One accepted group.
#[derive(Debug, Clone)]
pub struct FitGroup {
    pub kind: Kind,
    /// The face that replaced the strips.
    pub face: FaceId,
    pub strips: usize,
    /// A closed ring (a full cylinder or cone) or an arc.
    pub full: bool,
    pub axis: Vec3,
    /// A point on the axis (the apex of a cone).
    pub origin: Vec3,
    /// Cylinder radius; cone: radius at `origin` is 0, see `half_angle`.
    pub radius: f64,
    /// Cone half angle in radians (0 for a cylinder).
    pub half_angle: f64,
    /// Largest distance of a corner from the fitted surface.
    pub max_residual: f64,
    /// Chord sag of the facets (the most the fit can move the surface away from them).
    pub sag: f64,
}

#[derive(Debug, Clone)]
pub struct Declined {
    /// A face of the group (its first strip).
    pub face: FaceId,
    pub strips: usize,
    pub reason: &'static str,
}

#[derive(Debug, Clone, Default)]
pub struct FitReport {
    pub groups: Vec<FitGroup>,
    pub declined: Vec<Declined>,
    pub merged: usize,
    pub merge_skipped: bool,
    pub faces_before: usize,
    pub faces_after: usize,
}

// ------------------------------------------------------------------ small numerics

fn perp_basis(a: Vec3) -> (Vec3, Vec3) {
    let x = a.any_perp().normalize();
    (x, a.cross(x))
}

/// Circle through points by algebraic least squares (centred for stability): (centre, radius).
fn circle_fit(pts: &[[f64; 2]]) -> Option<([f64; 2], f64)> {
    let n = pts.len();
    if n < 3 {
        return None;
    }
    let (mx, my) = (pts.iter().map(|p| p[0]).sum::<f64>() / n as f64, pts.iter().map(|p| p[1]).sum::<f64>() / n as f64);
    // x^2 + y^2 + D x + E y + F = 0 on centred points: normal equations of [x y 1] [D E F]^T = -(x^2+y^2)
    let mut a = [[0.0f64; 3]; 3];
    let mut b = [0.0f64; 3];
    for p in pts {
        let (x, y) = (p[0] - mx, p[1] - my);
        let row = [x, y, 1.0];
        let rhs = -(x * x + y * y);
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] += row[i] * row[j];
            }
            b[i] += row[i] * rhs;
        }
    }
    let s = solve3(a, b)?;
    let (cx, cy) = (-s[0] / 2.0, -s[1] / 2.0);
    let r2 = cx * cx + cy * cy - s[2];
    if !(r2 > 0.0) || !r2.is_finite() {
        return None;
    }
    Some(([cx + mx, cy + my], r2.sqrt()))
}

fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for c in 0..3 {
        let p = (c..3).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-14 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..3 {
            let f = a[r][c] / a[c][c];
            for k in c..3 {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = [0.0; 3];
    for c in (0..3).rev() {
        let mut s = b[c];
        for k in c + 1..3 {
            s -= a[c][k] * x[k];
        }
        x[c] = s / a[c][c];
    }
    Some(x)
}

fn wrap_pi(a: f64) -> f64 {
    let mut x = a % TAU;
    if x > PI {
        x -= TAU;
    } else if x <= -PI {
        x += TAU;
    }
    x
}

/// A circle on the cross-section points `w[idx]`: centre, radius, the angular steps between the points, and whether they are
/// within `tol_fit` of the circle and equal in size.
struct Window {
    c: [f64; 2],
    r: f64,
    step: f64,
}

fn window(w: &[[f64; 2]], idx: &[usize], tol_fit: f64, max_step: f64) -> Option<Window> {
    let pts: Vec<[f64; 2]> = idx.iter().map(|&i| w[i]).collect();
    let (c, r) = circle_fit(&pts)?;
    let resid = pts.iter().map(|p| (((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2)).sqrt() - r).abs()).fold(0.0, f64::max);
    if resid > tol_fit {
        return None;
    }
    let ang: Vec<f64> = pts.iter().map(|p| (p[1] - c[1]).atan2(p[0] - c[0])).collect();
    let steps: Vec<f64> = ang.windows(2).map(|a| wrap_pi(a[1] - a[0])).collect();
    let dir = steps[0].signum();
    if steps.iter().any(|s| s.signum() != dir || s.abs() < 1e-9) {
        return None;
    }
    let (lo, hi) = steps.iter().fold((f64::INFINITY, 0.0f64), |(l, h), s| (l.min(s.abs()), h.max(s.abs())));
    if hi > max_step || hi - lo > 0.25 * hi {
        return None;
    }
    Some(Window { c, r, step: hi })
}

struct Arc {
    /// First segment and number of segments (segment i joins point i and i + 1, modulo the point count for a ring).
    start: usize,
    nseg: usize,
    full: bool,
    c: [f64; 2],
    r: f64,
    step: f64,
}

/// Maximal runs of equally stepped, concyclic points. `ring`: the points close up (segment n - 1 joins the last to the first).
fn find_arcs(w: &[[f64; 2]], ring: bool, min_strips: usize, tol_fit: f64, max_step: f64) -> Vec<Arc> {
    let n = w.len();
    let nseg_total = if ring { n } else { n - 1 };
    let pts_of = |s: usize, nseg: usize| -> Vec<usize> { (0..=nseg).map(|k| (s + k) % n).collect() };
    if ring && n >= min_strips {
        let idx: Vec<usize> = (0..n).collect();
        // the whole ring: the points in order, closed by the first one
        let mut closed = idx.clone();
        closed.push(0);
        if let Some(win) = window(w, &closed, tol_fit, max_step) {
            // equal steps that add up to exactly one turn
            let total: f64 = closed.windows(2).map(|p| {
                let (a, b) = (w[p[0]], w[p[1]]);
                wrap_pi((b[1] - win.c[1]).atan2(b[0] - win.c[0]) - (a[1] - win.c[1]).atan2(a[0] - win.c[0])).abs()
            }).sum();
            if (total - TAU).abs() < 1e-3 {
                return vec![Arc { start: 0, nseg: n, full: true, c: win.c, r: win.r, step: win.step }];
            }
        }
    }
    let ok = |s: usize, nseg: usize| nseg >= 1 && nseg <= nseg_total && window(w, &pts_of(s, nseg), tol_fit, max_step).is_some();
    let mut out: Vec<Arc> = vec![];
    let mut covered = vec![false; nseg_total];
    for s in 0..nseg_total {
        if covered[s] || (!ring && s + min_strips > nseg_total) {
            continue;
        }
        // a ring run that the segment before it could extend starts earlier: it is found from there
        if ring && s > 0 && ok((s + nseg_total - 1) % nseg_total, min_strips + 1) {
            continue;
        }
        if !ok(s, min_strips) {
            continue;
        }
        let mut nseg = min_strips;
        while (ring || s + nseg < nseg_total) && nseg < nseg_total && ok(s, nseg + 1) {
            nseg += 1;
        }
        if (0..nseg).any(|k| covered[(s + k) % nseg_total]) {
            continue;
        }
        let win = window(w, &pts_of(s, nseg), tol_fit, max_step).expect("checked");
        (0..nseg).for_each(|k| covered[(s + k) % nseg_total] = true);
        out.push(Arc { start: s, nseg, full: false, c: win.c, r: win.r, step: win.step });
    }
    out
}

// ------------------------------------------------------------------ chains of strips

#[derive(Clone, Copy)]
enum Axis {
    /// Rails parallel to the unit direction.
    Parallel(Vec3),
    /// Rails through the apex.
    Apex(Vec3),
}

struct Chain {
    axis: Axis,
    /// Strips in order; `rails[i]` is the rail before strip i; an open chain has one more rail than strips, a ring as many.
    faces: Vec<FaceId>,
    rails: Vec<EdgeId>,
    ring: bool,
}

struct Mesh<'a> {
    b: &'a Body,
    tol_fit: f64,
}

impl Mesh<'_> {
    fn ends(&self, e: EdgeId) -> (Vec3, Vec3) {
        let ed = self.b.edge(e);
        (self.b.vertex(ed.v0).p, self.b.vertex(ed.v1).p)
    }

    /// A strip candidate: planar, one loop, plain line edges.
    fn simple(&self, f: FaceId) -> bool {
        let face = self.b.face(f);
        if !face.alive || !matches!(face.surface, Surface::Plane { .. }) || face.loops.len() != 1 {
            return false;
        }
        let lp = self.b.lp(face.loops[0]);
        lp.coedges.len() >= 3 && lp.coedges.len() <= 8 && lp.coedges.iter().all(|&c| matches!(self.b.edge(self.b.coedge(c).edge).curve, Curve::Line { .. }))
    }

    fn other_face(&self, e: EdgeId, f: FaceId) -> Option<FaceId> {
        let fs = self.b.edge_faces(e);
        let cs = self.b.edge_coedges(e);
        if fs.len() != 2 || cs.len() != 2 || !fs.contains(&f) {
            return None;
        }
        Some(if fs[0] == f { fs[1] } else { fs[0] })
    }

    /// Edges of `f` that are folds towards another strip candidate.
    fn rails_of(&self, f: FaceId) -> Vec<(EdgeId, FaceId)> {
        let nf = self.b.planar_normal(f);
        self.b.face_edges(f).into_iter().filter_map(|e| {
            let g = self.other_face(e, f)?;
            (self.simple(g) && self.b.planar_normal(g).cross(nf).len() > 1e-6).then_some((e, g))
        }).collect()
    }

    /// Whether the rail edge fits the axis (parallel within the corner tolerance / through the apex).
    fn fits(&self, e: EdgeId, axis: Axis) -> bool {
        let (p, q) = self.ends(e);
        let len = p.dist(q);
        if len < 1e-12 {
            return false;
        }
        let d = (q - p) / len;
        match axis {
            Axis::Parallel(a) => d.cross(a).len() * len <= 2.0 * self.tol_fit,
            Axis::Apex(apex) => d.cross(apex - p).len() <= 2.0 * self.tol_fit,
        }
    }

    /// The axis two rails of one face define: parallel directions, or the point where their lines meet.
    fn axis_of(&self, e1: EdgeId, e2: EdgeId) -> Option<Axis> {
        let ((p1, q1), (p2, q2)) = (self.ends(e1), self.ends(e2));
        let (l1, l2) = (p1.dist(q1), p2.dist(q2));
        if l1 < 1e-12 || l2 < 1e-12 {
            return None;
        }
        let (d1, d2) = ((q1 - p1) / l1, (q2 - p2) / l2);
        let c = d1.cross(d2);
        if c.len() * l1.min(l2) <= 2.0 * self.tol_fit {
            return Some(Axis::Parallel(d1));
        }
        // closest points of the two lines (they are coplanar: the face)
        let w = p1 - p2;
        let (a, b, cc, dd, ee) = (d1.dot(d1), d1.dot(d2), d2.dot(d2), d1.dot(w), d2.dot(w));
        let den = a * cc - b * b;
        let s = (b * ee - cc * dd) / den;
        let t = (a * ee - b * dd) / den;
        let apex = (p1 + d1 * s + p2 + d2 * t) * 0.5;
        (apex.len().is_finite() && (p1 + d1 * s).dist(p2 + d2 * t) <= 2.0 * self.tol_fit).then_some(Axis::Apex(apex))
    }

    /// Grow a chain from `f` between its rails `eL` and `eR`.
    fn grow(&self, f: FaceId, e_l: EdgeId, e_r: EdgeId, axis: Axis, claimed: &HashSet<FaceId>) -> Chain {
        let mut faces = vec![f];
        let mut rails = vec![e_l, e_r];
        let step = |cur: FaceId, via: EdgeId, claimed: &HashSet<FaceId>| -> Option<(FaceId, EdgeId)> {
            let g = self.other_face(via, cur)?;
            if !self.simple(g) || claimed.contains(&g) {
                return None;
            }
            let next: Vec<EdgeId> = self.rails_of(g).into_iter().map(|x| x.0).filter(|&e| e != via && self.fits(e, axis)).collect();
            (next.len() == 1).then(|| (g, next[0]))
        };
        let mut ring = false;
        // to the right
        let (mut cur, mut via) = (f, e_r);
        loop {
            match step(cur, via, claimed) {
                Some((g, _)) if g == f => {
                    // back at the start through its left rail: the chain is a ring
                    ring = via == e_l;
                    break;
                }
                Some((g, next)) => {
                    if faces.contains(&g) {
                        break;
                    }
                    faces.push(g);
                    rails.push(next);
                    cur = g;
                    via = next;
                }
                None => break,
            }
            if faces.len() > 100_000 {
                break;
            }
        }
        if ring {
            // as many rails as strips: the last rail is the first again
            rails.pop();
            return Chain { axis, faces, rails, ring: true };
        }
        // to the left
        let (mut cur, mut via) = (f, e_l);
        while let Some((g, next)) = step(cur, via, claimed) {
            if faces.contains(&g) {
                break;
            }
            faces.insert(0, g);
            rails.insert(0, next);
            cur = g;
            via = next;
        }
        Chain { axis, faces, rails, ring: false }
    }
}

// ------------------------------------------------------------------ candidate groups

struct Run {
    kind: Kind,
    faces: Vec<FaceId>,
    /// Rails around the strips (see [`Chain`]): `rails[i]` before strip i, `rails[n]` after the last of an open run.
    rails: Vec<EdgeId>,
    full: bool,
    axis: Vec3,
    /// A point on the axis (the apex for a cone).
    origin: Vec3,
    radius: f64,
    tan_half: f64,
    max_residual: f64,
    sag: f64,
    /// Outward normal = natural normal of the fitted surface.
    sense: bool,
    /// The end chords of the strips (edges that are not rails), with their height along the axis.
    chords: Vec<(EdgeId, f64)>,
}

impl Run {
    fn height(&self, p: Vec3) -> f64 {
        (p - self.origin).dot(self.axis)
    }
    /// Radius of the surface at axial height `h` (from `origin`).
    fn radius_at(&self, h: f64) -> f64 {
        match self.kind {
            Kind::Cylinder => self.radius,
            Kind::Cone => h * self.tan_half,
        }
    }
    fn radial(&self, p: Vec3) -> Vec3 {
        let d = p - self.origin;
        d - self.axis * d.dot(self.axis)
    }
    /// Outward normal of the fitted surface at a point (sense applied).
    fn normal_at(&self, p: Vec3) -> Vec3 {
        let r = self.radial(p).normalize();
        let n = match self.kind {
            Kind::Cylinder => r,
            Kind::Cone => (r - self.axis * self.tan_half).normalize(),
        };
        if self.sense { n } else { -n }
    }
}

/// The runs inside one chain.
fn runs_of(m: &Mesh, ch: &Chain, o: &FitOpts, declined: &mut Vec<Declined>) -> Vec<Run> {
    let b = m.b;
    let nr = ch.rails.len();
    let ns = ch.faces.len();
    // a point on each rail, and the cross-section of the rails
    let rail_mid: Vec<Vec3> = ch.rails.iter().map(|&e| { let (p, q) = m.ends(e); (p + q) * 0.5 }).collect();
    let (kind, axis, apex) = match ch.axis {
        Axis::Parallel(a) => (Kind::Cylinder, a, Vec3::ZERO),
        Axis::Apex(p) => {
            if !o.cones {
                return vec![];
            }
            // unit directions from the apex: they lie on a circle of the unit sphere, whose plane normal is the axis
            let u: Vec<Vec3> = rail_mid.iter().map(|&q| (q - p).normalize()).collect();
            if u.iter().any(|v| v.len() < 0.5) {
                return vec![];
            }
            let mean = u.iter().fold(Vec3::ZERO, |s, &v| s + v) * (1.0 / u.len() as f64);
            let mut cov = [[0.0; 3]; 3];
            for v in &u {
                let d = *v - mean;
                let dd = [d.x, d.y, d.z];
                for i in 0..3 {
                    for j in 0..3 {
                        cov[i][j] += dd[i] * dd[j];
                    }
                }
            }
            let (_, vecs) = ck_topo::mass::eigen_sym(cov);
            let mut a = vecs[0];
            if a.dot(mean) < 0.0 {
                a = -a;
            }
            (Kind::Cone, a, p)
        }
    };
    let (bx, by) = perp_basis(axis);
    // 2D cross-section of the rails: the rail's projection (cylinder), or its gnomonic point at unit height (cone)
    let w: Vec<[f64; 2]> = rail_mid.iter().map(|&q| match kind {
        Kind::Cylinder => [q.dot(bx), q.dot(by)],
        Kind::Cone => {
            let d = (q - apex).normalize();
            let k = d.dot(axis);
            if k.abs() < 1e-9 { [f64::NAN, f64::NAN] } else { [d.dot(bx) / k, d.dot(by) / k] }
        }
    }).collect();
    if w.iter().any(|p| !p[0].is_finite()) {
        return vec![];
    }
    // the angular step of the 3D cone is the step of the gnomonic circle only roughly; the limit is checked on the 2D circle (exact for
    // a cylinder; a cone's rails are not equally spaced in the gnomonic plane unless the circle is centred on the axis, which it is)
    let tol2 = match kind {
        Kind::Cylinder => o.tol_fit,
        Kind::Cone => {
            let mean_h = rail_mid.iter().map(|&q| (q - apex).dot(axis)).sum::<f64>() / nr as f64;
            (o.tol_fit / mean_h.abs().max(1e-9)).max(1e-12)
        }
    };
    let arcs = find_arcs(&w, ch.ring, o.min_strips, tol2, o.max_step_deg.to_radians());
    let mut out = vec![];
    for arc in arcs {
        let seg_face = |k: usize| ch.faces[(arc.start + k) % ns];
        let faces: Vec<FaceId> = (0..arc.nseg).map(seg_face).collect();
        let rails: Vec<EdgeId> = if arc.full { (0..arc.nseg).map(|k| ch.rails[(arc.start + k) % nr]).collect() } else { (0..=arc.nseg).map(|k| ch.rails[(arc.start + k) % nr]).collect() };
        let (first_face, nstrips) = (faces[0], faces.len());
        let decline = |declined: &mut Vec<Declined>, reason: &'static str| declined.push(Declined { face: first_face, strips: nstrips, reason });
        // the fitted surface
        let (origin, radius, tan_half) = match kind {
            Kind::Cylinder => {
                let c3 = bx * arc.c[0] + by * arc.c[1];
                (c3, arc.r, 0.0)
            }
            Kind::Cone => {
                // the circle is centred on the axis in the gnomonic plane: its radius is tan(half angle)
                (apex, 0.0, arc.r)
            }
        };
        let mut run = Run { kind, faces, rails, full: arc.full, axis, origin, radius, tan_half, max_residual: 0.0, sag: 0.0, sense: true, chords: vec![] };
        if kind == Kind::Cone {
            if !(arc.r > 1e-6) || arc.c[0].hypot(arc.c[1]) > 0.05 * arc.r + 2.0 * tol2 {
                decline(declined, "coneAxis");
                continue;
            }
            // keep the axis pointing from the apex into the cone
            let h0 = run.height(rail_mid[0]);
            if h0 < 0.0 {
                run.axis = -run.axis;
            }
        }
        // corner residual and facet sag, in mm
        let mut resid: f64 = 0.0;
        for &e in &run.rails {
            let (p, q) = m.ends(e);
            for v in [p, q] {
                let h = run.height(v);
                resid = resid.max((run.radial(v).len() - run.radius_at(h)).abs() * if kind == Kind::Cone { 1.0 / (1.0 + run.tan_half * run.tan_half).sqrt() } else { 1.0 });
            }
        }
        let max_r = (0..run.rails.len()).map(|i| { let (p, q) = m.ends(run.rails[i]); run.radius_at(run.height(p)).max(run.radius_at(run.height(q))) }).fold(0.0, f64::max);
        let sag = max_r * (1.0 - (arc.step / 2.0).cos());
        run.max_residual = resid;
        run.sag = sag;
        if resid > o.tol_fit * 1.5 {
            decline(declined, "residual");
            continue;
        }
        if sag > o.tol {
            decline(declined, "sag");
            continue;
        }
        // convex or concave about the axis, the same all along
        let mut sign = 0i32;
        let mut ok = true;
        for &f in &run.faces {
            let mid = face_centroid(b, f);
            let s = (b.planar_normal(f).dot(run.radial(mid)) > 0.0) as i32 * 2 - 1;
            if sign == 0 { sign = s } else if sign != s { ok = false }
        }
        if !ok {
            decline(declined, "mixedSense");
            continue;
        }
        run.sense = sign > 0;
        // end chords
        let rail_set: HashSet<EdgeId> = run.rails.iter().copied().collect();
        let mut bad = None;
        for &f in &run.faces {
            for e in b.face_edges(f) {
                if rail_set.contains(&e) {
                    continue;
                }
                let (p, q) = m.ends(e);
                let (hp, hq) = (run.height(p), run.height(q));
                if (hp - hq).abs() > o.tol_fit * 2.0 {
                    bad = Some("obliqueEnd");
                }
                run.chords.push((e, (hp + hq) * 0.5));
            }
        }
        if let Some(r) = bad {
            decline(declined, r);
            continue;
        }
        out.push(run);
    }
    out
}

fn face_centroid(b: &Body, f: FaceId) -> Vec3 {
    let l = b.lp(b.face(f).loops[0]);
    let n = l.coedges.len() as f64;
    l.coedges.iter().fold(Vec3::ZERO, |s, &c| s + b.vertex(b.coedge_start(c)).p) * (1.0 / n)
}

// ------------------------------------------------------------------ editing

fn surface_of(run: &Run, b: &Body) -> (Surface, Frame) {
    let a = run.axis;
    // the seam is the first rail, at its end farther from the axis (a cone's rail may start at the apex)
    let ed = b.edge(run.rails[0]);
    let (p, q) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
    let p0 = if run.radial(p).len() >= run.radial(q).len() { p } else { q };
    let h = run.height(p0);
    let o = run.origin + a * h;
    let x = run.radial(p0).normalize();
    let fr = Frame { o, x, y: a.cross(x), z: a };
    match run.kind {
        Kind::Cylinder => (Surface::Cylinder { frame: fr, r: run.radius }, fr),
        Kind::Cone => (Surface::Cone { frame: fr, r: h * run.tan_half, semi_angle: run.tan_half.atan() }, fr),
    }
}

/// Angle of `p` about the frame's axis, in [0, 2 pi).
fn theta(fr: &Frame, p: Vec3) -> f64 {
    let d = p - fr.o;
    let t = d.dot(fr.y).atan2(d.dot(fr.x));
    if t < 0.0 { t + TAU } else { t }
}

/// Turn the end chord `e` of a fitted run into a circular arc about the axis (idempotent: a chord shared by two runs is the
/// same circle).
fn arc_edge(b: &mut Body, e: EdgeId, fr: &Frame, radius_at: impl Fn(f64) -> f64) {
    let (v0, v1) = (b.edge(e).v0, b.edge(e).v1);
    let (p0, p1) = (b.vertex(v0).p, b.vertex(v1).p);
    let h = ((p0 - fr.o).dot(fr.z) + (p1 - fr.o).dot(fr.z)) * 0.5;
    let c = fr.o + fr.z * h;
    let r = radius_at(h);
    let (t0, t1) = (theta(fr, p0), theta(fr, p1));
    let d = wrap_pi(t1 - t0);
    let (sigma, a, z) = if d >= 0.0 { (1.0, t0, fr.z) } else { (-1.0, -t0, -fr.z) };
    let y = fr.y * sigma;
    let curve = Curve::Circle { frame: Frame { o: c, x: fr.x, y, z }, r };
    let ed = &mut b.edges[e.idx()];
    ed.curve = curve;
    ed.t0 = a;
    ed.t1 = a + d.abs();
}

/// Chain the coedges into closed loops, preferring not to turn straight back along the edge just walked (the seam).
fn retrace(b: &mut Body, f: FaceId, pool: &[CoedgeId]) {
    let mut used = vec![false; pool.len()];
    for s in 0..pool.len() {
        if used[s] {
            continue;
        }
        let nl = b.add_loop(f);
        let mut k = s;
        loop {
            used[k] = true;
            let c = pool[k];
            b.coedges[c.idx()].lp = nl;
            b.loops[nl.idx()].coedges.push(c);
            let end = b.coedge_end(c);
            let here = b.coedge(c).edge;
            let cands: Vec<usize> = (0..pool.len()).filter(|&m| !used[m] && b.coedge_start(pool[m]) == end).collect();
            // at the seam go along it first (a periodic face's loop runs round, up the seam, round again and back down)
            let seam = |m: usize| pool.iter().filter(|&&x| b.coedge(x).edge == b.coedge(pool[m]).edge).count() == 2;
            let pick = cands.iter().find(|&&m| b.coedge(pool[m]).edge != here && seam(m)).or_else(|| cands.iter().find(|&&m| b.coedge(pool[m]).edge != here)).or(cands.first());
            match pick {
                Some(&m) => k = m,
                None => break,
            }
        }
    }
}

/// Exact pcurves of the coedges of a fitted face, continuous along each loop (the angle runs on across the seam).
fn set_pcurves(b: &mut Body, f: FaceId, fr: &Frame) {
    for l in b.face(f).loops.clone() {
        let mut cur_u: Option<f64> = None;
        // the turns the loop makes round the axis along its circles
        let wind: f64 = b.lp(l).coedges.iter().map(|&c| {
            let ce = b.coedge(c);
            let ed = b.edge(ce.edge);
            match &ed.curve {
                Curve::Circle { frame: cf, .. } => (if cf.z.dot(fr.z) > 0.0 { 1.0 } else { -1.0 }) * (if ce.rev { ed.t0 - ed.t1 } else { ed.t1 - ed.t0 }),
                _ => 0.0,
            }
        }).sum();
        for c in b.lp(l).coedges.clone() {
            let ce = b.coedge(c).clone();
            let ed = b.edge(ce.edge).clone();
            let (ts, te) = if ce.rev { (ed.t1, ed.t0) } else { (ed.t0, ed.t1) };
            let (mut o, d) = match &ed.curve {
                Curve::Circle { frame: cf, .. } => {
                    // the circle runs at angle phi + sigma t about the surface axis
                    let sigma = if cf.z.dot(fr.z) > 0.0 { 1.0 } else { -1.0 };
                    let phi = cf.x.dot(fr.y).atan2(cf.x.dot(fr.x));
                    (Vec2::new(phi, (cf.o - fr.o).dot(fr.z)), Vec2::new(sigma, 0.0))
                }
                Curve::Line { o, d } => {
                    let (p, q) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
                    let far = if radial_len(fr, p) >= radial_len(fr, q) { p } else { q };
                    (Vec2::new(theta(fr, far), (*o - fr.o).dot(fr.z)), Vec2::new(0.0, d.dot(fr.z)))
                }
                _ => continue,
            };
            // the angle runs on from the previous coedge, except out of a pole: a cone's apex is where the loop may jump a turn
            let start = b.vertex(b.coedge_start(c)).p;
            if let Some(u) = cur_u {
                // out of the pole the loop jumps back the turns it made round the axis (a cone to its apex: one)
                let want = if radial_len(fr, start) > 1e-9 { u } else { u - wind };
                o.x += TAU * ((want - (o.x + d.x * ts)) / TAU).round();
            }
            cur_u = Some(o.x + d.x * te);
            b.coedges[c.idx()].pcurve = Some(Curve2::Line { o, d });
        }
    }
}

fn radial_len(fr: &Frame, p: Vec3) -> f64 {
    let d = p - fr.o;
    (d - fr.z * d.dot(fr.z)).len()
}

/// Replace the strips of `run` by one face.
fn apply(b: &mut Body, run: &Run) -> FaceId {
    let (surface, fr) = surface_of(run, b);
    let first = run.faces[0];
    let shell = b.face(first).shell;
    let name = b.face(first).name.clone();
    let mut absorbed: Vec<String> = run.faces.iter().skip(1).map(|&f| b.face_id_str(f)).collect();
    absorbed.extend(run.faces.iter().flat_map(|&f| b.face(f).absorbed.clone()));
    // the inner rails go; a ring keeps one as its seam
    let seam = if run.full { Some(run.rails[0]) } else { None };
    let (lo, hi) = if run.full { (1, run.rails.len()) } else { (1, run.rails.len() - 1) };
    let mut gone: HashSet<EdgeId> = run.rails[lo..hi].iter().copied().collect();
    if let Some(s) = seam { gone.remove(&s); }
    for &e in &gone {
        for c in b.edge_coedges(e) {
            b.coedges[c.idx()].alive = false;
        }
        b.edges[e.idx()].alive = false;
    }
    let mut pool: Vec<CoedgeId> = vec![];
    for &f in &run.faces {
        for l in b.face(f).loops.clone() {
            b.loops[l.idx()].alive = false;
            pool.extend(b.lp(l).coedges.iter().copied().filter(|&c| b.coedges[c.idx()].alive));
        }
    }
    let g = b.add_face(shell, surface.clone(), run.sense, name);
    b.faces[g.idx()].absorbed = absorbed;
    for &f in &run.faces {
        b.faces[f.idx()].alive = false;
        b.faces[f.idx()].loops.clear();
        b.shells[shell.idx()].faces.retain(|&x| x != f);
    }
    retrace(b, g, &pool);
    set_pcurves(b, g, &fr);
    // the planes across the arcs now bound a circle: exact pcurves on them
    for &(e, _) in &run.chords {
        for c in b.edge_coedges(e) {
            let face = b.coedge_face(c);
            if face == g {
                continue;
            }
            if let Surface::Plane { frame } = b.face(face).surface.clone() {
                let curve = b.edge(e).curve.clone();
                b.coedges[c.idx()].pcurve = plane_pcurve(&frame, &curve);
            }
        }
    }
    gc(b);
    g
}

// ------------------------------------------------------------------ rings

/// The circle (rounded) and the face across, for a coedge of G on a circular edge.
fn arc_key(b: &Body, c: CoedgeId) -> Option<(FaceId, [i64; 5])> {
    let ed = b.edge(b.coedge(c).edge);
    let Curve::Circle { frame, r } = &ed.curve else { return None };
    let cs = b.edge_coedges(b.coedge(c).edge);
    if cs.len() != 2 {
        return None;
    }
    let other = if cs[0] == c { cs[1] } else { cs[0] };
    let q = |v: f64| (v * 1e5).round() as i64;
    // the sign of the normal does not matter: the circle is the same
    let z = if frame.z.dot(Vec3::new(0.577, 0.5771, 0.5772)) < 0.0 { -frame.z } else { frame.z };
    Some((b.coedge_face(other), [q(frame.o.x), q(frame.o.y), q(frame.o.z), q(*r), q(z.x * 1e3 + z.y * 1e5 + z.z * 1e7)]))
}

/// Two consecutive coedges whose shared vertex has no other edge.
fn joinable(b: &Body, a: CoedgeId, c: CoedgeId) -> bool {
    let v = b.coedge_end(a);
    v == b.coedge_start(c) && b.edge_ids().filter(|&e| { let ed = b.edge(e); ed.v0 == v || ed.v1 == v }).count() == 2
}

/// Join runs of arcs of one circle between G and the same other face into single edges (a full circle where they close up).
fn join_arcs(b: &mut Body, g: FaceId) {
    'again: loop {
        for l in b.face(g).loops.clone() {
            let cos = b.lp(l).coedges.clone();
            let n = cos.len();
            let keys: Vec<Option<(FaceId, [i64; 5])>> = cos.iter().map(|&c| arc_key(b, c)).collect();
            let link: Vec<bool> = (0..n).map(|i| keys[i].is_some() && keys[i] == keys[(i + 1) % n] && joinable(b, cos[i], cos[(i + 1) % n])).collect();
            for s in 0..n {
                if keys[s].is_none() || link[(s + n - 1) % n] {
                    continue;
                }
                let mut len = 1;
                while len < n && link[(s + len - 1) % n] {
                    len += 1;
                }
                if len >= 2 && join_run(b, g, l, &cos, s, len) {
                    continue 'again;
                }
            }
        }
        break;
    }
}

/// Replace coedges `cos[s..s + len]` of G's loop `l` (and their partners across) by one edge.
fn join_run(b: &mut Body, g: FaceId, l: LoopId, cos: &[CoedgeId], s: usize, len: usize) -> bool {
    let n = cos.len();
    let run: Vec<CoedgeId> = (0..len).map(|k| cos[(s + k) % n]).collect();
    let edges: Vec<EdgeId> = run.iter().map(|&c| b.coedge(c).edge).collect();
    let (first, last) = (run[0], run[len - 1]);
    let (vs, ve) = (b.coedge_start(first), b.coedge_end(last));
    let Curve::Circle { frame, r } = b.edge(edges[0]).curve.clone() else { return false };
    // the circle, running the way G travels
    let (z, y) = if b.coedge(first).rev { (-frame.z, -frame.y) } else { (frame.z, frame.y) };
    let nf = Frame { o: frame.o, x: frame.x, y, z };
    let ang = |p: Vec3| { let d = p - nf.o; d.dot(nf.y).atan2(d.dot(nf.x)) };
    let sweep = if vs == ve {
        TAU
    } else {
        run.iter().map(|&c| wrap_pi(ang(b.vertex(b.coedge_end(c)).p) - ang(b.vertex(b.coedge_start(c)).p)).abs()).sum::<f64>()
    };
    if !(sweep > 0.0 && sweep <= TAU + 1e-9) {
        return false;
    }
    let partners: Vec<CoedgeId> = run.iter().map(|&c| { let x = b.edge_coedges(b.coedge(c).edge); if x[0] == c { x[1] } else { x[0] } }).collect();
    let ol = b.coedge(partners[0]).lp;
    let other_face = b.coedge_face(partners[0]);
    if partners.iter().any(|&p| b.coedge(p).lp != ol) {
        return false;
    }
    // both loops must hold their run contiguously (cyclically); check before touching anything
    let contiguous = |b: &Body, lp: LoopId, cs: &[CoedgeId]| -> Option<usize> {
        let list = &b.lp(lp).coedges;
        let m = list.len();
        let st = list.iter().position(|&c| c == cs[0])?;
        (m >= cs.len() && (0..cs.len()).all(|k| list[(st + k) % m] == cs[k])).then_some(st)
    };
    let rev_partners: Vec<CoedgeId> = partners.iter().rev().copied().collect();
    let (Some(sg), Some(so)) = (contiguous(b, l, &run), contiguous(b, ol, &rev_partners)) else { return false };
    let t0 = ang(b.vertex(vs).p);
    let ne = b.add_edge(Curve::Circle { frame: nf, r }, t0, t0 + sweep, vs, ve);
    let push = |b: &mut Body, rev: bool, lp: LoopId| -> CoedgeId {
        b.coedges.push(ck_topo::body::Coedge { edge: ne, rev, lp, pcurve: None, alive: true });
        let c = CoedgeId(b.coedges.len() as u32 - 1);
        b.edges[ne.idx()].coedges.push(c);
        c
    };
    let (nc, nco) = (push(b, false, l), push(b, true, ol));
    for (lp, st, k, new) in [(l, sg, run.len(), nc), (ol, so, run.len(), nco)] {
        let list = &mut b.loops[lp.idx()].coedges;
        list.rotate_left(st);
        list.splice(0..k, [new]);
    }
    for &c in run.iter().chain(partners.iter()) {
        b.coedges[c.idx()].alive = false;
    }
    for &e in &edges {
        b.edges[e.idx()].alive = false;
    }
    gc(b);
    if let Surface::Plane { frame } = b.face(other_face).surface.clone() {
        let curve = b.edge(ne).curve.clone();
        b.coedges[nco.idx()].pcurve = plane_pcurve(&frame, &curve);
    }
    let fr = match &b.face(g).surface {
        Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => *frame,
        _ => return true,
    };
    set_pcurves(b, g, &fr);
    true
}

// ------------------------------------------------------------------ driver

fn diag(b: &Body) -> f64 {
    let (lo, hi) = b.bbox();
    (hi - lo).len()
}

/// Refit facet fans as cylinders and cones, in place. See the module comment for what is recognised.
pub fn fit_quadrics(b: &mut Body, o: &FitOpts) -> FitReport {
    let mut rep = FitReport { faces_before: b.num_faces(), ..Default::default() };
    if o.merge {
        if b.num_faces() <= o.merge_max {
            rep.merged = merge_coplanar(b, 1e-9);
        } else {
            rep.merge_skipped = true;
        }
    }
    let mut runs: Vec<Run> = vec![];
    {
        let m = Mesh { b, tol_fit: o.tol_fit };
        let mut claimed: HashSet<FaceId> = HashSet::new();
        let mut seen: HashSet<(FaceId, EdgeId, EdgeId)> = HashSet::new();
        let faces: Vec<FaceId> = b.face_ids().collect();
        for f in faces {
            if !m.simple(f) {
                continue;
            }
            let rails = m.rails_of(f);
            for i in 0..rails.len() {
                for j in i + 1..rails.len() {
                    let (e1, e2) = (rails[i].0, rails[j].0);
                    if seen.contains(&(f, e1, e2)) || claimed.contains(&f) {
                        continue;
                    }
                    let Some(axis) = m.axis_of(e1, e2) else { continue };
                    if matches!(axis, Axis::Apex(_)) && !o.cones {
                        continue;
                    }
                    let ch = m.grow(f, e1, e2, axis, &claimed);
                    // every (strip, its two rails) of the chain is covered
                    for (k, &cf) in ch.faces.iter().enumerate() {
                        let (l, r) = (ch.rails[k], ch.rails[(k + 1) % ch.rails.len()]);
                        seen.insert((cf, l, r));
                        seen.insert((cf, r, l));
                    }
                    if ch.faces.len() < o.min_strips {
                        continue;
                    }
                    for run in runs_of(&m, &ch, o, &mut rep.declined) {
                        claimed.extend(run.faces.iter().copied());
                        runs.push(run);
                    }
                }
            }
        }
    }
    // ends: each chord's other face must be a plane normal to the axis (it holds the circle) or a strip of a run on the same circle;
    // coaxial runs that meet smoothly are a curved profile. Drop what fails and look again.
    loop {
        let owner: HashMap<FaceId, usize> = runs.iter().enumerate().flat_map(|(i, r)| r.faces.iter().map(move |&f| (f, i))).collect();
        let mut drop: HashSet<usize> = HashSet::new();
        let mut why: HashMap<usize, &'static str> = HashMap::new();
        for (i, r) in runs.iter().enumerate() {
            for &(e, h) in &r.chords {
                let fs = b.edge_faces(e);
                let others: Vec<FaceId> = fs.into_iter().filter(|f| !r.faces.contains(f)).collect();
                if others.len() > 1 {
                    drop.insert(i);
                    why.insert(i, "endNotManifold");
                    continue;
                }
                let Some(&x) = others.first() else { continue };
                if let Some(&j) = owner.get(&x) {
                    // another run: same axis line, same circle at this height, not a smooth continuation
                    let s = &runs[j];
                    let p = b.vertex(b.edge(e).v0).p;
                    let same_axis = r.axis.cross(s.axis).len() < 1e-5 && (s.origin - r.origin).cross(r.axis).len() < 4.0 * o.tol_fit;
                    let _ = h;
                    let (n1, n2) = (r.normal_at(p), s.normal_at(p));
                    let crease = n1.dot(n2).clamp(-1.0, 1.0).acos().to_degrees();
                    let radius_ok = (r.radius_at(r.height(p)) - s.radius_at(s.height(p))).abs() <= o.tol_fit * 4.0;
                    if !same_axis || !radius_ok {
                        drop.insert(i);
                        why.insert(i, "endMismatch");
                    } else if crease < o.smooth_deg && !(r.kind == Kind::Cylinder && s.kind == Kind::Cylinder && crease < 0.5) {
                        drop.insert(i);
                        drop.insert(j);
                        why.insert(i, "smoothProfile");
                        why.insert(j, "smoothProfile");
                    }
                } else {
                    // a plane normal to the axis
                    let ok = matches!(b.face(x).surface, Surface::Plane { .. }) && b.planar_normal(x).cross(r.axis).len() < 1e-5;
                    if !ok {
                        drop.insert(i);
                        why.insert(i, "endNotPlane");
                    }
                }
            }
        }
        if drop.is_empty() {
            break;
        }
        let mut keep = vec![];
        for (i, r) in runs.into_iter().enumerate() {
            if drop.contains(&i) {
                rep.declined.push(Declined { face: r.faces[0], strips: r.faces.len(), reason: why.get(&i).copied().unwrap_or("end") });
            } else {
                keep.push(r);
            }
        }
        runs = keep;
    }
    // the end chords become circles once (a chord two runs share is the same circle), then the strips are replaced
    let mut done: HashSet<EdgeId> = HashSet::new();
    for r in &runs {
        let (_, fr) = surface_of(r, b);
        for &(e, _) in &r.chords {
            if done.insert(e) {
                arc_edge(b, e, &fr, |h| r.radius_at(r.height(fr.o + fr.z * h)));
            }
        }
    }
    let mut made: Vec<(usize, FaceId)> = vec![];
    for (i, r) in runs.iter().enumerate() {
        let g = apply(b, r);
        made.push((i, g));
    }
    for &(_, g) in &made {
        join_arcs(b, g);
    }
    for (i, g) in made {
        let r = &runs[i];
        rep.groups.push(FitGroup { kind: r.kind, face: g, strips: r.faces.len(), full: r.full, axis: r.axis, origin: r.origin, radius: if r.kind == Kind::Cylinder { r.radius } else { 0.0 }, half_angle: r.tan_half.atan(), max_residual: r.max_residual, sag: r.sag });
    }
    rep.faces_after = b.num_faces();
    rep
}

/// Defaults scaled to the body (see [`FitOpts::for_size`]).
pub fn default_opts(b: &Body) -> FitOpts {
    FitOpts::for_size(diag(b))
}

// ------------------------------------------------------------------ wire request

fn vec_json(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

/// Options of a `fitQuadrics` request (`opts` of the message) over the defaults for `b`.
fn opts_of(b: &Body, msg: &Json) -> FitOpts {
    let mut o = default_opts(b);
    let Some(m) = msg.get("opts") else { return o };
    if let Some(v) = m.get_f64("tol").filter(|v| *v > 0.0) {
        o.tol = v;
        o.tol_fit = v * 0.1;
    }
    if let Some(v) = m.get_f64("fitTol").filter(|v| *v > 0.0) {
        o.tol_fit = v;
    }
    if let Some(v) = m.get_f64("maxStepDeg").filter(|v| *v > 0.0) {
        o.max_step_deg = v;
    }
    if let Some(v) = m.get_f64("minStrips").filter(|v| *v >= 2.0) {
        o.min_strips = v as usize;
    }
    if let Some(v) = m.get_f64("smoothDeg").filter(|v| *v >= 0.0) {
        o.smooth_deg = v;
    }
    if let Some(v) = m.get_bool("cones") {
        o.cones = v;
    }
    if let Some(v) = m.get_bool("merge") {
        o.merge = v;
    }
    o
}

/// `{type:"fitQuadrics", tag, opts?:{tol, fitTol, maxStepDeg, minStrips, smoothDeg, cones, merge, volumeTol, dryRun}}`: refit the facet fans
/// of the tag's body as cylinders and cones (see the module comment). Answers the report and, when the fit is to be kept, the new body:
/// it must validate no worse than the original and keep the volume within `volumeTol` (relative, default 0.05, the change of volume is
/// about the chord sag of the facets), else the original stays and `reason` says why. `dryRun` reports without keeping.
pub fn request(body: &Body, msg: &Json) -> (Json, Option<Body>) {
    let o = opts_of(body, msg);
    let m = msg.get("opts");
    let vol_tol = m.and_then(|m| m.get_f64("volumeTol")).filter(|v| *v > 0.0).unwrap_or(0.05);
    let dry = m.and_then(|m| m.get_bool("dryRun")).unwrap_or(false);
    let (fitted, rep) = {
        let mut c = body.clone();
        let r = fit_quadrics(&mut c, &o);
        (c, r)
    };
    let (v0, v1) = (ck_topo::mass::mass_props(body).volume, ck_topo::mass::mass_props(&fitted).volume);
    let (i0, i1) = (ck_check::validate(body, 1e-6).len(), ck_check::validate(&fitted, 1e-6).len());
    let rel = if v0.abs() > 1e-12 { (v1 - v0) / v0.abs() } else { 0.0 };
    let reason = if rep.groups.is_empty() {
        Some("nothing")
    } else if i1 > i0 {
        Some("invalid")
    } else if rel.abs() > vol_tol {
        Some("volumeChanged")
    } else if dry {
        Some("dryRun")
    } else {
        None
    };
    let groups: Vec<Json> = rep.groups.iter().map(|g| {
        let kind = match g.kind { Kind::Cylinder => "cylinder", Kind::Cone => "cone" };
        let mut o = Obj::new().put("kind", kind).put("face", fitted.face_id_str(g.face).as_str()).put("strips", g.strips as f64).put("full", g.full).put("axis", vec_json(g.axis)).put("origin", vec_json(g.origin));
        o = match g.kind {
            Kind::Cylinder => o.put("radius", g.radius),
            Kind::Cone => o.put("halfAngle", g.half_angle),
        };
        o.put("maxResidual", g.max_residual).put("sag", g.sag).build()
    }).collect();
    let declined: Vec<Json> = rep.declined.iter().map(|d| Obj::new().put("face", body.face_id_str(d.face).as_str()).put("strips", d.strips as f64).put("reason", d.reason).build()).collect();
    let mut out = Obj::new()
        .put("applied", reason.is_none())
        .put("tol", o.tol)
        .put("fitTol", o.tol_fit)
        .put("faces", Obj::new().put("before", rep.faces_before as f64).put("after", fitted.num_faces() as f64).build())
        .put("volume", Obj::new().put("before", v0).put("after", v1).put("relative", rel).build())
        .put("issues", Obj::new().put("before", i0 as f64).put("after", i1 as f64).build())
        .put("merged", rep.merged as f64);
    if rep.merge_skipped {
        out = out.put("mergeSkipped", true);
    }
    if let Some(r) = reason {
        out = out.put("reason", r);
    }
    out = out.put("groups", Json::Arr(groups)).put("declined", Json::Arr(declined));
    (out.build(), reason.is_none().then_some(fitted))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_topo::body::VertexId;
    use ck_topo::mass::mass_props;

    /// Welded polygon soup -> Body: one lump, one shell. Polygons are counter-clockwise about their outward normal.
    pub(crate) fn mesh_body(polys: &[Vec<Vec3>]) -> Body {
        let faces: Vec<(Vec<Vec3>, Vec<Vec<Vec3>>)> = polys.iter().map(|p| (p.clone(), vec![])).collect();
        mesh_body_h(&faces)
    }

    /// The same with holes (clockwise about the outward normal).
    pub(crate) fn mesh_body_h(faces: &[(Vec<Vec3>, Vec<Vec<Vec3>>)]) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let mut verts: Vec<(Vec3, VertexId)> = vec![];
        let mut edges: HashMap<(u32, u32), EdgeId> = HashMap::new();
        for (i, (poly, holes)) in faces.iter().enumerate() {
            let mut n = Vec3::ZERO;
            for k in 0..poly.len() {
                let (p, q) = (poly[k], poly[(k + 1) % poly.len()]);
                n = n + Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
            }
            let n = n.normalize();
            let f = b.add_face(shell, Surface::Plane { frame: Frame::from_normal(poly[0], n) }, true, ck_topo::body::FaceName::other("m", &format!("F{i}")));
            for ring in std::iter::once(poly).chain(holes.iter()) {
                let lp = b.add_loop(f);
                let vs: Vec<VertexId> = ring.iter().map(|&p| {
                    if let Some(&(_, v)) = verts.iter().find(|(q, _)| q.dist(p) < 1e-7) {
                        return v;
                    }
                    let v = b.add_vertex(p);
                    verts.push((p, v));
                    v
                }).collect();
                for k in 0..vs.len() {
                    let (a, c) = (vs[k], vs[(k + 1) % vs.len()]);
                    let (lo, hi, rev) = if a.0 < c.0 { (a, c, false) } else { (c, a, true) };
                    let e = *edges.entry((lo.0, hi.0)).or_insert_with(|| {
                        let (pa, pb) = (b.vertex(lo).p, b.vertex(hi).p);
                        b.add_edge(Curve::line_through(pa, pb), 0.0, pa.dist(pb), lo, hi)
                    });
                    b.add_coedge(lp, e, rev, None);
                }
            }
        }
        b
    }

    fn ring(n: usize, r: f64, z: f64, a0: f64, span: f64) -> Vec<Vec3> {
        // n + 1 points over `span` radians when span < TAU; n points for a full turn
        let full = (span - TAU).abs() < 1e-12;
        let m = if full { n } else { n + 1 };
        (0..m).map(|i| { let t = a0 + span * i as f64 / n as f64; Vec3::new(r * t.cos(), r * t.sin(), z) }).collect()
    }

    /// A prism of `n` sides, radius `r`, from z = 0 to `h`.
    fn prism(n: usize, r: f64, h: f64) -> Vec<Vec<Vec3>> {
        let (lo, hi) = (ring(n, r, 0.0, 0.0, TAU), ring(n, r, h, 0.0, TAU));
        let mut polys = vec![lo.iter().rev().copied().collect::<Vec<_>>(), hi.clone()];
        for i in 0..n {
            let j = (i + 1) % n;
            polys.push(vec![lo[i], lo[j], hi[j], hi[i]]);
        }
        polys
    }

    fn volume(b: &Body) -> f64 {
        mass_props(b).volume
    }

    fn check(b: &Body) {
        let issues = ck_check::validate(b, 1e-6);
        assert!(issues.is_empty(), "{issues:?}");
    }

    fn opts(b: &Body) -> FitOpts {
        let mut o = default_opts(b);
        o.tol = 0.2;
        o.tol_fit = 1e-4;
        o
    }

    #[test]
    fn a_faceted_cylinder_becomes_three_faces_at_every_resolution() {
        for n in [24usize, 48, 96] {
            let mut b = mesh_body(&prism(n, 10.0, 20.0));
            assert_eq!(b.num_faces(), n + 2);
            check(&b);
            let v0 = volume(&b);
            let o = opts(&b);
            let rep = fit_quadrics(&mut b, &o);
            assert_eq!(rep.groups.len(), 1, "n={n} {:?}", rep.declined);
            assert_eq!(b.num_faces(), 3, "n={n}");
            assert_eq!(rep.groups[0].strips, n);
            assert!(rep.groups[0].full);
            check(&b);
            let v1 = volume(&b);
            let exact = PI * 100.0 * 20.0;
            assert!((v1 - exact).abs() < 1e-6 * exact, "n={n} v={v1} exact={exact}");
            // the facets enclose the inscribed prism: the fit adds the sag only
            let ratio = PI / (n as f64 / 2.0 * (TAU / n as f64).sin());   // circle over inscribed polygon
            assert!(((v1 - v0) / v0 - (ratio - 1.0)).abs() < 1e-9, "n={n} {v0} {v1}");
            assert!((rep.groups[0].radius - 10.0).abs() < 1e-9);
        }
    }

    #[test]
    fn counts_edges_of_a_fitted_cylinder() {
        let mut b = mesh_body(&prism(48, 10.0, 20.0));
        let o = opts(&b);
        fit_quadrics(&mut b, &o);
        // two circles and a seam
        assert_eq!(b.edge_ids().count(), 3);
    }

    #[test]
    fn square_and_hexagonal_prisms_stay_prisms() {
        for n in [4usize, 6, 8] {
            let mut b = mesh_body(&prism(n, 10.0, 20.0));
            let o = opts(&b);
            let rep = fit_quadrics(&mut b, &o);
            assert!(rep.groups.is_empty(), "n={n}");
            assert_eq!(b.num_faces(), n + 2);
        }
    }

    /// A frustum (or a cone when `r1` is 0) of `n` sides from z = 0 (radius r0) to z = h (radius r1).
    fn frustum(n: usize, r0: f64, r1: f64, h: f64) -> Vec<Vec<Vec3>> {
        let lo = ring(n, r0, 0.0, 0.0, TAU);
        let mut polys = vec![lo.iter().rev().copied().collect::<Vec<_>>()];
        if r1 > 0.0 {
            let hi = ring(n, r1, h, 0.0, TAU);
            polys.push(hi.clone());
            for i in 0..n {
                let j = (i + 1) % n;
                polys.push(vec![lo[i], lo[j], hi[j], hi[i]]);
            }
        } else {
            let apex = Vec3::new(0.0, 0.0, h);
            for i in 0..n {
                polys.push(vec![lo[i], lo[(i + 1) % n], apex]);
            }
        }
        polys
    }

    #[test]
    fn a_faceted_cone_and_frustum_become_cones() {
        for (n, r1) in [(24usize, 0.0), (48, 0.0), (48, 4.0), (96, 6.0)] {
            let mut b = mesh_body(&frustum(n, 10.0, r1, 15.0));
            check(&b);
            let v0 = volume(&b);
            let o = opts(&b);
            let rep = fit_quadrics(&mut b, &o);
            assert_eq!(rep.groups.len(), 1, "n={n} r1={r1} {:?}", rep.declined);
            let g = &rep.groups[0];
            assert_eq!(g.kind, Kind::Cone);
            assert!(g.full);
            assert_eq!(b.num_faces(), if r1 > 0.0 { 3 } else { 2 }, "n={n} r1={r1}");
            check(&b);
            let exact = PI * 15.0 / 3.0 * (100.0 + 10.0 * r1 + r1 * r1);
            let v1 = volume(&b);
            assert!((v1 - exact).abs() < 1e-5 * exact, "n={n} r1={r1} v={v1} exact={exact}");
            assert!(v1 > v0);
            assert!((g.half_angle - ((10.0 - r1) / 15.0f64).atan()).abs() < 1e-9, "{}", g.half_angle);
        }
    }

    /// An arc of `n` steps over `span` radians closed by a flat chord (a D shape), from z = 0 to `h`.
    fn d_shaft(n: usize, r: f64, h: f64, span: f64) -> Vec<Vec<Vec3>> {
        let (lo, hi) = (ring(n, r, 0.0, 0.0, span), ring(n, r, h, 0.0, span));
        let mut polys = vec![lo.iter().rev().copied().collect::<Vec<_>>(), hi.clone()];
        for i in 0..=n {
            let j = (i + 1) % (n + 1);
            polys.push(vec![lo[i], lo[j], hi[j], hi[i]]);
        }
        polys
    }

    #[test]
    fn a_cylinder_with_a_flat_keeps_the_flat_and_fits_the_arc() {
        let (n, r, h) = (40usize, 10.0, 12.0);
        let span = TAU * 300.0 / 360.0;
        let mut b = mesh_body(&d_shaft(n, r, h, span));
        assert_eq!(b.num_faces(), n + 1 + 2);
        check(&b);
        let o = opts(&b);
        let rep = fit_quadrics(&mut b, &o);
        assert_eq!(rep.groups.len(), 1, "{:?}", rep.declined);
        assert!(!rep.groups[0].full);
        assert_eq!(rep.groups[0].strips, n);
        assert_eq!(b.num_faces(), 4);   // the arc, the flat, two caps
        check(&b);
        // the D: the circle less the segment the chord cuts off
        let phi = TAU - span;
        let area = PI * r * r - r * r / 2.0 * (phi - phi.sin());
        let v = volume(&b);
        assert!((v - area * h).abs() < 1e-6 * area * h, "{v} {}", area * h);
        // two arcs and the chord edges: the cap circles are single arcs now
        assert_eq!(b.edge_ids().filter(|&e| matches!(b.edge(e).curve, Curve::Circle { .. })).count(), 2);
    }

    #[test]
    fn a_mixed_part_gets_each_cylinder_and_cone_and_keeps_its_planes() {
        let (n, bw) = (48usize, 20.0);
        let sq = |z: f64| vec![Vec3::new(-bw, -bw, z), Vec3::new(bw, -bw, z), Vec3::new(bw, bw, z), Vec3::new(-bw, bw, z)];
        let mut faces: Vec<(Vec<Vec3>, Vec<Vec<Vec3>>)> = vec![];
        // block: 40 x 40 x 10
        let (b0, b1) = (sq(0.0), sq(10.0));
        faces.push((b0.iter().rev().copied().collect(), vec![]));
        for i in 0..4 {
            let j = (i + 1) % 4;
            faces.push((vec![b0[i], b0[j], b1[j], b1[i]], vec![]));
        }
        // its top has the boss's footprint as a hole (clockwise)
        let foot = ring(n, 8.0, 10.0, 0.0, TAU);
        faces.push((b1.clone(), vec![foot.iter().rev().copied().collect()]));
        // boss: r 8 from 10 to 20, then a frustum 8 -> 4 up to 26 and a flat top
        let (c1, c2, c3) = (ring(n, 8.0, 10.0, 0.0, TAU), ring(n, 8.0, 20.0, 0.0, TAU), ring(n, 4.0, 26.0, 0.0, TAU));
        for i in 0..n {
            let j = (i + 1) % n;
            faces.push((vec![c1[i], c1[j], c2[j], c2[i]], vec![]));
            faces.push((vec![c2[i], c2[j], c3[j], c3[i]], vec![]));
        }
        faces.push((c3.clone(), vec![]));
        let mut b = mesh_body_h(&faces);
        check(&b);
        let o = opts(&b);
        let rep = fit_quadrics(&mut b, &o);
        assert_eq!(rep.groups.iter().map(|g| g.kind).collect::<Vec<_>>().len(), 2, "{:?}", rep.declined);
        assert!(rep.groups.iter().any(|g| g.kind == Kind::Cylinder) && rep.groups.iter().any(|g| g.kind == Kind::Cone));
        assert_eq!(b.num_faces(), 1 + 4 + 1 + 1 + 1 + 1);
        check(&b);
        let exact = 40.0 * 40.0 * 10.0 + PI * 64.0 * 10.0 + PI * 6.0 / 3.0 * (64.0 + 32.0 + 16.0);
        let v = volume(&b);
        assert!((v - exact).abs() < 1e-5 * exact, "{v} {exact}");
        // the circle between the boss and the frustum, the foot of the boss in the top and the top of the frustum: three whole circles
        assert_eq!(b.edge_ids().filter(|&e| matches!(b.edge(e).curve, Curve::Circle { .. })).count(), 3);
    }

    #[test]
    fn a_hole_is_a_concave_cylinder() {
        // a 40 x 40 x 10 block with a 48-sided hole of radius 6 through it
        let n = 48usize;
        let sq = |z: f64| vec![Vec3::new(-20.0, -20.0, z), Vec3::new(20.0, -20.0, z), Vec3::new(20.0, 20.0, z), Vec3::new(-20.0, 20.0, z)];
        let (b0, b1) = (sq(0.0), sq(10.0));
        let (h0, h1) = (ring(n, 6.0, 0.0, 0.0, TAU), ring(n, 6.0, 10.0, 0.0, TAU));
        let mut faces: Vec<(Vec<Vec3>, Vec<Vec<Vec3>>)> = vec![(b0.iter().rev().copied().collect(), vec![h0.clone()]), (b1.clone(), vec![h1.iter().rev().copied().collect()])];
        for i in 0..4 {
            let j = (i + 1) % 4;
            faces.push((vec![b0[i], b0[j], b1[j], b1[i]], vec![]));
        }
        for i in 0..n {
            let j = (i + 1) % n;
            faces.push((vec![h0[j], h0[i], h1[i], h1[j]], vec![]));
        }
        let mut b = mesh_body_h(&faces);
        check(&b);
        let o = opts(&b);
        let rep = fit_quadrics(&mut b, &o);
        assert_eq!(rep.groups.len(), 1, "{:?}", rep.declined);
        assert_eq!(b.num_faces(), 7);
        check(&b);
        let exact = 1600.0 * 10.0 - PI * 36.0 * 10.0;
        assert!((volume(&b) - exact).abs() < 1e-5 * exact, "{} {exact}", volume(&b));
    }

    #[test]
    fn triangulated_strips_are_merged_first() {
        let n = 48;
        let mut polys = prism(n, 10.0, 20.0);
        // split every side quad into two triangles
        let sides: Vec<Vec<Vec3>> = polys.split_off(2);
        for q in sides {
            polys.push(vec![q[0], q[1], q[2]]);
            polys.push(vec![q[0], q[2], q[3]]);
        }
        let mut b = mesh_body(&polys);
        assert_eq!(b.num_faces(), 2 + 2 * n);
        let o = opts(&b);
        let rep = fit_quadrics(&mut b, &o);
        assert_eq!(rep.merged, n);
        assert_eq!(rep.groups.len(), 1, "{:?}", rep.declined);
        assert_eq!(b.num_faces(), 3);
        check(&b);
    }

    fn uv_sphere(nlon: usize, nlat: usize, r: f64) -> Vec<Vec<Vec3>> {
        let pt = |i: usize, j: usize| {
            let (th, ph) = (TAU * i as f64 / nlon as f64, PI * j as f64 / nlat as f64);
            Vec3::new(r * ph.sin() * th.cos(), r * ph.sin() * th.sin(), r * ph.cos())
        };
        let mut polys = vec![];
        for i in 0..nlon {
            let k = (i + 1) % nlon;
            polys.push(vec![pt(i, 0), pt(i, 1), pt(k, 1)]);
            for j in 1..nlat - 1 {
                polys.push(vec![pt(i, j), pt(i, j + 1), pt(k, j + 1), pt(k, j)]);
            }
            polys.push(vec![pt(i, nlat - 1), pt(i, nlat), pt(k, nlat - 1)]);
        }
        polys
    }

    #[test]
    fn a_faceted_sphere_is_not_fitted() {
        for (nlon, nlat) in [(24usize, 12usize), (48, 24)] {
            let mut b = mesh_body(&uv_sphere(nlon, nlat, 10.0));
            check(&b);
            let before = b.clone();
            let mut o = opts(&b);
            o.tol = 0.5;
            let rep = fit_quadrics(&mut b, &o);
            assert!(rep.groups.is_empty(), "{nlon}x{nlat} {:?}", rep.groups);
            assert_eq!(b, before, "the body is untouched");
            assert!(rep.declined.iter().any(|d| d.reason == "smoothProfile"), "{:?}", rep.declined);
        }
    }

    #[test]
    fn a_random_triangle_mesh_is_not_fitted() {
        // an octahedron split twice, each vertex pushed in or out by a pseudo random amount
        let mut seed = 12345u64;
        let mut rnd = move || { seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407); ((seed >> 33) as f64) / (1u64 << 31) as f64 };
        let mut tris: Vec<[Vec3; 3]> = vec![];
        let (px, nx, py, ny, pz, nz) = (Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y, Vec3::Z, -Vec3::Z);
        for (a, b, c) in [(px, py, pz), (py, nx, pz), (nx, ny, pz), (ny, px, pz), (py, px, nz), (nx, py, nz), (ny, nx, nz), (px, ny, nz)] {
            tris.push([a, b, c]);
        }
        for _ in 0..3 {
            tris = tris.iter().flat_map(|t| {
                let m = |a: Vec3, b: Vec3| ((a + b) * 0.5).normalize();
                let (ab, bc, ca) = (m(t[0], t[1]), m(t[1], t[2]), m(t[2], t[0]));
                vec![[t[0], ab, ca], [ab, t[1], bc], [ca, bc, t[2]], [ab, bc, ca]]
            }).collect();
        }
        let mut cache: Vec<(Vec3, f64)> = vec![];
        let mut jitter = |p: Vec3| -> Vec3 {
            if let Some(&(_, k)) = cache.iter().find(|(q, _)| q.dist(p) < 1e-9) {
                return p * k;
            }
            let k = 10.0 * (1.0 + 0.15 * (rnd() - 0.5));
            cache.push((p, k));
            p * k
        };
        let polys: Vec<Vec<Vec3>> = tris.iter().map(|t| vec![jitter(t[0]), jitter(t[1]), jitter(t[2])]).collect();
        let mut b = mesh_body(&polys);
        let before = b.clone();
        let mut o = opts(&b);
        o.tol = 1.0;
        let rep = fit_quadrics(&mut b, &o);
        assert!(rep.groups.is_empty(), "{:?}", rep.groups);
        assert_eq!(b.num_faces(), before.num_faces());
        assert_eq!(b, before);
    }

    #[test]
    fn refusals_leave_the_body_alone() {
        // too coarse for the tolerance: 16 sides of radius 10 sag by 0.19 mm
        let mut b = mesh_body(&prism(16, 10.0, 20.0));
        let before = b.clone();
        let mut o = opts(&b);
        o.tol = 0.05;
        let rep = fit_quadrics(&mut b, &o);
        assert!(rep.groups.is_empty());
        assert!(rep.declined.iter().any(|d| d.reason == "sag"), "{:?}", rep.declined);
        assert_eq!(b, before);
        // a cylinder cut obliquely at the top: its end is an ellipse, which is not built
        let n = 48;
        let (lo, hi): (Vec<Vec3>, Vec<Vec3>) = (ring(n, 10.0, 0.0, 0.0, TAU), ring(n, 10.0, 30.0, 0.0, TAU).into_iter().map(|p| Vec3::new(p.x, p.y, 30.0 + 0.3 * p.x)).collect());
        let mut polys = vec![lo.iter().rev().copied().collect::<Vec<_>>(), hi.clone()];
        for i in 0..n {
            let j = (i + 1) % n;
            polys.push(vec![lo[i], lo[j], hi[j], hi[i]]);
        }
        let mut b = mesh_body(&polys);
        let before = b.clone();
        let o = opts(&b);
        let rep = fit_quadrics(&mut b, &o);
        assert!(rep.groups.is_empty());
        assert!(rep.declined.iter().any(|d| d.reason == "obliqueEnd"), "{:?}", rep.declined);
        assert_eq!(b, before);
    }

    #[test]
    fn corner_noise_within_the_fit_tolerance_still_fits() {
        let n = 48;
        let mut polys = prism(n, 10.0, 20.0);
        let mut k = 0u32;
        let mut wob = |p: Vec3| { k += 1; let s = ((k as f64) * 12.9898).sin() * 43758.5453; Vec3::new(p.x + 2e-5 * (s - s.floor() - 0.5), p.y + 2e-5 * ((s * 7.0) - (s * 7.0).floor() - 0.5), p.z) };
        let mut moved: Vec<(Vec3, Vec3)> = vec![];
        for poly in polys.iter_mut() {
            for p in poly.iter_mut() {
                let q = match moved.iter().find(|(o, _)| o.dist(*p) < 1e-9) { Some(&(_, m)) => m, None => { let m = wob(*p); moved.push((*p, m)); m } };
                *p = q;
            }
        }
        let mut b = mesh_body(&polys);
        let mut o = default_opts(&b);
        o.tol = 0.2;
        o.tol_fit = 2e-4;
        o.merge = false;
        let rep = fit_quadrics(&mut b, &o);
        assert_eq!(rep.groups.len(), 1, "{:?}", rep.declined);
        assert_eq!(b.num_faces(), 3);
        assert!(rep.groups[0].max_residual < 2e-4);
        assert!((volume(&b) - PI * 100.0 * 20.0).abs() < 2e-3 * PI * 100.0 * 20.0);
    }
}
