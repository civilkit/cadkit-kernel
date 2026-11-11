//! `projectCurve` (ck-model/src/features/project_curve.rs): project a sketch
//! (or a planar curve feature) perpendicular to its plane onto faces — a 3D
//! curve on the surface (groove path, guide curve). Faces stay unsplit.
//!
//! Payload: `sketch` | `curve` (feature id of an earlier curve: editCurve,
//! projectCurve, ...), `faces[]` (face names; empty = every face of the
//! body and of the surface bodies), `degree` (3), `tolerance` (0.001),
//! `direction` (`both` | `forward` | `reverse` — along / against the sketch
//! plane normal; the normal of a `curve` input is the plane normal with its
//! largest component positive).
//!
//! Check order (oracle): no solid and no surface body → `projectCurveNoFaces`;
//! neither `sketch` nor `curve` → `projectCurveNoInput`; `curve` (wins over
//! `sketch` when both are given) unresolved → `missingReference`, a
//! non-planar one → `projectCurvePlane`; `sketch` unresolved →
//! `missingSketch`; a sketch without a projectable entity (construction,
//! points, zero length, text) → `projectCurveNoInput`; degree outside 2..=9 →
//! `editCurveDegree {min:2,max:9}`; tolerance <= 0 or an unknown `direction` →
//! `invalidValue`; an unknown face name → `missingReference`; no hit →
//! `projectCurveMisses`; the hits do not form ONE chain →
//! `projectCurveSeveral {count, points}` (points: both ends of every open chain).
//!
//! Projection: every sample of every non-construction entity is cast along
//! the normal (both signs, one sign or the other, as `direction` says) against
//! EVERY face picked and ALL its crossings (near and far sheet of a cylinder,
//! sphere, torus...) that lie inside the trimmed face are kept. Consecutive
//! hits on one face form a run (one "piece"); the ends of a run are refined by
//! bisection to the exact boundary of the face or the silhouette of the
//! surface. Runs whose ends meet (closer than 1e-3, the oracle's join
//! distance, independent of `tolerance`) form a chain: near and far sheet
//! meet at a silhouette, neighbouring faces at their shared edge, entities at
//! their joints. One chain → one curve, several chains → `projectCurveSeveral`.
//! Hits where the ray only grazes the surface are dropped; a ray lying in a
//! face (e.g. the side face of a box seen edge-on) contributes nothing.
//!
//! The chain is fitted as one B-spline of `degree` through the runs in order
//! (ck-ops `curve_edit`: knots of multiplicity `degree` at the corners, the
//! control point count raised until the deviation is within `tolerance`, at
//! most 400; a curve that cannot reach the tolerance is returned at 400
//! control points like the oracle does).
//!
//! Output (oracle): `curves[] = [id, {pts, axis:false, t0:0, t1:1}]` and
//! `markers[] = [id, {degree, count, deviation, length, pieces, corners,
//! closed}]`. `pieces` = number of runs; `corners` = joints between runs where
//! the tangent turns by more than 1 degree; `closed` = ends coincide.

use super::curve_ref::sketch_entity_samples;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::{Frame, Vec3};
use ck_geom::curve::Curve;
use ck_json::Json;
use ck_ops::boolean::classify::{point_in_face_cached, LoopCache, Where};
use ck_ops::bvh::{face_box, Aabb};
use ck_ops::curve_edit::{edit_curve, EditError, EditOpts, PieceIn};
use ck_ops::intersect::cs;
use ck_topo::body::{Body, FaceId};

/// Runs whose ends are closer than this form one chain (oracle: a gap of
/// 0.0009 joins, 0.001 does not, whatever `tolerance` is).
const JOIN: f64 = 1e-3;
/// Samples per source entity after densifying / thinning. The chord of a sample pair is off the true curve by
/// h^2 k / 8: with 800..1040 samples that is ~3e-5 on a curve of curvature 1 (the floor of the reachable tolerance).
const DENSE_MIN: usize = 800;
const DENSE_MAX: usize = 1040;
/// A ray whose direction has less than this component along the surface normal only grazes it.
const GRAZE: f64 = 1e-7;

fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Forward,
    Reverse,
    Both,
}

/// One face the curve is projected onto.
struct Target<'a> {
    body: &'a Body,
    face: FaceId,
    /// Box of the trimmed face (padded): rays that miss it are not intersected with the surface.
    bbox: Aabb,
}

impl<'a> Target<'a> {
    fn new(body: &'a Body, face: FaceId) -> Target<'a> {
        Target { body, face, bbox: face_box(body, face).padded(1e-4) }
    }
}

struct Caster<'a> {
    targets: Vec<Target<'a>>,
    normal: Vec3,
    mode: Mode,
    big: f64,
    cache: LoopCache,
}

#[derive(Clone, Copy)]
struct Hit {
    p: Vec3,
    face: usize,
}

impl Caster<'_> {
    /// Every crossing of the line through `p` along the normal with target `ti`, inside the trimmed face.
    fn hits_on(&mut self, ti: usize, p: Vec3, out: &mut Vec<Hit>) {
        let (lo, hi) = match self.mode {
            Mode::Forward => (-1e-7, self.big),
            Mode::Reverse => (-self.big, 1e-7),
            Mode::Both => (-self.big, self.big),
        };
        if self.targets[ti].bbox.ray_hit(p, self.normal, lo, hi).is_none() {
            return;
        }
        let line = Curve::line(p, self.normal);
        let (body, face) = (self.targets[ti].body, self.targets[ti].face);
        let surf = &body.face(face).surface;
        let res = cs::intersect(&line, (lo, hi), surf, 1e-7);
        if !res.on.is_empty() {
            return;
        }
        for h in &res.hits {
            if h.t < lo - 1e-9 || h.t > hi + 1e-9 {
                continue;
            }
            let n = surf.normal(h.uv.0, h.uv.1);
            if n.dot(self.normal).abs() < GRAZE {
                continue;
            }
            if point_in_face_cached(body, face, h.p, 1e-5, &mut self.cache) == Where::Out {
                continue;
            }
            out.push(Hit { p: h.p, face: ti });
        }
    }

    fn hits(&mut self, p: Vec3) -> Vec<Hit> {
        let mut out = vec![];
        for ti in 0..self.targets.len() {
            self.hits_on(ti, p, &mut out);
        }
        out
    }

    /// Walk from the source point `good` (where the track at `q` exists) toward `bad` (where it does not) and
    /// return the last point of the track found: the boundary of the face / silhouette to ~1e-13 of the segment.
    /// `others` are the other hits of the `good` sample on this face (other sheets): a candidate nearer to one
    /// of them than to the track is that sheet, not the track.
    fn bisect(&mut self, ti: usize, q: Vec3, others: &[Vec3], good: Vec3, bad: Vec3) -> Option<(Vec3, f64)> {
        let (mut lo, mut hi) = (0.0f64, 1.0f64);
        let mut best: Option<(Vec3, f64)> = None;
        let mut q = q;
        let mut buf = vec![];
        for _ in 0..60 {
            if hi - lo < 1e-13 {
                break;
            }
            let m = 0.5 * (lo + hi);
            buf.clear();
            self.hits_on(ti, good + (bad - good) * m, &mut buf);
            let near = buf
                .iter()
                .map(|h| (h.p.dist(q), h.p))
                .filter(|(d, p)| others.iter().all(|o| *d <= p.dist(*o)))
                .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            match near {
                Some((_, p)) => {
                    lo = m;
                    best = Some((p, m));
                    q = p;
                }
                None => hi = m,
            }
        }
        best
    }
}

/// A run: consecutive samples of one entity hitting one face (sheet).
struct Track {
    pts: Vec<Vec3>,
    /// Source parameter of every point: the sample index, fractional for refined points.
    s: Vec<f64>,
    face: usize,
    first: usize,
    last: usize,
}

/// Project the dense polyline `src` of one entity: its runs.
fn project_entity(c: &mut Caster, src: &[Vec3]) -> Vec<Track> {
    let hs: Vec<Vec<Hit>> = src.iter().map(|p| c.hits(*p)).collect();
    let mut tracks: Vec<Track> = vec![];
    let mut open: Vec<usize> = vec![];
    for (i, hits) in hs.iter().enumerate() {
        // pair every open track with the nearest unused hit of its face
        let mut pairs: Vec<(f64, usize, usize)> = vec![];
        for (oi, &t) in open.iter().enumerate() {
            let q = *tracks[t].pts.last().unwrap();
            for (k, h) in hits.iter().enumerate() {
                if h.face == tracks[t].face {
                    pairs.push((h.p.dist(q), oi, k));
                }
            }
        }
        pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut used_hit = vec![false; hits.len()];
        let mut used_open = vec![false; open.len()];
        let mut next_open: Vec<usize> = vec![];
        for (_, oi, k) in pairs {
            if used_hit[k] || used_open[oi] {
                continue;
            }
            used_hit[k] = true;
            used_open[oi] = true;
            let t = open[oi];
            tracks[t].pts.push(hits[k].p);
            tracks[t].s.push(i as f64);
            tracks[t].last = i;
            next_open.push(t);
        }
        // tracks that ended between sample i-1 and i: find the exact end
        for (oi, &t) in open.iter().enumerate() {
            if used_open[oi] {
                continue;
            }
            let q = *tracks[t].pts.last().unwrap();
            let face = tracks[t].face;
            let others: Vec<Vec3> = hs[i - 1].iter().filter(|h| h.face == face && h.p.dist(q) > 1e-12).map(|h| h.p).collect();
            if let Some((e, m)) = c.bisect(face, q, &others, src[i - 1], src[i]) {
                if e.dist(q) > 1e-12 {
                    tracks[t].pts.push(e);
                    tracks[t].s.push((i - 1) as f64 + m);
                }
            }
        }
        // new tracks start at the unused hits
        for (k, h) in hits.iter().enumerate() {
            if used_hit[k] {
                continue;
            }
            let mut pts = vec![h.p];
            let mut ss = vec![i as f64];
            if i > 0 {
                let others: Vec<Vec3> = hits.iter().enumerate().filter(|(j, g)| *j != k && g.face == h.face).map(|(_, g)| g.p).collect();
                if let Some((e, m)) = c.bisect(h.face, h.p, &others, src[i], src[i - 1]) {
                    if e.dist(h.p) > 1e-12 {
                        pts.insert(0, e);
                        ss.insert(0, i as f64 - m);
                    }
                }
            }
            tracks.push(Track { pts, s: ss, face: h.face, first: i, last: i });
            next_open.push(tracks.len() - 1);
        }
        open = next_open;
    }
    // a closed entity whose run wraps around the start: join the run that ends at the last sample to the one that
    // starts at the first
    let n = src.len();
    if n > 2 && src[0].dist(src[n - 1]) < 1e-9 {
        let a = tracks.iter().position(|t| t.first == 0 && t.last + 1 != n);
        let b = tracks.iter().position(|t| t.last + 1 == n && t.first != 0);
        if let (Some(a), Some(b)) = (a, b) {
            if a != b && tracks[a].face == tracks[b].face && tracks[a].pts[0].dist(*tracks[b].pts.last().unwrap()) < 1e-6 {
                let ta = tracks.remove(a);
                let b = if b > a { b - 1 } else { b };
                let tb = &mut tracks[b];
                // the parameter continues past the last sample
                let off = (n - 1) as f64;
                tb.pts.extend_from_slice(&ta.pts[1..]);
                tb.s.extend(ta.s[1..].iter().map(|x| x + off));
                tb.last = ta.last;
            }
        }
    }
    tracks.retain(|t| t.pts.len() >= 2 && poly_len(&t.pts) > 1e-7);
    merge_folds(&mut tracks, n, src[0].dist(src[n - 1]) < 1e-9);
    for t in tracks.iter_mut() {
        refine_track(c, src, t);
    }
    tracks
}

/// Two runs of one face that meet in the same point at the same source parameter, strictly inside the entity, are
/// one stretch of the curve (two sheets passing a fold / silhouette inside the entity, or a run the sampling broke):
/// one piece. Where the meeting sits at an end of the entity the two stay two (the oracle counts them so), and a
/// closed entity (circle) keeps its sheets apart as the oracle does.
fn merge_folds(tracks: &mut Vec<Track>, n: usize, closed: bool) {
    if closed {
        return;
    }
    let interior = |s: f64| s > 1e-9 && s < (n - 1) as f64 - 1e-9;
    'again: loop {
        for i in 0..tracks.len() {
            for j in i + 1..tracks.len() {
                if tracks[i].face != tracks[j].face {
                    continue;
                }
                // (end of i, end of j): false = the first point, true = the last
                for (ei, ej) in [(true, false), (true, true), (false, false), (false, true)] {
                    let at = |t: &Track, last: bool| if last { (*t.s.last().unwrap(), *t.pts.last().unwrap()) } else { (t.s[0], t.pts[0]) };
                    let ((si, pi), (sj, pj)) = (at(&tracks[i], ei), at(&tracks[j], ej));
                    if !((si - sj).abs() < 1e-6 && interior(si) && pi.dist(pj) < JOIN) {
                        continue;
                    }
                    // result: i ending at its meeting end, then j starting at its meeting end
                    let b = tracks.remove(j);
                    let a = &mut tracks[i];
                    let (mut pts, mut ss) = (std::mem::take(&mut a.pts), std::mem::take(&mut a.s));
                    if !ei {
                        pts.reverse();
                        ss.reverse();
                    }
                    let (mut bp, mut bs) = (b.pts, b.s);
                    if ej {
                        bp.reverse();
                        bs.reverse();
                    }
                    pts.extend(bp.into_iter().skip(1));
                    ss.extend(bs.into_iter().skip(1));
                    a.pts = pts;
                    a.s = ss;
                    continue 'again;
                }
            }
        }
        break;
    }
}

/// Point of the source polyline at the (fractional) sample index `s`; indices past the end wrap (closed entities).
fn src_at(src: &[Vec3], s: f64) -> Vec3 {
    let n = src.len() - 1;
    let s = if s > n as f64 { s - n as f64 } else { s };
    let i = (s.floor() as usize).min(n - 1);
    src[i] + (src[i + 1] - src[i]) * (s - i as f64)
}

/// Even out the spacing of a run: the bisected ends can be far from the last sample (a silhouette, a boundary
/// crossed at a shallow angle) and the least-squares fit needs data all along the curve. Segments longer than
/// 2.5 times the median are halved at the source parameter, until none is left.
fn refine_track(c: &mut Caster, src: &[Vec3], t: &mut Track) {
    for _ in 0..20 {
        let mut lens: Vec<f64> = t.pts.windows(2).map(|w| w[0].dist(w[1])).collect();
        lens.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = lens[lens.len() / 2];
        let target = (2.5 * median).max(1e-9);
        let mut pts = Vec::with_capacity(t.pts.len() + 8);
        let mut ss = Vec::with_capacity(t.pts.len() + 8);
        let mut added = false;
        let mut buf = vec![];
        for k in 0..t.pts.len() {
            if k > 0 {
                let (a, b) = (t.pts[k - 1], t.pts[k]);
                let len = a.dist(b);
                if len > target && (t.s[k] - t.s[k - 1]).abs() > 1e-11 && pts.len() < 6000 {
                    let sm = 0.5 * (t.s[k] + t.s[k - 1]);
                    let mid = (a + b) * 0.5;
                    buf.clear();
                    c.hits_on(t.face, src_at(src, sm), &mut buf);
                    let near = buf.iter().map(|h| (h.p.dist(mid), h.p)).min_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
                    if let Some((d, p)) = near {
                        if d < 0.6 * len {
                            pts.push(p);
                            ss.push(sm);
                            added = true;
                        }
                    }
                }
            }
            pts.push(t.pts[k]);
            ss.push(t.s[k]);
        }
        t.pts = pts;
        t.s = ss;
        if !added {
            break;
        }
    }
}

/// Runs joined end to end. `closed`: the ends of the chain meet.
struct Chain {
    runs: Vec<Vec<Vec3>>,
    closed: bool,
}

/// Outward direction of the end of a run (`tail`: its last point, else its first), measured over a micro-metre.
fn end_dir(r: &[Vec3], tail: bool) -> Vec3 {
    let n = r.len();
    let e = if tail { r[n - 1] } else { r[0] };
    for k in 1..n {
        let p = if tail { r[n - 1 - k] } else { r[k] };
        if e.dist(p) > 1e-6 {
            return (e - p).try_normalize().unwrap_or(Vec3::ZERO);
        }
    }
    Vec3::ZERO
}

fn chain_up(mut runs: Vec<Vec<Vec3>>) -> Vec<Chain> {
    let mut out = vec![];
    while !runs.is_empty() {
        let mut chain = vec![runs.remove(0)];
        let mut closed = chain[0][0].dist(*chain[0].last().unwrap()) < JOIN;
        while !closed {
            let tail = *chain.last().unwrap().last().unwrap();
            let head = chain[0][0];
            let tdir = end_dir(chain.last().unwrap(), true);
            let hdir = end_dir(&chain[0], false);
            // candidates (smoothness, distance, run, reverse, at_tail); several runs meeting in one point (a sketch
            // corner on the silhouette of a surface) are paired by the smoothest continuation
            let mut best: Option<(f64, f64, usize, bool, bool)> = None;
            let mut cands = 0;
            for (i, r) in runs.iter().enumerate() {
                let (a, b) = (r[0], *r.last().unwrap());
                let (da, db) = (end_dir(r, false), end_dir(r, true));
                // the outward direction of the chain end is opposite to the outward direction of the run end it meets
                for (d, rev, at_tail, sm) in [
                    (tail.dist(a), false, true, -tdir.dot(da)),
                    (tail.dist(b), true, true, -tdir.dot(db)),
                    (head.dist(b), false, false, -hdir.dot(db)),
                    (head.dist(a), true, false, -hdir.dot(da)),
                ] {
                    if d < JOIN {
                        cands += 1;
                        let better = best.map_or(true, |x| if (x.0 - sm).abs() > 1e-6 { sm > x.0 } else { d < x.1 });
                        if better {
                            best = Some((sm, d, i, rev, at_tail));
                        }
                    }
                }
            }
            let _ = cands;
            let Some((_, _, i, rev, at_tail)) = best else { break };
            let mut r = runs.remove(i);
            if rev {
                r.reverse();
            }
            if at_tail {
                chain.push(r);
            } else {
                chain.insert(0, r);
            }
            closed = chain[0][0].dist(*chain.last().unwrap().last().unwrap()) < JOIN;
        }
        out.push(Chain { runs: chain, closed });
    }
    out
}

fn poly_len(pts: &[Vec3]) -> f64 {
    pts.windows(2).map(|w| w[0].dist(w[1])).sum()
}

/// Bring a sample polyline to 800..1040 samples: dense enough for the face tests and the fit, small enough to cast.
fn normalise_samples(pts: Vec<Vec3>) -> Vec<Vec3> {
    let mut v: Vec<Vec3> = Vec::with_capacity(pts.len());
    for p in pts {
        if v.last().map_or(true, |q: &Vec3| q.dist(p) > 1e-12) {
            v.push(p);
        }
    }
    if v.len() < 2 {
        return v;
    }
    if v.len() > DENSE_MAX {
        let k = v.len().div_ceil(DENSE_MAX - 8).max(2);
        let last = *v.last().unwrap();
        let mut t: Vec<Vec3> = v.iter().step_by(k).copied().collect();
        if t.last().map_or(true, |q| q.dist(last) > 1e-12) {
            t.push(last);
        }
        return t;
    }
    if v.len() < DENSE_MIN {
        let k = DENSE_MIN.div_ceil(v.len() - 1);
        let mut t = Vec::with_capacity((v.len() - 1) * k + 1);
        for w in v.windows(2) {
            for j in 0..k {
                t.push(w[0] + (w[1] - w[0]) * (j as f64 / k as f64));
            }
        }
        t.push(*v.last().unwrap());
        return t;
    }
    v
}

/// Is every sample within 1e-4 of one plane? Returns its unit normal (largest component positive); `None` for
/// points that are not coplanar or that lie on a line (no plane to speak of).
fn plane_normal(pts: &[Vec3]) -> Option<Vec3> {
    if pts.len() < 3 {
        return None;
    }
    let o = pts.iter().fold(Vec3::ZERO, |a, p| a + *p) / pts.len() as f64;
    // covariance matrix, then its eigenvectors by cyclic Jacobi rotations
    let mut a = [[0.0f64; 3]; 3];
    for p in pts {
        let d = [p.x - o.x, p.y - o.y, p.z - o.z];
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] += d[i] * d[j];
            }
        }
    }
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..30 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-300 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = if th >= 0.0 { 1.0 } else { -1.0 } / (th.abs() + (th * th + 1.0).sqrt());
            let (c, s) = (1.0 / (t * t + 1.0).sqrt(), t / (t * t + 1.0).sqrt());
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
            for k in 0..3 {
                let (vkp, vkq) = (v[k][p], v[k][q]);
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| a[i][i].partial_cmp(&a[j][j]).unwrap_or(std::cmp::Ordering::Equal));
    let (mid, big) = (a[idx[1]][idx[1]], a[idx[2]][idx[2]]);
    // collinear: the two smaller spreads vanish against the largest
    if !(big > 1e-18) || mid < 1e-10 * big {
        return None;
    }
    let n = Vec3::new(v[0][idx[0]], v[1][idx[0]], v[2][idx[0]]).try_normalize()?;
    let max_d = pts.iter().map(|p| (*p - o).dot(n).abs()).fold(0.0, f64::max);
    if max_d > 1e-4 {
        return None;
    }
    let m = [n.x.abs(), n.y.abs(), n.z.abs()];
    let big = if m[0] >= m[1] && m[0] >= m[2] { n.x } else if m[1] >= m[2] { n.y } else { n.z };
    Some(if big < 0.0 { -n } else { n })
}

/// Largest distance from 3000 points of `c` to the data polylines.
fn curve_to_data(c: &ck_geom::nurbs::bspline::BSplineCurve, data: &[PieceIn]) -> f64 {
    let (u0, u1) = c.domain();
    let n = 3000;
    let mut worst = 0.0f64;
    for i in 0..=n {
        let q = c.eval(u0 + (u1 - u0) * i as f64 / n as f64);
        let mut best = f64::INFINITY;
        for piece in data {
            for w in piece.pts.windows(2) {
                let ab = w[1] - w[0];
                let l2 = ab.dot(ab);
                let t = if l2 > 0.0 { ((q - w[0]).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
                best = best.min((w[0] + ab * t).dist(q));
            }
        }
        worst = worst.max(best);
    }
    worst
}

/// A numeric payload field: number, `=expression`, numeric string; anything else (absent, null, text that is not
/// a number) is the default; a failing expression is an error.
fn field(state: &RegenState, f: &Feature, key: &str, default: f64) -> Result<f64, FeatureError> {
    match f.raw.get(key) {
        Some(Json::Str(s)) if !s.trim_start().starts_with('=') => Ok(s.trim().parse::<f64>().ok().filter(|v| v.is_finite()).unwrap_or(default)),
        _ => state.num(f, key, default),
    }
}

fn edit_err(e: EditError) -> FeatureError {
    match e.code {
        "editCurveFitFailed" => FeatureError::new("projectCurveFailed").with("detail", Json::Str("fit failed".into())),
        "editCurveTooManyPoints" | "editCurveTooFewPoints" | "editCurveNotChain" | "editCurveNotSmooth" => {
            FeatureError::new("projectCurveFailed").with("detail", Json::Str(e.code.into()))
        }
        code => {
            let mut fe = FeatureError::new(code);
            for (k, v) in e.params {
                fe = fe.with(k, v.parse::<f64>().map(Json::Num).unwrap_or(Json::Str(v)));
            }
            fe
        }
    }
}

/// The source: one dense polyline per entity, and the projection direction (unit).
fn source(state: &RegenState, f: &Feature) -> Result<(Vec<Vec<Vec3>>, Vec3), FeatureError> {
    let curve = f.raw.get_str("curve");
    let sketch = f.raw.get_str("sketch");
    if curve.is_none() && sketch.is_none() {
        return Err(FeatureError::new("projectCurveNoInput"));
    }
    if let Some(id) = curve {
        let c = state
            .result
            .curves
            .iter()
            .find(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id))
            .ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
        let pts_json = c.as_arr().and_then(|a| a.get(1)).and_then(|o| o.get("pts")).and_then(|p| p.as_arr());
        let Some(pts_json) = pts_json else {
            return Err(FeatureError::new(codes::MISSING_REFERENCE));
        };
        let pts: Vec<Vec3> = pts_json
            .iter()
            .filter_map(|p| {
                let a = p.as_arr()?;
                Some(Vec3::new(a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2)?.as_f64()?))
            })
            .collect();
        if pts.len() < 2 {
            return Err(FeatureError::new("projectCurveNoInput"));
        }
        let normal = plane_normal(&pts).ok_or_else(|| FeatureError::new("projectCurvePlane"))?;
        return Ok((vec![normalise_samples(pts)], normal));
    }
    let sk = state.sketch(sketch.unwrap()).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let normal: Frame = sk.frame;
    let mut ents = vec![];
    for e in &sk.sketch.entities {
        if e.construction() {
            continue;
        }
        if let Some(pts) = sketch_entity_samples(sk, e.id()) {
            let pts = normalise_samples(pts);
            if pts.len() >= 2 {
                ents.push(pts);
            }
        }
    }
    if ents.is_empty() {
        return Err(FeatureError::new("projectCurveNoInput"));
    }
    let n = normal.z.try_normalize().ok_or_else(|| FeatureError::new("projectCurveNoInput"))?;
    Ok((ents, n))
}

pub fn exec_project_curve(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_none() && br.surfs.is_empty() {
        return Err(FeatureError::new("projectCurveNoFaces"));
    }
    let (ents, normal) = source(state, f)?;

    let degree = field(state, f, "degree", 3.0)?;
    let degree = degree.round().max(0.0) as usize;
    if !(2..=9).contains(&degree) {
        return Err(FeatureError::new("editCurveDegree").with("min", Json::Num(2.0)).with("max", Json::Num(9.0)));
    }
    let tolerance = field(state, f, "tolerance", 0.001)?;
    if !(tolerance > 0.0) {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    let mode = match f.raw.get("direction") {
        None | Some(Json::Null) => Mode::Both,
        Some(Json::Str(s)) => match s.as_str() {
            "both" => Mode::Both,
            "forward" => Mode::Forward,
            "reverse" => Mode::Reverse,
            _ => return Err(FeatureError::new(codes::INVALID_VALUE)),
        },
        Some(_) => Mode::Both,
    };

    // the faces
    let face_names: Vec<&str> = f.raw.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str()).collect()).unwrap_or_default();
    let mut targets: Vec<Target> = vec![];
    if face_names.is_empty() {
        for body in br.body.iter().chain(br.surfs.iter().map(|s| &s.1)) {
            targets.extend(body.face_ids().map(|face| Target::new(body, face)));
        }
    } else {
        for name in face_names {
            let slot = br.slot_of_face(name).ok_or_else(|| br.missing_face(name))?;
            let body = br.slot_body(slot).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
            let face = body.find_face(name).ok_or_else(|| crate::refs::missing_face([body], name))?;
            targets.push(Target::new(body, face));
        }
    }
    if targets.is_empty() {
        return Err(FeatureError::new("projectCurveNoFaces"));
    }

    // a ray long enough to cross everything from any source point
    let (mut lo, mut hi) = (Vec3::new(f64::MAX, f64::MAX, f64::MAX), Vec3::new(f64::MIN, f64::MIN, f64::MIN));
    for t in &targets {
        let (a, b) = (t.bbox.min, t.bbox.max);
        lo = Vec3::new(lo.x.min(a.x), lo.y.min(a.y), lo.z.min(a.z));
        hi = Vec3::new(hi.x.max(b.x), hi.y.max(b.y), hi.z.max(b.z));
    }
    let centre = (lo + hi) * 0.5;
    let src_far = ents.iter().flat_map(|e| e.iter()).map(|p| p.dist(centre)).fold(0.0, f64::max);
    let big = 2.0 * ((hi - lo).len() + src_far) + 1.0;

    let mut caster = Caster { targets, normal, mode, big, cache: LoopCache::new() };
    let mut runs: Vec<Vec<Vec3>> = vec![];
    for e in &ents {
        runs.extend(project_entity(&mut caster, e).into_iter().map(|t| t.pts));
    }
    if runs.is_empty() {
        return Err(FeatureError::new("projectCurveMisses"));
    }
    let pieces = runs.len();
    let mut chains = chain_up(runs);
    if chains.len() > 1 {
        let points: Vec<Json> = chains.iter().filter(|c| !c.closed).flat_map(|c| [c.runs[0][0], *c.runs.last().unwrap().last().unwrap()]).map(v3).collect();
        return Err(FeatureError::new("projectCurveSeveral").with("count", Json::Num(chains.len() as f64)).with("points", Json::Arr(points)));
    }
    let chain = chains.pop().unwrap();

    // the joints are exact for the fit: a gap below the join distance is closed by moving the start of the next run
    let mut runs = chain.runs;
    for i in 1..runs.len() {
        runs[i][0] = *runs[i - 1].last().unwrap();
    }
    if chain.closed {
        let first = runs[0][0];
        *runs.last_mut().unwrap().last_mut().unwrap() = first;
    }
    let length: f64 = runs.iter().map(|r| poly_len(r)).sum();
    let opts = EditOpts {
        degree,
        by_tolerance: true,
        count: degree + 1,
        tolerance,
        keep_ends: true,
        keep_tangents: false,
        keep_corners: true,
        entries: vec![],
        plane: None,
        planar: false,
    };
    let pieces_in: Vec<PieceIn> = runs.into_iter().map(|pts| PieceIn { pts }).collect();
    let mut opts = opts;
    let mut out = edit_curve(&pieces_in, &opts).map_err(edit_err)?;
    // editCurve measures the deviation at 201 places of the data; a long curve with few control points can swing
    // wider between them. Tighten the fit's tolerance (up to three times) until the curve itself, densely sampled,
    // stays within the requested tolerance of the data.
    for _ in 0..3 {
        if out.cvs.len() >= 400 || curve_to_data(&out.curve, &pieces_in) <= tolerance {
            break;
        }
        opts.tolerance *= 0.4;
        match edit_curve(&pieces_in, &opts) {
            Ok(o) => out = o,
            Err(_) => break,
        }
    }
    // display polyline: every knot span (a corner is a knot of full multiplicity) sampled on its own
    let mut ks: Vec<f64> = out.knots.clone();
    ks.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    let spans = ks.len().saturating_sub(1).max(1);
    let per = ((8 * out.cvs.len()).clamp(64, 2000)).div_ceil(spans).max(4);
    let mut pts: Vec<Vec3> = vec![out.curve.eval(ks[0])];
    for w in ks.windows(2) {
        for j in 1..=per {
            pts.push(out.curve.eval(w[0] + (w[1] - w[0]) * j as f64 / per as f64));
        }
    }

    state.result.curves.retain(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(f.id.as_str()));
    state.result.curves.push(Json::Arr(vec![
        Json::Str(f.id.clone()),
        Json::Obj(vec![
            ("pts".into(), Json::Arr(pts.iter().map(|p| v3(*p)).collect())),
            ("axis".into(), Json::Bool(false)),
            ("t0".into(), Json::Num(0.0)),
            ("t1".into(), Json::Num(1.0)),
        ]),
    ]));
    state.result.markers.retain(|m| m.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(f.id.as_str()));
    state.result.markers.push(Json::Arr(vec![
        Json::Str(f.id.clone()),
        Json::Obj(vec![
            ("degree".into(), Json::Num(degree as f64)),
            ("count".into(), Json::Num(out.cvs.len() as f64)),
            ("deviation".into(), Json::Num((out.deviation * 1e6).round() / 1e6)),
            ("length".into(), Json::Num(length)),
            ("pieces".into(), Json::Num(pieces as f64)),
            ("corners".into(), Json::Num(out.corners as f64)),
            ("closed".into(), Json::Bool(chain.closed)),
        ]),
    ]));
    Ok(())
}
