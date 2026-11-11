//! Guide curves of a sweep (ck-ops/src/guide.rs): the profile follows guide
//! curves while it travels along the path.
//!
//! Codes: `sweepGuidesCount` (more than two guides), `sweepGuideStretchTwo`
//! (`stretch` needs exactly two), `sweepGuideNotTouching` (a guide does not
//! touch the profile), `sweepGuideNotAlong` (it does not run along the path),
//! `sweepGuideShort` (it ends before the path does), `sweepGuidesSamePoint`,
//! `sweepGuideDegenerate`, `sweepGuideOnPath` (the contact lies on the path).
//!
//! Oracle probes (docs/ck-ops-stubs.md, tools/stub_cases.mjs `sweepGuide*`),
//! circle r=3 swept 20 along a line, guide from (3,0) to (6,20):
//! * `scale` (default, "scale and turn"): the section is scaled by the ratio
//!   of the guide's distance from the path to the profile contact's, and
//!   turned about the path so that the contact stays on the guide: a cone
//!   frustum, volume 1319.47 (closed form); a guide that shrinks, curves or
//!   runs along an arc path is followed the same way;
//! * `turn`: only the turning, a circle keeps its shape: volume 565.49;
//! * `stretch`: two guides; the section is stretched between them (volume
//!   848.23 for guides at x = +3 -> 6 and -3 -> -6; 753.98 for 6 / -4: the
//!   centre moves to the middle of the two guide points);
//! * `scale` with two guides: uniform scale by the ratio of the guide
//!   distances (1167.63 for 3 -> 6 and -3 -> -5, i.e. radius 5.5);
//!   `turn` with two guides: 565.49;
//! * a closed profile of one entity is cut into halves `S.e1~1` / `~2` like
//!   every general sweep (4 faces / 6 edges for a circle), a square is not
//!   split by a guide touching the middle of an edge (6 / 12);
//! * with `twist`: `sweepGuidesTwist` (checked by the feature).
//!
//! Construction (RECONSTRUCTED): the section carried along the path by
//! rotation-minimizing frames (`sweep::rmf_rows`) is mapped inside its plane
//! so that its contact points land on the guides; the faces are the
//! interpolated surfaces of `sweep::sweep_rows_general`. The guide point of a
//! row is where the guide crosses that row's section plane. The oracle ends
//! a guide that stops before the path in an odd way (probe
//! `sweepGuideShortG`, volume 538); here that is `sweepGuideShort`.

use crate::sweep::{path_samples, profile_ref_point, rmf_rows, sweep_rows_general, PathSeg};
use ck_core::xform::Xform;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::Body;
use ck_topo::make::Profile;

pub const GUIDES_COUNT: &str = "sweepGuidesCount";
pub const STRETCH_TWO: &str = "sweepGuideStretchTwo";
pub const NOT_TOUCHING: &str = "sweepGuideNotTouching";
pub const NOT_ALONG: &str = "sweepGuideNotAlong";
pub const SHORT: &str = "sweepGuideShort";
pub const SAME_POINT: &str = "sweepGuidesSamePoint";
pub const DEGENERATE: &str = "sweepGuideDegenerate";
pub const ON_PATH: &str = "sweepGuideOnPath";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuideMode {
    Scale,
    Turn,
    Stretch,
}

impl GuideMode {
    pub fn parse(s: Option<&str>) -> GuideMode {
        match s {
            Some("turn") => GuideMode::Turn,
            Some("stretch") => GuideMode::Stretch,
            _ => GuideMode::Scale,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct GuideError {
    pub code: String,
    pub detail: Option<String>,
}

fn err(code: &str, detail: &str) -> GuideError {
    GuideError { code: code.into(), detail: (!detail.is_empty()).then(|| detail.into()) }
}

/// Polyline helper with cumulative arc length.
struct Poly {
    p: Vec<Vec3>,
    s: Vec<f64>,
}

impl Poly {
    fn new(p: Vec<Vec3>) -> Poly {
        let mut s = vec![0.0];
        for w in p.windows(2) {
            s.push(s.last().unwrap() + w[0].dist(w[1]));
        }
        Poly { p, s }
    }
    fn len(&self) -> f64 {
        *self.s.last().unwrap()
    }
    fn reversed(&self) -> Poly {
        Poly::new(self.p.iter().rev().copied().collect())
    }
    fn at(&self, l: f64) -> Vec3 {
        let l = l.clamp(0.0, self.len());
        let i = match self.s.binary_search_by(|x| x.partial_cmp(&l).unwrap()) {
            Ok(i) => i.min(self.p.len() - 2),
            Err(i) => (i - 1).min(self.p.len() - 2),
        };
        let d = self.s[i + 1] - self.s[i];
        let t = if d > 0.0 { (l - self.s[i]) / d } else { 0.0 };
        self.p[i] + (self.p[i + 1] - self.p[i]) * t
    }
    /// First arc length `>= from` at which the guide crosses the plane (`o`, `n`).
    fn crossing(&self, o: Vec3, n: Vec3, from: f64) -> Option<f64> {
        let f = |p: Vec3| (p - o).dot(n);
        for i in 0..self.p.len() - 1 {
            if self.s[i + 1] < from - 1e-12 {
                continue;
            }
            let (a, b) = (f(self.p[i]), f(self.p[i + 1]));
            let scale = 1e-12 * (1.0 + o.len());
            let hit = (a.abs() <= scale) || (a < 0.0) != (b < 0.0) && (b - a).abs() > 0.0;
            if hit {
                let t = if a.abs() <= scale { 0.0 } else { a / (a - b) };
                let l = self.s[i] + t * (self.s[i + 1] - self.s[i]);
                if l >= from - 1e-9 {
                    return Some(l.max(from));
                }
            }
        }
        None
    }
}

/// The profile point nearest to `q` (boundary of every loop) and its distance.
fn nearest_on_profile(profile: &Profile, q: Vec3) -> (Vec3, f64) {
    let pf = &profile.frame;
    let world = |p: Vec2| pf.o + pf.x * p.x + pf.y * p.y;
    let mut best = (Vec3::ZERO, f64::INFINITY);
    for r in &profile.regions {
        for l in r.loops() {
            for s in &l.segs {
                let eval = |t: f64| world(s.curve.eval(t));
                let n = 256;
                let mut bt = s.t0;
                let mut bd = f64::INFINITY;
                for k in 0..=n {
                    let t = s.t0 + (s.t1 - s.t0) * k as f64 / n as f64;
                    let d = eval(t).dist(q);
                    if d < bd {
                        bd = d;
                        bt = t;
                    }
                }
                let step = (s.t1 - s.t0) / n as f64;
                let (mut lo, mut hi) = (bt - step.abs(), bt + step.abs());
                let (lo0, hi0) = (s.t0.min(s.t1), s.t0.max(s.t1));
                lo = lo.max(lo0);
                hi = hi.min(hi0);
                for _ in 0..60 {
                    let (a, b) = (lo + (hi - lo) * 0.381966, hi - (hi - lo) * 0.381966);
                    if eval(a).dist(q) < eval(b).dist(q) {
                        hi = b;
                    } else {
                        lo = a;
                    }
                }
                let p = eval(0.5 * (lo + hi));
                if p.dist(q) < best.1 {
                    best = (p, p.dist(q));
                }
            }
        }
    }
    best
}

/// A guide with its contact on the profile, running away from it.
struct Contact {
    poly: Poly,
    q0: Vec3,
}

/// Sweep `profile` along `path` while it follows `guides` (dense polylines).
/// The swept body and the contact point of every guide on the profile (the `guides` marker).
pub struct Guided {
    pub body: Body,
    pub contacts: Vec<Vec3>,
}

pub fn guided_sweep(feature: &str, profile: &Profile, path: &[PathSeg], guides: &[Vec<Vec3>], mode: GuideMode) -> Result<Guided, GuideError> {
    if guides.len() > 2 {
        return Err(err(GUIDES_COUNT, ""));
    }
    if mode == GuideMode::Stretch && guides.len() != 2 {
        return Err(err(STRETCH_TWO, ""));
    }
    if guides.is_empty() {
        return Err(err(GUIDES_COUNT, ""));
    }
    profile.validate().map_err(|e| err(crate::sweep::FAILED, &e))?;
    if path.is_empty() {
        return Err(err(crate::sweep::PATH_INVALID, ""));
    }
    let size = profile
        .regions
        .iter()
        .flat_map(|r| r.loops())
        .flat_map(|l| l.segs.iter())
        .map(|s| s.curve.eval(s.t0).len().max(s.curve.eval(s.t1).len()))
        .fold(1.0f64, f64::max);
    let touch_tol = 1e-4 * size.max(1.0);
    // ---- orient the path as the plain sweep does
    let q = profile_ref_point(profile);
    let (ps, pe) = (path[0].start(), path[path.len() - 1].end());
    let oriented: Vec<PathSeg> = if pe.dist(q) < ps.dist(q) - 1e-9 { path.iter().rev().map(|s| s.reversed()).collect() } else { path.to_vec() };
    let t0 = oriented[0].start_dir().ok_or_else(|| err(crate::sweep::PATH_INVALID, ""))?;
    if t0.dot(profile.frame.z).abs() < 0.9659258262890683 {
        return Err(err(crate::sweep::PROFILE_NOT_NORMAL, ""));
    }
    let n_seg = if oriented.len() == 1 { 96 } else { 48 };
    let (pts, tans) = path_samples(&oriented, n_seg).ok_or_else(|| err(crate::sweep::PATH_INVALID, ""))?;
    let rows: Vec<Xform> = rmf_rows(&pts, &tans);
    let mut phis = vec![0.0];
    for w in pts.windows(2) {
        phis.push(phis.last().unwrap() + w[0].dist(w[1]));
    }
    let nrows = rows.len();
    // ---- contacts
    let pf = &profile.frame;
    let mut contacts: Vec<Contact> = vec![];
    for g in guides {
        if g.len() < 2 {
            return Err(err(DEGENERATE, "the guide is not a curve"));
        }
        let mut poly = Poly::new(g.clone());
        if !(poly.len() > 1e-9) {
            return Err(err(DEGENERATE, "the guide has no length"));
        }
        let (qa, da) = nearest_on_profile(profile, poly.p[0]);
        let (qb, db) = nearest_on_profile(profile, *poly.p.last().unwrap());
        let (q0, d) = if da <= db { (qa, da) } else { poly = poly.reversed(); (qb, db) };
        if d > touch_tol {
            return Err(err(NOT_TOUCHING, ""));
        }
        contacts.push(Contact { poly, q0 });
    }
    if contacts.len() == 2 && contacts[0].q0.dist(contacts[1].q0) < 1e-6 * size {
        return Err(err(SAME_POINT, ""));
    }
    // ---- per row: the guide points in the row's section plane
    let c0 = pts[0] - pf.z * (pts[0] - pf.o).dot(pf.z); // path point projected on the profile plane... (below: centre)
    let _ = c0;
    // section plane of row i: through M_i(pf.o), normal M_i(pf.z)
    let plane = |i: usize| -> (Vec3, Vec3) {
        let o = rows[i].apply_point(pf.o);
        let n = rows[i].apply_vec(pf.z).normalize();
        (o, n)
    };
    // centre of the section: the path point projected onto the section plane
    let centre = |i: usize| -> Vec3 {
        let (o, n) = plane(i);
        pts[i] - n * (pts[i] - o).dot(n)
    };
    let mut gpts: Vec<Vec<Vec3>> = vec![vec![]; contacts.len()];
    for (j, c) in contacts.iter().enumerate() {
        let mut l = 0.0;
        for i in 0..nrows {
            let (o, n) = plane(i);
            let Some(l_i) = c.poly.crossing(o, n, l) else {
                // does the guide move along the path at all?
                let along = (c.poly.p[c.poly.p.len() - 1] - c.poly.p[0]).dot(n) > 1e-9;
                return Err(err(if along { SHORT } else { NOT_ALONG }, ""));
            };
            l = l_i;
            gpts[j].push(c.poly.at(l_i));
        }
    }
    // ---- the in-plane map of every row
    let c_0 = centre(0);
    let (_, n0) = plane(0);
    let in_plane = |v: Vec3, n: Vec3| v - n * v.dot(n);
    // rotation about n by the angle taking a to b (both in the plane)
    let angle = |a: Vec3, b: Vec3, n: Vec3| -> f64 { n.dot(a.cross(b)).atan2(a.dot(b)) };
    let rot = |v: Vec3, n: Vec3, ang: f64| -> Vec3 { v * ang.cos() + n.cross(v) * ang.sin() + n * (n.dot(v) * (1.0 - ang.cos())) };
    struct RowMap {
        n: Vec3,
        c: Vec3,    // centre the plane map works about (in the plane)
        cx: Vec3,   // image of the contact centre
        ang: f64,
        s: f64,
        k_dir: Option<(Vec3, f64)>, // stretch direction (in the row plane after turning) and factor
    }
    let mut maps: Vec<RowMap> = vec![];
    let r0 = (contacts[0].q0 - c_0).len();
    for i in 0..nrows {
        let (_, n) = plane(i);
        let ci = centre(i);
        let m = &rows[i];
        if contacts.len() == 1 {
            if r0 < 1e-9 * size {
                return Err(err(ON_PATH, ""));
            }
            let xq = m.apply_point(contacts[0].q0);
            let a = in_plane(xq - ci, n);
            let b = gpts[0][i] - ci;
            let b = in_plane(b, n);
            if b.len() < 1e-12 {
                return Err(err(ON_PATH, ""));
            }
            let s = if mode == GuideMode::Turn { 1.0 } else { b.len() / r0 };
            maps.push(RowMap { n, c: ci, cx: ci, ang: angle(a, b, n), s, k_dir: None });
        } else {
            // two guides
            let (q1, q2) = (contacts[0].q0, contacts[1].q0);
            let (x1, x2) = (m.apply_point(q1), m.apply_point(q2));
            let (g1, g2) = (gpts[0][i], gpts[1][i]);
            let cx = (x1 + x2) * 0.5;
            let ct = (g1 + g2) * 0.5;
            let d_rigid = in_plane(x1 - x2, n);
            let d_goal = in_plane(g1 - g2, n);
            if d_goal.len() < 1e-12 || d_rigid.len() < 1e-12 {
                return Err(err(SAME_POINT, ""));
            }
            let l0 = q1.dist(q2);
            let ang = angle(d_rigid, d_goal, n);
            let k = d_goal.len() / l0;
            match mode {
                GuideMode::Stretch => maps.push(RowMap { n, c: ct, cx, ang, s: 1.0, k_dir: Some((d_goal.normalize(), k)) }),
                GuideMode::Scale => maps.push(RowMap { n, c: ct, cx, ang, s: k, k_dir: None }),
                GuideMode::Turn => maps.push(RowMap { n, c: cx, cx, ang, s: 1.0, k_dir: None }),
            }
        }
    }
    let _ = n0;
    let apply = |i: usize, q: Vec3| -> Vec3 {
        let x = rows[i].apply_point(q);
        let mp = &maps[i];
        // split into the in-plane part about the contact centre and the height above the plane
        let h = (x - mp.cx).dot(mp.n);
        let u = in_plane(x - mp.cx, mp.n);
        let mut v = rot(u, mp.n, mp.ang);
        v = match mp.k_dir {
            Some((d, k)) => v + d * (v.dot(d) * (k - 1.0)),
            None => v * mp.s,
        };
        mp.c + v + mp.n * h
    };
    let _ = Vec2::ZERO;
    let body = sweep_rows_general(feature, profile, &phis, &apply).map_err(|e| err(&e, ""))?;
    Ok(Guided { body, contacts: contacts.iter().map(|c| c.q0).collect() })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::Frame;
    use ck_topo::make::{ProfileLoop, ProfileRegion, ProfileSeg};
    use std::f64::consts::PI;

    fn top() -> Frame {
        Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
    }

    fn circle(r: f64) -> Profile {
        let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(0.0, 0.0), r)]);
        Profile::new(top(), vec![ProfileRegion { outer: lp, holes: vec![] }])
    }

    fn square(h: f64) -> Profile {
        let p = [Vec2::new(-h, -h), Vec2::new(h, -h), Vec2::new(h, h), Vec2::new(-h, h)];
        let tags = ["e1", "e2", "e3", "e4"];
        let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
        Profile::new(top(), vec![ProfileRegion { outer: lp, holes: vec![] }])
    }

    fn path(len: f64) -> Vec<PathSeg> {
        vec![PathSeg::Line { a: Vec3::ZERO, b: Vec3::new(0.0, len, 0.0) }]
    }

    fn line(a: (f64, f64), b: (f64, f64)) -> Vec<Vec3> {
        (0..=20).map(|i| { let t = i as f64 / 20.0; Vec3::new(a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t, 0.0) }).collect()
    }

    fn vol(b: &Guided) -> f64 {
        ck_topo::mass::mass_props(&b.body).volume
    }

    /// oracle sweepGuideScale / Turn / Stretch2 / Asym / 2Scale / Shrink / square, closed forms
    #[test]
    fn scale_turn_stretch_match_the_oracle() {
        let p = circle(3.0);
        let g = line((3.0, 0.0), (6.0, 20.0));
        let b = guided_sweep("F5", &p, &path(20.0), &[g.clone()], GuideMode::Scale).unwrap();
        assert_eq!(b.body.num_faces(), 4);
        assert!(ck_topo::check::check(&b.body, 1e-6).is_empty());
        assert!((vol(&b) - PI * 20.0 / 3.0 * 63.0).abs() < 2e-3 * 1319.47, "{}", vol(&b));
        let b = guided_sweep("F5", &p, &path(20.0), &[g.clone()], GuideMode::Turn).unwrap();
        assert!((vol(&b) - 565.48668).abs() < 1e-2, "{}", vol(&b));
        let g2 = line((-3.0, 0.0), (-6.0, 20.0));
        let b = guided_sweep("F5", &p, &path(20.0), &[g.clone(), g2], GuideMode::Stretch).unwrap();
        assert!((vol(&b) - 848.23002).abs() < 1e-2 * 8.0, "{}", vol(&b));
        let g2 = line((-3.0, 0.0), (-4.0, 20.0));
        let b = guided_sweep("F5", &p, &path(20.0), &[g.clone(), g2], GuideMode::Stretch).unwrap();
        assert!((vol(&b) - 753.98224).abs() < 1e-2 * 7.5, "{}", vol(&b));
        let g2 = line((-3.0, 0.0), (-5.0, 20.0));
        let b = guided_sweep("F5", &p, &path(20.0), &[g.clone(), g2], GuideMode::Scale).unwrap();
        assert!((vol(&b) - 1167.62527).abs() < 1e-3 * 1167.6, "{}", vol(&b));
        let b = guided_sweep("F5", &p, &path(20.0), &[line((3.0, 0.0), (1.5, 20.0))], GuideMode::Scale).unwrap();
        assert!((vol(&b) - 329.86723).abs() < 1e-3 * 330.0, "{}", vol(&b));
        let b = guided_sweep("F5", &square(3.0), &path(20.0), &[g], GuideMode::Scale).unwrap();
        assert_eq!(b.body.num_faces(), 6);
        assert!((vol(&b) - 1680.0).abs() < 1e-3 * 1680.0, "{}", vol(&b));
    }

    #[test]
    fn errors() {
        let p = circle(3.0);
        let g = line((3.0, 0.0), (6.0, 20.0));
        let code = |r: Result<Guided, GuideError>| r.err().unwrap().code;
        assert_eq!(code(guided_sweep("F5", &p, &path(20.0), &[g.clone()], GuideMode::Stretch)), STRETCH_TWO);
        assert_eq!(code(guided_sweep("F5", &p, &path(20.0), &[g.clone(), g.clone(), g.clone()], GuideMode::Scale)), GUIDES_COUNT);
        assert_eq!(code(guided_sweep("F5", &p, &path(20.0), &[g.clone(), g.clone()], GuideMode::Stretch)), SAME_POINT);
        assert_eq!(code(guided_sweep("F5", &p, &path(20.0), &[line((4.0, 0.0), (6.0, 20.0))], GuideMode::Scale)), NOT_TOUCHING);
        assert_eq!(code(guided_sweep("F5", &p, &path(20.0), &[line((3.0, 0.0), (3.0, -5.0))], GuideMode::Scale)), NOT_ALONG);
        assert_eq!(code(guided_sweep("F5", &p, &path(20.0), &[line((3.0, 0.0), (5.0, 10.0))], GuideMode::Scale)), SHORT);
    }
}
