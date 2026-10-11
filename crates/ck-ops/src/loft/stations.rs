//! Stations of a loft: the profiles brought to a common number of pieces,
//! in matching order (ck-ops/src/loft/stations.rs).
//!
//! Pinned by oracle probes (random polygon / arc profile pairs, 250 cases
//! with the rails read back from the regen reply, all equal):
//!
//! * every profile is a closed chain of pieces (one per sketch entity piece);
//!   the common count is the maximum over the profiles (at least 2 for a
//!   closed chain, so a lone circle becomes two half circles);
//! * a profile with fewer pieces is split until it has that many: the LONGEST
//!   piece (arc length, first one on a tie) is bisected at its parameter
//!   midpoint, repeatedly (3 -> 6 gives cuts at 1/4, 1/2 of the first edge);
//! * a profile running the other way round (about the loft direction) than
//!   the first is reversed;
//! * the start of each profile is rotated to the cyclic shift that minimises
//!   the sum of squared distances between the piece start points of
//!   consecutive stations (equal to the centred cost when counts are equal;
//!   the first shift wins a tie; the cost is summed over WORLD-space points
//!   as dx*dx+dy*dy+dz*dz in piece order with a strict `<`, which reproduces
//!   even the float-noise decided ties of 216 symmetric polygon pairs).
//!   Stations are matched pairwise in order;
//! * the station parameter is the cumulative chord length between the
//!   centroids (= the mean of the piece start points) of the stations
//!   (exactly 0.5603921945628859 on the shifted three-square probe).

use ck_geom::curve::Curve;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::Vec3;
use std::f64::consts::FRAC_PI_2;

/// Geometry of one profile piece in world space; `at(s)`, `s` in [0, 1],
/// runs along the travel direction.
#[derive(Debug, Clone)]
pub enum Geom3 {
    Line { a: Vec3, b: Vec3 },
    /// `c + u cos t + v sin t` from `t0` to `t1` (t1 < t0 = clockwise).
    Conic { c: Vec3, u: Vec3, v: Vec3, t0: f64, t1: f64 },
    /// `curve(u)` from `u0` to `u1` (u1 < u0 = against the curve).
    Spline { curve: BSplineCurve, u0: f64, u1: f64 },
}

impl Geom3 {
    pub fn at(&self, s: f64) -> Vec3 {
        match self {
            Geom3::Line { a, b } => *a + (*b - *a) * s,
            Geom3::Conic { c, u, v, t0, t1 } => {
                let t = t0 + (t1 - t0) * s;
                *c + *u * t.cos() + *v * t.sin()
            }
            Geom3::Spline { curve, u0, u1 } => curve.eval(u0 + (u1 - u0) * s),
        }
    }
    pub fn start(&self) -> Vec3 {
        match self {
            Geom3::Line { a, .. } => *a,
            _ => self.at(0.0),
        }
    }
    pub fn end(&self) -> Vec3 {
        match self {
            Geom3::Line { b, .. } => *b,
            _ => self.at(1.0),
        }
    }
    pub fn is_line(&self) -> bool {
        matches!(self, Geom3::Line { .. })
    }
    /// Arc length.
    pub fn length(&self) -> f64 {
        match self {
            Geom3::Line { a, b } => a.dist(*b),
            Geom3::Conic { u, v, t0, t1, .. } => {
                if (u.len() - v.len()).abs() <= 1e-12 * u.len().max(1.0) && u.dot(*v).abs() <= 1e-12 * u.len2().max(1.0) {
                    u.len() * (t1 - t0).abs()
                } else {
                    self.numeric_length()
                }
            }
            Geom3::Spline { .. } => self.numeric_length(),
        }
    }
    fn numeric_length(&self) -> f64 {
        let n = 128;
        let mut l = 0.0;
        let mut p = self.at(0.0);
        for i in 1..=n {
            let q = self.at(i as f64 / n as f64);
            l += p.dist(q);
            p = q;
        }
        l
    }
    /// The part between `s0` and `s1` (in travel direction).
    pub fn sub(&self, s0: f64, s1: f64) -> Geom3 {
        match self {
            Geom3::Line { .. } => Geom3::Line { a: self.at(s0), b: self.at(s1) },
            Geom3::Conic { c, u, v, t0, t1 } => {
                Geom3::Conic { c: *c, u: *u, v: *v, t0: t0 + (t1 - t0) * s0, t1: t0 + (t1 - t0) * s1 }
            }
            Geom3::Spline { curve, u0, u1 } => {
                Geom3::Spline { curve: curve.clone(), u0: u0 + (u1 - u0) * s0, u1: u0 + (u1 - u0) * s1 }
            }
        }
    }
    pub fn reversed(&self) -> Geom3 {
        match self {
            Geom3::Line { a, b } => Geom3::Line { a: *b, b: *a },
            Geom3::Conic { c, u, v, t0, t1 } => Geom3::Conic { c: *c, u: *u, v: *v, t0: *t1, t1: *t0 },
            Geom3::Spline { curve, u0, u1 } => Geom3::Spline { curve: curve.clone(), u0: *u1, u1: *u0 },
        }
    }

    /// The piece as a rational B-spline on [0, 1] (arcs: rational
    /// quadratic, at most a quarter turn per span).
    pub fn to_nurbs(&self) -> BSplineCurve {
        match self {
            Geom3::Line { a, b } => BSplineCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![*a, *b]).unwrap(),
            Geom3::Conic { c, u, v, t0, t1 } => {
                let sweep = t1 - t0;
                let n = ((sweep.abs() / FRAC_PI_2 - 1e-9).ceil() as usize).max(1);
                let h = sweep / n as f64;
                let w1 = (h / 2.0).cos();
                let pt = |t: f64| *c + *u * t.cos() + *v * t.sin();
                let mut ctrl = vec![pt(*t0)];
                let mut w = vec![1.0];
                let mut knots = vec![0.0; 3];
                for i in 0..n {
                    let a = t0 + h * i as f64;
                    let mid = *c + (*u * (a + h / 2.0).cos() + *v * (a + h / 2.0).sin()) / w1;
                    ctrl.push(mid);
                    w.push(w1);
                    ctrl.push(pt(a + h));
                    w.push(1.0);
                    if i + 1 < n {
                        let k = (i + 1) as f64 / n as f64;
                        knots.extend([k, k]);
                    }
                }
                knots.extend([1.0; 3]);
                // exact end points
                *ctrl.last_mut().unwrap() = pt(*t1);
                BSplineCurve::new_rational(2, knots, ctrl, w).unwrap()
            }
            Geom3::Spline { curve, u0, u1 } => {
                let (lo, hi) = if u0 <= u1 { (*u0, *u1) } else { (*u1, *u0) };
                let (a, b) = curve.domain();
                let mut c = if lo > a || hi < b { curve.segment(lo.max(a), hi.min(b)) } else { curve.clone() };
                c = c.reparam(0.0, 1.0);
                if u1 < u0 {
                    c = c.reversed();
                }
                c
            }
        }
    }
}

/// A profile piece with the name of the sketch entity it comes from.
#[derive(Debug, Clone)]
pub struct Piece {
    pub tag: String,
    pub g: Geom3,
}

/// Convert a ck-geom curve trimmed to `[t0, t1]` (travelled backwards when
/// `rev`) into piece geometry.
pub fn geom_of(curve: &Curve, t0: f64, t1: f64, rev: bool) -> Option<Geom3> {
    let (a, b) = if rev { (t1, t0) } else { (t0, t1) };
    Some(match curve {
        Curve::Line { o, d } => Geom3::Line { a: *o + *d * a, b: *o + *d * b },
        Curve::Circle { frame, r } => Geom3::Conic { c: frame.o, u: frame.x * *r, v: frame.y * *r, t0: a, t1: b },
        Curve::Ellipse { frame, a: ra, b: rb } => {
            Geom3::Conic { c: frame.o, u: frame.x * *ra, v: frame.y * *rb, t0: a, t1: b }
        }
        Curve::BSpline(c) => Geom3::Spline { curve: c.clone(), u0: a, u1: b },
        _ => return None,
    })
}

/// Profile chain: pieces in travel order.
#[derive(Debug, Clone)]
pub struct Chain {
    pub closed: bool,
    pub pieces: Vec<Piece>,
}

impl Chain {
    /// Sample polygon (4 points per piece) for orientation tests.
    fn polygon(&self) -> Vec<Vec3> {
        let mut out = vec![];
        for p in &self.pieces {
            for k in 0..4 {
                out.push(p.g.at(k as f64 / 4.0));
            }
        }
        if let Some(l) = self.pieces.last() {
            out.push(l.g.end());
        }
        out
    }
    /// Newell normal of the (closed) sample polygon: twice the signed area
    /// vector.
    pub fn normal(&self) -> Vec3 {
        let pts = self.polygon();
        let mut n = Vec3::new(0.0, 0.0, 0.0);
        for i in 0..pts.len() {
            n = n + pts[i].cross(pts[(i + 1) % pts.len()]);
        }
        n
    }
    pub fn reversed(&self) -> Chain {
        Chain {
            closed: self.closed,
            pieces: self.pieces.iter().rev().map(|p| Piece { tag: p.tag.clone(), g: p.g.reversed() }).collect(),
        }
    }
    /// Bisect the longest piece (first on ties) until there are `n`.
    pub fn split_to(&mut self, n: usize) {
        while self.pieces.len() < n {
            let mut bi = 0;
            let mut bl = -1.0;
            for (i, p) in self.pieces.iter().enumerate() {
                let l = p.g.length();
                if l > bl {
                    bl = l;
                    bi = i;
                }
            }
            let p = self.pieces[bi].clone();
            let (a, b) = (p.g.sub(0.0, 0.5), p.g.sub(0.5, 1.0));
            self.pieces.splice(bi..=bi, [Piece { tag: p.tag.clone(), g: a }, Piece { tag: p.tag, g: b }]);
        }
    }
    pub fn rotated(&self, k: usize) -> Chain {
        let n = self.pieces.len();
        Chain { closed: self.closed, pieces: (0..n).map(|j| self.pieces[(j + k) % n].clone()).collect() }
    }
    pub fn centroid(&self) -> Vec3 {
        let mut c = Vec3::new(0.0, 0.0, 0.0);
        for p in &self.pieces {
            c = c + p.g.start();
        }
        c / self.pieces.len() as f64
    }
}

/// The matched stations.
#[derive(Debug, Clone)]
pub struct Stations {
    pub closed: bool,
    /// Entity name of every piece (from the first profile).
    pub tags: Vec<String>,
    pub chains: Vec<Chain>,
    /// Station parameters (cumulative centroid chord lengths).
    pub ts: Vec<f64>,
}

fn cost(a: &Chain, b: &Chain, k: usize) -> f64 {
    let n = a.pieces.len();
    (0..n).map(|j| a.pieces[j].g.start().dist2(b.pieces[(j + k) % n].g.start())).sum()
}

/// Bring the chains to common piece count and matching order.
pub fn match_chains(mut chains: Vec<Chain>) -> Stations {
    let closed = chains[0].closed;
    let mut n = chains.iter().map(|c| c.pieces.len()).max().unwrap_or(1);
    if closed && n < 2 {
        n = 2;
    }
    // directions about the loft: reverse chains running against the first
    let n0 = chains[0].normal();
    for c in chains.iter_mut().skip(1) {
        if closed && c.normal().dot(n0) < 0.0 {
            *c = c.reversed();
        }
    }
    for c in chains.iter_mut() {
        c.split_to(n);
    }
    for i in 1..chains.len() {
        if closed {
            let mut best = 0;
            let mut bv = f64::INFINITY;
            for k in 0..n {
                let c = cost(&chains[i - 1], &chains[i], k);
                if c < bv {
                    bv = c;
                    best = k;
                }
            }
            chains[i] = chains[i].rotated(best);
        } else {
            let fwd = cost(&chains[i - 1], &chains[i], 0);
            let r = chains[i].reversed();
            if cost(&chains[i - 1], &r, 0) < fwd {
                chains[i] = r;
            }
        }
    }
    let tags = chains[0].pieces.iter().map(|p| p.tag.clone()).collect();
    let cs: Vec<Vec3> = chains.iter().map(|c| c.centroid()).collect();
    let mut ts = vec![0.0];
    for i in 1..cs.len() {
        let t = ts[i - 1] + cs[i - 1].dist(cs[i]);
        ts.push(t);
    }
    Stations { closed, tags, chains, ts }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn poly(pts: &[[f64; 2]], y: f64) -> Chain {
        let n = pts.len();
        Chain {
            closed: true,
            pieces: (0..n)
                .map(|i| Piece {
                    tag: format!("e{}", i + 1),
                    g: Geom3::Line {
                        a: Vec3::new(pts[i][0], y, -pts[i][1]),
                        b: Vec3::new(pts[(i + 1) % n][0], y, -pts[(i + 1) % n][1]),
                    },
                })
                .collect(),
        }
    }

    /// Oracle (tri -> pentagon probe): the triangle's e1 and e3 are cut at
    /// their midpoints, e2 is not.
    #[test]
    fn triangle_to_pentagon() {
        let tri = poly(&[[10.0, 0.0], [-5.0, 8.660254037844386], [-5.0, -8.660254037844386]], 0.0);
        let pent: Vec<[f64; 2]> = (0..5)
            .map(|i| {
                let a = 2.0 * std::f64::consts::PI * i as f64 / 5.0;
                [6.0 * a.cos(), 6.0 * a.sin()]
            })
            .collect();
        let st = match_chains(vec![tri, poly(&pent, 30.0)]);
        let s: Vec<(f64, f64)> = st.chains[0].pieces.iter().map(|p| (p.g.start().x, -p.g.start().z)).collect();
        assert_eq!(s.len(), 5);
        assert!((s[1].0 - 2.5).abs() < 1e-12 && (s[1].1 - 4.330127018922193).abs() < 1e-12, "{s:?}");
        assert!((s[4].0 - 2.5).abs() < 1e-12 && (s[4].1 + 4.330127018922193).abs() < 1e-12, "{s:?}");
    }

    #[test]
    fn circle_splits_in_two() {
        let g = Geom3::Conic { c: Vec3::new(0.0, 0.0, 0.0), u: Vec3::new(8.0, 0.0, 0.0), v: Vec3::new(0.0, 0.0, -8.0), t0: 0.0, t1: std::f64::consts::TAU };
        let c = Chain { closed: true, pieces: vec![Piece { tag: "e1".into(), g }] };
        let st = match_chains(vec![c.clone(), c]);
        assert_eq!(st.chains[0].pieces.len(), 2);
    }
}
