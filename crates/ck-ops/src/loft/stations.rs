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
    /// The same piece shifted by `d` (centre-line station offset).
    pub fn translated(&self, d: Vec3) -> Geom3 {
        match self {
            Geom3::Line { a, b } => Geom3::Line { a: *a + d, b: *b + d },
            Geom3::Conic { c, u, v, t0, t1 } => Geom3::Conic { c: *c + d, u: *u, v: *v, t0: *t0, t1: *t1 },
            Geom3::Spline { curve, u0, u1 } => Geom3::Spline { curve: curve.clone().map(|p| p + d), u0: *u0, u1: *u1 },
        }
    }
    /// Arc length as the kernel measures it: one 8-point Gauss-Legendre
    /// panel per knot span of the piece's B-spline (`nurbs_length`). Pinned
    /// by the `f` marker bits of square profiles (a line of length 16 comes
    /// out 16 + 1ulp noise exactly like the oracle's, 4 sub-panels do not),
    /// and by the float-noise decided choice among equal circle quarters.
    pub fn length(&self) -> f64 {
        nurbs_length(&self.to_nurbs())
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
    /// The same piece shrunk to the single point `p` (same degree, knots and
    /// weights, so it is a compatible station of a loft that ends in a point).
    pub fn collapsed(&self, p: Vec3) -> Geom3 {
        match self {
            Geom3::Line { .. } => Geom3::Line { a: p, b: p },
            _ => {
                let c = self.to_nurbs();
                let (u0, u1) = c.domain();
                Geom3::Spline { curve: c.map(|_| p), u0, u1 }
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

const GL8: [(f64, f64); 8] = [
    (-0.9602898564975363, 0.1012285362903763),
    (-0.7966664774136267, 0.2223810344533745),
    (-0.5255324099163290, 0.3137066458778873),
    (-0.1834346424956498, 0.3626837833783620),
    (0.1834346424956498, 0.3626837833783620),
    (0.5255324099163290, 0.3137066458778873),
    (0.7966664774136267, 0.2223810344533745),
    (0.9602898564975363, 0.1012285362903763),
];

/// Length of a B-spline: GL8 on every knot span, summed in span order.
pub fn nurbs_length(c: &BSplineCurve) -> f64 {
    let (lo, hi) = c.domain();
    let mut breaks = vec![lo];
    for k in ck_geom::nurbs::basis::distinct_knots(&c.knots) {
        if k > lo && k < hi {
            breaks.push(k);
        }
    }
    breaks.push(hi);
    let mut total = 0.0;
    for w in breaks.windows(2) {
        let (m, h) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
        for &(x, wt) in GL8.iter() {
            total += wt * h * c.derivs(m + h * x, 1)[1].len();
        }
    }
    total
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
    /// Every piece is collapsed to one point (a loft end that is an apex).
    pub fn is_point(&self) -> bool {
        let Some(first) = self.pieces.first() else { return false };
        let p0 = first.g.start();
        let tol = 1e-12 * (1.0 + p0.len());
        self.pieces.iter().all(|q| q.g.start().dist(p0) <= tol && q.g.end().dist(p0) <= tol && q.g.at(0.5).dist(p0) <= tol)
    }
    /// This chain shrunk to the point `p`, with the same pieces and names.
    pub fn collapsed(&self, p: Vec3) -> Chain {
        Chain { closed: self.closed, pieces: self.pieces.iter().map(|q| Piece { tag: q.tag.clone(), g: q.g.collapsed(p) }).collect() }
    }
    pub fn reversed(&self) -> Chain {
        Chain {
            closed: self.closed,
            pieces: self.pieces.iter().rev().map(|p| Piece { tag: p.tag.clone(), g: p.g.reversed() }).collect(),
        }
    }
    /// Every piece shifted by `d` (centre-line station offset).
    pub fn translated(&self, d: Vec3) -> Chain {
        Chain {
            closed: self.closed,
            pieces: self.pieces.iter().map(|p| Piece { tag: p.tag.clone(), g: p.g.translated(d) }).collect(),
        }
    }
    /// Bisect the longest piece (first on ties) until there are `n`.
    pub fn split_to(&mut self, n: usize) {
        while !self.pieces.is_empty() && self.pieces.len() < n {
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
    /// Alternative to [`Chain::split_to`] (retry strategy): the extra cuts are shared out over the pieces in
    /// proportion to their length (largest remainder), each piece cut into equal parameter parts.
    pub fn split_even(&mut self, n: usize) {
        let m = self.pieces.len();
        if m == 0 || m >= n {
            return;
        }
        let lens: Vec<f64> = self.pieces.iter().map(|p| p.g.length()).collect();
        let total: f64 = lens.iter().sum();
        if !(total.is_finite() && total > 0.0) {
            return self.split_to(n);
        }
        let want: Vec<f64> = lens.iter().map(|l| l / total * n as f64).collect();
        let mut cnt: Vec<usize> = want.iter().map(|w| (w.floor() as usize).max(1)).collect();
        let mut sum: usize = cnt.iter().sum();
        while sum < n {
            let mut bi = 0;
            let mut br = f64::NEG_INFINITY;
            for i in 0..m {
                let r = want[i] - cnt[i] as f64;
                if r > br {
                    br = r;
                    bi = i;
                }
            }
            cnt[bi] += 1;
            sum += 1;
        }
        while sum > n {
            let mut bi = None;
            let mut br = f64::INFINITY;
            for i in 0..m {
                let r = want[i] - cnt[i] as f64;
                if cnt[i] > 1 && r < br {
                    br = r;
                    bi = Some(i);
                }
            }
            let Some(bi) = bi else { break };
            cnt[bi] -= 1;
            sum -= 1;
        }
        let mut out = vec![];
        for (p, c) in self.pieces.iter().zip(&cnt) {
            for q in 0..*c {
                out.push(Piece { tag: p.tag.clone(), g: p.g.sub(q as f64 / *c as f64, (q + 1) as f64 / *c as f64) });
            }
        }
        self.pieces = out;
    }
    pub fn rotated(&self, k: usize) -> Chain {
        let n = self.pieces.len();
        if n == 0 {
            return self.clone();
        }
        Chain { closed: self.closed, pieces: (0..n).map(|j| self.pieces[(j + k) % n].clone()).collect() }
    }
    pub fn centroid(&self) -> Vec3 {
        let mut c = Vec3::new(0.0, 0.0, 0.0);
        for p in &self.pieces {
            c = c + p.g.start();
        }
        let mut n = self.pieces.len();
        if n == 0 {
            return c;
        }
        if !self.closed {
            if let Some(l) = self.pieces.last() {
                c = c + l.g.end();
            }
            n += 1;
        }
        c / n as f64
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
    if n == 0 || b.pieces.len() != n {
        return f64::INFINITY;
    }
    (0..n).map(|j| a.pieces[j].g.start().dist2(b.pieces[(j + k) % n].g.start())).sum()
}

/// Alternative matching rules used by the loft validation gate when the default result is not a valid solid.
/// `Default` is the pinned oracle rule; the others change one decision each.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Strategy {
    /// Added to the cost-chosen start offset of every matched closed profile.
    pub shift: usize,
    /// Invert the orientation decision of the profiles after the first.
    pub flip: bool,
    /// Share the cuts of a profile with fewer pieces over its pieces by length ([`Chain::split_even`]).
    pub even: bool,
}

impl Stations {
    /// The invariants the face construction indexes by: one common piece count, one tag per piece, one
    /// parameter per station.
    pub fn is_consistent(&self) -> bool {
        let n = self.tags.len();
        n > 0
            && self.chains.len() >= 2
            && self.ts.len() == self.chains.len()
            && self.ts.iter().all(|t| t.is_finite())
            && self.chains.iter().all(|c| c.pieces.len() == n)
    }
}

/// Bring the chains to common piece count and matching order.
pub fn match_chains(chains: Vec<Chain>) -> Stations {
    match_chains_with(chains, &[])
}

/// Cut a closed chain at arc-length fraction `f` (from its first vertex, in
/// travel direction) and start it there. GUESS: the fraction is measured on
/// the chain as given (after the orientation fix) by piece length; a point
/// inside a piece splits it at the same fraction of its parameter.
fn start_at_fraction(c: &Chain, f: f64) -> Chain {
    let f = f.rem_euclid(1.0);
    if f < 1e-12 {
        return c.clone();
    }
    let lens: Vec<f64> = c.pieces.iter().map(|p| p.g.length()).collect();
    let total: f64 = lens.iter().sum();
    let mut target = f * total;
    let mut pieces = c.pieces.clone();
    for (i, l) in lens.iter().enumerate() {
        if target <= *l * 1e-9 {
            return Chain { closed: c.closed, pieces: pieces.clone() }.rotated(i);
        }
        if target < *l * (1.0 - 1e-9) {
            let s = target / l;
            let p = pieces[i].clone();
            pieces.splice(i..=i, [Piece { tag: p.tag.clone(), g: p.g.sub(0.0, s) }, Piece { tag: p.tag, g: p.g.sub(s, 1.0) }]);
            return Chain { closed: c.closed, pieces }.rotated(i + 1);
        }
        target -= l;
    }
    c.clone()
}

/// [`match_chains`] with per-profile start connectors (`connectors[i]`: the
/// fraction of the profile's length at which it starts; `None` = chosen by
/// the cost rule). Oracle probes: square -> square with `[null, 0.25]` /
/// `[0, 0.5]` give volumes 5000 / 3000 (the second profile starts one / two
/// corners round).
pub fn match_chains_with(chains: Vec<Chain>, connectors: &[Option<f64>]) -> Stations {
    match_chains_strategy(chains, connectors, Strategy::default())
}

/// [`match_chains_with`] under an alternative [`Strategy`] (`Strategy::default()` = the pinned rule).
pub fn match_chains_strategy(mut chains: Vec<Chain>, connectors: &[Option<f64>], strat: Strategy) -> Stations {
    if chains.is_empty() {
        return Stations { closed: true, tags: vec![], chains, ts: vec![] };
    }
    let closed = chains[0].closed;
    let mut n = chains.iter().map(|c| c.pieces.len()).max().unwrap_or(1);
    if closed && n < 2 {
        n = 2;
    }
    // directions about the loft: reverse chains running against the first
    let n0 = chains.iter().find(|c| !c.is_point()).map_or(Vec3::new(0.0, 0.0, 0.0), |c| c.normal());
    for c in chains.iter_mut().skip(1) {
        if closed && !c.is_point() && (c.normal().dot(n0) < 0.0) != strat.flip {
            *c = c.reversed();
        }
    }
    let conn = |i: usize| connectors.get(i).copied().flatten().filter(|_| closed);
    for (i, c) in chains.iter_mut().enumerate() {
        if let Some(f) = conn(i) {
            *c = start_at_fraction(c, f);
            n = n.max(c.pieces.len());
        }
    }
    for c in chains.iter_mut() {
        if strat.even {
            c.split_even(n);
        } else {
            c.split_to(n);
        }
    }
    for i in 1..chains.len() {
        if conn(i).is_some() {
            continue;
        }
        if closed {
            let mut best = 0;
            let mut bv = f64::INFINITY;
            // a point station is matched to any start: keep the order
            let free = chains[i - 1].is_point() || chains[i].is_point();
            for k in (0..n).take(if free { 1 } else { n }) {
                let c = cost(&chains[i - 1], &chains[i], k);
                if c < bv {
                    bv = c;
                    best = k;
                }
            }
            if !free && n > 0 {
                best = (best + strat.shift) % n;
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

    /// Retry strategies (validation gate): `split_even` shares the cuts out by length and keeps the loop closed;
    /// every strategy gives a consistent station set; the default strategy is the pinned rule.
    #[test]
    fn strategies_stay_consistent() {
        let sq = |y: f64, h: f64| poly(&[[-h, -h], [h, -h], [h, h], [-h, h]], y);
        let tri = poly(&[[10.0, 0.0], [-5.0, 8.66], [-5.0, -8.66]], 30.0);
        let mut c = sq(0.0, 4.0);
        let len0: f64 = c.pieces.iter().map(|p| p.g.length()).sum();
        c.split_even(7);
        assert_eq!(c.pieces.len(), 7);
        let len1: f64 = c.pieces.iter().map(|p| p.g.length()).sum();
        assert!((len0 - len1).abs() < 1e-9);
        for k in 0..c.pieces.len() {
            assert!(c.pieces[k].g.end().dist(c.pieces[(k + 1) % 7].g.start()) < 1e-9);
        }
        for strat in [Strategy::default(), Strategy { shift: 1, flip: false, even: true }, Strategy { shift: 0, flip: true, even: false }, Strategy { shift: 2, flip: true, even: true }] {
            let st = match_chains_strategy(vec![sq(0.0, 4.0), tri.clone(), sq(60.0, 3.0)], &[], strat);
            assert!(st.is_consistent(), "{strat:?}");
        }
        let a = match_chains_with(vec![sq(0.0, 4.0), tri.clone()], &[]);
        let b = match_chains_strategy(vec![sq(0.0, 4.0), tri], &[], Strategy::default());
        assert_eq!(a.chains[1].pieces[0].g.start(), b.chains[1].pieces[0].g.start());
    }

    /// Degenerate chains never index out of range.
    #[test]
    fn empty_chain_is_harmless() {
        let e = Chain { closed: true, pieces: vec![] };
        assert!(!e.is_point());
        let _ = e.centroid();
        let _ = e.rotated(3);
        let mut e2 = e.clone();
        e2.split_to(4);
        e2.split_even(4);
        assert!(e2.pieces.is_empty());
        let st = match_chains(vec![e.clone(), e]);
        assert!(!st.is_consistent());
    }

    #[test]
    fn circle_splits_in_two() {
        let g = Geom3::Conic { c: Vec3::new(0.0, 0.0, 0.0), u: Vec3::new(8.0, 0.0, 0.0), v: Vec3::new(0.0, 0.0, -8.0), t0: 0.0, t1: std::f64::consts::TAU };
        let c = Chain { closed: true, pieces: vec![Piece { tag: "e1".into(), g }] };
        let st = match_chains(vec![c.clone(), c]);
        assert_eq!(st.chains[0].pieces.len(), 2);
    }
}
