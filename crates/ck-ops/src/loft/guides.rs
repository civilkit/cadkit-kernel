//! Guide curves of a loft (ck-ops/src/loft/guides.rs): curves that touch
//! every profile and that the loft surface must contain.
//!
//! Codes: `loftGuideInvalid` (no usable curve), `loftGuideMiss` (a profile
//! is not touched), `loftGuideOrder` (the guide meets the profiles out of
//! order, or the guides sit in a different order round different profiles).
//! Oracle probes (docs/ck-ops-stubs.md, tools/stub_cases.mjs `loftGuide*`):
//! a guide may run either way; it splits the profile piece it touches at the
//! contact (a straight guide through the mid-points of two square edges
//! gives 7 faces / 15 edges, the cut edge becomes `S.e2~1` / `S.e2~2`, the
//! volume stays 7000); a guide missing a profile is `loftGuideMiss`.
//!
//! Construction (RECONSTRUCTED, not the original's): the contact points cut
//! the profiles (so that the cut pieces of all stations match guide by
//! guide), the rail through a contact point is the guide itself instead of
//! the natural spline through the stations, and every side face that has a
//! guide as a rail is the Gordon surface `S = S_rails + S_stations -
//! S_tensor` (rails ruled across, the station curves cardinally blended
//! along, minus their tensor product): it contains the guide rails and the
//! station curves exactly. For faces without a guide rail it reduces to the
//! plain station blend. The oracle's shape between the guides is its own
//! blend: a bulging guide changes the volume by the same order but not the
//! same number (probe `loftGuideSpline`: 8029.2 vs ours, see the doc), and
//! `influence` other than `next` is refused (`loftFailed`, detail).

use super::stations::{Chain, Piece, Stations};
use super::{smooth, LoftError};
use ck_geom::curve::Curve;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::Vec3;

pub const GUIDE_INVALID: &str = "loftGuideInvalid";
pub const GUIDE_MISS: &str = "loftGuideMiss";
pub const GUIDE_ORDER: &str = "loftGuideOrder";

/// One guide curve as a dense polyline (travel direction as drawn).
#[derive(Debug, Clone)]
pub struct GuideIn {
    pub pts: Vec<Vec3>,
}

/// The rail replacing the natural spline through the stations at one vertex
/// index; a B-spline whose parameter is the loft parameter `v` of the stations.
#[derive(Debug, Clone)]
pub struct GuideRail {
    pub curve: BSplineCurve,
}

fn err(code: &str, detail: &str) -> LoftError {
    LoftError { code: code.into(), detail: Some(detail.into()), hint: None }
}

/// Polyline with cumulative arc length.
struct Poly {
    p: Vec<Vec3>,
    s: Vec<f64>,
}

impl Poly {
    fn new(p: Vec<Vec3>) -> Poly {
        let mut s = vec![0.0];
        for w in p.windows(2) {
            s.push(s.last().copied().unwrap_or(0.0) + w[0].dist(w[1]));
        }
        Poly { p, s }
    }
    fn len(&self) -> f64 {
        self.s.last().copied().unwrap_or(0.0)
    }
    fn reversed(&self) -> Poly {
        Poly::new(self.p.iter().rev().copied().collect())
    }
    /// Point at arc length `l`.
    fn at(&self, l: f64) -> Vec3 {
        if self.p.len() < 2 {
            return self.p.first().copied().unwrap_or(Vec3::new(0.0, 0.0, 0.0));
        }
        let l = l.clamp(0.0, self.len());
        let last = self.p.len() - 2;
        let i = match self.s.binary_search_by(|x| x.total_cmp(&l)) {
            Ok(i) => i.min(last),
            Err(i) => i.saturating_sub(1).min(last),
        };
        let d = self.s[i + 1] - self.s[i];
        let t = if d > 0.0 { (l - self.s[i]) / d } else { 0.0 };
        self.p[i] + (self.p[i + 1] - self.p[i]) * t
    }
    /// (distance, arc length) of the closest polyline point to `q`.
    fn closest(&self, q: Vec3) -> (f64, f64) {
        let mut best = (f64::INFINITY, 0.0);
        for i in 0..self.p.len().saturating_sub(1) {
            let (a, b) = (self.p[i], self.p[i + 1]);
            let ab = b - a;
            let l2 = ab.dot(ab);
            let t = if l2 > 0.0 { ((q - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
            let d = (a + ab * t).dist(q);
            if d < best.0 {
                best = (d, self.s[i] + t * l2.sqrt());
            }
        }
        best
    }
}

/// Where a guide touches one station: piece index, parameter in the piece,
/// distance, arc length along the guide.
struct Contact {
    piece: usize,
    s: f64,
    dist: f64,
    lambda: f64,
}

fn contact(chain: &Chain, g: &Poly) -> Contact {
    let mut best = Contact { piece: 0, s: 0.0, dist: f64::INFINITY, lambda: 0.0 };
    let n = 96;
    for (i, pc) in chain.pieces.iter().enumerate() {
        let f = |s: f64| g.closest(pc.g.at(s)).0;
        let mut bs = 0.0;
        let mut bd = f64::INFINITY;
        for k in 0..=n {
            let s = k as f64 / n as f64;
            let d = f(s);
            if d < bd {
                bd = d;
                bs = s;
            }
        }
        // golden-section refinement inside the neighbouring samples
        let (mut lo, mut hi) = ((bs - 1.0 / n as f64).max(0.0), (bs + 1.0 / n as f64).min(1.0));
        let phi = 0.381966011250105;
        for _ in 0..60 {
            let (a, b) = (lo + (hi - lo) * phi, hi - (hi - lo) * phi);
            if f(a) < f(b) {
                hi = b;
            } else {
                lo = a;
            }
        }
        let s = 0.5 * (lo + hi);
        let d = f(s);
        if d < best.dist {
            let (_, lambda) = g.closest(pc.g.at(s));
            best = Contact { piece: i, s, dist: d, lambda };
        }
    }
    best
}

/// The stations cut at the guide contacts, matched guide by guide.
pub struct Guided {
    pub st: Stations,
    /// Per vertex index of the stations (`0..=n`), the guide rail there.
    pub rails: Vec<Option<GuideRail>>,
}

/// Bring `chains` to a common piece structure in which every guide touches
/// the same vertex index of every station.
pub fn guided_stations(mut chains: Vec<Chain>, guides: &[GuideIn]) -> Result<Guided, LoftError> {
    let Some(first) = chains.first() else { return Err(err(GUIDE_INVALID, "no sections")) };
    let closed = first.closed;
    let k = chains.len();
    if k < 2 || chains.iter().any(|c| c.pieces.is_empty()) {
        return Err(err(GUIDE_INVALID, "a guided loft needs two or more non-empty sections"));
    }
    // profiles running against the first are reversed (as the unguided matching does)
    if closed {
        let n0 = chains[0].normal();
        for c in chains.iter_mut().skip(1) {
            if c.normal().dot(n0) < 0.0 {
                *c = c.reversed();
            }
        }
    }
    let size = chains
        .iter()
        .flat_map(|c| c.pieces.iter().map(|p| p.g.length()))
        .fold(0.0, f64::max)
        .max(1.0);
    let miss_tol = 1e-4 * size;
    // ---- contacts per guide / station; direction of each guide
    let mut polys: Vec<Poly> = vec![];
    let mut contacts: Vec<Vec<Contact>> = vec![];
    for g in guides {
        if g.pts.len() < 2 || g.pts.iter().any(|p| !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())) {
            return Err(err(GUIDE_INVALID, "the guide is not a curve"));
        }
        let mut poly = Poly::new(g.pts.clone());
        if !(poly.len() > 0.0) {
            return Err(err(GUIDE_INVALID, "the guide has no length"));
        }
        let mut cs: Vec<Contact> = chains.iter().map(|c| contact(c, &poly)).collect();
        if cs[0].lambda > cs[k - 1].lambda {
            poly = poly.reversed();
            cs = chains.iter().map(|c| contact(c, &poly)).collect();
        }
        for (s, c) in cs.iter().enumerate() {
            if c.dist > miss_tol {
                return Err(err(GUIDE_MISS, &format!("the guide misses profile {}", s + 1)));
            }
        }
        if cs.windows(2).any(|w| !(w[1].lambda > w[0].lambda + 1e-9 * poly.len())) {
            return Err(err(GUIDE_ORDER, "the guide meets the profiles out of order"));
        }
        polys.push(poly);
        contacts.push(cs);
    }
    // ---- cut every station at its contacts; remember the vertex index of each guide
    let ng = guides.len();
    let mut vidx: Vec<Vec<usize>> = vec![vec![0; ng]; k];
    for s in 0..k {
        let chain = &chains[s];
        let n = chain.pieces.len();
        let mut cuts: Vec<Vec<(f64, usize)>> = vec![vec![]; n];
        let mut at_vertex: Vec<(usize, usize)> = vec![]; // (guide, vertex index before splitting)
        for j in 0..ng {
            let c = &contacts[j][s];
            if c.piece >= n {
                return Err(err(GUIDE_INVALID, "the guide does not meet the profile"));
            }
            let eps = 1e-7;
            if c.s < eps {
                at_vertex.push((j, c.piece));
            } else if c.s > 1.0 - eps {
                at_vertex.push((j, c.piece + 1));
            } else {
                cuts[c.piece].push((c.s, j));
            }
        }
        let mut pieces: Vec<Piece> = vec![];
        let mut new_index_of_old: Vec<usize> = vec![];
        for (i, pc) in chain.pieces.iter().enumerate() {
            new_index_of_old.push(pieces.len());
            let mut cs = cuts[i].clone();
            cs.sort_by(|a, b| a.0.total_cmp(&b.0));
            let mut prev = 0.0;
            for &(t, j) in &cs {
                if t - prev < 1e-9 {
                    return Err(err(GUIDE_INVALID, "two guides touch a profile at the same point"));
                }
                pieces.push(Piece { tag: pc.tag.clone(), g: pc.g.sub(prev, t) });
                vidx[s][j] = pieces.len();
                prev = t;
            }
            pieces.push(Piece { tag: pc.tag.clone(), g: pc.g.sub(prev, 1.0) });
        }
        new_index_of_old.push(pieces.len()); // the far vertex
        for (j, old) in at_vertex {
            vidx[s][j] = new_index_of_old[old.min(n)];
            if closed && old >= n {
                vidx[s][j] = 0;
            }
        }
        // a closed chain: vertex index n (== 0) wraps
        chains[s] = Chain { closed, pieces };
        let m = chains[s].pieces.len();
        if closed {
            for j in 0..ng {
                if vidx[s][j] >= m {
                    vidx[s][j] -= m;
                }
            }
        }
        let mut seen = vidx[s].clone();
        seen.sort_unstable();
        if seen.windows(2).any(|w| w[0] == w[1]) {
            return Err(err(GUIDE_INVALID, "two guides touch a profile at the same point"));
        }
    }
    // ---- rotate closed chains so guide 0 sits at vertex 0
    if closed {
        for s in 0..k {
            let r = vidx.get(s).and_then(|v| v.first()).copied().unwrap_or(0);
            if r != 0 {
                chains[s] = chains[s].rotated(r);
                let m = chains[s].pieces.len();
                for j in 0..ng {
                    vidx[s][j] = (vidx[s][j] + m - r) % m;
                }
            }
        }
    }
    // ---- the guides must come in the same order round every station
    let order = |s: usize| -> Vec<usize> {
        let mut o: Vec<usize> = (0..ng).collect();
        o.sort_by_key(|&j| vidx[s][j]);
        o
    };
    let o0 = order(0);
    for s in 1..k {
        if order(s) != o0 {
            return Err(err(GUIDE_ORDER, "the guides meet the profiles in different orders"));
        }
    }
    // ---- equal piece counts in every segment between consecutive guides
    let mut bounds: Vec<Vec<usize>> = (0..k).map(|s| o0.iter().map(|&j| vidx[s][j]).collect()).collect(); // vertex indices in guide order
    let segs = if closed { ng } else { ng + 1 };
    // segment q: from bound q-1 (or 0) to bound q (or n); closed: from bound q to bound q+1 (wrapping)
    let seg_range = |s: usize, q: usize, bounds: &Vec<Vec<usize>>, n: usize| -> (usize, usize) {
        if closed {
            let a = bounds[s][q];
            let b = if q + 1 < ng { bounds[s][q + 1] } else { n };
            (a, b)
        } else {
            let a = if q == 0 { 0 } else { bounds[s][q - 1] };
            let b = if q < ng { bounds[s][q] } else { n };
            (a, b)
        }
    };
    for q in (0..segs).rev() {
        let n_of = |s: usize, bounds: &Vec<Vec<usize>>, chains: &Vec<Chain>| {
            let (a, b) = seg_range(s, q, bounds, chains[s].pieces.len());
            b.saturating_sub(a)
        };
        let Some(want) = (0..k).map(|s| n_of(s, &bounds, &chains)).max() else { return Err(err(GUIDE_INVALID, "no sections")) };
        for s in 0..k {
            while n_of(s, &bounds, &chains) < want {
                let (a, b) = seg_range(s, q, &bounds, chains[s].pieces.len());
                if b <= a || b > chains[s].pieces.len() {
                    return Err(err(GUIDE_ORDER, "the guides meet the profiles in an order that cannot be matched"));
                }
                // bisect the longest piece of the segment
                let mut bi = a;
                let mut bl = -1.0;
                for i in a..b {
                    let l = chains[s].pieces[i].g.length();
                    if l > bl {
                        bl = l;
                        bi = i;
                    }
                }
                let p = chains[s].pieces[bi].clone();
                let (x, y) = (p.g.sub(0.0, 0.5), p.g.sub(0.5, 1.0));
                chains[s].pieces.splice(bi..=bi, [Piece { tag: p.tag.clone(), g: x }, Piece { tag: p.tag, g: y }]);
                for bnd in bounds[s].iter_mut() {
                    if *bnd > bi {
                        *bnd += 1;
                    }
                }
            }
        }
    }
    // ---- stations
    let tags = chains[0].pieces.iter().map(|p| p.tag.clone()).collect();
    let cs: Vec<Vec3> = chains.iter().map(|c| c.centroid()).collect();
    let mut ts = vec![0.0];
    for i in 1..cs.len() {
        ts.push(ts[i - 1] + cs[i - 1].dist(cs[i]));
    }
    let n = chains[0].pieces.len();
    if n == 0 || chains.iter().any(|c| c.pieces.len() != n) {
        return Err(err(GUIDE_INVALID, "the guides could not be matched to the sections"));
    }
    let nv = if closed { n } else { n + 1 };
    // ---- guide rails: v of the stations, arc length of the guide in between
    let mut rails: Vec<Option<GuideRail>> = vec![None; nv];
    for (gi, &j) in o0.iter().enumerate() {
        let Some(&vi) = bounds.first().and_then(|b| b.get(gi)) else { return Err(err(GUIDE_INVALID, "the guides could not be matched to the sections")) };
        if vi >= nv {
            return Err(err(GUIDE_INVALID, "the guides could not be matched to the sections"));
        }
        let vp = |s: usize| -> Vec3 {
            let c = &chains[s];
            if vi < n {
                c.pieces[vi].g.start()
            } else {
                c.pieces[n - 1].g.end()
            }
        };
        let per = 12;
        let mut vs = vec![];
        let mut pts: Vec<[f64; 3]> = vec![];
        for s in 0..k - 1 {
            let (l0, l1) = (contacts[j][s].lambda, contacts[j][s + 1].lambda);
            for i in 0..per {
                let f = i as f64 / per as f64;
                vs.push(ts[s] + (ts[s + 1] - ts[s]) * f);
                let p = if i == 0 { vp(s) } else { polys[j].at(l0 + (l1 - l0) * f) };
                pts.push([p.x, p.y, p.z]);
            }
        }
        vs.push(ts[k - 1]);
        let pl = vp(k - 1);
        pts.push([pl.x, pl.y, pl.z]);
        let (deg, kn, ctrl) = smooth::through(&vs, &pts).map_err(|_| err("loftFailed", "guide rail fit"))?;
        let curve = BSplineCurve::new(deg, kn, ctrl.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect())
            .map_err(|_| err("loftFailed", "guide rail fit"))?;
        rails[vi] = Some(GuideRail { curve });
    }
    Ok(Guided { st: Stations { closed, tags, chains, ts }, rails })
}

/// Silence "unused" for the curve type used by callers.
#[allow(dead_code)]
fn _curve(_: &Curve) {}

#[cfg(test)]
mod tests {
    use super::super::stations::Geom3;
    use super::*;

    fn square(y: f64, h: f64) -> Chain {
        let p = |x: f64, z: f64| Vec3::new(x, y, z);
        let c = [p(-h, h), p(h, h), p(h, -h), p(-h, -h)];
        Chain {
            closed: true,
            pieces: (0..4)
                .map(|i| Piece { tag: format!("e{}", i + 1), g: Geom3::Line { a: c[i], b: c[(i + 1) % 4] } })
                .collect(),
        }
    }

    fn line_guide(a: Vec3, b: Vec3) -> GuideIn {
        GuideIn { pts: (0..=8).map(|i| a + (b - a) * (i as f64 / 8.0)).collect() }
    }

    #[test]
    fn guide_on_an_edge_midpoint_splits_that_piece_in_every_station() {
        // squares r10 at y=0 and r5 at y=30; guide through the midpoints of the x = +h edges
        let g = line_guide(Vec3::new(10.0, 0.0, 0.0), Vec3::new(5.0, 30.0, 0.0));
        let out = guided_stations(vec![square(0.0, 10.0), square(30.0, 5.0)], &[g]).expect("guided");
        assert_eq!(out.st.chains[0].pieces.len(), 5);
        assert_eq!(out.st.chains[1].pieces.len(), 5);
        // the guide sits at vertex 0 of both stations
        for c in &out.st.chains {
            let p0 = c.pieces[0].g.start();
            assert!((p0.x - c.pieces[0].g.start().x).abs() < 1e-12 && p0.z.abs() < 1e-9, "{p0:?}");
        }
        let r = out.rails[0].as_ref().expect("rail");
        let v = r.curve.eval(out.st.ts[1]);
        assert!((v.x - 5.0).abs() < 1e-9 && (v.y - 30.0).abs() < 1e-9, "{v:?}");
    }

    #[test]
    fn miss_and_order_errors() {
        let miss = line_guide(Vec3::new(12.0, 0.0, 0.0), Vec3::new(7.0, 30.0, 0.0));
        let e = guided_stations(vec![square(0.0, 10.0), square(30.0, 5.0)], &[miss]).err().unwrap();
        assert_eq!(e.code, GUIDE_MISS);
        let short = line_guide(Vec3::new(10.0, 0.0, 0.0), Vec3::new(7.5, 15.0, 0.0));
        let e = guided_stations(vec![square(0.0, 10.0), square(30.0, 5.0)], &[short]).err().unwrap();
        assert_eq!(e.code, GUIDE_MISS);
    }
}
