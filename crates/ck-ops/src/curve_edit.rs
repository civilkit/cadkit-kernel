//! editCurve (ck-ops/src/curve_edit.rs): rebuild a chain of curves as ONE
//! clean B-spline of a chosen degree and number of control points, then shape
//! it by moving control points.
//!
//! Codes: `editCurveNotChain`, `editCurveNotSmooth {angle}`, `editCurveDegree
//! {min,max}`, `editCurveTooFewPoints {min}`, `editCurveTooManyPoints {max}`,
//! `editCurveFitFailed`, `editCurveWeight {entry}`, `editCurveNoPlane`; the
//! soft outcomes `editCurveEntryOutside` / `editCurveRemapped` /
//! `editCurveRefMissing` are reported per entry (`EntryOut`), not as errors.
//!
//! Oracle probes (tools/stub_cases.mjs `ec*`, docs/ck-ops-stubs.md): degree in
//! [2, 9]; `count` control points at least `degree + 1` (+ `degree` per kept
//! corner, at least 4 with end tangents), at most 400; a chain that turns at a
//! joint is `NotSmooth` unless `keepCorners` (then a knot of multiplicity
//! `degree` sits at each corner, so the curve passes a control point there);
//! `method: tolerance` raises the count until the deviation is within the
//! tolerance; entries `{cv, of, frame, dx, dy, dz, weight, disabled, ref}`
//! move a control point by an offset in the world frame or in the curve's
//! (tangent, normal, binormal) frame at that control point, an entry made for
//! `of` control points is remapped to the same relative position when the
//! count changed, one beyond the count is skipped; `weight` sets the NURBS
//! weight (must be > 0); `planar` projects the control points onto a plane.
//!
//! RECONSTRUCTED fit: least squares through the dense input samples with chord
//! length parameters and (corner-aware) uniform knots. The oracle's fit is a
//! different (interpolation-like, far less accurate) one: a 6-point cubic of a
//! wavy spline deviates 1.1 mm there, our fit is the least-squares optimum.

use ck_geom::nurbs::basis::{basis_funs, find_span};
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::Vec3;

pub const DEGREE_MIN: usize = 2;
pub const DEGREE_MAX: usize = 9;
pub const COUNT_MAX: usize = 400;

/// One input curve piece, densely sampled in its own direction.
#[derive(Debug, Clone)]
pub struct PieceIn {
    pub pts: Vec<Vec3>,
}

/// One control point edit (values already evaluated).
#[derive(Debug, Clone, Default)]
pub struct EntryIn {
    pub cv: usize,
    /// Number of control points the edit was made for (`None` = current).
    pub of: Option<usize>,
    pub curve_frame: bool,
    pub d: Vec3,
    pub weight: Option<f64>,
    pub disabled: bool,
    /// The control point keeps this offset to a reference point.
    pub ref_point: Option<Vec3>,
    /// A reference was given but does not resolve (the edit is skipped).
    pub ref_missing: bool,
}

#[derive(Debug, Clone)]
pub struct EditOpts {
    pub degree: usize,
    /// `method: "tolerance"` (else a fixed `count`).
    pub by_tolerance: bool,
    pub count: usize,
    pub tolerance: f64,
    pub keep_ends: bool,
    pub keep_tangents: bool,
    pub keep_corners: bool,
    pub entries: Vec<EntryIn>,
    /// `planar`: project onto the plane (origin, unit normal).
    pub plane: Option<(Vec3, Vec3)>,
    pub planar: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EditError {
    pub code: &'static str,
    pub params: Vec<(&'static str, String)>,
}

fn err(code: &'static str, params: Vec<(&'static str, String)>) -> EditError {
    EditError { code, params }
}

#[derive(Debug, Clone)]
pub struct EntryOut {
    pub cv: usize,
    pub before: Option<Vec3>,
    pub applied: bool,
    /// Why an entry was skipped / moved: `outside`, `remapped`, `refMissing`, `weight`.
    pub note: Option<&'static str>,
}

#[derive(Debug, Clone)]
pub struct EditOut {
    pub degree: usize,
    pub knots: Vec<f64>,
    /// Control points after the edits / before them.
    pub cvs: Vec<Vec3>,
    pub base: Vec<Vec3>,
    pub weights: Vec<f64>,
    /// (tangent, normal, binormal) at the Greville abscissa of every control point.
    pub frames: Vec<[Vec3; 3]>,
    /// Largest deviation of the rebuilt curve / of the edited one from the input.
    pub deviation: f64,
    pub edited: f64,
    /// 201 samples along the input: point and distance to the edited curve.
    pub devs: Vec<[f64; 4]>,
    pub corners: usize,
    pub entries: Vec<EntryOut>,
    /// Display polyline of the edited curve.
    pub pts: Vec<Vec3>,
    pub curve: BSplineCurve,
}

// ------------------------------------------------------------------ chain

/// The chain as one sample polyline and the sample indices of its corners.
struct Chain {
    pts: Vec<Vec3>,
    corners: Vec<usize>,
    max_angle: f64,
}

fn unit(a: Vec3, b: Vec3) -> Option<Vec3> {
    (b - a).try_normalize()
}

/// Order and orient `pieces` end to end.
fn chain_of(pieces: &[PieceIn]) -> Result<Chain, EditError> {
    let size = pieces.iter().flat_map(|p| p.pts.iter()).fold(1.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs()));
    let tol = 1e-6 * size.max(1.0);
    let mut pieces: Vec<Vec<Vec3>> = pieces.iter().map(|p| p.pts.clone()).filter(|p| p.len() >= 2).collect();
    if pieces.is_empty() {
        return Err(err("editCurveFitFailed", vec![]));
    }
    let mut chain: Vec<Vec3> = pieces.remove(0);
    let mut joints: Vec<usize> = vec![];
    while !pieces.is_empty() {
        let (head, tail) = (chain[0], *chain.last().unwrap());
        let mut found = None;
        for (i, p) in pieces.iter().enumerate() {
            let (a, b) = (p[0], *p.last().unwrap());
            if tail.dist(a) <= tol {
                found = Some((i, false, true));
            } else if tail.dist(b) <= tol {
                found = Some((i, true, true));
            } else if head.dist(b) <= tol {
                found = Some((i, false, false));
            } else if head.dist(a) <= tol {
                found = Some((i, true, false));
            }
            if found.is_some() {
                break;
            }
        }
        let Some((i, rev, at_tail)) = found else { return Err(err("editCurveNotChain", vec![])) };
        let mut p = pieces.remove(i);
        if rev {
            p.reverse();
        }
        if at_tail {
            joints.push(chain.len() - 1);
            chain.extend(p.into_iter().skip(1));
        } else {
            // prepend: p ends at the head
            let n = p.len();
            joints.iter_mut().for_each(|j| *j += n - 1);
            joints.push(n - 1);
            let mut c = p;
            c.pop();
            c.extend(chain);
            chain = c;
        }
    }
    joints.sort_unstable();
    // corners: the tangent turns at a joint
    let mut corners = vec![];
    let mut max_angle = 0.0f64;
    for &j in &joints {
        if j == 0 || j + 1 >= chain.len() {
            continue;
        }
        let (Some(a), Some(b)) = (unit(chain[j - 1], chain[j]), unit(chain[j], chain[j + 1])) else { continue };
        let ang = a.dot(b).clamp(-1.0, 1.0).acos().to_degrees();
        max_angle = max_angle.max(ang);
        if ang > 1.0 {
            corners.push(j);
        }
    }
    // enough samples for the largest control point count (under-determined fits are junk)
    let k = (4 * COUNT_MAX).div_ceil(chain.len().max(1)).max(1);
    if k > 1 {
        let mut dense = Vec::with_capacity(chain.len() * k);
        for w in chain.windows(2) {
            for j in 0..k {
                dense.push(w[0] + (w[1] - w[0]) * (j as f64 / k as f64));
            }
        }
        dense.push(*chain.last().unwrap());
        chain = dense;
        corners.iter_mut().for_each(|c| *c *= k);
    }
    Ok(Chain { pts: chain, corners, max_angle })
}

fn cumulative(pts: &[Vec3]) -> Vec<f64> {
    let mut s = vec![0.0];
    for w in pts.windows(2) {
        s.push(s.last().unwrap() + w[0].dist(w[1]));
    }
    let t = *s.last().unwrap();
    if t > 0.0 {
        s.iter_mut().for_each(|x| *x /= t);
    }
    s
}

// ------------------------------------------------------------------- fit

/// Banded Cholesky solve of `A x = b`; `a[i]` holds `A[i][i-bw ..= i]`.
fn solve_banded(a: &mut [Vec<f64>], b: &mut [f64], bw: usize) -> bool {
    let m = b.len();
    let at = |a: &[Vec<f64>], i: usize, j: usize| a[i][j + bw - i];
    for i in 0..m {
        for j in i.saturating_sub(bw)..=i {
            let mut sum = at(a, i, j);
            for k in i.saturating_sub(bw).max(j.saturating_sub(bw))..j {
                sum -= at(a, i, k) * at(a, j, k);
            }
            if i == j {
                if !(sum > 0.0) {
                    return false;
                }
                a[i][bw] = sum.sqrt();
            } else {
                a[i][j + bw - i] = sum / at(a, j, j);
            }
        }
    }
    for i in 0..m {
        let mut s = b[i];
        for k in i.saturating_sub(bw)..i {
            s -= at(a, i, k) * b[k];
        }
        b[i] = s / at(a, i, i);
    }
    for i in (0..m).rev() {
        let mut s = b[i];
        for k in i + 1..=(i + bw).min(m - 1) {
            s -= at(a, k, i) * b[k];
        }
        b[i] = s / at(a, i, i);
    }
    true
}

/// Clamped knot vector for `n` control points of degree `p` with C0 knots
/// (multiplicity `p`) at the parameters `corners`.
fn knots_for(n: usize, p: usize, corners: &[f64]) -> Option<Vec<f64>> {
    let interior = n.checked_sub(p + 1)?;
    let fixed = corners.len() * p;
    if fixed > interior {
        return None;
    }
    let free = interior - fixed;
    let mut anchors = vec![0.0];
    anchors.extend_from_slice(corners);
    anchors.push(1.0);
    // free knots per segment, proportional to the segment length
    let total: f64 = 1.0;
    let mut alloc: Vec<usize> = anchors.windows(2).map(|w| ((w[1] - w[0]) / total * free as f64).floor() as usize).collect();
    let mut left = free - alloc.iter().sum::<usize>();
    let mut order: Vec<usize> = (0..alloc.len()).collect();
    order.sort_by(|&x, &y| {
        let fx = (anchors[x + 1] - anchors[x]) * free as f64;
        let fy = (anchors[y + 1] - anchors[y]) * free as f64;
        (fy - fy.floor()).partial_cmp(&(fx - fx.floor())).unwrap()
    });
    for i in order {
        if left == 0 {
            break;
        }
        alloc[i] += 1;
        left -= 1;
    }
    let mut k = vec![0.0; p + 1];
    for (s, w) in anchors.windows(2).enumerate() {
        for j in 1..=alloc[s] {
            k.push(w[0] + (w[1] - w[0]) * j as f64 / (alloc[s] + 1) as f64);
        }
        if s + 1 < anchors.len() - 1 {
            for _ in 0..p {
                k.push(w[1]);
            }
        }
    }
    k.extend(std::iter::repeat(1.0).take(p + 1));
    (k.len() == n + p + 1).then_some(k)
}

#[derive(Clone, Copy)]
enum Cv {
    Fixed(Vec3),
    /// `P0 + a T` (a = unknown scalar `a`) or `Pn - b T` (`b`)
    TanStart(Vec3, Vec3),
    TanEnd(Vec3, Vec3),
    Free(usize),
}

struct Fit {
    knots: Vec<f64>,
    cvs: Vec<Vec3>,
}

fn fit(pts: &[Vec3], u: &[f64], corner_idx: &[usize], n: usize, o: &EditOpts) -> Option<Fit> {
    let p = o.degree;
    let cu: Vec<f64> = corner_idx.iter().map(|&i| u[i]).collect();
    let knots = knots_for(n, p, &cu)?;
    // classify control points
    let mut kind: Vec<Cv> = vec![Cv::Free(0); n];
    let (first, last) = (pts[0], *pts.last().unwrap());
    let mut fixed = vec![false; n];
    if o.keep_ends {
        kind[0] = Cv::Fixed(first);
        kind[n - 1] = Cv::Fixed(last);
        fixed[0] = true;
        fixed[n - 1] = true;
    }
    for (c, &ci) in cu.iter().zip(corner_idx) {
        // the control point a C0 knot passes through
        let span = find_span(n - 1, p, *c, &knots);
        let b = basis_funs(span, *c, p, &knots);
        let j = (0..=p).max_by(|&x, &y| b[x].partial_cmp(&b[y]).unwrap()).map(|j| span - p + j)?;
        if !fixed[j] {
            kind[j] = Cv::Fixed(pts[ci]);
            fixed[j] = true;
        }
    }
    let mut has_a = false;
    let mut has_b = false;
    if o.keep_tangents && o.keep_ends && n >= 4 {
        let t0 = unit(pts[0], pts[(pts.len() / 200).max(1)])?;
        let t1 = unit(*pts.last().unwrap(), pts[pts.len() - 1 - (pts.len() / 200).max(1)])?;
        if !fixed[1] {
            kind[1] = Cv::TanStart(first, t0);
            fixed[1] = true;
            has_a = true;
        }
        if !fixed[n - 2] {
            kind[n - 2] = Cv::TanEnd(last, t1);
            fixed[n - 2] = true;
            has_b = true;
        }
    }
    // unknown layout: [a] [free cvs xyz...] [b]
    let mut m = 0;
    let a_idx = if has_a {
        m += 1;
        Some(0)
    } else {
        None
    };
    for k in 0..n {
        if !fixed[k] {
            kind[k] = Cv::Free(m);
            m += 3;
        }
    }
    let b_idx = if has_b {
        m += 1;
        Some(m - 1)
    } else {
        None
    };
    if m == 0 {
        // everything fixed: the control points are the constraints
        let cvs = kind
            .iter()
            .map(|k| match k {
                Cv::Fixed(v) => *v,
                Cv::TanStart(p0, t) => *p0 + *t,
                Cv::TanEnd(p1, t) => *p1 - *t,
                Cv::Free(_) => first,
            })
            .collect();
        return Some(Fit { knots, cvs });
    }
    let bw = (3 * (p + 2) + 3).min(m - 1);
    let mut a: Vec<Vec<f64>> = (0..m).map(|_| vec![0.0; bw + 1]).collect();
    let mut rhs = vec![0.0; m];
    for (i, s) in pts.iter().enumerate() {
        let t = u[i].clamp(0.0, 1.0);
        let span = find_span(n - 1, p, t, &knots);
        let bf = basis_funs(span, t, p, &knots);
        for c in 0..3 {
            let comp = |v: Vec3| [v.x, v.y, v.z][c];
            let mut row: Vec<(usize, f64)> = vec![];
            let mut target = comp(*s);
            for j in 0..=p {
                let k = span - p + j;
                let w = bf[j];
                match kind[k] {
                    Cv::Fixed(v) => target -= w * comp(v),
                    Cv::Free(ix) => row.push((ix + c, w)),
                    Cv::TanStart(p0, tn) => {
                        target -= w * comp(p0);
                        row.push((a_idx.unwrap(), w * comp(tn)));
                    }
                    Cv::TanEnd(p1, tn) => {
                        target -= w * comp(p1);
                        row.push((b_idx.unwrap(), -w * comp(tn)));
                    }
                }
            }
            for &(ia, va) in &row {
                rhs[ia] += va * target;
                for &(ib, vb) in &row {
                    if ib <= ia && ia - ib <= bw {
                        a[ia][ib + bw - ia] += va * vb;
                    }
                }
            }
        }
    }
    let scale = (0..m).map(|i| a[i][bw]).fold(0.0, f64::max).max(1e-12);
    for i in 0..m {
        a[i][bw] += 1e-12 * scale;
    }
    if !solve_banded(&mut a, &mut rhs, bw) {
        return None;
    }
    let cvs = kind
        .iter()
        .map(|k| match k {
            Cv::Fixed(v) => *v,
            Cv::Free(ix) => Vec3::new(rhs[*ix], rhs[ix + 1], rhs[ix + 2]),
            Cv::TanStart(p0, t) => *p0 + *t * rhs[a_idx.unwrap()],
            Cv::TanEnd(p1, t) => *p1 - *t * rhs[b_idx.unwrap()],
        })
        .collect();
    Some(Fit { knots, cvs })
}

/// [`fit`] with a few rounds of parameter correction (each sample moves to the
/// closest parameter of the fitted curve, then the fit is repeated): chord
/// length parameters are poor for wavy input. The round with the smallest
/// maximum parameter-matched distance wins.
fn fit_refined(pts: &[Vec3], u0: &[f64], corner_idx: &[usize], n: usize, o: &EditOpts) -> Option<Fit> {
    let mut u = u0.to_vec();
    let mut best: Option<(f64, Fit)> = None;
    for _round in 0..4 {
        let f = fit(pts, &u, corner_idx, n, o)?;
        let c = curve_of(o.degree, &f.knots, &f.cvs, &vec![1.0; n])?;
        let mut worst = 0.0f64;
        let mut next = u.clone();
        for (i, s) in pts.iter().enumerate() {
            let mut t = u[i].clamp(0.0, 1.0);
            for _ in 0..4 {
                let d = c.derivs(t, 2);
                let diff = d[0] - *s;
                let den = d[1].dot(d[1]) + diff.dot(d[2]);
                if den.abs() < 1e-14 {
                    break;
                }
                t = (t - diff.dot(d[1]) / den).clamp(0.0, 1.0);
            }
            worst = worst.max(c.eval(u[i].clamp(0.0, 1.0)).dist(*s));
            if !corner_idx.contains(&i) && i != 0 && i + 1 != pts.len() {
                next[i] = t;
            }
        }
        if best.as_ref().map_or(true, |(w, _)| worst < *w) {
            best = Some((worst, f));
        }
        // keep the parameters non-decreasing
        for i in 1..next.len() {
            if next[i] < next[i - 1] {
                next[i] = next[i - 1];
            }
        }
        u = next;
    }
    best.map(|(_, f)| f)
}

fn curve_of(p: usize, knots: &[f64], cvs: &[Vec3], w: &[f64]) -> Option<BSplineCurve> {
    if w.iter().all(|x| (x - 1.0).abs() < 1e-15) {
        BSplineCurve::new(p, knots.to_vec(), cvs.to_vec()).ok()
    } else {
        BSplineCurve::new_rational(p, knots.to_vec(), cvs.to_vec(), w.to_vec()).ok()
    }
}

/// Distance from `q` to a dense polyline.
fn dist_poly(poly: &[Vec3], q: Vec3) -> f64 {
    let mut best = f64::INFINITY;
    for w in poly.windows(2) {
        let ab = w[1] - w[0];
        let l2 = ab.dot(ab);
        let t = if l2 > 0.0 { ((q - w[0]).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
        best = best.min((w[0] + ab * t).dist(q));
    }
    best
}

/// 201 samples along the input and their distance to `c`.
fn deviation(pts: &[Vec3], u: &[f64], c: &BSplineCurve) -> (f64, Vec<[f64; 4]>) {
    let (a, b) = c.domain();
    let nd = 1500;
    let poly: Vec<Vec3> = (0..=nd).map(|i| c.eval(a + (b - a) * i as f64 / nd as f64)).collect();
    let mut devs = vec![];
    let mut max = 0.0f64;
    let mut j = 0;
    for k in 0..=200 {
        let t = k as f64 / 200.0;
        while j + 2 < u.len() && u[j + 1] < t {
            j += 1;
        }
        let (u0, u1) = (u[j], u[j + 1]);
        let f = if u1 > u0 { ((t - u0) / (u1 - u0)).clamp(0.0, 1.0) } else { 0.0 };
        let q = pts[j] + (pts[j + 1] - pts[j]) * f;
        let d = dist_poly(&poly, q);
        max = max.max(d);
        devs.push([q.x, q.y, q.z, d]);
    }
    (max, devs)
}

/// Frames (tangent, normal, binormal) at the Greville abscissae.
fn frames_of(c: &BSplineCurve, cvs: &[Vec3]) -> Vec<[Vec3; 3]> {
    let p = c.degree;
    let n = cvs.len();
    // global binormal: Newell normal of the control polygon (largest component positive)
    let mut nw = Vec3::ZERO;
    for i in 0..n {
        nw = nw + cvs[i].cross(cvs[(i + 1) % n]);
    }
    let mut glob = nw.try_normalize();
    if let Some(g) = glob.as_mut() {
        let m = [g.x.abs(), g.y.abs(), g.z.abs()];
        let big = if m[0] >= m[1] && m[0] >= m[2] { g.x } else if m[1] >= m[2] { g.y } else { g.z };
        if big < 0.0 {
            *g = -*g;
        }
    }
    (0..n)
        .map(|i| {
            let g: f64 = c.knots[i + 1..i + 1 + p].iter().sum::<f64>() / p as f64;
            let g = g.clamp(0.0, 1.0);
            let d = c.derivs(g, 2);
            let t = d[1].try_normalize().unwrap_or(Vec3::X);
            let b = glob.filter(|g| g.cross(t).len() > 1e-6).unwrap_or_else(|| {
                let n2 = (d[2] - t * d[2].dot(t)).try_normalize();
                n2.map_or_else(|| if t.x.abs() < 0.9 { Vec3::X } else { Vec3::Y }.cross(t).normalize(), |n2| t.cross(n2))
            });
            // binormal perpendicular to t
            let b = (b - t * b.dot(t)).try_normalize().unwrap_or(Vec3::Z);
            let nrm = b.cross(t);
            [t, nrm, b]
        })
        .collect()
}

/// The whole operation.
pub fn edit_curve(pieces: &[PieceIn], o: &EditOpts) -> Result<EditOut, EditError> {
    if o.degree < DEGREE_MIN || o.degree > DEGREE_MAX {
        return Err(err("editCurveDegree", vec![("min", DEGREE_MIN.to_string()), ("max", DEGREE_MAX.to_string())]));
    }
    let ch = chain_of(pieces)?;
    if !ch.corners.is_empty() && !o.keep_corners {
        return Err(err("editCurveNotSmooth", vec![("angle", format!("{:.1}", ch.max_angle))]));
    }
    let corner_idx = if o.keep_corners { ch.corners.clone() } else { vec![] };
    let u = cumulative(&ch.pts);
    let p = o.degree;
    let mut min = p + 1 + corner_idx.len() * p;
    if o.keep_tangents && o.keep_ends {
        min = min.max(4);
    }
    let fit_n = |n: usize| fit_refined(&ch.pts, &u, &corner_idx, n, o);
    let n = if o.by_tolerance {
        if min > COUNT_MAX {
            return Err(err("editCurveTooManyPoints", vec![("max", COUNT_MAX.to_string())]));
        }
        let dev_of = |n: usize| -> Option<f64> {
            let f = fit_n(n)?;
            let c = curve_of(p, &f.knots, &f.cvs, &vec![1.0; n])?;
            Some(deviation(&ch.pts, &u, &c).0)
        };
        let (mut lo, mut hi) = (min, COUNT_MAX);
        if dev_of(hi).map_or(true, |d| d > o.tolerance) {
            hi = COUNT_MAX; // best effort: the largest count
            lo = hi;
        }
        while lo < hi {
            let mid = (lo + hi) / 2;
            if dev_of(mid).map_or(false, |d| d <= o.tolerance) {
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        lo
    } else {
        if o.count < min {
            return Err(err("editCurveTooFewPoints", vec![("min", min.to_string())]));
        }
        if o.count > COUNT_MAX {
            return Err(err("editCurveTooManyPoints", vec![("max", COUNT_MAX.to_string())]));
        }
        o.count
    };
    let f = fit_n(n).ok_or_else(|| err("editCurveFitFailed", vec![]))?;
    let base = f.cvs.clone();
    let mut weights = vec![1.0; n];
    let c0 = curve_of(p, &f.knots, &base, &weights).ok_or_else(|| err("editCurveFitFailed", vec![]))?;
    let (deviation0, _) = deviation(&ch.pts, &u, &c0);
    let frames = frames_of(&c0, &base);
    // ---- edits, in order
    let mut cvs = base.clone();
    let mut entries = vec![];
    for (ei, e) in o.entries.iter().enumerate() {
        if let Some(w) = e.weight {
            if !(w > 0.0) || !w.is_finite() {
                return Err(err("editCurveWeight", vec![("entry", (ei + 1).to_string())]));
            }
        }
        let mut cv = e.cv;
        let mut note = None;
        if let Some(of) = e.of {
            if of != n && of > 1 {
                cv = ((e.cv as f64) / (of - 1) as f64 * (n - 1) as f64).round() as usize;
                note = Some("remapped");
            }
        }
        if e.disabled {
            entries.push(EntryOut { cv, before: None, applied: false, note: Some("disabled") });
            continue;
        }
        if cv >= n {
            entries.push(EntryOut { cv, before: None, applied: false, note: Some("outside") });
            continue;
        }
        if e.ref_missing {
            entries.push(EntryOut { cv, before: None, applied: false, note: Some("refMissing") });
            continue;
        }
        let before = cvs[cv];
        let off = if e.curve_frame { frames[cv][0] * e.d.x + frames[cv][1] * e.d.y + frames[cv][2] * e.d.z } else { e.d };
        cvs[cv] = match e.ref_point {
            Some(r) => r + off,
            None => before + off,
        };
        if let Some(w) = e.weight {
            weights[cv] = w;
        }
        entries.push(EntryOut { cv, before: Some(before), applied: true, note });
    }
    if o.planar {
        let Some((po, pn)) = o.plane else { return Err(err("editCurveNoPlane", vec![])) };
        for c in cvs.iter_mut() {
            *c = *c - pn * (*c - po).dot(pn);
        }
    }
    let cur = curve_of(p, &f.knots, &cvs, &weights).ok_or_else(|| err("editCurveFitFailed", vec![]))?;
    let (edited, devs) = deviation(&ch.pts, &u, &cur);
    let (a, b) = cur.domain();
    let np = 200;
    let pts: Vec<Vec3> = (0..=np).map(|i| cur.eval(a + (b - a) * i as f64 / np as f64)).collect();
    Ok(EditOut {
        degree: p,
        knots: f.knots,
        cvs,
        base,
        weights,
        frames,
        deviation: deviation0,
        edited,
        devs,
        corners: corner_idx.len(),
        entries,
        pts,
        curve: cur,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(a: Vec3, b: Vec3, n: usize) -> PieceIn {
        PieceIn { pts: (0..=n).map(|i| a + (b - a) * (i as f64 / n as f64)).collect() }
    }

    fn opts() -> EditOpts {
        EditOpts {
            degree: 3,
            by_tolerance: false,
            count: 6,
            tolerance: 0.01,
            keep_ends: true,
            keep_tangents: false,
            keep_corners: false,
            entries: vec![],
            plane: None,
            planar: false,
        }
    }

    fn arc(r: f64, a0: f64, a1: f64, n: usize) -> PieceIn {
        PieceIn { pts: (0..=n).map(|i| { let a = a0 + (a1 - a0) * i as f64 / n as f64; Vec3::new(r * a.cos(), r * a.sin(), 0.0) }).collect() }
    }

    #[test]
    fn straight_chain_rebuilds_exactly_and_joins_in_any_order() {
        // oracle ecChainSmooth: two collinear lines -> degree 3, 6 control points, deviation 0
        let a = line(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 8);
        let b = line(Vec3::new(20.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 8); // reversed and listed second
        let out = edit_curve(&[a, b], &opts()).unwrap();
        assert_eq!(out.cvs.len(), 6);
        assert!(out.deviation < 1e-9, "{}", out.deviation);
        assert!((out.cvs[0].x).abs() < 1e-12 && (out.cvs[5].x - 20.0).abs() < 1e-12);
    }

    #[test]
    fn errors() {
        let a = line(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 8);
        let far = line(Vec3::new(0.0, 5.0, 0.0), Vec3::new(10.0, 7.0, 0.0), 8);
        assert_eq!(edit_curve(&[a.clone(), far], &opts()).unwrap_err().code, "editCurveNotChain");
        let corner = line(Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 0.0), 8);
        let e = edit_curve(&[a.clone(), corner.clone()], &opts()).unwrap_err();
        assert_eq!(e.code, "editCurveNotSmooth");
        assert_eq!(e.params[0].1, "90.0");
        // keepCorners: needs degree+1+degree points
        let mut o = opts();
        o.keep_corners = true;
        o.count = 4;
        assert_eq!(edit_curve(&[a.clone(), corner.clone()], &o).unwrap_err().code, "editCurveTooFewPoints");
        o.count = 8;
        let out = edit_curve(&[a.clone(), corner], &o).unwrap();
        assert_eq!(out.corners, 1);
        assert!(out.deviation < 1e-9, "polyline through a C0 knot is reproduced: {}", out.deviation);
        let mut o = opts();
        o.degree = 1;
        assert_eq!(edit_curve(&[a.clone()], &o).unwrap_err().code, "editCurveDegree");
        o.degree = 3;
        o.count = 500;
        assert_eq!(edit_curve(&[a.clone()], &o).unwrap_err().code, "editCurveTooManyPoints");
        o.count = 3;
        assert_eq!(edit_curve(&[a], &o).unwrap_err().code, "editCurveTooFewPoints");
    }

    #[test]
    fn circle_fit_and_tolerance_method() {
        let c = arc(10.0, 0.0, std::f64::consts::PI, 256);
        let mut o = opts();
        o.count = 8;
        let out = edit_curve(&[c.clone()], &o).unwrap();
        assert!(out.deviation < 5e-3, "{}", out.deviation);
        // ends stay put
        assert!(out.cvs[0].dist(Vec3::new(10.0, 0.0, 0.0)) < 1e-12);
        o.by_tolerance = true;
        o.tolerance = 1e-3;
        let out = edit_curve(&[c], &o).unwrap();
        assert!(out.deviation <= 1e-3 + 1e-9 && out.cvs.len() >= 6 && out.cvs.len() < 30, "{} {}", out.deviation, out.cvs.len());
    }

    #[test]
    fn entries_move_remap_weight_and_skip() {
        let c = arc(10.0, 0.0, std::f64::consts::PI, 256);
        let mut o = opts();
        o.count = 6;
        let base = edit_curve(&[c.clone()], &o).unwrap();
        o.entries = vec![
            EntryIn { cv: 2, of: Some(6), d: Vec3::new(0.0, 1.0, 0.0), ..Default::default() },
            EntryIn { cv: 2, of: Some(4), d: Vec3::new(1.0, 0.0, 0.0), ..Default::default() }, // 2 of 4 -> 3 of 6
            EntryIn { cv: 9, of: Some(6), d: Vec3::new(1.0, 0.0, 0.0), ..Default::default() }, // outside
            EntryIn { cv: 1, of: Some(6), curve_frame: true, d: Vec3::new(0.0, 1.0, 0.0), weight: Some(3.0), ..Default::default() },
        ];
        let out = edit_curve(&[c], &o).unwrap();
        assert!((out.cvs[2] - base.cvs[2] - Vec3::new(0.0, 1.0, 0.0)).len() < 1e-12);
        assert_eq!(out.entries[1].cv, 3);
        assert!((out.cvs[3] - base.cvs[3] - Vec3::new(1.0, 0.0, 0.0)).len() < 1e-12);
        assert!(!out.entries[2].applied && out.entries[2].note == Some("outside"));
        // curve-frame normal offset has unit length in the normal direction
        assert!(((out.cvs[1] - base.cvs[1]).len() - 1.0).abs() < 1e-9);
        assert_eq!(out.weights, vec![1.0, 3.0, 1.0, 1.0, 1.0, 1.0]);
        assert!(out.edited > out.deviation);
    }

    #[test]
    fn weight_zero_and_planar_projection() {
        let c = arc(10.0, 0.0, 3.0, 128);
        let mut o = opts();
        o.entries = vec![EntryIn { cv: 2, weight: Some(0.0), ..Default::default() }];
        assert_eq!(edit_curve(&[c.clone()], &o).unwrap_err().code, "editCurveWeight");
        o.entries.clear();
        o.planar = true;
        assert_eq!(edit_curve(&[c.clone()], &o).unwrap_err().code, "editCurveNoPlane");
        o.plane = Some((Vec3::ZERO, Vec3::Y));
        let out = edit_curve(&[c], &o).unwrap();
        assert!(out.cvs.iter().all(|p| p.y.abs() < 1e-12));
    }

    #[test]
    fn keep_tangents_keeps_the_end_directions() {
        let c = arc(10.0, 0.0, 2.0, 256);
        let mut o = opts();
        o.keep_tangents = true;
        o.count = 8;
        let out = edit_curve(&[c], &o).unwrap();
        let t0 = (out.cvs[1] - out.cvs[0]).normalize();
        assert!(t0.dot(Vec3::Y) > 1.0 - 1e-4, "{t0:?}");
        let t1 = (out.cvs[7] - out.cvs[6]).normalize();
        let want = Vec3::new(-(2.0f64).sin(), (2.0f64).cos(), 0.0);
        assert!(t1.dot(want) > 1.0 - 1e-4, "{t1:?}");
    }
}
