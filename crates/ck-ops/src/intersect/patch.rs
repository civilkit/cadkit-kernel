//! Patch-box seeding for the marching surface-surface intersection (the
//! original's scheme, reconstructed from `f2487` / `f3014`: a 10 x 10
//! parameter-grid of patch cells per surface, each with a padded box and a
//! slab along its centre normal; overlapping cell pairs are the only places
//! seeds are looked for: the boundary iso-curves of a surviving cell are
//! intersected with the other surface by a pinned 4-variable Newton, and one
//! free Newton per pair from the cell centres catches loops that cross no cell
//! boundary). No closest-point projection anywhere.

use super::ss::{inside, refine_uv, solve4, wrap_uv, Bounds, MPt, Uv4};
use crate::bvh::Aabb;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Counters for the log / tests: (intersections seeded, cell pairs, pinned Newton runs,
/// seeds, intersections that fell back to the projection grid).
pub static STATS: [AtomicUsize; 7] = [AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0)];

/// `STATS` plus (native only) the nanoseconds spent in the patch seeding and in the whole march.
pub fn stats() -> [usize; 7] {
    [0, 1, 2, 3, 4, 5, 6].map(|i| STATS[i].load(Ordering::Relaxed))
}

/// Add the elapsed time since `t` to `STATS[i]` (native builds only).
pub(super) fn add_time(i: usize, t: Option<std::time::Instant>) {
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(t) = t {
        STATS[i].fetch_add(t.elapsed().as_nanos() as usize, Ordering::Relaxed);
    }
    let _ = (i, t);
}

pub(super) fn now() -> Option<std::time::Instant> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        return Some(std::time::Instant::now());
    }
    #[allow(unreachable_code)]
    None
}

const N: usize = 10;

pub(super) struct Cell {
    pub u: (f64, f64),
    pub v: (f64, f64),
    pub bx: Aabb,
    pub pts: [Vec3; 9],
    pub pad: f64,
    /// Centre normal and the padded slab of the 9 points along it.
    pub slab: Option<(Vec3, f64, f64)>,
}

pub(super) struct Grid {
    pub us: Vec<f64>,
    pub vs: Vec<f64>,
    pub cells: Vec<Cell>,
}

impl Grid {
    pub fn cell(&self, i: usize, j: usize) -> &Cell {
        &self.cells[i * N + j]
    }
}

/// The 10 x 10 cell grid of `s` over the parameter box `uvb`.
pub(super) fn grid(s: &Surface, uvb: ((f64, f64), (f64, f64)), tol: f64) -> Grid {
    let ((u0, u1), (v0, v1)) = uvb;
    let us: Vec<f64> = (0..=2 * N).map(|k| u0 + (u1 - u0) * k as f64 / (2 * N) as f64).collect();
    let vs: Vec<f64> = (0..=2 * N).map(|k| v0 + (v1 - v0) * k as f64 / (2 * N) as f64).collect();
    // one 21 x 21 point lattice: the 3 x 3 samples of every cell are shared with its neighbours
    let p = s.eval_grid(&us, &vs);
    let at = |a: usize, b: usize| p[a * (2 * N + 1) + b];
    let mut cells = Vec::with_capacity(N * N);
    for i in 0..N {
        for j in 0..N {
            let (a, b) = (2 * i, 2 * j);
            let pts = [at(a, b), at(a + 1, b), at(a + 2, b), at(a, b + 1), at(a + 1, b + 1), at(a + 2, b + 1), at(a, b + 2), at(a + 1, b + 2), at(a + 2, b + 2)];
            let bx = Aabb::from_points(&pts);
            // sagitta of the 5 interior samples against the bilinear corner interpolant: the pad
            // (the 9 points do not bound the patch; twice the sagitta plus 2 % of the cell diagonal
            // plus the tolerance is a generous bound for a cell that is a tenth of the surface)
            let (c00, c20, c02, c22) = (pts[0], pts[2], pts[6], pts[8]);
            let bil = |x: f64, y: f64| (c00 * (1.0 - x) + c20 * x) * (1.0 - y) + (c02 * (1.0 - x) + c22 * x) * y;
            let sag = [(1, 0.5, 0.0), (3, 0.0, 0.5), (4, 0.5, 0.5), (5, 1.0, 0.5), (7, 0.5, 1.0)].iter().map(|&(k, x, y)| pts[k].dist(bil(x, y))).fold(0.0, f64::max);
            let pad = 2.0 * sag + 0.02 * bx.diagonal() + tol;
            let (um, vm) = (0.5 * (us[a] + us[a + 2]), 0.5 * (vs[b] + vs[b + 2]));
            let n = s.normal(um, vm);
            let slab = n.is_finite().then(|| {
                let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
                for q in &pts {
                    let d = (*q - pts[4]).dot(n);
                    lo = lo.min(d);
                    hi = hi.max(d);
                }
                (n, lo - pad, hi + pad)
            }).filter(|(n, _, _)| n.len() > 0.5);
            cells.push(Cell { u: (us[a], us[a + 2]), v: (vs[b], vs[b + 2]), bx: bx.padded(pad), pts, pad, slab });
        }
    }
    Grid { us: us.iter().step_by(2).copied().collect(), vs: vs.iter().step_by(2).copied().collect(), cells }
}

/// Signed distance of `p` from the surface of cell `c`: the exact implicit function for the
/// closed-form kinds (`ss::implicit`), else the distance to the cell's centre tangent plane.
fn signed(f: &Option<Box<dyn Fn(Vec3) -> f64>>, c: &Cell, p: Vec3) -> Option<f64> {
    if let Some(f) = f {
        return Some(f(p));
    }
    c.slab.map(|(n, _, _)| (p - c.pts[4]).dot(n))
}

/// Do the 9 samples of `a` straddle `b`'s surface (sign change of `signed`)?
fn straddles(f: &Option<Box<dyn Fn(Vec3) -> f64>>, a: &Cell, b: &Cell) -> bool {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for q in &a.pts {
        let Some(d) = signed(f, b, *q) else { return true };
        lo = lo.min(d);
        hi = hi.max(d);
    }
    lo <= 0.0 && hi >= 0.0
}

/// Separating-slab test: are all of `b`'s samples on one side of `a`'s slab?
fn slab_separates(a: &Cell, b: &Cell) -> bool {
    let Some((n, lo, hi)) = a.slab else { return false };
    let (mut blo, mut bhi) = (f64::INFINITY, f64::NEG_INFINITY);
    for q in &b.pts {
        let d = (*q - a.pts[4]).dot(n);
        blo = blo.min(d);
        bhi = bhi.max(d);
    }
    bhi + b.pad < lo || blo - b.pad > hi
}

/// Cell pairs (index into g1.cells, index into g2.cells) whose boxes overlap and whose slabs do
/// not separate them.
pub(super) fn pairs(g1: &Grid, g2: &Grid, tol: f64) -> Vec<(usize, usize)> {
    let mut out = vec![];
    for (i, a) in g1.cells.iter().enumerate() {
        let ab = a.bx.padded(tol);
        for (j, b) in g2.cells.iter().enumerate() {
            if ab.overlaps(&b.bx) && !slab_separates(a, b) && !slab_separates(b, a) {
                out.push((i, j));
            }
        }
    }
    out
}

/// Newton on (u1, v1, u2, v2) with S1 = S2 and `x[k]` pinned to `bound`.
pub(super) fn newton_pinned(s1: &Surface, s2: &Surface, mut x: Uv4, k: usize, bound: f64, tol: f64) -> Option<Uv4> {
    x[k] = bound;
    let eps = tol * 1e-4;
    let mut last = f64::INFINITY;
    for _ in 0..30 {
        let d1 = s1.derivs2(x[0], x[1]);
        let d2 = s2.derivs2(x[2], x[3]);
        let f = d1.p - d2.p;
        let fl = f.len();
        if fl < eps {
            return Some(x);
        }
        last = fl;
        let mut sel = [0.0; 5];
        sel[k] = 1.0;
        let m = [
            [d1.du.x, d1.dv.x, -d2.du.x, -d2.dv.x, -f.x],
            [d1.du.y, d1.dv.y, -d2.du.y, -d2.dv.y, -f.y],
            [d1.du.z, d1.dv.z, -d2.du.z, -d2.dv.z, -f.z],
            sel,
        ];
        let dx = solve4(m)?;
        for j in 0..4 {
            x[j] += dx[j];
        }
        let (a, c) = wrap_uv(s1, x[0], x[1]);
        let (e, g) = wrap_uv(s2, x[2], x[3]);
        x = [a, c, e, g];
        x[k] = bound;
    }
    (last <= 0.1 * tol).then_some(x)
}

/// Seeds of the intersection of `s1` and `s2` inside `b` from the cell boundaries of overlapping
/// patch-box pairs (plus one cell-centre Newton per pair). `None` = no cell pair overlaps at all
/// (the surfaces are disjoint inside `b`); `Some((seeds, strong))`: `strong` says some pair's
/// samples straddle the other surface (a curve almost certainly exists), which is when an empty
/// seed list warrants the projection-grid fallback.
pub(super) fn seeds(s1: &Surface, s2: &Surface, g1: &Grid, g2: &Grid, b: &Bounds, tol: f64, step: f64, pad: f64) -> Option<(Vec<MPt>, bool)> {
    let pr = pairs(g1, g2, tol);
    STATS[0].fetch_add(1, Ordering::Relaxed);
    STATS[1].fetch_add(pr.len(), Ordering::Relaxed);
    if pr.is_empty() {
        return None;
    }
    let (f1, f2) = (super::ss::implicit(s1), super::ss::implicit(s2));
    // partners per cell, both ways
    let mut p1: Vec<Vec<usize>> = vec![vec![]; N * N];
    let mut p2: Vec<Vec<usize>> = vec![vec![]; N * N];
    for &(i, j) in &pr {
        p1[i].push(j);
        p2[j].push(i);
    }
    let mut out: Vec<MPt> = vec![];
    let mut newtons = 0usize;
    let mut strong = false;
    let dedupe = (10.0 * tol).max(1e-12);
    let push = |out: &mut Vec<MPt>, p: Vec3, uv: Uv4| {
        if inside(b, p, pad) && !out.iter().any(|m| m.p.dist(p) < dedupe) {
            out.push(MPt { p, uv });
        }
    };
    // iso-curve sides of the cells of `ga` against `sb`: along each side the signed distance to
    // every partner cell changes sign where the side crosses the other surface; one pinned Newton
    // per sign change
    for swap in [false, true] {
        let (sa, sb, ga, gb, pa, fb) = if swap { (s2, s1, g2, g1, &p2, &f1) } else { (s1, s2, g1, g2, &p1, &f2) };
        let mut done = vec![false; 2 * (N + 1) * (N + 1)];
        let sid = |d: usize, i: usize, j: usize| d * (N + 1) * (N + 1) + i * (N + 1) + j;
        for ci in 0..N * N {
            if pa[ci].is_empty() {
                continue;
            }
            let (i, j) = (ci / N, ci % N);
            for (d, li, lj) in [(0, i, j), (0, i + 1, j), (1, i, j), (1, i, j + 1)] {
                let id = sid(d, li, lj);
                if done[id] {
                    continue;
                }
                done[id] = true;
                let mut partners: Vec<usize> = vec![];
                let adj: [Option<usize>; 2] = if d == 0 {
                    [(li > 0).then(|| (li - 1) * N + lj), (li < N).then(|| li * N + lj)]
                } else {
                    [(lj > 0).then(|| li * N + lj - 1), (lj < N).then(|| li * N + lj)]
                };
                for c in adj.into_iter().flatten() {
                    for &q in &pa[c] {
                        if !partners.contains(&q) {
                            partners.push(q);
                        }
                    }
                }
                if partners.is_empty() {
                    continue;
                }
                let (fixed, lo, hi) = if d == 0 { (ga.us[li], ga.vs[lj], ga.vs[lj + 1]) } else { (ga.vs[lj], ga.us[li], ga.us[li + 1]) };
                let m = 8usize;
                let dt = (hi - lo) / m as f64;
                let ts: Vec<f64> = (0..=m).map(|k| lo + dt * k as f64).collect();
                let ps: Vec<Vec3> = ts.iter().map(|&t| if d == 0 { sa.eval(fixed, t) } else { sa.eval(t, fixed) }).collect();
                let mut found: Vec<f64> = vec![];
                for &q in &partners {
                    let cb = &gb.cells[q];
                    let mut prev: Option<(f64, f64, bool)> = None; // (t, signed distance, in the box) of the previous sample
                    for k in 0..=m {
                        let p0 = ps[k];
                        let inb = cb.bx.contains(p0);
                        let Some(g) = signed(fb, cb, p0) else { prev = None; continue };
                        // a sign change counts when either end lies in the partner's box
                        let cross = match prev {
                            Some((_, pg, pin)) => (inb || pin) && pg * g <= 0.0,
                            None => inb && g == 0.0,
                        };
                        if cross {
                            // start at the linear interpolation of the sign change
                            let t = match prev {
                                Some((pt, pg, _)) if pg != g => pt + (ts[k] - pt) * pg.abs() / (pg.abs() + g.abs()),
                                _ => ts[k],
                            };
                            prev = Some((ts[k], g, inb));
                            if found.iter().any(|f| (f - t).abs() < 0.5 * dt) {
                                continue;
                            }
                            let (ua, va) = if d == 0 { (fixed, t) } else { (t, fixed) };
                            let pt = sa.eval(ua, va);
                            let (ub, vb) = refine_uv(sb, 0.5 * (cb.u.0 + cb.u.1), 0.5 * (cb.v.0 + cb.v.1), pt);
                            let x0 = if swap { [ub, vb, ua, va] } else { [ua, va, ub, vb] };
                            let kk = (if swap { 2 } else { 0 }) + d;
                            newtons += 1;
                            let Some(x) = newton_pinned(s1, s2, x0, kk, fixed, tol) else { continue };
                            let tt = x[(if swap { 2 } else { 0 }) + 1 - d];
                            let slack = 1e-6 * (hi - lo);
                            if tt < lo - slack || tt > hi + slack {
                                continue;
                            }
                            let p = (s1.eval(x[0], x[1]) + s2.eval(x[2], x[3])) * 0.5;
                            found.push(tt);
                            push(&mut out, p, x);
                        } else {
                            prev = Some((ts[k], g, inb));
                        }
                    }
                }
            }
        }
    }
    // loops crossing no cell boundary: one free Newton per pair whose centres lie within the
    // other cell's slab, from the cell centres
    for &(i, j) in &pr {
        let (ca, cb) = (&g1.cells[i], &g2.cells[j]);
        let st = straddles(&f2, ca, cb) && straddles(&f1, cb, ca);
        strong |= st;
        if !st {
            continue;
        }
        let uv = [0.5 * (ca.u.0 + ca.u.1), 0.5 * (ca.v.0 + ca.v.1), 0.5 * (cb.u.0 + cb.u.1), 0.5 * (cb.v.0 + cb.v.1)];
        let Some(t) = super::ss::tangent_uv(s1, s2, uv) else { continue };
        let pred = (ca.pts[4] + cb.pts[4]) * 0.5;
        newtons += 1;
        if let Some((p, uv)) = super::ss::correct_uv(s1, s2, pred, t, uv, tol) {
            if ca.bx.contains(p) || cb.bx.contains(p) {
                push(&mut out, p, uv);
            }
        }
    }
    let _ = step;
    STATS[2].fetch_add(newtons, Ordering::Relaxed);
    STATS[3].fetch_add(out.len(), Ordering::Relaxed);
    Some((out, strong))
}
