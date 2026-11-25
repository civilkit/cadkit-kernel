//! Folded free-form patches: a B-spline face whose surface normal turns by more than a right angle between two
//! neighbouring samples has a fold or a cusp. Its mesh shows a strip that faces the other way (a differently coloured
//! stripe with a hairline seam) while every edge still lies on the surface and the enclosed volume stays positive, so the
//! other checks pass. A smooth patch turns its normal by a few degrees between samples 1/24 of its extent apart.
//!
//! Only patches (one loop of four coedges, the side face of a loft or a sweep) are sampled, over the uv box of their
//! loop: a face trimmed to part of a larger surface may fold outside its trim without being wrong.

use crate::{face_ent, Issue};
use ck_geom::surface::Surface;
use ck_topo::body::Body;

const N: usize = 24;
/// Largest width, in samples, of a strip that faces the other way.
const STRIP: usize = 8;
/// An area element below this fraction of the largest on the grid is a collapsed corner, not a place to read a normal.
const DEGENERATE: f64 = 1e-3;
/// Above this share of the neighbour pairs along one direction reversed, the surface is wound, not folded.
const WOUND: f64 = 0.4;

/// Where the surface folds: the first sample `(u, v)` whose normal opposes the next one along u or v.
pub fn surface_fold(s: &Surface, (u0, u1): (f64, f64), (v0, v1): (f64, f64)) -> Option<(f64, f64)> {
    let at = |a: usize, c: usize| (u0 + (u1 - u0) * a as f64 / N as f64, v0 + (v1 - v0) * c as f64 / N as f64);
    // Samples on the rim where the area element |Su x Sv| has all but collapsed (the corner of a three-sided patch, where
    // one edge shrinks to a point) have a normal that is numerical noise: they are left out.
    let raw: Vec<Vec<Option<(ck_geom::Vec3, f64)>>> = (0..=N)
        .map(|a| {
            (0..=N)
                .map(|c| {
                    let (u, v) = at(a, c);
                    let n = s.normal(u, v);
                    let (su, sv) = s.partials(u, v);
                    (n.len() > 0.5 && n.x.is_finite() && n.y.is_finite() && n.z.is_finite()).then(|| (n, su.cross(sv).len()))
                })
                .collect()
        })
        .collect();
    let jmax = raw.iter().flatten().flatten().map(|x| x.1).fold(0.0_f64, f64::max);
    let nrm: Vec<Vec<Option<ck_geom::Vec3>>> =
        raw.iter()
            .enumerate()
            .map(|(a, r)| {
                r.iter()
                    .enumerate()
                    .map(|(c, x)| x.and_then(|(n, j)| (j > DEGENERATE * jmax || !(a == 0 || a == N || c == 0 || c == N)).then_some(n)))
                    .collect()
            })
            .collect();
    // Reversed neighbour pairs along u and along v, as (direction, line, index along it). A surface that winds many
    // times over its extent (the wall of a coil) turns every pair by the same angle, more than a right angle: the grid
    // does not resolve it, and a reversal says nothing. A single reversal that stays (the normal's sign changes where a
    // surface runs through a pole or the end of a section) is no fold either. A fold is a line across the patch where the
    // normal reverses, or a strip that faces the other way (it reverses and, a few samples on, reverses back).
    let mut rev: Vec<(usize, usize, usize)> = vec![];
    let mut pairs = [0usize; 2];
    for a in 0..=N {
        for c in 0..=N {
            let Some(n) = nrm[a][c] else { continue };
            for (d, (da, dc)) in [(1, 0), (0, 1)].into_iter().enumerate() {
                let (a2, c2) = (a + da, c + dc);
                if a2 > N || c2 > N {
                    continue;
                }
                if let Some(m) = nrm[a2][c2] {
                    pairs[d] += 1;
                    if n.dot(m) < 0.0 {
                        rev.push(if d == 0 { (0, c, a) } else { (1, a, c) });
                    }
                }
            }
        }
    }
    rev.sort();
    let wound = |d: usize| rev.iter().filter(|r| r.0 == d).count() as f64 >= WOUND * pairs[d] as f64;
    // a fold line that runs across the patch: the normal reverses at one index along most of the lines
    for d in 0..2 {
        if wound(d) {
            continue;
        }
        let mut across = vec![vec![]; N];
        for &(_, l, i) in rev.iter().filter(|r| r.0 == d) {
            across[i].push(l);
        }
        if let Some((i, ls)) = across.iter().enumerate().find(|(_, ls)| ls.len() * 2 > N) {
            let l = ls[ls.len() / 2];
            let (a, c) = if d == 0 { (i, l) } else { (l, i) };
            return Some(at(a, c));
        }
    }
    for w in rev.windows(2) {
        let ((d, l, i), (d2, l2, j)) = (w[0], w[1]);
        if d == d2 && l == l2 && j - i <= STRIP && !wound(d) {
            let (a, c) = if d == 0 { (i, l) } else { (l, i) };
            return Some(at(a, c));
        }
    }
    None
}

pub fn check(b: &Body, ctl: &crate::Ctl, out: &mut Vec<Issue>) {
    for f in b.face_ids() {
        if ctl.poll() {
            return;
        }
        let face = b.face(f);
        if !matches!(face.surface, Surface::BSpline(_)) || face.loops.len() != 1 {
            continue;
        }
        let l = face.loops[0];
        let cs: Vec<_> = b.lp(l).coedges.iter().copied().filter(|&c| b.coedge(c).alive).collect();
        if cs.len() != 4 {
            continue;
        }
        let (mut ur, mut vr) = ((f64::INFINITY, f64::NEG_INFINITY), (f64::INFINITY, f64::NEG_INFINITY));
        let mut prev = None;
        for &c in &cs {
            let e = b.edge(b.coedge(c).edge);
            for k in 0..=8 {
                let uv = b.coedge_uv(c, e.t0 + (e.t1 - e.t0) * k as f64 / 8.0, prev);
                prev = Some(uv);
                ur = (ur.0.min(uv.x), ur.1.max(uv.x));
                vr = (vr.0.min(uv.y), vr.1.max(uv.y));
            }
        }
        if !(ur.1 > ur.0 && vr.1 > vr.0) {
            continue;
        }
        if let Some((u, v)) = surface_fold(&face.surface, ur, vr) {
            let p = face.surface.eval(u, v);
            out.push(Issue {
                code: "FACE_FOLD",
                entity: face_ent(b, f),
                detail: format!("the surface folds over itself near ({:.3}, {:.3}, {:.3}): its normal reverses between neighbouring samples", p.x, p.y, p.z),
            });
        }
    }
}
