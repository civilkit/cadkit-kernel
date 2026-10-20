//! Face triangulation: a planar polygon with holes in 3D to triangles, by projecting on the face plane, bridging the
//! holes into the outer ring and clipping ears.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! OpenSKP's `triangulate_face_3d` (`_core.py`, `geometry.cpp`) has the same shape: 3/4-vertex and convex single-loop
//! fast paths, earcut for the rest, a fan as the last resort. The ear clipper here is written for this crate (no earcut
//! code); it carries a work budget and falls back to a fan, so it always terminates.

use crate::model::NO_ID;

#[cfg(test)]
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Newell normal of a closed 3D polygon (not normalised).
pub fn newell(pts: &[[f64; 3]]) -> [f64; 3] {
    let mut n = [0.0; 3];
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    n
}

/// Basis (u, v) of the plane with unit normal `n`, as OpenSKP does it (helper axis X unless `n` is nearly along it).
pub fn plane_basis(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let helper = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let mut u = cross(n, helper);
    let l = norm(u);
    if l > 0.0 {
        u = [u[0] / l, u[1] / l, u[2] / l];
    }
    let v = cross(n, u);
    (u, v)
}

fn area2(ring: &[[f64; 2]]) -> f64 {
    let mut a = 0.0;
    for i in 0..ring.len() {
        let (p, q) = (ring[i], ring[(i + 1) % ring.len()]);
        a += p[0] * q[1] - q[0] * p[1];
    }
    a
}

fn is_convex_2d(c: &[[f64; 2]]) -> bool {
    let n = c.len();
    if n < 3 {
        return false;
    }
    let mut sign = 0i32;
    for i in 0..n {
        let (a, b, d) = (c[i], c[(i + 1) % n], c[(i + 2) % n]);
        let cr = (b[0] - a[0]) * (d[1] - b[1]) - (b[1] - a[1]) * (d[0] - b[0]);
        if cr.abs() > 1e-9 {
            let s = if cr > 0.0 { 1 } else { -1 };
            if sign == 0 {
                sign = s;
            } else if s != sign {
                return false;
            }
        }
    }
    sign != 0
}

fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn seg_intersect(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    // proper or touching intersection of segments ab and cd (shared endpoints are excluded by the caller)
    let (o1, o2, o3, o4) = (orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b));
    if ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) && ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0)) {
        return true;
    }
    false
}

fn eq2(a: [f64; 2], b: [f64; 2]) -> bool {
    a[0] == b[0] && a[1] == b[1]
}

/// Triangulate a polygon with holes given as 2D rings (outer first). Returns triangles as index triples into the
/// concatenation of all rings (`ring0 ++ ring1 ++ ...`). `None` when the work budget ran out or the input is degenerate
/// (the caller then falls back to a fan).
fn earcut2(rings: &[Vec<[f64; 2]>], budget: &mut u64) -> Option<Vec<[usize; 3]>> {
    // flat vertex table and per ring index lists
    let mut pts: Vec<[f64; 2]> = vec![];
    let mut idx: Vec<Vec<usize>> = vec![];
    for r in rings {
        let start = pts.len();
        pts.extend_from_slice(r);
        let mut ix: Vec<usize> = (start..start + r.len()).collect();
        // drop consecutive duplicates and a closing repeat
        ix.dedup_by(|b, a| eq2(pts[*a], pts[*b]));
        while ix.len() > 1 && eq2(pts[ix[0]], pts[ix[ix.len() - 1]]) {
            ix.pop();
        }
        idx.push(ix);
    }
    if idx[0].len() < 3 {
        return None;
    }
    // orientation: outer CCW, holes CW
    let ring_pts = |ix: &Vec<usize>| -> Vec<[f64; 2]> { ix.iter().map(|&i| pts[i]).collect() };
    if area2(&ring_pts(&idx[0])) < 0.0 {
        idx[0].reverse();
    }
    let mut holes: Vec<Vec<usize>> = vec![];
    for ix in idx.iter().skip(1) {
        if ix.len() < 3 {
            continue;
        }
        let mut h = ix.clone();
        if area2(&ring_pts(&h)) > 0.0 {
            h.reverse();
        }
        holes.push(h);
    }
    // bridge holes, rightmost first
    let rightmost = |h: &Vec<usize>| h.iter().map(|&i| pts[i][0]).fold(f64::NEG_INFINITY, f64::max);
    holes.sort_by(|a, b| rightmost(b).partial_cmp(&rightmost(a)).unwrap_or(std::cmp::Ordering::Equal));
    let mut poly = idx[0].clone();
    let mut remaining = holes.clone();
    while !remaining.is_empty() {
        let h = remaining.remove(0);
        // hole vertex with max x
        let (hm, _) = h.iter().enumerate().fold((0usize, f64::NEG_INFINITY), |acc, (k, &i)| if pts[i][0] > acc.1 { (k, pts[i][0]) } else { acc });
        let m = pts[h[hm]];
        let mut cand: Vec<usize> = (0..poly.len()).collect();
        cand.sort_by(|&a, &b| {
            let da = (pts[poly[a]][0] - m[0]).powi(2) + (pts[poly[a]][1] - m[1]).powi(2);
            let db = (pts[poly[b]][0] - m[0]).powi(2) + (pts[poly[b]][1] - m[1]).powi(2);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        });
        let mut found = None;
        'cand: for &ci in &cand {
            let v = pts[poly[ci]];
            // segment m-v must not cross the polygon, the hole itself or the other holes
            let check = |ring: &[usize], budget: &mut u64| -> bool {
                for k in 0..ring.len() {
                    *budget = budget.saturating_sub(1);
                    let (a, b) = (pts[ring[k]], pts[ring[(k + 1) % ring.len()]]);
                    if eq2(a, v) || eq2(b, v) || eq2(a, m) || eq2(b, m) {
                        continue;
                    }
                    if seg_intersect(m, v, a, b) {
                        return true;
                    }
                }
                false
            };
            if *budget == 0 {
                return None;
            }
            if check(&poly, budget) || check(&h, budget) {
                continue 'cand;
            }
            for o in &remaining {
                if check(o, budget) {
                    continue 'cand;
                }
            }
            found = Some(ci);
            break;
        }
        let ci = found?;
        // splice: poly[..=ci] + hole from hm around + hole[hm] + poly[ci] + poly[ci+1..]
        let mut np: Vec<usize> = Vec::with_capacity(poly.len() + h.len() + 2);
        np.extend_from_slice(&poly[..=ci]);
        for k in 0..=h.len() {
            np.push(h[(hm + k) % h.len()]);
        }
        np.push(poly[ci]);
        np.extend_from_slice(&poly[ci + 1..]);
        poly = np;
    }
    clip_ears(&pts, &poly, budget)
}

fn clip_ears(pts: &[[f64; 2]], poly: &[usize], budget: &mut u64) -> Option<Vec<[usize; 3]>> {
    let n = poly.len();
    let mut prev: Vec<usize> = (0..n).map(|i| (i + n - 1) % n).collect();
    let mut next: Vec<usize> = (0..n).map(|i| (i + 1) % n).collect();
    let mut remaining = n;
    let mut tris = Vec::with_capacity(n.saturating_sub(2));
    let mut cur = 0usize;
    let mut since = 0usize;
    let p = |i: usize| pts[poly[i]];
    while remaining > 3 {
        if *budget == 0 {
            return None;
        }
        let (a, b, c) = (prev[cur], cur, next[cur]);
        let (pa, pb, pc) = (p(a), p(b), p(c));
        let ar = orient(pa, pb, pc);
        let mut remove_only = false;
        let mut ear = false;
        if ar.abs() <= 1e-18 * (1.0 + pa[0].abs() + pa[1].abs()).powi(2) || eq2(pa, pb) || eq2(pb, pc) {
            // collinear or duplicate: drop the vertex without a triangle
            remove_only = true;
        } else if ar > 0.0 {
            ear = true;
            let mut q = next[c];
            while q != a {
                *budget = budget.saturating_sub(1);
                let pq = p(q);
                if !(eq2(pq, pa) || eq2(pq, pb) || eq2(pq, pc)) && orient(pa, pb, pq) >= 0.0 && orient(pb, pc, pq) >= 0.0 && orient(pc, pa, pq) >= 0.0 {
                    ear = false;
                    break;
                }
                q = next[q];
            }
        }
        if ear || remove_only {
            if ear {
                tris.push([poly[a], poly[b], poly[c]]);
            }
            next[a] = c;
            prev[c] = a;
            remaining -= 1;
            cur = c;
            since = 0;
        } else {
            cur = next[cur];
            since += 1;
            if since > remaining {
                // no ear in a full pass: self-touching or degenerate input; clip the most convex vertex
                let mut best = cur;
                let mut best_a = f64::NEG_INFINITY;
                let mut v = cur;
                for _ in 0..remaining {
                    let o = orient(p(prev[v]), p(v), p(next[v]));
                    if o > best_a {
                        best_a = o;
                        best = v;
                    }
                    v = next[v];
                }
                let (a, b, c) = (prev[best], best, next[best]);
                if best_a > 0.0 {
                    tris.push([poly[a], poly[b], poly[c]]);
                }
                next[a] = c;
                prev[c] = a;
                remaining -= 1;
                cur = c;
                since = 0;
            }
        }
    }
    if remaining == 3 {
        let (a, b, c) = (prev[cur], cur, next[cur]);
        if orient(p(a), p(b), p(c)).abs() > 0.0 {
            tris.push([poly[a], poly[b], poly[c]]);
        }
    }
    Some(tris)
}

/// Port of `triangulate_face_3d`: `loops` are vertex-id lists (outer first), `pos` resolves an id. Returns triangles of
/// vertex ids; empty when a vertex is missing. `max_verts` bounds the work (above it only the outer fan is returned).
pub fn triangulate_loops(pos: &dyn Fn(u64) -> Option<[f64; 3]>, loops: &[Vec<u64>], normal: [f64; 3], max_verts: usize) -> Vec<[u64; 3]> {
    if loops.is_empty() || loops[0].len() < 3 {
        return vec![];
    }
    let total: usize = loops.iter().map(|l| l.len()).sum();
    if loops.len() == 1 && loops[0].len() == 3 {
        return vec![[loops[0][0], loops[0][1], loops[0][2]]];
    }
    // positions
    let mut p3: Vec<Vec<[f64; 3]>> = vec![];
    for l in loops {
        let mut v = Vec::with_capacity(l.len());
        for &id in l {
            if id == NO_ID {
                return vec![];
            }
            match pos(id) {
                Some(p) if p.iter().all(|x| x.is_finite()) => v.push(p),
                _ => return vec![],
            }
        }
        p3.push(v);
    }
    let fan = |l: &Vec<u64>| -> Vec<[u64; 3]> { (1..l.len() - 1).map(|i| [l[0], l[i], l[i + 1]]).collect() };
    if total > max_verts {
        return fan(&loops[0]);
    }
    let mut n = normal;
    let nl = norm(n);
    if nl.is_finite() && nl > 1e-6 {
        n = [n[0] / nl, n[1] / nl, n[2] / nl];
    } else {
        let nw = newell(&p3[0]);
        let l = norm(nw);
        n = if l > 1e-12 { [nw[0] / l, nw[1] / l, nw[2] / l] } else { [0.0, 0.0, 1.0] };
    }
    let (u, v) = plane_basis(n);
    let rings: Vec<Vec<[f64; 2]>> = p3.iter().map(|l| l.iter().map(|&p| [dot(p, u), dot(p, v)]).collect()).collect();
    if loops.len() == 1 && is_convex_2d(&rings[0]) {
        return fan(&loops[0]);
    }
    // flat id table in ring order
    let flat: Vec<u64> = loops.iter().flat_map(|l| l.iter().copied()).collect();
    let mut budget: u64 = 200_000_000;
    if let Some(t) = earcut2(&rings, &mut budget) {
        if !t.is_empty() {
            return t.into_iter().map(|[a, b, c]| [flat[a], flat[b], flat[c]]).collect();
        }
    }
    fan(&loops[0])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn area3(tris: &[[u64; 3]], pts: &[[f64; 3]]) -> f64 {
        tris.iter()
            .map(|t| {
                let (a, b, c) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
                norm(cross(sub(b, a), sub(c, a))) / 2.0
            })
            .sum()
    }

    #[test]
    fn square_with_hole() {
        let pts: Vec<[f64; 3]> = vec![[0., 0., 0.], [10., 0., 0.], [10., 10., 0.], [0., 10., 0.], [3., 3., 0.], [3., 7., 0.], [7., 7., 0.], [7., 3., 0.]];
        let loops = vec![vec![0u64, 1, 2, 3], vec![4, 5, 6, 7]];
        let t = triangulate_loops(&|i| pts.get(i as usize).copied(), &loops, [0., 0., 1.], 1000);
        assert!((area3(&t, &pts) - 84.0).abs() < 1e-9, "{t:?}");
        assert_eq!(t.len(), 8);
    }

    #[test]
    fn concave_l_shape() {
        let pts: Vec<[f64; 3]> = vec![[0., 0., 0.], [4., 0., 0.], [4., 2., 0.], [2., 2., 0.], [2., 4., 0.], [0., 4., 0.]];
        let t = triangulate_loops(&|i| pts.get(i as usize).copied(), &[vec![0, 1, 2, 3, 4, 5]], [0., 0., 1.], 1000);
        assert!((area3(&t, &pts) - 12.0).abs() < 1e-9);
        assert_eq!(t.len(), 4);
    }

    #[test]
    fn degenerate_inputs_terminate() {
        let pts: Vec<[f64; 3]> = vec![[0., 0., 0.]; 6];
        let _ = triangulate_loops(&|i| pts.get(i as usize).copied(), &[vec![0, 1, 2, 3, 4, 5]], [0., 0., 0.], 1000);
        let pts: Vec<[f64; 3]> = (0..8).map(|i| [i as f64, 0.0, 0.0]).collect();
        let _ = triangulate_loops(&|i| pts.get(i as usize).copied(), &[vec![0, 1, 2, 3, 4, 5, 6, 7]], [0., 0., 1.], 1000);
        let _ = triangulate_loops(&|_| Some([f64::NAN; 3]), &[vec![0, 1, 2, 3]], [0., 0., 1.], 1000);
    }
}
