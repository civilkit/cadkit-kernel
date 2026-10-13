//! View-dependent silhouette curves of smooth faces (sphere, cone, torus,
//! revolution/extrusion, B-spline, screw), extracted from the display mesh by
//! marching triangles on `f = n·z` (n = the per-vertex outward surface normal,
//! z = the view direction towards the viewer).
//!
//! The zero contour of `f` inside each triangle is interpolated linearly along
//! the triangle edges, so the curve is accurate to the display-mesh tolerance
//! and is trimmed exactly like the face (the mesh only covers the face).
//! Contour segments are welded by position into polylines (closed loops keep
//! their first point repeated at the end) and simplified with a 3D
//! Douglas-Peucker pass so straight generators (cones, ruled patches) come out
//! as two points. Cylinders keep their analytic generators in `mod.rs`.

use ck_core::linalg::Vec3;
use std::collections::HashMap;

const EPS: f64 = 1e-6; // |n·z| below this counts as exactly edge-on

/// Silhouette polylines (model space) of the mesh triangles whose `tri_face`
/// equals `face_wi`, for a view looking down `-z` (viewer at `+z`).
pub fn face_contours(m: &ck_tess::mesh::Mesh, face_wi: i32, z: Vec3, scale: f64, simplify: f64) -> Vec<Vec<(Vec3, Vec3)>> {
    let pos = |i: usize| Vec3::new(m.positions[i * 3] as f64, m.positions[i * 3 + 1] as f64, m.positions[i * 3 + 2] as f64);
    let nrm = |i: usize| Vec3::new(m.normals[i * 3] as f64, m.normals[i * 3 + 1] as f64, m.normals[i * 3 + 2] as f64);
    // 1. per-face range of f: a face whose normals never cross the view plane
    //    (or are all edge-on, i.e. effectively planar) has no silhouette
    let (mut fmin, mut fmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for (t, &f) in m.tri_face.iter().enumerate() {
        if f != face_wi {
            continue;
        }
        for k in 0..3 {
            let v = nrm(t * 3 + k).dot(z);
            fmin = fmin.min(v);
            fmax = fmax.max(v);
        }
    }
    if !(fmin < -EPS && fmax > EPS) || fmax - fmin < 1e-3 {
        return vec![];
    }
    // 2. marching triangles
    let q = (scale * 1e-6).max(1e-12);
    let key = |p: Vec3| ((p.x / q).round() as i64, (p.y / q).round() as i64, (p.z / q).round() as i64);
    let mut pts: Vec<Vec3> = vec![];
    let mut nrms: Vec<Vec3> = vec![];
    let mut ids: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut id_of = |p: Vec3, n: Vec3, pts: &mut Vec<Vec3>| -> usize {
        *ids.entry(key(p)).or_insert_with(|| {
            pts.push(p);
            nrms.push(n);
            pts.len() - 1
        })
    };
    let mut segs: Vec<(usize, usize)> = vec![];
    // edges lying exactly on the contour (both ends with f == 0: a seam column
    // or a rim ring seen edge-on): (sees +, sees -, triangle count)
    let mut zero_edges: HashMap<(usize, usize), (bool, bool, u32)> = HashMap::new();
    for (t, &f) in m.tri_face.iter().enumerate() {
        if f != face_wi {
            continue;
        }
        let c = [t * 3, t * 3 + 1, t * 3 + 2];
        let fv = [nrm(c[0]).dot(z), nrm(c[1]).dot(z), nrm(c[2]).dot(z)];
        // sign with a dead band: f32 normals make exact zeros noisy
        let sg: [i8; 3] = [0, 1, 2].map(|i| if fv[i] > EPS { 1 } else if fv[i] < -EPS { -1 } else { 0 });
        if sg.iter().all(|&s| s >= 0) && sg.iter().all(|&s| s <= 0) {
            continue; // all zero
        }
        if sg.iter().all(|&s| s >= 0) && sg.iter().filter(|&&s| s == 0).count() < 2 || sg.iter().all(|&s| s <= 0) && sg.iter().filter(|&&s| s == 0).count() < 2 {
            continue; // no sign change (at most a touching vertex)
        }
        let mut cross: Vec<usize> = vec![];
        for k in 0..3 {
            let (i, j) = (k, (k + 1) % 3);
            if sg[i] == 0 {
                cross.push(id_of(pos(c[i]), nrm(c[i]), &mut pts));
            }
            if sg[i] * sg[j] < 0 {
                let (mut a, mut b) = (i, j);
                // canonical orientation so the two triangles sharing the edge agree bit for bit
                if key(pos(c[a])) > key(pos(c[b])) {
                    std::mem::swap(&mut a, &mut b);
                }
                let (pa, pb) = (pos(c[a]), pos(c[b]));
                let (fa, fb) = (fv[a], fv[b]);
                let s = if (fa - fb).abs() > 1e-300 { (fa / (fa - fb)).clamp(0.0, 1.0) } else { 0.5 };
                let n = nrm(c[a]) * (1.0 - s) + nrm(c[b]) * s;
                let n = if n.len() > 1e-12 { n / n.len() } else { n };
                cross.push(id_of(pa + (pb - pa) * s, n, &mut pts));
            }
        }
        if cross.len() != 2 || cross[0] == cross[1] {
            continue;
        }
        if sg.iter().filter(|&&s| s == 0).count() == 2 {
            let third = sg.iter().find(|&&s| s != 0).copied().unwrap_or(0);
            let e = (cross[0].min(cross[1]), cross[0].max(cross[1]));
            let z = zero_edges.entry(e).or_insert((false, false, 0));
            z.0 |= third > 0;
            z.1 |= third < 0;
            z.2 += 1;
        } else {
            segs.push((cross[0], cross[1]));
        }
    }
    for (e, (pos_, neg, n)) in zero_edges {
        if (pos_ && neg) || n == 1 {
            segs.push(e);
        }
    }
    if segs.is_empty() {
        return vec![];
    }
    // 3. chain the segments
    let mut adj: Vec<Vec<usize>> = vec![vec![]; pts.len()];
    for (si, &(a, b)) in segs.iter().enumerate() {
        adj[a].push(si);
        adj[b].push(si);
    }
    let mut used = vec![false; segs.len()];
    let mut order: Vec<usize> = (0..pts.len()).filter(|&i| adj[i].len() == 1).collect();
    order.extend((0..pts.len()).filter(|&i| adj[i].len() != 1));
    let mut chains: Vec<Vec<usize>> = vec![];
    for start in order {
        loop {
            let Some(&s0) = adj[start].iter().find(|&&s| !used[s]) else { break };
            used[s0] = true;
            let mut chain = vec![start];
            let mut cur = if segs[s0].0 == start { segs[s0].1 } else { segs[s0].0 };
            chain.push(cur);
            while let Some(&s) = adj[cur].iter().find(|&&s| !used[s]) {
                used[s] = true;
                cur = if segs[s].0 == cur { segs[s].1 } else { segs[s].0 };
                chain.push(cur);
            }
            chains.push(chain);
        }
    }
    // 4. simplify
    chains
        .into_iter()
        .map(|c| {
            let p: Vec<Vec3> = c.iter().map(|&i| pts[i]).collect();
            simplify_dp(&p, simplify).into_iter().map(|i| (pts[c[i]], nrms[c[i]])).collect::<Vec<_>>()
        })
        .filter(|p| p.len() >= 2 && polyline_len(p) > simplify)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two triangles of one face whose vertex normals swing through the view
    /// plane give one contour segment of the expected length.
    #[test]
    fn contour_of_two_triangles() {
        let mut m = ck_tess::mesh::Mesh::default();
        let v = [[0.0f32, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]];
        let n = [[0.0f32, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 0.0, -1.0], [0.0, 0.0, 1.0]];
        for t in [[0usize, 1, 2], [0, 2, 3]] {
            for &i in &t {
                m.positions.extend_from_slice(&v[i]);
                m.normals.extend_from_slice(&n[i]);
            }
            m.tri_face.push(0);
        }
        let c = face_contours(&m, 0, Vec3::new(0.0, 0.0, 1.0), 1.0, 1e-6);
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].len(), 2);
        let l = (c[0][1].0 - c[0][0].0).len();
        assert!((l - 1.0).abs() < 1e-6, "{l}");
        // a face id with no triangles has no contour
        assert!(face_contours(&m, 1, Vec3::new(0.0, 0.0, 1.0), 1.0, 1e-6).is_empty());
    }
}

fn polyline_len(p: &[(Vec3, Vec3)]) -> f64 {
    p.windows(2).map(|w| (w[1].0 - w[0].0).len()).sum()
}

fn dist_seg(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    if l2 < 1e-300 {
        return (p - a).len();
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    (p - (a + ab * t)).len()
}

/// Douglas-Peucker; a closed loop (first == last) is split at its farthest point first.
fn simplify_dp(p: &[Vec3], eps: f64) -> Vec<usize> {
    let n = p.len();
    if n <= 2 {
        return (0..n).collect();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    let mut stack = vec![(0usize, n - 1)];
    if (p[0] - p[n - 1]).len() < 1e-12 {
        let far = (1..n - 1).max_by(|&i, &j| (p[i] - p[0]).len().total_cmp(&(p[j] - p[0]).len())).unwrap();
        keep[far] = true;
        stack = vec![(0, far), (far, n - 1)];
    }
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (mut worst, mut wi) = (0.0, a);
        for i in a + 1..b {
            let d = dist_seg(p[i], p[a], p[b]);
            if d > worst {
                worst = d;
                wi = i;
            }
        }
        if worst > eps {
            keep[wi] = true;
            stack.push((a, wi));
            stack.push((wi, b));
        }
    }
    (0..n).filter(|&i| keep[i]).collect()
}
