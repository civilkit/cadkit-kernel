//! Contact test for motion studies: no booleans, only the tessellation.
//!
//! Every component's triangles are taken once at the assembled pose ([`Scene::build`]); a pose moves them rigidly. Two components are
//! *in contact* when their surfaces properly cross (an edge of one pierces a triangle of the other: touching or coplanar overlap does
//! not count) and *near* when the closest distance between the surfaces is under the clearance. The triangles are the tessellation,
//! so the answer is within the chord height of the true surfaces (the scene is built with a fine chord) and the side tests use
//! [`EPS_SIDE`] because the mesh is stored in f32.
//!
//! Pairs that already touch at the pose a sweep starts from are *mated contact*: a pair that crosses there is ignored (reported as
//! `startContacts`), a pair at distance under [`TOUCH`] there is exempt from the clearance test (it still stops the motion if it crosses).

use super::mate::{Assembly, Xf};
use super::mate_kinds::inv;
use ck_core::linalg::Vec3;
use ck_tess::curvature::Tol;
use ck_topo::body::Body;

/// A surface must be this far (mm, 5 um: the mesh is f32 and a closed loop settles to about this) beyond the other triangle's plane on both ends of an edge to count as piercing it.
pub const EPS_SIDE: f64 = 5e-3;
/// Closest distance (mm) under which two surfaces count as touching at the start pose.
pub const TOUCH: f64 = 2e-2;

type Tri = [Vec3; 3];

/// The triangles of every component (and, last, of everything that is no component), at the assembled pose.
pub struct Scene {
    groups: Vec<Vec<Tri>>,
    /// Placement of each component when the triangles were taken.
    base: Vec<Xf>,
    ids: Vec<String>,
}

/// Why two components are reported.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Kind {
    /// The surfaces cross.
    Cross,
    /// Closer than the clearance.
    Near,
}

#[derive(Debug, Clone)]
pub struct Contact {
    pub a: usize,
    pub b: usize,
    pub kind: Kind,
    /// Closest distance between the surfaces (0 for a crossing).
    pub distance: f64,
    pub at: Vec3,
}

/// Pairs to ignore, by `(i, j)` with `i < j`: `all` = crossing at the start, `near` = touching at the start.
#[derive(Default, Clone)]
pub struct Exempt {
    pub all: Vec<(usize, usize)>,
    pub near: Vec<(usize, usize)>,
}

impl Scene {
    /// Triangles of `body` by component (face ranges of `asm`), at the placements `asm` has now. `chord` is the mesh chord height (mm).
    pub fn build(body: &Body, asm: &Assembly, chord: f64) -> Scene {
        let tess = ck_tess::mesh::tessellate_with(body, &Tol::new(chord, 0.2));
        let n = asm.components.len();
        let nf = body.num_faces();
        let mut owner = vec![n; nf.max(1)];
        for (k, c) in asm.components.iter().enumerate() {
            for f in c.range.faces.0..c.range.faces.1.min(nf) {
                owner[f] = k;
            }
        }
        let m = &tess.mesh;
        let mut groups: Vec<Vec<Tri>> = vec![vec![]; n + 1];
        for (t, &f) in m.tri_face.iter().enumerate() {
            let g = owner.get(f as usize).copied().unwrap_or(n);
            let v = |i: usize| Vec3::new(m.positions[t * 9 + i * 3] as f64, m.positions[t * 9 + i * 3 + 1] as f64, m.positions[t * 9 + i * 3 + 2] as f64);
            groups[g].push([v(0), v(1), v(2)]);
        }
        Scene { groups, base: asm.components.iter().map(|c| c.place).collect(), ids: asm.components.iter().map(|c| c.id.clone()).collect() }
    }
    pub fn components(&self) -> usize {
        self.ids.len()
    }
    pub fn id(&self, k: usize) -> &str {
        self.ids.get(k).map_or("", |s| s.as_str())
    }
    pub fn triangles(&self, k: usize) -> usize {
        self.groups.get(k).map_or(0, |g| g.len())
    }

    /// The groups at `places`: a moved component's triangles are transformed, the others borrowed.
    fn posed(&self, places: &[Xf]) -> Vec<Option<Vec<Tri>>> {
        (0..self.groups.len())
            .map(|k| {
                let (Some(p), Some(b)) = (places.get(k), self.base.get(k)) else { return None };
                let d = p.compose(&inv(b));
                if is_identity(&d) {
                    return None;
                }
                Some(self.groups[k].iter().map(|t| [d.point(t[0]), d.point(t[1]), d.point(t[2])]).collect())
            })
            .collect()
    }

    /// Every pair of components that is in contact or near at `places`, one entry per pair (the crossing, else the closest
    /// approach), skipping the exempt pairs. `stop_first` returns as soon as one is found.
    pub fn contacts(&self, places: &[Xf], clearance: f64, exempt: &Exempt, stop_first: bool) -> Vec<Contact> {
        let posed = self.posed(places);
        let tris = |k: usize| -> &[Tri] { posed[k].as_deref().unwrap_or(&self.groups[k]) };
        let n = self.groups.len();
        let boxes: Vec<Option<(Vec3, Vec3)>> = (0..n).map(|k| bbox(tris(k))).collect();
        let mut out = vec![];
        for i in 0..n {
            for j in i + 1..n {
                // two components that did not move cannot start touching
                if posed[i].is_none() && posed[j].is_none() {
                    continue;
                }
                if exempt.all.contains(&(i, j)) {
                    continue;
                }
                let (Some(bi), Some(bj)) = (boxes[i], boxes[j]) else { continue };
                let reach = clearance.max(EPS_SIDE);
                if !overlap(bi, bj, reach) {
                    continue;
                }
                let want_near = clearance > 0.0 && !exempt.near.contains(&(i, j));
                let found = pair(i, j, tris(i), tris(j), bi, bj, if want_near { clearance } else { 0.0 }).or_else(|| nested(i, j, tris(i), tris(j), bi, bj));
                if let Some(c) = found {
                    out.push(c);
                    if stop_first {
                        return out;
                    }
                }
            }
        }
        out
    }

    /// The pairs that touch at `places`: those that cross, and those closer than [`TOUCH`] (used to exempt them for a sweep).
    pub fn baseline(&self, places: &[Xf]) -> Exempt {
        let posed = self.posed(places);
        let tris = |k: usize| -> &[Tri] { posed[k].as_deref().unwrap_or(&self.groups[k]) };
        let n = self.groups.len();
        let boxes: Vec<Option<(Vec3, Vec3)>> = (0..n).map(|k| bbox(tris(k))).collect();
        let mut ex = Exempt::default();
        for i in 0..n {
            for j in i + 1..n {
                let (Some(bi), Some(bj)) = (boxes[i], boxes[j]) else { continue };
                if !overlap(bi, bj, TOUCH) {
                    continue;
                }
                if let Some(c) = pair(i, j, tris(i), tris(j), bi, bj, TOUCH).or_else(|| nested(i, j, tris(i), tris(j), bi, bj)) {
                    if c.kind == Kind::Cross {
                        ex.all.push((i, j));
                    } else {
                        ex.near.push((i, j));
                    }
                }
            }
        }
        ex
    }
}

fn is_identity(x: &Xf) -> bool {
    let i = Xf::IDENTITY;
    x.r.iter().zip(i.r.iter()).all(|(a, b)| (a - b).abs() < 1e-12) && x.t.iter().all(|t| t.abs() < 1e-9)
}

fn bbox(t: &[Tri]) -> Option<(Vec3, Vec3)> {
    let mut it = t.iter().flat_map(|t| t.iter());
    let f = *it.next()?;
    Some(it.fold((f, f), |(lo, hi), p| (lo.min(*p), hi.max(*p))))
}

fn overlap(a: (Vec3, Vec3), b: (Vec3, Vec3), r: f64) -> bool {
    a.0.x <= b.1.x + r && b.0.x <= a.1.x + r && a.0.y <= b.1.y + r && b.0.y <= a.1.y + r && a.0.z <= b.1.z + r && b.0.z <= a.1.z + r
}

fn tri_box(t: &Tri) -> (Vec3, Vec3) {
    (t[0].min(t[1]).min(t[2]), t[0].max(t[1]).max(t[2]))
}

/// One component wholly inside the other (counts as a crossing).
fn nested(i: usize, j: usize, ti: &[Tri], tj: &[Tri], bi: (Vec3, Vec3), bj: (Vec3, Vec3)) -> Option<Contact> {
    let p = contained(ti, tj, bj).or_else(|| contained(tj, ti, bi))?;
    Some(Contact { a: i, b: j, kind: Kind::Cross, distance: 0.0, at: p })
}

/// Contact of two components: the crossing if any (distance 0), else the closest approach under `near` (0 = do not look).
fn pair(i: usize, j: usize, ti: &[Tri], tj: &[Tri], bi: (Vec3, Vec3), bj: (Vec3, Vec3), near: f64) -> Option<Contact> {
    let reach = near.max(EPS_SIDE);
    // triangles of each side that reach the other's box
    let ci: Vec<usize> = (0..ti.len()).filter(|&k| overlap(tri_box(&ti[k]), bj, reach)).collect();
    let cj: Vec<usize> = (0..tj.len()).filter(|&k| overlap(tri_box(&tj[k]), bi, reach)).collect();
    if ci.is_empty() || cj.is_empty() {
        return None;
    }
    // grid over the candidates of j
    let lo = bi.0.max(bj.0) - Vec3::new(reach, reach, reach);
    let hi = bi.1.min(bj.1) + Vec3::new(reach, reach, reach);
    let ext = hi - lo;
    let avg = {
        let s: f64 = cj.iter().map(|&k| (tri_box(&tj[k]).1 - tri_box(&tj[k]).0).len()).sum();
        s / cj.len() as f64
    };
    let longest = ext.x.max(ext.y).max(ext.z).max(1e-9);
    let cell = avg.max(longest / 48.0).max(reach).max(1e-6);
    let dim = |e: f64| ((e / cell).ceil() as usize).clamp(1, 64);
    let (nx, ny, nz) = (dim(ext.x), dim(ext.y), dim(ext.z));
    let idx = |p: Vec3| -> (usize, usize, usize) {
        let f = |v: f64, l: f64, n: usize| (((v - l) / cell).floor().max(0.0) as usize).min(n - 1);
        (f(p.x, lo.x, nx), f(p.y, lo.y, ny), f(p.z, lo.z, nz))
    };
    let mut grid: Vec<Vec<u32>> = vec![vec![]; nx * ny * nz];
    for &k in &cj {
        let (a, b) = tri_box(&tj[k]);
        let (a, b) = (idx(a - Vec3::new(reach, reach, reach)), idx(b + Vec3::new(reach, reach, reach)));
        for x in a.0..=b.0 {
            for y in a.1..=b.1 {
                for z in a.2..=b.2 {
                    grid[(x * ny + y) * nz + z].push(k as u32);
                }
            }
        }
    }
    let mut best: Option<Contact> = None;
    let mut seen: Vec<u32> = vec![];
    for &a in &ci {
        let (lo_a, hi_a) = tri_box(&ti[a]);
        let (g0, g1) = (idx(lo_a - Vec3::new(reach, reach, reach)), idx(hi_a + Vec3::new(reach, reach, reach)));
        seen.clear();
        for x in g0.0..=g1.0 {
            for y in g0.1..=g1.1 {
                for z in g0.2..=g1.2 {
                    seen.extend_from_slice(&grid[(x * ny + y) * nz + z]);
                }
            }
        }
        seen.sort_unstable();
        seen.dedup();
        for &b in &seen {
            let (tb, ta) = (&tj[b as usize], &ti[a]);
            if !overlap(tri_box(ta), tri_box(tb), reach) {
                continue;
            }
            if let Some(p) = crossing(ta, tb) {
                return Some(Contact { a: i, b: j, kind: Kind::Cross, distance: 0.0, at: p });
            }
            if near > 0.0 {
                let (d, p) = distance(ta, tb);
                if d < near && best.as_ref().map_or(true, |c| d < c.distance) {
                    best = Some(Contact { a: i, b: j, kind: Kind::Near, distance: d, at: p });
                }
            }
        }
    }
    best
}

// ------------------------------------------------------------------ containment

/// Whether `p` is inside the closed surface `t` (parity of the crossings of a ray, slightly off the axes so it avoids shared edges).
fn inside(p: Vec3, t: &[Tri], bx: (Vec3, Vec3)) -> bool {
    if p.x < bx.0.x || p.x > bx.1.x || p.y < bx.0.y || p.y > bx.1.y || p.z < bx.0.z || p.z > bx.1.z {
        return false;
    }
    let d = Vec3::new(1.0, 3.1e-3, 1.7e-3);
    let mut n = 0usize;
    for tr in t {
        // Moller-Trumbore
        let (e1, e2) = (tr[1] - tr[0], tr[2] - tr[0]);
        let h = d.cross(e2);
        let a = e1.dot(h);
        if a.abs() < 1e-14 {
            continue;
        }
        let f = 1.0 / a;
        let s = p - tr[0];
        let u = f * s.dot(h);
        if !(0.0..=1.0).contains(&u) {
            continue;
        }
        let q = s.cross(e1);
        let v = f * d.dot(q);
        if v < 0.0 || u + v > 1.0 {
            continue;
        }
        if f * e2.dot(q) > 1e-9 {
            n += 1;
        }
    }
    n % 2 == 1
}

/// A point of `a` that lies inside the closed surface `b`, when `a` is wholly inside it (three spread samples must all be inside, so
/// two bodies that merely touch do not count). A body wholly inside another has no surface crossing, so [`pair`] cannot see it.
fn contained(a: &[Tri], b: &[Tri], bb: (Vec3, Vec3)) -> Option<Vec3> {
    if a.is_empty() || b.is_empty() {
        return None;
    }
    // samples: centroids of three spread triangles, nudged 1 % toward the middle of the body so that a face lying flush on the other's
    // surface (a cap against a plate) does not sit exactly on it
    let mid = {
        let mut s = Vec3::ZERO;
        for t in a {
            s = s + (t[0] + t[1] + t[2]).scale(1.0 / 3.0);
        }
        s.scale(1.0 / a.len() as f64)
    };
    let pts: Vec<Vec3> = [0, a.len() / 3, 2 * a.len() / 3]
        .iter()
        .map(|&k| {
            let c = (a[k][0] + a[k][1] + a[k][2]).scale(1.0 / 3.0);
            c + (mid - c).scale(0.01)
        })
        .collect();
    pts.iter().all(|p| inside(*p, b, bb)).then_some(pts[0])
}

// ------------------------------------------------------------------ triangle geometry

/// Where segment `p q` pierces triangle `t` (both ends beyond [`EPS_SIDE`] of its plane, on opposite sides, inside the triangle).
fn seg_tri(p: Vec3, q: Vec3, t: &Tri) -> Option<Vec3> {
    let (a, b, c) = (t[0], t[1], t[2]);
    let n = (b - a).cross(c - a);
    let nl = n.len();
    if nl < 1e-18 {
        return None;
    }
    let n = n.scale(1.0 / nl);
    let (dp, dq) = (n.dot(p - a), n.dot(q - a));
    if !((dp > EPS_SIDE && dq < -EPS_SIDE) || (dp < -EPS_SIDE && dq > EPS_SIDE)) {
        return None;
    }
    let x = p + (q - p).scale(dp / (dp - dq));
    // the piercing point must be inside by the same margin: a rim that only touches (a cylinder's end against a plate) is not a crossing
    let within = |p: Vec3, q: Vec3| {
        let e = q - p;
        let l = e.len();
        l > 0.0 && n.dot(e.cross(x - p)) / l >= EPS_SIDE
    };
    let inside = within(a, b) && within(b, c) && within(c, a);
    inside.then_some(x)
}

/// A point where the two triangles properly cross, if they do.
pub fn crossing(a: &Tri, b: &Tri) -> Option<Vec3> {
    for k in 0..3 {
        if let Some(p) = seg_tri(a[k], a[(k + 1) % 3], b) {
            return Some(p);
        }
        if let Some(p) = seg_tri(b[k], b[(k + 1) % 3], a) {
            return Some(p);
        }
    }
    None
}

fn closest_on_tri(p: Vec3, t: &Tri) -> Vec3 {
    // Ericson, Real-Time Collision Detection, 5.1.5
    let (a, b, c) = (t[0], t[1], t[2]);
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab.scale(d1 / (d1 - d3));
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac.scale(d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b).scale((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let den = 1.0 / (va + vb + vc);
    a + ab.scale(vb * den) + ac.scale(vc * den)
}

fn closest_segs(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (Vec3, Vec3) {
    // Ericson 5.1.9
    let (d1, d2, r) = (q1 - p1, q2 - p2, p1 - p2);
    let (a, e, f) = (d1.dot(d1), d2.dot(d2), d2.dot(r));
    let eps = 1e-18;
    let (s, t);
    if a <= eps && e <= eps {
        return (p1, p2);
    }
    if a <= eps {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= eps {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let den = a * e - b * b;
            let mut s0 = if den > eps { ((b * f - c * e) / den).clamp(0.0, 1.0) } else { 0.0 };
            let mut t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t0 = 0.0;
                s0 = (-c / a).clamp(0.0, 1.0);
            } else if t0 > 1.0 {
                t0 = 1.0;
                s0 = ((b - c) / a).clamp(0.0, 1.0);
            }
            s = s0;
            t = t0;
        }
    }
    (p1 + d1.scale(s), p2 + d2.scale(t))
}

/// Closest distance between two triangles that do not cross, and the midpoint of the closest pair of points.
pub fn distance(a: &Tri, b: &Tri) -> (f64, Vec3) {
    let mut best = (f64::INFINITY, Vec3::ZERO);
    let mut take = |p: Vec3, q: Vec3| {
        let d = p.dist(q);
        if d < best.0 {
            best = (d, (p + q).scale(0.5));
        }
    };
    for k in 0..3 {
        take(a[k], closest_on_tri(a[k], b));
        take(closest_on_tri(b[k], a), b[k]);
        for l in 0..3 {
            let (p, q) = closest_segs(a[k], a[(k + 1) % 3], b[l], b[(l + 1) % 3]);
            take(p, q);
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }
    #[test]
    fn crossing_triangles_are_found_and_touching_ones_are_not() {
        let flat: Tri = [v(0.0, 0.0, 0.0), v(10.0, 0.0, 0.0), v(0.0, 10.0, 0.0)];
        let through: Tri = [v(2.0, 2.0, -1.0), v(2.0, 2.0, 1.0), v(4.0, 2.0, 0.0)];
        let p = crossing(&flat, &through).expect("crosses");
        assert!((p.z).abs() < 1e-9 && (p.x - 2.0).abs() < 1e-9);
        let touching: Tri = [v(2.0, 2.0, 0.0), v(2.0, 2.0, 1.0), v(4.0, 2.0, 1.0)];
        assert!(crossing(&flat, &touching).is_none(), "an edge ending on the plane only touches");
        let coplanar: Tri = [v(1.0, 1.0, 0.0), v(5.0, 1.0, 0.0), v(1.0, 5.0, 0.0)];
        assert!(crossing(&flat, &coplanar).is_none(), "coplanar overlap is not a crossing");
    }
    #[test]
    fn distances_are_exact() {
        let a: Tri = [v(0.0, 0.0, 0.0), v(10.0, 0.0, 0.0), v(0.0, 10.0, 0.0)];
        let above: Tri = [v(1.0, 1.0, 0.5), v(3.0, 1.0, 0.5), v(1.0, 3.0, 0.5)];
        let (d, at) = distance(&a, &above);
        assert!((d - 0.5).abs() < 1e-12 && (at.z - 0.25).abs() < 1e-12, "{d} {at:?}");
        // edge to edge, skew: x axis edge vs an edge along y at height 2, shifted
        let b: Tri = [v(20.0, 0.0, 0.0), v(30.0, 0.0, 0.0), v(20.0, 10.0, 0.0)];
        let c: Tri = [v(25.0, -5.0, 2.0), v(25.0, 5.0, 2.0), v(25.0, 5.0, 8.0)];
        let (d, _) = distance(&b, &c);
        assert!((d - 2.0).abs() < 1e-12, "{d}");
    }
}

#[cfg(test)]
mod nested_tests {
    use super::*;
    fn cube(lo: f64, hi: f64) -> Vec<Tri> {
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
        let c = [v(lo, lo, lo), v(hi, lo, lo), v(hi, hi, lo), v(lo, hi, lo), v(lo, lo, hi), v(hi, lo, hi), v(hi, hi, hi), v(lo, hi, hi)];
        let q = |a: usize, b: usize, cc: usize, d: usize| [[c[a], c[b], c[cc]], [c[a], c[cc], c[d]]];
        [q(0, 1, 2, 3), q(4, 5, 6, 7), q(0, 1, 5, 4), q(2, 3, 7, 6), q(1, 2, 6, 5), q(0, 3, 7, 4)].concat()
    }
    #[test]
    fn a_cube_inside_a_cube_is_nested_and_one_beside_it_is_not() {
        let big = cube(0.0, 10.0);
        let bb = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0));
        assert!(contained(&cube(3.0, 5.0), &big, bb).is_some());
        assert!(contained(&cube(11.0, 12.0), &big, bb).is_none());
        // touching faces (a cube sitting on the big one) is not nested
        assert!(contained(&cube(10.0, 12.0), &big, bb).is_none());
        assert!(contained(&big, &cube(3.0, 5.0), (Vec3::new(3.0, 3.0, 3.0), Vec3::new(5.0, 5.0, 5.0))).is_none());
    }
}
