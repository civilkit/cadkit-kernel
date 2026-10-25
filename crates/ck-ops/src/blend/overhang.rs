//! The overhang corner (docs/mimo-brief-overhang-corner.md): four straight
//! selected edges meeting at one 4-valent vertex as two collinear pairs that
//! flip convex/concave, so no rolling ball runs through the corner and the
//! ordinary stripes end in planar steps.
//!
//! The corner is rebuilt after the ordinary stripes: a zone box of half-size
//! `r` about the vertex is cut out of the striped body and replaced by a hand
//! built solid `K`. `K`'s six planar faces exactly match the sections the cut
//! leaves on the box faces (so the union closes with no steps: the step caps
//! lose the sharp material that supported them to the zone, where `K`
//! replaces it), and its seventh face is one B-spline patch through the
//! hexagon where the four trimmed stripes end — G1 to each stripe along its
//! arc, tangent to the two wall planes along the two straight sides. The
//! patch interpolates a Coons grid of the six boundary chains with the first
//! interior rows projected into the neighbours' tangent planes (discrete G1);
//! its spine (the grid's `tau = 0.5` line) lies on the vertex-axis by the
//! chains' point symmetry.
//!
//! `detect` fires only when all of this holds, else the selection takes the
//! ordinary path unchanged: 4-valent vertex, all four incident edges selected,
//! straight between orthogonal planar faces with known convexity, two
//! antipodal direction pairs with convexity flipping inside each pair, each
//! pair sharing a face whose two ball-centre feet coincide, and exactly one
//! cross-pair arc-end coincidence per strike (the hexagon closes).

use super::corner::{Seg, ToolBuilder};
use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role};

/// One stripe of a corner: the unit edge direction away from the vertex,
/// convexity (round cuts / cove fills) and the two face outward normals.
#[derive(Clone)]
struct Strike {
    dir: Vec3,
    convex: bool,
    n: [Vec3; 2],
}

/// A detected overhang corner (see the module doc).
pub(super) struct Corner {
    pub v: Vec3,
    pub r: f64,
    pub edges: [EdgeId; 4],
    /// The corner patch can be built (perpendicular pairs, the boundary
    /// closes); a structural corner without this is refused instead.
    pub patch_ok: bool,
    /// Dot of the two pair directions (the refusal quotes the angle).
    pub a12: f64,
    s: [Strike; 4],
    pairs: [(usize, usize); 2],
}

impl Corner {
    /// Ball centre of strike `i`: `v + (n0 + n1) * (s * r / (1 + n0.n1))`.
    fn centre(&self, i: usize) -> Vec3 {
        let s = &self.s[i];
        let c12 = s.n[0].dot(s.n[1]);
        let sg = if s.convex { -1.0 } else { 1.0 };
        self.v + (s.n[0] + s.n[1]) * (sg * self.r / (1.0 + c12))
    }
    /// The tangent feet of strike `i` on its two faces.
    fn feet(&self, i: usize) -> [Vec3; 2] {
        let c = self.centre(i);
        let s = &self.s[i];
        [0, 1].map(|k| c - s.n[k] * ((c - self.v).dot(s.n[k])))
    }
    /// The arc end points of strike `i`: the feet pushed `r` along the edge
    /// (both feet lie in the plane through `v` perpendicular to the edge).
    fn ends(&self, i: usize) -> [Vec3; 2] {
        let f = self.feet(i);
        [f[0] + self.s[i].dir * self.r, f[1] + self.s[i].dir * self.r]
    }
    /// Centre of strike `i`'s trimmed-end arc (its axis projected into the
    /// end plane `{p : (p - v) . dir = r}`).
    fn arc_centre(&self, i: usize) -> Vec3 {
        let s = &self.s[i];
        let c = self.centre(i);
        c + s.dir * (self.r - (c - self.v).dot(s.dir))
    }
}

/// The overhang corners whose four edges are all in `sel`, or none. `r` is
/// the selection's radius; see the module doc for the conditions.
pub(super) fn detect(b: &Body, sel: &[EdgeId], r: f64) -> Vec<Corner> {
    let mut out = vec![];
    if !(r > 0.0) || !r.is_finite() {
        return out;
    }
    for v in b.vertex_ids() {
        let inc: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
        if inc.len() != 4 || !inc.iter().all(|e| sel.contains(e)) {
            continue;
        }
        let vp = b.vertex(v).p;
        let snap = 1e-6 * (1.0 + vp.len() + r);
        // straight, planar, orthogonal face pairs, known convexity
        let mut strikes: Vec<Strike> = vec![];
        for &e in &inc {
            let ed = b.edge(e);
            let ok = (|| {
                if !matches!(ed.curve, Curve::Line { .. }) || ed.v0 == ed.v1 {
                    return None;
                }
                let concave = super::mixed::line_edge_concave(b, e)?;
                let other = if ed.v0 == v { ed.v1 } else { ed.v0 };
                let fs = b.edge_faces(e);
                let n = [b.planar_normal(fs[0]), b.planar_normal(fs[1])];
                (n[0].dot(n[1]).abs() <= 1e-9).then_some(Strike { dir: (b.vertex(other).p - vp).normalize(), convex: !concave, n })
            })();
            let Some(s) = ok else {
                strikes.clear();
                break;
            };
            strikes.push(s);
        }
        if strikes.len() != 4 {
            continue;
        }
        let mut faces: Vec<FaceId> = vec![];
        for &e in &inc {
            for f in b.edge_faces(e) {
                if !faces.contains(&f) {
                    faces.push(f);
                }
            }
        }
        if faces.len() != 4 {
            continue;
        }
        // two antipodal pairs, convexity flips inside each, pairs orthogonal
        let mut used = [false; 4];
        let mut pairs: Vec<(usize, usize)> = vec![];
        for i in 0..4 {
            if used[i] {
                continue;
            }
            let Some(j) = (0..4).find(|&j| j != i && !used[j] && strikes[i].dir.dot(strikes[j].dir) < -1.0 + 1e-9) else {
                pairs.clear();
                break;
            };
            used[i] = true;
            used[j] = true;
            pairs.push((i, j));
        }
        if pairs.len() != 2 {
            continue;
        }
        let a12 = strikes[pairs[0].0].dir.dot(strikes[pairs[1].0].dir);
        let orthogonal = a12.abs() <= 1e-9;
        if pairs.iter().any(|&(i, j)| strikes[i].convex == strikes[j].convex) {
            continue;
        }
        // patch geometry (feet + closing coincidences) only for perpendicular pairs;
        // a structural corner with skewed pairs is refused by the caller instead
        let mut patch_ok = false;
        if orthogonal {
            let centre = |i: usize| -> Vec3 {
                let s = &strikes[i];
                let c12 = s.n[0].dot(s.n[1]);
                let sg = if s.convex { -1.0 } else { 1.0 };
                vp + (s.n[0] + s.n[1]) * (sg * r / (1.0 + c12))
            };
            let foot = |i: usize, k: usize| centre(i) - strikes[i].n[k] * ((centre(i) - vp).dot(strikes[i].n[k]));
            // each pair shares exactly one face; the two centres' feet on it coincide
            let mut feet_ok = true;
            for &(i, j) in &pairs {
                let mut hit: Vec<(usize, usize)> = vec![];
                for ki in 0..2 {
                    for kj in 0..2 {
                        if strikes[i].n[ki].dot(strikes[j].n[kj]) > 1.0 - 1e-9 {
                            hit.push((ki, kj));
                        }
                    }
                }
                if hit.len() != 1 || foot(i, hit[0].0).dist(foot(j, hit[0].1)) > snap {
                    feet_ok = false;
                    break;
                }
            }
            // exactly one cross-pair arc-end coincidence per strike, pairing the
            // two pairs off (this closes the boundary)
            let end = |i: usize, k: usize| foot(i, k) + strikes[i].dir * r;
            let mut hits: Vec<(usize, usize, usize, usize)> = vec![]; // (pi, k, qj, kj)
            for &i in &[pairs[0].0, pairs[0].1] {
                for k in 0..2 {
                    for &j in &[pairs[1].0, pairs[1].1] {
                        for kj in 0..2 {
                            if end(i, k).dist(end(j, kj)) <= snap {
                                hits.push((i, k, j, kj));
                            }
                        }
                    }
                }
            }
            let distinct = hits.len() == 2 && hits[0].0 != hits[1].0 && hits[0].2 != hits[1].2;
            patch_ok = feet_ok && distinct;
            // an orthogonal corner whose boundary does not close: fall back to the
            // ordinary stepped build (today's behaviour), no refusal
            if !patch_ok {
                continue;
            }
        }
        let Ok(s) = <[Strike; 4]>::try_from(strikes) else { continue };
        let Ok(edges) = <[EdgeId; 4]>::try_from(inc) else { continue };
        out.push(Corner { v: vp, r, edges, patch_ok, a12, s, pairs: [pairs[0], pairs[1]] });
    }
    out
}

/// A section of `body` on the plane `{p : (p - origin) . outward = 0}` as
/// segments oriented counter-clockwise about `outward`: the one face of
/// `body` lying in the plane with its outward normal to `-outward` (the cut's
/// section faces point into the zone), its single loop read off directly.
fn section(body: &Body, origin: Vec3, outward: Vec3, r: f64) -> Result<Vec<(Vec3, Vec3, Option<(Vec3, f64)>)>, BlendError> {
    let fail = || BlendError::detail("blendUnsupported", "overhang corner: zone section is not one planar face");
    let mut hits: Vec<FaceId> = vec![];
    for f in body.face_ids() {
        let Surface::Plane { frame } = &body.face(f).surface else { continue };
        if frame.z.dot(outward).abs() < 1.0 - 1e-9 || ((frame.o - origin).dot(outward)).abs() > 1e-6 * (1.0 + r) {
            continue;
        }
        if body.planar_normal(f).dot(-outward) > 0.9 {
            hits.push(f);
        }
    }
    if hits.len() != 1 || body.face(hits[0]).loops.len() != 1 {
        return Err(fail());
    }
    let lp = body.face(hits[0]).loops[0];
    let mut segs: Vec<(Vec3, Vec3, Option<(Vec3, f64)>)> = vec![];
    for &cid in &body.lp(lp).coedges {
        let co = body.coedge(cid);
        let ed = body.edge(co.edge);
        let (a, b) = (body.vertex(ed.v0).p, body.vertex(ed.v1).p);
        let (pa, pb) = if co.rev { (b, a) } else { (a, b) };
        let arc = match ed.curve {
            Curve::Line { .. } => None,
            Curve::Circle { frame, r } => Some((frame.o, r)),
            _ => return Err(fail()),
        };
        segs.push((pa, pb, arc));
    }
    // the loop as extracted runs about -outward; flip if needed (chord winding
    // agrees with plane_face's arc-sampled test for arcs under 180 degrees)
    let pts: Vec<Vec3> = segs.iter().map(|s| s.0).collect();
    let mut area = Vec3::ZERO;
    for i in 0..pts.len() {
        area = area + pts[i].cross(pts[(i + 1) % pts.len()]);
    }
    if area.dot(outward) < 0.0 {
        segs.reverse();
        for s in &mut segs {
            std::mem::swap(&mut s.0, &mut s.1);
        }
    }
    Ok(segs)
}

/// The zone box about `v`: half-size `r` along the orthonormal axes `a`,
/// six quads wound counter-clockwise about their outward normals.
fn zone_box(v: Vec3, r: f64, a: [Vec3; 3]) -> Body {
    let corner = |s: [i32; 3]| -> Vec3 { v + a[0] * (s[0] as f64 * r) + a[1] * (s[1] as f64 * r) + a[2] * (s[2] as f64 * r) };
    let mut polys: Vec<(Vec<Vec3>, FaceName)> = vec![];
    for k in 0..3usize {
        let (i, j) = match k {
            0 => (1usize, 2usize),
            1 => (2, 0),
            _ => (0, 1),
        };
        for &sk in &[-1i32, 1] {
            let mut q: Vec<Vec3> = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
                .iter()
                .map(|&(si, sj)| {
                    let mut s = [0i32; 3];
                    s[k] = sk;
                    s[i] = si;
                    s[j] = sj;
                    corner(s)
                })
                .collect();
            let mut n = Vec3::ZERO;
            for t in 0..4 {
                let (p, qq) = (q[t], q[(t + 1) % 4]);
                n = n + Vec3::new((p.y - qq.y) * (p.z + qq.z), (p.z - qq.z) * (p.x + qq.x), (p.x - qq.x) * (p.y + qq.y));
            }
            if n.dot(a[k] * sk as f64) < 0.0 {
                q.reverse();
            }
            polys.push((q, FaceName::other("zone", &format!("k{}{}", k, sk))));
        }
    }
    super::vertex::polyhedron(&polys)
}

/// Unit start direction and sweep of the shorter arc from `a` to `b` about `c`.
fn arc_sweep(a: Vec3, b: Vec3, c: Vec3) -> (Vec3, f64) {
    let (ea, eb) = ((a - c).normalize(), (b - c).normalize());
    (ea, ea.dot(eb).clamp(-1.0, 1.0).acos())
}

/// Point at fraction `t` of the compound chain of arcs
/// `(from, to, centre, axis, radius)`; the rotation runs the shorter way, so
/// the axis is derived from the arc's own endpoints (a strike direction has
/// no consistent sign across the four arcs — trusting it mirrors two of them
/// off the hexagon).
fn chain_at(segs: &[(Vec3, Vec3, Vec3, Vec3, f64)], t: f64) -> Vec3 {
    let sweeps: Vec<(Vec3, Vec3, Vec3, f64, f64)> = segs
        .iter()
        .map(|&(a, b, c, ax, rr)| {
            let ea = (a - c).normalize();
            let eb = (b - c).normalize();
            let th = ea.dot(eb).clamp(-1.0, 1.0).acos();
            let cr = ea.cross(eb);
            let axis = if cr.len() > 1e-12 { cr.normalize() } else { ax };
            (c, ea, axis, th, rr)
        })
        .collect();
    let total: f64 = sweeps.iter().map(|s| s.3).sum();
    let mut want = t * total;
    for (c, ea, axis, th, rr) in &sweeps {
        if want <= *th {
            return *c + (*ea * want.cos() + axis.cross(*ea) * want.sin()) * *rr;
        }
        want -= th;
    }
    let (c, ea, axis, th, rr) = sweeps.last().unwrap();
    *c + (*ea * th.cos() + axis.cross(*ea) * th.sin()) * *rr
}

/// Fill the interior of `grid` (the four boundary rows/columns are already set) with
/// the discrete Dirichlet Laplace solution.  Computed exactly in the sine basis of the
/// grid graph, which is orthonormal and symmetric, so the two transforms are the same
/// matrix product and the result is deterministic.
fn harmonic_fill(grid: &mut [Vec<Vec3>], n: usize) {
    let m = n - 2;
    let den = m as f64 + 1.0;
    let scale = (2.0 / den).sqrt();
    let mut v = vec![vec![0.0f64; m]; m];
    let mut lam = vec![0.0f64; m];
    for k in 0..m {
        lam[k] = 2.0 - 2.0 * ((k + 1) as f64 * std::f64::consts::PI / den).cos();
        for i in 0..m {
            v[k][i] = scale * ((k + 1) as f64 * (i + 1) as f64 * std::f64::consts::PI / den).sin();
        }
    }
    let mul = |a: &[Vec<f64>], b: &[Vec<f64>]| -> Vec<Vec<f64>> {
        let mut out = vec![vec![0.0f64; m]; m];
        for i in 0..m {
            for k in 0..m {
                let aik = a[i][k];
                for j in 0..m {
                    out[i][j] += aik * b[k][j];
                }
            }
        }
        out
    };
    let comp = |p: Vec3, c: usize| -> f64 {
        match c {
            0 => p.x,
            1 => p.y,
            _ => p.z,
        }
    };
    for c in 0..3 {
        let mut b = vec![vec![0.0f64; m]; m];
        for j in 0..m {
            for i in 0..m {
                if i == 0 {
                    b[i][j] += comp(grid[0][j + 1], c);
                }
                if i == m - 1 {
                    b[i][j] += comp(grid[n - 1][j + 1], c);
                }
                if j == 0 {
                    b[i][j] += comp(grid[i + 1][0], c);
                }
                if j == m - 1 {
                    b[i][j] += comp(grid[i + 1][n - 1], c);
                }
            }
        }
        let mut y = mul(&mul(&v, &b), &v);
        for k in 0..m {
            for l in 0..m {
                y[k][l] /= lam[k] + lam[l];
            }
        }
        let x = mul(&mul(&v, &y), &v);
        for j in 0..m {
            for i in 0..m {
                let p = &mut grid[i + 1][j + 1];
                match c {
                    0 => p.x = x[i][j],
                    1 => p.y = x[i][j],
                    _ => p.z = x[i][j],
                }
            }
        }
    }
}

/// The patch surface through the hexagon `h` = [W1, A_line, C_line, W2,
/// D_line, B_line] (module doc): a grid over the four boundary chains
/// (bottom W1..B_line through the two arcs, top C_line..D_line, sides the two
/// lines), filled with the discrete Dirichlet Laplace solution so the ~90 deg
/// corners where hexagon sides meet stay on the boundary, with the first
/// interior rows projected into the stripes' cylinder tangent planes and the
/// first interior columns into the wall planes (the seams' G1 neighbours),
/// then tensor-product interpolation of the grid.
fn patch_surface(
    h: &[Vec3; 6],
    bottom: [(Vec3, Vec3, Vec3, Vec3); 2], // (from, to, arc centre, strike centre+axis packed below)
    bottom_ax: [Vec3; 2],
    top: [(Vec3, Vec3, Vec3, Vec3); 2],
    top_ax: [Vec3; 2],
    wall_n: (Vec3, Vec3),
    n: usize,
) -> Result<BSplineSurface, BlendError> {
    let fail = |d: &str| BlendError::detail("blendUnsupported", &format!("overhang corner: patch fit: {d}"));
    if n < 5 || n % 2 == 0 {
        return Err(fail("grid size"));
    }
    let bl = |a: Vec3, b: Vec3, t: f64| a + (b - a) * t;
    // (from, to, arc centre, rotation axis, radius); the strike centre (`bottom[k].3`)
    // is only used for the G1 radial below
    let bc: [(Vec3, Vec3, Vec3, Vec3, f64); 2] = std::array::from_fn(|k| {
        let (a, b, c, _) = bottom[k];
        (a, b, c, bottom_ax[k], (a - c).len())
    });
    let tc: [(Vec3, Vec3, Vec3, Vec3, f64); 2] = std::array::from_fn(|k| {
        let (a, b, c, _) = top[k];
        (a, b, c, top_ax[k], (a - c).len())
    });
    let sig: Vec<f64> = (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
    let bw: Vec<f64> = bc.iter().map(|s| arc_sweep(s.0, s.1, s.2).1).collect();
    let tw: Vec<f64> = tc.iter().map(|s| arc_sweep(s.0, s.1, s.2).1).collect();
    let (bt, tt): (f64, f64) = (bw.iter().sum(), tw.iter().sum());
    // The four boundary chains.  The bottom and top chains each carry the ~90 degree
    // corner where two hexagon sides meet (s = 0.5), so a Coons blend propagates it
    // along the whole s = 0.5 line as a fold; the tessellator then breaks that fold
    // into thousands of needle triangles whose geometric normals flip sign, which is
    // what the renderer draws as a scribble at the pinch.  OCCT's reference patch is
    // a plate fill that confines the corner to the boundary, so the interior is the
    // discrete Dirichlet Laplace solution of the same four chains instead.
    let mut grid = vec![vec![Vec3::ZERO; n]; n];
    for i in 0..n {
        grid[i][0] = chain_at(&bc, sig[i]);
        grid[i][n - 1] = chain_at(&tc, sig[i]);
    }
    for j in 0..n {
        grid[0][j] = bl(h[1], h[2], sig[j]);
        grid[n - 1][j] = bl(h[5], h[4], sig[j]);
    }
    if std::env::var("CK_PATCH_FILL").as_deref() == Ok("coons") {
        for j in 1..n - 1 {
            for i in 1..n - 1 {
                let (s, t) = (sig[i], sig[j]);
                let bpt = chain_at(&bc, s);
                let tpt = chain_at(&tc, s);
                let lpt = bl(h[1], h[2], t);
                let rpt = bl(h[5], h[4], t);
                let bil = bl(bl(h[1], h[5], s), bl(h[2], h[4], s), t);
                grid[i][j] = bpt * (1.0 - t) + tpt * t + lpt * (1.0 - s) + rpt * s - bil;
            }
        }
    } else {
        harmonic_fill(&mut grid, n);
    }
    // strike (centre, axis) covering chain fraction `s`
    let strike_at = |w: &[f64], tot: f64, centres: &[Vec3; 2], ax: &[Vec3; 2], s: f64| -> (Vec3, Vec3) {
        let mut acc = 0.0;
        let mut pick = 1usize;
        for (k, wk) in w.iter().enumerate() {
            acc += wk;
            if s * tot <= acc + 1e-12 {
                pick = k;
                break;
            }
        }
        (centres[pick], ax[pick])
    };
    let bcent: [Vec3; 2] = [bottom[0].3, bottom[1].3];
    let tcent: [Vec3; 2] = [top[0].3, top[1].3];
    // G1 rows/columns: the fitted cubic's endpoint derivative mixes the first
    // three interior lines, so those are projected into the stripe cylinders'
    // tangent planes (one row leaves a g2 term ~24 degrees off). The projection
    // tapers off over the next few lines: a band that stops dead ends in a
    // crease the interpolant then rings around.
    let env_u = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<usize>().ok());
    let g1_full = env_u("CK_G1_FULL").unwrap_or(3);
    let g1_taper = env_u("CK_G1_TAPER").unwrap_or(7).max(g1_full);
    let g1w = |depth: usize| -> f64 {
        if g1_full == 0 {
            return 0.0;
        }
        if depth <= g1_full {
            1.0
        } else if depth >= g1_taper {
            0.0
        } else {
            let t = (depth - g1_full) as f64 / (g1_taper - g1_full) as f64;
            0.5 * (1.0 + (std::f64::consts::PI * t).cos())
        }
    };
    for &(vert, w, tot, cent, ax) in &[(0usize, &bw, bt, &bcent, &bottom_ax), (n - 1, &tw, tt, &tcent, &top_ax)] {
        let step: isize = if vert == 0 { 1 } else { -1 };
        for depth in 1..=g1_taper.max(1) {
            let j = vert as isize + step * depth as isize;
            if j < 0 || j >= n as isize {
                continue;
            }
            let j = j as usize;
            for i in 1..n - 1 {
                let q = grid[i][vert];
                let (sc, axis) = strike_at(w, tot, cent, ax, sig[i]);
                let mut rad = (q - sc) - axis * ((q - sc).dot(axis));
                if rad.len() < 1e-12 {
                    return Err(fail("radial"));
                }
                rad = rad.normalize();
                let p = grid[i][j];
                grid[i][j] = p - rad * ((p - q).dot(rad)) * g1w(depth);
                if std::env::var_os("CK_BLEND_DEBUG").is_some() && depth == 1 && (i == n / 4 || i == 3 * n / 4) {
                    eprintln!("g1 row: i={i} j={j} sc=({:.2},{:.2},{:.2}) shift={:.6}", sc.x, sc.y, sc.z, (p - grid[i][j]).len());
                }
            }
        }
    }
    // G1 columns: same stencil into the wall planes
    for &(vert, nrm) in &[(0usize, wall_n.0), (n - 1, wall_n.1)] {
        let step: isize = if vert == 0 { 1 } else { -1 };
        for depth in 1..=g1_taper.max(1) {
            let i = vert as isize + step * depth as isize;
            if i < 0 || i >= n as isize {
                continue;
            }
            let i = i as usize;
            for j in 1..n - 1 {
                let q = grid[vert][j];
                let p = grid[i][j];
                grid[i][j] = p - nrm * ((p - q).dot(nrm)) * g1w(depth);
            }
        }
    }
    ck_geom::fit::interpolate_surface(&grid, 3, 3).map_err(|e| fail(&e))
}

/// The pcurve of `curve` over `[t0, t1]` on `surf`: exact when the pair has a
/// closed form, else a projected sample fit.
fn refit_pcurve(surf: &Surface, curve: &Curve, t0: f64, t1: f64, tol: f64) -> Option<ck_geom::curve2::Curve2> {
    use ck_geom::pcurve::{exact_pcurve, pcurve_of, pcurve_of_dense};
    if let Some(pc) = boundary_pcurve(surf, curve, t0, t1, tol) {
        return Some(pc);
    }
    exact_pcurve(surf, curve, tol * 10.0)
        .or_else(|| pcurve_of(surf, curve, t0, t1, tol).ok())
        .or_else(|| pcurve_of_dense(surf, curve, t0, t1, tol, 1e-7).ok())
}

/// The pcurve of a curve lying on a B-spline surface's parametric boundary
/// (the hexagon chains of the corner patch): the boundary isoline is exactly
/// on the surface, so the uv is solved one-dimensionally along whichever of
/// the four isolines the curve follows — the generic projection fits jump
/// between near-equal closest points on the patch and come out ~1 mm off.
fn boundary_pcurve(surf: &Surface, c: &Curve, t0: f64, t1: f64, tol: f64) -> Option<ck_geom::curve2::Curve2> {
    let Surface::BSpline(bs) = surf else { return None };
    let n = 64;
    let iso_at = |fixed: bool, fv: f64, x: f64| if fixed { bs.eval(fv, x) } else { bs.eval(x, fv) };
    for &(fixed, fv) in &[(false, 0.0), (false, 1.0), (true, 0.0), (true, 1.0)] {
        let mut uvs: Vec<ck_geom::Vec2> = vec![];
        let mut ok = true;
        for k in 0..=n {
            let t = t0 + (t1 - t0) * k as f64 / n as f64;
            let p = c.eval(t);
            let iso = |x: f64| iso_at(fixed, fv, x);
            let mut best = (0.0f64, f64::INFINITY);
            for s in 0..=256 {
                let x = s as f64 / 256.0;
                let d = iso(x).dist(p);
                if d < best.1 {
                    best = (x, d);
                }
            }
            let (mut lo, mut hi) = ((best.0 - 1.0 / 256.0).max(0.0), (best.0 + 1.0 / 256.0).min(1.0));
            for _ in 0..60 {
                let m1 = lo + (hi - lo) / 3.0;
                let m2 = hi - (hi - lo) / 3.0;
                if iso(m1).dist(p) < iso(m2).dist(p) {
                    hi = m2;
                } else {
                    lo = m1;
                }
            }
            let x = 0.5 * (lo + hi);
            if iso(x).dist(p) > tol * (1.0 + p.len()) {
                ok = false;
                break;
            }
            uvs.push(if fixed { ck_geom::Vec2::new(fv, x) } else { ck_geom::Vec2::new(x, fv) });
        }
        if ok {
            let b = ck_geom::fit::interpolate(&uvs, 3, ck_geom::fit::Param::ChordLength).ok()?;
            return Some(ck_geom::curve2::Curve2::BSpline(b.reparam(t0, t1)));
        }
        if std::env::var_os("CK_BLEND_DEBUG").is_some() {
            let dmax = (0..=8).map(|k| {
                let t = t0 + (t1 - t0) * k as f64 / 8.0;
                let p = c.eval(t);
                let mut best = f64::INFINITY;
                for s in 0..=64 {
                    best = best.min(iso_at(fixed, fv, s as f64 / 64.0).dist(p));
                }
                best
            })
            .fold(0.0, f64::max);
            eprintln!("  isoline fixed={fixed} fv={fv}: coarse dmax {dmax:.6}");
        }
    }
    None
}

/// Repair the union's two known defects, locally (nothing in the boolean walk or assemble
/// is touched): `check`'s open seam twins — a face kept its own copy of the shared curve
/// (the stripe's circle on one side, the patch's copy on the other) because the arrangement
/// did not unify them — are merged onto the analytic curve, and pcurves whose fit is off
/// (`PcurveOff`) are refit or dropped (a missing pcurve is valid; the projection fallback
/// recomputes one when needed).
fn weld_seams(b: &mut Body, tol: f64) {
    let snap = tol * 10.0;
    let open: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge_coedges(e).len() == 1).collect();
    for (i, &x) in open.iter().enumerate() {
        if b.edge_coedges(x).len() != 1 {
            continue;
        }
        let mx = {
            let ed = b.edge(x);
            ed.curve.eval(0.5 * (ed.t0 + ed.t1))
        };
        let rx = if matches!(b.edge(x).curve, Curve::BSpline(_)) { 0 } else { 1 };
        for &y in &open[i + 1..] {
            if b.edge_coedges(x).len() != 1 || b.edge_coedges(y).len() != 1 || x == y {
                continue;
            }
            let (ax, bx, my, ry) = {
                let ey = b.edge(y);
                (b.vertex(ey.v0).p, b.vertex(ey.v1).p, ey.curve.eval(0.5 * (ey.t0 + ey.t1)), if matches!(ey.curve, Curve::BSpline(_)) { 0 } else { 1 })
            };
            let (ex0, ex1) = {
                let ex = b.edge(x);
                (b.vertex(ex.v0).p, b.vertex(ex.v1).p)
            };
            let fwd = ex0.dist(ax) <= snap && ex1.dist(bx) <= snap;
            let rvrs = ex0.dist(bx) <= snap && ex1.dist(ax) <= snap;
            if (!fwd && !rvrs) || mx.dist(my) > snap {
                continue;
            }
            // keep the analytic curve (line / circle over a fitted spline)
            let (keep, lose) = if rx >= ry { (x, y) } else { (y, x) };
            if b.edge_coedges(keep).len() != 1 || b.edge_coedges(lose).len() != 1 {
                continue;
            }
            let (kv0, kv1) = (b.edge(keep).v0, b.edge(keep).v1);
            let (lv0, lv1) = (b.edge(lose).v0, b.edge(lose).v1);
            let same = b.vertex(kv0).p.dist(b.vertex(lv0).p) <= snap && b.vertex(kv1).p.dist(b.vertex(lv1).p) <= snap;
            let co = b.edge_coedges(lose)[0];
            let face = b.coedge_face(co);
            let revc = b.coedge(co).rev;
            b.edges[lose.idx()].coedges.clear();
            b.coedges[co.idx()].edge = keep;
            b.coedges[co.idx()].rev = if same { revc } else { !revc };
            b.edges[keep.idx()].coedges.push(co);
            b.edges[lose.idx()].alive = false;
            let ed = b.edge(keep).clone();
            b.coedges[co.idx()].pcurve = refit_pcurve(&b.face(face).surface, &ed.curve, ed.t0, ed.t1, tol);
            break;
        }
    }
    // leftovers the arrangement kept only for bookkeeping
    let dead: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge_coedges(e).is_empty()).collect();
    for e in dead {
        b.edges[e.idx()].alive = false;
    }
}

fn refit_pcurves(b: &mut Body, tol: f64) {
    let dbg = std::env::var_os("CK_BLEND_DEBUG").is_some();
    let mut fixes: Vec<(ck_topo::body::CoedgeId, Option<ck_geom::curve2::Curve2>)> = vec![];
    let mut flips: Vec<EdgeId> = vec![];
    for issue in ck_topo::check::check(b, tol) {
        match issue {
            ck_topo::check::Issue::PcurveOff(c, dev) => {
                let co = b.coedge(c);
                let face = b.coedge_face(c);
                let ed = b.edge(co.edge).clone();
                let pc = refit_pcurve(&b.face(face).surface, &ed.curve, ed.t0, ed.t1, tol);
                if dbg {
                    let dev2 = pc.as_ref().map_or(f64::INFINITY, |pc| {
                        (0..=8).map(|k| {
                            let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0;
                            let uv = pc.eval(t);
                            b.face(face).surface.eval(uv.x, uv.y).dist(ed.curve.eval(t))
                        })
                        .fold(0.0, f64::max)
                    });
                    let onsurf = (0..=4).map(|k| {
                        let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / 4.0;
                        let p = ed.curve.eval(t);
                        let (u, v) = b.face(face).surface.closest_param(p);
                        b.face(face).surface.eval(u, v).dist(p)
                    })
                    .fold(0.0, f64::max);
                    let fs: Vec<String> = b.edge_faces(b.coedge(c).edge).into_iter().map(|f| b.face_id_str(f)).collect();
                    let ctr = match ed.curve {
                        Curve::Circle { frame, r } => format!("c=({:.3},{:.3},{:.3}) r={r}", frame.o.x, frame.o.y, frame.o.z),
                        _ => "-".into(),
                    };
                    eprintln!(
                        "pcurve: co={} face={} faces={:?} surf={} curve={} {ctr} t=({:.4},{:.4}) dev {:.6} -> {:.6} onsurf {:.6} rev={}",
                        c.idx(),
                        b.face_id_str(face),
                        fs,
                        b.face(face).surface.kind(),
                        ed.curve.kind(),
                        ed.t0,
                        ed.t1,
                        dev,
                        dev2,
                        onsurf,
                        co.rev
                    );
                }
                fixes.push((c, pc));
            }
            ck_topo::check::Issue::BadPairing(e) => flips.push(e),
            _ => {}
        }
    }
    for (c, pc) in fixes {
        b.coedges[c.idx()].pcurve = pc;
    }
    for e in flips {
        let cs = b.edge_coedges(e);
        if cs.len() == 2 && b.coedge(cs[0]).rev == b.coedge(cs[1]).rev {
            b.coedges[cs[1].idx()].rev = !b.coedge(cs[1]).rev;
        }
    }
}

/// Replace the striped body's zone box about the corner with `K` (module
/// doc). `Err` leaves the caller with the striped body unchanged — a valid,
/// nearly right solid, never a refusal.
pub(super) fn splice(body: &Body, c: &Corner, feature: &str, tag: usize, tol: f64) -> Result<Body, BlendError> {
    let fail = |d: &str| BlendError::detail("blendUnsupported", &format!("overhang corner: {d}"));
    let r = c.r;
    let snap = 1e-4 * (1.0 + r);
    let close = |a: Vec3, b: Vec3| a.dist(b) <= 1e-6 * (1.0 + a.len() + r);
    let (p0, p1) = (c.pairs[0].0, c.pairs[0].1);
    let (q0, q1) = (c.pairs[1].0, c.pairs[1].1);
    let ends = |i: usize| c.ends(i);
    // cross coincidences: one per strike of each pair, pairing the pairs off
    let mut hits: Vec<(usize, usize, usize, usize)> = vec![];
    for &i in &[p0, p1] {
        for k in 0..2 {
            for &j in &[q0, q1] {
                for kj in 0..2 {
                    if close(ends(i)[k], ends(j)[kj]) {
                        hits.push((i, k, j, kj));
                    }
                }
            }
        }
    }
    if hits.len() != 2 || hits[0].0 == hits[1].0 || hits[0].2 == hits[1].2 {
        return Err(fail("hexagon does not close"));
    }
    let m0 = if hits[0].0 == p0 { hits[0] } else { hits[1] };
    let m1 = if hits[0].0 == p0 { hits[1] } else { hits[0] };
    let (pa, ka, qb, kb) = m0;
    let (pc, kc, qd, kd) = m1;
    // hexagon: W1 = pa∩qb, A_line, C_line, W2 = pc∩qd, D_line, B_line
    let hex = [
        ends(pa)[ka],     // W1 (shared with qb)
        ends(pa)[1 - ka], // A_line
        ends(pc)[1 - kc], // C_line
        ends(pc)[kc],     // W2 (shared with qd)
        ends(qd)[1 - kd], // D_line
        ends(qb)[1 - kb], // B_line
    ];
    let arc_a = c.arc_centre(pa);
    let arc_c = c.arc_centre(pc);
    let arc_d = c.arc_centre(qd);
    let arc_b = c.arc_centre(qb);
    // box axes from the pair directions
    let a1 = c.s[p0].dir;
    let a2 = c.s[q0].dir;
    let a3 = a1.cross(a2).normalize();
    // the plane (through the box) containing a line side
    let line_plane = |mid: Vec3| -> Vec3 {
        for ax in [a1, a2, a3] {
            let d = (mid - c.v).dot(ax);
            if (d.abs() - r).abs() < 1e-6 * (1.0 + r) {
                return ax * d.signum();
            }
        }
        a3
    };
    let plane_l1 = line_plane((hex[1] + hex[2]) * 0.5);
    let plane_l2 = line_plane((hex[4] + hex[5]) * 0.5);
    // the wall planes = the pairs' shared faces (the seams' G1 neighbours)
    let wall_n = |pr: (usize, usize)| -> Vec3 {
        for ki in 0..2 {
            for kj in 0..2 {
                if c.s[pr.0].n[ki].dot(c.s[pr.1].n[kj]) > 1.0 - 1e-9 {
                    return c.s[pr.0].n[ki];
                }
            }
        }
        a3
    };
    let wn = (wall_n(c.pairs[0]), wall_n(c.pairs[1]));
    // cut the zone out; its six faces are the sections K must match
    let zbox = zone_box(c.v, r, [a1, a2, a3]);
    let cut = crate::boolean::boolean(body, &zbox, Op::Cut, tol).map(|o| o.body).map_err(|_| fail("zone cut"))?;
    // (box face outward, hex side endpoints in cycle order W1.., arc)
    let sides: [(Vec3, [usize; 2], Option<(Vec3, f64)>); 6] = [
        (c.s[pa].dir, [1, 0], Some((arc_a, r))),
        (plane_l1, [2, 1], None),
        (c.s[pc].dir, [3, 2], Some((arc_c, r))),
        (c.s[qd].dir, [4, 3], Some((arc_d, r))),
        (plane_l2, [5, 4], None),
        (c.s[qb].dir, [0, 5], Some((arc_b, r))),
    ];
    let mut tb = ToolBuilder::new();
    let hi: [usize; 6] = std::array::from_fn(|t| tb.vertex_shared(hex[t], snap));
    // the six section faces of K, oriented about their box outward normals
    let mut sect: Vec<Vec<Seg>> = vec![];
    for (k, &(out, _, _)) in sides.iter().enumerate() {
        let raw = section(&cut, c.v + out * r, out, r)?;
        let segs: Vec<Seg> = raw
            .into_iter()
            .map(|(a, b, arc)| Seg { a: tb.vertex_shared(a, snap), b: tb.vertex_shared(b, snap), arc })
            .collect();
        tb.plane_face(out, segs.clone(), FaceName::other(feature, &format!("Z{tag}{k}")));
        sect.push(segs);
    }
    // the patch loop: each hex side traversed opposite to its section face
    let mut loose: Vec<Seg> = vec![];
    for (k, &(_, idx, arc)) in sides.iter().enumerate() {
        let (ia, ib) = (hi[idx[0]], hi[idx[1]]);
        let has = sect[k]
            .iter()
            .find(|s| (s.a == ia && s.b == ib) || (s.a == ib && s.b == ia))
            .ok_or_else(|| fail("hex side not in its section"))?;
        // section runs ia->ib or ib->ia; the patch takes the opposite direction
        let (fa, fb) = if (has.a, has.b) == (ia, ib) { (ib, ia) } else { (ia, ib) };
        loose.push(Seg { a: fa, b: fb, arc });
    }
    // chain the six into one loop (they form the reverse of the cycle)
    let mut chain: Vec<Seg> = vec![];
    let mut cur = loose.remove(0);
    chain.push(cur);
    while !loose.is_empty() {
        let pos = chain.last().unwrap().b;
        let i = loose.iter().position(|s| s.a == pos).ok_or_else(|| fail("patch loop"))?;
        cur = loose.remove(i);
        chain.push(cur);
    }
    if chain.last().unwrap().b != chain[0].a {
        return Err(fail("patch loop"));
    }
    let pos_of = |ix: usize| hi.iter().position(|&h| h == ix).map(|t| hex[t]).ok_or_else(|| fail("hex vertex"));
    let mut outward = Vec3::ZERO;
    for t in 0..chain.len() {
        let (p, q) = (pos_of(chain[t].a)?, pos_of(chain[t].b)?);
        outward = outward + Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
    }
    if outward.len() < 1e-12 {
        return Err(fail("degenerate hexagon"));
    }
    outward = outward.normalize();
    let bottom = [(hex[1], hex[0], arc_a, c.centre(pa)), (hex[0], hex[5], arc_b, c.centre(qb))];
    let top = [(hex[2], hex[3], arc_c, c.centre(pc)), (hex[3], hex[4], arc_d, c.centre(qd))];
    let surf = patch_surface(&hex, bottom, [c.s[pa].dir, c.s[qb].dir], top, [c.s[pc].dir, c.s[qd].dir], wn, 49)?;
    tb.bspline_face(surf, chain, outward, FaceName::other(feature, &format!("P{tag}")));
    let k = tb.finish();
    let ki = ck_topo::check::check(&k, tol);
    if !ki.is_empty() {
        return Err(fail(&format!("K invalid: {ki:?}")));
    }
    // The union runs through the public imprint/select/build pipeline (nothing in the boolean
    // walk or assemble is edited): P0 touches the four stripes and the two wall planes
    // tangentially — their whole intersection is the shared boundary edges, which the marcher
    // cannot cross (blend/freeform's tangent-contact rule) — and it grazes the zone section
    // planes along the boundary arcs, where a marched curve would run beside the exact circle
    // instead of on it; both pairs are declared disjoint and the shared edges weld directly.
    let pname = FaceName::other(feature, &format!("P{tag}"));
    let patch_f = k.face_ids().find(|&f| k.face(f).name == pname).ok_or_else(|| fail("patch face"))?;
    let line_sides = [(hex[1], hex[2]), (hex[4], hex[5])];
    let arc_sides: [[usize; 2]; 4] = [[1, 0], [3, 2], [4, 3], [0, 5]];
    let hook = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> {
        let other = if g == patch_f {
            Some(f)
        } else if f == patch_f {
            Some(g)
        } else {
            None
        }?;
        let nf = &cut.face(other).name;
        let hit = if nf.feature == feature && matches!(nf.role, Role::Blend(_)) {
            true
        } else if let Surface::Plane { frame } = &cut.face(other).surface {
            let on = |p: Vec3| ((p - frame.o).dot(frame.z)).abs() <= 1e-6;
            let wall = ((c.v - frame.o).dot(frame.z)).abs() <= 1e-6 && line_sides.iter().any(|&(a, b)| on(a) && on(b));
            let section = [a1, a2, a3].iter().any(|&ax| frame.z.cross(ax).len() <= 1e-9 && (((c.v - frame.o).dot(ax)).abs() - r).abs() <= 1e-6 * (1.0 + r))
                && arc_sides.iter().any(|&idx| on(hex[idx[0]]) && on(hex[idx[1]]));
            wall || section
        } else {
            false
        };
        if hit {
            if std::env::var_os("CK_BLEND_DEBUG").is_some() {
                eprintln!("hook: P0 <-> {}", cut.face_id_str(other));
            }
            Some(Vec::new())
        } else {
            None
        }
    };
    let mut out: Option<Body> = None;
    let dbg = |m: &str| {
        if std::env::var_os("CK_BLEND_DEBUG").is_some() {
            eprintln!("overhang union: {m}");
        }
    };
    for &w in &[1.0f64, 10.0, 100.0] {
        let t = tol * w;
        let arr = crate::boolean::imprint::imprint_hooked(&cut, &k, t, Some(&hook));
        let class_tol = t.min(1e-5);
        let faces_out = match crate::boolean::assemble::select_class_tol(&arr, [&cut, &k], Op::Union, class_tol) {
            Ok(f) => f,
            Err(e) => {
                dbg(&format!("select@{w}: {e}"));
                continue;
            }
        };
        if !faces_out.iter().any(|f| {
            let mut n = f.name.clone();
            n.region = None;
            n == pname
        }) {
            dbg("patch face lost");
            continue;
        }
        let mut body = crate::boolean::assemble::build(&arr, &faces_out);
        super::freeform::drop_zero_edges(&mut body, 1e-4);
        weld_seams(&mut body, t);
        refit_pcurves(&mut body, t);
        let bad = ck_topo::check::check(&body, t);
        if !bad.is_empty() {
            dbg(&format!("check@{w}: {bad:?}"));
            continue;
        }
        if let Some(m) = crate::boolean::sanity::check_result(&cut, &k, Op::Union, &body) {
            dbg(&format!("sanity@{w}: {} {}", m.what, m.got));
            continue;
        }
        out = Some(body);
        break;
    }
    let mut out = out.ok_or_else(|| fail("patch union"))?;
    // faces: the input's order, anything new last
    let order: Vec<FaceId> = {
        let mut o: Vec<FaceId> = out.face_ids().collect();
        o.sort_by_key(|f| body.face_ids().position(|g| body.face_id_str(g) == out.face_id_str(*f)).unwrap_or(usize::MAX));
        o
    };
    out = crate::boolean::assemble::rebuild_in_order(&out, &order);
    Ok(out)
}