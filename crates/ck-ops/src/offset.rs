//! Face offsets of a closed body (used by shell; draft/thicken have their own).
//!
//! Reconstruction (semantics from oracle probes; the original's 24 KB
//! `offset_body` is not transliterated): every face moves along its outward
//! normal by its own signed distance, vertices are re-solved as the point at
//! the right distance from all incident offset surfaces, edge curves follow
//! their vertices (lines keep their direction, circles stay coaxial).
//! Topology and element order are those of the source body.
//!
//! Failure strings are kernel-verbatim (kernel.strings): "edge between A and
//! B shrinks to nothing", "an edge reverses", "the offset passes the
//! surface's centre of curvature" (code `offsetTooLarge`), unsupported
//! surfaces / curves → `offsetUnsupported`.

use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, VertexId};

#[derive(Debug, Clone, PartialEq)]
pub struct OffsetError {
    pub code: &'static str,
    pub detail: Option<String>,
}

impl OffsetError {
    fn new(code: &'static str, detail: impl Into<String>) -> OffsetError {
        OffsetError { code, detail: Some(detail.into()) }
    }
    /// True for the "offset became degenerate" family (shell maps these to
    /// `shellTooThick`).
    pub fn is_degenerate(&self) -> bool {
        self.code == "offsetDegenerate"
    }
}

const CENTRE: &str = "the offset passes the surface's centre of curvature";
const EPS: f64 = 1e-9;

/// Offset `s` by `d` along its natural normal. `None`: collapses (`Err`
/// path of the caller: offsetTooLarge) — unsupported kinds return
/// `Err(offsetUnsupported)` via [`offset_surface`].
fn offset_surface(s: &Surface, d: f64) -> Result<Surface, OffsetError> {
    let big = || OffsetError::new("offsetTooLarge", CENTRE);
    match s {
        Surface::Plane { frame } => Ok(Surface::Plane { frame: Frame { o: frame.o + frame.z.scale(d), ..frame.clone() } }),
        Surface::Cylinder { frame, r } => {
            let r2 = r + d;
            if r2 > EPS { Ok(Surface::Cylinder { frame: frame.clone(), r: r2 }) } else { Err(big()) }
        }
        Surface::Cone { frame, r, semi_angle } => {
            let r2 = r + d / semi_angle.cos();
            if r2 > EPS { Ok(Surface::Cone { frame: frame.clone(), r: r2, semi_angle: *semi_angle }) } else { Err(big()) }
        }
        Surface::Sphere { frame, r } => {
            let r2 = r + d;
            if r2 > EPS { Ok(Surface::Sphere { frame: frame.clone(), r: r2 }) } else { Err(big()) }
        }
        Surface::Torus { frame, major, minor } => {
            let m2 = minor + d;
            if m2 > EPS { Ok(Surface::Torus { frame: frame.clone(), major: *major, minor: m2 }) } else { Err(big()) }
        }
        _ => Err(OffsetError::new("offsetUnsupported", "surface type")),
    }
}

/// Solve `m · x = b` (3×3, Gaussian elimination with partial pivoting).
fn solve3(mut m: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for c in 0..3 {
        let p = (c..3).max_by(|&i, &j| m[i][c].abs().partial_cmp(&m[j][c].abs()).unwrap())?;
        if m[p][c].abs() < 1e-300 {
            return None;
        }
        m.swap(c, p);
        b.swap(c, p);
        for r in c + 1..3 {
            let f = m[r][c] / m[c][c];
            for k in c..3 {
                m[r][k] -= f * m[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = [0.0; 3];
    for c in (0..3).rev() {
        let mut s = b[c];
        for k in c + 1..3 {
            s -= m[c][k] * x[k];
        }
        x[c] = s / m[c][c];
    }
    Some(x)
}

/// Point near `p0` at zero signed distance from every surface in `surfs`
/// (Newton on the stacked distance residuals; minimum-norm when the
/// constraints leave a free direction, e.g. a seam vertex).
fn solve_point(p0: Vec3, surfs: &[&Surface]) -> Option<Vec3> {
    let mut p = p0;
    for _ in 0..40 {
        let mut jtj = [[0.0f64; 3]; 3];
        let mut jtr = [0.0f64; 3];
        let mut worst = 0.0f64;
        for s in surfs {
            let (u, v, q) = s.closest_point(p);
            let n = s.normal(u, v).try_normalize()?;
            let r = (p - q).dot(n);
            worst = worst.max(r.abs());
            let nn = [n.x, n.y, n.z];
            for i in 0..3 {
                for j in 0..3 {
                    jtj[i][j] += nn[i] * nn[j];
                }
                jtr[i] += nn[i] * r;
            }
        }
        for i in 0..3 {
            jtj[i][i] += 1e-12;
        }
        let dp = solve3(jtj, [-jtr[0], -jtr[1], -jtr[2]])?;
        let d = Vec3::new(dp[0], dp[1], dp[2]);
        p = p + d;
        if worst < 1e-13 && d.len() < 1e-13 {
            break;
        }
        if d.len() < 1e-14 {
            break;
        }
    }
    p.is_finite().then_some(p)
}

/// Offset the faces of `src`: `dist(face)` is the signed distance along the
/// face's OUTWARD normal (positive = grow). Returns the moved copy (same
/// topology and element order, pcurves dropped).
pub fn offset_body(src: &Body, dist: &dyn Fn(FaceId) -> f64) -> Result<Body, OffsetError> {
    let mut out = src.clone();

    // new surfaces
    let mut moved: Vec<Option<Surface>> = vec![None; src.faces.len()];
    for f in src.face_ids() {
        let face = src.face(f);
        let d = dist(f);
        let nat = if face.sense { d } else { -d };
        moved[f.idx()] = Some(offset_surface(&face.surface, nat).map_err(|e| {
            if e.code == "offsetTooLarge" {
                OffsetError::new("offsetDegenerate", format!("{} collapses and cannot be removed", src.face_id_str(f)))
            } else {
                e
            }
        })?);
    }

    // incident faces per vertex
    let mut vfaces: Vec<Vec<FaceId>> = vec![vec![]; src.vertices.len()];
    for f in src.face_ids() {
        for &l in &src.face(f).loops {
            if !src.lp(l).alive {
                continue;
            }
            for &c in &src.lp(l).coedges {
                if !src.coedge(c).alive {
                    continue;
                }
                for v in [src.coedge_start(c), src.coedge_end(c)] {
                    if !vfaces[v.idx()].contains(&f) {
                        vfaces[v.idx()].push(f);
                    }
                }
            }
        }
    }
    let mut newp: Vec<Vec3> = src.vertices.iter().map(|v| v.p).collect();
    for v in src.vertex_ids() {
        let surfs: Vec<&Surface> = vfaces[v.idx()].iter().map(|f| moved[f.idx()].as_ref().unwrap()).collect();
        if surfs.is_empty() {
            continue;
        }
        newp[v.idx()] = solve_point(src.vertex(v).p, &surfs)
            .ok_or_else(|| OffsetError::new("offsetDegenerate", "the offset faces do not meet at a corner"))?;
    }

    // edges follow their vertices
    for e in src.edge_ids() {
        let ed = src.edge(e);
        let fs = src.edge_faces(e);
        let name = |i: usize| fs.get(i).map(|&f| src.face_id_str(f)).unwrap_or_default();
        let (p0, p1) = (newp[ed.v0.idx()], newp[ed.v1.idx()]);
        let shrink = || OffsetError::new("offsetDegenerate", format!("edge between {} and {} shrinks to nothing", name(0), name(1)));
        let reverses = || OffsetError::new("offsetDegenerate", "an edge reverses");
        let closed = ed.v0 == ed.v1;
        let (curve, t0, t1) = match &ed.curve {
            Curve::Line { d, .. } => {
                let len = (p1 - p0).dot(*d);
                if len.abs() <= EPS {
                    return Err(shrink());
                }
                if len < 0.0 {
                    return Err(reverses());
                }
                (Curve::Line { o: p0 - d.scale(ed.t0), d: *d }, ed.t0, ed.t0 + len)
            }
            Curve::Circle { frame, r } => {
                let z = frame.z;
                let c = frame.o + z.scale((p0 - frame.o).dot(z));
                let r2 = (p0 - c).len();
                if r2 <= EPS {
                    return Err(shrink());
                }
                // angular extent is kept: vertices stay on their radial lines
                let _ = r;
                (Curve::Circle { frame: Frame { o: c, ..frame.clone() }, r: r2 }, ed.t0, ed.t1)
            }
            _ => return Err(OffsetError::new("offsetUnsupported", "edge curve type")),
        };
        let _ = closed;
        let o = &mut out.edges[e.idx()];
        o.curve = curve;
        o.t0 = t0;
        o.t1 = t1;
    }
    for v in src.vertex_ids() {
        out.vertices[v.idx()].p = newp[v.idx()];
    }
    for f in src.face_ids() {
        out.faces[f.idx()].surface = moved[f.idx()].take().unwrap();
    }
    for c in out.coedges.iter_mut() {
        c.pcurve = None;
    }
    Ok(out)
}

#[allow(unused)]
fn _types(_: EdgeId, _: VertexId) {}
