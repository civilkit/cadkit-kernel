//! Validity gate for blend results.
//!
//! Every blend is built as a swept tool applied by boolean, so a failure in the tool or in the
//! boolean can come back as an apparently successful body: edges used by four faces, an
//! inside-out solid, or a result that lost (or gained) far more material than the blends can
//! explain. [`check`] judges a finished result against the body it was built from and turns such a
//! result into a `blendFailed` error.
//!
//! What is checked (all relative to the input body, so an input that already carries a defect is
//! not blamed on the blend):
//! - structure: no more edges used by a number of coedges other than two than before, and no new
//!   structural issue of `ck_check::validate` (open or non-manifold shell, open loop, orientation,
//!   a face without a loop, a non-positive solid volume, several outer shells in one solid);
//! - volume: positive when the input's was, and `|dV|` within the analytic bound of
//!   [`volume_bound`]; a selection of only convex edges must not add material, one of only concave
//!   edges must not remove any.
//!
//! The volume bound is deliberately loose (it exists to catch gross over-removal, not to compare
//! with a reference). It is given for circular fillets and distance chamfers (conic, chord and
//! curvature-continuous sections, angle-form chamfers: structure only). A rolling-ball fillet of
//! radius `r` along an edge whose faces turn by `a` changes the volume by `L r^2 (tan(a/2) - a/2) k`,
//! with `L` the edge length and `k` the sweep factor of a section on the far side of a bending edge
//! (1 for straight edges and for sections on the near side); a chamfer by `L d1 d2 / 2 k`. Each vertex
//! that is not a smooth join of the selection adds `d^3` for the corner and end patches. The bound is
//! applied with 2 % slack and `1e-4 V`.

use super::build::{ChamferLeg, FilletProfile, Spec};
use super::BlendError;
use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId};
use ck_topo::mass::{lump_faces, signed_volume_quick};

/// `ck_check` issue codes that make a blend result a failure when the result has more of them than the input.
const FATAL: &[&str] = &[
    "SHELL_EMPTY", "SHELL_OPEN", "SHELL_NONMANIFOLD", "SHELL_ORIENTATION", "LOOP_OPEN", "LOOP_EMPTY", "FACE_NO_LOOP", "EULER",
    "VOLUME_NONPOSITIVE", "SOLID_MULTIPLE_OUTERS",
];

/// `CK_BLEND_GATE=0` switches the gate off (to compare an engine's raw results; native runs only).
fn enabled() -> bool {
    std::env::var_os("CK_BLEND_GATE").map_or(true, |v| v != "0")
}

/// Total signed volume of all solids of `b`.
pub fn volume(b: &Body) -> f64 {
    b.lump_ids().map(|l| signed_volume_quick(b, &lump_faces(b, l))).sum()
}

fn open_edges(b: &Body) -> usize {
    b.edge_ids().filter(|&e| !b.edge_coedges(e).is_empty() && b.edge_coedges(e).len() != 2).count()
}

fn fatal_counts(b: &Body, tol: f64) -> Vec<(&'static str, usize)> {
    let issues = ck_check::validate_ctl(b, tol, ck_check::DEFAULT_TOL_CAP, true, &ck_check::Ctl::new(None));
    let mut out: Vec<(&'static str, usize)> = vec![];
    for i in issues.iter().filter(|i| FATAL.contains(&i.code)) {
        match out.iter_mut().find(|(c, _)| *c == i.code) {
            Some((_, n)) => *n += 1,
            None => out.push((i.code, 1)),
        }
    }
    out
}

fn fail(detail: String) -> BlendError {
    BlendError::detail("blendFailed", &detail)
}

/// Edge geometry the volume bound needs: turning angle of the faces, convexity, length, bend radius.
struct EdgeInfo {
    len: f64,
    alpha: f64,
    convex: bool,
    /// radius of the edge curve (infinite for a line, NaN for a curve that is not a circle)
    bend: f64,
    /// `bis . u` for the unit bisector `bis` of the face normals and the unit vector `u` from the circle's axis to the edge
    /// (0 when the edge is not a circle): positive when the blend section lies on the far side of the edge
    far: f64,
    verts: [ck_topo::body::VertexId; 2],
}

fn edge_info(b: &Body, e: EdgeId) -> Option<EdgeInfo> {
    let ed = b.edge(e);
    let fs = b.edge_faces(e);
    if fs.len() != 2 {
        return None;
    }
    let pm = b.edge_mid(e);
    let nrm = |f| {
        let sf = &b.face(f).surface;
        let (u, v) = sf.closest_param(pm);
        b.outward_normal(f, u, v).normalize()
    };
    let (n1, n2) = (nrm(fs[0]), nrm(fs[1]));
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == fs[0])?;
    let t = ed.curve.tangent(0.5 * (ed.t0 + ed.t1)).normalize();
    let tdir = if b.coedge(co).rev { -t } else { t };
    let alpha = n1.dot(n2).clamp(-1.0, 1.0).acos();
    let (bend, u) = match &ed.curve {
        Curve::Line { .. } => (f64::INFINITY, Vec3::ZERO),
        c => match c.circle_info() {
            Some((cen, axis, r)) => {
                let d = pm - cen;
                (r, (d - axis.normalize() * d.dot(axis.normalize())).try_normalize().unwrap_or(Vec3::ZERO))
            }
            None => (f64::NAN, Vec3::ZERO),
        },
    };
    let convex = n1.cross(tdir).dot(n2) < 0.0;
    // The removed (convex) region lies inside the material, the added (concave) one outside: against / along the bisector of
    // the outward normals. A region on the far side of a bending edge has more volume than the straight sweep.
    let bis = (n1 + n2).try_normalize().unwrap_or(Vec3::ZERO);
    let far = if convex { -bis.dot(u) } else { bis.dot(u) };
    Some(EdgeInfo { len: b.edge_length(e), alpha, convex, bend, far, verts: [ed.v0, ed.v1] })
}

/// The picked edges and everything a blend propagates to: edges that continue a picked one tangentially at a
/// shared vertex and edges between the same two faces (a ring split by a seam).
fn affected(b: &Body, picks: &[(String, String, Vec3)]) -> Vec<EdgeId> {
    let mut out: Vec<EdgeId> = picks.iter().filter_map(|(fa, fb, at)| super::build::find_edge(b, fa, fb, *at)).collect();
    let mut i = 0;
    while i < out.len() {
        let e = out[i];
        i += 1;
        let ed = b.edge(e);
        let fs = b.edge_faces(e);
        for v in [ed.v0, ed.v1] {
            let d0 = away_dir(b, e, v);
            for x in b.edge_ids() {
                if out.contains(&x) || b.edge_coedges(x).is_empty() {
                    continue;
                }
                let xe = b.edge(x);
                let touches = xe.v0 == v || xe.v1 == v;
                let same_faces = {
                    let xf = b.edge_faces(x);
                    xf.len() == 2 && fs.len() == 2 && xf.iter().all(|f| fs.contains(f))
                };
                if (touches && away_dir(b, x, v).dot(d0) < -0.995) || (touches && same_faces) {
                    out.push(x);
                }
            }
        }
    }
    out
}

/// Number of vertices of the selection `sel` that are not the smooth join of exactly two of its edges.
fn chain_ends(b: &Body, sel: &[EdgeId]) -> usize {
    let mut verts: Vec<ck_topo::body::VertexId> = vec![];
    for &e in sel {
        for v in [b.edge(e).v0, b.edge(e).v1] {
            if !verts.contains(&v) {
                verts.push(v);
            }
        }
    }
    verts
        .into_iter()
        .filter(|&v| {
            let inc: Vec<EdgeId> = sel.iter().copied().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
            !(inc.len() == 2 && away_dir(b, inc[0], v).dot(away_dir(b, inc[1], v)) < -0.995)
        })
        .count()
}

/// Unit tangent of `e` at its end `v`, pointing along the edge away from `v`.
fn away_dir(b: &Body, e: EdgeId, v: ck_topo::body::VertexId) -> Vec3 {
    let xe = b.edge(e);
    let at_start = xe.v0 == v;
    let d = xe.curve.tangent(if at_start { xe.t0 } else { xe.t1 }).normalize();
    if at_start {
        d
    } else {
        -d
    }
}

/// Upper bound of `|dV|` for blending `picks` of `b` with `spec`, and whether the selection is all convex /
/// all concave. `None`: no bound is known (angle-form chamfers, non-circular fillet sections, edges the bound cannot
/// read).
pub fn volume_bound(b: &Body, picks: &[(String, String, Vec3)], spec: Spec) -> Option<(f64, bool, bool)> {
    let r = spec.size();
    let (d1, d2) = match spec {
        Spec::Fillet(_, FilletProfile::Circular) => (r, r),
        // conic, chord and curvature-continuous sections remove more or less than the circular one: only the
        // structural checks apply to them
        Spec::Fillet(..) => return None,
        Spec::Chamfer(a, ChamferLeg::Dist(c)) => (a, c),
        Spec::Chamfer(..) => return None,
    };
    let mut total = 0.0;
    let (mut all_convex, mut all_concave) = (true, true);
    let sel = affected(b, picks);
    for e in sel.iter().copied() {
        let info = edge_info(b, e)?;
        // tangent edges (alpha ~ 0) still remove a sliver; read them as a right angle
        let alpha = info.alpha.max(std::f64::consts::FRAC_PI_2).min(std::f64::consts::PI - 0.05);
        let area = match spec {
            Spec::Fillet(..) => r * r * ((alpha * 0.5).tan() - alpha * 0.5),
            _ => 0.5 * d1 * d2,
        };
        // the section's centroid lies within half the blend size of the edge
        let k = if info.bend.is_infinite() { 1.0 } else if info.bend.is_finite() { 1.0 + 0.5 * d1.max(d2) * info.far.max(0.0) / info.bend } else { 3.0 };
        total += info.len * area * k;
        if info.alpha > 1e-3 {
            all_convex &= info.convex;
            all_concave &= !info.convex;
        } else {
            all_convex = false;
            all_concave = false;
        }
    }
    // corners and chain ends: a vertex of the selection that is not an internal point of a tangent chain
    let m = d1.max(d2);
    total += chain_ends(b, &sel) as f64 * m * m * m;
    Some((total, all_convex, all_concave))
}

/// Structure and sign of the volume, see the module notes. Returns `(input volume, result volume)`.
fn check_solid(input: &Body, out: &Body, tol: f64) -> Result<(f64, f64), BlendError> {
    let (bad_in, bad_out) = (open_edges(input), open_edges(out));
    if bad_out > bad_in {
        let e = out.edge_ids().find(|&e| !out.edge_coedges(e).is_empty() && out.edge_coedges(e).len() != 2).unwrap();
        return Err(fail(format!(
            "the result is not a closed solid: {} edge(s) are not shared by exactly two faces (edge #{} is used by {})",
            bad_out - bad_in,
            e.idx(),
            out.edge_coedges(e).len()
        )));
    }
    let fo = fatal_counts(out, tol);
    if !fo.is_empty() {
        let fi = fatal_counts(input, tol);
        for (code, n) in &fo {
            let before = fi.iter().find(|(c, _)| c == code).map_or(0, |x| x.1);
            if *n > before {
                return Err(fail(format!("the result is not a valid solid ({code})")));
            }
        }
    }
    let (vi, vo) = (volume(input), volume(out));
    if vi > 0.0 && !(vo > 0.0) {
        return Err(fail(format!("the result has no positive volume ({vo:.6e})")));
    }
    Ok((vi, vo))
}

/// Judge `out`, the result of blending `picks` of `input`, see the module notes.
pub fn check(input: &Body, out: Body, picks: &[(String, String, Vec3)], spec: Spec, tol: f64) -> Result<Body, BlendError> {
    if !enabled() {
        return Ok(out);
    }
    let (vi, vo) = check_solid(input, &out, tol)?;
    if vi > 0.0 {
        let slack = 1e-4 * vi;
        let dv = vo - vi;
        if let Some((bound, convex, concave)) = volume_bound(input, picks, spec) {
            if dv.abs() > bound * 1.02 + slack {
                return Err(fail(format!("the volume changed by {dv:.6} but the blend can change it by at most {bound:.6}")));
            }
            if convex && dv > slack {
                return Err(fail(format!("convex edges were blended but the volume grew by {dv:.6}")));
            }
            if concave && dv < -slack {
                return Err(fail(format!("concave edges were blended but the volume shrank by {:.6}", -dv)));
            }
        }
    }
    Ok(out)
}

/// Judge the result of cutting the corner of `input` at a vertex with the three distances `dist`: a tetrahedron of
/// volume `d1 d2 d3 sin(...) / 6` is removed, at most `d1 d2 d3 / 6` for a right-angled corner; the bound uses
/// three times that to cover skew corners.
pub fn check_vertex_cut(input: &Body, out: Body, dist: [f64; 3], tol: f64) -> Result<Body, BlendError> {
    if !enabled() {
        return Ok(out);
    }
    let (vi, vo) = check_solid(input, &out, tol)?;
    if vi > 0.0 {
        let slack = 1e-4 * vi;
        let dv = vo - vi;
        let bound = 3.0 * dist[0] * dist[1] * dist[2] / 6.0;
        if dv > slack || -dv > bound * 1.02 + slack {
            return Err(fail(format!("the volume changed by {dv:.6} but a corner cut can remove at most {bound:.6}")));
        }
    }
    Ok(out)
}
