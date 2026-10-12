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

/// Offset surface of one face by `d` along its natural normal (exact
/// analytic kinds, fitted free-form ones; `reach` = extension past the face).
pub fn offset_face_surface(src: &Body, f: FaceId, d: f64, reach: f64) -> Result<Surface, OffsetError> {
    let face = src.face(f);
    if is_freeform(&face.surface) {
        let bx = face_uv_box(src, f).ok_or_else(|| OffsetError::new("offsetUnsupported", "face extent"))?;
        fit_offset_surface(&face.surface, d, bx, reach)
    } else {
        offset_surface(&face.surface, d)
    }
}

/// Fit tolerance of sampled offset surfaces / edge curves (mm).
const FIT_TOL: f64 = 1e-5;
/// Fitted offset edge curves must stay this close (mm) to both offset
/// surfaces: the base solid's own edges are only that accurate against
/// their faces (trimmed intersection curves), and the offset inherits it.
const EDGE_FIT_TOL: f64 = 3e-3;
/// As [`EDGE_FIT_TOL`] for edges between grazing (G1) neighbours.
const GRAZE_TOL: f64 = 1e-1;
/// Sample spans shorter than this (mm) are not refined further.
const MIN_SPAN: f64 = 0.04;

fn is_freeform(s: &Surface) -> bool {
    matches!(s, Surface::BSpline(_) | Surface::Revolution { .. } | Surface::Extrusion { .. } | Surface::Offset { .. })
}

/// (u, v) bounding box of the face's loops on its surface (periodic
/// parameters unwrapped along the loop), grown by a margin and clamped to a
/// finite domain.
fn face_uv_box(src: &Body, f: FaceId) -> Option<((f64, f64), (f64, f64))> {
    let s = &src.face(f).surface;
    let (mut u0, mut u1, mut v0, mut v1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    let mut prev: Option<ck_geom::Vec2> = None;
    for &l in &src.face(f).loops {
        if !src.lp(l).alive {
            continue;
        }
        for &c in &src.lp(l).coedges {
            if !src.coedge(c).alive {
                continue;
            }
            let (ta, tb) = src.coedge_params(c);
            let ed = src.edge(src.coedge(c).edge);
            for k in 0..=32 {
                let t = ta + (tb - ta) * k as f64 / 32.0;
                let uv = crate::boolean::classify::uv_near(s, ed.curve.eval(t), prev);
                prev = Some(uv);
                u0 = u0.min(uv.x);
                u1 = u1.max(uv.x);
                v0 = v0.min(uv.y);
                v1 = v1.max(uv.y);
            }
        }
    }
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()) {
        return None;
    }
    let (mu, mv) = (0.03 * (u1 - u0) + 1e-9, 0.03 * (v1 - v0) + 1e-9);
    let (a, b, c, d) = (u0 - mu, u1 + mu, v0 - mv, v1 + mv);
    Some(((a, b), (c, d)))
}

/// Tensor-product cubic interpolation of a rectangular point grid, returning
/// the surface and the (averaged chord-length) parameters of the grid lines.
fn fit_grid(grid: &[Vec<Vec3>]) -> Result<(ck_geom::nurbs::bsurf::BSplineSurface, Vec<f64>, Vec<f64>), OffsetError> {
    use ck_geom::fit::{averaged_knots, interpolate_with, params, Param};
    let bad = |e: String| OffsetError::new("offsetUnsupported", format!("fit: {e}"));
    let (nu, nv) = (grid.len(), grid[0].len());
    if nu < 4 || nv < 4 {
        return Err(bad("grid too small".into()));
    }
    let avg = |lines: Vec<Vec<Vec3>>| -> Vec<f64> {
        let k = lines[0].len();
        let mut acc = vec![0.0; k];
        for l in &lines {
            for (a, b) in acc.iter_mut().zip(params(l, Param::ChordLength)) {
                *a += b;
            }
        }
        acc.iter().map(|a| a / lines.len() as f64).collect()
    };
    let uk = avg((0..nv).map(|j| (0..nu).map(|i| grid[i][j]).collect()).collect());
    let vl = avg(grid.to_vec());
    let ku = averaged_knots(&uk, 3);
    let kv = averaged_knots(&vl, 3);
    let mut r = vec![vec![Vec3::ZERO; nv]; nu];
    for j in 0..nv {
        let col: Vec<Vec3> = (0..nu).map(|i| grid[i][j]).collect();
        let c = interpolate_with(&col, 3, &uk, ku.clone()).map_err(bad)?;
        for i in 0..nu {
            r[i][j] = c.ctrl[i];
        }
    }
    let mut ctrl = Vec::with_capacity(nu * nv);
    for i in 0..nu {
        ctrl.extend(interpolate_with(&r[i], 3, &vl, kv.clone()).map_err(bad)?.ctrl);
    }
    let s = ck_geom::nurbs::bsurf::BSplineSurface::new(3, 3, ku, kv, nu, nv, ctrl, None).map_err(bad)?;
    Ok((s, uk, vl))
}

/// Offset of a free-form surface by `d` along its NATURAL normal, as a
/// cubic B-spline interpolating a sampled grid over `uvbox`. The fit is
/// verified against the exact offset at cell centres (`FIT_TOL`); a
/// vanishing area element (the offset passes a centre of curvature) or an
/// offset sample closer than `|d|` to the base surface (the offset folds
/// over another sheet) is `offsetTooLarge`.
fn fit_offset_surface(s: &Surface, d: f64, uvbox: ((f64, f64), (f64, f64)), reach: f64) -> Result<Surface, OffsetError> {
    let big = || OffsetError::new("offsetTooLarge", CENTRE);
    if let Surface::Revolution { profile, axis_o, axis } = s {
        return offset_revolution(s, profile, *axis_o, *axis, d, uvbox.1, reach);
    }
    let ((mut u0, mut u1), (mut v0, mut v1)) = uvbox;
    if !(u1 > u0 && v1 > v0) {
        return Err(OffsetError::new("offsetUnsupported", "empty parameter range"));
    }
    let off = Surface::Offset { base: Box::new(s.clone()), dist: d };
    let ((du0, du1), (dv0, dv1)) = s.domain();
    // The cavity of a shell runs `reach` past an opened face, so its
    // neighbours' offsets must extend beyond the original surface: pad the box
    // by that distance (in parameters) and continue the surface past its
    // domain with its second-order Taylor polynomial.
    if reach > 0.0 {
        let (mid_u, mid_v) = ((0.5 * (u0 + u1)).clamp(du0, du1), (0.5 * (v0 + v1)).clamp(dv0, dv1));
        let sd = s.derivs2(mid_u, mid_v);
        let (lu, lv) = (sd.du.len().max(1e-9), sd.dv.len().max(1e-9));
        let (pu, pv) = (reach / lu, reach / lv);
        // only directions bounded by the domain need padding (periodic ones wrap)
        if du0.is_finite() && du1.is_finite() && s.periods().0.is_none() {
            u0 -= pu;
            u1 += pu;
        }
        if dv0.is_finite() && dv1.is_finite() && s.periods().1.is_none() {
            v0 -= pv;
            v1 += pv;
        }
    }
    let point = |u: f64, v: f64| -> Result<Vec3, OffsetError> {
        let (uc, vc) = (u.clamp(du0, du1), v.clamp(dv0, dv1));
        let (ea, eb) = (u - uc, v - vc);
        let inside = ea == 0.0 && eb == 0.0;
        let sd = s.derivs2(uc, vc);
        let (su, sv) = (sd.du + sd.duu * ea + sd.duv * eb, sd.dv + sd.duv * ea + sd.dvv * eb);
        let n = su.cross(sv).try_normalize().ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate surface point"))?;
        let base_p = sd.p + sd.du * ea + sd.dv * eb + (sd.duu * (ea * ea) + sd.duv * (2.0 * ea * eb) + sd.dvv * (eb * eb)) * 0.5;
        if !inside {
            return Ok(base_p + n * d);
        }
        // regular offset: I - d W must stay positive definite (both principal
        // factors 1 - d kappa > 0); G - d II is the symmetric matrix of
        // (od_u, od_v) against (S_u, S_v)
        let od = off.derivs2(uc, vc);
        let (g00, g01, g11) = (sd.du.dot(sd.du), sd.du.dot(sd.dv), sd.dv.dot(sd.dv));
        let (j00, j01, j11) = (od.du.dot(sd.du), 0.5 * (od.du.dot(sd.dv) + od.dv.dot(sd.du)), od.dv.dot(sd.dv));
        let (det_g, det_j) = (g00 * g11 - g01 * g01, j00 * j11 - j01 * j01);
        if det_g > 1e-24 && (j00 < 0.05 * g00 || j11 < 0.05 * g11 || det_j < 0.0025 * det_g) {
            return Err(big());
        }
        Ok(base_p + n * d)
    };
    // Sample sets per direction: the base's own knots (a spline offset keeps
    // its kinks there) or a uniform start, refined where the fit misses.
    let seed = |lo: f64, hi: f64, knots: Option<&[f64]>| -> Vec<f64> {
        let mut x: Vec<f64> = vec![lo, hi];
        match knots {
            Some(k) => {
                for &t in k {
                    if t > lo + 1e-9 * (hi - lo) && t < hi - 1e-9 * (hi - lo) {
                        x.push(t);
                    }
                }
                x.sort_by(f64::total_cmp);
                x.dedup_by(|a, b| (*a - *b).abs() <= 1e-9 * (hi - lo));
                // pad the extension (past the domain) with an interior sample
                x
            }
            None => (0..=8).map(|i| lo + (hi - lo) * i as f64 / 8.0).collect(),
        }
    };
    let (ku, kv) = match s {
        Surface::BSpline(bs) => (Some(bs.knots_u.clone()), Some(bs.knots_v.clone())),
        _ => (None, None),
    };
    let mut us = seed(u0, u1, ku.as_deref());
    let mut vs = seed(v0, v1, kv.as_deref());
    let mut last_dev = f64::INFINITY;
    for _round in 0..14 {
        let mut grid = vec![vec![Vec3::ZERO; vs.len()]; us.len()];
        for (i, &u) in us.iter().enumerate() {
            for (j, &v) in vs.iter().enumerate() {
                grid[i][j] = point(u, v)?;
            }
        }
        let (fit, uk, vk) = fit_grid(&grid)?;
        let fs = Surface::BSpline(Box::new(fit));
        // deviation of the exact offset from the fit at span midpoints, along a
        // spread of lines of the other direction
        let mut dev = 0.0f64;
        let (mut bad_u, mut bad_v) = (vec![], vec![]);
        let pick = |n: usize| -> Vec<usize> {
            let k = 6.min(n);
            (0..k).map(|t| t * (n - 1) / (k - 1).max(1)).collect()
        };
        for i in 0..us.len() - 1 {
            let um = 0.5 * (us[i] + us[i + 1]);
            let mut worst = 0.0f64;
            for &j in &pick(vs.len()) {
                let e = point(um, vs[j])?;
                let hint = (0.5 * (uk[i] + uk[i + 1]), vk[j]);
                worst = worst.max(fs.closest_point_near(e, hint).2.dist(e));
            }
            // spans shorter than MIN_SPAN (a normal kink of the base smeared over
            // one sample) cannot be resolved further and are tolerated
            let span = grid[i][vs.len() / 2].dist(grid[i + 1][vs.len() / 2]);
            if span < MIN_SPAN {
                continue;
            }
            dev = dev.max(worst);
            if worst > FIT_TOL {
                bad_u.push((i, worst));
            }
        }
        for j in 0..vs.len() - 1 {
            let vm = 0.5 * (vs[j] + vs[j + 1]);
            let mut worst = 0.0f64;
            for &i in &pick(us.len()) {
                let e = point(us[i], vm)?;
                let hint = (uk[i], 0.5 * (vk[j] + vk[j + 1]));
                worst = worst.max(fs.closest_point_near(e, hint).2.dist(e));
            }
            let span = grid[us.len() / 2][j].dist(grid[us.len() / 2][j + 1]);
            if span < MIN_SPAN {
                continue;
            }
            dev = dev.max(worst);
            if worst > FIT_TOL {
                bad_v.push((j, worst));
            }
        }
        last_dev = dev;
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            let wu = bad_u.iter().cloned().fold((0usize, 0.0f64), |a, b| if b.1 > a.1 { b } else { a });
            let wv = bad_v.iter().cloned().fold((0usize, 0.0f64), |a, b| if b.1 > a.1 { b } else { a });
            eprintln!(
                "offset fit: {} d={d} grid {}x{} dev={dev:.2e} worst u-span {} ({:.4},{:.4}) {:.1e}, v-span {} ({:.4},{:.4}) {:.1e}; domain {:?} box ({u0:.3},{u1:.3}) ({v0:.3},{v1:.3})",
                s.kind(), us.len(), vs.len(), wu.0, us[wu.0], us[(wu.0 + 1).min(us.len() - 1)], wu.1, wv.0, vs[wv.0], vs[(wv.0 + 1).min(vs.len() - 1)], wv.1, s.domain()
            );
        }
        if bad_u.is_empty() && bad_v.is_empty() {
            // folds: the offset must stay |d| from the base everywhere
            let m = 12;
            for i in 0..=m {
                for j in 0..=m {
                    let (u, v) = (u0 + (u1 - u0) * i as f64 / m as f64, v0 + (v1 - v0) * j as f64 / m as f64);
                    let q = point(u, v)?;
                    let interior = u >= du0 && u <= du1 && v >= dv0 && v <= dv1;
                    if interior && s.closest_point(q).2.dist(q) < d.abs() * (1.0 - 1e-3) - 1e-6 {
                        return Err(big());
                    }
                }
            }
            return Ok(fs);
        }
        if us.len() + bad_u.len() > 500 || vs.len() + bad_v.len() > 500 {
            break;
        }
        for &(i, _) in bad_u.iter().rev() {
            us.insert(i + 1, 0.5 * (us[i] + us[i + 1]));
        }
        for &(j, _) in bad_v.iter().rev() {
            vs.insert(j + 1, 0.5 * (vs[j] + vs[j + 1]));
        }
    }
    Err(OffsetError::new("offsetUnsupported", format!("surface offset fit tolerance ({last_dev:.1e})")))
}

/// Offset of a surface of revolution: the revolution of the offset profile.
/// The profile (in its meridian plane) moves along the in-plane surface
/// normal; the offset profile is a cubic B-spline interpolating samples over
/// `vbox` (padded by `reach` past the profile's ends, continued linearly), so
/// the result keeps the exact circular sweep. The offset must keep the
/// profile regular (`1 - d kappa > 0`) and the radius positive.
fn offset_revolution(s: &Surface, profile: &Curve, axis_o: Vec3, axis: Vec3, d: f64, vbox: (f64, f64), reach: f64) -> Result<Surface, OffsetError> {
    let big = || OffsetError::new("offsetTooLarge", CENTRE);
    let a = axis.try_normalize().ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate axis"))?;
    let (p0, p1) = profile.range();
    if !(p0.is_finite() && p1.is_finite()) {
        return Err(OffsetError::new("offsetUnsupported", "unbounded profile"));
    }
    let (mut v0, mut v1) = vbox;
    v0 = v0.min(p0);
    v1 = v1.max(p1);
    // pad past the profile ends by `reach` (profile speed converts to parameters)
    let speed = |v: f64| profile.deriv(v.clamp(p0, p1)).len().max(1e-9);
    let (pad0, pad1) = (reach / speed(p0), reach / speed(p1));
    let (e0, e1) = (p0 - pad0, p1 + pad1);
    v0 = v0.min(e0);
    v1 = v1.max(e1);
    // in-plane unit normal of the surface along the profile (u = 0)
    let normal_at = |vc: f64| -> Result<Vec3, OffsetError> {
        let sd = s.derivs2(0.0, vc);
        sd.du.cross(sd.dv).try_normalize().ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate surface point"))
    };
    let sample = |v: f64| -> Result<Vec3, OffsetError> {
        let vc = v.clamp(p0, p1);
        let n = normal_at(vc)?;
        let t = profile.deriv(vc);
        Ok(profile.eval(vc) + t * (v - vc) + n * d)
    };
    let check = |v: f64| -> Result<(), OffsetError> {
        let vc = v.clamp(p0, p1);
        let n = normal_at(vc)?;
        let dv = profile.derivs(vc, 2);
        let (pt, tan) = (dv[0], dv[1]);
        // radius of the offset point
        let rho = |q: Vec3| {
            let w = q - axis_o;
            (w - a * w.dot(a)).len()
        };
        let r0 = rho(pt);
        let r1 = rho(pt + n * d);
        if r0 > 1e-9 && r1 < 0.05 * r0 {
            return Err(big());
        }
        // profile regularity: the offset tangent must still run along the profile
        let dd = n.dot(dv[2]) / tan.dot(tan).max(1e-300);
        if tan.len() > 1e-12 && (1.0 - d * dd) < 0.05 {
            return Err(big());
        }
        Ok(())
    };
    let mut n = 24usize;
    loop {
        let vs: Vec<f64> = (0..=n).map(|k| v0 + (v1 - v0) * k as f64 / n as f64).collect();
        let mut pts = vec![];
        for &v in &vs {
            check(v)?;
            pts.push(sample(v)?);
        }
        let c = ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength).map_err(|e| OffsetError::new("offsetUnsupported", format!("profile fit: {e}")))?;
        // exact offset at the cell centres vs the fit
        let mut dev = 0.0f64;
        for k in 0..n {
            let q = sample(0.5 * (vs[k] + vs[k + 1]))?;
            let t = c.closest_param(q);
            dev = dev.max(c.eval(t).dist(q));
        }
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            eprintln!("offset revolution profile fit: d={d} n={n} dev={dev:.2e}");
        }
        if dev <= FIT_TOL {
            return Ok(Surface::Revolution { profile: Box::new(Curve::BSpline(c)), axis_o, axis });
        }
        n *= 2;
        if n > 800 {
            return Err(OffsetError::new("offsetUnsupported", format!("profile offset fit tolerance ({dev:.1e})")));
        }
    }
}

/// Signed distance residual of `q` to `s` and the unit normal at the foot
/// (the foot from `hint` when given).
fn foot_n(s: &Surface, q: Vec3) -> Option<(Vec3, Vec3)> {
    let (u, v, p) = s.closest_point(q);
    Some((p, s.normal(u, v).try_normalize()?))
}

/// The curve where two offset faces meet, replacing a spline edge: points
/// found per sample in the plane normal to the old edge, interpolated by a
/// cubic B-spline and verified against both offset surfaces.
fn offset_spline_edge(src: &Body, e: EdgeId, moved: &[Option<Surface>], dnat: &dyn Fn(FaceId) -> f64, p0: Vec3, p1: Vec3, end_gap: f64) -> Result<(Curve, f64, f64), OffsetError> {
    let ed = src.edge(e);
    let fs = src.edge_faces(e);
    if fs.len() != 2 || ed.v0 == ed.v1 {
        return Err(OffsetError::new("offsetUnsupported", "edge curve type"));
    }
    let (m1, m2) = (moved[fs[0].idx()].as_ref().unwrap(), moved[fs[1].idx()].as_ref().unwrap());
    let grazing = std::cell::Cell::new(false);
    let sample = |t: f64| -> Result<Vec3, OffsetError> {
        let p = ed.curve.eval(t);
        let tn = ed.curve.tangent(t).try_normalize().ok_or_else(|| OffsetError::new("offsetDegenerate", "an edge reverses"))?;
        let mut n = [Vec3::ZERO; 2];
        for (k, &f) in fs.iter().enumerate() {
            let sf = &src.face(f).surface;
            let (u, v, _) = sf.closest_point(p);
            n[k] = sf.normal(u, v).try_normalize().ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate normal"))?;
        }
        // (nearly) tangent faces (a blend meeting its neighbour): the offset
        // surfaces graze each other, so the edge is the offset of the shared
        // boundary itself
        if n[0].cross(n[1]).len() < 0.02 {
            // compare along the OUTWARD normals (the natural ones may be flipped)
            let sg = |f: FaceId| if src.face(f).sense { 1.0 } else { -1.0 };
            let (d0, d1) = (dnat(fs[0]) * sg(fs[0]), dnat(fs[1]) * sg(fs[1]));
            if (d0 - d1).abs() > 1e-9 {
                if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
                    eprintln!("grazing edge with unequal offsets: {} | {} d {d0} {d1}", src.face_id_str(fs[0]), src.face_id_str(fs[1]));
                }
                return Err(OffsetError::new("offsetDegenerate", "the offset faces do not meet at an edge"));
            }
            grazing.set(true);
            return Ok(p + (n[0] * sg(fs[0]) + n[1] * sg(fs[1])) * (0.5 * d0));
        }
        let mut q = p + crate::intersect::solve3([n[0], n[1], tn], [dnat(fs[0]), dnat(fs[1]), 0.0]).ok_or_else(|| {
            if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
                eprintln!("edge start system singular: {} | {} t={t}", src.face_id_str(fs[0]), src.face_id_str(fs[1]));
            }
            OffsetError::new("offsetDegenerate", "the offset faces do not meet at an edge")
        })?;
        for _ in 0..40 {
            let (q1, n1) = foot_n(m1, q).ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate normal"))?;
            let (q2, n2) = foot_n(m2, q).ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate normal"))?;
            let g = [(q - q1).dot(n1), (q - q2).dot(n2), (q - p).dot(tn)];
            if g.iter().all(|x| x.abs() < 1e-11) {
                return Ok(q);
            }
            let dq = crate::intersect::solve3([n1, n2, tn], [-g[0], -g[1], -g[2]]).ok_or_else(|| OffsetError::new("offsetDegenerate", "the offset faces do not meet at an edge"))?;
            q = q + dq;
        }
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            eprintln!("edge sample failed: {} | {} t={t} q={q:?} p={p:?}", src.face_id_str(fs[0]), src.face_id_str(fs[1]));
        }
        Err(OffsetError::new("offsetDegenerate", "the offset faces do not meet at an edge"))
    };
    let mut n = 24usize;
    loop {
        let mut pts = Vec::with_capacity(n + 1);
        for k in 0..=n {
            pts.push(match k {
                0 => p0,
                _ if k == n => p1,
                _ => sample(ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64)?,
            });
        }
        if pts.windows(2).any(|w| w[0].dist(w[1]) < 1e-12) {
            return Err(OffsetError::new("offsetDegenerate", "an edge reverses"));
        }
        let c = ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength).map_err(|e| OffsetError::new("offsetUnsupported", format!("edge fit: {e}")))?;
        let (a, b) = c.domain();
        let dev = (0..4 * n).map(|k| {
            let q = c.eval(a + (b - a) * (k as f64 + 0.5) / (4 * n) as f64);
            m1.closest_point(q).2.dist(q).max(m2.closest_point(q).2.dist(q))
        }).fold(0.0f64, f64::max);
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            eprintln!("offset edge fit: {} | {} n={n} dev={dev:.2e} grazing={}", src.face_id_str(fs[0]), src.face_id_str(fs[1]), grazing.get());
        }
        // grazing neighbours (G1 seams) leave a gap of ~ d x (normal mismatch)
        // between their offsets that no edge can close
        if dev <= if grazing.get() { GRAZE_TOL } else { EDGE_FIT_TOL.max((1.5 * end_gap).min(GRAZE_TOL)) } {
            return Ok((Curve::BSpline(c), a, b));
        }
        n *= 2;
        if n > 800 {
            return Err(OffsetError::new("offsetUnsupported", format!("edge offset fit tolerance ({dev:.1e})")));
        }
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
    // Levenberg-Marquardt: grazing (G1) neighbours make the system rank
    // deficient, so a damped step keeps the corner near where the faces
    // meet instead of flying along their common tangent. The damping fades
    // out once the residual is small (regular corners converge quadratically).
    let mut p = p0;
    let reach: f64 = surfs.iter().map(|s| s.closest_point(p0).2.dist(p0)).fold(0.0, f64::max);
    let _ = reach;
    let mut lambda: f64 = 1e-6;
    let mut last = f64::INFINITY;
    for _ in 0..80 {
        let mut jtj = [[0.0f64; 3]; 3];
        let mut jtr = [0.0f64; 3];
        let mut worst = 0.0f64;
        let mut sq = 0.0f64;
        for s in surfs {
            let (u, v, q) = s.closest_point(p);
            let n = s.normal(u, v).try_normalize()?;
            let r = (p - q).dot(n);
            worst = worst.max(r.abs());
            sq += r * r;
            let nn = [n.x, n.y, n.z];
            for i in 0..3 {
                for j in 0..3 {
                    jtj[i][j] += nn[i] * nn[j];
                }
                jtr[i] += nn[i] * r;
            }
        }
        if sq > last {
            lambda = (lambda * 10.0).min(1.0);
        } else {
            lambda = (lambda * 0.3).max(1e-12);
        }
        last = sq;
        let mut a = jtj;
        for i in 0..3 {
            a[i][i] += lambda;
        }
        let dp = solve3(a, [-jtr[0], -jtr[1], -jtr[2]])?;
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

    // new surfaces (free-form offsets extend `reach` past their faces: the
    // cavity of a shell overshoots an opened face by the wall thickness)
    let reach = 1.5 * src.face_ids().map(|f| dist(f).abs()).fold(0.0f64, f64::max);
    let mut moved: Vec<Option<Surface>> = vec![None; src.faces.len()];
    for f in src.face_ids() {
        let face = src.face(f);
        let d = dist(f);
        let nat = if face.sense { d } else { -d };
        let r = if is_freeform(&face.surface) {
            match face_uv_box(src, f) {
                Some(bx) => fit_offset_surface(&face.surface, nat, bx, reach),
                None => Err(OffsetError::new("offsetUnsupported", "face extent")),
            }
        } else {
            offset_surface(&face.surface, nat)
        };
        moved[f.idx()] = Some(r.map_err(|e| {
            if e.code == "offsetTooLarge" {
                OffsetError::new("offsetDegenerate", format!("{} collapses and cannot be removed", src.face_id_str(f)))
            } else {
                OffsetError::new(e.code, format!("{}: {}", src.face_id_str(f), e.detail.unwrap_or_default()))
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
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            let q = newp[v.idx()];
            let res = surfs.iter().map(|m| m.closest_point(q).2.dist(q)).fold(0.0f64, f64::max);
            eprintln!("vertex {:?} {:?} -> moved {:.3} residual {:.2e} faces {:?}", v, src.vertex(v).p, q.dist(src.vertex(v).p), res, vfaces[v.idx()].iter().map(|&f| src.face_id_str(f)).collect::<Vec<_>>());
        }
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
            Curve::BSpline(_) => {
                let dn = |f: FaceId| {
                    let d = dist(f);
                    if src.face(f).sense { d } else { -d }
                };
                // how far the solved end vertices sit from the offset surfaces
                // of the faces around them (grazing neighbours cannot all be met)
                let gap = [ed.v0, ed.v1]
                    .iter()
                    .map(|&v| {
                        let q = newp[v.idx()];
                        vfaces[v.idx()].iter().map(|&f| moved[f.idx()].as_ref().map_or(0.0, |m| m.closest_point(q).2.dist(q))).fold(0.0f64, f64::max)
                    })
                    .fold(0.0f64, f64::max);
                offset_spline_edge(src, e, &moved, &dn, p0, p1, gap)?
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

#[cfg(test)]
mod freeform_tests {
    use super::*;

    /// A bowl-shaped bicubic patch (radius of curvature about 10 at the centre).
    fn bowl() -> Surface {
        let n = 9;
        let grid: Vec<Vec<Vec3>> = (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| {
                        let (x, y) = (-4.0 + 8.0 * i as f64 / (n - 1) as f64, -4.0 + 8.0 * j as f64 / (n - 1) as f64);
                        Vec3::new(x, y, 10.0 - (100.0 - x * x - y * y).sqrt())
                    })
                    .collect()
            })
            .collect();
        Surface::BSpline(Box::new(ck_geom::fit::interpolate_surface(&grid, 3, 3).unwrap()))
    }

    #[test]
    fn offset_of_bspline_is_d_from_base() {
        let s = bowl();
        let ((u0, u1), (v0, v1)) = s.domain();
        for d in [2.0, -2.0, 0.5] {
            let o = fit_offset_surface(&s, d, ((u0, u1), (v0, v1)), 0.0).unwrap();
            for i in 0..=8 {
                for j in 0..=8 {
                    let p = o.eval(0.02 + 0.96 * i as f64 / 8.0, 0.02 + 0.96 * j as f64 / 8.0);
                    let dist = s.closest_point(p).2.dist(p);
                    assert!((dist - d.abs()).abs() < 5e-5, "d={d}: {dist}");
                }
            }
        }
    }

    #[test]
    fn offset_past_curvature_centre_is_too_large() {
        // the bowl's centre of curvature is ~10 away on the concave side
        let s = bowl();
        let dom = s.domain();
        let normal_up = {
            let ((u0, u1), (v0, v1)) = dom;
            s.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)).z > 0.0
        };
        // offset towards the centre of curvature (normal z>0 means towards +z, the bowl's concave side)
        let d = if normal_up { 14.0 } else { -14.0 };
        let e = fit_offset_surface(&s, d, dom, 0.0).unwrap_err();
        assert_eq!(e.code, "offsetTooLarge");
    }
}
