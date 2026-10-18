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
    /// Why the offset is impossible, in numbers (set for curvature / thin-wall failures).
    pub limit: Option<OffsetLimit>,
}

/// The numbers behind an offset that cannot be made.
#[derive(Debug, Clone, PartialEq)]
pub struct OffsetLimit {
    /// `"curvature"` (the offset passes a centre of curvature) or `"thin"` (two sheets of the
    /// body are closer than twice the offset).
    pub kind: &'static str,
    /// Smallest radius of curvature found (curvature), or the narrowest gap between the sheets (thin).
    pub min_radius: f64,
    /// The largest offset that still works (a hair under the radius, or half the gap).
    pub max_thickness: f64,
    /// The face that limits the offset.
    pub face: Option<String>,
}

impl OffsetError {
    fn new(code: &'static str, detail: impl Into<String>) -> OffsetError {
        OffsetError { code, detail: Some(detail.into()), limit: None }
    }
    fn with_limit(mut self, limit: OffsetLimit) -> OffsetError {
        self.limit = Some(limit);
        self
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
    // the offset reaches the centre of curvature: `r` is the radius it ran into (measured along the normal)
    let big_r = |r: f64| OffsetError::new("offsetTooLarge", CENTRE).with_limit(OffsetLimit { kind: "curvature", min_radius: r, max_thickness: floor_hundredth(LIMIT_SAFETY * r), face: None });
    match s {
        Surface::Plane { frame } => Ok(Surface::Plane { frame: Frame { o: frame.o + frame.z.scale(d), ..frame.clone() } }),
        Surface::Cylinder { frame, r } => {
            let r2 = r + d;
            if r2 > EPS { Ok(Surface::Cylinder { frame: frame.clone(), r: r2 }) } else { Err(big_r(*r)) }
        }
        Surface::Cone { frame, r, semi_angle } => {
            let r2 = r + d / semi_angle.cos();
            if r2 > EPS { Ok(Surface::Cone { frame: frame.clone(), r: r2, semi_angle: *semi_angle }) } else { Err(big_r(*r * semi_angle.cos())) }
        }
        Surface::Sphere { frame, r } => {
            let r2 = r + d;
            if r2 > EPS { Ok(Surface::Sphere { frame: frame.clone(), r: r2 }) } else { Err(big_r(*r)) }
        }
        Surface::Torus { frame, major, minor } => {
            let m2 = minor + d;
            if m2 > EPS { Ok(Surface::Torus { frame: frame.clone(), major: *major, minor: m2 }) } else { Err(big_r(*minor)) }
        }
        _ => Err(OffsetError::new("offsetUnsupported", "surface type")),
    }
}

/// Offset surface of one face by `d` along its natural normal (exact
/// analytic kinds, fitted free-form ones; `reach` = extension past the face).
pub fn offset_face_surface(src: &Body, f: FaceId, d: f64, reach: f64) -> Result<Surface, OffsetError> {
    let face = src.face(f);
    if is_freeform(&face.surface) {
        fit_face_with_margin(src, f, d, reach)
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
/// Offset edge curves are refined until they follow both offset surfaces this closely (mm).
const GOOD_EDGE_TOL: f64 = 5e-5;
/// Sample spans shorter than this (mm) are not refined further.
const MIN_SPAN: f64 = 0.04;

/// How far (mm) past a free-form face its offset surface is continued as a margin.
fn margin_cap(d: f64) -> f64 {
    (8.0 * d.abs()).max(1.0)
}

/// Fit the offset of free-form face `f`, continued past the face by a margin. The continuation of a
/// quickly turning surface (a loft's tapering last span) folds over a few thicknesses out, which the fit
/// reports as a tolerance failure: retry with a smaller margin (the margin is only slack for the edge
/// curves, which need the offset surfaces a little beyond the face).
fn fit_face_with_margin(src: &Body, f: FaceId, d: f64, reach: f64) -> Result<Surface, OffsetError> {
    let cap0 = margin_cap(d);
    let mut last = None;
    for k in 0..5 {
        let bx = face_uv_box(src, f, cap0 / f64::from(1u32 << (2 * k).min(8))).ok_or_else(|| OffsetError::new("offsetUnsupported", "face extent"))?;
        match fit_offset_surface(&src.face(f).surface, d, bx, reach) {
            Err(e) if e.code == "offsetUnsupported" && e.detail.as_deref().is_some_and(|m| m.contains("fit tolerance")) => last = Some(e),
            r => return r,
        }
    }
    Err(last.unwrap())
}

fn is_freeform(s: &Surface) -> bool {
    matches!(s, Surface::BSpline(_) | Surface::Screw { .. } | Surface::Revolution { .. } | Surface::Extrusion { .. } | Surface::Offset { .. })
}

/// (u, v) bounding box of the face's loops on its surface (periodic
/// parameters unwrapped along the loop), grown by a margin and clamped to a
/// finite domain.
fn face_uv_box(src: &Body, f: FaceId, cap_mm: f64) -> Option<((f64, f64), (f64, f64))> {
    let s = &src.face(f).surface;
    let (mut u0, mut u1, mut v0, mut v1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    let mut prev: Option<ck_geom::Vec2> = None;
    let fp = crate::boolean::classify::spline_fp(s);
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
                // an extruded closed spline has a multi-lap profile domain: the global
                // closest-point search may land on another lap, the pcurve never does
                let pc = if matches!(s, Surface::Extrusion { .. }) { src.coedge(c).pcurve.as_ref() } else { None };
                let uv = match pc {
                    Some(pc) => pc.eval(t),
                    None => crate::boolean::classify::uv_near_fp(s, fp, ed.curve.eval(t), prev),
                };
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
    let (mut mu, mut mv) = (0.03 * (u1 - u0) + 1e-9, 0.03 * (v1 - v0) + 1e-9);
    // On a long free-form face 3 % is many wall thicknesses: the second-order continuation of the
    // surface used there folds over (a loft whose last span turns quickly), so the margin is capped
    // to a few thicknesses in mm.
    if cap_mm.is_finite() && matches!(s, Surface::BSpline(_)) {
        let ((du0, du1), (dv0, dv1)) = s.domain();
        let (mut su, mut sv) = (1e-9f64, 1e-9f64);
        for i in 0..=4 {
            for j in 0..=4 {
                let (a, b) = ((u0 + (u1 - u0) * i as f64 / 4.0).clamp(du0, du1), (v0 + (v1 - v0) * j as f64 / 4.0).clamp(dv0, dv1));
                let q = s.derivs2(a, b);
                su = su.max(q.du.len());
                sv = sv.max(q.dv.len());
            }
        }
        mu = mu.min(cap_mm / su + 1e-9);
        mv = mv.min(cap_mm / sv + 1e-9);
    }
    let (a, b, c, d) = (u0 - mu, u1 + mu, v0 - mv, v1 + mv);
    Some(((a, b), (c, d)))
}

/// Tensor-product cubic interpolation of a rectangular point grid, returning
/// the surface and the (averaged chord-length) parameters of the grid lines.
pub(crate) fn fit_grid(grid: &[Vec<Vec3>]) -> Result<(ck_geom::nurbs::bsurf::BSplineSurface, Vec<f64>, Vec<f64>), OffsetError> {
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
    match fit_offset_surface_inner(s, d, uvbox, reach) {
        Err(mut e) if e.code == "offsetTooLarge" && e.limit.is_none() => {
            // say how tight the surface is: smallest radius on the side the offset moves to
            if let Some(r) = grid_min_radius(s, d, uvbox) {
                e.limit = Some(OffsetLimit { kind: "curvature", min_radius: r, max_thickness: floor_hundredth(LIMIT_SAFETY * r), face: None });
            }
            Err(e)
        }
        r => r,
    }
}

/// Smallest radius of curvature of `s` over `uvbox` (a 40 x 40 sample grid) among the principal
/// directions that curve toward the side an offset by `d` (along the natural normal) moves to;
/// `None` when no sample curves that way. The offset factor of a principal direction is
/// `1 - d kappa` with `kappa` the eigenvalue of `G^-1 II` (`II = n . S_xy`).
pub fn grid_min_radius(s: &Surface, d: f64, uvbox: ((f64, f64), (f64, f64))) -> Option<f64> {
    let ((u0, u1), (v0, v1)) = uvbox;
    let ((du0, du1), (dv0, dv1)) = s.domain();
    let (u0, u1, v0, v1) = (u0.max(du0), u1.min(du1), v0.max(dv0), v1.min(dv1));
    if !(u1 > u0 && v1 > v0) {
        return None;
    }
    let n = 40;
    let mut best: Option<f64> = None;
    for i in 0..=n {
        for j in 0..=n {
            let sd = s.derivs2(u0 + (u1 - u0) * i as f64 / n as f64, v0 + (v1 - v0) * j as f64 / n as f64);
            let Some(nn) = sd.du.cross(sd.dv).try_normalize() else { continue };
            let (e, f, g) = (sd.du.dot(sd.du), sd.du.dot(sd.dv), sd.dv.dot(sd.dv));
            let (l, m, nv) = (nn.dot(sd.duu), nn.dot(sd.duv), nn.dot(sd.dvv));
            let det = e * g - f * f;
            if det <= 1e-24 {
                continue;
            }
            // shape operator G^-1 II
            let (a, b, c, dd) = ((g * l - f * m) / det, (g * m - f * nv) / det, (e * m - f * l) / det, (e * nv - f * m) / det);
            let mean = 0.5 * (a + dd);
            let disc = (0.25 * (a - dd) * (a - dd) + b * c).max(0.0).sqrt();
            for k in [mean + disc, mean - disc] {
                if d * k > 1e-12 {
                    let r = 1.0 / k.abs();
                    best = Some(best.map_or(r, |x: f64| x.min(r)));
                }
            }
        }
    }
    best
}

/// Points where the surface collapses (a boundary row or column of control points is one point:
/// the apex of a loft ended by a point section).
/// Which boundary rows / columns of the parameter domain collapse to a point: (u min, u max, v min, v max).
pub fn pole_sides(s: &Surface) -> [bool; 4] {
    let ((u0, u1), (v0, v1)) = s.domain();
    let mut out = [false; 4];
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()) {
        return out;
    }
    for (edge, flag) in out.iter_mut().enumerate() {
        let pts: Vec<Vec3> = (0..=6)
            .map(|k| {
                let t = k as f64 / 6.0;
                match edge {
                    0 => s.eval(u0, v0 + (v1 - v0) * t),
                    1 => s.eval(u1, v0 + (v1 - v0) * t),
                    2 => s.eval(u0 + (u1 - u0) * t, v0),
                    _ => s.eval(u0 + (u1 - u0) * t, v1),
                }
            })
            .collect();
        let scale = 1.0 + pts[0].len();
        *flag = pts.iter().all(|q| q.dist(pts[0]) < 1e-7 * scale);
    }
    out
}

pub fn pole_points(s: &Surface) -> Vec<Vec3> {
    let ((u0, u1), (v0, v1)) = s.domain();
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()) {
        return vec![];
    }
    let mut out: Vec<Vec3> = vec![];
    for edge in 0..4 {
        let pts: Vec<Vec3> = (0..=6)
            .map(|k| {
                let t = k as f64 / 6.0;
                match edge {
                    0 => s.eval(u0, v0 + (v1 - v0) * t),
                    1 => s.eval(u1, v0 + (v1 - v0) * t),
                    2 => s.eval(u0 + (u1 - u0) * t, v0),
                    _ => s.eval(u0 + (u1 - u0) * t, v1),
                }
            })
            .collect();
        let scale = 1.0 + pts[0].len();
        if pts.iter().all(|q| q.dist(pts[0]) < 1e-7 * scale) && !out.iter().any(|q| q.dist(pts[0]) < 1e-7 * scale) {
            out.push(pts[0]);
        }
    }
    out
}

fn fit_offset_surface_inner(s: &Surface, d: f64, uvbox: ((f64, f64), (f64, f64)), reach: f64) -> Result<Surface, OffsetError> {
    let big = || OffsetError::new("offsetTooLarge", CENTRE);
    if let Surface::Revolution { profile, axis_o, axis } = s {
        return offset_revolution(s, profile, *axis_o, *axis, d, uvbox.1, reach);
    }
    if let Surface::Extrusion { profile, dir } = s {
        return offset_extrusion(profile, *dir, d, uvbox.0, reach);
    }
    let ((mut u0, mut u1), (mut v0, mut v1)) = uvbox;
    if !(u1 > u0 && v1 > v0) {
        return Err(OffsetError::new("offsetUnsupported", "empty parameter range"));
    }
    let off = Surface::Offset { base: Box::new(s.clone()), dist: d };
    let ((du0, du1), (dv0, dv1)) = s.domain();
    // An inward offset of a pole folds over itself near the apex (the offset cone's other nappe):
    // that fold is where the inner apex closes, not a failure. The curvature and fold checks skip
    // the neighbourhood of a pole, and the surface is not continued past it.
    let poles = pole_points(s);
    let pole_side = pole_sides(s);
    let near_pole = |p: Vec3| poles.iter().any(|q| q.dist(p) < 10.0 * d.abs());
    // the fit is judged right up to the apex, except in the last sliver where the normal field turns
    // over faster than any sampling can follow (and nothing of the solid is)
    let at_pole = |p: Vec3| poles.iter().any(|q| q.dist(p) < MIN_SPAN);

    // The cavity of a shell runs `reach` past an opened face, so its
    // neighbours' offsets must extend beyond the original surface: pad the box
    // by that distance (in parameters) and continue the surface past its
    // domain with its second-order Taylor polynomial.
    if reach > 0.0 {
        let (mid_u, mid_v) = ((0.5 * (u0 + u1)).clamp(du0, du1), (0.5 * (v0 + v1)).clamp(dv0, dv1));
        let sd = s.derivs2(mid_u, mid_v);
        let (mut lu, mut lv) = (sd.du.len().max(1e-9), sd.dv.len().max(1e-9));
        // A loft narrows towards its ends: the parameter speed at the mid point overstates the
        // speed there, so a pad of `reach` (mm) there is far too short in parameters. Take the
        // slowest speed over the box (not below 5 % of the fastest, a pole collapses it to zero).
        let (mut fu, mut fv) = (lu, lv);
        for i in 0..=6 {
            for j in 0..=6 {
                let (a, b) = ((u0 + (u1 - u0) * i as f64 / 6.0).clamp(du0, du1), (v0 + (v1 - v0) * j as f64 / 6.0).clamp(dv0, dv1));
                let q = s.derivs2(a, b);
                fu = fu.max(q.du.len());
                fv = fv.max(q.dv.len());
                lu = lu.min(q.du.len().max(1e-9));
                lv = lv.min(q.dv.len().max(1e-9));
            }
        }
        let (lu, lv) = (lu.max(0.05 * fu), lv.max(0.05 * fv));
        let (pu, pv) = (reach / lu, reach / lv);
        // periodic directions wrap; an unbounded direction (extrusion height) still has to
        // reach `reach` past the face, the 3 % margin of the box is not enough
        // (towards a pole the pad stops at the apex: the surface does not continue past it)
        if s.periods().0.is_none() {
            u0 = if pole_side[0] { (u0 - pu).max(du0) } else { u0 - pu };
            u1 = if pole_side[1] { (u1 + pu).min(du1) } else { u1 + pu };
        }
        if s.periods().1.is_none() {
            v0 = if pole_side[2] { (v0 - pv).max(dv0) } else { v0 - pv };
            v1 = if pole_side[3] { (v1 + pv).min(dv1) } else { v1 + pv };
        }
    }
    let (per_u, per_v) = s.periods();
    let wrap = |x: f64, lo: f64, hi: f64, per: Option<f64>| match per {
        Some(p) if lo.is_finite() && hi.is_finite() && p > 0.0 && (x < lo || x > hi) => lo + (x - lo).rem_euclid(p),
        _ => x,
    };
    let point = |u: f64, v: f64| -> Result<Vec3, OffsetError> {
        let (u, v) = (wrap(u, du0, du1, per_u), wrap(v, dv0, dv1, per_v));
        let (uc, vc) = (u.clamp(du0, du1), v.clamp(dv0, dv1));
        let (ea, eb) = (u - uc, v - vc);
        let inside = ea == 0.0 && eb == 0.0;
        let sd = s.derivs2(uc, vc);
        let (su, sv) = (sd.du + sd.duu * ea + sd.duv * eb, sd.dv + sd.duv * ea + sd.dvv * eb);
        // at a pole (collapsed row of a loft's point end) S_u x S_v vanishes: use the limit normal
        let n = match su.cross(sv) {
            c if c.len() > 1e-9 * su.len().max(sv.len()).powi(2) => c.normalize(),
            _ => s.normal(uc, vc).try_normalize().ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate surface point"))?,
        };
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
        if det_g > 1e-24 && !near_pole(sd.p) && (j00 < 0.05 * g00 || j11 < 0.05 * g11 || det_j < 0.0025 * det_g) {
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
                // a cubic fit needs at least four samples: a surface without interior knots in this
                // direction (a linear loft span) gets a uniform start
                while x.len() < 5 {
                    let mut y = x.clone();
                    for w in x.windows(2) {
                        y.push(0.5 * (w[0] + w[1]));
                    }
                    y.sort_by(f64::total_cmp);
                    x = y;
                }
                x
            }
            None => (0..=8).map(|i| lo + (hi - lo) * i as f64 / 8.0).collect(),
        }
    };
    let (ku, kv) = match s {
        Surface::BSpline(bs) => (Some(bs.knots_u.clone()), Some(bs.knots_v.clone())),
        _ => (None, None),
    };
    // sample points are kept between refinement rounds (a refined grid reuses every old node and
    // the span midpoints the deviation was measured at)
    let mut cache: std::collections::HashMap<(u64, u64), Vec3> = std::collections::HashMap::new();
    let mut cached = |u: f64, v: f64| -> Result<Vec3, OffsetError> {
        let key = (u.to_bits(), v.to_bits());
        if let Some(p) = cache.get(&key) {
            return Ok(*p);
        }
        let p = point(u, v)?;
        cache.insert(key, p);
        Ok(p)
    };
    let mut us = seed(u0, u1, ku.as_deref());
    let mut vs = seed(v0, v1, kv.as_deref());
    let mut last_dev = f64::INFINITY;
    for _round in 0..24 {
        let mut grid = vec![vec![Vec3::ZERO; vs.len()]; us.len()];
        for (i, &u) in us.iter().enumerate() {
            for (j, &v) in vs.iter().enumerate() {
                grid[i][j] = cached(u, v)?;
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
                if at_pole(s.eval(um.clamp(du0, du1), vs[j].clamp(dv0, dv1))) {
                    continue;
                }
                let e = cached(um, vs[j])?;
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
                if at_pole(s.eval(us[i].clamp(du0, du1), vm.clamp(dv0, dv1))) {
                    continue;
                }
                let e = cached(us[i], vm)?;
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
                    if interior && !near_pole(s.eval(u, v)) && s.closest_point(q).2.dist(q) < d.abs() * (1.0 - 1e-3) - 1e-6 {
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

/// Smallest radius of curvature of face `f` on the side an offset by `d` (along its natural
/// normal) moves to; `None` when the face does not curve that way (or is analytic: planes and
/// the plain quadrics are handled by their exact offset).
pub fn face_min_radius(src: &Body, f: FaceId, d: f64) -> Option<f64> {
    let face = src.face(f);
    let bx = face_uv_box(src, f, f64::INFINITY)?;
    match &face.surface {
        Surface::Extrusion { profile, dir } => {
            let (r0, r1) = profile.range();
            let (u0, u1) = (bx.0 .0.max(r0), bx.0 .1.min(r1));
            let mut best: Option<f64> = None;
            for i in 0..=600 {
                let c = profile.derivs(u0 + (u1 - u0) * i as f64 / 600.0, 2);
                let Some(n) = c[1].cross(*dir).try_normalize() else { continue };
                let kn = n.dot(c[2]) / c[1].len2();
                if d * kn > 1e-12 {
                    let r = 1.0 / kn.abs();
                    best = Some(best.map_or(r, |x: f64| x.min(r)));
                }
            }
            best
        }
        s if is_freeform(s) => grid_min_radius(s, d, bx),
        _ => None,
    }
}

/// The sentence a failed offset of `face` by `d` is reported with.
pub fn limit_message(face: &str, d: f64, l: &OffsetLimit) -> String {
    match l.kind {
        "thin" => format!(
            "{face} runs within {:.2} mm of another part of the same wall, too close for an offset of {:.2} mm: use at most {:.2} mm",
            l.min_radius - d.abs(),
            d.abs(),
            l.max_thickness
        ),
        _ => format!(
            "{face} curves too tightly for an offset of {:.2} mm: its smallest radius of curvature is {:.2} mm, use at most {:.2} mm",
            d.abs(),
            l.min_radius,
            l.max_thickness
        ),
    }
}

/// Safe fraction of the limiting radius / half gap reported as the largest thickness.
const LIMIT_SAFETY: f64 = 0.8;

/// Round a reported limit down to 0.01 mm.
fn floor_hundredth(x: f64) -> f64 {
    (x * 100.0).floor() / 100.0
}

/// Offset of an extruded curve: the extrusion of the offset curve. The profile moves along
/// the in-plane normal (`c' x dir` side, the surface's natural normal) and the offset curve is
/// a cubic B-spline interpolating samples over `ubox` (padded by `reach`), refined until it
/// follows the exact offset within `FIT_TOL`. The offset has to keep the profile regular
/// (`1 - d kappa > 0`: the radius of curvature on the offset side exceeds `|d|`) and must not
/// run into another part of the profile (a wall thinner than `2 |d|`); each failure comes with
/// the numbers (smallest radius / narrowest gap and the thickness that would work).
fn offset_extrusion(profile: &Curve, dir: Vec3, d: f64, ubox: (f64, f64), reach: f64) -> Result<Surface, OffsetError> {
    let (r0, r1) = profile.range();
    let (mut u0, mut u1) = ubox;
    if !(u1 > u0) {
        return Err(OffsetError::new("offsetUnsupported", "empty parameter range"));
    }
    let tmid = profile.derivs(0.5 * (u0 + u1), 1)[1].len().max(1e-9);
    let pad = reach / tmid;
    u0 = (u0 - pad).max(r0);
    u1 = (u1 + pad).min(r1);
    if !(u1 > u0) {
        return Err(OffsetError::new("offsetUnsupported", "empty parameter range"));
    }
    let interior = ubox;
    // (offset point, 1 - d kn, kn) at parameter u
    let sample = |u: f64| -> Result<(Vec3, f64, f64), OffsetError> {
        let c = profile.derivs(u, 2);
        let n = c[1].cross(dir).try_normalize().ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate surface point"))?;
        let kn = n.dot(c[2]) / c[1].len2();
        Ok((c[0] + n * d, 1.0 - d * kn, kn))
    };
    // curvature scan on a dense set
    let (mut worst, mut min_radius) = (f64::INFINITY, f64::INFINITY);
    for i in 0..=600 {
        let (_, factor, kn) = sample(u0 + (u1 - u0) * i as f64 / 600.0)?;
        worst = worst.min(factor);
        if d * kn > 1e-12 {
            min_radius = min_radius.min(1.0 / kn.abs());
        }
    }
    if worst < 0.1 {
        return Err(OffsetError::new("offsetTooLarge", CENTRE).with_limit(OffsetLimit {
            kind: "curvature",
            min_radius,
            max_thickness: floor_hundredth(LIMIT_SAFETY * min_radius),
            face: None,
        }));
    }
    // initial sample set: uniform plus the profile's knots
    let mut us: Vec<f64> = (0..=48).map(|i| u0 + (u1 - u0) * i as f64 / 48.0).collect();
    if let Curve::BSpline(bs) = profile {
        us.extend(bs.knots.iter().copied().filter(|&k| k > u0 && k < u1));
        us.sort_by(f64::total_cmp);
        us.dedup_by(|a, b| (*a - *b).abs() < 1e-9 * (u1 - u0));
    }
    let mut last_dev = f64::INFINITY;
    for _round in 0..16 {
        let pts: Vec<Vec3> = us.iter().map(|&u| sample(u).map(|s| s.0)).collect::<Result<_, _>>()?;
        if pts.windows(2).any(|w| w[0].dist(w[1]) < 1e-12) {
            return Err(OffsetError::new("offsetUnsupported", "degenerate offset sample"));
        }
        let fit = ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength).map_err(|e| OffsetError::new("offsetUnsupported", format!("fit: {e}")))?;
        let mut bad = vec![];
        let mut dev = 0.0f64;
        for i in 0..us.len() - 1 {
            if pts[i].dist(pts[i + 1]) < MIN_SPAN * 0.1 {
                continue;
            }
            let e = sample(0.5 * (us[i] + us[i + 1]))?.0;
            let w = fit.closest_point(e).1.dist(e);
            dev = dev.max(w);
            if w > FIT_TOL {
                bad.push(i);
            }
        }
        last_dev = dev;
        if bad.is_empty() {
            // thin walls: the offset must stay |d| from every other part of the profile
            let mut gap = f64::INFINITY;
            let m = 300;
            for i in 0..=m {
                let u = u0 + (u1 - u0) * i as f64 / m as f64;
                if u < interior.0 || u > interior.1 {
                    continue;
                }
                let q = sample(u)?.0;
                let t = profile.closest_param(q).clamp(r0, r1);
                let dist = profile.eval(t).dist(q);
                if dist < d.abs() * (1.0 - 1e-3) - 1e-6 {
                    gap = gap.min(d.abs() + dist);
                }
            }
            if gap.is_finite() {
                return Err(OffsetError::new("offsetTooLarge", "the offset runs into another part of the same wall").with_limit(OffsetLimit {
                    kind: "thin",
                    min_radius: gap,
                    max_thickness: floor_hundredth(LIMIT_SAFETY * 0.5 * gap),
                    face: None,
                }));
            }
            return Ok(Surface::Extrusion { profile: Box::new(Curve::BSpline(fit)), dir });
        }
        if us.len() + bad.len() > 3000 {
            break;
        }
        for &i in bad.iter().rev() {
            us.insert(i + 1, 0.5 * (us[i] + us[i + 1]));
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
    // A short original edge can move a long way (offsets slide along grazing
    // faces): its samples would cover only a sliver of the offset curve, so
    // the curve is sampled between its own end vertices instead (planes
    // through points of the chord p0-p1, normal to it).
    let _ = sample(0.5 * (ed.t0 + ed.t1))?; // sets `grazing` for the edge
    let dmax = fs.iter().map(|&f| dnat(f).abs()).fold(0.0f64, f64::max);
    let chord_sampling = !grazing.get() && src.edge_length(e) < 3.0 * dmax + 1e-9 && p0.dist(p1) > 1e-9;
    let sample_chord = |s: f64| -> Result<Vec3, OffsetError> {
        let x = p0 + (p1 - p0) * s;
        let dir = (p1 - p0).normalize();
        let mut q = x;
        for _ in 0..40 {
            let (q1, n1) = foot_n(m1, q).ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate normal"))?;
            let (q2, n2) = foot_n(m2, q).ok_or_else(|| OffsetError::new("offsetUnsupported", "degenerate normal"))?;
            let g = [(q - q1).dot(n1), (q - q2).dot(n2), (q - x).dot(dir)];
            if g.iter().all(|v| v.abs() < 1e-11) {
                return Ok(q);
            }
            let dq = crate::intersect::solve3([n1, n2, dir], [-g[0], -g[1], -g[2]]).ok_or_else(|| OffsetError::new("offsetDegenerate", "the offset faces do not meet at an edge"))?;
            q = q + dq;
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
                _ if chord_sampling => sample_chord(k as f64 / n as f64)?,
                _ => sample(ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64)?,
            });
        }
        if pts.windows(2).any(|w| w[0].dist(w[1]) < 1e-12) {
            return Err(OffsetError::new("offsetDegenerate", "an edge reverses"));
        }
        let c = ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength).map_err(|e| OffsetError::new("offsetUnsupported", format!("edge fit: {e}")))?;
        let (a, b) = c.domain();
        let devs: Vec<(f64, Vec3)> = (0..4 * n)
            .map(|k| {
                let q = c.eval(a + (b - a) * (k as f64 + 0.5) / (4 * n) as f64);
                (m1.closest_point(q).2.dist(q).max(m2.closest_point(q).2.dist(q)), q)
            })
            .collect();
        let dev = devs.iter().map(|x| x.0).fold(0.0f64, f64::max);
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() && dev > 1e-2 {
            let w = devs.iter().cloned().fold((0.0, Vec3::ZERO), |a, b| if b.0 > a.0 { b } else { a });
            let k = devs.iter().position(|x| x.0 == w.0).unwrap();
            eprintln!(
                "  worst at {:?} (sample {}/{}) p0 {:?} p1 {:?} orig edge {:?}->{:?}; dev to m1 {:.2e} m2 {:.2e}",
                w.1, k, devs.len(), p0, p1, ed.curve.eval(ed.t0), ed.curve.eval(ed.t1),
                m1.closest_point(w.1).2.dist(w.1), m2.closest_point(w.1).2.dist(w.1)
            );
        }
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            eprintln!("offset edge fit: {} | {} n={n} dev={dev:.2e} grazing={}", src.face_id_str(fs[0]), src.face_id_str(fs[1]), grazing.get());
        }
        // grazing neighbours (G1 seams) leave a gap of ~ d x (normal mismatch)
        // between their offsets that no edge can close
        let limit = if grazing.get() { GRAZE_TOL } else { EDGE_FIT_TOL.max((1.5 * end_gap).min(GRAZE_TOL)) };
        // Refine for quality, not just for the limit: a coarse fit that merely passes the limit
        // leaves the cut solid's edges off their faces by up to the limit (the validator and the
        // downstream booleans see that). Refine to `GOOD_EDGE_TOL`, accept the limit only once
        // refinement has stopped paying.
        let good = if grazing.get() { limit } else { limit.min(GOOD_EDGE_TOL) };
        if dev <= good || (dev <= limit && n >= 192) {
            return Ok((Curve::BSpline(c), a, b));
        }
        n *= 2;
        if n > 800 {
            return Err(OffsetError::new("offsetUnsupported", format!("edge offset fit tolerance ({dev:.1e})")));
        }
    }
}

/// The edge between an extruded wall and a plane normal to the extrusion is the wall's profile
/// itself placed in the plane: exact, no sampling. `None` when the edge is anything else.
fn exact_cap_edge(fs: &[FaceId], moved: &[Option<Surface>], p0: Vec3, p1: Vec3) -> Option<(Curve, f64, f64)> {
    if fs.len() != 2 {
        return None;
    }
    let (m1, m2) = (moved[fs[0].idx()].as_ref()?, moved[fs[1].idx()].as_ref()?);
    let (ext, pl) = match (m1, m2) {
        (Surface::Extrusion { .. }, Surface::Plane { .. }) => (m1, m2),
        (Surface::Plane { .. }, Surface::Extrusion { .. }) => (m2, m1),
        _ => return None,
    };
    let (Surface::Extrusion { profile, dir }, Surface::Plane { frame }) = (ext, pl) else { return None };
    let Curve::BSpline(bs) = &**profile else { return None };
    if frame.z.cross(*dir).len() > 1e-9 * dir.len() {
        return None;
    }
    let v = (frame.o - bs.eval(bs.domain().0)).dot(frame.z) / dir.dot(frame.z);
    let shift = *dir * v;
    let placed = bs.map(|p| p + shift);
    let (a, b) = placed.domain();
    let (ta, tb) = (placed.closest_param(p0), placed.closest_param(p1));
    let tol = 1e-6 * (1.0 + p0.len());
    if placed.eval(ta).dist(p0) > tol || placed.eval(tb).dist(p1) > tol {
        return None;
    }
    if tb > ta {
        Some((Curve::BSpline(placed), ta, tb))
    } else {
        Some((Curve::BSpline(placed.reversed()), a + b - ta, a + b - tb))
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

/// Length of the repeating stretch of a closed spline curve whose parameter has more than one
/// answer per point: a two-lap unrolling (the sketch builds closed splines that way so trimmed
/// pieces may wrap; `bs(t) == bs(t + L)` for `L` = half the domain) or a single closed lap (the
/// seam point is both the first and the last parameter).
fn lap_length(bs: &ck_geom::nurbs::bspline::BSplineCurve) -> Option<f64> {
    let (a, b) = bs.domain();
    let l = 0.5 * (b - a);
    if !(l.is_finite() && l > 0.0) {
        return None;
    }
    let scale = 1.0 + bs.eval(a).len();
    if (0..7).all(|k| {
        let t = a + l * (k as f64 + 0.37) / 7.0;
        bs.eval(t).dist(bs.eval(t + l)) < 1e-9 * scale
    }) {
        return Some(l);
    }
    (bs.eval(a).dist(bs.eval(b)) < 1e-9 * scale).then_some(b - a)
}

/// Copy of `src` whose two-lap closed-spline curves (wall profiles of extruded closed splines,
/// their cap edges) are cut down to the lap window the faces / edges actually use, keeping the
/// parameters (pcurves stay valid). A point on such a curve has two parameters one lap apart, so
/// every closest-parameter search (boolean, pcurve rebuild, mass) may land on the wrong lap and
/// scramble a face's pcurves; with one lap per curve there is a single answer.
pub fn single_lap_body(src: &Body) -> Body {
    use ck_geom::nurbs::bspline::BSplineCurve;
    let mut out = src.clone();
    let window = |bs: &BSplineCurve, lo: f64, hi: f64| -> Option<BSplineCurve> {
        let l = lap_length(bs)?;
        let (a, b) = bs.domain();
        let m = 0.04 * l;
        if hi - lo > 0.85 * l {
            return None;
        }
        Some(bs.segment((lo - m).max(a), (hi + m).min(b)))
    };
    for f in src.face_ids() {
        let Surface::Extrusion { profile, dir } = &src.face(f).surface else { continue };
        let Curve::BSpline(bs) = &**profile else { continue };
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for &l in &src.face(f).loops {
            if !src.lp(l).alive {
                continue;
            }
            for &c in &src.lp(l).coedges {
                if !src.coedge(c).alive {
                    continue;
                }
                let Some(pc) = &src.coedge(c).pcurve else { return src.clone() };
                let (ta, tb) = src.coedge_params(c);
                for k in 0..=16 {
                    let u = pc.eval(ta + (tb - ta) * k as f64 / 16.0).x;
                    lo = lo.min(u);
                    hi = hi.max(u);
                }
            }
        }
        if let Some(w) = window(bs, lo, hi) {
            out.faces[f.idx()].surface = Surface::Extrusion { profile: Box::new(Curve::BSpline(w)), dir: *dir };
        }
    }
    for e in src.edge_ids() {
        let ed = src.edge(e);
        let Curve::BSpline(bs) = &ed.curve else { continue };
        if let Some(w) = window(bs, ed.t0.min(ed.t1), ed.t0.max(ed.t1)) {
            out.edges[e.idx()].curve = Curve::BSpline(w);
        }
    }
    out
}

/// Offset the faces of `src`: `dist(face)` is the signed distance along the
/// face's OUTWARD normal (positive = grow). Returns the moved copy (same
/// topology and element order, pcurves dropped).
pub fn offset_body(src: &Body, dist: &dyn Fn(FaceId) -> f64) -> Result<Body, OffsetError> {
    move_faces_body(src, &|f| FaceMove::Offset(dist(f)))
}

/// What happens to one face in [`move_faces_body`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaceMove {
    /// Along the outward normal by a signed distance (positive grows); planes slide, round
    /// surfaces change radius.
    Offset(f64),
    /// The face's surface translates rigidly (a tangent fillet carried along with a pushed wall).
    Shift(Vec3),
}

/// [`offset_body`] generalised: each face either offsets along its normal or translates
/// rigidly. Vertices are re-solved on the moved surfaces (seeded where a translated face
/// puts them), edges follow, circles about a translated axis move with it.
pub fn move_faces_body(src: &Body, mv: &dyn Fn(FaceId) -> FaceMove) -> Result<Body, OffsetError> {
    let dist = |f: FaceId| match mv(f) {
        FaceMove::Offset(d) => d,
        FaceMove::Shift(_) => 0.0,
    };
    let shift_of = |f: FaceId| match mv(f) {
        FaceMove::Shift(s) => Some(s),
        FaceMove::Offset(_) => None,
    };
    let mut out = src.clone();

    // new surfaces (free-form offsets extend `reach` past their faces: the
    // cavity of a shell overshoots an opened face by the wall thickness)
    let reach = 1.5 * src.face_ids().map(|f| dist(f).abs()).fold(0.0f64, f64::max);
    let dbg_clock = std::env::var_os("CK_OFFSET_DEBUG").is_some().then(std::time::Instant::now);
    let mut moved: Vec<Option<Surface>> = vec![None; src.faces.len()];
    for f in src.face_ids() {
        let face = src.face(f);
        let d = dist(f);
        let nat = if face.sense { d } else { -d };
        let r = if let Some(s) = shift_of(f) {
            Ok(ck_topo::body::transform_surface(&face.surface, &ck_core::xform::Xform::translation(s)))
        } else if is_freeform(&face.surface) {
            fit_face_with_margin(src, f, nat, reach)
        } else {
            offset_surface(&face.surface, nat)
        };
        moved[f.idx()] = Some(r.map_err(|e| {
            if e.code == "offsetTooLarge" {
                let name = src.face_id_str(f);
                match e.limit {
                    Some(mut l) => {
                        let msg = limit_message(&name, d, &l);
                        l.face = Some(name);
                        OffsetError::new("offsetDegenerate", msg).with_limit(l)
                    }
                    None => OffsetError::new("offsetDegenerate", format!("{name} collapses and cannot be removed")),
                }
            } else {
                OffsetError::new(e.code, format!("{}: {}", src.face_id_str(f), e.detail.unwrap_or_default()))
            }
        })?);
    }

    if let Some(t) = dbg_clock {
        eprintln!("offset_body: surfaces {:.2}s", t.elapsed().as_secs_f64());
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
        let seed = vfaces[v.idx()].iter().find_map(|&f| shift_of(f)).map_or(src.vertex(v).p, |s| src.vertex(v).p + s);
        newp[v.idx()] = solve_point(seed, &surfs)
            .ok_or_else(|| OffsetError::new("offsetDegenerate", "the offset faces do not meet at a corner"))?;
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            let q = newp[v.idx()];
            let res = surfs.iter().map(|m| m.closest_point(q).2.dist(q)).fold(0.0f64, f64::max);
            eprintln!("vertex {:?} {:?} -> moved {:.3} residual {:.2e} faces {:?}", v, src.vertex(v).p, q.dist(src.vertex(v).p), res, vfaces[v.idx()].iter().map(|&f| src.face_id_str(f)).collect::<Vec<_>>());
            for (k, m) in surfs.iter().enumerate() {
                let (u, w, c) = m.closest_point(q);
                let (bu, bv, bp) = src.face(vfaces[v.idx()][k]).surface.closest_point(src.vertex(v).p);
                eprintln!("   base uv ({bu:.4},{bv:.4}) dist {:.2e} dom {:?}", bp.dist(src.vertex(v).p), src.face(vfaces[v.idx()][k]).surface.domain());
                eprintln!("   surf {} dist {:.3e} at uv ({u:.4},{w:.4}) domain {:?}", src.face_id_str(vfaces[v.idx()][k]), c.dist(q), m.domain());
            }
        }
    }

    if let Some(t) = dbg_clock {
        eprintln!("offset_body: vertices {:.2}s", t.elapsed().as_secs_f64());
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
        let mut return_edge: Option<(Curve, f64, f64)> = None;
        let (curve, t0, t1) = match &ed.curve {
            Curve::Line { d, .. } => {
                let len = (p1 - p0).dot(*d);
                if len.abs() <= EPS {
                    return Err(shrink());
                }
                if len < 0.0 {
                    return Err(reverses());
                }
                let line = Curve::Line { o: p0 - d.scale(ed.t0), d: *d };
                // A straight edge of a plane and a free-form face (the rim of a loft's end cap) stays
                // straight only when the free-form offset is a ruled continuation of it; in general
                // the offsets meet in a curve. Where the straight line misses the offset surfaces or
                // the far vertex, sample the true meeting curve instead.
                let freeform_nb = fs.len() == 2 && fs.iter().any(|&f| is_freeform(&src.face(f).surface));
                if freeform_nb {
                    let end = line.eval(ed.t0 + len);
                    let miss = (0..=8)
                        .map(|k| line.eval(ed.t0 + len * k as f64 / 8.0))
                        .map(|q| fs.iter().map(|&f| moved[f.idx()].as_ref().map_or(0.0, |m| m.closest_point(q).2.dist(q))).fold(0.0f64, f64::max))
                        .fold(end.dist(p1), f64::max);
                    if miss > 1e-5 {
                        let dn = |f: FaceId| {
                            let d = dist(f);
                            if src.face(f).sense { d } else { -d }
                        };
                        let gap = [ed.v0, ed.v1]
                            .iter()
                            .map(|&v| {
                                let q = newp[v.idx()];
                                vfaces[v.idx()].iter().map(|&f| moved[f.idx()].as_ref().map_or(0.0, |m| m.closest_point(q).2.dist(q))).fold(0.0f64, f64::max)
                            })
                            .fold(0.0f64, f64::max);
                        return_edge = Some(offset_spline_edge(src, e, &moved, &dn, p0, p1, gap)?);
                    }
                }
                match return_edge.take() {
                    Some(r) => r,
                    None => (line, ed.t0, ed.t0 + len),
                }
            }
            Curve::Circle { frame, r } => {
                let z = frame.z;
                // a circle about the axis of a translated round surface moves with it
                let carried = fs
                    .iter()
                    .filter_map(|&f| shift_of(f).map(|s| (f, s)))
                    .find(|&(f, _)| match &src.face(f).surface {
                        Surface::Cylinder { frame: sf, .. } | Surface::Cone { frame: sf, .. } => {
                            sf.z.cross(z).len() < 1e-9 && (frame.o - sf.o).cross(sf.z).len() < 1e-7
                        }
                        _ => false,
                    })
                    .map_or(Vec3::ZERO, |(_, s)| s);
                let fo = frame.o + carried;
                let c = fo + z.scale((p0 - fo).dot(z));
                let r2 = (p0 - c).len();
                if r2 <= EPS {
                    return Err(shrink());
                }
                // angular extent is kept: vertices stay on their radial lines
                let _ = r;
                (Curve::Circle { frame: Frame { o: c, ..frame.clone() }, r: r2 }, ed.t0, ed.t1)
            }
            Curve::BSpline(_) => {
                if fs.iter().any(|&f| shift_of(f).is_some()) {
                    return Err(OffsetError::new("offsetUnsupported", "free-form edge of a translated face"));
                }
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
                match exact_cap_edge(&fs, &moved, p0, p1) {
                    Some(r) => r,
                    None => offset_spline_edge(src, e, &moved, &dn, p0, p1, gap)?,
                }
            }
            _ => return Err(OffsetError::new("offsetUnsupported", "edge curve type")),
        };
        let _ = closed;
        let o = &mut out.edges[e.idx()];
        o.curve = curve;
        o.t0 = t0;
        o.t1 = t1;
    }
    if let Some(t) = dbg_clock {
        eprintln!("offset_body: edges {:.2}s", t.elapsed().as_secs_f64());
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
