//! Analytic refit (stage 2): smooth regions of facets become ONE cylinder, cone or sphere face each.
//!
//! 1. Segmentation. Coplanar regions (stage 1) that meet a neighbour region at a dihedral angle below `smooth_deg` are
//!    facet candidates. From each unclaimed candidate (smallest first) a cluster grows over smooth edges for each primitive
//!    type: a neighbour region joins when all its corners lie within `fit_tol` of the current surface, its centroid within
//!    `tol` (the chord sag) and its normal agrees with the surface normal; the surface is refit (least squares,
//!    Levenberg-Marquardt) after each growth round. The type that claims most triangles wins (a cylinder before a cone or
//!    sphere on a tie). A tangent plane next to a fillet stops the growth because its far corners are off the surface.
//! 2. Boundaries come from the mesh: each region's boundary loops are split into runs, one per neighbouring face. A run
//!    next to an analytic face must be an exact curve on it: a circle coaxial with a cylinder or cone, any plane circle on a
//!    sphere (coaxial with the sphere's frame), a ruling line of a cylinder or a line through a cone's apex. Runs between two
//!    planes are polylines simplified to their corners. A run that is none of these sends its analytic face(s) back to facets
//!    and the topology is built again (each round sends back at least one face, so it ends).
//! 3. Periodic faces get a seam (a ruling, a cone generator or a meridian) at an angle shared by their ring loops, and poles
//!    (a sphere cap, a cone tip) a degenerate seam end; exact pcurves are written as straight lines in (u, v).
//! 4. The body is validated; the faces named by any issue go back to facets (up to three rounds), else the faceted body of
//!    stage 1 stays.
//! Regions that fail stay faceted, so the result is always as valid as the faceted solid.

use crate::{he_from, he_to, region_loops, Closed, Regions};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_json::{Json, Obj};
use ck_topo::body::{Body, EdgeId, FaceName, ShellId, VertexId};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

/// Tolerances of the refit (model units; `None` = relative to the diagonal).
#[derive(Debug, Clone)]
pub struct FitParams {
    /// Largest distance of a facet's centroid from the fitted surface: the chord sag (default 0.5 % of the diagonal).
    pub tol: Option<f64>,
    /// Largest distance of a facet corner from the fitted surface (default 2e-5 of the diagonal: tessellations of CAD
    /// models put their corners on the surface).
    pub fit_tol: Option<f64>,
    /// Neighbouring facets at a smaller dihedral angle (degrees) may belong to one curved surface.
    pub smooth_deg: f64,
    /// Fewest triangles of a fitted face.
    pub min_triangles: usize,
    /// Most planar regions the refit is tried on (above: the faceted body stays, `reason` in the report).
    pub max_regions: usize,
    /// Most seeds tried (each grows a cylinder, a cone and a sphere).
    pub max_seeds: usize,
    /// Work budget of the seed loop, in seconds of a native machine (`maxSeconds` of the request, alias `refitSeconds`; 0: unlimited). It is
    /// counted in work units ([`UNITS_PER_SECOND`]: points evaluated against a surface, facets tested), so it holds on a
    /// host without a clock; where the host has a clock the same number of seconds is also a time limit. When it runs out
    /// the seed loop stops: what is fitted so far stays, the rest stays faceted (`budgetExhausted` in the report).
    pub max_seconds: f64,
}

/// Work units a native machine spends per second in the seed loop (measured on the synthetic free-form blob of
/// `tests/perf_freeform.rs`: 1.3e7 to 3.1e7 units per second of a native machine, the low end taken; a wasm host is slower by
/// about 1.5 to 2).
pub const UNITS_PER_SECOND: f64 = 1.5e7;

mod work {
    //! The work counter of the seed loop (thread local: the conversion is single threaded).
    use std::cell::Cell;
    thread_local! {
        static SPENT: Cell<u64> = const { Cell::new(0) };
        static LIMIT: Cell<u64> = const { Cell::new(u64::MAX) };
        static CLOCK: Cell<Option<(f64, f64)>> = const { Cell::new(None) };
        static OVER: Cell<bool> = const { Cell::new(false) };
    }
    pub fn start(seconds: f64) {
        SPENT.with(|c| c.set(0));
        OVER.with(|c| c.set(false));
        let on = seconds > 0.0 && seconds.is_finite();
        LIMIT.with(|c| c.set(if on { (seconds * super::UNITS_PER_SECOND) as u64 } else { u64::MAX }));
        CLOCK.with(|c| c.set(if on { ck_core::budget::mark().map(|m| (m, seconds * 1000.0)) } else { None }));
    }
    pub fn spend(n: usize) {
        SPENT.with(|c| c.set(c.get().saturating_add(n as u64)));
    }
    pub fn spent() -> u64 {
        SPENT.with(|c| c.get())
    }
    /// Has the work budget (units, the clock's seconds, the regen's time budget) run out? Latched.
    pub fn over() -> bool {
        if OVER.with(|c| c.get()) {
            return true;
        }
        let o = SPENT.with(|c| c.get()) >= LIMIT.with(|c| c.get())
            || CLOCK.with(|c| c.get()).is_some_and(|(m, ms)| ck_core::budget::since(Some(m)) >= ms)
            || ck_core::budget::poll();
        if o {
            OVER.with(|c| c.set(true));
        }
        o
    }
}

impl Default for FitParams {
    fn default() -> Self {
        FitParams { tol: None, fit_tol: None, smooth_deg: 40.0, min_triangles: 4, max_regions: 200_000, max_seeds: 4_000, max_seconds: 3.0 }
    }
}

/// One fitted face.
#[derive(Debug, Clone)]
pub struct Fitted {
    pub kind: &'static str,
    pub face: String,
    pub triangles: usize,
    /// Axis (cylinder, cone) and a point on it (cone: the apex; sphere: the centre).
    pub axis: Vec3,
    pub origin: Vec3,
    pub radius: f64,
    pub half_angle: f64,
    pub max_residual: f64,
    pub rms_residual: f64,
}

#[derive(Debug, Clone, Default)]
pub struct AnalyticReport {
    pub planes: usize,
    pub cylinders: usize,
    pub cones: usize,
    pub spheres: usize,
    /// Planar faces that are facets of a curved surface (they meet a neighbour at a smooth angle) left unfitted.
    pub faceted: usize,
    pub tol: f64,
    pub fit_tol: f64,
    pub fitted: Vec<Fitted>,
    /// Faces fitted but sent back to facets, and why.
    pub declined: Vec<(&'static str, String)>,
    /// Whether the analytic body was kept (else the faceted one is the result), and why not.
    pub applied: bool,
    pub reason: Option<String>,
    pub rounds: usize,
    /// The work budget ([`FitParams::work_units`], the host clock budget) ran out: the seeds not yet tried stay faceted.
    pub budget_exhausted: bool,
    /// Share of the triangles that sit in a fitted (curved) face, 0..1.
    pub fitted_fraction: f64,
    /// Work units spent by the seed loop.
    pub work_units: u64,
}

pub fn report_json(a: &AnalyticReport) -> Json {
    let fitted: Vec<Json> = a
        .fitted
        .iter()
        .map(|f| {
            let v3 = |v: Vec3| Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)]);
            let mut o = Obj::new().put("kind", f.kind).put("face", f.face.as_str()).put("triangles", f.triangles as f64).put("origin", v3(f.origin));
            if f.kind != "sphere" {
                o = o.put("axis", v3(f.axis));
            }
            if f.kind == "cone" {
                o = o.put("halfAngle", f.half_angle);
            } else {
                o = o.put("radius", f.radius);
            }
            o.put("maxResidual", f.max_residual).put("rmsResidual", f.rms_residual).build()
        })
        .collect();
    let declined: Vec<Json> = a.declined.iter().map(|(why, what)| Obj::new().put("reason", *why).put("detail", what.as_str()).build()).collect();
    let mut o = Obj::new()
        .put("applied", a.applied)
        .put("planes", a.planes as f64)
        .put("cylinders", a.cylinders as f64)
        .put("cones", a.cones as f64)
        .put("spheres", a.spheres as f64)
        .put("faceted", a.faceted as f64)
        .put("tol", a.tol)
        .put("fitTol", a.fit_tol)
        .put("rounds", a.rounds as f64)
        .put("budgetExhausted", a.budget_exhausted)
        .put("fittedFraction", a.fitted_fraction)
        .put("workUnits", a.work_units as f64);
    if let Some(r) = &a.reason {
        o = o.put("reason", r.as_str());
    }
    o.put("fitted", Json::Arr(fitted)).put("declined", Json::Arr(declined)).build()
}

// ------------------------------------------------------------------ primitives and fitting

#[derive(Debug, Clone, Copy)]
pub enum Prim {
    Cyl { o: Vec3, a: Vec3, r: f64 },
    /// Radius grows along `a` from the apex: r(h) = h tan(ang).
    Cone { apex: Vec3, a: Vec3, ang: f64 },
    Sphere { c: Vec3, r: f64 },
}

impl Prim {
    fn name(&self) -> &'static str {
        match self {
            Prim::Cyl { .. } => "cylinder",
            Prim::Cone { .. } => "cone",
            Prim::Sphere { .. } => "sphere",
        }
    }
    /// Signed distance (positive outside / away from the axis).
    fn sdist(&self, p: Vec3) -> f64 {
        match *self {
            Prim::Cyl { o, a, r } => (p - o).reject(a).len() - r,
            Prim::Cone { apex, a, ang } => {
                let q = p - apex;
                let h = q.dot(a);
                let rho = q.reject(a).len();
                rho * ang.cos() - h * ang.sin()
            }
            Prim::Sphere { c, r } => p.dist(c) - r,
        }
    }
    fn dist(&self, p: Vec3) -> f64 {
        self.sdist(p).abs()
    }
    /// Unit normal pointing away from the axis / centre.
    fn normal(&self, p: Vec3) -> Vec3 {
        match *self {
            Prim::Cyl { o, a, .. } => (p - o).reject(a).try_normalize().unwrap_or(a.any_perp().normalize()),
            Prim::Cone { apex, a, ang } => {
                let rad = (p - apex).reject(a).try_normalize().unwrap_or(a.any_perp().normalize());
                rad * ang.cos() - a * ang.sin()
            }
            Prim::Sphere { c, .. } => (p - c).try_normalize().unwrap_or(Vec3::Z),
        }
    }
    fn params(&self) -> Vec<f64> {
        match *self {
            Prim::Cyl { o, a, r } => vec![o.x, o.y, o.z, a.x, a.y, a.z, r],
            Prim::Cone { apex, a, ang } => vec![apex.x, apex.y, apex.z, a.x, a.y, a.z, ang],
            Prim::Sphere { c, r } => vec![c.x, c.y, c.z, r],
        }
    }
    fn with(&self, x: &[f64]) -> Prim {
        let v = |i: usize| Vec3::new(x[i], x[i + 1], x[i + 2]);
        match self {
            Prim::Cyl { .. } => Prim::Cyl { o: v(0), a: v(3).try_normalize().unwrap_or(Vec3::Z), r: x[6] },
            Prim::Cone { .. } => Prim::Cone { apex: v(0), a: v(3).try_normalize().unwrap_or(Vec3::Z), ang: x[6] },
            Prim::Sphere { .. } => Prim::Sphere { c: v(0), r: x[3] },
        }
    }
}

/// Eigen decomposition of a symmetric 3x3 matrix (Jacobi): eigenvalues ascending with their vectors.
fn eigen3(m: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
    let mut a = m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-30 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for k in 0..3 {
                let (vkp, vkq) = (v[k][p], v[k][q]);
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]));
    let vals = [a[idx[0]][idx[0]], a[idx[1]][idx[1]], a[idx[2]][idx[2]]];
    let vecs = [0, 1, 2].map(|k| Vec3::new(v[0][idx[k]], v[1][idx[k]], v[2][idx[k]]));
    (vals, vecs)
}

fn outer(acc: &mut [[f64; 3]; 3], v: Vec3, w: f64) {
    let a = [v.x, v.y, v.z];
    for i in 0..3 {
        for j in 0..3 {
            acc[i][j] += w * a[i] * a[j];
        }
    }
}

/// Solve the dense system `a x = b` (partial pivoting); None when singular.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-300 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            for k in c..n {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let s: f64 = (r + 1..n).map(|k| a[r][k] * x[k]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    Some(x)
}

/// Levenberg-Marquardt on the signed distances of `pts` (numeric Jacobian).
fn refine(p: Prim, pts: &[Vec3], scale: f64) -> Prim {
    let x = lm(p.params(), scale, &|x: &[f64]| {
        let q = p.with(x);
        pts.iter().map(|&s| q.sdist(s)).collect()
    });
    p.with(&x)
}

/// Levenberg-Marquardt minimisation of the squared residuals `res(x)` from `x` (numeric Jacobian).
fn lm(x0: Vec<f64>, scale: f64, res0: &dyn Fn(&[f64]) -> Vec<f64>) -> Vec<f64> {
    let res = |x: &[f64]| {
        let r = res0(x);
        work::spend(r.len());
        r
    };
    let mut x = x0;
    let n = x.len();
    let cost = |r: &[f64]| r.iter().map(|v| v * v).sum::<f64>();
    let mut r = res(&x);
    let mut c = cost(&r);
    let mut lambda = 1e-3;
    for _ in 0..100 {
        if work::over() {
            break;
        }
        let mut jt: Vec<Vec<f64>> = Vec::with_capacity(n);
        for k in 0..n {
            let h = 1e-7 * scale.max(x[k].abs()).max(1e-9);
            let mut xp = x.clone();
            xp[k] += h;
            let rp = res(&xp);
            jt.push(rp.iter().zip(&r).map(|(a, b)| (a - b) / h).collect());
        }
        let jtj: Vec<Vec<f64>> = (0..n).map(|i| (0..n).map(|j| jt[i].iter().zip(&jt[j]).map(|(a, b)| a * b).sum()).collect()).collect();
        let jtr: Vec<f64> = (0..n).map(|i| jt[i].iter().zip(&r).map(|(a, b)| a * b).sum()).collect();
        let mut improved = false;
        for _ in 0..8 {
            let mut a = jtj.clone();
            for i in 0..n {
                a[i][i] += lambda * (jtj[i][i] + 1e-12 * scale * scale);
            }
            let Some(dx) = solve(a, jtr.iter().map(|v| -v).collect()) else { lambda *= 10.0; continue };
            let xn: Vec<f64> = x.iter().zip(&dx).map(|(a, b)| a + b).collect();
            let rn = res(&xn);
            let cn = cost(&rn);
            if cn < c {
                let rel = (c - cn) / c.max(1e-300);
                x = xn;
                r = rn;
                c = cn;
                lambda = (lambda * 0.3).max(1e-12);
                improved = rel > 1e-10;
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    x
}

/// Facet data of a cluster: corner points, facet normals (unit, outward of the solid), facet areas.
struct Data<'a> {
    pts: &'a [Vec3],
    normals: &'a [(Vec3, f64)],
    scale: f64,
}

fn centroid(p: &[Vec3]) -> Vec3 {
    p.iter().fold(Vec3::ZERO, |a, &b| a + b) / p.len().max(1) as f64
}

fn fit_sphere(d: &Data) -> Option<Prim> {
    // |p|^2 = 2 c.p + k, centred for conditioning
    let m = centroid(d.pts);
    let mut a = vec![vec![0.0; 4]; 4];
    let mut b = vec![0.0; 4];
    for &p in d.pts {
        let q = p - m;
        let row = [2.0 * q.x, 2.0 * q.y, 2.0 * q.z, 1.0];
        let rhs = q.len2();
        for i in 0..4 {
            for j in 0..4 {
                a[i][j] += row[i] * row[j];
            }
            b[i] += row[i] * rhs;
        }
    }
    let x = solve(a, b)?;
    let c = Vec3::new(x[0], x[1], x[2]);
    let r2 = x[3] + c.len2();
    if !(r2 > 0.0) {
        return None;
    }
    Some(refine(Prim::Sphere { c: c + m, r: r2.sqrt() }, d.pts, d.scale))
}

/// Axis from the normals: the direction most perpendicular to all of them.
fn axis_from_normals(d: &Data) -> Vec3 {
    let mut m = [[0.0; 3]; 3];
    for &(n, w) in d.normals {
        outer(&mut m, n, w);
    }
    eigen3(m).1[0].normalize()
}

/// Circle through 2D points (Kasa): centre, radius.
fn circle2(p: &[(f64, f64)]) -> Option<((f64, f64), f64)> {
    let (mx, my) = (p.iter().map(|q| q.0).sum::<f64>() / p.len() as f64, p.iter().map(|q| q.1).sum::<f64>() / p.len() as f64);
    let mut a = vec![vec![0.0; 3]; 3];
    let mut b = vec![0.0; 3];
    for &(x, y) in p {
        let (u, v) = (x - mx, y - my);
        let row = [2.0 * u, 2.0 * v, 1.0];
        let rhs = u * u + v * v;
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] += row[i] * row[j];
            }
            b[i] += row[i] * rhs;
        }
    }
    let x = solve(a, b)?;
    let r2 = x[2] + x[0] * x[0] + x[1] * x[1];
    (r2 > 0.0).then(|| ((x[0] + mx, x[1] + my), r2.sqrt()))
}

fn fit_cylinder(d: &Data) -> Option<Prim> {
    let a = axis_from_normals(d);
    let (x, y) = { let x = a.any_perp().normalize(); (x, a.cross(x)) };
    let m = centroid(d.pts);
    let p2: Vec<(f64, f64)> = d.pts.iter().map(|&p| ((p - m).dot(x), (p - m).dot(y))).collect();
    let ((cx, cy), r) = circle2(&p2)?;
    let o = m + x * cx + y * cy;
    Some(refine(Prim::Cyl { o, a, r }, d.pts, d.scale))
}

/// Cone about axis `a` through the points: axis point from a circle fit across the axis, radius against height linear.
fn cone_about(d: &Data, a: Vec3) -> Option<Prim> {
    let (x, y) = { let x = a.any_perp().normalize(); (x, a.cross(x)) };
    let m = centroid(d.pts);
    let p2: Vec<(f64, f64)> = d.pts.iter().map(|&p| ((p - m).dot(x), (p - m).dot(y))).collect();
    let ((cx, cy), _) = circle2(&p2)?;
    let o = m + x * cx + y * cy;
    let rh: Vec<(f64, f64)> = d.pts.iter().map(|&p| { let q = p - o; (q.dot(a), q.reject(a).len()) }).collect();
    let n = rh.len() as f64;
    let (sh, sr) = (rh.iter().map(|x| x.0).sum::<f64>() / n, rh.iter().map(|x| x.1).sum::<f64>() / n);
    let (mut shh, mut shr) = (0.0, 0.0);
    for &(h, r) in &rh {
        shh += (h - sh) * (h - sh);
        shr += (h - sh) * (r - sr);
    }
    if shh <= 0.0 {
        return None;
    }
    let k = shr / shh;
    if k.abs() < 1e-9 {
        return None;
    }
    // radius zero at the apex; the axis points the way the radius grows
    let hz = sh - sr / k;
    let apex = o + a * hz;
    let (a, k) = if k < 0.0 { (-a, -k) } else { (a, k) };
    if !apex.is_finite() || apex.dist(m) > 100.0 * d.scale {
        return None;
    }
    Some(refine(Prim::Cone { apex, a, ang: k.atan() }, d.pts, d.scale))
}

fn fit_cone(d: &Data) -> Option<Prim> {
    // axis candidates: most perpendicular to the normals (good for slender cones) and least variance of the normals
    // (their tips lie on a circle about the axis)
    let mut cov = [[0.0; 3]; 3];
    let wsum: f64 = d.normals.iter().map(|x| x.1).sum();
    let nm = d.normals.iter().fold(Vec3::ZERO, |a, &(n, w)| a + n * w) / wsum.max(1e-300);
    for &(n, w) in d.normals {
        outer(&mut cov, n - nm, w);
    }
    let cands = [axis_from_normals(d), eigen3(cov).1[0].normalize()];
    let mut tries: Vec<Prim> = cands.iter().filter_map(|&a| cone_about(d, a)).collect();
    // from the best cylinder: axis point o, axis a, radius r0 at o and slope t (a cylinder is t = 0, so the fit can
    // move smoothly from a cylinder to a cone; a small patch of a slender cone looks like a tilted cylinder)
    if let Some(Prim::Cyl { o, a, r }) = fit_cylinder(d) {
        let res = |x: &[f64]| -> Vec<f64> {
            let o = Vec3::new(x[0], x[1], x[2]);
            let a = Vec3::new(x[3], x[4], x[5]).try_normalize().unwrap_or(Vec3::Z);
            let (r0, t) = (x[6], x[7]);
            let c = 1.0 / (1.0 + t * t).sqrt();
            d.pts.iter().map(|&p| { let q = p - o; let h = q.dot(a); (q.reject(a).len() - (r0 + h * t)) * c }).collect()
        };
        let m = centroid(d.pts);
        let o = o + a * (m - o).dot(a);
        let x = lm(vec![o.x, o.y, o.z, a.x, a.y, a.z, r, 0.0], d.scale, &res);
        let (o, a, r0, t) = (Vec3::new(x[0], x[1], x[2]), Vec3::new(x[3], x[4], x[5]).normalize(), x[6], x[7]);
        if t.abs() > 1e-9 {
            let (a, t) = if t < 0.0 { (-a, -t) } else { (a, t) };
            // r0 + h t = 0 at the apex (h along the original axis); with a flipped, radius grows along a
            let apex = o - a * (r0 / t);
            if apex.is_finite() {
                let c = Prim::Cone { apex, a, ang: t.atan() };
                tries.push(c);
                tries.push(refine(c, d.pts, d.scale));
            }
        }
    }
    let mut best: Option<(f64, Prim)> = None;
    for p in tries {
        {
            let e = d.pts.iter().map(|&q| p.dist(q)).fold(0.0, f64::max);
            if e.is_finite() && best.as_ref().is_none_or(|b| e < b.0) {
                best = Some((e, p));
            }
        }
    }
    best.map(|b| b.1)
}

fn plausible(p: &Prim, diag: f64) -> bool {
    match *p {
        Prim::Cyl { r, a, .. } => r.is_finite() && r > 0.0 && r < 2.0 * diag && a.is_finite(),
        Prim::Sphere { r, c } => r.is_finite() && r > 0.0 && r < 2.0 * diag && c.is_finite(),
        Prim::Cone { ang, apex, a } => ang.is_finite() && ang > 0.5f64.to_radians() && ang < 85f64.to_radians() && apex.is_finite() && a.is_finite(),
    }
}

// ------------------------------------------------------------------ segmentation

struct Seg<'a> {
    m: &'a Closed,
    rg: &'a Regions,
    /// Distinct corners of each region.
    verts: Vec<Vec<u32>>,
    /// Triangles of each region.
    tris: Vec<Vec<u32>>,
    /// Smooth neighbours of each region.
    smooth: Vec<Vec<u32>>,
    tol: f64,
    fit_tol: f64,
    diag: f64,
}

impl Seg<'_> {
    /// Fit data of a set of regions: their distinct corners (slivers' left out) and one normal per region weighted by its
    /// area, each list thinned evenly to at most 400 entries.
    fn data(&self, regs: &[u32]) -> (Vec<Vec3>, Vec<(Vec3, f64)>) {
        let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
        let mut pts = vec![];
        let mut nrm = vec![];
        for &r in regs {
            for &i in &self.verts[r as usize] {
                if seen.insert(i) {
                    pts.push(self.m.pts[i as usize]);
                }
            }
            nrm.push((self.rg.normal[r as usize], self.rg.area[r as usize]));
        }
        fn thin<T: Copy>(v: Vec<T>, cap: usize) -> Vec<T> {
            if v.len() <= cap {
                return v;
            }
            let step = v.len() as f64 / cap as f64;
            (0..cap).map(|i| v[(i as f64 * step) as usize]).collect()
        }
        (thin(pts, 400), thin(nrm, 400))
    }

    /// Does region `r` lie on `p` (corners within fit_tol, centroid within tol, normal agreeing in `sense`)?
    fn fits(&self, r: u32, p: &Prim, sense: f64) -> bool {
        self.fits_within(r, p, sense, self.fit_tol)
    }

    /// `fits` with corner tolerance `ctol`.
    fn fits_within(&self, r: u32, p: &Prim, sense: f64, ctol: f64) -> bool {
        let ru = r as usize;
        work::spend(self.verts[ru].len() + 1);
        if self.verts[ru].iter().any(|&i| p.dist(self.m.pts[i as usize]) > ctol) {
            return false;
        }
        let c = self.rg.point[ru];
        if p.dist(c) > self.tol {
            return false;
        }
        // the facet must face the same way as the surface; a thin facet near a pole may lean far, its corners decide
        let n = p.normal(c);
        self.rg.normal[ru].dot(n) * sense >= 0.1
    }

    /// Grow a cluster of primitive type `kind` from `seed`; None when it does not fit.
    /// Grow from several starting sets (the seed's direct neighbours, a wider neighbourhood, and the seed with each pair
    /// of its neighbours, for when one neighbour belongs to the next surface); the largest cluster wins.
    fn grow(&self, seed: u32, kind: usize, claimed: &[bool]) -> Option<(Prim, Vec<u32>, f64)> {
        let sa = self.rg.area[seed as usize];
        let similar = |q: u32| { let qa = self.rg.area[q as usize]; !claimed[q as usize] && qa <= 10.0 * sa && qa >= 0.1 * sa };
        let near: Vec<u32> = self.smooth[seed as usize].iter().copied().filter(|&q| similar(q)).collect();
        let mut inits: Vec<Vec<u32>> = vec![];
        inits.push(std::iter::once(seed).chain(near.iter().copied()).collect());
        // breadth first up to 24 regions
        let mut wide: Vec<u32> = vec![seed];
        let mut k = 0;
        while k < wide.len() && wide.len() < 24 {
            work::spend(wide.len());
            let r = wide[k];
            for &q in &self.smooth[r as usize] {
                if wide.len() < 24 && similar(q) && !wide.contains(&q) {
                    wide.push(q);
                }
            }
            k += 1;
        }
        inits.push(wide);
        if near.len() > 2 {
            let m = near.len().min(4);
            for i in 0..m {
                for j in i + 1..m {
                    inits.push(vec![seed, near[i], near[j]]);
                }
            }
        }
        let mut best: Option<(Prim, Vec<u32>, f64)> = None;
        for (i, init) in inits.into_iter().enumerate() {
            // the pairs only when the neighbourhoods gave nothing (they are for a seed next to another surface)
            if i >= 2 && best.is_some() {
                break;
            }
            if let Some(g) = self.grow_from(seed, kind, claimed, init) {
                if best.as_ref().is_none_or(|b| g.1.len() > b.1.len()) {
                    best = Some(g);
                }
            }
        }
        best
    }

    fn grow_from(&self, seed: u32, kind: usize, claimed: &[bool], init: Vec<u32>) -> Option<(Prim, Vec<u32>, f64)> {
        // the starting set is fitted, trimmed to what fits (connected to the seed), refit and trimmed again: a
        // neighbourhood that crosses into another surface settles on the seed's
        let mut set = init;
        if set.len() < 2 {
            return None;
        }
        let mut p: Option<Prim> = None;
        let mut sense = 1.0;
        for _ in 0..5 {
            if work::over() {
                return None;
            }
            let (pts, nrm) = self.data(&set);
            let d = Data { pts: &pts, normals: &nrm, scale: self.diag };
            let q = match (kind, p) {
                (_, Some(prev)) => refine(prev, d.pts, self.diag),
                (0, None) => fit_cylinder(&d)?,
                (1, None) => fit_cone(&d)?,
                _ => fit_sphere(&d)?,
            };
            if !plausible(&q, self.diag) {
                return None;
            }
            sense = {
                let s: f64 = set.iter().map(|&r| self.rg.normal[r as usize].dot(q.normal(self.rg.point[r as usize]))).sum();
                if s >= 0.0 { 1.0 } else { -1.0 }
            };
            let n0 = set.len();
            let keep: Vec<u32> = set.iter().copied().filter(|&r| self.fits(r, &q, sense)).collect();
            if !keep.contains(&seed) {
                return None;
            }
            // connected to the seed through what is kept
            let mut conn = vec![seed];
            let mut i = 0;
            while i < conn.len() {
                let r = conn[i];
                work::spend(keep.len() + conn.len());
                for &q2 in &self.smooth[r as usize] {
                    if keep.contains(&q2) && !conn.contains(&q2) {
                        conn.push(q2);
                    }
                }
                i += 1;
            }
            set = conn;
            p = Some(q);
            if set.len() == n0 {
                break;
            }
            if set.len() < 2 {
                return None;
            }
        }
        let mut p = p?;
        let mut inset: HashMap<u32, ()> = set.iter().map(|&r| (r, ())).collect();
        for _round in 0..40 {
            if work::over() {
                return None;
            }
            let before = set.len();
            let mut k = 0;
            while k < set.len() {
                let r = set[k];
                for &q in &self.smooth[r as usize] {
                    if claimed[q as usize] || inset.contains_key(&q) {
                        continue;
                    }
                    if self.fits(q, &p, sense) {
                        inset.insert(q, ());
                        set.push(q);
                    }
                }
                k += 1;
            }
            let (pts, nrm) = self.data(&set);
            let d = Data { pts: &pts, normals: &nrm, scale: self.diag };
            let q = refine(p, d.pts, self.diag);
            if !plausible(&q, self.diag) {
                return None;
            }
            p = q;
            let n0 = set.len();
            set.retain(|&r| self.fits(r, &p, sense));
            if !set.contains(&seed) {
                return None;
            }
            if set.len() != n0 {
                inset = set.iter().map(|&r| (r, ())).collect();
            }
            if set.len() == before {
                break;
            }
        }
        Some((p, set, sense))
    }
}

// ------------------------------------------------------------------ the face graph

#[derive(Debug, Clone)]
enum Geo {
    Plane { n: Vec3, p: Vec3 },
    Prim { p: Prim, sense: f64 },
}

#[derive(Debug, Clone)]
struct FaceIn {
    geo: Geo,
    /// The planar regions it was made of.
    regions: Vec<u32>,
    comp: u32,
    triangles: usize,
}

/// The exact curve of a run.
#[derive(Debug, Clone)]
enum Cv {
    /// Straight segments between the kept vertices of the run.
    Poly(Vec<u32>),
    /// One line from the first to the last vertex, on `o + d t`.
    Line { o: Vec3, d: Vec3 },
    /// A circle (centre, unit normal, radius); open runs are arcs.
    Circle { c: Vec3, n: Vec3, r: f64 },
}

#[derive(Debug, Clone)]
struct Run {
    /// Mesh vertices along the run, in the direction face `a` walks it.
    verts: Vec<u32>,
    a: usize,
    b: usize,
    closed: bool,
    cv: Option<Cv>,
}

/// A loop of a face as runs: (run, forward).
type LoopRuns = Vec<(usize, bool)>;

struct Graph {
    runs: Vec<Run>,
    loops: Vec<Vec<LoopRuns>>,
}

fn build_graph(m: &Closed, face_of_tri: &[u32], nf: usize, twin: &[usize]) -> Graph {
    let tris = &m.tris;
    let raw = region_loops(tris, twin, face_of_tri, nf);
    let mut runs: Vec<Run> = vec![];
    let mut run_of: HashMap<usize, usize> = HashMap::new(); // half-edge -> its run
    let mut loops: Vec<Vec<LoopRuns>> = vec![vec![]; nf];
    for f in 0..nf {
        for lp in &raw[f] {
            let n = lp.len();
            let nb: Vec<usize> = lp.iter().map(|&h| face_of_tri[twin[h] / 3] as usize).collect();
            let splits: Vec<usize> = (0..n).filter(|&k| nb[k] != nb[(k + n - 1) % n]).collect();
            let mut out: LoopRuns = vec![];
            let segs: Vec<(usize, usize)> = if splits.is_empty() {
                vec![(0, n)]
            } else {
                (0..splits.len()).map(|i| { let s = splits[i]; let e = splits[(i + 1) % splits.len()]; (s, if e > s { e - s } else { e + n - s }) }).collect()
            };
            for (s, len) in segs {
                let hs: Vec<usize> = (0..len).map(|k| lp[(s + k) % n]).collect();
                let closed = splits.is_empty();
                // the run seen from the other side (a closed run may start anywhere along it)
                let last_twin = twin[*hs.last().unwrap()];
                if let Some(&r) = run_of.get(&last_twin) {
                    out.push((r, false));
                    continue;
                }
                let mut verts: Vec<u32> = hs.iter().map(|&h| he_from(tris, h)).collect();
                if !closed {
                    verts.push(he_to(tris, *hs.last().unwrap()));
                }
                let r = runs.len();
                runs.push(Run { verts, a: f, b: nb[s], closed, cv: None });
                for &h in &hs {
                    run_of.insert(h, r);
                }
                out.push((r, true));
            }
            loops[f].push(out);
        }
    }
    Graph { runs, loops }
}

/// Douglas-Peucker: indices of the kept points of an open polyline (ends kept).
fn simplify(p: &[Vec3], tol: f64) -> Vec<usize> {
    let mut keep = vec![false; p.len()];
    keep[0] = true;
    keep[p.len() - 1] = true;
    let mut stack = vec![(0usize, p.len() - 1)];
    while let Some((i, j)) = stack.pop() {
        if j <= i + 1 {
            continue;
        }
        let d = p[j] - p[i];
        let l = d.len();
        let (mut far, mut fd) = (i, 0.0);
        for k in i + 1..j {
            let x = if l > 0.0 { (p[k] - p[i]).reject(d / l).len() } else { p[k].dist(p[i]) };
            if x > fd {
                fd = x;
                far = k;
            }
        }
        if fd > tol {
            keep[far] = true;
            stack.push((i, far));
            stack.push((far, j));
        }
    }
    (0..p.len()).filter(|&k| keep[k]).collect()
}

/// Plane through points: (centroid, unit normal, largest distance).
fn plane_of(p: &[Vec3]) -> (Vec3, Vec3, f64) {
    let c = centroid(p);
    let mut m = [[0.0; 3]; 3];
    for &q in p {
        outer(&mut m, q - c, 1.0);
    }
    let n = eigen3(m).1[0].normalize();
    let d = p.iter().map(|&q| (q - c).dot(n).abs()).fold(0.0, f64::max);
    (c, n, d)
}

/// The circle of a run on the analytic face `p` (exact on it), checked against the run's points.
fn circle_on(p: &Prim, pts: &[Vec3], ftol: f64, itol: f64, closed: bool) -> Option<(Vec3, Vec3, f64)> {
    if pts.len() < 3 {
        return None;
    }
    let (cen, n, off) = plane_of(pts);
    if off > itol {
        return None;
    }
    let (c, n, r) = match *p {
        Prim::Cyl { o, a, r } => {
            if a.cross(n).len() * r > ftol {
                return None;
            }
            (o + a * (cen - o).dot(a), a, r)
        }
        Prim::Cone { apex, a, ang } => {
            let rr = pts.iter().map(|&q| (q - apex).reject(a).len()).sum::<f64>() / pts.len() as f64;
            if a.cross(n).len() * rr > ftol {
                return None;
            }
            let h = (cen - apex).dot(a);
            (apex + a * h, a, h * ang.tan())
        }
        Prim::Sphere { c, r } => {
            let d = (cen - c).dot(n);
            if d.abs() >= r {
                return None;
            }
            (c + n * d, n, (r * r - d * d).sqrt())
        }
    };
    let d = |q: Vec3| { let v = q - c; (v.reject(n).len() - r).abs().max(v.dot(n).abs()) };
    let ends = if closed { itol } else { ftol };
    (r > 0.0 && run_ok(pts, ends, itol, &d)).then_some((c, n, r))
}

/// Points of a run: the ends (they become vertices) within `ftol`, the points between within `itol`.
fn run_ok(pts: &[Vec3], ftol: f64, itol: f64, d: &dyn Fn(Vec3) -> f64) -> bool {
    let n = pts.len();
    pts.iter().enumerate().all(|(k, &q)| d(q) <= if k == 0 || k + 1 == n { ftol } else { itol })
}

/// The line of a run on `p` (a ruling of a cylinder, a generator of a cone).
fn line_on(p: &Prim, pts: &[Vec3], ftol: f64, itol: f64) -> Option<(Vec3, Vec3)> {
    let (s, e) = (pts[0], *pts.last().unwrap());
    let l = s.dist(e);
    if l <= ftol {
        return None;
    }
    let (o, d) = match *p {
        Prim::Cyl { o, a, r } => {
            let rad = (s - o).reject(a).try_normalize()?;
            let foot = o + a * (s - o).dot(a) + rad * r;
            let d = if (e - s).dot(a) >= 0.0 { a } else { -a };
            (foot, d)
        }
        Prim::Cone { apex, .. } => {
            let far = if s.dist(apex) >= e.dist(apex) { s } else { e };
            let g = (far - apex).try_normalize()?;
            let d = if (e - s).dot(g) >= 0.0 { g } else { -g };
            (apex + g * (s - apex).dot(g), d)
        }
        Prim::Sphere { .. } => return None,
    };
    (run_ok(pts, ftol, itol, &|q: Vec3| (q - o).reject(d).len())).then_some((o, d))
}

fn surf_dist(g: &Geo, q: Vec3) -> f64 {
    match g {
        Geo::Plane { n, p } => (q - *p).dot(*n).abs(),
        Geo::Prim { p, .. } => p.dist(q),
    }
}

/// Geometry of every run; Err(face) names an analytic face whose boundary is not exact.
/// `ftol`: run ends (vertices) and the other face's fit; `itol`: mesh points along a run (the chord sag).
fn run_curves(g: &mut Graph, faces: &[FaceIn], m: &Closed, ftol: f64, ptol: f64, itol: f64) -> Result<(), Vec<usize>> {
    let mut bad: Vec<usize> = vec![];
    for r in 0..g.runs.len() {
        let run = &g.runs[r];
        let pts: Vec<Vec3> = run.verts.iter().map(|&i| m.pts[i as usize]).collect();
        let prims: Vec<(usize, Prim)> = [run.a, run.b].iter().filter_map(|&f| match faces[f].geo { Geo::Prim { p, .. } => Some((f, p)), _ => None }).collect();
        let cv = if prims.is_empty() {
            if run.closed {
                // two planes meet in a line: a closed run between planes is a polygon; keep its corners
                let mut open = pts.clone();
                open.push(pts[0]);
                let k = simplify(&open, ptol);
                Some(Cv::Poly(k[..k.len() - 1].iter().map(|&i| run.verts[i]).collect()))
            } else {
                Some(Cv::Poly(simplify(&pts, ptol).into_iter().map(|i| run.verts[i]).collect()))
            }
        } else {
            let (f0, p0) = prims[0];
            let mut c = None;
            if !run.closed {
                if let Some((o, d)) = line_on(&p0, &pts, ftol, itol) {
                    c = Some(Cv::Line { o, d });
                }
            }
            if c.is_none() {
                if let Some((cc, n, rr)) = circle_on(&p0, &pts, ftol, itol, run.closed) {
                    c = Some(Cv::Circle { c: cc, n, r: rr });
                }
            }
            // the other face must hold the curve too
            if let Some(cv) = &c {
                let other = if run.a == f0 { run.b } else { run.a };
                let samples: Vec<Vec3> = match cv {
                    Cv::Line { o, d } => {
                        let t1 = (pts[pts.len() - 1] - *o).dot(*d);
                        let t0 = (pts[0] - *o).dot(*d);
                        (0..=8).map(|k| *o + *d * (t0 + (t1 - t0) * k as f64 / 8.0)).collect()
                    }
                    Cv::Circle { c, n, r } => {
                        let x = n.any_perp().normalize();
                        let y = n.cross(x);
                        (0..16).map(|k| { let t = TAU * k as f64 / 16.0; *c + (x * t.cos() + y * t.sin()) * *r }).collect()
                    }
                    Cv::Poly(_) => vec![],
                };
                let dev = samples.iter().map(|&q| surf_dist(&faces[other].geo, q)).fold(0.0, f64::max);
                let lim = match faces[other].geo { Geo::Plane { .. } => ptol.max(ftol), _ => ftol };
                // a circle only on the part of the plane the run covers: an arc's far side is not checked against a plane
                let arc_ok = matches!(cv, Cv::Circle { .. }) && !run.closed && matches!(faces[other].geo, Geo::Plane { .. }) && {
                    let (_, n_run, off) = plane_of(&pts);
                    off <= ptol && match &faces[other].geo { Geo::Plane { n, .. } => n.cross(n_run).len() < 1e-3, _ => false }
                };
                if dev > lim && !arc_ok {
                    c = None;
                }
            }
            if c.is_none() {
                if std::env::var("CK_MESH_DEBUG").is_ok() {
                    let (_, n, off) = plane_of(&pts);
                    let rad: Vec<f64> = pts.iter().map(|&q| p0.dist(q)).collect();
                    let mx = rad.iter().cloned().fold(0.0, f64::max);
                    eprintln!("run {r} faces {}|{} closed {} pts {} plane off {off:.2e} n {n:?} max surf dev {mx:.2e} first/last dev {:.2e} {:.2e}", run.a, run.b, run.closed, pts.len(), rad[0], rad[rad.len() - 1]);
                }
                for &(f, _) in &prims {
                    bad.push(f);
                }
            }
            c
        };
        g.runs[r].cv = cv;
    }
    if bad.is_empty() {
        Ok(())
    } else {
        bad.sort_unstable();
        bad.dedup();
        Err(bad)
    }
}

// ------------------------------------------------------------------ building the body

/// Angle of `p` about frame `fr`'s z axis.
fn theta(fr: &Frame, p: Vec3) -> f64 {
    let d = p - fr.o;
    d.dot(fr.y).atan2(d.dot(fr.x))
}

fn wrap(a: f64) -> f64 {
    let mut x = a % TAU;
    if x > PI {
        x -= TAU;
    } else if x <= -PI {
        x += TAU;
    }
    x
}

/// One edge to make: from vertex key `v0` to `v1` along a curve, plus the run it belongs to.
#[derive(Debug, Clone)]
struct EdgeIn {
    v0: usize,
    v1: usize,
    curve: Curve,
    t0: f64,
    t1: f64,
}

struct Builder<'a> {
    m: &'a Closed,
    faces: &'a [FaceIn],
    /// Vertex positions (a mesh vertex keeps its position; made ones are exact curve points).
    vpos: Vec<Vec3>,
    vkey: HashMap<u32, usize>,
    edges: Vec<EdgeIn>,
    /// Per run: its edges in run order (forward = the run's direction).
    run_edges: Vec<Vec<usize>>,
    /// Per face loop: coedges (edge, reversed).
    face_loops: Vec<Vec<Vec<(usize, bool)>>>,
    frames: Vec<Option<Frame>>,
    /// Centroid of every planar region (which pole a sphere cap holds).
    region_points: Vec<Vec3>,
    /// Vertices a seam ends on: they must not move.
    fixed: std::collections::HashSet<usize>,
}

impl Builder<'_> {
    fn mv(&mut self, i: u32) -> usize {
        if let Some(&k) = self.vkey.get(&i) {
            return k;
        }
        let k = self.vpos.len();
        self.vpos.push(self.m.pts[i as usize]);
        self.vkey.insert(i, k);
        k
    }
    fn new_v(&mut self, p: Vec3) -> usize {
        self.vpos.push(p);
        self.vpos.len() - 1
    }
    fn add_edge(&mut self, e: EdgeIn) -> usize {
        self.edges.push(e);
        self.edges.len() - 1
    }
}

/// Circle frame with x towards `start`, turning the way `p_next` lies (so the arc runs with increasing t).
fn circle_frame(c: Vec3, n: Vec3, start: Vec3, dir_pts: &[Vec3]) -> Frame {
    let x = (start - c).reject(n).try_normalize().unwrap_or(n.any_perp().normalize());
    let mut z = n;
    // signed sweep along the points about n
    let mut s = 0.0;
    for w in dir_pts.windows(2) {
        let (a, b) = ((w[0] - c).reject(n), (w[1] - c).reject(n));
        s += a.cross(b).dot(n).atan2(a.dot(b));
    }
    if s < 0.0 {
        z = -n;
    }
    Frame { o: c, x, y: z.cross(x), z }
}

fn param_on_circle(fr: &Frame, p: Vec3) -> f64 {
    let d = p - fr.o;
    let t = d.dot(fr.y).atan2(d.dot(fr.x));
    if t < 0.0 { t + TAU } else { t }
}

/// Edges of every run (the seams come later).
fn make_run_edges(bd: &mut Builder, g: &Graph) {
    let nr = g.runs.len();
    bd.run_edges = vec![vec![]; nr];
    for r in 0..nr {
        let run = &g.runs[r];
        let pts: Vec<Vec3> = run.verts.iter().map(|&i| bd.m.pts[i as usize]).collect();
        match run.cv.as_ref().unwrap() {
            Cv::Poly(keep) => {
                let n = keep.len();
                let segs = if run.closed { n } else { n - 1 };
                for k in 0..segs {
                    let (a, b) = (keep[k], keep[(k + 1) % n]);
                    let (va, vb) = (bd.mv(a), bd.mv(b));
                    let (pa, pb) = (bd.vpos[va], bd.vpos[vb]);
                    let e = bd.add_edge(EdgeIn { v0: va, v1: vb, curve: Curve::line_through(pa, pb), t0: 0.0, t1: pa.dist(pb) });
                    bd.run_edges[r].push(e);
                }
            }
            Cv::Line { o, d } => {
                let (va, vb) = (bd.mv(run.verts[0]), bd.mv(*run.verts.last().unwrap()));
                let (t0, t1) = ((bd.vpos[va] - *o).dot(*d), (bd.vpos[vb] - *o).dot(*d));
                let e = bd.add_edge(EdgeIn { v0: va, v1: vb, curve: Curve::Line { o: *o, d: *d }, t0, t1 });
                bd.run_edges[r].push(e);
            }
            Cv::Circle { c, n, r: rad } => {
                if run.closed {
                    // one vertex, placed later by the seam pass if a periodic face needs it elsewhere
                    let fr = circle_frame(*c, *n, pts[0], &pts.iter().copied().chain(std::iter::once(pts[0])).collect::<Vec<_>>());
                    let v = bd.new_v(fr.o + fr.x * *rad);
                    let e = bd.add_edge(EdgeIn { v0: v, v1: v, curve: Curve::Circle { frame: fr, r: *rad }, t0: 0.0, t1: TAU });
                    bd.run_edges[r].push(e);
                } else {
                    let fr = circle_frame(*c, *n, pts[0], &pts);
                    let (va, vb) = (bd.mv(run.verts[0]), bd.mv(*run.verts.last().unwrap()));
                    let t1 = param_on_circle(&fr, bd.vpos[vb]);
                    let t1 = if t1 < 1e-9 { TAU } else { t1 };
                    let e = bd.add_edge(EdgeIn { v0: va, v1: vb, curve: Curve::Circle { frame: fr, r: *rad }, t0: 0.0, t1 });
                    bd.run_edges[r].push(e);
                }
            }
        }
    }
}

/// Split circular edge `e` at the point of angle `th` about the face frame; returns the new vertex.
fn split_circle_at(bd: &mut Builder, run: usize, e: usize, p: Vec3) -> usize {
    let ed = bd.edges[e].clone();
    let Curve::Circle { frame, r } = ed.curve else { unreachable!() };
    let t = param_on_circle(&frame, p);
    let q = frame.o + (frame.x * t.cos() + frame.y * t.sin()) * r;
    if ed.v0 == ed.v1 && ed.t1 - ed.t0 >= TAU - 1e-12 && !bd.fixed.contains(&ed.v0) {
        // a full circle whose vertex no seam uses yet: move its single vertex there
        let fr = Frame { o: frame.o, x: (q - frame.o) / r, y: frame.z.cross((q - frame.o) / r), z: frame.z };
        let v = ed.v0;
        bd.vpos[v] = q;
        bd.edges[e].curve = Curve::Circle { frame: fr, r };
        return v;
    }
    let tt = if t < ed.t0 { t + TAU } else { t };
    let v = bd.new_v(q);
    let second = EdgeIn { v0: v, v1: ed.v1, curve: ed.curve.clone(), t0: tt, t1: ed.t1 };
    bd.edges[e].v1 = v;
    bd.edges[e].t1 = tt;
    let ne = bd.add_edge(second);
    let pos = bd.run_edges[run].iter().position(|&x| x == e).unwrap();
    bd.run_edges[run].insert(pos + 1, ne);
    v
}

/// The coedges of a loop of runs: (edge, reversed) in walking order.
fn loop_coedges(bd: &Builder, lr: &LoopRuns) -> Vec<(usize, bool)> {
    let mut out = vec![];
    for &(r, fwd) in lr {
        let es = &bd.run_edges[r];
        if fwd {
            out.extend(es.iter().map(|&e| (e, false)));
        } else {
            out.extend(es.iter().rev().map(|&e| (e, true)));
        }
    }
    out
}

fn co_start(bd: &Builder, c: (usize, bool)) -> usize {
    if c.1 { bd.edges[c.0].v1 } else { bd.edges[c.0].v0 }
}
fn eval_edge(e: &EdgeIn, t: f64) -> Vec3 {
    e.curve.eval(t)
}

/// Turns of a loop about the face axis (in units of 2 pi), from the coedge geometry.
fn winding(bd: &Builder, fr: &Frame, lp: &[(usize, bool)]) -> f64 {
    let mut s = 0.0;
    for &c in lp {
        let e = &bd.edges[c.0];
        let n = 16;
        let mut prev = None;
        for k in 0..=n {
            let t = e.t0 + (e.t1 - e.t0) * k as f64 / n as f64;
            let q = eval_edge(e, t);
            if (q - fr.o).reject(fr.z).len() < 1e-9 * (1.0 + q.len()) {
                continue;
            }
            let th = theta(fr, q);
            if let Some(p) = prev {
                let d = wrap(th - p);
                s += if c.1 { -d } else { d };
            }
            prev = Some(th);
        }
    }
    s / TAU
}

struct SeamPlan {
    face: usize,
    rings: Vec<usize>,
    seam_vs: Vec<usize>,
    poles: Vec<Vec3>,
}

/// Seams, poles and frames of the periodic faces; Err(faces) when a face's loops are not a shape we can parametrise.
fn periodic_faces(bd: &mut Builder, g: &Graph) -> Result<(), Vec<usize>> {
    let nf = bd.faces.len();
    bd.frames = vec![None; nf];
    let mut bad = vec![];
    let mut plans: Vec<SeamPlan> = vec![];
    for f in 0..nf {
        let Geo::Prim { p, .. } = bd.faces[f].geo else { continue };
        let loops: Vec<Vec<(usize, bool)>> = g.loops[f].iter().map(|lr| loop_coedges(bd, lr)).collect();
        // the face's axis frame
        let (o, z) = match p {
            Prim::Cyl { o, a, .. } => (o, a),
            Prim::Cone { apex, a, .. } => (apex, a),
            Prim::Sphere { c, .. } => {
                // the axis of its circles (they must share it)
                let mut ax: Option<Vec3> = None;
                let mut ok = true;
                for lp in &loops {
                    for &(e, _) in lp {
                        if let Curve::Circle { frame, .. } = &bd.edges[e].curve {
                            match ax {
                                None => ax = Some(frame.z),
                                Some(a) => ok &= a.cross(frame.z).len() < 1e-6,
                            }
                        }
                    }
                }
                if !ok {
                    bad.push(f);
                    continue;
                }
                (c, ax.unwrap_or(Vec3::Z))
            }
        };
        // a cone's frame sits at the mean height of its boundary (radius > 0 there), on the axis
        let o = match p {
            Prim::Cone { apex, a, .. } => {
                let hs: Vec<f64> = loops.iter().flatten().map(|&c| (bd.vpos[co_start(bd, c)] - apex).dot(a)).collect();
                let h = hs.iter().sum::<f64>() / hs.len().max(1) as f64;
                apex + a * h.max(0.0)
            }
            _ => o,
        };
        let x0 = z.any_perp().normalize();
        let mut fr = Frame { o, x: x0, y: z.cross(x0), z };
        let winds: Vec<f64> = loops.iter().map(|lp| winding(bd, &fr, lp)).collect();
        let rings: Vec<usize> = (0..winds.len()).filter(|&k| winds[k].abs() > 0.5).collect();
        let nonrings = winds.len() - rings.len();
        if (!rings.is_empty() && nonrings > 0) || rings.len() > 2 {
            bad.push(f);
            continue;
        }
        if rings.is_empty() && !winds.is_empty() {
            // a patch: the seam away from it
            let mut mean = Vec3::ZERO;
            for lp in &loops {
                for &c in lp {
                    mean = mean + (bd.vpos[co_start(bd, c)] - o).reject(z);
                }
            }
            let x = (-mean).try_normalize().unwrap_or(x0);
            fr = Frame { o, x, y: z.cross(x), z };
            bd.frames[f] = Some(fr);
            continue;
        }
        let poles: Vec<Vec3> = match (rings.len(), p) {
            (2, Prim::Cyl { .. }) | (2, Prim::Cone { .. }) | (2, Prim::Sphere { .. }) => vec![],
            (1, Prim::Cone { apex, .. }) => vec![apex],
            (1, Prim::Sphere { c, r }) => {
                // the pole on the side of the face: where its facets are
                let e = &bd.edges[loops[rings[0]][0].0];
                let ring_h = (eval_edge(e, e.t0) - c).dot(z);
                let regs = &bd.faces[f].regions;
                let hs: f64 = regs.iter().map(|&rg| (bd.region_points[rg as usize] - c).dot(z)).sum::<f64>() / regs.len() as f64;
                vec![if hs > ring_h { c + z * r } else { c - z * r }]
            }
            (0, Prim::Sphere { c, r }) => vec![c - z * r, c + z * r],
            _ => {
                bad.push(f);
                continue;
            }
        };
        // the seam angle: the first vertex of the first ring
        let th = match rings.first() {
            Some(&k) => theta(&fr, bd.vpos[co_start(bd, loops[k][0])]),
            None => 0.0,
        };
        let x = (fr.x * th.cos() + fr.y * th.sin()).normalize();
        fr = Frame { o, x, y: z.cross(x), z };
        bd.frames[f] = Some(fr);
        // every ring gets a vertex at angle 0 of the frame
        let r_at = |q: Vec3| (q - o).reject(z).len();
        let mut seam_vs: Vec<usize> = vec![];
        let mut failed = false;
        for &k in &rings {
            let lp = loop_coedges(bd, &g.loops[f][k]);
            let found = lp.iter().map(|&c| co_start(bd, c)).find(|&v| {
                let q = bd.vpos[v];
                wrap(theta(&fr, q)).abs() * r_at(q).max(1e-12) < 1e-9 * (1.0 + q.len())
            });
            let v = match found {
                Some(v) => v,
                None => {
                    // split the circle coedge that passes angle 0
                    let mut hit = None;
                    'runs: for &(run, _) in &g.loops[f][k] {
                        for &e in &bd.run_edges[run] {
                            let ed = &bd.edges[e];
                            if !matches!(ed.curve, Curve::Circle { .. }) {
                                continue;
                            }
                            if ed.v0 == ed.v1 {
                                hit = Some((run, e));
                                break 'runs;
                            }
                            let n = 64;
                            let mut prev = theta(&fr, eval_edge(ed, ed.t0));
                            for s in 1..=n {
                                let t = ed.t0 + (ed.t1 - ed.t0) * s as f64 / n as f64;
                                let cur = theta(&fr, eval_edge(ed, t));
                                if ((prev <= 0.0 && cur > 0.0) || (prev >= 0.0 && cur < 0.0)) && (cur - prev).abs() < PI {
                                    hit = Some((run, e));
                                    break 'runs;
                                }
                                prev = cur;
                            }
                        }
                    }
                    let Some((run, e)) = hit else {
                        failed = true;
                        break;
                    };
                    let q0 = eval_edge(&bd.edges[e], bd.edges[e].t0);
                    let q = o + z * (q0 - o).dot(z) + fr.x * r_at(q0);
                    split_circle_at(bd, run, e, q)
                }
            };
            bd.fixed.insert(v);
            seam_vs.push(v);
        }
        if failed {
            bad.push(f);
            continue;
        }
        plans.push(SeamPlan { face: f, rings, seam_vs, poles });
    }
    if !bad.is_empty() {
        return Err(bad);
    }
    // the loops with every split in place, then the seams
    bd.face_loops = (0..nf).map(|f| g.loops[f].iter().map(|lr| loop_coedges(bd, lr)).collect()).collect();
    for pl in plans {
        let f = pl.face;
        let Geo::Prim { p, sense } = bd.faces[f].geo else { continue };
        let fr = bd.frames[f].unwrap();
        let z = fr.z;
        let mut ends: Vec<usize> = pl.seam_vs.clone();
        for &q in &pl.poles {
            let v = bd.new_v(q);
            ends.push(v);
        }
        let seam_curve = |a: Vec3, b: Vec3| -> (Curve, f64, f64) {
            match p {
                Prim::Sphere { c, r } => {
                    // meridian in the plane of x and z: point(t) = c + r (cos t x + sin t z), t = latitude
                    let mf = Frame { o: c, x: fr.x, y: z, z: fr.x.cross(z) };
                    let lat = |q: Vec3| { let d = q - c; d.dot(z).atan2(d.dot(fr.x)) };
                    (Curve::Circle { frame: mf, r }, lat(a), lat(b))
                }
                _ => {
                    let d = (b - a).normalize();
                    (Curve::Line { o: a, d }, 0.0, a.dist(b))
                }
            }
        };
        let (va, vb) = (ends[0], ends[1]);
        let (cv, t0, t1) = seam_curve(bd.vpos[va], bd.vpos[vb]);
        let (va, vb, cv, t0, t1) = if t1 > t0 {
            (va, vb, cv, t0, t1)
        } else {
            let (cv2, s0, s1) = seam_curve(bd.vpos[vb], bd.vpos[va]);
            (vb, va, cv2, s0, s1)
        };
        let seam = bd.add_edge(EdgeIn { v0: va, v1: vb, curve: cv, t0, t1 });
        let rot = |bd: &Builder, lp: &Vec<(usize, bool)>, v: usize| -> Vec<(usize, bool)> {
            let k = lp.iter().position(|&c| co_start(bd, c) == v).unwrap_or(0);
            lp[k..].iter().chain(lp[..k].iter()).copied().collect()
        };
        let lp: Vec<(usize, bool)> = match pl.rings.len() {
            2 => {
                let l1 = rot(bd, &bd.face_loops[f][pl.rings[0]], pl.seam_vs[0]);
                let l2 = rot(bd, &bd.face_loops[f][pl.rings[1]], pl.seam_vs[1]);
                // l1 ends at seam_vs[0]; the seam runs on to seam_vs[1]
                let up = (seam, bd.edges[seam].v0 != pl.seam_vs[0]);
                let down = (seam, !up.1);
                l1.into_iter().chain(std::iter::once(up)).chain(l2).chain(std::iter::once(down)).collect()
            }
            1 => {
                let l1 = rot(bd, &bd.face_loops[f][pl.rings[0]], pl.seam_vs[0]);
                let out = (seam, bd.edges[seam].v0 != pl.seam_vs[0]);
                l1.into_iter().chain([out, (seam, !out.1)]).collect()
            }
            _ => {
                if sense > 0.0 { vec![(seam, false), (seam, true)] } else { vec![(seam, true), (seam, false)] }
            }
        };
        bd.face_loops[f] = vec![lp];
    }
    Ok(())
}

/// uv line of coedge `c` on the periodic face frame: (origin, direction) in the edge parameter.
fn pcurve_line(bd: &Builder, fr: &Frame, p: &Prim, c: (usize, bool)) -> Option<(Vec2, Vec2)> {
    let e = &bd.edges[c.0];
    let vof = |q: Vec3| -> f64 {
        match p {
            Prim::Sphere { c: cc, r } => ((q - *cc).dot(fr.z) / r).clamp(-1.0, 1.0).asin(),
            _ => (q - fr.o).dot(fr.z),
        }
    };
    match &e.curve {
        Curve::Circle { frame: cf, .. } if cf.z.cross(fr.z).len() < 1e-6 => {
            let sigma = if cf.z.dot(fr.z) > 0.0 { 1.0 } else { -1.0 };
            let phi = cf.x.dot(fr.y).atan2(cf.x.dot(fr.x));
            Some((Vec2::new(phi, vof(eval_edge(e, e.t0))), Vec2::new(sigma, 0.0)))
        }
        Curve::Circle { frame: cf, .. } => {
            // a meridian (sphere seam): its plane holds the axis, t is the latitude
            if !matches!(p, Prim::Sphere { .. }) || cf.z.dot(fr.z).abs() > 1e-6 {
                return None;
            }
            let u = theta(fr, cf.o + cf.x);
            let sgn = if cf.y.dot(fr.z) > 0.0 { 1.0 } else { -1.0 };
            Some((Vec2::new(u, 0.0), Vec2::new(0.0, sgn)))
        }
        Curve::Line { o, d } => {
            let (a, b) = (eval_edge(e, e.t0), eval_edge(e, e.t1));
            let far = if (a - fr.o).reject(fr.z).len() >= (b - fr.o).reject(fr.z).len() { a } else { b };
            let u = theta(fr, far);
            Some((Vec2::new(u, (*o - fr.o).dot(fr.z)), Vec2::new(0.0, d.dot(fr.z))))
        }
        _ => None,
    }
}

/// Make the body; returns it with the map face index -> body face.
fn emit(bd: &Builder, m: &Closed) -> (Body, Vec<ck_topo::body::FaceId>) {
    let mut b = Body::new();
    let ncomp = m.comps.len();
    let mut shell_of: Vec<Option<ShellId>> = vec![None; ncomp];
    let mut lump_of = vec![None; ncomp];
    for c in 0..ncomp {
        if m.comps[c].depth % 2 == 0 {
            let l = b.add_lump();
            lump_of[c] = Some(l);
            shell_of[c] = Some(b.add_shell(l));
        }
    }
    for c in 0..ncomp {
        if m.comps[c].depth % 2 == 1 {
            let l = m.comps[c].parent.and_then(|p| lump_of[p]).unwrap_or_else(|| b.add_lump());
            shell_of[c] = Some(b.add_shell(l));
        }
    }
    let vids: Vec<VertexId> = bd.vpos.iter().map(|&p| b.add_vertex(p)).collect();
    let mut eids: Vec<Option<EdgeId>> = vec![None; bd.edges.len()];
    let mut fids = vec![];
    for (fi, f) in bd.faces.iter().enumerate() {
        let shell = shell_of[f.comp as usize].unwrap();
        let (surface, sense, fr) = match &f.geo {
            Geo::Plane { n, p } => (Surface::plane(*p, *n), true, None),
            Geo::Prim { p, sense } => {
                let fr = bd.frames[fi].unwrap();
                let s = match *p {
                    Prim::Cyl { r, .. } => Surface::Cylinder { frame: fr, r },
                    Prim::Sphere { r, .. } => Surface::Sphere { frame: fr, r },
                    Prim::Cone { apex, a, ang } => Surface::Cone { frame: fr, r: (fr.o - apex).dot(a) * ang.tan(), semi_angle: ang },
                };
                (s, *sense > 0.0, Some(fr))
            }
        };
        let face = b.add_face(shell, surface.clone(), sense, FaceName::other("mesh", &format!("F{fi}")));
        fids.push(face);
        for lp in &bd.face_loops[fi] {
            let l = b.add_loop(face);
            // periodic faces: pcurves continuous along the loop (out of a pole the angle jumps back the loop's turns)
            let mut cur_u: Option<f64> = None;
            // a full sphere's loop is its seam twice: one side at 2 pi, the other at 0
            let full = lp.len() == 2 && lp[0].0 == lp[1].0;
            let wind = match (&f.geo, fr) {
                _ if full => TAU,
                (Geo::Prim { .. }, Some(fr)) => winding(bd, &fr, lp) * TAU,
                _ => 0.0,
            };
            for &c in lp {
                let e = &bd.edges[c.0];
                let eid = *eids[c.0].get_or_insert_with(|| b.add_edge(e.curve.clone(), e.t0, e.t1, vids[e.v0], vids[e.v1]));
                let pc = match (&f.geo, fr) {
                    (Geo::Plane { .. }, _) => match &surface {
                        Surface::Plane { frame } => ck_topo::make::plane_pcurve(frame, &e.curve),
                        _ => None,
                    },
                    (Geo::Prim { p, .. }, Some(fr)) => pcurve_line(bd, &fr, p, c).map(|(mut o, d)| {
                        let (ts, te) = if c.1 { (e.t1, e.t0) } else { (e.t0, e.t1) };
                        let start = bd.vpos[co_start(bd, c)];
                        let at_pole = (start - fr.o).reject(fr.z).len() < 1e-9 * (1.0 + start.len());
                        match cur_u {
                            Some(u) => {
                                let want = if at_pole { u - wind } else { u };
                                o.x += TAU * ((want - (o.x + d.x * ts)) / TAU).round();
                            }
                            None => {
                                // start the loop in [0, 2 pi); a full sphere's first seam at 2 pi
                                let s = o.x + d.x * ts;
                                o.x -= TAU * (s / TAU).floor();
                                if full {
                                    o.x += TAU;
                                }
                            }
                        }
                        cur_u = Some(o.x + d.x * te);
                        Curve2::Line { o, d }
                    }),
                    _ => None,
                };
                b.add_coedge(l, eid, c.1, pc);
            }
        }
    }
    // tolerances where an edge is off a face or a vertex off its curves (noise of the fits)
    let mut faces_of_edge: Vec<Vec<usize>> = vec![vec![]; bd.edges.len()];
    for (fi, lps) in bd.face_loops.iter().enumerate() {
        for lp in lps {
            for &(e, _) in lp {
                faces_of_edge[e].push(fi);
            }
        }
    }
    for (k, e) in bd.edges.iter().enumerate() {
        let Some(id) = eids[k] else { continue };
        let mut dev: f64 = 0.0;
        for &fi in &faces_of_edge[k] {
            for s in 0..=8 {
                let q = e.curve.eval(e.t0 + (e.t1 - e.t0) * s as f64 / 8.0);
                dev = dev.max(surf_dist(&bd.faces[fi].geo, q));
            }
        }
        let p0 = e.curve.eval(e.t0);
        if dev > 1e-7 * (1.0 + p0.len()) {
            let et = &mut b.edges[id.idx()].tol;
            *et = et.max(1.5 * dev);
        }
        for (v, t) in [(e.v0, e.t0), (e.v1, e.t1)] {
            let d = bd.vpos[v].dist(e.curve.eval(t));
            let p = bd.vpos[v];
            if d > 1e-7 * (1.0 + p.len()) {
                let vt = &mut b.vertices[vids[v].idx()].tol;
                *vt = vt.max(1.5 * d);
            }
        }
    }
    (b, fids)
}

// ------------------------------------------------------------------ driver

fn issue_faces(issues: &[ck_check::Issue], names: &[String]) -> Vec<usize> {
    let mut out = vec![];
    for i in issues {
        for (k, n) in names.iter().enumerate() {
            let hit = i.entity.contains(n.as_str()) && {
                // "mesh.F1" is a prefix of "mesh.F12": the next character must not be a digit
                i.entity.match_indices(n.as_str()).any(|(at, _)| !i.entity[at + n.len()..].starts_with(|c: char| c.is_ascii_digit()))
            };
            if hit {
                out.push(k);
            }
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// Refit: the analytic body (None when nothing was fitted or it did not validate; the faceted body then stays).
pub fn refit(m: &Closed, rg: &Regions, p: &FitParams, diag: f64, plane_tol: f64, _angle_tol_deg: f64) -> (Option<Body>, AnalyticReport) {
    // debug timing (never on wasm: no environment there, and no clock)
    let t_start = std::env::var("CK_MESH_DEBUG").is_ok().then(std::time::Instant::now);
    let tol = p.tol.filter(|v| *v > 0.0).unwrap_or(0.005 * diag);
    let maxc = m.pts.iter().map(|q| q.x.abs().max(q.y.abs()).max(q.z.abs())).fold(0.0, f64::max);
    let fit_tol = p.fit_tol.filter(|v| *v > 0.0).unwrap_or((2e-5 * diag).max(1e-6 * maxc));
    let mut rep = AnalyticReport { tol, fit_tol, ..Default::default() };
    if rg.n > p.max_regions {
        rep.reason = Some(format!("{} planar regions exceed the refit limit of {}", rg.n, p.max_regions));
        return (None, rep);
    }
    let nt = m.tris.len();
    // region contents and smooth adjacency
    let mut tris_of: Vec<Vec<u32>> = vec![vec![]; rg.n];
    let mut verts_of: Vec<Vec<u32>> = vec![vec![]; rg.n];
    for t in 0..nt {
        let r = rg.region[t] as usize;
        tris_of[r].push(t as u32);
        // a sliver's corners say nothing about the region's surface (its odd corner may sit on the neighbour's)
        if !rg.sliver[t] {
            verts_of[r].extend(m.tris[t]);
        }
    }
    for t in 0..nt {
        let r = rg.region[t] as usize;
        if verts_of[r].is_empty() {
            verts_of[r].extend(m.tris[t]);
        }
    }
    for v in verts_of.iter_mut() {
        v.sort_unstable();
        v.dedup();
    }
    let cos_smooth = p.smooth_deg.to_radians().cos();
    let mut smooth: Vec<Vec<u32>> = vec![vec![]; rg.n];
    for h in 0..3 * nt {
        let (a, b) = (rg.region[h / 3], rg.region[rg.twin[h] / 3]);
        if a != b && rg.normal[a as usize].dot(rg.normal[b as usize]) >= cos_smooth {
            smooth[a as usize].push(b);
        }
    }
    for s in smooth.iter_mut() {
        s.sort_unstable();
        s.dedup();
    }
    let seg = Seg { m, rg, verts: verts_of, tris: tris_of, smooth, tol, fit_tol, diag };
    let mut claimed = vec![false; rg.n];
    let mut order: Vec<u32> = (0..rg.n as u32).filter(|&r| !seg.smooth[r as usize].is_empty()).collect();
    // seeds: the largest facets first (the middle of a surface; slivers sit at its edges)
    order.sort_by(|&a, &b| rg.area[b as usize].total_cmp(&rg.area[a as usize]).then(a.cmp(&b)));
    let mut clusters: Vec<(Prim, Vec<u32>, f64)> = vec![];
    // a seed that grows nothing rules out its direct neighbours as seeds too (they can still be claimed by a growth); a
    // budget of attempts bounds the work on a mesh that has no analytic surface at all (a scan)
    let mut tried = vec![false; rg.n];
    let mut attempts = 0usize;
    work::start(p.max_seconds);
    for &s in &order {
        if claimed[s as usize] || tried[s as usize] {
            continue;
        }
        if work::over() {
            rep.budget_exhausted = true;
            rep.reason = Some(format!("the refit's work budget ({} s) ran out after {attempts} seeds; what is not fitted stays faceted", p.max_seconds));
            break;
        }
        attempts += 1;
        if attempts > p.max_seeds {
            rep.reason = Some(format!("seed budget of {} used up; the rest stays faceted", p.max_seeds));
            break;
        }
        let mut best: Option<(Prim, Vec<u32>, f64, usize)> = None;
        let dbg = std::env::var("CK_MESH_DEBUG").is_ok() && clusters.len() < 3;
        if dbg && s < 6 {
            let su = s as usize;
            eprintln!("seed {s}: tris {} area {:.3} normal {:?} smooth {:?} areas {:?}", seg.tris[su].len(), rg.area[su], rg.normal[su], seg.smooth[su], seg.smooth[su].iter().map(|&q| rg.area[q as usize]).collect::<Vec<_>>());
        }
        for kind in 0..3 {
            let got = seg.grow(s, kind, &claimed);
            if dbg {
                eprintln!("seed {s} kind {kind}: {:?}", got.as_ref().map(|g| (g.0, g.1.len())));
            }
            if let Some((pr, set, sense)) = got {
                let cov: usize = set.iter().map(|&r| seg.tris[r as usize].len()).sum();
                // a later type must claim clearly more to win (cylinder, then cone, then sphere)
                let better = match &best {
                    None => true,
                    Some(b) => cov as f64 > 1.05 * b.3 as f64,
                };
                if better {
                    best = Some((pr, set, sense, cov));
                }
            }
        }
        let Some((pr, set, sense, cov)) = best else {
            tried[s as usize] = true;
            for &q in &seg.smooth[s as usize] {
                tried[q as usize] = true;
            }
            continue;
        };
        // a real curved face: enough triangles and facets turning by more than the merge angle
        if cov < p.min_triangles || set.len() < 2 {
            continue;
        }
        for &r in &set {
            claimed[r as usize] = true;
        }
        clusters.push((pr, set, sense));
    }
    rep.work_units = work::spent();
    if work::over() && !rep.budget_exhausted && attempts < order.len() {
        rep.budget_exhausted = true;
    }
    if let Some(t) = t_start {
        eprintln!("growth done at {:.0} ms: {} clusters, {attempts} seeds", t.elapsed().as_secs_f64() * 1e3, clusters.len());
    }
    // absorb: unclaimed regions next to a cluster whose corners lie on its surface (pole fans, slivers)
    let mut absorbed = vec![false; rg.n];
    {
        let mut owner = vec![usize::MAX; rg.n];
        for (k, (_, set, _)) in clusters.iter().enumerate() {
            for &r in set {
                owner[r as usize] = k;
            }
        }
        let mut adj: Vec<Vec<u32>> = vec![vec![]; rg.n];
        for h in 0..3 * nt {
            let (a, b) = (rg.region[h / 3], rg.region[rg.twin[h] / 3]);
            if a != b {
                adj[a as usize].push(b);
            }
        }
        for _ in 0..8 {
            let mut changed = false;
            for r in 0..rg.n {
                if owner[r] != usize::MAX {
                    continue;
                }
                let mut ks: Vec<usize> = adj[r].iter().map(|&q| owner[q as usize]).filter(|&k| k != usize::MAX).collect();
                ks.sort_unstable();
                ks.dedup();
                for k in ks {
                    let (pr, _, sense) = clusters[k];
                    // a facet between two clusters may have a corner on neither (a tessellator's transition triangle):
                    // within the chord sag it joins
                    if seg.fits_within(r as u32, &pr, sense, tol) {
                        absorbed[r] = true;
                        owner[r] = k;
                        claimed[r] = true;
                        clusters[k].1.push(r as u32);
                        changed = true;
                        break;
                    }
                }
            }
            if !changed {
                break;
            }
        }
    }
    // merge neighbouring clusters of one surface (a growth that stopped at a seam or a sliver and started again)
    loop {
        let mut owner = vec![usize::MAX; rg.n];
        for (k, (_, set, _)) in clusters.iter().enumerate() {
            for &r in set {
                owner[r as usize] = k;
            }
        }
        let mut pairs: Vec<(usize, usize)> = vec![];
        for h in 0..3 * nt {
            let (a, b) = (owner[rg.region[h / 3] as usize], owner[rg.region[rg.twin[h] / 3] as usize]);
            if a != usize::MAX && b != usize::MAX && a < b {
                pairs.push((a, b));
            }
        }
        pairs.sort_unstable();
        pairs.dedup();
        let mut merged = false;
        for (a, b) in pairs {
            let (pa, sa_) = (clusters[a].0, clusters[a].2);
            let (pb, sb_) = (clusters[b].0, clusters[b].2);
            if pa.name() != pb.name() || sa_ != sb_ {
                continue;
            }
            let mut set = clusters[a].1.clone();
            set.extend(clusters[b].1.iter().copied());
            let core: Vec<u32> = set.iter().copied().filter(|&r| !absorbed[r as usize]).collect();
            let (pts, _) = seg.data(&core);
            let big = if clusters[a].1.len() >= clusters[b].1.len() { pa } else { pb };
            let q = refine(big, &pts, diag);
            let ok = |r: u32| if absorbed[r as usize] { seg.fits_within(r, &q, sa_, tol) } else { seg.fits(r, &q, sa_) };
            if plausible(&q, diag) && set.iter().all(|&r| ok(r)) {
                clusters[a] = (q, set, sa_);
                clusters.remove(b);
                merged = true;
                break;
            }
        }
        if !merged {
            break;
        }
    }
    if std::env::var("CK_MESH_DEBUG").is_ok() {
        for (k, (pr, set, sense)) in clusters.iter().enumerate() {
            eprintln!("cluster {k}: {:?} regions {} sense {sense}", pr, set.len());
        }
        let left: Vec<u32> = order.iter().copied().filter(|&r| !claimed[r as usize]).collect();
        eprintln!("unclaimed candidates {}", left.len());
        if let Some((pr, _, sense)) = clusters.first() {
            for &r in left.iter().take(12) {
                let ru = r as usize;
                let dv = seg.verts[ru].iter().map(|&i| pr.dist(m.pts[i as usize])).fold(0.0, f64::max);
                let dc = pr.dist(rg.point[ru]);
                let na = rg.normal[ru].dot(pr.normal(rg.point[ru])) * sense;
                eprintln!("  region {r}: tris {} area {:.3e} corner dev {dv:.2e} centroid dev {dc:.2e} normal {na:.4} smooth nbrs {}", seg.tris[ru].len(), rg.area[ru], seg.smooth[ru].len());
            }
        }
    }
    if clusters.is_empty() {
        rep.reason = Some("nothing to fit".into());
        rep.faceted = order.len();
        return (None, rep);
    }
    // faces: clusters first, then the remaining planar regions
    let mut keep_cluster = vec![true; clusters.len()];
    let region_points: Vec<Vec3> = rg.point.clone();
    for round in 0..6 {
        rep.rounds = round + 1;
        if let Some(t) = t_start {
            eprintln!("round {round} at {:.0} ms, clusters kept {}", t.elapsed().as_secs_f64() * 1e3, keep_cluster.iter().filter(|k| **k).count());
        }
        if !keep_cluster.iter().any(|k| *k) {
            rep.reason = Some("no fitted face has an exact boundary".into());
            rep.faceted = order.len();
            return (None, rep);
        }
        let mut faces: Vec<FaceIn> = vec![];
        let mut face_of_region = vec![usize::MAX; rg.n];
        for (k, (pr, set, sense)) in clusters.iter().enumerate() {
            if !keep_cluster[k] {
                continue;
            }
            let fi = faces.len();
            for &r in set {
                face_of_region[r as usize] = fi;
            }
            let triangles = set.iter().map(|&r| seg.tris[r as usize].len()).sum();
            faces.push(FaceIn { geo: Geo::Prim { p: *pr, sense: *sense }, regions: set.clone(), comp: rg.comp[set[0] as usize], triangles });
        }
        let cluster_faces = faces.len();
        let cluster_index: Vec<usize> = (0..clusters.len()).filter(|&k| keep_cluster[k]).collect();
        for r in 0..rg.n {
            if face_of_region[r] == usize::MAX {
                face_of_region[r] = faces.len();
                faces.push(FaceIn { geo: Geo::Plane { n: rg.normal[r], p: rg.point[r] }, regions: vec![r as u32], comp: rg.comp[r], triangles: rg.count[r] });
            }
        }
        let mut face_of_tri: Vec<u32> = rg.region.iter().map(|&r| face_of_region[r as usize] as u32).collect();
        // a triangle whose corners are off its face but on a neighbour's goes to the neighbour: a sliver, or a facet of
        // the next surface that coplanar merging put with a strip (at a tangent junction)
        for _ in 0..2 {
            for t in 0..nt {
                if !rg.sliver[t] && !matches!(faces[face_of_tri[t] as usize].geo, Geo::Prim { .. }) {
                    continue;
                }
                let on = |f: usize| {
                    let lim = match faces[f].geo { Geo::Plane { .. } => plane_tol, _ => fit_tol };
                    m.tris[t].iter().all(|&i| surf_dist(&faces[f].geo, m.pts[i as usize]) <= lim)
                };
                if on(face_of_tri[t] as usize) {
                    continue;
                }
                if let Some(g) = (0..3).map(|k| face_of_tri[rg.twin[3 * t + k] / 3] as usize).find(|&g| on(g)) {
                    face_of_tri[t] = g as u32;
                }
            }
        }
        let mut g = build_graph(m, &face_of_tri, faces.len(), &rg.twin);
        let drop = |bad: Vec<usize>, why: &'static str, rep: &mut AnalyticReport, keep: &mut Vec<bool>| {
            for f in bad {
                if f < cluster_faces {
                    let k = cluster_index[f];
                    keep[k] = false;
                    rep.declined.push((why, format!("{} of {} triangles", clusters[k].0.name(), faces[f].triangles)));
                }
            }
        };
        if let Err(bad) = run_curves(&mut g, &faces, m, fit_tol, plane_tol, tol.max(fit_tol)) {
            drop(bad, "boundaryNotExact", &mut rep, &mut keep_cluster);
            continue;
        }
        let mut bd = Builder { m, faces: &faces, vpos: vec![], vkey: HashMap::new(), edges: vec![], run_edges: vec![], face_loops: vec![], frames: vec![], region_points: region_points.clone(), fixed: Default::default() };
        make_run_edges(&mut bd, &g);
        if let Err(bad) = periodic_faces(&mut bd, &g) {
            drop(bad, "noSeam", &mut rep, &mut keep_cluster);
            continue;
        }
        let (body, fids) = emit(&bd, m);
        let issues = ck_check::validate(&body, 1e-6);
        if !issues.is_empty() {
            if std::env::var("CK_MESH_DEBUG").is_ok() {
                for i in issues.iter().take(8) {
                    eprintln!("issue {} {} {}", i.code, i.entity, i.detail);
                }
            }
            let names: Vec<String> = fids.iter().map(|&f| body.face_id_str(f)).collect();
            let mut bad: Vec<usize> = issue_faces(&issues, &names).into_iter().filter(|&f| f < cluster_faces).collect();
            if bad.is_empty() {
                // an issue on planar faces only (next to a fitted face): send back the fitted faces next to them
                bad = (0..cluster_faces).collect();
                if round >= 2 {
                    rep.reason = Some(format!("validation: {} {}", issues[0].code, issues[0].detail));
                    return (None, rep);
                }
            }
            drop(bad, "invalid", &mut rep, &mut keep_cluster);
            if keep_cluster.iter().all(|k| !k) {
                rep.reason = Some(format!("validation: {} {}", issues[0].code, issues[0].detail));
                return (None, rep);
            }
            continue;
        }
        // report
        for (fi, f) in faces.iter().enumerate() {
            match &f.geo {
                Geo::Plane { .. } => {
                    rep.planes += 1;
                    if !seg.smooth[f.regions[0] as usize].is_empty() && f.regions.len() == 1 {
                        rep.faceted += 1;
                    }
                }
                Geo::Prim { p: pr, .. } => {
                    match pr {
                        Prim::Cyl { .. } => rep.cylinders += 1,
                        Prim::Cone { .. } => rep.cones += 1,
                        Prim::Sphere { .. } => rep.spheres += 1,
                    }
                    let (mut mx, mut s2, mut n) = (0.0f64, 0.0, 0usize);
                    for &r in &f.regions {
                        for &v in &seg.verts[r as usize] {
                            let d = pr.dist(m.pts[v as usize]);
                            mx = mx.max(d);
                            s2 += d * d;
                            n += 1;
                        }
                    }
                    let (axis, origin, radius, half) = match *pr {
                        Prim::Cyl { o, a, r } => (a, o, r, 0.0),
                        Prim::Cone { apex, a, ang } => (a, apex, 0.0, ang),
                        Prim::Sphere { c, r } => (Vec3::Z, c, r, 0.0),
                    };
                    rep.fitted.push(Fitted { kind: pr.name(), face: body.face_id_str(fids[fi]), triangles: f.triangles, axis, origin, radius, half_angle: half, max_residual: mx, rms_residual: (s2 / n.max(1) as f64).sqrt() });
                }
            }
        }
        // planes next to a tangent fitted face are not facets: only count regions still smooth to another planar region
        rep.faceted = faces
            .iter()
            .filter(|f| matches!(f.geo, Geo::Plane { .. }))
            .filter(|f| seg.smooth[f.regions[0] as usize].iter().any(|&q| matches!(faces[face_of_region[q as usize]].geo, Geo::Plane { .. })))
            .count();
        rep.applied = true;
        return (Some(body), rep);
    }
    rep.reason = Some("no exact boundary after six rounds".into());
    (None, rep)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Facets (triangles: 3 corners, normal, area) of a patch of `p` over (u, v) in [u0, u1] x [v0, v1].
    fn patch(eval: &dyn Fn(f64, f64) -> Vec3, u: (f64, f64), v: (f64, f64), n: usize, outward: Vec3) -> (Vec<Vec3>, Vec<(Vec3, f64)>) {
        let mut pts = vec![];
        let mut nrm = vec![];
        for i in 0..n {
            for j in 0..n {
                let g = |a: usize, b: usize| eval(u.0 + (u.1 - u.0) * a as f64 / n as f64, v.0 + (v.1 - v.0) * b as f64 / n as f64);
                for t in [[g(i, j), g(i + 1, j), g(i + 1, j + 1)], [g(i, j), g(i + 1, j + 1), g(i, j + 1)]] {
                    let c = (t[1] - t[0]).cross(t[2] - t[0]);
                    let c = if c.dot(t[0] - outward) < 0.0 { -c } else { c };
                    pts.extend(t);
                    nrm.push((c.normalize(), c.len() * 0.5));
                }
            }
        }
        (pts, nrm)
    }

    #[test]
    fn fits_recover_the_primitives() {
        // a cone patch: apex (0, 40, 0), axis -y, half angle atan(0.25)
        let k: f64 = 0.25;
        let cone = |u: f64, h: f64| Vec3::new(h * k * u.cos(), 40.0 - h, h * k * u.sin());
        // a tiny patch is poorly conditioned (it is nearly a tilted cylinder): looser there
        for (span, n, lim) in [(0.3, 2, 1e-3), (1.0, 3, 1e-6), (6.0, 8, 1e-6)] {
            let (pts, nrm) = patch(&cone, (0.2, 0.2 + span), (20.0, 30.0), n, Vec3::new(0.0, 0.0, 0.0));
            let d = Data { pts: &pts, normals: &nrm, scale: 50.0 };
            let p = fit_cone(&d).expect("cone");
            let e = pts.iter().map(|&q| p.dist(q)).fold(0.0, f64::max);
            eprintln!("span {span}: {p:?} max residual {e:e}");
            assert!(e < lim, "{p:?} {e}");
        }
        // refining from a wrong local cone converges to the true one
        let (pts, _) = patch(&cone, (0.0, 2.0), (20.0, 40.0), 6, Vec3::new(0.0, 0.0, 0.0));
        let wrong = Prim::Cone { apex: Vec3::new(-2.86, 51.6, 0.53), a: Vec3::new(0.0604, -0.998, -0.0113).normalize(), ang: 0.1835 };
        let p = refine(wrong, &pts, 50.0);
        let e = pts.iter().map(|&q| p.dist(q)).fold(0.0, f64::max);
        eprintln!("refined {p:?} {e:e}");
        assert!(e < 1e-6);
        let cyl = |u: f64, h: f64| Vec3::new(5.0 * u.cos(), h, 5.0 * u.sin());
        let (pts, nrm) = patch(&cyl, (0.0, 0.5), (0.0, 10.0), 3, Vec3::new(0.0, 0.0, 0.0));
        let p = fit_cylinder(&Data { pts: &pts, normals: &nrm, scale: 20.0 }).unwrap();
        assert!(pts.iter().all(|&q| p.dist(q) < 1e-8), "{p:?}");
        let sph = |u: f64, v: f64| Vec3::new(3.0 * v.cos() * u.cos(), 3.0 * v.cos() * u.sin(), 3.0 * v.sin()) + Vec3::new(1.0, 2.0, 3.0);
        let (pts, nrm) = patch(&sph, (0.0, 0.4), (0.1, 0.5), 3, Vec3::new(1.0, 2.0, 3.0));
        let p = fit_sphere(&Data { pts: &pts, normals: &nrm, scale: 20.0 }).unwrap();
        assert!(pts.iter().all(|&q| p.dist(q) < 1e-8), "{p:?}");
    }
}
