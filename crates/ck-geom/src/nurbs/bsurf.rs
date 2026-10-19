//! Tensor-product B-spline / NURBS surfaces.
//!
//! Field naming pinned by kernel.strings key run `deg_u deg_v knots_u
//! knots_v ctrl` (surface wire record); a second run `knotsU knotsV nu nv
//! stride controlNet` names the inspect/analysis view. Control net is
//! row-major with u as the slow index: `ctrl[i * nv + j]`, i < nu, j < nv
//! (GUESS: u-major matches STEP B_SPLINE_SURFACE control_points_list
//! ordering, which the STEP writer emits as nested lists per u).
//! STEP: B_SPLINE_SURFACE_WITH_KNOTS / RATIONAL_B_SPLINE_SURFACE.

use super::basis::{self, ders_basis_funs, find_span};
use super::bspline::BSplineCurve;
use super::rational::{rat_surface_derivs, validate_weights};
use crate::surface::SurfDerivs;
use crate::Vec3;

#[derive(Debug, Clone, PartialEq)]
pub struct BSplineSurface {
    pub deg_u: usize,
    pub deg_v: usize,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
    pub nu: usize,
    pub nv: usize,
    /// `ctrl[i * nv + j]`
    pub ctrl: Vec<Vec3>,
    pub weights: Option<Vec<f64>>,
}

/// The span and basis rows (value and two derivatives) of the last u and last v of
/// [`BSplineSurface::derivs2_cached`] (cubic surfaces).
#[derive(Default)]
pub struct Basis2Cache {
    u: Option<(u64, usize, [[f64; 4]; 3])>,
    v: Option<(u64, usize, [[f64; 4]; 3])>,
}

pub struct CpPrep {
    nu: usize,
    cv: usize,
    boxes: Vec<(Vec3, Vec3)>,
    vs: Vec<(f64, usize)>,
    bv: Vec<(usize, Vec<f64>)>,
    cols: Vec<Vec<usize>>,
    /// (u span, iu, iv, first / last u grid column of the cell)
    cells: Vec<(usize, usize, usize, usize, usize)>,
    ubas: Vec<(usize, Vec<f64>)>,
    cell_scale: f64,
    /// box of every cell box
    hull: (Vec3, Vec3),
    /// Non-rational nets: the seed grid points `[a * (nv + 1) + k]` and the
    /// box of each (u column, v span) row `[a * cv + iv]`, computed once with
    /// the very expressions the query scan used per call (the scan is then a
    /// distance lookup; results are bit-identical).
    grid: std::cell::OnceCell<(Vec<Vec3>, Vec<(Vec3, Vec3)>)>,
    /// the grid is wanted (non-rational, not too big); it is built by the query that finds the surface asked
    /// for often enough to repay it (`BSplineSurface::grid_for`)
    grid_ok: bool,
    uses: std::cell::Cell<u32>,
    /// Tiles of up to 4 x 4 neighbouring cells: the box of their cell boxes, and their indices into
    /// `cells` (`tile_cells[tile_start[t]..tile_start[t + 1]]`). A tile's box distance is no larger than
    /// any of its cells' (the same expression on smaller / larger corners), so a far tile is skipped
    /// whole when the nearest cells are picked (see `nearest_cells`).
    tiles: Vec<(Vec3, Vec3)>,
    tile_start: Vec<u32>,
    tile_cells: Vec<u32>,
}

/// Distance from `q` to the box `(lo, hi)`, infinite for an empty box (lo > hi): the lower bound of
/// the closest-point scan.
#[inline]
fn box_dist(lo: Vec3, hi: Vec3, q: Vec3) -> f64 {
    if lo.x > hi.x {
        return f64::INFINITY;
    }
    Vec3::new((lo.x - q.x).max(q.x - hi.x).max(0.0), (lo.y - q.y).max(q.y - hi.y).max(0.0), (lo.z - q.z).max(q.z - hi.z).max(0.0)).len()
}

/// Cells the scan visits first: the 8 nearest by box distance (ties by cell index).
const FIRST_CELLS: usize = 8;

/// The nearest cells of an unbounded query (`nearest_cells`): their indices into `CpPrep::cells`,
/// nearest first, and the box distance of the next nearest cell.
struct Nearest {
    chosen: [usize; FIRST_CELLS],
    lb9: f64,
}

/// Working storage of one bounded closest-point query (see `SCRATCH`).
#[derive(Default)]
struct CpScratch {
    lb: Vec<f64>,
    cells: Vec<(f64, usize, usize, usize, usize, usize)>,
    order: Vec<usize>,
    cell_best: Vec<(f64, f64, f64, f64)>,
}

thread_local! {
    static SCRATCH: std::cell::RefCell<CpScratch> = const { std::cell::RefCell::new(CpScratch { lb: Vec::new(), cells: Vec::new(), order: Vec::new(), cell_best: Vec::new() }) };
}

impl BSplineSurface {
    pub fn new(
        deg_u: usize,
        deg_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        nu: usize,
        nv: usize,
        ctrl: Vec<Vec3>,
        weights: Option<Vec<f64>>,
    ) -> Result<Self, String> {
        if ctrl.len() != nu * nv {
            return Err(format!("control net {} != {} x {}", ctrl.len(), nu, nv));
        }
        basis::validate(&knots_u, nu, deg_u)?;
        basis::validate(&knots_v, nv, deg_v)?;
        if let Some(w) = &weights {
            validate_weights(w, ctrl.len())?;
        }
        Ok(BSplineSurface { deg_u, deg_v, knots_u, knots_v, nu, nv, ctrl, weights })
    }

    /// Bilinear patch through four corners: p00 (u0,v0), p10 (u1,v0), p01, p11.
    pub fn bilinear(p00: Vec3, p10: Vec3, p01: Vec3, p11: Vec3) -> Self {
        BSplineSurface {
            deg_u: 1,
            deg_v: 1,
            knots_u: vec![0., 0., 1., 1.],
            knots_v: vec![0., 0., 1., 1.],
            nu: 2,
            nv: 2,
            ctrl: vec![p00, p01, p10, p11],
            weights: None,
        }
    }

    pub fn is_rational(&self) -> bool {
        super::rational::is_rational(self.weights.as_deref())
    }

    #[inline]
    pub fn cp(&self, i: usize, j: usize) -> Vec3 {
        self.ctrl[i * self.nv + j]
    }

    #[inline]
    pub fn weight(&self, i: usize, j: usize) -> f64 {
        self.weights.as_ref().map_or(1.0, |w| w[i * self.nv + j])
    }

    /// ((u0,u1),(v0,v1))
    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        (
            (self.knots_u[self.deg_u], self.knots_u[self.nu]),
            (self.knots_v[self.deg_v], self.knots_v[self.nv]),
        )
    }

    pub fn eval(&self, u: f64, v: f64) -> Vec3 {
        if self.weights.is_none() && self.deg_u < 12 && self.deg_v < 12 {
            let ((u0, u1), (v0, v1)) = self.domain();
            let (u, v) = (u.clamp(u0, u1), v.clamp(v0, v1));
            let (p, q) = (self.deg_u, self.deg_v);
            let su = find_span(self.nu - 1, p, u, &self.knots_u);
            let sv = find_span(self.nv - 1, q, v, &self.knots_v);
            let nu = [basis::basis_funs2(su, u, p, &self.knots_u)];
            let nv = [basis::basis_funs2(sv, v, q, &self.knots_v)];
            let mut tp = [Vec3::ZERO; 12];
            for s in 0..=q {
                for r in 0..=p {
                    tp[s] = tp[s] + self.cp(su - p + r, sv - q + s) * (nu[0][r] * 1.0);
                }
            }
            let mut a = Vec3::ZERO;
            for s in 0..=q {
                a = a + tp[s] * nv[0][s];
            }
            return a;
        }
        if self.deg_u < 12 && self.deg_v < 12 {
            // allocation-free rational path (same arithmetic as `derivs(.., 0)`)
            let ((u0, u1), (v0, v1)) = self.domain();
            let (u, v) = (u.clamp(u0, u1), v.clamp(v0, v1));
            let (p, q) = (self.deg_u, self.deg_v);
            let su = find_span(self.nu - 1, p, u, &self.knots_u);
            let sv = find_span(self.nv - 1, q, v, &self.knots_v);
            let nu = [basis::basis_funs2(su, u, p, &self.knots_u)];
            let nv = [basis::basis_funs2(sv, v, q, &self.knots_v)];
            let mut tp = [Vec3::ZERO; 12];
            let mut tw = [0.0f64; 12];
            for s in 0..=q {
                for r in 0..=p {
                    let (i, j) = (su - p + r, sv - q + s);
                    let wt = self.weight(i, j);
                    tp[s] = tp[s] + self.cp(i, j) * (nu[0][r] * wt);
                    tw[s] += nu[0][r] * wt;
                }
            }
            let (mut a, mut w) = (Vec3::ZERO, 0.0);
            for s in 0..=q {
                a = a + tp[s] * nv[0][s];
                w += tw[s] * nv[0][s];
            }
            return a / w;
        }
        self.derivs(u, v, 0)[0][0]
    }

    /// Points on the grid `us x vs` (row-major, `i * vs.len() + j`), each
    /// bit-identical to `eval(us[i], vs[j])`: the basis functions are computed
    /// once per row / column instead of once per point.
    pub fn eval_grid(&self, us: &[f64], vs: &[f64]) -> Vec<Vec3> {
        if self.deg_u >= 12 || self.deg_v >= 12 {
            return us.iter().flat_map(|&u| vs.iter().map(move |&v| self.eval(u, v))).collect();
        }
        let ((u0, u1), (v0, v1)) = self.domain();
        let (p, q) = (self.deg_u, self.deg_v);
        let rows: Vec<(usize, [f64; 12])> = us
            .iter()
            .map(|&u| {
                let u = u.clamp(u0, u1);
                let su = find_span(self.nu - 1, p, u, &self.knots_u);
                (su, basis::basis_funs2(su, u, p, &self.knots_u))
            })
            .collect();
        let cols: Vec<(usize, [f64; 12])> = vs
            .iter()
            .map(|&v| {
                let v = v.clamp(v0, v1);
                let sv = find_span(self.nv - 1, q, v, &self.knots_v);
                (sv, basis::basis_funs2(sv, v, q, &self.knots_v))
            })
            .collect();
        let mut out = Vec::with_capacity(us.len() * vs.len());
        for &(su, nu) in &rows {
            for &(sv, nv) in &cols {
                let mut tp = [Vec3::ZERO; 12];
                if self.weights.is_none() {
                    for s in 0..=q {
                        for r in 0..=p {
                            tp[s] = tp[s] + self.cp(su - p + r, sv - q + s) * (nu[r] * 1.0);
                        }
                    }
                    let mut a = Vec3::ZERO;
                    for s in 0..=q {
                        a = a + tp[s] * nv[s];
                    }
                    out.push(a);
                } else {
                    let mut tw = [0.0f64; 12];
                    for s in 0..=q {
                        for r in 0..=p {
                            let (i, j) = (su - p + r, sv - q + s);
                            let wt = self.weight(i, j);
                            tp[s] = tp[s] + self.cp(i, j) * (nu[r] * wt);
                            tw[s] += nu[r] * wt;
                        }
                    }
                    let (mut a, mut w) = (Vec3::ZERO, 0.0);
                    for s in 0..=q {
                        a = a + tp[s] * nv[s];
                        w += tw[s] * nv[s];
                    }
                    out.push(a / w);
                }
            }
        }
        out
    }

    /// `skl[k][l]` = ∂^{k+l} S / ∂u^k ∂v^l for k + l <= d (A3.6 / A4.4).
    pub fn derivs(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vec3>> {
        let ((u0, u1), (v0, v1)) = self.domain();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);
        let (p, q) = (self.deg_u, self.deg_v);
        let su = find_span(self.nu - 1, p, u, &self.knots_u);
        let sv = find_span(self.nv - 1, q, v, &self.knots_v);
        let nu = ders_basis_funs(su, u, p, d, &self.knots_u);
        let nv = ders_basis_funs(sv, v, q, d, &self.knots_v);
        let mut a = vec![vec![Vec3::ZERO; d + 1]; d + 1];
        let mut w = vec![vec![0.0; d + 1]; d + 1];
        for k in 0..=d.min(p) {
            // temp[s] = Σ_r N^(k)_u[r] * Pw[su-p+r][sv-q+s]
            let mut tp = vec![Vec3::ZERO; q + 1];
            let mut tw = vec![0.0; q + 1];
            for s in 0..=q {
                for r in 0..=p {
                    let (i, j) = (su - p + r, sv - q + s);
                    let wt = self.weight(i, j);
                    tp[s] = tp[s] + self.cp(i, j) * (nu[k][r] * wt);
                    tw[s] += nu[k][r] * wt;
                }
            }
            for l in 0..=(d - k).min(q) {
                for s in 0..=q {
                    a[k][l] = a[k][l] + tp[s] * nv[l][s];
                    w[k][l] += tw[s] * nv[l][s];
                }
            }
        }
        if self.weights.is_none() {
            return a;
        }
        rat_surface_derivs(&a, &w, d)
    }

    /// The four third partials `(S_uuu, S_uuv, S_uvv, S_vvv)` of a non-rational surface of degrees 1 to 5, the
    /// entries `derivs(u, v, 3)` gives for them, bit for bit, without allocating. `None` for any other surface.
    pub fn thirds_nr(&self, u: f64, v: f64) -> Option<[Vec3; 4]> {
        let (p, q) = (self.deg_u, self.deg_v);
        if self.weights.is_some() || p == 0 || q == 0 || p >= 6 || q >= 6 {
            return None;
        }
        let ((u0, u1), (v0, v1)) = self.domain();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);
        let su = find_span(self.nu - 1, p, u, &self.knots_u);
        let sv = find_span(self.nv - 1, q, v, &self.knots_v);
        let nu = basis::ders_basis_funs3(su, u, p, 3, &self.knots_u);
        let nv = basis::ders_basis_funs3(sv, v, q, 3, &self.knots_v);
        let mut a = [[Vec3::ZERO; 4]; 4];
        for k in 0..=3usize.min(p) {
            let mut tp = [Vec3::ZERO; 6];
            for s in 0..=q {
                for r in 0..=p {
                    tp[s] = tp[s] + self.cp(su - p + r, sv - q + s) * (nu[k][r] * 1.0);
                }
            }
            for l in 0..=(3 - k).min(q) {
                for s in 0..=q {
                    a[k][l] = a[k][l] + tp[s] * nv[l][s];
                }
            }
        }
        Some([a[3][0], a[2][1], a[1][2], a[0][3]])
    }

    /// `(S, S_u, S_v)` of a non-rational surface (degrees 1 to 11 in both directions), the entries
    /// `derivs(u, v, 1)` gives for them, bit for bit, without allocating. `None` for any other surface.
    pub fn derivs1_nr(&self, u: f64, v: f64) -> Option<(Vec3, Vec3, Vec3)> {
        let (p, q) = (self.deg_u, self.deg_v);
        if self.weights.is_some() || p == 0 || q == 0 || p >= 12 || q >= 12 {
            return None;
        }
        let ((u0, u1), (v0, v1)) = self.domain();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);
        let su = find_span(self.nu - 1, p, u, &self.knots_u);
        let sv = find_span(self.nv - 1, q, v, &self.knots_v);
        let nu = basis::ders_basis_funs2(su, u, p, 1, &self.knots_u);
        let nv = basis::ders_basis_funs2(sv, v, q, 1, &self.knots_v);
        let mut tp = [[Vec3::ZERO; 12]; 2];
        for s in 0..=q {
            let row = &self.ctrl[(sv - q + s)..];
            for r in 0..=p {
                let c = row[(su - p + r) * self.nv];
                tp[0][s] = tp[0][s] + c * (nu[0][r] * 1.0);
                tp[1][s] = tp[1][s] + c * (nu[1][r] * 1.0);
            }
        }
        let (mut a00, mut a01, mut a10) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        for s in 0..=q {
            a00 = a00 + tp[0][s] * nv[0][s];
            a01 = a01 + tp[0][s] * nv[1][s];
            a10 = a10 + tp[1][s] * nv[0][s];
        }
        Some((a00, a10, a01))
    }

    /// Non-rational second derivatives (allocation free, same arithmetic as `derivs`).
    #[inline(always)]
    fn derivs2_nr(&self, u: f64, v: f64, p: usize, q: usize) -> SurfDerivs {
        // allocation-free non-rational path (same arithmetic as `derivs`)
        let ((u0, u1), (v0, v1)) = self.domain();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);
        let su = find_span(self.nu - 1, p, u, &self.knots_u);
        let sv = find_span(self.nv - 1, q, v, &self.knots_v);
        if p == 3 && q == 3 {
            let nu = basis::ders_basis_funs2_p3(su, u, 2, &self.knots_u);
            let nv = basis::ders_basis_funs2_p3(sv, v, 2, &self.knots_v);
            return self.accumulate2(su, sv, &nu, &nv, 3, 3);
        }
        let nu = basis::ders_basis_funs2(su, u, p, 2, &self.knots_u);
        let nv = basis::ders_basis_funs2(sv, v, q, 2, &self.knots_v);
        self.accumulate2(su, sv, &nu, &nv, p, q)
    }

    /// The control point sums of `derivs2_nr` from the spans and basis rows.
    #[inline(always)]
    fn accumulate2<const W: usize>(&self, su: usize, sv: usize, nu: &[[f64; W]; 3], nv: &[[f64; W]; 3], p: usize, q: usize) -> SurfDerivs {
        let mut a = [[Vec3::ZERO; 3]; 3];
        // one pass over the control points for the three u orders (each
        // accumulator sees the same operations in the same order as a
        // pass per order: bit-identical)
        let ku = 2usize.min(p);
        let mut tp = [[Vec3::ZERO; W]; 3];
        for s in 0..=q {
            let row = &self.ctrl[(sv - q + s)..];
            for r in 0..=p {
                let c = row[(su - p + r) * self.nv];
                tp[0][s] = tp[0][s] + c * (nu[0][r] * 1.0);
                if ku >= 1 {
                    tp[1][s] = tp[1][s] + c * (nu[1][r] * 1.0);
                }
                if ku >= 2 {
                    tp[2][s] = tp[2][s] + c * (nu[2][r] * 1.0);
                }
            }
        }
        for k in 0..=ku {
            for l in 0..=(2 - k).min(q) {
                for s in 0..=q {
                    a[k][l] = a[k][l] + tp[k][s] * nv[l][s];
                }
            }
        }
        SurfDerivs { p: a[0][0], du: a[1][0], dv: a[0][1], duu: a[2][0], duv: a[1][1], dvv: a[0][2] }
    }

    /// [`derivs2`](Self::derivs2) for a run of evaluations (a Newton iteration) that often repeats one of the
    /// two parameters: the span and basis rows of the last `u` and of the last `v` are kept in `cache`. The
    /// same numbers, bit for bit.
    pub fn derivs2_cached(&self, u: f64, v: f64, cache: &mut Basis2Cache) -> SurfDerivs {
        let (p, q) = (self.deg_u, self.deg_v);
        if self.weights.is_some() || p != 3 || q != 3 {
            return self.derivs2(u, v);
        }
        let ((u0, u1), (v0, v1)) = self.domain();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);
        if cache.u.as_ref().map_or(true, |c| c.0 != u.to_bits()) {
            let su = find_span(self.nu - 1, 3, u, &self.knots_u);
            cache.u = Some((u.to_bits(), su, basis::ders_basis_funs2_p3(su, u, 2, &self.knots_u)));
        }
        if cache.v.as_ref().map_or(true, |c| c.0 != v.to_bits()) {
            let sv = find_span(self.nv - 1, 3, v, &self.knots_v);
            cache.v = Some((v.to_bits(), sv, basis::ders_basis_funs2_p3(sv, v, 2, &self.knots_v)));
        }
        let (cu, cv) = (cache.u.as_ref().unwrap(), cache.v.as_ref().unwrap());
        self.accumulate2(cu.1, cv.1, &cu.2, &cv.2, 3, 3)
    }

    /// Point + first + second partials.
    pub fn derivs2(&self, u: f64, v: f64) -> SurfDerivs {
        let (p, q) = (self.deg_u, self.deg_v);
        if self.weights.is_none() && p < 12 && q < 12 {
            return if p == 3 && q == 3 { self.derivs2_nr(u, v, 3, 3) } else { self.derivs2_nr(u, v, p, q) };
        }
        if p < 12 && q < 12 {
            // allocation-free rational path: homogeneous derivatives, then the
            // quotient rule to second order (same result as `rat_surface_derivs`)
            let ((u0, u1), (v0, v1)) = self.domain();
            let u = u.clamp(u0, u1);
            let v = v.clamp(v0, v1);
            let su = find_span(self.nu - 1, p, u, &self.knots_u);
            let sv = find_span(self.nv - 1, q, v, &self.knots_v);
            let nu = basis::ders_basis_funs2(su, u, p, 2, &self.knots_u);
            let nv = basis::ders_basis_funs2(sv, v, q, 2, &self.knots_v);
            let mut a = [[Vec3::ZERO; 3]; 3];
            let mut w = [[0.0f64; 3]; 3];
            for k in 0..=2usize.min(p) {
                let mut tp = [Vec3::ZERO; 12];
                let mut tw = [0.0f64; 12];
                for s in 0..=q {
                    for r in 0..=p {
                        let (i, j) = (su - p + r, sv - q + s);
                        let wt = self.weight(i, j);
                        tp[s] = tp[s] + self.cp(i, j) * (nu[k][r] * wt);
                        tw[s] += nu[k][r] * wt;
                    }
                }
                for l in 0..=(2 - k).min(q) {
                    for s in 0..=q {
                        a[k][l] = a[k][l] + tp[s] * nv[l][s];
                        w[k][l] += tw[s] * nv[l][s];
                    }
                }
            }
            let w0 = w[0][0];
            let pt = a[0][0] / w0;
            let du = (a[1][0] - pt * w[1][0]) / w0;
            let dv = (a[0][1] - pt * w[0][1]) / w0;
            let duu = (a[2][0] - du * (2.0 * w[1][0]) - pt * w[2][0]) / w0;
            let dvv = (a[0][2] - dv * (2.0 * w[0][1]) - pt * w[0][2]) / w0;
            let duv = (a[1][1] - dv * w[1][0] - du * w[0][1] - pt * w[1][1]) / w0;
            return SurfDerivs { p: pt, du, dv, duu, duv, dvv };
        }
        let s = self.derivs(u, v, 2);
        SurfDerivs { p: s[0][0], du: s[1][0], dv: s[0][1], duu: s[2][0], duv: s[1][1], dvv: s[0][2] }
    }

    /// Evaluator for repeated first-order evaluation along one iso-v line
    /// of a non-rational surface (None for rational / very high degree).
    /// The v basis is computed once; `segment` collapses the v direction
    /// for one u knot span so each `eval1` costs only a degree-p u basis.
    pub fn vline(&self, v: f64) -> Option<VLine<'_>> {
        if self.weights.is_some() || self.deg_u >= 12 || self.deg_v >= 12 {
            return None;
        }
        let (_, (v0, v1)) = self.domain();
        let v = v.clamp(v0, v1);
        let sv = find_span(self.nv - 1, self.deg_v, v, &self.knots_v);
        let nv = basis::ders_basis_funs2(sv, v, self.deg_v, 1, &self.knots_v);
        Some(VLine { s: self, sv, nv })
    }

    /// Unit normal Su × Sv (zero vector at degenerate points).
    pub fn normal(&self, u: f64, v: f64) -> Vec3 {
        let d = self.derivs2(u, v);
        let c = d.du.cross(d.dv);
        let (a, b) = (d.du.len(), d.dv.len());
        let sc = a.max(b);
        // a regular point, or a degenerate one with nothing better to offer
        if c.len() > 1e-9 * sc * sc || !(sc > 0.0) {
            return c.normalize();
        }
        self.pole_normal(u, v, &d)
    }

    /// Limit of the unit normal at a pole (a collapsed row or column of control points: one of
    /// `S_u`, `S_v` vanishes along the edge). Approaching the edge from the interior,
    /// `S_u ~ S_uv (v - v_pole)` (or the same with u and v swapped), so the limit direction is
    /// `S_uv x S_v` with the sign of the step towards the interior.
    fn pole_normal(&self, u: f64, v: f64, d: &SurfDerivs) -> Vec3 {
        let ((u0, u1), (v0, v1)) = self.domain();
        let hu = if u < 0.5 * (u0 + u1) { 1.0 } else { -1.0 };
        let hv = if v < 0.5 * (v0 + v1) { 1.0 } else { -1.0 };
        let (a, b) = (d.du.len(), d.dv.len());
        let c = if a <= b { d.duv.cross(d.dv) * hv } else { d.du.cross(d.duv) * hu };
        if c.len() > 1e-9 * a.max(b).max(d.duv.len()).powi(2) {
            return c.normalize();
        }
        // the corner of a doubly collapsed patch: step into the interior
        let (su, sv) = (1e-4 * (u1 - u0) * hu, 1e-4 * (v1 - v0) * hv);
        let n = self.derivs2(u + su, v + sv);
        n.du.cross(n.dv).normalize()
    }

    fn column(&self, j: usize) -> BSplineCurve {
        BSplineCurve {
            degree: self.deg_u,
            knots: self.knots_u.clone(),
            ctrl: (0..self.nu).map(|i| self.cp(i, j)).collect(),
            weights: self.weights.as_ref().map(|_| (0..self.nu).map(|i| self.weight(i, j)).collect()),
        }
    }

    fn row(&self, i: usize) -> BSplineCurve {
        BSplineCurve {
            degree: self.deg_v,
            knots: self.knots_v.clone(),
            ctrl: (0..self.nv).map(|j| self.cp(i, j)).collect(),
            weights: self.weights.as_ref().map(|_| (0..self.nv).map(|j| self.weight(i, j)).collect()),
        }
    }

    /// Iso-parametric curve at fixed `u` (a curve in v). Exact.
    pub fn iso_u(&self, u: f64) -> BSplineCurve {
        // Evaluate each v-column... i.e. collapse u: for each j, homogeneous
        // point of the u-curve through column j at u.
        let mut ctrl = Vec::with_capacity(self.nv);
        let mut ws = Vec::with_capacity(self.nv);
        for j in 0..self.nv {
            let c = self.column(j);
            let h = hom_eval(&c, u);
            ctrl.push(h.0);
            ws.push(h.1);
        }
        let weights = if self.weights.is_some() { Some(ws) } else { None };
        BSplineCurve { degree: self.deg_v, knots: self.knots_v.clone(), ctrl, weights }
    }

    /// Iso-parametric curve at fixed `v` (a curve in u). Exact.
    pub fn iso_v(&self, v: f64) -> BSplineCurve {
        let mut ctrl = Vec::with_capacity(self.nu);
        let mut ws = Vec::with_capacity(self.nu);
        for i in 0..self.nu {
            let h = hom_eval(&self.row(i), v);
            ctrl.push(h.0);
            ws.push(h.1);
        }
        let weights = if self.weights.is_some() { Some(ws) } else { None };
        BSplineCurve { degree: self.deg_u, knots: self.knots_u.clone(), ctrl, weights }
    }

    /// Insert knot `u` `r` times in the u direction.
    pub fn insert_knot_u(&self, u: f64, r: usize) -> Self {
        let cols: Vec<BSplineCurve> = (0..self.nv).map(|j| self.column(j).insert_knot(u, r)).collect();
        let nu = cols[0].ctrl.len();
        let mut ctrl = vec![Vec3::ZERO; nu * self.nv];
        let mut ws = vec![1.0; nu * self.nv];
        for (j, c) in cols.iter().enumerate() {
            for i in 0..nu {
                ctrl[i * self.nv + j] = c.ctrl[i];
                ws[i * self.nv + j] = c.weight(i);
            }
        }
        BSplineSurface {
            knots_u: cols[0].knots.clone(),
            nu,
            ctrl,
            weights: self.weights.as_ref().map(|_| ws),
            ..self.clone()
        }
    }

    /// Insert knot `v` `r` times in the v direction.
    pub fn insert_knot_v(&self, v: f64, r: usize) -> Self {
        self.transposed().insert_knot_u(v, r).transposed()
    }

    /// Swap u and v.
    pub fn transposed(&self) -> Self {
        let mut ctrl = Vec::with_capacity(self.ctrl.len());
        let mut ws = Vec::with_capacity(self.ctrl.len());
        for j in 0..self.nv {
            for i in 0..self.nu {
                ctrl.push(self.cp(i, j));
                ws.push(self.weight(i, j));
            }
        }
        BSplineSurface {
            deg_u: self.deg_v,
            deg_v: self.deg_u,
            knots_u: self.knots_v.clone(),
            knots_v: self.knots_u.clone(),
            nu: self.nv,
            nv: self.nu,
            ctrl,
            weights: self.weights.as_ref().map(|_| ws),
        }
    }

    /// Parameters of the closest point to `q` (grid seed + 2D Newton).
    pub fn closest_param(&self, q: Vec3) -> (f64, f64) {
        self.closest_param_fp(self.fingerprint(), q)
    }

    /// [`closest_param`](Self::closest_param) with the surface's [`fingerprint`](Self::fingerprint)
    /// computed by the caller (one pass over the control net per call otherwise: a polyline of
    /// hundreds of points projected onto one surface pays it once).
    pub fn closest_param_fp(&self, fp: u64, q: Vec3) -> (f64, f64) {
        // the same foot point is asked for again and again (every loop of an
        // arrangement re-samples its start; each fragment of a face re-projects
        // the same edges): memoised per (surface, point)
        use std::cell::RefCell;
        use std::collections::HashMap;
        // (two generations of 2^16 entries: when the current one is full it becomes the old one, and an old
        // entry asked for again moves back, so a turnover keeps the points still in use; clearing the whole
        // table at once made the next operation re-project every point of its faces)
        type Gen = HashMap<(u64, u64, u64, u64), (f64, f64)>;
        thread_local! {
            static MEMO: RefCell<(Gen, Gen)> = RefCell::new((HashMap::new(), HashMap::new()));
        }
        fn insert(m: &mut (Gen, Gen), key: (u64, u64, u64, u64), r: (f64, f64)) {
            if m.0.len() >= 1 << 16 {
                m.1 = std::mem::take(&mut m.0);
            }
            m.0.insert(key, r);
        }
        let key = (fp, q.x.to_bits(), q.y.to_bits(), q.z.to_bits());
        let hit = MEMO.with(|m| {
            let mut m = m.borrow_mut();
            if let Some(r) = m.0.get(&key) {
                return Some(*r);
            }
            let r = m.1.remove(&key)?;
            insert(&mut m, key, r);
            Some(r)
        });
        if let Some(r) = hit {
            return r;
        }
        let r = self.closest_param_within_with(&self.cp_prep(fp), q, f64::INFINITY).unwrap_or_else(|| {
            let ((u0, _), (v0, _)) = self.domain();
            (u0, v0)
        });
        MEMO.with(|m| insert(&mut m.borrow_mut(), key, r));
        r
    }

    /// Hash of the whole surface (degrees, knots, control net, weights).
    /// Memoised per object by a cheap signature (address, dimensions, an
    /// additive checksum of every coordinate, sampled points, knots): the
    /// control net of a big fitted surface is otherwise hashed on every
    /// cached query.
    pub fn fingerprint(&self) -> u64 {
        use std::cell::RefCell;
        use std::collections::HashMap;
        thread_local! {
            static QUICK: RefCell<HashMap<[u64; 4], u64>> = RefCell::new(HashMap::new());
        }
        let mut sums = [0u64; 4];
        for (i, c) in self.ctrl.iter().enumerate() {
            let l = i & 3;
            sums[l] = sums[l].wrapping_add(c.x.to_bits()).wrapping_add(c.y.to_bits().rotate_left(21)).wrapping_add(c.z.to_bits().rotate_left(42));
        }
        let mut sig = 0xcbf29ce484222325u64;
        let mut mix = |x: u64| {
            sig = (sig ^ x).wrapping_mul(0x100000001b3);
            sig ^= sig >> 29;
        };
        mix(self.ctrl.as_ptr() as usize as u64);
        mix(self.deg_u as u64 | (self.deg_v as u64) << 8 | (self.nu as u64) << 16 | (self.nv as u64) << 40);
        mix(self.weights.as_ref().map_or(0, |w| w.as_ptr() as usize as u64 ^ w.len() as u64));
        for &k in self.knots_u.iter().chain(&self.knots_v) {
            mix(k.to_bits());
        }
        let n = self.ctrl.len();
        for j in 0..16usize.min(n) {
            let c = self.ctrl[j * n / 16.max(1)];
            mix(c.x.to_bits() ^ c.y.to_bits().rotate_left(7) ^ c.z.to_bits().rotate_left(13));
        }
        if let Some(w) = &self.weights {
            let mut ws = 0u64;
            for x in w {
                ws = ws.wrapping_add(x.to_bits());
            }
            mix(ws);
        }
        let key = [sig, sums[0], sums[1] ^ sums[2].rotate_left(9), sums[3]];
        if let Some(fp) = QUICK.with(|m| m.borrow().get(&key).copied()) {
            return fp;
        }
        let fp = self.fingerprint_full();
        QUICK.with(|m| {
            let mut m = m.borrow_mut();
            if m.len() >= 4096 {
                m.clear();
            }
            m.insert(key, fp);
        });
        fp
    }

    /// The content hash itself: four independent lanes over the whole surface.
    fn fingerprint_full(&self) -> u64 {
        let mut lanes: [u64; 4] = [0xcbf29ce484222325, 0x9e3779b97f4a7c15, 0xc2b2ae3d27d4eb4f, 0x165667b19e3779f9];
        let mut n = 0usize;
        let mut mix = |x: u64| {
            let l = &mut lanes[n & 3];
            *l = (*l ^ x).wrapping_mul(0x100000001b3);
            *l ^= *l >> 29;
            n += 1;
        };
        mix(self.deg_u as u64 | (self.deg_v as u64) << 8 | (self.nu as u64) << 16 | (self.nv as u64) << 40);
        for &k in self.knots_u.iter().chain(&self.knots_v) {
            mix(k.to_bits());
        }
        for c in &self.ctrl {
            mix(c.x.to_bits());
            mix(c.y.to_bits());
            mix(c.z.to_bits());
        }
        if let Some(w) = &self.weights {
            for x in w {
                mix(x.to_bits());
            }
        }
        lanes.iter().fold(0u64, |h, &l| (h ^ l).wrapping_mul(0x9e3779b97f4a7c15).rotate_left(23))
    }

    /// [`closest_param`](Self::closest_param), but `None` when the surface is
    /// not within `maxd` of `q`: cells whose control-net box is farther away
    /// are never sampled, so a far query costs one pass over the boxes.
    pub fn closest_param_within(&self, q: Vec3, maxd: f64) -> Option<(f64, f64)> {
        self.closest_param_within_with(&self.cp_prep(self.fingerprint()), q, maxd)
    }

    /// Prepared seed grid of this surface, for many bounded queries
    /// ([`closest_param_within_with`](Self::closest_param_within_with)).
    pub fn closest_prep(&self) -> std::rc::Rc<CpPrep> {
        self.cp_prep(self.fingerprint())
    }

    pub fn closest_param_within_with(&self, prep: &CpPrep, q: Vec3, maxd: f64) -> Option<(f64, f64)> {
        let ((u0, _), (v0, _)) = self.domain();
        let boxes = &prep.boxes;
        // (exact) farther than `maxd` from the box of the whole net
        if maxd.is_finite() {
            let (lo, hi) = prep.hull;
            let g = Vec3::new((lo.x - q.x).max(q.x - hi.x).max(0.0), (lo.y - q.y).max(q.y - hi.y).max(0.0), (lo.z - q.z).max(q.z - hi.z).max(0.0));
            if g.len() > maxd {
                return None;
            }
        }
        // (the grid samples are up to a cell away from the foot point: the
        // sample bound is looser than `maxd`, the final distance is checked)
        let best = (maxd + prep.cell_scale, u0, v0);
        // The seed grid is the same as a brute-force scan, but samples in
        // cells whose control-net box is farther from `q` than the best
        // sample so far are skipped (convex hull property: exact pruning).
        // Long multi-span surfaces (screw sweeps) made the plain scan
        // O(spans) per call.
        // Per-query working storage, kept across calls (the SSI seeding asks
        // thousands of bounded queries per surface: no allocation per query;
        // taken out of the slot for the call so a nested query gets its own)
        let mut sc = SCRATCH.with(|s| std::mem::take(&mut *s.borrow_mut()));
        // An unbounded query visits the nearest cells first, then every other cell whose box is
        // nearer than the best sample so far. When the 9th nearest box is no nearer than the best
        // sample of the first 8 cells, every later visit does nothing: the first 8 cells (found
        // through the tiles, without the distance of every cell) give the very same result.
        if maxd.is_infinite() && prep.tiles.len() > 1 {
            if let Some(near) = self.nearest_cells(prep, q, &mut sc) {
                if let Ok(r) = self.closest_param_within_scan(prep, q, maxd, &mut sc, best, Some(&near)) {
                    SCRATCH.with(|s| *s.borrow_mut() = sc);
                    return r;
                }
            }
        }
        sc.lb.clear();
        sc.lb.extend(boxes.iter().map(|&(lo, hi)| box_dist(lo, hi, q)));
        let r = self.closest_param_within_scan(prep, q, maxd, &mut sc, best, None).unwrap_or(None);
        SCRATCH.with(|s| *s.borrow_mut() = sc);
        r
    }

    /// The unbounded query by the full scan only (what `closest_param_within_with` must reproduce),
    /// and whether the nearest-cells path would have answered it.
    #[cfg(test)]
    fn closest_param_full_scan(&self, q: Vec3) -> (Option<(f64, f64)>, bool) {
        let prep = self.closest_prep();
        let ((u0, _), (v0, _)) = self.domain();
        let best = (f64::INFINITY + prep.cell_scale, u0, v0);
        let mut sc = CpScratch::default();
        let quick = self.nearest_cells(&prep, q, &mut sc).is_some_and(|near| self.closest_param_within_scan(&prep, q, f64::INFINITY, &mut sc, best, Some(&near)).is_ok());
        sc.lb.clear();
        sc.lb.extend(prep.boxes.iter().map(|&(lo, hi)| box_dist(lo, hi, q)));
        (self.closest_param_within_scan(&prep, q, f64::INFINITY, &mut sc, best, None).unwrap(), quick)
    }

    /// The `FIRST_CELLS` nearest cells to `q` by (box distance, cell index), in that order, into
    /// `sc.cells` (as the scan builds its cell list), and the distance of the next one; the box
    /// distance of every tile stays in `sc.lb`. Tiles whose box is farther than the current last
    /// candidate are skipped (exact: a cell is no nearer than its tile). `None` when the surface has
    /// too few cells or a distance is NaN.
    fn nearest_cells(&self, prep: &CpPrep, q: Vec3, sc: &mut CpScratch) -> Option<Nearest> {
        const N: usize = FIRST_CELLS + 1;
        if prep.cells.len() < 256 {
            return None;
        }
        let cv = prep.cv;
        sc.lb.clear();
        sc.lb.extend(prep.tiles.iter().map(|&(lo, hi)| box_dist(lo, hi, q)));
        if sc.lb.iter().any(|d| d.is_nan()) {
            return None;
        }
        // the nearest tile first (a tight candidate bound early), then the rest in order
        let first = (0..sc.lb.len()).fold(0, |b, t| if sc.lb[t] < sc.lb[b] { t } else { b });
        let mut top = [(f64::INFINITY, usize::MAX); N];
        let mut n = 0usize;
        for t in std::iter::once(first).chain((0..prep.tiles.len()).filter(|&t| t != first)) {
            if n == N && sc.lb[t] > top[N - 1].0 {
                continue;
            }
            for &ci in &prep.tile_cells[prep.tile_start[t] as usize..prep.tile_start[t + 1] as usize] {
                let ci = ci as usize;
                let (_, iu, iv, _, _) = prep.cells[ci];
                let (lo, hi) = prep.boxes[iu * cv + iv];
                let d = box_dist(lo, hi, q);
                if d.is_nan() {
                    return None;
                }
                let less = |a: (f64, usize), b: (f64, usize)| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).is_lt();
                if n == N && !less((d, ci), top[N - 1]) {
                    continue;
                }
                let mut j = n.min(N - 1);
                while j > 0 && less((d, ci), top[j - 1]) {
                    top[j] = top[j - 1];
                    j -= 1;
                }
                top[j] = (d, ci);
                n = (n + 1).min(N);
            }
        }
        if n < N {
            return None;
        }
        sc.cells.clear();
        sc.cells.extend(top[..FIRST_CELLS].iter().map(|&(d, ci)| {
            let (su, iu, iv, alo, ahi) = prep.cells[ci];
            (d, su, iu, iv, alo, ahi)
        }));
        Some(Nearest { chosen: std::array::from_fn(|j| top[j].1), lb9: top[N - 1].0 })
    }

    /// The seed scan and Newton polish. `pre`: `sc.cells` holds only the nearest cells and `sc.lb`
    /// the tile distances (`nearest_cells`): the later cells are visited only when the outcome does
    /// not depend on the order the full scan would visit them in, else `Err` (the caller then runs
    /// the full scan).
    fn closest_param_within_scan(&self, prep: &CpPrep, q: Vec3, maxd: f64, sc: &mut CpScratch, mut best: (f64, f64, f64), pre: Option<&Nearest>) -> Result<Option<(f64, f64)>, ()> {
        let ((u0, u1), (v0, v1)) = self.domain();
        let (pu, pv) = (self.deg_u, self.deg_v);
        let (nu, cv, vs, bv, cols, ubas) = (prep.nu, prep.cv, &prep.vs, &prep.bv, &prep.cols, &prep.ubas);
        let mut found = false;
        let CpScratch { lb, cells, order, cell_best } = sc;
        // (exact) no cell's control-net box within `maxd`: the surface lies in
        // the union of the boxes, so no point of it is that near
        if pre.is_none() && maxd.is_finite() && lb.iter().all(|&d| d > maxd) {
            return Ok(None);
        }
        // grid evaluation without the derivative machinery (non-rational)
        let fast = self.weights.is_none();
        if pre.is_none() {
            cells.clear();
            cells.extend(prep.cells.iter().map(|&(su, iu, iv, alo, ahi)| (lb[iu * cv + iv], su, iu, iv, alo, ahi)));
        }
        let cells: &[(f64, usize, usize, usize, usize, usize)] = cells;
        let u_at = |a: usize| u0 + (u1 - u0) * a as f64 / nu as f64;
        let found_flag = std::cell::Cell::new(false);
        // best grid sample of every visited cell (multi-start seeds)
        cell_best.clear();
        let cell_best: std::cell::RefCell<&mut Vec<(f64, f64, f64, f64)>> = std::cell::RefCell::new(cell_best);
        let grid = self.grid_for(prep);
        let nv1 = vs.len();
        let visit = |c: &(f64, usize, usize, usize, usize, usize), best: &mut (f64, f64, f64)| {
            let (clb, su, _iu, iv, alo, ahi) = *c;
            if clb >= best.0 || cols[iv].is_empty() {
                return;
            }
            let mut local = (f64::INFINITY, 0.0, 0.0);
            // (one buffer per cell: a row overwrites the entries it reads)
            let mut tcol = [Vec3::ZERO; 16];
            for a in alo..=ahi {
                let u = u_at(a);
                if ubas[a].0 != su {
                    continue;
                }
                let bu = &ubas[a].1;
                // non-rational: collapse the u direction once per (u, cell)
                // (every v sample of the cell lies in the same v span)
                if fast && pv < 16 && grid.is_none() {
                    if let Some(&k0) = cols[iv].first() {
                        let sv = bv[k0].0;
                        for j in 0..=pv {
                            let mut t = Vec3::ZERO;
                            for (i, &wu) in bu.iter().enumerate() {
                                t = t + self.cp(su - pu + i, sv - pv + j) * wu;
                            }
                            tcol[j] = t;
                        }
                    }
                }
                // bounded query (no multi-start polish reads `local`): a row
                // whose control-point box is no nearer than the best sample
                // cannot improve it
                if fast && pv < 16 && maxd.is_finite() {
                    let (lo, hi) = match grid {
                        Some((_, rows)) => rows[a * cv + iv],
                        None => {
                            let (mut lo, mut hi) = (tcol[0], tcol[0]);
                            for t in &tcol[1..=pv] {
                                lo = lo.min(*t);
                                hi = hi.max(*t);
                            }
                            (lo, hi)
                        }
                    };
                    let g = Vec3::new((lo.x - q.x).max(q.x - hi.x).max(0.0), (lo.y - q.y).max(q.y - hi.y).max(0.0), (lo.z - q.z).max(q.z - hi.z).max(0.0));
                    if g.len() >= best.0 {
                        continue;
                    }
                }
                // the common case, on the precomputed grid: the same test and updates as below, with the
                // bound of the skip test kept between samples (it changes only when `best` or `local` does)
                if let (true, Some((pts, _))) = (fast && pv < 16, grid) {
                    let bound = |best: &(f64, f64, f64), local: &(f64, f64, f64)| {
                        let lim = best.0.max(local.0);
                        lim * lim * (1.0 + 1e-9)
                    };
                    let mut lim2 = bound(best, &local);
                    for &k in &cols[iv] {
                        let d2 = (pts[a * nv1 + k] - q).len2();
                        if d2 > lim2 {
                            continue;
                        }
                        let (d, v) = (d2.sqrt(), vs[k].0);
                        if d < local.0 {
                            local = (d, u, v);
                        }
                        if d < best.0 {
                            *best = (d, u, v);
                            found_flag.set(true);
                        }
                        lim2 = bound(best, &local);
                    }
                    continue;
                }
                for &k in &cols[iv] {
                    let v = vs[k].0;
                    let d = if fast && pv < 16 {
                        let p = match grid {
                            Some((pts, _)) => pts[a * nv1 + k],
                            None => {
                                let bvk = &bv[k].1;
                                let mut p = Vec3::ZERO;
                                for (j, &wv) in bvk.iter().enumerate() {
                                    p = p + tcol[j] * wv;
                                }
                                p
                            }
                        };
                        // clearly no nearer than the best / local samples: skip the root
                        let d2 = (p - q).len2();
                        let lim = best.0.max(local.0);
                        if d2 > lim * lim * (1.0 + 1e-9) {
                            continue;
                        }
                        d2.sqrt()
                    } else if fast {
                        let (sv, ref bvk) = bv[k];
                        let mut p = Vec3::ZERO;
                        for (i, &wu) in bu.iter().enumerate() {
                            for (j, &wv) in bvk.iter().enumerate() {
                                p = p + self.cp(su - pu + i, sv - pv + j) * (wu * wv);
                            }
                        }
                        p.dist(q)
                    } else {
                        self.eval(u, v).dist(q)
                    };
                    if d < local.0 {
                        local = (d, u, v);
                    }
                    if d < best.0 {
                        *best = (d, u, v);
                        found_flag.set(true);
                    }
                }
            }
            if local.0.is_finite() {
                cell_best.borrow_mut().push((local.0, local.1, local.2, clb));
            }
        };
        // seed from the nearest few cells (a cell can hold no grid point),
        // then every other cell whose box can still beat the best sample
        order.clear();
        order.extend(0..cells.len());
        let cmp = |x: &usize, y: &usize| cells[*x].0.total_cmp(&cells[*y].0).then(x.cmp(y));
        let k = 8.min(order.len());
        if k > 0 && k < order.len() {
            order.select_nth_unstable_by(k - 1, cmp);
        }
        order[..k].sort_by(cmp);
        for &i in order.iter() {
            visit(&cells[i], &mut best);
        }
        if let Some(near) = pre {
            if !(near.lb9 >= best.0) {
                // The full scan visits the other cells in the order of its partial sort, each one only
                // while its box is nearer than the best sample so far. That order does not matter when
                // every cell nearer than the best sample of the nearest cells (set T) is also nearer
                // than the final best sample (so all of T is visited whatever the order), and no two
                // cells of T share their best sample distance (the same best sample, and the same order
                // of their multi-start seeds after the stable sort by distance). Then visit T in any
                // order; else the full scan decides.
                if !best.0.is_finite() {
                    return Err(());
                }
                let b8 = best.0;
                let n8 = cell_best.borrow().len();
                let mut t_lbs: Vec<f64> = vec![];
                for t in 0..prep.tiles.len() {
                    if !(lb[t] < b8) {
                        continue;
                    }
                    for &ci in &prep.tile_cells[prep.tile_start[t] as usize..prep.tile_start[t + 1] as usize] {
                        let ci = ci as usize;
                        let (su, iu, iv, alo, ahi) = prep.cells[ci];
                        let (lo, hi) = prep.boxes[iu * cv + iv];
                        let clb = box_dist(lo, hi, q);
                        if !(clb < b8) || cols[iv].is_empty() || near.chosen.contains(&ci) {
                            continue;
                        }
                        // (with an infinite bound the visit records the cell's own best sample)
                        visit(&(clb, su, iu, iv, alo, ahi), &mut (f64::INFINITY, 0.0, 0.0));
                        t_lbs.push(clb);
                    }
                }
                let cb = cell_best.borrow();
                let extra = &cb[n8..];
                let fin = extra.iter().fold(best, |b, e| if e.0 < b.0 { (e.0, e.1, e.2) } else { b });
                let mut ds: Vec<f64> = extra.iter().map(|e| e.0).collect();
                ds.sort_by(f64::total_cmp);
                if t_lbs.iter().any(|&l| !(l < fin.0)) || ds.windows(2).any(|w| w[0] == w[1]) {
                    return Err(());
                }
                drop(cb);
                best = fin;
            }
        }
        found = found || found_flag.get();
        if !found {
            return Ok(None);
        }
        // (the polish is a function of the seed alone: the nearest cells' best samples, the restarts from the
        // opposite edge of a pinned foot, repeat starts within one query)
        let done: std::cell::RefCell<Vec<((u64, u64), (f64, f64))>> = Default::default();
        let newton0 = |seed: (f64, f64)| {
            let key = (seed.0.to_bits(), seed.1.to_bits());
            if let Some(r) = done.borrow().iter().find(|e| e.0 == key) {
                return r.1;
            }
            let bc = std::cell::RefCell::new(Basis2Cache::default());
            let r = crate::surface::newton_surface(|u, v| self.derivs2_cached(u, v, &mut bc.borrow_mut()), q, seed, ((u0, u1), (v0, v1)));
            done.borrow_mut().push((key, r));
            r
        };
        // a surface closed on itself (a tube: u = 0 and u = 2 pi are one
        // curve): a foot point just across the closing isoline is reached
        // by restarting from the opposite domain edge
        let newton = |seed: (f64, f64)| {
            let r = newton0(seed);
            let d = self.eval(r.0, r.1).dist(q);
            if d <= 1e-9 {
                return r;
            }
            let (mut best, mut bd) = (r, d);
            let ends = |x: f64, lo: f64, hi: f64| -> Option<f64> {
                let tol = 1e-12 * (1.0 + hi.abs());
                if (x - lo).abs() <= tol {
                    Some(hi)
                } else if (x - hi).abs() <= tol {
                    Some(lo)
                } else {
                    None
                }
            };
            let mut tries = [(0.0f64, 0.0f64); 3];
            let mut nt = 0usize;
            if let Some(u) = ends(r.0, u0, u1) {
                tries[nt] = (u, r.1);
                nt += 1;
            }
            if let Some(v) = ends(r.1, v0, v1) {
                tries[nt] = (r.0, v);
                nt += 1;
            }
            if nt == 2 {
                tries[2] = (tries[0].0, tries[1].1);
                nt = 3;
            }
            for &t in &tries[..nt] {
                let c = newton0(t);
                let dc = self.eval(c.0, c.1).dist(q);
                if dc < bd {
                    best = c;
                    bd = dc;
                }
            }
            best
        };
        let first = newton((best.1, best.2));
        let in_range = |uv: (f64, f64)| self.eval(uv.0, uv.1).dist(q) <= maxd;
        // Near the surface (a point ON it, say) the best grid sample can sit
        // on another sheet of a self-overlapping sweep: also polish the best
        // samples of the other cells, keep the nearest result (not for the
        // bounded query, which the SSI seeding calls thousands of times).
        let fd = self.eval(first.0, first.1).dist(q);
        if maxd.is_infinite() && fd > 1e-9 && best.0 < 4.0 * fd.max(1e-3) + self.cell_scale() {
            let cb = cell_best.into_inner();
            cb.sort_by(|a, b| a.0.total_cmp(&b.0));
            let mut res = (fd, first);
            for c in cb.iter().take(6) {
                if (c.1 - best.1).abs() < 1e-12 && (c.2 - best.2).abs() < 1e-12 {
                    continue;
                }
                let uv = newton((c.1, c.2));
                let d = self.eval(uv.0, uv.1).dist(q);
                if d < res.0 {
                    res = (d, uv);
                }
                if res.0 <= 1e-9 {
                    break;
                }
            }
            return Ok(in_range(res.1).then_some(res.1));
        }
        Ok(in_range(first).then_some(first))
    }

    /// The seed grid points and row boxes of `prep`, once the surface has been asked for often enough that
    /// they repay building (the scan computes the very same numbers on the fly, bit for bit, without them).
    fn grid_for<'a>(&self, prep: &'a CpPrep) -> Option<&'a (Vec<Vec3>, Vec<(Vec3, Vec3)>)> {
        if !prep.grid_ok {
            return None;
        }
        if let Some(g) = prep.grid.get() {
            return Some(g);
        }
        let n = prep.uses.get().saturating_add(1);
        prep.uses.set(n);
        // (the grid costs about as many point evaluations as it has points; a query visits a few hundred)
        let after = 16 + ((prep.nu + 1) * (prep.vs.len())) as u32 / 1000;
        if n < after {
            return None;
        }
        Some(prep.grid.get_or_init(|| self.build_grid(prep)))
    }

    /// The seed grid points `[a * (nv + 1) + k]` and the box of each (u column, v span) row `[a * cv + iv]`.
    fn build_grid(&self, prep: &CpPrep) -> (Vec<Vec3>, Vec<(Vec3, Vec3)>) {
        let (nu, cv, bv, cols, ubas) = (prep.nu, prep.cv, &prep.bv, &prep.cols, &prep.ubas);
        let nv = prep.vs.len() - 1;
        let (pu, pv) = (self.deg_u, self.deg_v);
        let mut pts = vec![Vec3::ZERO; (nu + 1) * (nv + 1)];
        let mut rows = vec![(Vec3::ZERO, Vec3::ZERO); (nu + 1) * cv.max(1)];
        for a in 0..=nu {
            let (su, ref bu) = ubas[a];
            for (iv, col) in cols.iter().enumerate() {
                let Some(&k0) = col.first() else { continue };
                let sv = bv[k0].0;
                let mut tcol = [Vec3::ZERO; 16];
                for j in 0..=pv {
                    let mut t = Vec3::ZERO;
                    for (i, &wu) in bu.iter().enumerate() {
                        t = t + self.cp(su - pu + i, sv - pv + j) * wu;
                    }
                    tcol[j] = t;
                }
                let (mut lo, mut hi) = (tcol[0], tcol[0]);
                for t in &tcol[1..=pv] {
                    lo = lo.min(*t);
                    hi = hi.max(*t);
                }
                rows[a * cv + iv] = (lo, hi);
                for &k in col {
                    let mut p = Vec3::ZERO;
                    for (j, &wv) in bv[k].1.iter().enumerate() {
                        p = p + tcol[j] * wv;
                    }
                    pts[a * (nv + 1) + k] = p;
                }
            }
        }
        (pts, rows)
    }

    /// Per-surface constants of the closest-point seed grid (spans, cell
    /// boxes, v-sample bases, per-u bases), cached by a hash of the surface:
    /// a query then allocates nothing per sample.
    fn cp_prep(&self, key: u64) -> std::rc::Rc<CpPrep> {
        use std::cell::RefCell;
        use std::rc::Rc;
        thread_local! {
            static CACHE: RefCell<Vec<(u64, Rc<CpPrep>)>> = const { RefCell::new(Vec::new()) };
        }
        if let Some(r) = CACHE.with(|c| c.borrow().iter().find(|e| e.0 == key).map(|e| e.1.clone())) {
            return r;
        }
        let ((u0, u1), (v0, v1)) = self.domain();
        let cells_n = (basis::distinct_count(&self.knots_u) - 1) * (basis::distinct_count(&self.knots_v) - 1);
        // long multi-span surfaces (screw sweeps): a coarser sample grid per
        // cell, the Newton polish recovers the foot point
        let dens = if cells_n >= 256 { 1 } else { 3 };
        let nu = ((basis::distinct_count(&self.knots_u) - 1) * (self.deg_u + 1) * dens).max(12);
        let nv = ((basis::distinct_count(&self.knots_v) - 1) * (self.deg_v + 1) * dens).max(12);
        let (pu, pv) = (self.deg_u, self.deg_v);
        let spans = |kn: &[f64], p: usize, n: usize| -> Vec<usize> {
            let mut idx = vec![usize::MAX; kn.len()];
            let mut k = 0;
            for s in p..n {
                if kn[s] < kn[s + 1] {
                    idx[s] = k;
                    k += 1;
                }
            }
            idx
        };
        let (iu_of, iv_of) = (spans(&self.knots_u, pu, self.nu), spans(&self.knots_v, pv, self.nv));
        let (cu, cv) = (iu_of.iter().filter(|&&i| i != usize::MAX).count(), iv_of.iter().filter(|&&i| i != usize::MAX).count());
        let boxes = self.cell_boxes(&iu_of, &iv_of, cu, cv);
        let fast = self.weights.is_none();
        let vs: Vec<(f64, usize)> = (0..=nv)
            .map(|b| {
                let v = v0 + (v1 - v0) * b as f64 / nv as f64;
                (v, iv_of[find_span(self.nv - 1, pv, v, &self.knots_v)])
            })
            .collect();
        let bv: Vec<(usize, Vec<f64>)> = if fast {
            vs.iter()
                .map(|&(v, _)| {
                    let s = find_span(self.nv - 1, pv, v, &self.knots_v);
                    (s, basis::basis_funs(s, v, pv, &self.knots_v))
                })
                .collect()
        } else {
            vec![]
        };
        let mut cols: Vec<Vec<usize>> = vec![vec![]; cv.max(1)];
        for (k, &(_, iv)) in vs.iter().enumerate() {
            cols[iv].push(k);
        }
        let span_u = (u1 - u0).max(f64::MIN_POSITIVE);
        let mut cells = Vec::with_capacity(cu * cv);
        for su in pu..self.nu {
            let iu = iu_of[su];
            if iu == usize::MAX {
                continue;
            }
            let (k0, k1) = (self.knots_u[su], self.knots_u[su + 1]);
            let alo = ((((k0 - u0) / span_u) * nu as f64).floor() - 1.0).max(0.0) as usize;
            let ahi = ((((k1 - u0) / span_u) * nu as f64).ceil() + 1.0).min(nu as f64).max(0.0) as usize;
            for sv in pv..self.nv {
                let iv = iv_of[sv];
                if iv != usize::MAX {
                    cells.push((su, iu, iv, alo, ahi));
                }
            }
        }
        let ubas: Vec<(usize, Vec<f64>)> = (0..=nu)
            .map(|a| {
                let u = u0 + (u1 - u0) * a as f64 / nu as f64;
                let su = find_span(self.nu - 1, pu, u, &self.knots_u);
                (su, if fast { basis::basis_funs(su, u, pu, &self.knots_u) } else { vec![] })
            })
            .collect();
        let hull = boxes.iter().filter(|b| b.0.x <= b.1.x).fold((Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY)), |a, b| (a.0.min(b.0), a.1.max(b.1)));
        let grid_ok = fast && pv < 16 && (nu + 1) * (nv + 1) <= 1 << 18;
        // tiles of 4 x 4 cells (by span index) for `nearest_cells`
        let (tu, tv) = (cu.div_ceil(4).max(1), cv.div_ceil(4).max(1));
        let mut members: Vec<Vec<u32>> = vec![vec![]; tu * tv];
        for (ci, &(_, iu, iv, _, _)) in cells.iter().enumerate() {
            members[(iu / 4) * tv + iv / 4].push(ci as u32);
        }
        let (mut tiles, mut tile_start, mut tile_cells) = (vec![], vec![0u32], vec![]);
        for m in members.into_iter().filter(|m| !m.is_empty()) {
            let inf = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let (mut lo, mut hi) = (inf, -inf);
            for &ci in &m {
                let (_, iu, iv, _, _) = cells[ci as usize];
                let b = boxes[iu * cv + iv];
                if b.0.x <= b.1.x {
                    lo = lo.min(b.0);
                    hi = hi.max(b.1);
                }
            }
            tiles.push((lo, hi));
            tile_cells.extend(m);
            tile_start.push(tile_cells.len() as u32);
        }
        let prep = Rc::new(CpPrep { nu, cv, boxes: (*boxes).clone(), vs, bv, cols, cells, ubas, cell_scale: self.cell_scale(), hull, grid: Default::default(), grid_ok, uses: Default::default(), tiles, tile_start, tile_cells });
        CACHE.with(|c| {
            let mut c = c.borrow_mut();
            if c.len() >= 24 {
                c.remove(0);
            }
            c.push((key, prep.clone()));
        });
        prep
    }

    /// Typical size of a knot-span cell (diagonal of the control-net box of
    /// the first span pair): scale of the "near the surface" test above.
    fn cell_scale(&self) -> f64 {
        let (pu, pv) = (self.deg_u, self.deg_v);
        let (mut lo, mut hi) = (self.cp(0, 0), self.cp(0, 0));
        for i in 0..=pu.min(self.nu - 1) {
            for j in 0..=pv.min(self.nv - 1) {
                lo = lo.min(self.cp(i, j));
                hi = hi.max(self.cp(i, j));
            }
        }
        (hi - lo).len()
    }

    /// Control-net boxes of the knot-span cells, `[iu * cv + iv]` (an empty
    /// box, lo > hi, for a cell index with no span). Long multi-span surfaces
    /// (screw sweeps) are queried thousands of times, so their boxes are kept
    /// in a small thread-local cache keyed by a hash of the whole surface.
    fn cell_boxes(&self, iu_of: &[usize], iv_of: &[usize], cu: usize, cv: usize) -> std::rc::Rc<Vec<(Vec3, Vec3)>> {
        use std::cell::RefCell;
        use std::rc::Rc;
        thread_local! {
            static CACHE: RefCell<Vec<(u64, Rc<Vec<(Vec3, Vec3)>>)>> = const { RefCell::new(Vec::new()) };
        }
        let big = cu * cv >= 256;
        let key = if big {
            let mut h: u64 = 0xcbf29ce484222325;
            let mut mix = |x: u64| {
                h = (h ^ x).wrapping_mul(0x100000001b3);
                h ^= h >> 29;
            };
            mix(self.deg_u as u64 | (self.deg_v as u64) << 8 | (self.nu as u64) << 16 | (self.nv as u64) << 40);
            for &k in self.knots_u.iter().chain(&self.knots_v) {
                mix(k.to_bits());
            }
            for c in &self.ctrl {
                mix(c.x.to_bits());
                mix(c.y.to_bits());
                mix(c.z.to_bits());
            }
            Some(h)
        } else {
            None
        };
        if let Some(k) = key {
            if let Some(r) = CACHE.with(|c| c.borrow().iter().find(|e| e.0 == k).map(|e| e.1.clone())) {
                return r;
            }
        }
        let (pu, pv) = (self.deg_u, self.deg_v);
        let inf = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut out = vec![(inf, -inf); cu * cv.max(1)];
        for su in pu..self.nu {
            let iu = iu_of[su];
            if iu == usize::MAX {
                continue;
            }
            for sv in pv..self.nv {
                let iv = iv_of[sv];
                if iv == usize::MAX {
                    continue;
                }
                let (mut lo, mut hi) = (inf, -inf);
                for i in su - pu..=su {
                    for j in sv - pv..=sv {
                        let c = self.cp(i, j);
                        lo = lo.min(c);
                        hi = hi.max(c);
                    }
                }
                out[iu * cv + iv] = (lo, hi);
            }
        }
        let out = Rc::new(out);
        if let Some(k) = key {
            CACHE.with(|c| {
                let mut c = c.borrow_mut();
                if c.len() >= 6 {
                    c.remove(0);
                }
                c.push((k, out.clone()));
            });
        }
        out
    }

    /// Apply a point map to the control net (exact for affine maps).
    pub fn map(&self, f: impl Fn(Vec3) -> Vec3) -> Self {
        BSplineSurface { ctrl: self.ctrl.iter().map(|&c| f(c)).collect(), ..self.clone() }
    }
}

/// Homogeneous evaluation: returns (Euclidean point, weight) — used to
/// build exact iso curves of rational surfaces.
/// See [`BSplineSurface::vline`].
#[derive(Clone, Copy)]
pub struct VLine<'a> {
    s: &'a BSplineSurface,
    sv: usize,
    nv: [[f64; 12]; 3],
}

/// A VLine restricted to the u knot span containing a given parameter.
pub struct VSegment<'a> {
    s: &'a BSplineSurface,
    su: usize,
    c: [Vec3; 12],
    d: [Vec3; 12],
}

impl<'a> VLine<'a> {
    /// Segment for the span containing `u`; None when `u` is outside the
    /// domain.
    pub fn segment(&self, u: f64) -> Option<VSegment<'a>> {
        let s = self.s;
        let ((u0, u1), _) = s.domain();
        if !(u >= u0 && u <= u1) {
            return None;
        }
        let (p, q) = (s.deg_u, s.deg_v);
        let su = find_span(s.nu - 1, p, u, &s.knots_u);
        let mut c = [Vec3::ZERO; 12];
        let mut d = [Vec3::ZERO; 12];
        for r in 0..=p {
            let i = su - p + r;
            let (mut a, mut b) = (Vec3::ZERO, Vec3::ZERO);
            for t in 0..=q {
                let cp = s.cp(i, self.sv - q + t);
                a = a + cp * self.nv[0][t];
                b = b + cp * self.nv[1][t];
            }
            c[r] = a;
            d[r] = b;
        }
        Some(VSegment { s, su, c, d })
    }
}

impl VSegment<'_> {
    /// (point, S_u, S_v) at `u` (within the segment's span).
    #[inline]
    pub fn eval1(&self, u: f64) -> (Vec3, Vec3, Vec3) {
        self.eval1_basis(&self.basis(u))
    }

    /// The u basis functions and first derivatives `eval1` uses at `u` (the same for every segment of
    /// this knot span, whatever the v line: a caller evaluating many v lines at the same `u` takes it once).
    #[inline]
    pub fn basis(&self, u: f64) -> [[f64; 12]; 3] {
        basis::ders_basis_funs2(self.su, u, self.s.deg_u, 1, &self.s.knots_u)
    }

    /// `eval1` with the u basis of [`basis`](Self::basis) given.
    #[inline]
    pub fn eval1_basis(&self, b: &[[f64; 12]; 3]) -> (Vec3, Vec3, Vec3) {
        let p = self.s.deg_u;
        let (mut pt, mut du, mut dv) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        for r in 0..=p {
            pt = pt + self.c[r] * b[0][r];
            du = du + self.c[r] * b[1][r];
            dv = dv + self.d[r] * b[0][r];
        }
        (pt, du, dv)
    }

    /// True when `[a, b]` lies inside this segment's knot span.
    pub fn covers(&self, a: f64, b: f64) -> bool {
        let k = &self.s.knots_u;
        let (lo, hi) = (k[self.su], k[self.su + 1]);
        a.min(b) >= lo - 1e-12 && a.max(b) <= hi + 1e-12
    }
}

fn hom_eval(c: &BSplineCurve, u: f64) -> (Vec3, f64) {
    let p = c.degree;
    let (a, b) = c.domain();
    let u = u.clamp(a, b);
    let span = find_span(c.ctrl.len() - 1, p, u, &c.knots);
    let n = basis::basis_funs(span, u, p, &c.knots);
    let mut pw = Vec3::ZERO;
    let mut w = 0.0;
    for j in 0..=p {
        let i = span - p + j;
        let wi = c.weight(i);
        pw = pw + c.ctrl[i] * (n[j] * wi);
        w += n[j] * wi;
    }
    (pw / w, w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;

    fn surf(rational: bool) -> BSplineSurface {
        let (nu, nv) = (4, 5);
        let mut ctrl = Vec::new();
        let mut ws = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                let (x, y) = (i as f64, j as f64);
                ctrl.push(Vec3::new(x, y, (x * 0.7).sin() * (y * 0.5).cos() + 0.1 * x * y));
                ws.push(1.0 + 0.3 * ((i + 2 * j) % 3) as f64);
            }
        }
        BSplineSurface::new(
            3,
            2,
            basis::clamped_uniform(nu, 3),
            vec![0., 0., 0., 0.3, 0.6, 1., 1., 1.],
            nu,
            nv,
            ctrl,
            if rational { Some(ws) } else { None },
        )
        .unwrap()
    }

    /// `derivs1_nr` is the first-order `derivs` bit for bit (also at knots and outside the domain).
    #[test]
    fn derivs1_nr_is_bit_identical() {
        let s = surf(false);
        assert!(surf(true).derivs1_nr(0.3, 0.3).is_none());
        let bits = |v: Vec3| (v.x.to_bits(), v.y.to_bits(), v.z.to_bits());
        for i in 0..=40 {
            for j in 0..=40 {
                let (u, v) = (-0.1 + 1.2 * i as f64 / 40.0, -0.1 + 1.2 * j as f64 / 40.0);
                let d = s.derivs(u, v, 1);
                let (p, du, dv) = s.derivs1_nr(u, v).unwrap();
                assert_eq!((bits(p), bits(du), bits(dv)), (bits(d[0][0]), bits(d[1][0]), bits(d[0][1])), "{u} {v}");
            }
        }
        for (u, v) in [(0.3, 0.6), (0.0, 1.0), (0.5, 0.3), (1.0, 0.0)] {
            let d = s.derivs(u, v, 1);
            let (p, du, dv) = s.derivs1_nr(u, v).unwrap();
            assert_eq!((bits(p), bits(du), bits(dv)), (bits(d[0][0]), bits(d[1][0]), bits(d[0][1])), "{u} {v}");
        }
    }

    /// `thirds_nr` is the third-order `derivs` bit for bit, for several degree pairs (also at knots and outside).
    #[test]
    fn thirds_nr_is_bit_identical() {
        let bits = |v: Vec3| (v.x.to_bits(), v.y.to_bits(), v.z.to_bits());
        for (p, q) in [(1usize, 1usize), (2, 3), (3, 3), (3, 2), (4, 5), (5, 4)] {
            let (nu, nv) = (p + 4, q + 3);
            let mut ctrl = vec![];
            for i in 0..nu {
                for j in 0..nv {
                    let (x, y) = (i as f64, j as f64);
                    ctrl.push(Vec3::new(x + 0.1 * y * y, y, (x * 0.7).sin() * (y * 0.5).cos() + 0.1 * x * y));
                }
            }
            let mut ku = basis::clamped_uniform(nu, p);
            // a repeated interior knot
            if nu > p + 2 {
                let k = p + 1;
                ku[k + 1] = ku[k];
            }
            let s = BSplineSurface::new(p, q, ku, basis::clamped_uniform(nv, q), nu, nv, ctrl, None).unwrap();
            for i in 0..=30 {
                for j in 0..=30 {
                    let (u, v) = (-0.1 + 1.2 * i as f64 / 30.0, -0.1 + 1.2 * j as f64 / 30.0);
                    let d = s.derivs(u, v, 3);
                    let t = s.thirds_nr(u, v).unwrap();
                    assert_eq!([bits(t[0]), bits(t[1]), bits(t[2]), bits(t[3])], [bits(d[3][0]), bits(d[2][1]), bits(d[1][2]), bits(d[0][3])], "{p} {q} {u} {v}");
                }
            }
        }
    }

    /// `derivs2_cached` is `derivs2` bit for bit along runs that repeat one parameter, repeat both, and jump.
    #[test]
    fn derivs2_cached_is_bit_identical() {
        let bits = |d: &SurfDerivs| [d.p, d.du, d.dv, d.duu, d.duv, d.dvv].map(|v| (v.x.to_bits(), v.y.to_bits(), v.z.to_bits()));
        for rational in [false, true] {
            let s = surf(rational);
            let mut cache = Basis2Cache::default();
            let mut seed = 5u64;
            let mut rnd = || {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (seed >> 11) as f64 / (1u64 << 53) as f64
            };
            let (mut u, mut v) = (0.4, 0.4);
            for k in 0..2000 {
                match k % 5 {
                    0 => u = rnd() * 1.2 - 0.1,
                    1 => v = rnd() * 1.2 - 0.1,
                    2 => {}
                    3 => u = 1.5,
                    _ => {
                        u = rnd();
                        v = rnd();
                    }
                }
                assert_eq!(bits(&s.derivs2_cached(u, v, &mut cache)), bits(&s.derivs2(u, v)), "{u} {v}");
            }
        }
    }

    #[test]
    fn corners_interpolated() {
        let s = surf(true);
        assert!(vclose(s.eval(0., 0.), s.cp(0, 0), 1e-14));
        assert!(vclose(s.eval(1., 1.), s.cp(3, 4), 1e-14));
        assert!(vclose(s.eval(1., 0.), s.cp(3, 0), 1e-14));
    }

    #[test]
    fn partials_fd() {
        for s in [surf(false), surf(true)] {
            let h = 1e-5;
            for &(u, v) in &[(0.2, 0.4), (0.7, 0.1), (0.5, 0.8)] {
                let d = s.derivs2(u, v);
                let fu = (s.eval(u + h, v) - s.eval(u - h, v)) / (2.0 * h);
                let fv = (s.eval(u, v + h) - s.eval(u, v - h)) / (2.0 * h);
                assert!(vclose(d.du, fu, 1e-7));
                assert!(vclose(d.dv, fv, 1e-7));
                let fuu = (s.eval(u + h, v) - d.p * 2.0 + s.eval(u - h, v)) / (h * h);
                let fvv = (s.eval(u, v + h) - d.p * 2.0 + s.eval(u, v - h)) / (h * h);
                let fuv = (s.eval(u + h, v + h) - s.eval(u + h, v - h) - s.eval(u - h, v + h) + s.eval(u - h, v - h))
                    / (4.0 * h * h);
                assert!(vclose(d.duu, fuu, 1e-3));
                assert!(vclose(d.dvv, fvv, 1e-3));
                assert!(vclose(d.duv, fuv, 1e-4));
            }
        }
    }

    #[test]
    fn iso_and_insert_exact() {
        for s in [surf(false), surf(true)] {
            let cu = s.iso_u(0.37);
            let cv = s.iso_v(0.52);
            let ku = s.insert_knot_u(0.4, 2);
            let kv = s.insert_knot_v(0.45, 1);
            for i in 0..=10 {
                let t = i as f64 / 10.0;
                assert!(vclose(cu.eval(t), s.eval(0.37, t), 1e-12));
                assert!(vclose(cv.eval(t), s.eval(t, 0.52), 1e-12));
                assert!(vclose(ku.eval(t, 0.3), s.eval(t, 0.3), 1e-12));
                assert!(vclose(kv.eval(0.6, t), s.eval(0.6, t), 1e-12));
            }
            let t = s.transposed();
            assert!(vclose(t.eval(0.3, 0.8), s.eval(0.8, 0.3), 1e-12));
        }
    }

    #[test]
    fn closest_param_recovers() {
        let s = surf(true);
        let p = s.eval(0.33, 0.71);
        let n = s.normal(0.33, 0.71);
        let (u, v) = s.closest_param(p + n * 0.05);
        assert!(close(u, 0.33, 1e-7) && close(v, 0.71, 1e-7), "{u} {v}");
    }

    #[test]
    fn bilinear_plane_normal() {
        let s = BSplineSurface::bilinear(Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(1., 1., 0.));
        assert!(vclose(s.normal(0.5, 0.5), Vec3::Z, 1e-15));
        assert!(vclose(s.eval(0.5, 0.25), Vec3::new(0.5, 0.25, 0.), 1e-15));
    }

    #[test]
    fn rational_fast_paths_match_generic() {
        let s = surf(true);
        for &(u, v) in &[(0.0, 0.0), (0.2, 0.9), (0.55, 0.31), (1.0, 1.0), (0.77, 0.6)] {
            let g = s.derivs(u, v, 2);
            assert!(vclose(s.eval(u, v), g[0][0], 1e-13));
            let d = s.derivs2(u, v);
            for (a, b) in [(d.p, g[0][0]), (d.du, g[1][0]), (d.dv, g[0][1]), (d.duu, g[2][0]), (d.duv, g[1][1]), (d.dvv, g[0][2])] {
                assert!(vclose(a, b, 1e-11), "{a:?} vs {b:?}");
            }
        }
    }

    /// Full rational circle (unit radius, 9 control points) times a segment: u closes on itself.
    fn closed_tube() -> BSplineSurface {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let ring = [(1., 0., 1.), (1., 1., h), (0., 1., 1.), (-1., 1., h), (-1., 0., 1.), (-1., -1., h), (0., -1., 1.), (1., -1., h), (1., 0., 1.)];
        let mut ctrl = vec![];
        let mut ws = vec![];
        for &(x, y, w) in &ring {
            for z in [0.0, 2.0] {
                ctrl.push(Vec3::new(x, y, z));
                ws.push(w);
            }
        }
        BSplineSurface::new(2, 1, vec![0., 0., 0., 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1., 1., 1.], vec![0., 0., 1., 1.], 9, 2, ctrl, Some(ws)).unwrap()
    }

    #[test]
    fn closest_param_across_the_closing_isoline() {
        let s = closed_tube();
        // just short of the closing isoline u = 1: the nearest grid sample is on u = 0
        for &(u, v) in &[(0.9995, 0.3), (0.9999, 1.7 / 2.0), (0.0004, 0.5), (0.99999, 0.0)] {
            let p = s.eval(u, v);
            let (cu, cv) = s.closest_param(p);
            assert!(s.eval(cu, cv).dist(p) < 1e-9, "{u} {v} -> {cu} {cv}: {}", s.eval(cu, cv).dist(p));
        }
    }

    #[test]
    fn derivs2_is_bit_identical_to_derivs() {
        let mut x = 0x2545f4914f6cdd1du64;
        let mut rnd = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x >> 11) as f64 / (1u64 << 53) as f64
        };
        for (p, q) in [(1usize, 1usize), (2, 3), (3, 3), (3, 2), (5, 4)] {
            let (nu, nv) = (p + 3, q + 4);
            let ctrl: Vec<Vec3> = (0..nu * nv).map(|_| Vec3::new(rnd(), rnd(), rnd())).collect();
            let s = BSplineSurface::new(p, q, basis::clamped_uniform(nu, p), basis::clamped_uniform(nv, q), nu, nv, ctrl, None).unwrap();
            for _ in 0..50 {
                let (u, v) = (rnd(), rnd());
                let a = s.derivs2(u, v);
                let b = s.derivs(u, v, 2);
                for (x, y) in [(a.p, b[0][0]), (a.du, b[1][0]), (a.dv, b[0][1]), (a.duu, b[2][0]), (a.duv, b[1][1]), (a.dvv, b[0][2])] {
                    assert_eq!((x.x.to_bits(), x.y.to_bits(), x.z.to_bits()), (y.x.to_bits(), y.y.to_bits(), y.z.to_bits()));
                }
            }
        }
    }

    /// The nearest-cells shortcut of the unbounded query gives the full scan's answer bit for bit,
    /// both where it answers and where it hands over to the full scan (points on, near, beyond the
    /// edge of and far from a wavy many-span surface, plain and rational).
    #[test]
    fn nearest_cells_path_matches_the_full_scan() {
        let (nu, nv) = (23, 19);
        for rational in [false, true] {
            let mut ctrl = Vec::new();
            let mut ws = Vec::new();
            for i in 0..nu {
                for j in 0..nv {
                    let (x, y) = (i as f64 * 0.7, j as f64 * 0.9);
                    ctrl.push(Vec3::new(x + 0.2 * (y * 0.8).sin(), y, 2.0 * (x * 0.6).sin() * (y * 0.45).cos() + 0.05 * x * y));
                    ws.push(1.0 + 0.4 * ((i * 7 + j * 3) % 5) as f64 / 4.0);
                }
            }
            let s = BSplineSurface::new(3, 3, basis::clamped_uniform(nu, 3), basis::clamped_uniform(nv, 3), nu, nv, ctrl, rational.then_some(ws)).unwrap();
            let ((u0, u1), (v0, v1)) = s.domain();
            let (mut quick, mut full) = (0, 0);
            let mut seed = 12345u64;
            let mut rnd = || {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (seed >> 11) as f64 / (1u64 << 53) as f64
            };
            for k in 0..3000 {
                let (u, v) = (u0 - 0.1 + 1.2 * (u1 - u0) * rnd(), v0 - 0.1 + 1.2 * (v1 - v0) * rnd());
                let (uc, vc) = (u.clamp(u0, u1), v.clamp(v0, v1));
                let p = s.eval(uc, vc);
                let n = s.normal(uc, vc).normalize();
                let off = [0.0, 1e-5, -3e-3, 0.4, -2.0, 15.0][k % 6];
                let q = p + n * off + Vec3::new(u - uc, v - vc, 0.0) * 3.0;
                let got = s.closest_param_within_with(&s.closest_prep(), q, f64::INFINITY);
                let (want, qk) = s.closest_param_full_scan(q);
                let bits = |r: Option<(f64, f64)>| r.map(|(a, b)| (a.to_bits(), b.to_bits()));
                assert_eq!(bits(got), bits(want), "query {k} at {q:?} (rational {rational})");
                if qk {
                    quick += 1;
                } else {
                    full += 1;
                }
            }
            // both paths ran
            assert!(quick > 300 && full > 30, "quick {quick} full {full}");
        }
    }

    #[test]
    fn closest_param_within_bounds_the_search() {
        let s = closed_tube();
        let p = s.eval(0.3, 0.4) + Vec3::new(0.0, 0.0, 0.0);
        assert!(s.closest_param_within(p, 1e-6).is_some());
        let far = Vec3::new(10.0, 0.0, 1.0);
        assert!(s.closest_param_within(far, 5.0).is_none());
        let (u, v) = s.closest_param_within(far, 20.0).unwrap();
        assert!((s.eval(u, v).dist(far) - 9.0).abs() < 1e-9);
    }
}

#[cfg(test)]
mod rational_fast_path_tests {
    use super::*;

    /// A rational patch (quarter cylinder, degree 2 x 1): the allocation-free
    /// `eval` / `derivs2` agree with the generic `derivs`.
    #[test]
    fn rational_fast_path_matches_generic() {
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        let ring = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0), Vec3::new(0.0, 1.0, 0.0)];
        let w = [1.0, s2, 1.0];
        let mut ctrl = vec![];
        let mut wt = vec![];
        for i in 0..3 {
            for z in [0.0, 2.0] {
                ctrl.push(ring[i] + Vec3::new(0.0, 0.0, z));
                wt.push(w[i]);
            }
        }
        let s = BSplineSurface::new(2, 1, vec![0., 0., 0., 1., 1., 1.], vec![0., 0., 1., 1.], 3, 2, ctrl, Some(wt)).unwrap();
        for &(u, v) in &[(0.1, 0.2), (0.5, 0.5), (0.93, 0.77), (0.0, 1.0)] {
            let g = s.derivs(u, v, 2);
            let e = s.eval(u, v);
            let d = s.derivs2(u, v);
            for (a, b) in [(e, g[0][0]), (d.p, g[0][0]), (d.du, g[1][0]), (d.dv, g[0][1]), (d.duu, g[2][0]), (d.duv, g[1][1]), (d.dvv, g[0][2])] {
                assert!(a.dist(b) < 1e-12, "{a:?} vs {b:?}");
            }
        }
    }
}
