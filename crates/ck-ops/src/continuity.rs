//! G0/G1/G2/G3 edge continuity analysis (the `continuity` request).
//!
//! Per manifold edge (two coedges) the edge is sampled `n + 1` times at
//! uniform parameters; at each sample the two faces are compared:
//!
//! * G0 `gap`: the larger of the two point-to-other-face-boundary distances,
//!   each a 40-step golden-section minimum over a window of ±dt around the
//!   sample (so a parametrisation mismatch between the faces is not a gap);
//! * G1 `angle`: angle (degrees) between the face normals (outward);
//! * G2 `g2`: relative difference of the normal curvatures across the edge,
//!   `|kA - kB| / max(|kA|, |kB|, 1e-3)`;
//! * G3 `g3`: relative sum of the curvature rates (3rd-order one-sided
//!   difference of the normal curvature, 3 Newton-corrected steps of `h`
//!   across the edge on each face), `|rA + rB| / max(|rA|, |rB|, 1e-5)`.
//!
//! Every sample gets a severity (largest tolerance ratio over the levels up
//! to the required one); the worst sample and the per-level maxima are
//! refined by a 20-step golden-section search between the neighbouring
//! samples. An edge is a `corner` when every sample is more than
//! 100 × the angle tolerance off tangent, and `open` unless it has exactly
//! two coedges.
//!
//! Wasm provenance (kernel.wasm): f2774 sample, f2775 min distance, f2776
//! curvature + rate, f2821 normal curvature, f2777 refinement, f1741 /
//! f2817 judge_edge, f2816 + f2305 sides. f1740/f2820 are the thread
//! fan-out over edges (results are identical to the sequential map).

use ck_core::linalg::Vec3;
use ck_geom::surface::Surface;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, CoedgeId, EdgeId, Face, FaceId};

const PHI: f64 = 0.6180339887498949;

/// Check settings (`required` is the level 0..=3 for G0..=G3).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// gap (mm), angle (deg), curvature (fraction), rate (fraction)
    pub tol: [f64; 4],
    pub n: u32,
    pub required: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { tol: [0.01, 0.1, 0.05, 0.1], n: 48, required: 2 }
    }
}

type V = [f64; 3];

/// One measured sample (wasm record, 0x98 bytes).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// G3 data valid (both sides produced a rate)
    pub rate_ok: bool,
    pub rate: [f64; 2],
    pub g3: f64,
    pub t: f64,
    pub s: f64,
    pub at: V,
    pub normal: V,
    pub gap: f64,
    /// degrees
    pub angle: f64,
    pub curv: [f64; 2],
    pub g2: f64,
    /// 0 ok, 1 warn, 2 fail
    pub status: u8,
    /// first failing level 0..=3
    pub fails: Option<u8>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Kind {
    /// judged edge: worst sample (with the level it was worst at), per-level
    /// maxima [gap, angle, g2, g3], edge status (max sample status)
    Judged { status: u8, max: [Option<f64>; 4], worst: Sample, level: u8 },
    /// tangent nowhere: sampled but not judged
    Corner,
    /// not exactly two coedges
    Open,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeReport {
    pub edge: EdgeId,
    /// faces of coedge 0 and 1 (`sides`)
    pub sides: Option<[FaceId; 2]>,
    pub length: f64,
    pub kind: Kind,
    pub samples: Vec<Sample>,
}

struct Side<'a> {
    face: &'a Face,
    coedge: CoedgeId,
    /// face sense
    sense: bool,
    /// coedge reversed
    rev: bool,
}

struct Ctx<'a> {
    body: &'a Body,
    edge: &'a ck_topo::body::Edge,
    set: &'a Settings,
    t0: f64,
    t1: f64,
    h: f64,
    dt: f64,
    a: Side<'a>,
    b: Side<'a>,
    n: usize,
    ts: Vec<f64>,
    st: Vec<f64>,
}

fn v3(p: Vec3) -> V {
    [p.x, p.y, p.z]
}

/// (surface point, Su, Sv) from the second-derivative evaluation.
fn pt_d1(s: &Surface, u: f64, v: f64) -> (V, V, V) {
    let d = s.derivs2(u, v);
    (v3(d.p), v3(d.du), v3(d.dv))
}

/// f2821: normal curvature of the surface at (u, v) along `n x dir`.
fn normal_curvature(s: &Surface, sense: bool, u: f64, v: f64, dir: V) -> Option<f64> {
    let d = s.derivs2(u, v);
    let nn = v3(s.normal(u, v));
    let n = if sense { nn } else { [-nn[0], -nn[1], -nn[2]] };
    let (su, sv) = (v3(d.du), v3(d.dv));
    let cz = n[0] * dir[1] - n[1] * dir[0];
    let cx = n[1] * dir[2] - dir[1] * n[2];
    let cy = dir[0] * n[2] - n[0] * dir[2];
    let len = (cz * cz + cx * cx + cy * cy).sqrt();
    if !(len > 1e-300 && len != f64::INFINITY) {
        return None;
    }
    let e = su[0] * su[0] + su[1] * su[1] + su[2] * su[2];
    let g = sv[0] * sv[0] + sv[1] * sv[1] + sv[2] * sv[2];
    let eg = e * g;
    let f = su[0] * sv[0] + su[1] * sv[1] + su[2] * sv[2];
    let det = eg - f * f;
    let lim = if eg.is_nan() { 1e-20 } else { eg.max(1e-20) };
    if !(lim * 1e-20 < det) {
        return None;
    }
    let inv = 1.0 / len;
    let a = cx * inv * su[0] + cy * inv * su[1] + cz * inv * su[2];
    let b = cx * inv * sv[0] + cy * inv * sv[1] + cz * inv * sv[2];
    let du = (a * g - f * b) / det;
    let dv = (e * b - a * f) / det;
    let w2 = su[2] * du + sv[2] * dv;
    let w0 = su[0] * du + sv[0] * dv;
    let w1 = su[1] * du + sv[1] * dv;
    let ww = w2 * w2 + w0 * w0 + w1 * w1;
    if !(1e-20 <= ww) {
        return None;
    }
    let c = dir[1] * n[1] + n[0] * dir[0] + dir[2] * n[2];
    let mut sq = 1.0 - c * c;
    if sq.is_nan() {
        sq = 0.0;
    }
    if !(0.0 < sq) {
        return None;
    }
    let uu = du * du;
    let uv = dv * (du + du);
    let vv = dv * dv;
    let num = n[0] * (uu * d.duu.x + uv * d.duv.x + vv * d.dvv.x)
        + n[1] * (uu * d.duu.y + uv * d.duv.y + vv * d.dvv.y)
        + n[2] * (uu * d.duu.z + uv * d.duv.z + vv * d.dvv.z);
    Some((num / ww) / sq.sqrt())
}

/// f2776 result: (curvature, rate) of one side.
#[derive(Clone, Copy)]
struct CurvRate {
    k: Option<f64>,
    rate: Option<f64>,
}

impl<'a> Ctx<'a> {
    fn uv(&self, s: &Side, t: f64) -> (f64, f64) {
        let p = self.body.coedge_uv(s.coedge, t, None);
        (p.x, p.y)
    }

    /// f2773: 3D point of a side's pcurve at `t`.
    fn side_pt(&self, s: &Side, t: f64) -> V {
        let (u, v) = self.uv(s, t);
        v3(s.face.surface.eval(u, v))
    }

    /// f2775: golden-section minimum of |side(t) - q| on [lo, hi]
    /// (40 steps; the end points are included).
    fn min_dist(&self, lo: f64, hi: f64, s: &Side, q: V) -> f64 {
        let mut d = (hi - lo) * PHI;
        let mut x1 = hi - d;
        let p1 = self.side_pt(s, x1);
        d = lo + d; // wasm reuses the temporary
        let mut x2 = d;
        let p2 = self.side_pt(s, x2);
        let (dx, dy, dz) = (p1[0] - q[0], p1[1] - q[1], p1[2] - q[2]);
        let mut d1 = (dz * dz + dx * dx + dy * dy).sqrt();
        let (dx, dy, dz) = (p2[0] - q[0], p2[1] - q[1], p2[2] - q[2]);
        let mut d2 = (dx * dx + dy * dy + dz * dz).sqrt();
        let (mut a, mut b) = (lo, hi);
        for _ in 0..40 {
            let old1 = d1;
            if d1 < d2 {
                let xn = x2 + (x2 - a) * -PHI;
                let p = self.side_pt(s, xn);
                let (dx, dy, dz) = (p[0] - q[0], p[1] - q[1], p[2] - q[2]);
                d1 = (dx * dx + dy * dy + dz * dz).sqrt();
                b = x2;
                // new x2 = old x1, new x1 = xn
                d2 = old1;
                x2 = x1;
                x1 = xn;
            } else {
                let xn = x1 + (b - x1) * PHI;
                let p = self.side_pt(s, xn);
                let (dx, dy, dz) = (p[0] - q[0], p[1] - q[1], p[2] - q[2]);
                let dn = (dx * dx + dy * dy + dz * dz).sqrt();
                a = x1;
                d1 = d2;
                d2 = dn;
                x1 = x2;
                x2 = xn;
            }
        }
        let plo = self.side_pt(s, lo);
        let phi_ = self.side_pt(s, hi);
        let (dx, dy, dz) = (phi_[0] - q[0], phi_[1] - q[1], phi_[2] - q[2]);
        let dhi = (dy * dy + dx * dx + dz * dz).sqrt();
        let (dx, dy, dz) = (plo[0] - q[0], plo[1] - q[1], plo[2] - q[2]);
        let dlo = (dz * dz + dx * dx + dy * dy).sqrt();
        let _ = (a, b);
        d2.min(d1).min(dlo).min(dhi)
    }

    /// f2776: normal curvature at (u0, v0) and its rate across the edge
    /// (`m` = in-face conormal, `tg` = edge tangent, `p` = sample point).
    fn curv_rate(&self, u0: f64, v0: f64, s: &Side, p: V, tg: V, m: V) -> CurvRate {
        let h = self.h;
        let surf = &s.face.surface;
        let Some(k0) = normal_curvature(surf, s.sense, u0, v0, tg) else {
            return CurvRate { k: None, rate: None };
        };
        let d = surf.derivs2(u0, v0);
        let (su, sv) = (v3(d.du), v3(d.dv));
        let e = su[0] * su[0] + su[1] * su[1] + su[2] * su[2];
        let g = sv[0] * sv[0] + sv[1] * sv[1] + sv[2] * sv[2];
        let f = su[0] * sv[0] + su[1] * sv[1] + su[2] * sv[2];
        let eg = e * g;
        let det = eg - f * f;
        let lim = if eg.is_nan() { 1e-20 } else { eg.max(1e-20) };
        if det <= lim * 1e-20 {
            return CurvRate { k: Some(k0), rate: None };
        }
        let b = sv[2] * m[2] + sv[1] * m[1] + sv[0] * m[0];
        let a = su[2] * m[2] + su[1] * m[1] + su[0] * m[0];
        let mut vn = v0 + h * ((e * b - f * a) / det);
        let mut un = u0 + h * ((g * a - f * b) / det);
        let (mut pu, mut pv) = (u0, v0);
        let mut ks = [k0, 0.0, 0.0, 0.0];
        for k in 1..=3usize {
            let target = h * k as f64;
            let (mut u, mut v) = (un, vn);
            let mut count = -29i32;
            loop {
                let (sp, su, sv) = pt_d1(surf, u, v);
                let dx = sp[0] - p[0];
                let dy = sp[1] - p[1];
                let dz = sp[2] - p[2];
                let r1 = tg[0] * dx + tg[1] * dy + tg[2] * dz;
                let r2 = m[0] * dx + m[1] * dy + m[2] * dz;
                if r1.abs() + (r2 - target).abs() < (target.abs() + 1.0) * 1e-11 {
                    break;
                }
                let j11 = tg[0] * su[0] + tg[1] * su[1] + tg[2] * su[2];
                let j22 = m[0] * sv[0] + m[1] * sv[1] + m[2] * sv[2];
                let j21 = m[0] * su[0] + m[1] * su[1] + m[2] * su[2];
                let j12 = tg[0] * sv[0] + tg[1] * sv[1] + tg[2] * sv[2];
                let a1 = j11 * j22;
                let a2 = j21 * j12;
                if (a1 - a2).abs() < 1e-300 {
                    return CurvRate { k: Some(k0), rate: None };
                }
                let rr = r2 - target;
                let dd = a1 - a2;
                u -= (r1 * j22 - rr * j12) / dd;
                if !u.is_finite() {
                    return CurvRate { k: Some(k0), rate: None };
                }
                v -= (rr * j11 - r1 * j21) / dd;
                let last = count == 0;
                count += 1;
                if !v.is_finite() || last {
                    return CurvRate { k: Some(k0), rate: None };
                }
            }
            let Some(kk) = normal_curvature(surf, s.sense, u, v, tg) else {
                return CurvRate { k: Some(k0), rate: None };
            };
            ks[k] = kk;
            vn = v + (v - pv);
            un = u + (u - pu);
            pv = v;
            pu = u;
        }
        let rate = (ks[1] * 18.0 + ks[0] * -11.0 + ks[2] * -9.0 + ks[3] + ks[3]) / (h * 6.0);
        CurvRate { k: Some(k0), rate: Some(rate) }
    }

    /// f2774: measure the continuity at edge parameter `t` (arc length `s`).
    fn sample(&self, t: f64, s: f64) -> Sample {
        let (a, b) = (&self.a, &self.b);
        let tg = v3(self.edge.curve.tangent(t));
        let (ua, va) = self.uv(a, t);
        let (ub, vb) = self.uv(b, t);
        let pa = v3(a.face.surface.eval(ua, va));
        let pb = v3(b.face.surface.eval(ub, vb));
        let lo = (t - self.dt).max(self.t0);
        let hi = (t + self.dt).min(self.t1);
        let gb = self.min_dist(lo, hi, b, pa);
        let ga = self.min_dist(lo, hi, a, pb);
        let gap = gb.max(ga);
        let mut na = v3(a.face.surface.normal(ua, va));
        if !a.sense {
            na = [-na[0], -na[1], -na[2]];
        }
        let mut nb = v3(b.face.surface.normal(ub, vb));
        if !b.sense {
            nb = [-nb[0], -nb[1], -nb[2]];
        }
        // angle between the normals
        let cz = na[0] * nb[1] - na[1] * nb[0];
        let cx = na[1] * nb[2] - nb[1] * na[2];
        let cy = nb[0] * na[2] - na[0] * nb[2];
        let sn = (cz * cz + cx * cx + cy * cy).sqrt();
        let cs = nb[1] * na[1] + na[0] * nb[0] + nb[2] * na[2];
        let ang = sn.atan2(cs);
        // conormals n x T' (T' = T on reversed coedges, else -T)
        let conormal = |n: V, rev: bool| -> (f64, f64, f64, f64) {
            let (ty, tx, tz) = if rev { (tg[1], tg[0], tg[2]) } else { (-tg[1], -tg[0], -tg[2]) };
            let cz = n[0] * ty - n[1] * tx;
            let cx = n[1] * tz - ty * n[2];
            let cy = tx * n[2] - n[0] * tz;
            (cx, cy, cz, (cz * cz + cx * cx + cy * cy).sqrt())
        };
        let ok_len = |l: f64| l > 1e-300 && l != f64::INFINITY && !l.is_nan();
        let (cax, cay, caz, la) = conormal(na, a.rev);
        let ra = if ok_len(la) {
            let inv = 1.0 / la;
            let m = [cax * inv, cay * inv, caz * inv];
            Some(self.curv_rate(ua, va, a, pa, tg, m))
        } else {
            None
        };
        let (cbx, cby, cbz, lb) = conormal(nb, b.rev);
        let (ka, kb, rates) = {
            let ka = match ra {
                Some(r) => r.k.unwrap_or(0.0),
                None => 0.0,
            };
            if !ok_len(lb) {
                (ka, 0.0, None)
            } else {
                let inv = 1.0 / lb;
                let m = [cbx * inv, cby * inv, cbz * inv];
                let rb = self.curv_rate(ub, vb, b, pb, tg, m);
                let kb = rb.k.unwrap_or(0.0);
                let rates = match (ra.and_then(|r| r.rate), rb.rate) {
                    (Some(x), Some(y)) => Some((x, y)),
                    _ => None,
                };
                (ka, kb, rates)
            }
        };
        // mid normal
        let sx = na[0] + nb[0];
        let sy = na[1] + nb[1];
        let sz = na[2] + nb[2];
        let sl = (sx * sx + sy * sy + sz * sz).sqrt();
        let normal = if sl > 1e-300 && sl != f64::INFINITY && !sl.is_nan() {
            let inv = 1.0 / sl;
            [sx * inv, sy * inv, sz * inv]
        } else {
            na
        };
        let den = ka.abs().max(kb.abs());
        let den = if den.is_nan() { 0.001 } else { den.max(0.001) };
        let g2 = (ka - kb).abs() / den;
        let (rate_ok, rate, g3) = match rates {
            Some((x, y)) => {
                let d = x.abs().max(y.abs());
                let d = if d.is_nan() { 1e-5 } else { d.max(1e-5) };
                (true, [x, y], (x + y).abs() / d)
            }
            None => (false, [0.0, 0.0], 0.0),
        };
        let tol = &self.set.tol;
        let mut r = gap / tol[0];
        let (fails, status);
        if r > 1.0 {
            fails = Some(0);
            status = if r <= 3.0 { 1 } else { 2 };
        } else if self.set.required == 0 {
            fails = None;
            status = 0;
        } else {
            let deg = ang * 57.29577951308232;
            r = deg / tol[1];
            if r > 1.0 {
                fails = Some(1);
                status = if r <= 3.0 { 1 } else { 2 };
            } else if self.set.required == 1 {
                fails = None;
                status = 0;
            } else {
                r = g2 / tol[2];
                if r > 1.0 {
                    fails = Some(2);
                    status = if r <= 3.0 { 1 } else { 2 };
                } else if self.set.required == 2 || !rate_ok || {
                    r = g3 / tol[3];
                    r <= 1.0
                } {
                    fails = None;
                    status = 0;
                } else {
                    fails = Some(3);
                    status = if r <= 3.0 { 1 } else { 2 };
                }
            }
        }
        Sample {
            rate_ok,
            rate,
            g3,
            t,
            s,
            at: v3(self.edge.curve.eval(t)),
            normal,
            gap,
            angle: ang * 57.29577951308232,
            curv: [ka, kb],
            g2,
            status,
            fails,
        }
    }

    /// Arc length at parameter `t` interpolated from the sample table.
    fn arc_at(&self, t: f64) -> f64 {
        let nf = self.n as f64;
        let x = ((t - self.t0) / (self.t1 - self.t0)) * nf;
        let x = if 0.0 <= x { x } else { 0.0 };
        let x = if x <= nf { x } else { nf };
        let i = (x.trunc() as usize).min(self.n - 1);
        let s0 = self.st[i];
        s0 + (x - i as f64) * (self.st[i + 1] - s0)
    }

    fn sample_at(&self, t: f64) -> Sample {
        self.sample(t, self.arc_at(t))
    }

    /// f2777: maximise `obj` over [lo, hi] (20 golden-section steps).
    fn refine(&self, lo: f64, hi: f64, obj: &dyn Fn(&Sample) -> Option<f64>) -> Option<(Sample, f64)> {
        let val = |o: Option<f64>| o.unwrap_or(f64::NEG_INFINITY);
        let (mut lo, mut hi) = (lo, hi);
        let d = (hi - lo) * PHI;
        let mut x1 = hi - d;
        let mut x2 = lo + d;
        let mut r1 = self.sample_at(x1);
        let mut f1 = val(obj(&r1));
        let mut r2 = self.sample_at(x2);
        let mut f2 = val(obj(&r2));
        for _ in 0..20 {
            if f1 <= f2 {
                let xn = x1 + (hi - x1) * PHI;
                r1 = r2;
                let rn = self.sample_at(xn);
                let fnew = val(obj(&rn));
                lo = x1;
                let nx1 = x2;
                f1 = f2;
                f2 = fnew;
                r2 = rn;
                x1 = nx1;
                x2 = xn;
            } else {
                let xn = x2 + (x2 - lo) * -PHI;
                r2 = r1;
                let rn = self.sample_at(xn);
                let fnew = val(obj(&rn));
                hi = x2;
                let nx2 = x1;
                f2 = f1;
                f1 = fnew;
                r1 = rn;
                x2 = nx2;
                x1 = xn;
            }
        }
        let (best, bv) = if f1 < f2 { (r2, f2) } else { (r1, f1) };
        if bv.abs() < f64::INFINITY {
            Some((best, bv))
        } else {
            None
        }
    }
}

/// Largest tolerance ratio over the levels up to the required one that are
/// still within tolerance below (closure 0x160 of f1741).
fn severity(r: &Sample, set: &Settings) -> f64 {
    let tol = &set.tol;
    let req = set.required;
    let r0 = r.gap / tol[0];
    let mut sev = if r0 < 0.0 { 0.0 } else { r0 };
    if r0 <= 1.0 && req != 0 {
        let r1 = r.angle / tol[1];
        let m1 = if r1 < sev { sev } else { r1 };
        sev = m1;
        if r1 <= 1.0 && req != 1 {
            let r2 = r.g2 / tol[2];
            let m2 = if r2 < m1 { m1 } else { r2 };
            sev = m2;
            if r.rate_ok && req != 2 && r2 <= 1.0 {
                let r3 = r.g3 / tol[3];
                sev = if r3 < m2 { m2 } else { r3 };
            }
        }
    }
    sev
}

/// Value of level `k` at a sample if all lower levels are within tolerance
/// (closure 0x15f of f1741).
fn level_value(r: &Sample, k: usize, tol: &[f64; 4]) -> Option<f64> {
    let ok = if r.rate_ok {
        (k < 1 || !(tol[0] < r.gap)) && (k < 2 || !(tol[1] < r.angle)) && (k < 3 || !(tol[2] < r.g2))
    } else {
        (k < 1 || r.gap <= tol[0]) && (k < 2 || !(tol[1] < r.angle)) && (k < 3 || !(tol[2] < r.g2))
    };
    if !ok {
        return None;
    }
    match k {
        0 => Some(r.gap),
        1 => Some(r.angle),
        2 => Some(r.g2),
        _ => {
            if r.rate_ok {
                Some(r.g3)
            } else {
                None
            }
        }
    }
}

/// Level the severity is attained at (ties go to the higher level).
fn worst_level(r: &Sample, set: &Settings) -> u8 {
    let tol = &set.tol;
    let req = set.required;
    let r0 = r.gap / tol[0];
    if r0 > 1.0 || req == 0 {
        return 0;
    }
    let r1 = r.angle / tol[1];
    let r0c = if r0 < 0.0 { 0.0 } else { r0 };
    let l1 = (r0c <= r1) as u8;
    if !(r1 <= 1.0 && req != 1) {
        return l1;
    }
    let r2 = r.g2 / tol[2];
    let m1 = if r1 < r0c { r0c } else { r1 };
    if !r.rate_ok {
        return if r2 < m1 { l1 } else { 2 };
    }
    let l2 = if r2 < m1 { l1 } else { 2 };
    if req != 2 && r2 <= 1.0 {
        let r3 = r.g3 / tol[3];
        let m2 = if r2 < m1 { m1 } else { r2 };
        let lt = r3 < m2 && !r3.is_nan() && !m2.is_nan();
        return if lt { l2 } else { 3 };
    }
    l2
}

/// f2816 + f2305: the faces/coedges on the two sides of a manifold edge.
fn sides(body: &Body, e: EdgeId) -> Option<[Side<'_>; 2]> {
    let cs = body.edge_coedges(e);
    if cs.len() != 2 {
        return None;
    }
    let mk = |c: CoedgeId| {
        let face = body.face(body.coedge_face(c));
        Side { face, coedge: c, sense: face.sense, rev: body.coedge(c).rev }
    };
    Some([mk(cs[0]), mk(cs[1])])
}

/// f1741: judge one edge.
pub fn judge_edge(body: &Body, e: EdgeId, set: &Settings) -> EdgeReport {
    let edge = body.edge(e);
    let Some([a, b]) = sides(body, e) else {
        return EdgeReport { edge: e, sides: None, length: body.edge_length(e), kind: Kind::Open, samples: vec![] };
    };
    let face_ids = [body.coedge_face(a.coedge), body.coedge_face(b.coedge)];
    let (t0, t1) = (edge.t0, edge.t1);
    let total = edge.curve.length_between(t0, t1);
    let range = t1 - t0;
    let n = if set.n < 3 { 2 } else { set.n as usize };
    let nf = n as f64;
    let m = total * 0.001;
    let m = if 1e-4 <= m { m } else { 1e-4 };
    let h = if m <= 0.01 { m } else { 0.01 };
    let ts: Vec<f64> = (0..=n).map(|i| t0 + (range * i as f64) / nf).collect();
    let mut st = vec![0.0; n + 1];
    for i in 1..=n {
        st[i] = st[i - 1] + edge.curve.length_between(ts[i - 1], ts[i]);
    }
    let cx = Ctx { body, edge, set, t0, t1, h, dt: range / nf, a, b, n, ts, st };
    let samples: Vec<Sample> = (0..=n).map(|i| cx.sample(cx.ts[i], cx.st[i])).collect();
    let length = cx.st[n];
    let tol = &set.tol;
    let judged = samples.iter().any(|r| !(tol[1] * 100.0 < r.angle));
    if !judged {
        return EdgeReport { edge: e, sides: Some(face_ids), length, kind: Kind::Corner, samples };
    }
    // worst sample
    let sev: Vec<f64> = samples.iter().map(|r| severity(r, set)).collect();
    let mx = sev.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
    let eps = mx.abs() * 1e-9;
    let thr = mx - eps;
    let mut wi = 0usize;
    let mut wd: Option<i64> = None;
    for (i, &sv) in sev.iter().enumerate() {
        if sv < thr {
            continue;
        }
        let d = (n as i64 - 2 * i as i64).abs();
        if wd.map_or(true, |bd| d < bd) {
            wd = Some(d);
            wi = i;
        }
    }
    let lo_i = wi - (wi != 0) as usize;
    let hi_i = (wi + 1).min(n);
    let refined = cx.refine(cx.ts[lo_i], cx.ts[hi_i], &|r| Some(severity(r, set)));
    let worst = match refined {
        Some((r, v)) if !(v <= eps + sev[wi]) => r,
        _ => samples[wi],
    };
    let level = worst_level(&worst, set);
    // per-level maxima
    let mut max = [None; 4];
    for k in 0..4 {
        let mut best: Option<(usize, f64)> = None;
        for (i, r) in samples.iter().enumerate() {
            if let Some(v) = level_value(r, k, tol) {
                if best.map_or(true, |(_, bv)| bv.total_cmp(&v) != std::cmp::Ordering::Greater) {
                    best = Some((i, v));
                }
            }
        }
        if let Some((i, v)) = best {
            let lo_i = i - (i != 0) as usize;
            let hi_i = (i + 1).min(n);
            let mv = match cx.refine(cx.ts[lo_i], cx.ts[hi_i], &|r| level_value(r, k, tol)) {
                Some((_, rv)) => rv.max(v),
                None => v,
            };
            max[k] = Some(mv);
        }
    }
    let status = samples.iter().map(|r| r.status).max().unwrap_or(0);
    EdgeReport { edge: e, sides: Some(face_ids), length, kind: Kind::Judged { status, max, worst, level }, samples }
}

// ------------------------------------------------------------- the request

/// `{type:"continuity", refs, whole, required, tol}` -> the `result` object.
/// `refs` select edges (face: its edges; edge: `faces` pair, `at` picks
/// among several); with no refs `whole` selects every judged edge.
pub fn reply(body: &Body, msg: &Json) -> Json {
    let required = match msg.get("required").and_then(Json::as_str).map(|s| s.to_ascii_uppercase()) {
        Some(s) if s == "G0" => 0,
        Some(s) if s == "G1" => 1,
        Some(s) if s == "G3" => 3,
        _ => 2,
    };
    let defaults = Settings::default().tol;
    let names = ["gap", "angle", "curvature", "rate"];
    let mut tol = defaults;
    for (i, k) in names.iter().enumerate() {
        if let Some(v) = msg.get("tol").and_then(|t| t.get_f64(k)) {
            tol[i] = v;
        }
    }
    let set = Settings { tol, n: 48, required };
    let mut tolj = Obj::new();
    for (i, k) in names.iter().enumerate() {
        tolj = tolj.put(k, tol[i]);
    }
    // wire edges (one per chain) with their keys
    let mut wire: Vec<(EdgeId, String, Vec<String>)> = vec![];
    for (seq, chain) in body.wire_chains().into_iter().enumerate() {
        let e = chain[0];
        if body.is_split_rail(e) {
            continue;
        }
        let mut fs: Vec<String> = body.edge_faces(e).iter().map(|&f| body.face_id_str(f)).collect();
        fs.sort();
        let key = if fs.len() >= 2 {
            ck_topo::body::edge_key(&fs[0], &fs[1], seq)
        } else {
            format!("{}#{}", fs[0], seq)
        };
        wire.push((e, key, fs));
    }
    let refs: &[Json] = msg.get_arr("refs").unwrap_or(&[]);
    let whole = msg.get_bool("whole").unwrap_or(false);
    let mut picked: Vec<usize> = vec![];
    let add = |i: usize, picked: &mut Vec<usize>| {
        if !picked.contains(&i) {
            picked.push(i);
        }
    };
    for r in refs {
        match r.get_str("kind") {
            Some("face") => {
                let Some(f) = r.get_str("id").and_then(|id| body.find_face(id)) else { continue };
                for (i, (e, _, _)) in wire.iter().enumerate() {
                    if body.edge_faces(*e).contains(&f) {
                        add(i, &mut picked);
                    }
                }
            }
            Some("edge") => {
                let mut want: Vec<String> =
                    r.get_arr("faces").unwrap_or(&[]).iter().filter_map(|j| j.as_str().map(String::from)).collect();
                want.sort();
                let cand: Vec<usize> = (0..wire.len()).filter(|&i| wire[i].2 == want).collect();
                let at = r.get("at").and_then(Json::as_vec3).map(|a| Vec3::new(a[0], a[1], a[2]));
                let pick = match (at, cand.len()) {
                    (_, 0) => None,
                    (Some(p), _) => cand.iter().copied().min_by(|&x, &y| {
                        let dx = body.edge(wire[x].0).curve.closest_point(p).1.dist(p);
                        let dy = body.edge(wire[y].0).curve.closest_point(p).1.dist(p);
                        dx.total_cmp(&dy)
                    }),
                    (None, _) => Some(cand[0]),
                };
                if let Some(i) = pick {
                    add(i, &mut picked);
                }
            }
            _ => {}
        }
    }
    let explicit = !refs.is_empty();
    let sel: Vec<usize> = if explicit {
        picked
    } else if whole {
        (0..wire.len()).collect()
    } else {
        vec![]
    };
    let mut edges = vec![];
    for i in sel {
        let (e, key, fs) = &wire[i];
        let rep = judge_edge(body, *e, &set);
        if !explicit && !matches!(rep.kind, Kind::Judged { .. }) {
            continue;
        }
        edges.push(edge_json(body, key, fs, &rep));
    }
    Obj::new().put("required", format!("G{required}")).put("tol", tolj.build()).put("edges", Json::Arr(edges)).build()
}

fn arr3(v: V) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Num(x)).collect())
}

fn sample_json(r: &Sample, level: Option<u8>) -> Json {
    let mut o = Obj::new()
        .put("t", r.t)
        .put("s", r.s)
        .put("at", arr3(r.at))
        .put("normal", arr3(r.normal))
        .put("gap", r.gap)
        .put("angle", r.angle)
        .put("curvature", Json::Arr(vec![Json::Num(r.curv[0]), Json::Num(r.curv[1])]))
        .put("g2", r.g2);
    o = if r.rate_ok {
        o.put("rate", Json::Arr(vec![Json::Num(r.rate[0]), Json::Num(r.rate[1])])).put("g3", r.g3)
    } else {
        o.put("rate", Json::Null).put("g3", Json::Null)
    };
    o = match r.fails {
        Some(k) => o.put("fails", format!("G{k}")),
        None => o.put("fails", Json::Null),
    };
    o = o.put("status", status_name(r.status));
    if let Some(l) = level {
        o = o.put("level", format!("G{l}"));
    }
    o.build()
}

fn status_name(s: u8) -> &'static str {
    ["ok", "warn", "fail"][s as usize]
}

fn edge_json(body: &Body, key: &str, faces: &[String], rep: &EdgeReport) -> Json {
    let mut o = Obj::new()
        .put("key", key)
        .put("faces", Json::Arr(faces.iter().map(|s| Json::from(s.as_str())).collect()));
    if let Some(s) = rep.sides {
        o = o.put("sides", Json::Arr(s.iter().map(|&f| Json::from(body.face_id_str(f))).collect()));
    }
    match &rep.kind {
        Kind::Judged { status, max, worst, level } => {
            let m: Vec<Json> = max.iter().map(|x| x.map_or(Json::Null, Json::Num)).collect();
            o = o
                .put("kind", "judged")
                .put("status", status_name(*status))
                .put("length", rep.length)
                .put("max", Json::Arr(m))
                .put("worst", sample_json(worst, Some(*level)));
        }
        Kind::Corner => {
            o = o
                .put("kind", "corner")
                .put("status", Json::Null)
                .put("length", rep.length)
                .put("max", Json::Arr(vec![Json::Null; 4]))
                .put("worst", Json::Null);
        }
        Kind::Open => {
            // GUESS: open edges carry no measurements.
            o = o
                .put("kind", "open")
                .put("status", Json::Null)
                .put("length", rep.length)
                .put("max", Json::Arr(vec![Json::Null; 4]))
                .put("worst", Json::Null);
        }
    }
    o.put("samples", Json::Arr(rep.samples.iter().map(|r| sample_json(r, None)).collect())).build()
}
