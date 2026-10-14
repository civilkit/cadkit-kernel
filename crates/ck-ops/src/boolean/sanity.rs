//! Post-boolean sanity check: a result that assembled into a closed manifold
//! can still be wrong (a whole region of an operand dropped, a rim that lies
//! exactly on the other solid's wall mis-classified). Such a body validates
//! clean, so the only independent evidence is its volume and box measured
//! against the operands'.
//!
//! Bounds (signed volumes, `tol` absorbs the face-integral error):
//! - union:     max(va, vb) <= vr <= va + vb
//! - cut:       va - vb <= vr <= va
//! - intersect: 0 <= vr <= min(va, vb)
//! - the overlap the result implies (union: va + vb - vr, cut: va - vr,
//!   intersect: vr) cannot exceed the volume of the operands' box overlap.
//!   This is the bound that catches a union which silently drops most of
//!   one operand (the dropped part reads as a huge overlap) even when the
//!   result is still larger than the bigger operand.
//! - union only: the result box contains both operand boxes.
//!
//! The check is skipped when it cannot be meaningful: an operand that is
//! not a closed solid (sheet, open shell), non-positive, or a non-finite
//! volume. Disjoint, tangent, coincident and multi-solid configurations
//! satisfy the bounds exactly, so they never trigger it.

use super::Op;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::Body;
use ck_topo::mass::face_signed_volume;

/// A result outside the bounds that `op` allows.
#[derive(Debug, Clone, PartialEq)]
pub struct VolumeMismatch {
    pub expected_min: f64,
    pub expected_max: f64,
    pub got: f64,
    /// `"volume"` or `"box"`.
    pub what: &'static str,
}

/// Net signed volume of a body (void shells count negative), `None` when it
/// is not a closed solid.
fn solid_volume(b: &Body) -> Option<f64> {
    if b.num_faces() == 0 {
        return Some(0.0);
    }
    if b.edge_ids().any(|e| b.edge_coedges(e).len() != 2) {
        return None;
    }
    // Boolean chains keep most faces from one result to the next (a cut through a shell wall
    // changes a handful of faces among hundreds of spline faces, and a face integral of a spline
    // face costs milliseconds), so the volume is summed from per-face integrals memoised by a
    // hash of each face's surface and trimming.
    thread_local! {
        static MEMO: std::cell::RefCell<std::collections::HashMap<u64, f64>> = std::cell::RefCell::new(std::collections::HashMap::new());
    }
    let mut v = 0.0;
    for f in b.face_ids() {
        let key = face_key(b, f);
        let cached = MEMO.with(|m| m.borrow().get(&key).copied());
        let fv = match cached {
            Some(x) => x,
            None => {
                let x = face_signed_volume(b, f);
                if x.is_finite() {
                    MEMO.with(|m| {
                        let mut m = m.borrow_mut();
                        if m.len() >= 8192 {
                            m.clear();
                        }
                        m.insert(key, x);
                    });
                }
                x
            }
        };
        v += fv;
    }
    v.is_finite().then_some(v)
}

/// Hash of everything one face's volume integral depends on: its surface and sense and the ends
/// and middle of every coedge's edge curve.
fn face_key(b: &Body, f: ck_topo::body::FaceId) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    let mut mix = |x: u64| {
        h = (h ^ x).wrapping_mul(0x100000001b3);
        h ^= h >> 29;
    };
    let face = b.face(f);
    mix(face.surface.fingerprint());
    mix(face.sense as u64);
    for &l in &face.loops {
        mix(0xa5a5);
        for &c in &b.lp(l).coedges {
            let ed = b.edge(b.coedge(c).edge);
            mix(b.coedge(c).rev as u64);
            for t in [ed.t0, 0.5 * (ed.t0 + ed.t1), ed.t1] {
                let p = ed.curve.eval(t);
                mix(p.x.to_bits());
                mix(p.y.to_bits());
                mix(p.z.to_bits());
            }
        }
    }
    h
}

/// Volume tolerance: face integrals of curved and spline faces are good to
/// about 1e-5; allow 1e-3 of the operands.
fn tolerance(va: f64, vb: f64) -> f64 {
    1e-3 * (va.abs() + vb.abs())
}

/// Conservative box of a solid: vertices, sampled edges and a uv grid over
/// every curved face (edges alone miss a torus or sphere extreme), padded by
/// 0.5% of the diagonal.
fn loose_box(b: &Body) -> (Vec3, Vec3) {
    let (mut lo, mut hi) = b.bbox();
    for f in b.face_ids() {
        let face = b.face(f);
        if matches!(face.surface, Surface::Plane { .. }) {
            continue;
        }
        let (mut a, mut z) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for &l in &face.loops {
            for &c in &b.lp(l).coedges {
                let (t0, t1) = b.coedge_params(c);
                for k in 0..=8 {
                    let uv = b.coedge_uv(c, t0 + (t1 - t0) * k as f64 / 8.0, None);
                    a = Vec2::new(a.x.min(uv.x), a.y.min(uv.y));
                    z = Vec2::new(z.x.max(uv.x), z.y.max(uv.y));
                }
            }
        }
        if !(a.x.is_finite() && z.x.is_finite()) {
            continue;
        }
        let n = 32;
        let us: Vec<f64> = (0..=n).map(|i| a.x + (z.x - a.x) * i as f64 / n as f64).collect();
        let vs: Vec<f64> = (0..=n).map(|j| a.y + (z.y - a.y) * j as f64 / n as f64).collect();
        for p in face.surface.eval_grid(&us, &vs) {
            if p.is_finite() {
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
    }
    let pad = 0.005 * (hi - lo).len();
    (lo - Vec3::new(pad, pad, pad), hi + Vec3::new(pad, pad, pad))
}

fn box_overlap_volume(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> f64 {
    let lo = a.0.max(b.0);
    let hi = a.1.min(b.1);
    let d = hi - lo;
    if d.x <= 0.0 || d.y <= 0.0 || d.z <= 0.0 {
        0.0
    } else {
        d.x * d.y * d.z
    }
}

/// Check `result` against the bounds of `a op b`. `None` = plausible (or not
/// checkable).
pub fn check_result(a: &Body, b: &Body, op: Op, result: &Body) -> Option<VolumeMismatch> {
    if a.num_faces() == 0 || b.num_faces() == 0 {
        return None;
    }
    let va = solid_volume(a)?;
    let vb = solid_volume(b)?;
    if va <= 0.0 || vb <= 0.0 {
        return None;
    }
    let vr = solid_volume(result)?;
    let tol = tolerance(va, vb);
    let (ba, bb) = (loose_box(a), loose_box(b));
    let ov = box_overlap_volume(ba, bb).min(va).min(vb);
    // implied overlap vi in [0, ov]; vr as a function of vi
    let (lo, hi) = match op {
        Op::Union => (va + vb - ov, va + vb),
        Op::Cut => (va - ov, va),
        Op::Intersect => (0.0, ov),
    };
    let (lo, hi) = (lo - tol, hi + tol);
    if !(vr >= lo && vr <= hi) {
        return Some(VolumeMismatch { expected_min: lo.max(0.0).min(hi), expected_max: hi, got: vr, what: "volume" });
    }
    if op == Op::Union {
        let (alo, ahi) = a.bbox();
        let (blo, bhi) = b.bbox();
        // (the loose box: a sphere or cylinder bulge has no edge on it, so the edge box of a union that
        // keeps only a hemisphere of a tool would read as short of the tool's own box)
        let (rlo, rhi) = loose_box(result);
        let (lo, hi) = (alo.min(blo), ahi.max(bhi));
        let slack = 1e-2 * (hi - lo).len() + 1e-6;
        // positive when the result falls short of the operands
        let short = [rlo.x - lo.x, rlo.y - lo.y, rlo.z - lo.z, hi.x - rhi.x, hi.y - rhi.y, hi.z - rhi.z]
            .into_iter()
            .fold(0.0f64, f64::max);
        if short > slack {
            return Some(VolumeMismatch { expected_min: va.max(vb), expected_max: va + vb, got: vr, what: "box" });
        }
    }
    None
}
