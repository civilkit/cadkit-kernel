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

/// A result outside the bounds that `op` allows.
#[derive(Debug, Clone, PartialEq)]
pub struct VolumeMismatch {
    pub expected_min: f64,
    pub expected_max: f64,
    pub got: f64,
    /// `"volume"` or `"box"`.
    pub what: &'static str,
}

thread_local! {
    /// `[fine hits, fine misses, coarse hits, coarse misses]` (a miss is one face integral).
    static COUNTS: std::cell::Cell<[u64; 4]> = const { std::cell::Cell::new([0; 4]) };
}

/// Work counters of the volume check on this thread: `[fine hits, fine misses, coarse hits, coarse
/// misses]`, a miss being one face integral (for the work-counter tests).
pub fn work_counts() -> [u64; 4] {
    COUNTS.with(|c| c.get())
}

/// Net signed volume of a body (void shells count negative), `None` when it
/// is not a closed solid. `coarse` selects the cheap quadrature tier.
fn solid_volume(b: &Body, coarse: bool) -> Option<f64> {
    if b.num_faces() == 0 {
        return Some(0.0);
    }
    if !closed(b) {
        return None;
    }
    // Boolean chains keep most faces from one result to the next (a cut through a shell wall
    // changes a handful of faces among hundreds of spline faces, and a face integral of a spline
    // face costs milliseconds), so the volume is summed from per-face integrals memoised by a
    // hash of each face's surface and trimming.
    let tier = coarse as usize;
    let mut v = 0.0;
    for f in b.face_ids() {
        let (fv, hit) = ck_topo::mass::face_volume_memo(b, f, coarse);
        COUNTS.with(|c| {
            let mut a = c.get();
            a[2 * tier + !hit as usize] += 1;
            c.set(a);
        });
        v += fv;
    }
    v.is_finite().then_some(v)
}

/// Every edge has exactly two coedges (what `solid_volume` requires before it integrates anything).
fn closed(b: &Body) -> bool {
    b.num_faces() == 0 || !b.edge_ids().any(|e| b.edge_coedges(e).len() != 2)
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
        let g = crate::bvh::surface_grid_box(&face.surface, (a.x, z.x), (a.y, z.y), 32, true);
        lo = lo.min(g.min);
        hi = hi.max(g.max);
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

/// How far from a bound of the window (as a fraction of the tolerance) the coarse tier's volumes must
/// lie for its pass verdict to stand. The coarse quadrature is good to about 1e-6 of a volume (the
/// tolerance is 1e-3), so a tenth of the tolerance is far outside its error and still inside the
/// margin of every ordinary pass (a result that keeps all of an operand sits a whole tolerance from
/// the bound).
const GUARD: f64 = 0.1;

/// Admissible window of the result volume for `op`, tolerance included.
fn window(op: Op, va: f64, vb: f64, box_ov: f64) -> (f64, f64) {
    let tol = tolerance(va, vb);
    let ov = box_ov.min(va).min(vb);
    // implied overlap vi in [0, ov]; vr as a function of vi
    let (lo, hi) = match op {
        Op::Union => (va + vb - ov, va + vb),
        Op::Cut => (va - ov, va),
        Op::Intersect => (0.0, ov),
    };
    (lo - tol, hi + tol)
}

/// Check `result` against the bounds of `a op b`. `None` = plausible (or not
/// checkable).
///
/// Two tiers (the verdict only decides pass or fail and never feeds geometry): the volumes are first
/// summed with the cheap quadrature, and a pass that clears every bound by [`GUARD`] tolerances (and
/// the box test) is final. Everything else, every failure included, is decided by the exact rule.
pub fn check_result(a: &Body, b: &Body, op: Op, result: &Body) -> Option<VolumeMismatch> {
    if a.num_faces() == 0 || b.num_faces() == 0 {
        return None;
    }
    // an operand or result that is not a closed solid is not checkable: both tiers end in `None` without
    // a volume, so skip the integrals the tiers would otherwise do on the other bodies first
    if !closed(a) || !closed(b) || !closed(result) {
        log_verdict(op, true, &None);
        return None;
    }
    let (ba, bb) = (loose_box(a), loose_box(b));
    let box_ov = box_overlap_volume(ba, bb);
    let coarse_pass = coarse_passes(a, b, op, result, box_ov);
    let shadow = cfg!(feature = "sanity-shadow") || shadow_env();
    let verdict = if coarse_pass && !shadow { None } else { check_exact(a, b, op, result, box_ov) };
    if coarse_pass && shadow {
        // proof mode: the exact rule must agree with the coarse pass
        assert!(verdict.is_none(), "sanity: coarse tier passed what the exact tier rejects: {verdict:?}");
    }
    log_verdict(op, coarse_pass, &verdict);
    verdict
}

/// Tier 1: the cheap volumes put the result inside the window with the guard band to spare and the
/// box test passes.
fn coarse_passes(a: &Body, b: &Body, op: Op, result: &Body, box_ov: f64) -> bool {
    let vols = (|| {
        let va = solid_volume(a, true)?;
        let vb = solid_volume(b, true)?;
        let guard = GUARD * tolerance(va, vb);
        if !(va > guard && vb > guard) {
            return None;
        }
        let vr = solid_volume(result, true)?;
        let (lo, hi) = window(op, va, vb, box_ov);
        (vr >= lo + guard && vr <= hi - guard).then_some(())
    })();
    vols.is_some() && !box_short(a, b, op, result)
}

/// The exact rule (`None` = plausible or not checkable).
fn check_exact(a: &Body, b: &Body, op: Op, result: &Body, box_ov: f64) -> Option<VolumeMismatch> {
    let va = solid_volume(a, false)?;
    let vb = solid_volume(b, false)?;
    if va <= 0.0 || vb <= 0.0 {
        return None;
    }
    let vr = solid_volume(result, false)?;
    let (lo, hi) = window(op, va, vb, box_ov);
    if !(vr >= lo && vr <= hi) {
        return Some(VolumeMismatch { expected_min: lo.max(0.0).min(hi), expected_max: hi, got: vr, what: "volume" });
    }
    if box_short(a, b, op, result) {
        return Some(VolumeMismatch { expected_min: va.max(vb), expected_max: va + vb, got: vr, what: "box" });
    }
    None
}

/// Union only: the result's box falls short of the operands' boxes.
fn box_short(a: &Body, b: &Body, op: Op, result: &Body) -> bool {
    if op != Op::Union {
        return false;
    }
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
    short > slack
}

#[cfg(not(target_arch = "wasm32"))]
fn shadow_env() -> bool {
    std::env::var_os("CK_SANITY_SHADOW").is_some()
}
#[cfg(target_arch = "wasm32")]
fn shadow_env() -> bool {
    false
}

/// `CK_SANITY_LOG=1` (native): one line per check with its verdict, to diff runs.
#[cfg(not(target_arch = "wasm32"))]
fn log_verdict(op: Op, coarse: bool, v: &Option<VolumeMismatch>) {
    if std::env::var_os("CK_SANITY_LOG").is_some() {
        eprintln!("SANITY {:?} {} tier={}", op, v.as_ref().map_or("pass", |m| m.what), if coarse { "coarse" } else { "exact" });
    }
}
#[cfg(target_arch = "wasm32")]
fn log_verdict(_: Op, _: bool, _: &Option<VolumeMismatch>) {}
