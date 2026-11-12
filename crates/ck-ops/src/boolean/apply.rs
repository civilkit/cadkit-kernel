//! Apply (ck-ops/src/boolean/apply.rs): the boolean entry point plus the
//! pattern-instance helpers that feed it (instances are transformed copies
//! of a seed body, renamed `@<feature>.<k>`, unioned into the target).
//!
//! Pattern rules pinned by oracle probes (box seed `F2`, pattern `F3`):
//! - lpattern: instance k (1..count-1) = translation k·spacing·dir
//!   (`reverse` negates dir).
//! - cpattern: rotation about the axis by k·step (right-hand rule,
//!   `reverse` negates). `equal`: step = angle/count when angle = 360,
//!   else angle/(count-1); not `equal`: step = angle. count includes the
//!   seed. (count 3, 90°, equal → 45° steps; not equal → 90°; 360 equal →
//!   120°.)
//! - instance faces: `F2.S.e1@F3.1`, `@F3.2`; the result is rebuilt in
//!   face-walk edge order even for disjoint copies: `F2.S.e1|F2.S.e4` is
//!   #9 / #21 / #33 (12 edges per box, walk C0, C1, S.e1 → 4+4+2).

use super::assemble::{build, select, select_split};
use super::imprint::imprint;
use super::classify::{Class, Solid};
use super::Op;
use ck_core::xform::Xform;
use ck_geom::Vec3;
use ck_topo::body::Body;
use ck_topo::check::Issue;
#[cfg(test)]
use ck_topo::check::check;

#[derive(Debug, Clone, PartialEq)]
pub struct BoolOut {
    pub body: Body,
    /// Cut / intersect that changed nothing (feature warning `cutNoEffect`).
    pub no_effect: bool,
    /// The result is outside the volume / box bounds the operands allow
    /// ([`super::sanity`]): wrong but topologically valid. `None` = plausible.
    pub mismatch: Option<super::sanity::VolumeMismatch>,
    /// The two operands did not interact at all (no face pair came near): a tool
    /// inside the body's cavity or away from it. Such a cut is a valid no-op.
    pub disjoint: bool,
}

/// `a op b`. Errors: `booleanFailed` (topology could not be assembled).
///
/// Fallback ladder: marched intersection curves meet their neighbours only
/// to ~1e-6, so a configuration whose edge endpoints sit within a few
/// model tolerances of each other (a sliver where a cylinder grazes a
/// torus) can leave the arrangement open at `tol`. On `booleanFailed` the
/// operation is retried with the welding tolerance raised 10x and 100x
/// (the vertex pool, edge de-duplication and trims all follow `tol`); the
/// first result that assembles into a closed manifold wins.
pub fn boolean(a: &Body, b: &Body, op: Op, tol: f64) -> Result<BoolOut, String> {
    FAIL_DIAG.with(|d| d.borrow_mut().take());
    // Two operands that are the same solid at the same place (a part inserted twice at one pose): every face
    // pair is coincident, which the general path grinds through at minutes per attempt. The answer is known.
    if a.num_faces() > 0 && identical_solids(a, b, tol) {
        return Ok(match op {
            Op::Union | Op::Intersect => BoolOut { body: a.clone(), no_effect: false, mismatch: None, disjoint: false },
            Op::Cut => BoolOut { body: Body::new(), no_effect: false, mismatch: None, disjoint: false },
        });
    }
    // An operand with an edge between two distinct vertices no farther apart than the welding tolerance (a 1e-6 mm slab, the
    // clamp of a zero-depth extrude, is exactly that): the weld merges the two ends, the side faces collapse and the result is not a solid
    // (open or non-manifold shell, no mesh triangles, material dropped). Refuse it instead of returning that.
    if a.num_faces() > 0 && b.num_faces() > 0 {
        if let Some(len) = [a, b].iter().find_map(|x| thin_edge(x, tol)) {
            return Err(format!("booleanFailed;thinFeature={len:.3e}"));
        }
    }
    // Two free-form faces that lie on each other to a few thousandths of their size but are not the same surface (a part
    // inserted twice whose blades were fitted twice: gap 7e-3 mm on 100 mm blades) cannot be closed by any welding
    // tolerance of the ladder (the marched intersection crosses back and forth along the whole outline); the attempts
    // then grind through every tolerance and the robust mode for minutes. Refuse at once with the faces named.
    if a.num_faces() > 0 && b.num_faces() > 0 {
        let unweldable = tol * 1000.0;
        if let Some((fa, fb, gap)) = near_coincident_within(a, b, unweldable, |diag| (diag * 2e-3).min(0.1)) {
            return Err(format!("booleanFailed;nearCoincident={fa}|{fb}|{gap:.3e}"));
        }
    }
    // Operands far from the origin (a part at 1e6 mm) lose precision to the absolute
    // tolerances: welding and closest-point tests compare distances of ~1e-6 while f64
    // squared distances near 1e12 resolve only ~1e-4. Move such operands to a local
    // origin for the computation and move the result back. A part whose offset is small
    // against its own size keeps the direct path (bit-identical results at normal offsets).
    let local = local_origin(a, b);
    let out = match local {
        Some(o) => {
            let mut al = a.clone();
            let mut bl = b.clone();
            al.transform(&Xform::translation(-o));
            bl.transform(&Xform::translation(-o));
            boolean_inner(&al, &bl, op, tol).map(|mut r| {
                r.body.transform(&Xform::translation(o));
                r
            })
        }
        None => boolean_inner(a, b, op, tol),
    };
    out.map_err(|e| {
        // a failure after the time budget ran out is the budget's, not the geometry's
        if ck_core::budget::expired() {
            return BUDGET_EXCEEDED.to_string();
        }
        if e != "booleanFailed" {
            return e;
        }
        let diag = FAIL_DIAG.with(|d| d.borrow_mut().take());
        match near_coincident(a, b) {
            Some((fa, fb, gap)) => format!("booleanFailed;nearCoincident={fa}|{fb}|{gap:.3e}"),
            None => match diag {
                Some(d) => format!("booleanFailed;{d}"),
                None => e,
            },
        }
    })
}

/// Length of the shortest edge of `b` that joins two distinct vertices no farther apart than the welding tolerance (the
/// vertex pool merges points within `tol`, so such an edge collapses and takes its side faces with it), `None` when there is
/// none. The limit is `tol` plus a relative `1e-3` for the rounding of the transforms that placed the operand: the clamped
/// zero-depth slab is `MIN_DEPTH == 1e-6 == tol` thick. A valid sliver beyond that (1.2e-6 mm, from line-tangent cylinders)
/// is not refused: its ends stay apart in the pool.
pub(crate) fn thin_edge(b: &Body, tol: f64) -> Option<f64> {
    let lim = tol * (1.0 + 1e-3);
    let mut worst: Option<f64> = None;
    for e in b.edge_ids() {
        let ed = b.edge(e);
        if ed.v0 == ed.v1 || b.vertex(ed.v0).p.dist(b.vertex(ed.v1).p) > lim {
            continue;
        }
        let len = b.edge_length(e);
        if len <= lim && worst.map_or(true, |w| len < w) {
            worst = Some(len);
        }
    }
    worst
}

/// `a` and `b` are the same solid in the same place: equal element counts, and every face of `a` pairs with a
/// distinct face of `b` of the same surface kind and sense whose boundary samples lie on each other's surface
/// (within a few `tol`) and whose boundary has the same extent and length. Conservative: any doubt says no and the
/// general path runs.
pub(crate) fn identical_solids(a: &Body, b: &Body, tol: f64) -> bool {
    use ck_geom::surface::Surface;
    if a.num_faces() != b.num_faces() || a.edge_ids().count() != b.edge_ids().count() || a.vertex_ids().count() != b.vertex_ids().count() {
        return false;
    }
    let m = (tol * 10.0).max(1e-7);
    let (alo, ahi) = a.bbox();
    let (blo, bhi) = b.bbox();
    if (alo - blo).len() > m || (ahi - bhi).len() > m {
        return false;
    }
    struct Fp {
        pts: Vec<Vec3>,
        lo: Vec3,
        hi: Vec3,
        len: f64,
        nedges: usize,
    }
    let fp = |bd: &Body, f| {
        let mut pts = vec![];
        let mut len = 0.0;
        let es = bd.face_edges(f);
        for &e in &es {
            let ed = bd.edge(e);
            len += bd.edge_length(e);
            for k in 0..=8 {
                pts.push(ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0));
            }
        }
        let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
        for &p in &pts {
            lo = lo.min(p);
            hi = hi.max(p);
        }
        Fp { pts, lo, hi, len, nedges: es.len() }
    };
    let kind = |s: &Surface| std::mem::discriminant(s);
    let fa: Vec<_> = a.face_ids().map(|f| (f, fp(a, f))).collect();
    let fb: Vec<_> = b.face_ids().map(|f| (f, fp(b, f))).collect();
    let mut used = vec![false; fb.len()];
    for (f, pa) in &fa {
        let mut hit = None;
        for (j, (g, pb)) in fb.iter().enumerate() {
            if used[j]
                || a.face(*f).sense != b.face(*g).sense
                || kind(&a.face(*f).surface) != kind(&b.face(*g).surface)
                || pa.nedges != pb.nedges
                || (pa.lo - pb.lo).len() > m
                || (pa.hi - pb.hi).len() > m
                || (pa.len - pb.len).abs() > m * pa.len.max(1.0)
            {
                continue;
            }
            if pb.pts.iter().all(|&p| a.face(*f).surface.distance(p) <= m) && pa.pts.iter().all(|&p| b.face(*g).surface.distance(p) <= m) {
                hit = Some(j);
                break;
            }
        }
        match hit {
            Some(j) => used[j] = true,
            None => return false,
        }
    }
    true
}

/// Centre of the operands' combined box when it is far from the origin relative to their
/// size (the offset a part at 1e6 mm has against a 30 mm body); `None` for ordinary parts.
fn local_origin(a: &Body, b: &Body) -> Option<Vec3> {
    let (alo, ahi) = a.bbox();
    let (blo, bhi) = b.bbox();
    let (lo, hi) = (alo.min(blo), ahi.max(bhi));
    let c = (lo + hi) * 0.5;
    let diag = (hi - lo).len().max(1.0);
    (c.len() > 1e4 * diag).then_some(c)
}

thread_local! {
    /// Where the last assembled body of a failed boolean was not closed: `edgeFaces=<f1>|<f2>..;issue=<kind>;at=<x>|<y>|<z>`
    /// (the faces that border the offending edges, the first kind of defect, the point of the first one), taken by
    /// [`boolean`] into the error so the caller can say which faces to look at.
    static FAIL_DIAG: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
}

/// The pair of free-form faces (one of `a`, one of `b`) that nearly coincide: every sampled edge point of the `b` face is
/// within 0.1 of the `a` face's surface, yet the faces are not the same surface (gap beyond welding tolerance). Marched
/// intersection of two such walls crosses back and forth along the whole outline, so the arrangement cannot close; the pair is
/// the one to make exactly coincident (same curve) or to move apart. Smallest worst-case gap wins. Diagnosis only (after a failure).
fn near_coincident(a: &Body, b: &Body) -> Option<(String, String, f64)> {
    near_coincident_within(a, b, NEAR_FLOOR, |_| 0.1)
}

/// Gaps below this are the same surface for [`near_coincident_within`].
const NEAR_FLOOR: f64 = 1e-7;

/// [`near_coincident`] with the largest gap that still counts given per tool-face size (`limit(diagonal)`).
fn near_coincident_within(a: &Body, b: &Body, min_gap: f64, limit: impl Fn(f64) -> f64) -> Option<(String, String, f64)> {
    use ck_geom::surface::Surface;
    let freeform = |bd: &Body, f| matches!(bd.face(f).surface, Surface::Extrusion { .. } | Surface::BSpline(_) | Surface::Offset { .. } | Surface::Screw { .. });
    let fa: Vec<_> = a.face_ids().filter(|&f| freeform(a, f)).collect();
    let fb: Vec<_> = b.face_ids().filter(|&f| freeform(b, f)).collect();
    if fa.is_empty() || fb.is_empty() || fa.len() * fb.len() > 2000 {
        return None;
    }
    let samples = |f| -> Vec<Vec3> {
        let mut out = vec![];
        for e in b.face_edges(f) {
            let ed = b.edge(e);
            for k in 0..=8 {
                out.push(ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0));
            }
        }
        out
    };
    let mut best: Option<(String, String, f64)> = None;
    for &g in &fb {
        let pts = samples(g);
        let max_gap = {
            let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
            for &p in &pts {
                lo = lo.min(p);
                hi = hi.max(p);
            }
            limit((hi - lo).len())
        };
        for &f in &fa {
            let mut worst = 0.0f64;
            for &p in &pts {
                worst = worst.max(a.face(f).surface.distance(p));
                if worst > max_gap {
                    break;
                }
            }
            if worst > max_gap || worst < NEAR_FLOOR {
                continue;
            }
            if best.as_ref().is_none_or(|x| worst < x.2) {
                best = Some((a.face_id_str(f), b.face_id_str(g), worst));
            }
        }
    }
    // The tightest pair decides: when it is closer than `min_gap` the welding ladder may still close it, so nothing is
    // refused (reporting a looser pair of the same walls instead would name a gap that is not the true one).
    best.filter(|x| x.2 >= min_gap)
}

/// The ordinary ladder, then (only when it failed to assemble) the robust one: the same attempts with the
/// sliver-tolerant machinery of [`super::robust`] switched on. Results of operations that already succeed are
/// therefore untouched (same bits, same speed); the robust mode only ever turns a `booleanFailed` into a result.
fn boolean_inner(a: &Body, b: &Body, op: Op, tol: f64) -> Result<BoolOut, String> {
    let started = ck_core::budget::mark();
    let r = boolean_plain(a, b, op, tol);
    if !matches!(&r, Err(e) if e == "booleanFailed") {
        return r;
    }
    let _guard = super::RobustGuard::new();
    for t in [tol * 10.0, tol, tol * 100.0] {
        if ck_core::budget::expired() {
            return Err(BUDGET_EXCEEDED.into());
        }
        // A boolean that has already ground on for ROBUST_RETRY_MS without assembling is not going to be rescued
        // by three more full attempts (a failing fuse of two large bodies took minutes to report): say so.
        let spent = ck_core::budget::since(started);
        if spent > ROBUST_RETRY_MS {
            return Err(format!("{BUDGET_EXCEEDED};elapsedMs={spent:.0};budgetMs={ROBUST_RETRY_MS:.0}"));
        }
        super::bool_dbg!("robust retry at tolerance {:e}", t);
        if let Ok(out) = boolean_at(a, b, op, t) {
            return Ok(if out.no_effect { out } else { sanity(a, b, op, out) });
        }
    }
    r
}

fn boolean_plain(a: &Body, b: &Body, op: Op, tol: f64) -> Result<BoolOut, String> {
    // A free-form operand (B-spline or offset faces) meets the other solid along marched curves whose
    // ends agree with their neighbours only to ~1e-6 to 1e-5, so at the base welding tolerance the first
    // attempt of such a cut or union tends to leave the arrangement open (a full wasted attempt before the
    // 10x retry, 30 to 50 percent of the boolean on the large sample's cuts): start the ladder at 10x, and
    // fall back to the base tolerance only if the whole raised ladder fails. (Only for the default welding
    // tolerance: callers that pass a larger one, the shell and the lip, chose it already.)
    let free_form = |b: &Body| b.face_ids().any(|f| matches!(b.face(f).surface, ck_geom::surface::Surface::BSpline(_) | ck_geom::surface::Surface::Offset { .. }));
    if tol <= 1.5e-6 && (free_form(a) || free_form(b)) {
        let r = boolean_ladder(a, b, op, tol * 10.0);
        if r.is_ok() || ck_core::budget::expired() {
            return r;
        }
        return boolean_ladder(a, b, op, tol);
    }
    boolean_ladder(a, b, op, tol)
}

fn boolean_ladder(a: &Body, b: &Body, op: Op, tol: f64) -> Result<BoolOut, String> {
    #[cfg(not(target_arch = "wasm32"))]
    let clock = std::time::Instant::now();
    let first = boolean_at(a, b, op, tol);
    #[cfg(not(target_arch = "wasm32"))]
    super::bool_dbg!("attempt at tolerance {:e}: {:.1}s", tol, clock.elapsed().as_secs_f64());
    match &first {
        Err(e) if e == "booleanFailed" => {
            for k in [10.0, 100.0] {
                super::bool_dbg!("retrying with tolerance {:e}", tol * k);
                let r = boolean_at(a, b, op, tol * k);
                #[cfg(not(target_arch = "wasm32"))]
                super::bool_dbg!("attempt at tolerance {:e}: {:.1}s ({})", tol * k, clock.elapsed().as_secs_f64(), if r.is_ok() { "ok" } else { "failed" });
                if let Ok(out) = r {
                    return Ok(sanity(a, b, op, out));
                }
            }
            first
        }
        Ok(out) if !out.no_effect => {
            let out = sanity(a, b, op, out.clone());
            if out.mismatch.is_none() {
                return Ok(out);
            }
            // wrong but valid: a coincident rim can classify differently at a
            // looser welding tolerance; keep the first plausible result
            for k in [10.0, 100.0] {
                if let Ok(r) = boolean_at(a, b, op, tol * k) {
                    let r = sanity(a, b, op, r);
                    if r.mismatch.is_none() {
                        return Ok(r);
                    }
                }
            }
            // the re-fusing of two coincident same-surface fragments (two cone bands on
            // one cone surface, seams at different angles) can leave a degenerate face
            // and drop material: retry as a split-only pass, which keeps both faces
            if let Ok(r) = boolean_at_merge(a, b, op, tol, false) {
                let r = sanity(a, b, op, r);
                if r.mismatch.is_none() {
                    return Ok(r);
                }
            }
            Ok(out)
        }
        _ => first,
    }
}

/// Attach the [`super::sanity`] verdict to a finished result.
fn sanity(a: &Body, b: &Body, op: Op, mut out: BoolOut) -> BoolOut {
    if !out.no_effect {
        out.mismatch = super::sanity::check_result(a, b, op, &out.body);
        if let Some(m) = &out.mismatch {
            super::bool_dbg!("sanity: {} got {} outside [{}, {}]", m.what, m.got, m.expected_min, m.expected_max);
        }
    }
    out
}

/// A lone sphere (a revolved semicircle: one face, poles and a seam) whose pole lies on a surface of the
/// other operand meets it in a circle through the poles, which the face's uv polygon cannot hold (the
/// pole is a whole edge in uv): the trim fragments collapse and the sphere face is dropped. A sphere is
/// the same point set in any orientation, so turn it about its centre until no pole sits on the other
/// body (the face keeps its name; only its seam and poles move).
fn unpole(a: Body, other: &Body) -> Body {
    use ck_geom::surface::Surface;
    if a.num_faces() != 1 {
        return a;
    }
    let f = a.face_ids().next().unwrap();
    let Surface::Sphere { frame, r } = &a.face(f).surface else { return a };
    let (o, z, r) = (frame.o, frame.z, *r);
    // Two whole spheres meet in a circle in the plane normal to the line of their centres. Turned so its axis lies
    // along that line, each sphere sees the circle as a latitude circle (it crosses the seam once, the case the
    // face walk handles); with the axis across the line the circle crosses the seam twice, the seam splits into
    // three pieces inside the cap and the cap's two halves are walked as separate cycles.
    if other.num_faces() == 1 {
        let g = other.face_ids().next().unwrap();
        if let Surface::Sphere { frame: fo, .. } = &other.face(g).surface {
            let d = fo.o - o;
            if d.len() > 1e-9 {
                let n = d.normalize();
                let ax = z.cross(n);
                if ax.len() > 1e-9 {
                    let mut c = a.clone();
                    c.transform(&Xform::rotation(o, ax.normalize(), z.dot(n).clamp(-1.0, 1.0).acos()));
                    return c;
                }
                return a;
            }
        }
    }
    let near = |p: Vec3| other.face_ids().any(|g| other.face(g).surface.distance(p) < 1e-3 * r.max(1.0));
    if !near(o + z * r) && !near(o - z * r) {
        return a;
    }
    // first choice: the axis along the normal of a plane that cuts the sphere (the section is then a
    // latitude circle, exact pcurve, hemisphere cap); then arbitrary tilts
    let mut tries: Vec<(Vec3, f64)> = vec![];
    for g in other.face_ids() {
        if let Surface::Plane { frame } = &other.face(g).surface {
            let n = frame.z;
            let ax = z.cross(n);
            if ax.len() > 1e-9 && (o - frame.o).dot(n).abs() < r {
                tries.push((ax, z.dot(n).clamp(-1.0, 1.0).acos()));
            }
        }
    }
    tries.extend([(Vec3::new(1.0, 1.37, 0.61), 0.7), (Vec3::new(0.3, -1.0, 1.9), 1.1), (Vec3::new(-1.0, 0.45, 0.8), 2.3)]);
    for (axis, ang) in tries {
        let x = Xform::rotation(o, axis.normalize(), ang);
        let mut c = a.clone();
        c.transform(&x);
        let Surface::Sphere { frame, .. } = &c.face(f).surface else { continue };
        let (p, q) = (frame.o + frame.z * r, frame.o - frame.z * r);
        if !near(p) && !near(q) {
            return c;
        }
    }
    a
}

/// Wall-clock ms after which the robust retries of a failed boolean are not started (only measurable with a host clock).
const ROBUST_RETRY_MS: f64 = 12_000.0;

/// Error code of a boolean stopped by the time budget (`ck_core::budget`).
pub const BUDGET_EXCEEDED: &str = "booleanBudgetExceeded";

/// Midpoints of the edges that coincide with another edge of the same operand (the operand's own
/// self-contact: two half rings touching along a line). A boolean's four-face edge between such a pair
/// is inherited, not a failure. Found through a spatial hash of the midpoints, so the cost is linear in
/// the operand's edge count, not the quadratic all-pairs scan that ran over the whole host.
fn inherited_contact_mids(a: &Body, b: &Body) -> Vec<Vec3> {
    let mut out = vec![];
    for x in [a, b] {
        let mids: Vec<(Vec3, Vec3, Vec3)> = x
            .edge_ids()
            .map(|e| {
                let ed = x.edge(e);
                (ed.curve.eval(0.5 * (ed.t0 + ed.t1)), ed.curve.eval(ed.t0), ed.curve.eval(ed.t1))
            })
            .collect();
        if mids.is_empty() {
            continue;
        }
        let scale = mids.iter().map(|(m, s, e)| m.len().max(s.len()).max(e.len())).fold(0.0f64, f64::max);
        let cell = 1e-5 * (1.0 + scale);
        let key = |p: Vec3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64);
        let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<usize>> = Default::default();
        for (i, (m, _, _)) in mids.iter().enumerate() {
            grid.entry(key(*m)).or_default().push(i);
        }
        for (i, (m1, s1, e1)) in mids.iter().enumerate() {
            let t = 1e-6 * (1.0 + m1.len());
            let (cx, cy, cz) = key(*m1);
            let mut coincides = false;
            'cells: for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let Some(list) = grid.get(&(cx + dx, cy + dy, cz + dz)) else { continue };
                        for &j in list {
                            if j <= i {
                                continue;
                            }
                            let (m2, s2, e2) = mids[j];
                            if m1.dist(m2) < t && ((s1.dist(s2) < t && e1.dist(e2) < t) || (s1.dist(e2) < t && e1.dist(s2) < t)) {
                                coincides = true;
                                break 'cells;
                            }
                        }
                    }
                }
            }
            if coincides {
                out.push(*m1);
            }
        }
    }
    out
}

fn boolean_at(a: &Body, b: &Body, op: Op, tol: f64) -> Result<BoolOut, String> {
    boolean_at_merge(a, b, op, tol, true)
}

/// [`boolean_at`] with the re-fusing of coincident same-facing fragments switchable off
/// (the split-only pass: every fragment stays a face of its own).
fn boolean_at_merge(a: &Body, b: &Body, op: Op, tol: f64, merge: bool) -> Result<BoolOut, String> {
    if ck_core::budget::expired() {
        return Err(BUDGET_EXCEEDED.into());
    }
    if b.num_faces() == 0 {
        return Ok(BoolOut { body: a.clone(), no_effect: op != Op::Union, mismatch: None, disjoint: false });
    }
    if a.num_faces() == 0 {
        return Ok(match op {
            Op::Union => BoolOut { body: b.clone(), no_effect: false, mismatch: None, disjoint: false },
            _ => BoolOut { body: Body::new(), no_effect: true, mismatch: None, disjoint: false },
        });
    }
    // (results keep the caller's bodies for the no-effect returns)
    let (a0, b0) = (a, b);
    let a1 = relocate_seams(a0, b0, dropped(op, 0), tol);
    let b1 = relocate_seams(b0, a0, dropped(op, 1), tol);
    // two-lap closed-spline walls (extruded sketch splines): every closest-parameter search on such a
    // curve may land on the other lap, so edge membership tests (a section lying on a face boundary)
    // and loop sampling fail; cut the curves down to the lap each face / edge uses
    let (a1, b1) = (crate::offset::single_lap_body(&a1), crate::offset::single_lap_body(&b1));
    let (a1, b1) = (unpole(a1.clone(), &b1), unpole(b1.clone(), &a1));
    let (a, b) = (&a1, &b1);
    let arr = imprint(a, b, tol);
    if ck_core::budget::expired() {
        return Err(BUDGET_EXCEEDED.into());
    }
    if arr.interacting_pairs == 0 {
        // no face interaction: containment decides
        let pa = first_point(a);
        let pb = first_point(b);
        let a_in_b = ck_topo::region::classify(b, pa, tol) == ck_topo::region::Containment::Inside;
        let b_in_a = ck_topo::region::classify(a, pb, tol) == ck_topo::region::Containment::Inside;
        match op {
            Op::Cut if !a_in_b && !b_in_a => return Ok(BoolOut { body: a0.clone(), no_effect: true, mismatch: None, disjoint: true }),
            Op::Intersect if !a_in_b && !b_in_a => return Ok(BoolOut { body: Body::new(), no_effect: true, mismatch: None, disjoint: true }),
            _ => {} // nested: the general path (void shells / swallowed bodies)
        }
    }
    let faces = if merge {
        select(&arr, [a, b], op)?
    } else {
        select_split(&arr, [a, b], op)?
    };
    if ck_core::budget::expired() {
        return Err(BUDGET_EXCEEDED.into());
    }
    // nothing removed from A and nothing added from B → unchanged
    let a_faces = a.num_faces();
    // (a face split by an imprint and rejoined is listed with the merged
    // faces but walked as untouched)
    let whole_a = |f: &super::assemble::OutFace| f.group == 0 || (f.group == 4 && f.walk.len() == 1 && f.walk[0].0 .0 == 0);
    let untouched_a = faces.iter().filter(|f| whole_a(f)).count();
    if op == Op::Cut && untouched_a == a_faces && faces.iter().all(|f| whole_a(f)) {
        return Ok(BoolOut { body: a0.clone(), no_effect: true, mismatch: None, disjoint: false });
    }
    let mut body = build(&arr, &faces);
    // (only the connectivity issues are used below)
    let mut issues = ck_topo::check::check_topology(&body, tol);
    // two lumps of one operand touching along a line (the two half rings of a thin-wall extrude of two
    // arcs) have distinct but coincident edges that the arrangement welds into one edge of four faces:
    // that edge is the input's own contact, not a failure of the boolean (checked lazily, only when
    // a four-face edge shows up)
    let mut inherited: Option<Vec<Vec3>> = None;
    let mut is_inherited = |m: Vec3| -> bool {
        let list = inherited.get_or_insert_with(|| inherited_contact_mids(a, b));
        list.iter().any(|p| p.dist(m) < 1e-6 * (1.0 + m.len()))
    };
    // parts of the result that touch along an edge: split the four-face edge into one per material wedge
    // (the contacts the operands already had as two coincident edges are split too, with CK_SPLIT_INHERITED unset: no)
    let split_inherited = true;
    {
        let contact: Vec<_> = issues
            .iter()
            .filter_map(|i| match i {
                Issue::NonManifoldEdge(e, 4) => Some(*e),
                _ => None,
            })
            .filter(|&e| {
                let ed = body.edge(e);
                split_inherited || !is_inherited(ed.curve.eval(0.5 * (ed.t0 + ed.t1)))
            })
            .collect();
        if !contact.is_empty() && super::contact::split_contact_edges(&mut body, &contact) > 0 {
            ck_topo::region::rebuild_lumps(&mut body);
            issues = ck_topo::check::check_topology(&body, tol);
        }
    }
    let bad_issues: Vec<_> = issues
        .into_iter()
        .filter(|i| matches!(i, Issue::OpenEdge(_) | Issue::NonManifoldEdge(..) | Issue::BrokenLoop(..) | Issue::BadPairing(_)))
        .filter(|i| match i {
            Issue::NonManifoldEdge(e, 4) => {
                let ed = body.edge(*e);
                let m = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
                !is_inherited(m)
            }
            _ => true,
        })
        .collect();
    // two solids that touch only along an edge (or a line of tangency): every face is kept whole and the
    // contact edge carries four faces. The union is the two lumps side by side, not a failure.
    if op == Op::Union
        && !bad_issues.is_empty()
        && bad_issues.iter().all(|i| matches!(i, Issue::NonManifoldEdge(_, 4)))
        && faces.iter().all(|f| f.group == 0 || f.group == 2)
        && faces.len() == a.num_faces() + b.num_faces()
    {
        let mut out = a0.clone();
        out.append(b0);
        return Ok(BoolOut { body: out, no_effect: false, mismatch: None, disjoint: false });
    }
    if !bad_issues.is_empty() {
        super::bool_dbg!("arrangement:\n{}", arr.dump());
        for i in &bad_issues {
            if let Issue::OpenEdge(e) | Issue::NonManifoldEdge(e, _) | Issue::BadPairing(e) = i {
                let ed = body.edge(*e);
                let faces: Vec<_> = body.edge_coedges(*e).iter().map(|&c| body.face_id_str(body.coedge_face(c))).collect();
                super::bool_dbg!(
                    "  bad edge {:?}: {} nco={} seam={} v0={:?} v1={:?} t[{:.5},{:.5}] p0 {:?} p1 {:?} faces {:?}",
                    e,
                    ed.curve.kind(),
                    body.edge_coedges(*e).len(),
                    body.is_seam(*e),
                    ed.v0.idx(),
                    ed.v1.idx(),
                    ed.t0,
                    ed.t1,
                    ed.curve.eval(ed.t0),
                    ed.curve.eval(ed.t1),
                    faces
                );
            }
        }
        super::bool_dbg!("assembled body invalid: {} issues, first {:?}", bad_issues.len(), &bad_issues[..bad_issues.len().min(6)]);
        let mut names: Vec<String> = vec![];
        let mut first: Option<(&'static str, Vec3)> = None;
        for i in &bad_issues {
            let (e, kind) = match i {
                Issue::OpenEdge(e) => (*e, "openEdge"),
                Issue::NonManifoldEdge(e, _) => (*e, "nonManifoldEdge"),
                Issue::BadPairing(e) => (*e, "badPairing"),
                _ => continue,
            };
            let ed = body.edge(e);
            first.get_or_insert((kind, ed.curve.eval(0.5 * (ed.t0 + ed.t1))));
            for &c in body.edge_coedges(e).iter() {
                let n = body.face_id_str(body.coedge_face(c));
                if !names.contains(&n) && names.len() < 8 {
                    names.push(n);
                }
            }
        }
        if let Some((kind, p)) = first {
            FAIL_DIAG.with(|d| *d.borrow_mut() = Some(format!("edgeFaces={};issue={kind};at={:.4}|{:.4}|{:.4}", names.join("|"), p.x, p.y, p.z)));
        }
        return Err("booleanFailed".into());
    }
    // A shell whose Euler characteristic is odd (V-E+F-(L-F)) cannot be a closed
    // orientable boundary: an exact tangency leaves the result non-manifold at the
    // contact (three sheets meet along a line, or two loops meet at a point). The
    // geometry is assembled, but the body is not valid; refuse it structurally
    // rather than return an invalid solid with a plausible volume.
    if let Some(chi) = odd_shell(&body) {
        return Err(format!(
            "booleanTangent;euler={chi};hint=the operands touch tangentially (a wall grazes the other with no crossing); move one operand or widen the model tolerance"
        ));
    }
    Ok(BoolOut { body, no_effect: false, mismatch: None, disjoint: false })
}

/// The Euler characteristic of the first shell component for which
/// `V - E + F - (L - F)` is odd or above 2 (`None` when every component is
/// consistent). Seam edges are one edge with two coedges of one face.
fn odd_shell(b: &Body) -> Option<i64> {
    use std::collections::{HashMap, HashSet};
    let faces: Vec<_> = b.face_ids().collect();
    let mut by_edge: HashMap<usize, Vec<usize>> = HashMap::default();
    for (i, &f) in faces.iter().enumerate() {
        for e in b.face_edges(f) {
            by_edge.entry(e.idx()).or_default().push(i);
        }
    }
    let mut seen = vec![false; faces.len()];
    for start in 0..faces.len() {
        if seen[start] {
            continue;
        }
        let mut comp = vec![start];
        seen[start] = true;
        let mut k = 0;
        while k < comp.len() {
            let i = comp[k];
            k += 1;
            for e in b.face_edges(faces[i]) {
                for &j in by_edge.get(&e.idx()).map_or(&[][..], |v| v) {
                    if !seen[j] {
                        seen[j] = true;
                        comp.push(j);
                    }
                }
            }
        }
        let (mut es, mut vs, mut loops) = (HashSet::new(), HashSet::new(), 0i64);
        let mut curved = false;
        for &i in &comp {
            let f = faces[i];
            curved |= !matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. });
            loops += b.face(f).loops.len() as i64;
            for e in b.face_edges(f) {
                if es.insert(e.idx()) {
                    let ed = b.edge(e);
                    vs.insert(ed.v0.idx());
                    vs.insert(ed.v1.idx());
                }
            }
        }
        // only a smooth (curved) tangency: two flat pieces touching along an edge or a
        // point is a separate contact case, not a wall grazing another wall
        if !curved {
            continue;
        }
        let f = comp.len() as i64;
        let chi = vs.len() as i64 - es.len() as i64 + f - (loops - f);
        if chi % 2 != 0 || chi > 2 {
            return Some(chi);
        }
    }
    None
}

/// The class of the other body's region whose faces `op` discards on side
/// `side` (see `assemble::select`'s keep rule).
fn dropped(op: Op, side: usize) -> Class {
    match (op, side) {
        (Op::Union, _) | (Op::Cut, 0) => Class::In,
        (Op::Intersect, _) | (Op::Cut, _) => Class::Out,
    }
}

/// Move the seam of every cylinder / cone face whose two boundary circles
/// both lie in the region the boolean discards (through holes: the tool's
/// caps) to the angle where the boolean's closed intersection circles
/// start (`imprint::closed_start`, 0.62·2π of the face frame). The
/// oracle has no seams, so such a hole is bounded by single closed circles;
/// with the seam there our intersection circles are not split at it and
/// export (and tessellate) as one edge. Faces whose circles survive keep
/// their seam (their circles start at c + r·x).
pub fn relocate_seams(b: &Body, other: &Body, drop: Class, tol: f64) -> Body {
    use ck_geom::curve::Curve;
    use ck_geom::pcurve::{exact_pcurve, pcurve_of};
    use ck_geom::surface::Surface;
    use std::f64::consts::TAU;
    let mut out = b.clone();
    let solid = Solid::new(other, tol);
    let (t0, _) = super::imprint::closed_start(TAU);
    let candidate = |f: ck_topo::body::FaceId| -> Option<(ck_geom::Frame, Vec<ck_topo::body::EdgeId>, Vec<ck_topo::body::EdgeId>)> {
        let face = b.face(f);
        let frame = match &face.surface {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => *frame,
            _ => return None,
        };
        if face.loops.len() != 1 {
            return None;
        }
        let cs = b.lp(face.loops[0]).coedges.clone();
        let mut seams = vec![];
        let mut circles = vec![];
        for &c in &cs {
            let e = b.coedge(c).edge;
            if b.is_seam(e) {
                if !seams.contains(&e) {
                    seams.push(e);
                }
            } else {
                circles.push(e);
            }
        }
        if seams.len() != 1 || circles.len() != 2 {
            return None;
        }
        let full = |e: ck_topo::body::EdgeId| {
            let ed = b.edge(e);
            matches!(ed.curve, Curve::Circle { .. }) && ed.v0 == ed.v1 && ((ed.t1 - ed.t0) - TAU).abs() < 1e-9
        };
        if !circles.iter().all(|&e| full(e)) {
            return None;
        }
        let in_drop = circles.iter().all(|&e| {
            let p = b.edge_mid(e);
            solid.classify(p, frame.z) == drop
        });
        if !in_drop {
            return None;
        }
        Some((frame, seams, circles))
    };
    let cands: Vec<_> = b.face_ids().filter_map(|f| candidate(f).map(|c| (f, c))).collect();
    let ids: Vec<ck_topo::body::FaceId> = cands.iter().map(|c| c.0).collect();
    // a face sharing a circle (so its seam end vertex) with a seamed
    // cylinder / cone that keeps its seam cannot move its circle starts
    let mut ok: Vec<bool> = vec![true; cands.len()];
    loop {
        let mut changed = false;
        for (i, (f, c)) in cands.iter().enumerate() {
            if !ok[i] {
                continue;
            }
            let blocked = c.2.iter().any(|&e| {
                b.edge_faces(e).into_iter().any(|g| {
                    g != *f && matches!(b.face(g).surface, Surface::Cylinder { .. } | Surface::Cone { .. }) && ids.iter().position(|&x| x == g).map_or(true, |k| !ok[k])
                })
            });
            if blocked {
                ok[i] = false;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    for (i, (f, (frame, seams, circles))) in cands.into_iter().enumerate() {
        if !ok[i] {
            continue;
        }
        let s = seams[0];
        let radial = frame.x * t0.cos() + frame.y * t0.sin();
        for &e in &circles {
            let ed = b.edge(e).clone();
            let Curve::Circle { frame: cf, r } = ed.curve else { continue };
            let target = cf.o + radial * r;
            let mut t = ed.curve.closest_param(target);
            // same start convention as imprint: the run [t, t + 2π]
            t = t0 + (t - t0 - TAU * ((t - t0) / TAU).round());
            out.edges[e.idx()].t0 = t;
            out.edges[e.idx()].t1 = t + TAU;
            out.vertices[ed.v0.idx()].p = ed.curve.eval(t);
            // a radial line from the circle's centre to the moved vertex (the seam of
            // a revolved disc cap) turns with it, or the cap's boundary no longer
            // closes: its uv polygon loses a wedge and point classification against
            // this body goes wrong (a hose cone cut read a point outside as inside)
            for e2 in b.edge_ids() {
                let x = b.edge(e2);
                if e2 == e || e2 == s || circles.contains(&e2) || !matches!(x.curve, Curve::Line { .. }) {
                    continue;
                }
                let other = if x.v0 == ed.v0 {
                    x.v1
                } else if x.v1 == ed.v0 {
                    x.v0
                } else {
                    continue;
                };
                if b.vertex(other).p.dist(cf.o) > tol * 10.0 {
                    continue;
                }
                let (pa, pb) = (out.vertex(x.v0).p, out.vertex(x.v1).p);
                let len = pa.dist(pb);
                out.edges[e2.idx()].curve = Curve::Line { o: pa, d: (pb - pa) / len };
                out.edges[e2.idx()].t0 = 0.0;
                out.edges[e2.idx()].t1 = len;
            }
        }
        let se = b.edge(s).clone();
        let (pa, pb) = (out.vertex(se.v0).p, out.vertex(se.v1).p);
        let len = pa.dist(pb);
        out.edges[s.idx()].curve = Curve::Line { o: pa, d: (pb - pa) / len };
        out.edges[s.idx()].t0 = 0.0;
        out.edges[s.idx()].t1 = len;
        // pcurves of the touched faces (cylinder loop kept uv-continuous)
        let mut faces: Vec<ck_topo::body::FaceId> = vec![f];
        for &e in &circles {
            for g in b.edge_faces(e) {
                if !faces.contains(&g) {
                    faces.push(g);
                }
            }
        }
        for g in faces {
            let surf = out.face(g).surface.clone();
            let (pu, pv) = surf.periods();
            for l in out.face(g).loops.clone() {
                let mut prev_end: Option<ck_geom::Vec2> = None;
                for c in out.lp(l).coedges.clone() {
                    let ce = out.coedge(c).clone();
                    let ed = out.edge(ce.edge).clone();
                    let pc = exact_pcurve(&surf, &ed.curve, tol * 10.0).or_else(|| pcurve_of(&surf, &ed.curve, ed.t0, ed.t1, tol).ok());
                    let pc = pc.map(|mut pc| {
                        let (ta, tb) = if ce.rev { (ed.t1, ed.t0) } else { (ed.t0, ed.t1) };
                        if let Some(prev) = prev_end {
                            let st = pc.eval(ta);
                            let du = pu.map_or(0.0, |p| ((prev.x - st.x) / p).round() * p);
                            let dv = pv.map_or(0.0, |p| ((prev.y - st.y) / p).round() * p);
                            if du != 0.0 || dv != 0.0 {
                                pc = super::assemble::shift2(&pc, ck_geom::Vec2::new(du, dv));
                            }
                        }
                        prev_end = Some(pc.eval(tb));
                        pc
                    });
                    out.coedges[c.idx()].pcurve = pc;
                }
            }
        }
    }
    out
}

fn first_point(b: &Body) -> Vec3 {
    b.vertex_ids().next().map(|v| b.vertex(v).p).unwrap_or(Vec3::ZERO)
}

// ------------------------------------------------------------ patterns

/// lpattern instance transforms (k = 1..count-1).
pub fn linear_instances(dir: Vec3, spacing: f64, count: usize, reverse: bool) -> Vec<Xform> {
    let d = dir.normalize() * if reverse { -1.0 } else { 1.0 };
    (1..count.max(1)).map(|k| Xform::translation(d * (spacing * k as f64))).collect()
}

/// cpattern step in degrees.
pub fn circular_step(angle_deg: f64, count: usize, equal: bool) -> f64 {
    if !equal {
        return angle_deg;
    }
    // GUESS: "full circle" test exact to 1e-9 degrees
    if (angle_deg.abs() - 360.0).abs() < 1e-9 {
        angle_deg / count.max(1) as f64
    } else {
        angle_deg / (count.max(2) - 1) as f64
    }
}

/// cpattern instance transforms about the axis through `o` along `axis`.
pub fn circular_instances(o: Vec3, axis: Vec3, angle_deg: f64, count: usize, equal: bool, reverse: bool) -> Vec<Xform> {
    let step = circular_step(angle_deg, count, equal) * if reverse { -1.0 } else { 1.0 };
    (1..count.max(1)).map(|k| Xform::rotation(o, axis, (step * k as f64).to_radians())).collect()
}

/// Transformed, renamed copy of `seed` (`@<feature>.<k>`, k from 1).
pub fn instance(seed: &Body, x: &Xform, feature: &str, k: usize) -> Body {
    let mut b = seed.clone();
    b.transform(x);
    b.push_instance(&format!("{}.{}", feature, k));
    b
}

/// Union every instance of `seed` into `target` (instances in order).
pub fn apply_instances(target: &Body, seed: &Body, xs: &[Xform], feature: &str, tol: f64) -> Result<Body, String> {
    let mut out = target.clone();
    for (i, x) in xs.iter().enumerate() {
        let inst = instance(seed, x, feature, i + 1);
        out = boolean(&out, &inst, Op::Union, tol)?.body;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body, rect_tags};
    use ck_topo::mass::mass_props;

    /// (id, normal for planar faces)
    fn faces(b: &Body) -> Vec<(String, Option<[f64; 3]>)> {
        b.face_ids()
            .map(|f| {
                let n = matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. })
                    .then(|| b.planar_normal(f).to_array().map(|x| if x.abs() < 1e-12 { 0.0 } else { x }));
                (b.face_id_str(f), n)
            })
            .collect()
    }

    fn assert_faces(b: &Body, want: &[(&str, Option<[f64; 3]>)]) {
        let got = faces(b);
        let g: Vec<&str> = got.iter().map(|x| x.0.as_str()).collect();
        let w: Vec<&str> = want.iter().map(|x| x.0).collect();
        assert_eq!(g, w);
        for (x, y) in got.iter().zip(want) {
            assert_eq!(x.1, y.1, "{}", x.0);
        }
    }

    fn assert_mass(b: &Body, vol: f64, area: f64, com: [f64; 3]) {
        let m = mass_props(b);
        assert!((m.volume - vol).abs() < 1e-9 * vol, "volume {} vs {}", m.volume, vol);
        assert!((m.area - area).abs() < 1e-9 * area, "area {} vs {}", m.area, area);
        for i in 0..3 {
            assert!((m.com.get(i) - com[i]).abs() < 1e-9 * 40.0, "com {:?} vs {:?}", m.com, com);
        }
    }

    fn valid(b: &Body) {
        let issues = check(b, 1e-6);
        assert!(issues.is_empty(), "{issues:?}");
    }

    const NX: Option<[f64; 3]> = Some([1., 0., 0.]);
    const MX: Option<[f64; 3]> = Some([-1., 0., 0.]);
    const NY: Option<[f64; 3]> = Some([0., 1., 0.]);
    const MY: Option<[f64; 3]> = Some([0., -1., 0.]);
    const NZ: Option<[f64; 3]> = Some([0., 0., 1.]);
    const MZ: Option<[f64; 3]> = Some([0., 0., -1.]);

    fn the_box() -> Body {
        box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0)
    }

    /// The content-keyed memos (edge boxes, pcurves, surface grids) return what the computation returns: a
    /// boolean run with warm tables gives the same bits as the first, cold run, and the repeat hits them.
    #[test]
    fn warm_memos_give_the_same_body() {
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let first = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap().body;
        let second_tool = cyl_body("F5", 8.0, 10.0, 3.0, -1.0, 11.0);
        let chained = boolean(&first, &second_tool, Op::Cut, 1e-6).unwrap().body;
        let [hits, _] = ck_core::memo::counts();
        let again = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap().body;
        let again_chained = boolean(&again, &second_tool, Op::Cut, 1e-6).unwrap().body;
        assert!(ck_core::memo::counts()[0] > hits);
        assert_eq!(format!("{first:?}"), format!("{again:?}"));
        assert_eq!(format!("{chained:?}"), format!("{again_chained:?}"));
    }

    #[test]
    fn oracle_box_minus_through_cylinder() {
        // probe box_cyl: extrude cut throughBoth of circle (20,10) r5
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let r = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap();
        valid(&r.body);
        assert_faces(
            &r.body,
            &[
                ("F2.S.e1", NZ),
                ("F2.S.e2", NX),
                ("F2.S.e3", MZ),
                ("F2.S.e4", MX),
                ("F2.C0", MY),
                ("F2.C1", NY),
                ("F4.S.e1", None),
            ],
        );
        assert_mass(&r.body, 7214.601836602553, 2957.0796326794866, [20., 5., -10.]);
        // the hole faces inward
        let f = r.body.find_face("F4.S.e1").unwrap();
        assert!(r.body.outward_normal(f, 0.0, 0.0).dot(ck_geom::Vec3::X) < 0.0);
        // edge keys: oracle has the 12 box edges + two circles C0|F4, C1|F4
        let wire: Vec<String> = r
            .body
            .wire_chains()
            .iter()
            .map(|ch| {
                let fs = r.body.edge_faces(ch[0]);
                let mut ids: Vec<String> = fs.iter().map(|&f| r.body.face_id_str(f)).collect();
                ids.sort();
                ids.join("|")
            })
            .collect();
        assert_eq!(wire.len(), 14);
        assert!(wire.contains(&"F2.C0|F4.S.e1".to_string()) && wire.contains(&"F2.C1|F4.S.e1".to_string()));
        // oracle edge order (#0..#13): face-walk order of the output faces
        assert_eq!(
            wire,
            [
                "F2.C0|F2.S.e1",
                "F2.S.e1|F2.S.e2",
                "F2.C1|F2.S.e1",
                "F2.S.e1|F2.S.e4",
                "F2.C0|F2.S.e2",
                "F2.S.e2|F2.S.e3",
                "F2.C1|F2.S.e2",
                "F2.C0|F2.S.e3",
                "F2.S.e3|F2.S.e4",
                "F2.C1|F2.S.e3",
                "F2.C0|F2.S.e4",
                "F2.C1|F2.S.e4",
                "F2.C0|F4.S.e1",
                "F2.C1|F4.S.e1"
            ]
        );
    }

    #[test]
    fn oracle_blind_pocket() {
        // probe blindcut: circle (20,10) r5, cut flipped depth 4 from C0
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, 0.0, 4.0);
        let r = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap();
        valid(&r.body);
        assert_faces(
            &r.body,
            &[
                ("F2.C1", NY),
                ("F2.S.e1", NZ),
                ("F2.S.e2", NX),
                ("F2.S.e3", MZ),
                ("F2.S.e4", MX),
                ("F2.C0", MY),
                ("F4.C1", MY),
                ("F4.S.e1", None),
            ],
        );
        assert_mass(&r.body, 7685.840734641021, 2925.6637061435913, [20., 5.122625205051293, -10.]);
        // circle edges start at c + r·x (oracle pts[0] = [25, 0, -10])
        let c0 = r.body.find_face("F2.C0").unwrap();
        let circ = r.body.face_edges(c0).into_iter().find(|&e| r.body.edge(e).curve.kind() == "circle").unwrap();
        let p = r.body.vertex(r.body.edge(circ).v0).p;
        assert!((p - Vec3::new(25., 0., -10.)).len() < 1e-9, "{p:?}");
    }

    #[test]
    fn oracle_cut_no_effect() {
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, -4.0, 0.0);
        let r = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap();
        assert!(r.no_effect);
        assert_eq!(r.body, the_box());
    }

    #[test]
    fn oracle_slot_split() {
        let tool = rect_tags("F4", ["s1", "s2", "s3", "s4"], 15.0, -5.0, 25.0, 25.0, -1.0, 11.0);
        let r = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap();
        valid(&r.body);
        let mut got: Vec<String> = faces(&r.body).into_iter().map(|x| x.0).collect();
        got.sort();
        let mut want = vec![
            "F2.S.e2", "F2.S.e4", "F2.C0~1", "F2.C0~2", "F2.C1~1", "F2.C1~2", "F2.S.e1~1", "F2.S.e1~2", "F2.S.e3~2",
            "F2.S.e3~1", "F4.S.s2", "F4.S.s4",
        ];
        want.sort();
        assert_eq!(got, want);
        assert_eq!(r.body.num_lumps(), 2);
        assert_mass(&r.body, 6000.0, 2600.0, [20., 5., -10.]);
        // ~1 = the x < 15 piece
        let f = r.body.find_face("F2.C0~1").unwrap();
        let x: f64 = r.body.face_edges(f).iter().map(|&e| r.body.edge_mid(e).x).fold(f64::MIN, f64::max);
        assert!(x <= 15.0 + 1e-9);
        // lump 0 holds the first face (oracle: F2.S.e2, the x > 25 piece)
        let first = r.body.face_ids().next().unwrap();
        assert_eq!(r.body.face_id_str(first), "F2.S.e2");
        assert_eq!(r.body.lump_index(r.body.face_lump(first)), 0);
    }

    #[test]
    fn oracle_union_overlapping_box() {
        let tool = rect_tags("F4", ["s1", "s2", "s3", "s4"], 30.0, 5.0, 50.0, 15.0, 0.0, 5.0);
        let r = boolean(&the_box(), &tool, Op::Union, 1e-6).unwrap();
        valid(&r.body);
        assert_faces(
            &r.body,
            &[
                ("F2.C1", NY),
                ("F2.S.e1", NZ),
                ("F2.S.e3", MZ),
                ("F2.S.e4", MX),
                ("F2.S.e2", NX),
                ("F4.S.s2", NX),
                ("F4.C1", NY),
                ("F4.S.s1", NZ),
                ("F4.S.s3", MZ),
                ("F2.C0", MY),
            ],
        );
        assert_mass(&r.body, 8500.0, 3100.0, [21.470588235294116, 4.852941176470589, -10.]);
    }

    #[test]
    fn oracle_union_flush_box() {
        let tool = rect_tags("F4", ["s1", "s2", "s3", "s4"], 30.0, 5.0, 50.0, 15.0, 0.0, 10.0);
        let r = boolean(&the_box(), &tool, Op::Union, 1e-6).unwrap();
        valid(&r.body);
        assert_faces(
            &r.body,
            &[
                ("F2.S.e1", NZ),
                ("F2.S.e3", MZ),
                ("F2.S.e4", MX),
                ("F2.S.e2~2", NX),
                ("F2.S.e2~1", NX),
                ("F4.S.s2", NX),
                ("F4.S.s1", NZ),
                ("F4.S.s3", MZ),
                ("F2.C0", MY),
                ("F2.C1", NY),
            ],
        );
        assert_mass(&r.body, 9000.0, 3200.0, [22.777777777777775, 5., -10.]);
    }

    #[test]
    fn intersect_box_cylinder() {
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let r = boolean(&the_box(), &tool, Op::Intersect, 1e-6).unwrap();
        valid(&r.body);
        let m = mass_props(&r.body);
        let v = std::f64::consts::PI * 25.0 * 10.0;
        assert!((m.volume - v).abs() < 1e-9 * v, "{}", m.volume);
    }

    #[test]
    fn disjoint_union_appends() {
        let tool = box_body("F4", 50.0, 0.0, 60.0, 10.0, 0.0, 5.0);
        let r = boolean(&the_box(), &tool, Op::Union, 1e-6).unwrap();
        assert_eq!(r.body.num_lumps(), 2);
        assert_eq!(r.body.num_faces(), 12);
    }

    #[test]
    fn pattern_transforms_match_oracle() {
        // probe: box (10,0)-(14,2) on top, cpattern about y, count 3
        let rot = circular_instances(Vec3::ZERO, Vec3::Y, 90.0, 3, true, false);
        let p = rot[0].apply_point(Vec3::new(10., 1.5, 0.));
        assert!((p - Vec3::new(7.0710678118654755, 1.5, -7.071067811865475)).len() < 1e-14);
        let p = rot[1].apply_point(Vec3::new(10., 1.5, 0.));
        assert!((p - Vec3::new(6.123233995736766e-16, 1.5, -10.)).len() < 1e-14);
        let rot = circular_instances(Vec3::ZERO, Vec3::Y, 360.0, 3, true, false);
        let p = rot[1].apply_point(Vec3::new(10., 1.5, 0.));
        assert!((p - Vec3::new(-5.000000000000004, 1.5, 8.660254037844386)).len() < 1e-14);
        assert_eq!(circular_step(90.0, 3, false), 90.0);
        let rev = circular_instances(Vec3::ZERO, Vec3::Y, 90.0, 3, true, true);
        assert!((rev[0].apply_point(Vec3::new(10., 1.5, 0.)) - Vec3::new(7.0710678118654755, 1.5, 7.071067811865475)).len() < 1e-14);
        let lin = linear_instances(Vec3::Z, 5.0, 3, true);
        assert_eq!(lin.len(), 2);
        assert_eq!(lin[1].apply_point(Vec3::new(10., 1.5, 0.)), Vec3::new(10., 1.5, -10.));
        // instances unioned: disjoint copies are appended, renamed
        let seed = box_body("F2", 10.0, 0.0, 14.0, 2.0, 0.0, 3.0);
        let lin = linear_instances(Vec3::Z, 5.0, 3, false);
        let out = apply_instances(&seed, &seed, &lin, "F3", 1e-6).unwrap();
        let ids: Vec<String> = out.face_ids().map(|f| out.face_id_str(f)).filter(|s| s.starts_with("F2.S.e1")).collect();
        assert_eq!(ids, ["F2.S.e1", "F2.S.e1@F3.1", "F2.S.e1@F3.2"]);
        // oracle edges #9 / #21 = F2.S.e1|F2.S.e4 of the seed / copy 1
        let wire = out.wire_edges();
        let mut f9: Vec<String> = out.edge_faces(wire[9]).iter().map(|&f| out.face_id_str(f)).collect();
        f9.sort();
        assert_eq!(f9, ["F2.S.e1", "F2.S.e4"]);
        let e = wire[21];
        let mut fs: Vec<String> = out.edge_faces(e).iter().map(|&f| out.face_id_str(f)).collect();
        fs.sort();
        assert_eq!(fs, ["F2.S.e1@F3.1", "F2.S.e4@F3.1"]);
        assert_eq!(out.edge_mid(e), Vec3::new(10., 1.5, 5.));
    }
}

#[cfg(test)]
mod oracle_edges {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body, rect_tags};

    /// Wire edge `key` (sorted face pair + `#n`) and `mid`, oracle replies
    /// of the probes (tools/q.mjs regen of box F2 + feature F4).
    #[rustfmt::skip]
    const ORACLE: &[(&str, &[(&str, [f64; 3])])] = &[
        ("blindcut", &[
            ("F2.C1|F2.S.e1#0", [20.0, 10.0, 0.0]),
            ("F2.C1|F2.S.e2#1", [40.0, 10.0, -10.0]),
            ("F2.C1|F2.S.e3#2", [20.0, 10.0, -20.0]),
            ("F2.C1|F2.S.e4#3", [0.0, 10.0, -10.0]),
            ("F2.C0|F2.S.e1#4", [20.0, 0.0, 0.0]),
            ("F2.S.e1|F2.S.e2#5", [40.0, 5.0, 0.0]),
            ("F2.S.e1|F2.S.e4#6", [0.0, 5.0, 0.0]),
            ("F2.C0|F2.S.e2#7", [40.0, 0.0, -10.0]),
            ("F2.S.e2|F2.S.e3#8", [40.0, 5.0, -20.0]),
            ("F2.C0|F2.S.e3#9", [20.0, 0.0, -20.0]),
            ("F2.S.e3|F2.S.e4#10", [0.0, 5.0, -20.0]),
            ("F2.C0|F2.S.e4#11", [0.0, 0.0, -10.0]),
            ("F2.C0|F4.S.e1#12", [15.0, 0.0, -10.0]),
            ("F4.C1|F4.S.e1#13", [15.0, 4.0, -10.0]),
        ]),
        ("slot", &[
            ("F2.C0~2|F2.S.e2#0", [40.0, 0.0, -10.0]),
            ("F2.S.e2|F2.S.e3~2#1", [40.0, 5.0, -20.0]),
            ("F2.C1~2|F2.S.e2#2", [40.0, 10.0, -10.0]),
            ("F2.S.e1~2|F2.S.e2#3", [40.0, 5.0, 0.0]),
            ("F2.C0~1|F2.S.e4#4", [0.0, 0.0, -10.0]),
            ("F2.S.e1~1|F2.S.e4#5", [0.0, 5.0, 0.0]),
            ("F2.C1~1|F2.S.e4#6", [0.0, 10.0, -10.0]),
            ("F2.S.e3~1|F2.S.e4#7", [0.0, 5.0, -20.0]),
            ("F2.C0~1|F2.S.e3~1#8", [7.5, 0.0, -20.0]),
            ("F2.C0~1|F4.S.s4#9", [15.0, 0.0, -10.0]),
            ("F2.C0~1|F2.S.e1~1#10", [7.5, 0.0, 0.0]),
            ("F2.C0~2|F2.S.e3~2#11", [32.5, 0.0, -20.0]),
            ("F2.C0~2|F2.S.e1~2#12", [32.5, 0.0, 0.0]),
            ("F2.C0~2|F4.S.s2#13", [25.0, 0.0, -10.0]),
            ("F2.C1~1|F2.S.e1~1#14", [7.5, 10.0, 0.0]),
            ("F2.C1~1|F4.S.s4#15", [15.0, 10.0, -10.0]),
            ("F2.C1~1|F2.S.e3~1#16", [7.5, 10.0, -20.0]),
            ("F2.C1~2|F2.S.e1~2#17", [32.5, 10.0, 0.0]),
            ("F2.C1~2|F2.S.e3~2#18", [32.5, 10.0, -20.0]),
            ("F2.C1~2|F4.S.s2#19", [25.0, 10.0, -10.0]),
            ("F2.S.e1~1|F4.S.s4#20", [15.0, 5.0, 0.0]),
            ("F2.S.e1~2|F4.S.s2#21", [25.0, 5.0, 0.0]),
            ("F2.S.e3~2|F4.S.s2#22", [25.0, 5.0, -20.0]),
            ("F2.S.e3~1|F4.S.s4#23", [15.0, 5.0, -20.0]),
        ]),
        ("union", &[
            ("F2.C1|F2.S.e1#0", [20.0, 10.0, 0.0]),
            ("F2.C1|F2.S.e2#1", [40.0, 10.0, -10.0]),
            ("F2.C1|F2.S.e3#2", [20.0, 10.0, -20.0]),
            ("F2.C1|F2.S.e4#3", [0.0, 10.0, -10.0]),
            ("F2.C0|F2.S.e1#4", [20.0, 0.0, 0.0]),
            ("F2.S.e1|F2.S.e2#5", [40.0, 5.0, 0.0]),
            ("F2.S.e1|F2.S.e4#6", [0.0, 5.0, 0.0]),
            ("F2.C0|F2.S.e3#7", [20.0, 0.0, -20.0]),
            ("F2.S.e3|F2.S.e4#8", [0.0, 5.0, -20.0]),
            ("F2.S.e2|F2.S.e3#9", [40.0, 5.0, -20.0]),
            ("F2.C0|F2.S.e4#10", [0.0, 0.0, -10.0]),
            ("F2.C0|F2.S.e2#11", [40.0, 0.0, -17.5]),
            ("F2.C0|F2.S.e2#12", [40.0, 0.0, -2.5]),
            ("F2.S.e2|F4.S.s1#13", [40.0, 2.5, -5.0]),
            ("F2.S.e2|F4.C1#14", [40.0, 5.0, -10.0]),
            ("F2.S.e2|F4.S.s3#15", [40.0, 2.5, -15.0]),
            ("F2.C0|F4.S.s2#16", [50.0, 0.0, -10.0]),
            ("F4.S.s2|F4.S.s3#17", [50.0, 2.5, -15.0]),
            ("F4.C1|F4.S.s2#18", [50.0, 5.0, -10.0]),
            ("F4.S.s1|F4.S.s2#19", [50.0, 2.5, -5.0]),
            ("F2.C0|F4.S.s3#20", [45.0, 0.0, -15.0]),
            ("F2.C0|F4.S.s1#21", [45.0, 0.0, -5.0]),
            ("F4.C1|F4.S.s1#22", [45.0, 5.0, -5.0]),
            ("F4.C1|F4.S.s3#23", [45.0, 5.0, -15.0]),
        ]),
        ("unionflush", &[
            ("F2.C0|F2.S.e1#0", [20.0, 0.0, 0.0]),
            ("F2.S.e1|F2.S.e2~2#1", [40.0, 5.0, 0.0]),
            ("F2.C1|F2.S.e1#2", [20.0, 10.0, 0.0]),
            ("F2.S.e1|F2.S.e4#3", [0.0, 5.0, 0.0]),
            ("F2.C0|F2.S.e3#4", [20.0, 0.0, -20.0]),
            ("F2.S.e3|F2.S.e4#5", [0.0, 5.0, -20.0]),
            ("F2.C1|F2.S.e3#6", [20.0, 10.0, -20.0]),
            ("F2.S.e2~1|F2.S.e3#7", [40.0, 5.0, -20.0]),
            ("F2.C0|F2.S.e4#8", [0.0, 0.0, -10.0]),
            ("F2.C1|F2.S.e4#9", [0.0, 10.0, -10.0]),
            ("F2.C0|F2.S.e2~1#10", [40.0, 0.0, -17.5]),
            ("F2.C0|F2.S.e2~2#11", [40.0, 0.0, -2.5]),
            ("F2.C1|F2.S.e2~2#12", [40.0, 10.0, -2.5]),
            ("F2.C1|F2.S.e2~1#13", [40.0, 10.0, -17.5]),
            ("F2.S.e2~2|F4.S.s1#14", [40.0, 5.0, -5.0]),
            ("F2.S.e2~1|F4.S.s3#15", [40.0, 5.0, -15.0]),
            ("F2.C0|F4.S.s2#16", [50.0, 0.0, -10.0]),
            ("F4.S.s2|F4.S.s3#17", [50.0, 5.0, -15.0]),
            ("F2.C1|F4.S.s2#18", [50.0, 10.0, -10.0]),
            ("F4.S.s1|F4.S.s2#19", [50.0, 5.0, -5.0]),
            ("F2.C0|F4.S.s3#20", [45.0, 0.0, -15.0]),
            ("F2.C0|F4.S.s1#21", [45.0, 0.0, -5.0]),
            ("F2.C1|F4.S.s1#22", [45.0, 10.0, -5.0]),
            ("F2.C1|F4.S.s3#23", [45.0, 10.0, -15.0]),
        ]),
    ];

    #[test]
    fn edge_keys_and_mids_match_oracle() {
        let a = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let tool = |n: &str| match n {
            "blindcut" => (cyl_body("F4", 20.0, 10.0, 5.0, 0.0, 4.0), Op::Cut),
            "slot" => (rect_tags("F4", ["s1", "s2", "s3", "s4"], 15.0, -5.0, 25.0, 25.0, -1.0, 11.0), Op::Cut),
            "union" => (rect_tags("F4", ["s1", "s2", "s3", "s4"], 30.0, 5.0, 50.0, 15.0, 0.0, 5.0), Op::Union),
            _ => (rect_tags("F4", ["s1", "s2", "s3", "s4"], 30.0, 5.0, 50.0, 15.0, 0.0, 10.0), Op::Union),
        };
        for (name, want) in ORACLE {
            let (t, op) = tool(name);
            let b = boolean(&a, &t, op, 1e-6).unwrap().body;
            let wire = b.wire_edges();
            assert_eq!(wire.len(), want.len(), "{name}");
            for (i, (&e, (key, mid))) in wire.iter().zip(want.iter()).enumerate() {
                let fs = b.edge_faces(e);
                let k = ck_topo::body::edge_key(&b.face_id_str(fs[0]), &b.face_id_str(fs[1]), i);
                assert_eq!(&k, key, "{name}");
                assert!((b.edge_mid(e) - Vec3::new(mid[0], mid[1], mid[2])).len() < 1e-9, "{name} {key}");
            }
        }
    }
}

#[cfg(test)]
mod oracle_cylinders {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body, cyl_on, top};
    use ck_geom::Frame;
    use ck_topo::mass::mass_props;

    /// (probe, oracle face ids, oracle edge keys)
    #[rustfmt::skip]
    const ORACLE: &[(&str, &[&str], &[&str])] = &[
        ("edgehole", &["F2.S.e1", "F2.S.e3", "F2.S.e4", "F2.C0", "F2.C1", "F2.S.e2~2", "F2.S.e2~1", "F4.S.e1"],
            &["F2.C0|F2.S.e1#0", "F2.S.e1|F2.S.e2~2#1", "F2.C1|F2.S.e1#2", "F2.S.e1|F2.S.e4#3", "F2.C0|F2.S.e3#4", "F2.S.e3|F2.S.e4#5", "F2.C1|F2.S.e3#6", "F2.S.e2~1|F2.S.e3#7", "F2.C0|F2.S.e4#8", "F2.C1|F2.S.e4#9", "F2.C0|F2.S.e2~1#10", "F2.C0|F4.S.e1#11", "F2.C0|F2.S.e2~2#12", "F2.C1|F2.S.e2~2#13", "F2.C1|F4.S.e1#14", "F2.C1|F2.S.e2~1#15", "F2.S.e2~2|F4.S.e1#16", "F2.S.e2~1|F4.S.e1#17"]),
        ("boss", &["F2.S.e1", "F2.S.e3", "F2.S.e4", "F2.C1", "F2.S.e2~2", "F2.S.e2~1", "F4.C1", "F4.S.e1", "F2.C0"],
            &["F2.C0|F2.S.e1#0", "F2.S.e1|F2.S.e2~2#1", "F2.C1|F2.S.e1#2", "F2.S.e1|F2.S.e4#3", "F2.C0|F2.S.e3#4", "F2.S.e3|F2.S.e4#5", "F2.C1|F2.S.e3#6", "F2.S.e2~1|F2.S.e3#7", "F2.C0|F2.S.e4#8", "F2.C1|F2.S.e4#9", "F2.C0|F2.S.e2~1#10", "F2.C0|F2.S.e2~2#11", "F2.C1|F2.S.e2~2#12", "F2.C1|F4.S.e1#13", "F2.C1|F2.S.e2~1#14", "F2.S.e2~2|F4.S.e1#15", "F2.S.e2~1|F4.S.e1#16", "F4.C1|F4.S.e1#17", "F2.C0|F4.S.e1#18"]),
        ("fronthole", &["F2.C0", "F2.C1", "F2.S.e2", "F2.S.e4", "F2.S.e1", "F2.S.e3", "F4.S.e1"],
            &["F2.C0|F2.S.e4#0", "F2.C0|F2.S.e3#1", "F2.C0|F2.S.e2#2", "F2.C0|F2.S.e1#3", "F2.C1|F2.S.e1#4", "F2.C1|F2.S.e2#5", "F2.C1|F2.S.e3#6", "F2.C1|F2.S.e4#7", "F2.S.e2|F2.S.e3#8", "F2.S.e1|F2.S.e2#9", "F2.S.e1|F2.S.e4#10", "F2.S.e3|F2.S.e4#11", "F2.S.e1|F4.S.e1#12", "F2.S.e3|F4.S.e1#13"]),
        ("cylcross", &["F2.C0", "F2.C1", "F2.S.e1", "F4.S.e1"],
            &["F2.C0|F2.S.e1#0", "F2.C1|F2.S.e1#1", "F2.S.e1|F4.S.e1#2", "F2.S.e1|F4.S.e1#3"]),
    ];

    fn case(name: &str) -> (Body, Body, Op, f64, f64, [f64; 3]) {
        let bx = || box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        match name {
            // cut throughBoth of circle (40,10) r5 on the box's x=40 edge
            "edgehole" => (bx(), cyl_body("F4", 40.0, 10.0, 5.0, -1.0, 11.0), Op::Cut, 7607.300918301279, 2778.539816339745, [19.07711705162717, 5., -10.]),
            // boss circle (38,10) r5, blind 15, overhanging the x=40 face
            "boss" => (bx(), cyl_body("F4", 38.0, 10.0, 5.0, 0.0, 15.0), Op::Union, 8590.86743798146, 3020.989738109662, [21.312693341198823, 5.342834193869538, -10.]),
            // front-plane circle (20,5) r3 cut throughBoth (along z)
            "fronthole" => (bx(), cyl_on("F4", Frame::identity(), 20.0, 5.0, 3.0, -25.0, 5.0), Op::Cut, 7434.513322353839, 3120.4424506661585, [20., 5., -10.]),
            // cylinder r10 h30 crossed by a front-plane hole r4 at y=15
            _ => (cyl_on("F2", top(), 0.0, 0.0, 10.0, 0.0, 30.0), cyl_on("F4", Frame::identity(), 0.0, 15.0, 4.0, -15.0, 15.0), Op::Cut, 8439.998272825493, 2892.5020588265716, [0., 15., 0.]),
        }
    }

    #[test]
    fn faces_edges_mass_match_oracle() {
        for (name, faces, keys) in ORACLE {
            let (a, b, op, vol, area, com) = case(name);
            let r = boolean(&a, &b, op, 1e-6).unwrap_or_else(|e| panic!("{name}: {e}"));
            let issues = check(&r.body, 1e-6);
            assert!(issues.is_empty(), "{name}: {issues:?}");
            let got: Vec<String> = r.body.face_ids().map(|f| r.body.face_id_str(f)).collect();
            assert_eq!(got, *faces, "{name}");
            // oracle edges = chains of our wire edges through seam points
            let wire: Vec<_> = crate::boolean::assemble::wire_chains(&r.body).into_iter().map(|c| c[0]).collect();
            let ks: Vec<String> = wire
                .iter()
                .enumerate()
                .map(|(i, &e)| {
                    let fs = r.body.edge_faces(e);
                    ck_topo::body::edge_key(&r.body.face_id_str(fs[0]), &r.body.face_id_str(fs[1]), i)
                })
                .collect();
            let mut ks_sorted: Vec<String> = ks.iter().map(|k| k.split('#').next().unwrap().to_string()).collect();
            let mut want_sorted: Vec<String> = keys.iter().map(|k| k.split('#').next().unwrap().to_string()).collect();
            ks_sorted.sort();
            want_sorted.sort();
            assert_eq!(ks_sorted, want_sorted, "{name} edge sets");
            let m = mass_props(&r.body);
            // marched SSI (cylcross) is a fitted curve: looser tolerance
            let rel = if *name == "cylcross" { 1e-6 } else { 1e-9 };
            assert!((m.volume - vol).abs() <= rel * vol, "{name} volume {} vs {vol}", m.volume);
            assert!((m.area - area).abs() <= rel * area, "{name} area {} vs {area}", m.area);
            assert!((m.com - Vec3::new(com[0], com[1], com[2])).len() <= rel * 40.0, "{name} com {:?}", m.com);
            if *name != "cylcross" {
                assert_eq!(ks, *keys, "{name} edge order");
            }
        }
    }
}

#[cfg(test)]
mod flange_tests {
    use super::*;
    use crate::boolean::tests::top;
    use ck_geom::{Frame, Vec2, Vec3};
    use ck_topo::mass::mass_props;
    use ck_topo::make::{extrude, revolve::revolve, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

    /// Hex prism (corner radius 7.5055, across flats 13) + revolved chamfer
    /// triangle cut (flangeBolt F4+F6 / flangeNut F4 shape).
    #[test]
    fn hex_prism_minus_revolved_chamfer_cone() {
        let front = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
        let r = 7.505553499465135;
        let p: Vec<Vec2> = (0..6).map(|i| { let a = std::f64::consts::FRAC_PI_3 * i as f64 + std::f64::consts::FRAC_PI_2; Vec2::new(r * a.cos(), r * a.sin()) }).collect();
        let lp = ProfileLoop::new((0..6).map(|i| ProfileSeg::line(&format!("e{}", i + 7), p[i], p[(i + 1) % 6])).collect());
        let prof = Profile::new(front, vec![ProfileRegion { outer: lp, holes: vec![] }]);
        let hex = extrude("F4", &prof, ExtrudeOpts { s0: 0.0, s1: -5.3, dir: None, draft: 0.0 }).unwrap();
        let q = [Vec2::new(6.11, 5.3), Vec2::new(8.5, 3.920132856636794), Vec2::new(8.5, 5.3)];
        let tl = ProfileLoop::new((0..3).map(|i| ProfileSeg::line(&format!("e{}", i + 4), q[i], q[(i + 1) % 3])).collect());
        let tp = Profile::new(top(), vec![ProfileRegion { outer: tl, holes: vec![] }]);
        let tool = revolve("F6", &tp, Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), 0.0, std::f64::consts::TAU).unwrap();
        let r = boolean(&hex, &tool, Op::Cut, 1e-6);
        let r = r.unwrap();
        assert_eq!(r.body.num_faces(), 9);
        let (vh, vt, vr) = (mass_props(&hex).volume, mass_props(&tool).volume, mass_props(&r.body).volume);
        assert!(vr < vh && vr > vh - vt, "{vr} {vh} {vt}");
    }
}
