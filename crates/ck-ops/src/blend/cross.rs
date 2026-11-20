//! Two rounds that cross: the two edges of one planar strip face are rounded with radii whose sum is more than the strip's
//! width (a 4 wide box, r = 3 on both top edges). The rolling ball of each edge lands past the middle of the strip, so
//! the rounds cannot meet tangentially; the solid is the INTERSECTION of the two rounded profiles: each round is the
//! ordinary single-edge round, the strip face is gone and the two round faces meet in a sharp crease along the strip.
//! Built as the common of the two single-edge results (each one built and checked by the ordinary path), so every face
//! keeps its name: the rounds are `B<k>` of their picks, the crease is the edge between them.
//!
//! Only this configuration: exactly two picks, straight convex edges between planar faces, parallel, the two edges of one
//! face that is a four-edged strip between them, each radius at most the strip's width (a single round wider than its face
//! stays refused) and their sum more than it. It is tried only after the ordinary build refused the selection as an
//! overflow, so a selection that builds today is never changed. The feature reports a `blendRoundsCross` warning with the
//! width and the radius at which the rounds would just meet (see [`take_rounds_cross`]).

use super::build::{find_edge, in_place_layout, BlendRef};
use super::model::{FilletProfile, Spec};
use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, FaceName, Role};

/// What a crossing build made: the strip's width and the two radii (in pick order).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoundsCross {
    pub width: f64,
    pub radii: [f64; 2],
}

thread_local! {
    static LAST: std::cell::Cell<Option<RoundsCross>> = const { std::cell::Cell::new(None) };
}

/// The crossing the last blend build made, if it made one (and forgets it). The feature layer clears it before a build
/// and reads it after the build succeeded.
pub fn take_rounds_cross() -> Option<RoundsCross> {
    LAST.with(|c| c.take())
}

/// Records a crossing build for the feature's warning (used by [`super::cross_multi`]).
pub(super) fn record(c: RoundsCross) {
    LAST.with(|l| l.set(Some(c)));
}

/// The strip width between the two picks when they are the configuration this module builds (see the module docs).
fn strip_width(b: &Body, edges: &[(String, String, Vec3)], radii: [f64; 2]) -> Option<f64> {
    if edges.len() != 2 {
        return None;
    }
    let e: Vec<_> = edges.iter().map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect::<Option<_>>()?;
    if e[0] == e[1] {
        return None;
    }
    let mut dirs = vec![];
    for &x in &e {
        let ed = b.edge(x);
        if !matches!(ed.curve, Curve::Line { .. }) || ed.tol > 0.0 {
            return None;
        }
        let fs = b.edge_faces(x);
        if fs.len() != 2 || fs[0] == fs[1] || !fs.iter().all(|&f| matches!(b.face(f).surface, Surface::Plane { .. })) {
            return None;
        }
        if !super::validity::edge_info(b, x)?.convex {
            return None;
        }
        let (p0, p1) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
        dirs.push((p0, (p1 - p0).try_normalize()?));
    }
    // the one face both edges border: a strip of four edges in one loop
    let (fa, fb) = (b.edge_faces(e[0]), b.edge_faces(e[1]));
    let common: Vec<_> = fa.iter().filter(|f| fb.contains(f)).copied().collect();
    if common.len() != 1 {
        return None;
    }
    let strip = common[0];
    if b.face(strip).loops.len() != 1 || b.face_edges(strip).len() != 4 {
        return None;
    }
    let ((p, d), (q, d2)) = (dirs[0], dirs[1]);
    if d.cross(d2).len() > 1e-9 {
        return None;
    }
    let off = q - p;
    let w = (off - d * off.dot(d)).len();
    let scale = 1.0 + w;
    if !(w > 1e-9 * scale) {
        return None;
    }
    // the edges' ends face each other across the strip (a rectangle, not a parallelogram running past one end)
    let (s0, s1) = (b.vertex(b.edge(e[1]).v0).p - p, b.vertex(b.edge(e[1]).v1).p - p);
    let (t0, t1) = (0.0, b.vertex(b.edge(e[0]).v1).p.dist(p));
    let (u0, u1) = (s0.dot(d).min(s1.dot(d)), s0.dot(d).max(s1.dot(d)));
    if (u0 - t0).abs() > 1e-7 * scale || (u1 - t1).abs() > 1e-7 * scale {
        return None;
    }
    let [r1, r2] = radii;
    if !(r1 > 0.0 && r2 > 0.0) || r1 > w * (1.0 + 1e-9) || r2 > w * (1.0 + 1e-9) || r1 + r2 <= w * (1.0 + 1e-9) {
        return None;
    }
    Some(w)
}

/// The crossing rounds of `edges` with `radii` (see the module docs); `None` when the selection is not that configuration.
pub(super) fn try_crossing(b: &Body, edges: &[(String, String, Vec3)], radii: [f64; 2], feature: &str, tol: f64) -> Option<Result<Body, BlendError>> {
    let w = strip_width(b, edges, radii)?;
    let saved = super::rounds::EDGE_RADII.with(|c| c.borrow_mut().take());
    let res = build(b, edges, radii, feature, tol);
    super::rounds::EDGE_RADII.with(|c| *c.borrow_mut() = saved);
    let res = res.and_then(|out| {
        // judged as the selection it is: both picks, the larger radius (the volume bound of two rounds holds the crossing's
        // smaller removal); the joint guard runs inside
        super::validity::check(b, out, edges, Spec::Fillet(radii[0].max(radii[1]), FilletProfile::Circular), tol)
    });
    if res.is_ok() {
        LAST.with(|c| c.set(Some(RoundsCross { width: w, radii })));
    }
    Some(res)
}

fn build(b: &Body, edges: &[(String, String, Vec3)], radii: [f64; 2], feature: &str, tol: f64) -> Result<Body, BlendError> {
    let mut parts: Vec<Body> = vec![];
    for k in 0..2 {
        let spec = Spec::Fillet(radii[k], FilletProfile::Circular);
        let one = &edges[k..k + 1];
        let mut out = super::rounds::blend_edges_inner(b, one, spec, feature, tol).and_then(|o| super::validity::check(b, o, one, spec, tol))?;
        // the single round is `B0`: give it the number of its pick
        for f in out.face_ids().collect::<Vec<_>>() {
            let n = &out.face(f).name;
            if n.feature == feature && matches!(n.role, Role::Blend(0)) {
                out.faces[f.idx()].name.role = Role::Blend(k as u32);
            }
        }
        parts.push(out);
    }
    let raw = crate::boolean::boolean(&parts[0], &parts[1], Op::Intersect, tol).map_err(|m| {
        super::diag::dbg_fail(&format!("crossing rounds: common of the two rounds failed: {m}"));
        BlendError::detail("blendFailed", "the two rounds cross, and their common could not be built")
    })?;
    let mut refs = vec![];
    for (k, (fa, fb, at)) in edges.iter().enumerate() {
        let e = find_edge(b, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge"))?;
        let ed = b.edge(e);
        refs.push(BlendRef { name: FaceName::blend(feature, k as u32).id(), faces: [fa.clone(), fb.clone()], ends: [b.vertex(ed.v0).p, b.vertex(ed.v1).p] });
    }
    let out = in_place_layout(b, &raw.body, &refs, tol);
    // both rounds must be there, meeting in the crease
    for k in 0..2u32 {
        let name = FaceName::blend(feature, k);
        if !out.face_ids().any(|f| {
            let mut n = out.face(f).name.clone();
            n.region = None;
            n == name
        }) {
            return Err(BlendError::detail("blendFailed", "the two rounds cross, and their common lost one of them"));
        }
    }
    Ok(out)
}
