//! Crossing rounds over SEVERAL walls at once (extends [`super::cross`], which builds the one case of two parallel edges of
//! one strip face): a plate whose outline is a polygon with the top and bottom edges of some or all walls rounded, the two
//! rounds of a wall wider together than the wall is high. The solid is the INTERSECTION of the ordinary single-edge results,
//! one per pick (each built and checked by the ordinary path), taken in a canonical order (by the picks' face names), so the
//! result does not depend on the pick order. Where two picks meet at a vertex whose third edge is not picked, the
//! intersection is the mitre of the two rounds, exact also at corners that are not right angles (a build of the two groups
//! of rounds by the ordinary corner builder left a sliver wall and up to 2e-3 of the volume at the skewed corners of a
//! quadrilateral plate; docs/dead-ends.md). For a prism whose walls are perpendicular to the caps the section at height z is
//! the outline with each rounded wall moved inward by the larger of the two rounds' depths there; the closed form is in
//! `crates/ck-model/tests/blend_cross_multi.rs`.
//!
//! Only this configuration: straight convex edges between planar faces; a vertex carries at most two picks (a corner patch of
//! three rounds is not an intersection) and two picks at a vertex share a face; every strip is a four-edged planar face with
//! its two selected edges parallel and facing each other (a rectangle), each radius at most the strip's width (a single round
//! wider than its face stays refused) and at least one strip with the two radii summing to more than its width. Tried only
//! after the ordinary build refused the selection as an overflow. The feature's `blendRoundsCross` warning names the narrowest
//! crossing strip.

use super::build::{find_edge, in_place_layout, BlendRef};
use super::cross::{record, RoundsCross};
use super::model::{FilletProfile, Spec};
use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, FaceName, Role};

/// The narrowest crossing strip: its width and the pick indices of its two edges.
struct Plan {
    narrow: (f64, [usize; 2]),
}

/// The plan when the picks are the configuration this module builds (see the module docs).
fn plan(b: &Body, edges: &[(String, String, Vec3)], radii: &[f64]) -> Option<Plan> {
    let n = edges.len();
    if n < 2 || radii.len() != n || !radii.iter().all(|r| *r > 0.0 && r.is_finite()) {
        return None;
    }
    let e: Vec<_> = edges.iter().map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect::<Option<_>>()?;
    for i in 0..n {
        if e[..i].contains(&e[i]) {
            return None;
        }
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
    // the strips: faces with exactly two picks that are a four-edged loop, the picks parallel and facing each other.
    // (parity constraints: `true` = the two picks must be in different groups)
    let mut cons: Vec<(usize, usize, bool)> = vec![];
    let mut narrow: Option<(f64, [usize; 2])> = None;
    let mut seen: Vec<ck_topo::body::FaceId> = vec![];
    for &x in &e {
        for f in b.edge_faces(x) {
            if seen.contains(&f) {
                continue;
            }
            seen.push(f);
            let picks: Vec<usize> = (0..n).filter(|&i| b.edge_faces(e[i]).contains(&f)).collect();
            if picks.len() != 2 || b.face(f).loops.len() != 1 || b.face_edges(f).len() != 4 {
                continue;
            }
            let (i, j) = (picks[0], picks[1]);
            let ((p, d), (q, d2)) = (dirs[i], dirs[j]);
            if d.cross(d2).len() > 1e-9 {
                continue;
            }
            let off = q - p;
            let w = (off - d * off.dot(d)).len();
            let scale = 1.0 + w;
            if !(w > 1e-9 * scale) {
                continue;
            }
            // the edges' ends face each other across the strip (a rectangle, not a parallelogram running past one end)
            let (s0, s1) = (b.vertex(b.edge(e[j]).v0).p - p, b.vertex(b.edge(e[j]).v1).p - p);
            let (t0, t1) = (0.0, b.vertex(b.edge(e[i]).v1).p.dist(p));
            let (u0, u1) = (s0.dot(d).min(s1.dot(d)), s0.dot(d).max(s1.dot(d)));
            if (u0 - t0).abs() > 1e-7 * scale || (u1 - t1).abs() > 1e-7 * scale {
                continue;
            }
            let (ri, rj) = (radii[i], radii[j]);
            let tol = w * (1.0 + 1e-9);
            if ri + rj > tol {
                // a round wider than its strip stays refused
                if ri > tol || rj > tol {
                    return None;
                }
                cons.push((i, j, true));
                if narrow.is_none_or(|(w0, _)| w < w0) {
                    narrow = Some((w, [i, j]));
                }
            } else {
                cons.push((i, j, false));
            }
        }
    }
    let narrow = narrow?;
    // a vertex carries at most two picks, and two of them share a face
    for v in b.vertex_ids() {
        let at: Vec<usize> = (0..n).filter(|&i| b.edge(e[i]).v0 == v || b.edge(e[i]).v1 == v).collect();
        if at.len() > 2 {
            return None;
        }
        if at.len() == 2 {
            let (fa, fb) = (b.edge_faces(e[at[0]]), b.edge_faces(e[at[1]]));
            if !fa.iter().any(|f| fb.contains(f)) {
                return None;
            }
        }
    }
    let _ = cons;
    Some(Plan { narrow })
}

/// The crossing rounds of `edges` with `radii` (see the module docs); `None` when the selection is not that configuration.
pub(super) fn try_crossing_multi(b: &Body, edges: &[(String, String, Vec3)], radii: &[f64], feature: &str, tol: f64) -> Option<Result<Body, BlendError>> {
    let plan = plan(b, edges, radii)?;
    let saved = super::rounds::EDGE_RADII.with(|c| c.borrow_mut().take());
    let res = build(b, edges, radii, &plan, feature, tol);
    super::rounds::EDGE_RADII.with(|c| *c.borrow_mut() = saved);
    let rmax = radii.iter().cloned().fold(0.0, f64::max);
    let res = res.and_then(|out| {
        // judged as the selection it is: all picks, the larger radius (the volume bound of the rounds holds the crossing's
        // smaller removal); the joint guard runs inside
        super::validity::check(b, out, edges, Spec::Fillet(rmax, FilletProfile::Circular), tol)
    });
    if res.is_ok() {
        let (w, [i, j]) = plan.narrow;
        record(RoundsCross { width: w, radii: [radii[i], radii[j]] });
    }
    Some(res)
}

fn build(b: &Body, edges: &[(String, String, Vec3)], radii: &[f64], plan: &Plan, feature: &str, tol: f64) -> Result<Body, BlendError> {
    // canonical order: by the sorted face names of the pick (then its point), so the pick order does not matter
    let key = |k: usize| {
        let (fa, fb, at) = &edges[k];
        let (x, y) = if fa <= fb { (fa.clone(), fb.clone()) } else { (fb.clone(), fa.clone()) };
        (x, y, [at.x.to_bits() as i128, at.y.to_bits() as i128, at.z.to_bits() as i128])
    };
    let mut order: Vec<usize> = (0..edges.len()).collect();
    order.sort_by_key(|&k| key(k));
    let mut acc: Option<Body> = None;
    for &k in &order {
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
        acc = Some(match acc {
            None => out,
            Some(prev) => crate::boolean::boolean(&prev, &out, Op::Intersect, tol)
                .map_err(|m| {
                    super::diag::dbg_fail(&format!("crossing rounds: common of the rounds failed: {m}"));
                    BlendError::detail("blendFailed", "the rounds cross, and their common could not be built")
                })?
                .body,
        });
    }
    let raw = acc.ok_or_else(|| BlendError::code("missingEdge"))?;
    let mut refs = vec![];
    for (k, (fa, fb, at)) in edges.iter().enumerate() {
        let e = find_edge(b, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge"))?;
        let ed = b.edge(e);
        refs.push(BlendRef { name: FaceName::blend(feature, k as u32).id(), faces: [fa.clone(), fb.clone()], ends: [b.vertex(ed.v0).p, b.vertex(ed.v1).p] });
    }
    let out = in_place_layout(b, &raw, &refs, tol);
    // both rounds of the narrowest strip must be there
    for k in plan.narrow.1 {
        let name = FaceName::blend(feature, k as u32);
        if !out.face_ids().any(|f| {
            let mut n = out.face(f).name.clone();
            n.region = None;
            n == name
        }) {
            return Err(BlendError::detail("blendFailed", "the rounds cross, and their common lost one of them"));
        }
    }
    Ok(out)
}
