//! The `interference` request: pairwise intersection of the body's lumps.
//!
//! `{bodies: <lump count>, pairs:[{a, b, volume, com, box}]}` where `a` / `b`
//! are a face id of each lump (the client maps them back to its bodies) and
//! `volume / com / box` describe the common part (`aName` / `bName`: the lump's body name, when it has one); a pair whose boolean fails
//! reports `error` instead. Oracle: parts built by the feature set never
//! contain overlapping lumps (overlapping solids merge), so only
//! `{bodies:n, pairs:[]}` is pinned; the pair record is a GUESS (key order
//! a, b, volume, com, box; `box` = `{min,max}`).

use crate::boolean::{boolean, Op};
use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use crate::bvh::{face_box, Aabb, Bvh};
use ck_topo::body::{Body, FaceId, LumpId};

const TOL: f64 = ck_core::linalg::EPS_LINEAR;

fn arr(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

/// A copy of `body` holding only lump `keep`.
pub fn extract_lump(body: &Body, keep: LumpId) -> Body {
    let mut b = body.clone();
    let mut keep_face = vec![false; body.faces.len()];
    for f in ck_topo::mass::lump_faces(body, keep) {
        keep_face[f.idx()] = true;
    }
    // what `ck_topo::edit::delete_face` does per face, with the dangling-entity sweep (`gc`) run once at the end
    let mut any = false;
    for f in body.face_ids().collect::<Vec<_>>() {
        if keep_face[f.idx()] {
            continue;
        }
        any = true;
        b.faces[f.idx()].alive = false;
        let sh = b.faces[f.idx()].shell;
        b.shells[sh.idx()].faces.retain(|&x| x != f);
        for l in b.faces[f.idx()].loops.clone() {
            b.loops[l.idx()].alive = false;
            for c in b.loops[l.idx()].coedges.clone() {
                b.coedges[c.idx()].alive = false;
            }
        }
    }
    if any {
        ck_topo::edit::gc(&mut b);
    }
    for l in body.lump_ids().collect::<Vec<_>>() {
        if l != keep {
            b.lumps[l.idx()].alive = false;
            for s in b.lumps[l.idx()].shells.clone() {
                b.shells[s.idx()].alive = false;
            }
        }
    }
    b
}

fn boxes_overlap(a: &(Vec3, Vec3), b: &(Vec3, Vec3)) -> bool {
    let ((alo, ahi), (blo, bhi)) = (a, b);
    alo.x <= bhi.x + TOL && blo.x <= ahi.x + TOL && alo.y <= bhi.y + TOL && blo.y <= ahi.y + TOL && alo.z <= bhi.z + TOL && blo.z <= ahi.z + TOL
}

/// Margin (relative to the face's box diagonal) added to the box of a curved face: the box is sampled (boundary edges plus a
/// 12 x 12 grid over the face's uv range), so a bulge between samples is covered by this slack. 5% is far above the sag of a
/// 12-interval sampling of a circle (1 - cos(15 deg) = 3.4% of the radius at the very worst, and only on a face that is a
/// full revolution).
const CURVED_SLACK: f64 = 0.05;

/// A box that holds the face (padded for tolerance), or `None` when the surface kind is one this prefilter does not
/// bound (procedural and swept surfaces): such a face is treated as touching everything.
pub(crate) fn conservative_face_box(body: &Body, f: FaceId) -> Option<Aabb> {
    use ck_geom::surface::Surface;
    // what `imprint` pads its face boxes by (EDGE_TOL + max(tol, 1e-6)), doubled
    let base = 2.0 * (crate::boolean::imprint::EDGE_TOL + TOL.max(1e-6));
    let surf = &body.face(f).surface;
    let mut bx = face_box(body, f);
    match surf {
        Surface::Plane { .. } => {}
        Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. } => {}
        Surface::BSpline(bs) => {
            // the whole control net (convex hull property), not just the trimmed part: the face can only lie inside it
            bx = bx.union(&Aabb::from_points(&bs.ctrl));
        }
        _ => return None,
    }
    if bx.is_empty() || !bx.min.is_finite() || !bx.max.is_finite() {
        return None;
    }
    let slack = if matches!(surf, Surface::Plane { .. }) { 0.0 } else { CURVED_SLACK * bx.diagonal() };
    Some(bx.padded(base + slack))
}

/// A box that holds the whole solid `body` (the box of the union of its conservative face boxes), or `None` when a face
/// has a surface kind that is not bounded (see [`conservative_face_box`]). A solid lies inside the axis-aligned box of
/// its boundary, so a point outside this box is outside the solid.
pub(crate) fn conservative_hull(body: &Body) -> Option<Aabb> {
    let mut hull = Aabb::EMPTY;
    for f in body.face_ids() {
        hull = hull.union(&conservative_face_box(body, f)?);
    }
    if hull.is_empty() { None } else { Some(hull) }
}

/// Box of the vertices of `faces` (a subset of the points of the lump).
fn vertex_box(body: &Body, faces: &[FaceId]) -> Aabb {
    let mut bx = Aabb::EMPTY;
    for &f in faces {
        for e in body.face_edges(f) {
            let ed = body.edge(e);
            bx.add_point(body.vertex(ed.v0).p);
            bx.add_point(body.vertex(ed.v1).p);
        }
    }
    bx
}

fn contains_box(outer: &Aabb, inner: &Aabb) -> bool {
    inner.is_empty() || (outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && outer.min.z <= inner.min.z && inner.max.x <= outer.max.x && inner.max.y <= outer.max.y && inner.max.z <= outer.max.z)
}

/// What the prefilter knows about one lump (computed once, only for lumps in a candidate pair).
struct LumpBoxes {
    /// padded box per face, or `None`: a face the prefilter cannot bound
    faces: Option<Vec<Aabb>>,
    /// union of the face boxes (a superset of the lump)
    hull: Aabb,
    /// box of the lump's vertices (a subset of the lump)
    verts: Aabb,
}

fn lump_boxes(body: &Body, l: LumpId) -> LumpBoxes {
    let fs = ck_topo::mass::lump_faces(body, l);
    let boxes: Option<Vec<Aabb>> = fs.iter().map(|&f| conservative_face_box(body, f)).collect();
    let hull = boxes.as_ref().map_or(Aabb::EMPTY, |bs| bs.iter().fold(Aabb::EMPTY, |a, b| a.union(b)));
    LumpBoxes { faces: boxes, hull, verts: vertex_box(body, &fs) }
}

/// True only when the common part of lumps `a` and `b` is certainly empty, so that `boolean(Intersect)` would answer
/// "disjoint, nothing" (`BoolOut.disjoint`, no faces) and the pair would not be reported.
///
/// Proof. `boolean_at_merge` imprints the two bodies on each other: it pairs the faces whose boxes
/// (`bvh::face_box`, padded by `EDGE_TOL + max(tol, 1e-6)`) overlap, and only a pair that yields cut segments or a
/// coincidence counts in `interacting_pairs`. Here every face box is the same `face_box` with at least twice that padding
/// (more on a curved face, plus the whole control net on a B-spline face, and a face of any other surface kind makes the
/// answer "unknown"), so when no face box of `a` meets any face box of `b` no pair is a candidate and `interacting_pairs`
/// is 0. Then (apply.rs) the result is decided by containment alone: `Intersect` of two bodies that are not nested is
/// the empty "disjoint" result. A body can be inside the other only if all of its points are inside the other's hull
/// box (a solid lies inside the axis-aligned box of its boundary, which is inside the box of the union of its
/// face boxes), in particular its vertices; so when the vertex box of neither lump fits in the hull box of the other, they are not nested. Whenever
/// any of these tests cannot prove it, the caller runs the exact path.
fn provably_disjoint(x: &LumpBoxes, y: &LumpBoxes) -> bool {
    let (Some(fx), Some(fy)) = (&x.faces, &y.faces) else { return false };
    if fx.is_empty() || fy.is_empty() {
        return false;
    }
    let tree = Bvh::build(fy);
    if fx.iter().any(|b| tree.any_overlap(b)) {
        return false;
    }
    !contains_box(&y.hull, &x.verts) && !contains_box(&x.hull, &y.verts)
}

/// A work cap for [`reply_budgeted`]: the `overlap` info of `validate` is information only and must not dominate it.
#[derive(Clone, Copy, Debug)]
pub struct Cap {
    /// Wall-clock allowance in ms, read from the host clock (`ck_core::budget`) before each pair; used only when the host
    /// has a clock.
    pub ms: f64,
    /// Without a host clock: the most exact pair booleans to run.
    pub pairs: usize,
}

/// The `interference` reply.
pub fn reply(body: &Body) -> Json {
    reply_impl(body, true, None)
}

/// [`reply`] that stops starting new pair booleans when `cap` is used up (checked before each pair, never inside a boolean).
/// The cheapest pairs (fewest faces) run first. A reply that was cut short says so with `truncated: true` and
/// `uncheckedPairs: n` after `pairs`; a reply that ran every pair is byte-identical to [`reply`].
pub fn reply_budgeted(body: &Body, cap: Cap) -> Json {
    reply_impl(body, true, Some(cap))
}

/// [`reply`] without the prefilter: a full `boolean(Intersect)` for every pair of lumps whose boxes touch (kept as the
/// reference the prefilter is tested against).
pub fn reply_exhaustive(body: &Body) -> Json {
    reply_impl(body, false, None)
}

fn reply_impl(body: &Body, prefilter: bool, cap: Option<Cap>) -> Json {
    let lumps: Vec<LumpId> = body.lump_ids().collect();
    // Each lump's box without building its single-lump copy (that copy costs a pass over the body per lump); the copies
    // are made only for lumps whose box meets another's.
    let live = body.face_ids().count();
    let mut nfaces = vec![0usize; lumps.len()];
    let boxes: Vec<(Vec3, Vec3)> = lumps
        .iter()
        .enumerate()
        .map(|(k, &l)| {
            let fs = ck_topo::mass::lump_faces(body, l);
            nfaces[k] = fs.len();
            // a lump that owns every live face leaves nothing to delete (and nothing for the sweep to drop)
            if fs.len() == live { body.bbox() } else { body.bbox_of_faces(&fs) }
        })
        .collect();
    let mut parts: Vec<Option<Body>> = lumps.iter().map(|_| None).collect();
    let mut lboxes: Vec<Option<LumpBoxes>> = lumps.iter().map(|_| None).collect();
    let first_face = |l: LumpId| ck_topo::mass::lump_faces(body, l).first().map(|&f| body.face_id_str(f)).unwrap_or_default();
    // the pairs that need the exact boolean, in (i, j) order
    let mut todo: Vec<(usize, usize)> = vec![];
    for i in 0..lumps.len() {
        for j in i + 1..lumps.len() {
            if !boxes_overlap(&boxes[i], &boxes[j]) {
                continue;
            }
            if prefilter {
                for k in [i, j] {
                    if lboxes[k].is_none() {
                        lboxes[k] = Some(lump_boxes(body, lumps[k]));
                    }
                }
                if provably_disjoint(lboxes[i].as_ref().unwrap(), lboxes[j].as_ref().unwrap()) {
                    continue;
                }
            }
            todo.push((i, j));
        }
    }
    // run order: cheapest first when capped (stable, so ties keep (i, j) order); the records are put back in (i, j) order
    let mut order: Vec<usize> = (0..todo.len()).collect();
    if cap.is_some() {
        order.sort_by_key(|&k| nfaces[todo[k].0] + nfaces[todo[k].1]);
    }
    let t0 = ck_core::budget::mark();
    let mut done = 0usize;
    let mut unchecked = 0usize;
    let mut found: Vec<(usize, Json)> = vec![];
    for &k in &order {
        if let Some(c) = cap {
            let out = if t0.is_some() { ck_core::budget::since(t0) >= c.ms } else { done >= c.pairs };
            if out {
                unchecked += 1;
                continue;
            }
        }
        done += 1;
        let (i, j) = todo[k];
        for x in [i, j] {
            if parts[x].is_none() {
                parts[x] = Some(extract_lump(body, lumps[x]));
            }
        }
        let (pi, pj) = (parts[i].as_ref().unwrap(), parts[j].as_ref().unwrap());
        let mut rec = Obj::new().put("a", first_face(lumps[i])).put("b", first_face(lumps[j]));
        // body names (docs/adr/0004), only on lumps that have one
        if let Some(n) = &body.lump(lumps[i]).name {
            rec = rec.put("aName", n.as_str());
        }
        if let Some(n) = &body.lump(lumps[j]).name {
            rec = rec.put("bName", n.as_str());
        }
        match boolean(pi, pj, Op::Intersect, TOL) {
            Ok(out) if out.body.num_faces() > 0 => {
                let mp = ck_topo::mass::mass_props(&out.body);
                if mp.volume.abs() < 1e-9 {
                    continue;
                }
                let (lo, hi) = out.body.bbox();
                let bx = Obj::new().put("min", arr(lo)).put("max", arr(hi)).build();
                found.push((k, rec.put("volume", mp.volume).put("com", arr(mp.com)).put("box", bx).build()));
            }
            Ok(_) => {}
            Err(e) => found.push((k, rec.put("error", e).build())),
        }
    }
    found.sort_by_key(|(k, _)| *k);
    let mut o = Obj::new().put("bodies", lumps.len()).put("pairs", Json::Arr(found.into_iter().map(|(_, j)| j).collect()));
    if unchecked > 0 {
        o = o.put("truncated", true).put("uncheckedPairs", unchecked);
    }
    o.build()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;

    #[test]
    fn disjoint_and_overlapping_lumps() {
        let a = box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0);
        let mut two = a.clone();
        two.append(&box_body("F4", 30.0, 0.0, 40.0, 10.0, 0.0, 10.0));
        let r = reply(&two);
        assert_eq!(r.get_f64("bodies"), Some(2.0));
        assert!(r.get_arr("pairs").unwrap().is_empty());
        let mut over = a.clone();
        over.append(&box_body("F4", 5.0, 5.0, 15.0, 15.0, 0.0, 10.0));
        let r = reply(&over);
        let p = &r.get_arr("pairs").unwrap()[0];
        assert!((p.get_f64("volume").unwrap().abs() - 250.0).abs() < 1e-6, "{}", ck_json::to_string(p));
    }

    /// The one-gc extraction and the box without a copy agree with the per-face `delete_face` extraction they replace.
    #[test]
    fn extraction_and_lump_box_match_the_per_face_deletion() {
        let mut body = box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0);
        body.append(&box_body("F4", 5.0, 5.0, 15.0, 15.0, 0.0, 10.0));
        body.append(&box_body("F6", 30.0, 0.0, 40.0, 10.0, 0.0, 12.0));
        for l in body.lump_ids().collect::<Vec<_>>() {
            let keep = ck_topo::mass::lump_faces(&body, l);
            let mut old = body.clone();
            for f in body.face_ids().collect::<Vec<_>>() {
                if !keep.contains(&f) {
                    ck_topo::edit::delete_face(&mut old, f);
                }
            }
            for o in body.lump_ids().collect::<Vec<_>>() {
                if o != l {
                    old.lumps[o.idx()].alive = false;
                    for s in old.lumps[o.idx()].shells.clone() {
                        old.shells[s.idx()].alive = false;
                    }
                }
            }
            let new = extract_lump(&body, l);
            assert_eq!(new.face_ids().collect::<Vec<_>>(), old.face_ids().collect::<Vec<_>>());
            assert_eq!(new.edge_ids().collect::<Vec<_>>(), old.edge_ids().collect::<Vec<_>>());
            assert_eq!(new.vertex_ids().collect::<Vec<_>>(), old.vertex_ids().collect::<Vec<_>>());
            assert_eq!(new.bbox(), old.bbox());
            assert_eq!(body.bbox_of_faces(&keep), old.bbox());
        }
    }
}
