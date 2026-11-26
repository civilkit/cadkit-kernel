//! Edge lookup by (face, face, point) picks, and the pick normalisation the blend rounds use.

use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName};

/// Faces named `id`; a face an earlier tool split into regions (`F4.C1~1`, `~2`) is still
/// named by its base id in the selection.
pub(super) fn faces_named(b: &Body, id: &str) -> Vec<FaceId> {
    if let Some(found) = b.find_face(id) {
        let exact: Vec<FaceId> = b.face_ids().filter(|&f| b.face_id_str(f) == id).collect();
        // (a name an earlier tool absorbed into a coplanar neighbour, the other half of a wall a split line cut in two,
        // is that neighbour: the pick on it still names the merged face's edge)
        return if exact.is_empty() { vec![found] } else { exact };
    }
    let Some(mut want) = FaceName::parse(id) else { return vec![] };
    want.region = None;
    b.face_ids()
        .filter(|&f| {
            let mut n = b.face(f).name.clone();
            n.region = None;
            n == want
        })
        .collect()
}

/// Edge between the two named faces closest to `at` (fillet `edges[]` ref).
///
/// A selection whose face names no longer fit the geometry (a saved document naming a wall that is now the
/// other half of a split rim, so the numbers shifted) falls back, when the named pair does border an edge but
/// not near `at`, to the edge through `at` between faces of the same two features.
pub fn find_edge(b: &Body, fa: &str, fb: &str, at: Vec3) -> Option<EdgeId> {
    let named = |id: &str| faces_named(b, id);
    let best = find_edge_named(b, fa, fb, at, &named);
    let dist = |e: EdgeId| {
        let ed = b.edge(e);
        crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
    };
    if best.is_some_and(|e| dist(e) > 1e-4 * (1.0 + at.len())) {
        if let Some(e) = find_edge_by_position(b, fa, fb, at) {
            return Some(e);
        }
    }
    best
}

/// The id without its region suffix (`F4.S.e1~2` -> `F4.S.e1`).
pub(super) fn family_id(id: &str) -> &str {
    match id.rfind('~') {
        Some(i) if i + 1 < id.len() && id[i + 1..].bytes().all(|c| c.is_ascii_digit()) && !id.contains('@') => &id[..i],
        _ => id,
    }
}

/// The edge through `at` between faces of the two features the names belong to. For a saved selection whose
/// face names no longer fit (the `~k` or wall numbers shifted when a boolean merged the pieces of a cone, or an
/// earlier kernel numbered them differently) but whose pick point still sits on the edge. Both names must still
/// belong to a face (or a face's family, or an absorbed name), so a misspelt face stays `missingEdge`.
pub fn find_edge_by_position(b: &Body, fa: &str, fb: &str, at: Vec3) -> Option<EdgeId> {
    let exists = |id: &str| {
        b.find_face(id).is_some()
            || b.face_ids().any(|f| family_id(&b.face_id_str(f)) == family_id(id) || b.face(f).absorbed.iter().any(|a| family_id(a) == family_id(id)))
    };
    if !exists(fa) || !exists(fb) {
        return None;
    }
    let (ha, hb) = (FaceName::parse(fa)?, FaceName::parse(fb)?);
    let scale = 1.0 + at.len();
    let by_feature = |f: FaceId, feat: &str| b.face(f).name.feature == feat;
    let dist = |e: EdgeId| {
        let ed = b.edge(e);
        crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
    };
    b.edge_ids()
        .filter(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2
                && ((by_feature(fs[0], &ha.feature) && by_feature(fs[1], &hb.feature)) || (by_feature(fs[1], &ha.feature) && by_feature(fs[0], &hb.feature)))
                && dist(e) <= 1e-6 * scale
        })
        .min_by(|&x, &y| dist(x).total_cmp(&dist(y)))
}

pub(super) fn find_edge_named(b: &Body, fa: &str, fb: &str, at: Vec3, named: &dyn Fn(&str) -> Vec<FaceId>) -> Option<EdgeId> {
    let (fs_a, fs_b) = (named(fa), named(fb));
    b.edge_ids()
        .filter(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2 && ((fs_a.contains(&fs[0]) && fs_b.contains(&fs[1])) || (fs_a.contains(&fs[1]) && fs_b.contains(&fs[0])))
        })
        .min_by(|&x, &y| {
            let d = |e: EdgeId| {
                let ed = b.edge(e);
                crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
            };
            d(x).total_cmp(&d(y))
        })
}

/// `(fa, fb)` with the face whose coedge runs the selected edge forward
/// first (unchanged when the edge is not found).
pub fn forward_first(b: &Body, fa: &str, fb: &str, at: Vec3) -> (String, String, Vec3) {
    let keep = (fa.to_string(), fb.to_string(), at);
    let Some(e) = find_edge(b, fa, fb, at) else { return keep };
    let fwd = b.edge_coedges(e).into_iter().find(|&c| !b.coedge(c).rev).map(|c| b.coedge_face(c));
    match (fwd, b.find_face(fb)) {
        (Some(f), Some(g)) if f == g => (fb.to_string(), fa.to_string(), at),
        _ => keep,
    }
}

/// The edge of `out` a pick `(fa, fb, at)` of the selection's input body means, when the names no longer find it: an
/// earlier tool's rebuild lost the name of a coplanar wall it merged into its neighbour (the other half of a wall a split
/// line cut in two). Found by geometry: the planes of the pick's two faces in the input, and the point.
pub(super) fn find_pick(out: &Body, fa: &str, fb: &str, at: Vec3) -> Option<EdgeId> {
    if let Some(e) = find_edge(out, fa, fb, at) {
        return Some(e);
    }
    use ck_geom::surface::Surface;
    let planes: Option<[(Vec3, Vec3); 2]> = super::analytic::ORIGINAL.with(|o| {
        let o = o.borrow();
        let ob = o.as_ref()?;
        let e0 = find_edge(ob, fa, fb, at)?;
        let fs = ob.edge_faces(e0);
        if fs.len() != 2 || !fs.iter().all(|&f| matches!(ob.face(f).surface, Surface::Plane { .. })) {
            return None;
        }
        let p = ob.vertex(ob.edge(e0).v0).p;
        Some([(p, ob.planar_normal(fs[0]).normalize()), (p, ob.planar_normal(fs[1]).normalize())])
    });
    let planes = planes?;
    let tol = 1e-6 * (1.0 + at.len());
    let on = |f: ck_topo::body::FaceId, (p, n): (Vec3, Vec3)| {
        matches!(out.face(f).surface, Surface::Plane { .. }) && out.planar_normal(f).normalize().dot(n) > 1.0 - 1e-9 && {
            let q = out.vertex(out.edge(out.face_edges(f)[0]).v0).p;
            (q - p).dot(n).abs() < tol
        }
    };
    out.edge_ids()
        .filter(|&e| {
            let fs = out.edge_faces(e);
            fs.len() == 2 && ((on(fs[0], planes[0]) && on(fs[1], planes[1])) || (on(fs[0], planes[1]) && on(fs[1], planes[0])))
        })
        .map(|e| {
            let ed = out.edge(e);
            (e, crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1)
        })
        .filter(|&(_, d)| d <= tol)
        .min_by(|x, y| x.1.total_cmp(&y.1))
        .map(|(e, _)| e)
}
