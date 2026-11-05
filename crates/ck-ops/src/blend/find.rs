//! Edge lookup by (face, face, point) picks, and the pick normalisation the blend rounds use.

use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName};

/// Faces named `id`; a face an earlier tool split into regions (`F4.C1~1`, `~2`) is still
/// named by its base id in the selection.
pub(super) fn faces_named(b: &Body, id: &str) -> Vec<FaceId> {
    if let Some(f) = b.find_face(id) {
        return vec![f];
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
        .min_by(|&x, &y| dist(x).partial_cmp(&dist(y)).unwrap())
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
            d(x).partial_cmp(&d(y)).unwrap()
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
