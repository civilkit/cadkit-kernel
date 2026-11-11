//! Reference resolution against the current body: face ids -> faces, planar
//! faces -> sketch/plane frames.
//!
//! Face frames (`{"kind":"face","id":"F2.C1"}` sketch planes and plane-feature
//! bases) are pinned by oracle probes on extruded boxes / prisms:
//! * `z` = the face's OUTWARD normal;
//! * `o` = the world origin projected onto the face plane (`z·(z·p)`);
//! * `x` = `normalize(Y × z)`, except when `|z.y| > 0.99` (bisected between
//!   0.98997 and 0.99002), where `x = X` exactly — NOT projected into the
//!   plane, so near-horizontal faces get a slightly skew frame (oracle:
//!   `z = (0.0994, 0.995, 0)` -> `x = (1,0,0)`);
//! * `y = z × x`, unnormalised (oracle: `y = (0,0,-0.995)` in that case).
//!
//! This reproduces the std frames for axis-aligned faces (front/top/right,
//! and for the opposite normals x is mirrored except on ±Y).
//! Non-planar or unknown faces -> `None` (`missingReference`).

use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use crate::regen::FeatureError;
use ck_json::Json;
use ck_topo::body::{Body, FaceId};

/// The live face named `id`.
pub fn find_face(body: &Body, id: &str) -> Option<FaceId> {
    body.find_face(id)
}

/// Frame of the planar face `id` (see module docs).
pub fn face_frame(body: &Body, id: &str) -> Option<Frame> {
    let f = find_face(body, id)?;
    let Surface::Plane { frame } = &body.face(f).surface else {
        // GUESS: only analytic planes count as planar for sketching.
        return None;
    };
    let z = body.planar_normal(f);
    let o = z * z.dot(frame.o);
    let x = if z.y.abs() > 0.99 { Vec3::X } else { Vec3::Y.cross(z).normalize() };
    let y = z.cross(x);
    Some(Frame { o, x, y, z })
}

/// Edit distance between two face ids (to put the likeliest candidates first).
fn distance(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let mut row: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.iter().enumerate() {
        let mut prev = row[0];
        row[0] = i + 1;
        for (j, cb) in b.iter().enumerate() {
            let cur = row[j + 1];
            row[j + 1] = (prev + usize::from(ca != cb)).min(row[j] + 1).min(row[j + 1] + 1);
            prev = cur;
        }
    }
    row[b.len()]
}

/// `missingReference` for the face id `id` that no face of `bodies` carries, with what is there instead so the reference can
/// be repaired in one step: params `ref` (the id), `candidates` (up to six live face ids, those of the same feature first,
/// closest name first; else the three closest names of the whole model) and `hint`.
pub fn missing_face<'a>(bodies: impl IntoIterator<Item = &'a Body>, id: &str) -> FeatureError {
    let feat = id.split('.').next().unwrap_or(id);
    let names: Vec<String> = bodies.into_iter().flat_map(|b| b.face_ids().map(|f| b.face_id_str(f)).collect::<Vec<_>>()).collect();
    let mut same: Vec<&String> = names.iter().filter(|n| n.split('.').next() == Some(feat)).collect();
    same.sort_by_key(|n| (distance(n, id), n.as_str()));
    let (cands, hint): (Vec<String>, String) = if !same.is_empty() {
        let c: Vec<String> = same.iter().take(6).map(|n| (*n).clone()).collect();
        let h = format!("face {id} no longer exists; the faces of {feat} now are {}", c.join(", "));
        (c, h)
    } else {
        let mut all: Vec<&String> = names.iter().collect();
        all.sort_by_key(|n| (distance(n, id), n.as_str()));
        let c: Vec<String> = all.iter().take(3).map(|n| (*n).clone()).collect();
        let h = if c.is_empty() {
            format!("face {id} no longer exists and the model has no faces at this step")
        } else {
            format!("no face of {feat} exists at this step (the feature was removed, moved below this one, or consumed); the closest names are {}", c.join(", "))
        };
        (c, h)
    };
    FeatureError::new(crate::regen::codes::MISSING_REFERENCE)
        .with("ref", Json::Str(id.to_string()))
        .with("candidates", Json::Arr(cands.into_iter().map(Json::Str).collect()))
        .with("hint", Json::Str(hint))
}

/// `missingEdge` for the pick between faces `a` and `b` that no edge joins any more. Says why: a name that no face carries
/// (with the missing-face candidates, params `gone`, `candidates`), or two live faces that no longer meet (params
/// `neighbours`: the faces `a` now meets). Params `faces` and `hint` always.
pub fn missing_edge(body: &Body, a: &str, b: &str) -> FeatureError {
    let base = FeatureError::new("missingEdge").with("faces", Json::Arr(vec![Json::Str(a.into()), Json::Str(b.into())]));
    let gone: Vec<&str> = [a, b].into_iter().filter(|n| body.find_face(n).is_none()).collect();
    if let Some(first) = gone.first() {
        let m = missing_face([body], first);
        let hint = m.params.iter().find(|(k, _)| k == "hint").and_then(|(_, v)| v.as_str()).unwrap_or("").to_string();
        let cands = m.params.iter().find(|(k, _)| k == "candidates").map(|(_, v)| v.clone()).unwrap_or(Json::Arr(vec![]));
        return base
            .with("gone", Json::Arr(gone.iter().map(|n| Json::Str((*n).to_string())).collect()))
            .with("candidates", cands)
            .with("hint", Json::Str(format!("the edge between {a} and {b} is gone: {hint}")));
    }
    let fa = body.find_face(a).expect("checked above");
    let mut near: Vec<String> = vec![];
    for e in body.face_edges(fa) {
        for g in body.edge_faces(e) {
            if g != fa {
                let n = body.face_id_str(g);
                if !near.contains(&n) {
                    near.push(n);
                }
            }
        }
    }
    near.sort_by_key(|n| (distance(n, b), n.clone()));
    near.truncate(6);
    let hint = if near.is_empty() {
        format!("{a} and {b} both exist but no edge joins them any more ({a} has no neighbour faces)")
    } else {
        format!("{a} and {b} both exist but no edge joins them any more; {a} now meets {}", near.join(", "))
    };
    base.with("neighbours", Json::Arr(near.into_iter().map(Json::Str).collect())).with("hint", Json::Str(hint))
}

#[cfg(test)]
mod tests {
    use super::distance;
    #[test]
    fn closest_names_come_first() {
        assert_eq!(distance("F3.S.e2", "F3.S.e2"), 0);
        assert!(distance("F3.S.e2", "F3.S.e3") < distance("F3.S.e2", "F3.C1"));
        assert_eq!(distance("", "ab"), 2);
    }
}
