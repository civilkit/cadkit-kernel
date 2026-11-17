//! Drafts made after the fact: a `Draft` feature that tilts planar faces of earlier features about a neutral plane.
//!
//! The faces carry the tag of the feature that made them, not the draft's, and stand tilted on the final body. They are
//! found as planar faces of earlier features whose normal makes the draft's angle with a principal axis (the pull
//! direction) and that meet a plane normal to that axis along an edge (the neutral plane: the face turns about that
//! edge). Read back, the earlier feature sees the face as it was (straight, through the same edge); the draft step
//! then tilts it again.

use crate::{Cx, Status};
use ck_geom::Vec3;
use ck_recog::model::{comp, Kind, Model};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;
use std::collections::HashMap;

/// One face a later draft tilted.
#[derive(Debug, Clone)]
pub struct Later {
    pub face: FaceId,
    /// Position and id of the draft feature in the file's list, its angle in degrees (as the file lists it).
    pub pos: usize,
    pub feat: i64,
    pub angle: f64,
    /// The face of the neutral plane the draft face meets (a cap normal to the pull direction).
    pub neutral: FaceId,
    /// The face as it was before the draft.
    pub before: Kind,
}

pub fn is_draft(f: &SwFeature) -> bool {
    f.kind == "Draft" || f.kind == "DraftXpert" || f.name.starts_with("Draft")
}

/// The draft angle of a feature, degrees.
pub fn angle(f: &SwFeature) -> Option<f64> {
    f.dims.iter().find(|d| d.name == "D1").and_then(|d| d.value).filter(|a| a.abs() > 1e-6 && a.abs() < 89.0)
}

/// Faces tilted by later drafts, with the neutral plane of choice number `variant` for each (they are tried in order of
/// their level along the pull axis; the first has the draft face turn about the lowest one).
pub fn find(m: &Model, tags: &[Vec<(i64, i64)>], features: &[SwFeature], variant: usize) -> Vec<Later> {
    candidates(m, tags, features).into_iter().filter_map(|(l, cands)| cands.get(variant.min(cands.len().saturating_sub(1))).cloned().map(|(h, before)| Later { neutral: h, before, ..l })).collect()
}

/// The largest number of neutral planes any face of any draft can turn about.
pub fn variants(m: &Model, tags: &[Vec<(i64, i64)>], features: &[SwFeature]) -> usize {
    candidates(m, tags, features).iter().map(|c| c.1.len()).max().unwrap_or(0)
}

fn candidates(m: &Model, tags: &[Vec<(i64, i64)>], features: &[SwFeature]) -> Vec<(Later, Vec<(FaceId, Kind)>)> {
    let pos: HashMap<i64, usize> = features.iter().enumerate().map(|(i, f)| (f.id, i)).collect();
    let mut out: Vec<Vec<(Later, Vec<(FaceId, Kind)>)>> = vec![];
    // The faces a later draft tilted, as they were before it: a draft made earlier turned about a neutral plane that the later
    // draft may have tilted in its turn, so drafts are read latest first and an earlier one sees those faces straightened.
    let mut undone: HashMap<FaceId, Kind> = HashMap::new();
    let drafts: Vec<(usize, &SwFeature)> = features.iter().enumerate().filter(|(_, f)| is_draft(f)).collect();
    for &(pd, d) in drafts.iter().rev() {
        let mut found = vec![];
        let Some(a) = angle(d) else { continue };
        let sin = a.abs().to_radians().sin();
        let later_only = |g: FaceId| tags.get(g.idx()).map_or(false, |v| !v.is_empty() && v.iter().all(|t| pos.get(&t.0).map_or(true, |&p| p > pd)));
        for &g in &m.faces {
            // (a face a later draft tilted is that draft's: an earlier draft of the same angle does not tilt it again)
            if undone.contains_key(&g) {
                continue;
            }
            let Kind::Plane { n, d: dn } = m.kind(g).clone() else { continue };
            let Some(ts) = tags.get(g.idx()).filter(|v| !v.is_empty()) else { continue };
            // a face of the draft's own would have been tilted by it; this is for faces of earlier features alone
            if !ts.iter().all(|t| pos.get(&t.0).map_or(false, |&p| p < pd)) {
                continue;
            }
            for i in 0..3 {
                let sg = comp(n, i);
                if (sg.abs() - sin).abs() > 1e-6 {
                    continue;
                }
                let mut e = Vec3::ZERO;
                match i {
                    0 => e.x = 1.0,
                    1 => e.y = 1.0,
                    _ => e.z = 1.0,
                }
                // the neutral plane: a cap normal to the pull axis that this face meets, directly or through a face a
                // later feature made (a blend at the foot of the wall)
                let mut caps: Vec<(FaceId, f64)> = vec![];
                let is_cap = |h: FaceId| match undone.get(&h).unwrap_or_else(|| m.kind(h)) {
                    Kind::Plane { n: nh, d } if nh.dot(e).abs() > 1.0 - 1e-9 => Some(d * nh.dot(e)),
                    _ => None,
                };
                for ed in m.edges(g) {
                    let Some(h) = m.across(ed, g) else { continue };
                    if let Some(l) = is_cap(h) {
                        caps.push((h, l));
                    } else if later_only(h) {
                        for ed2 in m.edges(h) {
                            if let Some(h2) = m.across(ed2, h) {
                                if let Some(l) = is_cap(h2) {
                                    caps.push((h2, l));
                                }
                            }
                        }
                    }
                }
                caps.sort_by(|x, y| x.1.total_cmp(&y.1));
                caps.dedup_by(|x, y| (x.1 - y.1).abs() < 1e-6);
                if caps.is_empty() {
                    continue;
                }
                let c = (1.0 - sg * sg).sqrt();
                let list: Vec<(FaceId, Kind)> = caps
                    .iter()
                    .map(|&(h, l)| {
                        // the face's plane through the line it makes with the neutral plane, straightened
                        let nperp = (n - e * sg).normalize();
                        (h, Kind::Plane { n: nperp, d: (dn - sg * l) / c })
                    })
                    .collect();
                let l0 = Later { face: g, pos: pd, feat: d.id, angle: a, neutral: list[0].0, before: list[0].1.clone() };
                found.push((l0, list));
                break;
            }
        }
        // one SolidWorks draft has one neutral plane: the level most of its faces can turn about comes first in every
        // face's list (a face's own first candidate, a floor of a later hole it happens to meet, is not that plane)
        if found.len() > 1 {
            let mut levels: Vec<(f64, usize)> = vec![];
            for (_, list) in &found {
                for &(h, _) in list {
                    let lv = match m.kind(h) {
                        Kind::Plane { n, d } => d * n.dot(Vec3::new(n.x.signum(), n.y.signum(), n.z.signum())),
                        _ => continue,
                    };
                    match levels.iter_mut().find(|x| (x.0 - lv).abs() < 1e-6) {
                        Some(x) => x.1 += 1,
                        None => levels.push((lv, 1)),
                    }
                }
            }
            if let Some(&(best, count)) = levels.iter().max_by_key(|x| x.1) {
                if count > 1 {
                    for (l, list) in found.iter_mut() {
                        let at = |h: FaceId| match m.kind(h) {
                            Kind::Plane { n, d } => ((d * n.dot(Vec3::new(n.x.signum(), n.y.signum(), n.z.signum()))) - best).abs() < 1e-6,
                            _ => false,
                        };
                        if let Some(k) = list.iter().position(|&(h, _)| at(h)) {
                            if k > 0 {
                                let item = list.remove(k);
                                list.insert(0, item);
                                l.neutral = list[0].0;
                                l.before = list[0].1.clone();
                            }
                        }
                    }
                }
            }
        }
        for (l, list) in &found {
            undone.entry(l.face).or_insert_with(|| list[0].1.clone());
        }
        out.push(found);
    }
    // in the order of the file's list, as before
    out.into_iter().rev().flatten().collect()
}

/// The draft step: the faces the file's draft feature tilted, tilted again about the cap they turn about.
pub fn rebuild(cx: &mut Cx, f: &SwFeature) -> Result<(String, Status), String> {
    let mine: Vec<Later> = cx.later.iter().filter(|l| l.feat == f.id).cloned().collect();
    if mine.is_empty() {
        return Ok(("no face of earlier features is tilted by its angle".into(), Status::Skipped));
    }
    // a face that stands tilted already in the document rebuilt so far (its feature was read back on the tilted face:
    // the floor of a slab cut, a drafted extrude) is left out, face by face: the draft adds nothing for it
    let total = mine.len();
    let mine: Vec<Later> = match cx.sess.borrow_mut().rebuild(&cx.tree.feats).ok().and_then(|(b, _)| b) {
        Some(b) => {
            let mb = Model::new(&b);
            mine.into_iter().filter(|l| !mb.faces.iter().any(|&h| crate::fastverify::same_surface(cx.m.kind(l.face), mb.kind(h), 1e-6))).collect()
        }
        None => mine,
    };
    if mine.is_empty() {
        return Ok((format!("{total} face(s) stand tilted already (the draft is part of their features)"), Status::Rebuilt));
    }
    let name_of = |cx: &Cx, g: FaceId| cx.lookup(g).ok_or_else(|| format!("face {} has no name in the rebuilt document", cx.m.b.face_id_str(g)));
    // faces sharing a neutral cap go in one draft
    let mut groups: Vec<(FaceId, Vec<FaceId>)> = vec![];
    for l in &mine {
        match groups.iter_mut().find(|g| g.0 == l.neutral) {
            Some(g) => g.1.push(l.face),
            None => groups.push((l.neutral, vec![l.face])),
        }
    }
    let mut note = String::new();
    for (neutral, faces) in groups {
        // the names the rebuilt document gives the faces may not single them out: a cap the body holds only as pieces
        // `<name>~k` (a profile with several outer loops, a cap its own sketch's holes split), sides the profile did not
        // touch (all `<id>.S.e1`), a pattern copy's name the kernel does not keep. The face the draft tilts is found by
        // its geometry on the body before the draft: the rebuilt faces on the plane the file's face had before the
        // draft that contain its points, named exactly; a face no rebuilt face matches keeps its name. The neutral is
        // a plane: any named rebuilt face on it serves.
        let body = cx.sess.borrow_mut().rebuild(&cx.tree.feats).ok().and_then(|(b, _)| b);
        // the faces stand tilted already (the extrude that made them was read back with this draft built in): nothing
        // to add for them
        if let Some(b) = &body {
            let mb = Model::new(b);
            if faces.iter().all(|&g| mb.faces.iter().any(|&h| crate::fastverify::same_surface(cx.m.kind(g), mb.kind(h), 1e-6))) {
                if note.is_empty() {
                    note = format!("{} face(s) stand tilted already (the draft is part of their extrude)", faces.len());
                }
                continue;
            }
        }
        let on_plane = |b: &ck_topo::body::Body| -> Option<String> {
            let mb = Model::new(b);
            let want = cx.m.kind(neutral);
            mb.faces.iter().find(|&&h| crate::fastverify::same_surface(want, mb.kind(h), 1e-6)).map(|&h| b.face_id_str(h))
        };
        let nn = match (name_of(cx, neutral), &body) {
            // (a named neutral the body holds only as pieces `<name>~k`: any face on its plane)
            (Ok(n), Some(b)) if b.find_face(&n).is_none() => on_plane(b).unwrap_or(n),
            (Ok(n), _) => n,
            (Err(e), Some(b)) => on_plane(b).ok_or(e)?,
            (Err(e), None) => return Err(e),
        };
        let mut names: Vec<ck_json::Json> = vec![];
        for &g in &faces {
            let mut found: Vec<ck_json::Json> = vec![];
            if let (Some(b), Some(before)) = (&body, mine.iter().find(|l| l.face == g).map(|l| l.before.clone())) {
                let mb = Model::new(b);
                let pts = crate::face_points(cx.m, g);
                // (the file's face is tilted: its points are moved back onto the plane it had)
                let on = |p: Vec3| match &before {
                    Kind::Plane { n, d } => p - *n * (n.dot(p) - *d),
                    _ => p,
                };
                for &h in &mb.faces {
                    if crate::fastverify::same_surface(&before, mb.kind(h), 1e-6) && pts.iter().any(|&p| !matches!(ck_ops::boolean::classify::point_in_face(b, h, on(p), 1e-6), ck_ops::boolean::classify::Where::Out)) {
                        found.push(ck_json::Json::Str(b.face_id_str(h)));
                    }
                }
            }
            if found.is_empty() {
                // (a name the rebuilt body does not hold and no face where the file's face stood: the face is not in
                // the rebuilt document, an earlier step did not make it; said as such)
                let nm = name_of(cx, g)?;
                if body.as_ref().is_some_and(|b| b.find_face(&nm).is_none() && !b.face_ids().any(|h| b.face_id_str(h).split('~').next() == Some(nm.as_str()))) {
                    return Err(format!("face {} ({nm}) is not in the rebuilt document: no face on the plane it had before the draft, and no face of that name", cx.m.b.face_id_str(g)));
                }
                names.push(ck_json::Json::Str(nm));
            } else {
                names.extend(found);
            }
        }
        names.dedup();
        let a = mine[0].angle;
        let first = cx.tree.feats.len();
        let mut done = false;
        for sign in [1.0, -1.0] {
            cx.tree.feats.truncate(first);
            let id = cx.tree.ids.next();
            let o = ck_json::Obj::new()
                .put("type", "draft")
                .put("id", id.as_str())
                .put("neutral", ck_json::Obj::new().put("kind", "face").put("id", nn.as_str()).build())
                .put("faces", ck_json::Json::Arr(names.clone()))
                .put("angle", ck_recog::emit::n(sign * a))
                .build();
            cx.tree.push(o);
            let ok = match cx.sess.borrow_mut().rebuild(&cx.tree.feats) {
                Ok((Some(b), errs)) if errs.is_empty() => {
                    let mb = Model::new(&b);
                    faces.iter().all(|&g| mb.faces.iter().any(|&h| crate::fastverify::same_surface(cx.m.kind(g), mb.kind(h), 1e-6)))
                }
                _ => false,
            };
            if ok {
                done = true;
                note = format!("{} face(s) tilted {:.4} deg about the cap {}", faces.len(), sign * a, nn);
                break;
            }
        }
        if !done {
            cx.tree.feats.truncate(first);
            return Err("the draft does not give the file's faces (either sign)".into());
        }
    }
    Ok((note, Status::Rebuilt))
}
