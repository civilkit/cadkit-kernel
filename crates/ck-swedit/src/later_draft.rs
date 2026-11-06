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
    let mut out = vec![];
    for (pd, d) in features.iter().enumerate().filter(|(_, f)| is_draft(f)) {
        let Some(a) = angle(d) else { continue };
        let sin = a.abs().to_radians().sin();
        let later_only = |g: FaceId| tags.get(g.idx()).map_or(false, |v| !v.is_empty() && v.iter().all(|t| pos.get(&t.0).map_or(true, |&p| p > pd)));
        for &g in &m.faces {
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
                let is_cap = |h: FaceId| match m.kind(h) {
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
                out.push((l0, list));
                break;
            }
        }
    }
    out
}

/// The draft step: the faces the file's draft feature tilted, tilted again about the cap they turn about.
pub fn rebuild(cx: &mut Cx, f: &SwFeature) -> Result<(String, Status), String> {
    let mine: Vec<Later> = cx.later.iter().filter(|l| l.feat == f.id).cloned().collect();
    if mine.is_empty() {
        return Ok(("no face of earlier features is tilted by its angle".into(), Status::Skipped));
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
        let nn = name_of(cx, neutral)?;
        let names: Vec<ck_json::Json> = faces.iter().map(|&g| name_of(cx, g).map(ck_json::Json::Str)).collect::<Result<_, _>>()?;
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
