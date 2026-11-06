//! Move Face (a SolidWorks feature that pushes faces out or in along their normals, the neighbours following).
//!
//! The file records the feature's distance (`D1`) but we do not read the faces it names. The moved faces are found from
//! the body: they are planar faces of the cut before it whose edge with a face of an earlier feature lies inside the
//! material of the body before that cut (the neighbour was extended up to the moved face; where the face had stayed, its
//! edge would lie on the neighbour's own boundary). The cut is then read back with those faces at the place they had
//! before the move (`Cx::shift`), and a `moveFace` feature (offset mode: positive adds material) pushes them out by the
//! listed distance. Whether the move is right is decided by the checks on the whole history, as for every other step.

use crate::{classify, dim, draft, Class, Cx, Status, Step};
use ck_json::Obj;
use ck_ops::contain::{Containment, Loc};
use ck_recog::model::Kind;
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;
use std::collections::HashMap;

pub fn rebuild(cx: &mut Cx, f: &SwFeature, features: &[SwFeature], steps: &[Step], ranges: &mut [(usize, usize)], made: &mut HashMap<i64, (usize, usize)>, groups: &HashMap<i64, Vec<FaceId>>) -> Option<String> {
    let d = dim(cx, f, "D1")?;
    if !(d.abs() > 1e-9) {
        return None;
    }
    // the cut the move follows: the last step that emitted features
    let pi = (0..steps.len()).rev().find(|&i| steps[i].status == Status::Rebuilt && ranges[i].1 > ranges[i].0)?;
    let fp = features.iter().find(|x| x.id == steps[pi].sw_id)?;
    let Class::Extrude { cut: true } = classify(fp) else { return None };
    let faces = groups.get(&fp.id)?.clone();
    let (a, b) = ranges[pi];
    if b != cx.tree.feats.len() {
        return None;
    }
    let m = cx.m;
    // the body before the cut
    let before = cx.sess.borrow_mut().rebuild(&cx.tree.feats[..a]).ok().and_then(|(b, _)| b)?;
    let bc = Containment::new(&before, crate::TOL);
    let mut moved: Vec<FaceId> = vec![];
    for &g in &faces {
        if !matches!(m.kind(g), Kind::Plane { .. }) || cx.tags.get(g.idx()).map_or(true, |v| v.len() != 1) {
            continue;
        }
        let inside = m.edges(g).into_iter().any(|e| {
            let Some(h) = m.across(e, g) else { return false };
            matches!(m.kind(h), Kind::Plane { .. }) && cx.tags.get(h.idx()).map_or(false, |v| !v.is_empty() && v.iter().all(|t| t.0 < fp.id)) && bc.classify_point(m.mid(e), crate::TOL) == Loc::In
        });
        if inside {
            moved.push(g);
        }
    }
    if moved.is_empty() {
        return None;
    }
    // the cut again, with those faces where they were before the move
    let gone: Vec<String> = cx.tree.feats[a..b].iter().filter_map(|j| j.get_str("id").map(|s| format!("{s}."))).collect();
    let saved_feats: Vec<_> = cx.tree.feats.drain(a..).collect();
    let saved_names = cx.names.clone();
    cx.names.retain(|_, n| !gone.iter().any(|p| n.starts_with(p.as_str())));
    for &g in &moved {
        cx.shift.insert(g, d);
    }
    let r = draft::plain_or_drafted(cx, fp, &faces, true);
    cx.shift.clear();
    let restore = |cx: &mut Cx, feats: Vec<ck_json::Json>, names| {
        cx.tree.feats.truncate(a);
        cx.tree.feats.extend(feats);
        cx.names = names;
    };
    if r.is_err() {
        restore(cx, saved_feats, saved_names);
        return None;
    }
    let mut names: Vec<String> = vec![];
    for &g in &moved {
        match cx.lookup(g) {
            Some(n) if !names.contains(&n) => names.push(n),
            Some(_) => {}
            None => {
                restore(cx, saved_feats, saved_names);
                return None;
            }
        }
    }
    let end = cx.tree.feats.len();
    ranges[pi] = (a, end);
    made.insert(fp.id, (a, end));
    let id = cx.tree.ids.next();
    let arr = names.iter().map(|n| ck_json::Json::Str(n.clone())).collect::<Vec<_>>();
    cx.tree.feats.push(Obj::new().put("type", "moveFace").put("id", id.as_str()).put("faces", ck_json::Json::Arr(arr)).put("mode", "offset").put("distance", crate::jn(d)).build());
    Some(format!("{} face(s) of {} read back {:.4} short, pushed out by {:.4} (found from the body: the neighbours were extended)", moved.len(), fp.name, d, d))
}
