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
use ck_topo::body::{Body, LumpId};

const TOL: f64 = 1e-6;

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

/// The `interference` reply.
pub fn reply(body: &Body) -> Json {
    let lumps: Vec<LumpId> = body.lump_ids().collect();
    // Each lump's box without building its single-lump copy (that copy costs a pass over the body per lump); the copies
    // are made only for lumps whose box meets another's.
    let live = body.face_ids().count();
    let boxes: Vec<(Vec3, Vec3)> = lumps
        .iter()
        .map(|&l| {
            let fs = ck_topo::mass::lump_faces(body, l);
            // a lump that owns every live face leaves nothing to delete (and nothing for the sweep to drop)
            if fs.len() == live { body.bbox() } else { body.bbox_of_faces(&fs) }
        })
        .collect();
    let mut parts: Vec<Option<Body>> = lumps.iter().map(|_| None).collect();
    let first_face = |l: LumpId| ck_topo::mass::lump_faces(body, l).first().map(|&f| body.face_id_str(f)).unwrap_or_default();
    let mut pairs = vec![];
    for i in 0..lumps.len() {
        for j in i + 1..lumps.len() {
            if !boxes_overlap(&boxes[i], &boxes[j]) {
                continue;
            }
            for k in [i, j] {
                if parts[k].is_none() {
                    parts[k] = Some(extract_lump(body, lumps[k]));
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
                    pairs.push(rec.put("volume", mp.volume).put("com", arr(mp.com)).put("box", bx).build());
                }
                Ok(_) => {}
                Err(e) => pairs.push(rec.put("error", e).build()),
            }
        }
    }
    Obj::new().put("bodies", lumps.len()).put("pairs", Json::Arr(pairs)).build()
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
