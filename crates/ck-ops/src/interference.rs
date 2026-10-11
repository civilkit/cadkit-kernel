//! The `interference` request: pairwise intersection of the body's lumps.
//!
//! `{bodies: <lump count>, pairs:[{a, b, volume, com, box}]}` where `a` / `b`
//! are a face id of each lump (the client maps them back to its bodies) and
//! `volume / com / box` describe the common part; a pair whose boolean fails
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
    let keep_faces = ck_topo::mass::lump_faces(body, keep);
    for f in body.face_ids().collect::<Vec<_>>() {
        if !keep_faces.contains(&f) {
            ck_topo::edit::delete_face(&mut b, f);
        }
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

fn boxes_overlap(a: &Body, b: &Body) -> bool {
    let ((alo, ahi), (blo, bhi)) = (a.bbox(), b.bbox());
    alo.x <= bhi.x + TOL && blo.x <= ahi.x + TOL && alo.y <= bhi.y + TOL && blo.y <= ahi.y + TOL && alo.z <= bhi.z + TOL && blo.z <= ahi.z + TOL
}

/// The `interference` reply.
pub fn reply(body: &Body) -> Json {
    let lumps: Vec<LumpId> = body.lump_ids().collect();
    let parts: Vec<Body> = lumps.iter().map(|&l| extract_lump(body, l)).collect();
    let first_face = |l: LumpId| ck_topo::mass::lump_faces(body, l).first().map(|&f| body.face_id_str(f)).unwrap_or_default();
    let mut pairs = vec![];
    for i in 0..parts.len() {
        for j in i + 1..parts.len() {
            if !boxes_overlap(&parts[i], &parts[j]) {
                continue;
            }
            let rec = Obj::new().put("a", first_face(lumps[i])).put("b", first_face(lumps[j]));
            match boolean(&parts[i], &parts[j], Op::Intersect, TOL) {
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
}
