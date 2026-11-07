//! A mirror of the whole body after an imported starting body (`undo.rs`): SolidWorks' Mirror of bodies tags every
//! face of the copy with the mirror's id, so the copies are the mirror's faces and their seeds the faces of the rest
//! of the body, the imported ones among them (a mirror of features copies only faces of features of its own).
//!
//! The mirror is read back as our `mirror` of the body (`body: true`) in the plane that carries every copy onto a
//! face of the rest. Its material is undone by cutting away the half space on the copies' side of the plane: for a
//! body `B` on one side of the plane, the file's solid is `B + R(B)`, the cut leaves `B`, and the mirror run again
//! gives `B + R(B)` back. A body that crosses the plane loses material to the cut that the mirror does not give back,
//! which the checks of the steps and the verification of the whole history find.

use ck_core::linalg::Plane;
use ck_core::xform::Xform;
use ck_geom::Vec3;
use ck_json::{Json, Obj};
use ck_recog::emit::{arr, n, vec3};
use ck_recog::model::{Kind, Model};
use ck_topo::body::FaceId;
use std::collections::HashSet;

/// The mirror plane `n . x = c` and the side of it (sign of `n . x - c`) the copies lie on.
pub struct Found {
    pub n: Vec3,
    pub c: f64,
    pub side: f64,
}

/// The plane of a body mirror whose copies are the faces tagged `id`, the rest of the body their seeds; `None` when
/// no plane carries every copy onto a face of the rest, or no copy is the image of a face of the sources (`src`).
pub fn find(m: &Model, tags: &[Vec<(i64, i64)>], id: i64, src: &HashSet<i64>) -> Option<Found> {
    let tagged = |g: FaceId, f: &dyn Fn(i64) -> bool| tags.get(g.idx()).is_some_and(|v| v.iter().any(|t| f(t.0)));
    let copies: Vec<FaceId> = m.faces.iter().copied().filter(|&g| tagged(g, &|t| t == id)).collect();
    let rest: Vec<FaceId> = m.faces.iter().copied().filter(|&g| !tagged(g, &|t| t == id)).collect();
    if copies.is_empty() || rest.is_empty() {
        return None;
    }
    // candidate planes from a few analytic copies (the largest boxes first) against every face of the rest
    let analytic = |g: &FaceId| !matches!(m.kind(*g), Kind::Other(_));
    let mut sample: Vec<FaceId> = copies.iter().copied().filter(analytic).collect();
    let size = |g: FaceId| ck_ops::bvh::face_box(m.b, g).diagonal();
    sample.sort_by(|a, b| size(*b).partial_cmp(&size(*a)).unwrap_or(std::cmp::Ordering::Equal));
    sample.truncate(8);
    let mut raw: Vec<(Vec3, f64)> = vec![];
    for &p in &sample {
        for &s in &rest {
            crate::pattern::refl_planes(m.kind(s), m.kind(p), &mut raw);
        }
    }
    let mut groups: Vec<((Vec3, f64), usize)> = vec![];
    for (mv, c) in raw {
        let first = [mv.x, mv.y, mv.z].into_iter().find(|v| v.abs() > 1e-9).unwrap_or(1.0);
        let (mv, c) = if first < 0.0 { (-mv, -c) } else { (mv, c) };
        match groups.iter_mut().find(|g| (g.0 .0 - mv).len() < 1e-7 && (g.0 .1 - c).abs() < 1e-5) {
            Some(g) => g.1 += 1,
            None => groups.push(((mv, c), 1)),
        }
    }
    groups.sort_by(|a, b| b.1.cmp(&a.1));
    // (a plane every sampled copy has an image across: the full test below is the costly one)
    // (a copy a later step changed may have lost its seed: a few in a hundred may have no image, the checks of the
    // steps and the verification hold the plane to the rest)
    let allowed = copies.len() / 20;
    'cands: for ((nv, c), _) in groups.into_iter().take(6) {
        let x = Xform::mirror(&Plane::new(nv * c, nv));
        let (mut from_src, mut miss) = (false, 0usize);
        for &p in &copies {
            let Some(s) = rest.iter().copied().find(|&s| m.kind(s).name() == m.kind(p).name() && crate::pattern::face_img(m, s, &x, p)) else {
                miss += 1;
                if miss > allowed {
                    continue 'cands;
                }
                continue;
            };
            from_src |= tagged(s, &|t| src.contains(&t));
        }
        if !from_src {
            continue;
        }
        // the copies' side: where their sample points lie (on the plane: no side)
        let d: f64 = copies.iter().flat_map(|&p| crate::face_points(m, p)).map(|q| nv.dot(q) - c).sum();
        if d.abs() < 1e-9 {
            continue;
        }
        return Some(Found { n: nv, c, side: d.signum() });
    }
    None
}

/// The features that undo the mirror on the file's solid: a frame plane on the mirror plane, a square sketch on it
/// larger than the body, and its cut into the copies' side, ids from `next`.
pub fn tools(m: &Model, fd: &Found, next: &mut dyn FnMut() -> String) -> Vec<Json> {
    let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    for v in m.b.vertex_ids() {
        let p = m.b.vertex(v).p;
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let o = fd.n * fd.c;
    let size = (hi - lo).len() + (lo - o).len().max((hi - o).len()) + 1.0;
    let t = if fd.n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let x = (t - fd.n * t.dot(fd.n)).normalize();
    let y = fd.n.cross(x);
    let (pid, sid, tid) = (next(), next(), next());
    let pt = |id: &str, a: f64, b: f64| Obj::new().put("id", id).put("x", n(a)).put("y", n(b)).build();
    let ln = |id: &str, a: &str, b: &str| Obj::new().put("type", "line").put("id", id).put("p1", a).put("p2", b).build();
    vec![
        Obj::new().put("type", "plane").put("id", pid.as_str()).put("mode", "frame").put("origin", vec3(o)).put("xDir", vec3(x)).put("yDir", vec3(y)).build(),
        Obj::new()
            .put("type", "sketch")
            .put("id", sid.as_str())
            .put("plane", Obj::new().put("kind", "feature").put("id", pid.as_str()).build())
            .put("points", arr(vec![pt("p1", -size, -size), pt("p2", size, -size), pt("p3", size, size), pt("p4", -size, size)]))
            .put("entities", arr(vec![ln("e1", "p1", "p2"), ln("e2", "p2", "p3"), ln("e3", "p3", "p4"), ln("e4", "p4", "p1")]))
            .put("constraints", arr(vec![]))
            .build(),
        // (the frame's normal is the mirror plane's: a cut runs against it unless flipped)
        ck_recog::emit::extrude(&tid, &sid, "cut", size, fd.side > 0.0, false),
    ]
}
