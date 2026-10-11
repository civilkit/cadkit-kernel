//! The `curvatureAt` request: principal curvatures of a face's surface at the
//! point closest to a probe point.
//!
//! `{points:[{face, p:[x,y,z]}]}` -> one record per point:
//! `{p, n, k1, k2, d1, d2, gaussian, mean}` (the projected point on the
//! UNTRIMMED surface, the outward normal, principal curvatures `k1 >= k2`
//! with their tangent directions; positive where the surface bends away from
//! the normal) or `{"error":"notFound"}` for an unknown face / malformed
//! entry. Numerics: [`ck_geom::analysis::surface_curvature`].

use ck_core::linalg::Vec3;
use ck_geom::analysis::surface_curvature;
use ck_json::{Json, Obj};
use ck_topo::body::Body;

fn arr(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

pub fn not_found() -> Json {
    Obj::new().put("error", "notFound").build()
}

/// One probe -> record.
pub fn probe(body: &Body, entry: &Json) -> Json {
    let face = entry.get_str("face").and_then(|id| body.find_face(id));
    let p = entry.get("p").and_then(Json::as_vec3);
    let (Some(f), Some(p)) = (face, p) else { return not_found() };
    let face = body.face(f);
    let (u, v, pt) = face.surface.closest_point(Vec3::from_array(p));
    let mut k = surface_curvature(&face.surface.derivs2(u, v), !face.sense);
    if k.singular {
        // oracle (sphere poles): evaluate the frame a hair inside the
        // domain, 1e-7 of its extent, while `p` stays the projected point
        let (_, (v0, v1)) = face.surface.domain();
        if v0.is_finite() && v1.is_finite() {
            let eps = (v1 - v0) * 1e-7;
            let nv = if (v - v0).abs() < 1e-9 {
                v + eps
            } else if (v - v1).abs() < 1e-9 {
                v - eps
            } else {
                v
            };
            if nv != v {
                k = surface_curvature(&face.surface.derivs2(u, nv), !face.sense);
            }
        }
    }
    let mut o = Obj::new().put("p", arr(pt));
    if k.singular {
        o = o.put("singular", true);
    }
    o.put("n", arr(k.n))
        .put("k1", k.k1)
        .put("k2", k.k2)
        .put("d1", arr(k.d1))
        .put("d2", arr(k.d2))
        .put("gaussian", k.gaussian)
        .put("mean", k.mean)
        .build()
}

/// The `curvatureAt` reply (`points` not an array -> `[]`).
pub fn reply(body: &Body, msg: &Json) -> Json {
    Json::Arr(msg.get_arr("points").unwrap_or(&[]).iter().map(|e| probe(body, e)).collect())
}
