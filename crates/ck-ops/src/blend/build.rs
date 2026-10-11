//! Blend construction (ck-ops/src/blend/build.rs) — FIRST PASS: the
//! simplest case only, a constant-radius fillet along a straight edge
//! between two planar faces (convex or concave), built as a boolean with
//! a "corner prism" tool whose arc face becomes the blend face `B<k>`.
//!
//! Oracle evidence (probes on box F2, fillet feature F3):
//! - `fillet r=2` on `F2.C1|F2.S.e1`: faces `C0, S.e3` (untouched, original
//!   order), `C1, S.e1` (the edge's faces, ref order), `S.e4, S.e2` (end
//!   faces at the edge's start, end vertex), `F3.B0` (cylinder, meta
//!   `{feature:"F3"}`); end-face edges are circle arcs c = axis point,
//!   n = edge direction (sign from the end face), r = 2.
//! - two edges → `B0`, `B1` in ref order; touched faces in first-touch order.
//! - errors: radius 0 → `invalidValue`; no / unknown edge → `missingEdge`;
//!   radius 30 → `blendOverflow` with detail
//!   `"F3.B0: spring leaves its face away from its edges"`; the body is
//!   left unchanged in all error cases.
//! - GUESS/known mismatch: the oracle blends IN PLACE (blend/inplace.rs):
//!   surviving edges keep their arena number in the wire key `#n`, split
//!   pieces and new edges are appended (fillet probe: #0..#4, #6, #8..#11,
//!   #14, #16..#19). This boolean-based pass renumbers edges in face-walk
//!   order; face ids, order, geometry and massProps match.

use super::BlendError;
use crate::boolean::assemble::{build, select};
use crate::boolean::classify::{point_in_face, Where};
use crate::boolean::imprint::imprint;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

/// Edge between the two named faces closest to `at` (fillet `edges[]` ref).
pub fn find_edge(b: &Body, fa: &str, fb: &str, at: Vec3) -> Option<EdgeId> {
    let (f, g) = (b.find_face(fa)?, b.find_face(fb)?);
    b.edge_ids()
        .filter(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2 && fs.contains(&f) && fs.contains(&g)
        })
        .min_by(|&x, &y| {
            let d = |e: EdgeId| {
                let ed = b.edge(e);
                crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
            };
            d(x).partial_cmp(&d(y)).unwrap()
        })
}

/// Constant-radius fillets of `edges` (face-name pairs + pick point, in
/// ref order), blend faces `B0, B1, …` of `feature`.
pub fn fillet(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    if !(r > 0.0) || !r.is_finite() {
        return Err(BlendError::code("invalidValue"));
    }
    if edges.is_empty() {
        return Err(BlendError::code("missingEdge"));
    }
    let mut out = b.clone();
    let mut touched: Vec<String> = vec![];
    for (k, (fa, fb, at)) in edges.iter().enumerate() {
        let e = find_edge(&out, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge"))?;
        let (f1, f2) = (out.find_face(fa).unwrap(), out.find_face(fb).unwrap());
        let (body, hint) = fillet_line_edge_hint(&out, e, [f1, f2], r, feature, k as u32, tol)?;
        for h in hint {
            if !touched.contains(&h) {
                touched.push(h);
            }
        }
        out = body;
    }
    if edges.len() == 1 {
        return Ok(out);
    }
    // one blend op over all edges: untouched faces, touched faces in
    // first-touch order, blend faces (probe fil2)
    let base = |id: &str| FaceName::parse(id).map(|mut n| {
        n.region = None;
        n.id()
    });
    let rank = |f: FaceId| -> (usize, usize) {
        let name = &out.face(f).name;
        if name.feature == feature && matches!(name.role, Role::Blend(_)) {
            return (2, 0);
        }
        match touched.iter().position(|t| base(t) == base(&name.id())) {
            Some(p) => (1, p),
            None => (0, 0),
        }
    };
    let mut order: Vec<FaceId> = out.face_ids().collect();
    order.sort_by_key(|&f| rank(f)); // stable: keeps relative order within ties
    Ok(crate::boolean::assemble::rebuild_in_order(&out, &order))
}

/// One straight edge between two planar faces.
pub fn fillet_line_edge(b: &Body, e: EdgeId, faces: [FaceId; 2], r: f64, feature: &str, k: u32, tol: f64) -> Result<Body, BlendError> {
    fillet_line_edge_hint(b, e, faces, r, feature, k, tol).map(|x| x.0)
}

/// As [`fillet_line_edge`], also returning the touched-face order hint
/// (edge faces, then end faces).
fn fillet_line_edge_hint(
    b: &Body,
    e: EdgeId,
    faces: [FaceId; 2],
    r: f64,
    feature: &str,
    k: u32,
    tol: f64,
) -> Result<(Body, Vec<String>), BlendError> {
    let ed = b.edge(e).clone();
    let Curve::Line { .. } = ed.curve else { return Err(BlendError::code("blendUnsupported")) };
    for f in faces {
        if !matches!(b.face(f).surface, Surface::Plane { .. }) {
            return Err(BlendError::code("blendUnsupported"));
        }
    }
    let p0 = b.vertex(ed.v0).p;
    let p1 = b.vertex(ed.v1).p;
    let len = p0.dist(p1);
    let dir = (p1 - p0) / len;
    let n1 = b.planar_normal(faces[0]);
    let n2 = b.planar_normal(faces[1]);
    let c12 = n1.dot(n2);
    if (1.0 + c12).abs() < 1e-9 || (1.0 - c12).abs() < 1e-9 {
        return Err(BlendError::code("blendUnsupported")); // tangent / folded faces
    }
    // convex iff the direction into F1 (left of its coedge about the
    // outward normal) points against n2
    let pm = (p0 + p1) * 0.5;
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == faces[0]).ok_or_else(|| BlendError::code("missingEdge"))?;
    let tdir = if b.coedge(co).rev { -dir } else { dir };
    let convex = n1.cross(tdir).dot(n2) < 0.0;
    let s = if convex { -1.0 } else { 1.0 };
    let off = (n1 + n2) * (s * r / (1.0 + c12));
    // contact (spring) lines must stay on their faces
    let name = FaceName::blend(feature, k);
    for (f, n) in [(faces[0], n1), (faces[1], n2)] {
        let t = pm + off - n * (s * r);
        if point_in_face(b, f, t, tol) == Where::Out {
            return Err(BlendError::detail("blendOverflow", &format!("{}: spring leaves its face away from its edges", name.id())));
        }
    }
    // tool profile in the plane ⟂ dir through p0 − dir·margin
    // convex: the tool pierces the end faces (the cut trims it); concave:
    // GUESS end faces ⟂ the edge, the tool ends flush with them
    let margin = if convex { r.max(len * 0.05) + 1.0 } else { 0.0 };
    let o = p0 - dir * margin;
    let fr = Frame::from_normal(o, dir);
    let uv = |p: Vec3| {
        let q = fr.world_to_uv(p);
        Vec2::new(q.x, q.y)
    };
    let (cp, t1, t2) = (uv(p0), uv(p0 + off - n1 * (s * r)), uv(p0 + off - n2 * (s * r)));
    let cc = uv(p0 + off);
    let short_ccw = (t1 - cc).cross(t2 - cc) > 0.0;
    let arc = if short_ccw { ProfileSeg::arc("arc", cc, t1, t2) } else { ProfileSeg::arc("arc", cc, t2, t1).reversed() };
    let lp = ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), arc]);
    let prof = Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]).normalized();
    let mut tool = extrude(feature, &prof, ExtrudeOpts { s0: 0.0, s1: len + 2.0 * margin, dir: None, draft: 0.0 })
        .map_err(|_| BlendError::code("blendFailed"))?;
    for f in &mut tool.faces {
        if f.name.role == Role::Side("arc".into()) {
            f.name = name.clone();
        }
    }
    let op = if convex { Op::Cut } else { Op::Union };
    let arr = imprint(b, &tool, tol);
    let mut faces_out = select(&arr, [b, &tool], op).map_err(|_| BlendError::code("blendFailed"))?;
    // order: untouched; edge faces (ref order), end faces (start, end);
    // other touched faces; blend
    let ends: Vec<FaceId> = [ed.v0, ed.v1]
        .iter()
        .flat_map(|&v| {
            b.edge_ids()
                .filter(move |&x| b.edge(x).v0 == v || b.edge(x).v1 == v)
                .flat_map(|x| b.edge_faces(x))
                .filter(|f| !faces.contains(f))
                .collect::<Vec<_>>()
        })
        .collect();
    let mut hint: Vec<String> = faces.iter().map(|&f| b.face_id_str(f)).collect();
    for f in ends {
        let id = b.face_id_str(f);
        if !hint.contains(&id) {
            hint.push(id);
        }
    }
    for f in &mut faces_out {
        if f.group == 1 {
            let mut base = f.name.clone();
            base.region = None;
            let pos = hint.iter().position(|h| FaceName::parse(h).map(|mut x| {
                x.region = None;
                x
            }) == Some(base.clone()));
            f.key = (1, pos.unwrap_or(hint.len()), f.key.1 * 1000 + f.key.2);
        } else if f.group == 4 {
            f.key = (1, hint.len() + 1, f.key.1);
        }
        // walk items follow the face key (edge numbering = face-walk order)
        let key = f.key;
        for w in &mut f.walk {
            w.0 = key;
        }
    }
    faces_out.sort_by_key(|f| f.key);
    if !faces_out.iter().any(|f| f.name == name) {
        return Err(BlendError::code("blendFailed"));
    }
    let body = build(&arr, &faces_out);
    let bad = ck_topo::check::check(&body, tol)
        .into_iter()
        .any(|i| matches!(i, ck_topo::check::Issue::OpenEdge(_) | ck_topo::check::Issue::BrokenLoop(..)));
    if bad {
        return Err(BlendError::code("blendFailed"));
    }
    Ok((body, hint))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;
    use ck_topo::mass::mass_props;

    fn the_box() -> Body {
        box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0)
    }
    fn ids(b: &Body) -> Vec<String> {
        b.face_ids().map(|f| b.face_id_str(f)).collect()
    }
    fn mass(b: &Body, vol: f64, area: f64, com: [f64; 3]) {
        let m = mass_props(b);
        assert!((m.volume - vol).abs() < 1e-9 * vol, "{} vs {vol}", m.volume);
        assert!((m.area - area).abs() < 1e-9 * area, "{} vs {area}", m.area);
        assert!((m.com - Vec3::new(com[0], com[1], com[2])).len() < 1e-8, "{:?}", m.com);
    }
    fn e(a: &str, b: &str, at: [f64; 3]) -> (String, String, Vec3) {
        (a.into(), b.into(), Vec3::new(at[0], at[1], at[2]))
    }

    #[test]
    fn oracle_fillet_one_edge() {
        let r = fillet(&the_box(), &[e("F2.C1", "F2.S.e1", [20., 10., 0.])], 2.0, "F3", 1e-6).unwrap();
        assert!(ck_topo::check::check(&r, 1e-6).is_empty());
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e3", "F2.C1", "F2.S.e1", "F2.S.e4", "F2.S.e2", "F3.B0"]);
        mass(&r, 7965.6637061435895, 2763.946891450771, [20.000000000000004, 4.980372983260395, -10.041179705330975]);
        let b0 = r.find_face("F3.B0").unwrap();
        assert_eq!(r.face(b0).surface.kind(), "cylinder");
        // end-face arcs: c = (0,8,-2) / (40,8,-2), r = 2
        let arcs: Vec<(Vec3, f64)> = r
            .face_edges(b0)
            .into_iter()
            .filter_map(|x| r.edge(x).curve.circle_info().map(|(c, _, rr)| (c, rr)))
            .collect();
        assert_eq!(arcs.len(), 2);
        for (c, rr) in arcs {
            assert!((rr - 2.0).abs() < 1e-12);
            assert!((c.y - 8.0).abs() < 1e-12 && (c.z + 2.0).abs() < 1e-12);
        }
    }

    #[test]
    fn oracle_fillet_two_edges_and_vertical() {
        let r = fillet(
            &the_box(),
            &[e("F2.C1", "F2.S.e1", [20., 10., 0.]), e("F2.C1", "F2.S.e3", [20., 10., -20.])],
            2.0,
            "F3",
            1e-6,
        )
        .unwrap();
        assert_eq!(ids(&r), ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e4", "F2.S.e2", "F2.S.e3", "F3.B0", "F3.B1"]);
        mass(&r, 7931.327412287184, 2727.8937829015417, [20., 4.960576028002486, -10.]);
        let r = fillet(&the_box(), &[e("F2.S.e1", "F2.S.e2", [40., 5., 0.])], 2.0, "F3", 1e-6).unwrap();
        assert_eq!(ids(&r), ["F2.S.e3", "F2.S.e4", "F2.S.e1", "F2.S.e2", "F2.C0", "F2.C1", "F3.B0"]);
        mass(&r, 7991.4159265358985, 2789.6991118430774, [19.978996631231546, 5., -10.010261751083915]);
    }

    #[test]
    fn oracle_fillet_errors() {
        let b = the_box();
        let one = [e("F2.C1", "F2.S.e1", [20., 10., 0.])];
        assert_eq!(fillet(&b, &one, 0.0, "F3", 1e-6).unwrap_err(), BlendError::code("invalidValue"));
        assert_eq!(fillet(&b, &[], 2.0, "F3", 1e-6).unwrap_err(), BlendError::code("missingEdge"));
        assert_eq!(
            fillet(&b, &[e("F2.C1", "F2.S.e9", [20., 10., 0.])], 2.0, "F3", 1e-6).unwrap_err(),
            BlendError::code("missingEdge")
        );
        assert_eq!(
            fillet(&b, &one, 30.0, "F3", 1e-6).unwrap_err(),
            BlendError::detail("blendOverflow", "F3.B0: spring leaves its face away from its edges")
        );
    }

    #[test]
    fn concave_fillet_adds_material() {
        // L-shape: box with a step cut → concave edge at the step
        let a = the_box();
        let tool = box_body("F4", 0.0, 0.0, 20.0, 20.0, 5.0, 11.0);
        let l = crate::boolean::boolean(&a, &tool, Op::Cut, 1e-6).unwrap().body;
        let v0 = mass_props(&l).volume;
        // concave edge between the step floor (y=5) and the step wall (x=20)
        let floor = l.face_ids().find(|&f| l.face_id_str(f).starts_with("F4.C0")).unwrap();
        let wall = l.face_ids().find(|&f| l.face_id_str(f) == "F4.S.e2").unwrap();
        let (fa, fb) = (l.face_id_str(floor), l.face_id_str(wall));
        let r = fillet(&l, &[(fa, fb, Vec3::new(20., 5., -10.))], 1.0, "F5", 1e-6).unwrap();
        let v = mass_props(&r).volume;
        let add = (1.0 - std::f64::consts::PI / 4.0) * 20.0;
        assert!((v - v0 - add).abs() < 1e-9 * v, "{v} {v0}");
    }
}
