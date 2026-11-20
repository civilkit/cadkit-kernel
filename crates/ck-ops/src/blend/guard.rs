//! The joint guard: a post-condition gate on a finished blend (fillet or chamfer) that judges the PROPERTIES a correct
//! result has, so that a defect at a joint of rounded edges (a tool that ran on past a corner and left a fin, a corner
//! patch that crosses another face) is refused with a precise reason instead of being returned as a valid-looking solid.
//! It runs after the structural and volume checks of [`super::validity`], on every attempt of the selection driver (so a
//! later attempt still gets its chance) and on the results of the builders that do not go through it.
//!
//! Only the faces the blend made are examined (the faces of the result whose names the input does not have: blend
//! faces `B<k>`, corner patches, the pieces of the tools' own faces), so the work is bounded by the size of the blend:
//! - extent: every point of a blend face lies within reach of the blended edges. Beside an edge that is the section's
//!   width (`r tan(a / 2)` for faces turning by `a`, at least `r`); past an end vertex, where rounds meet in a mitre, the
//!   run-on `r / tan(t / 2)` for the smallest angle `t` between the edge and the other edges there (`3 r` more at a corner
//!   of three selected edges or of both senses, which carries a corner patch). Both with 30 % slack.
//! - crossing: no triangle of a new face crosses a triangle of another face away from the edges and vertices the two
//!   faces share, by more than the tessellation's chord on both sides of the other face's surface;
//! - containment: a selection of convex edges only removes material, so every point of a new face lies in the input
//!   solid or on its boundary (within `0.02 r`, `0.15 r` for a fitted free-form face; both classifiers agreeing); one of concave edges only adds material, so none lies strictly inside the input;
//! - collateral: no edge away from the blend (on no new face, more than `3 r` outside the new faces' box) lies off its faces
//!   where the input's edge with the same ends did not (a whole-body clean-up that reached past the blend);
//! - slivers: no new face narrower (twice its area over its perimeter) than `2e-4 r` (and `50 tol`).
//!
//! `CK_BLEND_GUARD=0` switches the guard off (native runs only); `CK_BLEND_GUARD_DEBUG=1` prints the measurements.

use super::BlendError;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role, VertexId};

fn enabled() -> bool {
    std::env::var_os("CK_BLEND_GUARD").map_or(true, |v| v != "0")
}

fn debug() -> bool {
    std::env::var_os("CK_BLEND_GUARD_DEBUG").is_some()
}

/// Slack on the extent bounds.
const SLACK: f64 = 1.3;

/// A refusal of the guard: code `blendResultInvalid`, the failing check named in the detail and in `params.check`.
fn refuse(check: &str, detail: String) -> BlendError {
    super::diag::dbg_fail(&format!("guard {check}: {detail}"));
    BlendError {
        code: "blendResultInvalid".into(),
        detail: Some(format!("the blend was built but its result fails the {check} check: {detail}")),
        params: vec![
            ("check".into(), check.into()),
            (
                "hint".into(),
                "the result was not used. Try a smaller size, blend the edges that meet at this corner in separate features, round the sketch corner before the cut, or blend before the feature that made the corner".into(),
            ),
        ],
    }
}

/// The face name without its region suffix, as text.
fn base_id(b: &Body, f: FaceId) -> String {
    let mut n: FaceName = b.face(f).name.clone();
    n.region = None;
    n.id()
}

/// The unit tangent of `e` leaving vertex `v` along the edge.
fn away(b: &Body, e: EdgeId, v: VertexId) -> Option<Vec3> {
    let ed = b.edge(e);
    let fwd = if ed.t1 >= ed.t0 { 1.0 } else { -1.0 };
    let t = if ed.v0 == v { ed.curve.tangent(ed.t0) * fwd } else { ed.curve.tangent(ed.t1) * -fwd };
    t.try_normalize()
}

/// One blended edge of the input, as the extent check reads it.
struct Reach {
    poly: Vec<Vec3>,
    /// width of the section beside the edge
    side: f64,
    /// bound at each end (`side` combined with the end's run-on)
    end: [f64; 2],
}

fn reaches(input: &Body, sel: &[EdgeId], r: f64) -> Vec<Reach> {
    sel.iter()
        .filter_map(|&e| {
            let ed = input.edge(e);
            let info = super::validity::edge_info(input, e)?;
            let a = info.alpha.min(150f64.to_radians());
            // curved faces: the contact can sit farther out than the planar section's
            let curved = input.edge_faces(e).iter().any(|&f| !matches!(input.face(f).surface, ck_geom::surface::Surface::Plane { .. }));
            let side = r * (0.5 * a).tan().max(1.0) * if curved { 1.5 } else { 1.0 };
            let end = |v: VertexId| -> f64 {
                let Some(d) = away(input, e, v) else { return side * 4.0 };
                // smallest angle to another edge at the vertex: the mitre's run-on past it
                let mut t = std::f64::consts::PI;
                for x in input.edge_ids().filter(|&x| x != e && (input.edge(x).v0 == v || input.edge(x).v1 == v)) {
                    if let Some(dx) = away(input, x, v) {
                        t = t.min(d.dot(dx).clamp(-1.0, 1.0).acos());
                    }
                }
                let mut run = r / (0.5 * t).tan().max(0.02);
                // a vertex where three selected edges meet, or edges of both senses (a mixed corner: the concave edge
                // is filled first and the rims roll round it on a torus of major radius 2 r), carries a corner patch
                // that reaches about 3 r farther from the vertex
                let at: Vec<EdgeId> = sel.iter().copied().filter(|&x| input.edge(x).v0 == v || input.edge(x).v1 == v).collect();
                let senses: Vec<bool> = at.iter().filter_map(|&x| super::validity::edge_info(input, x).map(|i| i.convex)).collect();
                if at.len() >= 3 || (senses.iter().any(|&c| c) && senses.iter().any(|&c| !c)) {
                    run += 3.0 * r;
                }
                (run * run + side * side).sqrt()
            };
            let n = if matches!(ed.curve, ck_geom::curve::Curve::Line { .. }) { 1 } else { 64 };
            let poly: Vec<Vec3> = (0..=n).map(|i| ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / n as f64)).collect();
            // the polyline runs from t0 to t1: its first point is v0 when t0 is the start
            Some(Reach { poly, side, end: [end(ed.v0), end(ed.v1)] })
        })
        .collect()
}

/// `(distance from q to the polyline, which end is nearest when the nearest point is an end: 0, 1, none)`.
fn nearest(poly: &[Vec3], q: Vec3) -> (f64, Option<usize>) {
    let mut best = (f64::INFINITY, None);
    for i in 0..poly.len() - 1 {
        let (a, c) = (poly[i], poly[i + 1]);
        let d = c - a;
        let dd = d.dot(d);
        let t = if dd > 0.0 { ((q - a).dot(d) / dd).clamp(0.0, 1.0) } else { 0.0 };
        let dist = (a + d * t - q).len();
        if dist < best.0 {
            let end = if i == 0 && t <= 0.0 {
                Some(0)
            } else if i == poly.len() - 2 && t >= 1.0 {
                Some(1)
            } else {
                None
            };
            best = (dist, end);
        }
    }
    best
}

struct Tri {
    p: [Vec3; 3],
    lo: Vec3,
    hi: Vec3,
}

fn tri(p: [Vec3; 3]) -> Tri {
    let lo = Vec3::new(p[0].x.min(p[1].x).min(p[2].x), p[0].y.min(p[1].y).min(p[2].y), p[0].z.min(p[1].z).min(p[2].z));
    let hi = Vec3::new(p[0].x.max(p[1].x).max(p[2].x), p[0].y.max(p[1].y).max(p[2].y), p[0].z.max(p[1].z).max(p[2].z));
    Tri { p, lo, hi }
}

/// Segment `p q` crosses the interior of triangle `t`: the crossing point.
fn seg_tri(p: Vec3, q: Vec3, t: &[Vec3; 3]) -> Option<Vec3> {
    const EPS: f64 = 1e-7;
    let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
    let d = q - p;
    let h = d.cross(e2);
    let a = e1.dot(h);
    if a.abs() < 1e-14 * (e1.len() * e2.len() * d.len()).max(1e-300) {
        return None;
    }
    let f = 1.0 / a;
    let s = p - t[0];
    let u = f * s.dot(h);
    if u < EPS || u > 1.0 - EPS {
        return None;
    }
    let qv = s.cross(e1);
    let v = f * d.dot(qv);
    if v < EPS || u + v > 1.0 - EPS {
        return None;
    }
    let w = f * e2.dot(qv);
    (w > EPS && w < 1.0 - EPS).then(|| p + d * w)
}

/// Signed distance of `p` from the surface of face `f` (positive on the outward side).
fn signed(b: &Body, f: FaceId, p: Vec3) -> f64 {
    let (u, v, q) = b.face(f).surface.closest_point(p);
    (p - q).dot(b.outward_normal(f, u, v).normalize())
}

/// The validator's limit on an edge's distance from its faces (`ck_check`'s `EDGE_OFF_SURFACE`).
fn off_surface(b: &Body, e: EdgeId, tol: f64) -> Option<f64> {
    let ed = b.edge(e);
    let scale = 1.0 + ed.curve.eval(ed.t0).len();
    let lim = (tol * scale * 10.0).max(1e-6 * scale).max(ed.tol);
    let dev = ck_check::face_deviation(b, e, 8);
    (dev > lim).then_some(dev)
}

/// Collateral: an edge of the result away from the blend (not on a new face, its samples more than `3 r` outside the
/// new faces' box) that lies off its faces, where the input's edge with the same end points did not (or there is no such
/// edge: its ends moved). A blend changes only the geometry next to the edges it rounds; a clean-up of the whole result
/// that reaches farther (a sliver collapse that took a short model edge 20 mm away) leaves a body that `validate` rejects.
fn collateral(input: &Body, out: &Body, new: &[FaceId], r: f64, tol: f64) -> Result<(), BlendError> {
    let (lo, hi) = out.bbox_of_faces(new);
    let pad = 3.0 * r + 1e-3;
    let (lo, hi) = (lo - Vec3::new(pad, pad, pad), hi + Vec3::new(pad, pad, pad));
    let inside = |p: Vec3| p.x >= lo.x && p.y >= lo.y && p.z >= lo.z && p.x <= hi.x && p.y <= hi.y && p.z <= hi.z;
    let key = |p: Vec3| ((p.x * 1e6).round() as i64, (p.y * 1e6).round() as i64, (p.z * 1e6).round() as i64);
    let pair = |b: &Body, e: EdgeId| {
        let (a, c) = (key(b.vertex(b.edge(e).v0).p), key(b.vertex(b.edge(e).v1).p));
        if a <= c { (a, c) } else { (c, a) }
    };
    let mut before: Option<std::collections::HashMap<_, Vec<EdgeId>>> = None;
    for e in out.edge_ids() {
        let fs = out.edge_faces(e);
        if fs.len() != 2 || fs.iter().any(|f| new.contains(f)) {
            continue;
        }
        let ed = out.edge(e);
        if (0..=8).any(|k| inside(ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0))) {
            continue;
        }
        // (a cheap pass first: most far edges are untouched and on their faces)
        let Some(dev) = off_surface(out, e, tol) else { continue };
        let map = before.get_or_insert_with(|| {
            let mut m: std::collections::HashMap<_, Vec<EdgeId>> = std::collections::HashMap::new();
            for x in input.edge_ids() {
                m.entry(pair(input, x)).or_default().push(x);
            }
            m
        });
        let was_off = map.get(&pair(out, e)).is_some_and(|xs| xs.iter().any(|&x| off_surface(input, x, tol).is_some()));
        if !was_off {
            let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
            let names: Vec<String> = fs.iter().map(|&f| out.face_id_str(f)).collect();
            return Err(refuse(
                "collateral",
                format!("the edge between {} near ({:.4}, {:.4}, {:.4}), away from the blended edges, lies {dev:.2e} mm off its faces (the blend changed geometry it does not touch)", names.join(" and "), p.x, p.y, p.z),
            ));
        }
    }
    Ok(())
}

/// Judge `out`, the blend of `picks` of `input` with size `r` (the largest radius or distance of the selection).
/// `convex` / `concave`: the selection is of convex (resp. concave) edges only.
pub(super) fn check(input: &Body, out: &Body, picks: &[(String, String, Vec3)], r: f64, convex: bool, concave: bool, tol: f64) -> Result<(), BlendError> {
    if !enabled() || !(r > 0.0) || !r.is_finite() {
        return Ok(());
    }
    let sel = super::validity::affected(input, picks);
    if sel.is_empty() {
        return Ok(());
    }
    let old: std::collections::HashSet<String> = input.face_ids().map(|f| base_id(input, f)).collect();
    let new: Vec<FaceId> = out.face_ids().filter(|&f| !old.contains(&base_id(out, f))).collect();
    if new.is_empty() {
        return Ok(());
    }
    collateral(input, out, &new, r, tol)?;
    // tessellate the new faces and the faces whose boxes meet theirs
    let ttol = ck_tess::mesh::body_tol(out);
    let samples = ck_tess::mesh::sample_edges(out, &ttol);
    let chord = ttol.chord.max(tol);
    let size = {
        let (lo, hi) = out.bbox();
        (hi - lo).len()
    };
    let pad = 2.0 * chord + 10.0 * tol;
    let boxes: Vec<(FaceId, Vec3, Vec3)> = out.face_ids().map(|f| {
        let (lo, hi) = out.bbox_of_faces(&[f]);
        (f, lo - Vec3::new(pad, pad, pad), hi + Vec3::new(pad, pad, pad))
    }).collect();
    let bx = |f: FaceId| boxes.iter().find(|x| x.0 == f).map(|x| (x.1, x.2)).unwrap();
    let meets = |a: (Vec3, Vec3), c: (Vec3, Vec3)| !(a.1.x < c.0.x || c.1.x < a.0.x || a.1.y < c.0.y || c.1.y < a.0.y || a.1.z < c.0.z || c.1.z < a.0.z);
    let mut need: Vec<FaceId> = new.clone();
    for &(g, lo, hi) in &boxes {
        if !need.contains(&g) && new.iter().any(|&f| meets(bx(f), (lo, hi))) {
            need.push(g);
        }
    }
    let mut mesh: std::collections::HashMap<usize, Vec<Tri>> = std::collections::HashMap::new();
    let mut verts: std::collections::HashMap<usize, Vec<Vec3>> = std::collections::HashMap::new();
    for &f in &need {
        // (a face the tessellator cannot mesh is judged by the other checks only)
        let Ok(m) = ck_tess::face::mesh_face(out, f, &samples.params, &samples.virt, &samples.ssi, samples.cuts[f.idx()], &ttol) else { continue };
        let tris: Vec<Tri> = m.tris.iter().map(|t| tri([m.pts[t[0] as usize], m.pts[t[1] as usize], m.pts[t[2] as usize]])).collect();
        mesh.insert(f.idx(), tris);
        verts.insert(f.idx(), m.pts.clone());
    }
    // ---- extent
    let reach = reaches(input, &sel, r);
    let ptol = 10.0 * tol + 1e-9 * size;
    for &f in &new {
        if !matches!(out.face(f).name.role, Role::Blend(_)) {
            continue;
        }
        let Some(ps) = verts.get(&f.idx()) else { continue };
        for &q in ps {
            let mut ok = false;
            let mut worst = (f64::INFINITY, 0.0);
            for rc in &reach {
                let (d, end) = nearest(&rc.poly, q);
                let lim = SLACK * end.map_or(rc.side, |i| rc.end[i]) + ptol;
                if d <= lim {
                    ok = true;
                    break;
                }
                if d - lim < worst.0 {
                    worst = (d - lim, d);
                }
            }
            if !ok {
                return Err(refuse(
                    "extent",
                    format!("{} reaches ({:.4}, {:.4}, {:.4}), {:.4} mm from the blended edges, more than the blend of size {r} can (it runs on past an edge end, e.g. a fin past a corner)", out.face_id_str(f), q.x, q.y, q.z, worst.1),
                ));
            }
        }
    }
    // ---- containment
    if convex || concave {
        let ctol = (100.0 * tol).max(1e-9 * size);
        let reach_tol = (0.02 * r).max(100.0 * tol);
        let solid = crate::boolean::classify::Solid::new(input, ctol);
        // a second, independent classifier (ray parity over a face BVH): a point is judged misplaced only when both agree
        let cont = crate::contain::Containment::new(input, ctol);
        for &f in &new {
            let Some(ps) = verts.get(&f.idx()) else { continue };
            for &q in ps {
                let c = solid.classify(q, Vec3::Z);
                use crate::boolean::classify::Class;
                let bad = if convex {
                    c == Class::Out && cont.classify_point(q, ctol) == crate::contain::Loc::Out
                } else {
                    c == Class::In && cont.classify_point(q, ctol) == crate::contain::Loc::In
                };
                // (and farther from the input's boundary than a fitted blend surface can stray: a free-form round is an
                // approximation, and its contact lines lie on edges of the input)
                let off = if bad { cont.nearest(q).map_or(f64::INFINITY, |(d, _, _)| d) } else { 0.0 };
                // (a free-form round fitted to a twisted or swept face strays farther: up to 0.1 r outside the solid on
                // a 45 to 60 degree twisted extrude, which the twist tests accept; analytic rounds keep the tight bound)
                let fitted = matches!(out.face(f).surface, ck_geom::surface::Surface::BSpline(_));
                let bad = bad && off > if fitted { 0.15 * r } else { reach_tol };
                if bad {
                    let what = if convex { "outside the solid it was cut from (a round of convex edges only removes material)" } else { "inside the solid it was added to (a round of concave edges only adds material)" };
                    return Err(refuse("containment", format!("{} has the point ({:.4}, {:.4}, {:.4}) {what}, {off:.4} mm from its boundary", out.face_id_str(f), q.x, q.y, q.z)));
                }
            }
        }
    }
    // ---- crossing
    let ign = (4.0 * chord).max(0.05 * r).max(100.0 * tol);
    let depth = 2.0 * chord + 10.0 * tol;
    let poly_of = |e: EdgeId| -> Vec<Vec3> {
        let ed = out.edge(e);
        let ps = &samples.params[e.idx()];
        if ps.len() >= 2 { ps.iter().map(|&t| ed.curve.eval(t)).collect() } else { vec![out.vertex(ed.v0).p, out.vertex(ed.v1).p] }
    };
    for (i, &f) in new.iter().enumerate() {
        let Some(tf) = mesh.get(&f.idx()) else { continue };
        let ef: Vec<EdgeId> = out.face_edges(f);
        let vf: Vec<VertexId> = ef.iter().flat_map(|&e| [out.edge(e).v0, out.edge(e).v1]).collect();
        for &g in &need {
            if g == f || new[..i].contains(&g) || !meets(bx(f), bx(g)) {
                continue;
            }
            let Some(tg) = mesh.get(&g.idx()) else { continue };
            let eg: Vec<EdgeId> = out.face_edges(g);
            let shared_e: Vec<Vec<Vec3>> = ef.iter().filter(|e| eg.contains(e)).map(|&e| poly_of(e)).collect();
            let shared_v: Vec<Vec3> = eg.iter().flat_map(|&e| [out.edge(e).v0, out.edge(e).v1]).filter(|v| vf.contains(v)).map(|v| out.vertex(v).p).collect();
            let near_shared = |x: Vec3| shared_v.iter().any(|p| p.dist(x) < ign) || shared_e.iter().any(|pl| pl.len() >= 2 && nearest(pl, x).0 < ign);
            for a in tf {
                for c in tg {
                    if a.hi.x < c.lo.x || c.hi.x < a.lo.x || a.hi.y < c.lo.y || c.hi.y < a.lo.y || a.hi.z < c.lo.z || c.hi.z < a.lo.z {
                        continue;
                    }
                    let hit = (0..3).find_map(|k| seg_tri(a.p[k], a.p[(k + 1) % 3], &c.p)).or_else(|| (0..3).find_map(|k| seg_tri(c.p[k], c.p[(k + 1) % 3], &a.p)));
                    let Some(x) = hit else { continue };
                    if near_shared(x) {
                        continue;
                    }
                    // a real crossing: each triangle reaches past the other face's surface on both sides by more than the chord
                    let straddles = |t: &Tri, h: FaceId| {
                        let s: Vec<f64> = t.p.iter().map(|&p| signed(out, h, p)).collect();
                        s.iter().cloned().fold(f64::NEG_INFINITY, f64::max) > depth && s.iter().cloned().fold(f64::INFINITY, f64::min) < -depth
                    };
                    if straddles(a, g) && straddles(c, f) {
                        return Err(refuse("crossing", format!("{} crosses {} near ({:.4}, {:.4}, {:.4})", out.face_id_str(f), out.face_id_str(g), x.x, x.y, x.z)));
                    }
                }
            }
        }
    }
    // ---- slivers
    for &f in &new {
        let Some(tf) = mesh.get(&f.idx()) else { continue };
        let area: f64 = tf.iter().map(|t| 0.5 * (t.p[1] - t.p[0]).cross(t.p[2] - t.p[0]).len()).sum();
        let per: f64 = out.face_edges(f).iter().map(|&e| out.edge_length(e)).sum();
        if !(per > 0.0) {
            continue;
        }
        let width = 2.0 * area / per;
        if debug() {
            eprintln!("guard: {} area {area:.3e} perimeter {per:.3e} width {width:.3e}", out.face_id_str(f));
        }
        if width < (2e-4 * r).max(50.0 * tol) {
            return Err(refuse("sliver", format!("{} is a sliver {width:.2e} mm wide (area {area:.2e} mm2) left where the blend meets the faces beside it", out.face_id_str(f))));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::boolean::tests::box_body;
    use crate::surfacing::knit::BodyExt;
    use ck_geom::curve::Curve;
    use ck_geom::Vec3;

    /// A round of one box edge, then a far edge (the opposite bottom edge, 18 mm away) moved 7.5e-4 off its faces with its
    /// pcurves dropped, as the old whole-body sliver collapse left the fan shroud's grille: the collateral check refuses it,
    /// the untouched round passes.
    #[test]
    fn collateral_damage_far_from_the_blend_is_refused() {
        let input = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let picks = vec![("F2.C1".to_string(), "F2.S.e1".to_string(), Vec3::new(20.0, 10.0, 0.0))];
        let out = super::super::build::fillet(&input, &picks, 2.0, "F3", 1e-6).unwrap();
        assert!(super::check(&input, &out, &picks, 2.0, true, false, 1e-6).is_ok());
        let mut bad = out.clone();
        let e = bad
            .edge_ids()
            .find(|&e| {
                let n: Vec<String> = bad.edge_faces(e).iter().map(|&f| bad.face_id_str(f)).collect();
                n.contains(&"F2.C0".to_string()) && n.contains(&"F2.S.e3".to_string())
            })
            .expect("far edge");
        let Curve::Line { o, d } = bad.edge(e).curve.clone() else { panic!("not a line") };
        bad.edge_mut(e).curve = Curve::Line { o: o + Vec3::new(0.0, 7.5e-4, 0.0), d };
        for c in bad.edge(e).coedges.clone() {
            bad.coedge_mut(c).pcurve = None;
        }
        let err = super::check(&input, &bad, &picks, 2.0, true, false, 1e-6).unwrap_err();
        assert_eq!(err.code, "blendResultInvalid");
        assert!(err.params.iter().any(|(k, v)| k == "check" && v == "collateral"), "{:?}", err.detail);
    }
}
