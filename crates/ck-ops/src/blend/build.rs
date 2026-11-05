//! Public API of the blend builders (ck-ops/src/blend/build.rs): `fillet`, `chamfer`, `chamfer_legs`,
//! `fillet_unequal`, `chamfer_legs_on_face` and re-exports of the pieces in the sibling files (see
//! docs/ck-ops-stubs.md for the layout). A blend is built as a boolean with a swept tool (a "corner prism" whose
//! arc face becomes the blend face `B<k>`) along each selected edge.
//!
//! The notes below are the first oracle probes (a straight edge between two planar faces).
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

use super::rounds::{CHAMFER_FIRST, EDGE_RADII, TANGENT_FLUSH};
use super::BlendError;
use ck_geom::Vec3;
use ck_topo::body::{Body, FaceName};
pub(super) use super::apply::{apply_tool, apply_tool_hooked};
pub(super) use super::diag::last_fail;
pub(super) use super::ends::clip_halfspace;
pub use super::find::{find_edge, find_edge_by_position, forward_first};
pub use super::layout::{in_place_layout, in_place_layout_opts, BlendRef, LayoutOpts};
pub use super::analytic::fillet_line_edge;
pub use super::model::{ChamferLeg, FilletProfile, Spec};
pub use super::rounds::blend_edges;

/// Constant-radius fillets of `edges` (face-name pairs + pick point, in
/// ref order), blend faces `B0, B1, …` of `feature`.
///
/// Each pair is first normalised so that the face whose loop runs the edge
/// FORWARD comes first (oracle box probes, bracket F12: selection
/// `["F2.S.e8","F2.S.e9"]` lists `F12.B0|F2.S.e9` before `…|F2.S.e8`, and
/// the touched faces come e9, e8). The result is written in the original's
/// IN-PLACE arena layout ([`in_place_layout`]), so wire `#n` keep the
/// input's numbers.
pub fn fillet(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    blend_edges(b, edges, Spec::Fillet(r, FilletProfile::Circular), feature, tol)
}

/// Equal-distance chamfers (`Chamfer(d, d)` in the trace) of `edges`.
pub fn chamfer(b: &Body, edges: &[(String, String, Vec3)], d: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    blend_edges(b, edges, Spec::Chamfer(d, ChamferLeg::Dist(d)), feature, tol)
}

/// Chamfer with distance `d1` on the FIRST face of every edge pick and a second leg given by a
/// distance or by the angle between the chamfer and the first face (oracle `distances` / `angle`).
pub fn chamfer_legs(b: &Body, edges: &[(String, String, Vec3)], d1: f64, leg: ChamferLeg, feature: &str, tol: f64) -> Result<Body, BlendError> {
    blend_edges(b, edges, Spec::Chamfer(d1, leg), feature, tol)
}

/// Circular fillets with a radius per edge (`radii[k]` for `edges[k]`). Every edge is built as the
/// ordinary constant-radius blend (analytic cylinders, the same rounds and junction rules); where
/// edges of different radii meet, the blend faces end in a mitre, which is not tangent-continuous.
/// Three convex or concave edges that share a radius at their vertex still get the sphere patch.
pub fn fillet_unequal(b: &Body, edges: &[(String, String, Vec3)], radii: &[f64], feature: &str, tol: f64) -> Result<Body, BlendError> {
    if radii.len() != edges.len() || !radii.iter().all(|r| *r > 0.0 && r.is_finite()) {
        return Err(BlendError::code("invalidValue"));
    }
    let rmax = radii.iter().cloned().fold(0.0, f64::max);
    EDGE_RADII.with(|c| *c.borrow_mut() = Some(radii.to_vec()));
    let r = blend_edges(b, edges, Spec::Fillet(rmax, FilletProfile::Circular), feature, tol);
    EDGE_RADII.with(|c| *c.borrow_mut() = None);
    r
}

/// [`chamfer_legs`] with the first leg (`d1`, or the distance of the angle form) on the face named
/// `first` for every edge, whichever side of the pick it is on; `flip` takes the other face of each
/// edge instead. `first` is a face id, `~k` region suffix ignored.
pub fn chamfer_legs_on_face(b: &Body, edges: &[(String, String, Vec3)], d1: f64, leg: ChamferLeg, first: &str, flip: bool, feature: &str, tol: f64) -> Result<Body, BlendError> {
    let name = FaceName::parse(first).map(|mut n| {
        n.region = None;
        n
    });
    let Some(name) = name else { return Err(BlendError::code("missingEdge")) };
    CHAMFER_FIRST.with(|c| *c.borrow_mut() = Some((name, flip)));
    let r = blend_edges(b, edges, Spec::Chamfer(d1, leg), feature, tol);
    CHAMFER_FIRST.with(|c| *c.borrow_mut() = None);
    r
}

/// Sets [`TANGENT_FLUSH`] for its lifetime (a selection that is a chain of tangent edges).
pub struct TangentChainGuard(bool);

impl TangentChainGuard {
    pub fn new(on: bool) -> TangentChainGuard {
        let old = TANGENT_FLUSH.with(|c| c.replace(on || c.get()));
        TangentChainGuard(old)
    }
}

impl Drop for TangentChainGuard {
    fn drop(&mut self) {
        TANGENT_FLUSH.with(|c| c.set(self.0));
    }
}

/// [`blend_edges`] with the ends of tangent-continuing selected edges stopping flush (a selection that is
/// a chain of tangent edges: the rolling ball runs on from one edge into the next, so no tool may reach
/// past the junction).
pub fn blend_edges_tangent_chain(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Result<Body, BlendError> {
    TANGENT_FLUSH.with(|c| c.set(true));
    let r = blend_edges(b, edges, spec, feature, tol);
    TANGENT_FLUSH.with(|c| c.set(false));
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;
    use crate::boolean::Op;
    use ck_geom::surface::Surface;
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
    fn cylinder_top_circle() {
        // oracle volumes (tools/brep_cases.mjs filletCylTop / chamferCylTop)
        let cyl = crate::boolean::tests::cyl_body("F2", 5.0, 3.0, 10.0, 0.0, 10.0);
        let top = cyl.face_ids().find(|&f| cyl.face_id_str(f) == "F2.C1").unwrap();
        let side = cyl.face_ids().find(|&f| matches!(cyl.face(f).surface, Surface::Cylinder { .. })).unwrap();
        let (a, bb) = (cyl.face_id_str(top), cyl.face_id_str(side));
        let at = Vec3::new(15., 10., -3.);
        let r = fillet(&cyl, &[(a.clone(), bb.clone(), at)], 2.0, "F3", 1e-6).unwrap();
        assert!((mass_props(&r).volume - 3090.0668137679036).abs() < 1e-6);
        assert_eq!(r.face(r.find_face("F3.B0").unwrap()).surface.kind(), "torus");
        let r = chamfer(&cyl, &[(a, bb, at)], 1.0, "F3", 1e-6).unwrap();
        assert!((mass_props(&r).volume - 3111.223924605093).abs() < 1e-6);
        assert_eq!(r.face(r.find_face("F3.B0").unwrap()).surface.kind(), "cone");
    }

    #[test]
    fn oracle_chamfers_on_the_box() {
        // chamferOne / chamferTwo / chamferRail / chamferBig (oracle)
        let b = the_box();
        let one = [e("F2.C1", "F2.S.e1", [20., 10., 0.])];
        let r = chamfer(&b, &one, 2.0, "F3", 1e-6).unwrap();
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e3", "F2.C1", "F2.S.e1", "F2.S.e4", "F2.S.e2", "F3.B0"]);
        assert!((mass_props(&r).volume - 7920.0).abs() < 1e-9);
        let two = [one[0].clone(), e("F2.C1", "F2.S.e3", [20., 10., -20.])];
        assert!((mass_props(&chamfer(&b, &two, 2.0, "F3", 1e-6).unwrap()).volume - 7840.0).abs() < 1e-9);
        assert_eq!(
            chamfer(&b, &one, 30.0, "F3", 1e-6).unwrap_err(),
            BlendError::detail("blendOverflow", "F3.B0: spring leaves its face away from its edges")
        );
        // a ring of four runs in rounds; blend names stay selection-indexed
        let ring = [
            e("F2.C1", "F2.S.e1", [20., 10., 0.]),
            e("F2.C1", "F2.S.e2", [40., 10., -10.]),
            e("F2.C1", "F2.S.e3", [20., 10., -20.]),
            e("F2.C1", "F2.S.e4", [0., 10., -10.]),
        ];
        let r = chamfer(&b, &ring, 2.0, "F3", 1e-6).unwrap();
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e1", "F2.S.e2", "F2.S.e3", "F3.B1", "F2.C1", "F2.S.e4", "F3.B2", "F3.B0", "F3.B3"]);
    }

    #[test]
    fn vertex_chamfer_on_the_box() {
        use crate::blend::vertex::chamfer_vertex;
        let b = the_box();
        // distances 2 / 3 / 4 along the vertex edges in arena order (rail, +x, -z)
        let r = chamfer_vertex(&b, Vec3::new(0., 10., 0.), [2., 3., 4.], "F3", 1e-6).unwrap();
        assert!((mass_props(&r).volume - (8000.0 - 2.0 * 3.0 * 4.0 / 6.0)).abs() < 1e-9);
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e2", "F2.S.e3", "F2.S.e1", "F2.C1", "F2.S.e4", "F3.B0"]);
        // oracle normal (-0.512, 0.768, 0.384)
        let n = r.planar_normal(r.find_face("F3.B0").unwrap());
        assert!((n - Vec3::new(-4., 6., 3.).normalize()).len() < 1e-12, "{n:?}");
        assert_eq!(chamfer_vertex(&b, Vec3::new(0., 10., 0.), [10., 1., 1.], "F3", 1e-6).unwrap_err(), BlendError::code("blendTooLarge"));
        assert_eq!(chamfer_vertex(&b, Vec3::new(0., 10., 0.), [1., 0., 1.], "F3", 1e-6).unwrap_err(), BlendError::code("invalidValue"));
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
