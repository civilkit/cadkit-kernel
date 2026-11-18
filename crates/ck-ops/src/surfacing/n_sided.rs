//! N-sided fill of a closed boundary ring as N four-sided patches, G0 or G1 to the neighbours.
//!
//! This is separate from [`super::fill::fill`] on purpose: `fill` is pinned by oracle probes to
//! append ONE planar (or ruled G0) face, whose closing chord and face naming are fixed, and a fill
//! made of N free-form patches round a centre does not fit that single face. This function returns
//! the patch surfaces; building faces and the seam edges from them is left to the caller (the
//! feature layer), so none of the pinned `fill` behaviour or goldens is touched.
//!
//! The ring must be closed (no chord); an open chain is `fillNotClosed`. With continuity `G1` each
//! boundary edge that has a neighbouring face leaves tangent to that face (the cross tangent is
//! `edge tangent x face normal`, taken away from the face, assuming the face loops run so the face
//! interior lies to the left of the face normal, as the rest of the kernel keeps them); an edge with
//! no face gets only G0. `G2`/`G3` are refused (`fillContinuity`): the patches are G1.

use crate::surfacing::fill::{walk_chain, FillError, Step};
use ck_geom::convert::curve_to_nurbs;
pub use ck_geom::nsided::NSidedPatch;
use ck_geom::nsided::{cross_from_normals, n_sided_patch, NSidedError, NSidedSide};
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId};

fn map_err(e: NSidedError) -> FillError {
    match e {
        NSidedError::Input(s) => FillError { code: "fillInvalid", detail: s },
        NSidedError::Geometry(s) => FillError { code: "fillFailed", detail: s },
        t @ NSidedError::Tolerance { .. } => FillError { code: "fillFailed", detail: t.to_string() },
    }
}

/// The patches filling the closed ring `chain` of `body` (module docs). `tol` bounds the distance of
/// the patches' sides from the edges.
pub fn fill_n_sided(body: &Body, chain: &[EdgeId], continuity: &str, tol: f64) -> Result<NSidedPatch, FillError> {
    let g1 = match continuity {
        "G0" => false,
        "G1" => true,
        "G2" | "G3" => return Err(FillError { code: "fillContinuity", detail: "the n-sided fill is G0 or G1".into() }),
        _ => return Err(FillError { code: "fillInvalid", detail: String::new() }),
    };
    if !(tol.is_finite() && tol > 0.0) {
        return Err(FillError { code: "fillInvalid", detail: "the tolerance must be finite and positive".into() });
    }
    if chain.is_empty() {
        return Err(FillError { code: "fillInvalid", detail: "the edges do not form one chain".into() });
    }
    let steps = walk_chain(body, chain)?;
    let n = steps.len();
    let vpos = |e: EdgeId, start: bool| {
        let ed = body.edge(e);
        body.vertex(if start { ed.v0 } else { ed.v1 }).p
    };
    let walk_start = |s: &Step| vpos(s.e, s.fwd);
    let walk_end = |s: &Step| vpos(s.e, !s.fwd);
    if n == 0 || walk_start(&steps[0]).dist(walk_end(&steps[n - 1])) > tol.max(1e-6) {
        return Err(FillError { code: "fillNotClosed", detail: "the chain is open".into() });
    }
    if n < 3 {
        return Err(FillError { code: "fillInvalid", detail: "an n-sided fill needs a ring of at least 3 edges".into() });
    }
    let centroid = steps.iter().map(walk_start).fold(Vec3::ZERO, |a, p| a + p) * (1.0 / n as f64);
    let scale = 0.5 * steps.iter().map(|s| walk_start(s).dist(centroid)).sum::<f64>() / n as f64;

    let mut sides = Vec::with_capacity(n);
    for s in &steps {
        let ed = body.edge(s.e);
        let c = curve_to_nurbs(&ed.curve, ed.t0, ed.t1);
        let c = if s.fwd { c } else { c.reversed() };
        let mut side = NSidedSide::new(c.clone());
        if g1 {
            if let Some(&co) = body.edge_coedges(s.e).first() {
                let face = body.face(body.coedge_face(co));
                // the walk runs with the face's coedge when both go v0 -> v1 or both v1 -> v0
                let same = s.fwd == !body.coedge(co).rev;
                let sig = if same { 1.0 } else { -1.0 };
                let fs = if face.sense { 1.0 } else { -1.0 };
                let nf = |t: f64| {
                    let p = c.eval(t);
                    let (u, v) = face.surface.closest_param(p);
                    face.surface.normal(u, v) * (-sig * fs)
                };
                side.cross = Some(cross_from_normals(&c, &nf, scale.max(1e-9), None).map_err(map_err)?);
            }
        }
        sides.push(side);
    }
    n_sided_patch(&sides, tol).map_err(map_err)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::curve::Curve;
    use ck_geom::surface::Surface;
    use ck_topo::body::{FaceName, Role};

    /// The rim ring of a square tube (z = 10) and its four outer wall faces, whose loops keep the
    /// wall interior to the left of the outward normal.
    fn tube_rim() -> (Body, Vec<EdgeId>) {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let ring = [(5.0, -5.0), (5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0)];
        let vs: Vec<_> = ring.iter().map(|&(x, y)| b.add_vertex(Vec3::new(x, y, 10.0))).collect();
        let mut edges = vec![];
        for i in 0..4 {
            let (p, q) = (b.vertex(vs[i]).p, b.vertex(vs[(i + 1) % 4]).p);
            let e = b.add_edge(Curve::line_through(p, q), 0.0, (q - p).len(), vs[i], vs[(i + 1) % 4]);
            edges.push(e);
            let mid = (p + q) * 0.5;
            let out = Vec3::new(mid.x, mid.y, 0.0).normalize();
            let name = FaceName { feature: "F1".into(), role: Role::Other(format!("W{i}")), region: None, suffix: String::new() };
            let f = b.add_face(shell, Surface::plane(mid, out), true, name);
            let lp = b.add_loop(f);
            b.add_coedge(lp, e, true, None);
        }
        (b, edges)
    }

    #[test]
    fn ring_fill_g0_and_g1() {
        let (body, chain) = tube_rim();
        let g0 = fill_n_sided(&body, &chain, "G0", 1e-6).unwrap();
        assert_eq!(g0.patches.len(), 4);
        assert!(g0.boundary_dev <= 1e-6);
        // a flat square at the rim
        for s in &g0.patches {
            assert!((s.eval(0.5, 0.5).z - 10.0).abs() < 1e-6);
        }
        let g1 = fill_n_sided(&body, &chain, "G1", 1e-6).unwrap();
        assert!(g1.boundary_dev <= 1e-6 && g1.seam_gap < 1e-9);
        // leaves the walls vertically (normal horizontal) and bulges away from the tube
        for s in &g1.patches {
            let nb = s.normal(0.5, 0.0);
            assert!(nb.z.abs() < 0.2, "{nb:?}");
            assert!(s.eval(1.0, 1.0).z > 10.5);
        }
        assert!(g1.seam_angle < 0.05);
    }

    #[test]
    fn ring_fill_errors() {
        let (body, chain) = tube_rim();
        assert_eq!(fill_n_sided(&body, &chain[..3], "G0", 1e-6).unwrap_err().code, "fillNotClosed");
        assert_eq!(fill_n_sided(&body, &chain, "G2", 1e-6).unwrap_err().code, "fillContinuity");
        assert_eq!(fill_n_sided(&body, &chain, "G9", 1e-6).unwrap_err().code, "fillInvalid");
        assert_eq!(fill_n_sided(&body, &chain, "G0", f64::NAN).unwrap_err().code, "fillInvalid");
        assert!(fill_n_sided(&body, &[], "G0", 1e-6).is_err());
        assert!(fill_n_sided(&body, &[chain[0], chain[2]], "G0", 1e-6).is_err());
    }
}
