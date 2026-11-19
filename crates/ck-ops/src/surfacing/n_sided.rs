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

/// The closed ring `chain` of `body` filled by N free-form faces round a centre vertex.
///
/// Each ring edge is split at its parameter midpoint (the neighbouring face's loop gets the two
/// halves), N inner seam edges run from those midpoints to a new centre vertex, and patch `k` is
/// one face bounded by the first half of ring side `k`, inner seam `k`, inner seam `k - 1` and the
/// second half of ring side `k - 1`. Seams are shared by two coedges of opposite sense, and the
/// ring coedges run opposite to the neighbouring faces'. The faces are named
/// `<feature>.N<k>` (`k` = the side's position in the walk of the ring, from 0) and appended in
/// that order, so the naming depends on nothing but the picks.
pub fn fill_n_sided_body(body: &Body, chain: &[EdgeId], continuity: &str, feature: &str, tol: f64) -> Result<Body, FillError> {
    use ck_geom::curve::Curve;
    use ck_geom::surface::Surface;
    use ck_topo::body::{FaceName, Role};

    let patch = fill_n_sided(body, chain, continuity, tol)?;
    let steps = walk_chain(body, chain)?;
    let n = steps.len();
    let fail = |d: &str| FillError { code: "fillFailed", detail: d.into() };
    if patch.patches.len() != n {
        return Err(fail("the patch count does not match the ring"));
    }
    // does the walk run with the neighbouring faces? (their coedges must run opposite to ours)
    let body_rev = |e: EdgeId| body.edge_coedges(e).iter().find(|&&c| body.coedge(c).alive).map(|&c| body.coedge(c).rev).unwrap_or(false);
    let agree = steps.iter().filter(|s| s.fwd == !body_rev(s.e)).count();
    let backwards = agree * 2 > n;

    let mut out = body.clone();
    let lump = out.lump_ids().next().ok_or_else(|| fail("the body has no lump"))?;
    let shell = out.add_shell(lump);

    // split every ring edge at its parameter midpoint; each half is (edge, stored against the walk)
    let mut mids = Vec::with_capacity(n);
    let mut first = Vec::with_capacity(n);
    let mut second = Vec::with_capacity(n);
    for (k, s) in steps.iter().enumerate() {
        let (t0, t1) = (out.edge(s.e).t0, out.edge(s.e).t1);
        let (vm, ne) = ck_topo::edit::split_edge(&mut out, s.e, 0.5 * (t0 + t1));
        if out.vertex(vm).p.dist(patch.patches[k].eval(1.0, 0.0)) > tol.max(1e-6) * 10.0 {
            return Err(fail("a ring edge's parameter midpoint is not the patch's side midpoint"));
        }
        mids.push(vm);
        if s.fwd {
            first.push((s.e, false));
            second.push((ne, false));
        } else {
            first.push((ne, true));
            second.push((s.e, true));
        }
    }
    // the centre and the inner seams (middle of side k -> centre)
    let vc = out.add_vertex(patch.centre);
    let mut seams = Vec::with_capacity(n);
    for (k, p) in patch.patches.iter().enumerate() {
        let c = p.iso_u(p.domain().0 .1);
        let (a, b) = c.domain();
        seams.push(out.add_edge(Curve::BSpline(c), a, b, mids[k], vc));
    }
    for (k, p) in patch.patches.iter().enumerate() {
        let name = FaceName { feature: feature.into(), role: Role::Other(format!("N{k}")), region: None, suffix: String::new() };
        let f = out.add_face(shell, Surface::BSpline(Box::new(p.clone())), !backwards, name);
        let lp = out.add_loop(f);
        let prev = (k + n - 1) % n;
        // counter-clockwise in (u, v): along side k, down seam k, back up seam k - 1, along side k - 1
        let mut co: Vec<(EdgeId, bool)> = vec![first[k], (seams[k], false), (seams[prev], true), second[prev]];
        if backwards {
            co.reverse();
            for c in co.iter_mut() {
                c.1 = !c.1;
            }
        }
        for (e, rev) in co {
            out.add_coedge(lp, e, rev, None);
        }
    }
    Ok(out)
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
