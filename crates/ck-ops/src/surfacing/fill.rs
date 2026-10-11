//! Fill: patch a surface across a chain of boundary edges.
//!
//! Semantics pinned by oracle probes (U-shaped sheet: bottom + 3 walls of a
//! 10-box, fill the top rims):
//! - The patch face `F6.F` (role `F`, meta `{feature, surface:true}`) is
//!   appended LAST to the body; its boundary = the chain edges (shared,
//!   traversed OPPOSITE to the body faces' coedges) + the closing chord —
//!   the straight segment from the chain's start to its end, stored
//!   start→end and traversed reversed in the loop (probe f2a: chain
//!   (0,0,10)→(10,0,10)→(10,10,10), chord edge `F6.F#11`
//!   (0,0,10)→(10,10,10); the patch is the triangle + chord).
//! - The patch surface is the plane when the closed boundary is planar
//!   (probe: patch kind `plane`); otherwise a ruled G0 surface (Hermite
//!   band for G1+ is not reconstructed here).
//! - A degenerate patch → `fillFailed` with the kernel's validity detail
//!   "the fill is not a valid face: face orientation: F0 (F6.F): parameter
//!   area 0.000e0" (probe: the same edge twice in the chain).
//! - Edges that don't connect head-to-tail → `fillInvalid` ("the edges do
//!   not form one chain"); a single open edge → `fillNotClosed` ("the
//!   chain is open", GUESS — unprobed); continuity above G0 on a
//!   non-planar boundary → `fillContinuity` (GUESS).
//!
//! HOW TO WIRE (ck-model, Gate B): feature `fill {edges:[{faces,at}…],
//! merge, continuity, fairing}` → resolve the refs to a chain of boundary
//! edges of the current body (nearest-edge-to-`at` per face; two-face refs
//! pick the shared edge), then `fill::fill(&body, &chain, continuity,
//! &feat_id, 1e-6)`. `merge:false` produces the patch as its own surface
//! body instead of appending to the input body.

use ck_geom::curve::Curve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceName, Role};

#[derive(Debug, Clone)]
pub struct FillError {
    pub code: &'static str,
    pub detail: String,
}

impl FillError {
    fn code(code: &'static str) -> FillError {
        FillError { code, detail: String::new() }
    }
    fn detail(code: &'static str, detail: impl Into<String>) -> FillError {
        FillError { code, detail: detail.into() }
    }
}

impl std::fmt::Display for FillError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.detail.is_empty() {
            write!(f, "{}", self.code)
        } else {
            write!(f, "{}: {}", self.code, self.detail)
        }
    }
}

/// A chain step: the edge and the direction of the walk (start vertex).
struct Step {
    e: EdgeId,
    /// the walk traverses the edge v0→v1
    fwd: bool,
}

/// Order the chain head-to-tail; the walk starts at chain[0]'s v0 unless
/// the second edge forces the flip.
fn walk_chain(b: &Body, chain: &[EdgeId]) -> Result<Vec<Step>, FillError> {
    if chain.is_empty() {
        return Err(FillError::detail("fillInvalid", "the edges do not form one chain"));
    }
    let mut steps: Vec<Step> = chain.iter().map(|&e| Step { e, fwd: true }).collect();
    if chain.len() >= 2 {
        let e1 = chain[1];
        let connects = |fwd: bool| {
            let tail = if fwd { b.edge(chain[0]).v1 } else { b.edge(chain[0]).v0 };
            b.edge(e1).v0 == tail || b.edge(e1).v1 == tail
        };
        if !connects(true) && connects(false) {
            steps[0].fwd = false;
        } else if !connects(true) && !connects(false) {
            return Err(FillError::detail("fillInvalid", "the edges do not form one chain"));
        }
    }
    for i in 1..steps.len() {
        let prev = &steps[i - 1];
        let tail = if prev.fwd { b.edge(prev.e).v1 } else { b.edge(prev.e).v0 };
        let e = b.edge(steps[i].e);
        if e.v0 == tail {
            steps[i].fwd = true;
        } else if e.v1 == tail {
            steps[i].fwd = false;
        } else {
            return Err(FillError::detail("fillInvalid", "the edges do not form one chain"));
        }
    }
    Ok(steps)
}

/// Fill a chain of boundary edges of `body` with a patch face.
pub fn fill(body: &Body, chain: &[EdgeId], continuity: &str, feature: &str, tol: f64) -> Result<Body, FillError> {
    let steps = walk_chain(body, chain)?;
    let closed_ring = steps.len() == 1 && {
        let e = body.edge(steps[0].e);
        body.vertex(e.v0).p.dist(body.vertex(e.v1).p) <= tol
    };
    if steps.len() == 1 && !closed_ring {
        return Err(FillError::detail("fillNotClosed", "the chain is open"));
    }
    match continuity {
        "G0" | "G1" | "G2" | "G3" => {}
        _ => return Err(FillError::code("fillInvalid")),
    }

    let first = &steps[0];
    let start = if first.fwd { body.edge(first.e).curve.eval(body.edge(first.e).t0) } else { body.edge(first.e).curve.eval(body.edge(first.e).t1) };
    let last = steps.last().unwrap();
    let end = if last.fwd { body.edge(last.e).curve.eval(body.edge(last.e).t1) } else { body.edge(last.e).curve.eval(body.edge(last.e).t0) };

    let mut out = body.clone();
    let lump = out.lump_ids().next().expect("body has a lump");
    let shell = out.add_shell(lump);
    let name = FaceName { feature: feature.into(), role: Role::Other("F".into()), region: None, suffix: String::new() };

    let (surface, area) = if closed_ring {
        planar_or_ruled(&sample_chain(body, &steps, 16), &[], tol)
    } else {
        planar_or_ruled(&sample_chain(body, &steps, 16), &[start, end], tol)
    };
    let nf = out.add_face(shell, surface, true, name);

    // loop: the chain edges traversed opposite the body faces' coedges
    let nloop = out.add_loop(nf);
    for s in &steps {
        let body_rev = body
            .edge_coedges(s.e)
            .iter()
            .find(|&&c| body.coedge(c).alive)
            .map(|&c| body.coedge(c).rev)
            .unwrap_or(false);
        out.add_coedge(nloop, s.e, !body_rev, None);
    }
    if !closed_ring {
        // the chord: stored start->end, traversed reversed (probe-pinned)
        let va = out.add_vertex(start);
        let vb = out.add_vertex(end);
        let ce = out.add_edge(Curve::line_through(start, end), 0.0, (end - start).len(), va, vb);
        out.add_coedge(nloop, ce, true, None);
    }

    // validity: the kernel's face check rejects a zero-parameter-area patch
    if area <= 1e-12 {
        let id = format!("{}.F", feature);
        return Err(FillError::detail(
            "fillFailed",
            format!("the fill is not a valid face: face orientation: F0 ({id}): parameter area {area:.3e}"),
        ));
    }
    Ok(out)
}

/// Sample the chain into a polyline (junction points + interior samples).
fn sample_chain(b: &Body, steps: &[Step], per_edge: usize) -> Vec<Vec3> {
    let mut pts = Vec::new();
    for s in steps {
        let e = b.edge(s.e);
        for k in 0..per_edge {
            let f = k as f64 / per_edge as f64;
            let t = if s.fwd { e.t0 + (e.t1 - e.t0) * f } else { e.t1 - (e.t1 - e.t0) * f };
            pts.push(e.curve.eval(t));
        }
    }
    let last = steps.last().unwrap();
    let e = b.edge(last.e);
    pts.push(e.curve.eval(if last.fwd { e.t1 } else { e.t0 }));
    pts
}

/// Plane patch when the closed boundary (chain + chord) is planar; else a
/// ruled deg-1×1 surface between the chain polyline and the chord. Returns
/// (surface, area).
fn planar_or_ruled(chain_pts: &[Vec3], chord: &[Vec3], tol: f64) -> (Surface, f64) {
    let mut pts: Vec<Vec3> = chain_pts.to_vec();
    if chord.len() == 2 && chord[0].dist(chord[1]) > tol {
        // the loop runs the chord end->start
        pts.push(chord[1]);
        pts.push(chord[0]);
    }
    // drop a duplicate closing point
    if pts.len() > 2 && pts.first().unwrap().dist(*pts.last().unwrap()) <= tol {
        pts.pop();
    }
    // Newell normal
    let mut n = Vec3::ZERO;
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        n = n + a.cross(b);
    }
    let area = n.len() * 0.5;
    let n = if n.len() > 1e-15 { n.normalize() } else { n };
    // planarity: every point within tol of the plane through pts[0]
    let planar = pts.iter().all(|&p| (p - pts[0]).dot(n).abs() <= tol.max(1e-6));
    if planar {
        (Surface::plane(pts[0], n), area)
    } else {
        // ruled: deg 1 in v between the chain polyline and its chord
        let nu = chain_pts.len();
        let knots_u = flat_knots(1, nu);
        let knots_v = vec![0.0, 0.0, 1.0, 1.0];
        let d0 = *chord.first().unwrap_or(&chain_pts[0]);
        let d1 = *chord.last().unwrap_or(chain_pts.last().unwrap());
        let mut net: Vec<Vec3> = Vec::with_capacity(nu * 2);
        for i in 0..nu {
            let t = i as f64 / (nu - 1).max(1) as f64;
            net.push(chain_pts[i]);
            net.push(d0.lerp(d1, t));
        }
        let bs = BSplineSurface::new(1, 1, knots_u, knots_v, nu, 2, net, None)
            .unwrap_or_else(|_| BSplineSurface::bilinear(chain_pts[0], chain_pts[nu - 1], d0, d1));
        (Surface::BSpline(Box::new(bs)), area)
    }
}

fn flat_knots(deg: usize, n_ctrl: usize) -> Vec<f64> {
    let inner = n_ctrl - deg - 1;
    let mut k = vec![0.0; deg + 1];
    for i in 1..=inner {
        k.push(i as f64);
    }
    k.extend(std::iter::repeat(inner as f64).take(deg + 1));
    k
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The U-shaped sheet of the probe: bottom (z=0) + 3 walls (y=0, x=10,
    /// y=10) of a 10-box, open at the top and at x=0. Faces named like the
    /// oracle: F3.P, F4.S.e0..e2, shared corner vertices.
    fn u_sheet() -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let nm = |r: &str| FaceName { feature: "F3".into(), role: Role::Other(r.into()), region: None, suffix: String::new() };
        let sn = |e: &str| FaceName { feature: "F4".into(), role: Role::Side(e.into()), region: None, suffix: String::new() };
        let (z0, z1) = (0.0, 10.0);
        let bottom = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let bf = b.add_face(shell, Surface::plane(Vec3::ZERO, Vec3::Z), true, nm("P"));
        {
            let lp = b.add_loop(bf);
            let p: Vec<Vec3> = bottom.iter().map(|&(x, y)| Vec3::new(x, y, z0)).collect();
            let vs: Vec<_> = p.iter().map(|&q| b.add_vertex(q)).collect();
            for i in 0..4 {
                let e = b.add_edge(Curve::line_through(p[i], p[(i + 1) % 4]), 0.0, (p[(i + 1) % 4] - p[i]).len(), vs[i], vs[(i + 1) % 4]);
                b.add_coedge(lp, e, false, None);
            }
        }
        let walls: [(&str, [f64; 3], [f64; 3]); 3] = [
            ("e0", [0.0, 0.0, z0], [10.0, 0.0, z0]),
            ("e1", [10.0, 0.0, z0], [10.0, 10.0, z0]),
            ("e2", [10.0, 10.0, z0], [0.0, 10.0, z0]),
        ];
        let bot: Vec<_> = bottom.iter().map(|&(x, y)| b.add_vertex(Vec3::new(x, y, z0))).collect();
        let top: Vec<_> = bottom.iter().map(|&(x, y)| b.add_vertex(Vec3::new(x, y, z1))).collect();
        for (ent, a, c) in walls {
            let pa = Vec3::new(a[0], a[1], a[2]);
            let pb = Vec3::new(c[0], c[1], c[2]);
            let travel = (pb - pa).normalize();
            let nn = travel.cross(Vec3::Z);
            let f = b.add_face(shell, Surface::plane(pa, nn), true, sn(ent));
            let lp = b.add_loop(f);
            let ia = bottom.iter().position(|&(x, y)| (Vec3::new(x, y, z0) - pa).len() < 1e-9).unwrap();
            let ib = bottom.iter().position(|&(x, y)| (Vec3::new(x, y, z0) - pb).len() < 1e-9).unwrap();
            let (va_b, vb_b, va_t, vb_t) = (bot[ia], bot[ib], top[ia], top[ib]);
            let p = [pa, pb, pb + Vec3::new(0.0, 0.0, z1 - z0), pa + Vec3::new(0.0, 0.0, z1 - z0)];
            let mut side = |q0: Vec3, q1: Vec3, v0: ck_topo::body::VertexId, v1: ck_topo::body::VertexId| {
                b.add_edge(Curve::line_through(q0, q1), 0.0, (q1 - q0).len(), v0, v1)
            };
            let rail_a = side(pa, p[3], va_b, va_t);
            let rim0 = side(pa, pb, va_b, vb_b);
            let rail_b = side(pb, p[2], vb_b, vb_t);
            let rim1 = side(p[2], p[3], vb_t, va_t);
            b.add_coedge(lp, rim0, false, None);
            b.add_coedge(lp, rail_b, false, None);
            b.add_coedge(lp, rim1, true, None);
            b.add_coedge(lp, rail_a, true, None);
        }
        b
    }

    fn rim_of(b: &Body, feat: &str, ent: &str) -> EdgeId {
        let f = b
            .face_ids()
            .find(|&f| b.face(f).name.feature == feat && b.face(f).name.meta_entity() == Some(ent.to_string()))
            .unwrap();
        b.face_edges(f)
            .into_iter()
            .find(|&e| {
                let ed = b.edge(e);
                b.vertex(ed.v0).p.z > 9.0 && b.vertex(ed.v1).p.z > 9.0
            })
            .unwrap()
    }

    #[test]
    fn fill_three_rims_oracle() {
        // probe f3a: chain of the 3 top rims -> patch = whole top square,
        // chord edge (0,0,10)->(0,10,10) stored start->end
        let body = u_sheet();
        let e0 = rim_of(&body, "F4", "e0");
        let e1 = rim_of(&body, "F4", "e1");
        let e2 = rim_of(&body, "F4", "e2");
        let b = fill(&body, &[e0, e1, e2], "G0", "F6", 1e-6).unwrap();
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(fids.last().unwrap(), "F6.F");
        assert_eq!(fids.len(), 5);
        let pf = b.face_ids().last().unwrap();
        assert!(b.face(pf).surface.is_planar(1e-9));
        let chord = b.edge_ids().last().unwrap();
        let ed = b.edge(chord);
        assert!(ed.curve.eval(ed.t0).dist(Vec3::new(0.0, 0.0, 10.0)) < 1e-9);
        assert!(ed.curve.eval(ed.t1).dist(Vec3::new(0.0, 10.0, 10.0)) < 1e-9);
        for e in [e0, e1, e2] {
            assert_eq!(b.edge_coedges(e).len(), 2);
        }
    }

    #[test]
    fn fill_two_rims_triangle_oracle() {
        // probe f2a: chain of 2 rims -> triangular patch with the diagonal
        // chord (0,0,10)->(10,10,10)
        let body = u_sheet();
        let e0 = rim_of(&body, "F4", "e0");
        let e1 = rim_of(&body, "F4", "e1");
        let b = fill(&body, &[e0, e1], "G0", "F6", 1e-6).unwrap();
        let chord = b.edge_ids().last().unwrap();
        let ed = b.edge(chord);
        assert!(ed.curve.eval(ed.t0).dist(Vec3::new(0.0, 0.0, 10.0)) < 1e-9);
        assert!(ed.curve.eval(ed.t1).dist(Vec3::new(10.0, 10.0, 10.0)) < 1e-9);
        let pf = b.face_ids().last().unwrap();
        assert!(b.face(pf).surface.is_planar(1e-9));
    }

    #[test]
    fn fill_degenerate_chain_fails_like_oracle() {
        // probe f2b: the same edge twice -> fillFailed with the kernel's
        // validity detail
        let body = u_sheet();
        let e0 = rim_of(&body, "F4", "e0");
        let err = fill(&body, &[e0, e0], "G0", "F6", 1e-6).unwrap_err();
        assert_eq!(err.code, "fillFailed");
        assert_eq!(
            err.detail,
            "the fill is not a valid face: face orientation: F0 (F6.F): parameter area 0.000e0"
        );
    }

    #[test]
    fn fill_chain_errors() {
        let body = u_sheet();
        let e0 = rim_of(&body, "F4", "e0");
        let e2 = rim_of(&body, "F4", "e2");
        let err = fill(&body, &[e0, e2], "G0", "F6", 1e-6).unwrap_err();
        assert_eq!(err.code, "fillInvalid");
        assert_eq!(err.detail, "the edges do not form one chain");
        let err = fill(&body, &[e0], "G0", "F6", 1e-6).unwrap_err();
        assert_eq!(err.code, "fillNotClosed");
        assert_eq!(err.detail, "the chain is open");
    }
}
