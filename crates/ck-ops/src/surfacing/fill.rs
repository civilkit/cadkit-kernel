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

use crate::surfacing::fair::{fair, Fairing};
use ck_geom::curve::Curve;
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
pub fn fill(
    body: &Body,
    chain: &[EdgeId],
    continuity: &str,
    fairing: Fairing,
    strength: f64,
    feature: &str,
    tol: f64,
) -> Result<Body, FillError> {
    let steps = walk_chain(body, chain)?;
    // a chain whose last edge ends where the first begins is a closed ring
    // (no chord; oracle: filling the 4 rim edges of a tube adds 1 face, 0 edges)
    let closed_ring = {
        let (f, l) = (&steps[0], &steps[steps.len() - 1]);
        let s0 = if f.fwd { body.edge(f.e).v0 } else { body.edge(f.e).v1 };
        let s1 = if l.fwd { body.edge(l.e).v1 } else { body.edge(l.e).v0 };
        body.vertex(s0).p.dist(body.vertex(s1).p) <= tol
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

    let mut boundary = sample_chain(body, &steps, 16);
    if !closed_ring {
        boundary.push(end);
        boundary.push(start);
    }
    let surface = fair(&boundary, fairing, strength);
    // validity area (planar patch area; the ruled fallback passes)
    let n = surface_normal_of(&surface, start);
    let area = polygon_area(&boundary, n).max(if surface.is_planar(tol.max(1e-6)) { 0.0 } else { 1.0 });
    let nf = out.add_face(shell, surface, true, name);

    // loop: the chain edges traversed opposite the body faces' coedges
    let nloop = out.add_loop(nf);
    let body_rev = |e: EdgeId| {
        body.edge_coedges(e).iter().find(|&&c| body.coedge(c).alive).map(|&c| body.coedge(c).rev).unwrap_or(false)
    };
    // The walk traverses step i v0->v1 iff `fwd`; the neighbouring face
    // traverses it v0->v1 iff !body_rev. The patch loop must oppose its
    // neighbours: keep the walk order when it does, run the whole loop
    // backwards (order AND directions) when the walk follows them.
    let agree = steps.iter().filter(|s| s.fwd == !body_rev(s.e)).count();
    let backwards = agree * 2 > steps.len();
    let mut co: Vec<(EdgeId, bool)> = steps.iter().map(|s| (s.e, if backwards { s.fwd } else { !s.fwd })).collect();
    if !closed_ring {
        // the chord: stored start->end, traversed reversed (probe-pinned)
        let va = out.add_vertex(start);
        let vb = out.add_vertex(end);
        let ce = out.add_edge(Curve::line_through(start, end), 0.0, (end - start).len(), va, vb);
        co.push((ce, !backwards));
    }
    if backwards {
        co.reverse();
    }
    for (e, rev) in co {
        out.add_coedge(nloop, e, rev, None);
    }

    // the loop direction is fixed by the neighbours (opposite to theirs):
    // pick the patch's sense so the loop runs CCW about its outward normal
    // (oracle: filling both rims of a tube gives opposite plane normals)
    if matches!(out.face(nf).surface, Surface::Plane { .. }) && !ck_topo::orient::loops_consistent(&out, nf) {
        out.faces[nf.idx()].sense = !out.faces[nf.idx()].sense;
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

/// Planar polygon area about normal `n` (Newell).
fn polygon_area(pts: &[Vec3], n: Vec3) -> f64 {
    let mut a = 0.0;
    for i in 0..pts.len() {
        let p = pts[i];
        let q = pts[(i + 1) % pts.len()];
        a += n.dot(p.cross(q));
    }
    (a * 0.5).abs()
}

/// The patch's outward normal for the area validity check.
fn surface_normal_of(s: &Surface, at: Vec3) -> Vec3 {
    match s {
        Surface::Plane { frame } => frame.z,
        _ => {
            let (u, v) = match s {
                Surface::Plane { .. } => (0.0, 0.0),
                _ => {
                    let (uu, vv) = s.closest_param(at);
                    (uu, vv)
                }
            };
            s.normal(u, v).normalize()
        }
    }
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
        let b = fill(&body, &[e0, e1, e2], "G0", Fairing::MeowNurbs, 0.5, "F6", 1e-6).unwrap();
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
        let b = fill(&body, &[e0, e1], "G0", Fairing::MeowNurbs, 0.5, "F6", 1e-6).unwrap();
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
        let err = fill(&body, &[e0, e0], "G0", Fairing::MeowNurbs, 0.5, "F6", 1e-6).unwrap_err();
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
        let err = fill(&body, &[e0, e2], "G0", Fairing::MeowNurbs, 0.5, "F6", 1e-6).unwrap_err();
        assert_eq!(err.code, "fillInvalid");
        assert_eq!(err.detail, "the edges do not form one chain");
        let err = fill(&body, &[e0], "G0", Fairing::MeowNurbs, 0.5, "F6", 1e-6).unwrap_err();
        assert_eq!(err.code, "fillNotClosed");
        assert_eq!(err.detail, "the chain is open");
    }
}
