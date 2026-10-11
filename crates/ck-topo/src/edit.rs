//! Local topology edits (first pass): split edge, split face by a curve,
//! delete face, merge coplanar faces.

use crate::body::{Body, CoedgeId, Coedge, EdgeId, FaceId, FaceName, LoopId, VertexId};
use crate::make::plane_pcurve;
use crate::orient::fix_planar_loops;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::Vec2;

/// Split edge `e` at parameter `t` (t0 < t < t1). The original edge keeps
/// [t0, t]; returns (new vertex, new edge [t, t1]). Every coedge of `e`
/// gets a partner coedge inserted next to it in its loop.
pub fn split_edge(b: &mut Body, e: EdgeId, t: f64) -> (VertexId, EdgeId) {
    let (curve, v1) = {
        let ed = b.edge(e);
        (ed.curve.clone(), ed.v1)
    };
    let p = curve.eval(t);
    let v = b.add_vertex(p);
    let t1 = b.edge(e).t1;
    b.edges[e.idx()].t1 = t;
    b.edges[e.idx()].v1 = v;
    let ne = b.add_edge(curve, t, t1, v, v1);
    for c in b.edge_coedges(e) {
        let ce = b.coedge(c).clone();
        b.coedges.push(Coedge { edge: ne, rev: ce.rev, lp: ce.lp, pcurve: ce.pcurve.clone(), alive: true });
        let nc = CoedgeId(b.coedges.len() as u32 - 1);
        b.edges[ne.idx()].coedges.push(nc);
        let lc = &mut b.loops[ce.lp.idx()].coedges;
        let pos = lc.iter().position(|&x| x == c).unwrap();
        // forward: c (t0..t) then nc (t..t1); reversed: nc (t1..t) then c
        if ce.rev {
            lc.insert(pos, nc);
        } else {
            lc.insert(pos + 1, nc);
        }
    }
    (v, ne)
}

/// Reverse a LINE edge's parameterisation (t -> -t): swaps its vertices,
/// flips its coedges and their (line) pcurves. Returns false if `e` is not
/// a line or carries a non-line pcurve.
pub fn reverse_line_edge(b: &mut Body, e: EdgeId) -> bool {
    let cs = b.edge_coedges(e);
    if !matches!(b.edge(e).curve, Curve::Line { .. })
        || cs.iter().any(|&c| matches!(b.coedge(c).pcurve, Some(ref p) if !matches!(p, Curve2::Line { .. })))
    {
        return false;
    }
    let ed = &mut b.edges[e.idx()];
    if let Curve::Line { o, d } = ed.curve {
        ed.curve = Curve::Line { o, d: -d };
    }
    let (t0, t1) = (ed.t0, ed.t1);
    ed.t0 = -t1;
    ed.t1 = -t0;
    std::mem::swap(&mut ed.v0, &mut ed.v1);
    for c in cs {
        let ce = &mut b.coedges[c.idx()];
        ce.rev = !ce.rev;
        if let Some(Curve2::Line { o, d }) = ce.pcurve {
            ce.pcurve = Some(Curve2::Line { o, d: -d });
        }
    }
    true
}

/// Delete a face (leaves its edges open). Edges / vertices left unused
/// are flagged dead.
pub fn delete_face(b: &mut Body, f: FaceId) {
    if !b.faces[f.idx()].alive {
        return;
    }
    b.faces[f.idx()].alive = false;
    let sh = b.faces[f.idx()].shell;
    b.shells[sh.idx()].faces.retain(|&x| x != f);
    for l in b.faces[f.idx()].loops.clone() {
        b.loops[l.idx()].alive = false;
        for c in b.loops[l.idx()].coedges.clone() {
            b.coedges[c.idx()].alive = false;
        }
    }
    gc(b);
}

/// Flag edges without live coedges and vertices without live edges dead.
pub fn gc(b: &mut Body) {
    for i in 0..b.edges.len() {
        if b.edges[i].alive && b.edge_coedges(EdgeId(i as u32)).is_empty() {
            b.edges[i].alive = false;
        }
    }
    let mut used = vec![false; b.vertices.len()];
    for e in b.edge_ids() {
        used[b.edge(e).v0.idx()] = true;
        used[b.edge(e).v1.idx()] = true;
    }
    for (i, u) in used.into_iter().enumerate() {
        if !u {
            b.vertices[i].alive = false;
        }
    }
}

/// Split face `f` with a new edge `curve[t0,t1]` from vertex `va` (at t0)
/// to `vb` (at t1); both must lie on the same loop of `f`. The original
/// face keeps the part containing the loop run va→vb; the other part
/// becomes a new face named `name` (same surface/sense). `pc` = the new
/// edge's pcurve on the surface (computed for planes when None).
#[allow(clippy::too_many_arguments)]
pub fn split_face(
    b: &mut Body,
    f: FaceId,
    va: VertexId,
    vb: VertexId,
    curve: Curve,
    t0: f64,
    t1: f64,
    pc: Option<Curve2>,
    name: FaceName,
) -> Result<(EdgeId, FaceId), String> {
    let loops = b.face(f).loops.clone();
    let (li, i, j) = loops
        .iter()
        .enumerate()
        .find_map(|(k, &l)| {
            let cs = &b.lp(l).coedges;
            let i = cs.iter().position(|&c| b.coedge_start(c) == va)?;
            let j = cs.iter().position(|&c| b.coedge_start(c) == vb)?;
            Some((k, i, j))
        })
        .ok_or_else(|| "splitFaceVerticesNotOnLoop".to_string())?; // GUESS: error string
    let l = loops[li];
    let cs = b.lp(l).coedges.clone();
    let n = cs.len();
    let run = |from: usize, to: usize| -> Vec<CoedgeId> {
        let mut out = vec![];
        let mut k = from;
        while k != to {
            out.push(cs[k]);
            k = (k + 1) % n;
        }
        out
    };
    let (run1, run2) = (run(i, j), run(j, i));
    let pc = pc.or_else(|| match &b.face(f).surface {
        Surface::Plane { frame } => plane_pcurve(frame, &curve),
        _ => None,
    });
    let e = b.add_edge(curve, t0, t1, va, vb);
    // loop 1 (stays on f): run1 then e reversed
    b.loops[l.idx()].coedges = run1;
    b.add_coedge(l, e, true, pc.clone());
    // loop 2 (new face): run2 then e forward
    let (surf, sense, shell) = {
        let fc = b.face(f);
        (fc.surface.clone(), fc.sense, fc.shell)
    };
    let nf = b.add_face(shell, surf, sense, name);
    let nl = b.add_loop(nf);
    for c in run2 {
        b.coedges[c.idx()].lp = nl;
        b.loops[nl.idx()].coedges.push(c);
    }
    b.add_coedge(nl, e, false, pc);
    // re-home inner loops that fall inside the new face (uv test)
    let poly2 = loop_uv_polygon(b, nl);
    for &h in loops.iter().filter(|&&x| x != l) {
        let probe = loop_uv_polygon(b, h)[0];
        if point_in_poly(probe, &poly2) {
            b.faces[f.idx()].loops.retain(|&x| x != h);
            b.faces[nf.idx()].loops.push(h);
            b.loops[h.idx()].face = nf;
        }
    }
    Ok((e, nf))
}

fn loop_uv_polygon(b: &Body, l: LoopId) -> Vec<Vec2> {
    let mut out = vec![];
    let mut prev = None;
    for &c in &b.lp(l).coedges {
        let (ta, tb) = b.coedge_params(c);
        for k in 0..8 {
            let uv = b.coedge_uv(c, ta + (tb - ta) * k as f64 / 8.0, prev);
            prev = Some(uv);
            out.push(uv);
        }
    }
    out
}

fn point_in_poly(p: Vec2, poly: &[Vec2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, c) = (poly[i], poly[(i + 1) % n]);
        if (a.y > p.y) != (c.y > p.y) && p.x < a.x + (p.y - a.y) * (c.x - a.x) / (c.y - a.y) {
            inside = !inside;
        }
    }
    inside
}

/// Merge pairs of adjacent coplanar planar faces (same plane, same outward
/// normal, same shell). The lower face id survives (keeps its name).
/// Returns the number of merges.
pub fn merge_coplanar(b: &mut Body, tol: f64) -> usize {
    let mut merges = 0;
    loop {
        let mut pair = None;
        'search: for e in b.edge_ids() {
            let fs = b.edge_faces(e);
            if fs.len() != 2 {
                continue;
            }
            let (f, g) = (fs[0].min(fs[1]), fs[0].max(fs[1]));
            if b.face(f).shell != b.face(g).shell {
                continue;
            }
            if let (Surface::Plane { frame: pf }, Surface::Plane { frame: pg }) = (&b.face(f).surface, &b.face(g).surface) {
                let nf = b.planar_normal(f);
                let ng = b.planar_normal(g);
                if nf.dot(ng) > 1.0 - 1e-12 && (pg.o - pf.o).dot(pf.z).abs() < tol {
                    pair = Some((f, g));
                    break 'search;
                }
            }
        }
        let Some((f, g)) = pair else { break };
        merge_faces(b, f, g);
        merges += 1;
    }
    merges
}

/// Merge planar face `g` into coplanar face `f` (removing shared edges).
pub fn merge_faces(b: &mut Body, f: FaceId, g: FaceId) {
    let frame = match &b.face(f).surface {
        Surface::Plane { frame } => *frame,
        _ => return,
    };
    // drop shared edges
    for e in b.face_edges(g) {
        let fs = b.edge_faces(e);
        if fs.len() == 2 && fs.contains(&f) && fs.contains(&g) {
            for c in b.edge_coedges(e) {
                b.coedges[c.idx()].alive = false;
            }
            b.edges[e.idx()].alive = false;
        }
    }
    // collect remaining coedges of both faces
    let mut pool: Vec<CoedgeId> = vec![];
    for &face in &[f, g] {
        for l in b.face(face).loops.clone() {
            b.loops[l.idx()].alive = false;
            pool.extend(b.lp(l).coedges.iter().copied().filter(|&c| b.coedges[c.idx()].alive));
        }
    }
    for &c in &pool {
        if b.coedge_face(c) == g {
            let e = b.coedge(c).edge;
            b.coedges[c.idx()].pcurve = plane_pcurve(&frame, &b.edge(e).curve.clone());
        }
    }
    b.faces[f.idx()].loops.clear();
    // re-trace loops by chaining end -> start
    let mut used = vec![false; pool.len()];
    for s in 0..pool.len() {
        if used[s] {
            continue;
        }
        let nl = b.add_loop(f);
        let mut k = s;
        loop {
            used[k] = true;
            let c = pool[k];
            b.coedges[c.idx()].lp = nl;
            b.loops[nl.idx()].coedges.push(c);
            let end = b.coedge_end(c);
            match (0..pool.len()).find(|&m| !used[m] && b.coedge_start(pool[m]) == end) {
                Some(m) => k = m,
                None => break,
            }
        }
    }
    b.faces[g.idx()].alive = false;
    let sh = b.face(g).shell;
    b.shells[sh.idx()].faces.retain(|&x| x != g);
    fix_planar_loops(b, f);
    gc(b);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::check;
    use crate::make::tests::box_body;
    use crate::mass::mass_props;
    use ck_geom::Vec3;

    #[test]
    fn split_edge_keeps_validity_and_mass() {
        let mut b = box_body();
        let e = b.wire_edges()[4]; // C0|S.e1 bottom edge (0,0,0)->(40,0,0)
        let (v, ne) = split_edge(&mut b, e, 15.0);
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert!(b.vertex(v).p.dist(Vec3::new(15.0, 0.0, 0.0)) < 1e-12);
        assert_eq!(b.edge_faces(ne).len(), 2);
        assert!((mass_props(&b).volume - 8000.0).abs() < 1e-9);
    }

    #[test]
    fn split_then_merge_face() {
        let mut b = box_body();
        // split the C1 cap (y=10) along x=15 by first splitting its two long edges
        let es = b.wire_edges();
        let (e_a, e_b) = (es[5], es[9]); // C1|e1 (0,10,0)->(40,10,0), C1|e3 (40,10,-20)->(0,10,-20)
        let (va, _) = split_edge(&mut b, e_a, 15.0);
        let (vb, _) = split_edge(&mut b, e_b, 25.0);
        let c = Curve::line_through(b.vertex(vb).p, b.vertex(va).p);
        let len = b.vertex(va).p.dist(b.vertex(vb).p);
        let (_, nf) = split_face(&mut b, FaceId(1), vb, va, c, 0.0, len, None, FaceName::cap("F2", 1, Some(2))).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(b.num_faces(), 7);
        assert!((mass_props(&b).volume - 8000.0).abs() < 1e-9);
        assert!((mass_props(&b).area - 2800.0).abs() < 1e-9);
        assert_eq!(b.face_id_str(nf), "F2.C1~2");
        assert_eq!(merge_coplanar(&mut b, 1e-9), 1);
        assert_eq!(b.num_faces(), 6);
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert!((mass_props(&b).volume - 8000.0).abs() < 1e-9);
    }

    #[test]
    fn delete_face_opens_body() {
        let mut b = box_body();
        delete_face(&mut b, FaceId(1));
        assert_eq!(b.num_faces(), 5);
        assert_eq!(b.wire_edges().len(), 12);
    }
}
