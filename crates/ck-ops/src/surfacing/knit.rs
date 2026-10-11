//! Knit: sew faces into a body by merging coincident edges.
//!
//! Semantics pinned by the original kernel (large sample F23 "Knit outer
//! skin": knit F14/F16/F19/F22, solid=false):
//! - Faces may come from different bodies/features; edges that coincide
//!   geometrically (within tol) become one shared edge with a coedge per face.
//! - If `solid` and the result has open boundaries -> `knitNotClosed`.
//! - Endpoints match but curves diverge -> `knitNearMiss`.
//!
//! Face names survive (the kernel's knit result keeps `F14.S.e4`-style ids).
//!
//! HOW TO WIRE (ck-model/src/features/knit.rs, Gate B):
//!   let bodies: Vec<Body> = refs.surfaces.iter().map(|r| regen_face_body(r)).collect();
//!   let body = ck_ops::surfacing::knit::knit(&bodies, solid, 1e-6)
//!       .map_err(|e| FeatureError{ code: e.code, msg: e.detail })?;
//!   // register result under the feature id; visible only via body faces.

use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, ShellId, VertexId};

#[derive(Debug, Clone)]
pub struct KnitError {
    pub code: &'static str,
    pub detail: String,
}

impl std::fmt::Display for KnitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code, self.detail)
    }
}

fn err(code: &'static str, detail: impl Into<String>) -> KnitError {
    KnitError { code, detail: detail.into() }
}

/// Copy a face (with its topology) from `src` into `dst`'s shell.
fn copy_face(
    dst: &mut Body,
    shell: ShellId,
    src: &Body,
    f: FaceId,
    vmap: &mut std::collections::HashMap<(usize, VertexId), VertexId>,
    emap: &mut std::collections::HashMap<(usize, EdgeId), EdgeId>,
) -> FaceId {
    let src_id = src as *const _ as usize;
    let face = src.face(f);
    let nf = dst.add_face(shell, face.surface.clone(), face.sense, face.name.clone());
    for &lid in &face.loops {
        if !src.lp(lid).alive {
            continue;
        }
        let nloop = dst.add_loop(nf);
        for &cid in &src.lp(lid).coedges {
            let co = src.coedge(cid);
            if !co.alive {
                continue;
            }
            let e = src.edge(co.edge);
            let v0 = *vmap
                .entry((src_id, e.v0))
                .or_insert_with(|| dst.add_vertex(src.vertex(e.v0).p));
            let v1 = *vmap
                .entry((src_id, e.v1))
                .or_insert_with(|| dst.add_vertex(src.vertex(e.v1).p));
            let ne = *emap
                .entry((src_id, co.edge))
                .or_insert_with(|| dst.add_edge(e.curve.clone(), e.t0, e.t1, v0, v1));
            dst.add_coedge(nloop, ne, co.rev, co.pcurve.clone());
        }
    }
    nf
}

/// Sample edge geometry range-normalised (arc-length fraction for robustness
/// across parameterisations).
fn sample(b: &Body, e: EdgeId, n: usize) -> Vec<ck_geom::Vec3> {
    let e = b.edge(e);
    // coarse arc-length table
    let steps = 64;
    let mut acc = Vec::with_capacity(steps + 1);
    let mut total = 0.0f64;
    let mut prev = e.curve.eval(e.t0);
    acc.push((e.t0, 0.0));
    for i in 1..=steps {
        let t = e.t0 + (e.t1 - e.t0) * (i as f64 / steps as f64);
        let p = e.curve.eval(t);
        total += p.dist(prev);
        acc.push((t, total));
        prev = p;
    }
    (0..n)
        .map(|i| {
            let target = total * (i as f64 / (n - 1) as f64);
            // binary search
            let mut lo = 0;
            let mut hi = acc.len() - 1;
            while lo + 1 < hi {
                let mid = (lo + hi) / 2;
                if acc[mid].1 < target {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let (t_a, s_a) = acc[lo];
            let (t_b, s_b) = acc[lo + 1];
            let f = if s_b > s_a { (target - s_a) / (s_b - s_a) } else { 0.0 };
            e.curve.eval(t_a + (t_b - t_a) * f)
        })
        .collect()
}

/// Does edge `a` geometrically match edge `b`? Returns whether `b` runs
/// reversed relative to `a`.
fn edges_match(b: &Body, a: EdgeId, b2: EdgeId, tol: f64) -> Option<bool> {
    let sa = sample(b, a, 9);
    let sb = sample(b, b2, 9);
    let close = |x: &ck_geom::Vec3, y: &ck_geom::Vec3| x.dist(*y) <= tol;
    if sa.iter().zip(sb.iter()).all(|(x, y)| close(x, y)) {
        return Some(false);
    }
    if sa.iter().zip(sb.iter().rev()).all(|(x, y)| close(x, y)) {
        return Some(true);
    }
    None
}

/// Sew bodies into one. `tol` is the coincidence tolerance.
pub fn knit(bodies: &[Body], solid: bool, tol: f64) -> Result<Body, KnitError> {
    let mut b = Body::new();
    let mut vmap = std::collections::HashMap::new();
    let mut emap = std::collections::HashMap::new();
    let mut faces: Vec<FaceId> = Vec::new();
    for src in bodies {
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        for f in src.face_ids().collect::<Vec<_>>() {
            faces.push(copy_face(&mut b, shell, src, f, &mut vmap, &mut emap));
        }
    }

    // merge coincident edges
    let all: Vec<EdgeId> = b.edge_ids().collect();
    let mut killed = std::collections::HashSet::new();
    for i in 0..all.len() {
        if killed.contains(&all[i]) || !b.edge(all[i]).alive {
            continue;
        }
        for j in (i + 1)..all.len() {
            if killed.contains(&all[j]) || !b.edge(all[j]).alive {
                continue;
            }
            let (ea, eb) = (all[i], all[j]);
            let (a0, a1) = (b.edge(ea).v0, b.edge(ea).v1);
            let (b0, b1) = (b.edge(eb).v0, b.edge(eb).v1);
            let pos = |v: VertexId| b.vertex(v).p;
            let fwd = pos(a0).dist(pos(b0)) <= tol && pos(a1).dist(pos(b1)) <= tol;
            let rev = pos(a0).dist(pos(b1)) <= tol && pos(a1).dist(pos(b0)) <= tol;
            if !fwd && !rev {
                continue;
            }
            match edges_match(&b, ea, eb, tol) {
                Some(rev_geom) => {
                    // keep pcurves only when the curve objects are identical
                    // (same parameterisation) — ck-topo pcurves are edge-
                    // parameterised; otherwise drop to projection fallback.
                    let same_curve = b.edge(ea).curve == b.edge(eb).curve
                        && b.edge(ea).t0 == b.edge(eb).t0
                        && b.edge(ea).t1 == b.edge(eb).t1
                        && !rev_geom;
                    merge_edge_pair(&mut b, ea, eb, rev);
                    if !same_curve {
                        for c in b.edge(ea).coedges.clone() {
                            b.coedge_mut(c).pcurve = None;
                        }
                    }
                    killed.insert(eb);
                }
                None => {
                    let mid_a = mid_point(&b, ea);
                    let mid_b = mid_point(&b, eb);
                    if mid_a.dist(mid_b) > tol * 100.0 {
                        return Err(err(
                            "knitNearMiss",
                            format!("edges diverge: {} vs {}", ea.idx(), eb.idx()),
                        ));
                    }
                }
            }
        }
    }

    reloop(&mut b);
    assign_shells(&mut b);

    let bl = super::boundary::open_boundaries(&b, &b.face_ids().collect::<Vec<_>>());
    // Every boundary loop is an aperture of the shell (open sheets have
    // rings; solids have none). Dangling chains are reported separately.
    if solid && !bl.is_empty() {
        let rings = bl.iter().filter(|l| l.closed).count();
        let danglings = bl.len() - rings;
        return Err(err(
            "knitNotClosed",
            format!("{} open ring(s), {} dangling chain(s)", rings, danglings),
        ));
    }
    Ok(b)
}

fn mid_point(b: &Body, e: EdgeId) -> ck_geom::Vec3 {
    let e = b.edge(e);
    e.curve.eval((e.t0 + e.t1) * 0.5)
}

/// Merge edge `eb` into `ea`: move eb's coedges onto ea with adjusted rev,
/// redirect eb's endpoints onto ea's vertices, kill eb.
fn merge_edge_pair(b: &mut Body, ea: EdgeId, eb: EdgeId, rev: bool) {
    let (a0, a1) = (b.edge(ea).v0, b.edge(ea).v1);
    let (b0, b1) = (b.edge(eb).v0, b.edge(eb).v1);
    let coedges: Vec<CoedgeId> = b.edge(eb).coedges.clone();
    for c in coedges {
        let was_rev = b.coedge(c).rev;
        let lp = b.coedge(c).lp;
        let pc = b.coedge(c).pcurve.clone();
        b.coedge_mut(c).edge = ea;
        b.coedge_mut(c).rev = was_rev ^ rev;
        b.coedge_mut(c).lp = lp;
        b.coedge_mut(c).pcurve = pc;
        b.edge_mut(ea).coedges.push(c);
    }
    redirect_vertex(b, b0, if rev { a1 } else { a0 });
    redirect_vertex(b, b1, if rev { a0 } else { a1 });
    b.edge_mut(eb).alive = false;
}

/// Redirect all topology referencing vertex `from` to `to`.
fn redirect_vertex(b: &mut Body, from: VertexId, to: VertexId) {
    b.vertex_mut(from).alive = false;
    for e in b.edge_ids().collect::<Vec<_>>() {
        if !b.edge(e).alive {
            continue;
        }
        if b.edge(e).v0 == from {
            b.edge_mut(e).v0 = to;
        }
        if b.edge(e).v1 == from {
            b.edge_mut(e).v1 = to;
        }
    }
}

/// Re-chain each live loop so coedges run head-to-tail.
fn reloop(b: &mut Body) {
    let loops: Vec<_> = b.loop_ids().collect();
    for l in loops {
        let face = b.lp(l).face;
        if !b.face(face).alive {
            continue;
        }
        let cs: Vec<CoedgeId> = b
            .lp(l)
            .coedges
            .iter()
            .copied()
            .filter(|&c| b.coedge(c).alive && b.edge(b.coedge(c).edge).alive)
            .collect();
        if cs.is_empty() {
            continue;
        }
        let mut ordered: Vec<CoedgeId> = vec![cs[0]];
        let mut used = std::collections::HashSet::from([cs[0]]);
        while ordered.len() < cs.len() {
            let tail = b.coedge_end(*ordered.last().unwrap());
            match cs.iter().find(|&&c| !used.contains(&c) && b.coedge_start(c) == tail) {
                Some(&c) => {
                    used.insert(c);
                    ordered.push(c);
                }
                None => break,
            }
        }
        b.lp_mut(l).coedges = ordered;
    }
}

/// Assign shells by connectivity (shared edges), one lump per group.
fn assign_shells(b: &mut Body) {
    let faces: Vec<FaceId> = b.face_ids().collect();
    if faces.is_empty() {
        return;
    }
    let mut parent: std::collections::HashMap<usize, usize> =
        faces.iter().map(|f| (f.idx(), f.idx())).collect();
    fn find(p: &mut std::collections::HashMap<usize, usize>, x: usize) -> usize {
        let mut r = x;
        while p[&r] != r {
            r = p[&r];
        }
        p.insert(x, r);
        r
    }
    let mut edge_faces: std::collections::HashMap<EdgeId, Vec<FaceId>> = std::collections::HashMap::new();
    for &f in &faces {
        for &l in &b.face(f).loops {
            for &c in &b.lp(l).coedges {
                if b.coedge(c).alive {
                    edge_faces.entry(b.coedge(c).edge).or_default().push(f);
                }
            }
        }
    }
    for fs in edge_faces.values() {
        if fs.len() >= 2 {
            let r0 = find(&mut parent, fs[0].idx());
            for f in &fs[1..] {
                let r = find(&mut parent, f.idx());
                parent.insert(r, r0);
            }
        }
    }
    let mut groups: std::collections::HashMap<usize, Vec<FaceId>> = std::collections::HashMap::new();
    for &f in &faces {
        let r = find(&mut parent, f.idx());
        groups.entry(r).or_default().push(f);
    }
    let lump = b.lump_ids().next().unwrap();
    let new_shell = b.add_shell(lump);
    for g in groups.values() {
        for &f in g {
            b.face_mut(f).shell = new_shell;
        }
    }
    for s in b.shell_ids().collect::<Vec<_>>() {
        if s != new_shell {
            b.shell_mut(s).alive = false;
        }
    }
    b.shell_mut(new_shell).faces = faces;
    // one lump owns the merged shell; kill the rest
    for l in b.lump_ids().collect::<Vec<_>>() {
        if !b.lump(l).shells.contains(&new_shell) {
            b.lump_mut(l).alive = false;
        }
    }
}

/// Mutable/id helpers ck-topo doesn't expose; scoped to ck-ops.
pub(crate) trait BodyExt {
    fn loop_ids(&self) -> impl Iterator<Item = ck_topo::body::LoopId> + '_;
    fn lp_mut(&mut self, l: ck_topo::body::LoopId) -> &mut ck_topo::body::Loop;
    fn coedge_mut(&mut self, c: CoedgeId) -> &mut ck_topo::body::Coedge;
    fn edge_mut(&mut self, e: EdgeId) -> &mut ck_topo::body::Edge;
    fn vertex_mut(&mut self, v: VertexId) -> &mut ck_topo::body::Vertex;
    fn face_mut(&mut self, f: FaceId) -> &mut ck_topo::body::Face;
    fn shell_mut(&mut self, s: ShellId) -> &mut ck_topo::body::Shell;
    fn lump_mut(&mut self, l: ck_topo::body::LumpId) -> &mut ck_topo::body::Lump;
}

impl BodyExt for Body {
    fn loop_ids(&self) -> impl Iterator<Item = ck_topo::body::LoopId> + '_ {
        (0..self.loops.len())
            .filter(|&i| self.loops[i].alive)
            .map(|i| ck_topo::body::LoopId(i as u32))
    }
    fn lp_mut(&mut self, l: ck_topo::body::LoopId) -> &mut ck_topo::body::Loop {
        &mut self.loops[l.idx()]
    }
    fn coedge_mut(&mut self, c: CoedgeId) -> &mut ck_topo::body::Coedge {
        &mut self.coedges[c.idx()]
    }
    fn edge_mut(&mut self, e: EdgeId) -> &mut ck_topo::body::Edge {
        &mut self.edges[e.idx()]
    }
    fn vertex_mut(&mut self, v: VertexId) -> &mut ck_topo::body::Vertex {
        &mut self.vertices[v.idx()]
    }
    fn face_mut(&mut self, f: FaceId) -> &mut ck_topo::body::Face {
        &mut self.faces[f.idx()]
    }
    fn shell_mut(&mut self, s: ShellId) -> &mut ck_topo::body::Shell {
        &mut self.shells[s.idx()]
    }
    fn lump_mut(&mut self, l: ck_topo::body::LumpId) -> &mut ck_topo::body::Lump {
        &mut self.lumps[l.idx()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::surface::Surface;
    use ck_geom::{Vec2, Vec3};
    use ck_topo::body::{FaceName, Role};

    fn name(feat: &str) -> FaceName {
        FaceName { feature: feat.into(), role: Role::Other("P".into()), region: None, suffix: String::new() }
    }

    /// Square planar sheet with pcurves, built like ck-topo does.
    fn square(feat: &str, o: Vec3, dx: Vec3, dy: Vec3) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let n = dx.cross(dy).normalize();
        let surf = Surface::plane(o, n);
        let f = b.add_face(shell, surf.clone(), true, name(feat));
        let p = |i: usize| o + dx.scale([0.0, 1.0, 1.0, 0.0][i]) + dy.scale([0.0, 0.0, 1.0, 1.0][i]);
        let lp = b.add_loop(f);
        let vs: Vec<_> = (0..4).map(|i| b.add_vertex(p(i))).collect();
        let mut es = Vec::new();
        for i in 0..4 {
            let a = p(i);
            let bb = p((i + 1) % 4);
            let len = (bb - a).len();
            let e = b.add_edge(ck_geom::curve::Curve::line(a, bb - a), 0.0, len, vs[i], vs[(i + 1) % 4]);
            // pcurve in surface UV
            let uv = |p: Vec3| {
                let (u, v) = surf.closest_param(p);
                Vec2::new(u, v)
            };
            let q0 = uv(a);
            let q1 = uv(bb);
            let pc = ck_geom::curve2::Curve2::line_through(q0, q1);
            b.add_coedge(lp, e, false, Some(pc));
            es.push(e);
        }
        let _ = es;
        b
    }

    #[test]
    fn knit_two_sheets_shares_edge() {
        let a = square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let bs = square("F3", Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let b = knit(&[a, bs], false, 1e-6).unwrap();
        assert_eq!(b.face_ids().count(), 2);
        assert_eq!(b.edge_ids().count(), 7, "8 - 1 merged");
        // shared edge has 2 coedges
        let shared = b.edge_ids().find(|&e| b.edge_coedges(e).len() == 2).expect("shared edge");
        assert!((b.edge(shared).v0 != b.edge(shared).v1));
        // one open boundary loop of 6 edges
        let faces: Vec<FaceId> = b.face_ids().collect();
        let bl = super::super::boundary::open_boundaries(&b, &faces);
        assert_eq!(bl.len(), 1);
        assert_eq!(bl[0].len(), 6);
    }

    #[test]
    fn knit_open_solid_fails() {
        let a = square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let bs = square("F3", Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let e = knit(&[a, bs], true, 1e-6).unwrap_err();
        assert_eq!(e.code, "knitNotClosed");
    }

    #[test]
    fn knit_box_is_closed() {
        let sides = [
            (Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0)),
            (Vec3::new(0.0, 0.0, 10.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0)),
            (Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 10.0, 0.0)),
            (Vec3::ZERO, Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 10.0, 0.0)),
            (Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0)),
            (Vec3::new(0.0, 10.0, 0.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0)),
        ];
        let bodies: Vec<Body> = sides.iter().map(|&(o, dx, dy)| square("F2", o, dx, dy)).collect();
        let b = knit(&bodies, true, 1e-6).unwrap();
        assert_eq!(b.face_ids().count(), 6);
        assert_eq!(b.edge_ids().count(), 12);
        // closed manifold
        for e in b.edge_ids() {
            assert_eq!(b.edge_coedges(e).len(), 2, "edge {} not paired", e.idx());
        }
    }

    #[test]
    fn knit_preserves_names() {
        let a = square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let bs = square("F3", Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let b = knit(&[a, bs], false, 1e-6).unwrap();
        let feats: std::collections::HashSet<String> =
            b.face_ids().map(|f| b.face(f).name.feature.clone()).collect();
        assert!(feats.contains("F2") && feats.contains("F3"));
    }
}
