//! Open-boundary walking on shells.
//!
//! A "boundary" is a chain of edges used by exactly one live coedge among a
//! set of faces (open sheets have free edges; manifold solids have none).
//! Loops are returned head-to-tail with the face on the left; this is the
//! input format fill/knit/thicken/extend all consume.
//!
//! Error strings this feeds (kernel.strings): knitNotClosed, knitNearMiss,
//! fillNotClosed, thickenReach, extendNotBoundary, extendPointBehind.

use ck_topo::body::{Body, EdgeId, FaceId, VertexId};

/// One open boundary chain, oriented face-on-the-left.
#[derive(Debug, Clone)]
pub struct BndLoop {
    /// (edge, traverse v0->v1?) in chain order.
    pub steps: Vec<(EdgeId, bool)>,
    /// True when the chain closes on itself (head == tail). A well-formed
    /// boundary loop of a sheet IS a closed ring — "open" means the shell
    /// has apertures (free edges), not that the chain dangles. A dangling
    /// chain (false) indicates a tolerance gap / non-manifold input.
    pub closed: bool,
}

impl BndLoop {
    pub fn start_vertex(&self, b: &Body) -> VertexId {
        let (e, fwd) = self.steps[0];
        if fwd {
            b.edge(e).v0
        } else {
            b.edge(e).v1
        }
    }
    pub fn end_vertex(&self, b: &Body) -> VertexId {
        let (e, fwd) = *self.steps.last().unwrap();
        if fwd {
            b.edge(e).v1
        } else {
            b.edge(e).v0
        }
    }
    pub fn len(&self) -> usize {
        self.steps.len()
    }
    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }
}

/// Edges of `faces` that are free (no second live face in the whole body
/// uses them). Returns (edge, the single coedge within `faces`).
fn free_edges(b: &Body, faces: &[FaceId]) -> Vec<(EdgeId, ck_topo::body::CoedgeId)> {
    let mut count: std::collections::HashMap<EdgeId, Vec<ck_topo::body::CoedgeId>> =
        std::collections::HashMap::new();
    for &f in faces {
        for &l in &b.face(f).loops {
            if !b.lp(l).alive {
                continue;
            }
            for &c in &b.lp(l).coedges {
                let co = b.coedge(c);
                if co.alive {
                    count.entry(co.edge).or_default().push(c);
                }
            }
        }
    }
    let mut out = Vec::new();
    for (e, cs) in count {
        if !b.edge(e).alive {
            continue;
        }
        // A truly free edge has exactly one live coedge in the whole body.
        // A seam edge has 2 coedges on ONE face (periodic face inside the
        // set) — never a boundary.
        if b.is_seam(e) {
            continue;
        }
        let total_live = b.edge_coedges(e).iter().filter(|&&c| b.coedge(c).alive).count();
        if total_live <= 1 {
            if let Some(&c) = cs.first() {
                out.push((e, c));
            }
        }
    }
    out.sort_by_key(|(e, _)| e.idx());
    out
}

/// Find all open boundary loops of `faces`, chained head-to-tail with the
/// face on the left. Vertices are matched exactly; callers sew with
/// tolerance before calling when needed.
pub fn open_boundaries(b: &Body, faces: &[FaceId]) -> Vec<BndLoop> {
    let free = free_edges(b, faces);
    if free.is_empty() {
        return vec![];
    }
    // oriented traversal: coedge.rev == false means edge runs v0->v1 in
    // loop direction (ck-topo convention).
    use std::collections::HashMap;
    let mut by_start: HashMap<VertexId, Vec<usize>> = HashMap::new();
    let mut by_end: HashMap<VertexId, Vec<usize>> = HashMap::new();
    let mut dir = Vec::with_capacity(free.len());
    for &(e, c) in &free {
        let fwd = !b.coedge(c).rev;
        dir.push(fwd);
        let (s, t) = if fwd {
            (b.edge(e).v0, b.edge(e).v1)
        } else {
            (b.edge(e).v1, b.edge(e).v0)
        };
        by_start.entry(s).or_default().push(dir.len() - 1);
        by_end.entry(t).or_default().push(dir.len() - 1);
    }

    let mut used = vec![false; free.len()];
    let mut loops = Vec::new();
    for i in 0..free.len() {
        if used[i] {
            continue;
        }
        let mut chain: Vec<usize> = vec![i];
        used[i] = true;
        // extend forward: chain end vertex matches another segment's start
        loop {
            let last = *chain.last().unwrap();
            let end_v = if dir[last] {
                b.edge(free[last].0).v1
            } else {
                b.edge(free[last].0).v0
            };
            let found = by_start
                .get(&end_v)
                .and_then(|v| v.iter().find(|&&j| !used[j]))
                .copied();
            match found {
                Some(j) => {
                    used[j] = true;
                    chain.push(j);
                }
                None => break,
            }
        }
        // extend backward: segments whose end matches the chain start
        loop {
            let first = chain[0];
            let start_v = if dir[first] {
                b.edge(free[first].0).v0
            } else {
                b.edge(free[first].0).v1
            };
            let found = by_end
                .get(&start_v)
                .and_then(|v| v.iter().find(|&&j| !used[j]))
                .copied();
            match found {
                Some(j) => {
                    used[j] = true;
                    chain.insert(0, j);
                }
                None => break,
            }
        }
        let closed = {
            let first = chain[0];
            let last = *chain.last().unwrap();
            let sv = if dir[first] {
                b.edge(free[first].0).v0
            } else {
                b.edge(free[first].0).v1
            };
            let ev = if dir[last] {
                b.edge(free[last].0).v1
            } else {
                b.edge(free[last].0).v0
            };
            sv == ev
        };
        let steps = chain
            .iter()
            .map(|&k| {
                let (e, c) = free[k];
                (e, !b.coedge(c).rev)
            })
            .collect();
        loops.push(BndLoop { steps, closed });
    }
    loops.sort_by_key(|l| l.steps[0].0.idx());
    loops
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::curve::Curve;
    use ck_geom::surface::Surface;
    use ck_geom::Vec3;
    use ck_topo::body::{Body, FaceName, Role};

    fn name(feat: &str) -> FaceName {
        FaceName { feature: feat.into(), role: Role::Other("P".into()), region: None, suffix: String::new() }
    }

    /// One planar square face (10x10 at z=0) as a sheet body.
    fn square(feat: &str, o: Vec3, dx: Vec3, dy: Vec3) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let n = dx.cross(dy).normalize();
        let f = b.add_face(shell, Surface::plane(o, n), true, name(feat));
        let p = |i: usize| o + dx.scale([0.0, 1.0, 1.0, 0.0][i]) + dy.scale([0.0, 0.0, 1.0, 1.0][i]);
        let lp = b.add_loop(f);
        let vs: Vec<_> = (0..4).map(|i| b.add_vertex(p(i))).collect();
        let es: Vec<_> = (0..4)
            .map(|i| {
                let a = p(i);
                let bb = p((i + 1) % 4);
                let d = (bb - a).normalize();
                let len = (bb - a).len();
                b.add_edge(Curve::line(a, d), 0.0, len, vs[i], vs[(i + 1) % 4])
            })
            .collect();
        for i in 0..4 {
            b.add_coedge(lp, es[i], false, None);
        }
        b
    }

    #[test]
    fn single_sheet_has_one_boundary() {
        let b = square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let faces: Vec<FaceId> = b.face_ids().collect();
        let loops = open_boundaries(&b, &faces);
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].len(), 4);
        // a square sheet's boundary ring closes head-to-tail
        assert!(loops[0].closed);
        // face on the left => loop is CCW about +z => starts v0->v1 on e0
        assert!(loops[0].steps[0].1);
    }

    #[test]
    fn two_adjacent_squares_share_one_edge() {
        let a = square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let bs = square("F3", Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        // knit them first (copies + sews); use knit to get one body
        let b = super::super::knit::knit(&[a, bs], false, 1e-6).unwrap();
        let faces: Vec<FaceId> = b.face_ids().collect();
        assert_eq!(faces.len(), 2);
        let loops = open_boundaries(&b, &faces);
        assert_eq!(loops.len(), 1, "one outer boundary loop");
        assert_eq!(loops[0].len(), 6, "shared edge is interior");
    }

    #[test]
    fn closed_body_has_no_boundary() {
        // 6 squares of a box
        let sides = [
            (Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0)),
            (Vec3::new(0.0, 0.0, 10.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0)),
            (Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 10.0, 0.0)),
            (Vec3::ZERO, Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 10.0, 0.0)),
            (Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0)),
            (Vec3::new(0.0, 10.0, 0.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0)),
        ];
        let bodies: Vec<Body> = sides.iter().map(|&(o, dx, dy)| square("F2", o, dx, dy)).collect();
        let b = super::super::knit::knit(&bodies, false, 1e-6).unwrap();
        let faces: Vec<FaceId> = b.face_ids().collect();
        assert_eq!(faces.len(), 6);
        let loops = open_boundaries(&b, &faces);
        assert!(loops.is_empty(), "closed cube shell: {:?}", loops.len());
    }
}
