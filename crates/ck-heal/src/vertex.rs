// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/heal/src/fix/wireframe.rs
// (gap closing by vertex merging) and crates/heal/src/fix/split_vertex.rs (vertex bookkeeping).
//! Near-miss vertex merging: vertices that bound an unpaired / gapped /
//! tiny edge and lie within the tolerance of each other become one vertex.

use crate::Fixed;
use ck_topo::body::{Body, VertexId};
use std::collections::HashSet;

/// Vertices worth considering: endpoints of free or unused edges, of edges
/// shorter than `tol`, and both sides of a loop discontinuity. Healthy
/// vertices are never touched, so a valid body is a fixed point.
fn candidates(b: &Body, tol: f64) -> Vec<VertexId> {
    let mut set: HashSet<VertexId> = HashSet::new();
    for e in b.edge_ids() {
        let n = b.edge_coedges(e).len();
        if n < 2 || b.edge_length(e) < tol {
            set.insert(b.edge(e).v0);
            set.insert(b.edge(e).v1);
        }
    }
    for f in b.face_ids() {
        for &l in &b.face(f).loops {
            if !b.lp(l).alive {
                continue;
            }
            let cs: Vec<_> = b
                .lp(l)
                .coedges
                .iter()
                .copied()
                .filter(|&c| b.coedge(c).alive)
                .collect();
            for i in 0..cs.len() {
                let (ve, vs) = (b.coedge_end(cs[i]), b.coedge_start(cs[(i + 1) % cs.len()]));
                if ve != vs {
                    set.insert(ve);
                    set.insert(vs);
                }
            }
        }
    }
    let mut v: Vec<VertexId> = set.into_iter().collect();
    v.sort();
    v
}

fn find(p: &mut [usize], mut i: usize) -> usize {
    while p[i] != i {
        p[i] = p[p[i]];
        i = p[i];
    }
    i
}

/// Merge candidate vertices closer than `tol` (representative = lowest id).
/// Returns the number of vertices merged away.
pub fn merge_near_vertices(b: &mut Body, tol: f64, out: &mut Vec<Fixed>) -> usize {
    let mut vs = candidates(b, tol);
    vs.sort_by(|&a, &c| b.vertex(a).p.x.partial_cmp(&b.vertex(c).p.x).unwrap());
    let mut parent: Vec<usize> = (0..vs.len()).collect();
    for i in 0..vs.len() {
        let pi = b.vertex(vs[i]).p;
        for j in i + 1..vs.len() {
            let pj = b.vertex(vs[j]).p;
            if pj.x - pi.x > tol {
                break;
            }
            if pi.dist(pj) <= tol {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                if ri != rj {
                    // lowest vertex id becomes the root
                    if vs[ri] < vs[rj] {
                        parent[rj] = ri;
                    } else {
                        parent[ri] = rj;
                    }
                }
            }
        }
    }
    let mut remap: Vec<Option<VertexId>> = vec![None; b.vertices.len()];
    let mut n = 0;
    for i in 0..vs.len() {
        let r = find(&mut parent, i);
        if r != i {
            let (from, to) = (vs[i], vs[r]);
            remap[from.idx()] = Some(to);
            out.push(Fixed {
                code: "VERTEX_MERGED",
                entity: format!("vertex#{}", from.idx()),
                detail: format!(
                    "merged into vertex#{} (gap {:.3e})",
                    to.idx(),
                    b.vertex(from).p.dist(b.vertex(to).p)
                ),
            });
            n += 1;
        }
    }
    if n == 0 {
        return 0;
    }
    for e in &mut b.edges {
        if let Some(t) = remap[e.v0.idx()] {
            e.v0 = t;
        }
        if let Some(t) = remap[e.v1.idx()] {
            e.v1 = t;
        }
    }
    for (i, r) in remap.iter().enumerate() {
        if r.is_some() {
            b.vertices[i].alive = false;
        }
    }
    n
}
