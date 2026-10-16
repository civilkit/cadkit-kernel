//! Contact edges: two parts of a result that touch along an edge (a box set on another one's edge, the
//! corner of a cut leaving two pieces joined by a line) leave one edge shared by four faces. A solid has
//! two faces per edge, so the edge is split into two coincident edges, one per material wedge: the four
//! faces around it are sorted by the angle of their tangent plane about the edge, and the two consecutive
//! pairs that bound material (the material side of a face is opposite its outward normal) keep the original
//! edge and get the twin. The pieces stay joined at the end vertices only (and are separate lumps where
//! nothing else joins them).

use super::classify::outward;
use ck_geom::Vec3;
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, VertexId};

/// Split every listed 4-face edge into two 2-face edges where the material wedges can be told apart.
/// Returns how many edges were split.
pub fn split_contact_edges(b: &mut Body, edges: &[EdgeId]) -> usize {
    let mut n_split = 0;
    for &e in edges {
        let cs = b.edge_coedges(e);
        if cs.len() != 4 {
            continue;
        }
        let ed = b.edge(e).clone();
        let tm = 0.5 * (ed.t0 + ed.t1);
        let h = 1e-4 * (ed.t1 - ed.t0).abs().max(1e-9);
        let m = ed.curve.eval(tm);
        let Some(d) = (ed.curve.eval(tm + h) - ed.curve.eval(tm - h)).try_normalize() else { continue };
        struct Wedge {
            c: CoedgeId,
            t: Vec3,
            mat: Vec3,
        }
        let perp = |v: Vec3| (v - d * v.dot(d)).try_normalize();
        let mut ws: Vec<Wedge> = vec![];
        for &c in &cs {
            let f = b.coedge_face(c);
            let face = b.face(f);
            let nrm = outward(&face.surface, face.sense, m);
            let u = if b.coedge(c).rev { -d } else { d };
            let (Some(t), Some(mat)) = (perp(nrm.cross(u)), perp(-nrm)) else { break };
            ws.push(Wedge { c, t, mat });
        }
        if ws.len() != 4 {
            continue;
        }
        let e1 = ws[0].t;
        let e2 = d.cross(e1);
        let ang = |w: &Wedge| w.t.dot(e2).atan2(w.t.dot(e1));
        let mut order: Vec<usize> = (0..4).collect();
        order.sort_by(|&i, &j| ang(&ws[i]).total_cmp(&ang(&ws[j])));
        // wedge i: from face order[i] counter-clockwise (about d) to the next face
        let material = |i: usize| {
            let (a, c) = (&ws[order[i]], &ws[order[(i + 1) % 4]]);
            d.cross(a.t).dot(a.mat) > 0.0 && d.cross(c.t).dot(c.mat) < 0.0
        };
        let first = (0..4).find(|&i| material(i));
        let Some(i) = first else {
            super::bool_dbg!("contact edge {e:?}: no material wedge");
            continue;
        };
        if !material(i) || !material((i + 2) % 4) || material((i + 1) % 4) || material((i + 3) % 4) {
            super::bool_dbg!("contact edge {e:?}: wedges {:?}", (0..4).map(material).collect::<Vec<_>>());
            continue;
        }
        // the pair (i + 2, i + 3) moves to a twin edge
        let moved = [ws[order[(i + 2) % 4]].c, ws[order[(i + 3) % 4]].c];
        let e2 = b.add_edge(ed.curve.clone(), ed.t0, ed.t1, ed.v0, ed.v1);
        let moved_faces = [b.coedge_face(moved[0]), b.coedge_face(moved[1])];
        for c in moved {
            b.coedges[c.idx()].edge = e2;
            b.edges[e.idx()].coedges.retain(|&x| x != c);
            b.edges[e2.idx()].coedges.push(c);
        }
        for v in [ed.v0, ed.v1] {
            split_vertex(b, v, moved_faces);
        }
        n_split += 1;
    }
    n_split
}

/// After a contact edge was split, the vertex at its end is still shared by both sheets. Give the moved sheet a
/// vertex of its own when the faces around `v` fall into two groups (faces joined through the edges at `v`, the two
/// twin edges counting separately): the edges of the moved group are re-pointed to the copy.
fn split_vertex(b: &mut Body, v: VertexId, moved: [FaceId; 2]) {
    let edges: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
    if edges.iter().any(|&e| b.edge(e).v0 == b.edge(e).v1) {
        return;
    }
    // union-find over faces joined by an edge at v
    let mut parent: std::collections::HashMap<FaceId, FaceId> = Default::default();
    fn find(p: &mut std::collections::HashMap<FaceId, FaceId>, f: FaceId) -> FaceId {
        let q = *p.entry(f).or_insert(f);
        if q == f {
            return f;
        }
        let r = find(p, q);
        p.insert(f, r);
        r
    }
    for &e in &edges {
        let fs = b.edge_faces(e);
        for w in fs.windows(2) {
            let (x, y) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
            if x != y {
                parent.insert(x, y);
            }
        }
        if let Some(&f) = fs.first() {
            find(&mut parent, f);
        }
    }
    let root = find(&mut parent, moved[0]);
    if find(&mut parent, moved[1]) != root {
        return;
    }
    let roots: std::collections::HashSet<FaceId> = parent.keys().copied().collect::<Vec<_>>().into_iter().map(|f| find(&mut parent, f)).collect();
    if roots.len() < 2 {
        return;
    }
    let nv = b.add_vertex(b.vertex(v).p);
    for &e in &edges {
        let Some(&f) = b.edge_faces(e).first() else { continue };
        if find(&mut parent, f) == root {
            if b.edge(e).v0 == v {
                b.edges[e.idx()].v0 = nv;
            }
            if b.edge(e).v1 == v {
                b.edges[e.idx()].v1 = nv;
            }
        }
    }
}
