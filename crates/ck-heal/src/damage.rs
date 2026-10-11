//! Deliberate damage for self-tests (unit tests, the `heal.repair` gate
//! metric): each helper breaks a valid body the way real import / boolean
//! defects do, without changing any face geometry. Not part of brepkit.

use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, VertexId};
use ck_topo::edit::split_edge;
use ck_topo::orient::flip_face;
use std::collections::HashMap;

pub const KINDS: &[&str] = &[
    "detach",
    "shells",
    "flip",
    "invert",
    "reverseCoedge",
    "shuffle",
    "tiny",
];

/// Apply a named damage. Returns false when the body has nothing to damage.
pub fn apply(b: &mut Body, kind: &str, shift: f64) -> bool {
    let faces: Vec<FaceId> = b.face_ids().collect();
    if faces.len() < 2 {
        return false;
    }
    match kind {
        "detach" => detach_faces(b, &faces[..1], shift, false),
        "shells" => detach_faces(b, &faces[..faces.len() / 2], shift, true),
        "flip" => {
            flip_face(b, faces[0]);
            true
        }
        "invert" => {
            for f in faces {
                flip_face(b, f);
            }
            true
        }
        "reverseCoedge" => {
            let pick = b.face_ids().find_map(|f| {
                b.face(f)
                    .loops
                    .first()
                    .and_then(|&l| b.lp(l).coedges.first().copied())
            });
            match pick {
                Some(c) => {
                    b.coedges[c.idx()].rev = !b.coedges[c.idx()].rev;
                    true
                }
                None => false,
            }
        }
        "shuffle" => {
            let pick = b.face_ids().find_map(|f| {
                b.face(f)
                    .loops
                    .first()
                    .copied()
                    .filter(|&l| b.lp(l).coedges.len() >= 3)
            });
            match pick {
                Some(l) => {
                    let n = b.lp(l).coedges.len();
                    b.loops[l.idx()].coedges.swap(1, n - 1);
                    true
                }
                None => false,
            }
        }
        "tiny" => tiny_edge(b),
        _ => false,
    }
}

/// Un-sew `faces` from the rest of the body: their boundary gets its own
/// vertices (offset by `shift`) and edges; optionally into a new lump.
pub fn detach_faces(b: &mut Body, faces: &[FaceId], shift: f64, new_lump: bool) -> bool {
    let mut vmap: HashMap<VertexId, VertexId> = HashMap::new();
    let mut emap: HashMap<EdgeId, EdgeId> = HashMap::new();
    let mut n = 0.0;
    let mut clone_v = |b: &mut Body, v: VertexId, vmap: &mut HashMap<VertexId, VertexId>| {
        *vmap.entry(v).or_insert_with(|| {
            n += 1.0;
            let p = b.vertex(v).p + Vec3::new(shift, 0.5 * shift, -0.25 * shift) * (1.0 + 0.1 * n);
            b.add_vertex(p)
        })
    };
    for &f in faces {
        for e in b.face_edges(f) {
            if emap.contains_key(&e) {
                continue;
            }
            let (v0, v1) = (b.edge(e).v0, b.edge(e).v1);
            let (n0, n1) = (clone_v(b, v0, &mut vmap), clone_v(b, v1, &mut vmap));
            let mine: Vec<_> = b
                .edge_coedges(e)
                .into_iter()
                .filter(|&c| faces.contains(&b.coedge_face(c)))
                .collect();
            if mine.len() == b.edge_coedges(e).len() {
                b.edges[e.idx()].v0 = n0;
                b.edges[e.idx()].v1 = n1;
                emap.insert(e, e);
            } else {
                let ed = b.edge(e).clone();
                let ne = b.add_edge(ed.curve, ed.t0, ed.t1, n0, n1);
                for c in mine {
                    b.edges[e.idx()].coedges.retain(|&x| x != c);
                    b.edges[ne.idx()].coedges.push(c);
                    b.coedges[c.idx()].edge = ne;
                }
                emap.insert(e, ne);
            }
        }
    }
    if new_lump {
        let lump = b.add_lump();
        let sh = b.add_shell(lump);
        for &f in faces {
            let old = b.face(f).shell;
            b.shells[old.idx()].faces.retain(|&x| x != f);
            b.shells[sh.idx()].faces.push(f);
            b.faces[f.idx()].shell = sh;
        }
    }
    !emap.is_empty()
}

/// Split a line edge a hair from its start: a sliver edge, `1e-9` of the
/// original length, with its own vertex.
pub fn tiny_edge(b: &mut Body) -> bool {
    let pick = b.edge_ids().find(|&e| {
        matches!(b.edge(e).curve, ck_geom::curve::Curve::Line { .. })
            && !b.is_seam(e)
            && b.edge_coedges(e).len() == 2
    });
    let Some(e) = pick else { return false };
    let ed = b.edge(e);
    let t = ed.t0 + (ed.t1 - ed.t0) * 1e-9;
    split_edge(b, e, t);
    true
}
