// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/heal/src/fix/small_face.rs
// and crates/heal/src/fix/wireframe.rs (degenerate edge removal).
//! Degenerate edge and face removal. An edge shorter than the tolerance
//! whose ends were merged is dropped from every loop; a face left without
//! a loop is deleted (never the last face of a shell, as in brepkit).

use crate::Fixed;
use ck_topo::body::{Body, EdgeId, FaceId};
use ck_topo::edit::{delete_face, gc};

pub fn remove_degenerate(b: &mut Body, tol: f64, out: &mut Vec<Fixed>) -> usize {
    let mut n = 0;
    let es: Vec<EdgeId> = b.edge_ids().collect();
    let mut touched: Vec<FaceId> = vec![];
    for e in es {
        if b.edge(e).v0 != b.edge(e).v1 {
            continue;
        }
        let len = b.edge_length(e);
        if len >= tol {
            continue;
        }
        for c in b.edge_coedges(e) {
            let l = b.coedge(c).lp;
            b.loops[l.idx()].coedges.retain(|&x| x != c);
            b.coedges[c.idx()].alive = false;
            let f = b.lp(l).face;
            if !touched.contains(&f) {
                touched.push(f);
            }
        }
        b.edges[e.idx()].alive = false;
        out.push(Fixed {
            code: "EDGE_REMOVED",
            entity: format!("edge#{}", e.idx()),
            detail: format!("degenerate edge (length {len:.3e} < {tol:.1e}) dropped"),
        });
        n += 1;
    }
    for &f in &touched {
        if !b.face(f).alive {
            continue;
        }
        let mut outer_gone = false;
        for (i, l) in b.face(f).loops.clone().into_iter().enumerate() {
            if b.lp(l).coedges.is_empty() {
                b.loops[l.idx()].alive = false;
                b.faces[f.idx()].loops.retain(|&x| x != l);
                outer_gone |= i == 0;
            }
        }
        if outer_gone || b.face(f).loops.is_empty() {
            let sh = b.face(f).shell;
            if b.shell(sh)
                .faces
                .iter()
                .filter(|&&g| b.face(g).alive)
                .count()
                > 1
            {
                out.push(Fixed {
                    code: "FACE_REMOVED",
                    entity: b.face_id_str(f),
                    detail: "small face collapsed below tolerance".into(),
                });
                delete_face(b, f);
                n += 1;
            }
        }
    }
    // zero-area bigons: a two-edge loop whose edges trace the same curve
    for f in touched {
        if !b.face(f).alive || b.face(f).loops.len() != 1 {
            continue;
        }
        let cs: Vec<_> = b.lp(b.face(f).loops[0]).coedges.iter().copied().filter(|&c| b.coedge(c).alive).collect();
        if cs.len() != 2 {
            continue;
        }
        let (ea, eb) = (b.coedge(cs[0]).edge, b.coedge(cs[1]).edge);
        let (a, c) = (b.edge(ea), b.edge(eb));
        let pair = |e: &ck_topo::body::Edge| (e.v0.min(e.v1), e.v0.max(e.v1));
        let sh = b.face(f).shell;
        if ea != eb && pair(a) == pair(c) && crate::sew::coincide(b, ea, eb, tol).is_some() && b.shell(sh).faces.iter().filter(|&&g| b.face(g).alive).count() > 1 {
            out.push(Fixed { code: "FACE_REMOVED", entity: b.face_id_str(f), detail: "zero-area two-edge face dropped".into() });
            delete_face(b, f);
            n += 1;
        }
    }
    gc(b);
    n
}
