// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/heal/src/fix/wire.rs
// (fix_reorder, fix_connected, fix_direction) and crates/heal/src/analysis/wire_order.rs.
//! Wire (loop) repair: chain coedges head-to-tail, flip coedges that run
//! against the chain, split a loop that closes early into several loops.

use crate::Fixed;
use ck_topo::body::{Body, CoedgeId, FaceId, LoopId};

/// Is the loop a closed head-to-tail cycle in its stored order?
fn closed(b: &Body, cs: &[CoedgeId]) -> bool {
    !cs.is_empty()
        && (0..cs.len()).all(|i| b.coedge_end(cs[i]) == b.coedge_start(cs[(i + 1) % cs.len()]))
}

/// Greedy chaining. Returns the closed sub-chains (coedge, flip?) covering
/// every coedge, or None when some piece stays open.
fn chain(b: &Body, cs: &[CoedgeId]) -> Option<Vec<Vec<(CoedgeId, bool)>>> {
    let mut rest: Vec<CoedgeId> = cs.to_vec();
    let mut out = vec![];
    while !rest.is_empty() {
        let first = rest.remove(0);
        let start = b.coedge_start(first);
        let mut cur = vec![(first, false)];
        let mut end = b.coedge_end(first);
        while end != start || cur.len() == 0 {
            if let Some(i) = rest.iter().position(|&c| b.coedge_start(c) == end) {
                let c = rest.remove(i);
                end = b.coedge_end(c);
                cur.push((c, false));
            } else if let Some(i) = rest.iter().position(|&c| b.coedge_end(c) == end) {
                let c = rest.remove(i);
                end = b.coedge_start(c);
                cur.push((c, true));
            } else {
                return None;
            }
        }
        out.push(cur);
    }
    Some(out)
}

/// Repair every open loop. Returns the number of loops fixed.
pub fn fix_loops(b: &mut Body, out: &mut Vec<Fixed>) -> usize {
    let mut n = 0;
    let faces: Vec<FaceId> = b.face_ids().collect();
    for f in faces {
        for l in b.face(f).loops.clone() {
            if !b.lp(l).alive {
                continue;
            }
            let cs: Vec<CoedgeId> = b
                .lp(l)
                .coedges
                .iter()
                .copied()
                .filter(|&c| b.coedge(c).alive)
                .collect();
            if cs.is_empty() || closed(b, &cs) {
                continue;
            }
            let Some(chains) = chain(b, &cs) else {
                out.push(Fixed {
                    code: "LOOP_UNFIXED",
                    entity: b.face_id_str(f),
                    detail: "loop edges do not chain into closed wires".into(),
                });
                continue;
            };
            let flips = chains.iter().flatten().filter(|x| x.1).count();
            let pieces = chains.len();
            let reordered = chains[0]
                .iter()
                .map(|x| x.0)
                .ne(cs.iter().copied().take(chains[0].len()));
            for (k, ch) in chains.into_iter().enumerate() {
                let target: LoopId = if k == 0 { l } else { b.add_loop(f) };
                for &(c, flip) in &ch {
                    if flip {
                        b.coedges[c.idx()].rev = !b.coedges[c.idx()].rev;
                    }
                    b.coedges[c.idx()].lp = target;
                }
                b.loops[target.idx()].coedges = ch.into_iter().map(|x| x.0).collect();
            }
            out.push(Fixed {
                code: "LOOP_CHAINED",
                entity: b.face_id_str(f),
                detail: format!(
                    "loop re-chained ({} coedge flipped, {} loop(s){})",
                    flips,
                    pieces,
                    if reordered { ", reordered" } else { "" }
                ),
            });
            n += 1;
        }
    }
    n
}
