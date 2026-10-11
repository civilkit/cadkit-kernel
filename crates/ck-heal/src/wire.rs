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

/// Budgeted search for closed chains through every coedge (forward uses
/// tried before flipped ones); up to 8 solutions are collected.
#[allow(clippy::too_many_arguments)]
fn dfs(
    b: &Body,
    start: ck_topo::body::VertexId,
    end: ck_topo::body::VertexId,
    rest: &mut Vec<CoedgeId>,
    cur: &mut Vec<(CoedgeId, bool)>,
    budget: &mut usize,
    flips: usize,
    sols: &mut Vec<Vec<(CoedgeId, bool)>>,
) {
    if rest.is_empty() {
        if end == start {
            sols.push(cur.clone());
        }
        return;
    }
    if *budget == 0 || sols.len() >= 8 {
        return;
    }
    *budget -= 1;
    for i in 0..rest.len() {
        let c = rest[i];
        for flip in [false, true] {
            if flip && flips == 0 {
                continue;
            }
            let (from, to) = if flip { (b.coedge_end(c), b.coedge_start(c)) } else { (b.coedge_start(c), b.coedge_end(c)) };
            if from != end {
                continue;
            }
            rest.remove(i);
            cur.push((c, flip));
            dfs(b, start, to, rest, cur, budget, flips - flip as usize, sols);
            cur.pop();
            rest.insert(i, c);
        }
    }
}

/// uv discontinuities along a chain (flipped coedges run their pcurve backwards).
fn seq_gaps(b: &Body, seq: &[(CoedgeId, bool)]) -> usize {
    let ends = |&(c, flip): &(CoedgeId, bool)| {
        let (t0, t1) = b.coedge_params(c);
        let (ts, te) = if flip { (t1, t0) } else { (t0, t1) };
        (b.coedge_uv(c, ts, None), b.coedge_uv(c, te, None))
    };
    (0..seq.len())
        .filter(|&i| {
            let (a, e) = (ends(&seq[i]).1, ends(&seq[(i + 1) % seq.len()]).0);
            (a - e).len() > 1e-6 * (1.0 + a.len())
        })
        .count()
}

/// Chain the coedges into closed wires covering every coedge: one wire if
/// the search finds it, else greedy sub-chains. None when a piece stays open.
/// Fewest coedge flips wins (a flip swaps which side of the edge a coedge
/// uses); among ties the chain whose pcurves meet in (u,v).
fn chain(b: &Body, cs: &[CoedgeId]) -> Option<Vec<Vec<(CoedgeId, bool)>>> {
    for flips in 0..=cs.len().min(3) {
        let mut sols = vec![];
        for first_flip in [false, true] {
            if first_flip && flips == 0 {
                continue;
            }
            let (start, end) = if first_flip { (b.coedge_end(cs[0]), b.coedge_start(cs[0])) } else { (b.coedge_start(cs[0]), b.coedge_end(cs[0])) };
            let mut rest: Vec<CoedgeId> = cs[1..].to_vec();
            let mut cur = vec![(cs[0], first_flip)];
            let mut budget = 4_000;
            dfs(b, start, end, &mut rest, &mut cur, &mut budget, flips - first_flip as usize, &mut sols);
        }
        if let Some(best) = sols.into_iter().min_by_key(|s| seq_gaps(b, s)) {
            return Some(vec![best]);
        }
    }
    let mut rest: Vec<CoedgeId> = cs.to_vec();
    let mut out = vec![];
    while !rest.is_empty() {
        let first = rest.remove(0);
        let start = b.coedge_start(first);
        let mut cur = vec![(first, false)];
        let mut end = b.coedge_end(first);
        while let Some(i) = rest.iter().position(|&c| b.coedge_start(c) == end) {
            let c = rest.remove(i);
            end = b.coedge_end(c);
            cur.push((c, false));
            if end == start {
                break;
            }
        }
        if end != start {
            return None;
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
