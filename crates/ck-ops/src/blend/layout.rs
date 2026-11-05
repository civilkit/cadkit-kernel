//! Re-lays a blended body in the original kernel's in-place arena numbering (`#n` wire keys).

use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId};

/// One selected blend: its face id, the edge's two faces (forward face
/// first) and end points (v0, v1) on the input body.
#[derive(Debug, Clone)]
pub struct BlendRef {
    pub name: String,
    pub faces: [String; 2],
    pub ends: [Vec3; 2],
}

/// Re-lay `out` (the blended body) in the original's in-place arena, so
/// [`Body::wire_numbered`] gives the oracle `#n` (bracket F11 / F12):
/// - an input edge that survives unchanged keeps its slot;
/// - an input edge the blend trims is split in parameter order (in
///   ascending input slot order): its first piece keeps the slot, each
///   further piece takes the next new slot; pieces the blend removed stay
///   as dead slots (F11: C0|S.e9 #2 trimmed at its end keeps #2, C0|S.e8
///   #1 trimmed at its start dies and its kept piece is #76, S.e10|S.e9 #6
///   trimmed at both ends -> #80 kept, #81 dead);
/// - removed input edges stay dead slots;
/// - then per blend (selection order) its edges: spring on the forward
///   face, spring on the other face, end arc at v0, end arc at v1.
pub fn in_place_layout(input: &Body, out: &Body, refs: &[BlendRef], tol: f64) -> Body {
    in_place_layout_opts(input, out, refs, tol, &LayoutOpts::default())
}

/// Overrides of the in-place numbering (vertex chamfer: three slots are
/// reserved right after the input's and handed to the trimmed edges by their
/// vertex-edge order).
#[derive(Debug, Clone, Default)]
pub struct LayoutOpts {
    /// Dead / reserved slots after the input's (None: two per junction).
    pub reserve: Option<usize>,
    /// (input slot, slot of its kept piece when the edge is trimmed at its start).
    pub piece_slots: Vec<(usize, usize)>,
    /// Vertex-blend corners (three fillets meeting in a sphere patch): no
    /// junction (miter / deferred spring / slot) there.
    pub corners: Vec<Vec3>,
}

pub fn in_place_layout_opts(input: &Body, out: &Body, refs: &[BlendRef], tol: f64, opts: &LayoutOpts) -> Body {
    let (in_slots, n_in) = input.wire_numbered();
    let out_chains = out.wire_chains();
    let names = |b: &Body, e: EdgeId| {
        let mut v: Vec<String> = b.edge_faces(e).iter().map(|&f| b.face_id_str(f)).collect();
        v.sort();
        v
    };
    let ends = |b: &Body, ch: &[EdgeId]| (b.vertex(b.edge(ch[0]).v0).p, b.vertex(b.edge(*ch.last().unwrap()).v1).p);
    let near = |p: Vec3, q: Vec3| p.dist(q) <= tol * 10.0;
    let on = |b: &Body, ch: &[EdgeId], p: Vec3| {
        ch.iter().any(|&e| {
            let ed = b.edge(e);
            crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1 <= tol * 10.0
        })
    };
    let mut slot_of: Vec<Option<usize>> = vec![None; out_chains.len()];
    let mut used_in = vec![false; in_slots.len()];
    // 1. unchanged edges
    for (k, ch) in out_chains.iter().enumerate() {
        let (p, q) = ends(out, ch);
        let nm = names(out, ch[0]);
        let mid = out.chain_mid(ch);
        for (i, (s, ich)) in in_slots.iter().enumerate() {
            if used_in[i] || names(input, ich[0]) != nm {
                continue;
            }
            let (a, b2) = ends(input, ich);
            if ((near(a, p) && near(b2, q)) || (near(a, q) && near(b2, p))) && near(input.chain_mid(ich), mid) {
                slot_of[k] = Some(*s);
                used_in[i] = true;
                break;
            }
        }
    }
    // 2. trimmed edges, split in parameter order
    // Oracle (box fillet chains): the new slots follow the touched input
    // edges in ascending slot order; a selected edge uses one slot per end
    // it shares with another selected edge (a junction), a trimmed
    // neighbour one per window it opens (a rail trimmed at one end: one, an
    // edge trimmed at both ends: two).
    let near_pt = |p: Vec3, q: Vec3| p.dist(q) <= tol * 10.0;
    let at_corner = |p: Vec3| opts.corners.iter().any(|&q| near_pt(p, q));
    let junction_ends = |k: usize| -> usize {
        (0..2)
            .filter(|&e| !at_corner(refs[k].ends[e]) && (0..refs.len()).any(|j| j != k && refs[j].ends.iter().any(|&q| near_pt(refs[k].ends[e], q))))
            .count()
    };
    let mut sel_cost: Vec<usize> = vec![0; in_slots.len()];
    for (k, r) in refs.iter().enumerate() {
        let mut want = r.faces.to_vec();
        want.sort();
        for (i, (_, ich)) in in_slots.iter().enumerate() {
            let (a, b2) = ends(input, ich);
            if names(input, ich[0]) == want
                && ((near(a, r.ends[0]) && near(b2, r.ends[1])) || (near(a, r.ends[1]) && near(b2, r.ends[0])))
            {
                sel_cost[i] = junction_ends(k);
                break;
            }
        }
    }
    let mut next = n_in + opts.reserve.unwrap_or(0);
    // ascending input slot order (oracle: the new slots follow it)
    let mut slot_order: Vec<usize> = (0..in_slots.len()).collect();
    slot_order.sort_by_key(|&i| in_slots[i].0);
    for i in slot_order {
        let (s, ich) = &in_slots[i];
        if opts.reserve.is_none() {
            next += sel_cost[i];
        }
        if used_in[i] || ich.len() != 1 {
            continue;
        }
        let ie = input.edge(ich[0]);
        let nm = names(input, ich[0]);
        let pieces: Vec<usize> = (0..out_chains.len())
            .filter(|&k| slot_of[k].is_none() && names(out, out_chains[k][0]) == nm && {
                let (p, q) = ends(out, &out_chains[k]);
                on(input, ich, p) && on(input, ich, q) && on(input, ich, out.chain_mid(&out_chains[k]))
            })
            .collect();
        if pieces.is_empty() {
            continue;
        }
        let param = |p: Vec3| crate::boolean::imprint::dist_in_range(&ie.curve, (ie.t0, ie.t1), p).0;
        let mut cuts: Vec<f64> = vec![];
        for &k in &pieces {
            let (p, q) = ends(out, &out_chains[k]);
            for t in [param(p), param(q)] {
                let slack = crate::intersect::param_slack(&ie.curve, t, tol * 10.0);
                if t > ie.t0 + slack && t < ie.t1 - slack && !cuts.iter().any(|&c| (c - t).abs() <= slack) {
                    cuts.push(t);
                }
            }
        }
        cuts.sort_by(|a, b| a.total_cmp(b));
        let mut bounds = vec![ie.t0];
        bounds.extend(cuts);
        bounds.push(ie.t1);
        for (j, w) in bounds.windows(2).enumerate() {
            let slot = if j == 0 {
                *s
            } else if let Some(&(_, forced)) = opts.piece_slots.iter().find(|(is, _)| is == s) {
                forced
            } else {
                next += 1;
                next - 1
            };
            let tm = 0.5 * (w[0] + w[1]);
            if let Some(&k) = pieces.iter().find(|&&k| {
                let t = param(out.chain_mid(&out_chains[k]));
                t > w[0] && t < w[1] && slot_of[k].is_none()
            }) {
                let _ = tm;
                slot_of[k] = Some(slot);
            }
        }
        used_in[i] = true;
    }
    // 3. blend edges. Oracle (chains of box fillets, probes filletChain3 /
    // filletTop4): per blend in selection order its edges [spring on the
    // forward face, spring on the other face, end arc at v0, end arc at v1]
    // - except that where another selected edge meets the blend's v0 the
    // spring on the COMMON face is deferred; the junctions follow, by
    // blend pair (i<j) lexicographically: the miter edge B<i>|B<j>, then
    // the deferred spring of the blend that starts at the junction.
    let near_pt = |p: Vec3, q: Vec3| p.dist(q) <= tol * 10.0;
    let meets = |a: usize, ea: usize, b2: usize| !at_corner(refs[a].ends[ea]) && refs[b2].ends.iter().any(|&q| near_pt(refs[a].ends[ea], q));
    let common_face = |a: usize, b2: usize| refs[a].faces.iter().find(|f| refs[b2].faces.contains(f)).cloned();
    // deferred spring face of blend a (junction at its v0)
    let deferred: Vec<Option<String>> = (0..refs.len())
        .map(|a| (0..refs.len()).filter(|&o| o != a && meets(a, 0, o)).find_map(|o| common_face(a, o)))
        .collect();
    let is_blend = |n: &String| refs.iter().any(|r| &r.name == n);
    for (ri, r) in refs.iter().enumerate() {
        let mut mine: Vec<(usize, usize, f64)> = vec![];
        for (k, ch) in out_chains.iter().enumerate() {
            if slot_of[k].is_some() {
                continue;
            }
            let nm = names(out, ch[0]);
            if !nm.contains(&r.name) {
                continue;
            }
            let other = nm.iter().find(|n| **n != r.name).cloned().unwrap_or_default();
            if is_blend(&other) || deferred[ri].as_deref() == Some(other.as_str()) {
                continue; // miter / deferred spring: the junction pass
            }
            let (p, q) = ends(out, ch);
            let d0 = p.dist(r.ends[0]).min(q.dist(r.ends[0]));
            let d1 = p.dist(r.ends[1]).min(q.dist(r.ends[1]));
            let rank = if other == r.faces[0] {
                0
            } else if other == r.faces[1] {
                1
            } else if d0 <= d1 {
                2
            } else {
                3
            };
            mine.push((rank, k, d0.min(d1)));
        }
        mine.sort_by(|a, b| a.0.cmp(&b.0).then(a.2.total_cmp(&b.2)));
        for (_, k, _) in mine {
            slot_of[k] = Some(next);
            next += 1;
        }
    }
    for i in 0..refs.len() {
        for j in i + 1..refs.len() {
            if !(meets(i, 0, j) || meets(i, 1, j)) {
                continue;
            }
            let find = |x: &str, y: &str, slot_of: &Vec<Option<usize>>| {
                (0..out_chains.len()).find(|&k| {
                    slot_of[k].is_none() && {
                        let nm = names(out, out_chains[k][0]);
                        nm.len() == 2 && nm.iter().any(|n| n == x) && nm.iter().any(|n| n == y)
                    }
                })
            };
            if let Some(k) = find(&refs[i].name, &refs[j].name, &slot_of) {
                slot_of[k] = Some(next);
                next += 1;
            }
            let owner = if meets(i, 0, j) { Some(i) } else if meets(j, 0, i) { Some(j) } else { None };
            if let Some(o) = owner {
                if let Some(f) = &deferred[o] {
                    if let Some(k) = find(&refs[o].name, f, &slot_of) {
                        slot_of[k] = Some(next);
                        next += 1;
                    }
                }
            }
        }
    }
    // 4. anything else, in out order
    for k in 0..out_chains.len() {
        if slot_of[k].is_none() {
            slot_of[k] = Some(next);
            next += 1;
        }
    }
    // ---- write the arena: slot order, dead placeholders, seams last
    let mut res = Body::new();
    let lump = res.add_lump();
    let shell = res.add_shell(lump);
    let mut vmap: Vec<Option<ck_topo::body::VertexId>> = vec![None; out.vertices.len()];
    for v in out.vertex_ids() {
        vmap[v.idx()] = Some(res.add_vertex(out.vertex(v).p));
    }
    let mut by_slot: Vec<Option<usize>> = vec![None; next];
    for (k, s) in slot_of.iter().enumerate() {
        by_slot[s.unwrap()] = Some(k);
    }
    let mut emap: Vec<Option<EdgeId>> = vec![None; out.edges.len()];
    let dummy = res.add_vertex(Vec3::ZERO);
    res.vertices[dummy.idx()].alive = false;
    for s in 0..next {
        match by_slot[s] {
            Some(k) => {
                for &e in &out_chains[k] {
                    let ed = out.edge(e);
                    emap[e.idx()] =
                        Some(res.add_edge(ed.curve.clone(), ed.t0, ed.t1, vmap[ed.v0.idx()].unwrap(), vmap[ed.v1.idx()].unwrap()));
                }
            }
            None => {
                let d = res.add_edge(Curve::Line { o: Vec3::ZERO, d: Vec3::X }, 0.0, 0.0, dummy, dummy);
                res.edges[d.idx()].alive = false;
            }
        }
    }
    for e in out.edge_ids() {
        if emap[e.idx()].is_none() {
            let ed = out.edge(e);
            emap[e.idx()] = Some(res.add_edge(ed.curve.clone(), ed.t0, ed.t1, vmap[ed.v0.idx()].unwrap(), vmap[ed.v1.idx()].unwrap()));
        }
    }
    for f in out.face_ids() {
        let face = out.face(f);
        let nf = res.add_face(shell, face.surface.clone(), face.sense, face.name.clone());
        for &l in &face.loops {
            let nl = res.add_loop(nf);
            for &c in &out.lp(l).coedges {
                let ce = out.coedge(c).clone();
                res.add_coedge(nl, emap[ce.edge.idx()].unwrap(), ce.rev, ce.pcurve);
            }
        }
    }
    ck_topo::region::rebuild_lumps(&mut res);
    res
}
