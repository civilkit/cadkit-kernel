// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/heal/src/upgrade/shell_sewing.rs
// and crates/heal/src/fix/shell.rs.
//! Sewing: pair coincident free edges into one two-sided edge, then merge
//! the shells (and lumps) the sewn faces belonged to.

use crate::Fixed;
use ck_geom::curve::Curve;
use ck_topo::body::{Body, EdgeId, FaceId, ShellId, VertexId};
use ck_topo::edit::gc;
use std::collections::HashMap;

fn pt(c: &Curve, t0: f64, t1: f64, f: f64) -> ck_geom::Vec3 {
    c.eval(t0 + (t1 - t0) * f)
}

/// Do the two edges (already sharing their vertex pair) trace the same
/// curve? Some(true) = same direction, Some(false) = opposite, None = no.
pub(crate) fn coincide(b: &Body, ea: EdgeId, eb: EdgeId, tol: f64) -> Option<bool> {
    let (a, c) = (b.edge(ea), b.edge(eb));
    let lim = 5.0 * tol;
    let fr = [0.0, 0.25, 0.5, 0.75, 1.0];
    let same = fr
        .iter()
        .all(|&f| pt(&a.curve, a.t0, a.t1, f).dist(pt(&c.curve, c.t0, c.t1, f)) <= lim);
    if same {
        return Some(true);
    }
    let opp = fr
        .iter()
        .all(|&f| pt(&a.curve, a.t0, a.t1, f).dist(pt(&c.curve, c.t0, c.t1, 1.0 - f)) <= lim);
    opp.then_some(false)
}

/// Sew pairs of free edges. Returns the faces joined, per pair.
pub fn sew_free_edges(b: &mut Body, tol: f64, out: &mut Vec<Fixed>) -> Vec<(FaceId, FaceId)> {
    let mut groups: HashMap<(VertexId, VertexId), Vec<EdgeId>> = HashMap::new();
    for e in b.edge_ids() {
        if b.edge_coedges(e).len() == 1 {
            let ed = b.edge(e);
            groups
                .entry((ed.v0.min(ed.v1), ed.v0.max(ed.v1)))
                .or_default()
                .push(e);
        }
    }
    let mut keys: Vec<_> = groups.keys().copied().collect();
    keys.sort();
    let mut joined = vec![];
    for k in keys {
        let mut es = groups[&k].clone();
        es.sort();
        let mut used = vec![false; es.len()];
        for i in 0..es.len() {
            if used[i] {
                continue;
            }
            for j in i + 1..es.len() {
                if used[j] {
                    continue;
                }
                let Some(same) = coincide(b, es[i], es[j], tol) else {
                    continue;
                };
                used[i] = true;
                used[j] = true;
                let (ea, eb) = (es[i], es[j]);
                let cb = b.edge_coedges(eb)[0];
                let ca = b.edge_coedges(ea)[0];
                let keep_pc = same && {
                    let (a, c) = (b.edge(ea), b.edge(eb));
                    (a.t0 - c.t0).abs() < 1e-12
                        && (a.t1 - c.t1).abs() < 1e-12
                        && pt(&a.curve, a.t0, a.t1, 0.5).dist(pt(&c.curve, c.t0, c.t1, 0.5)) < 1e-9
                };
                b.edges[eb.idx()].coedges.retain(|&c| c != cb);
                b.edges[ea.idx()].coedges.push(cb);
                let co = &mut b.coedges[cb.idx()];
                co.edge = ea;
                if !same {
                    co.rev = !co.rev;
                }
                if !keep_pc {
                    co.pcurve = None;
                }
                b.edges[eb.idx()].alive = false;
                let (fa, fb) = (b.coedge_face(ca), b.coedge_face(cb));
                joined.push((fa, fb));
                out.push(Fixed {
                    code: "EDGE_SEWN",
                    entity: format!("edge#{}", ea.idx()),
                    detail: format!(
                        "free edge#{} sewn ({}|{}){}",
                        eb.idx(),
                        b.face_id_str(fa),
                        b.face_id_str(fb),
                        if same { "" } else { ", direction reversed" }
                    ),
                });
                break;
            }
        }
    }
    gc(b);
    joined
}

fn root(p: &mut HashMap<ShellId, ShellId>, s: ShellId) -> ShellId {
    let q = p[&s];
    if q == s {
        return s;
    }
    let r = root(p, q);
    p.insert(s, r);
    r
}

/// Split every shell into its edge-connected components (extra components
/// become their own shells in the same lump, i.e. voids stay voids).
fn split_disconnected(b: &mut Body, out: &mut Vec<Fixed>) {
    for s in b.shell_ids().collect::<Vec<_>>() {
        let faces: Vec<FaceId> = b.shell(s).faces.iter().copied().filter(|&f| b.face(f).alive).collect();
        let mut seen: std::collections::HashSet<FaceId> = Default::default();
        let mut comps: Vec<Vec<FaceId>> = vec![];
        for &seed in &faces {
            if !seen.insert(seed) {
                continue;
            }
            let (mut comp, mut stack) = (vec![seed], vec![seed]);
            while let Some(f) = stack.pop() {
                for g in b.face_neighbors(f) {
                    if faces.contains(&g) && seen.insert(g) {
                        comp.push(g);
                        stack.push(g);
                    }
                }
            }
            comps.push(comp);
        }
        for comp in comps.into_iter().skip(1) {
            let lump = b.shell(s).lump;
            let ns = b.add_shell(lump);
            for f in comp {
                b.shells[s.idx()].faces.retain(|&x| x != f);
                b.shells[ns.idx()].faces.push(f);
                b.faces[f.idx()].shell = ns;
            }
            out.push(Fixed { code: "SHELL_SPLIT", entity: format!("shell#{}", s.idx()), detail: format!("disconnected component moved to shell#{}", ns.idx()) });
        }
    }
}

/// Merge the shells of sewn faces (and their lumps).
pub fn merge_shells(b: &mut Body, joined: &[(FaceId, FaceId)], out: &mut Vec<Fixed>) {
    let mut p: HashMap<ShellId, ShellId> = b.shell_ids().map(|s| (s, s)).collect();
    for &(fa, fb) in joined {
        let (ra, rb) = (
            root(&mut p, b.face(fa).shell),
            root(&mut p, b.face(fb).shell),
        );
        if ra != rb {
            let (keep, gone) = (ra.min(rb), ra.max(rb));
            p.insert(gone, keep);
        }
    }
    // shells left without faces (e.g. after their faces were sewn elsewhere)
    for s in b.shell_ids().collect::<Vec<_>>() {
        if b.shell(s).faces.iter().all(|&f| !b.face(f).alive) {
            let l = b.shell(s).lump;
            b.shells[s.idx()].alive = false;
            b.lumps[l.idx()].shells.retain(|&x| x != s);
            out.push(Fixed { code: "SHELL_REMOVED", entity: format!("shell#{}", s.idx()), detail: "empty shell dropped".into() });
            if b.lump(l).shells.iter().all(|&x| !b.shell(x).alive) {
                b.lumps[l.idx()].alive = false;
            }
        }
    }
    let ss: Vec<ShellId> = b.shell_ids().collect();
    for s in ss {
        let r = root(&mut p, s);
        if r == s {
            continue;
        }
        let faces = std::mem::take(&mut b.shells[s.idx()].faces);
        for &f in &faces {
            b.faces[f.idx()].shell = r;
        }
        b.shells[r.idx()].faces.extend(faces);
        b.shells[s.idx()].alive = false;
        let (ls, lr) = (b.shell(s).lump, b.shell(r).lump);
        b.lumps[ls.idx()].shells.retain(|&x| x != s);
        out.push(Fixed {
            code: "SHELL_MERGED",
            entity: format!("shell#{}", s.idx()),
            detail: format!("merged into shell#{}", r.idx()),
        });
        if ls != lr {
            // the rest of the source lump (voids) travels with the merge
            for t in std::mem::take(&mut b.lumps[ls.idx()].shells) {
                b.shells[t.idx()].lump = lr;
                b.lumps[lr.idx()].shells.push(t);
            }
            b.lumps[ls.idx()].alive = false;
            out.push(Fixed {
                code: "LUMP_MERGED",
                entity: format!("lump#{}", ls.idx()),
                detail: format!("merged into lump#{}", lr.idx()),
            });
        }
    }
    if !joined.is_empty() {
        split_disconnected(b, out);
    }
}
