// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/check/src/validate/shell.rs
//! Closed-manifold shell, opposite-orientation pairing, connectivity,
//! Euler-Poincare.

use crate::{edge_ent, face_ent, Issue};
use ck_topo::body::{Body, EdgeId, FaceId};
use std::collections::{HashMap, HashSet};

pub fn check(b: &Body, out: &mut Vec<Issue>) {
    for e in b.edge_ids() {
        let cs = b.edge_coedges(e);
        match cs.len() {
            0 => out.push(Issue { code: "EDGE_UNUSED", entity: edge_ent(b, e), detail: "edge has no face use".into() }),
            1 => out.push(Issue { code: "SHELL_OPEN", entity: edge_ent(b, e), detail: "free (boundary) edge: used by 1 face".into() }),
            2 => {
                if b.coedge(cs[0]).rev == b.coedge(cs[1]).rev {
                    out.push(Issue {
                        code: "SHELL_ORIENTATION",
                        entity: edge_ent(b, e),
                        detail: "two face uses run the same direction".into(),
                    });
                }
            }
            n => out.push(Issue { code: "SHELL_NONMANIFOLD", entity: edge_ent(b, e), detail: format!("edge used by {n} faces") }),
        }
    }
    for s in b.shell_ids() {
        let faces: Vec<FaceId> = b.shell(s).faces.iter().copied().filter(|&f| b.face(f).alive).collect();
        if faces.is_empty() {
            out.push(Issue { code: "SHELL_EMPTY", entity: format!("shell#{}", s.idx()), detail: "shell has no faces".into() });
            continue;
        }
        connectivity(b, &faces, out);
    }
    euler_components(b, out);
}

/// Euler-Poincare per edge-connected face component of the whole body.
/// A surface patch (fill / knit) may sit in its own single-face shell next to
/// the shell it caps, so a per-shell count would see a bare disc; the
/// component spanning both is the closed manifold. Components are found
/// across shells by shared edges.
fn euler_components(b: &Body, out: &mut Vec<Issue>) {
    let faces: Vec<FaceId> = b
        .shell_ids()
        .flat_map(|s| b.shell(s).faces.iter().copied().collect::<Vec<_>>())
        .filter(|&f| b.face(f).alive)
        .collect();
    let mut by_edge: HashMap<EdgeId, Vec<usize>> = HashMap::new();
    for (i, &f) in faces.iter().enumerate() {
        for e in b.face_edges(f) {
            by_edge.entry(e).or_default().push(i);
        }
    }
    let mut seen = vec![false; faces.len()];
    let mut comp_no = 0;
    for start in 0..faces.len() {
        if seen[start] {
            continue;
        }
        seen[start] = true;
        let mut comp = vec![];
        let mut stack = vec![start];
        while let Some(i) = stack.pop() {
            comp.push(faces[i]);
            for e in b.face_edges(faces[i]) {
                for &j in &by_edge[&e] {
                    if !seen[j] {
                        seen[j] = true;
                        stack.push(j);
                    }
                }
            }
        }
        euler(b, &comp, comp_no, out);
        comp_no += 1;
    }
}

fn connectivity(b: &Body, faces: &[FaceId], out: &mut Vec<Issue>) {
    if faces.len() <= 1 {
        return;
    }
    let mut by_edge: HashMap<EdgeId, Vec<usize>> = HashMap::new();
    for (i, &f) in faces.iter().enumerate() {
        for e in b.face_edges(f) {
            by_edge.entry(e).or_default().push(i);
        }
    }
    let mut seen = HashSet::from([0usize]);
    let mut stack = vec![0usize];
    while let Some(i) = stack.pop() {
        for e in b.face_edges(faces[i]) {
            for &j in by_edge.get(&e).map(|v| v.as_slice()).unwrap_or(&[]) {
                if seen.insert(j) {
                    stack.push(j);
                }
            }
        }
    }
    if seen.len() < faces.len() {
        let miss = (0..faces.len()).find(|i| !seen.contains(i)).unwrap();
        out.push(Issue {
            code: "SHELL_DISCONNECTED",
            entity: face_ent(b, faces[miss]),
            detail: format!("{} of {} faces reachable from the first face", seen.len(), faces.len()),
        });
    }
}

/// V - E + F - (L - F) = 2(1 - genus) for a closed orientable shell:
/// must be even and <= 2.
fn euler(b: &Body, faces: &[FaceId], idx: usize, out: &mut Vec<Issue>) {
    let mut es = HashSet::new();
    let mut vs = HashSet::new();
    let mut loops = 0i64;
    for &f in faces {
        loops += b.face(f).loops.len() as i64;
        for e in b.face_edges(f) {
            if es.insert(e) {
                vs.insert(b.edge(e).v0);
                vs.insert(b.edge(e).v1);
            }
        }
    }
    let f = faces.len() as i64;
    let chi = vs.len() as i64 - es.len() as i64 + f - (loops - f);
    if chi % 2 != 0 || chi > 2 {
        out.push(Issue {
            code: "EULER",
            entity: format!("component#{idx}"),
            detail: format!("V-E+F-(L-F) = {chi} (V={} E={} F={} L={})", vs.len(), es.len(), f, loops),
        });
    }
}
