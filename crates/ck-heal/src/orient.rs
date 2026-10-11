// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/heal/src/fix/face.rs
// (orientation), crates/heal/src/fix/shell.rs (face orientation propagation)
// and crates/heal/src/fix/solid.rs (inside-out solid).
//! Orientation repair: loops against their face sense, faces against their
//! neighbours, and an inside-out (negative signed volume) lump.

use crate::Fixed;
use ck_geom::surface::Surface;
use ck_topo::body::{Body, EdgeId, FaceId, ShellId};
use ck_topo::mass::{lump_faces, signed_volume};
use ck_topo::orient::{fix_planar_loops, flip_face, loop_uv_area, loops_consistent, reverse_loop};
use std::collections::{HashMap, HashSet};

/// Faces on an edge whose two uses run the same way.
fn conflict_faces(b: &Body) -> Vec<FaceId> {
    let mut out: Vec<FaceId> = vec![];
    for e in b.edge_ids() {
        let cs = b.edge_coedges(e);
        if cs.len() == 2 && b.coedge(cs[0]).rev == b.coedge(cs[1]).rev {
            for c in cs {
                let f = b.coedge_face(c);
                if !out.contains(&f) {
                    out.push(f);
                }
            }
        }
    }
    out
}

/// Planar: outer loop CCW (positive uv area w.r.t. sense) first, holes CW.
fn planar_loops_ok(b: &Body, f: FaceId) -> bool {
    let sg = if b.face(f).sense { 1.0 } else { -1.0 };
    b.face(f).loops.iter().enumerate().all(|(i, &l)| {
        let a = loop_uv_area(b, l) * sg;
        if i == 0 {
            a > 0.0
        } else {
            a < 0.0
        }
    })
}

/// Are the face's loops consistent with its sense?
fn loops_ok(b: &Body, f: FaceId) -> bool {
    if matches!(b.face(f).surface, Surface::Plane { .. }) {
        planar_loops_ok(b, f)
    } else {
        loops_consistent(b, f)
    }
}

/// Consecutive coedges of a loop whose face-surface (u,v) ends do not meet.
fn uv_gaps(b: &Body, f: FaceId) -> usize {
    let mut n = 0;
    for &l in &b.face(f).loops {
        let cs: Vec<_> = b.lp(l).coedges.iter().copied().filter(|&c| b.coedge(c).alive).collect();
        for i in 0..cs.len() {
            let (c, d) = (cs[i], cs[(i + 1) % cs.len()]);
            let (_, te) = b.coedge_params(c);
            let (ts, _) = b.coedge_params(d);
            let (a, e) = (b.coedge_uv(c, te, None), b.coedge_uv(d, ts, None));
            if (a - e).len() > 1e-6 * (1.0 + a.len()) {
                n += 1;
            }
        }
    }
    n
}

/// 0 when the face's loops are consistent with its sense and meet in (u,v).
fn loop_score(b: &Body, f: FaceId) -> usize {
    (!loops_ok(b, f)) as usize + uv_gaps(b, f)
}

/// A coedge running against its neighbour in a face whose loops are
/// inconsistent with the face sense (or do not meet in uv): flip just that
/// coedge when doing so makes the loops consistent again (a stray coedge,
/// not a wrong face).
fn fix_stray_coedges(b: &mut Body, out: &mut Vec<Fixed>) -> usize {
    let mut n = 0;
    for f in conflict_faces(b) {
        if b.face(f).loops.is_empty() || loop_score(b, f) == 0 {
            continue;
        }
        let cands: Vec<_> = b
            .face(f)
            .loops
            .iter()
            .flat_map(|&l| b.lp(l).coedges.clone())
            .filter(|&c| {
                let e = b.coedge(c).edge;
                let cs = b.edge_coedges(e);
                cs.len() == 2 && !b.is_seam(e) && cs.iter().any(|&o| o != c && b.coedge(o).rev == b.coedge(c).rev)
            })
            .collect();
        for c in cands {
            b.coedges[c.idx()].rev = !b.coedges[c.idx()].rev;
            if loop_score(b, f) == 0 {
                out.push(Fixed { code: "COEDGE_FLIPPED", entity: b.face_id_str(f), detail: "coedge ran against its neighbour and its loop".into() });
                n += 1;
                break;
            }
            b.coedges[c.idx()].rev = !b.coedges[c.idx()].rev;
        }
    }
    // a seam whose two uses agree: reverse one of them
    for e in b.edge_ids().collect::<Vec<_>>() {
        let cs = b.edge_coedges(e);
        if cs.len() == 2 && b.is_seam(e) && b.coedge(cs[0]).rev == b.coedge(cs[1]).rev {
            b.coedges[cs[1].idx()].rev = !b.coedges[cs[1].idx()].rev;
            out.push(Fixed { code: "COEDGE_FLIPPED", entity: format!("edge#{}", e.idx()), detail: "seam uses made opposite".into() });
            n += 1;
        }
    }
    n
}

/// Fix faces whose loops run against their sense (only faces that also
/// disagree with a neighbour are examined, so healthy faces stay put).
pub fn fix_loop_senses(b: &mut Body, out: &mut Vec<Fixed>) -> usize {
    let mut n = fix_stray_coedges(b, out);
    for f in conflict_faces(b) {
        if b.face(f).loops.is_empty() {
            continue;
        }
        let changed = if matches!(b.face(f).surface, Surface::Plane { .. }) {
            !planar_loops_ok(b, f) && fix_planar_loops(b, f)
        } else if !loops_consistent(b, f) {
            for l in b.face(f).loops.clone() {
                reverse_loop(b, l);
            }
            true
        } else {
            false
        };
        if changed {
            out.push(Fixed {
                code: "LOOP_REVERSED",
                entity: b.face_id_str(f),
                detail: "loop direction restored against face sense".into(),
            });
            n += 1;
        }
    }
    n
}

/// Make faces of each shell mutually consistent, then make every lump
/// outward (positive signed volume). Net flips are reported.
pub fn fix_orientation(b: &mut Body, out: &mut Vec<Fixed>) -> usize {
    let mut parity: HashMap<FaceId, bool> = HashMap::new();
    let mut flip = |b: &mut Body, f: FaceId| {
        flip_face(b, f);
        let p = parity.entry(f).or_insert(false);
        *p = !*p;
    };
    let shells: Vec<ShellId> = b.shell_ids().collect();
    for s in shells {
        let faces: HashSet<FaceId> = b
            .shell(s)
            .faces
            .iter()
            .copied()
            .filter(|&f| b.face(f).alive)
            .collect();
        let mut order: Vec<FaceId> = faces.iter().copied().collect();
        order.sort();
        let mut seen: HashSet<FaceId> = HashSet::new();
        for &seed in &order {
            if !seen.insert(seed) {
                continue;
            }
            let mut stack = vec![seed];
            while let Some(f) = stack.pop() {
                let es: Vec<EdgeId> = b.face_edges(f);
                for e in es {
                    let cs = b.edge_coedges(e);
                    if cs.len() != 2 {
                        continue;
                    }
                    let (f0, f1) = (b.coedge_face(cs[0]), b.coedge_face(cs[1]));
                    if f0 == f1 {
                        continue;
                    }
                    let other = if f0 == f { f1 } else { f0 };
                    if !faces.contains(&other) || !seen.insert(other) {
                        continue;
                    }
                    if b.coedge(cs[0]).rev == b.coedge(cs[1]).rev {
                        flip(b, other);
                    }
                    stack.push(other);
                }
            }
        }
    }
    // Per lump: edge-connected components. The largest |volume| is the
    // outer surface (outward, positive); any other component is a void
    // (inward, negative).
    let lumps: Vec<_> = b.lump_ids().collect();
    for l in lumps {
        let fs = lump_faces(b, l);
        if fs.is_empty() {
            continue;
        }
        let fset: HashSet<FaceId> = fs.iter().copied().collect();
        let mut seen: HashSet<FaceId> = HashSet::new();
        let mut comps: Vec<(Vec<FaceId>, f64)> = vec![];
        for &seed in &fs {
            if !seen.insert(seed) {
                continue;
            }
            let (mut comp, mut stack) = (vec![seed], vec![seed]);
            while let Some(f) = stack.pop() {
                for g in b.face_neighbors(f) {
                    if fset.contains(&g) && seen.insert(g) {
                        comp.push(g);
                        stack.push(g);
                    }
                }
            }
            let v = signed_volume(b, &comp);
            comps.push((comp, v));
        }
        let big = (0..comps.len()).max_by(|&i, &j| comps[i].1.abs().partial_cmp(&comps[j].1.abs()).unwrap()).unwrap();
        for (i, (comp, v)) in comps.iter().enumerate() {
            let want_pos = i == big;
            if (*v > 0.0) != want_pos && *v != 0.0 {
                for &f in comp {
                    flip(b, f);
                }
            }
        }
    }
    let mut net: Vec<FaceId> = parity.into_iter().filter(|x| x.1).map(|x| x.0).collect();
    net.sort();
    let mut n = 0;
    for l in b.lump_ids().collect::<Vec<_>>() {
        let fs = lump_faces(b, l);
        if fs.len() > 1 && fs.iter().all(|f| net.contains(f)) {
            out.push(Fixed {
                code: "SOLID_FLIPPED",
                entity: format!("lump#{}", b.lump_index(l)),
                detail: format!("inside-out solid: all {} faces flipped", fs.len()),
            });
            net.retain(|f| !fs.contains(f));
            n += fs.len();
        }
    }
    for f in net {
        out.push(Fixed {
            code: "FACE_FLIPPED",
            entity: b.face_id_str(f),
            detail: "face orientation made consistent with its neighbours".into(),
        });
        n += 1;
    }
    n
}
