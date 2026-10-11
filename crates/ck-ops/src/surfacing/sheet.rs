//! Sheet extraction: keep the named faces of a body as an open sheet.
//!
//! Pinned by oracle probe df1 (box F2 + `deleteFace F2.C1`): the result
//! surface (registered under the feature id) holds the 5 remaining faces
//! with their names; the deleted face's edges survive as OPEN edges of
//! their other face (`F2.S.e0#5`), and the edge list keeps the original
//! body's edge order (rails #0-3, bottom #4-7… — the copy walks the body's
//! edges in order, then the faces).
//!
//! HOW TO WIRE (ck-model, Gate B): feature `deleteFace {faces, mode:
//! "delete"}` → `sheet::sheet(&body, &faces)`; the result registers under
//! the feature id and the solid slot empties (the probe result is a
//! surface body). `mode: "heal"`/continuity modes are not part of this op.

use crate::surfacing::knit::BodyExt;
use ck_topo::body::{Body, EdgeId, FaceId, ShellId, VertexId};

/// Extract `faces` (to KEEP) from `body` into a fresh sheet body.
pub fn sheet(body: &Body, faces: &[FaceId]) -> Body {
    let keep: std::collections::HashSet<FaceId> = faces.iter().copied().collect();
    // edges used by any kept face, in body edge order
    let used: std::collections::HashSet<EdgeId> = faces
        .iter()
        .flat_map(|&f| body.face_edges(f))
        .filter(|&e| body.edge(e).alive)
        .collect();
    let mut out = Body::new();
    let lump = out.add_lump();
    let shell = out.add_shell(lump);
    // edges in body order, shared once
    let mut emap: std::collections::HashMap<EdgeId, EdgeId> = Default::default();
    let mut vmap: std::collections::HashMap<VertexId, VertexId> = Default::default();
    for e in body.edge_ids().collect::<Vec<_>>() {
        if !used.contains(&e) {
            continue;
        }
        let ed = body.edge(e);
        let v0 = *vmap.entry(ed.v0).or_insert_with(|| out.add_vertex(body.vertex(ed.v0).p));
        let v1 = *vmap.entry(ed.v1).or_insert_with(|| out.add_vertex(body.vertex(ed.v1).p));
        let ne = out.add_edge(ed.curve.clone(), ed.t0, ed.t1, v0, v1);
        emap.insert(e, ne);
    }
    // faces in body order
    for f in body.face_ids().collect::<Vec<_>>() {
        if !keep.contains(&f) {
            continue;
        }
        let face = body.face(f);
        let nf = out.add_face(shell, face.surface.clone(), face.sense, face.name.clone());
        for &lid in &face.loops {
            if !body.lp(lid).alive {
                continue;
            }
            let nloop = out.add_loop(nf);
            for &c in &body.lp(lid).coedges {
                let co = body.coedge(c);
                if !co.alive {
                    continue;
                }
                let Some(&ne) = emap.get(&co.edge) else { continue };
                out.add_coedge(nloop, ne, co.rev, co.pcurve.clone());
            }
        }
    }
    // one shell per connected group (like the knit result)
    assign_shells(&mut out);
    out
}

/// One shell per connected group of faces (shared edges), one lump.
fn assign_shells(b: &mut Body) {
    let faces: Vec<FaceId> = b.face_ids().collect();
    if faces.is_empty() {
        return;
    }
    let mut parent: std::collections::HashMap<usize, usize> =
        faces.iter().map(|f| (f.idx(), f.idx())).collect();
    fn find(p: &mut std::collections::HashMap<usize, usize>, x: usize) -> usize {
        let mut r = x;
        while p[&r] != r {
            r = p[&r];
        }
        p.insert(x, r);
        r
    }
    let mut edge_faces: std::collections::HashMap<EdgeId, Vec<FaceId>> = std::collections::HashMap::new();
    for &f in &faces {
        for &l in &b.face(f).loops {
            for &c in &b.lp(l).coedges {
                if b.coedge(c).alive {
                    edge_faces.entry(b.coedge(c).edge).or_default().push(f);
                }
            }
        }
    }
    for fs in edge_faces.values() {
        if fs.len() >= 2 {
            let r0 = find(&mut parent, fs[0].idx());
            for f in &fs[1..] {
                let r = find(&mut parent, f.idx());
                parent.insert(r, r0);
            }
        }
    }
    let mut groups: std::collections::HashMap<usize, Vec<FaceId>> = std::collections::HashMap::new();
    for &f in &faces {
        let r = find(&mut parent, f.idx());
        groups.entry(r).or_default().push(f);
    }
    let lump = b.lump_ids().next().unwrap();
    let new_shell: ShellId = b.add_shell(lump);
    for g in groups.values() {
        for &f in g {
            b.face_mut(f).shell = new_shell;
        }
    }
    for s in b.shell_ids().collect::<Vec<_>>() {
        if s != new_shell {
            b.shell_mut(s).alive = false;
        }
    }
    b.shell_mut(new_shell).faces = faces;
    for l in b.lump_ids().collect::<Vec<_>>() {
        if !b.lump(l).shells.contains(&new_shell) {
            b.lump_mut(l).alive = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::curve::Curve;
    use ck_geom::surface::Surface;
    use ck_geom::Vec3;
    use ck_topo::body::{FaceName, Role};

    fn cap(k: u32) -> FaceName {
        FaceName { feature: "F2".into(), role: Role::Cap(k), region: None, suffix: String::new() }
    }
    fn side(e: &str) -> FaceName {
        FaceName { feature: "F2".into(), role: Role::Side(e.into()), region: None, suffix: String::new() }
    }

    fn box_body() -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let p = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let bot: Vec<_> = p.iter().map(|&(x, y)| b.add_vertex(Vec3::new(x, y, 0.0))).collect();
        let top: Vec<_> = p.iter().map(|&(x, y)| b.add_vertex(Vec3::new(x, y, 10.0))).collect();
        // the sweep's creation order: rails first, then per segment its
        // C0 and C1 edges
        let mut bot_e = vec![ck_topo::body::EdgeId(0); 4];
        let mut top_e = vec![ck_topo::body::EdgeId(0); 4];
        let mut rails = Vec::new();
        for i in 0..4 {
            rails.push(b.add_edge(Curve::line_through(p[i].into3(0.0), p[i].into3(10.0)), 0.0, 10.0, bot[i], top[i]));
        }
        for i in 0..4 {
            let j = (i + 1) % 4;
            bot_e[i] = b.add_edge(Curve::line_through(p[i].into3(0.0), p[j].into3(0.0)), 0.0, 10.0, bot[i], bot[j]);
            top_e[i] = b.add_edge(Curve::line_through(p[i].into3(10.0), p[j].into3(10.0)), 0.0, 10.0, top[i], top[j]);
        }
        let f0 = b.add_face(shell, Surface::plane(Vec3::new(0.0, 0.0, 0.0), -Vec3::Z), true, cap(0));
        {
            let lp = b.add_loop(f0);
            for i in 0..4 {
                b.add_coedge(lp, bot_e[i], true, None);
            }
        }
        let f1 = b.add_face(shell, Surface::plane(Vec3::new(0.0, 0.0, 10.0), Vec3::Z), true, cap(1));
        {
            let lp = b.add_loop(f1);
            for i in 0..4 {
                b.add_coedge(lp, top_e[i], false, None);
            }
        }
        let walls: [(&str, [f64; 3], [f64; 3]); 4] = [
            ("e0", [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]),
            ("e1", [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]),
            ("e2", [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]),
            ("e3", [0.0, 10.0, 0.0], [0.0, 0.0, 0.0]),
        ];
        for (k, (ent, a, c)) in walls.iter().enumerate() {
            let pa = Vec3::new(a[0], a[1], a[2]);
            let pb = Vec3::new(c[0], c[1], c[2]);
            let travel = (pb - pa).normalize();
            let nn = travel.cross(Vec3::Z);
            let f = b.add_face(shell, Surface::plane(pa, nn), true, side(ent));
            let lp = b.add_loop(f);
            b.add_coedge(lp, bot_e[k], false, None);
            b.add_coedge(lp, rails[(k + 1) % 4], false, None);
            b.add_coedge(lp, top_e[k], true, None);
            b.add_coedge(lp, rails[k], true, None);
        }
        b
    }

    trait Into3 {
        fn into3(self, z: f64) -> Vec3;
    }
    impl Into3 for (f64, f64) {
        fn into3(self, z: f64) -> Vec3 {
            Vec3::new(self.0, self.1, z)
        }
    }

    #[test]
    fn delete_face_leaves_open_sheet_oracle() {
        // probe df1: deleteFace F2.C1 -> the 5 remaining faces, the top
        // edges open, the edge order preserved
        let body = box_body();
        let c1 = body.face_ids().find(|&f| body.face(f).name.id() == "F2.C1").unwrap();
        let b = sheet(&body, &body.face_ids().filter(|&f| f != c1).collect::<Vec<_>>());
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(fids.len(), 5);
        assert!(fids.contains(&"F2.C0".to_string()));
        assert!(fids.contains(&"F2.S.e0".to_string()));
        // the deleted cap's edges are open (single-face keys)
        let open: Vec<String> = b
            .edge_ids()
            .filter(|&e| b.edge_coedges(e).len() == 1)
            .map(|e| {
                let f = b.edge_faces(e)[0];
                format!("{}#{}", b.face(f).name.id(), e.idx())
            })
            .collect();
        assert_eq!(open.len(), 4, "the 4 top edges open");
        // edge order = the body's: rails first (2 faces), then bottom, top
        let eids: Vec<EdgeId> = b.edge_ids().collect();
        assert_eq!(eids.len(), 12);
        let (p, q) = {
            let ed = b.edge(eids[0]);
            (b.vertex(ed.v0).p, b.vertex(ed.v1).p)
        };
        assert!(p.dist(Vec3::new(0.0, 0.0, 0.0)) < 1e-9 && q.dist(Vec3::new(0.0, 0.0, 10.0)) < 1e-9);
        let (p, q) = {
            let ed = b.edge(eids[4]);
            (b.vertex(ed.v0).p, b.vertex(ed.v1).p)
        };
        assert!(p.dist(Vec3::new(0.0, 0.0, 0.0)) < 1e-9 && q.dist(Vec3::new(10.0, 0.0, 0.0)) < 1e-9);
        // the top edge of e0 sits at index 5 (rails, then per side its
        // bottom + top — the oracle probe's order)
        let (p, q) = {
            let ed = b.edge(eids[5]);
            (b.vertex(ed.v0).p, b.vertex(ed.v1).p)
        };
        assert!(p.dist(Vec3::new(0.0, 0.0, 10.0)) < 1e-9 && q.dist(Vec3::new(10.0, 0.0, 10.0)) < 1e-9, "{p:?} {q:?}");
    }
}
