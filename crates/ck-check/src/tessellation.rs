//! Display-mesh watertightness of a body that claims to be a valid closed shell. A boolean can assemble a body whose
//! B-rep checks pass while a face lost its trims, so that it tessellates to nothing: the hole shows as a see-through patch
//! and every rim edge of the missing face is used by a single triangle. The mesh is the cheapest independent witness.
//!
//! The functions are pure over the mesh arrays (`positions` 9 floats per triangle, `tri_face` the wire face index of each
//! triangle = enumeration index of `Body::face_ids`), so ck-check does not depend on the tessellator; the caller (the
//! `validate` request) tessellates.
use crate::Issue;
use ck_topo::body::Body;
use std::collections::HashMap;

/// Number of mesh edges used by exactly one triangle, with the wire faces they border (ascending, deduplicated).
/// Vertices closer than `q` are the same vertex.
pub fn open_mesh_edges(positions: &[f32], tri_face: &[i32], q: f64) -> (usize, Vec<i32>) {
    open_mesh_edges_tol(positions, tri_face, q, 0.0)
}

fn seg_dist(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let l2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let u = if l2 > 0.0 { ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / l2).clamp(0.0, 1.0) } else { 0.0 };
    ((ap[0] - u * ab[0]).powi(2) + (ap[1] - u * ab[1]).powi(2) + (ap[2] - u * ab[2]).powi(2)).sqrt()
}

/// [`open_mesh_edges`], not counting an open edge both of whose ends lie within `crack` of an open edge of ANOTHER face: two
/// faces that sample their shared edge with different points (a T-junction, a hairline crack of at most the chord deviation)
/// leave such pairs, and they are no visible hole. `crack = 0` counts every edge used once.
pub fn open_mesh_edges_tol(positions: &[f32], tri_face: &[i32], q: f64, crack: f64) -> (usize, Vec<i32>) {
    let n = positions.len() / 3;
    let cell = |p: [f64; 3]| [(p[0] / q).floor() as i64, (p[1] / q).floor() as i64, (p[2] / q).floor() as i64];
    let mut reps: Vec<[f64; 3]> = vec![];
    let mut grid: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
    let mut ids: Vec<u32> = Vec::with_capacity(n);
    for i in 0..n {
        let p = [positions[3 * i] as f64, positions[3 * i + 1] as f64, positions[3 * i + 2] as f64];
        let c = cell(p);
        let mut found = None;
        'outer: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(v) = grid.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) {
                        for &r in v {
                            let o = reps[r as usize];
                            if (o[0] - p[0]).powi(2) + (o[1] - p[1]).powi(2) + (o[2] - p[2]).powi(2) <= q * q {
                                found = Some(r);
                                break 'outer;
                            }
                        }
                    }
                }
            }
        }
        let id = found.unwrap_or_else(|| {
            reps.push(p);
            let id = reps.len() as u32 - 1;
            grid.entry(c).or_default().push(id);
            id
        });
        ids.push(id);
    }
    let mut uses: HashMap<(u32, u32), (u32, i32)> = HashMap::new();
    for t in 0..n / 3 {
        let v = [ids[3 * t], ids[3 * t + 1], ids[3 * t + 2]];
        for e in 0..3 {
            let (a, b) = (v[e], v[(e + 1) % 3]);
            if a == b {
                continue;
            }
            let k = if a < b { (a, b) } else { (b, a) };
            let slot = uses.entry(k).or_insert((0, tri_face.get(t).copied().unwrap_or(-1)));
            slot.0 += 1;
        }
    }
    let mut open_edges: Vec<(i32, [f64; 3], [f64; 3])> = uses
        .iter()
        .filter(|(_, u)| u.0 == 1)
        .map(|(k, u)| (u.1, reps[k.0 as usize], reps[k.1 as usize]))
        .collect();
    if crack > 0.0 && !open_edges.is_empty() {
        // grid of the open edges (every cell an edge passes through), then test each end against the other faces' edges
        let cs = (crack * 4.0).max(1e-6);
        let cell = |p: [f64; 3]| [(p[0] / cs).floor() as i64, (p[1] / cs).floor() as i64, (p[2] / cs).floor() as i64];
        let mut grid: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
        for (i, (_, a, b)) in open_edges.iter().enumerate() {
            let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
            let n = ((len / (cs * 0.5)).ceil() as usize).clamp(1, 4096);
            let mut last = [i64::MIN; 3];
            for k in 0..=n {
                let t = k as f64 / n as f64;
                let c = cell([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])]);
                if c != last {
                    grid.entry(c).or_default().push(i as u32);
                    last = c;
                }
            }
        }
        let near_other = |p: [f64; 3], face: i32| -> bool {
            let c = cell(p);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(v) = grid.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) {
                            for &j in v {
                                let (f, a, b) = &open_edges[j as usize];
                                if *f != face && seg_dist(p, *a, *b) <= crack {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
            false
        };
        let keep: Vec<bool> = open_edges.iter().map(|(f, a, b)| !(near_other(*a, *f) && near_other(*b, *f))).collect();
        let mut k = keep.iter();
        open_edges.retain(|_| *k.next().unwrap());
    }
    let mut faces: Vec<i32> = open_edges.iter().map(|e| e.0).collect();
    let open = faces.len();
    faces.sort_unstable();
    faces.dedup();
    (open, faces)
}

/// `TESSELLATION_OPEN` issues: one per face of `b` that has no triangle (`failed` = faces the tessellator gave up on, from
/// the mesh), and one for the mesh edges used by a single triangle (entity = the first bordering faces).
pub fn check_mesh(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize]) -> Vec<Issue> {
    check_mesh_inner(b, positions, tri_face, failed, &[], false)
}

/// [`check_mesh`] with the tessellator's reason for each failed face (`why[i]` belongs to `failed[i]`), and the per-face
/// mesh-area witness (costs an exact area integral per face: the opt-in `validate` path only, not the boolean's rim check).
pub fn check_mesh_why(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize], why: &[String]) -> Vec<Issue> {
    check_mesh_inner(b, positions, tri_face, failed, why, true)
}

fn check_mesh_inner(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize], why: &[String], area: bool) -> Vec<Issue> {
    let mut out = vec![];
    let ids: Vec<_> = b.face_ids().collect();
    let mut has = vec![false; ids.len()];
    for &t in tri_face {
        if let Some(h) = has.get_mut(t as usize) {
            *h = true;
        }
    }
    for (i, &f) in ids.iter().enumerate() {
        if !has[i] {
            let reason = match failed.iter().position(|&x| x == i) {
                Some(k) => match why.get(k).filter(|w| !w.is_empty()) {
                    Some(w) => format!("tessellation failed: {w}"),
                    None => "tessellation failed".to_string(),
                },
                None => "no triangles".to_string(),
            };
            out.push(Issue { code: "TESSELLATION_OPEN", entity: b.face_id_str(f), detail: format!("face has no mesh triangles ({reason})") });
        }
    }
    // A face whose mesh covers only part of it (a cylinder wall meshed around its cut-outs but not along its length) shows no
    // open edge of its own, only the rim edges of its neighbours: compare the mesh area with the exact face area.
    let mut mesh_area = vec![0.0f64; ids.len()];
    for (t, &fi) in tri_face.iter().enumerate() {
        if let (Some(a), Some(p)) = (mesh_area.get_mut(fi as usize), positions.get(9 * t..9 * t + 9)) {
            let u = [(p[3] - p[0]) as f64, (p[4] - p[1]) as f64, (p[5] - p[2]) as f64];
            let w = [(p[6] - p[0]) as f64, (p[7] - p[1]) as f64, (p[8] - p[2]) as f64];
            *a += 0.5 * ((u[1] * w[2] - u[2] * w[1]).powi(2) + (u[2] * w[0] - u[0] * w[2]).powi(2) + (u[0] * w[1] - u[1] * w[0]).powi(2)).sqrt();
        }
    }
    for (i, &f) in ids.iter().enumerate() {
        if !area || !has[i] {
            continue;
        }
        let exact = ck_topo::mass::face_integral::<1>(b, f, 1, &|_, n| [n.len()])[0].abs();
        if exact.is_finite() && exact > 0.0 && (mesh_area[i] < 0.9 * exact || mesh_area[i] > 1.1 * exact) {
            out.push(Issue {
                code: "TESSELLATION_OPEN",
                entity: b.face_id_str(f),
                detail: format!("the mesh covers {:.0}% of the face area ({:.4e} of {:.4e})", 100.0 * mesh_area[i] / exact, mesh_area[i], exact),
            });
        }
    }
    // vertex merge distance: well below the mesh feature size, above f32 rounding of the coordinates
    let mut hi = 0.0f64;
    for &x in positions {
        hi = hi.max((x as f64).abs());
    }
    let q = (hi * 1e-5).max(1e-4);
    // a shared edge sampled differently by its two faces leaves a hairline of at most the chord deviation: not a hole
    let (open, faces) = open_mesh_edges_tol(positions, tri_face, q, (hi * 2e-3).max(0.01));
    if open > 0 {
        let names: Vec<String> = faces.iter().take(6).filter_map(|&i| ids.get(i as usize).map(|&f| b.face_id_str(f))).collect();
        out.push(Issue {
            code: "TESSELLATION_OPEN",
            entity: names.join("|"),
            detail: format!("{open} mesh edges are used by a single triangle (the display mesh has holes)"),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat(p: &[[f64; 3]; 4], t: &[[usize; 3]]) -> Vec<f32> {
        t.iter().flat_map(|tri| tri.iter().flat_map(|&i| p[i].map(|x| x as f32))).collect()
    }

    #[test]
    fn closed_tetrahedron_has_no_open_edges() {
        let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
        let pos = flat(&p, &[[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]]);
        assert_eq!(open_mesh_edges(&pos, &[0, 1, 2, 3], 1e-4).0, 0);
    }

    #[test]
    fn missing_face_leaves_its_rim_open() {
        let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
        let pos = flat(&p, &[[0, 2, 1], [0, 1, 3], [1, 2, 3]]);
        let (open, faces) = open_mesh_edges(&pos, &[0, 1, 2], 1e-4);
        assert_eq!(open, 3);
        assert_eq!(faces, vec![0, 1, 2]);
    }

    #[test]
    fn t_junction_is_forgiven_by_the_crack_tolerance() {
        // face 0: one triangle on the x axis edge (0,0,0)-(2,0,0); face 1 samples that edge with an extra point at x = 1
        let tris: [[[f64; 3]; 3]; 3] = [
            [[0., 0., 0.], [2., 0., 0.], [0., 2., 0.]],
            [[0., 0., 0.], [1., 0., 0.], [0., 0., -1.]],
            [[1., 0., 0.], [2., 0., 0.], [0., 0., -1.]],
        ];
        let pos: Vec<f32> = tris.iter().flatten().flat_map(|p| p.map(|x| x as f32)).collect();
        let faces = [0, 1, 1];
        let strict = open_mesh_edges(&pos, &faces, 1e-4).0;
        let tol = open_mesh_edges_tol(&pos, &faces, 1e-4, 1e-3).0;
        assert_eq!(strict - tol, 3, "the three edges along the x axis pair up across faces");
    }
}
