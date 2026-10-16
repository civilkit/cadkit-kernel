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
    let mut faces: Vec<i32> = uses.values().filter(|u| u.0 == 1).map(|u| u.1).collect();
    let open = faces.len();
    faces.sort_unstable();
    faces.dedup();
    (open, faces)
}

/// `TESSELLATION_OPEN` issues: one per face of `b` that has no triangle (`failed` = faces the tessellator gave up on, from
/// the mesh), and one for the mesh edges used by a single triangle (entity = the first bordering faces).
pub fn check_mesh(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize]) -> Vec<Issue> {
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
            let why = if failed.contains(&i) { "tessellation failed" } else { "no triangles" };
            out.push(Issue { code: "TESSELLATION_OPEN", entity: b.face_id_str(f), detail: format!("face has no mesh triangles ({why})") });
        }
    }
    // vertex merge distance: well below the mesh feature size, above f32 rounding of the coordinates
    let mut hi = 0.0f64;
    for &x in positions {
        hi = hi.max((x as f64).abs());
    }
    let q = (hi * 1e-5).max(1e-4);
    let (open, faces) = open_mesh_edges(positions, tri_face, q);
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
}
