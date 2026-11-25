//! Display-mesh watertightness of a body that claims to be a valid closed shell. A boolean can assemble a body whose
//! B-rep checks pass while a face lost its trims, so that it tessellates to nothing: the hole shows as a see-through patch
//! and every rim edge of the missing face is used by a single triangle. The mesh is the cheapest independent witness.
//!
//! The functions are pure over the mesh arrays (`positions` 9 floats per triangle, `tri_face` the wire face index of each
//! triangle = enumeration index of `Body::face_ids`), so ck-check does not depend on the tessellator; the caller (the
//! `validate` request) tessellates.
use crate::Issue;
use ck_topo::body::Body;
use ck_core::fxhash::FxHashMap;

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

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Distance from `p` to the triangle `a b c` (closest-point-on-triangle, Ericson).
fn point_tri_dist(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    let q = if d1 <= 0.0 && d2 <= 0.0 {
        a
    } else {
        let bp = sub(p, b);
        let (d3, d4) = (dot(ab, bp), dot(ac, bp));
        if d3 >= 0.0 && d4 <= d3 {
            b
        } else {
            let vc = d1 * d4 - d3 * d2;
            if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
                let v = d1 / (d1 - d3);
                [a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]]
            } else {
                let cp = sub(p, c);
                let (d5, d6) = (dot(ab, cp), dot(ac, cp));
                if d6 >= 0.0 && d5 <= d6 {
                    c
                } else {
                    let vb = d5 * d2 - d1 * d6;
                    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
                        let w = d2 / (d2 - d6);
                        [a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]]
                    } else {
                        let va = d3 * d6 - d5 * d4;
                        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
                            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                            [b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2])]
                        } else {
                            let den = 1.0 / (va + vb + vc);
                            let (v, w) = (vb * den, vc * den);
                            [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w]
                        }
                    }
                }
            }
        }
    };
    let d = sub(p, q);
    dot(d, d).sqrt()
}

/// Whether the faces `a` and `b` may explain a hairline crack between them: always when `adj` is `None`, else when `adj` says
/// they are neighbours in the B-rep.
fn may_pair(adj: Option<&dyn Fn(i32, i32) -> bool>, a: i32, b: i32) -> bool {
    a != b && adj.map_or(true, |f| f(a, b))
}

/// For each open edge `(face, a, b)`: `None` when some of nine samples along it lie farther than `crack` from every triangle of
/// another (neighbouring, see [`may_pair`]) face; else `Some(gap)`, the largest distance of a sample to the nearest such triangle.
fn uncovered_edges(positions: &[f32], tri_face: &[i32], edges: &[(i32, [f64; 3], [f64; 3])], crack: f64, adj: Option<&dyn Fn(i32, i32) -> bool>) -> Vec<Option<f64>> {
    let nt = positions.len() / 9;
    let pt = |t: usize, k: usize| [positions[9 * t + 3 * k] as f64, positions[9 * t + 3 * k + 1] as f64, positions[9 * t + 3 * k + 2] as f64];
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for t in 0..nt {
        for k in 0..3 {
            let p = pt(t, k);
            for d in 0..3 {
                lo[d] = lo[d].min(p[d]);
                hi[d] = hi[d].max(p[d]);
            }
        }
    }
    let ext = (0..3).map(|d| hi[d] - lo[d]).fold(0.0, f64::max);
    if nt == 0 || !(ext > 0.0) {
        return vec![None; edges.len()];
    }
    let cs = (ext / 48.0).max(crack * 2.0);
    let cell = |p: [f64; 3]| [((p[0] - lo[0]) / cs).floor() as i64, ((p[1] - lo[1]) / cs).floor() as i64, ((p[2] - lo[2]) / cs).floor() as i64];
    let mut grid: FxHashMap<[i64; 3], Vec<u32>> = FxHashMap::default();
    // each triangle's box grown by `crack`: a point farther than that from the box is farther than `crack` from the triangle
    let mut boxes: Vec<([f64; 3], [f64; 3])> = Vec::with_capacity(nt);
    let mut big: Vec<u32> = vec![];
    const BIG_SPAN: i64 = 256;
    for t in 0..nt {
        let (a, b, c) = (pt(t, 0), pt(t, 1), pt(t, 2));
        let mn = [a[0].min(b[0]).min(c[0]) - crack, a[1].min(b[1]).min(c[1]) - crack, a[2].min(b[2]).min(c[2]) - crack];
        let mx = [a[0].max(b[0]).max(c[0]) + crack, a[1].max(b[1]).max(c[1]) + crack, a[2].max(b[2]).max(c[2]) + crack];
        boxes.push((mn, mx));
        let (c0, c1) = (cell(mn), cell(mx));
        let span = (c1[0] - c0[0] + 1) * (c1[1] - c0[1] + 1) * (c1[2] - c0[2] + 1);
        if span > 200_000 {
            continue;
        }
        // a long thin triangle (a sliver spanning the part) would be listed in thousands of cells: keep those apart and test
        // them for every query instead (same answer, linear in the few such triangles)
        if span > BIG_SPAN {
            big.push(t as u32);
            continue;
        }
        for x in c0[0]..=c1[0] {
            for y in c0[1]..=c1[1] {
                for z in c0[2]..=c1[2] {
                    grid.entry([x, y, z]).or_default().push(t as u32);
                }
            }
        }
    }
    edges
        .iter()
        .map(|(face, a, b)| {
            let mut gap = 0.0f64;
            for k in 0..=8 {
                let u = k as f64 / 8.0;
                let p = [a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])];
                let mut best = f64::INFINITY;
                for &t in grid.get(&cell(p)).map_or(&[][..], |v| &v[..]).iter().chain(big.iter()) {
                    let t = t as usize;
                    let (mn, mx) = &boxes[t];
                    if may_pair(adj, *face, tri_face.get(t).copied().unwrap_or(-1)) && (0..3).all(|d| p[d] >= mn[d] && p[d] <= mx[d]) {
                        best = best.min(point_tri_dist(p, pt(t, 0), pt(t, 1), pt(t, 2)));
                    }
                }
                if best > crack {
                    return None;
                }
                gap = gap.max(best);
            }
            Some(gap)
        })
        .collect()
}

/// Every mesh edge (between two distinct vertices, matched by their exact f32 coordinates, -0 = 0) is used by at least two
/// triangles.
fn exact_closed(positions: &[f32]) -> bool {
    let key = |i: usize| [0, 1, 2].map(|k| (positions[3 * i + k] + 0.0).to_bits());
    let mut uses: FxHashMap<([u32; 3], [u32; 3]), u32> = FxHashMap::default();
    for t in 0..positions.len() / 9 {
        let v = [key(3 * t), key(3 * t + 1), key(3 * t + 2)];
        for e in 0..3 {
            let (a, b) = (v[e], v[(e + 1) % 3]);
            if a == b {
                continue;
            }
            *uses.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
        }
    }
    uses.values().all(|&u| u >= 2)
}

/// [`open_mesh_edges`], not counting an open edge both of whose ends lie within `crack` of an open edge of ANOTHER face: two
/// faces that sample their shared edge with different points (a T-junction, a hairline crack of at most the chord deviation)
/// leave such pairs, and they are no visible hole. `crack = 0` counts every edge used once.
pub fn open_mesh_edges_tol(positions: &[f32], tri_face: &[i32], q: f64, crack: f64) -> (usize, Vec<i32>) {
    let (open, faces, _) = open_mesh_edges_detail(positions, tri_face, q, crack, None);
    (open, faces)
}

/// [`open_mesh_edges_tol`] that also returns the open edges it forgave as hairlines: `(face, gap)` per forgiven edge (so a
/// face with three forgiven edges appears three times), `gap` the largest distance between the edge and the face that
/// covers it. A body with a missing face thinner than `crack` gives exactly such edges, so the caller can report them
/// without calling the mesh open.
///
/// `adj(a, b)` says whether faces `a` and `b` are neighbours in the B-rep (share an edge). With `Some(adj)` an open edge is
/// forgiven only against a neighbouring face: a sampling difference along a shared edge explains the crack, but an open
/// edge next to a face that is no neighbour is a hole (a thin face between two neighbours that lost its triangles leaves
/// its two rims exactly like that). `None` forgives against any other face.
pub fn open_mesh_edges_detail(positions: &[f32], tri_face: &[i32], q: f64, crack: f64, adj: Option<&dyn Fn(i32, i32) -> bool>) -> (usize, Vec<i32>, Vec<(i32, f64)>) {
    // A mesh that is closed with its vertices matched bit for bit has no hole. The weld below then can only invent one: it
    // joins each vertex to the first representative within `q`, so a feature smaller than `q` (the rim circle of radius 1e-4
    // where a revolved cone almost reaches its axis, crash fuzz `case_244`) is merged into a few points in an order-dependent
    // way, and the triangles around it read as open edges.
    if exact_closed(positions) {
        return (0, vec![], vec![]);
    }
    let mut forgiven: Vec<(i32, f64)> = vec![];
    let n = positions.len() / 3;
    let cell = |p: [f64; 3]| [(p[0] / q).floor() as i64, (p[1] / q).floor() as i64, (p[2] / q).floor() as i64];
    let mut reps: Vec<[f64; 3]> = vec![];
    let mut grid: FxHashMap<[i64; 3], Vec<u32>> = FxHashMap::default();
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
    let mut uses: FxHashMap<(u32, u32), (u32, i32)> = FxHashMap::default();
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
    if crack > 0.0 && std::env::var("CK_OPEN_DBG").map_or(false, |v| v == "2") {
        for (f, a, b) in &open_edges {
            eprintln!("RAWOPEN face {f} {:.4?} -> {:.4?}", a, b);
        }
    }
    if crack > 0.0 && !open_edges.is_empty() {
        // grid of the open edges (every cell an edge passes through), then test each end against the other faces' edges
        let cs = (crack * 4.0).max(1e-6);
        let cell = |p: [f64; 3]| [(p[0] / cs).floor() as i64, (p[1] / cs).floor() as i64, (p[2] / cs).floor() as i64];
        let mut grid: FxHashMap<[i64; 3], Vec<u32>> = FxHashMap::default();
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
        // distance from `p` to the nearest open edge of another (neighbouring) face, when within `crack`
        let near_other = |p: [f64; 3], face: i32| -> Option<f64> {
            let mut best: Option<f64> = None;
            let c = cell(p);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(v) = grid.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) {
                            for &j in v {
                                let (f, a, b) = &open_edges[j as usize];
                                if may_pair(adj, face, *f) {
                                    let d = seg_dist(p, *a, *b);
                                    if d <= crack && best.map_or(true, |m| d < m) {
                                        best = Some(d);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            best
        };
        let gaps: Vec<Option<f64>> = open_edges
            .iter()
            .map(|(f, a, b)| match (near_other(*a, *f), near_other(*b, *f)) {
                (Some(x), Some(y)) => Some(x.max(y)),
                _ => None,
            })
            .collect();
        forgiven.extend(open_edges.iter().zip(&gaps).filter_map(|(e, g)| g.map(|g| (e.0, g))));
        let mut k = gaps.iter();
        open_edges.retain(|_| k.next().unwrap().is_none());
    }
    if crack > 0.0 && !open_edges.is_empty() {
        // An open edge whose every sample lies within `crack` of a triangle of ANOTHER face has a neighbour that covers it
        // to within the chord deviation: the two faces sample their shared edge differently, leaving a hairline of that
        // width (a fine trim tail ending at a cusp, say), which is no hole.
        let gaps = uncovered_edges(positions, tri_face, &open_edges, crack, adj);
        forgiven.extend(open_edges.iter().zip(&gaps).filter_map(|(e, g)| g.map(|g| (e.0, g))));
        let mut k = gaps.iter();
        open_edges.retain(|_| k.next().unwrap().is_none());
    }
    if crack > 0.0 && std::env::var("CK_OPEN_DBG").is_ok() {
        for (f, a, b) in &open_edges {
            eprintln!("OPENEDGE face {f} {:.4?} -> {:.4?}", a, b);
        }
    }
    let mut faces: Vec<i32> = open_edges.iter().map(|e| e.0).collect();
    let open = faces.len();
    faces.sort_unstable();
    faces.dedup();
    (open, faces, forgiven)
}

/// `TESSELLATION_OPEN` issues: one per face of `b` that has no triangle (`failed` = faces the tessellator gave up on, from
/// the mesh), and one for the mesh edges used by a single triangle (entity = the first bordering faces).
pub fn check_mesh(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize]) -> Vec<Issue> {
    check_mesh_inner(b, positions, tri_face, failed, &[], false, &crate::Ctl::unlimited()).0
}

/// [`check_mesh`] with the tessellator's reason for each failed face (`why[i]` belongs to `failed[i]`), and the per-face
/// mesh-area witness (costs an exact area integral per face: the opt-in `validate` path only, not the boolean's rim check).
pub fn check_mesh_why(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize], why: &[String]) -> Vec<Issue> {
    check_mesh_inner(b, positions, tri_face, failed, why, true, &crate::Ctl::unlimited()).0
}

/// [`check_mesh_why`] under a time limit: the area witness (`meshAreaWitness`) and the open-edge search (`meshOpenEdges`)
/// are timed in `ctl`, the second is left out when the limit ran out during or before the first.
pub fn check_mesh_ctl(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize], why: &[String], ctl: &crate::Ctl) -> Vec<Issue> {
    check_mesh_inner(b, positions, tri_face, failed, why, true, ctl).0
}

/// [`check_mesh_ctl`] that also returns the hairline note: the open mesh edges that were forgiven because another face's
/// triangles or open edges lie within the crack tolerance. They are not issues (the display mesh of a sound body has such
/// cracks), but a face that is missing and thinner than the tolerance leaves exactly this, so `validate` lists them as
/// `meshNotes` entries with the code `TESSELLATION_HAIRLINE`. Empty when nothing was forgiven.
pub fn check_mesh_ctl_notes(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize], why: &[String], ctl: &crate::Ctl) -> (Vec<Issue>, Vec<String>) {
    check_mesh_inner(b, positions, tri_face, failed, why, true, ctl)
}

fn check_mesh_inner(b: &Body, positions: &[f32], tri_face: &[i32], failed: &[usize], why: &[String], area: bool, ctl: &crate::Ctl) -> (Vec<Issue>, Vec<String>) {
    let mut notes: Vec<String> = vec![];
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
    // a face whose area is off by less than 5e-4 of the squared part size is a sliver or a speck: its chord error
    // (a polygon inscribed in a thin crescent loses a fifth of it) cannot be seen, and is not a hole
    let hi_all = positions.iter().fold(0.0f64, |m, &x| m.max((x as f64).abs()));
    let floor = 5e-4 * hi_all * hi_all;
    let witness = |out: &mut Vec<Issue>| {
        for (i, &f) in ids.iter().enumerate() {
            if !area || !has[i] {
                continue;
            }
            if ctl.expired() {
                return;
            }
            // the exact area of a spline / swept / revolved face is an integral that costs more than meshing the whole body (a
            // few seconds on a large sample shell): witness the analytic faces, where it is cheap, and leave the rest to the
            // open-edge and no-triangle checks
            if !matches!(
                b.face(f).surface,
                ck_geom::surface::Surface::Plane { .. } | ck_geom::surface::Surface::Cylinder { .. } | ck_geom::surface::Surface::Cone { .. } | ck_geom::surface::Surface::Sphere { .. } | ck_geom::surface::Surface::Torus { .. }
            ) {
                continue;
            }
            // the area element of a cylinder or cone is a polynomial in v: the tabulated path costs a tenth on a face with a long
            // or tangled boundary (an invalid body), and is the same number on the rest
            let exact = ck_topo::mass::face_integral_vpoly::<1>(b, f, 1, &|_, n| [n.len()])[0].abs();
            if exact.is_finite() && exact > 0.0 && (mesh_area[i] < 0.9 * exact || mesh_area[i] > 1.1 * exact) && (mesh_area[i] - exact).abs() > floor {
                out.push(Issue {
                    code: "TESSELLATION_OPEN",
                    entity: b.face_id_str(f),
                    detail: format!("the mesh covers {:.0}% of the face area ({:.4e} of {:.4e})", 100.0 * mesh_area[i] / exact, mesh_area[i], exact),
                });
            }
        }
    };
    ctl.stage("meshAreaWitness", || witness(&mut out));
    // vertex merge distance: well below the mesh feature size, above f32 rounding of the coordinates
    let mut hi = 0.0f64;
    for &x in positions {
        hi = hi.max((x as f64).abs());
    }
    let q = (hi * 1e-5).max(1e-4);
    // a shared edge sampled differently by its two faces leaves a hairline of at most the chord deviation: not a hole
    let crack = (hi * 2e-3).max(0.01);
    // faces are neighbours when some live B-rep edge borders both (seam pieces of one surface included)
    let mut pairs: ck_core::fxhash::FxHashSet<(i32, i32)> = Default::default();
    {
        let index: FxHashMap<_, i32> = ids.iter().enumerate().map(|(i, &f)| (f, i as i32)).collect();
        for e in b.edge_ids() {
            let fs: Vec<i32> = b.edge_faces(e).iter().filter_map(|f| index.get(f).copied()).collect();
            for (i, &x) in fs.iter().enumerate() {
                for &y in &fs[i + 1..] {
                    pairs.insert((x.min(y), x.max(y)));
                }
            }
        }
    }
    let adj = |x: i32, y: i32| pairs.contains(&(x.min(y), x.max(y)));
    let (open, faces, forgiven) = ctl.stage("meshOpenEdges", || open_mesh_edges_detail(positions, tri_face, q, crack, Some(&adj))).unwrap_or((0, vec![], vec![]));
    if !forgiven.is_empty() {
        let max_gap = forgiven.iter().fold(0.0f64, |m, e| m.max(e.1));
        let mut fs: Vec<i32> = forgiven.iter().map(|e| e.0).collect();
        fs.sort_unstable();
        fs.dedup();
        let names: Vec<String> = fs.iter().take(6).filter_map(|&i| ids.get(i as usize).map(|&f| b.face_id_str(f))).collect();
        notes.push(format!(
            "TESSELLATION_HAIRLINE {}: {} open mesh edges forgiven as hairline cracks between neighbouring faces (within {:.3e} mm, measured gap up to {:.3e} mm); a face missing but thinner than that would look the same",
            names.join("|"),
            forgiven.len(),
            crack,
            max_gap
        ));
    }
    if open > 0 {
        let names: Vec<String> = faces.iter().take(6).filter_map(|&i| ids.get(i as usize).map(|&f| b.face_id_str(f))).collect();
        out.push(Issue {
            code: "TESSELLATION_OPEN",
            entity: names.join("|"),
            detail: format!("{open} mesh edges are used by a single triangle (the display mesh has holes)"),
        });
    }
    (out, notes)
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
    fn edge_covered_by_another_faces_triangles_is_a_hairline_but_a_hole_rim_is_not() {
        // face 0: one triangle on the x axis edge; face 1 covers that edge with triangles whose own edges are shifted by
        // 0.004 (a fine trim tail): the edge of face 0 is a hairline. A lone triangle's edges stay open.
        let tris: [[[f64; 3]; 3]; 3] = [
            [[0., 0., 0.], [4., 0., 0.], [0., 4., 0.]],
            [[0., 0.004, -0.5], [4., 0.004, -0.5], [4., 0.004, 0.5]],
            [[0., 0.004, -0.5], [4., 0.004, 0.5], [0., 0.004, 0.5]],
        ];
        let pos: Vec<f32> = tris.iter().flatten().flat_map(|p| p.map(|x| x as f32)).collect();
        let (strict, _) = open_mesh_edges_tol(&pos, &[0, 1, 1], 1e-4, 0.0);
        let (hairline, _) = open_mesh_edges_tol(&pos, &[0, 1, 1], 1e-4, 0.01);
        let (lone, _) = open_mesh_edges_tol(&pos[..9], &[0], 1e-4, 0.01);
        assert_eq!((strict, hairline), (7, 6), "face 0's x-axis edge is covered by face 1");
        assert_eq!(lone, 3, "a lone triangle's three rim edges are holes");
    }

    #[test]
    fn forgiven_hairline_edges_are_listed_but_a_hole_rim_is_not() {
        let tris: [[[f64; 3]; 3]; 3] = [
            [[0., 0., 0.], [4., 0., 0.], [0., 4., 0.]],
            [[0., 0.004, -0.5], [4., 0.004, -0.5], [4., 0.004, 0.5]],
            [[0., 0.004, -0.5], [4., 0.004, 0.5], [0., 0.004, 0.5]],
        ];
        let pos: Vec<f32> = tris.iter().flatten().flat_map(|p| p.map(|x| x as f32)).collect();
        let (open, _, forgiven) = open_mesh_edges_detail(&pos, &[0, 1, 1], 1e-4, 0.01, Some(&|_, _| true));
        let (strict, _, none) = open_mesh_edges_detail(&pos, &[0, 1, 1], 1e-4, 0.0, None);
        assert_eq!((open, forgiven.len()), (6, 1), "the x-axis edge of face 0 is forgiven and listed");
        assert!((forgiven[0].1 - 0.004).abs() < 1e-6, "the measured gap is reported: {:?}", forgiven);
        assert_eq!((strict, none.len()), (7, 0), "no tolerance, nothing forgiven");
        let (lone, _, f) = open_mesh_edges_detail(&pos[..9], &[0], 1e-4, 0.01, None);
        assert_eq!((lone, f.len()), (3, 0), "a hole rim is reported, not forgiven");
    }

    /// A thin strip face (face 1) between faces 0 and 2 lost all its triangles: the two rims it leaves are 0.05 apart, inside
    /// the crack tolerance. Face 0 and face 2 are both neighbours of the strip but not of each other, so the rims are holes.
    /// If the faces were neighbours (a shared edge sampled differently) the same geometry is a forgiven hairline.
    #[test]
    fn rims_of_a_missing_thin_strip_between_non_neighbours_are_a_hole() {
        let tris: [[[f64; 3]; 3]; 2] = [
            [[0., 0., 0.], [4., 0., 0.], [0., 4., 0.]],
            [[0., 0.05, 0.], [4., 0.05, 0.], [0., 0.05, 4.]],
        ];
        let pos: Vec<f32> = tris.iter().flatten().flat_map(|p| p.map(|x| x as f32)).collect();
        let faces = [0, 2];
        let strip_only = |a: i32, b: i32| (a == 0 && b == 1) || (a == 1 && b == 0) || (a == 1 && b == 2) || (a == 2 && b == 1);
        let (old, _, old_f) = open_mesh_edges_detail(&pos, &faces, 1e-4, 0.2, None);
        let (open, open_faces, forgiven) = open_mesh_edges_detail(&pos, &faces, 1e-4, 0.2, Some(&strip_only));
        assert_eq!((old, old_f.len()), (4, 2), "without adjacency the two rims facing each other are forgiven");
        assert_eq!((open, forgiven.len()), (6, 0), "faces 0 and 2 are no neighbours: both rims stay open");
        assert_eq!(open_faces, vec![0, 2]);
        let (open, _, forgiven) = open_mesh_edges_detail(&pos, &faces, 1e-4, 0.2, Some(&|a, b| a != b));
        assert_eq!((open, forgiven.len()), (4, 2), "neighbouring faces: forgiven as before");
        let widest = forgiven.iter().fold(0.0f64, |m, e| m.max(e.1));
        assert!((widest - 0.05).abs() < 1e-6, "gap {:?}", forgiven);
    }

    #[test]
    fn covered_edge_is_not_forgiven_against_a_non_neighbour() {
        // the fine-trim-tail case (face 1 covers face 0's edge at 0.004) with faces that do not share an edge: a hole
        let tris: [[[f64; 3]; 3]; 3] = [
            [[0., 0., 0.], [4., 0., 0.], [0., 4., 0.]],
            [[0., 0.004, -0.5], [4., 0.004, -0.5], [4., 0.004, 0.5]],
            [[0., 0.004, -0.5], [4., 0.004, 0.5], [0., 0.004, 0.5]],
        ];
        let pos: Vec<f32> = tris.iter().flatten().flat_map(|p| p.map(|x| x as f32)).collect();
        let (open, _, f) = open_mesh_edges_detail(&pos, &[0, 1, 1], 1e-4, 0.01, Some(&|_, _| false));
        assert_eq!((open, f.len()), (7, 0));
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

    /// A closed bipyramid over a ring of radius 1e-4 (smaller than the weld distance) is closed bit for bit: no open edge.
    /// A tetrahedron without one face still has three. (The case where the weld itself invented open edges is the revolve
    /// in `ck-model/tests/fuzz_invalid_results.rs`, `rim_circle_below_the_weld_distance_is_not_a_hole`.)
    #[test]
    fn closed_mesh_with_a_feature_below_the_weld_distance() {
        let n = 20;
        let ring: Vec<[f64; 3]> = (0..n).map(|k| {
            let a = std::f64::consts::TAU * k as f64 / n as f64;
            [1e-4 * a.cos(), 1e-4 * a.sin(), 0.0]
        }).collect();
        let (top, bot) = ([0.0, 0.0, 10.0], [0.0, 0.0, -10.0]);
        let mut pos: Vec<f32> = vec![];
        for k in 0..n {
            let (a, b) = (ring[k], ring[(k + 1) % n]);
            for t in [[a, b, top], [b, a, bot]] {
                pos.extend(t.iter().flat_map(|p| p.map(|x| x as f32)));
            }
        }
        let faces = vec![0; pos.len() / 9];
        assert_eq!(open_mesh_edges_detail(&pos, &faces, 1.67e-4, 0.0, None).0, 0);
        // a real hole is still one: a tetrahedron without one face
        let p = [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]];
        let tet = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        let pos = flat(&p, &tet);
        assert_eq!(open_mesh_edges_detail(&pos, &[0, 1, 2, 3], 1.67e-4, 0.0, None).0, 0);
        assert_eq!(open_mesh_edges_detail(&pos[..27], &[0, 1, 2], 1.67e-4, 0.0, None).0, 3);
    }
}
