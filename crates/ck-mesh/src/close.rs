//! Mesh closing: the stage before the watertight check (opt in with [`crate::Opts::close`]).
//!
//! 1. Gap welding: vertices on open edges that lie within `tol` of each other (and share no triangle) become one vertex
//!    (cracks between sheets whose vertices were never welded). Welding that makes an edge non-manifold is refused
//!    (`meshGapAmbiguous`).
//! 2. T-junctions: a boundary vertex that lies within `tol` of an open edge (not at its ends) splits the triangle on
//!    that edge, so the edge on the other side finds its twin.
//! 3. The surface is made consistently oriented (the fill must run opposite to the neighbouring triangles).
//! 4. Hole filling: the open edges are traced into loops. A loop up to `max_hole_edges` edges and `max_hole_area` of the
//!    mesh's surface area is filled: nearly planar loops by ear clipping in the best-fit plane, other loops up to 50
//!    edges by the minimum-area triangulation (dynamic programming), longer ones by ear clipping in the best-fit plane.
//!    A fill that crosses itself, the loop, or the rest of the mesh is refused with the reason.
//!
//! Everything done is reported ([`CloseReport`]): what was invented is visible. Non-manifold edges are not repaired.

use crate::{edge_use, he_from, he_to, vec_json, Refusal};
use ck_geom::Vec3;
use ck_json::{Json, Obj};
use std::collections::{HashMap, HashSet};

/// Options of the closing stage. `None` tolerances are relative to the bounding-box diagonal.
#[derive(Debug, Clone)]
pub struct CloseOpts {
    /// Absolute gap tolerance (mm); default [`CloseOpts::REL_TOL`] x diagonal.
    pub tol: Option<f64>,
    /// Fill holes (false: only weld gaps and stitch cracks, what remains is refused as `meshNotWatertight`).
    pub fill: bool,
    /// Most edges of a hole that is filled.
    pub max_hole_edges: usize,
    /// Largest hole area, as a fraction of the surface area of the mesh around it.
    pub max_hole_area: f64,
    /// A loop is planar when no vertex is farther than this fraction of the loop's diameter from its best-fit plane.
    pub planar_tol: f64,
}

impl CloseOpts {
    pub const REL_TOL: f64 = 1e-4;
    /// Longest loop triangulated by the minimum-area dynamic programme (cubic cost).
    pub const DP_LIMIT: usize = 50;
}

impl Default for CloseOpts {
    fn default() -> Self {
        CloseOpts { tol: None, fill: true, max_hole_edges: 500, max_hole_area: 0.25, planar_tol: 1e-3 }
    }
}

/// One filled hole.
#[derive(Debug, Clone)]
pub struct HoleInfo {
    pub edges: usize,
    pub area: f64,
    pub planar: bool,
    /// Largest distance of a loop vertex from the loop's best-fit plane.
    pub max_plane_dev: f64,
    pub triangles: usize,
    /// "ear" (planar cap, ear clipping), "minArea" (dynamic programme), "earProjected" (ear clipping of a long non-planar loop).
    pub method: &'static str,
    /// Centre of the loop.
    pub at: Vec3,
    /// The triangles added to close the hole, as corner coordinates (what a dry run shows; not in [`CloseReport::json`]).
    pub fill: Vec<[Vec3; 3]>,
}

/// What the closing stage did.
#[derive(Debug, Clone, Default)]
pub struct CloseReport {
    pub tol: f64,
    pub open_edges: usize,
    /// Boundary vertices merged into another (each merge removes one vertex).
    pub merged_vertices: usize,
    /// Triangles dropped by welding (collapsed or duplicate).
    pub dropped: usize,
    /// Boundary vertices inserted into an open edge (T-junctions and cracks stitched).
    pub stitched: usize,
    /// Triangles turned to make the surface consistent.
    pub reoriented: usize,
    pub holes: Vec<HoleInfo>,
    pub filled_triangles: usize,
    pub filled_area: f64,
}

impl CloseReport {
    pub fn json(&self) -> Json {
        self.json_with(false)
    }

    /// [`json`](Self::json); with `fills` each hole also carries `fill:{positions:[x,y,z,..], indices:[a,b,c,..]}`, the
    /// triangles invented for it (corners shared within a hole are listed once).
    pub fn json_with(&self, fills: bool) -> Json {
        let holes: Vec<Json> = self
            .holes
            .iter()
            .map(|h| {
                let mut o = Obj::new()
                    .put("edges", h.edges as f64)
                    .put("area", h.area)
                    .put("planar", h.planar)
                    .put("maxPlaneDev", h.max_plane_dev)
                    .put("triangles", h.triangles as f64)
                    .put("method", h.method)
                    .put("at", vec_json(h.at));
                if fills {
                    o = o.put("fill", fill_json(&h.fill));
                }
                o.build()
            })
            .collect();
        Obj::new()
            .put("tol", self.tol)
            .put("openEdges", self.open_edges as f64)
            .put("mergedVertices", self.merged_vertices as f64)
            .put("droppedTriangles", self.dropped as f64)
            .put("stitched", self.stitched as f64)
            .put("reoriented", self.reoriented as f64)
            .put("filledHoles", self.holes.len() as f64)
            .put("filledTriangles", self.filled_triangles as f64)
            .put("filledArea", self.filled_area)
            .put("holes", Json::Arr(holes))
            .build()
    }
}

fn key(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

fn all_edges(tris: &[[u32; 3]]) -> HashSet<(u32, u32)> {
    let mut s = HashSet::with_capacity(tris.len() * 2);
    for t in tris {
        for k in 0..3 {
            s.insert(key(t[k], t[(k + 1) % 3]));
        }
    }
    s
}

fn area_of(pts: &[Vec3], t: &[u32; 3]) -> f64 {
    crate::tri_cross(pts, t).len() * 0.5
}

/// Run the stage on welded, cleaned triangles (no non-manifold edges). `tris` and `pts` are changed in place.
/// `{positions:[x,y,z,..], indices:[a,b,c,..]}` of triangles given by corner coordinates (equal corners listed once).
fn fill_json(tris: &[[Vec3; 3]]) -> Json {
    let mut pos: Vec<Vec3> = vec![];
    let mut idx: Vec<f64> = vec![];
    for t in tris {
        for p in t {
            let i = pos.iter().position(|q| q.x == p.x && q.y == p.y && q.z == p.z).unwrap_or_else(|| {
                pos.push(*p);
                pos.len() - 1
            });
            idx.push(i as f64);
        }
    }
    Obj::new()
        .put("positions", Json::Arr(pos.iter().flat_map(|p| [Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)]).collect()))
        .put("indices", Json::Arr(idx.into_iter().map(Json::Num).collect()))
        .build()
}

pub fn close(pts: &mut Vec<Vec3>, tris: &mut Vec<[u32; 3]>, c: &CloseOpts, diag: f64, max_listed: usize, rep: &mut CloseReport) -> Result<(), Refusal> {
    let tol = c.tol.filter(|v| *v > 0.0 && v.is_finite()).unwrap_or(CloseOpts::REL_TOL * diag);
    rep.tol = tol;
    let eu = edge_use(tris);
    rep.open_edges = eu.open.len();
    if eu.open.is_empty() {
        return Ok(());
    }
    gap_weld(pts, tris, tol, max_listed, rep)?;
    stitch(pts, tris, tol, max_listed, rep)?;
    let eu = edge_use(tris);
    if eu.open.is_empty() {
        return Ok(());
    }
    rep.reoriented = orient(tris, &eu, pts)?;
    if !c.fill {
        return Ok(());
    }
    fill_holes(pts, tris, c, diag, max_listed, rep)
}

fn ambiguous(msg: String, hint: &str, reason: &'static str, at: Vec<Json>) -> Refusal {
    Refusal::new("meshGapAmbiguous", msg, hint).with("reason", reason).with("positions", Json::Arr(at))
}

fn boundary_vertices(tris: &[[u32; 3]], open: &[usize]) -> Vec<u32> {
    let mut v: Vec<u32> = open.iter().flat_map(|&h| [he_from(tris, h), he_to(tris, h)]).collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Refuse when the triangles have non-manifold edges that `before` did not have.
fn check_manifold(pts: &[Vec3], tris: &[[u32; 3]], max_listed: usize, what: &str) -> Result<(), Refusal> {
    let eu = edge_use(tris);
    if eu.nonmanifold.is_empty() {
        return Ok(());
    }
    let at: Vec<Json> = eu
        .nonmanifold
        .iter()
        .take(max_listed)
        .map(|&(h, _)| vec_json((pts[he_from(tris, h) as usize] + pts[he_to(tris, h) as usize]) * 0.5))
        .collect();
    Err(ambiguous(
        format!("{what} would make {} edges used by more than two triangles", eu.nonmanifold.len()),
        "lower close.tolerance so that only the cracks that should close do, or separate the surfaces that lie closer than the tolerance",
        "weldNonManifold",
        at,
    )
    .with("nonManifoldEdges", eu.nonmanifold.len() as f64))
}

/// Step 1: merge boundary vertices within `tol` that share no triangle.
fn gap_weld(pts: &mut Vec<Vec3>, tris: &mut Vec<[u32; 3]>, tol: f64, max_listed: usize, rep: &mut CloseReport) -> Result<(), Refusal> {
    let eu = edge_use(tris);
    let bv = boundary_vertices(tris, &eu.open);
    let adj = all_edges(tris);
    let (mut lo, _) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), 0);
    for &v in &bv {
        lo = lo.min(pts[v as usize]);
    }
    let cell = |q: Vec3| (((q.x - lo.x) / tol).floor() as i64, ((q.y - lo.y) / tol).floor() as i64, ((q.z - lo.z) / tol).floor() as i64);
    let mut cells: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    let mut members: HashMap<u32, Vec<u32>> = HashMap::new();
    let mut rep_of: HashMap<u32, u32> = HashMap::new();
    let t2 = tol * tol;
    for &v in &bv {
        let q = pts[v as usize];
        let k = cell(q);
        let mut best: Option<(f64, u32)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(rs) = cells.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) else { continue };
                    for &r in rs {
                        let d2 = pts[r as usize].dist2(q);
                        if d2 <= t2 && best.map_or(true, |b| d2 < b.0) && !members[&r].iter().any(|&m| adj.contains(&key(m, v))) {
                            best = Some((d2, r));
                        }
                    }
                }
            }
        }
        match best {
            Some((_, r)) => {
                members.get_mut(&r).unwrap().push(v);
                rep_of.insert(v, r);
            }
            None => {
                cells.entry(k).or_default().push(v);
                members.insert(v, vec![v]);
                rep_of.insert(v, v);
            }
        }
    }
    let merged: usize = members.values().map(|m| m.len() - 1).sum();
    if merged == 0 {
        return Ok(());
    }
    // the cluster's vertex is its centroid
    let mut new_pos: Vec<(u32, Vec3)> = vec![];
    for (&r, m) in &members {
        if m.len() > 1 {
            let s = m.iter().fold(Vec3::new(0.0, 0.0, 0.0), |a, &v| a + pts[v as usize]);
            new_pos.push((r, s / m.len() as f64));
        }
    }
    for (r, p) in new_pos {
        pts[r as usize] = p;
    }
    let before = tris.len();
    let mut out = Vec::with_capacity(before);
    for t in tris.iter() {
        let w = [rep_of.get(&t[0]).copied().unwrap_or(t[0]), rep_of.get(&t[1]).copied().unwrap_or(t[1]), rep_of.get(&t[2]).copied().unwrap_or(t[2])];
        if w[0] != w[1] && w[1] != w[2] && w[2] != w[0] {
            out.push(w);
        }
    }
    *tris = out;
    crate::drop_duplicates(tris);
    rep.dropped += before - tris.len();
    rep.merged_vertices += merged;
    check_manifold(pts, tris, max_listed, "welding the vertices on the gaps")
}

/// Step 2: boundary vertices on the interior of an open edge split the triangle on it.
fn stitch(pts: &mut Vec<Vec3>, tris: &mut Vec<[u32; 3]>, tol: f64, max_listed: usize, rep: &mut CloseReport) -> Result<(), Refusal> {
    for _pass in 0..8 {
        let eu = edge_use(tris);
        if eu.open.is_empty() {
            break;
        }
        let mut bv = boundary_vertices(tris, &eu.open);
        bv.sort_by(|&a, &b| pts[a as usize].x.total_cmp(&pts[b as usize].x));
        let xs: Vec<f64> = bv.iter().map(|&v| pts[v as usize].x).collect();
        let mut split: HashMap<usize, (usize, Vec<(f64, u32)>)> = HashMap::new();
        for &h in &eu.open {
            let t = h / 3;
            if split.contains_key(&t) {
                continue;
            }
            let (a, b) = (he_from(tris, h), he_to(tris, h));
            let (pa, pb) = (pts[a as usize], pts[b as usize]);
            let e = pb - pa;
            let l2 = e.len2();
            if l2 <= 0.0 {
                continue;
            }
            let (x0, x1) = (pa.x.min(pb.x) - tol, pa.x.max(pb.x) + tol);
            let start = xs.partition_point(|&x| x < x0);
            let mut cands: Vec<(f64, u32)> = vec![];
            for i in start..bv.len() {
                if xs[i] > x1 {
                    break;
                }
                let v = bv[i];
                if v == a || v == b || tris[t].contains(&v) {
                    continue;
                }
                let q = pts[v as usize];
                let s = (q - pa).dot(e) / l2;
                if s <= 0.0 || s >= 1.0 {
                    continue;
                }
                if (q - (pa + e * s)).len() <= tol {
                    cands.push((s, v));
                }
            }
            if !cands.is_empty() {
                cands.sort_by(|x, y| x.0.total_cmp(&y.0));
                cands.dedup_by_key(|c| c.1);
                split.insert(t, (h % 3, cands));
            }
        }
        if split.is_empty() {
            break;
        }
        let mut out = Vec::with_capacity(tris.len() + split.len());
        for (t, tri) in tris.iter().enumerate() {
            match split.get(&t) {
                None => out.push(*tri),
                Some((k, cands)) => {
                    let (p, q, r) = (tri[*k], tri[(*k + 1) % 3], tri[(*k + 2) % 3]);
                    let mut chain = vec![p];
                    chain.extend(cands.iter().map(|c| c.1));
                    chain.push(q);
                    for w in chain.windows(2) {
                        out.push([w[0], w[1], r]);
                    }
                    rep.stitched += cands.len();
                }
            }
        }
        *tris = out;
        let before = tris.len();
        crate::drop_duplicates(tris);
        rep.dropped += before - tris.len();
    }
    check_manifold(pts, tris, max_listed, "stitching the cracks")
}

/// Step 3: turn triangles so that every shared edge runs opposite; returns the number turned (the minority of each piece).
fn orient(tris: &mut [[u32; 3]], eu: &crate::EdgeUse, pts: &[Vec3]) -> Result<usize, Refusal> {
    let nt = tris.len();
    let mut comp = vec![u32::MAX; nt];
    let mut flip = vec![false; nt];
    let mut turned = 0;
    let mut ncomp = 0;
    let mut stack = vec![];
    for s in 0..nt {
        if comp[s] != u32::MAX {
            continue;
        }
        comp[s] = ncomp;
        stack.push(s);
        let mut members = vec![s];
        while let Some(t) = stack.pop() {
            for k in 0..3 {
                let h = 3 * t + k;
                let g = eu.twin[h];
                if g == usize::MAX {
                    continue;
                }
                let u = g / 3;
                let want = flip[t] ^ (he_from(tris, h) == he_from(tris, g));
                if comp[u] == u32::MAX {
                    comp[u] = ncomp;
                    flip[u] = want;
                    stack.push(u);
                    members.push(u);
                } else if flip[u] != want {
                    let m = (pts[he_from(tris, h) as usize] + pts[he_to(tris, h) as usize]) * 0.5;
                    return Err(Refusal::new(
                        "meshNonManifold",
                        "the surface is not orientable (a Moebius-like strip): it cannot bound a solid",
                        "the mesh intersects itself; repair it in a mesh tool",
                    )
                    .with("reason", "nonOrientable")
                    .with("positions", Json::Arr(vec![vec_json(m)])));
                }
            }
        }
        let nf = members.iter().filter(|&&t| flip[t]).count();
        // turn the minority only (the piece keeps the orientation most of its triangles came in with)
        let turn_flipped = 2 * nf <= members.len();
        for &t in &members {
            if flip[t] == turn_flipped {
                tris[t].swap(1, 2);
                turned += 1;
            }
        }
        ncomp += 1;
    }
    Ok(turned)
}

// ------------------------------------------------------------------ holes

struct Fail {
    code: &'static str,
    reason: &'static str,
    msg: String,
    hint: String,
    edges: usize,
    area: f64,
    at: Vec3,
}

fn fill_holes(pts: &[Vec3], tris: &mut Vec<[u32; 3]>, c: &CloseOpts, diag: f64, max_listed: usize, rep: &mut CloseReport) -> Result<(), Refusal> {
    let eu = edge_use(tris);
    let mut out_of: HashMap<u32, Vec<usize>> = HashMap::new();
    for &h in &eu.open {
        out_of.entry(he_from(tris, h)).or_default().push(h);
    }
    let mesh_area: f64 = tris.iter().map(|t| area_of(pts, t)).sum();
    let mut fails: Vec<Fail> = vec![];
    // loops
    let mut loops: Vec<Vec<u32>> = vec![];
    let mut used: HashSet<usize> = HashSet::new();
    let mut pinched: HashSet<u32> = HashSet::new();
    let mut ordered = eu.open.clone();
    ordered.sort_unstable();
    for &h0 in &ordered {
        if used.contains(&h0) {
            continue;
        }
        let start = he_from(tris, h0);
        let mut lp = vec![start];
        let mut h = h0;
        let mut ok = true;
        loop {
            used.insert(h);
            let nv = he_to(tris, h);
            if nv == start {
                break;
            }
            let outs = out_of.get(&nv).map(|v| v.as_slice()).unwrap_or(&[]);
            if outs.len() != 1 {
                let at = pts[nv as usize];
                if outs.is_empty() {
                    fails.push(Fail { code: "meshGapAmbiguous", reason: "brokenLoop", msg: "a chain of open edges does not close up".into(), hint: "the open edges do not form closed loops (an edge between more than two triangles, or an inconsistent surface); repair the mesh".into(), edges: lp.len(), area: 0.0, at });
                } else if pinched.insert(nv) {
                    fails.push(Fail { code: "meshGapAmbiguous", reason: "pinchedLoop", msg: format!("{} holes touch at one vertex: which loop each open edge belongs to is ambiguous", outs.len()), hint: "separate the holes (a vertex shared by two holes), or fill them in a mesh tool".into(), edges: lp.len(), area: 0.0, at });
                }
                ok = false;
                break;
            }
            h = outs[0];
            if used.contains(&h) {
                ok = false;
                break;
            }
            lp.push(he_from(tris, h));
        }
        if ok {
            loops.push(lp);
        }
    }
    let exists_base = all_edges(tris);
    let mut fill_edges: HashSet<(u32, u32)> = HashSet::new();
    let grid = Grid::new(pts, tris, diag);
    let eps = 1e-9 * diag;
    let mut fills: Vec<[u32; 3]> = vec![];
    let mut new_tris: Vec<[u32; 3]> = vec![];
    let _ = &mut fills;
    for lp in &loops {
        match fill_one(pts, lp, c, mesh_area, diag, eps, &|a, b| exists_base.contains(&key(a, b)) || fill_edges.contains(&key(a, b)), &grid, tris, &new_tris) {
            Ok((tri, mut info)) => {
                info.fill = tri.iter().map(|t| [pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]]).collect();
                for t in &tri {
                    for k in 0..3 {
                        fill_edges.insert(key(t[k], t[(k + 1) % 3]));
                    }
                }
                rep.filled_area += tri.iter().map(|t| area_of(pts, t)).sum::<f64>();
                rep.filled_triangles += tri.len();
                rep.holes.push(info);
                new_tris.extend(tri);
            }
            Err(f) => fails.push(f),
        }
    }
    if let Some(first) = fails.first() {
        let list: Vec<Json> = fails
            .iter()
            .take(max_listed)
            .map(|f| Obj::new().put("code", f.code).put("reason", f.reason).put("edges", f.edges as f64).put("area", f.area).put("at", vec_json(f.at)).build())
            .collect();
        let at: Vec<Json> = fails.iter().take(max_listed).map(|f| vec_json(f.at)).collect();
        let msg = if fails.len() == 1 { first.msg.clone() } else { format!("{} of {} holes cannot be closed; first: {}", fails.len(), loops.len() + fails.len() - rep.holes.len().min(loops.len()), first.msg) };
        return Err(Refusal::new(first.code, msg, first.hint.clone())
            .with("reason", first.reason)
            .with("failedHoles", fails.len() as f64)
            .with("holes", Json::Arr(list))
            .with("positions", Json::Arr(at)));
    }
    tris.extend(new_tris);
    Ok(())
}

fn fail(code: &'static str, reason: &'static str, msg: String, hint: &str, edges: usize, area: f64, at: Vec3) -> Fail {
    Fail { code, reason, msg, hint: hint.into(), edges, area, at }
}

#[allow(clippy::too_many_arguments)]
fn fill_one(
    pts: &[Vec3],
    lp: &[u32],
    c: &CloseOpts,
    mesh_area: f64,
    diag: f64,
    eps: f64,
    exists: &dyn Fn(u32, u32) -> bool,
    grid: &Grid,
    tris: &[[u32; 3]],
    prior: &[[u32; 3]],
) -> Result<(Vec<[u32; 3]>, HoleInfo), Fail> {
    let n = lp.len();
    let p: Vec<Vec3> = lp.iter().map(|&v| pts[v as usize]).collect();
    let cen = p.iter().fold(Vec3::new(0.0, 0.0, 0.0), |a, &q| a + q) / n as f64;
    let mut nw = Vec3::new(0.0, 0.0, 0.0);
    for i in 0..n {
        nw = nw + (p[i] - cen).cross(p[(i + 1) % n] - cen);
    }
    let proj_area = nw.len() * 0.5;
    let diam = p.iter().map(|q| q.dist(cen)).fold(0.0, f64::max) * 2.0;
    let limit_area = c.max_hole_area * mesh_area;
    let too_large = |reason: &'static str, msg: String, area: f64| {
        fail("meshHoleTooLarge", reason, msg, "raise close.maxHoleEdges / close.maxHoleArea if this hole should be filled, or close it in a mesh tool", n, area, cen)
    };
    if n > c.max_hole_edges {
        return Err(too_large("edges", format!("a hole of {n} edges is above the limit of {} edges", c.max_hole_edges), proj_area));
    }
    if proj_area > limit_area {
        return Err(too_large(
            "area",
            format!("a hole of area {proj_area:.4} is above {:.0} % of the mesh surface ({mesh_area:.4})", c.max_hole_area * 100.0),
            proj_area,
        ));
    }
    // poly: the loop reversed, so that its triangles run opposite to the neighbouring ones
    let poly: Vec<u32> = lp.iter().rev().copied().collect();
    let pp: Vec<Vec3> = poly.iter().map(|&v| pts[v as usize]).collect();
    // the loop's plane: the Newell normal, or (a bow tie has no net area) the normal of the widest triple of its vertices
    let oriented = proj_area > 1e-12 * diam * diam;
    let nf = if oriented {
        -(nw / nw.len())
    } else {
        let a = *p.iter().max_by(|x, y| x.dist(cen).total_cmp(&y.dist(cen))).unwrap() - cen;
        let b = p.iter().map(|q| a.cross(*q - cen)).max_by(|x, y| x.len().total_cmp(&y.len())).unwrap();
        if b.len() > 1e-12 * diam * diam { b / b.len() } else { Vec3::new(0.0, 0.0, 0.0) }
    };
    let have_plane = nf.len() > 0.5;
    let dev = if have_plane { pp.iter().map(|q| (*q - cen).dot(nf).abs()).fold(0.0, f64::max) } else { diam };
    let planar = have_plane && dev <= c.planar_tol * diam;
    let (tri, method): (Vec<[u32; 3]>, &'static str) = if n == 3 {
        (vec![[poly[0], poly[1], poly[2]]], if planar { "ear" } else { "minArea" })
    } else if planar || n > CloseOpts::DP_LIMIT {
        if !have_plane {
            return Err(fail("meshHoleSelfIntersects", "loopCrossesItself", format!("a hole of {n} edges has no area (its loop is a line)"), "the boundary of this hole folds onto itself; repair it in a mesh tool", n, 0.0, cen));
        }
        let u = if nf.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
        let u = (u - nf * u.dot(nf)).normalize();
        let v = nf.cross(u);
        let p2: Vec<(f64, f64)> = pp.iter().map(|q| ((*q - cen).dot(u), (*q - cen).dot(v))).collect();
        if !oriented || !simple_polygon(&p2, 1e-12 * diam * diam) {
            return Err(fail(
                "meshHoleSelfIntersects",
                "loopCrossesItself",
                format!("the boundary of a hole of {n} edges crosses itself when seen along its best-fit plane"),
                "twisted or overlapping geometry around this hole; repair it in a mesh tool (the fill would be ambiguous)",
                n,
                proj_area,
                cen,
            ));
        }
        match ear_clip(&poly, &p2, exists, diam) {
            Some(t) => (t, if planar { "ear" } else { "earProjected" }),
            None => {
                return Err(fail("meshHoleSelfIntersects", "noTriangulation", format!("no valid triangulation of a hole of {n} edges (every cap would reuse an existing edge or cut across the loop)"), "repair this hole in a mesh tool", n, proj_area, cen))
            }
        }
    } else {
        match min_area(&poly, &pp, exists, diam) {
            Some(t) => (t, "minArea"),
            None => return Err(fail("meshHoleSelfIntersects", "noTriangulation", format!("no valid triangulation of a hole of {n} edges (every cap would reuse an existing edge)"), "repair this hole in a mesh tool", n, proj_area, cen)),
        }
    };
    let area: f64 = tri.iter().map(|t| area_of(pts, t)).sum();
    if area > limit_area {
        return Err(too_large("area", format!("a hole of area {area:.4} is above {:.0} % of the mesh surface ({mesh_area:.4})", c.max_hole_area * 100.0), area));
    }
    // the fill must not cross itself or anything else
    let at_fail = |why: &'static str, msg: String| fail("meshHoleSelfIntersects", why, msg, "the cap of this hole would pass through other parts of the mesh: close it in a mesh tool, or lower close.maxHoleArea", n, area, cen);
    for i in 0..tri.len() {
        for j in 0..i {
            if improper(pts, &tri[i], &tri[j], eps) {
                return Err(at_fail("fillIntersectsItself", format!("the cap of a hole of {n} edges would cross itself")));
            }
        }
        for q in prior {
            if improper(pts, &tri[i], q, eps) {
                return Err(at_fail("fillIntersectsFill", format!("the cap of a hole of {n} edges would cross the cap of another hole")));
            }
        }
        let (lo, hi) = tri_bbox(pts, &tri[i], eps);
        let mut hit = false;
        grid.query(lo, hi, |m| {
            if !hit && improper(pts, &tri[i], &tris[m as usize], eps) {
                hit = true;
            }
        });
        if hit {
            return Err(at_fail("fillIntersectsMesh", format!("the cap of a hole of {n} edges would cut through the mesh")));
        }
    }
    let _ = diag;
    let info = HoleInfo { edges: n, area, planar, max_plane_dev: dev, triangles: tri.len(), method, at: cen, fill: vec![] };
    Ok((tri, info))
}

fn simple_polygon(p: &[(f64, f64)], eps: f64) -> bool {
    let n = p.len();
    let cr = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    for i in 0..n {
        let (a, b) = (p[i], p[(i + 1) % n]);
        for j in i + 1..n {
            if j == i + 1 || (i == 0 && j == n - 1) {
                continue;
            }
            let (c, d) = (p[j], p[(j + 1) % n]);
            let (o1, o2, o3, o4) = (cr(a, b, c), cr(a, b, d), cr(c, d, a), cr(c, d, b));
            if o1 * o2 < -eps * eps && o3 * o4 < -eps * eps {
                return false;
            }
        }
    }
    true
}

/// Ear clipping of a counter-clockwise polygon (`p2` its 2D coordinates). Ears are tried best-shaped first; a chord that is
/// already an edge of the mesh is not used (it would make the edge non-manifold).
fn ear_clip(poly: &[u32], p2: &[(f64, f64)], exists: &dyn Fn(u32, u32) -> bool, diam: f64) -> Option<Vec<[u32; 3]>> {
    let n = poly.len();
    let mut list: Vec<usize> = (0..n).collect();
    let mut out = vec![];
    let mut chords: HashSet<(u32, u32)> = HashSet::new();
    let eps = 1e-12 * diam * diam;
    let cr = |a: usize, b: usize, c: usize| (p2[b].0 - p2[a].0) * (p2[c].1 - p2[a].1) - (p2[b].1 - p2[a].1) * (p2[c].0 - p2[a].0);
    let d2 = |a: usize, b: usize| (p2[a].0 - p2[b].0).powi(2) + (p2[a].1 - p2[b].1).powi(2);
    while list.len() > 3 {
        let m = list.len();
        let mut cands: Vec<(f64, usize)> = vec![];
        for i in 0..m {
            let (a, b, c) = (list[(i + m - 1) % m], list[i], list[(i + 1) % m]);
            let area2 = cr(a, b, c);
            if area2 <= eps {
                continue;
            }
            let q = 2.0 * 3f64.sqrt() * area2 / (d2(a, b) + d2(b, c) + d2(c, a));
            cands.push((q, i));
        }
        cands.sort_by(|x, y| y.0.total_cmp(&x.0));
        let mut done = false;
        for &(_, i) in &cands {
            let (a, b, c) = (list[(i + m - 1) % m], list[i], list[(i + 1) % m]);
            let ck = key(poly[a], poly[c]);
            if exists(poly[a], poly[c]) || chords.contains(&ck) {
                continue;
            }
            let inside = list.iter().any(|&w| w != a && w != b && w != c && cr(a, b, w) >= -eps && cr(b, c, w) >= -eps && cr(c, a, w) >= -eps);
            if inside {
                continue;
            }
            out.push([poly[a], poly[b], poly[c]]);
            chords.insert(ck);
            list.remove(i);
            done = true;
            break;
        }
        if !done {
            return None;
        }
    }
    let (a, b, c) = (list[0], list[1], list[2]);
    if cr(a, b, c) <= eps {
        return None;
    }
    out.push([poly[a], poly[b], poly[c]]);
    Some(out)
}

/// Minimum-area triangulation of a polygon in space (Barequet and Sharir's dynamic programme): the chords are never
/// edges the mesh already has.
fn min_area(poly: &[u32], p: &[Vec3], exists: &dyn Fn(u32, u32) -> bool, diam: f64) -> Option<Vec<[u32; 3]>> {
    let n = poly.len();
    let inf = f64::INFINITY;
    let mut w = vec![vec![inf; n]; n];
    let mut kk = vec![vec![usize::MAX; n]; n];
    for i in 0..n - 1 {
        w[i][i + 1] = 0.0;
    }
    for len in 2..n {
        for i in 0..n - len {
            let j = i + len;
            if !(i == 0 && j == n - 1) && exists(poly[i], poly[j]) {
                continue;
            }
            for k in i + 1..j {
                if w[i][k] == inf || w[k][j] == inf {
                    continue;
                }
                let cross = (p[k] - p[i]).cross(p[j] - p[i]);
                let a = cross.len() * 0.5;
                let per = p[i].dist(p[k]) + p[k].dist(p[j]) + p[j].dist(p[i]);
                let pen = if a <= 1e-9 * diam * diam { 1e6 * diam * diam } else { 0.0 };
                let s = w[i][k] + w[k][j] + a + 1e-6 * diam * per + pen;
                if s < w[i][j] {
                    w[i][j] = s;
                    kk[i][j] = k;
                }
            }
        }
    }
    if w[0][n - 1] == inf || w[0][n - 1] >= 1e5 * diam * diam {
        return None;
    }
    let mut out = vec![];
    let mut st = vec![(0usize, n - 1)];
    while let Some((i, j)) = st.pop() {
        if j == i + 1 {
            continue;
        }
        let k = kk[i][j];
        out.push([poly[i], poly[k], poly[j]]);
        st.push((i, k));
        st.push((k, j));
    }
    Some(out)
}

// ------------------------------------------------------------------ triangle intersection

fn tri_bbox(pts: &[Vec3], t: &[u32; 3], pad: f64) -> (Vec3, Vec3) {
    let a = pts[t[0] as usize];
    let (b, c) = (pts[t[1] as usize], pts[t[2] as usize]);
    let pad = Vec3::new(pad, pad, pad);
    (a.min(b).min(c) - pad, a.max(b).max(c) + pad)
}

/// Uniform grid over the triangles' boxes.
struct Grid {
    lo: Vec3,
    cell: f64,
    cells: HashMap<(i32, i32, i32), Vec<u32>>,
    big: Vec<u32>,
}

impl Grid {
    fn new(pts: &[Vec3], tris: &[[u32; 3]], diag: f64) -> Grid {
        let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut ext = 0.0;
        for t in tris {
            let (l, h) = tri_bbox(pts, t, 0.0);
            lo = lo.min(l);
            ext += (h - l).x.max((h - l).y).max((h - l).z);
        }
        let cell = (2.0 * ext / tris.len().max(1) as f64).max(diag / 256.0).max(1e-300);
        let mut g = Grid { lo, cell, cells: HashMap::new(), big: vec![] };
        for (i, t) in tris.iter().enumerate() {
            let (l, h) = tri_bbox(pts, t, 0.0);
            let (a, b) = (g.key(l), g.key(h));
            let count = (b.0 - a.0 + 1) as i64 * (b.1 - a.1 + 1) as i64 * (b.2 - a.2 + 1) as i64;
            if count > 64 {
                g.big.push(i as u32);
                continue;
            }
            for x in a.0..=b.0 {
                for y in a.1..=b.1 {
                    for z in a.2..=b.2 {
                        g.cells.entry((x, y, z)).or_default().push(i as u32);
                    }
                }
            }
        }
        g
    }
    fn key(&self, q: Vec3) -> (i32, i32, i32) {
        (((q.x - self.lo.x) / self.cell).floor() as i32, ((q.y - self.lo.y) / self.cell).floor() as i32, ((q.z - self.lo.z) / self.cell).floor() as i32)
    }
    /// Triangles whose cells meet the box (may repeat).
    fn query(&self, lo: Vec3, hi: Vec3, mut f: impl FnMut(u32)) {
        for &m in &self.big {
            f(m);
        }
        let (a, b) = (self.key(lo), self.key(hi));
        let count = (b.0 - a.0 + 1) as i64 * (b.1 - a.1 + 1) as i64 * (b.2 - a.2 + 1) as i64;
        if count > 4096 {
            for v in self.cells.values() {
                v.iter().for_each(|&m| f(m));
            }
            return;
        }
        for x in a.0..=b.0 {
            for y in a.1..=b.1 {
                for z in a.2..=b.2 {
                    if let Some(v) = self.cells.get(&(x, y, z)) {
                        v.iter().for_each(|&m| f(m));
                    }
                }
            }
        }
    }
}

/// Segment `p`-`q` crosses the interior of triangle `t` (touching and lying in its plane do not count).
fn seg_tri(p: Vec3, q: Vec3, t: [Vec3; 3], eps: f64) -> bool {
    let n = (t[1] - t[0]).cross(t[2] - t[0]);
    let nl = n.len();
    if nl <= 1e-300 {
        return false;
    }
    let nn = n / nl;
    let (dp, dq) = ((p - t[0]).dot(nn), (q - t[0]).dot(nn));
    if dp * dq >= 0.0 || dp.abs() <= eps || dq.abs() <= eps {
        return false;
    }
    let s = dp / (dp - dq);
    let h = p + (q - p) * s;
    (0..3).all(|k| {
        let (u, v) = (t[k], t[(k + 1) % 3]);
        (v - u).cross(h - u).dot(nn) > eps * (v - u).len()
    })
}

/// Two triangles that meet where they should not: they cross, overlap in a plane, or fold back onto each other.
/// Triangles that share a vertex or an edge only meet properly along it.
fn improper(pts: &[Vec3], a: &[u32; 3], b: &[u32; 3], eps: f64) -> bool {
    let pa = [pts[a[0] as usize], pts[a[1] as usize], pts[a[2] as usize]];
    let pb = [pts[b[0] as usize], pts[b[1] as usize], pts[b[2] as usize]];
    let shared: Vec<(usize, usize)> = (0..3).flat_map(|i| (0..3).filter(move |&j| a[i] == b[j]).map(move |j| (i, j))).collect();
    match shared.len() {
        3 => true,
        2 => {
            let oa = (0..3).find(|i| !shared.iter().any(|s| s.0 == *i)).unwrap();
            let ob = (0..3).find(|j| !shared.iter().any(|s| s.1 == *j)).unwrap();
            let (p, q) = (pa[shared[0].0], pa[shared[1].0]);
            let e = q - p;
            let na = e.cross(pa[oa] - p);
            if na.len() <= 1e-300 || e.len() <= 1e-300 {
                return false;
            }
            let nn = na / na.len();
            if (pb[ob] - p).dot(nn).abs() > eps {
                return false;
            }
            let m = nn.cross(e / e.len());
            (pa[oa] - p).dot(m) * (pb[ob] - p).dot(m) > 0.0
        }
        1 => {
            let (ia, ib) = shared[0];
            let v = pa[ia];
            let (a1, a2) = (pa[(ia + 1) % 3], pa[(ia + 2) % 3]);
            let (b1, b2) = (pb[(ib + 1) % 3], pb[(ib + 2) % 3]);
            if seg_tri(a1, a2, pb, eps) || seg_tri(b1, b2, pa, eps) {
                return true;
            }
            let n = (a1 - v).cross(a2 - v);
            if n.len() <= 1e-300 {
                return false;
            }
            let nn = n / n.len();
            if [b1, b2].iter().any(|q| (*q - v).dot(nn).abs() > eps) {
                return false;
            }
            // coplanar: do the two wedges at the shared vertex overlap?
            let dir = |x: Vec3| (x - v).normalize();
            let (da1, da2, db1, db2) = (dir(a1), dir(a2), dir(b1), dir(b2));
            let cr = |x: Vec3, y: Vec3| x.cross(y).dot(nn);
            let inside = |d: Vec3, w1: Vec3, w2: Vec3| {
                let s = cr(w1, w2).signum();
                cr(w1, d) * s > 1e-9 && cr(d, w2) * s > 1e-9
            };
            let bis = |x: Vec3, y: Vec3| (x + y).try_normalize();
            inside(db1, da1, da2)
                || inside(db2, da1, da2)
                || inside(da1, db1, db2)
                || inside(da2, db1, db2)
                || bis(db1, db2).map_or(false, |d| inside(d, da1, da2))
                || bis(da1, da2).map_or(false, |d| inside(d, db1, db2))
        }
        _ => {
            for k in 0..3 {
                if seg_tri(pa[k], pa[(k + 1) % 3], pb, eps) || seg_tri(pb[k], pb[(k + 1) % 3], pa, eps) {
                    return true;
                }
            }
            // coplanar overlap
            let n = (pa[1] - pa[0]).cross(pa[2] - pa[0]);
            if n.len() <= 1e-300 {
                return false;
            }
            let nn = n / n.len();
            if pb.iter().any(|q| (*q - pa[0]).dot(nn).abs() > eps) {
                return false;
            }
            let o = |p: Vec3, q: Vec3, r: Vec3| (q - p).cross(r - p).dot(nn);
            let inside = |x: Vec3, t: &[Vec3; 3]| {
                let s = o(t[0], t[1], t[2]).signum();
                (0..3).all(|k| o(t[k], t[(k + 1) % 3], x) * s > eps * (t[(k + 1) % 3] - t[k]).len())
            };
            for i in 0..3 {
                for j in 0..3 {
                    let (p, q, r, s) = (pa[i], pa[(i + 1) % 3], pb[j], pb[(j + 1) % 3]);
                    let tol = eps * (q - p).len().max((s - r).len());
                    if o(p, q, r) * o(p, q, s) < -tol * tol && o(r, s, p) * o(r, s, q) < -tol * tol {
                        return true;
                    }
                }
            }
            pb.iter().any(|&x| inside(x, &pa)) || pa.iter().any(|&x| inside(x, &pb))
        }
    }
}
