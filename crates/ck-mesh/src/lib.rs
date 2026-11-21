//! Triangle mesh to B-rep solid.
//!
//! Input: a triangle soup (positions, optionally indexed), as STL / OBJ / the kit's imported mesh records carry it.
//! Output: a valid [`Body`] of planar faces (coplanar triangles merged into one face, holes kept as inner loops), one
//! shell per closed component, voids as inner shells of the lump that holds them; or a [`Refusal`] with a stable code.
//! With [`Opts::analytic`] the faceted body is then refit with exact cylinders, cones and spheres where the facets
//! support it ([`analytic`]); what does not fit stays faceted, so the result is valid either way.
//!
//! Steps (each reported in [`Report`]):
//! 1. weld vertices closer than `weld_tol` (default 1e-6 of the bounding-box diagonal), drop collapsed triangles and
//!    duplicate triangles (an opposite pair is an internal double wall and both go);
//! 2. edge use count: an edge used once is open (`meshNotWatertight`, with the positions of the gaps), more than twice is
//!    non-manifold (`meshNonManifold`);
//! 3. orientation: every component is made consistent from its first triangle, then turned outward (positive volume);
//!    a component inside another (ray parity) is a void and faces inward;
//! 4. coplanar region growing (seeds by area, a neighbour joins when its normal is within `angle_tol_deg` of the seed
//!    plane and its corners within `plane_tol` of it); sliver triangles (height below `plane_tol`) join a neighbour;
//! 5. region boundary loops traced on the half-edges, vertices between only two regions removed (they are on a straight
//!    edge), one planar face per region;
//! 6. validation (ck-check) and, only when it finds something, ck-heal; the volume is checked against the mesh.
//!
//! The wire request and its report: docs/agent-queries.md, "meshToSolid". Design references: THIRD_PARTY.md.

pub mod analytic;
pub mod blob;
pub mod close;
pub mod decimate;
pub mod deviation;
pub mod request;

use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceName, ShellId, VertexId};
use std::collections::HashMap;

/// Options. Tolerances are model units (mm); `None` means "relative to the bounding-box diagonal".
#[derive(Debug, Clone)]
pub struct Opts {
    /// Vertices closer than this are one vertex (default 1e-6 x diagonal).
    pub weld_tol: Option<f64>,
    /// Largest distance of a triangle corner from the plane of the face it is merged into (default 1e-5 x diagonal).
    pub plane_tol: Option<f64>,
    /// Largest angle between a triangle's normal and the plane of the face it is merged into (degrees).
    pub angle_tol_deg: f64,
    /// Most triangles accepted (`meshTooLarge` above).
    pub max_triangles: usize,
    /// Most faces after coplanar merging (`meshTooLarge` above): a scan of a free-form surface stays one face per triangle.
    pub max_faces: usize,
    /// Refit cylinders, cones and spheres after merging.
    pub analytic: bool,
    pub fit: analytic::FitParams,
    /// Measure the deviation of the result from the mesh (sampled, both ways).
    pub deviation: bool,
    /// Most positions listed in a refusal.
    pub max_listed: usize,
    /// Close the mesh before the watertight check (weld gaps, stitch cracks, fill holes). `None` (the default): an open
    /// mesh is refused.
    pub close: Option<close::CloseOpts>,
    /// Quadric decimation within a tolerance right after welding (`opts.decimate`); `None`: not asked for.
    pub decimate: Option<decimate::DecimateOpts>,
    /// Stop after the closing stage and the watertight check ([`preview`]): nothing is oriented, merged or built.
    pub dry_run: bool,
}

impl Default for Opts {
    fn default() -> Self {
        Opts {
            weld_tol: None,
            plane_tol: None,
            angle_tol_deg: 0.05,
            max_triangles: 200_000,
            max_faces: 50_000,
            analytic: true,
            fit: analytic::FitParams::default(),
            deviation: true,
            max_listed: 20,
            close: None,
            decimate: None,
            dry_run: false,
        }
    }
}

/// A refusal: stable `code`, human `message`, what to do (`hint`), and data (counts, positions).
#[derive(Debug, Clone)]
pub struct Refusal {
    pub code: &'static str,
    pub message: String,
    pub hint: String,
    pub data: Vec<(&'static str, ck_json::Json)>,
}

impl Refusal {
    fn new(code: &'static str, message: impl Into<String>, hint: impl Into<String>) -> Refusal {
        Refusal { code, message: message.into(), hint: hint.into(), data: vec![] }
    }
    fn with(mut self, k: &'static str, v: impl Into<ck_json::Json>) -> Refusal {
        self.data.push((k, v.into()));
        self
    }
    pub fn get_f64(&self, k: &str) -> Option<f64> {
        self.data.iter().find(|(n, _)| *n == k).and_then(|(_, v)| v.as_f64())
    }
}

/// What was done.
#[derive(Debug, Clone, Default)]
pub struct Report {
    pub input_triangles: usize,
    pub input_vertices: usize,
    pub welded_vertices: usize,
    pub weld_tol: f64,
    pub plane_tol: f64,
    pub angle_tol_deg: f64,
    pub diagonal: f64,
    /// Triangles with two corners welded together (dropped).
    pub collapsed: usize,
    /// Duplicate triangles dropped (same corners).
    pub duplicates: usize,
    /// Zero-height triangles (three distinct corners on a line) merged into a neighbour.
    pub slivers: usize,
    /// Triangles turned to agree with their component.
    pub flipped: usize,
    /// Components that were inside out (turned as a whole).
    pub inside_out: usize,
    pub components: usize,
    pub lumps: usize,
    pub voids: usize,
    /// Planar faces after merging (before the analytic refit).
    pub planar_faces: usize,
    pub mesh_volume: f64,
    pub body_volume: f64,
    /// Validation issues of the result (0: valid) and ck-heal fixes applied to get there.
    pub issues: usize,
    pub heal_fixes: usize,
    pub analytic: Option<analytic::AnalyticReport>,
    pub deviation: Option<deviation::Deviation>,
    /// What the closing stage did (None: not asked for).
    pub closing: Option<close::CloseReport>,
    /// What the decimation stage did (None: not asked for).
    pub decimation: Option<decimate::DecimateReport>,
    /// Host-clock time of each stage in ms (`weld`, `close`, `orient`, `regions`, `faces`, `refit`, `validate`, `deviation`),
    /// in order; empty when the host has no clock (`ck_core::budget::set_clock`).
    pub stages: Vec<(&'static str, f64)>,
    /// The faceted body had more than [`QUICK_VALIDATION_FACES`] faces and got the quick validation.
    pub quick_validation: bool,
}

/// Above this many faces a body built straight from the mesh gets the quick validation.
pub const QUICK_VALIDATION_FACES: usize = 5000;

impl Report {
    /// Note the time since `*mark` as stage `name` and restart the mark.
    pub fn lap(&mut self, name: &'static str, mark: &mut Option<f64>) {
        if mark.is_some() {
            self.stages.push((name, ck_core::budget::since(*mark)));
            *mark = ck_core::budget::mark();
        }
    }

    pub fn volume_rel_err(&self) -> f64 {
        if self.mesh_volume.abs() > 0.0 {
            (self.body_volume - self.mesh_volume) / self.mesh_volume.abs()
        } else {
            0.0
        }
    }
}

pub struct Converted {
    pub body: Body,
    pub report: Report,
}

/// A cleaned, welded, oriented, closed triangle mesh.
#[derive(Debug, Clone)]
pub struct Closed {
    pub pts: Vec<Vec3>,
    pub tris: Vec<[u32; 3]>,
    /// Component of each triangle.
    pub comp: Vec<u32>,
    /// Per component: signed volume (after orientation), nesting depth, the component it sits directly in.
    pub comps: Vec<Comp>,
}

#[derive(Debug, Clone)]
pub struct Comp {
    pub volume: f64,
    pub depth: usize,
    pub parent: Option<usize>,
}

impl Closed {
    pub fn volume(&self) -> f64 {
        self.comps.iter().map(|c| c.volume).sum()
    }
}

// ------------------------------------------------------------------ welding

fn bbox(p: &[Vec3]) -> (Vec3, Vec3) {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = -lo;
    for &q in p {
        lo = lo.min(q);
        hi = hi.max(q);
    }
    (lo, hi)
}

/// Weld points within `tol` (grid of cell `tol`, 27 neighbour cells): (welded points, index of each input point).
fn weld(p: &[Vec3], tol: f64, lo: Vec3) -> (Vec<Vec3>, Vec<u32>) {
    let mut cells: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::with_capacity(p.len());
    let mut out: Vec<Vec3> = Vec::with_capacity(p.len() / 2);
    let mut map = Vec::with_capacity(p.len());
    let key = |q: Vec3| (((q.x - lo.x) / tol).floor() as i64, ((q.y - lo.y) / tol).floor() as i64, ((q.z - lo.z) / tol).floor() as i64);
    let t2 = tol * tol;
    for &q in p {
        let k = key(q);
        let mut found = None;
        'n: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(v) = cells.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
                        for &i in v {
                            if out[i as usize].dist2(q) <= t2 {
                                found = Some(i);
                                break 'n;
                            }
                        }
                    }
                }
            }
        }
        let i = found.unwrap_or_else(|| {
            let i = out.len() as u32;
            out.push(q);
            cells.entry(k).or_default().push(i);
            i
        });
        map.push(i);
    }
    (out, map)
}

// ------------------------------------------------------------------ half-edges

/// Half-edge `h = 3 t + k` runs from corner k to corner k + 1 of triangle t.
fn he_from(tris: &[[u32; 3]], h: usize) -> u32 {
    tris[h / 3][h % 3]
}
fn he_to(tris: &[[u32; 3]], h: usize) -> u32 {
    tris[h / 3][(h % 3 + 1) % 3]
}

struct EdgeUse {
    /// Twin of every half-edge (usize::MAX when the edge is open or non-manifold).
    twin: Vec<usize>,
    /// Half-edges of open edges (one per edge) and of non-manifold edges (one per edge, with its use count).
    open: Vec<usize>,
    nonmanifold: Vec<(usize, usize)>,
}

fn edge_use(tris: &[[u32; 3]]) -> EdgeUse {
    let n = tris.len() * 3;
    let mut keys: Vec<(u64, u32)> = (0..n)
        .map(|h| {
            let (a, b) = (he_from(tris, h), he_to(tris, h));
            let (lo, hi) = if a < b { (a, b) } else { (b, a) };
            (((lo as u64) << 32) | hi as u64, h as u32)
        })
        .collect();
    keys.sort_unstable();
    let mut twin = vec![usize::MAX; n];
    let (mut open, mut nonmanifold) = (vec![], vec![]);
    let mut i = 0;
    while i < keys.len() {
        let mut j = i + 1;
        while j < keys.len() && keys[j].0 == keys[i].0 {
            j += 1;
        }
        match j - i {
            1 => open.push(keys[i].1 as usize),
            2 => {
                let (a, b) = (keys[i].1 as usize, keys[i + 1].1 as usize);
                twin[a] = b;
                twin[b] = a;
            }
            m => nonmanifold.push((keys[i].1 as usize, m)),
        }
        i = j;
    }
    EdgeUse { twin, open, nonmanifold }
}

fn tri_cross(pts: &[Vec3], t: &[u32; 3]) -> Vec3 {
    let (a, b, c) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
    (b - a).cross(c - a)
}

fn vec_json(v: Vec3) -> ck_json::Json {
    ck_json::Json::Arr(vec![ck_json::Json::Num(v.x), ck_json::Json::Num(v.y), ck_json::Json::Num(v.z)])
}

/// Number of chains of open edges (holes), by union-find on their vertices.
fn open_loops(tris: &[[u32; 3]], open: &[usize]) -> usize {
    let mut parent: HashMap<u32, u32> = HashMap::new();
    fn find(p: &mut HashMap<u32, u32>, x: u32) -> u32 {
        let mut r = x;
        while let Some(&q) = p.get(&r) {
            if q == r {
                break;
            }
            r = q;
        }
        p.insert(x, r);
        r
    }
    for &h in open {
        let (a, b) = (he_from(tris, h), he_to(tris, h));
        parent.entry(a).or_insert(a);
        parent.entry(b).or_insert(b);
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb {
            parent.insert(ra, rb);
        }
    }
    let keys: Vec<u32> = parent.keys().copied().collect();
    let mut roots: Vec<u32> = keys.into_iter().map(|k| find(&mut parent, k)).collect();
    roots.sort_unstable();
    roots.dedup();
    roots.len()
}

/// Ray parity: is `p` inside the closed triangle set `tris` (any orientation)? Ray along a fixed skew direction.
fn inside(pts: &[Vec3], tris: &[[u32; 3]], p: Vec3) -> bool {
    let d = Vec3::new(0.5773, 0.5774, 0.5775).normalize();
    let mut hits = 0usize;
    for t in tris {
        let (a, b, c) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
        // Moller-Trumbore
        let (e1, e2) = (b - a, c - a);
        let pv = d.cross(e2);
        let det = e1.dot(pv);
        if det.abs() < 1e-300 {
            continue;
        }
        let inv = 1.0 / det;
        let s = p - a;
        let u = s.dot(pv) * inv;
        if !(0.0..=1.0).contains(&u) {
            continue;
        }
        let q = s.cross(e1);
        let v = d.dot(q) * inv;
        if v < 0.0 || u + v > 1.0 {
            continue;
        }
        if e2.dot(q) * inv > 0.0 {
            hits += 1;
        }
    }
    hits % 2 == 1
}

/// Drop duplicate triangles (same corner set; an opposite pair cancels, a wall between two glued solids, equal ones are kept
/// once); returns how many went.
pub(crate) fn drop_duplicates(tris: &mut Vec<[u32; 3]>) -> usize {
    let canon = |t: &[u32; 3]| -> ([u32; 3], bool) {
        let m = (0..3).min_by_key(|&k| t[k]).unwrap();
        let r = [t[m], t[(m + 1) % 3], t[(m + 2) % 3]];
        if r[1] < r[2] { (r, true) } else { ([r[0], r[2], r[1]], false) }
    };
    let mut order: Vec<(([u32; 3], bool), usize)> = tris.iter().enumerate().map(|(i, t)| (canon(t), i)).collect();
    order.sort_unstable();
    let mut keep = vec![true; tris.len()];
    let mut i = 0;
    while i < order.len() {
        let mut j = i + 1;
        while j < order.len() && order[j].0 .0 == order[i].0 .0 {
            j += 1;
        }
        if j - i > 1 {
            let pos: Vec<usize> = order[i..j].iter().filter(|x| x.0 .1).map(|x| x.1).collect();
            let neg: Vec<usize> = order[i..j].iter().filter(|x| !x.0 .1).map(|x| x.1).collect();
            for &k in pos.iter().chain(neg.iter()) {
                keep[k] = false;
            }
            if pos.len() > neg.len() {
                keep[pos[0]] = true;
            } else if neg.len() > pos.len() {
                keep[neg[0]] = true;
            }
        }
        i = j;
    }
    let before = tris.len();
    let old = std::mem::take(tris);
    *tris = old.into_iter().zip(keep).filter(|(_, k)| *k).map(|(t, _)| t).collect();
    before - tris.len()
}

/// Steps 1 to 3: weld, clean, check, orient, classify components. `positions` xyz, `indices` triangle corners (None: soup).
pub fn close_mesh(positions: &[f64], indices: Option<&[u32]>, o: &Opts, rep: &mut Report) -> Result<Closed, Refusal> {
    if positions.len() % 3 != 0 {
        return Err(Refusal::new("meshBadInput", format!("{} position values is not a multiple of 3", positions.len()), "send x, y, z per vertex"));
    }
    let nv = positions.len() / 3;
    let raw: Vec<Vec3> = (0..nv).map(|i| Vec3::new(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2])).collect();
    if let Some(i) = raw.iter().position(|p| !p.is_finite()) {
        return Err(Refusal::new("meshBadInput", format!("vertex {i} is not a finite point"), "remove NaN / infinite coordinates").with("vertex", i as f64));
    }
    let corners: Vec<u32> = match indices {
        Some(ix) => {
            if ix.len() % 3 != 0 {
                return Err(Refusal::new("meshBadInput", format!("{} indices is not a multiple of 3", ix.len()), "send three vertex indices per triangle"));
            }
            if let Some(&bad) = ix.iter().find(|&&i| i as usize >= nv) {
                return Err(Refusal::new("meshBadInput", format!("index {bad} is out of range ({nv} vertices)"), "indices count from 0").with("index", bad as f64));
            }
            ix.to_vec()
        }
        None => {
            if nv % 3 != 0 {
                return Err(Refusal::new("meshBadInput", format!("{nv} vertices of a triangle soup is not a multiple of 3"), "send nine values per triangle, or indices"));
            }
            (0..nv as u32).collect()
        }
    };
    let ntri = corners.len() / 3;
    rep.input_triangles = ntri;
    rep.input_vertices = nv;
    if ntri > o.max_triangles && o.decimate.is_none() {
        return Err(Refusal::new("meshTooLarge", format!("{ntri} triangles exceed the limit of {}", o.max_triangles), "set opts.decimate {tol} (quadric decimation within a deviation tolerance, or send the mesh through meshDecimate first), or raise opts.maxTriangles")
            .with("triangles", ntri as f64)
            .with("limit", o.max_triangles as f64));
    }
    if ntri == 0 {
        return Err(Refusal::new("meshDegenerate", "the mesh has no triangles", "send at least four triangles enclosing a volume"));
    }
    let used: Vec<Vec3> = {
        let mut seen = vec![false; nv];
        corners.iter().for_each(|&i| seen[i as usize] = true);
        raw.iter().zip(seen).filter(|(_, s)| *s).map(|(p, _)| *p).collect()
    };
    let (lo, hi) = bbox(&used);
    let diag = (hi - lo).len();
    rep.diagonal = diag;
    if !(diag > 0.0) {
        return Err(Refusal::new("meshDegenerate", "all vertices are at one point", "send a mesh enclosing a volume"));
    }
    if !(diag < 1e12) || !(lo.len().max(hi.len()) < 1e12) {
        return Err(Refusal::new("meshBadInput", format!("coordinates up to {:.3e} are out of range", lo.len().max(hi.len())), "send coordinates in model units (mm) below 1e12"));
    }
    let mut mark = ck_core::budget::mark();
    let weld_tol = o.weld_tol.filter(|v| *v > 0.0).unwrap_or(1e-6 * diag);
    rep.weld_tol = weld_tol;
    let (mut pts, map) = weld(&raw, weld_tol, lo);
    // welded triangles; collapsed ones go
    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(ntri);
    for t in corners.chunks_exact(3) {
        let w = [map[t[0] as usize], map[t[1] as usize], map[t[2] as usize]];
        if w[0] == w[1] || w[1] == w[2] || w[2] == w[0] {
            rep.collapsed += 1;
        } else {
            tris.push(w);
        }
    }
    rep.duplicates = drop_duplicates(&mut tris);
    rep.lap("weld", &mut mark);
    if let Some(r) = over_budget("welding") {
        return Err(r);
    }
    if let Some(d) = &o.decimate {
        rep.decimation = Some(decimate::apply(&mut pts, &mut tris, d)?);
        if tris.len() > o.max_triangles {
            return Err(Refusal::new("meshTooLarge", format!("{} triangles exceed the limit of {} even after decimation", tris.len(), o.max_triangles), "lower opts.decimate.targetTriangles or raise opts.maxTriangles")
                .with("triangles", tris.len() as f64)
                .with("limit", o.max_triangles as f64));
        }
        rep.lap("decimate", &mut mark);
        if let Some(r) = over_budget("decimation") {
            return Err(r);
        }
    }
    if let Some(co) = &o.close {
        let mut cr = close::CloseReport::default();
        let eu0 = edge_use(&tris);
        // non-manifold edges are refused below as before; closing only works on a manifold surface
        let res = if eu0.nonmanifold.is_empty() { close::close(&mut pts, &mut tris, co, diag, o.max_listed, &mut cr) } else { Ok(()) };
        rep.closing = Some(cr);
        res?;
        rep.lap("close", &mut mark);
        if let Some(r) = over_budget("closing") {
            return Err(r);
        }
    }
    // used welded vertices
    rep.welded_vertices = {
        let mut s = vec![false; pts.len()];
        tris.iter().flatten().for_each(|&i| s[i as usize] = true);
        s.iter().filter(|x| **x).count()
    };
    if tris.len() < 4 {
        return Err(Refusal::new("meshDegenerate", format!("only {} usable triangles after welding", tris.len()), "the mesh must enclose a volume; check the weld tolerance (opts.weldTol)")
            .with("triangles", tris.len() as f64));
    }
    let eu = edge_use(&tris);
    let mid = |h: usize| (pts[he_from(&tris, h) as usize] + pts[he_to(&tris, h) as usize]) * 0.5;
    if !eu.nonmanifold.is_empty() {
        let list: Vec<ck_json::Json> = eu.nonmanifold.iter().take(o.max_listed).map(|&(h, _)| vec_json(mid(h))).collect();
        let most = eu.nonmanifold.iter().map(|x| x.1).max().unwrap_or(0);
        return Err(Refusal::new(
            "meshNonManifold",
            format!("{} edges are used by more than two triangles (up to {most})", eu.nonmanifold.len()),
            "separate the bodies that touch along these edges, or remove internal faces",
        )
        .with("nonManifoldEdges", eu.nonmanifold.len() as f64)
        .with("openEdges", eu.open.len() as f64)
        .with("positions", ck_json::Json::Arr(list)));
    }
    if !eu.open.is_empty() {
        let list: Vec<ck_json::Json> = eu.open.iter().take(o.max_listed).map(|&h| vec_json(mid(h))).collect();
        let loops = open_loops(&tris, &eu.open);
        return Err(Refusal::new(
            "meshNotWatertight",
            format!("{} open edges in {loops} gap(s): the mesh does not enclose a volume", eu.open.len()),
            "set opts.close to weld cracks and fill small holes (the reply reports what was added), close the holes in a mesh tool, or raise opts.weldTol if the gaps are cracks between vertices that should be one",
        )
        .with("openEdges", eu.open.len() as f64)
        .with("openLoops", loops as f64)
        .with("positions", ck_json::Json::Arr(list)));
    }
    if o.dry_run {
        return Ok(Closed { pts, tris, comp: vec![], comps: vec![] });
    }
    if let Some(r) = over_budget("orienting") {
        return Err(r);
    }
    // orientation: flip[t] so that every twin pair runs opposite; components by the same walk
    let nt = tris.len();
    let mut comp = vec![u32::MAX; nt];
    let mut flip = vec![false; nt];
    let mut ncomp = 0u32;
    let mut stack = vec![];
    for s in 0..nt {
        if comp[s] != u32::MAX {
            continue;
        }
        comp[s] = ncomp;
        stack.push(s);
        while let Some(t) = stack.pop() {
            for k in 0..3 {
                let h = 3 * t + k;
                let g = eu.twin[h];
                let u = g / 3;
                // in the input the pair runs the same way: the neighbour needs the opposite flip
                let same = he_from(&tris, h) == he_from(&tris, g);
                let want = flip[t] ^ same;
                if comp[u] == u32::MAX {
                    comp[u] = ncomp;
                    flip[u] = want;
                    stack.push(u);
                } else if flip[u] != want {
                    return Err(Refusal::new(
                        "meshNonManifold",
                        "the surface is not orientable (a Moebius-like strip): it cannot bound a solid",
                        "the mesh intersects itself; repair it in a mesh tool",
                    )
                    .with("reason", "nonOrientable")
                    .with("positions", ck_json::Json::Arr(vec![vec_json(mid(h))])));
                }
            }
        }
        ncomp += 1;
    }
    let apply = |tris: &mut Vec<[u32; 3]>, t: usize| tris[t].swap(1, 2);
    for t in 0..nt {
        if flip[t] {
            apply(&mut tris, t);
        }
    }
    let ncomp = ncomp as usize;
    let centre = (lo + hi) * 0.5;
    let mut vols = vec![0.0; ncomp];
    for (t, tri) in tris.iter().enumerate() {
        let (a, b, c) = (pts[tri[0] as usize] - centre, pts[tri[1] as usize] - centre, pts[tri[2] as usize] - centre);
        vols[comp[t] as usize] += a.dot(b.cross(c)) / 6.0;
    }
    // components turned outward; flipped triangles counted against the input
    let mut ccount = vec![0usize; ncomp];
    comp.iter().for_each(|&c| ccount[c as usize] += 1);
    for c in 0..ncomp {
        // a closed component of no volume (two coincident sheets, or everything on a plane)
        let cb = {
            let ps: Vec<Vec3> = tris.iter().zip(&comp).filter(|(_, &k)| k as usize == c).flat_map(|(t, _)| t.iter().map(|&i| pts[i as usize])).collect();
            let (l, h) = bbox(&ps);
            (h - l).len()
        };
        if vols[c].abs() <= 1e-9 * cb * cb * cb {
            return Err(Refusal::new("meshDegenerate", format!("a closed component of {} triangles has no volume", ccount[c]), "remove flat or doubled sheets")
                .with("component", c as f64)
                .with("triangles", ccount[c] as f64));
        }
    }
    let outward: Vec<bool> = vols.iter().map(|v| *v > 0.0).collect();
    for t in 0..nt {
        if !outward[comp[t] as usize] {
            apply(&mut tris, t);
        }
    }
    // triangles the walk turned, per component (against the component's first triangle)
    let mut nflip = vec![0usize; ncomp];
    for t in 0..nt {
        if flip[t] {
            nflip[comp[t] as usize] += 1;
        }
    }
    for v in vols.iter_mut() {
        *v = v.abs();
    }
    // nesting: a component inside an odd number of others is a void
    let mut by_comp: Vec<Vec<[u32; 3]>> = vec![vec![]; ncomp];
    for (t, tri) in tris.iter().enumerate() {
        by_comp[comp[t] as usize].push(*tri);
    }
    let mut containers: Vec<Vec<usize>> = vec![vec![]; ncomp];
    if ncomp > 1 {
        for c in 0..ncomp {
            let p = pts[by_comp[c][0][0] as usize];
            for d in 0..ncomp {
                if d != c && vols[d] > vols[c] && inside(&pts, &by_comp[d], p) {
                    containers[c].push(d);
                }
            }
        }
    }
    let mut comps: Vec<Comp> = (0..ncomp).map(|c| Comp { volume: vols[c], depth: containers[c].len(), parent: None }).collect();
    for c in 0..ncomp {
        // the direct container: the deepest one
        comps[c].parent = containers[c].iter().copied().max_by_key(|&d| containers[d].len());
        if comps[c].depth % 2 == 1 {
            comps[c].volume = -vols[c];
        }
    }
    for t in 0..nt {
        if comps[comp[t] as usize].depth % 2 == 1 {
            apply(&mut tris, t);
        }
    }
    // report against the input: triangles that disagreed with the majority of their component were flipped; a component
    // whose majority had to turn came in inside out
    for c in 0..ncomp {
        let maj = 2 * nflip[c] > ccount[c];
        rep.flipped += nflip[c].min(ccount[c] - nflip[c]);
        let void = comps[c].depth % 2 == 1;
        if maj ^ !outward[c] ^ void {
            rep.inside_out += 1;
        }
    }
    rep.components = ncomp;
    rep.voids = comps.iter().filter(|c| c.depth % 2 == 1).count();
    rep.lumps = ncomp - rep.voids;
    rep.mesh_volume = comps.iter().map(|c| c.volume).sum();
    rep.lap("orient", &mut mark);
    Ok(Closed { pts, tris, comp, comps })
}

// ------------------------------------------------------------------ coplanar faces

/// One planar face: plane, loops (welded vertex indices; the first is the outer loop, counter-clockwise about `normal`).
#[derive(Debug, Clone)]
pub struct PlanarFace {
    pub normal: Vec3,
    pub point: Vec3,
    pub comp: u32,
    pub loops: Vec<Vec<u32>>,
    pub triangles: usize,
}

/// Coplanar regions of a closed mesh: region of every triangle, and per region its plane and component.
#[derive(Debug, Clone)]
pub struct Regions {
    pub region: Vec<u32>,
    pub n: usize,
    pub normal: Vec<Vec3>,
    pub point: Vec<Vec3>,
    pub comp: Vec<u32>,
    /// Triangles per region, and region area.
    pub count: Vec<usize>,
    pub area: Vec<f64>,
    /// Twin of every half-edge of the closed mesh.
    pub twin: Vec<usize>,
    /// Zero-height triangles (merged into a neighbour's region).
    pub sliver: Vec<bool>,
}

/// Step 4: coplanar regions.
pub fn planar_regions(m: &Closed, plane_tol: f64, angle_tol_deg: f64, rep: &mut Report) -> Result<Regions, Refusal> {
    let (pts, tris) = (&m.pts, &m.tris);
    let nt = tris.len();
    let eu = edge_use(tris);
    let cross: Vec<Vec3> = tris.iter().map(|t| tri_cross(pts, t)).collect();
    let longest = |t: usize| -> (usize, f64) {
        (0..3).map(|k| (k, pts[tris[t][k] as usize].dist(pts[tris[t][(k + 1) % 3] as usize]))).fold((0, 0.0), |a, b| if b.1 > a.1 { b } else { a })
    };
    // a sliver: height below the plane tolerance, so its normal means nothing
    let sliver: Vec<bool> = (0..nt).map(|t| cross[t].len() <= plane_tol * longest(t).1).collect();
    rep.slivers = sliver.iter().filter(|x| **x).count();
    let cos_tol = angle_tol_deg.to_radians().cos();
    let mut region = vec![u32::MAX; nt];
    let mut order: Vec<usize> = (0..nt).filter(|&t| !sliver[t]).collect();
    order.sort_by(|&a, &b| cross[b].len().total_cmp(&cross[a].len()).then(a.cmp(&b)));
    let mut nreg = 0u32;
    let mut queue = std::collections::VecDeque::new();
    for &s in &order {
        if region[s] != u32::MAX {
            continue;
        }
        let n0 = cross[s].normalize();
        let d0 = n0.dot(pts[tris[s][0] as usize]);
        region[s] = nreg;
        queue.push_back(s);
        while let Some(t) = queue.pop_front() {
            for k in 0..3 {
                let u = eu.twin[3 * t + k] / 3;
                if region[u] != u32::MAX {
                    continue;
                }
                // a sliver's normal means nothing: its corners decide (the growth must pass through slivers)
                if !sliver[u] && (cross[u] / cross[u].len()).dot(n0) < cos_tol {
                    continue;
                }
                if tris[u].iter().all(|&i| (n0.dot(pts[i as usize]) - d0).abs() <= plane_tol) {
                    region[u] = nreg;
                    queue.push_back(u);
                }
            }
        }
        nreg += 1;
    }
    // slivers join the region across their longest edge (a needle lies along it); only when that neighbour is itself an
    // unassigned sliver for good does the next longest edge count, then the shortest
    let ranked: Vec<[usize; 3]> = (0..nt)
        .map(|t| {
            let mut ks = [0usize, 1, 2];
            let len = |k: usize| pts[tris[t][k] as usize].dist(pts[tris[t][(k + 1) % 3] as usize]);
            ks.sort_by(|&a, &b| len(b).total_cmp(&len(a)));
            ks
        })
        .collect();
    let mut rank = 0;
    loop {
        let mut changed = false;
        let mut left = false;
        for t in 0..nt {
            if region[t] != u32::MAX {
                continue;
            }
            match ranked[t][..=rank].iter().map(|&k| region[eu.twin[3 * t + k] / 3]).find(|&r| r != u32::MAX) {
                Some(r) => {
                    region[t] = r;
                    changed = true;
                }
                None => left = true,
            }
        }
        if !left {
            break;
        }
        if !changed {
            if rank == 2 {
                return Err(Refusal::new("meshDegenerate", "a component is made only of zero-area triangles", "remove the degenerate triangles"));
            }
            rank += 1;
        } else {
            rank = 0;
        }
    }
    let nreg = nreg as usize;
    // region planes: area-weighted normal, mean of the corners
    let mut nsum = vec![Vec3::ZERO; nreg];
    let mut psum = vec![Vec3::ZERO; nreg];
    let mut cnt = vec![0usize; nreg];
    let mut rcomp = vec![0u32; nreg];
    for t in 0..nt {
        let r = region[t] as usize;
        if !sliver[t] {
            nsum[r] = nsum[r] + cross[t];
        }
        for &i in &tris[t] {
            psum[r] = psum[r] + pts[i as usize];
        }
        cnt[r] += 1;
        rcomp[r] = m.comp[t];
    }
    let mut area = vec![0.0; nreg];
    for t in 0..nt {
        area[region[t] as usize] += 0.5 * cross[t].len();
    }
    let normal: Vec<Vec3> = nsum.iter().map(|n| n.try_normalize().unwrap_or(Vec3::Z)).collect();
    let point: Vec<Vec3> = (0..nreg).map(|r| psum[r] / (3 * cnt[r]) as f64).collect();
    if std::env::var("CK_MESH_DEBUG").is_ok() {
        for t in 0..nt {
            let r = region[t] as usize;
            let d = tris[t].iter().map(|&i| (pts[i as usize] - point[r]).dot(normal[r]).abs()).fold(0.0, f64::max);
            if d > 2.0 * plane_tol {
                eprintln!("tri {t} region {r} off {d:.3e} sliver {} cross {:.3e} longest {:.3e} pts {:?}", sliver[t], cross[t].len(), longest(t).1, tris[t].iter().map(|&i| pts[i as usize]).collect::<Vec<_>>());
            }
        }
    }
    Ok(Regions { region, n: nreg, normal, point, comp: rcomp, count: cnt, area, twin: eu.twin, sliver })
}

/// Boundary loops of a region map: per region, loops of half-edges (whose twin is in another region), chained by turning
/// about the end vertex inside the region.
pub(crate) fn region_loops(tris: &[[u32; 3]], twin: &[usize], region: &[u32], nreg: usize) -> Vec<Vec<Vec<usize>>> {
    let nt = tris.len();
    let boundary = |h: usize| region[twin[h] / 3] != region[h / 3];
    let next = |h: usize| -> usize {
        let mut g = 3 * (h / 3) + (h % 3 + 1) % 3;
        let mut guard = 0;
        while !boundary(g) {
            let tw = twin[g];
            g = 3 * (tw / 3) + (tw % 3 + 1) % 3;
            guard += 1;
            if guard > nt {
                break;
            }
        }
        g
    };
    let mut seen = vec![false; 3 * nt];
    let mut loops_of: Vec<Vec<Vec<usize>>> = vec![vec![]; nreg];
    for h0 in 0..3 * nt {
        if seen[h0] || !boundary(h0) {
            continue;
        }
        let mut lp = vec![];
        let mut h = h0;
        while !seen[h] {
            seen[h] = true;
            lp.push(h);
            h = next(h);
        }
        loops_of[region[h0 / 3] as usize].push(lp);
    }
    loops_of
}

/// Step 5: the regions' boundary loops as planar faces.
pub fn planar_faces(m: &Closed, rg: &Regions, plane_tol: f64) -> Vec<PlanarFace> {
    let (pts, tris) = (&m.pts, &m.tris);
    let nt = tris.len();
    let (region, nreg) = (&rg.region, rg.n);
    let loops_of: Vec<Vec<Vec<u32>>> =
        region_loops(tris, &rg.twin, region, nreg).into_iter().map(|ls| ls.into_iter().map(|l| l.into_iter().map(|h| he_from(tris, h)).collect()).collect()).collect();
    let (rcomp, cnt) = (&rg.comp, &rg.count);
    // vertices between exactly two regions lie on a straight edge: drop them (keeping every loop at three or more)
    let mut vreg: HashMap<u32, (u32, u32, u8)> = HashMap::new(); // first two regions, count (2 = more than one distinct)
    for t in 0..nt {
        let r = region[t];
        for &i in &tris[t] {
            let e = vreg.entry(i).or_insert((r, u32::MAX, 0));
            if e.0 != r && e.1 != r {
                if e.1 == u32::MAX {
                    e.1 = r;
                } else {
                    e.2 = 1; // a third region
                }
            }
        }
    }
    let mut occ: HashMap<u32, u32> = HashMap::new();
    for lps in &loops_of {
        for lp in lps {
            for &v in lp {
                *occ.entry(v).or_insert(0) += 1;
            }
        }
    }
    let mut drop: std::collections::HashSet<u32> = vreg
        .iter()
        .filter(|(v, &(_, b, third))| b != u32::MAX && third == 0 && occ.get(v) == Some(&2))
        .map(|(v, _)| *v)
        .collect();
    // collinear for real (two regions on one plane would not be)
    for lps in &loops_of {
        for lp in lps {
            let n = lp.len();
            for k in 0..n {
                let v = lp[k];
                if drop.contains(&v) {
                    let (a, c) = (pts[lp[(k + n - 1) % n] as usize], pts[lp[(k + 1) % n] as usize]);
                    let p = pts[v as usize];
                    let d = (c - a).normalize();
                    let off = (p - a).reject(d).len();
                    if !(off <= plane_tol) {
                        drop.remove(&v);
                    }
                }
            }
        }
    }
    loop {
        let mut undo = vec![];
        for lps in &loops_of {
            for lp in lps {
                let kept = lp.iter().filter(|v| !drop.contains(v)).count();
                if kept < 3 {
                    undo.extend(lp.iter().copied().filter(|v| drop.contains(v)));
                }
            }
        }
        if undo.is_empty() {
            break;
        }
        for v in undo {
            drop.remove(&v);
        }
    }
    let mut faces = Vec::with_capacity(nreg);
    for r in 0..nreg {
        let (n, point) = (rg.normal[r], rg.point[r]);
        let mut loops: Vec<(f64, Vec<u32>)> = loops_of[r]
            .iter()
            .map(|lp| {
                let kept: Vec<u32> = lp.iter().copied().filter(|v| !drop.contains(v)).collect();
                let mut a = Vec3::ZERO;
                for k in 0..kept.len() {
                    a = a + pts[kept[k] as usize].cross(pts[kept[(k + 1) % kept.len()] as usize]);
                }
                (a.dot(n), kept)
            })
            .collect();
        loops.sort_by(|a, b| b.0.total_cmp(&a.0));
        faces.push(PlanarFace { normal: n, point, comp: rcomp[r], loops: loops.into_iter().map(|x| x.1).collect(), triangles: cnt[r] });
    }
    faces
}

/// Step 5b: the body of the planar faces. One lump per outer component, its voids as further shells.
pub fn build_body(m: &Closed, faces: &[PlanarFace]) -> Body {
    let mut b = Body::new();
    let ncomp = m.comps.len();
    let mut shell_of: Vec<Option<ShellId>> = vec![None; ncomp];
    let mut lump_of = vec![None; ncomp];
    for c in 0..ncomp {
        if m.comps[c].depth % 2 == 0 {
            let l = b.add_lump();
            lump_of[c] = Some(l);
            shell_of[c] = Some(b.add_shell(l));
        }
    }
    for c in 0..ncomp {
        if m.comps[c].depth % 2 == 1 {
            let parent = m.comps[c].parent.and_then(|p| lump_of[p]);
            let l = parent.unwrap_or_else(|| b.add_lump());
            shell_of[c] = Some(b.add_shell(l));
        }
    }
    let mut verts: HashMap<(u32, u32), VertexId> = HashMap::new();
    let mut edges: HashMap<(u32, u32), EdgeId> = HashMap::new();
    for (i, f) in faces.iter().enumerate() {
        let shell = shell_of[f.comp as usize].unwrap();
        let face = b.add_face(shell, Surface::plane(f.point, f.normal), true, FaceName::other("mesh", &format!("F{i}")));
        for ring in &f.loops {
            let lp = b.add_loop(face);
            for k in 0..ring.len() {
                let (p, q) = (ring[k], ring[(k + 1) % ring.len()]);
                let (lo, hi, rev) = if p < q { (p, q, false) } else { (q, p, true) };
                let e = match edges.get(&(lo, hi)) {
                    Some(&e) => e,
                    None => {
                        let mut vid = |i: u32, b: &mut Body| *verts.entry((f.comp, i)).or_insert_with(|| b.add_vertex(m.pts[i as usize]));
                        let (va, vb) = (vid(lo, &mut b), vid(hi, &mut b));
                        let (pa, pb) = (m.pts[lo as usize], m.pts[hi as usize]);
                        let e = b.add_edge(Curve::line_through(pa, pb), 0.0, pa.dist(pb), va, vb);
                        edges.insert((lo, hi), e);
                        e
                    }
                };
                b.add_coedge(lp, e, rev, None);
            }
        }
    }
    b
}

/// The whole conversion. See the module comment.
pub fn mesh_to_solid(positions: &[f64], indices: Option<&[u32]>, o: &Opts) -> Result<Converted, Refusal> {
    let mut rep = Report { angle_tol_deg: o.angle_tol_deg, ..Default::default() };
    let m = close_mesh(positions, indices, o, &mut rep)?;
    let mut mark = ck_core::budget::mark();
    // a decimated scan has no panels that are flat within 1e-5 of the diagonal, only triangles that happen to be nearly so
    let plane_tol = o.plane_tol.filter(|v| *v > 0.0).unwrap_or(if o.decimate.is_some() { 1e-7 } else { 1e-5 } * rep.diagonal);
    rep.plane_tol = plane_tol;
    if let Some(r) = over_budget("merging coplanar triangles") {
        return Err(r);
    }
    let rg = planar_regions(&m, plane_tol, o.angle_tol_deg, &mut rep)?;
    rep.lap("regions", &mut mark);
    let faces = planar_faces(&m, &rg, plane_tol);
    rep.lap("faces", &mut mark);
    if let Some(r) = over_budget("building the planar faces") {
        return Err(r);
    }
    rep.planar_faces = faces.len();
    let too_large = |n: usize, what: &str| {
        Refusal::new(
            "meshTooLarge",
            format!("{n} faces {what} exceed the limit of {} (a free-form or scanned surface keeps one face per triangle)", o.max_faces),
            "set opts.decimate {tol, targetTriangles?} (quadric decimation within a deviation tolerance, or send the mesh through meshDecimate first), or raise opts.maxFaces",
        )
        .with("triangles", m.tris.len() as f64)
        .with("faces", n as f64)
        .with("limit", o.max_faces as f64)
    };
    let vtol = 1e-6;
    // the analytic refit first (it validates what it returns); the faceted body is the fallback
    let mut body = None;
    if o.analytic {
        let (ab, ar) = analytic::refit(&m, &rg, &o.fit, rep.diagonal, plane_tol, o.angle_tol_deg);
        let mut ar = ar;
        ar.fitted_fraction = ar.fitted.iter().map(|f| f.triangles).sum::<usize>() as f64 / m.tris.len().max(1) as f64;
        rep.analytic = Some(ar);
        rep.lap("refit", &mut mark);
        if let Some(r) = over_budget("the analytic refit") {
            return Err(r);
        }
        if let Some(ab) = ab {
            if ab.num_faces() > o.max_faces {
                return Err(too_large(ab.num_faces(), "after the analytic refit"));
            }
            body = Some(ab);
        }
    }
    // nothing was refit: the body is the mesh's own triangles, merged where coplanar
    let faceted_only = body.is_none() || rep.analytic.as_ref().is_some_and(|a| !a.applied && a.fitted.is_empty());
    let (body, issues) = match body {
        Some(b) => (b, 0),
        None => {
            if faces.len() > o.max_faces {
                return Err(too_large(faces.len(), "after merging coplanar triangles"));
            }
            let mut b = build_body(&m, &faces);
            // a body of thousands of planar faces straight from a closed, oriented mesh: the topology and bounding checks
            // only (the loop-crossing and volume checks cost seconds on a scan and the mesh already passed the closed test)
            let quick = b.num_faces() > QUICK_VALIDATION_FACES;
            rep.quick_validation = quick;
            let mut issues = ck_check::validate_ctl(&b, vtol, ck_check::DEFAULT_TOL_CAP, quick, &ck_check::Ctl::unlimited());
            if !issues.is_empty() {
                let hr = ck_heal::heal(&mut b, plane_tol.max(vtol));
                rep.heal_fixes = hr.fixed.len();
                issues = ck_check::validate(&b, vtol);
            }
            (b, issues.len())
        }
    };
    rep.issues = issues;
    rep.lap("validate", &mut mark);
    if let Some(r) = over_budget("validation") {
        return Err(r);
    }
    // the volume of a large body that is the mesh's own triangles is the mesh's (integrating 50000 faces costs seconds)
    rep.body_volume = if faceted_only && body.num_faces() > QUICK_VALIDATION_FACES { rep.mesh_volume } else { ck_topo::mass::mass_props(&body).volume };
    if o.deviation {
        // a large body built from the mesh's own triangles (nothing refit) deviates by nothing; a huge refit body is not sampled
        rep.deviation = Some(if faceted_only && body.num_faces() > QUICK_VALIDATION_FACES {
            deviation::Deviation { note: Some("exact: nothing was refit, the faces are the mesh's own"), ..Default::default() }
        } else if body.num_faces() > deviation::MAX_FACES {
            deviation::Deviation { note: Some("not measured: too many faces to tessellate and sample in time"), ..Default::default() }
        } else {
            deviation::measure(&body, &m)
        });
    }
    rep.lap("deviation", &mut mark);
    if let Some(r) = over_budget("measuring the deviation") {
        return Err(r);
    }
    Ok(Converted { body, report: rep })
}

/// The conversion up to and including the closing stage and the watertight check, nothing built: the report (with, per
/// hole, the triangles `opts.close` would add: [`close::HoleInfo::fill`]) or the refusal the conversion would give.
pub fn preview(positions: &[f64], indices: Option<&[u32]>, o: &Opts) -> Result<Report, Refusal> {
    let mut o = o.clone();
    o.dry_run = true;
    let mut rep = Report { angle_tol_deg: o.angle_tol_deg, ..Default::default() };
    close_mesh(positions, indices, &o, &mut rep)?;
    Ok(rep)
}

/// The refusal that stops the conversion between two stages when the regen's time budget (`ck_core::budget`) has run
/// out. The conversion is a function of its input only, so a stopped run leaves nothing behind. `meshBudgetExceeded`
/// is rewritten to the budget's own error by the regen driver; on the wire request (no clock) it never occurs.
fn over_budget(stage: &str) -> Option<Refusal> {
    if ck_core::budget::expired() {
        Some(Refusal::new("meshBudgetExceeded", format!("the time limit ran out before {stage}"), "raise the time budget, or decimate the mesh"))
    } else {
        None
    }
}
