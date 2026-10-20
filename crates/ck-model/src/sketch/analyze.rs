//! Read-only sketch analysis: `sketchAnalyze` (structural findings, chains
//! and the profile reason, so an agent gets exact, structured feedback
//! instead of a bare `noClosedProfile`) and `sketchFreedoms` (what the
//! solver can still move: per point, per radius, per entity).
//!
//! Additive: the oracle kernel carries no sketch-analysis query or finding
//! names (searched: `strings` of `kernel.wasm` and the wasm2c dump
//! `c2/decomp/all.c`, which holds no C string literals — `openEnd`,
//! `nearCoincident`, `zeroLength`, `tinySegment`, `isolatedPoint`,
//! `selfIntersection` do not occur near sketch code; the original client
//! derives "fully defined" itself by pin-and-solve probes). The finding
//! names below are ours.
//!
//! Nothing here builds regions or touches `profile`; the analysis reads the
//! sketch document only. `freedoms` shares `status::sketch_status`'s build,
//! convergence and rank (same `dof`), and adds the null-space completion.
//!

use super::solver::{build, jacobian};
use super::status::{converge, row_basis, Converge};
use super::{Sketch, SketchEntity};
use ck_json::{Json, Obj};

/// Default near-coincident tolerance (mm): distinct points closer than this
/// are flagged unless a coincident constraint joins them. Wire: `tolerance`,
/// `<= 0` disables the check.
pub const NEAR_TOL: f64 = 0.01;
/// Default tiny-segment length (mm): a curve shorter than this and not zero
/// is `tinySegment`. Wire: `tiny`, `<= 0` disables the check.
pub const TINY: f64 = 1e-3;

/// One finding: a kind, the entities and points it names, a location in
/// sketch coordinates and kind-specific numbers (`gap`, `distance`,
/// `length`, `degree`).
#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    pub kind: &'static str,
    pub entities: Vec<String>,
    pub points: Vec<String>,
    pub at: Option<[f64; 2]>,
    pub gap: Option<f64>,
    pub nearest: Option<String>,
    pub distance: Option<f64>,
    pub length: Option<f64>,
    pub degree: Option<usize>,
}

impl Finding {
    fn new(kind: &'static str) -> Self {
        Finding {
            kind,
            entities: vec![],
            points: vec![],
            at: None,
            gap: None,
            nearest: None,
            distance: None,
            length: None,
            degree: None,
        }
    }
    fn at(mut self, p: [f64; 2]) -> Self {
        self.at = Some(p);
        self
    }
    fn ent(mut self, ids: &[&str]) -> Self {
        self.entities = ids.iter().map(|s| s.to_string()).collect();
        self
    }
    fn pts(mut self, ids: &[&str]) -> Self {
        self.points = ids.iter().map(|s| s.to_string()).collect();
        self
    }

    pub fn to_json(&self) -> Json {
        let mut o = Obj::new().put("kind", self.kind);
        if !self.entities.is_empty() {
            o = o.put("entities", Json::Arr(self.entities.iter().map(|s| Json::Str(s.clone())).collect()));
        }
        if !self.points.is_empty() {
            o = o.put("points", Json::Arr(self.points.iter().map(|s| Json::Str(s.clone())).collect()));
        }
        if let Some([x, y]) = self.at {
            o = o.put("at", Json::Arr(vec![Json::Num(x), Json::Num(y)]));
        }
        if let Some(g) = self.gap {
            o = o.put("gap", Json::Num(g));
        }
        if let Some(d) = self.distance {
            o = o.put("distance", Json::Num(d));
        }
        if let Some(l) = self.length {
            o = o.put("length", Json::Num(l));
        }
        if let Some(n) = &self.nearest {
            o = o.put("nearest", Json::Str(n.clone()));
        }
        if let Some(d) = self.degree {
            o = o.put("degree", Json::Num(d as f64));
        }
        o.build()
    }
}

/// A connected chain of entities (connected = sharing endpoint point ids;
/// circles, ellipses, closed splines and lettering are chains of their own).
#[derive(Debug, Clone, PartialEq)]
pub struct Chain {
    pub entities: Vec<String>,
    /// No endpoint dangles (every endpoint point carries at least two ends).
    pub closed: bool,
    /// True when every entity of the chain is construction geometry.
    pub construction: bool,
}

/// The full analysis of one sketch.
#[derive(Debug, Clone, PartialEq)]
pub struct Analysis {
    /// The near-coincident tolerance applied (echoed on the wire).
    pub tolerance: f64,
    /// The tiny-segment length applied (echoed on the wire).
    pub tiny: f64,
    pub findings: Vec<Finding>,
    pub chains: Vec<Chain>,
    /// Closed, non-construction chains — what a profile can be built from.
    pub closed_chains: usize,
    /// Why there is no profile when `closed_chains` is 0: the biggest
    /// `openEnd` gap, `empty` (no curves), or `constructionOnly`.
    pub reason: Option<Finding>,
}

impl Analysis {
    pub fn to_json(&self) -> Json {
        let mut profile = Obj::new().put("closedChains", Json::Num(self.closed_chains as f64));
        if let Some(r) = &self.reason {
            profile = profile.put("reason", r.to_json());
        }
        Obj::new()
            .put("tolerance", self.tolerance)
            .put("tiny", self.tiny)
            .put("findings", Json::Arr(self.findings.iter().map(Finding::to_json).collect()))
            .put(
                "chains",
                Json::Arr(
                    self.chains
                        .iter()
                        .map(|c| {
                            Obj::new()
                                .put("entities", Json::Arr(c.entities.iter().map(|s| Json::Str(s.clone())).collect()))
                                .put("closed", Json::Bool(c.closed))
                                .put("construction", Json::Bool(c.construction))
                                .build()
                        })
                        .collect(),
                ),
            )
            .put("profile", profile.build())
            .build()
    }
}


/// One point's freedom: how many independent directions remain for it in
/// the null space of the Jacobian (0 = fixed or fully determined, 1 =
/// exactly `dir`, 2 = any direction in the sketch plane). A point with no
/// unknowns (fixed) and a point the constraints pin are both `free: 0`.
#[derive(Debug, Clone, PartialEq)]
pub struct PointFreedom {
    pub id: String,
    pub free: usize,
    /// Only for `free == 1`: a unit direction, sign-normalised (the larger
    /// component positive, ties to x).
    pub dir: Option<[f64; 2]>,
}

/// One circle/ellipse/arc radius: free when some null-space motion still
/// changes it (arcs have no radius unknown — derived from their endpoints).
#[derive(Debug, Clone, PartialEq)]
pub struct RadiusFreedom {
    pub id: String,
    pub free: bool,
}

/// Per-entity summary: the freedoms of its referenced points plus one when
/// its own radius is free (a free circle counts centre 2 + radius 1 = 3).
/// Shared points count in every entity that references them.
#[derive(Debug, Clone, PartialEq)]
pub struct EntityFreedom {
    pub id: String,
    pub kind: &'static str,
    pub free: usize,
}

/// The `sketchFreedoms` reply: solver degrees of freedom and what they mean
/// per point, radius and entity.
#[derive(Debug, Clone, PartialEq)]
pub struct Freedoms {
    /// Unknowns - rank, exactly `status::sketch_status().dof`.
    pub dof: usize,
    /// The drawn coordinates satisfy every constraint, or converge.
    pub solved: bool,
    pub points: Vec<PointFreedom>,
    pub radii: Vec<RadiusFreedom>,
    pub entities: Vec<EntityFreedom>,
}

impl Freedoms {
    pub fn to_json(&self) -> Json {
        let arr = |v: Vec<Json>| Json::Arr(v);
        Obj::new()
            .put("dof", self.dof as f64)
            .put("solved", self.solved)
            .put(
                "points",
                arr(
                    self.points
                        .iter()
                        .map(|p| {
                            let mut o = Obj::new().put("id", p.id.as_str()).put("free", p.free as f64);
                            if let Some(d) = p.dir {
                                o = o.put("dir", Json::Arr(vec![Json::Num(d[0]), Json::Num(d[1])]));
                            }
                            o.build()
                        })
                        .collect(),
                ),
            )
            .put(
                "radii",
                arr(self.radii.iter().map(|r| Obj::new().put("id", r.id.as_str()).put("free", r.free).build()).collect()),
            )
            .put(
                "entities",
                arr(
                    self.entities
                        .iter()
                        .map(|e| Obj::new().put("id", e.id.as_str()).put("kind", e.kind).put("free", e.free as f64).build())
                        .collect(),
                ),
            )
            .build()
    }
}

/// Sign-normalise a direction: unit length, the larger component positive,
/// x wins a tie, `-0.0` folded to `0.0`.
fn sign_norm(v: [f64; 2]) -> [f64; 2] {
    let l = v[0].hypot(v[1]);
    let [a, b] = [v[0] / l, v[1] / l];
    let flip = if a.abs() >= b.abs() { a < 0.0 } else { b < 0.0 };
    let [a, b] = if flip { [-a, -b] } else { [a, b] };
    [a + 0.0, b + 0.0]
}

fn entity_freedoms(sk: &Sketch, points: &[PointFreedom], radii: &[RadiusFreedom]) -> Vec<EntityFreedom> {
    sk.entities
        .iter()
        .map(|e| {
            let mut free: usize = e.point_ids().into_iter().map(|pid| points.iter().find(|p| p.id == pid).map_or(0, |p| p.free)).sum();
            if matches!(e, SketchEntity::Circle { .. } | SketchEntity::Ellipse { .. } | SketchEntity::Arc { .. }) {
                free += radii.iter().find(|r| r.id == e.id()).map_or(0, |r| usize::from(r.free));
            }
            EntityFreedom { id: e.id().to_string(), kind: e.type_name(), free }
        })
        .collect()
}

/// Solver freedoms (`sketchFreedoms`): build + converge exactly like
/// `status::sketch_status` (so `dof` agrees with it), then complete the row
/// basis to a basis of the Jacobian's null space and project it onto each
/// point's `x`/`y` unknowns (rank 0/1/2 = `free`, the sole direction when
/// 1) and onto each radius (circles/ellipses via their `r` unknown, arcs
/// via `dr = u·(motion of p1 - motion of c)` at the solved coordinates).
/// If the system cannot be built every non-fixed point counts 2 and no
/// radius is free — `sketch_status`'s fallback.
pub fn freedoms(sk: &Sketch) -> Freedoms {
    let Ok(sys) = build(sk) else {
        let points = sk
            .points
            .iter()
            .map(|p| PointFreedom { id: p.id.clone(), free: if p.fixed { 0 } else { 2 }, dir: None })
            .collect::<Vec<_>>();
        let radii = sk
            .entities
            .iter()
            .filter(|e| matches!(e, SketchEntity::Circle { .. } | SketchEntity::Ellipse { .. } | SketchEntity::Arc { .. }))
            .map(|e| RadiusFreedom { id: e.id().to_string(), free: false })
            .collect::<Vec<_>>();
        let dof = sk.points.iter().filter(|p| !p.fixed).count() * 2;
        let entities = entity_freedoms(sk, &points, &radii);
        return Freedoms { dof, solved: false, points, radii, entities };
    };
    let n = sys.x.len();
    let Converge { mut x, solved, .. } = converge(&sys);
    let jac = jacobian(&sys, &mut x);
    let (basis, _) = row_basis(&jac, n);
    let dof = n - basis.q.len();
    // Null space: each unit vector projected off the row space, then
    // Gram-Schmidt — the projected units span exactly the null space.
    let mut nulls: Vec<Vec<f64>> = Vec::new();
    for i in 0..n {
        let mut z = vec![0.0; n];
        z[i] = 1.0;
        for q in &basis.q {
            let d = q[i];
            for (a, b) in z.iter_mut().zip(q) {
                *a -= d * b;
            }
        }
        for g in &nulls {
            let d: f64 = z.iter().zip(g).map(|(a, b)| a * b).sum();
            for (a, b) in z.iter_mut().zip(g) {
                *a -= d * b;
            }
        }
        let l = z.iter().map(|a| a * a).sum::<f64>().sqrt();
        if l > 1e-9 {
            for a in z.iter_mut() {
                *a /= l;
            }
            nulls.push(z);
        }
    }
    // every point's projected motion in its own x/y plane
    let mut points = Vec::new();
    for p in &sk.points {
        let (Some(cx), Some(cy)) = (
            sys.key_idx.get(&format!("{}.x", p.id)).copied(),
            sys.key_idx.get(&format!("{}.y", p.id)).copied(),
        ) else {
            points.push(PointFreedom { id: p.id.clone(), free: 0, dir: None });
            continue;
        };
        let mut base: Option<[f64; 2]> = None;
        let mut two = false;
        for z in &nulls {
            let v = [z[cx], z[cy]];
            if v[0].abs() <= 1e-9 && v[1].abs() <= 1e-9 {
                continue;
            }
            match base {
                None => base = Some(v),
                Some(b) if !two => {
                    let cr = (b[0] * v[1] - b[1] * v[0]).abs();
                    let scale = (b[0] * b[0] + b[1] * b[1]).sqrt() * (v[0] * v[0] + v[1] * v[1]).sqrt();
                    if cr > 1e-9 * scale {
                        two = true;
                    }
                }
                _ => {}
            }
        }
        let free = if two { 2 } else { base.is_some() as usize };
        let dir = if free == 1 { base.map(sign_norm) } else { None };
        points.push(PointFreedom { id: p.id.clone(), free, dir });
    }
    // radii
    let pos_of = |pid: &str| -> [f64; 2] {
        match (sys.key_idx.get(&format!("{pid}.x")), sys.key_idx.get(&format!("{pid}.y"))) {
            (Some(&i), Some(&j)) => [x[i], x[j]],
            _ => sk.point_xy(pid).unwrap_or([0.0, 0.0]),
        }
    };
    let mut radii = Vec::new();
    for e in &sk.entities {
        let (id, free) = match e {
            SketchEntity::Circle { id, .. } | SketchEntity::Ellipse { id, .. } => {
                let free = sys.key_idx.get(&format!("{id}.r")).is_some_and(|&k| nulls.iter().any(|z| z[k].abs() > 1e-9));
                (id.clone(), free)
            }
            SketchEntity::Arc { id, c, p1, .. } => {
                let (cp, a) = (pos_of(c), pos_of(p1));
                let r = (a[0] - cp[0]).hypot(a[1] - cp[1]);
                let u = if r > 1e-12 { [(a[0] - cp[0]) / r, (a[1] - cp[1]) / r] } else { [0.0, 0.0] };
                let free = nulls.iter().any(|z| {
                    let d = |pid: &str| -> [f64; 2] {
                        match (sys.key_idx.get(&format!("{pid}.x")), sys.key_idx.get(&format!("{pid}.y"))) {
                            (Some(&i), Some(&j)) => [z[i], z[j]],
                            _ => [0.0, 0.0],
                        }
                    };
                    let (ma, mc) = (d(p1), d(c));
                    (u[0] * (ma[0] - mc[0]) + u[1] * (ma[1] - mc[1])).abs() > 1e-9
                });
                (id.clone(), free)
            }
            _ => continue,
        };
        radii.push(RadiusFreedom { id, free });
    }
    let entities = entity_freedoms(sk, &points, &radii);
    Freedoms { dof, solved, points, radii, entities }
}

fn pos(sk: &Sketch, id: &str) -> Option<[f64; 2]> {
    sk.point_xy(id)
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The two endpoints of an entity: the ids it chains through. None for
/// points, lettering, circles, ellipses and closed splines (self-closed).
fn endpoints(e: &SketchEntity) -> Option<(&str, &str)> {
    match e {
        SketchEntity::Line { p1, p2, .. } | SketchEntity::Arc { p1, p2, .. } => Some((p1, p2)),
        SketchEntity::Spline { pts, closed, .. } | SketchEntity::CvSpline { pts, closed, .. } => {
            if *closed || pts.len() < 2 {
                None
            } else {
                Some((pts[0].as_str(), pts[pts.len() - 1].as_str()))
            }
        }
        _ => None,
    }
}

/// Entities a profile can chain through: everything but bare points.
fn chainable(e: &SketchEntity) -> bool {
    !matches!(e, SketchEntity::Point { .. })
}

/// Entities the geometric findings apply to: chains without lettering
/// (lettering outlines are analysed by the text queries).
fn analyzable(e: &SketchEntity) -> bool {
    chainable(e) && !matches!(e, SketchEntity::Text { .. })
}

fn coincident_pairs(sk: &Sketch) -> Vec<(String, String)> {
    sk.constraints
        .iter()
        .filter(|c| c.kind == "coincident" || c.kind == "concentric")
        .filter(|c| !c.suppressed)
        .filter_map(|c| Some((c.refs.first()?.clone(), c.refs.get(1)?.clone())))
        .collect()
}

fn is_coincident(pairs: &[(String, String)], a: &str, b: &str) -> bool {
    pairs.iter().any(|(x, y)| (x == a && y == b) || (x == b && y == a))
}

/// Degree per point id: how many entity ends touch that point.
fn endpoint_degrees(ents: &[&SketchEntity]) -> Vec<(String, usize)> {
    let mut degree: Vec<(String, usize)> = Vec::new();
    for e in ents {
        if let Some((a, b)) = endpoints(e) {
            for p in [a, b] {
                match degree.iter_mut().find(|(id, _)| id.as_str() == p) {
                    Some((_, d)) => *d += 1,
                    None => degree.push((p.to_string(), 1)),
                }
            }
        }
    }
    degree
}

/// Connected components over shared endpoint point ids. Returns one entity-id
/// list per chain, in first-entity order.
fn chain_components(ents: &[&SketchEntity]) -> Vec<Vec<String>> {
    let mut ecomp: Vec<Option<usize>> = vec![None; ents.len()];
    let mut point_comp: Vec<(String, usize)> = Vec::new();
    let mut next = 0usize;
    let comp_of = |pc: &[(String, usize)], p: &str| pc.iter().find(|(id, _)| id.as_str() == p).map(|(_, c)| *c);
    for (i, e) in ents.iter().enumerate() {
        let Some((a, b)) = endpoints(e) else {
            ecomp[i] = Some(next);
            next += 1;
            continue;
        };
        let (ca, cb) = (comp_of(&point_comp, a), comp_of(&point_comp, b));
        let c = match (ca, cb) {
            (Some(x), Some(y)) if x == y => x,
            (Some(x), Some(y)) => {
                for (_, c) in &mut point_comp {
                    if *c == y {
                        *c = x;
                    }
                }
                for c in ecomp.iter_mut().flatten() {
                    if *c == y {
                        *c = x;
                    }
                }
                x
            }
            (Some(x), None) => {
                point_comp.push((b.to_string(), x));
                x
            }
            (None, Some(y)) => {
                point_comp.push((a.to_string(), y));
                y
            }
            (None, None) => {
                point_comp.push((a.to_string(), next));
                point_comp.push((b.to_string(), next));
                let c = next;
                next += 1;
                c
            }
        };
        ecomp[i] = Some(c);
    }
    let mut groups: Vec<Vec<String>> = Vec::new();
    for (i, e) in ents.iter().enumerate() {
        let c = ecomp[i].unwrap();
        if groups.len() <= c {
            groups.resize(c + 1, Vec::new());
        }
        groups[c].push(e.id().to_string());
    }
    groups.retain(|g| !g.is_empty());
    groups
}

/// Analyse `sketch` with the given near-coincident tolerance and
/// tiny-segment length (mm).
pub fn analyze(sk: &Sketch, tolerance: f64, tiny: f64) -> Analysis {
    let curves: Vec<&SketchEntity> = sk.entities.iter().filter(|e| analyzable(e)).collect();
    let chained: Vec<&SketchEntity> = sk.entities.iter().filter(|e| chainable(e)).collect();
    let mut findings: Vec<Finding> = Vec::new();

    // zeroLength / tinySegment
    for e in &curves {
        let len: Option<f64> = match **e {
            SketchEntity::Line { ref p1, ref p2, .. } => match (pos(sk, p1), pos(sk, p2)) {
                (Some(a), Some(b)) => Some(dist(a, b)),
                _ => None,
            },
            SketchEntity::Circle { r, .. } | SketchEntity::Ellipse { r, .. } => Some(r),
            SketchEntity::Arc { ref c, ref p1, ref p2, .. } => match (pos(sk, c), pos(sk, p1), pos(sk, p2)) {
                (Some(cc), Some(a), Some(b)) => {
                    // sweep in (0, 2pi] (client tn()): p1 == p2 is a full turn
                    let a1 = (a[1] - cc[1]).atan2(a[0] - cc[0]);
                    let mut sweep = (b[1] - cc[1]).atan2(b[0] - cc[0]) - a1;
                    while sweep <= 1e-12 {
                        sweep += std::f64::consts::TAU;
                    }
                    while sweep > std::f64::consts::TAU + 1e-12 {
                        sweep -= std::f64::consts::TAU;
                    }
                    Some((a[0] - cc[0]).hypot(a[1] - cc[1]) * sweep)
                }
                _ => None,
            },
            _ => None,
        };
        let Some(l) = len else { continue };
        let id = e.id();
        let at = midpoint_of(sk, e);
        if l <= 0.0 {
            findings.push(Finding::new("zeroLength").ent(&[id]).at_opt(at));
        } else if l < tiny {
            findings.push(Finding::new("tinySegment").ent(&[id]).at_opt(at).length(l));
        }
    }

    // duplicate: same kind, same referenced points, same radius
    for i in 0..curves.len() {
        for j in i + 1..curves.len() {
            let (a, b) = (curves[i], curves[j]);
            let same = match (a, b) {
                (SketchEntity::Line { p1, p2, .. }, SketchEntity::Line { p1: q1, p2: q2, .. }) => {
                    (p1 == q1 && p2 == q2) || (p1 == q2 && p2 == q1)
                }
                (SketchEntity::Circle { c, r, .. }, SketchEntity::Circle { c: d, r: s, .. }) => c == d && (r - s).abs() <= 1e-9,
                (SketchEntity::Arc { c, p1, p2, .. }, SketchEntity::Arc { c: d, p1: q1, p2: q2, .. }) => {
                    c == d && ((p1 == q1 && p2 == q2) || (p1 == q2 && p2 == q1))
                }
                _ => false,
            };
            if same {
                findings.push(Finding::new("duplicate").ent(&[a.id(), b.id()]));
            }
        }
    }

    // overlap: collinear lines with overlapping interiors
    for i in 0..curves.len() {
        for j in i + 1..curves.len() {
            let (SketchEntity::Line { p1, p2, .. }, SketchEntity::Line { p1: q1, p2: q2, .. }) = (curves[i], curves[j]) else {
                continue;
            };
            let (Some(a), Some(b), Some(c), Some(d)) = (pos(sk, p1), pos(sk, p2), pos(sk, q1), pos(sk, q2)) else {
                continue;
            };
            let u = [b[0] - a[0], b[1] - a[1]];
            let lu = u[0].hypot(u[1]);
            if lu <= 1e-12 {
                continue;
            }
            let n = [-u[1] / lu, u[0] / lu];
            let side = |p: [f64; 2]| ((p[0] - a[0]) * n[0] + (p[1] - a[1]) * n[1]).abs();
            if side(c) > tolerance || side(d) > tolerance {
                continue;
            }
            let par = |p: [f64; 2]| ((p[0] - a[0]) * u[0] + (p[1] - a[1]) * u[1]) / (lu * lu);
            let (c0, c1) = (par(c), par(d));
            let (lo, hi) = (c0.min(c1), c1.max(c0));
            if lo.max(0.0) < hi.min(1.0) - 1e-9 {
                findings.push(Finding::new("overlap").ent(&[curves[i].id(), curves[j].id()]));
            }
        }
    }

    let degree = endpoint_degrees(&curves);
    let deg = |p: &str| degree.iter().find(|(id, _)| id.as_str() == p).map_or(0, |(_, d)| *d);

    // openEnd: a dangling end, with the gap to the nearest other dangling end
    let pairs = coincident_pairs(sk);
    let mut ends: Vec<(String, String, Option<[f64; 2]>)> = Vec::new();
    for e in &curves {
        if let Some((a, b)) = endpoints(e) {
            for p in [a, b] {
                if deg(p) == 1 {
                    ends.push((e.id().to_string(), p.to_string(), pos(sk, p)));
                }
            }
        }
    }
    for (eid, pid, at) in &ends {
        let Some(at) = at else {
            findings.push(Finding::new("openEnd").ent(&[eid]).pts(&[pid.as_str()]));
            continue;
        };
        let mut best: Option<(f64, &str, &str)> = None;
        for (oid, opid, oat) in &ends {
            if oid == eid && opid == pid {
                continue;
            }
            let Some(oat) = oat else { continue };
            let d = dist(*at, *oat);
            if best.map_or(true, |(b, _, _)| d < b) {
                best = Some((d, oid, opid));
            }
        }
        let mut f = Finding::new("openEnd").ent(&[eid]).pts(&[pid.as_str()]).at(*at);
        if let Some((d, oid, opid)) = best {
            f.gap = Some(d);
            f.nearest = Some(oid.to_string());
            f.points.push(opid.to_string());
        }
        findings.push(f);
    }

    // nearCoincident: distinct points closer than `tolerance`, not tied by a
    // coincident/concentric constraint
    let mut all: Vec<(String, [f64; 2])> = sk.points.iter().map(|p| (p.id.clone(), [p.x, p.y])).collect();
    all.sort_by(|a, b| a.0.cmp(&b.0));
    for i in 0..all.len() {
        for j in i + 1..all.len() {
            let (ida, pa) = &all[i];
            let (idb, pb) = &all[j];
            let d = dist(*pa, *pb);
            if ida != idb && d < tolerance && !is_coincident(&pairs, ida, idb) {
                findings.push(
                    Finding::new("nearCoincident")
                        .pts(&[ida.as_str(), idb.as_str()])
                        .at([(pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5])
                        .distance(d),
                );
            }
        }
    }

    // branch: more than two entity ends at one point
    for (p, d) in &degree {
        if *d > 2 {
            let ents: Vec<String> = curves
                .iter()
                .filter(|e| endpoints(e).map_or(false, |(a, b)| a == p.as_str() || b == p.as_str()))
                .map(|e| e.id().to_string())
                .collect();
            let mut f = Finding::new("branch").pts(&[p.as_str()]).degree(*d);
            f.entities = ents;
            if let Some(q) = sk.point(p) {
                f = f.at([q.x, q.y]);
            }
            findings.push(f);
        }
    }

    // isolatedPoint: referenced by no entity at all
    for p in &sk.points {
        if !sk.entities.iter().any(|e| e.point_ids().iter().any(|r| *r == p.id)) {
            findings.push(Finding::new("isolatedPoint").pts(&[p.id.as_str()]).at([p.x, p.y]));
        }
    }

    findings.extend(self_intersections(sk, &curves, tolerance));

    // chains and the profile reason
    let mut chains: Vec<Chain> = Vec::new();
    for g in chain_components(&chained) {
        let closed = g.iter().all(|id| {
            sk.entity(id).and_then(endpoints).map_or(true, |(a, b)| deg(a) >= 2 && deg(b) >= 2)
        });
        let construction = g.iter().all(|id| sk.entity(id).map_or(false, |e| e.construction()));
        chains.push(Chain { entities: g, closed, construction });
    }
    let closed_chains = chains.iter().filter(|c| c.closed && !c.construction).count();
    let reason = if closed_chains > 0 {
        None
    } else if curves.is_empty() {
        Some(Finding::new("empty"))
    } else {
        findings
            .iter()
            .filter(|f| f.kind == "openEnd")
            .max_by(|a, b| a.gap.unwrap_or(0.0).total_cmp(&b.gap.unwrap_or(0.0)))
            .cloned()
            .or_else(|| Some(Finding::new("constructionOnly")))
    };
    Analysis { tolerance, tiny, findings, chains, closed_chains, reason }
}

impl Finding {
    fn at_opt(self, p: Option<[f64; 2]>) -> Self {
        match p {
            Some(p) => self.at(p),
            None => self,
        }
    }
    fn length(mut self, l: f64) -> Self {
        self.length = Some(l);
        self
    }
    fn distance(mut self, d: f64) -> Self {
        self.distance = Some(d);
        self
    }
    fn degree(mut self, d: usize) -> Self {
        self.degree = Some(d);
        self
    }
}

/// Representative location of an entity (midpoint of a line, centre of a
/// circle/arc, mean of spline points).
fn midpoint_of(sk: &Sketch, e: &SketchEntity) -> Option<[f64; 2]> {
    let pts: Vec<[f64; 2]> = match e {
        SketchEntity::Line { p1, p2, .. } | SketchEntity::Arc { p1, p2, .. } => {
            vec![pos(sk, p1)?, pos(sk, p2)?]
        }
        SketchEntity::Circle { c, .. } | SketchEntity::Ellipse { c, .. } => return pos(sk, c),
        SketchEntity::Spline { pts, .. } | SketchEntity::CvSpline { pts, .. } => {
            pts.iter().filter_map(|p| pos(sk, p)).collect()
        }
        _ => return None,
    };
    if pts.is_empty() {
        return None;
    }
    let n = pts.len() as f64;
    Some([pts.iter().map(|p| p[0]).sum::<f64>() / n, pts.iter().map(|p| p[1]).sum::<f64>() / n])
}

/// A polyline of an entity for crossing detection (control polygon for
/// splines, uniform samples for lines/arcs/circles/ellipses).
fn sample_curve(sk: &Sketch, e: &SketchEntity, n: usize) -> Vec<[f64; 2]> {
    match e {
        SketchEntity::Line { p1, p2, .. } => match (pos(sk, p1), pos(sk, p2)) {
            (Some(a), Some(b)) => vec![a, b],
            _ => vec![],
        },
        SketchEntity::Circle { c, r, .. } => match pos(sk, c) {
            Some(cc) => (0..=n)
                .map(|i| {
                    let t = std::f64::consts::TAU * i as f64 / n as f64;
                    [cc[0] + r * t.cos(), cc[1] + r * t.sin()]
                })
                .collect(),
            None => vec![],
        },
        SketchEntity::Arc { c, p1, p2, .. } => match (pos(sk, c), pos(sk, p1), pos(sk, p2)) {
            (Some(cc), Some(a), Some(b)) => {
                let a1 = (a[1] - cc[1]).atan2(a[0] - cc[0]);
                let mut sweep = (b[1] - cc[1]).atan2(b[0] - cc[0]) - a1;
                while sweep <= 1e-12 {
                    sweep += std::f64::consts::TAU;
                }
                while sweep > std::f64::consts::TAU + 1e-12 {
                    sweep -= std::f64::consts::TAU;
                }
                let r = (a[0] - cc[0]).hypot(a[1] - cc[1]);
                (0..=n)
                    .map(|i| {
                        let t = a1 + sweep * i as f64 / n as f64;
                        [cc[0] + r * t.cos(), cc[1] + r * t.sin()]
                    })
                    .collect()
            }
            _ => vec![],
        },
        SketchEntity::Ellipse { c, p, r, .. } => match (pos(sk, c), pos(sk, p)) {
            (Some(cc), Some(pp)) => {
                let a = (pp[0] - cc[0]).hypot(pp[1] - cc[1]);
                (0..=n)
                    .map(|i| {
                        let t = std::f64::consts::TAU * i as f64 / n as f64;
                        [cc[0] + a * t.cos(), cc[1] + r * t.sin()]
                    })
                    .collect()
            }
            _ => vec![],
        },
        SketchEntity::Spline { pts, .. } | SketchEntity::CvSpline { pts, .. } => pts.iter().filter_map(|p| pos(sk, p)).collect(),
        _ => vec![],
    }
}

/// Proper crossing of two segments (strictly inside both).
fn seg_cross(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Option<[f64; 2]> {
    let r = [b[0] - a[0], b[1] - a[1]];
    let s = [d[0] - c[0], d[1] - c[1]];
    let den = r[0] * s[1] - r[1] * s[0];
    if den.abs() < 1e-15 {
        return None;
    }
    let t = ((c[0] - a[0]) * s[1] - (c[1] - a[1]) * s[0]) / den;
    let u = ((c[0] - a[0]) * r[1] - (c[1] - a[1]) * r[0]) / den;
    if t > 1e-9 && t < 1.0 - 1e-9 && u > 1e-9 && u < 1.0 - 1e-9 {
        Some([a[0] + t * r[0], a[1] + t * r[1]])
    } else {
        None
    }
}

/// Interior crossings of any entity pair (circles included), skipping
/// crossings near a shared endpoint (touching there is not a crossing).
fn self_intersections(sk: &Sketch, curves: &[&SketchEntity], near: f64) -> Vec<Finding> {
    let mut out: Vec<Finding> = Vec::new();
    let samples: Vec<Vec<[f64; 2]>> = curves.iter().map(|e| sample_curve(sk, e, 48)).collect();
    let mut touch: Vec<[f64; 2]> = Vec::new();
    for e in curves {
        if let Some((a, b)) = endpoints(e) {
            for p in [a, b] {
                if let Some(q) = pos(sk, p) {
                    touch.push(q);
                }
            }
        }
    }
    for i in 0..curves.len() {
        for j in i + 1..curves.len() {
            let mut hit: Option<[f64; 2]> = None;
            'outer: for w in samples[i].windows(2) {
                for v in samples[j].windows(2) {
                    if let Some(x) = seg_cross(w[0], w[1], v[0], v[1]) {
                        if touch.iter().any(|t| dist(*t, x) <= near) {
                            continue;
                        }
                        hit = Some(x);
                        break 'outer;
                    }
                }
            }
            if let Some(x) = hit {
                out.push(Finding::new("selfIntersection").ent(&[curves[i].id(), curves[j].id()]).at(x));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sk(pts: &str, ents: &str, cons: &str) -> Sketch {
        let src = format!(
            r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{pts}],"entities":[{ents}],"constraints":[{cons}]}}"#
        );
        Sketch::from_json(&ck_json::parse(&src).unwrap())
    }
    fn an(sk: &Sketch) -> Analysis {
        analyze(sk, NEAR_TOL, TINY)
    }
    fn has(a: &Analysis, kind: &str) -> bool {
        a.findings.iter().any(|f| f.kind == kind)
    }
    fn one<'a>(a: &'a Analysis, kind: &str) -> &'a Finding {
        a.findings.iter().find(|f| f.kind == kind).unwrap_or_else(|| panic!("{kind} not in {:#?}", a.findings))
    }

    // ---- brief fixtures

    #[test]
    fn clean_square_has_no_findings_and_one_closed_chain() {
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":10},{"id":"p4","x":0,"y":10}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
               {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}"#,
            "",
        ));
        assert!(a.findings.is_empty(), "{:#?}", a.findings);
        assert_eq!(a.closed_chains, 1);
        assert_eq!(a.chains.len(), 1);
        assert!(a.chains[0].closed, "{:?}", a.chains);
        assert!(a.reason.is_none());
    }

    #[test]
    fn open_square_gap_is_open_end_with_the_gap() {
        // the e3-e4 corner misses: p4 (0,10) and p5 (0,10.02) are 0.02 apart
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":10},
               {"id":"p4","x":0,"y":10},{"id":"p5","x":0,"y":10.02}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
               {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p1","p2":"p5"}"#,
            "",
        ));
        let opens: Vec<_> = a.findings.iter().filter(|f| f.kind == "openEnd").collect();
        assert_eq!(opens.len(), 2, "{:#?}", a.findings);
        let at_p4 = opens.iter().find(|f| f.points[0] == "p4").expect("open end at p4");
        assert!((at_p4.gap.unwrap() - 0.02).abs() < 1e-9, "{at_p4:?}");
        assert_eq!(at_p4.nearest.as_deref(), Some("e4"));
        assert_eq!(at_p4.points, ["p4", "p5"]);
        assert_eq!(at_p4.at.unwrap(), [0.0, 10.0]);
        let at_p5 = opens.iter().find(|f| f.points[0] == "p5").expect("open end at p5");
        assert!((at_p5.gap.unwrap() - 0.02).abs() < 1e-9, "{at_p5:?}");
        // one open chain of all four lines, no profile
        assert_eq!(a.closed_chains, 0);
        assert!(!a.chains[0].closed && a.chains[0].entities.len() == 4, "{:?}", a.chains);
        let r = a.reason.as_ref().expect("reason");
        assert_eq!(r.kind, "openEnd");
        assert!((r.gap.unwrap() - 0.02).abs() < 1e-9);
        // the biggest gap; a tie resolves to the later finding (e4's p5 end)
        assert_eq!(r.entities, ["e4"]);
        assert_eq!(r.points, ["p5", "p4"]);
        assert_eq!(r.at.unwrap(), [0.0, 10.02]);
        assert_eq!(r.nearest.as_deref(), Some("e3"));
    }

    #[test]
    fn duplicate_line_is_flagged() {
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p1"}"#,
            "",
        ));
        let f = one(&a, "duplicate");
        assert!(f.entities == ["e1", "e2"], "{f:?}");
        assert!(!has(&a, "openEnd"), "{:#?}", a.findings);
    }

    #[test]
    fn overlapping_collinear_lines_are_flagged() {
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":4,"y":0},{"id":"p4","x":14,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p3","p2":"p4"}"#,
            "",
        ));
        let f = one(&a, "overlap");
        assert!(f.entities == ["e1", "e2"], "{f:?}");
        // parallel but offset lines are not an overlap
        let b = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":4,"y":1},{"id":"p4","x":14,"y":1}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p3","p2":"p4"}"#,
            "",
        ));
        assert!(!has(&b, "overlap"), "{:#?}", b.findings);
    }

    #[test]
    fn tee_junction_is_a_branch() {
        // e1, e3, e4 all end at p2
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":5},{"id":"p4","x":10,"y":-5}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e3","p1":"p2","p2":"p3"},
               {"type":"line","id":"e4","p1":"p2","p2":"p4"}"#,
            "",
        ));
        let f = one(&a, "branch");
        assert_eq!(f.degree, Some(3), "{f:?}");
        assert_eq!(f.points, ["p2"]);
        assert_eq!(f.entities.len(), 3, "{f:?}");
        assert_eq!(f.at.unwrap(), [10.0, 0.0]);
    }

    #[test]
    fn zero_length_line_is_flagged() {
        let a = an(&sk(r#"{"id":"p1","x":0,"y":0}"#, r#"{"type":"line","id":"e1","p1":"p1","p2":"p1"}"#, ""));
        let f = one(&a, "zeroLength");
        assert!(f.entities == ["e1"], "{f:?}");
        assert!(!has(&a, "isolatedPoint"), "{:#?}", a.findings);
    }

    #[test]
    fn tiny_segment_is_flagged() {
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":0.0005,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            "",
        ));
        let f = one(&a, "tinySegment");
        assert!(f.length.unwrap() < TINY && f.length.unwrap() > 0.0, "{f:?}");
        assert!(f.at.is_some());
        // the two ends are 0.0005 apart: nearCoincident too
        let n = one(&a, "nearCoincident");
        assert!((n.distance.unwrap() - 0.0005).abs() < 1e-12, "{n:?}");
        // a longer line is not tiny
        let b = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            "",
        ));
        assert!(!has(&b, "tinySegment"));
    }

    #[test]
    fn bow_tie_crosses_itself() {
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":10},{"id":"p3","x":0,"y":10},{"id":"p4","x":10,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p3","p2":"p4"}"#,
            "",
        ));
        let f = one(&a, "selfIntersection");
        assert!(f.entities == ["e1", "e2"], "{f:?}");
        let at = f.at.unwrap();
        assert!((at[0] - 5.0).abs() < 1e-6 && (at[1] - 5.0).abs() < 1e-6, "{f:?}");
        assert_eq!(a.closed_chains, 0);
        assert_eq!(a.chains.len(), 2, "{:?}", a.chains);
    }

    // ---- the rest of the kinds and the reply shape

    #[test]
    fn coincident_constraint_suppresses_near_coincident() {
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":0.005,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            r#"{"type":"coincident","id":"c1","refs":["p1","p2"]}"#,
        ));
        assert!(!has(&a, "nearCoincident"), "{:#?}", a.findings);
        // tolerance 0 disables the check
        let b = analyze(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":0.005,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            "",
        ), 0.0, TINY);
        assert!(!has(&b, "nearCoincident"));
    }

    #[test]
    fn suppressed_coincident_does_not_suppress_near_coincident() {
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":0.005,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            r#"{"type":"coincident","id":"c1","refs":["p1","p2"],"suppressed":true}"#,
        ));
        let f = one(&a, "nearCoincident");
        assert_eq!(f.points, ["p1", "p2"], "{f:?}");
        assert!((f.distance.unwrap() - 0.005).abs() < 1e-12);
    }

    #[test]
    fn circle_is_its_own_closed_chain() {
        let a = an(&sk(r#"{"id":"c","x":0,"y":0}"#, r#"{"type":"circle","id":"e1","c":"c","r":5}"#, ""));
        assert_eq!(a.closed_chains, 1);
        assert_eq!(a.chains.len(), 1);
        assert!(a.chains[0].closed && !a.chains[0].construction);
        assert!(a.findings.is_empty(), "{:#?}", a.findings);
        assert!(a.reason.is_none());
    }

    #[test]
    fn construction_geometry_is_not_a_profile() {
        let a = an(&sk(
            r#"{"id":"c","x":0,"y":0}"#,
            r#"{"type":"circle","id":"e1","c":"c","r":5,"construction":true}"#,
            "",
        ));
        assert_eq!(a.closed_chains, 0);
        assert!(a.chains[0].construction);
        assert_eq!(a.reason.as_ref().unwrap().kind, "constructionOnly");
    }

    #[test]
    fn isolated_point_is_flagged() {
        let a = an(&sk(r#"{"id":"p1","x":0,"y":0}"#, "", ""));
        let f = one(&a, "isolatedPoint");
        assert_eq!(f.points, ["p1"]);
        assert_eq!(f.at.unwrap(), [0.0, 0.0]);
        assert_eq!(a.reason.as_ref().unwrap().kind, "empty");
        assert_eq!(a.closed_chains, 0);
        // a point a line ends on is not isolated
        let b = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            "",
        ));
        assert!(!has(&b, "isolatedPoint"));
    }

    #[test]
    fn lettering_counts_as_a_closed_chain_but_is_not_analysed() {
        let a = an(&sk(
            r#"{"id":"t1","x":0,"y":0}"#,
            r#"{"type":"text","id":"e1","string":"M2","p":"t1","height":5,"at":[0,0]}"#,
            "",
        ));
        assert_eq!(a.closed_chains, 1, "{:#?}", a);
        assert!(a.findings.is_empty(), "{:#?}", a.findings);
        assert!(a.reason.is_none());
    }

    #[test]
    fn reply_shape() {
        let a = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            "",
        ));
        let j = a.to_json();
        let s = ck_json::to_string(&j);
        assert!(s.contains("\"tolerance\":0.01"), "{s}");
        assert!(s.contains("\"tiny\":0.001"), "{s}");
        assert!(s.contains("\"closedChains\":0"), "{s}");
        assert!(s.contains("\"kind\":\"openEnd\""), "{s}");
        assert!(s.contains("\"gap\""), "{s}");
        assert!(s.contains("\"closed\":false"), "{s}");
        // reason is omitted when a profile exists
        let clean = an(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p1"}"#,
            "",
        ));
        assert_eq!(clean.closed_chains, 1);
        let s2 = ck_json::to_string(&clean.to_json());
        assert!(!s2.contains("reason"), "{s2}");
    }
    // ---- sketchFreedoms fixtures

    use crate::sketch::status::sketch_status;

    fn status_dof(sk: &Sketch) -> usize {
        sketch_status(sk).dof
    }

    #[test]
    fn free_point_has_two_directions() {
        let k = sk(r#"{"id":"p1","x":1,"y":2}"#, "", "");
        let f = freedoms(&k);
        assert_eq!((f.dof, f.solved), (2, true), "{f:?}");
        assert_eq!(f.points, vec![PointFreedom { id: "p1".into(), free: 2, dir: None }]);
        assert!(f.radii.is_empty() && f.entities.is_empty());
        assert_eq!(f.dof, status_dof(&k));
    }

    #[test]
    fn half_determined_line_is_one_direction() {
        let k = sk(
            r#"{"id":"p1","x":0,"y":0,"fixed":true},{"id":"p2","x":10,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            r#"{"type":"horizontal","id":"c1","refs":["e1"]}"#,
        );
        let f = freedoms(&k);
        assert_eq!((f.dof, f.solved), (1, true), "{f:?}");
        assert_eq!(f.dof, status_dof(&k));
        assert_eq!(f.points[0], PointFreedom { id: "p1".into(), free: 0, dir: None });
        assert_eq!(f.points[1], PointFreedom { id: "p2".into(), free: 1, dir: Some([1.0, 0.0]) });
        assert_eq!(f.entities, vec![EntityFreedom { id: "e1".into(), kind: "line", free: 1 }]);
    }

    const RECT_PTS: &str = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":5},{"id":"p4","x":0,"y":5}"#;
    const RECT_ENTS: &str = r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
        {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}"#;
    const HV: &str = r#"{"type":"horizontal","id":"c1","refs":["e1"]},{"type":"vertical","id":"c2","refs":["e2"]},
        {"type":"horizontal","id":"c3","refs":["e3"]},{"type":"vertical","id":"c4","refs":["e4"]}"#;

    #[test]
    fn fully_defined_rectangle_has_no_freedoms() {
        let cons = format!(
            r#"{HV},{{"type":"fix","id":"c5","refs":["p1"],"value":[0,0]}},{{"type":"distance","id":"c6","refs":["p1","p2"],"value":10,"offset":null}},{{"type":"distance","id":"c7","refs":["p2","p3"],"value":5,"offset":null}}"#
        );
        let k = sk(RECT_PTS, RECT_ENTS, &cons);
        let f = freedoms(&k);
        assert_eq!((f.dof, f.solved), (0, true), "{f:?}");
        assert_eq!(f.dof, status_dof(&k));
        assert!(f.points.iter().all(|p| p.free == 0 && p.dir.is_none()), "{f:?}");
        assert!(f.entities.iter().all(|e| e.free == 0), "{f:?}");
    }

    #[test]
    fn rectangle_pinned_at_one_corner_keeps_its_two_shape_modes() {
        let cons = format!(r#"{HV},{{"type":"fix","id":"c5","refs":["p1"],"value":[0,0]}}"#);
        let k = sk(RECT_PTS, RECT_ENTS, &cons);
        let f = freedoms(&k);
        // H/V against a fixed corner pins p2.y and p4.x, leaving width and
        // height: p2 slides in x, p4 slides in y, p3 is the corner of both
        assert_eq!(f.dof, 2, "{f:?}");
        assert_eq!(f.dof, status_dof(&k));
        assert_eq!(f.points[0], PointFreedom { id: "p1".into(), free: 0, dir: None });
        assert_eq!(f.points[1], PointFreedom { id: "p2".into(), free: 1, dir: Some([1.0, 0.0]) });
        assert_eq!(f.points[2], PointFreedom { id: "p3".into(), free: 2, dir: None });
        assert_eq!(f.points[3], PointFreedom { id: "p4".into(), free: 1, dir: Some([0.0, 1.0]) });
        assert_eq!(f.entities, vec![
            EntityFreedom { id: "e1".into(), kind: "line", free: 1 },
            EntityFreedom { id: "e2".into(), kind: "line", free: 3 },
            EntityFreedom { id: "e3".into(), kind: "line", free: 3 },
            EntityFreedom { id: "e4".into(), kind: "line", free: 1 },
        ]);
    }

    #[test]
    fn circle_centre_and_radius_are_free_until_constrained() {
        let k = sk(r#"{"id":"c","x":0,"y":0}"#, r#"{"type":"circle","id":"e1","c":"c","r":5}"#, "");
        let f = freedoms(&k);
        assert_eq!((f.dof, f.solved), (3, true), "{f:?}");
        assert_eq!(f.dof, status_dof(&k));
        assert_eq!(f.points[0], PointFreedom { id: "c".into(), free: 2, dir: None });
        assert_eq!(f.radii, vec![RadiusFreedom { id: "e1".into(), free: true }]);
        assert_eq!(f.entities, vec![EntityFreedom { id: "e1".into(), kind: "circle", free: 3 }]);
        // a radius constraint pins the radius, not the centre
        let k2 = sk(
            r#"{"id":"c","x":0,"y":0}"#,
            r#"{"type":"circle","id":"e1","c":"c","r":5}"#,
            r#"{"type":"radius","id":"k1","refs":["e1"],"value":5}"#,
        );
        let f2 = freedoms(&k2);
        assert_eq!((f2.dof, f2.solved), (2, true), "{f2:?}");
        assert_eq!(f2.dof, status_dof(&k2));
        assert_eq!(f2.points[0].free, 2);
        assert_eq!(f2.radii, vec![RadiusFreedom { id: "e1".into(), free: false }]);
        assert_eq!(f2.entities[0].free, 2);
    }

    #[test]
    fn point_on_circle_frees_tangentially() {
        let k = sk(
            r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"p","x":5,"y":0}"#,
            r#"{"type":"circle","id":"e1","c":"o","r":5}"#,
            r#"{"type":"radius","id":"k1","refs":["e1"],"value":5},{"type":"pointOnCircle","id":"k2","refs":["p","e1"]}"#,
        );
        let f = freedoms(&k);
        assert_eq!((f.dof, f.solved), (1, true), "{f:?}");
        assert_eq!(f.dof, status_dof(&k));
        assert_eq!(f.points[0], PointFreedom { id: "o".into(), free: 0, dir: None });
        // tangent at (5,0): y only, sign-normalised positive
        assert_eq!(f.points[1], PointFreedom { id: "p".into(), free: 1, dir: Some([0.0, 1.0]) });
        assert_eq!(f.radii, vec![RadiusFreedom { id: "e1".into(), free: false }]);
    }

    #[test]
    fn arc_radius_follows_its_fixed_end_and_centre() {
        // centre and p1 fixed: the arc equation keeps |p2 - c|, so only p2
        // moves (tangent), and no motion changes the radius
        let k = sk(
            r#"{"id":"c0","x":0,"y":0,"fixed":true},{"id":"a","x":10,"y":0,"fixed":true},{"id":"b","x":0,"y":10}"#,
            r#"{"type":"arc","id":"e1","c":"c0","p1":"a","p2":"b"}"#,
            "",
        );
        let f = freedoms(&k);
        assert_eq!((f.dof, f.solved), (1, true), "{f:?}");
        assert_eq!(f.dof, status_dof(&k));
        assert_eq!(f.points[1], PointFreedom { id: "a".into(), free: 0, dir: None });
        assert_eq!(f.points[2], PointFreedom { id: "b".into(), free: 1, dir: Some([1.0, 0.0]) });
        assert_eq!(f.radii, vec![RadiusFreedom { id: "e1".into(), free: false }]);
        assert_eq!(f.entities, vec![EntityFreedom { id: "e1".into(), kind: "arc", free: 1 }]);
    }

    #[test]
    fn unbuildable_sketch_falls_back_to_two_per_free_point() {
        // a constraint on a missing point makes the build fail (entities
        // alone never do), so the status fallback applies
        let k = sk(
            r#"{"id":"p1","x":0,"y":0}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#,
            r#"{"type":"distance","id":"c1","refs":["p1","p9"],"value":5,"offset":null}"#,
        );
        let f = freedoms(&k);
        assert_eq!((f.dof, f.solved), (2, false), "{f:?}");
        assert_eq!(f.points, vec![PointFreedom { id: "p1".into(), free: 2, dir: None }]);
        assert_eq!(f.entities, vec![EntityFreedom { id: "e1".into(), kind: "line", free: 2 }]);
        assert_eq!(f.dof, status_dof(&k));
    }

    #[test]
    fn freedoms_reply_shape() {
        let f = freedoms(&sk(r#"{"id":"p1","x":1,"y":2}"#, "", ""));
        let s = ck_json::to_string(&f.to_json());
        assert!(s.contains("\"dof\":2"), "{s}");
        assert!(s.contains("\"solved\":true"), "{s}");
        assert!(s.contains("\"id\":\"p1\""), "{s}");
        assert!(s.contains("\"free\":2"), "{s}");
    }
}
