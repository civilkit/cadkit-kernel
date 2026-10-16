//! Measurement and analysis requests (read-only): `distance`, `maxDistance`, `thickness`, `thicknessAt`,
//! `closestPoint`, `angles`, `clearance`, `symmetry`. Wire shapes: docs/agent-queries.md "Measurement".
//!
//! Everything is built from two primitives on the existing kernel pieces: `nearest(prim, q)` (exact closest point of a
//! face / edge / vertex / point to a point; `Containment::face_nearest`, `dist_in_range`) and `samples(prim)`.
//! The distance of two primitives is a multi-start alternating projection (sample A, project on B, keep the best
//! starts, then project back and forth until the gain is below 1e-15 relative), exact for segment/segment pairs
//! (closed form) and for every pair where the projections are exact (plane, cylinder, cone, sphere, torus, B-spline by
//! the surface's own closest point). Sets of primitives (a lump, the whole body) are pruned by box distance.

use crate::boolean::classify::{face_loops_uv, Where};
use crate::boolean::imprint::dist_in_range;
use crate::bvh::{edge_box, face_box, Aabb};
use crate::contain::{Containment, Loc};
use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, EdgeId, FaceId, VertexId};
use std::collections::HashMap;

const TOL: f64 = 1e-6;
const MAX_POINTS: usize = 2000;

fn v3(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

fn vec3_of(j: Option<&Json>, what: &str) -> Result<Vec3, String> {
    let a = j.and_then(|j| j.as_vec3()).ok_or_else(|| format!("{what} must be [x, y, z]"))?;
    if !a.iter().all(|x| x.is_finite()) {
        return Err(format!("{what} must be finite"));
    }
    Ok(Vec3::new(a[0], a[1], a[2]))
}

/// `opts.k` else top-level `k`.
fn opt<'a>(msg: &'a Json, k: &str) -> Option<&'a Json> {
    msg.get("opts").and_then(|o| o.get(k)).or_else(|| msg.get(k))
}
fn opt_f(msg: &Json, k: &str, d: f64) -> f64 {
    opt(msg, k).and_then(|j| j.as_f64()).filter(|x| x.is_finite()).unwrap_or(d)
}
fn opt_n(msg: &Json, k: &str, d: usize, max: usize) -> usize {
    opt(msg, k).and_then(|j| j.as_usize()).unwrap_or(d).clamp(1, max)
}
fn opt_s<'a>(msg: &'a Json, k: &str) -> Option<&'a str> {
    opt(msg, k).and_then(|j| j.as_str())
}
fn deg(r: f64) -> f64 {
    r.to_degrees()
}


// ---------------------------------------------------------------- time budget

type Clock = fn() -> f64;

thread_local! {
    static CLOCK: std::cell::Cell<Option<Clock>> = const { std::cell::Cell::new(None) };
    /// (start ms, limit ms, truncated)
    static BUDGET: std::cell::Cell<(f64, f64, bool)> = const { std::cell::Cell::new((0.0, f64::INFINITY, false)) };
}

/// Install the monotonic clock (ms); without one (native tests) no request is ever cut short.
pub fn set_clock(f: Clock) {
    CLOCK.with(|c| c.set(Some(f)));
}

fn clock_now() -> f64 {
    CLOCK.with(|c| c.get()).map_or(0.0, |f| f())
}

/// Start a request: `opts.timeMs` (default 8000) is the wall-clock budget after which loops stop and the reply is
/// marked `truncated`.
fn begin(msg: &Json) {
    let limit = opt_f(msg, "timeMs", 8000.0).max(1.0);
    BUDGET.with(|b| b.set((clock_now(), limit, false)));
}

/// Has the budget run out? (marks the reply truncated)
fn over() -> bool {
    if CLOCK.with(|c| c.get()).is_none() {
        return false;
    }
    BUDGET.with(|b| {
        let (s, l, t) = b.get();
        if t {
            return true;
        }
        let out = clock_now() - s > l;
        if out {
            b.set((s, l, true));
        }
        out
    })
}

/// Mark the reply partial for a reason other than time (sample / pair caps).
fn mark_truncated() {
    BUDGET.with(|b| {
        let (s, l, _) = b.get();
        b.set((s, l, true));
    });
}

/// Add `elapsedMs` and `truncated` to an object reply.
fn stamp(mut j: Json) -> Json {
    let (s, _, t) = BUDGET.with(|b| b.get());
    if let Json::Obj(m) = &mut j {
        m.retain(|(k, _)| k != "truncated");
        m.push(("elapsedMs".into(), Json::Num((clock_now() - s).max(0.0).round())));
        m.push(("truncated".into(), Json::Bool(t)));
    }
    j
}

pub fn distance_reply(body: &Body, other: Option<&Body>, msg: &Json) -> Result<Json, String> {
    begin(msg);
    distance_inner(body, other, msg).map(stamp)
}

pub fn max_distance_reply(body: &Body, other: Option<&Body>, msg: &Json) -> Result<Json, String> {
    begin(msg);
    max_distance_inner(body, other, msg).map(stamp)
}

pub fn thickness_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    begin(msg);
    thickness_inner(body, msg).map(stamp)
}

pub fn thickness_at_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    begin(msg);
    thickness_at_inner(body, msg).map(stamp)
}

pub fn closest_point_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    begin(msg);
    closest_point_inner(body, msg).map(stamp)
}

pub fn angles_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    begin(msg);
    angles_inner(body, msg).map(stamp)
}

pub fn clearance_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    begin(msg);
    clearance_inner(body, msg).map(stamp)
}

pub fn symmetry_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    begin(msg);
    symmetry_inner(body, msg).map(stamp)
}

// ---------------------------------------------------------------- context

/// A body with its containment structure, edge keys and face index.
pub struct Ctx<'a> {
    pub body: &'a Body,
    pub c: Containment<'a>,
    fidx: HashMap<FaceId, usize>,
    /// `(key, chain)` for every listed wire edge (same keys as `describe` / `resolve`)
    keys: Vec<(String, Vec<EdgeId>)>,
    ekey: HashMap<EdgeId, usize>,
    pub diag: f64,
    /// Per face: its display-mesh points and the longest triangle edge `h`. Every mesh point lies on the face, so
    /// the nearest mesh point is an upper bound of the face distance and (nearest - h) a lower bound (up to the
    /// mesh covering radius): used to order and prune the exact projections on bodies with many faces.
    mesh: std::cell::OnceCell<HashMap<FaceId, (Vec<Vec3>, f64)>>,
}

impl<'a> Ctx<'a> {
    pub fn new(body: &'a Body) -> Ctx<'a> {
        let c = Containment::new(body, TOL);
        let fidx = c.faces().iter().enumerate().map(|(i, &f)| (f, i)).collect();
        let mut keys = vec![];
        for (seq, chain) in body.wire_numbered().0 {
            let e = chain[0];
            if body.is_split_rail(e) {
                continue;
            }
            let mut names: Vec<String> = body.edge_faces(e).iter().map(|&f| body.face_id_str(f)).collect();
            names.sort();
            names.dedup();
            if names.is_empty() {
                continue;
            }
            let key = if names.len() >= 2 { ck_topo::body::edge_key(&names[0], &names[1], seq) } else { format!("{}#{}", names[0], seq) };
            keys.push((key, chain));
        }
        let mut ekey = HashMap::new();
        for (i, (_, ch)) in keys.iter().enumerate() {
            for &e in ch {
                ekey.insert(e, i);
            }
        }
        let (lo, hi) = body.bbox();
        let diag = if lo.x.is_finite() && hi.x.is_finite() { (hi - lo).len().max(1e-9) } else { 1.0 };
        Ctx { body, c, fidx, keys, ekey, diag, mesh: Default::default() }
    }

    fn mesh(&self) -> &HashMap<FaceId, (Vec<Vec3>, f64)> {
        self.mesh.get_or_init(|| {
            let tess = ck_tess::mesh::tessellate_memo(self.body);
            let mut m = HashMap::new();
            for fm in tess.faces.iter().flatten() {
                let Some(f) = fm.face else { continue };
                let mut h = 0.0f64;
                for t in &fm.tris {
                    let (a, b, c) = (fm.pts[t[0] as usize], fm.pts[t[1] as usize], fm.pts[t[2] as usize]);
                    h = h.max(a.dist(b)).max(b.dist(c)).max(c.dist(a));
                }
                m.insert(f, (fm.pts.clone(), h));
            }
            m
        })
    }

    fn edge_name(&self, e: EdgeId) -> String {
        match self.ekey.get(&e) {
            Some(&i) => self.keys[i].0.clone(),
            None => format!("e{}", e.idx()),
        }
    }

    fn face_lump_index(&self, f: FaceId) -> usize {
        self.body.lump_index(self.body.face_lump(f))
    }
}

// ---------------------------------------------------------------- primitives

#[derive(Clone, Copy, Debug)]
pub enum Prim {
    Face(FaceId),
    Edge(EdgeId),
    Vtx(VertexId),
    Pt(Vec3),
}

fn prim_json(ctx: &Ctx, p: Prim) -> Json {
    match p {
        Prim::Face(f) => Obj::new().put("kind", "face").put("id", ctx.body.face_id_str(f)).build(),
        Prim::Edge(e) => Obj::new().put("kind", "edge").put("id", ctx.edge_name(e)).build(),
        Prim::Vtx(v) => Obj::new().put("kind", "vertex").put("id", format!("V{}", v.idx())).build(),
        Prim::Pt(_) => Obj::new().put("kind", "point").build(),
    }
}

fn prim_box(ctx: &Ctx, p: Prim) -> Aabb {
    match p {
        Prim::Face(f) => face_box(ctx.body, f),
        Prim::Edge(e) => edge_box(ctx.body, e),
        Prim::Vtx(v) => Aabb::from_points(&[ctx.body.vertex(v).p]),
        Prim::Pt(q) => Aabb::from_points(&[q]),
    }
}

fn prim_nearest(ctx: &Ctx, p: Prim, q: Vec3) -> Option<(f64, Vec3)> {
    match p {
        Prim::Face(f) => ctx.c.face_nearest(*ctx.fidx.get(&f)?, q),
        Prim::Edge(e) => {
            let ed = ctx.body.edge(e);
            let (t, d) = dist_in_range(&ed.curve, (ed.t0, ed.t1), q);
            d.is_finite().then(|| (d, ed.curve.eval(t)))
        }
        Prim::Vtx(v) => {
            let x = ctx.body.vertex(v).p;
            Some((x.dist(q), x))
        }
        Prim::Pt(x) => Some((x.dist(q), x)),
    }
}

/// uv box of a face's trim loops (falls back to the finite natural domain).
fn face_uv_box(ctx: &Ctx, f: FaceId) -> Option<((f64, f64), (f64, f64))> {
    let loops = face_loops_uv(ctx.body, f);
    let (mut u0, mut u1, mut v0, mut v1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    let mut any = false;
    if !loops.iter().all(|l| l.len() < 3) {
        for l in &loops {
            for p in l {
                if p.x.is_finite() && p.y.is_finite() {
                    u0 = u0.min(p.x);
                    u1 = u1.max(p.x);
                    v0 = v0.min(p.y);
                    v1 = v1.max(p.y);
                    any = true;
                }
            }
        }
    }
    if !any {
        let ((a, b), (c, d)) = ctx.body.face(f).surface.domain();
        if ![a, b, c, d].iter().all(|x| x.is_finite()) {
            return None;
        }
        (u0, u1, v0, v1) = (a, b, c, d);
    }
    (u1 > u0 && v1 > v0).then_some(((u0, u1), (v0, v1)))
}

/// `n x n` cell-centre grid over the face's uv box: `Some((u, v, point))` where the cell centre is inside the face.
fn face_grid(ctx: &Ctx, f: FaceId, n: usize) -> Vec<Option<(f64, f64, Vec3)>> {
    let Some(((u0, u1), (v0, v1))) = face_uv_box(ctx, f) else { return vec![None; 0] };
    let Some(&fi) = ctx.fidx.get(&f) else { return vec![] };
    let surf = &ctx.body.face(f).surface;
    let mut out = Vec::with_capacity(n * n);
    for i in 0..n {
        for j in 0..n {
            let u = u0 + (u1 - u0) * (i as f64 + 0.5) / n as f64;
            let v = v0 + (v1 - v0) * (j as f64 + 0.5) / n as f64;
            if ctx.c.point_in_face(fi, u, v, TOL) == Where::In {
                let p = surf.eval(u, v);
                out.push(p.x.is_finite().then_some((u, v, p)));
            } else {
                out.push(None);
            }
        }
    }
    out
}

fn prim_samples(ctx: &Ctx, p: Prim, n: usize) -> Vec<Vec3> {
    match p {
        Prim::Pt(x) => vec![x],
        Prim::Vtx(v) => vec![ctx.body.vertex(v).p],
        Prim::Edge(e) => {
            let ed = ctx.body.edge(e);
            ed.curve.sample(ed.t0, ed.t1, n.max(2) * 2)
        }
        Prim::Face(f) => {
            // interior first: ties between equally good starts then resolve to interior points
            let mut out: Vec<Vec3> = face_grid(ctx, f, n).into_iter().flatten().map(|g| g.2).collect();
            for e in ctx.body.face_edges(f) {
                let ed = ctx.body.edge(e);
                out.extend(ed.curve.sample(ed.t0, ed.t1, 8));
            }
            out
        }
    }
}

// ---------------------------------------------------------------- pair distance

#[derive(Clone, Copy, Debug)]
pub struct Hit {
    pub d: f64,
    pub a: Vec3,
    pub b: Vec3,
}

fn refine(ca: &Ctx, pa: Prim, cb: &Ctx, pb: Prim, start: Vec3) -> Option<Hit> {
    refine_n(ca, pa, cb, pb, start, 400)
}

fn refine_n(ca: &Ctx, pa: Prim, cb: &Ctx, pb: Prim, start: Vec3, iters: usize) -> Option<Hit> {
    let mut x = start;
    let mut best: Option<Hit> = None;
    let mut prev = f64::INFINITY;
    for _ in 0..iters {
        let (_, y) = prim_nearest(cb, pb, x)?;
        let (d, x2) = prim_nearest(ca, pa, y)?;
        x = x2;
        if best.map_or(true, |h| d < h.d) {
            best = Some(Hit { d, a: x2, b: y });
        }
        if prev - d <= 1e-15 * (1.0 + d) {
            break;
        }
        prev = d;
    }
    best
}

/// Exact minimum of two trimmed lines.
fn line_line(ca: &Ctx, ea: EdgeId, cb: &Ctx, eb: EdgeId) -> Option<Hit> {
    let (a, b) = (ca.body.edge(ea), cb.body.edge(eb));
    let (oa, ob) = (a.curve.eval(0.0), b.curve.eval(0.0));
    let (da, db) = (a.curve.eval(1.0) - oa, b.curve.eval(1.0) - ob);
    let mut best: Option<Hit> = None;
    let mut push = |pa: Vec3, pb: Vec3| {
        let d = pa.dist(pb);
        if best.map_or(true, |h| d < h.d) {
            best = Some(Hit { d, a: pa, b: pb });
        }
    };
    // unconstrained solution
    let w = oa - ob;
    let (aa, bb, cc, dd, ee) = (da.dot(da), da.dot(db), db.dot(db), da.dot(w), db.dot(w));
    let den = aa * cc - bb * bb;
    if den.abs() > 1e-14 * aa * cc {
        let s = (bb * ee - cc * dd) / den;
        let t = (aa * ee - bb * dd) / den;
        let eps = 1e-12;
        if s >= a.t0 - eps && s <= a.t1 + eps && t >= b.t0 - eps && t <= b.t1 + eps {
            push(a.curve.eval(s), b.curve.eval(t));
        }
    }
    for &t in &[a.t0, a.t1] {
        let p = a.curve.eval(t);
        if let Some((_, q)) = prim_nearest(cb, Prim::Edge(eb), p) {
            push(p, q);
        }
    }
    for &t in &[b.t0, b.t1] {
        let q = b.curve.eval(t);
        if let Some((_, p)) = prim_nearest(ca, Prim::Edge(ea), q) {
            push(p, q);
        }
    }
    best
}

fn is_line(ctx: &Ctx, e: EdgeId) -> bool {
    matches!(ctx.body.edge(e).curve, ck_geom::curve::Curve::Line { .. })
}

pub fn pair_min(ca: &Ctx, pa: Prim, cb: &Ctx, pb: Prim, n: usize) -> Option<Hit> {
    match (pa, pb) {
        (Prim::Pt(_) | Prim::Vtx(_), _) => {
            let x = prim_samples(ca, pa, 1)[0];
            let (d, y) = prim_nearest(cb, pb, x)?;
            return Some(Hit { d, a: x, b: y });
        }
        (_, Prim::Pt(_) | Prim::Vtx(_)) => {
            let y = prim_samples(cb, pb, 1)[0];
            let (d, x) = prim_nearest(ca, pa, y)?;
            return Some(Hit { d, a: x, b: y });
        }
        (Prim::Edge(ea), Prim::Edge(eb)) if is_line(ca, ea) && is_line(cb, eb) => return line_line(ca, ea, cb, eb),
        _ => {}
    }
    let mut starts: Vec<(f64, Vec3)> = vec![];
    for s in prim_samples(ca, pa, n) {
        if let Some((d, _)) = prim_nearest(cb, pb, s) {
            starts.push((d, s));
        }
    }
    for s in prim_samples(cb, pb, n) {
        if let Some((d, x)) = prim_nearest(ca, pa, s) {
            starts.push((d, x));
        }
    }
    starts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let mut best: Option<Hit> = None;
    let mut used: Vec<Vec3> = vec![];
    for (_, s) in starts {
        if used.len() >= 4 {
            break;
        }
        if used.iter().any(|u| u.dist(s) < 1e-9) {
            continue;
        }
        used.push(s);
        if let Some(h) = refine(ca, pa, cb, pb, s) {
            if best.map_or(true, |b| h.d < b.d) {
                best = Some(h);
            }
        }
    }
    best
}

// ---------------------------------------------------------------- entities

pub struct Ent<'a> {
    pub ctx: &'a Ctx<'a>,
    pub prims: Vec<Prim>,
    boxes: Vec<Aabb>,
}

fn box_dist(a: &Aabb, b: &Aabb) -> f64 {
    let g = |l0: f64, h0: f64, l1: f64, h1: f64| (l1 - h0).max(l0 - h1).max(0.0);
    let (dx, dy, dz) = (g(a.min.x, a.max.x, b.min.x, b.max.x), g(a.min.y, a.max.y, b.min.y, b.max.y), g(a.min.z, a.max.z, b.min.z, b.max.z));
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn box_dist_pt(a: &Aabb, q: Vec3) -> f64 {
    box_dist(a, &Aabb::from_points(&[q]))
}

impl<'a> Ent<'a> {
    fn new(ctx: &'a Ctx<'a>, prims: Vec<Prim>) -> Ent<'a> {
        let boxes = prims.iter().map(|&p| prim_box(ctx, p)).collect();
        Ent { ctx, prims, boxes }
    }

    /// Nearest point of the entity to `q`: `(distance, point, prim index)`.
    pub fn nearest(&self, q: Vec3) -> Option<(f64, Vec3, usize)> {
        // many faces: visit them by their nearest display-mesh point (a sound upper bound of the face distance, since
        // mesh points lie on the face) and prune with the box distance only: a mesh can be incomplete, so it is never
        // used as a lower bound
        let use_mesh = self.prims.len() > 12 && self.prims.iter().all(|p| matches!(p, Prim::Face(_)));
        let mesh = if use_mesh { Some(self.ctx.mesh()) } else { None };
        let mut ub = f64::INFINITY;
        let mut order: Vec<(f64, f64, usize)> = self
            .boxes
            .iter()
            .enumerate()
            .map(|(i, b)| {
                let lb = box_dist_pt(b, q);
                if let (Some(m), Prim::Face(f)) = (mesh, self.prims[i]) {
                    if let Some((pts, _)) = m.get(&f) {
                        let md = pts.iter().map(|p| p.dist(q)).fold(f64::INFINITY, f64::min);
                        ub = ub.min(md);
                        return (lb, md, i);
                    }
                }
                (lb, lb, i)
            })
            .collect();
        order.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        let mut best: Option<(f64, Vec3, usize)> = None;
        for (lb, _, i) in order {
            let bound = best.map_or(ub, |b| b.0.min(ub));
            if lb > bound + 1e-12 {
                if mesh.is_some() {
                    continue;
                }
                break;
            }
            if let Some((d, p)) = prim_nearest(self.ctx, self.prims[i], q) {
                if best.map_or(true, |b| d < b.0) {
                    best = Some((d, p, i));
                }
            }
        }
        best
    }

    fn solid_like(&self) -> bool {
        self.prims.iter().any(|p| matches!(p, Prim::Face(_)))
    }
}

/// Closest pair of display-mesh points of two faces: `(distance, point on A, slack)`; the true distance lies in
/// `[distance - slack, distance]` (up to the mesh covering radius).
fn mesh_pair(ca: &Ctx, pa: Prim, cb: &Ctx, pb: Prim) -> Option<(f64, Vec3, f64)> {
    let (Prim::Face(fa), Prim::Face(fb)) = (pa, pb) else { return None };
    let ((ma, ha), (mb, hb)) = (ca.mesh().get(&fa)?, cb.mesh().get(&fb)?);
    let mut best = (f64::INFINITY, Vec3::ZERO);
    for p in ma {
        for q in mb {
            let d = p.dist(*q);
            if d < best.0 {
                best = (d, *p);
            }
        }
    }
    best.0.is_finite().then_some((best.0, best.1, ha + hb))
}

/// Minimum distance of two entities with the prims it is attained on.
fn ent_min(a: &Ent, b: &Ent, n: usize) -> Option<(Hit, usize, usize)> {
    let mut pairs: Vec<(f64, usize, usize)> = vec![];
    for (i, ba) in a.boxes.iter().enumerate() {
        for (j, bb) in b.boxes.iter().enumerate() {
            pairs.push((box_dist(ba, bb), i, j));
        }
    }
    pairs.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
    let mut best: Option<(Hit, usize, usize)> = None;
    for (lb, i, j) in pairs {
        if over() {
            break;
        }
        if best.map_or(false, |(h, _, _)| lb > h.d + 1e-12) {
            break;
        }
        if let Some(h) = pair_min(a.ctx, a.prims[i], b.ctx, b.prims[j], n) {
            if best.map_or(true, |(bh, _, _)| h.d < bh.d) {
                best = Some((h, i, j));
            }
        }
    }
    best
}

/// Entity specifier -> primitives. `body`, `lump:N`, `V<n>`, `e<n>`, an edge key, a face id, `[x,y,z]`, or an object
/// `{face|edge|vertex|lump|point|faces|edges}`, or an array of specs.
fn parse_entity(ctx: &Ctx, j: &Json, what: &str) -> Result<Vec<Prim>, String> {
    match j {
        Json::Arr(a) if a.len() == 3 && a.iter().all(|x| x.as_f64().is_some()) => Ok(vec![Prim::Pt(vec3_of(Some(j), what)?)]),
        Json::Arr(a) => {
            let mut out = vec![];
            for x in a {
                out.extend(parse_entity(ctx, x, what)?);
            }
            Ok(out)
        }
        Json::Str(s) => parse_name(ctx, s, what),
        Json::Obj(_) => {
            if let Some(v) = j.get("point") {
                return Ok(vec![Prim::Pt(vec3_of(Some(v), what)?)]);
            }
            if let Some(Json::Bool(true)) = j.get("body") {
                return parse_name(ctx, "body", what);
            }
            for k in ["face", "edge", "vertex", "lump", "faces", "edges"] {
                if let Some(v) = j.get(k) {
                    return match v {
                        Json::Str(s) => parse_name(ctx, &if k == "lump" { format!("lump:{s}") } else if k == "vertex" { format!("V{s}") } else { s.clone() }, what),
                        Json::Num(n) if k == "lump" => parse_name(ctx, &format!("lump:{}", *n as usize), what),
                        Json::Num(n) if k == "vertex" => parse_name(ctx, &format!("V{}", *n as usize), what),
                        other => parse_entity(ctx, other, what),
                    };
                }
            }
            Err(format!("{what}: expected a face / edge / vertex id, a point, a lump or \"body\""))
        }
        _ => Err(format!("{what}: expected a face / edge / vertex id, a point, a lump or \"body\"")),
    }
}

fn parse_name(ctx: &Ctx, s: &str, what: &str) -> Result<Vec<Prim>, String> {
    let b = ctx.body;
    if s == "body" {
        return Ok(b.face_ids().map(Prim::Face).collect());
    }
    if let Some(n) = s.strip_prefix("lump:") {
        let n: usize = n.parse().map_err(|_| format!("{what}: bad lump index {s}"))?;
        let v: Vec<Prim> = b.face_ids().filter(|&f| ctx.face_lump_index(f) == n).map(Prim::Face).collect();
        if v.is_empty() {
            return Err(format!("{what}: no lump {n} (the body has {})", b.num_lumps()));
        }
        return Ok(v);
    }
    if let Some(n) = s.strip_prefix('V').and_then(|n| n.parse::<usize>().ok()) {
        let v = b.vertex_ids().find(|v| v.idx() == n).ok_or_else(|| format!("{what}: no vertex {s}"))?;
        return Ok(vec![Prim::Vtx(v)]);
    }
    if let Some(i) = ctx.keys.iter().position(|(k, _)| k == s) {
        return Ok(ctx.keys[i].1.iter().map(|&e| Prim::Edge(e)).collect());
    }
    if let Some(n) = s.strip_prefix('e').and_then(|n| n.parse::<usize>().ok()) {
        if let Some(e) = b.edge_ids().find(|e| e.idx() == n) {
            return Ok(vec![Prim::Edge(e)]);
        }
    }
    let v: Vec<Prim> = b.face_ids().filter(|&f| b.face_id_str(f) == s).map(Prim::Face).collect();
    if v.is_empty() {
        return Err(format!("{what}: unknown entity {s:?} (face ids as in describe, edge keys, V<n>, lump:<n>, body)"));
    }
    Ok(v)
}

fn ent_label(spec: Option<&Json>) -> Json {
    match spec {
        Some(Json::Str(s)) => Json::Str(s.clone()),
        _ => Json::Null,
    }
}

// ---------------------------------------------------------------- distance

fn other_ctx_body<'a>(body: &'a Body, other: Option<&'a Body>) -> &'a Body {
    other.unwrap_or(body)
}

/// `{type:"distance", tag, tagB?, a, b, opts?:{samples}}`.
fn distance_inner(body: &Body, other: Option<&Body>, msg: &Json) -> Result<Json, String> {
    let ca = Ctx::new(body);
    let cb_owned;
    let cb: &Ctx = if other.is_some() {
        cb_owned = Ctx::new(other_ctx_body(body, other));
        &cb_owned
    } else {
        &ca
    };
    let sa = msg.get("a").ok_or("distance: a and b are required (face id | edge key | V<n> | [x,y,z] | lump:<n> | body)")?;
    let sb = msg.get("b").ok_or("distance: a and b are required (face id | edge key | V<n> | [x,y,z] | lump:<n> | body)")?;
    let pa = parse_entity(&ca, sa, "distance: a")?;
    let pb = parse_entity(cb, sb, "distance: b")?;
    let n = opt_n(msg, "samples", 7, 40);
    let ea = Ent::new(&ca, pa);
    let eb = Ent::new(cb, pb);
    let (h, i, j) = ent_min(&ea, &eb, n).ok_or("distance: no distance could be computed (empty or degenerate entities)")?;
    let mut o = Obj::new();
    let mut dist = h.d;
    let mut intersects = h.d <= 1e-9;
    let mut kind: Option<&str> = None;
    if intersects {
        dist = 0.0;
        // extent of the contact: the richest over all touching primitive pairs (a face resting on a face is an area,
        // along an edge a curve, at a corner a point)
        let mut rank = 1;
        let mut checked = 0;
        for (pi, bi) in ea.boxes.iter().enumerate() {
            for (pj, bj) in eb.boxes.iter().enumerate() {
                if rank == 3 || checked > 400 || box_dist(bi, bj) > 1e-7 {
                    continue;
                }
                checked += 1;
                let sm = prim_samples(&ca, ea.prims[pi], n);
                let near = sm.iter().filter(|&&s| prim_nearest(cb, eb.prims[pj], s).map_or(false, |(d, _)| d <= 1e-7)).count();
                let r = if near <= 1 { 1 } else if (near as f64) < 0.5 * sm.len() as f64 { 2 } else { 3 };
                rank = rank.max(r);
            }
        }
        kind = Some(["point", "point", "curve", "area"][rank]);
    } else if other.is_some() && ea.solid_like() && eb.solid_like() {
        // one solid inside the other: no boundary contact, but the solids intersect
        let sa0 = prim_samples(&ca, ea.prims[i], 3).first().copied();
        let sb0 = prim_samples(cb, eb.prims[j], 3).first().copied();
        let ina = sa0.map_or(false, |p| cb.c.classify_point(p, TOL) == Loc::In);
        let inb = sb0.map_or(false, |p| ca.c.classify_point(p, TOL) == Loc::In);
        if ina || inb {
            intersects = true;
            kind = Some("containment");
            o = o.put("boundaryDistance", h.d);
            dist = 0.0;
        }
    }
    let dir = (h.b - h.a).len();
    o = o
        .put("distance", dist)
        .put("pointA", v3(h.a))
        .put("pointB", v3(h.b))
        .put("a", prim_json(&ca, ea.prims[i]))
        .put("b", prim_json(cb, eb.prims[j]))
        .put("intersects", intersects)
        .put("direction", if dir > 1e-12 { v3((h.b - h.a) * (1.0 / dir)) } else { Json::Null })
        .put("aLabel", ent_label(Some(sa)))
        .put("bLabel", ent_label(Some(sb)));
    if let Some(k) = kind {
        o = o.put("intersection", Obj::new().put("kind", k).build());
    }
    Ok(o.build())
}

/// Surface samples of a set of faces: `(point, face)`, about `budget` in total, plus the covering radius bound.
fn body_samples(ctx: &Ctx, faces: &[FaceId], budget: usize) -> (Vec<(Vec3, FaceId)>, f64) {
    let nf = faces.len().max(1);
    let g = (((budget as f64 / nf as f64).sqrt()).round() as usize).clamp(3, 80);
    // boundary samples per edge: a quarter of the budget over all face-edge incidences
    let incid: usize = faces.iter().map(|&f| ctx.body.face_edges(f).len()).sum::<usize>().max(1);
    let ne = (budget / 4 / incid).clamp(2, 24);
    let mut out = vec![];
    let mut cover = 0.0f64;
    for &f in faces {
        let grid = face_grid(ctx, f, g);
        for (k, c) in grid.iter().enumerate() {
            let Some((_, _, p)) = c else { continue };
            out.push((*p, f));
            let (i, j) = (k / g, k % g);
            if i + 1 < g {
                if let Some((_, _, q)) = grid[(i + 1) * g + j] {
                    cover = cover.max(p.dist(q));
                }
            }
            if j + 1 < g {
                if let Some((_, _, q)) = grid[i * g + j + 1] {
                    cover = cover.max(p.dist(q));
                }
            }
        }
        for e in ctx.body.face_edges(f) {
            let ed = ctx.body.edge(e);
            let pts = ed.curve.sample(ed.t0, ed.t1, ne);
            for w in pts.windows(2) {
                cover = cover.max(w[0].dist(w[1]));
            }
            out.extend(pts.into_iter().map(|p| (p, f)));
        }
    }
    // cell half-diagonal; boundary-adjacent cells that have no grid sample are within one more cell of an edge sample
    (out, cover)
}

/// A permutation of `0..n` that visits the range evenly (stride walk), so a run cut short still covers every face.
fn stride_order(n: usize) -> Vec<usize> {
    if n < 3 {
        return (0..n).collect();
    }
    let mut k = (n as f64 * 0.6180339887) as usize | 1;
    while gcd(k, n) != 1 {
        k += 2;
    }
    (0..n).map(|i| i * k % n).collect()
}

fn gcd(a: usize, b: usize) -> usize {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// `{type:"maxDistance", tag, tagB?, a?, b?, opts?:{samples, both}}`: one-sided Hausdorff distance from `a` to `b`
/// (both default to the whole body), sampled densely with an error bound.
fn max_distance_inner(body: &Body, other: Option<&Body>, msg: &Json) -> Result<Json, String> {
    let ca = Ctx::new(body);
    let cb_owned;
    let cb: &Ctx = if other.is_some() {
        cb_owned = Ctx::new(other_ctx_body(body, other));
        &cb_owned
    } else {
        &ca
    };
    let budget = opt_n(msg, "samples", 3000, 400000);
    let both = matches!(opt(msg, "both"), Some(Json::Bool(true)));
    let default = Json::Str("body".into());
    let pa = parse_entity(&ca, msg.get("a").unwrap_or(&default), "maxDistance: a")?;
    let pb = parse_entity(cb, msg.get("b").unwrap_or(&default), "maxDistance: b")?;
    let one = |xa: &Ctx, pa: &[Prim], xb: &Ctx, pb: Vec<Prim>| -> Result<Json, String> {
        let eb = Ent::new(xb, pb);
        let faces: Vec<FaceId> = pa.iter().filter_map(|p| if let Prim::Face(f) = p { Some(*f) } else { None }).collect();
        let mut pts: Vec<(Vec3, Option<FaceId>)> = vec![];
        let mut bound = 0.0;
        if !faces.is_empty() {
            let (s, c) = body_samples(xa, &faces, budget);
            bound = c;
            pts.extend(s.into_iter().map(|(p, f)| (p, Some(f))));
        }
        for p in pa {
            match p {
                Prim::Face(_) => {}
                other => {
                    let sm = prim_samples(xa, *other, 64);
                    for w in sm.windows(2) {
                        bound = f64::max(bound, w[0].dist(w[1]) * 0.5);
                    }
                    pts.extend(sm.into_iter().map(|p| (p, None)));
                }
            }
        }
        let (mut mx, mut at, mut near, mut af) = (-1.0f64, Vec3::ZERO, Vec3::ZERO, None);
        let (mut sum2, mut cnt) = (0.0, 0usize);
        let total_pts = pts.len();
        let order = stride_order(total_pts);
        for oi in order {
            if over() {
                break;
            }
            let (p, f) = &pts[oi];
            if let Some((d, q, _)) = eb.nearest(*p) {
                sum2 += d * d;
                cnt += 1;
                if d > mx {
                    (mx, at, near, af) = (d, *p, q, *f);
                }
            }
        }
        if cnt == 0 {
            return Err("maxDistance: nothing to sample".into());
        }
        let mut o = Obj::new()
            .put("maxDistance", mx)
            .put("location", v3(at))
            .put("nearest", v3(near))
            .put("errorBound", bound)
            .put("upperBound", mx + bound)
            .put("rms", (sum2 / cnt as f64).sqrt())
            .put("samples", cnt as f64)
            .put("coverage", cnt as f64 / total_pts.max(1) as f64);
        if let Some(f) = af {
            o = o.put("face", xa.body.face_id_str(f));
        }
        Ok(o.build())
    };
    let ab = one(&ca, &pa, cb, pb.clone())?;
    if !both {
        return Ok(ab);
    }
    let ba = one(cb, &pb, &ca, pa.clone())?;
    let m = ab.get_f64("maxDistance").unwrap_or(0.0).max(ba.get_f64("maxDistance").unwrap_or(0.0));
    let e = ab.get_f64("errorBound").unwrap_or(0.0).max(ba.get_f64("errorBound").unwrap_or(0.0));
    Ok(Obj::new().put("hausdorff", m).put("errorBound", e).put("aToB", ab).put("bToA", ba).build())
}

// ---------------------------------------------------------------- thickness

struct Th {
    t: f64,
    p: Vec3,
    far: Vec3,
    face: FaceId,
    far_face: FaceId,
    n: Vec3,
}

fn ray_thickness(ctx: &Ctx, f: FaceId, u: f64, v: f64) -> Option<Th> {
    let n = ctx.body.outward_normal(f, u, v);
    if !(n.x.is_finite() && n.y.is_finite() && n.z.is_finite()) || n.len() < 0.5 {
        return None;
    }
    let n = n.normalize();
    let p = ctx.body.face(f).surface.eval(u, v);
    let hits = ctx.c.ray_hits(p, -n, ctx.diag * 2.0 + 1.0, TOL);
    let h = hits.iter().find(|h| h.t > 1e-6 && !h.entering && !h.tangent)?;
    Some(Th { t: h.t, p, far: h.point, face: f, far_face: h.face, n })
}

/// Largest ball tangent at `p` (inside, along `-n`) that stays inside: radius by bisection on `[0, t/2]`.
fn inscribed_radius(ctx: &Ctx, p: Vec3, n: Vec3, t: f64) -> f64 {
    let all = |c: Vec3| ctx.c.nearest(c).map_or(0.0, |x| x.0);
    let (mut lo, mut hi) = (0.0, t * 0.5);
    if all(p - n * hi) >= hi * (1.0 - 1e-7) - 1e-9 {
        return hi;
    }
    for _ in 0..26 {
        let mid = 0.5 * (lo + hi);
        if all(p - n * mid) >= mid * (1.0 - 1e-7) - 1e-9 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

fn th_json(ctx: &Ctx, t: &Th, thickness: f64) -> Json {
    Obj::new()
        .put("thickness", thickness)
        .put("point", v3(t.p))
        .put("farPoint", v3(t.far))
        .put("face", ctx.body.face_id_str(t.face))
        .put("farFace", ctx.body.face_id_str(t.far_face))
        .put("direction", v3(-t.n))
        .build()
}

/// `{type:"thickness", tag, opts?:{samples, top, radius, below, method:"ray"|"sphere", sphereSamples}}`.
fn thickness_inner(body: &Body, msg: &Json) -> Result<Json, String> {
    let ctx = Ctx::new(body);
    let budget = opt_n(msg, "samples", 2500, 200000);
    let top = opt_n(msg, "top", 10, 200);
    let below = opt_f(msg, "below", f64::INFINITY);
    let sphere = opt_s(msg, "method") == Some("sphere");
    let faces: Vec<FaceId> = body.face_ids().collect();
    if faces.is_empty() {
        return Err("thickness: the body has no faces".into());
    }
    let g = (((budget as f64 / faces.len() as f64).sqrt()).round() as usize).clamp(3, 80);
    let mut ths: Vec<Th> = vec![];
    let (mut total, mut noexit) = (0usize, 0usize);
    for &f in &faces {
        for c in face_grid(&ctx, f, g).into_iter().flatten() {
            if over() {
                break;
            }
            total += 1;
            match ray_thickness(&ctx, f, c.0, c.1) {
                Some(t) => ths.push(t),
                None => noexit += 1,
            }
        }
    }
    ths.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
    // optional: local thickness (largest inscribed ball) for the thinnest ray samples
    let mut vals: Vec<(f64, usize)> = ths.iter().enumerate().map(|(i, t)| (t.t, i)).collect();
    if sphere {
        let k = opt_n(msg, "sphereSamples", 200, 5000).min(ths.len());
        for (v, i) in vals.iter_mut().take(k) {
            if over() {
                break;
            }
            let t = &ths[*i];
            // a sample closer to its face's boundary than half its thickness is a corner: the ball is cut by the
            // neighbouring face there (a sharp edge always dips to 0), not by a thin wall, so it keeps its ray value
            let edge_d = body
                .face_edges(t.face)
                .into_iter()
                .map(|e| {
                    let ed = body.edge(e);
                    dist_in_range(&ed.curve, (ed.t0, ed.t1), t.p).1
                })
                .fold(f64::INFINITY, f64::min);
            if edge_d >= 0.5 * t.t * 0.999 {
                *v = 2.0 * inscribed_radius(&ctx, t.p, t.n, t.t);
            }
        }
        vals.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    }
    let mut o = Obj::new().put("method", if sphere { "sphere" } else { "ray" }).put("samples", total as f64).put("noExit", noexit as f64);
    let Some(&(minv, mi)) = vals.first() else {
        return Ok(o.put("minThickness", Json::Null).put("thin", Json::Arr(vec![])).build());
    };
    let m = &ths[mi];
    o = o.put("minThickness", minv).put("location", th_json(&ctx, m, minv));
    // clustering: greedy, thinnest first; representatives at least `radius` apart
    let radius = opt_f(msg, "radius", (minv * 2.0).max(ctx.diag * 0.02));
    let mut reps: Vec<(f64, usize)> = vec![];
    for &(v, i) in &vals {
        if v > below || reps.len() >= top {
            break;
        }
        if reps.iter().all(|&(_, j)| ths[j].p.dist(ths[i].p) > radius) {
            reps.push((v, i));
        }
    }
    let sum: f64 = ths.iter().map(|t| t.t).sum();
    o = o
        .put("meanThickness", sum / ths.len().max(1) as f64)
        .put("clusterRadius", radius)
        .put("thin", Json::Arr(reps.iter().map(|&(v, i)| th_json(&ctx, &ths[i], v)).collect()));
    Ok(o.build())
}

/// `{type:"thicknessAt", tag, point | face}`: wall thickness along the inward normal at the surface point nearest to
/// `point`, or the min / max / mean over `face`.
fn thickness_at_inner(body: &Body, msg: &Json) -> Result<Json, String> {
    let ctx = Ctx::new(body);
    if let Some(p) = msg.get("point") {
        let q = vec3_of(Some(p), "thicknessAt: point")?;
        let (d, f, np) = ctx.c.nearest(q).ok_or("thicknessAt: the body has no faces")?;
        let (u, v) = body.face(f).surface.closest_param(np);
        let t = ray_thickness(&ctx, f, u, v).ok_or("thicknessAt: no exit found along the inward normal (open shell or edge point)")?;
        let mut r = th_json(&ctx, &t, t.t);
        if let Json::Obj(_) = r {
            r = Obj::new().put("distanceToSurface", d).put("result", r).build();
        }
        return Ok(r);
    }
    if let Some(id) = msg.get_str("face") {
        let prims = parse_name(&ctx, id, "thicknessAt: face")?;
        let g = opt_n(msg, "samples", 24, 100);
        let (mut mn, mut mx, mut sum, mut cnt) = (None::<Th>, f64::NEG_INFINITY, 0.0, 0usize);
        for p in prims {
            let Prim::Face(f) = p else { return Err("thicknessAt: face must name a face".into()) };
            for c in face_grid(&ctx, f, g).into_iter().flatten() {
                if let Some(t) = ray_thickness(&ctx, f, c.0, c.1) {
                    mx = mx.max(t.t);
                    sum += t.t;
                    cnt += 1;
                    if mn.as_ref().map_or(true, |m| t.t < m.t) {
                        mn = Some(t);
                    }
                }
            }
        }
        let Some(m) = mn else { return Err("thicknessAt: no thickness samples on that face".into()) };
        return Ok(Obj::new().put("min", th_json(&ctx, &m, m.t)).put("max", mx).put("mean", sum / cnt as f64).put("samples", cnt as f64).build());
    }
    Err("thicknessAt: point [x, y, z] or face id is required".into())
}

// ---------------------------------------------------------------- closestPoint

/// What the point `p` (on face `f`) sits on: vertex / edge / face interior.
fn locate(ctx: &Ctx, f: FaceId, p: Vec3) -> Json {
    let b = ctx.body;
    for e in b.face_edges(f) {
        let ed = b.edge(e);
        for v in [ed.v0, ed.v1] {
            if b.vertex(v).p.dist(p) <= 1e-7 {
                return Obj::new().put("kind", "vertex").put("id", format!("V{}", v.idx())).build();
            }
        }
    }
    for e in b.face_edges(f) {
        let ed = b.edge(e);
        if dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1 <= 1e-7 {
            return Obj::new().put("kind", "edge").put("id", ctx.edge_name(e)).put("face", b.face_id_str(f)).build();
        }
    }
    Obj::new().put("kind", "face").put("id", b.face_id_str(f)).build()
}

/// `{type:"closestPoint", tag, points:[[x,y,z]], on?:"body"|"face"|"edge"|"vertex", ids?:[..], k?}`.
fn closest_point_inner(body: &Body, msg: &Json) -> Result<Json, String> {
    let pts = msg.get("points").and_then(|p| p.as_arr()).ok_or("closestPoint: points [[x, y, z], ...] is required")?;
    if pts.len() > MAX_POINTS {
        return Err(format!("closestPoint: at most {MAX_POINTS} points per request"));
    }
    let on = msg.get_str("on").unwrap_or("body");
    if !["body", "face", "edge", "vertex"].contains(&on) {
        return Err("closestPoint: on must be 'body', 'face', 'edge' or 'vertex'".into());
    }
    let k = msg.get("k").and_then(|k| k.as_usize()).unwrap_or(1).clamp(1, 50);
    let ctx = Ctx::new(body);
    let ids: Option<Vec<Prim>> = match msg.get("ids") {
        Some(j) => Some(parse_entity(&ctx, j, "closestPoint: ids")?),
        None => None,
    };
    let universe: Vec<Prim> = match (&ids, on) {
        (Some(v), _) => v.clone(),
        (None, "face") | (None, "body") => body.face_ids().map(Prim::Face).collect(),
        (None, "edge") => ctx.keys.iter().flat_map(|(_, ch)| ch.iter().map(|&e| Prim::Edge(e))).collect(),
        _ => body.vertex_ids().map(Prim::Vtx).collect(),
    };
    let ent = Ent::new(&ctx, universe.clone());
    let mut out = vec![];
    for q in pts {
        let q = vec3_of(Some(q), "closestPoint: each point")?;
        if over() {
            out.push(Json::Null);
            continue;
        }
        let mut cands: Vec<(f64, Vec3, Prim)> = if k == 1 {
            // box-pruned search
            ent.nearest(q).map(|(d, x, i)| vec![(d, x, universe[i])]).unwrap_or_default()
        } else {
            universe.iter().filter_map(|&p| prim_nearest(&ctx, p, q).map(|(d, x)| (d, x, p))).collect()
        };
        cands.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut list = vec![];
        for (d, x, p) in cands.iter().take(k) {
            let mut o = Obj::new().put("distance", *d).put("point", v3(*x));
            o = match (on, p) {
                ("body", Prim::Face(f)) => o.put("entity", locate(&ctx, *f, *x)).put("face", ctx.body.face_id_str(*f)),
                (_, pr) => o.put("entity", prim_json(&ctx, *pr)),
            };
            if let Prim::Face(f) = p {
                let (u, v) = body.face(*f).surface.closest_param(*x);
                o = o.put("uv", Json::Arr(vec![Json::Num(u), Json::Num(v)]));
            }
            if let Prim::Edge(e) = p {
                let ed = body.edge(*e);
                o = o.put("t", ed.curve.closest_param(*x));
            }
            list.push(o.build());
        }
        if list.is_empty() {
            out.push(Json::Null);
        } else if k == 1 {
            let mut first = list.remove(0);
            if on == "body" {
                let loc = ctx.c.classify_point(q, TOL);
                if let Json::Obj(_) = first {
                    let d = first.get_f64("distance").unwrap_or(0.0);
                    let signed = match loc {
                        Loc::In => -d,
                        _ => d,
                    };
                    first = Obj::new().put("signedDistance", signed).put("inside", loc == Loc::In).put("nearest", first).build();
                }
            }
            out.push(first);
        } else {
            out.push(Json::Arr(list));
        }
    }
    Ok(Obj::new().put("on", on).put("results", Json::Arr(out)).build())
}

// ---------------------------------------------------------------- angles

/// Interior dihedral angle (radians, through the material) at parameter `t` of edge `e` between its two faces.
/// Returns `(interior, normal angle, kind)`.
fn dihedral(ctx: &Ctx, e: EdgeId, f1: FaceId, f2: FaceId, t: f64) -> Option<(f64, f64, &'static str)> {
    let b = ctx.body;
    let ed = b.edge(e);
    let p = ed.curve.eval(t);
    let tg = ed.curve.tangent(t);
    if tg.len() < 1e-12 {
        return None;
    }
    let tg = tg.normalize();
    let nrm = |f: FaceId, at: Vec3| {
        let (u, v) = b.face(f).surface.closest_param(at);
        b.outward_normal(f, u, v)
    };
    let (n1, n2) = (nrm(f1, p), nrm(f2, p));
    if n1.len() < 0.5 || n2.len() < 0.5 {
        return None;
    }
    let cosn = n1.dot(n2).clamp(-1.0, 1.0);
    let na = cosn.acos();
    if na < 0.5f64.to_radians() {
        return Some((std::f64::consts::PI, na, "smooth"));
    }
    // direction into face 1 away from the edge
    let w = n1.cross(tg);
    if w.len() < 1e-12 {
        return None;
    }
    let w = w.normalize();
    let i1 = *ctx.fidx.get(&f1)?;
    let delta = (ctx.diag * 1e-3).min(0.05 * b.edge_length(e).max(1e-9));
    let mut w1 = w;
    for s in [1.0, -1.0] {
        let q = p + w * (s * delta);
        let (u, v) = b.face(f1).surface.closest_param(q);
        if ctx.c.point_in_face(i1, u, v, TOL) == Where::In {
            w1 = w * s;
            break;
        }
    }
    let convex = n2.dot(w1) < 0.0;
    Some(if convex { (std::f64::consts::PI - na, na, "convex") } else { (std::f64::consts::PI + na, na, "concave") })
}

/// `{type:"angles", tag, edges?:"all"|[keys], faces?:[a, b], draft?:{pull, faces?, min?}}`.
fn angles_inner(body: &Body, msg: &Json) -> Result<Json, String> {
    let ctx = Ctx::new(body);
    let mut o = Obj::new();
    let mut any = false;
    if let Some(es) = msg.get("edges") {
        any = true;
        let prims: Vec<Prim> = match es {
            Json::Str(s) if s == "all" => ctx.keys.iter().map(|(_, ch)| Prim::Edge(ch[0])).collect(),
            other => parse_entity(&ctx, other, "angles: edges")?,
        };
        let mut out = vec![];
        for p in prims {
            if over() {
                break;
            }
            let Prim::Edge(e) = p else { return Err("angles: edges must name edges".into()) };
            let fs = body.edge_faces(e);
            if fs.len() != 2 {
                continue; // boundary, seam or non-manifold
            }
            let samples: Vec<_> = (1..=5)
                .filter_map(|k| {
                    let ed = body.edge(e);
                    dihedral(&ctx, e, fs[0], fs[1], ed.t0 + (ed.t1 - ed.t0) * k as f64 / 6.0)
                })
                .collect();
            let Some(&(mid, na, kind)) = samples.get(samples.len() / 2) else { continue };
            let (mn, mx) = samples.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, s| (a.0.min(s.0), a.1.max(s.0)));
            out.push(
                Obj::new()
                    .put("edge", ctx.edge_name(e))
                    .put("faces", Json::Arr(fs.iter().map(|&f| Json::Str(body.face_id_str(f))).collect()))
                    .put("angle", deg(mid))
                    .put("normalAngle", deg(na))
                    .put("type", kind)
                    .put("min", deg(mn))
                    .put("max", deg(mx))
                    .put("at", v3(body.edge_mid(e)))
                    .build(),
            );
        }
        o = o.put("edges", Json::Arr(out));
    }
    if let Some(fs) = msg.get("faces") {
        any = true;
        let a = fs.as_arr().filter(|a| a.len() == 2).ok_or("angles: faces must be [faceA, faceB]")?;
        let pa = parse_entity(&ctx, &a[0], "angles: faces[0]")?;
        let pb = parse_entity(&ctx, &a[1], "angles: faces[1]")?;
        let (Some(Prim::Face(fa)), Some(Prim::Face(fb))) = (pa.first().copied(), pb.first().copied()) else { return Err("angles: faces must name faces".into()) };
        let shared: Vec<EdgeId> = body.face_edges(fa).into_iter().filter(|e| body.edge_faces(*e).contains(&fb)).collect();
        let mut r = Obj::new();
        if let Some(&e) = shared.first() {
            let ed = body.edge(e);
            if let Some((ang, na, kind)) = dihedral(&ctx, e, fa, fb, 0.5 * (ed.t0 + ed.t1)) {
                r = r.put("angle", deg(ang)).put("normalAngle", deg(na)).put("type", kind).put("sharedEdge", ctx.edge_name(e)).put("at", v3(body.edge_mid(e)));
            }
        } else {
            let h = pair_min(&ctx, Prim::Face(fa), &ctx, Prim::Face(fb), 7).ok_or("angles: faces could not be measured")?;
            let nrm = |f: FaceId, at: Vec3| {
                let (u, v) = body.face(f).surface.closest_param(at);
                body.outward_normal(f, u, v)
            };
            let (n1, n2) = (nrm(fa, h.a), nrm(fb, h.b));
            let na = n1.dot(n2).clamp(-1.0, 1.0).acos();
            r = r.put("normalAngle", deg(na)).put("angle", deg(std::f64::consts::PI - na)).put("distance", h.d).put("pointA", v3(h.a)).put("pointB", v3(h.b)).put("parallel", na < 1e-6 || (std::f64::consts::PI - na) < 1e-6);
        }
        o = o.put("between", r.build());
    }
    if let Some(d) = msg.get("draft").or_else(|| msg.get("pull").map(|_| msg)) {
        any = true;
        let pull = vec3_of(d.get("pull").or_else(|| msg.get("pull")), "angles: draft.pull")?;
        if pull.len() < 1e-12 {
            return Err("angles: draft.pull must not be zero".into());
        }
        let pull = pull.normalize();
        let min = d.get_f64("min");
        let ids: Vec<FaceId> = match d.get("faces") {
            Some(Json::Str(s)) if s != "all" => parse_name(&ctx, s, "angles: draft.faces")?.into_iter().filter_map(|p| if let Prim::Face(f) = p { Some(f) } else { None }).collect(),
            Some(Json::Arr(a)) => parse_entity(&ctx, &Json::Arr(a.to_vec()), "angles: draft.faces")?.into_iter().filter_map(|p| if let Prim::Face(f) = p { Some(f) } else { None }).collect(),
            _ => body.face_ids().collect(),
        };
        let mut out = vec![];
        let mut viol = vec![];
        for f in ids {
            let (mut lo, mut hi, mut cnt) = (f64::INFINITY, f64::NEG_INFINITY, 0usize);
            for c in face_grid(&ctx, f, 5).into_iter().flatten() {
                let n = body.outward_normal(f, c.0, c.1);
                if n.len() < 0.5 {
                    continue;
                }
                let a = deg(n.normalize().dot(pull).clamp(-1.0, 1.0).asin());
                lo = lo.min(a);
                hi = hi.max(a);
                cnt += 1;
            }
            if cnt == 0 {
                continue;
            }
            let class = if hi < -0.01 || lo < -0.01 { "undercut" } else if hi.abs() < 0.01 && lo.abs() < 0.01 { "vertical" } else if lo > 89.99 || hi < -89.99 { "parting" } else { "ok" };
            let name = body.face_id_str(f);
            let r = Obj::new().put("face", name.clone()).put("draft", 0.5 * (lo + hi)).put("min", lo).put("max", hi).put("class", class).build();
            if let Some(m) = min {
                // a wall (not a parting face) whose draft is below the requested minimum
                if lo.abs() < 89.0 && hi.abs() < 89.0 && lo < m {
                    viol.push(Json::Str(name));
                }
            }
            out.push(r);
        }
        o = o.put("draft", Json::Arr(out));
        if min.is_some() {
            o = o.put("violations", Json::Arr(viol));
        }
    }
    if !any {
        return Err("angles: edges ('all' | [keys]), faces [a, b] or draft {pull} is required".into());
    }
    Ok(o.build())
}

// ---------------------------------------------------------------- clearance

/// What lies at `mid`, the middle of the shortest segment between two faces: material (`wall`), air (`gap`).
/// A midpoint on a boundary (the segment ends on a cap edge) is nudged towards the middle of the box.
fn between(ctx: &Ctx, mid: Vec3) -> &'static str {
    let verdict = |l: Loc| match l {
        Loc::In => Some("wall"),
        Loc::Out => Some("gap"),
        Loc::On => None,
    };
    if let Some(v) = verdict(ctx.c.classify_point(mid, TOL)) {
        return v;
    }
    let (lo, hi) = ctx.body.bbox();
    let to_centre = (lo + hi) * 0.5 - mid;
    let nudge = if to_centre.len() > 1e-12 { to_centre.normalize() } else { Vec3::Y };
    for k in [1e-4, 1e-3, 1e-2] {
        if let Some(v) = verdict(ctx.c.classify_point(mid + nudge * (ctx.diag * k), TOL)) {
            return v;
        }
    }
    "contact"
}

/// `{type:"clearance", tag, min, a?, b?, include?:"gap"|"wall"|"both", limit?}`: face pairs closer than `min`.
fn clearance_inner(body: &Body, msg: &Json) -> Result<Json, String> {
    let min = msg.get_f64("min").or_else(|| opt(msg, "min").and_then(|j| j.as_f64())).filter(|m| *m > 0.0 && m.is_finite()).ok_or("clearance: min (a positive distance) is required")?;
    let ctx = Ctx::new(body);
    let include = opt_s(msg, "include").unwrap_or("both");
    let limit = opt_n(msg, "limit", 200, 5000);
    let faces_of = |j: Option<&Json>, w: &str| -> Result<Vec<FaceId>, String> {
        match j {
            None => Ok(body.face_ids().collect()),
            Some(j) => Ok(parse_entity(&ctx, j, w)?.into_iter().filter_map(|p| if let Prim::Face(f) = p { Some(f) } else { None }).collect()),
        }
    };
    let fa = faces_of(msg.get("a"), "clearance: a")?;
    let fb = faces_of(msg.get("b"), "clearance: b")?;
    if fa.len().max(fb.len()) > 4000 {
        return Err("clearance: more than 4000 faces".into());
    }
    let verts = |f: FaceId| -> Vec<VertexId> {
        let mut v = vec![];
        for e in body.face_edges(f) {
            let ed = body.edge(e);
            v.push(ed.v0);
            v.push(ed.v1);
        }
        v
    };
    let vs: HashMap<FaceId, Vec<VertexId>> = fa.iter().chain(fb.iter()).map(|&f| (f, verts(f))).collect();
    let boxes: HashMap<FaceId, Aabb> = fa.iter().chain(fb.iter()).map(|&f| (f, face_box(body, f))).collect();
    let same_set = msg.get("a").is_none() && msg.get("b").is_none();
    let (mut tested, mut hits) = (0usize, vec![]);
    let max_pairs = opt_n(msg, "maxPairs", 4000, 200000);
    // candidate pairs: box distance under `min`, not touching; closest boxes first so a cut run keeps the worst
    let mut seen: std::collections::HashSet<(usize, usize)> = Default::default();
    let mut cand: Vec<(f64, FaceId, FaceId)> = vec![];
    for &a in &fa {
        for &b in &fb {
            if a == b || (same_set && a.idx() > b.idx()) {
                continue;
            }
            let key = (a.idx().min(b.idx()), a.idx().max(b.idx()));
            let bd = box_dist(&boxes[&a], &boxes[&b]);
            if bd >= min || !seen.insert(key) || vs[&a].iter().any(|v| vs[&b].contains(v)) {
                continue;
            }
            cand.push((bd, a, b));
        }
    }
    cand.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
    if cand.len() > max_pairs {
        cand.truncate(max_pairs);
        mark_truncated();
    }
    for (_, a, b) in cand {
        if over() {
            break;
        }
        // the display meshes bound the distance; one refinement from the closest mesh pair gives the exact value
        let h = match mesh_pair(&ctx, Prim::Face(a), &ctx, Prim::Face(b)) {
            Some((md, pa, slack)) => {
                if md - 2.0 * slack >= min {
                    continue;
                }
                tested += 1;
                match refine_n(&ctx, Prim::Face(a), &ctx, Prim::Face(b), pa, 60) {
                    Some(h) => h,
                    None => continue,
                }
            }
            None => {
                tested += 1;
                match pair_min(&ctx, Prim::Face(a), &ctx, Prim::Face(b), 6) {
                    Some(h) => h,
                    None => continue,
                }
            }
        };
        if h.d >= min {
            continue;
        }
        let mid = (h.a + h.b) * 0.5;
        let kind = if h.d < 1e-9 { "contact" } else { between(&ctx, mid) };
        if (include == "gap" && kind == "wall") || (include == "wall" && kind == "gap") {
            continue;
        }
        hits.push((h, a, b, kind));
    }
    hits.sort_by(|x, y| x.0.d.partial_cmp(&y.0.d).unwrap_or(std::cmp::Ordering::Equal));
    let total = hits.len();
    if total > limit {
        mark_truncated();
    }
    let pairs = hits
        .iter()
        .take(limit)
        .map(|(h, a, b, k)| {
            Obj::new()
                .put("a", body.face_id_str(*a))
                .put("b", body.face_id_str(*b))
                .put("distance", h.d)
                .put("kind", *k)
                .put("pointA", v3(h.a))
                .put("pointB", v3(h.b))
                .build()
        })
        .collect();
    Ok(Obj::new().put("min", min).put("count", total as f64).put("pairsTested", tested as f64).put("listedAll", total <= limit).put("pairs", Json::Arr(pairs)).build())
}

// ---------------------------------------------------------------- symmetry

/// `{type:"symmetry", tag, planes?:[{origin, normal}], opts?:{samples, tol}}`.
fn symmetry_inner(body: &Body, msg: &Json) -> Result<Json, String> {
    let ctx = Ctx::new(body);
    let mp = ck_topo::mass::mass_props(body);
    let com = mp.com;
    let (lo, hi) = body.bbox();
    let centre = (lo + hi) * 0.5;
    let budget = opt_n(msg, "samples", 600, 100000);
    let tol = opt_f(msg, "tol", 1e-4 * ctx.diag);
    let mut cands: Vec<(Vec3, Vec3, String)> = vec![];
    let add = |o: Vec3, n: Vec3, why: &str, c: &mut Vec<(Vec3, Vec3, String)>| {
        if n.len() < 1e-9 {
            return;
        }
        let n = n.normalize();
        if c.iter().any(|(o2, n2, _)| n2.dot(n).abs() > 1.0 - 1e-9 && (n.dot(o - *o2)).abs() < 1e-9 * ctx.diag) {
            return;
        }
        c.push((o, n, why.to_string()));
    };
    if let Some(ps) = msg.get("planes").and_then(|p| p.as_arr()) {
        for p in ps {
            let o = vec3_of(p.get("origin"), "symmetry: plane.origin")?;
            let n = vec3_of(p.get("normal"), "symmetry: plane.normal")?;
            if n.len() < 1e-12 {
                return Err("symmetry: plane.normal must not be zero".into());
            }
            add(o, n, "given", &mut cands);
        }
    } else {
        for (n, nm) in [(Vec3::X, "x"), (Vec3::Y, "y"), (Vec3::Z, "z")] {
            add(com, n, &format!("axis {nm} through the centre of mass"), &mut cands);
        }
        for (i, a) in mp.axes.iter().enumerate() {
            add(com, *a, &format!("principal axis {i}"), &mut cands);
        }
        for (n, nm) in [(Vec3::X, "x"), (Vec3::Y, "y"), (Vec3::Z, "z")] {
            add(centre, n, &format!("axis {nm} through the bbox centre"), &mut cands);
        }
    }
    let faces: Vec<FaceId> = body.face_ids().collect();
    let (pts, _) = body_samples(&ctx, &faces, budget);
    let all = Ent::new(&ctx, faces.iter().map(|&f| Prim::Face(f)).collect());
    let full = matches!(opt(msg, "full"), Some(Json::Bool(true)));
    let mut res: Vec<(f64, Json)> = vec![];
    for (o, n, why) in cands {
        let (mut mx, mut s2, mut cnt, mut worst) = (0.0f64, 0.0, 0usize, Vec3::ZERO);
        let mut exact = true;
        for oi in stride_order(pts.len()) {
            if over() {
                break;
            }
            // one point farther than `tol` from the body disproves the plane: stop after a few dozen points, the
            // reported deviation is then a lower bound (`exact:false`)
            if mx > tol && cnt >= 40 && !full {
                exact = false;
                break;
            }
            let p = &pts[oi].0;
            let q = *p - n * (2.0 * n.dot(*p - o));
            if let Some((d, _, _)) = all.nearest(q) {
                s2 += d * d;
                cnt += 1;
                if d > mx {
                    (mx, worst) = (d, *p);
                }
            }
        }
        let rms = (s2 / cnt.max(1) as f64).sqrt();
        res.push((
            mx,
            Obj::new()
                .put("origin", v3(o))
                .put("normal", v3(n))
                .put("basis", why)
                .put("maxDeviation", mx)
                .put("rmsDeviation", rms)
                .put("symmetric", mx <= tol)
                .put("exact", exact)
                .put("worstPoint", v3(worst))
                .build(),
        ));
    }
    res.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let best = res.first().map(|r| r.1.clone());
    let syms: Vec<Json> = res.iter().filter(|r| r.0 <= tol).map(|r| r.1.clone()).collect();
    Ok(Obj::new()
        .put("tol", tol)
        .put("samples", pts.len() as f64)
        .put("symmetric", Json::Arr(syms))
        .put("best", best.unwrap_or(Json::Null))
        .put("planes", Json::Arr(res.into_iter().map(|r| r.1).collect()))
        .build())
}

#[cfg(test)]
mod tests;
