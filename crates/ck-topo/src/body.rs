//! The B-rep data model: Body { lumps -> shells -> faces -> loops of
//! coedges -> edges -> vertices }, arena/index based.
//!
//! Conventions (used by every module in this crate):
//! - A face carries a `Surface` + `sense`: outward normal = natural normal
//!   (Su × Sv) when `sense`, its negation otherwise.
//! - Every loop is oriented counter-clockwise about the face's OUTWARD
//!   normal (material on the left when walking a coedge). The first loop of
//!   a face is the outer loop.
//! - A coedge traverses its edge reversed when `rev`. Its optional pcurve is
//!   parameterised by the EDGE parameter (same t as `edge.curve`), so
//!   `pcurve.eval(t)` is the (u,v) of `edge.curve.eval(t)` on the face.
//! - A seam edge (periodic faces) has both of its coedges in the same face;
//!   seams are not exported on the wire.
//! - Arenas never shrink; deleted entities are flagged `alive = false`
//!   (edit.rs). Ids stay stable, which keeps the wire edge `#N` order equal to
//!   creation order.
//!
//! Wire naming grammar (pinned against oracle regens, see README):
//! face id  = `<feature>.<role>[~<region>]<instance suffix>`
//!   role   = `S.<entity>` side face swept from sketch entity,
//!            `C<k>` cap (k=0 low/start cap, k=1 high/end cap),
//!            `B<k>` blend/chamfer face k of the feature
//!   region = 1-based profile region index, only when the profile has >1
//!            region (`F2.C0~1`, `F2.C0~2`)
//!   suffix = `@<pattern feature>.<instance>` and/or `@<insertPart>.<n>`
//!            chain (`F8.S.h1@F9.1@F1.1`)
//! meta     = {feature, entity: `<entity><suffix>`} (entity only for `S.`)
//! edge key = `<id a>|<id b>#<n>`: the two face ids sorted bytewise, n = index
//!            of the edge in the exported edge list (seams skipped).

use ck_core::xform::Xform;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_json::{Json, Obj};

macro_rules! id_type {
    ($($n:ident),*) => {$(
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $n(pub u32);
        impl $n {
            #[inline]
            pub fn idx(self) -> usize {
                self.0 as usize
            }
        }
    )*};
}
id_type!(VertexId, EdgeId, CoedgeId, LoopId, FaceId, ShellId, LumpId);

#[derive(Debug, Clone, PartialEq)]
pub struct Vertex {
    pub p: Vec3,
    /// Tolerance (mm): how far the vertex may lie from its incident edge
    /// curves. 0 = exact. Set by the healing pass, never by modelling ops.
    pub tol: f64,
    pub alive: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    pub curve: Curve,
    /// Trim range on `curve`, t0 at v0, t1 at v1 (t0 < t1).
    pub t0: f64,
    pub t1: f64,
    pub v0: VertexId,
    pub v1: VertexId,
    pub coedges: Vec<CoedgeId>,
    /// Tolerance (mm): how far the curve may lie from the faces it bounds
    /// (the exporter's fitted curves and free-form faces disagree by a few
    /// 1e-3). 0 = exact. Set by the healing pass; `ck_check::validate`
    /// accepts a deviation up to `max(model tol, tol)`.
    pub tol: f64,
    pub alive: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Coedge {
    pub edge: EdgeId,
    pub rev: bool,
    pub lp: LoopId,
    pub pcurve: Option<Curve2>,
    pub alive: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Loop {
    pub face: FaceId,
    pub coedges: Vec<CoedgeId>,
    pub alive: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Face {
    pub surface: Surface,
    pub sense: bool,
    pub loops: Vec<LoopId>,
    pub shell: ShellId,
    pub name: FaceName,
    /// Face ids of other bodies' faces that merged into this one (coincident
    /// union); `find_face` resolves them after an exact match fails, so a
    /// later feature still finds the name its document refers to.
    pub absorbed: Vec<String>,
    pub alive: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Shell {
    pub faces: Vec<FaceId>,
    pub lump: LumpId,
    pub alive: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Lump {
    pub shells: Vec<ShellId>,
    pub alive: bool,
    /// Document-level body name (`body` field of the feature that made the lump), if it has one.
    pub name: Option<String>,
}

// ---------------------------------------------------------------- naming

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Role {
    /// `S.<entity>`
    Side(String),
    /// `C<k>`
    Cap(u32),
    /// `B<k>`
    Blend(u32),
    /// Anything else, written verbatim after `<feature>.`
    Other(String),
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct FaceName {
    pub feature: String,
    pub role: Role,
    /// 1-based profile region (`~k`), only for multi-region profiles.
    pub region: Option<u32>,
    /// Instance chain incl. leading `@`s, e.g. `@F9.1@F1.1` (may be empty).
    pub suffix: String,
}

impl FaceName {
    pub fn side(feature: &str, entity: &str) -> FaceName {
        FaceName { feature: feature.into(), role: Role::Side(entity.into()), region: None, suffix: String::new() }
    }
    pub fn cap(feature: &str, k: u32, region: Option<u32>) -> FaceName {
        FaceName { feature: feature.into(), role: Role::Cap(k), region, suffix: String::new() }
    }
    pub fn blend(feature: &str, k: u32) -> FaceName {
        FaceName { feature: feature.into(), role: Role::Blend(k), region: None, suffix: String::new() }
    }
    pub fn other(feature: &str, role: &str) -> FaceName {
        FaceName { feature: feature.into(), role: Role::Other(role.into()), region: None, suffix: String::new() }
    }

    /// Wire face id.
    pub fn id(&self) -> String {
        let mut s = String::with_capacity(24);
        s.push_str(&self.feature);
        s.push('.');
        match &self.role {
            Role::Side(e) => {
                s.push_str("S.");
                s.push_str(e);
            }
            Role::Cap(k) => {
                s.push('C');
                s.push_str(&k.to_string());
            }
            Role::Blend(k) => {
                s.push('B');
                s.push_str(&k.to_string());
            }
            Role::Other(o) => s.push_str(o),
        }
        if let Some(r) = self.region {
            // `~k` sits before the instance suffix (probe: insertPart of a
            // two-region extrude -> `F2.C0~1@F1.1`).
            s.push('~');
            s.push_str(&r.to_string());
        }
        s.push_str(&self.suffix);
        s
    }

    /// `meta.entity` (Side faces only): entity + instance suffix.
    pub fn meta_entity(&self) -> Option<String> {
        match &self.role {
            Role::Side(e) => Some(format!("{}{}", e, self.suffix)),
            _ => None,
        }
    }

    /// Append one instance level (`F9.1` -> suffix `@F9.1`).
    pub fn push_instance(&mut self, inst: &str) {
        self.suffix.push('@');
        self.suffix.push_str(inst);
    }

    /// Parse a wire face id (inverse of [`FaceName::id`]).
    pub fn parse(id: &str) -> Option<FaceName> {
        let dot = id.find('.')?;
        let feature = &id[..dot];
        let rest = &id[dot + 1..];
        let (body, suffix) = match rest.find('@') {
            Some(i) => (&rest[..i], &rest[i..]),
            None => (rest, ""),
        };
        let (body, region) = match body.rfind('~') {
            Some(i) if !body.starts_with("S.") => (&body[..i], body[i + 1..].parse::<u32>().ok()),
            _ => (body, None),
        };
        let num = |s: &str| -> Option<u32> {
            if !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()) {
                s.parse().ok()
            } else {
                None
            }
        };
        let role = if let Some(e) = body.strip_prefix("S.") {
            Role::Side(e.into())
        } else if let Some(k) = body.strip_prefix('C').and_then(num) {
            Role::Cap(k)
        } else if let Some(k) = body.strip_prefix('B').and_then(num) {
            Role::Blend(k)
        } else {
            Role::Other(body.into())
        };
        Some(FaceName { feature: feature.into(), role, region, suffix: suffix.into() })
    }
}

/// Wire edge key: the two face ids sorted bytewise, `|`-joined, `#seq`.
pub fn edge_key(a: &str, b: &str, seq: usize) -> String {
    let (x, y) = if a <= b { (a, b) } else { (b, a) };
    format!("{}|{}#{}", x, y, seq)
}

// ---------------------------------------------------------------- body

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Body {
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Edge>,
    pub coedges: Vec<Coedge>,
    pub loops: Vec<Loop>,
    pub faces: Vec<Face>,
    pub shells: Vec<Shell>,
    pub lumps: Vec<Lump>,
}

impl Body {
    pub fn new() -> Body {
        Body::default()
    }

    /// Estimated heap + inline bytes of this body (arenas, surface and curve
    /// control data, face names). Used to bound the regen state cache; not
    /// exact, but proportional to the real allocation.
    pub fn est_bytes(&self) -> usize {
        use std::mem::size_of;
        fn bs_curve<P>(c: &ck_geom::nurbs::bspline::BSplineCurve<P>) -> usize
        where
            P: ck_geom::Pt,
        {
            c.knots.len() * 8 + c.ctrl.len() * size_of::<P>() + c.weights.as_ref().map_or(0, |w| w.len() * 8)
        }
        fn curve(c: &Curve) -> usize {
            match c {
                Curve::BSpline(b) => bs_curve(b),
                _ => 0,
            }
        }
        fn curve2(c: &Curve2) -> usize {
            match c {
                Curve2::BSpline(b) => bs_curve(b),
                _ => 0,
            }
        }
        fn surf(s: &Surface) -> usize {
            match s {
                Surface::BSpline(b) => {
                    size_of::<ck_geom::nurbs::bsurf::BSplineSurface>()
                        + (b.knots_u.len() + b.knots_v.len()) * 8
                        + b.ctrl.len() * size_of::<Vec3>()
                        + b.weights.as_ref().map_or(0, |w| w.len() * 8)
                }
                Surface::Revolution { profile, .. } | Surface::Extrusion { profile, .. } | Surface::Screw { profile, .. } => {
                    size_of::<Curve>() + curve(profile)
                }
                Surface::Offset { base, .. } => size_of::<Surface>() + surf(base),
                _ => 0,
            }
        }
        let mut n = size_of::<Body>();
        n += self.vertices.len() * size_of::<Vertex>();
        n += self.edges.iter().map(|e| size_of::<Edge>() + e.coedges.len() * size_of::<CoedgeId>() + curve(&e.curve)).sum::<usize>();
        n += self.coedges.iter().map(|c| size_of::<Coedge>() + c.pcurve.as_ref().map_or(0, curve2)).sum::<usize>();
        n += self.loops.iter().map(|l| size_of::<Loop>() + l.coedges.len() * size_of::<CoedgeId>()).sum::<usize>();
        n += self
            .faces
            .iter()
            .map(|f| {
                size_of::<Face>()
                    + surf(&f.surface)
                    + f.loops.len() * size_of::<LoopId>()
                    + f.name.feature.len()
                    + f.name.suffix.len()
                    + f.absorbed.iter().map(|a| a.len()).sum::<usize>()
                    + match &f.name.role {
                        Role::Side(s) | Role::Other(s) => s.len(),
                        _ => 0,
                    }
            })
            .sum::<usize>();
        n += self.shells.iter().map(|s| size_of::<Shell>() + s.faces.len() * size_of::<FaceId>()).sum::<usize>();
        n += self.lumps.iter().map(|l| size_of::<Lump>() + l.shells.len() * size_of::<ShellId>()).sum::<usize>();
        n
    }

    // ---- accessors
    pub fn vertex(&self, v: VertexId) -> &Vertex {
        &self.vertices[v.idx()]
    }
    pub fn edge(&self, e: EdgeId) -> &Edge {
        &self.edges[e.idx()]
    }
    pub fn coedge(&self, c: CoedgeId) -> &Coedge {
        &self.coedges[c.idx()]
    }
    pub fn lp(&self, l: LoopId) -> &Loop {
        &self.loops[l.idx()]
    }
    pub fn face(&self, f: FaceId) -> &Face {
        &self.faces[f.idx()]
    }
    pub fn shell(&self, s: ShellId) -> &Shell {
        &self.shells[s.idx()]
    }
    pub fn lump(&self, l: LumpId) -> &Lump {
        &self.lumps[l.idx()]
    }

    // ---- live iterators
    pub fn face_ids(&self) -> impl Iterator<Item = FaceId> + '_ {
        (0..self.faces.len()).filter(|&i| self.faces[i].alive).map(|i| FaceId(i as u32))
    }
    pub fn edge_ids(&self) -> impl Iterator<Item = EdgeId> + '_ {
        (0..self.edges.len()).filter(|&i| self.edges[i].alive).map(|i| EdgeId(i as u32))
    }
    pub fn vertex_ids(&self) -> impl Iterator<Item = VertexId> + '_ {
        (0..self.vertices.len()).filter(|&i| self.vertices[i].alive).map(|i| VertexId(i as u32))
    }
    pub fn lump_ids(&self) -> impl Iterator<Item = LumpId> + '_ {
        (0..self.lumps.len()).filter(|&i| self.lumps[i].alive).map(|i| LumpId(i as u32))
    }
    pub fn shell_ids(&self) -> impl Iterator<Item = ShellId> + '_ {
        (0..self.shells.len()).filter(|&i| self.shells[i].alive).map(|i| ShellId(i as u32))
    }
    pub fn num_faces(&self) -> usize {
        self.face_ids().count()
    }
    pub fn num_lumps(&self) -> usize {
        self.lump_ids().count()
    }

    // ---- construction
    pub fn add_lump(&mut self) -> LumpId {
        self.lumps.push(Lump { shells: vec![], alive: true, name: None });
        LumpId(self.lumps.len() as u32 - 1)
    }
    pub fn add_shell(&mut self, lump: LumpId) -> ShellId {
        self.shells.push(Shell { faces: vec![], lump, alive: true });
        let s = ShellId(self.shells.len() as u32 - 1);
        self.lumps[lump.idx()].shells.push(s);
        s
    }
    pub fn add_vertex(&mut self, p: Vec3) -> VertexId {
        self.vertices.push(Vertex { p, tol: 0.0, alive: true });
        VertexId(self.vertices.len() as u32 - 1)
    }
    pub fn add_edge(&mut self, curve: Curve, t0: f64, t1: f64, v0: VertexId, v1: VertexId) -> EdgeId {
        self.edges.push(Edge { curve, t0, t1, v0, v1, coedges: vec![], tol: 0.0, alive: true });
        EdgeId(self.edges.len() as u32 - 1)
    }
    pub fn add_face(&mut self, shell: ShellId, surface: Surface, sense: bool, name: FaceName) -> FaceId {
        self.faces.push(Face { surface, sense, loops: vec![], shell, name, absorbed: vec![], alive: true });
        let f = FaceId(self.faces.len() as u32 - 1);
        self.shells[shell.idx()].faces.push(f);
        f
    }
    pub fn add_loop(&mut self, face: FaceId) -> LoopId {
        self.loops.push(Loop { face, coedges: vec![], alive: true });
        let l = LoopId(self.loops.len() as u32 - 1);
        self.faces[face.idx()].loops.push(l);
        l
    }
    /// Append a coedge to the end of loop `lp`.
    pub fn add_coedge(&mut self, lp: LoopId, edge: EdgeId, rev: bool, pcurve: Option<Curve2>) -> CoedgeId {
        self.coedges.push(Coedge { edge, rev, lp, pcurve, alive: true });
        let c = CoedgeId(self.coedges.len() as u32 - 1);
        self.loops[lp.idx()].coedges.push(c);
        self.edges[edge.idx()].coedges.push(c);
        c
    }

    // ---- topology queries
    /// Live coedges of an edge.
    pub fn edge_coedges(&self, e: EdgeId) -> Vec<CoedgeId> {
        self.edges[e.idx()].coedges.iter().copied().filter(|c| self.coedges[c.idx()].alive).collect()
    }
    pub fn coedge_face(&self, c: CoedgeId) -> FaceId {
        self.loops[self.coedges[c.idx()].lp.idx()].face
    }
    /// Distinct faces using edge `e` (coedge order).
    pub fn edge_faces(&self, e: EdgeId) -> Vec<FaceId> {
        let mut out: Vec<FaceId> = vec![];
        for c in self.edge_coedges(e) {
            let f = self.coedge_face(c);
            if !out.contains(&f) {
                out.push(f);
            }
        }
        out
    }
    /// Seam: two (or more) coedges, all on one face.
    pub fn is_seam(&self, e: EdgeId) -> bool {
        let cs = self.edge_coedges(e);
        cs.len() >= 2 && self.edge_faces(e).len() == 1
    }
    /// Coedge start vertex (in traversal direction).
    pub fn coedge_start(&self, c: CoedgeId) -> VertexId {
        let ce = &self.coedges[c.idx()];
        let e = &self.edges[ce.edge.idx()];
        if ce.rev {
            e.v1
        } else {
            e.v0
        }
    }
    pub fn coedge_end(&self, c: CoedgeId) -> VertexId {
        let ce = &self.coedges[c.idx()];
        let e = &self.edges[ce.edge.idx()];
        if ce.rev {
            e.v0
        } else {
            e.v1
        }
    }
    /// Edge parameters at the coedge's (start, end).
    pub fn coedge_params(&self, c: CoedgeId) -> (f64, f64) {
        let ce = &self.coedges[c.idx()];
        let e = &self.edges[ce.edge.idx()];
        if ce.rev {
            (e.t1, e.t0)
        } else {
            (e.t0, e.t1)
        }
    }
    /// The other coedge of a manifold edge (None for open / non-manifold).
    pub fn partner(&self, c: CoedgeId) -> Option<CoedgeId> {
        let cs = self.edge_coedges(self.coedges[c.idx()].edge);
        if cs.len() == 2 {
            Some(if cs[0] == c { cs[1] } else { cs[0] })
        } else {
            None
        }
    }
    pub fn face_edges(&self, f: FaceId) -> Vec<EdgeId> {
        let mut out = vec![];
        let n: usize = self.faces[f.idx()].loops.iter().map(|&l| self.loops[l.idx()].coedges.len()).sum();
        // big faces (gear caps, pattern lids): dedupe through a set, not a scan
        let mut seen = ck_core::fxhash::FxHashSet::default();
        for &l in &self.faces[f.idx()].loops {
            for &c in &self.loops[l.idx()].coedges {
                let e = self.coedges[c.idx()].edge;
                if n > 16 {
                    if seen.insert(e) {
                        out.push(e);
                    }
                } else if !out.contains(&e) {
                    out.push(e);
                }
            }
        }
        out
    }
    /// Faces sharing an edge with `f`.
    pub fn face_neighbors(&self, f: FaceId) -> Vec<FaceId> {
        let mut out = vec![];
        for e in self.face_edges(f) {
            for g in self.edge_faces(e) {
                if g != f && !out.contains(&g) {
                    out.push(g);
                }
            }
        }
        out
    }
    pub fn face_lump(&self, f: FaceId) -> LumpId {
        self.shells[self.faces[f.idx()].shell.idx()].lump
    }
    /// Wire lump index (dense rank among live lumps).
    pub fn lump_index(&self, l: LumpId) -> usize {
        self.lump_ids().position(|x| x == l).unwrap_or(0)
    }
    pub fn find_face(&self, id: &str) -> Option<FaceId> {
        self.face_ids().find(|&f| self.faces[f.idx()].name.id() == id).or_else(|| {
            self.face_ids().find(|&f| self.faces[f.idx()].absorbed.iter().any(|a| a == id))
        })
    }
    pub fn face_id_str(&self, f: FaceId) -> String {
        self.faces[f.idx()].name.id()
    }

    // ---- geometry queries
    /// Outward unit normal of face `f` at (u, v).
    pub fn outward_normal(&self, f: FaceId, u: f64, v: f64) -> Vec3 {
        let face = &self.faces[f.idx()];
        let n = face.surface.normal(u, v);
        if face.sense {
            n
        } else {
            -n
        }
    }
    /// Edge midpoint (curve at mid parameter) — wire `mid`.
    pub fn edge_mid(&self, e: EdgeId) -> Vec3 {
        let e = &self.edges[e.idx()];
        e.curve.eval(0.5 * (e.t0 + e.t1))
    }
    pub fn edge_length(&self, e: EdgeId) -> f64 {
        let e = &self.edges[e.idx()];
        e.curve.length_between(e.t0, e.t1)
    }
    /// (u,v) of coedge `c` at edge parameter `t`: pcurve if present, else
    /// projection of the 3D point (`near` disambiguates periodic params).
    pub fn coedge_uv(&self, c: CoedgeId, t: f64, near: Option<Vec2>) -> Vec2 {
        self.coedge_uv_fp(c, t, near, None)
    }

    /// [`Self::coedge_uv`] with the face's B-spline surface hash (`ck_geom::nurbs::bsurf::BSplineSurface::fingerprint`)
    /// taken by the caller: a loop of samples pays the pass over the control net once, not once per point. The
    /// same answers, bit for bit (`None` = taken per call).
    pub fn coedge_uv_fp(&self, c: CoedgeId, t: f64, near: Option<Vec2>, fp: Option<u64>) -> Vec2 {
        let ce = &self.coedges[c.idx()];
        if let Some(pc) = &ce.pcurve {
            return pc.eval(t);
        }
        let face = &self.faces[self.coedge_face(c).idx()];
        let p = self.edges[ce.edge.idx()].curve.eval(t);
        let (mut u, mut v) = match (&face.surface, fp) {
            (ck_geom::surface::Surface::BSpline(bs), Some(fp)) => bs.closest_param_fp(fp, p),
            _ => face.surface.closest_param(p),
        };
        if let Some(n) = near {
            let (pu, pv) = face.surface.periods();
            if let Some(per) = pu {
                u = ck_geom::pcurve::unwrap_near(u, n.x, per);
            }
            if let Some(per) = pv {
                v = ck_geom::pcurve::unwrap_near(v, n.y, per);
            }
        }
        Vec2::new(u, v)
    }

    /// Axis-aligned bounds of vertices and sampled edges (exact enough for
    /// tolerancing; the wire `box` comes from the mesh in ck-tess).
    pub fn bbox(&self) -> (Vec3, Vec3) {
        let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = -lo;
        for v in self.vertex_ids() {
            lo = lo.min(self.vertices[v.idx()].p);
            hi = hi.max(self.vertices[v.idx()].p);
        }
        for e in self.edge_ids() {
            let ed = &self.edges[e.idx()];
            if matches!(ed.curve, Curve::Line { .. }) {
                continue;
            }
            let (a, b) = sampled_box(&ed.curve, ed.t0, ed.t1);
            lo = lo.min(a);
            hi = hi.max(b);
        }
        (lo, hi)
    }

    /// Box of a face from its boundary edges' control points: lines by
    /// their ends, splines by the control net of the trimmed piece.
    /// GUESS for conics (sampled arc). This is the key the oracle orders
    /// `~k` pieces of one face name by (see [`Body::number_split_faces`]).
    pub fn face_ctrl_box(&self, f: FaceId) -> (Vec3, Vec3) {
        let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = -lo;
        for e in self.face_edges(f) {
            let ed = &self.edges[e.idx()];
            let pts = match &ed.curve {
                Curve::Line { .. } => vec![self.vertices[ed.v0.idx()].p, self.vertices[ed.v1.idx()].p],
                Curve::BSpline(c) => clamped_net(c, ed.t0.min(ed.t1), ed.t0.max(ed.t1)),
                c => c.sample(ed.t0, ed.t1, 64),
            };
            for p in pts {
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
        (lo, hi)
    }

    /// Give `~k` suffixes to live faces that would otherwise share one id
    /// (one sketch entity split into several side faces, e.g. a closed
    /// spline profile cut at half length): k = rank of the face's
    /// [`Body::face_ctrl_box`] centre, compared (x, y, z) lexicographically
    /// in world axes. Oracle probes: closed-spline extrudes (60 random
    /// shapes per plane + symmetric ones on front/top/right) name `S.e1~1`
    /// the piece with the smaller control-box centre x (the curve's own
    /// box or mid point get 2..9 of 60 wrong), ties broken by y then z;
    /// near-ties of 4e-16 in y fall through to z, so the compare has a
    /// tolerance; the loft probe (`~1..~4` around a circle→square loft)
    /// fits the same rule. Faces that already carry a region keep it.
    pub fn number_split_faces(&mut self) {
        let mut groups: std::collections::BTreeMap<String, Vec<FaceId>> = Default::default();
        for f in self.face_ids() {
            if self.faces[f.idx()].name.region.is_none() {
                groups.entry(self.face_id_str(f)).or_default().push(f);
            }
        }
        for fs in groups.into_values().filter(|g| g.len() > 1) {
            let mut keyed: Vec<(Vec3, FaceId)> = fs
                .into_iter()
                .map(|f| {
                    let (lo, hi) = self.face_ctrl_box(f);
                    ((lo + hi) * 0.5, f)
                })
                .collect();
            // GUESS: tie tolerance 1e-9 (pinned only to lie in [4e-16, 0.05))
            let cmp = |p: f64, q: f64| if (p - q).abs() <= 1e-9 { std::cmp::Ordering::Equal } else { p.total_cmp(&q) };
            keyed.sort_by(|a, b| {
                let (p, q) = (a.0, b.0);
                cmp(p.x, q.x).then(cmp(p.y, q.y)).then(cmp(p.z, q.z))
            });
            for (k, (_, f)) in keyed.into_iter().enumerate() {
                self.faces[f.idx()].name.region = Some(k as u32 + 1);
            }
        }
    }

    /// Edge between two pieces of one split face (same feature / role /
    /// suffix, different `~k`) lying on the same surface: the oracle keeps
    /// it in the topology (it takes a wire number and is written to STEP)
    /// but leaves it out of `body.edges` (closed-spline extrude: rails #0,
    /// #1 between `S.e1~2` and `S.e1~1` are missing from the reply).
    pub fn is_split_rail(&self, e: EdgeId) -> bool {
        let fs = self.edge_faces(e);
        if fs.len() != 2 {
            return false;
        }
        let (a, b) = (&self.faces[fs[0].idx()], &self.faces[fs[1].idx()]);
        a.name.region != b.name.region
            && a.name.feature == b.name.feature
            && a.name.role == b.name.role
            && a.name.suffix == b.name.suffix
            && a.surface == b.surface
    }

    // ---- whole-body ops
    /// Append all of `other` (ids are offset); returns the new lump ids.
    pub fn append(&mut self, other: &Body) -> Vec<LumpId> {
        let (ov, oe, oc, ol, of, os, olu) = (
            self.vertices.len() as u32,
            self.edges.len() as u32,
            self.coedges.len() as u32,
            self.loops.len() as u32,
            self.faces.len() as u32,
            self.shells.len() as u32,
            self.lumps.len() as u32,
        );
        self.vertices.extend(other.vertices.iter().cloned());
        for e in &other.edges {
            let mut e = e.clone();
            e.v0 = VertexId(e.v0.0 + ov);
            e.v1 = VertexId(e.v1.0 + ov);
            e.coedges.iter_mut().for_each(|c| c.0 += oc);
            self.edges.push(e);
        }
        for c in &other.coedges {
            let mut c = c.clone();
            c.edge = EdgeId(c.edge.0 + oe);
            c.lp = LoopId(c.lp.0 + ol);
            self.coedges.push(c);
        }
        for l in &other.loops {
            let mut l = l.clone();
            l.face = FaceId(l.face.0 + of);
            l.coedges.iter_mut().for_each(|c| c.0 += oc);
            self.loops.push(l);
        }
        for f in &other.faces {
            let mut f = f.clone();
            f.shell = ShellId(f.shell.0 + os);
            f.loops.iter_mut().for_each(|l| l.0 += ol);
            self.faces.push(f);
        }
        for s in &other.shells {
            let mut s = s.clone();
            s.lump = LumpId(s.lump.0 + olu);
            s.faces.iter_mut().for_each(|f| f.0 += of);
            self.shells.push(s);
        }
        let mut out = vec![];
        for (i, l) in other.lumps.iter().enumerate() {
            let mut l = l.clone();
            l.shells.iter_mut().for_each(|s| s.0 += os);
            if l.alive {
                out.push(LumpId(olu + i as u32));
            }
            self.lumps.push(l);
        }
        out
    }

    /// Append `@inst` to every face name (pattern instance / insertPart).
    pub fn push_instance(&mut self, inst: &str) {
        for f in &mut self.faces {
            f.name.push_instance(inst);
            for a in &mut f.absorbed {
                a.push('@');
                a.push_str(inst);
            }
        }
    }

    /// Apply a RIGID transform (rotation + translation). Mirrors are not
    /// supported here (they flip frame handedness and invalidate pcurves).
    pub fn transform(&mut self, x: &Xform) {
        for v in &mut self.vertices {
            v.p = x.apply_point(v.p);
        }
        for e in &mut self.edges {
            e.curve = e.curve.transform(|p| x.apply_point(p), |d| x.apply_vec(d));
        }
        for f in &mut self.faces {
            f.surface = transform_surface(&f.surface, x);
        }
    }

    // ---- wire export
    /// `body.faces[]` in the oracle wire shape:
    /// `{id, kind, planar, meta:{feature[,entity]}[, normal][, lump]}`.
    /// `lump` is present only when the body has more than one lump.
    pub fn faces_json(&self) -> Json {
        let multi = self.num_lumps() > 1;
        let mut out = vec![];
        for f in self.face_ids() {
            let face = &self.faces[f.idx()];
            // pinned: a refit (spline) face is never reported planar
            let planar = face.surface.kind() != "spline" && face.surface.is_planar(1e-9);
            let meta = Obj::new()
                .put("feature", face.name.feature.as_str())
                .put_opt("entity", face.name.meta_entity());
            let mut o = Obj::new()
                .put("id", face.name.id())
                .put("kind", wire_kind(&face.surface))
                .put("planar", planar)
                .put("meta", meta);
            if planar {
                o = o.put("normal", v3(self.planar_normal(f)));
            }
            if multi {
                o = o.put("lump", self.lump_index(self.face_lump(f)) as f64);
            }
            out.push(o.build());
        }
        Json::Arr(out)
    }

    /// Outward normal of a planar face.
    pub fn planar_normal(&self, f: FaceId) -> Vec3 {
        let face = &self.faces[f.idx()];
        let n = match &face.surface {
            Surface::Plane { frame } => frame.z,
            s => {
                // GUESS: non-Plane planar surfaces report the normal at the
                // first loop's first vertex parameter.
                let (u, v) = self
                    .faces[f.idx()]
                    .loops
                    .first()
                    .and_then(|l| self.loops[l.idx()].coedges.first())
                    .map(|&c| {
                        let p = self.vertices[self.coedge_start(c).idx()].p;
                        s.closest_param(p)
                    })
                    .unwrap_or((0.0, 0.0));
                s.normal(u, v)
            }
        };
        if face.sense {
            n
        } else {
            -n
        }
    }

    /// Edges exported on the wire, in order (live, non-seam, ≥1 face).
    pub fn wire_edges(&self) -> Vec<EdgeId> {
        self.edge_ids().filter(|&e| !self.is_seam(e) && !self.edge_faces(e).is_empty()).collect()
    }

    /// Oracle edges as chains of wire edges.
    ///
    /// The original kernel's periodic faces have no seam edges (STEP
    /// replies: cylinder faces bounded by bare circles, full spheres / tori
    /// with no loop at all; ck-ops probe `boss`: the boss's bottom arc is
    /// ONE edge `F2.C0|F4.S.e1` running through the point where our seam
    /// lands). ck-topo keeps seams internally (uv-closed loops for the
    /// mass integrator, mesher and booleans), so an arc that crosses a seam
    /// is split at a seam vertex. Consecutive wire edges on the same carrier
    /// curve with the same face pair, meeting head-to-tail at a vertex whose
    /// only other edges are seams, form one chain = one oracle edge.
    /// Chains come in wire order (position of their first member), each
    /// ordered head to tail; unsplit edges are 1-chains.
    pub fn wire_chains(&self) -> Vec<Vec<EdgeId>> {
        let wire = self.wire_edges();
        let mut is_wire = vec![false; self.edges.len()];
        for &e in &wire {
            is_wire[e.idx()] = true;
        }
        let mut inc: Vec<Vec<EdgeId>> = vec![vec![]; self.vertices.len()];
        for e in self.edge_ids() {
            let ed = &self.edges[e.idx()];
            inc[ed.v0.idx()].push(e);
            if ed.v1 != ed.v0 {
                inc[ed.v1.idx()].push(e);
            }
        }
        let pair = |e: EdgeId| {
            let mut f = self.edge_faces(e);
            f.sort();
            f
        };
        let same_carrier = |a: EdgeId, c: EdgeId| {
            let (x, y) = (&self.edges[a.idx()].curve, &self.edges[c.idx()].curve);
            match (x, y) {
                (Curve::Circle { frame: f, r }, Curve::Circle { frame: g, r: s }) => {
                    (f.o - g.o).len() < 1e-9 && (r - s).abs() < 1e-9 && f.z.dot(g.z) > 1.0 - 1e-12
                }
                (Curve::Line { o, d }, Curve::Line { o: p, d: q }) => {
                    d.cross(*q).len() < 1e-12 && d.dot(*q) > 0.0 && (*p - *o).cross(*d).len() < 1e-9
                }
                _ => x == y,
            }
        };
        let mut used = vec![false; self.edges.len()];
        let mut out = vec![];
        for &e in &wire {
            if used[e.idx()] {
                continue;
            }
            used[e.idx()] = true;
            let pe = pair(e);
            let mut chain = vec![e];
            for fwd in [true, false] {
                loop {
                    let end = if fwd { *chain.last().unwrap() } else { chain[0] };
                    let v = if fwd { self.edges[end.idx()].v1 } else { self.edges[end.idx()].v0 };
                    let joins = |n: EdgeId| {
                        let ed = &self.edges[n.idx()];
                        !used[n.idx()]
                            && is_wire[n.idx()]
                            && (if fwd { ed.v0 } else { ed.v1 }) == v
                            && ed.v0 != ed.v1
                            && same_carrier(n, end)
                            && pair(n) == pe
                    };
                    let Some(n) = inc[v.idx()].iter().copied().find(|&n| joins(n)) else { break };
                    if !inc[v.idx()].iter().all(|&o| o == n || o == end || self.is_seam(o)) {
                        break;
                    }
                    used[n.idx()] = true;
                    if fwd {
                        chain.push(n)
                    } else {
                        chain.insert(0, n)
                    }
                }
            }
            // a closed ring of one conic starts where its parameter run
            // starts: the piece whose t0 is no other piece's t1 (ck-ops
            // imprint gives closed SSI circles the range [0.62·2π − 2π,
            // 0.62·2π] and our seam splits it at 0; a ring split at t
            // inside [0, 2π] keeps its original start)
            let n = chain.len();
            if n > 1 && self.edges[chain[0].idx()].v0 == self.edges[chain[n - 1].idx()].v1 {
                let ends: Vec<f64> = chain.iter().map(|&c| self.edges[c.idx()].t1).collect();
                let starts: Vec<usize> = (0..n).filter(|&k| !ends.contains(&self.edges[chain[k].idx()].t0)).collect();
                if starts.len() == 1 {
                    chain.rotate_left(starts[0]);
                }
            }
            out.push(chain);
        }
        out
    }

    /// Arena index of `e` as the original kernel would number it: its arena
    /// has no seam edges, so seams (alive or dead) take no slot. This is
    /// the wire `#n` after an IN-PLACE op (blend/inplace.rs keeps surviving
    /// edges' numbers, dead ones leave gaps). Oracle probe: cylinder (top
    /// plane circle r10, blind 10) + fillet r2 of `F2.C1|F2.S.e7` gives
    /// `F2.C0|F2.S.e7#0, F2.C1|F3.B0#2, F2.S.e7|F3.B0#3` — the dead C1
    /// circle is #1 and there is no slot for the rail seam (our arena:
    /// seam 0, C0 1, C1 2).
    /// Wire chains with their oracle edge number `#n`: arena slots in order,
    /// where every DEAD non-seam edge also takes a slot (an in-place op —
    /// fillet — keeps its survivors' numbers and leaves gaps for dead
    /// edges; ck-ops blend writes its result in that arena layout) and a
    /// live chain takes the slot of its first arena member. Seams and the
    /// further pieces of a chain take none. Without dead edges this is the
    /// chain's position in [`Body::wire_chains`]. Also returns the slot
    /// count (the next free number).
    pub fn wire_numbered(&self) -> (Vec<(usize, Vec<EdgeId>)>, usize) {
        let chains = self.wire_chains();
        let mut head = vec![usize::MAX; self.edges.len()];
        let mut member = vec![false; self.edges.len()];
        for (k, ch) in chains.iter().enumerate() {
            let first = ch.iter().map(|e| e.idx()).min().unwrap();
            head[first] = k;
            for e in ch {
                member[e.idx()] = true;
            }
        }
        let mut num = vec![0usize; chains.len()];
        let mut slot = 0usize;
        for i in 0..self.edges.len() {
            let ed = &self.edges[i];
            if ed.alive {
                if head[i] != usize::MAX {
                    num[head[i]] = slot;
                    slot += 1;
                }
            } else if !member[i] && !self.edge_was_seam(EdgeId(i as u32)) {
                slot += 1;
            }
        }
        (num.into_iter().zip(chains).collect(), slot)
    }

    pub fn seamless_index(&self, e: EdgeId) -> usize {
        (0..e.idx()).filter(|&i| !self.edge_was_seam(EdgeId(i as u32))).count()
    }
    /// Seam test that also works for dead edges (all coedges, alive or
    /// not, on one face).
    fn edge_was_seam(&self, e: EdgeId) -> bool {
        let cs = &self.edges[e.idx()].coedges;
        cs.len() >= 2 && cs.iter().all(|&c| self.coedge_face(c) == self.coedge_face(cs[0]))
    }

    /// Point at half the arc length of a chain (= the curve at
    /// mid-parameter for one edge, and for a seam-split arc).
    pub fn chain_mid(&self, chain: &[EdgeId]) -> Vec3 {
        if chain.len() == 1 {
            return self.edge_mid(chain[0]);
        }
        // pieces of one conic with contiguous parameters: the mid parameter
        // of the whole run (what the oracle's single edge reports)
        let (f, l) = (&self.edges[chain[0].idx()], &self.edges[chain[chain.len() - 1].idx()]);
        if matches!(f.curve, Curve::Circle { .. } | Curve::Ellipse { .. })
            && chain.windows(2).all(|w| {
                let (a, b) = (&self.edges[w[0].idx()], &self.edges[w[1].idx()]);
                a.curve == b.curve && a.t1 == b.t0
            })
        {
            return f.curve.eval(0.5 * (f.t0 + l.t1));
        }
        let lens: Vec<f64> = chain.iter().map(|&e| self.edge_length(e)).collect();
        let mut half = 0.5 * lens.iter().sum::<f64>();
        for (&e, &l) in chain.iter().zip(&lens) {
            if half <= l || e == *chain.last().unwrap() {
                let ed = &self.edges[e.idx()];
                // conics / lines: arc length is linear in the parameter
                let f = if l > 0.0 { (half / l).clamp(0.0, 1.0) } else { 0.5 };
                return ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * f);
            }
            half -= l;
        }
        unreachable!()
    }

    /// `body.edges[]` in the oracle wire shape:
    /// `{pts, faces, key, mid, circle:{c,n,r} | line:true}`, one entry per
    /// [`Body::wire_chains`] chain. `pts` comes from `pts(edge)` (the
    /// tessellator owns the discretisation; see [`Body::sample_edge`] for a
    /// stand-alone default), concatenated along a chain.
    pub fn edges_json(&self, pts: &dyn Fn(&Body, EdgeId) -> Vec<Vec3>) -> Json {
        let mut out = vec![];
        for (seq, chain) in self.wire_numbered().0 {
            let e = chain[0];
            if self.is_split_rail(e) {
                continue; // numbered, not listed
            }
            let mut names: Vec<String> = self.edge_faces(e).iter().map(|&f| self.face_id_str(f)).collect();
            names.sort();
            // pieces of one sheet entity share a face name (surface loft):
            // the oracle lists it once and keys `<id>#n`
            names.dedup();
            let key = if names.len() >= 2 {
                edge_key(&names[0], &names[1], seq)
            } else {
                // GUESS: open edge (one face) keys as `<id>#n`.
                format!("{}#{}", names[0], seq)
            };
            let mut pl: Vec<Vec3> = vec![];
            for &c in &chain {
                let q = pts(self, c);
                let skip = usize::from(!pl.is_empty() && !q.is_empty());
                pl.extend(q.into_iter().skip(skip));
            }
            let p: Vec<Json> = pl.into_iter().map(v3).collect();
            let mut o = Obj::new()
                .put("pts", Json::Arr(p))
                .put("faces", Json::Arr(names.iter().map(|s| Json::from(s.as_str())).collect()))
                .put("key", key)
                .put("mid", v3(self.chain_mid(&chain)));
            let ed = &self.edges[e.idx()];
            match &ed.curve {
                Curve::Circle { .. } => {
                    let (c, n, r) = ed.curve.circle_info().unwrap();
                    o = o.put("circle", Obj::new().put("c", v3(c)).put("n", v3(n)).put("r", r));
                }
                Curve::Line { .. } => o = o.put("line", true),
                _ => {}
            }
            out.push(o.build());
        }
        Json::Arr(out)
    }

    /// Default polyline for an edge: lines are their two vertices; curves
    /// use a chord-height rule.
    /// GUESS: oracle counts fit sagitta ≈ 3.74e-4 × (model bbox diagonal)
    /// within ±1 segment (see README); `tol` = that sagitta. The real rule
    /// lives with the mesher (ck-tess) and has extra clamps for small arcs.
    pub fn sample_edge(&self, e: EdgeId, tol: f64) -> Vec<Vec3> {
        let ed = &self.edges[e.idx()];
        let (a, b) = (self.vertices[ed.v0.idx()].p, self.vertices[ed.v1.idx()].p);
        let sample = || {
            let n = match &ed.curve {
                Curve::Line { .. } => 1,
                Curve::Circle { r, .. } => arc_segments(*r, ed.t1 - ed.t0, tol),
                Curve::Ellipse { a, b, .. } => arc_segments(a.max(*b), ed.t1 - ed.t0, tol),
                Curve::BSpline(_) | Curve::Conic { .. } | Curve::Sinusoid { .. } => {
                    let len = ed.curve.length_between(ed.t0, ed.t1);
                    ((len / (8.0 * tol).sqrt().max(1e-9)).ceil() as usize).clamp(2, 512)
                }
            };
            let mut pts = ed.curve.sample(ed.t0, ed.t1, n);
            pts[0] = a;
            *pts.last_mut().unwrap() = b;
            pts
        };
        if matches!(ed.curve, Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. }) {
            return sample();
        }
        // a free-form edge costs an arc-length integral and its samples: pure in the curve, range, ends and tolerance,
        // so an edge an edit leaves alone is a lookup (content-keyed, bit-identical)
        thread_local! {
            static MEMO: std::cell::RefCell<ck_core::memo::Memo<std::rc::Rc<Vec<Vec3>>>> = std::cell::RefCell::new(ck_core::memo::Memo::new(1 << 22));
        }
        let mut key = Vec::with_capacity(64);
        ed.curve.push_key(&mut key);
        key.extend([ed.t0.to_bits(), ed.t1.to_bits(), tol.to_bits()]);
        for p in [a, b] {
            key.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        }
        let v = ck_core::memo::cached_sized(&MEMO, key, || std::rc::Rc::new(sample()), |v| 3 * v.len());
        (*v).clone()
    }
}

fn arc_segments(r: f64, sweep: f64, tol: f64) -> usize {
    if tol >= r {
        return ((sweep.abs() / (std::f64::consts::PI / 2.0)).ceil() as usize).max(1);
    }
    let step = 2.0 * (1.0 - tol / r).acos();
    ((sweep.abs() / step).ceil() as usize).clamp(1, 1024)
}

/// Wire face `kind`: the kind table in kernel.strings is
/// `plane cylinder cone sphere torus spline offset extrusion revolution`;
/// oracle probes report `extrusion` (closed-spline / ellipse extrudes) and
/// `revolution` (spline revolve). `offset` unprobed.
pub fn wire_kind(s: &Surface) -> &'static str {
    s.kind()
}

/// Control net of the piece [u0,u1] of `c`, clamped at both ends (knots
/// raised to multiplicity p there, also at an unclamped domain end of a
/// periodic curve) — the oracle represents a closed cv-spline as a clamped
/// curve and its halves by knot insertion (cvs.step: 8 CPs on (0..5),
/// halves 6 CPs on (0,1,2,2.5) / (2.5,3,4,5)).
fn clamped_net(c: &ck_geom::nurbs::bspline::BSplineCurve<Vec3>, u0: f64, u1: f64) -> Vec<Vec3> {
    let p = c.degree;
    let c = c.insert_knot(u0, p).insert_knot(u1, p);
    let first_ge = |u: f64| c.knots.iter().position(|&k| k >= u).unwrap_or(c.knots.len());
    let last_le = |u: f64| c.knots.iter().rposition(|&k| k <= u).unwrap_or(0);
    let i0 = last_le(u0).saturating_sub(p);
    let i1 = first_ge(u1).saturating_sub(1).min(c.ctrl.len() - 1);
    if i0 > i1 {
        return c.ctrl.clone();
    }
    c.ctrl[i0..=i1].to_vec()
}

pub(crate) fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

/// Rigid transform of a surface.
pub fn transform_surface(s: &Surface, x: &Xform) -> Surface {
    let tf = |f: &Frame| Frame { o: x.apply_point(f.o), x: x.apply_vec(f.x), y: x.apply_vec(f.y), z: x.apply_vec(f.z) };
    match s {
        Surface::Plane { frame } => Surface::Plane { frame: tf(frame) },
        Surface::Cylinder { frame, r } => Surface::Cylinder { frame: tf(frame), r: *r },
        Surface::Cone { frame, r, semi_angle } => Surface::Cone { frame: tf(frame), r: *r, semi_angle: *semi_angle },
        Surface::Sphere { frame, r } => Surface::Sphere { frame: tf(frame), r: *r },
        Surface::Torus { frame, major, minor } => Surface::Torus { frame: tf(frame), major: *major, minor: *minor },
        Surface::Revolution { profile, axis_o, axis } => Surface::Revolution {
            profile: Box::new(profile.transform(|p| x.apply_point(p), |d| x.apply_vec(d))),
            axis_o: x.apply_point(*axis_o),
            axis: x.apply_vec(*axis),
        },
        Surface::Extrusion { profile, dir } => Surface::Extrusion {
            profile: Box::new(profile.transform(|p| x.apply_point(p), |d| x.apply_vec(d))),
            dir: x.apply_vec(*dir),
        },
        Surface::BSpline(b) => {
            let mut b = b.clone();
            b.ctrl.iter_mut().for_each(|p| *p = x.apply_point(*p));
            Surface::BSpline(b)
        }
        // a mirror reverses the handedness: M R(a, u) = R(-M a, u) M, so the axis flips and the rise changes sign
        Surface::Screw { profile, axis_o, axis, rise, urange, vrange } => {
            let m = x.det() < 0.0;
            Surface::Screw {
                profile: Box::new(profile.transform(|p| x.apply_point(p), |d| x.apply_vec(d))),
                axis_o: x.apply_point(*axis_o),
                axis: if m { -x.apply_vec(*axis) } else { x.apply_vec(*axis) },
                rise: if m { -*rise } else { *rise },
                urange: *urange,
                vrange: *vrange,
            }
        }
        Surface::Offset { base, dist } => Surface::Offset { base: Box::new(transform_surface(base, x)), dist: *dist },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seam_split_arcs_export_as_one_edge() {
        use crate::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
        let prof = Profile::new(
            crate::make::tests::top_frame(),
            vec![ProfileRegion {
                outer: ProfileLoop::new(vec![ProfileSeg::circle("e7", Vec2::new(5.0, 3.0), 10.0)]),
                holes: vec![],
            }],
        );
        let mut b = extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap();
        let before = ck_json::to_string(&b.edges_json(&|b, e| b.sample_edge(e, 0.01)));
        // oracle arena numbering skips the seam rail: C0 circle #0, C1 #1
        let w = b.wire_edges();
        assert_eq!((w[0].idx(), w[1].idx()), (1, 2));
        assert_eq!((b.seamless_index(w[0]), b.seamless_index(w[1])), (0, 1));
        // split the C0 circle (seam vertex at t=0) at t=2: two arcs joined at
        // the seam vertex and at the new vertex -> still one oracle edge
        let c0 = b.wire_edges()[0];
        crate::edit::split_edge(&mut b, c0, 2.0);
        assert_eq!(b.wire_edges().len(), 3);
        let ch = b.wire_chains();
        assert_eq!(ch.len(), 2);
        assert_eq!(ch[0].len(), 2);
        let after = b.edges_json(&|b, e| b.sample_edge(e, 0.01));
        let a = after.as_arr().unwrap();
        assert_eq!(a.len(), 2);
        let bj = ck_json::parse(&before).unwrap();
        let b0 = &bj.as_arr().unwrap()[0];
        assert_eq!(a[0].get_str("key"), b0.get_str("key"));
        let (m0, m1) = (a[0].get("mid").unwrap().as_vec3().unwrap(), b0.get("mid").unwrap().as_vec3().unwrap());
        assert!((0..3).all(|i| (m0[i] - m1[i]).abs() < 1e-9), "{m0:?} {m1:?}");
    }

    #[test]
    fn face_name_roundtrip() {
        for id in [
            "F2.S.e1",
            "F2.C0",
            "F2.C1~2",
            "F4.S.e1@F1.1",
            "F8.S.h1@F9.3@F1.1",
            "F10.B0@F2.1",
            "F11.B3@F1.1",
            "F4.C0@F9.1@F1.1",
        ] {
            let n = FaceName::parse(id).unwrap();
            assert_eq!(n.id(), id);
        }
        let n = FaceName::parse("F8.S.h1@F9.1@F1.1").unwrap();
        assert_eq!(n.feature, "F8");
        assert_eq!(n.meta_entity().as_deref(), Some("h1@F9.1@F1.1"));
        assert_eq!(FaceName::parse("F4.C0@F1.1").unwrap().meta_entity(), None);
        assert_eq!(FaceName::parse("F2.C0~1").unwrap().region, Some(1));
    }

    #[test]
    fn edge_keys_sorted_bytewise() {
        // oracle-flange.json: "F11.B1@F1.1|F6.S.e6@F1.1#24", "F2.S.e12@F1.1|F4.S.e1@F1.1#1"
        assert_eq!(edge_key("F6.S.e6@F1.1", "F11.B1@F1.1", 24), "F11.B1@F1.1|F6.S.e6@F1.1#24");
        assert_eq!(edge_key("F4.S.e1@F1.1", "F2.S.e12@F1.1", 1), "F2.S.e12@F1.1|F4.S.e1@F1.1#1");
        // box probe: "F2.S.e10|F2.S.e9#0"
        assert_eq!(edge_key("F2.S.e9", "F2.S.e10", 0), "F2.S.e10|F2.S.e9#0");
    }

    #[test]
    fn assembly_append_transform() {
        // insertPart-style: each instance gets `@F<n>.1` and its own lump
        let mut asm = Body::new();
        for (k, dx) in [(1, 0.0), (2, 100.0)] {
            let mut b = crate::make::tests::box_body();
            b.push_instance(&format!("F{k}.1"));
            b.transform(&Xform::rotation(Vec3::ZERO, Vec3::Z, 0.3).compose(&Xform::translation(Vec3::new(dx, 0.0, 0.0))));
            asm.append(&b);
        }
        assert!(crate::check::check(&asm, 1e-9).is_empty());
        let fj = asm.faces_json();
        let f = &fj.as_arr().unwrap()[8];
        assert_eq!(f.get_str("id"), Some("F2.S.e1@F2.1"));
        assert_eq!(f.path(&["meta", "entity"]).and_then(|j| j.as_str()), Some("e1@F2.1"));
        assert_eq!(f.get("lump").and_then(|j| j.as_f64()), Some(1.0));
        let m = crate::mass::mass_props(&asm);
        assert!((m.volume - 16000.0).abs() < 1e-8 && (m.area - 5600.0).abs() < 1e-8);
        let k = asm.edges_json(&|b, e| b.sample_edge(e, 0.01));
        assert_eq!(k.as_arr().unwrap()[12].get_str("key"), Some("F2.S.e1@F2.1|F2.S.e4@F2.1#12"));
    }

    #[test]
    fn instance_suffix() {
        let mut n = FaceName::side("F8", "h1");
        n.push_instance("F9.1");
        n.push_instance("F1.1");
        assert_eq!(n.id(), "F8.S.h1@F9.1@F1.1");
    }
}

/// Box of 65 samples of `c` over `[t0, t1]` (the edge part of [`Body::bbox`]), memoised by the exact curve and
/// range: a boolean measures the boxes of both operands and of its result, mostly the same edges each time.
fn sampled_box(c: &Curve, t0: f64, t1: f64) -> (Vec3, Vec3) {
    thread_local! {
        static MEMO: std::cell::RefCell<ck_core::memo::Memo<(Vec3, Vec3)>> = std::cell::RefCell::new(ck_core::memo::Memo::new(1 << 19));
    }
    let mut key = Vec::with_capacity(32);
    c.push_key(&mut key);
    key.extend([t0.to_bits(), t1.to_bits()]);
    ck_core::memo::cached(&MEMO, key, || {
        let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = -lo;
        for p in c.sample(t0, t1, 64) {
            lo = lo.min(p);
            hi = hi.max(p);
        }
        (lo, hi)
    })
}
