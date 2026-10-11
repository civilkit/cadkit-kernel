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
        self.lumps.push(Lump { shells: vec![], alive: true });
        LumpId(self.lumps.len() as u32 - 1)
    }
    pub fn add_shell(&mut self, lump: LumpId) -> ShellId {
        self.shells.push(Shell { faces: vec![], lump, alive: true });
        let s = ShellId(self.shells.len() as u32 - 1);
        self.lumps[lump.idx()].shells.push(s);
        s
    }
    pub fn add_vertex(&mut self, p: Vec3) -> VertexId {
        self.vertices.push(Vertex { p, alive: true });
        VertexId(self.vertices.len() as u32 - 1)
    }
    pub fn add_edge(&mut self, curve: Curve, t0: f64, t1: f64, v0: VertexId, v1: VertexId) -> EdgeId {
        self.edges.push(Edge { curve, t0, t1, v0, v1, coedges: vec![], alive: true });
        EdgeId(self.edges.len() as u32 - 1)
    }
    pub fn add_face(&mut self, shell: ShellId, surface: Surface, sense: bool, name: FaceName) -> FaceId {
        self.faces.push(Face { surface, sense, loops: vec![], shell, name, alive: true });
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
        for &l in &self.faces[f.idx()].loops {
            for &c in &self.loops[l.idx()].coedges {
                let e = self.coedges[c.idx()].edge;
                if !out.contains(&e) {
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
        self.face_ids().find(|&f| self.faces[f.idx()].name.id() == id)
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
        let ce = &self.coedges[c.idx()];
        if let Some(pc) = &ce.pcurve {
            return pc.eval(t);
        }
        let face = &self.faces[self.coedge_face(c).idx()];
        let p = self.edges[ce.edge.idx()].curve.eval(t);
        let (mut u, mut v) = face.surface.closest_param(p);
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
            for p in ed.curve.sample(ed.t0, ed.t1, 64) {
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
        (lo, hi)
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
            let planar = face.surface.is_planar(1e-9);
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

    /// `body.edges[]` in the oracle wire shape:
    /// `{pts, faces, key, mid, circle:{c,n,r} | line:true}`. `pts` comes from
    /// `pts(edge)` (the tessellator owns the discretisation; see
    /// [`Body::sample_edge`] for a stand-alone default).
    pub fn edges_json(&self, pts: &dyn Fn(&Body, EdgeId) -> Vec<Vec3>) -> Json {
        let mut out = vec![];
        for (seq, e) in self.wire_edges().into_iter().enumerate() {
            let mut names: Vec<String> = self.edge_faces(e).iter().map(|&f| self.face_id_str(f)).collect();
            names.sort();
            let key = if names.len() >= 2 {
                edge_key(&names[0], &names[1], seq)
            } else {
                // GUESS: open edge (one face) keys as `<id>#n`.
                format!("{}#{}", names[0], seq)
            };
            let p: Vec<Json> = pts(self, e).into_iter().map(v3).collect();
            let mut o = Obj::new()
                .put("pts", Json::Arr(p))
                .put("faces", Json::Arr(names.iter().map(|s| Json::from(s.as_str())).collect()))
                .put("key", key)
                .put("mid", v3(self.edge_mid(e)));
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
        let n = match &ed.curve {
            Curve::Line { .. } => 1,
            Curve::Circle { r, .. } => arc_segments(*r, ed.t1 - ed.t0, tol),
            Curve::Ellipse { a, b, .. } => arc_segments(a.max(*b), ed.t1 - ed.t0, tol),
            Curve::BSpline(_) => {
                let len = ed.curve.length_between(ed.t0, ed.t1);
                ((len / (8.0 * tol).sqrt().max(1e-9)).ceil() as usize).clamp(2, 512)
            }
        };
        let mut pts = ed.curve.sample(ed.t0, ed.t1, n);
        pts[0] = a;
        *pts.last_mut().unwrap() = b;
        pts
    }
}

fn arc_segments(r: f64, sweep: f64, tol: f64) -> usize {
    if tol >= r {
        return ((sweep.abs() / (std::f64::consts::PI / 2.0)).ceil() as usize).max(1);
    }
    let step = 2.0 * (1.0 - tol / r).acos();
    ((sweep.abs() / step).ceil() as usize).clamp(1, 1024)
}

/// Wire face `kind`.
/// GUESS: swept (extrusion/revolution) and offset surfaces are reported as
/// `spline` — the frontend only knows plane/cylinder/cone/sphere/torus/spline
/// (oracle-flange.json + probes); the kernel likely converts them to NURBS.
pub fn wire_kind(s: &Surface) -> &'static str {
    match s {
        Surface::Extrusion { .. } | Surface::Revolution { .. } | Surface::Offset { .. } => "spline",
        s => s.kind(),
    }
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
        Surface::Offset { base, dist } => Surface::Offset { base: Box::new(transform_surface(base, x)), dist: *dist },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
