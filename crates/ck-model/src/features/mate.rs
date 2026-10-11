//! Assemblies: `insertPart` components, `mate` constraints, component
//! patterns (ck-model/src/features/mate.rs in rs_paths.txt).
//!
//! Pinned by oracle probes (two-box / box+cylinder assemblies):
//! * a component is the nested part's regenerated body, its faces renamed
//!   `<id>@<insert feature>.1` (`F2.S.e1@F1.1`, nested pattern suffixes stay
//!   in front: `F8.S.h1@F9.1@F1.1`), placed by `place:{o,x,y,z}` or by
//!   `dx/dy/dz` + `axis` + `angle` (deg, rotation about the world axis
//!   through the origin, then the translation), and APPENDED to the body as
//!   its own lumps — assembly bodies are never united (flange golden: 10
//!   lumps, 182 faces);
//! * `components[]` = `{id, dof, place}` keyed (sorted) by insert id; `dof`
//!   6 free, 0 `fixed`; mates lower it (see [`super::mate_solve`]);
//! * nested regens ignore the nested `rollback`, emit no sketches / planes
//!   into the assembly reply; a nested error -> `insertPartErrors` with
//!   `detail: "<id>: <code>"` of the FIRST error in id order (the reply's
//!   error map order), no component; features but no body ->
//!   `insertPartEmpty`; an empty feature list -> nothing (no error, no
//!   component);
//! * trace: when the executed inserts reference >= 2 DISTINCT part
//!   documents, a `"<n> inserted parts"` span (one nested `regenerate N
//!   features` per distinct part, first-use order) opens the regen before
//!   any feature; otherwise the single part's nested regen sits under the
//!   first insertPart span that uses it. Identical parts are regenerated
//!   once (later inserts reuse the result silently).
//! * component `cpattern` (`seeds: []`, `bodies: [{face}]`): every
//!   component owning one of the faces is copied about the axis; copies are
//!   renamed `@<pattern>.<k>` AFTER the component suffix
//!   (`F4.C0@F6.1@F12.1`) and appended after all bodies, seed by seed
//!   (flange golden); copies are not listed in `components[]`.

use super::{Brep, FeatureKind, Placement};
use crate::regen::{self, FeatureError, RegenError, RegenOpts, RegenState, Tracer};
use crate::{Feature, Part};
use ck_core::linalg::{Frame, Mat3, Vec3};
use ck_core::xform::Xform;
use ck_json::{Json, Obj};
use ck_topo::body::Body;

/// Kernel error codes (kernel.strings).
pub const INSERT_PART_ERRORS: &str = "insertPartErrors";
pub const INSERT_PART_EMPTY: &str = "insertPartEmpty";
pub const MATE_CONFLICT: &str = "mateConflict";
pub const MATE_GEOMETRY: &str = "mateGeometry";
pub const MATE_SAME_BODY: &str = "mateSameBody";
pub const MATE_FIXED: &str = "mateFixed";

/// Index ranges of a component's entities inside the assembly body
/// (components are appended, so their arenas are contiguous).
#[derive(Debug, Clone, Default)]
pub struct Span3 {
    pub vertices: (usize, usize),
    pub edges: (usize, usize),
    pub coedges: (usize, usize),
    pub loops: (usize, usize),
    pub faces: (usize, usize),
    pub shells: (usize, usize),
    pub lumps: (usize, usize),
}

/// One inserted component.
#[derive(Debug, Clone)]
pub struct Component {
    /// insertPart feature id (`F6`).
    pub id: String,
    pub fixed: bool,
    /// Current placement (local -> world).
    pub place: Xf,
    /// Its entities in the assembly body.
    pub range: Span3,
    /// Remaining degrees of freedom (wire `dof`).
    pub dof: u32,
}

impl Component {
    pub fn to_json(&self) -> Json {
        Obj::new().put("id", self.id.as_str()).put("dof", self.dof as f64).put("place", regen::frame_json(&self.place.to_xform().to_frame())).build()
    }
    pub fn owns_face(&self, f: ck_topo::body::FaceId) -> bool {
        (self.range.faces.0..self.range.faces.1).contains(&f.idx())
    }
}

/// Result of regenerating one nested part document.
#[derive(Clone)]
pub struct Nested {
    pub body: Option<Body>,
    pub errors: Vec<RegenError>,
    pub n_features: usize,
}

/// Collects a nested regen's trace spans into a [`super::Span`] tree.
#[derive(Default)]
pub struct SpanSink {
    stack: Vec<super::Span>,
    pub roots: Vec<super::Span>,
    open: Vec<usize>,
}

impl Tracer for SpanSink {
    fn begin(&mut self, label: &str) -> usize {
        self.stack.push(super::Span::leaf(label));
        self.open.push(self.stack.len() - 1);
        self.stack.len() - 1
    }
    fn relabel(&mut self, id: usize, label: &str) {
        if let Some(s) = self.stack.get_mut(id) {
            s.label = label.to_string();
        }
    }
    fn end(&mut self, _id: usize) {
        // spans close in LIFO order
        let i = self.open.pop().expect("balanced spans");
        debug_assert_eq!(i, self.stack.len() - 1);
        let s = self.stack.pop().unwrap();
        match self.stack.last_mut() {
            Some(p) => p.children.push(s),
            None => self.roots.push(s),
        }
    }
}

/// [`Brep`] + a span sink: the exec of a nested regen.
struct NestedExec {
    brep: Brep,
    sink: SpanSink,
}

impl regen::FeatureExec for NestedExec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.sink)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, state: &RegenState, face: &str) -> Option<Frame> {
        regen::FeatureExec::face_frame(&mut self.brep, state, face)
    }
    fn prepare(&mut self, state: &RegenState, features: &[&Feature]) {
        self.brep.prepare(state, features);
        self.brep.flush_spans(&mut self.sink);
    }
    fn exec(&mut self, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
        let r = regen::FeatureExec::exec(&mut self.brep, state, f);
        self.brep.flush_spans(&mut self.sink);
        r
    }
}

/// Regenerate a nested part: `(trace span "regenerate N features", result)`.
pub fn regen_nested(part: &Part) -> (super::Span, Nested) {
    let mut ex = NestedExec { brep: Brep::new(), sink: SpanSink::default() };
    // pinned: the nested `rollback` is ignored
    let opts = RegenOpts { up_to: None, ignore_rollback: true };
    let r = regen::regen(part, &opts, &mut ex);
    let n = part.features.len();
    let span = super::Span { label: format!("regenerate {n} features"), children: ex.sink.roots };
    let mut errors = r.errors;
    errors.sort_by(|a, b| a.feature.cmp(&b.feature));
    (span, Nested { body: ex.brep.body, errors, n_features: n })
}

/// Canonical text of a nested part (cache key).
fn part_key(p: &Part) -> String {
    ck_json::to_string(&p.raw)
}

/// Assembly state carried by [`Brep`].
#[derive(Default, Clone)]
pub struct Assembly {
    pub components: Vec<Component>,
    /// Face index ranges of every appended assembly body (components and
    /// their pattern copies).
    pub face_ranges: Vec<(usize, usize)>,
    /// Edge index ranges of the same bodies.
    pub edge_ranges: Vec<(usize, usize)>,
    /// Nested regen results by part text (filled by the pre-pass or lazily).
    pub cache: Vec<(String, Nested)>,
}

impl Brep {
    /// Pre-pass (oracle trace): >= 2 distinct inserted parts are regenerated
    /// up front under `"<n> inserted parts"`.
    pub fn prepare(&mut self, _state: &RegenState, features: &[&Feature]) {
        let mut keys: Vec<(String, &Part)> = vec![];
        for f in features {
            if let FeatureKind::InsertPart { part: Some(p), .. } = &f.kind {
                let k = part_key(p);
                if !keys.iter().any(|(x, _)| *x == k) {
                    keys.push((k, p));
                }
            }
        }
        if keys.len() < 2 {
            return;
        }
        let mut sp = super::Span::leaf(format!("{} inserted parts", keys.len()));
        for (k, p) in keys {
            let (s, n) = regen_nested(p);
            sp.children.push(s);
            self.asm.cache.push((k, n));
        }
        self.spans.push(sp);
    }

    /// `insertPart`.
    pub fn exec_insert(&mut self, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
        let FeatureKind::InsertPart { part: Some(p), place, fixed, .. } = &f.kind else { unreachable!() };
        let k = part_key(p);
        let nested = match self.asm.cache.iter().find(|(x, _)| *x == k) {
            Some((_, n)) => n.clone(),
            None => {
                if p.features.is_empty() {
                    // pinned: no nested span, no error, no component
                    return Ok(());
                }
                let (s, n) = regen_nested(p);
                self.spans.push(s);
                self.asm.cache.push((k, n.clone()));
                n
            }
        };
        if let Some(e) = nested.errors.first() {
            let detail = format!("{}: {}", e.feature, e.error.code);
            return Err(FeatureError::new(INSERT_PART_ERRORS).with("detail", Json::Str(detail)));
        }
        let Some(mut b) = nested.body else {
            if nested.n_features == 0 {
                return Ok(());
            }
            return Err(FeatureError::new(INSERT_PART_EMPTY));
        };
        let x = placement(state, f, place)?;
        b.push_instance(&format!("{}.1", f.id));
        x.body(&mut b);
        let range = self.append_body(&b);
        let c = Component { id: f.id.clone(), fixed: *fixed, place: x, range, dof: if *fixed { 0 } else { 6 } };
        self.asm.components.push(c);
        Ok(())
    }

    /// Append `b` to the body as new lumps; returns its entity ranges.
    pub fn append_body(&mut self, b: &Body) -> Span3 {
        let body = self.body.get_or_insert_with(Body::new);
        let n = |b: &Body| [b.vertices.len(), b.edges.len(), b.coedges.len(), b.loops.len(), b.faces.len(), b.shells.len(), b.lumps.len()];
        let a = n(body);
        body.append(b);
        let z = n(body);
        self.asm.face_ranges.push((a[4], z[4]));
        self.asm.edge_ranges.push((a[1], z[1]));
        Span3 {
            vertices: (a[0], z[0]),
            edges: (a[1], z[1]),
            coedges: (a[2], z[2]),
            loops: (a[3], z[3]),
            faces: (a[4], z[4]),
            shells: (a[5], z[5]),
            lumps: (a[6], z[6]),
        }
    }

    /// Faces whose wire normals are re-normalised (see `faces_json`).
    pub fn renorm_faces(&self) -> (Vec<(usize, usize)>, Vec<(usize, usize)>) {
        (self.asm.face_ranges.clone(), self.asm.edge_ranges.clone())
    }

    /// The `components[]` wire array (sorted by id).
    pub fn components_json(&self) -> Vec<Json> {
        let mut cs: Vec<&Component> = self.asm.components.iter().collect();
        cs.sort_by(|a, b| a.id.cmp(&b.id));
        cs.into_iter().map(|c| c.to_json()).collect()
    }

    /// Component owning the face named `id`.
    pub fn component_of(&self, id: &str) -> Option<usize> {
        let b = self.body.as_ref()?;
        let f = b.find_face(id)?;
        self.asm.components.iter().position(|c| c.owns_face(f))
    }

    /// Component `cpattern` (`bodies: [{face}]`).
    pub fn exec_component_pattern(&mut self, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
        let FeatureKind::CPattern { bodies, axis, equal, reverse, .. } = &f.kind else { unreachable!() };
        let count = state.num(f, "count", 0.0)?;
        let angle = state.num(f, "angle", 360.0)?;
        let (o, d) = axis.as_ref().and_then(super::pattern::axis_of).ok_or_else(|| FeatureError::new(regen::codes::MISSING_REFERENCE))?;
        let n = if !count.is_finite() || count < 1.0 { 1 } else { count.round() as usize };
        let step = ck_ops::boolean::apply::circular_step(angle, n, *equal) * if *reverse { -1.0 } else { 1.0 };
        // GUESS: per-instance angle (step·k) to radians, rotation about the
        // axis through `o` as `t = o − R·o`
        let xs: Vec<Xf> = (1..n.max(1))
            .map(|k| {
                let mut x = Xf::rotation(d, (step * k as f64).to_radians());
                let ro = x.vec(o);
                x.t = [o.x - ro.x, o.y - ro.y, o.z - ro.z];
                x
            })
            .collect();
        let mut seeds = vec![];
        for j in bodies {
            let id = j.get_str("face").unwrap_or("");
            // GUESS: an unknown face -> missingReference
            let c = self.component_of(id).ok_or_else(|| FeatureError::new(regen::codes::MISSING_REFERENCE))?;
            if !seeds.contains(&c) {
                seeds.push(c);
            }
        }
        // pinned (probe): copies follow component order, not `bodies` order
        seeds.sort();
        for c in seeds {
            let seed = self.component_body(c);
            for (k, x) in xs.iter().enumerate() {
                let mut b = seed.clone();
                x.body(&mut b);
                b.push_instance(&format!("{}.{}", f.id, k + 1));
                self.append_body(&b);
            }
        }
        Ok(())
    }

    /// A standalone copy of component `c`'s (world) body, arena order kept
    /// (pinned: pattern copies list edges in the seed's order).
    pub fn component_body(&self, c: usize) -> Body {
        sub_body(self.body.as_ref().expect("component body"), &self.asm.components[c].range)
    }
}

/// Initial placement of an insert.
fn placement(state: &RegenState, f: &Feature, p: &Placement) -> Result<Xf, FeatureError> {
    if let Some(fr) = &p.frame {
        return Ok(Xf::from_frame(fr));
    }
    let mut x = Xf::IDENTITY;
    if let Some(ax) = p.axis.as_ref().and_then(super::pattern::axis_of) {
        let a = state.num(f, "angle", 0.0)?;
        // GUESS: rotation about the world axis through the origin, degrees
        // via `to_radians`
        x = Xf::rotation(ax.1, a.to_radians());
    }
    x.t = [p.dx, p.dy, p.dz];
    Ok(x)
}

/// `mate` (first pass: recorded, no solve yet).
pub fn exec_mate(br: &mut Brep, _state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Mate { a, b, .. } = &f.kind else { unreachable!() };
    if br.body.is_none() || br.asm.components.is_empty() {
        return Err(FeatureError::new(regen::codes::NOTHING_TO_CUT));
    }
    let ca = a.as_ref().and_then(|r| br.component_of(r.as_str()));
    let cb = b.as_ref().and_then(|r| br.component_of(r.as_str()));
    let (Some(_ca), Some(_cb)) = (ca, cb) else { return Err(FeatureError::new(regen::codes::MISSING_REFERENCE)) };
    Ok(())
}

/// The entities of `r` as a standalone body (ids re-based).
pub fn sub_body(b: &Body, r: &Span3) -> Body {
    use ck_topo::body::{CoedgeId, EdgeId, FaceId, LoopId, LumpId, ShellId, VertexId};
    let o = |x: (usize, usize)| x.0 as u32;
    let (ov, oe, oc, ol, of, os, olu) = (o(r.vertices), o(r.edges), o(r.coedges), o(r.loops), o(r.faces), o(r.shells), o(r.lumps));
    let mut out = Body::new();
    out.vertices = b.vertices[r.vertices.0..r.vertices.1].to_vec();
    out.edges = b.edges[r.edges.0..r.edges.1]
        .iter()
        .map(|e| {
            let mut e = e.clone();
            e.v0 = VertexId(e.v0.0 - ov);
            e.v1 = VertexId(e.v1.0 - ov);
            e.coedges.iter_mut().for_each(|c| *c = CoedgeId(c.0 - oc));
            e
        })
        .collect();
    out.coedges = b.coedges[r.coedges.0..r.coedges.1]
        .iter()
        .map(|c| {
            let mut c = c.clone();
            c.edge = EdgeId(c.edge.0 - oe);
            c.lp = LoopId(c.lp.0 - ol);
            c
        })
        .collect();
    out.loops = b.loops[r.loops.0..r.loops.1]
        .iter()
        .map(|l| {
            let mut l = l.clone();
            l.face = FaceId(l.face.0 - of);
            l.coedges.iter_mut().for_each(|c| *c = CoedgeId(c.0 - oc));
            l
        })
        .collect();
    out.faces = b.faces[r.faces.0..r.faces.1]
        .iter()
        .map(|f| {
            let mut f = f.clone();
            f.shell = ShellId(f.shell.0 - os);
            f.loops.iter_mut().for_each(|l| *l = LoopId(l.0 - ol));
            f
        })
        .collect();
    out.shells = b.shells[r.shells.0..r.shells.1]
        .iter()
        .map(|s| {
            let mut s = s.clone();
            s.lump = LumpId(s.lump.0 - olu);
            s.faces.iter_mut().for_each(|f| *f = FaceId(f.0 - of));
            s
        })
        .collect();
    out.lumps = b.lumps[r.lumps.0..r.lumps.1]
        .iter()
        .map(|l| {
            let mut l = l.clone();
            l.shells.iter_mut().for_each(|s| *s = ShellId(s.0 - os));
            l
        })
        .collect();
    out
}

// ------------------------------------------------------------------ Xf

/// The kernel's rigid transform with its exact arithmetic (decompiled:
/// rotation from axis/angle = func f3910, product = f3945/f3920, frame
/// transform = f3928, body transform = f3499/f3800/f3762). `r` is ROW-major;
/// `R·v = (r0·vx + r1·vy) + r2·vz` per row, points `R·p + t`.
/// GUESS: these belong to ck-core::xform in the original; kept here because
/// ck-core's `Xform` uses a different association order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Xf {
    pub r: [f64; 9],
    pub t: [f64; 3],
}

/// `len > 1e-300` guard of the kernel's inlined normalisations.
const TINY: f64 = 1e-300;

impl Xf {
    pub const IDENTITY: Xf = Xf { r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], t: [0.0; 3] };

    pub fn translation(t: Vec3) -> Xf {
        Xf { t: [t.x, t.y, t.z], ..Xf::IDENTITY }
    }

    /// func f3910: rotation about the unit `axis` (normalised first) by
    /// `angle` radians (Rodrigues, kernel association).
    pub fn rotation(axis: Vec3, angle: f64) -> Xf {
        let l = ((axis.x * axis.x + axis.y * axis.y) + axis.z * axis.z).sqrt();
        let (x, y, z) = if l > TINY && l.is_finite() {
            let i = 1.0 / l;
            (axis.x * i, axis.y * i, axis.z * i)
        } else {
            (0.0, 0.0, 0.0)
        };
        let c = ck_topo::trig::cos(angle);
        let k = 1.0 - c;
        let s = ck_topo::trig::sin(angle);
        let ky = k * y;
        let zky = z * ky;
        let sx = s * x;
        let kx = k * x;
        let zkx = z * kx;
        let sy = s * y;
        let sz = s * z;
        let ykx = y * kx;
        Xf {
            r: [c + x * kx, ykx - sz, sy + zkx, sz + ykx, c + y * ky, zky - sx, zkx - sy, zky + sx, c + z * (k * z)],
            t: [0.0; 3],
        }
    }

    fn row(&self, i: usize, v: Vec3) -> f64 {
        (v.x * self.r[3 * i] + v.y * self.r[3 * i + 1]) + v.z * self.r[3 * i + 2]
    }

    /// `R·v`.
    pub fn vec(&self, v: Vec3) -> Vec3 {
        Vec3::new(self.row(0, v), self.row(1, v), self.row(2, v))
    }

    /// `R·p + t`.
    pub fn point(&self, p: Vec3) -> Vec3 {
        Vec3::new(self.row(0, p) + self.t[0], self.row(1, p) + self.t[1], self.row(2, p) + self.t[2])
    }

    /// `normalize(R·v)` (func f3928: `sqrt(z² + (x² + y²))`, times `1/len`).
    pub fn unit(&self, v: Vec3) -> Vec3 {
        let w = self.vec(v);
        let l = (w.z * w.z + (w.x * w.x + w.y * w.y)).sqrt();
        if !(l > TINY) || !l.is_finite() {
            return Vec3::ZERO;
        }
        let i = 1.0 / l;
        Vec3::new(w.x * i, w.y * i, w.z * i)
    }

    /// func f3928.
    pub fn frame(&self, f: &Frame) -> Frame {
        Frame { o: self.point(f.o), x: self.unit(f.x), y: self.unit(f.y), z: self.unit(f.z) }
    }

    /// func f3921: `cbrt(|det R|)`.
    pub fn scale(&self) -> f64 {
        let r = &self.r;
        let d = r[2] * (r[3] * r[7] - r[4] * r[6]) + (r[0] * (r[4] * r[8] - r[5] * r[7]) + r[1] * (r[5] * r[6] - r[3] * r[8]));
        d.abs().cbrt()
    }

    /// func f3945: `a·b` (rotation parts only).
    pub fn mul_r(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
        let mut o = [0.0; 9];
        for i in 0..3 {
            for j in 0..3 {
                o[3 * i + j] = (b[j] * a[3 * i] + b[3 + j] * a[3 * i + 1]) + b[6 + j] * a[3 * i + 2];
            }
        }
        o
    }

    /// func f3920: `self ∘ b` (b first): `R = Ra·Rb`, `t = ta + Ra·tb`.
    pub fn compose(&self, b: &Xf) -> Xf {
        let tb = Vec3::new(b.t[0], b.t[1], b.t[2]);
        Xf { r: Xf::mul_r(&self.r, &b.r), t: [self.t[0] + self.row(0, tb), self.t[1] + self.row(1, tb), self.t[2] + self.row(2, tb)] }
    }

    /// As a ck-core [`Xform`] (columns), e.g. for the `place` frame.
    pub fn to_xform(&self) -> Xform {
        let r = &self.r;
        Xform::new(
            Mat3::from_cols(Vec3::new(r[0], r[3], r[6]), Vec3::new(r[1], r[4], r[7]), Vec3::new(r[2], r[5], r[8])),
            Vec3::new(self.t[0], self.t[1], self.t[2]),
        )
    }

    pub fn from_frame(f: &Frame) -> Xf {
        Xf { r: [f.x.x, f.y.x, f.z.x, f.x.y, f.y.y, f.z.y, f.x.z, f.y.z, f.z.z], t: [f.o.x, f.o.y, f.o.z] }
    }

    /// func f3762.
    pub fn curve(&self, c: &ck_geom::curve::Curve) -> ck_geom::curve::Curve {
        use ck_geom::curve::Curve;
        match c {
            Curve::Line { o, d } => Curve::Line { o: self.point(*o), d: self.vec(*d) },
            Curve::Circle { frame, r } => Curve::Circle { frame: self.frame(frame), r: self.scale() * r },
            // GUESS: the kernel ellipse is centre + two axis vectors
            Curve::Ellipse { frame, a, b } => Curve::Ellipse { frame: self.frame(frame), a: *a, b: *b },
            Curve::BSpline(s) => Curve::BSpline(s.map(|p| self.point(p))),
        }
    }

    /// func f3800.
    pub fn surface(&self, s: &ck_geom::surface::Surface) -> ck_geom::surface::Surface {
        use ck_geom::surface::Surface;
        let k = self.scale();
        match s {
            Surface::Plane { frame } => Surface::Plane { frame: self.frame(frame) },
            Surface::Cylinder { frame, r } => Surface::Cylinder { frame: self.frame(frame), r: k * r },
            Surface::Cone { frame, r, semi_angle } => Surface::Cone { frame: self.frame(frame), r: k * r, semi_angle: *semi_angle },
            Surface::Sphere { frame, r } => Surface::Sphere { frame: self.frame(frame), r: k * r },
            Surface::Torus { frame, major, minor } => Surface::Torus { frame: self.frame(frame), major: k * major, minor: k * minor },
            // GUESS: the kernel stores a full axis frame; ours keeps o + unit axis
            Surface::Revolution { profile, axis_o, axis } => {
                Surface::Revolution { profile: Box::new(self.curve(profile)), axis_o: self.point(*axis_o), axis: self.unit(*axis) }
            }
            Surface::Extrusion { profile, dir } => Surface::Extrusion { profile: Box::new(self.curve(profile)), dir: self.vec(*dir) },
            Surface::BSpline(b) => Surface::BSpline(Box::new(b.map(|p| self.point(p)))),
            Surface::Offset { base, dist } => Surface::Offset { base: Box::new(self.surface(base)), dist: k * dist },
        }
    }

    /// func f3499 on a whole body.
    pub fn body(&self, b: &mut Body) {
        let all = Span3 {
            vertices: (0, b.vertices.len()),
            edges: (0, b.edges.len()),
            faces: (0, b.faces.len()),
            ..Span3::default()
        };
        self.range(b, &all);
    }

    /// func f3499 on a component's entity ranges.
    pub fn range(&self, b: &mut Body, r: &Span3) {
        for v in &mut b.vertices[r.vertices.0..r.vertices.1] {
            v.p = self.point(v.p);
        }
        for e in &mut b.edges[r.edges.0..r.edges.1] {
            e.curve = self.curve(&e.curve);
        }
        for f in &mut b.faces[r.faces.0..r.faces.1] {
            f.surface = self.surface(&f.surface);
        }
    }
}
