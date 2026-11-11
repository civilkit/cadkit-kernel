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
//!   error map order), no component; features but no body, and an empty
//!   feature list, are both a quiet no-op (no error, no component);
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
    let opts = RegenOpts { ignore_rollback: true, ..RegenOpts::default() };
    let r = regen::regen(part, &opts, &mut ex);
    let n = part.features.len();
    let span = super::Span { label: format!("regenerate {n} features"), children: ex.sink.roots };
    let mut errors = r.errors;
    errors.sort_by(|a, b| a.feature.cmp(&b.feature));
    (span, Nested { body: ex.brep.body, errors, n_features: n })
}

/// A face reference: the exact id, else the first face whose id minus its
/// split index `~k` equals it (GUESS: the stepper mates name
/// `F3.S.e41@F2.1` while the oracle body only has `F3.S.e41~1@F2.1`..
/// `~48`, and the mate resolves).
pub fn find_ref(b: &Body, id: &str) -> Option<ck_topo::body::FaceId> {
    b.find_face(id).or_else(|| {
        b.face_ids().find(|&f| {
            let mut n = b.face(f).name.clone();
            n.region.is_some() && {
                n.region = None;
                n.id() == id
            }
        })
    })
}

/// Error codes that only mean "not rebuilt yet" (see [`Brep::exec_insert`]).
pub fn is_rebuild_gap(code: &str) -> bool {
    code == "notImplemented" || code == "blendUnsupported"
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
    /// Mates so far, in feature order.
    pub mates: Vec<MateRec>,
    /// Nested regen results by part text (filled by the pre-pass or lazily).
    pub cache: Vec<(String, Nested)>,
    /// Mates rejected by the solver (conflicting), `(mate id, mates it
    /// conflicted with)`, in feature order (the `assembly.conflicts` report).
    /// The flag: the rejected mate was of an extended kind (oracle kinds
    /// only surface it when the report is asked for).
    pub conflicts: Vec<(String, Vec<String>, bool)>,
}

impl Assembly {
    /// No components, mates, ranges or nested results.
    pub fn is_empty(&self) -> bool {
        self.components.is_empty()
            && self.face_ranges.is_empty()
            && self.edge_ranges.is_empty()
            && self.mates.is_empty()
            && self.cache.is_empty()
            && self.conflicts.is_empty()
    }
}

impl Brep {
    /// Pre-pass (oracle trace): >= 2 distinct inserted parts are regenerated
    /// up front under `"<n> inserted parts"`.
    pub fn prepare(&mut self, _state: &RegenState, features: &[&Feature]) {
        let mut keys: Vec<(String, &Part)> = vec![];
        for f in features {
            if let FeatureKind::InsertPart { part: Some(p), .. } = &f.kind {
                let k = part_key(p);
                if !keys.iter().any(|(x, _)| *x == k) && !self.asm.cache.iter().any(|(x, _)| *x == k) {
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
        // REBUILD-ONLY: errors of operations not reconstructed yet (ck-ops
        // gaps: chamfer, sweep, curved-edge fillets) do not abort the insert
        // so assemblies can be measured; the oracle never produces them.
        if let Some(e) = nested.errors.iter().find(|e| !is_rebuild_gap(&e.error.code)) {
            let detail = format!("{}: {}", e.feature, e.error.code);
            return Err(FeatureError::new(INSERT_PART_ERRORS).with("detail", Json::Str(detail)));
        }
        let Some(mut b) = nested.body else {
            // features but no solid body: the part is empty (145 of SELENE's are empty by
            // design), and an empty feature list reached the same place above. Quiet no-op,
            // no component.
            return Ok(());
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

    /// The assembly DOF report (`None` without extended mates unless
    /// `force`); also writes the joint DOF into the components.
    pub fn finalize_assembly(&mut self, force: bool) -> Option<Json> {
        super::mate_kinds::finalize(self, force)
    }

    /// Component owning the face named `id`.
    pub fn component_of(&self, id: &str) -> Option<usize> {
        let b = self.body.as_ref()?;
        let f = find_ref(b, id)?;
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

/// A recorded mate: the moving component and the faces.
#[derive(Debug, Clone)]
pub struct MateRec {
    pub id: String,
    pub kind: super::MateKind,
    /// component index that moves
    pub mover: usize,
    /// face on the mover / on the reference side
    pub moving: String,
    pub target: String,
    pub distance: f64,
    pub flip: bool,
    /// Extended kinds only ([`super::mate_kinds`]).
    pub ext: Option<Box<super::mate_kinds::Ext>>,
}

/// Mate geometry of a face: plane (point, outward normal) or axis (point, dir).
#[derive(Debug, Clone, Copy)]
enum Geo {
    Plane { o: Vec3, n: Vec3 },
    Axis { o: Vec3, d: Vec3 },
}

/// Kernel normalisation (`sqrt(z² + (x² + y²))`, times `1/len`).
pub fn knorm(v: Vec3) -> Vec3 {
    let l = (v.z * v.z + (v.x * v.x + v.y * v.y)).sqrt();
    if !(l > TINY) || !l.is_finite() {
        return Vec3::ZERO;
    }
    let i = 1.0 / l;
    Vec3::new(v.x * i, v.y * i, v.z * i)
}

/// Outward normal of a plane as the kernel derives it: `normalize(x × y)`
/// (pinned: after a solved rotation the target normal x-component is
/// `r12·r20 − r22·r10`, not the transformed frame z), negated when the
/// face is reversed.
pub fn plane_normal(fr: &Frame, sense: bool) -> Vec3 {
    let (x, y) = (fr.x, fr.y);
    let n = knorm(Vec3::new(x.y * y.z - x.z * y.y, x.z * y.x - x.x * y.z, x.x * y.y - x.y * y.x));
    if sense { n } else { -n }
}

/// Directions are re-normalised (pinned: a cylinder turned 170° about x
/// gives the record axis `(0, −0.984807753012208, 0.17364817766693028)`,
/// i.e. its transformed frame z normalised once more).
fn face_geo(b: &Body, id: &str) -> Option<Geo> {
    use ck_geom::surface::Surface;
    let f = find_ref(b, id)?;
    match &b.face(f).surface {
        Surface::Plane { frame } => Some(Geo::Plane { o: frame.o, n: plane_normal(frame, b.face(f).sense) }),
        Surface::Cylinder { frame, .. } => Some(Geo::Axis { o: frame.o, d: knorm(frame.z) }),
        // GUESS: cones also give their axis
        Surface::Cone { frame, .. } => Some(Geo::Axis { o: frame.o, d: knorm(frame.z) }),
        _ => None,
    }
}

/// Records of one mate for its mover (probes: coincident = normals opposed
/// (`flip` -> aligned) + point on plane; distance = the same at `d`;
/// parallel = the direction record only; concentric = axis directions
/// aligned with the sign of their current dot + point on axis).
pub fn mate_recs(b: &Body, m: &MateRec) -> Result<Vec<Rec>, FeatureError> {
    use super::MateKind as K;
    let geo = |id: &str| face_geo(b, id).ok_or_else(|| FeatureError::new(MATE_GEOMETRY));
    let (gm, gt) = (geo(&m.moving)?, geo(&m.target)?);
    let s = if m.flip { 1.0 } else { -1.0 };
    Ok(match (m.kind, gm, gt) {
        (K::Coincident | K::Distance, Geo::Plane { o: p, n: a }, Geo::Plane { o: q, n }) => {
            let d = if m.kind == K::Distance { m.distance } else { 0.0 };
            vec![Rec::Dir { a, b: n, s }, Rec::PtPlane { p, q, n, d }]
        }
        (K::Parallel, Geo::Plane { n: a, .. }, Geo::Plane { n, .. }) => vec![Rec::Dir { a, b: n, s }],
        (K::Concentric, Geo::Axis { o: p, d: a }, Geo::Axis { o: q, d: u }) => {
            // GUESS: flip ignored; sign from the current alignment
            let dot = a.dot(u);
            vec![Rec::Dir { a, b: u, s: if dot < 0.0 { -1.0 } else { 1.0 } }, Rec::PtLine { p, q, u }]
        }
        _ => return Err(FeatureError::new(MATE_GEOMETRY)),
    })
}

/// func f3769: box of the arc `c + cos t·X + sin t·Y`, `t ∈ [t0, t1]`
/// (end points + the per-axis critical angles `atan2(Y_i, X_i)` (+π)).
fn arc_box(c: Vec3, x: Vec3, y: Vec3, t0: f64, t1: f64) -> (Vec3, Vec3) {
    let tau = std::f64::consts::TAU;
    let pt0 = |t: f64| {
        let (s, co) = (ck_topo::trig::sin(t), ck_topo::trig::cos(t));
        Vec3::new((c.x + co * x.x) + s * y.x, (c.y + co * x.y) + s * y.y, (c.z + co * x.z) + s * y.z)
    };
    let (a, b) = (pt0(t0), pt0(t1));
    let (mut lo, mut hi) = (a.min(b), a.max(b));
    for (xi, yi) in [(x.x, y.x), (x.y, y.y), (x.z, y.z)] {
        if xi.abs() < TINY && yi.abs() < TINY {
            continue;
        }
        let base = yi.atan2(xi);
        for cand in [base, base + std::f64::consts::PI] {
            let mut d = (cand - t0) % tau;
            if d < 0.0 {
                d += tau;
            }
            let t = t0 + if d >= tau { 0.0 } else { d };
            if t <= t1 {
                let (s, co) = (ck_topo::trig::sin(t), ck_topo::trig::cos(t));
                let p = Vec3::new(y.x * s + (c.x + x.x * co), y.y * s + (c.y + x.y * co), y.z * s + (c.z + x.z * co));
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
    }
    (lo, hi)
}

/// func f3768 (lines, circles; GUESS: other curves from their sampled
/// points).
fn edge_box(e: &ck_topo::body::Edge) -> (Vec3, Vec3) {
    use ck_geom::curve::Curve;
    match &e.curve {
        Curve::Line { o, d } => {
            let p = |t: f64| Vec3::new(o.x + t * d.x, o.y + t * d.y, o.z + t * d.z);
            let (a, b) = (p(e.t0), p(e.t1));
            (a.min(b), a.max(b))
        }
        Curve::Circle { frame, r } => arc_box(frame.o, frame.x.scale(*r), frame.y.scale(*r), e.t0, e.t1),
        Curve::Ellipse { frame, a, b } => arc_box(frame.o, frame.x.scale(*a), frame.y.scale(*b), e.t0, e.t1),
        c => {
            let pts = c.sample(e.t0, e.t1, 64);
            let mut lo = pts[0];
            let mut hi = pts[0];
            for q in pts {
                lo = lo.min(q);
                hi = hi.max(q);
            }
            (lo, hi)
        }
    }
}

/// Bounds of a component: the union of its face boxes (func f3512 = the
/// surface box over the face mesh's uv bounds ∪ every edge box padded by
/// the edge tolerance 1e-6). Pinned: a 40×10×20 box solves with scale
/// 22.912880002304444 = half the diagonal grown by 1e-6 per side. GUESS:
/// the surface-box part is skipped (it lies inside the edge boxes for the
/// planar / cylindrical faces of the samples).
pub fn comp_box(b: &Body, r: &Span3) -> Option<(Vec3, Vec3)> {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = -lo;
    let tol = 1e-6;
    for fi in r.faces.0..r.faces.1 {
        let f = ck_topo::body::FaceId(fi as u32);
        if !b.face(f).alive {
            continue;
        }
        for e in b.face_edges(f) {
            let (a, z) = edge_box(b.edge(e));
            if a.x <= z.x && a.y <= z.y && a.z <= z.z {
                lo = lo.min(Vec3::new(a.x - tol, a.y - tol, a.z - tol));
                hi = hi.max(Vec3::new(z.x + tol, z.y + tol, z.z + tol));
            }
        }
    }
    (lo.x <= hi.x).then_some((lo, hi))
}

impl Assembly {
    pub fn recs_for(&self, b: &Body, k: usize) -> Result<Vec<Rec>, FeatureError> {
        let mut out = vec![];
        for m in self.mates.iter().filter(|m| m.mover == k) {
            if m.ext.is_some() {
                out.extend(super::mate_kinds::recs_ext(self, b, m)?);
            } else {
                out.extend(mate_recs(b, m)?);
            }
        }
        Ok(out)
    }
}

/// func f1375: Gauss-Seidel over the components (the new mate's mover
/// first, then the others in insertion order), each solved against its
/// own mates with everything else frozen, until a pass moves nothing.
pub(super) fn solve_all(br: &mut Brep, first: usize) -> Result<(), FeatureError> {
    let n = br.asm.components.len();
    let order: Vec<usize> = std::iter::once(first).chain((0..n).filter(|&k| k != first)).collect();
    let mut moves = 0usize;
    loop {
        let mut changed = false;
        for &k in &order {
            if br.asm.components[k].fixed {
                continue;
            }
            let mut moved = solve_one(br, k)?;
            if br.asm.mates.iter().any(|m| m.mover == k && m.ext.is_some()) {
                moved |= super::mate_kinds::apply_posts(br, k)?;
            }
            if moved {
                changed = true;
                moves += 1;
                if moves > 8 * n {
                    return Err(FeatureError::new(MATE_CONFLICT));
                }
            }
        }
        if !changed {
            return Ok(());
        }
    }
}

/// One component against its own mates (everything else frozen); `true`
/// when it moved.
fn solve_one(br: &mut Brep, k: usize) -> Result<bool, FeatureError> {
    let body = br.body.as_ref().unwrap();
    let recs = br.asm.recs_for(body, k)?;
    if recs.is_empty() {
        return Ok(false);
    }
    let range = br.asm.components[k].range.clone();
    let bx = comp_box(body, &range);
    DEBUG.with(|d| d.borrow_mut().push(format!("box {:?}", bx)));
    let (c, scale) = match bx {
        Some((lo, hi)) => {
            let (ex, ey, ez) = (hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
            (Vec3::new((hi.x + lo.x) * 0.5, (hi.y + lo.y) * 0.5, (hi.z + lo.z) * 0.5), (ez * ez + (ex * ex + ey * ey)).sqrt() * 0.5)
        }
        None => (Vec3::ZERO, 0.0),
    };
    DEBUG.with(|d| d.borrow_mut().push(format!("solve comp {k} center {:?} scale {:?} recs {:?}", c, scale, recs)));
    let sv = solve(&recs, c, scale).ok_or_else(|| FeatureError::new(MATE_CONFLICT))?;
    DEBUG.with(|d| d.borrow_mut().push(format!("  -> iters {} delta {:?}", sv.iterations, sv.x)));
    if sv.iterations == 0 {
        return Ok(false);
    }
    sv.x.range(br.body.as_mut().unwrap(), &range);
    let comp = &mut br.asm.components[k];
    comp.place = sv.x.compose(&comp.place);
    Ok(true)
}

/// Numeric rank of the rows (rotation columns scaled like the solver).
/// GUESS: Gaussian elimination, pivots below 1e-8 · max count as zero.
fn rank(rows: &[Row], l8: f64) -> usize {
    let mut m: Vec<[f64; 6]> = rows.iter().map(|r| [r.j[0] / l8, r.j[1] / l8, r.j[2] / l8, r.j[3], r.j[4], r.j[5]]).collect();
    let amax = m.iter().flat_map(|r| r.iter()).fold(0.0f64, |a, v| a.max(v.abs()));
    if amax == 0.0 {
        return 0;
    }
    let tol = amax * 1e-8;
    let mut rk = 0;
    for col in 0..6 {
        let Some(p) = (rk..m.len()).max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs())) else { break };
        if m[p][col].abs() <= tol {
            continue;
        }
        m.swap(rk, p);
        for i in rk + 1..m.len() {
            let f = m[i][col] / m[rk][col];
            for j in col..6 {
                m[i][j] -= f * m[rk][j];
            }
        }
        rk += 1;
    }
    rk
}

/// Remaining DOF of every component (GUESS: 6 − rank of its mate rows at
/// the solved position; fixed = 0).
pub(super) fn update_dof(br: &mut Brep) {
    if br.asm.mates.iter().any(|m| m.ext.is_some()) {
        // joint DOF is computed once by `mate_kinds::finalize`
        return;
    }
    let Some(body) = br.body.as_ref() else { return };
    for k in 0..br.asm.components.len() {
        let c = &br.asm.components[k];
        if c.fixed {
            continue;
        }
        let Ok(recs) = br.asm.recs_for(body, k) else { continue };
        let mut rs = vec![];
        let (cen, scale) = match comp_box(body, &c.range) {
            Some((lo, hi)) => ((lo + hi).scale(0.5), lo.dist(hi) * 0.5),
            None => (Vec3::ZERO, 0.0),
        };
        for r in &recs {
            rows(r, &Xf::IDENTITY.r, Vec3::ZERO, cen, &mut rs);
        }
        let dof = 6 - rank(&rs, scale.max(1.0)).min(6);
        br.asm.components[k].dof = dof as u32;
    }
}

/// `mate`.
pub fn exec_mate(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Mate { kind, a, b, flip, .. } = &f.kind else { unreachable!() };
    if br.body.is_none() || br.asm.components.is_empty() {
        return Err(FeatureError::new(regen::codes::NOTHING_TO_CUT));
    }
    if kind.is_extended() {
        return super::mate_kinds::exec_ext(br, state, f);
    }
    let (ra, rb) = (a.as_ref().map(|r| r.as_str().to_string()), b.as_ref().map(|r| r.as_str().to_string()));
    let ca = ra.as_deref().and_then(|r| br.component_of(r));
    let cb = rb.as_deref().and_then(|r| br.component_of(r));
    let (Some(ca), Some(cb)) = (ca, cb) else { return Err(FeatureError::new(regen::codes::MISSING_REFERENCE)) };
    if ca == cb {
        return Err(FeatureError::new(MATE_SAME_BODY));
    }
    let (fa, fb) = (br.asm.components[ca].fixed, br.asm.components[cb].fixed);
    if fa && fb {
        return Err(FeatureError::new(MATE_FIXED));
    }
    let distance = state.num(f, "distance", 0.0)?;
    let (mover, moving, target) = if fa { (cb, rb.unwrap(), ra.unwrap()) } else { (ca, ra.unwrap(), rb.unwrap()) };
    let m = MateRec { id: f.id.clone(), kind: *kind, mover, moving, target, distance, flip: *flip, ext: None };
    // validate the geometry before keeping it
    mate_recs(br.body.as_ref().unwrap(), &m)?;
    br.asm.mates.push(m);
    let r = solve_all(br, mover);
    if r.is_err() {
        let new = br.asm.mates.pop().unwrap();
        if let Some(tc) = br.component_of(&new.target) {
            super::mate_kinds::record_conflict(br, &new, tc);
        }
    }
    update_dof(br);
    r
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
            // GUESS: analytic carrier frames transform like ellipse frames
            Curve::Conic { frame, p, e } => Curve::Conic { frame: self.frame(frame), p: self.scale() * p, e: *e },
            Curve::Sinusoid { frame, amp, freq, phase } => {
                Curve::Sinusoid { frame: self.frame(frame), amp: self.scale() * amp, freq: *freq, phase: *phase }
            }
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
            Surface::Screw { profile, axis_o, axis, rise, urange, vrange } => Surface::Screw {
                profile: Box::new(self.curve(profile)),
                axis_o: self.point(*axis_o),
                axis: self.unit(*axis),
                rise: k * rise,
                urange: *urange,
                vrange: *vrange,
            },
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

// ------------------------------------------------------------ mate solver

thread_local! {
    /// Solver trace for native debugging (`examples/asm_debug.rs`).
    pub static DEBUG: std::cell::RefCell<Vec<String>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// One constraint record of the moving component (kernel 352-byte records,
/// kinds decoded from func f1352). Vectors are world-space at the time the
/// record was built; the trial motion is `p ↦ R(p − c) + c + t`.
#[derive(Debug, Clone, Copy)]
pub enum Rec {
    /// kind 0: `R·a = s·b` (unit directions; 3 angular rows).
    Dir { a: Vec3, b: Vec3, s: f64 },
    /// kind 2: `n·(p' − q) = d` (1 positional row).
    PtPlane { p: Vec3, q: Vec3, n: Vec3, d: f64 },
    /// kind 4: `p'` on the line `q + λu` (3 positional rows).
    PtLine { p: Vec3, q: Vec3, u: Vec3 },
    /// extended: `(R·a)·b = c` (1 angular row).
    DirDot { a: Vec3, b: Vec3, c: f64 },
    /// extended: `|p' − q| = d` (1 positional row).
    PtDist { p: Vec3, q: Vec3, d: f64 },
    /// extended: distance of `p'` from the line `q + λu` is `d`.
    LineDist { p: Vec3, q: Vec3, u: Vec3, d: f64 },
    /// extended: the moving plane `(p, n)` and the fixed point `q`:
    /// `(R·n)·(q − p') = d`.
    PlanePt { p: Vec3, n: Vec3, q: Vec3, d: f64 },
}

impl Rec {
    /// kernel kind byte (`<= 1` = direction records, used to perturb).
    fn kind(&self) -> u8 {
        match self {
            Rec::Dir { .. } => 0,
            Rec::PtPlane { .. } => 2,
            Rec::PtLine { .. } => 4,
            Rec::DirDot { .. } => 1,
            Rec::PtDist { .. } => 3,
            Rec::LineDist { .. } => 5,
            Rec::PlanePt { .. } => 6,
        }
    }
    fn first(&self) -> Vec3 {
        match *self {
            Rec::Dir { a, .. } | Rec::DirDot { a, .. } => a,
            Rec::PtPlane { p, .. } | Rec::PtLine { p, .. } | Rec::PtDist { p, .. } | Rec::LineDist { p, .. } | Rec::PlanePt { p, .. } => p,
        }
    }
}

/// A Jacobian row: residual, `∂/∂ω` (3), `∂/∂t` (3).
#[derive(Debug, Clone, Copy)]
pub struct Row {
    pub r: f64,
    pub j: [f64; 6],
}

fn rdot(r: &[f64; 9], i: usize, v: Vec3) -> f64 {
    (r[3 * i] * v.x + r[3 * i + 1] * v.y) + r[3 * i + 2] * v.z
}

/// func f1352 for the decoded kinds: push the rows, return
/// `(angular error, positional error)`.
pub fn rows(rec: &Rec, r: &[f64; 9], t: Vec3, c: Vec3, out: &mut Vec<Row>) -> (f64, f64) {
    match *rec {
        Rec::Dir { a, b, s } => {
            let l16 = rdot(r, 0, a);
            let l14 = rdot(r, 2, a);
            let l24 = rdot(r, 1, a);
            let r1 = l16 - b.x * s;
            let r2 = l24 - b.y * s;
            let r3 = l14 - b.z * s;
            out.push(Row { r: r1, j: [-0.0, l14, -l24, 0.0, 0.0, 0.0] });
            out.push(Row { r: r2, j: [-l14, -0.0, l16, 0.0, 0.0, 0.0] });
            out.push(Row { r: r3, j: [l24, -l16, -0.0, 0.0, 0.0, 0.0] });
            (((r1 * r1) + (r2 * r2) + (r3 * r3)).sqrt(), 0.0)
        }
        Rec::PtPlane { p, q, n, d } => {
            let dv = Vec3::new(p.x - c.x, p.y - c.y, p.z - c.z);
            let l15 = rdot(r, 0, dv);
            let l16 = rdot(r, 1, dv);
            let l24 = rdot(r, 2, dv);
            let (l22, l14, l23) = (n.x, n.y, n.z);
            let l28 = (l14 * l15) - (l22 * l16);
            let l25 = (l22 * l24) - (l23 * l15);
            let l31 = (l23 * l16) - (l14 * l24);
            let res = ((l23 * ((t.z + (c.z + l24)) - q.z)) + ((l22 * ((t.x + (c.x + l15)) - q.x)) + (l14 * ((t.y + (c.y + l16)) - q.y)))) - d;
            out.push(Row { r: res, j: [l31, l25, l28, l22, l14, l23] });
            (0.0, res.abs())
        }
        Rec::DirDot { a, b, c: cc } => {
            let ra = Vec3::new(rdot(r, 0, a), rdot(r, 1, a), rdot(r, 2, a));
            let res = ra.dot(b) - cc;
            let g = ra.cross(b);
            out.push(Row { r: res, j: [g.x, g.y, g.z, 0.0, 0.0, 0.0] });
            (res.abs(), 0.0)
        }
        Rec::PtDist { p, q, d } => {
            let w = Vec3::new(rdot(r, 0, p - c), rdot(r, 1, p - c), rdot(r, 2, p - c));
            let v = Vec3::new(t.x + c.x + w.x - q.x, t.y + c.y + w.y - q.y, t.z + c.z + w.z - q.z);
            let rho = v.len();
            let m = if rho > 1e-12 { v.scale(1.0 / rho) } else { Vec3::new(1.0, 0.0, 0.0) };
            let g = w.cross(m);
            let res = rho - d;
            out.push(Row { r: res, j: [g.x, g.y, g.z, m.x, m.y, m.z] });
            (0.0, res.abs())
        }
        Rec::LineDist { p, q, u, d } => {
            let w = Vec3::new(rdot(r, 0, p - c), rdot(r, 1, p - c), rdot(r, 2, p - c));
            let v = Vec3::new(t.x + c.x + w.x - q.x, t.y + c.y + w.y - q.y, t.z + c.z + w.z - q.z);
            let vp = v - u.scale(v.dot(u));
            let rho = vp.len();
            let m = if rho > 1e-12 { vp.scale(1.0 / rho) } else { u.any_perp().normalize() };
            let g = w.cross(m);
            let res = rho - d;
            out.push(Row { r: res, j: [g.x, g.y, g.z, m.x, m.y, m.z] });
            (0.0, res.abs())
        }
        Rec::PlanePt { p, n, q, d } => {
            let w = Vec3::new(rdot(r, 0, p - c), rdot(r, 1, p - c), rdot(r, 2, p - c));
            let np = Vec3::new(rdot(r, 0, n), rdot(r, 1, n), rdot(r, 2, n));
            let pp = Vec3::new(t.x + c.x + w.x, t.y + c.y + w.y, t.z + c.z + w.z);
            let qv = q - pp;
            let res = np.dot(qv) - d;
            let g = np.cross(qv + w);
            out.push(Row { r: res, j: [g.x, g.y, g.z, -np.x, -np.y, -np.z] });
            (0.0, res.abs())
        }
        Rec::PtLine { p, q, u } => {
            let (l25, l15, l17) = (p.x - c.x, p.y - c.y, p.z - c.z);
            let dv = Vec3::new(l25, l15, l17);
            let l14 = rdot(r, 0, dv);
            let (l23, l24) = (u.x, u.y);
            let l16 = 0.0 - (l23 * l24);
            let l22 = rdot(r, 1, dv);
            let l27 = 1.0 - (l23 * l23);
            let l33 = (l14 * l16) - (l22 * l27);
            let l25 = rdot(r, 2, dv);
            let l15 = u.z;
            let l34 = l23 * l15;
            let l26 = 0.0 - l34;
            let l35 = (l25 * l27) - (l14 * l26);
            let l36 = l25 * l16;
            let l37 = (l22 * l26) - l36;
            let l32 = (t.z + (c.z + l25)) - q.z;
            let l28 = (t.x + (c.x + l14)) - q.x;
            let l31 = (t.y + (c.y + l22)) - q.y;
            let l17 = (l15 * l32) + ((l23 * l28) + (l24 * l31));
            let l38 = l24 * l17;
            let rx = l28 - (l23 * l17);
            let l28b = l24 * l15;
            let l24b = 1.0 - (l24 * l24);
            let l39 = l15 * l15;
            let l40 = l15 * l17;
            let ry = l31 - l38;
            let l17b = 1.0 - l39;
            out.push(Row { r: rx, j: [l37, l35, l33, l27, l16, l26] });
            let l31b = (l14 * l24b) - (l22 * l16);
            let l26b = 0.0 - l28b;
            let l33b = l36 - (l14 * l26b);
            let l35b = (l22 * l26b) - (l25 * l24b);
            let rz = l32 - l40;
            out.push(Row { r: ry, j: [l35b, l33b, l31b, l16, l24b, l26b] });
            let l24c = 0.0 - l28b;
            let l16c = 0.0 - l34;
            let l26c = (l14 * l24c) - (l22 * l16c);
            let l14c = (l25 * l16c) - (l14 * l17b);
            let l22c = (l22 * l17b) - (l25 * l24c);
            out.push(Row { r: rz, j: [l22c, l14c, l26c, l16c, l24c, l17b] });
            (0.0, ((rz * rz) + ((rx * rx) + (ry * ry))).sqrt())
        }
    }
}

/// Outcome of [`solve`]: the delta transform and the iteration count
/// (0 = already satisfied).
#[derive(Debug, Clone, Copy)]
pub struct Solved {
    pub x: Xf,
    pub iterations: u32,
}

fn fmax(a: f64, b: f64) -> f64 {
    a.max(b)
}

/// func f1351: damped Gauss-Newton (LM diagonal 1e-4 rotation / 1e-10
/// translation of the largest diagonal entry) on one rigid body with
/// numeric-free analytic rows; rotation columns scaled by `1/max(scale,1)`;
/// steps capped at 0.5 rad; a stalled start is kicked by 0.001 rad once;
/// at most 200 iterations. Converged: positional ≤ 1e-7, angular ≤ 1e-9.
pub fn solve(recs: &[Rec], c: Vec3, scale: f64) -> Option<Solved> {
    let scale = if scale.is_nan() { 1.0 } else { scale };
    let l8 = if scale > 1.0 { scale } else { 1.0 };
    let l9 = 1.0 / l8;
    let mut r = Xf::IDENTITY.r;
    let mut t = Vec3::ZERO;
    let mut perturbed = false;
    let mut iter = 0u32;
    loop {
        let mut rs: Vec<Row> = vec![];
        let (mut pos, mut ang) = (0.0f64, 0.0f64);
        for rec in recs {
            let (a, p) = rows(rec, &r, t, c, &mut rs);
            pos = fmax(pos, p);
            ang = fmax(ang, a);
        }
        if pos <= 1e-7 && ang <= 1e-9 {
            let tx = Vec3::new((c.x - rdot(&r, 0, c)) + t.x, (c.y - rdot(&r, 1, c)) + t.y, (c.z - rdot(&r, 2, c)) + t.z);
            return Some(Solved { x: Xf { r, t: [tx.x, tx.y, tx.z] }, iterations: iter });
        }
        let mut a = [[0.0f64; 6]; 6];
        let mut b = [0.0f64; 6];
        for row in &rs {
            let j = [row.j[0] / l8, row.j[1] / l8, row.j[2] / l8, row.j[3], row.j[4], row.j[5]];
            for i in 0..6 {
                for k in 0..6 {
                    a[i][k] += j[i] * j[k];
                }
                b[i] -= j[i] * row.r;
            }
        }
        let mut g = 0.0f64;
        for v in b {
            g = fmax(g, v.abs());
        }
        iter += 1;
        let mut m = 0.0f64;
        for i in 0..6 {
            m = fmax(m, a[i][i]);
        }
        let m = fmax(m, f64::MIN_POSITIVE);
        for i in 3..6 {
            a[i][i] += m * 1e-10;
        }
        for i in 0..3 {
            a[i][i] += m * 1e-4;
        }
        if g <= (pos + ang) * 1e-12 {
            if perturbed {
                return None;
            }
            let v = match recs.iter().find(|q| q.kind() <= 1) {
                Some(q) => {
                    let f = q.first();
                    Vec3::new(rdot(&r, 0, f), rdot(&r, 1, f), rdot(&r, 2, f))
                }
                None => Vec3::new(0.0, 0.0, 1.0),
            };
            let rot = Xf::rotation(perp(v), 0.001);
            r = Xf::mul_r(&rot.r, &r);
            perturbed = true;
            if iter < 200 {
                continue;
            }
            return None;
        }
        let mut am: Vec<Vec<f64>> = a.iter().map(|row| row.to_vec()).collect();
        let mut bv = b.to_vec();
        let x = crate::sketch::solver::lu_solve(&mut am, &mut bv)?;
        let (mut w0, mut w1, mut w2) = (l9 * x[0], l9 * x[1], l9 * x[2]);
        let mut ww = (w0 * w0 + w1 * w1) + w2 * w2;
        let n = ww.sqrt();
        if n > 0.5 {
            let k = 0.5 / n;
            w0 *= k;
            w1 *= k;
            w2 *= k;
            ww = (w0 * w0 + w1 * w1) + w2 * w2;
        }
        if ww > 0.0 {
            let an = ww.sqrt();
            let i = 1.0 / an;
            let rot = Xf::rotation(Vec3::new(w0 * i, w1 * i, w2 * i), an);
            r = Xf::mul_r(&rot.r, &r);
        }
        t = Vec3::new(x[3] + t.x, x[4] + t.y, x[5] + t.z);
        if iter == 200 {
            return None;
        }
    }
}

/// func f3907: unit `v × e` with `e` the axis of the smallest |component|.
fn perp(v: Vec3) -> Vec3 {
    let (ax, ay, az) = (v.x.abs(), v.y.abs(), v.z.abs());
    let p1 = ax <= ay && ax <= az;
    let l8 = ay <= az;
    let ey = if l8 { if p1 { 0.0 } else { 1.0 } } else { 0.0 };
    let ex = if p1 { 1.0 } else { 0.0 };
    let ez = if p1 { 0.0 } else if l8 { 0.0 } else { 1.0 };
    let l3 = (v.x * ey) - (v.y * ex);
    let l2 = (v.z * ex) - (v.x * ez);
    let l4 = (v.y * ez) - (v.z * ey);
    let l = ((l3 * l3) + ((l2 * l2) + (l4 * l4))).sqrt();
    if !(l > TINY) || !l.is_finite() {
        return Vec3::ZERO;
    }
    let i = 1.0 / l;
    Vec3::new(l4 * i, l2 * i, l3 * i)
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
