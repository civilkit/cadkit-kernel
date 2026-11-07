//! ck-swedit: an editable feature history for an imported SolidWorks part.
//!
//! The part file gives the exact body (`ck-sldprt`), the feature list with its types, order and dimension values
//! (`swXmlContents/KeyWords`, `ck_sldprt::meta`) and, on every face, the id of the feature that made it
//! (`FACE_ID_2001`). Nothing of the sketches needs decoding: for each feature, in tree order, the faces it left on
//! the final body are read back into the feature's own parameters:
//!
//! * extrude (boss or cut): the faces give the direction, the extent along it and the profile, taken from the
//!   outline of a cap face when the cap's rim is untouched by later features, else from the cycle of the side faces;
//! * fillet / chamfer: every strip face between two support faces gives one edge pick (the two supports plus a
//!   point on the sharp edge they would meet at) and the radius / distance;
//! * hole wizard: the hole groups of `ck-recog` restricted to the feature's faces;
//! * shell: the faces whose rim borders the shell's inner faces are the removed ones.
//!
//! Each step is emitted as a feature of our own part document (plane, sketch, extrude, fillet, chamfer, hole,
//! shell); the document is regenerated and the result compared with the imported body (`ck_recog::rebuild::verify`).
//! A feature that cannot be read back, a regen error or a failed verification stops the rebuild there: the steps
//! before it that pass their local checks stay editable and an `importBody` feature puts the file's own body in place
//! of theirs (`prefix.rs`, the remainder); with no such step the file keeps its plain solid. The reason is reported
//! per feature.

pub mod blend;
pub mod draft;
pub mod fastverify;
pub mod hole;
pub mod imported;
pub mod loft;
mod reorder;
mod opening;
pub mod moveface;
pub mod pattern;
pub mod prefix;
pub mod profile;
pub mod revolve;
pub mod session;
pub mod shell;
pub mod sweep;
mod swap;
mod hoist;
pub mod later_draft;
pub mod rib;
pub mod thread;
pub mod undo;
mod unblend;
mod unmirror;

use ck_geom::Vec3;
use ck_json::{Json, Obj};
use ck_recog::emit::{n, Tree};
use ck_recog::model::{principal, Kind, Model, PlaneSpec};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::{Body, FaceId};
use std::collections::HashMap;

/// Coincidence tolerance (mm).
pub const TOL: f64 = 1e-5;

/// The imported body with the SolidWorks feature id of every face and the feature list.
pub struct Tagged<'a> {
    pub body: &'a Body,
    /// Per face index (`FaceId::idx`): the (feature id, kind code) tags the face carries. A face two features share
    /// (a cut wall flush with a side, a side a later boss extended) carries one tag per feature. The kind code
    /// (`FACE_ID_2001[0]`): 40 is an extrude's end cap, 74 a side, 72 a fillet face.
    pub tags: Vec<Vec<(i64, i64)>>,
    pub features: &'a [SwFeature],
    /// Millimetres per display length unit of the file's dimensions (25.4 for an inch document).
    pub unit_mm: f64,
    /// The file's solid as an `importBody` feature `F1` puts it in our document (faces `F1.I<k>F<n>`), for a history
    /// that starts with imported bodies (`undo.rs`); `None` otherwise.
    pub start: Option<std::rc::Rc<Body>>,
}

/// Millimetres per display unit (`SwUnits::length` names: mm, cm, m, in, ft, ...).
pub fn unit_mm(meta: &ck_sldprt::SwMeta) -> f64 {
    match meta.units.as_ref().and_then(|u| u.length.as_deref()) {
        Some("cm") => 10.0,
        Some("m") => 1000.0,
        Some("in") => 25.4,
        Some("ft") => 304.8,
        Some("um") => 1e-3,
        Some("mil") => 0.0254,
        _ => 1.0,
    }
}

/// `FACE_ID_2001` kind code of an extrude's end cap (sample census: always a plane).
pub const KIND_CAP: i64 = 40;
/// Tag kind of a face a split line cut out of another feature's face.
pub const KIND_SPLIT: i64 = 62;

fn same_surface_kind(a: &Kind, b: &Kind) -> bool {
    blend::same_surface(a, b)
}

impl<'a> Tagged<'a> {
    pub fn from_import(imp: &'a ck_sldprt::SldprtImport) -> Tagged<'a> {
        let b = &imp.body;
        // face tags follow the order of the built faces (names repeat in a multi-body part, so not by name)
        let mut tags = vec![vec![]; b.faces.len()];
        let mut by_name: HashMap<&str, &ck_sldprt::FaceTag> = HashMap::new();
        for t in &imp.face_tags {
            by_name.entry(t.face.as_str()).or_insert(t);
        }
        let built: Vec<FaceId> = imp.parts.iter().flat_map(|p| p.faces.iter().copied()).collect();
        if built.len() == imp.face_tags.len() {
            for (f, t) in built.iter().zip(&imp.face_tags) {
                tags[f.idx()] = t.sw_ids.iter().map(|id| (id.feature, id.kind)).collect();
            }
        } else {
            for f in b.face_ids() {
                if let Some(t) = by_name.get(b.face_id_str(f).as_str()) {
                    tags[f.idx()] = t.sw_ids.iter().map(|id| (id.feature, id.kind)).collect();
                }
            }
        }
        let start = undo::start_body(imp, &tags);
        Tagged { body: b, tags, features: &imp.meta.features, unit_mm: unit_mm(&imp.meta), start }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Status {
    Rebuilt,
    /// The feature left no face on the final body: nothing to read back (and nothing to rebuild).
    Skipped,
    /// The feature could not be read back; the file falls back to its plain solid from here on.
    Unsupported,
}

#[derive(Debug, Clone)]
pub struct Step {
    pub sw_id: i64,
    pub name: String,
    pub kind: String,
    /// Faces of the final body the feature made.
    pub faces: usize,
    pub status: Status,
    pub note: String,
}

pub struct Outcome {
    /// Our part document's feature list (empty when nothing rebuilt).
    pub features: Vec<Json>,
    pub steps: Vec<Step>,
    pub body: Option<Body>,
    pub verify: Option<ck_recog::rebuild::Verify>,
    /// Regen errors (`feature: code`).
    pub errors: Vec<String>,
    /// Why the history does not rebuild to the end (first unsupported feature, regen error or verification): with
    /// `prefix > 0` the rest is the imported remainder, else the file falls back to its plain solid.
    pub fallback: Option<String>,
    pub notes: Vec<String>,
    /// SolidWorks steps (entries of `steps`) kept as editable features before the imported remainder; 0 = none.
    pub prefix: usize,
    /// What the `importBody` feature that ends `features` stands for (only with `prefix > 0`).
    pub remainder: Option<prefix::Remainder>,
    /// Local checks of the steps, from the first up to the one that failed (only for a history that does not rebuild
    /// to the end).
    pub checks: Vec<prefix::Check>,
    /// Our features of each entry of `steps` (`features[a..b]`; empty for a skipped or unsupported step).
    pub ranges: Vec<(usize, usize)>,
}

impl Outcome {
    /// The rebuild reproduces the imported body (symmetric difference and area below 1e-6 relative).
    pub fn exact(&self) -> bool {
        self.fallback.is_none() && self.verify.as_ref().map_or(false, |v| v.exact)
    }
    pub fn close(&self) -> bool {
        self.fallback.is_none() && self.verify.as_ref().map_or(false, |v| v.close)
    }

    /// The file has an editable history: rebuilt to the end (exact or close) or an editable prefix and the remainder.
    pub fn editable(&self) -> bool {
        self.exact() || self.close() || self.prefix > 0
    }

    /// Our part document: `features`, and with a remainder the `blobs` table holding `file` (the part file, as base64
    /// text) under the key the `importBody` feature names. Meant for an editable outcome ([`Outcome::editable`]).
    pub fn part_json(&self, file: Option<&[u8]>) -> Json {
        let mut feats = self.features.clone();
        let mut blobs = Obj::new();
        // (the remainder that ends the list, or the starting body of an imported part: any `importBody` with no blob yet)
        if let Some(bytes) = file {
            let text = ck_text::base64_encode(bytes);
            let key = ck_model::features::import_step::blob_key(&text);
            let mut used = false;
            for f in feats.iter_mut() {
                if let Json::Obj(kv) = f {
                    if kv.iter().any(|(k, v)| k == "type" && matches!(v, Json::Str(s) if s == "importBody")) {
                        if let Some(v) = kv.iter_mut().find(|(k, _)| k == "blob") {
                            v.1 = Json::Str(key.clone());
                            used = true;
                        }
                    }
                }
            }
            if used {
                blobs = blobs.put(key.as_str(), text.as_str());
            }
        }
        Obj::new().put("format", "ck-part").put("version", 1.0).put("blobs", blobs.build()).put("features", Json::Arr(feats)).build()
    }

    pub fn json(&self) -> Json {
        let steps: Vec<Json> = self
            .steps
            .iter()
            .map(|s| {
                Obj::new()
                    .put("id", s.sw_id as f64)
                    .put("name", s.name.as_str())
                    .put("kind", s.kind.as_str())
                    .put("faces", s.faces as f64)
                    .put("status", format!("{:?}", s.status).to_lowercase().as_str())
                    .put("note", s.note.as_str())
                    .build()
            })
            .collect();
        Obj::new()
            .put("exact", self.exact())
            .put("close", self.close())
            .put("fallback", self.fallback.as_deref().map_or(Json::Null, |x| Json::Str(x.into())))
            .put("prefix", self.prefix as f64)
            .put("remainder", self.remainder.as_ref().map_or(Json::Null, |r| r.json()))
            .put("checks", Json::Arr(self.checks.iter().map(|c| c.json()).collect()))
            .put("steps", Json::Arr(steps))
            .put("features", Json::Arr(self.features.clone()))
            .put("verify", self.verify.as_ref().map_or(Json::Null, |v| v.json()))
            .put("errors", Json::Arr(self.errors.iter().map(|e| Json::Str(e.clone())).collect()))
            .put("notes", Json::Arr(self.notes.iter().map(|e| Json::Str(e.clone())).collect()))
            .build()
    }
}

/// What a SolidWorks feature is for the rebuild.
#[derive(Debug, Clone, PartialEq)]
pub enum Class {
    /// Sketches, planes, axes, coordinate systems, folders: no geometry of their own.
    Skip,
    Extrude { cut: bool },
    Fillet,
    Chamfer,
    Hole,
    Shell,
    Rib,
    Revolve { cut: bool },
    /// Cosmetic features (threads): annotations on faces of earlier features, no geometry of their own.
    Cosmetic,
    /// A draft made after the fact (`later_draft.rs`).
    Draft,
    Unsupported(String),
}

/// Classify a KeyWords feature by its type and name (`Extrusion` is the type of both `Boss-ExtrudeN` and `Cut-ExtrudeN`).
pub fn classify(f: &SwFeature) -> Class {
    let (k, name) = (f.kind.as_str(), f.name.as_str());
    let starts = |p: &str| name.starts_with(p) || k.starts_with(p);
    match k {
        "Sketch" | "Plane" | "RefPlane" | "Axis" | "RefAxis" | "Coordinate System" | "CoordSys" | "OriginProfileFeature" | "3DSketch" | "3DProfileFeature" | "Point" | "RefPoint" | "Split Line" | "SplitLine" => Class::Skip,
        _ if k.starts_with("Sketch") => Class::Skip,
        // a thin extrude (`Cut-Extrude-Thin`, `Boss-Extrude-Thin`) sweeps the wall between its sketch curve and the
        // curve's offset: the faces it left are read back as those of a plain extrude of that wall's outline (the
        // same solid; the wall thickness is not a parameter of the rebuilt step)
        "Boss-Extrude" | "Extrusion" | "Cut-Extrude" | "ICE" | "Extrude" | "Boss-Extrude-Thin" | "Cut-Extrude-Thin" => Class::Extrude { cut: starts("Cut") },
        _ if later_draft::is_draft(f) && k != "Sketch" => Class::Draft,
        "Fillet" => Class::Fillet,
        "Chamfer" => Class::Chamfer,
        "HoleWizard" | "Hole" => Class::Hole,
        "Shell" => Class::Shell,
        "Rib" => Class::Rib,
        "Revolve" | "RevCut" | "Revolution" | "Revolve-Cut" | "Cut-Revolve" => {
            if name.contains("Thin") || k.contains("Thin") {
                Class::Unsupported("thin revolve".into())
            } else {
                Class::Revolve { cut: k.contains("Cut") || starts("Cut") }
            }
        }
        "Thread" | "CosmeticThread" | "Cosmetic Thread" | "ThreadCosmetic" => Class::Cosmetic,
        other => Class::Unsupported(format!("feature type {other}")),
    }
}

/// A sketch frame: origin and axes in world space plus the `plane` reference of the sketch.
#[derive(Debug, Clone)]
pub struct Fr {
    pub o: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
    pub plane: Json,
}

impl Fr {
    pub fn xy(&self, p: Vec3) -> (f64, f64) {
        let d = p - self.o;
        (d.dot(self.x), d.dot(self.y))
    }
}

/// The rebuild in progress.
pub struct Cx<'a> {
    pub m: &'a Model<'a>,
    pub tags: &'a [Vec<(i64, i64)>],
    pub unit_mm: f64,
    pub tree: Tree,
    /// Name of every face of the imported body in our document, as far as it is known.
    pub names: HashMap<FaceId, String>,
    pub notes: Vec<String>,
    /// Faces a later `Draft` feature tilted, and the model with them as they were before it.
    pub later: Vec<later_draft::Later>,
    pub undrafted: Option<&'a Model<'a>>,
    /// The running regeneration of `tree.feats` (see [`session`]).
    pub sess: std::cell::RefCell<session::Session>,
    /// Planar faces read back at the place they had before a later move-face feature pushed them out along their normal:
    /// the distance each was moved (mm). Only the side traces of a profile (`profile::side_chain`) honour it.
    pub shift: HashMap<FaceId, f64>,
    /// The regeneration of the whole history ended with errors (set before the reordered readings are tried).
    pub regen_failed: bool,
    /// For a part that starts with imported bodies, the starting body's name of every face of the file's solid, also
    /// those a later step changed (`names` leaves them out): the supports of a blend fall back to it (`blend.rs`).
    pub start_names: HashMap<FaceId, String>,
}

impl<'a> Cx<'a> {
    /// The sketch plane normal to `z` through `at`: a standard plane (offset) when `z` is a principal axis, else a
    /// plane feature given by its frame. The frame's normal is the standard plane's (+axis), which may be `-z`.
    pub fn frame(&mut self, z: Vec3, at: Vec3) -> Fr {
        if let Some((i, _)) = principal(z) {
            let spec = PlaneSpec::for_axis(i);
            let level = at.dot(spec.zs);
            let plane = self.tree.plane(&spec, level);
            return Fr { o: spec.zs * level, x: spec.xs, y: spec.ys, z: spec.zs, plane };
        }
        let z = z.normalize();
        let a = [Vec3::X, Vec3::Y, Vec3::Z];
        let k = (0..3).min_by(|&i, &j| ck_recog::model::comp(z, i).abs().partial_cmp(&ck_recog::model::comp(z, j).abs()).unwrap()).unwrap();
        let x = (a[k] - z * a[k].dot(z)).normalize();
        let y = z.cross(x);
        let o = z * at.dot(z);
        let id = self.tree.ids.next();
        self.tree.push(
            Obj::new()
                .put("type", "plane")
                .put("id", id.as_str())
                .put("mode", "frame")
                .put("origin", ck_recog::emit::vec3(o))
                .put("xDir", ck_recog::emit::vec3(x))
                .put("yDir", ck_recog::emit::vec3(y))
                .build(),
        );
        Fr { o, x, y, z, plane: Obj::new().put("kind", "feature").put("id", id.as_str()).build() }
    }

    /// The file tags the face as an end cap of feature `feat`.
    pub fn is_cap(&self, f: FaceId, feat: i64) -> bool {
        self.tags.get(f.idx()).map_or(false, |v| v.iter().any(|t| t.0 == feat && t.1 == KIND_CAP))
    }

    /// The feature that made a face of the imported body (its smallest tag).
    pub fn tag_of(&self, f: FaceId) -> Option<i64> {
        self.tags.get(f.idx()).and_then(|v| v.iter().map(|t| t.0).min())
    }

    /// Our name of a face of the imported body. A piece a split line cut out of a face (it carries the split line's
    /// tag only, and the split line is not rebuilt) is part of the face its same-surface neighbours across the split
    /// belong to, and so is named like it.
    pub fn name_of(&self, f: FaceId) -> Result<String, String> {
        self.lookup(f).ok_or_else(|| format!("face {} (feature {:?}) has no name in the rebuilt document", self.m.b.face_id_str(f), self.tag_of(f)))
    }

    /// [`Cx::name_of`] without the error.
    pub fn lookup(&self, f: FaceId) -> Option<String> {
        if let Some(n) = self.names.get(&f) {
            return Some(n.clone());
        }
        let split_only = |g: FaceId| self.tags.get(g.idx()).map_or(false, |v| !v.is_empty() && v.iter().all(|t| t.1 == KIND_SPLIT));
        if !split_only(f) {
            return None;
        }
        // walk the pieces: split-only neighbours on the same surface, until one is named
        let mut seen = vec![f];
        let mut todo = vec![f];
        while let Some(g) = todo.pop() {
            for e in self.m.edges(g) {
                let Some(h) = self.m.across(e, g) else { continue };
                if seen.contains(&h) || !same_surface_kind(self.m.kind(g), self.m.kind(h)) {
                    continue;
                }
                if let Some(n) = self.names.get(&h) {
                    return Some(n.clone());
                }
                if split_only(h) {
                    seen.push(h);
                    todo.push(h);
                }
            }
        }
        None
    }
}

/// Points of a face: its vertices, plus the mid point of every edge (so a face bounded by a closed circle has
/// points on it).
pub fn face_points(m: &Model, f: FaceId) -> Vec<Vec3> {
    let mut out = vec![];
    for e in m.b.face_edges(f) {
        out.push(m.p0(e));
        out.push(m.p1(e));
        out.push(m.mid(e));
    }
    out
}

/// Rebuild the history of a tagged body. When it stops on a chamfer or fillet, the blend is also tried earlier in
/// the history (see [`hoisted`]).
pub fn rebuild_history(t: &Tagged) -> Outcome {
    if let Some(o) = imported::rebuild(t) {
        return o;
    }
    let o = rebuild_ordered(t);
    if o.exact() || o.close() {
        return o;
    }
    let o = hoisted(t, &o).unwrap_or(o);
    // imported bodies (at the start or joined later): the steps after them on the file's solid with their material undone
    match undo::rebuild(t) {
        Some(u) if u.exact() || u.close() || undo::kept(&u) > undo::kept(&o) => u,
        _ => o,
    }
}

/// A chamfer or fillet that SolidWorks ran after another blend can meet that blend's faces at a vertex where the
/// kernel ends it differently (a chamfer plane that runs on past the end of its edge, down the slanted faces of an
/// earlier chamfer, is capped at the edge's end): the blend picks its edges from faces of the base solid only, so
/// run before the earlier blends it cuts the same corner. A blend that removes the corner wedge (a planar cut) or
/// adds the corner round leaves the same solid in either order. The history is read again with the step that
/// stopped it moved before each earlier blend (nearest first) and kept only when the whole body then verifies.
thread_local! {
    /// Inside a trial read of `hoisted`: only a read that verifies whole is used, so the work that salvages a partial one
    /// (reformulations, the editable prefix) is skipped.
    static TRIAL: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

fn hoisted(t: &Tagged, o: &Outcome) -> Option<Outcome> {
    let id = o.remainder.as_ref()?.from.0;
    let blend = |f: &SwFeature| matches!(classify(f), Class::Fillet | Class::Chamfer);
    let p = t.features.iter().position(|f| f.id == id)?;
    if !blend(&t.features[p]) {
        return None;
    }
    // (a blend can only run before another to change the corner they make where the two meet on the final body)
    let model = Model::new(t.body);
    let earlier: Vec<usize> = (0..p).rev().filter(|&q| blend(&t.features[q]) && reorder::touches(t, &model, id, t.features[q].id)).take(3).collect();
    for q in earlier {
        let mut fs: Vec<SwFeature> = t.features.to_vec();
        let f = fs.remove(p);
        let name = f.name.clone();
        fs.insert(q, f);
        let t2 = Tagged { body: t.body, tags: t.tags.clone(), features: &fs, unit_mm: t.unit_mm, start: t.start.clone() };
        TRIAL.with(|t| t.set(true));
        let mut o2 = rebuild_ordered(&t2);
        TRIAL.with(|t| t.set(false));
        if o2.fallback.is_none() && (o2.exact() || o2.close()) {
            let before = t.features[q].name.clone();
            o2.notes.push(format!("{name} read back before {before} (geometry-equivalent, not the file's order)"));
            for s in o2.steps.iter_mut().filter(|s| s.sw_id == id) {
                s.note = format!("{} (before {before}: geometry-equivalent, not the file's order)", s.note);
            }
            return Some(o2);
        }
    }
    None
}


fn rebuild_ordered(t: &Tagged) -> Outcome {
    // a later draft turns its faces about a neutral plane that the file does not say: each candidate in turn
    let nv = later_draft::variants(&Model::new(t.body), &t.tags, t.features).max(1);
    let mut best = rebuild_ordered_v(t, 0);
    for v in 1..nv {
        if best.fallback.is_none() {
            break;
        }
        let o = rebuild_ordered_v(t, v);
        if o.fallback.is_none() || o.prefix > best.prefix {
            best = o;
        }
    }
    best
}

fn rebuild_ordered_v(t: &Tagged, variant: usize) -> Outcome {
    read_v(t, variant, None)
}

/// [`rebuild_ordered_v`], with the document started by `start` (the starting body of an imported part, `undo.rs`).
pub(crate) fn read_v(t: &Tagged, variant: usize, start: Option<&undo::Start>) -> Outcome {
    let m = Model::new(t.body);
    let later = later_draft::find(&m, &t.tags, t.features, variant);
    let mu = if later.is_empty() { None } else { Some(m.with_kinds(&later.iter().map(|l| (l.face, l.before.clone())).collect::<Vec<_>>())) };
    let mut cx = Cx { m: &m, tags: &t.tags, unit_mm: t.unit_mm, tree: Tree::default(), names: HashMap::new(), notes: vec![], later, undrafted: mu.as_ref(), sess: Default::default(), shift: HashMap::new(), regen_failed: false, start_names: HashMap::new() };
    if start.is_some() {
        cx.sess = std::cell::RefCell::new(session::Session::with_start(t.start.clone()));
    }
    let mut groups: HashMap<i64, Vec<FaceId>> = HashMap::new();
    for &f in &m.faces {
        let mut ids: Vec<i64> = t.tags.get(f.idx()).map(|v| v.iter().map(|t| t.0).collect()).unwrap_or_default();
        ids.sort();
        ids.dedup();
        for id in ids {
            groups.entry(id).or_default().push(f);
        }
    }
    let mut holes: Option<Vec<ck_recog::holes::Hole>> = None;
    // feature ranges each rebuilt SolidWorks feature emitted (patterns and mirrors name their seeds through them)
    let mut made: HashMap<i64, (usize, usize)> = HashMap::new();
    let mut steps = vec![];
    // our features of each step (`cx.tree.feats[a..b]`)
    let mut ranges: Vec<(usize, usize)> = vec![];
    let mut fallback = None;
    for f in t.features {
        let class = classify(f);
        if class == Class::Skip {
            continue;
        }
        let faces = groups.get(&f.id).cloned().unwrap_or_default();
        if let Some(st) = start.filter(|st| st.src.contains(&f.id)) {
            // the starting body stands for every source feature: the first one holds the document's start features (in
            // place of the body so far: its faces are the starting body's from here on)
            let first = steps.iter().all(|s: &Step| !st.src.contains(&s.sw_id));
            let n = cx.tree.feats.len();
            if first {
                let (feats, names, all) = undo::place(st, &mut cx.tree.ids);
                cx.tree.feats.extend(feats);
                cx.names = names;
                cx.start_names = all;
            }
            let n1 = cx.tree.feats.len();
            ranges.push((n, n1));
            let (status, note) = if first { (Status::Rebuilt, st.note.clone()) } else { (Status::Skipped, "part of the starting body (the file's solid with the later steps undone)".to_string()) };
            steps.push(Step { sw_id: f.id, name: f.name.clone(), kind: f.kind.clone(), faces: faces.len(), status, note });
            made.insert(f.id, (n, n1));
            continue;
        }
        // after the starting body only steps whose material can be undone (extrudes, revolves, fillets and chamfers)
        // are kept, so the read ends at the first other step that made faces (it would only be read on material it did
        // not make)
        if let Some(st) = start {
            let past = steps.iter().any(|s| st.src.contains(&s.sw_id));
            if past && !faces.is_empty() && !matches!(class, Class::Extrude { .. } | Class::Revolve { .. } | Class::Cosmetic | Class::Fillet | Class::Chamfer) && f.kind != "Mirror" {
                let n = cx.tree.feats.len();
                ranges.push((n, n));
                let why = "after the starting body: a step whose material is not undone in it".to_string();
                fallback = Some(format!("{} ({}): {}", f.name, f.kind, why));
                steps.push(Step { sw_id: f.id, name: f.name.clone(), kind: f.kind.clone(), faces: faces.len(), status: Status::Unsupported, note: why });
                break;
            }
        }
        if class == Class::Draft {
            let first = cx.tree.feats.len();
            let mut step = Step { sw_id: f.id, name: f.name.clone(), kind: f.kind.clone(), faces: faces.len(), status: Status::Rebuilt, note: String::new() };
            match later_draft::rebuild(&mut cx, f) {
                Ok((note, status)) => {
                    step.note = note;
                    step.status = status;
                    made.insert(f.id, (first, cx.tree.feats.len()));
                    ranges.push((first, cx.tree.feats.len()));
                    steps.push(step);
                    continue;
                }
                Err(why) => {
                    cx.tree.feats.truncate(first);
                    ranges.push((first, first));
                    step.status = Status::Unsupported;
                    step.note = why.clone();
                    fallback = Some(format!("{} ({}): {}", f.name, f.kind, why));
                    steps.push(step);
                    break;
                }
            }
        }
        let mut step = Step { sw_id: f.id, name: f.name.clone(), kind: f.kind.clone(), faces: faces.len(), status: Status::Rebuilt, note: String::new() };
        if faces.is_empty() && f.kind == "Move Face" {
            if let Some(note) = moveface::rebuild(&mut cx, f, t.features, &steps, &mut ranges, &mut made, &groups) {
                step.note = note;
                let n = cx.tree.feats.len();
                made.insert(f.id, (n - 1, n));
                ranges.push((n - 1, n));
                steps.push(step);
                continue;
            }
        }
        if faces.is_empty() {
            step.status = Status::Skipped;
            step.note = "no faces on the final body".into();
            let n = cx.tree.feats.len();
            ranges.push((n, n));
            steps.push(step);
            continue;
        }
        let first = cx.tree.feats.len();
        let r = match &class {
            Class::Extrude { cut } => draft::plain_or_drafted(&mut cx, f, &faces, *cut),
            Class::Fillet => blend::blend(&mut cx, f, &faces, false),
            Class::Chamfer => blend::blend(&mut cx, f, &faces, true),
            Class::Hole => {
                let hs = holes.get_or_insert_with(|| ck_recog::holes::find_holes(&m, &mut Default::default()));
                // a hole whose mouth is no plain circle (it breaks through an edge) is still a round cut
                hole::hole(&mut cx, f, &faces, hs)
                    .or_else(|e| profile::extrude(&mut cx, f, &faces, true).map(|n| format!("as a cut ({e}): {n}")))
                    .or_else(|e| hole::hole_own(&mut cx, f, &faces).map_err(|g| format!("{e}; from its own faces: {g}")))
            }
            Class::Shell => shell::shell(&mut cx, f, &faces),
            Class::Rib => rib::rebuild(&mut cx, f, &faces),
            Class::Revolve { cut } => revolve::revolve(&mut cx, f, &faces, *cut),
            // (a thread that made faces of its own is modelled: swept along a helix, `thread.rs`)
            Class::Cosmetic => cosmetic(&cx, f, &faces).or_else(|e| thread::rebuild(&mut cx, f, &faces).map_err(|w| format!("{e}; {w}"))),
            // (after an imported starting body a mirror is undone only as a mirror of the whole body)
            Class::Unsupported(why) if start.is_some() && f.kind == "Mirror" => body_mirror(&mut cx, f, &start.expect("checked").src).unwrap_or_else(|| Err(format!("{why}; no plane mirrors the body onto the mirror's faces"))),
            Class::Unsupported(why) => match pattern::rebuild(&mut cx, f, &faces, &pattern::Env { groups: &groups, made: &made, features: t.features })
                .or_else(|| loft::rebuild(&mut cx, f, &faces))
                .or_else(|| sweep::rebuild(&mut cx, f, &faces))
            {
                Some(r) => r,
                None => Err(why.clone()),
            },
            Class::Skip | Class::Draft => unreachable!(),
        };
        match r {
            Ok(note) => {
                step.note = note;
                if class == Class::Cosmetic && cx.tree.feats.len() == first {
                    step.status = Status::Skipped;
                }
                made.insert(f.id, (first, cx.tree.feats.len()));
                ranges.push((first, cx.tree.feats.len()));
            }
            Err(why) => {
                // what the failed read-back emitted is not part of any prefix
                cx.tree.feats.truncate(first);
                ranges.push((first, first));
                step.status = Status::Unsupported;
                step.note = why.clone();
                fallback = Some(format!("{} ({}): {}", f.name, f.kind, why));
                steps.push(step);
                break;
            }
        }
        steps.push(step);
    }
    // a history that stops (here or at its regen) may still read back in a geometry-equivalent order (`opening.rs`)
    let alt = |cx: &mut Cx, steps: &[Step], ranges: &[(usize, usize)]| {
        // (a trial read of the hoisting below only matters when it verifies whole: no further reordering inside it)
        if TRIAL.with(|t| t.get()) || start.is_some() {
            return None;
        }
        opening::reformulate(t, cx, steps, ranges, &groups).or_else(|| reorder::reformulate(t, cx, steps, ranges)).or_else(|| swap::reorder(t))
    };

    if fallback.is_some() {
        if let Some(o) = alt(&mut cx, &steps, &ranges) {
            return o;
        }
    }
    // features a step wants run earlier (the filler of a round, `hoist.rs`)
    let (mut ranges, mut hfeats) = (ranges, cx.tree.feats.clone());
    hoist::apply(&mut hfeats, &mut ranges, &steps);
    let mut out = Outcome { features: hfeats, steps, body: None, verify: None, errors: vec![], fallback, notes: std::mem::take(&mut cx.notes), prefix: 0, remainder: None, checks: vec![], ranges };
    if out.fallback.is_some() {
        if !TRIAL.with(|t| t.get()) && start.is_none() {
            keep_prefix(t, &mut out, &cx.sess);
        }
        return out;
    }
    if out.features.is_empty() {
        out.fallback = Some("no feature rebuilt".into());
        return out;
    }
    if start.is_some_and(|st| !st.verify) {
        out.fallback = Some("not verified (a first reading: which steps to undo)".into());
        return out;
    }
    full_regen_in(t, &mut out, &cx.sess);
    if out.fallback.is_some() {
        cx.regen_failed = !out.errors.is_empty();
        if let Some(o) = alt(&mut cx, &out.steps, &out.ranges) {
            return o;
        }
        if !TRIAL.with(|t| t.get()) && start.is_none() {
            keep_prefix(t, &mut out, &cx.sess);
        }
    }
    out
}

/// A mirror of the whole body after an imported starting body (`unmirror.rs`), as our `mirror` with `body: true`.
fn body_mirror(cx: &mut Cx, f: &SwFeature, src: &std::collections::HashSet<i64>) -> Option<Result<String, String>> {
    let fd = unmirror::find(cx.m, cx.tags, f.id, src)?;
    let plane = cx.frame(fd.n, fd.n * fd.c).plane;
    let id = cx.tree.ids.next();
    cx.tree.push(Obj::new().put("type", "mirror").put("id", id.as_str()).put("seeds", Json::Arr(vec![])).put("plane", plane).put("body", true).build());
    Some(Ok(format!("mirror of the body in the plane n=({:.4}, {:.4}, {:.4}) at {:.4}", fd.n.x, fd.n.y, fd.n.z, fd.c)))
}

/// [`full_regen_in`] for a candidate order of the steps: the features are run a step at a time and the run stops at
/// the first step that errors (the candidate cannot verify then, and the steps after it only cascade the failure).
pub(crate) fn full_regen_early(t: &Tagged, out: &mut Outcome, sess: &std::cell::RefCell<session::Session>) {
    let mut ends: Vec<usize> = out.ranges.iter().map(|r| r.1).filter(|&e| e > 0 && e < out.features.len()).collect();
    ends.dedup();
    for e in ends {
        if let Ok((_, errs)) = sess.borrow_mut().rebuild(&out.features[..e]) {
            if !errs.is_empty() {
                out.errors = errs;
                out.fallback = Some(format!("regen: {}", out.errors.join(", ")));
                return;
            }
        }
    }
    full_regen_in(t, out, sess)
}

/// Regenerate the whole history and verify it against the imported body.
fn full_regen(t: &Tagged, out: &mut Outcome) {
    full_regen_in(t, out, &Default::default())
}

/// [`full_regen`] continuing the regeneration `sess` already ran (the features so far).
fn full_regen_in(t: &Tagged, out: &mut Outcome, sess: &std::cell::RefCell<session::Session>) {
    match sess.borrow_mut().rebuild(&out.features) {
        Ok((body, errs)) => {
            out.errors = errs;
            if !out.errors.is_empty() {
                out.fallback = Some(format!("regen: {}", out.errors.join(", ")));
            }
            match body {
                Some(b) => {
                    let v = fastverify::verify_split(t.body, &b, undo::merges(t, &b).unwrap_or_else(|| split_merges(t)));
                    if out.fallback.is_none() && !v.close {
                        out.fallback = Some(format!("rebuilt body differs: relative error {:.2e}, area {:.2e}", v.rel, v.area_rel));
                    }
                    out.verify = Some(v);
                    out.body = Some(b);
                }
                None => {
                    if out.fallback.is_none() {
                        out.fallback = Some("regen produced no body".into());
                    }
                }
            }
        }
        Err(e) => out.fallback = Some(format!("part: {e}")),
    }
}

/// How many faces fewer than the import the rebuild has for the split lines it does not rebuild: per group of
/// neighbouring faces on one surface joined through pieces a split line cut out (faces tagged by the split line only),
/// the group's size less one.
fn split_merges(t: &Tagged) -> usize {
    let m = Model::new(t.body);
    let split_only = |g: FaceId| t.tags.get(g.idx()).map_or(false, |v| !v.is_empty() && v.iter().all(|x| x.1 == KIND_SPLIT));
    let mut seen: std::collections::HashSet<FaceId> = Default::default();
    let mut merges = 0;
    for &f in m.faces.iter().filter(|&&f| split_only(f)) {
        if !seen.insert(f) {
            continue;
        }
        let (mut todo, mut size) = (vec![f], 1usize);
        while let Some(g) = todo.pop() {
            for e in m.edges(g) {
                let Some(h) = m.across(e, g) else { continue };
                if !seen.contains(&h) && (split_only(g) || split_only(h)) && same_surface_kind(m.kind(g), m.kind(h)) {
                    seen.insert(h);
                    size += 1;
                    todo.push(h);
                }
            }
        }
        merges += size - 1;
    }
    merges
}

/// The history does not rebuild to the end (`out.fallback`): keep the leading steps that pass their checks and end
/// with the imported remainder (`prefix.rs`). With no such step the outcome stays the plain-solid fallback.
pub(crate) fn keep_prefix(t: &Tagged, out: &mut Outcome, sess: &std::cell::RefCell<session::Session>) {
    let ranges = out.ranges.clone();
    let n = out.steps.iter().position(|s| s.status == Status::Unsupported).unwrap_or(out.steps.len());
    if n == 0 || ranges.len() < n {
        return;
    }
    let (snaps, errs) = match sess.borrow_mut().snapshots(&out.features[..ranges[n - 1].1], &ranges[..n]) {
        Ok(x) => x,
        Err(e) => {
            out.notes.push(format!("prefix: {e}"));
            return;
        }
    };
    let checks = prefix::check_steps(&prefix::Input { body: t.body, tags: &t.tags, features: t.features, steps: &out.steps, snaps: &snaps, errs: &errs });
    let mut k = checks.iter().take_while(|c| c.ok()).count();
    // the step that ends the prefix: the first to fail its checks, else the first not read back
    let fail = k;
    let reason = match checks.get(k) {
        Some(c) => c.failure().unwrap_or_default(),
        None => out.fallback.clone().unwrap_or_default(),
    };
    if k >= out.steps.len() {
        // every step passes its checks, yet the whole does not verify: the wrong step is not localised, keep none
        out.notes.push(format!("prefix: every step passes its checks but the history does not verify ({reason}); not kept"));
        k = 0;
    }
    out.checks = checks;
    // the prefix ends with its last rebuilt step: steps with no features of their own (skipped, cosmetic) after it
    // belong to the remainder, and a prefix of such steps only is no prefix
    while k > 0 && out.steps[k - 1].status != Status::Rebuilt {
        k -= 1;
    }
    if k == 0 {
        return;
    }
    // (the reason is the failed check, else why the history stops: the unsupported step, the regen or the verify)
    let s = &out.steps[fail.min(out.steps.len() - 1)];
    let at = t.features.iter().position(|f| f.id == out.steps[k].sw_id).unwrap_or(t.features.len());
    let rest = t.features[at..].iter().filter(|f| classify(f) != Class::Skip).map(|f| (f.id, f.name.clone(), f.kind.clone())).collect();
    let rem = prefix::Remainder { from: (s.sw_id, s.name.clone(), s.kind.clone()), reason, steps: rest };
    out.features.truncate(ranges[k - 1].1);
    out.ranges.truncate(k);
    let next = out.features.iter().filter_map(|f| f.get_str("id").and_then(|i| i.strip_prefix('F')).and_then(|n| n.parse::<usize>().ok())).max().unwrap_or(0) + 1;
    out.features.push(rem.feature(&format!("F{next}")));
    out.prefix = k;
    out.remainder = Some(rem);
    out.body = None;
}

/// A cosmetic feature (thread) puts no geometry on the body: its faces, if any, are the ones earlier features made.
/// Skipped (nothing to rebuild) and recorded in the step's note; a feature that made faces of its own is not cosmetic.
pub fn cosmetic(cx: &Cx, f: &SwFeature, faces: &[FaceId]) -> Result<String, String> {
    let own = faces.iter().filter(|&&g| cx.tag_of(g) == Some(f.id)).count();
    if own > 0 {
        return Err(format!("{} has {own} face(s) of its own (a modelled, not cosmetic, feature)", f.kind));
    }
    Ok(format!("cosmetic {}: no geometry, skipped ({} face(s) of earlier features annotated)", f.kind, faces.len()))
}

/// The first dimension of a feature whose name starts with `prefix` (e.g. `D1`, `Hole Dia.`), in mm.
pub fn dim(cx: &Cx, f: &SwFeature, prefix: &str) -> Option<f64> {
    f.dims.iter().find(|d| d.name.starts_with(prefix)).and_then(|d| d.value).map(|v| v * cx.unit_mm)
}

/// A note comparing a measured value (mm) with the dimension the file lists.
pub fn dim_note(cx: &Cx, f: &SwFeature, prefix: &str, what: &str, measured: f64) -> String {
    match dim(cx, f, prefix) {
        Some(v) if (v - measured).abs() <= 1e-3 * (1.0 + measured.abs()) => format!("{what} {measured:.4} = {}", f.dims.iter().find(|d| d.name.starts_with(prefix)).map(|d| d.text.as_str()).unwrap_or("")),
        Some(v) => format!("{what} {measured:.4} (file lists {prefix} = {v})"),
        None => format!("{what} {measured:.4} (no dimension listed)"),
    }
}

pub fn jn(v: f64) -> Json {
    n(v)
}
