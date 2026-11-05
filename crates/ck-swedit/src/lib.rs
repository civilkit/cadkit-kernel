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
pub mod loft;
mod opening;
pub mod pattern;
pub mod prefix;
pub mod profile;
pub mod revolve;
pub mod shell;
pub mod thread;

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
        Tagged { body: b, tags, features: &imp.meta.features, unit_mm: unit_mm(&imp.meta) }
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
        if let (Some(_), Some(bytes), Some(Json::Obj(kv))) = (&self.remainder, file, feats.last_mut()) {
            let text = ck_text::base64_encode(bytes);
            let key = ck_model::features::import_step::blob_key(&text);
            if let Some(v) = kv.iter_mut().find(|(k, _)| k == "blob") {
                v.1 = Json::Str(key.clone());
            }
            blobs = blobs.put(key.as_str(), text.as_str());
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
    Revolve { cut: bool },
    /// Cosmetic features (threads): annotations on faces of earlier features, no geometry of their own.
    Cosmetic,
    Unsupported(String),
}

/// Classify a KeyWords feature by its type and name (`Extrusion` is the type of both `Boss-ExtrudeN` and `Cut-ExtrudeN`).
pub fn classify(f: &SwFeature) -> Class {
    let (k, name) = (f.kind.as_str(), f.name.as_str());
    let starts = |p: &str| name.starts_with(p) || k.starts_with(p);
    match k {
        "Sketch" | "Plane" | "RefPlane" | "Axis" | "RefAxis" | "Coordinate System" | "CoordSys" | "OriginProfileFeature" | "3DSketch" | "3DProfileFeature" | "Point" | "RefPoint" | "Split Line" | "SplitLine" => Class::Skip,
        _ if k.starts_with("Sketch") => Class::Skip,
        "Boss-Extrude" | "Extrusion" | "Cut-Extrude" | "ICE" => {
            if name.contains("Thin") || k.contains("Thin") {
                Class::Unsupported("thin extrude".into())
            } else {
                Class::Extrude { cut: starts("Cut") }
            }
        }
        "Fillet" => Class::Fillet,
        "Chamfer" => Class::Chamfer,
        "HoleWizard" | "Hole" => Class::Hole,
        "Shell" => Class::Shell,
        "Revolve" | "RevCut" | "Revolution" | "Revolve-Cut" => {
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

/// Rebuild the history of a tagged body.
pub fn rebuild_history(t: &Tagged) -> Outcome {
    let m = Model::new(t.body);
    let mut cx = Cx { m: &m, tags: &t.tags, unit_mm: t.unit_mm, tree: Tree::default(), names: HashMap::new(), notes: vec![] };
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
        let mut step = Step { sw_id: f.id, name: f.name.clone(), kind: f.kind.clone(), faces: faces.len(), status: Status::Rebuilt, note: String::new() };
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
            Class::Revolve { cut } => revolve::revolve(&mut cx, f, &faces, *cut),
            // (a thread that made faces of its own is modelled: swept along a helix, `thread.rs`)
            Class::Cosmetic => cosmetic(&cx, f, &faces).or_else(|e| thread::rebuild(&mut cx, f, &faces).map_err(|w| format!("{e}; {w}"))),
            Class::Unsupported(why) => match pattern::rebuild(&mut cx, f, &faces, &pattern::Env { groups: &groups, made: &made, features: t.features }).or_else(|| loft::rebuild(&mut cx, f, &faces)) {
                Some(r) => r,
                None => Err(why.clone()),
            },
            Class::Skip => unreachable!(),
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
    let alt = |cx: &mut Cx, steps: &[Step], ranges: &[(usize, usize)]| opening::reformulate(t, cx, steps, ranges, &groups);
    if fallback.is_some() {
        if let Some(o) = alt(&mut cx, &steps, &ranges) {
            return o;
        }
    }
    let mut out = Outcome { features: cx.tree.feats.clone(), steps, body: None, verify: None, errors: vec![], fallback, notes: std::mem::take(&mut cx.notes), prefix: 0, remainder: None, checks: vec![], ranges };
    if out.fallback.is_some() {
        keep_prefix(t, &mut out);
        return out;
    }
    if out.features.is_empty() {
        out.fallback = Some("no feature rebuilt".into());
        return out;
    }
    full_regen(t, &mut out);
    if out.fallback.is_some() {
        if let Some(o) = alt(&mut cx, &out.steps, &out.ranges) {
            return o;
        }
        keep_prefix(t, &mut out);
    }
    out
}

/// Regenerate the whole history and verify it against the imported body.
fn full_regen(t: &Tagged, out: &mut Outcome) {
    match ck_recog::rebuild::rebuild(&out.features) {
        Ok((body, errs)) => {
            out.errors = errs;
            if !out.errors.is_empty() {
                out.fallback = Some(format!("regen: {}", out.errors.join(", ")));
            }
            match body {
                Some(b) => {
                    let v = fastverify::verify(t.body, &b);
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

/// The history does not rebuild to the end (`out.fallback`): keep the leading steps that pass their checks and end
/// with the imported remainder (`prefix.rs`). With no such step the outcome stays the plain-solid fallback.
fn keep_prefix(t: &Tagged, out: &mut Outcome) {
    let ranges = out.ranges.clone();
    let n = out.steps.iter().position(|s| s.status == Status::Unsupported).unwrap_or(out.steps.len());
    if n == 0 || ranges.len() < n {
        return;
    }
    let (snaps, errs) = match prefix::snapshots(&out.features[..ranges[n - 1].1], &ranges[..n]) {
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
