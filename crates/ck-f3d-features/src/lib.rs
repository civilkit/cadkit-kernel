//! A Fusion 360 design's feature timeline (the `ck-f3d-dump` IR) as a CadKit part feature list.
//!
//! [`timeline_to_features`] walks the timeline in order and writes the ck-part features it can carry: sketches (on a
//! standard plane, or on a `plane` feature given by its frame), construction planes, extrudes (blind, symmetric,
//! through all as a cut; join, cut, new body) and revolves (a full turn or evenly either side of the sketch plane).
//! Everything else becomes a suppressed `placeholder` feature that names the Fusion feature type and says why, and the
//! features built on something that did not map become placeholders too, so a history is never drawn with a step
//! missing in the middle without saying so. Each timeline item gets an [`ItemReport`] (mapped or placeholder), and the
//! whole import a list of [`Note`]s.
//!
//! The history is never trusted by itself: [`guard::verify`] regenerates the written features with the kernel and
//! compares the volume and box of the result with the body the ASM reader (`ck-asm`) builds from the same file, and
//! the import is marked [`Verdict::Unverified`] when they differ (with the delta), never silently wrong.
//!
//! Units: the dump is in cm and radians, a part in mm and degrees. The logic of the sketch, extrude and revolve
//! mapping is a port of Dutchman's `dutchman-ckpart` (Apache-2.0, see THIRD_PARTY.md); the types are ours.

pub mod guard;
mod blend;
mod bodies;
mod pattern;
mod sketch;
mod solid;

use std::collections::BTreeMap;

use ck_asm::vec3::{cross, unit};
use ck_f3d_dump::{Detail, Dump, TimelineItem, Vec3};

use serde_json::{Value, json};

pub use guard::{Check, Measure, Verdict};

/// Fusion's centimetres in a part's millimetres.
pub const CM_TO_MM: f64 = 10.0;

/// What became of one timeline item.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// Written as the ck-part feature(s) of the same meaning.
    Mapped,
    /// Written as a suppressed placeholder naming the Fusion feature type.
    Placeholder,
}

/// One timeline item and what it became.
#[derive(Clone, Debug)]
pub struct ItemReport {
    /// The timeline index of the item.
    pub index: i64,
    pub name: String,
    /// `Sketch`, `ExtrudeFeature`, ... (empty if the file does not say).
    pub object_type: String,
    pub status: Status,
    /// The ids of the ck-part features written for it (a sketch on a new plane writes the plane too).
    pub features: Vec<String>,
    /// Why it is a placeholder, or what was left out of a mapped item.
    pub note: Option<String>,
}

/// A remark about the import, with the timeline item it is about when there is one.
#[derive(Clone, Debug)]
pub struct Note {
    pub item: Option<i64>,
    pub text: String,
}

/// The result of [`timeline_to_features`].
#[derive(Clone, Debug, Default)]
pub struct FeatureImport {
    /// The ck-part features, in history order.
    pub features: Vec<Value>,
    /// One report per timeline item, in timeline order.
    pub items: Vec<ItemReport>,
    pub notes: Vec<Note>,
    /// The next feature number (the part's `seq`).
    pub seq: u64,
}

impl FeatureImport {
    /// The part document, `{"format":"ck-part","version":1,"features":[...],"seq":n}`.
    pub fn part(&self) -> Value {
        json!({"format": "ck-part", "version": 1, "features": self.features, "seq": self.seq})
    }

    pub fn mapped(&self) -> usize {
        self.items.iter().filter(|i| i.status == Status::Mapped).count()
    }

    pub fn placeholders(&self) -> usize {
        self.items.iter().filter(|i| i.status == Status::Placeholder).count()
    }
}

/// A sketch plane's frame (mm): origin and in-plane axes; the normal is `x` crossed with `y`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Frame {
    pub origin: Vec3,
    pub x: Vec3,
    pub y: Vec3,
}

impl Frame {
    /// A frame from an origin and two directions; `None` when an axis is zero or the two are parallel.
    pub(crate) fn new(origin: Vec3, x: Vec3, y: Vec3) -> Option<Frame> {
        let (x, y) = (unit(x)?, unit(y)?);
        unit(cross(x, y))?;
        Some(Frame { origin, x, y })
    }

    pub(crate) fn normal(&self) -> Vec3 {
        unit(cross(self.x, self.y)).unwrap_or([0.0, 0.0, 1.0])
    }

    fn near(&self, o: &Frame) -> bool {
        let close = |a: Vec3, b: Vec3, tol: f64| a.iter().zip(b).all(|(p, q)| (p - q).abs() <= tol);
        close(self.origin, o.origin, 1e-6) && close(self.x, o.x, 1e-9) && close(self.y, o.y, 1e-9)
    }
}

/// What the mapping of one item may need of the ones before it.
pub(crate) struct Builder {
    pub features: Vec<Value>,
    next: u64,
    next_plane: u64,
    /// Planes given by their frame so far (and the reference to each).
    planes: Vec<(Frame, Value)>,
    /// Mapped sketches by timeline index, and by name.
    pub sketches: BTreeMap<i64, sketch::SketchInfo>,
    pub sketch_by_name: BTreeMap<String, i64>,
    /// Whether a feature that builds a body was written.
    pub builds: bool,
    /// Whether an item that builds or changes the body became a placeholder.
    pub lost: bool,
    /// Mapped extrudes and revolves by timeline index (the seeds a pattern or mirror can repeat), and how many new bodies were made.
    pub seeds: BTreeMap<i64, String>,
    pub new_bodies: usize,
    /// The design has bodies in more than one component: each component's bodies are named and a feature acts on its own
    /// component's body only.
    pub multi: bool,
    /// The component of the item being mapped, and how many bodies it has made so far.
    pub comp: String,
    pub comp_bodies: BTreeMap<String, usize>,
    /// Where the item's component sits in the design (`None`: at the origin, unrotated).
    pub place: Option<Placement>,
    /// The design names its bodies: it has a `Combine`, a `CopyPasteBodies`, or a `Move` of bodies the file names by the
    /// item that made them (`producer`, `body_index`). Then every body a mapped item makes is a named body of the part
    /// (`B<item>`), and those items select bodies by those names (`bodies.rs`).
    pub named: bool,
    /// The timeline index of the item being mapped.
    pub cur: i64,
    /// The part's name of each body the history made so far, and its component, by (the item that made it, its index
    /// among that item's bodies): the name the file gives a body, taken to the part.
    pub bodies: BTreeMap<(i64, i64), (String, String)>,
    /// The Fusion type of each timeline item by its index (why a body is not one the history has).
    pub kinds: BTreeMap<i64, String>,
    /// The names of bodies a combine consumed (its tools without `keepTools`): a later feature of a design with bodies in
    /// several components must not name them among its component's bodies.
    pub consumed: std::collections::BTreeSet<String>,
}

/// A component's placement in the design: a rotation and a translation (mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Placement {
    pub rot: [[f64; 3]; 3],
    pub t: Vec3,
}

impl Placement {
    const IDENTITY: Placement = Placement { rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: [0.0; 3] };

    /// From a row-major 4 x 4 in cm; `None` for the identity, an error for a scaling or a mirror.
    pub(crate) fn from_mat(m: &ck_f3d_dump::Mat4) -> Result<Option<Placement>, String> {
        if m.iter().flatten().any(|v| !v.is_finite()) {
            return Err("its component's placement is not a number".into());
        }
        let rot = [[m[0][0], m[0][1], m[0][2]], [m[1][0], m[1][1], m[1][2]], [m[2][0], m[2][1], m[2][2]]];
        let col = |c: usize| [rot[0][c], rot[1][c], rot[2][c]];
        let dotp = |a: Vec3, b: Vec3| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        for i in 0..3 {
            for j in i..3 {
                let want = if i == j { 1.0 } else { 0.0 };
                if (dotp(col(i), col(j)) - want).abs() > 1e-6 {
                    return Err("its component's placement scales or shears".into());
                }
            }
        }
        if dotp(cross(col(0), col(1)), col(2)) < 0.0 {
            return Err("its component's placement mirrors".into());
        }
        let t = [m[0][3] * CM_TO_MM, m[1][3] * CM_TO_MM, m[2][3] * CM_TO_MM];
        let identity = (0..3).all(|i| (0..3).all(|j| (rot[i][j] - f64::from(u8::from(i == j))).abs() < 1e-12)) && t.iter().all(|v| v.abs() < 1e-9);
        Ok((!identity).then_some(Placement { rot, t }))
    }

    /// The motion that takes something placed by `base` to where `self` places it (`x -> R x + t`, a rotation about the
    /// origin and then a translation): `self` after the inverse of `base`.
    fn relative_to(&self, base: &Option<Placement>) -> Placement {
        let b = base.unwrap_or(Placement::IDENTITY);
        let mut rot = [[0.0; 3]; 3];
        for (i, row) in rot.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = (0..3).map(|k| self.rot[i][k] * b.rot[j][k]).sum();
            }
        }
        // t = t_self - R b.t
        let rb = Placement { rot, t: [0.0; 3] }.dir(b.t);
        Placement { rot, t: [self.t[0] - rb[0], self.t[1] - rb[1], self.t[2] - rb[2]] }
    }

    /// The rotation as an axis (unit) and an angle in degrees; `None` for no rotation.
    pub(crate) fn axis_angle(&self) -> Option<(Vec3, f64)> {
        let r = &self.rot;
        let trace = r[0][0] + r[1][1] + r[2][2];
        let angle = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
        if angle < 1e-9 {
            return None;
        }
        let axis = if std::f64::consts::PI - angle < 1e-6 {
            // a half turn: the axis is the column of R + I with the largest entry
            let c = (0..3)
                .map(|j| [r[0][j] + f64::from(u8::from(j == 0)), r[1][j] + f64::from(u8::from(j == 1)), r[2][j] + f64::from(u8::from(j == 2))])
                .max_by(|a, b| (a[0].powi(2) + a[1].powi(2) + a[2].powi(2)).total_cmp(&(b[0].powi(2) + b[1].powi(2) + b[2].powi(2))))?;
            unit(c)?
        } else {
            unit([r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]])?
        };
        Some((axis, angle.to_degrees()))
    }

    pub(crate) fn dir(&self, v: Vec3) -> Vec3 {
        let r = &self.rot;
        [
            r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
            r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
            r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
        ]
    }

    pub(crate) fn point(&self, v: Vec3) -> Vec3 {
        let d = self.dir(v);
        [d[0] + self.t[0], d[1] + self.t[1], d[2] + self.t[2]]
    }
}

/// How a component sits in the design.
enum Where {
    /// One occurrence, at this placement.
    One(Option<Placement>),
    /// Several occurrences, all rigid: the features are built at the first one's placement and copied to the others.
    Many(Vec<Option<Placement>>),
    /// The occurrence's placement is not usable.
    Bad(String),
}

/// The placement of each component by name, from the occurrence tree.
fn placements(dump: &Dump) -> BTreeMap<String, Where> {
    fn walk(nodes: &[ck_f3d_dump::OccurrenceNode], out: &mut BTreeMap<String, Vec<Result<Option<Placement>, String>>>) {
        for n in nodes {
            if let Some(Some(c)) = n.component.as_ref() {
                let r = match n.transform2.as_ref().or(n.transform.as_ref()) {
                    Some(m) => Placement::from_mat(m),
                    None => Err("its component's occurrence has no placement".into()),
                };
                out.entry(c.clone()).or_default().push(r);
            }
            if let Some(ch) = n.children.as_ref() {
                walk(ch, out);
            }
        }
    }
    let mut all = BTreeMap::new();
    walk(dump.occurrences.as_deref().unwrap_or(&[]), &mut all);
    all.into_iter()
        .map(|(k, mut v)| {
            let w = if v.len() > 1 {
                match v.iter().find_map(|r| r.as_ref().err()) {
                    Some(e) => Where::Bad(e.clone()),
                    None => Where::Many(v.into_iter().filter_map(Result::ok).collect()),
                }
            } else {
                match v.pop() {
                    Some(Ok(p)) => Where::One(p),
                    Some(Err(e)) => Where::Bad(e),
                    None => Where::Bad("its component has no occurrence".into()),
                }
            };
            (k, w)
        })
        .collect()
}

impl Builder {
    pub(crate) fn next_id(&mut self) -> String {
        self.next += 1;
        format!("F{}", self.next)
    }

    /// The plane reference for a frame: a standard plane when the frame is one, else a `plane` feature (added once).
    pub(crate) fn plane_for(&mut self, name: &str, frame: &Frame) -> Value {
        for std_name in ["front", "top", "right", "xz", "yz"] {
            let f = ck_core::linalg::std_plane(std_name);
            let std = Frame {
                origin: [f.o.x, f.o.y, f.o.z],
                x: [f.x.x, f.x.y, f.x.z],
                y: [f.y.x, f.y.y, f.y.z],
            };
            if std.near(frame) {
                return json!({"kind": "std", "id": std_name});
            }
        }
        if let Some((_, r)) = self.planes.iter().find(|(f, _)| f.near(frame)) {
            return r.clone();
        }
        self.add_plane(name, frame)
    }

    /// Adds a `plane` feature given by its frame and returns the reference to it.
    pub(crate) fn add_plane(&mut self, name: &str, frame: &Frame) -> Value {
        self.next_plane += 1;
        let id = format!("P{}", self.next_plane);
        self.features.push(json!({
            "type": "plane", "id": id, "name": format!("{name} plane"), "mode": "frame",
            "origin": frame.origin, "xDir": frame.x, "yDir": frame.y,
        }));
        let r = json!({"kind": "feature", "id": id});
        self.planes.push((*frame, r.clone()));
        r
    }

    /// A point / direction (mm) given in the item's component's space, in the design's.
    pub(crate) fn world_point(&self, v: Vec3) -> Vec3 {
        self.place.map_or(v, |p| p.point(v))
    }

    pub(crate) fn world_dir(&self, v: Vec3) -> Vec3 {
        self.place.map_or(v, |p| p.dir(v))
    }

    /// A frame (mm) given in the item's component's space, in the design's.
    pub(crate) fn world(&self, f: Frame) -> Frame {
        match &self.place {
            None => f,
            Some(p) => Frame { origin: p.point(f.origin), x: p.dir(f.x), y: p.dir(f.y) },
        }
    }

    /// A placeholder feature for an item that does not map.
    fn placeholder(&mut self, name: &str, kind: &str, why: &str) {
        let id = self.next_id();
        self.features.push(json!({
            "type": "placeholder", "id": id, "name": name, "suppressed": true, "source": kind, "note": why,
        }));
    }
}

/// Fusion item types that build and change no body (planes, axes, joints, organisation), or only place or copy whole
/// components (`ComponentFromBodies` moves bodies into a new component, `CopyPasteOccurrence` and `DerivedInstance` add
/// another occurrence of one, `GeometricRelationship` relates two placements): none of them changes a face or an edge of a
/// body, so a blend or a face extrude after one still finds what the file names. They stay placeholders (the final
/// design has their placements, which the guard sees) but do not mark the history as lost.
fn builds_nothing(kind: &str) -> bool {
    matches!(
        kind,
        "Sketch" | "ConstructionPlane" | "ConstructionAxis" | "ConstructionPoint" | "Canvas" | "Occurrence" | "JointOrigin"
            | "Joint" | "AsBuiltJoint" | "RigidGroup" | "Snapshot" | "GroundOccurrence" | "Group" | "Parameter"
            | "ComponentInsert" | "ComponentFromBodies" | "CopyPasteOccurrence" | "DerivedInstance" | "GeometricRelationship"
    )
}

/// Maps the timeline of a design dump. A pure function of the dump.
pub fn timeline_to_features(dump: &Dump) -> FeatureImport {
    let mut b = Builder {
        features: Vec::new(),
        next: 0,
        next_plane: 0,
        planes: Vec::new(),
        sketches: BTreeMap::new(),
        sketch_by_name: BTreeMap::new(),
        builds: false,
        lost: false,
        seeds: BTreeMap::new(),
        new_bodies: 0,
        multi: false,
        comp: String::new(),
        comp_bodies: BTreeMap::new(),
        place: None,
        named: false,
        cur: 0,
        bodies: BTreeMap::new(),
        kinds: BTreeMap::new(),
        consumed: Default::default(),
    };
    let mut out = FeatureImport::default();
    let items = dump.timeline_items();
    if items.is_empty() {
        out.notes.push(Note { item: None, text: "the file has no feature timeline (a direct-modelling design)".into() });
    }
    let root = root_component(dump);
    let where_ = placements(dump);
    // components placed several times: the position of their last item that makes or changes a body, after which their
    // bodies are copied to the other occurrences
    let mut copy_after: BTreeMap<usize, String> = BTreeMap::new();
    for (pos, item) in items.iter().enumerate() {
        if let Some(Some(c)) = item.component.as_ref()
            && root.as_deref() != Some(c.as_str())
            && matches!(where_.get(c), Some(Where::Many(_)))
            && matches!(item.detail.as_ref(), Some(Detail::Extrude(_) | Detail::Revolve(_) | Detail::Fillet(_) | Detail::Chamfer(_)))
        {
            copy_after.retain(|_, v| v != c);
            copy_after.insert(pos, c.clone());
        }
    }
    // a component placed several times is copied by its bodies' names, so its bodies are named
    b.multi = body_components(items) > 1 || !copy_after.is_empty();
    b.named = names_bodies(items);
    for (pos, item) in items.iter().enumerate() {
        b.kinds.insert(item.index.unwrap_or(pos as i64), item.object_type().unwrap_or("").to_owned());
    }
    for (pos, item) in items.iter().enumerate() {
        let idx = item.index.unwrap_or(pos as i64);
        let name = item.name().map(str::to_owned).unwrap_or_else(|| format!("Item {idx}"));
        let kind = item.object_type().unwrap_or("").to_owned();
        let before = b.features.len();
        let (builds, lost) = (b.builds, b.lost);
        let bodies = b.comp_bodies.clone();
        let new_bodies = b.new_bodies;
        let named_bodies = (b.bodies.clone(), b.consumed.clone());
        let result = map_item(&mut b, item, idx, &name, &kind, root.as_deref(), &where_);
        let (status, note) = match result {
            Ok(note) => (Status::Mapped, note),
            Err(why) => {
                // nothing the failed mapping wrote stays (a sketch adds nothing when it fails, but be sure)
                b.features.truncate(before);
                b.builds = builds;
                b.comp_bodies = bodies;
                b.new_bodies = new_bodies;
                (b.bodies, b.consumed) = named_bodies;
                b.lost = lost || !builds_nothing(&kind);
                b.placeholder(&name, if kind.is_empty() { "unknown" } else { &kind }, &why);
                (Status::Placeholder, Some(why))
            }
        };
        let features = b.features[before..]
            .iter()
            .filter_map(|f| f.get("id").and_then(Value::as_str).map(str::to_owned))
            .collect();
        if let Some(n) = &note {
            out.notes.push(Note { item: Some(idx), text: format!("{name}: {n}") });
        }
        out.items.push(ItemReport { index: idx, name, object_type: kind, status, features, note });
    }
    // the copies come last, so that the blends of the other items regenerate a history without them
    for (pos, c) in &copy_after {
        let Some(Where::Many(ps)) = where_.get(c) else { continue };
        let before = b.features.len();
        let at = out.items.get(*pos).map_or(*pos as i64, |i| i.index);
        match place_copies(&mut b, c, ps) {
            Ok(n) => out.notes.push(Note { item: Some(at), text: format!("component {c} copied to its {n} other occurrence(s)") }),
            Err(why) => {
                out.notes.push(Note { item: Some(at), text: format!("the other occurrences of component {c} are not placed: {why}") });
            }
        }
        let ids: Vec<String> = b.features[before..].iter().filter_map(|f| f.get("id").and_then(Value::as_str).map(str::to_owned)).collect();
        if let Some(it) = out.items.get_mut(*pos) {
            it.features.extend(ids);
        }
    }
    out.seq = b.next + 1;
    out.features = b.features;
    out
}

/// Copies of a component's bodies at its other occurrences (`moveBody` with `copy`), as moves from the first occurrence's
/// placement, where the component's features were built. Returns the number of occurrences copied to.
fn place_copies(b: &mut Builder, comp: &str, places: &[Option<Placement>]) -> Result<usize, String> {
    let have = b.comp_bodies.get(comp).copied().unwrap_or(0);
    if have == 0 {
        return Err("it made no body".into());
    }
    let own = format!("{comp} body");
    let names: Vec<String> = (0..have).map(|n| if n == 0 { own.clone() } else { format!("{own} {}", n + 1) }).filter(|n| !b.consumed.contains(n)).collect();
    let base = places[0];
    for (k, p) in places.iter().enumerate().skip(1) {
        let rel = p.unwrap_or(Placement::IDENTITY).relative_to(&base);
        for n in &names {
            let id = b.next_id();
            let mut f = json!({
                "type": "moveBody", "id": id, "name": format!("{n} at occurrence {}", k + 1), "bodies": [{"body": n}],
                "body": format!("{n} @{}", k + 1), "copy": true, "translate": rel.t,
            });
            if let Some((axis, angle)) = rel.axis_angle() {
                f["rotate"] = json!({"axis": axis, "origin": [0.0, 0.0, 0.0], "angle": angle});
            }
            b.features.push(f);
        }
    }
    Ok(places.len() - 1)
}

/// The name of the root component: the one the file says is the root, else the one most items belong to.
fn root_component(dump: &Dump) -> Option<String> {
    if let Some(c) = dump.components.as_deref().unwrap_or(&[]).iter().find(|c| c.is_root == Some(true))
        && let Some(Some(n)) = c.name.as_ref()
    {
        return Some(n.clone());
    }
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    for it in dump.timeline_items() {
        if let Some(Some(c)) = it.component.as_ref() {
            *counts.entry(c.as_str()).or_default() += 1;
        }
    }
    counts.into_iter().max_by_key(|(_, n)| *n).map(|(c, _)| c.to_owned())
}

/// How many components have an item that makes a body.
fn body_components(items: &[TimelineItem]) -> usize {
    let mut set: std::collections::BTreeSet<&str> = std::collections::BTreeSet::new();
    for it in items {
        if matches!(it.detail.as_ref(), Some(Detail::Extrude(_) | Detail::Revolve(_)))
            && let Some(Some(c)) = it.component.as_ref()
        {
            set.insert(c.as_str());
        }
    }
    set.len()
}

/// Whether the design selects bodies by the item that made them: it has a combine or a paste of bodies, or a move whose
/// input is a body that names its producer.
fn names_bodies(items: &[TimelineItem]) -> bool {
    items.iter().any(|it| match it.object_type() {
        Some("CombineFeature" | "CopyPasteBodies") => true,
        Some("MoveFeature") => matches!(it.detail.as_ref(), Some(Detail::Other(m)) if bodies::move_names_producers(m)),
        _ => false,
    })
}

/// Maps one item: `Ok(note)` when it was written (the note says what was left out), `Err(why)` when it was not.
fn map_item(
    b: &mut Builder,
    item: &TimelineItem,
    idx: i64,
    name: &str,
    kind: &str,
    root: Option<&str>,
    where_: &BTreeMap<String, Where>,
) -> Result<Option<String>, String> {
    b.place = None;
    b.cur = idx;
    b.comp = String::new();
    if let Some(Some(c)) = item.component.as_ref() {
        b.comp = c.clone();
        if root != Some(c.as_str()) {
            match where_.get(c) {
                Some(Where::One(p)) => b.place = *p,
                Some(Where::Many(ps)) => b.place = ps[0],
                Some(Where::Bad(why)) => return Err(why.clone()),
                None => return Err("its component has no occurrence in the design".into()),
            }
        }
    }
    // An item the file suppresses, or that lies after its timeline marker, is not in the body the file ends with.
    let suppressed = item.file_state() != ck_f3d_dump::ItemState::Normal || item.is_rolled_back == Some(true);
    let before = b.features.len();
    let (builds, lost) = (b.builds, b.lost);
    let bodies = b.comp_bodies.clone();
    let new_bodies = b.new_bodies;
    let named_bodies = (b.bodies.clone(), b.consumed.clone());
    let mut note = None;
    match item.detail.as_ref() {
        Some(Detail::Sketch(d)) => {
            let info = sketch::map(b, name, d)?;
            b.sketch_by_name.insert(name.to_owned(), idx);
            b.sketches.insert(idx, info);
        }
        Some(Detail::Extrude(d)) => note = solid::extrude(b, name, d)?,
        Some(Detail::Revolve(d)) => note = solid::revolve(b, name, d)?,
        Some(Detail::ConstructionPlane(d)) => solid::construction_plane(b, name, d)?,
        Some(Detail::Fillet(d)) => note = blend::fillet(b, name, d)?,
        Some(Detail::Chamfer(d)) => note = blend::chamfer(b, name, d)?,
        Some(Detail::CircularPattern(d)) => note = pattern::circular(b, name, d)?,
        Some(Detail::RectangularPattern(d)) => note = pattern::rectangular(b, name, d)?,
        Some(d) if kind == "CombineFeature" && pattern::mirror_map(d).is_some() => {
            note = bodies::combine(b, name, pattern::mirror_map(d).expect("checked"))?;
        }
        Some(d) if kind == "CopyPasteBodies" && pattern::mirror_map(d).is_some() => {
            note = bodies::copy_paste(b, name, pattern::mirror_map(d).expect("checked"))?;
        }
        Some(d) if kind == "MoveFeature" && pattern::mirror_map(d).is_some() => {
            note = pattern::move_one_body(b, name, pattern::mirror_map(d).expect("checked"))?;
        }
        Some(d) if kind == "MirrorFeature" && pattern::mirror_map(d).is_some() => {
            note = pattern::mirror(b, name, pattern::mirror_map(d).expect("checked"))?;
        }
        _ => {
            return Err(match kind {
                "" => "this item's type is not known".to_owned(),
                k => format!("{k} is not mapped to a CadKit part yet"),
            });
        }
    }
    if !suppressed
        && matches!(item.detail.as_ref(), Some(Detail::Extrude(_) | Detail::Revolve(_)))
        && let Some(f) = b.features[before..].iter().find(|f| matches!(f.get("type").and_then(Value::as_str), Some("extrude" | "revolve")) && f.get("endRef").is_none())
        && let Some(id) = f.get("id").and_then(Value::as_str)
    {
        let id = id.to_owned();
        b.seeds.insert(idx, id);
    }
    if suppressed {
        for f in b.features[before..].iter_mut() {
            if f.get("type").and_then(Value::as_str) != Some("plane") {
                f["suppressed"] = json!(true);
            }
        }
        b.builds = builds;
        b.lost = lost;
        b.comp_bodies = bodies;
        b.new_bodies = new_bodies;
        (b.bodies, b.consumed) = named_bodies;
        let why = "suppressed or rolled back in the file";
        note = Some(note.map_or_else(|| why.to_owned(), |n| format!("{n}; {why}")));
    }
    Ok(note)
}
