//! Sketches and their feature parameters, read from `Config-N-ResolvedFeatures`.
//!
//! The stream is a serialized object graph in the layout of the MFC `CArchive` object format (a new class is
//! `FF FF <schema u16> <name length u16> <ASCII name>`, a later object of a known class is `0x8000 | class number`, text
//! is `FF FE FF <u8 length> <UTF-16LE>`). Its field layouts are per class and are not all known, so this reader does not
//! walk the object graph: it finds the records it understands by their fixed shape and ignores the rest. The shapes were
//! derived from sample parts (see `docs/sldprt-sketch.md` for each record, what was checked against which geometry, and
//! the doubts that remain); lengths are metres and angles radians, as stored.
//!
//! What is read, per sketch (a "profile feature": a named feature record followed by sketch entity records):
//!
//! * points (`u32 id`, an entity header, the point's x and y as doubles in the sketch plane, the curve handles it lies on);
//! * the curve table (a record per line, arc or circle naming its end points by their position in the point list);
//! * the dimension parameters (`D1`, ... with their value) and the extrude parameters of the features that follow.
//!
//! The sketch plane is reported as the id of the plane feature the sketch refers to (`PlaneRef`), not as a frame: where
//! the frame is stored it is stored next to records whose meaning is not settled (see the doc).
//!
//! Never panics: every read is bounds-checked and the work is capped by [`Limits`] (`max_stream`) and the entity caps
//! below. Errors carry stable codes: `sketch.no_stream`, `sketch.malformed`, `limit.sketch_entities`.

use crate::container::Container;
use crate::error::{Limits, SldprtError, R};
use std::collections::HashMap;

/// Most entity records (points and curves) read from one stream.
pub const MAX_ENTITIES: usize = 2_000_000;
/// Most feature-name records read from one stream.
pub const MAX_FEATURES: usize = 200_000;

/// A handle to a curve a point lies on, as written in the point's record: the class (known by name or by tag) and the
/// curve's id within its class.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HandleClass {
    Point,
    Line,
    Arc,
    /// A curve class with a name this reader does not interpret (ellipse, spline, ...), or a tag it could not place.
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurveHandle {
    pub class: HandleClass,
    pub id: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointRole {
    /// A point of the sketch (an end point, a centre, a free point).
    Free,
    /// The point a circle is stored by (a point on the circle, its record carries the circle's handle).
    Rim,
    /// A reference to something outside the sketch (the origin, an edge of another feature): `name` is its name.
    External,
    /// The centre or an axis end point an ellipse is stored with (not a candidate for the centre of an arc or circle).
    Axis,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SkPoint {
    /// The id stored in the record (not the position; positions are used by the curve table).
    pub id: u32,
    /// Sketch plane coordinates, metres. `(0, 0)` and `located == false` for a reference to something with no coordinates
    /// of its own in the record (a temporary axis); such a record still takes its place in the point list.
    pub x: f64,
    pub y: f64,
    pub located: bool,
    pub role: PointRole,
    /// For an external reference, its name (`Point1@Origin`).
    pub name: Option<String>,
    /// The two small counters after the coordinates (meaning not settled; `(0, n)` for a plain point on `n` curves,
    /// `(6, 0)` for the origin reference, `(2, 2)` for the rim point of a circle).
    pub counters: (u16, u16),
    pub on: Vec<CurveHandle>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SkCurve {
    /// A line between two points (indices into [`Sketch::points`]).
    Line { a: usize, b: usize, construction: bool, handle: Option<u16>, sense: Option<i32> },
    /// An arc from `start` to `end` (counter-clockwise from start unless the stream says otherwise: direction is not
    /// stored, see the doc) about `centre`.
    Arc { start: usize, end: usize, centre: Option<usize>, centre_certain: bool, construction: bool, handle: Option<u16>, sense: Option<i32> },
    /// A full circle stored by one point on it.
    Circle { rim: usize, centre: Option<usize>, centre_certain: bool, construction: bool, handle: Option<u16>, sense: Option<i32> },
    /// A full ellipse stored by a point on it (`rim`), its centre and the end points of its axes (the stream gives the indices of
    /// four points; two of them, each perpendicular to the other from the centre, are on the axes). `major` is the point at the
    /// end of the major semi-axis, `minor` the minor semi-axis in metres. Only an ellipse whose points all check is read.
    Ellipse { rim: usize, centre: usize, major: usize, minor: f64, construction: bool, handle: Option<u16> },
    /// A curve whose class could not be told (the end points are given).
    Unresolved { a: usize, b: usize, construction: bool },
}

/// What a geometric relation says. The type codes below were checked against the geometry of every sample sketch whose
/// targets decode (counts in `docs/sldprt-sketch.md`): a code whose records all satisfy one predicate is named for it. Any
/// other code is kept as [`RelationKind::Other`] with its number (17, the commonest, takes a single line, arc or point and
/// is probably a fix; 13 relates two lines and a point; 7, 16 and 32 relate two lines that are construction lines, whose
/// handles this reader cannot match).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RelationKind {
    /// Type 4: a line is horizontal in the sketch.
    Horizontal,
    /// Type 5: a line is vertical.
    Vertical,
    /// Type 9: two points coincide, or a point lies on a line or arc.
    Coincident,
    /// Type 27: two lines are collinear.
    Collinear,
    /// Types 14 and 57: two lines have equal length, or two arcs equal radius (the two codes were not told apart).
    Equal,
    /// Type 6: a line and an arc, or two arcs, are tangent.
    Tangent,
    /// Type 12: a point is the midpoint of a line.
    Midpoint,
    /// Type 11: two points are symmetric about a line.
    Symmetric,
    /// Type 25: two points have the same y.
    HorizontalPoints,
    /// Type 26: two points have the same x.
    VerticalPoints,
    Other(u32),
}

/// What a relation refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RelTarget {
    /// A point, by index into [`Sketch::points`] (the point handle's id is the record position).
    Point(usize),
    /// A line or arc by the id of its handle (see the `handle` of [`SkCurve`]).
    Line(u16),
    Arc(u16),
    /// A handle of a class this reader did not place.
    Other(u16),
}

#[derive(Debug, Clone, PartialEq)]
pub struct Relation {
    pub kind: RelationKind,
    pub targets: Vec<RelTarget>,
}

/// A named parameter with a numeric value (a sketch dimension `D2` or an extrude depth).
#[derive(Debug, Clone, PartialEq)]
pub struct Param {
    pub name: String,
    /// SI value as stored: metres or radians.
    pub value: f64,
    /// Byte offset in the stream (for ordering).
    pub offset: usize,
    /// Byte offset just after the value.
    pub end: usize,
}

/// What a dimension measures. The kind is settled by the operands and the value: the value must equal the measure on the
/// geometry the sketch stores (see [`Dimension`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DimKind {
    /// Distance between two points, or from a point to a line.
    Distance,
    /// Horizontal / vertical distance between two points (along the sketch's x / y axis).
    HDist,
    VDist,
    /// Radius or diameter of a circle or arc.
    Radius,
    Diameter,
    /// Angle between two lines (radians).
    Angle,
}

/// A driving dimension of a sketch: the operands are the handles written after the parameter record, and the dimension is kept
/// only when the measure it names on the sketch's geometry equals the parameter's value (to 1e-6 relative).
#[derive(Debug, Clone, PartialEq)]
pub struct Dimension {
    /// The parameter's own name (`D1`).
    pub name: String,
    pub kind: DimKind,
    /// Metres or radians as stored.
    pub value: f64,
    pub targets: Vec<RelTarget>,
    /// Stream offset of the parameter record.
    pub offset: usize,
}

/// The plane a sketch is on, as far as the stream says. The stream lists every plane or feature the sketch refers to (its
/// own plane, the planes of dimension anchors, the feature whose face it sits on); which one is the sketch's own is settled
/// here only when exactly one of them is a plane feature and none is another feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneRef {
    /// A plane feature (the id as in the feature tree; 2, 3 and 4 are Front, Top and Right in the parts looked at).
    Plane(u32),
    /// A feature that is not a plane (the sketch is on one of its faces).
    OnFeature(u32),
    /// Several candidates; [`Sketch::plane_refs`] lists them.
    Ambiguous,
    /// Not resolved yet, or no reference record found.
    Unknown,
}

/// Where a sketch's axes point in model space (metres; the sketch plane through the model origin, the offset along the normal
/// is not part of the frame).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneFrame {
    /// Model direction of the sketch's x axis and y axis.
    pub x: [f64; 3],
    pub y: [f64; 3],
    /// The model point of sketch `(0, 0)` (metres): zero for the standard planes, the stored translation for a plane the
    /// stream gives a transform for.
    pub o: [f64; 3],
    /// True when sample bodies confirm the mapping (a sketch with points off the origin matched body vertices under it and
    /// under no other of the eight axis assignments); false when it follows from the plane convention alone.
    pub verified: bool,
}

impl PlaneRef {
    /// The frame of the three standard planes (feature ids 2, 3 and 4 in every sample part: Front, Top, Right). Front (x to +X,
    /// y to +Y) and Right (x to -Z, y to +Y) are confirmed on sample bodies. Top (x to +X, y to -Z, normal +Y) is the
    /// right-handed choice with SolidWorks' +Y normal and was not told apart from its mirror by any sample (the Top sketches
    /// of the samples are symmetric about the origin). Offset planes and faces: `None`.
    pub fn standard_frame(self) -> Option<PlaneFrame> {
        match self {
            PlaneRef::Plane(2) => Some(PlaneFrame { x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], o: [0.0; 3], verified: true }),
            PlaneRef::Plane(3) => Some(PlaneFrame { x: [1.0, 0.0, 0.0], y: [0.0, 0.0, -1.0], o: [0.0; 3], verified: true }),
            PlaneRef::Plane(4) => Some(PlaneFrame { x: [0.0, 0.0, -1.0], y: [0.0, 1.0, 0.0], o: [0.0; 3], verified: true }),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Sketch {
    pub name: String,
    /// The feature id of the sketch (as in the `KeyWords` tree and the `FACE_ID` tags).
    pub feature_id: u16,
    pub plane: PlaneRef,
    /// Every feature id the sketch's plane-reference records name, in stream order (small numbers; save stamps and `-1`
    /// are dropped). The first is often the origin reference record's own id, not a plane.
    pub plane_refs: Vec<u32>,
    /// The transform stored with a plane reference (feature id, row-major rotation with the axes as columns, translation): the
    /// records of the sketch's plane references that carry one (a `1` flag byte).
    pub plane_frames: Vec<(u32, [f64; 12])>,
    /// The frame the stream gives the sketch's plane when it is not one of the three standard ones (see [`Sketch::frame`]).
    pub stored_frame: Option<PlaneFrame>,
    pub points: Vec<SkPoint>,
    pub curves: Vec<SkCurve>,
    /// Dimension parameters found in the sketch's range of the stream.
    pub params: Vec<Param>,
    /// Geometric relations (horizontal, coincident, ...) in the order the stream lists them.
    pub relations: Vec<Relation>,
    /// Driving dimensions whose operands decoded and whose value matches the geometry.
    pub dims: Vec<Dimension>,
    /// The operands found after each parameter record (the parameter's index in [`Sketch::params`] and its operands), kept to
    /// classify the dimensions again once the circle centres are settled.
    pub raw_dims: Vec<(usize, Vec<RelTarget>)>,
    /// Stream offset of the sketch's feature record.
    pub offset: usize,
    /// What could not be read exactly (a curve with no end points, a centre that is a guess, ...); empty for a sketch that
    /// decoded cleanly.
    pub notes: Vec<String>,
}

impl Sketch {
    /// True when every curve is resolved and every arc and circle centre is certain.
    pub fn exact(&self) -> bool {
        self.notes.is_empty()
            && self.curves.iter().all(|c| match c {
                SkCurve::Line { .. } => true,
                SkCurve::Arc { centre_certain, centre, .. } | SkCurve::Circle { centre_certain, centre, .. } => *centre_certain && centre.is_some(),
                SkCurve::Ellipse { .. } => true,
                SkCurve::Unresolved { .. } => false,
            })
    }

    /// Settles [`Sketch::plane`] from the ids of the plane features and of all features of the file (the feature tree of
    /// `SwMeta`): see [`PlaneRef`]. `ignore` are ids that are named by every sketch without being its plane (the origin).
    pub fn resolve_plane(&mut self, planes: &[u32], features: &[u32], ignore: &[u32]) {
        let own = self.feature_id as u32;
        let mut pl: Vec<u32> = self.plane_refs.iter().copied().filter(|r| planes.contains(r)).collect();
        pl.dedup();
        pl.sort_unstable();
        pl.dedup();
        let mut ft: Vec<u32> = self.plane_refs.iter().copied().filter(|r| *r != own && features.contains(r) && !planes.contains(r) && !ignore.contains(r)).collect();
        ft.sort_unstable();
        ft.dedup();
        self.plane = match (pl.as_slice(), ft.as_slice()) {
            ([p], []) => PlaneRef::Plane(*p),
            ([], [f]) => PlaneRef::OnFeature(*f),
            ([], []) => PlaneRef::Unknown,
            _ => PlaneRef::Ambiguous,
        };
        // a plane that is not a standard one: the transform stored with the reference to it (the sketch point `s` is at
        // `R (s - t)` in the model: the axes are the columns of `R`, the origin `-R t`)
        let id = match self.plane {
            PlaneRef::Plane(i) | PlaneRef::OnFeature(i) if self.plane.standard_frame().is_none() => Some(i),
            _ => None,
        };
        self.stored_frame = id.and_then(|i| {
            let mut found = self.plane_frames.iter().filter(|(k, _)| *k == i).map(|(_, m)| *m).collect::<Vec<_>>();
            found.dedup();
            let [m] = found[..] else { return None };
            let t = [m[9], m[10], m[11]];
            let r = |a: usize| -> f64 { -(m[3 * a] * t[0] + m[3 * a + 1] * t[1] + m[3 * a + 2] * t[2]) };
            Some(PlaneFrame { x: [m[0], m[3], m[6]], y: [m[1], m[4], m[7]], o: [r(0), r(1), r(2)], verified: false })
        });
    }

    /// The frame of the sketch's plane: one of the three standard planes', else the transform stored with the sketch's plane
    /// reference ([`Sketch::stored_frame`]).
    pub fn frame(&self) -> Option<PlaneFrame> {
        self.plane.standard_frame().or(self.stored_frame)
    }

    /// The model-space position (metres) of sketch point `(x, y)`, when the plane's frame is known.
    pub fn to_model(&self, x: f64, y: f64) -> Option<[f64; 3]> {
        let f = self.frame()?;
        Some([f.x[0] * x + f.y[0] * y + f.o[0], f.x[1] * x + f.y[1] * y + f.o[1], f.x[2] * x + f.y[2] * y + f.o[2]])
    }

    /// The radius of a circle or arc (metres), when its centre is known.
    pub fn radius(&self, c: &SkCurve) -> Option<f64> {
        let (p, ctr) = match c {
            SkCurve::Arc { start, centre: Some(ct), .. } => (*start, *ct),
            SkCurve::Circle { rim, centre: Some(ct), .. } => (*rim, *ct),
            _ => return None,
        };
        let (a, b) = (self.points.get(p)?, self.points.get(ctr)?);
        Some((a.x - b.x).hypot(a.y - b.y))
    }
}

/// A feature record of the stream: a named object followed by its fields up to the next one.
#[derive(Debug, Clone, PartialEq)]
pub struct FeatureRec {
    pub name: String,
    pub id: u16,
    /// The three bytes after the name's four zero bytes: for an extrusion the family word (`0x0140` boss, `0x01ca` cut) and
    /// an operation byte; zero for most other classes (the same fields `classes.rs` reads to tell a cut from a boss).
    pub word: u16,
    pub op: u8,
    /// Start of the record (the class tag in front of the name) and of the next feature record (or the stream end).
    pub offset: usize,
    pub end: usize,
    /// Class names declared inside the record (the first object of a class declares it; later objects use a tag, so an
    /// empty list does not mean the feature has no such class).
    pub classes: Vec<String>,
    /// The parameters (`D1`, ...) written inside the record, in order.
    pub params: Vec<Param>,
    /// The end type word of the feature's `moEndSpec_c` (an extrusion) or `moRevEndSpec_c` (a revolve): the u32 sixteen bytes
    /// after the object's class token. What each value means is settled against the bodies (`docs/sldprt-sketch.md`).
    pub end_type: Option<u32>,
    /// The start type word of the feature's `moFromEndSpec_c` (0: the sketch plane).
    pub from_type: Option<u32>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SketchSet {
    /// The stream numbers its objects from a start number in a u32 header (SolidWorks 2015 and later); older streams
    /// have a u16 header and slightly different entity records.
    pub modern: bool,
    pub features: Vec<FeatureRec>,
    pub sketches: Vec<Sketch>,
}

impl SketchSet {
    /// The sketch of feature `id`.
    pub fn sketch(&self, id: u16) -> Option<&Sketch> {
        self.sketches.iter().find(|s| s.feature_id == id)
    }

    /// The feature record named `name` (the first).
    pub fn feature(&self, name: &str) -> Option<&FeatureRec> {
        self.features.iter().find(|f| f.name == name)
    }

    /// The sketch a feature at stream position `index` of [`SketchSet::features`] is built on: the nearest sketch before it
    /// that no other feature of the same kind used. Plain adjacency: the stream does not say (a reference record exists
    /// but its layout is not settled), so the caller must verify the choice against the body.
    pub fn sketch_before(&self, index: usize) -> Option<&Sketch> {
        let at = self.features.get(index)?.offset;
        self.sketches.iter().filter(|s| s.offset < at).max_by_key(|s| s.offset)
    }
}

fn u16_at(d: &[u8], i: usize) -> Option<u16> {
    Some(u16::from_le_bytes(d.get(i..i + 2)?.try_into().ok()?))
}
fn u32_at(d: &[u8], i: usize) -> Option<u32> {
    Some(u32::from_le_bytes(d.get(i..i + 4)?.try_into().ok()?))
}
fn i32_at(d: &[u8], i: usize) -> Option<i32> {
    Some(i32::from_le_bytes(d.get(i..i + 4)?.try_into().ok()?))
}
fn f64_at(d: &[u8], i: usize) -> Option<f64> {
    Some(f64::from_le_bytes(d.get(i..i + 8)?.try_into().ok()?))
}

/// The stream's header kind: a u32 start number then a u16 then the first class (modern), or a u16 then the first class.
fn is_modern(d: &[u8]) -> bool {
    d.get(6..8) == Some(&[0xff, 0xff]) && d.get(4..6) != Some(&[0xff, 0xff])
}

/// `FF FF 01 00 <len u16> <ASCII name>` at `i`: the class name and the position after it.
fn class_decl(d: &[u8], i: usize) -> Option<(&str, usize)> {
    if d.get(i..i + 4)? != [0xff, 0xff, 0x01, 0x00] {
        return None;
    }
    let l = u16_at(d, i + 4)? as usize;
    if !(3..=64).contains(&l) {
        return None;
    }
    let s = d.get(i + 6..i + 6 + l)?;
    if !s[0].is_ascii_alphabetic() && s[0] != b'_' || !s.iter().all(|c| c.is_ascii_alphanumeric() || *c == b'_') {
        return None;
    }
    Some((std::str::from_utf8(s).ok()?, i + 6 + l))
}

/// `FF FE FF <len> <UTF-16LE>` at `i`: the text and the position after it.
fn text_at(d: &[u8], i: usize) -> Option<(String, usize)> {
    if d.get(i..i + 3)? != [0xff, 0xfe, 0xff] {
        return None;
    }
    let l = *d.get(i + 3)? as usize;
    if l == 0 || l > 120 {
        return None;
    }
    let raw = d.get(i + 4..i + 4 + 2 * l)?;
    let u: Vec<u16> = raw.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
    if u.iter().any(|&x| x < 32) {
        return None;
    }
    Some((String::from_utf16_lossy(&u), i + 4 + 2 * l))
}

/// What follows a text field that names an object: four zero bytes, three bytes that are zero for most classes (an extrusion
/// writes a family word `0x0140` boss / `0x01ca` cut and an operation byte there), a flag byte (`0x40`, `0xc0`), then for a
/// feature a u16 id and two zero bytes; for a parameter `FF FF FF FF`.
enum Named {
    Feature { id: u16, word: u16, op: u8 },
    Param { value: f64, end: usize },
}

fn named_after(d: &[u8], j: usize, modern: bool) -> Option<Named> {
    if d.get(j..j + 4)? != [0; 4] {
        return None;
    }
    let flag = *d.get(j + 7)?;
    if !matches!(flag, 0x40 | 0x80 | 0xc0) {
        return None;
    }
    let mut k = j + 8;
    if d.get(k..k + 4)? == [0xff; 4] {
        // parameter: ff ff ff ff, then (modern) 4 zero bytes, an empty text, 2 zero bytes; older streams: 2 zero bytes. Then the value.
        k += 4;
        if modern {
            if d.get(k..k + 4)? != [0; 4] {
                return None;
            }
            k += 4;
            if d.get(k..k + 3) == Some(&[0xff, 0xfe, 0xff]) {
                k += 4 + 2 * (*d.get(k + 3)? as usize);
            }
        }
        if d.get(k..k + 2)? != [0, 0] {
            return None;
        }
        let v = f64_at(d, k + 2)?;
        if !v.is_finite() {
            return None;
        }
        return Some(Named::Param { value: v, end: k + 10 });
    }
    let id = u16_at(d, k)?;
    if d.get(k + 2..k + 4)? != [0, 0] {
        return None;
    }
    Some(Named::Feature { id, word: u16_at(d, j + 4)?, op: *d.get(j + 6)? })
}

/// Walks the stream the way `CArchive` numbers its objects (a new class takes a number and its first object the next, a
/// later object of a known class takes one), to give each class its tag. Fields whose layout is unknown can hold a
/// two-byte value that looks like a tag, so the numbers drift in big streams: callers treat the result as an estimate.
fn class_tags(d: &[u8], modern: bool) -> Vec<(String, u16)> {
    let mut out: Vec<(String, u16)> = vec![];
    let (start, mut pos) = if modern { (u32_at(d, 0).unwrap_or(0) as usize, 6) } else { (0, 2) };
    let mut num = start.saturating_sub(1);
    let mut known: HashMap<usize, ()> = HashMap::new();
    let n = d.len();
    while pos + 1 < n {
        if d[pos] == 0xff && d[pos + 1] == 0xff {
            if let Some((name, next)) = class_decl(d, pos) {
                num += 1;
                if num < 0x7fff {
                    out.push((name.to_string(), 0x8000 | num as u16));
                }
                known.insert(num, ());
                num += 1;
                pos = next;
                continue;
            }
        }
        if let Some(t) = u16_at(d, pos) {
            let k = (t & 0x7fff) as usize;
            if t & 0x8000 != 0 && t != 0xffff && (known.contains_key(&k) || (start > 0 && k > 0 && k < start)) {
                num += 1;
                pos += 2;
                continue;
            }
        }
        pos += 1;
    }
    out
}

fn handle_class_of(name: &str) -> HandleClass {
    if name == "sgPointHandle" {
        return HandleClass::Point;
    }
    curve_class_of(name).unwrap_or(HandleClass::Other)
}

fn curve_class_of(name: &str) -> Option<HandleClass> {
    match name {
        "sgLineHandle" => Some(HandleClass::Line),
        "sgArcHandle" => Some(HandleClass::Arc),
        n if n.starts_with("sg") && n.ends_with("Handle") && ["Ellipse", "Spline", "Parabola", "Conic", "Curve"].iter().any(|k| n.contains(k)) => Some(HandleClass::Other),
        _ => None,
    }
}

/// Raw entity record before classes are resolved.
struct RawPoint {
    id: u32,
    x: f64,
    y: f64,
    located: bool,
    name: Option<String>,
    counters: (u16, u16),
    handles: Vec<(Tag, u16)>,
    at: usize,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Tag {
    /// Class brought in inline by the handle (its name).
    Inline(HandleClass),
    /// A class tag of the stream.
    Tag(u16),
}

struct RawSeg {
    /// The six u16 after the sense: an arc or circle has its centre's index and then `FFFE FFFF` (four i32 -2); an ellipse its
    /// centre's index and the indices of the four end points of its axes.
    tail: [u16; 6],
    sense: Option<i32>,
    a: u16,
    c: u16,
    flag: u16,
    at: usize,
}

enum RawEnt {
    P(RawPoint),
    S(RawSeg),
}

/// Parses the handles listed after a point's counters: `<tag or class declaration> <u16 id> FF FF FF FF` (modern: then four
/// zero bytes). Stops at the first thing that is not a handle.
fn parse_handles(d: &[u8], mut k: usize, end: usize, modern: bool) -> Vec<(Tag, u16)> {
    let mut out = vec![];
    while out.len() < 4096 && k + 8 <= end {
        let tag = if d.get(k..k + 2) == Some(&[0xff, 0xff]) {
            match class_decl(d, k) {
                Some((name, next)) => {
                    k = next;
                    Tag::Inline(handle_class_of(name))
                }
                None => break,
            }
        } else {
            let t = match u16_at(d, k) {
                Some(t) if t & 0x8000 != 0 && t != 0xffff => t,
                _ => break,
            };
            k += 2;
            Tag::Tag(t)
        };
        let Some(id) = u16_at(d, k) else { break };
        if d.get(k + 2..k + 6) != Some(&[0xff; 4]) {
            break;
        }
        k += 6;
        // four zero bytes follow in newer streams (and in the relations of every version); a class tag always has its high
        // bit set, so the next handle cannot be mistaken for them
        if d.get(k..k + 4) == Some(&[0; 4]) {
            k += 4;
        } else if modern {
            break;
        }
        out.push((tag, id));
    }
    out
}

/// Entity records: `u32 id`, then the header `FF FF <schema u16> <version u8> FF*8`, fixed fields, `F0 3F` (the double 1.0's
/// top bytes), and either a point (`1E 00` then x and y) or a curve (three u16: first end, last end, flag; 0 = construction).
fn entities(d: &[u8], modern: bool, limit: usize) -> R<Vec<RawEnt>> {
    // headers
    let mut heads = vec![];
    let mut i = 4usize;
    while i + 13 <= d.len() {
        if d[i] == 0xff && d[i + 1] == 0xff && d[i + 2] <= 0x40 && d[i + 3] == 0 && (1..=8).contains(&d[i + 4]) && d[i + 5..i + 13] == [0xff; 8] {
            heads.push(i);
            if heads.len() > limit {
                return Err(SldprtError::new("limit.sketch_entities", format!("more than {limit} sketch entity records")));
            }
            i += 13;
        } else {
            i += 1;
        }
    }
    // The 8 zero bytes after `F0 3F` are there in the newest streams only: decide by the points of the whole stream.
    let (mut with, mut without) = (0usize, 0usize);
    for &h in &heads {
        let end = (h + 4096).min(d.len());
        if let Some(rel) = d.get(h + 13..end).and_then(|s| s.windows(2).position(|w| w == [0xf0, 0x3f])) {
            let k = h + 13 + rel + 2;
            if d.get(k..k + 2) == Some(&[0x1e, 0x00]) {
                without += 1;
            } else if d.get(k..k + 8) == Some(&[0; 8]) && d.get(k + 8..k + 10) == Some(&[0x1e, 0x00]) {
                with += 1;
            }
        }
    }
    let zeros = if with + without == 0 { modern } else { with >= without };
    let mut out = vec![];
    for (n, &h) in heads.iter().enumerate() {
        let end = heads.get(n + 1).copied().unwrap_or(d.len()).min(h + 4096);
        let Some(id) = u32_at(d, h - 4) else { continue };
        // the fixed fields of a point or curve record; other records with the same header (sketch headers, ...) have zeros there
        if d.get(h + 13..h + 17) != Some(&[0, 0, 0x80, 0xbf]) || d.get(h + 33..h + 35) != Some(&[0x80, 0xbf]) {
            continue;
        }
        // right after them: two zero bytes in a plain record, a class tag or declaration in a reference to another object
        if d.get(h + 35..h + 37).map_or(false, |s| s != [0, 0]) {
            let found = d.get(h + 35..end).and_then(|s| {
                (0..s.len().saturating_sub(2)).filter(|&o| s[o..o + 2] == [0xf0, 0x3f]).find_map(|o| {
                    let mut k = h + 35 + o + 2;
                    if zeros {
                        if d.get(k..k + 8) != Some(&[0; 8]) {
                            return None;
                        }
                        k += 8;
                    }
                    if d.get(k..k + 2) != Some(&[0x1e, 0x00]) {
                        return None;
                    }
                    let (x, y) = (f64_at(d, k + 2)?, f64_at(d, k + 10)?);
                    (x.is_finite() && y.is_finite()).then_some((k, x, y))
                })
            });
            let name = d.get(h + 35..found.map_or(end, |f| f.0)).and_then(|s| (0..s.len()).find_map(|o| text_at(s, o).map(|t| t.0)).filter(|t| t.contains('@')));
            let (x, y, located, c, hs) = match found {
                Some((k, x, y)) => (x, y, true, (u16_at(d, k + 18).unwrap_or(0), u16_at(d, k + 20).unwrap_or(0)), parse_handles(d, k + 22, end, modern)),
                None => (0.0, 0.0, false, (0, 0), vec![]),
            };
            out.push(RawEnt::P(RawPoint { id, x, y, located, name, counters: c, handles: hs, at: h }));
            continue;
        }
        // the first `F0 3F` after the fixed fields
        let Some(rel) = d.get(h + 13..end).and_then(|s| s.windows(2).position(|w| w == [0xf0, 0x3f])) else { continue };
        let f = h + 13 + rel;
        let mut k = f + 2;
        if zeros {
            if d.get(k..k + 8) != Some(&[0; 8]) {
                continue;
            }
            k += 8;
        }
        if d.get(k..k + 2) == Some(&[0x1e, 0x00]) {
            let (Some(x), Some(y)) = (f64_at(d, k + 2), f64_at(d, k + 10)) else { continue };
            if !(x.is_finite() && y.is_finite()) {
                continue;
            }
            let c1 = u16_at(d, k + 18).unwrap_or(0);
            let c2 = u16_at(d, k + 20).unwrap_or(0);
            // an external reference carries the name of what it refers to between the header and the coordinates
            let name = d.get(h + 13..f).and_then(|s| {
                (0..s.len()).find_map(|o| text_at(s, o).map(|t| t.0)).filter(|t| t.contains('@'))
            });
            let handles = parse_handles(d, k + 22, end, modern);
            out.push(RawEnt::P(RawPoint { id, x, y, located: true, name, counters: (c1, c2), handles, at: h }));
        } else if let (Some(a), Some(c), Some(fl)) = (u16_at(d, k), u16_at(d, k + 2), u16_at(d, k + 4)) {
            out.push(RawEnt::S(RawSeg { tail: std::array::from_fn(|i| u16_at(d, k + 20 + 2 * i).unwrap_or(0xffff)), sense: i32_at(d, k + 16), a, c, flag: fl, at: h }));
        }
    }
    Ok(out)
}

/// Feature records and parameter records: a text field preceded by a class tag (high bit set), followed by `Named`.
fn named_records(d: &[u8], modern: bool, limit: usize) -> R<(Vec<(usize, String, u16, u16, u8)>, Vec<Param>)> {
    let mut feats = vec![];
    let mut params = vec![];
    let mut i = 4usize;
    while i + 4 < d.len() {
        if d[i] == 0xff && d[i + 1] == 0xfe && d[i + 2] == 0xff {
            if let Some((name, j)) = text_at(d, i) {
                let tagged = u16_at(d, i - 2).map_or(false, |t| t & 0x8000 != 0 && t != 0xffff);
                if tagged {
                    match named_after(d, j, modern) {
                        Some(Named::Feature { id, word, op }) => {
                            feats.push((i - 2, name, id, word, op));
                            if feats.len() > limit {
                                return Err(SldprtError::new("limit.sketch_entities", format!("more than {limit} feature records")));
                            }
                        }
                        Some(Named::Param { value, end }) => {
                            params.push(Param { name, value, offset: i, end });
                            if params.len() > limit {
                                return Err(SldprtError::new("limit.sketch_entities", "too many parameter records"));
                            }
                        }
                        None => {}
                    }
                }
                i = j;
                continue;
            }
        }
        i += 1;
    }
    Ok((feats, params))
}

/// The plane feature a sketch refers to: the u32 before the file's save stamp in the last plane-reference record of its
/// range. A plane reference is the sub-record tag found after the first `moCompRefPlane_c` declaration, `02 00`, 41 zero
/// bytes, 16 bytes of `FF`, zeros, then the feature id and the stamp.
fn plane_refs(d: &[u8]) -> (Vec<u8>, Vec<(usize, u32, Option<[f64; 12]>)>) {
    let key = b"moCompRefPlane_c";
    let Some(p) = d.windows(key.len()).position(|w| w == key) else { return (vec![], vec![]) };
    let Some(sub) = d.get(p + key.len()..p + key.len() + 4).filter(|s| s[2..] == [2, 0]) else { return (vec![], vec![]) };
    let sub = sub.to_vec();
    let mut out = vec![];
    let mut i = 0;
    while i + 4 < d.len() {
        if d[i..i + 4] == sub[..] {
            // 16 bytes of 0xff within 120 bytes
            if let Some(r) = d.get(i + 4..(i + 140).min(d.len())).and_then(|s| s.windows(16).position(|w| w.iter().all(|&b| b == 0xff))) {
                let mut j = i + 4 + r + 16;
                let z = j;
                while d.get(j) == Some(&0) && j < z + 40 {
                    j += 1;
                }
                if let Some(id) = u32_at(d, j) {
                    // after the save stamp: `00 00`, the plane object's id, `00 00`, a flag byte, and with flag 1 twelve f64: the
                    // rotation (row by row, the axes as columns) and the translation
                    let fr = (d.get(j + 14) == Some(&1)).then(|| (0..12).map(|k| f64_at(d, j + 15 + 8 * k)).collect::<Option<Vec<f64>>>()).flatten().and_then(|v| {
                        let m: [f64; 12] = v.try_into().ok()?;
                        let col = |c: usize| [m[c], m[3 + c], m[6 + c]];
                        let (u, w, n) = (col(0), col(1), col(2));
                        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
                        let unit = |a: [f64; 3]| (dot(a, a) - 1.0).abs() < 1e-6;
                        (m.iter().all(|x| x.is_finite() && x.abs() < 100.0) && unit(u) && unit(w) && unit(n) && dot(u, w).abs() < 1e-6 && dot(u, n).abs() < 1e-6 && dot(w, n).abs() < 1e-6).then_some(m)
                    });
                    out.push((i, id, fr));
                }
            }
            i += 4;
        } else {
            i += 1;
        }
    }
    (sub, out)
}

/// Maps the tags seen in point handle lists to curve classes. The classes brought in inline by the lists are curve classes in
/// the order the stream declares them; observed tags are ordered the same way. When several classes are declared the observed
/// tags are matched to them in order (the nearest-by-number choice when fewer tags than classes were seen).
fn resolve_tags(observed: &mut Vec<u16>, declared: &[(String, u16)]) -> HashMap<u16, HandleClass> {
    observed.sort_unstable();
    observed.dedup();
    // curve classes in declaration order with their estimated tag
    let classes: Vec<(HandleClass, u16)> = declared.iter().filter_map(|(n, t)| curve_class_of(n).map(|c| (c, *t))).collect();
    let mut map = HashMap::new();
    if classes.is_empty() {
        return map;
    }
    // choose the monotone injective assignment of observed tags to classes with the least total distance
    fn best(obs: &[u16], cls: &[(HandleClass, u16)], i: usize, j: usize, cost: u64, cur: &mut Vec<(u16, HandleClass)>, out: &mut (u64, Vec<(u16, HandleClass)>)) {
        if i == obs.len() {
            if cost < out.0 {
                *out = (cost, cur.clone());
            }
            return;
        }
        if obs.len() - i > cls.len() - j.min(cls.len()) {
            return;
        }
        for k in j..cls.len() {
            let dist = (obs[i] as i64 - cls[k].1 as i64).unsigned_abs();
            cur.push((obs[i], cls[k].0));
            best(obs, cls, i + 1, k + 1, cost + dist, cur, out);
            cur.pop();
        }
    }
    if observed.len() <= classes.len() {
        let mut out = (u64::MAX, vec![]);
        best(observed, &classes, 0, 0, 0, &mut vec![], &mut out);
        for (t, c) in out.1 {
            map.insert(t, c);
        }
    }
    map
}

/// Reads the sketches of a `Config-N-ResolvedFeatures` stream.
pub fn parse(d: &[u8], limits: &Limits) -> R<SketchSet> {
    if d.len() > limits.max_stream {
        return Err(SldprtError::new("limit.stream", format!("feature stream of {} bytes exceeds the limit", d.len())));
    }
    if d.len() < 16 {
        return Err(SldprtError::new("sketch.malformed", "feature stream too short"));
    }
    let modern = is_modern(d);
    let ents = entities(d, modern, MAX_ENTITIES)?;
    let (feats, params) = named_records(d, modern, MAX_FEATURES)?;
    let declared = class_tags(d, modern);
    let mut observed: Vec<u16> = vec![];
    for e in &ents {
        if let RawEnt::P(p) = e {
            for (t, _) in &p.handles {
                if let Tag::Tag(t) = t {
                    observed.push(*t);
                }
            }
        }
    }
    let tag_class = resolve_tags(&mut observed, &declared);
    let (_, planes) = plane_refs(d);

    // feature ranges
    let mut features: Vec<FeatureRec> = vec![];
    let mut classes_at: Vec<(usize, usize, &str)> = vec![];
    let mut i = 0;
    while i + 6 < d.len() {
        if d[i] == 0xff && d[i + 1] == 0xff {
            if let Some((n, next)) = class_decl(d, i) {
                classes_at.push((i, next, n));
                i = next;
                continue;
            }
        }
        i += 1;
    }
    for (n, (off, name, id, word, op)) in feats.iter().enumerate() {
        let end = feats.get(n + 1).map_or(d.len(), |f| f.0);
        features.push(FeatureRec {
            name: name.clone(),
            id: *id,
            word: *word,
            op: *op,
            offset: *off,
            end,
            // a class is declared right before the first object of that class: the declaration that ends where a record starts is
            // that record's, not the previous one's
            classes: classes_at.iter().filter(|(o, e, _)| (o >= off && *e != end) || *e == *off).map(|(_, _, n)| n.to_string()).filter({
                let _ = &end;
                |_| true
            }).collect(),
            params: params.iter().filter(|p| p.offset >= *off && p.offset < end).cloned().collect(),
            end_type: None,
            from_type: None,
        });
    }

    // the class token of `moEndSpec_c`: the walk of `class_tags` drifts in a big stream, so the token is taken from the stream itself:
    // the one that stands in front of the object's fixed start (`00 00 01 00`, an end type below 16 sixteen bytes on), as often as
    // there are extrusion features after the first (which declares the class instead of using a token), nearest the estimate
    let n_ext = features.iter().filter(|f| f.word == 0x0140 || f.word == 0x01ca).count();
    let est_end = declared.iter().find(|(n, _)| n == "moEndSpec_c").map(|(_, t)| *t);
    let end_tag = if n_ext > 1 {
        let mut count: HashMap<u16, usize> = HashMap::new();
        for i in 0..d.len().saturating_sub(24) {
            let t = u16::from_le_bytes([d[i], d[i + 1]]);
            if t & 0x8000 != 0 && t != 0xffff && d[i + 2..i + 6] == [0, 0, 1, 0] && u32_at(d, i + 18).map_or(false, |v| v < 16) {
                *count.entry(t).or_default() += 1;
            }
        }
        // (a token is a small class number: the high bytes of a negative double look like one, `0xbfxx`, and are left out)
        let near = |t: u16| est_end.map_or(true, |e| (t as i64 - e as i64).abs() < 0x1000) && t < 0xbc00;
        let best = count.iter().filter(|(t, &c)| near(**t) && c >= ((n_ext - 1) / 2).max(1) && c <= n_ext).map(|(t, c)| (*t, *c)).max_by_key(|(t, c)| (*c, std::cmp::Reverse(est_end.map_or(0, |e| (*t as i64 - e as i64).abs()))));
        best.map(|b| b.0).or(est_end)
    } else {
        est_end
    };
    let mut declared_end = declared.clone();
    if let Some(t) = end_tag {
        declared_end.retain(|(n, _)| n != "moEndSpec_c");
        declared_end.push(("moEndSpec_c".to_string(), t));
    }
    for f in features.iter_mut() {
        f.end_type = object_word(d, f, &declared_end, if f.classes.iter().any(|c| c == "moRevEndSpec_c") { "moRevEndSpec_c" } else { "moEndSpec_c" }, 16, &[0, 0, 1, 0]).or_else(|| object_word(d, f, &declared, "moRevEndSpec_c", 0, &[]));
        f.from_type = object_word(d, f, &declared, "moFromEndSpec_c", 0, &[]);
    }
    let mut sketches = vec![];
    for f in &features {
        let in_range: Vec<&RawEnt> = ents.iter().filter(|e| {
            let at = match e {
                RawEnt::P(p) => p.at,
                RawEnt::S(s) => s.at,
            };
            at >= f.offset && at < f.end
        }).collect();
        let pts: Vec<&RawPoint> = in_range.iter().filter_map(|e| if let RawEnt::P(p) = e { Some(p) } else { None }).collect();
        if pts.is_empty() {
            continue;
        }
        let segs: Vec<&RawSeg> = in_range.iter().filter_map(|e| if let RawEnt::S(s) = e { Some(s) } else { None }).collect();
        let mut sk = build_sketch(f, &pts, &segs, &tag_class, &planes);
        let (rels, raw_dims) = relations(d, f, modern, &tag_class, &declared);
        sk.relations = rels;
        resolve_centres(&mut sk);
        sk.dims = classify_dims(&sk, &f.params, &raw_dims, &|_| true);
        sk.raw_dims = raw_dims;
        sketches.push(sk);
    }
    Ok(SketchSet { modern, features, sketches })
}

/// The u32 at `at` bytes after the first object of class `name` inside the feature record `f`: the object is introduced by its
/// class declaration (the first object of the class in the stream) or by the class token the walk of `class_tags` gave it (a
/// token is an estimate, so the bytes right after it must start with `prefix`).
fn object_word(d: &[u8], f: &FeatureRec, declared: &[(String, u16)], name: &str, at: usize, prefix: &[u8]) -> Option<u32> {
    let end = f.end.min(d.len());
    let mut i = f.offset;
    let tag = declared.iter().find(|(n, _)| n == name).map(|(_, t)| *t);
    while i + 6 < end {
        if d[i] == 0xff && d[i + 1] == 0xff {
            if let Some((n, next)) = class_decl(d, i) {
                if n == name {
                    return u32_at(d, next + at);
                }
                i = next;
                continue;
            }
        }
        if Some(u16_at(d, i)?) == tag && d.get(i + 2..i + 2 + prefix.len()) == Some(prefix) && !prefix.is_empty() {
            return u32_at(d, i + 2 + at);
        }
        i += 1;
    }
    None
}

fn class_of(t: Tag, m: &HashMap<u16, HandleClass>) -> HandleClass {
    match t {
        Tag::Inline(c) => c,
        Tag::Tag(t) => m.get(&t).copied().unwrap_or(HandleClass::Other),
    }
}

fn build_sketch(f: &FeatureRec, pts: &[&RawPoint], segs: &[&RawSeg], tc: &HashMap<u16, HandleClass>, planes: &[(usize, u32, Option<[f64; 12]>)]) -> Sketch {
    let mut notes = vec![];
    let mut points: Vec<SkPoint> = pts
        .iter()
        .map(|p| SkPoint {
            id: p.id,
            x: p.x,
            y: p.y,
            located: p.located,
            role: if p.name.is_some() || p.counters.0 == 6 { PointRole::External } else if p.counters == (2, 2) { PointRole::Rim } else { PointRole::Free },
            name: p.name.clone(),
            counters: p.counters,
            on: p.handles.iter().map(|(t, id)| CurveHandle { class: class_of(*t, tc), id: *id }).collect(),
        })
        .collect();
    let n = points.len();
    let mut curves = vec![];
    // handles already given to a curve: two curves over the same end points (a line and the arc of a half circle) each take one
    let mut used: Vec<CurveHandle> = vec![];
    let mut skipped = 0usize;
    for s in segs {
        let (a, c) = (s.a as usize, s.c as usize);
        // the sketch's own header record (not a curve)
        if s.a == 31 && s.c == 0 && s.flag == 0 {
            continue;
        }
        let construction = s.flag == 0;
        // a curve record's flag is 0 (construction) or 1; other values and indices outside the point list belong to records
        // of the same shape that are not sketch curves (counted, not read)
        if s.flag > 1 || a >= n || c >= n {
            skipped += 1;
            continue;
        }
        // (an arc or circle record has `FFFE FFFF` after its centre's index; an ellipse the indices of four distinct points)
        let axes = &s.tail[1..5];
        if a == c && s.tail[1] != 0xfffe && axes.iter().all(|&k| (k as usize) < n) && (0..4).all(|i| (0..i).all(|j| axes[i] != axes[j])) {
            // an ellipse: the rim point, the centre and the axis points are listed
            match ellipse_of(&points, a, &s.tail) {
                Some((centre, major, minor)) => {
                    for k in [centre, major] {
                        points[k].role = PointRole::Axis;
                    }
                    for &k in &s.tail[1..5] {
                        if (k as usize) < n && points[k as usize].role == PointRole::Free {
                            points[k as usize].role = PointRole::Axis;
                        }
                    }
                    let handle = points[a].on.iter().find(|h| !used.contains(h)).copied();
                    used.extend(handle);
                    points[a].role = PointRole::Rim;
                    curves.push(SkCurve::Ellipse { rim: a, centre, major, minor, construction, handle: handle.map(|h| h.id) });
                }
                None => {
                    notes.push("an ellipse whose axis points do not check".into());
                    curves.push(SkCurve::Unresolved { a, b: c, construction });
                }
            }
            continue;
        }
        if a == c {
            points[a].role = PointRole::Rim;
            let handle = points[a].on.iter().find(|h| h.class == HandleClass::Arc && !used.contains(h)).copied();
            used.extend(handle);
            curves.push(SkCurve::Circle { rim: a, centre: None, centre_certain: false, construction, handle: handle.map(|h| h.id), sense: s.sense });
            continue;
        }
        // the class: a handle both end points list and no earlier curve took
        let shared = points[a].on.iter().find(|h| points[c].on.contains(h) && !used.contains(h)).copied();
        used.extend(shared);
        match shared.map(|h| h.class) {
            Some(HandleClass::Arc) => {
                curves.push(SkCurve::Arc { start: a, end: c, centre: None, centre_certain: false, construction, handle: shared.map(|h| h.id), sense: s.sense });
            }
            Some(HandleClass::Line) | None => curves.push(SkCurve::Line { a, b: c, construction, handle: shared.map(|h| h.id), sense: s.sense }),
            Some(_) => {
                notes.push("a curve of a class this reader does not read (ellipse, spline, ...)".into());
                curves.push(SkCurve::Unresolved { a, b: c, construction });
            }
        }
    }
    if skipped > 0 {
        notes.push(format!("{skipped} curve-shaped records were not curves of this sketch"));
    }
    let plane_refs: Vec<u32> = planes.iter().filter(|(o, id, _)| *o >= f.offset && *o < f.end && *id <= 0xffff).map(|(_, id, _)| *id).collect();
    let plane_frames: Vec<(u32, [f64; 12])> = planes.iter().filter(|(o, id, _)| *o >= f.offset && *o < f.end && *id <= 0xffff).filter_map(|(_, id, fr)| fr.map(|m| (*id, m))).collect();
    Sketch { name: f.name.clone(), feature_id: f.id, plane: PlaneRef::Unknown, plane_refs, plane_frames, stored_frame: None, points, curves, params: f.params.clone(), relations: vec![], dims: vec![], raw_dims: vec![], offset: f.offset, notes }
}

/// The centre, the end of the major semi-axis and the minor semi-axis of the ellipse the record names: among the points the
/// record lists (the rim point and the four axis end points) two lie perpendicular to each other from the centre, and every
/// listed point lies on the ellipse with those semi-axes.
fn ellipse_of(points: &[SkPoint], rim: usize, tail: &[u16; 6]) -> Option<(usize, usize, f64)> {
    let centre = tail[0] as usize;
    let c = points.get(centre).filter(|p| p.located)?;
    let mut cands: Vec<usize> = vec![rim];
    for &k in &tail[1..5] {
        let k = k as usize;
        if k != centre && !cands.contains(&k) {
            cands.push(k);
        }
    }
    let at = |k: usize| points.get(k).filter(|p| p.located).map(|p| (p.x - c.x, p.y - c.y));
    let vs: Vec<(usize, (f64, f64))> = cands.iter().filter_map(|&k| at(k).map(|v| (k, v))).collect();
    if vs.len() != cands.len() {
        return None;
    }
    for i in 0..vs.len() {
        for j in i + 1..vs.len() {
            let (u, v) = (vs[i].1, vs[j].1);
            let (lu, lv) = (u.0.hypot(u.1), v.0.hypot(v.1));
            if lu < 1e-12 || lv < 1e-12 || (u.0 * v.0 + u.1 * v.1).abs() > 1e-9 * lu * lv {
                continue;
            }
            let (eu, ev) = ((u.0 / lu, u.1 / lu), (v.0 / lv, v.1 / lv));
            let ok = vs.iter().all(|(_, w)| {
                let (x, y) = ((w.0 * eu.0 + w.1 * eu.1) / lu, (w.0 * ev.0 + w.1 * ev.1) / lv);
                (x * x + y * y - 1.0).abs() < 1e-6
            });
            if ok {
                return Some(if lu >= lv { (centre, vs[i].0, lv) } else { (centre, vs[j].0, lu) });
            }
        }
    }
    None
}

/// The relations of a sketch: `u32 type`, `02 00 00 00 00 00 FE FF 00 00`, then the handles of what is related. The point
/// handle tag is estimated from the numbering (its distance below the line tag in the CArchive count is stable) because points
/// are named only by this handle class.
fn relations(d: &[u8], f: &FeatureRec, modern: bool, tc: &HashMap<u16, HandleClass>, declared: &[(String, u16)]) -> (Vec<Relation>, Vec<(usize, Vec<RelTarget>)>) {
    let mut raw: Vec<(u32, Vec<(Tag, u16)>)> = vec![];
    let end = f.end.min(d.len());
    let mut i = f.offset;
    while i + 16 <= end {
        // `<u32 type> 02 00 00 00`, two (2015 to 2022 streams) or four (2016 streams) zero bytes, `FE FF 00 00`
        if d[i + 4..i + 8] == [2, 0, 0, 0] {
            let zc = if d[i + 8..i + 14] == [0, 0, 0xfe, 0xff, 0, 0] {
                Some(2)
            } else if d.get(i + 8..i + 16) == Some(&[0, 0, 0, 0, 0xfe, 0xff, 0, 0]) {
                Some(4)
            } else {
                None
            };
            if let (Some(zc), Some(t)) = (zc, u32_at(d, i)) {
                if t < 4096 {
                    let at = i + 8 + zc + 4;
                    raw.push((t, parse_handles(d, at, end.min(at + 200), modern)));
                    i = at;
                    continue;
                }
            }
        }
        i += 1;
    }
    // the handles written after each parameter record (the operands of a dimension): the first handle found after the value
    let mut raw_dims: Vec<(usize, Vec<(Tag, u16)>)> = vec![];
    for (pi, p) in f.params.iter().enumerate() {
        let stop = f.params.get(pi + 1).map_or(end, |q| q.offset).min(p.end + 240).min(end);
        let mut k = p.end;
        let mut found = vec![];
        while k + 8 <= stop {
            let hs = parse_handles(d, k, stop, modern);
            if !hs.is_empty() {
                found = hs;
                break;
            }
            k += 1;
        }
        raw_dims.push((pi, found));
    }
    let est = |n: &str| declared.iter().find(|(c, _)| c == n).map(|(_, t)| *t as i64);
    let obs = |c: HandleClass| tc.iter().filter(|(_, k)| **k == c).map(|(t, _)| *t as i64).min();
    let anchor = [("sgLineHandle", HandleClass::Line), ("sgArcHandle", HandleClass::Arc)].iter().find_map(|(n, c)| Some((est(n)?, obs(*c)?)));
    let want = match (est("sgPointHandle"), anchor) {
        (Some(p), Some((e, o))) => Some(o - (e - p)),
        _ => None,
    };
    let tag_of = |(t, _): &(Tag, u16)| if let Tag::Tag(t) = t { Some(*t) } else { None };
    let rel_unknown: Vec<u16> = raw.iter().flat_map(|(_, h)| h.iter()).filter_map(tag_of).filter(|t| !tc.contains_key(t)).collect();
    let dim_unknown: Vec<u16> = raw_dims.iter().flat_map(|(_, h)| h.iter()).filter_map(tag_of).filter(|t| !tc.contains_key(t)).collect();
    let pick = |unknown: &[u16]| want.and_then(|w| unknown.iter().copied().filter(|t| (*t as i64 - w).abs() <= 6).min_by_key(|t| (*t as i64 - w).abs()));
    // (the point handle tag comes from the relations; the dimensions' handles are used only where the relations give none)
    let pt_tag = pick(&rel_unknown).or_else(|| pick(&dim_unknown));
    let target = |(tag, id): (Tag, u16)| match tag {
        Tag::Inline(HandleClass::Point) => RelTarget::Point(id as usize),
        Tag::Inline(HandleClass::Line) => RelTarget::Line(id),
        Tag::Inline(HandleClass::Arc) => RelTarget::Arc(id),
        Tag::Inline(_) => RelTarget::Other(id),
        Tag::Tag(t) if Some(t) == pt_tag => RelTarget::Point(id as usize),
        Tag::Tag(t) => match tc.get(&t) {
            Some(HandleClass::Line) => RelTarget::Line(id),
            Some(HandleClass::Arc) => RelTarget::Arc(id),
            // (a class whose first object was declared inline in a point's list has no observed tag: the number the walk of
            // `class_tags` gave its declaration, exact in a small stream)
            _ => match declared.iter().find(|(_, x)| *x == t).map(|(n, _)| n.as_str()) {
                Some("sgLineHandle") => RelTarget::Line(id),
                Some("sgArcHandle") => RelTarget::Arc(id),
                Some("sgPointHandle") => RelTarget::Point(id as usize),
                _ => RelTarget::Other(id),
            },
        },
    };
    let dims: Vec<(usize, Vec<RelTarget>)> = raw_dims.into_iter().map(|(i, hs)| (i, hs.into_iter().map(target).collect())).collect();
    let rels = raw
        .into_iter()
        .map(|(t, hs)| Relation {
            kind: match t {
                4 => RelationKind::Horizontal,
                5 => RelationKind::Vertical,
                9 => RelationKind::Coincident,
                27 => RelationKind::Collinear,
                14 | 57 => RelationKind::Equal,
                6 => RelationKind::Tangent,
                12 => RelationKind::Midpoint,
                11 => RelationKind::Symmetric,
                25 => RelationKind::HorizontalPoints,
                26 => RelationKind::VerticalPoints,
                o => RelationKind::Other(o),
            },
            targets: hs.into_iter().map(target).collect(),
        })
        .collect();
    (rels, dims)
}

/// The dimensions whose operands decoded and whose parameter value is the measure of those operands on the sketch's own geometry.
/// A parameter record is written for the driving value and again for its display: the first that matches is kept, one dimension
/// per set of operands and kind.
fn classify_dims(sk: &Sketch, params: &[Param], raw: &[(usize, Vec<RelTarget>)], keep: &dyn Fn(&Param) -> bool) -> Vec<Dimension> {
    let mut out: Vec<Dimension> = vec![];
    let pt = |i: usize| sk.points.get(i).filter(|p| p.located).map(|p| (p.x, p.y));
    let line = |h: u16| sk.curves.iter().find_map(|c| match c {
        SkCurve::Line { a, b, handle: Some(x), .. } if *x == h => Some((pt(*a)?, pt(*b)?)),
        _ => None,
    });
    let arc = |h: u16| sk.curves.iter().find(|c| matches!(c, SkCurve::Arc { handle: Some(x), .. } | SkCurve::Circle { handle: Some(x), .. } if *x == h));
    let close = |a: f64, b: f64| (a - b).abs() <= 1e-9 + 1e-7 * b.abs();
    for (pi, targets) in raw {
        let Some(p) = params.get(*pi) else { continue };
        if !keep(p) {
            continue;
        }
        let v = p.value;
        if !v.is_finite() || targets.is_empty() || targets.iter().any(|t| matches!(t, RelTarget::Other(_))) {
            continue;
        }
        let kind = match targets.as_slice() {
            [RelTarget::Arc(h)] => arc(*h).and_then(|c| sk.radius(c)).and_then(|r| if close(v, r) { Some(DimKind::Radius) } else if close(v, 2.0 * r) { Some(DimKind::Diameter) } else { None }),
            [RelTarget::Point(a), RelTarget::Point(b)] => match (pt(*a), pt(*b)) {
                (Some(p), Some(q)) => {
                    let (dx, dy) = ((q.0 - p.0).abs(), (q.1 - p.1).abs());
                    if close(v, dx.hypot(dy)) {
                        Some(DimKind::Distance)
                    } else if close(v, dx) {
                        Some(DimKind::HDist)
                    } else if close(v, dy) {
                        Some(DimKind::VDist)
                    } else {
                        None
                    }
                }
                _ => None,
            },
            [RelTarget::Point(a), RelTarget::Line(h)] => match (pt(*a), line(*h)) {
                (Some(p), Some((l0, l1))) => {
                    let (ux, uy) = (l1.0 - l0.0, l1.1 - l0.1);
                    let l = ux.hypot(uy);
                    (l > 0.0 && close(v, ((p.0 - l0.0) * uy - (p.1 - l0.1) * ux).abs() / l)).then_some(DimKind::Distance)
                }
                _ => None,
            },
            [RelTarget::Line(h1), RelTarget::Line(h2)] => match (line(*h1), line(*h2)) {
                (Some((a0, a1)), Some((b0, b1))) => {
                    let (ux, uy) = (a1.0 - a0.0, a1.1 - a0.1);
                    let (wx, wy) = (b1.0 - b0.0, b1.1 - b0.1);
                    let (lu, lw) = (ux.hypot(uy), wx.hypot(wy));
                    if lu > 0.0 && lw > 0.0 {
                        let th = ((ux * wx + uy * wy) / (lu * lw)).clamp(-1.0, 1.0).acos();
                        let parallel = (ux * wy - uy * wx).abs() < 1e-9 * lu * lw;
                        if parallel && close(v, ((b0.0 - a0.0) * uy - (b0.1 - a0.1) * ux).abs() / lu) {
                            Some(DimKind::Distance)
                        } else if (close(v, th) || close(v, std::f64::consts::PI - th) || close(v, std::f64::consts::TAU - th)) && v < 7.0 {
                            Some(DimKind::Angle)
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                }
                _ => None,
            },
            _ => None,
        };
        if let Some(kind) = kind {
            if !out.iter().any(|d| d.kind == kind && d.targets == *targets) {
                out.push(Dimension { name: p.name.clone(), kind, value: v, targets: targets.clone(), offset: p.offset });
            }
        }
    }
    out
}

fn dist(points: &[SkPoint], a: usize, b: usize) -> f64 {
    (points[a].x - points[b].x).hypot(points[a].y - points[b].y)
}

/// A point that can be a centre: stored with no curve handle (a centre lies on no curve), not an end point.
fn centre_candidates(points: &[SkPoint]) -> Vec<usize> {
    (0..points.len()).filter(|&i| points[i].on.is_empty() && points[i].role != PointRole::Rim).collect()
}

/// Chooses the centre of every arc and circle of a sketch (the stream stores none: an arc is stored by its two end points, a
/// circle by one point on it, and a centre is an ordinary point with no curve handle).
///
/// * An arc's candidates are the handle-less points equidistant from both ends. A circle has no such test: its candidate is
///   the handle-less point written just before its rim point (in every sample), else the sketch's origin reference.
/// * Relations constrain the choice: arcs related as equal have the same radius, a tangent line is at the radius from the
///   centre, tangent arcs have their centres at the sum or difference of the radii. The assignments of candidates that satisfy
///   all of them are searched; when exactly one does, an arc's centre is certain.
/// * A circle's centre is certain only when it is the sketch's only possible centre (one circle, no arcs, one handle-less
///   point), or when a dimension of the sketch equals its diameter or radius ([`confirm_radius`]).
fn resolve_centres(s: &mut Sketch) {
    struct Item {
        curve: usize,
        /// The point the radius is measured from (an arc's start, a circle's rim) and the other end of an arc.
        p: usize,
        q: Option<usize>,
        handle: Option<u16>,
        cands: Vec<usize>,
        prefer: Option<usize>,
    }
    let n = s.points.len();
    let d = |a: usize, b: usize| (s.points[a].x - s.points[b].x).hypot(s.points[a].y - s.points[b].y);
    let free: Vec<usize> = (0..n).filter(|&i| s.points[i].located && s.points[i].on.is_empty() && s.points[i].role == PointRole::Free).collect();
    let origin = (0..n).find(|&k| s.points[k].role == PointRole::External && s.points[k].located && s.points[k].name.as_deref().map_or(false, |m| m.contains("Origin")));
    let mut items: Vec<Item> = vec![];
    for (ci, c) in s.curves.iter().enumerate() {
        match c {
            SkCurve::Arc { start, end, handle, .. } => {
                let (a, b) = (*start, *end);
                let tol = 1e-9 * d(a, b).max(1e-3);
                let eq = |k: usize| k != a && k != b && (d(k, a) - d(k, b)).abs() <= tol && d(k, a) > tol;
                let mut cands: Vec<usize> = free.iter().copied().filter(|&k| eq(k)).collect();
                if let Some(o) = origin.filter(|&o| eq(o)) {
                    cands.push(o);
                }
                let prefer = a.max(b).checked_sub(1).filter(|p| cands.contains(p));
                items.push(Item { curve: ci, p: a, q: Some(b), handle: *handle, cands, prefer });
            }
            SkCurve::Circle { rim, handle, .. } => {
                let r = *rim;
                let cands: Vec<usize> = match r.checked_sub(1).filter(|&p| free.contains(&p) && d(p, r) > 1e-12) {
                    Some(p) => vec![p],
                    None => origin.filter(|&o| d(o, r) > 1e-12).into_iter().collect(),
                };
                let prefer = cands.first().copied();
                items.push(Item { curve: ci, p: r, q: None, handle: *handle, cands, prefer });
            }
            _ => {}
        }
    }
    if items.is_empty() {
        return;
    }
    let item_of = |h: u16| items.iter().position(|i| i.handle == Some(h));
    let line_of = |h: u16| s.curves.iter().find_map(|c| if let SkCurve::Line { a, b, handle: Some(x), .. } = c { (*x == h).then_some((*a, *b)) } else { None });
    #[derive(Clone, Copy)]
    enum Con {
        EqualR(usize, usize),
        TanLine(usize, (usize, usize)),
        TanArc(usize, usize),
    }
    let mut cons: Vec<Con> = vec![];
    for r in &s.relations {
        match (r.kind, r.targets.as_slice()) {
            (RelationKind::Equal, [RelTarget::Arc(a), RelTarget::Arc(b)]) => {
                if let (Some(i), Some(j)) = (item_of(*a), item_of(*b)) {
                    cons.push(Con::EqualR(i, j));
                }
            }
            (RelationKind::Tangent, [RelTarget::Arc(a), RelTarget::Line(l)] | [RelTarget::Line(l), RelTarget::Arc(a)]) => {
                if let (Some(i), Some(ln)) = (item_of(*a), line_of(*l)) {
                    cons.push(Con::TanLine(i, ln));
                }
            }
            (RelationKind::Tangent, [RelTarget::Arc(a), RelTarget::Arc(b)]) => {
                if let (Some(i), Some(j)) = (item_of(*a), item_of(*b)) {
                    cons.push(Con::TanArc(i, j));
                }
            }
            _ => {}
        }
    }
    let radius = |it: &Item, c: usize| d(it.p, c);
    let ok = |asg: &[Option<usize>], k: usize| -> bool {
        let eps = |r: f64| 1e-9 * r.max(1e-3);
        cons.iter().all(|c| match *c {
            Con::EqualR(i, j) if i == k || j == k => match (asg[i], asg[j]) {
                (Some(a), Some(b)) => (radius(&items[i], a) - radius(&items[j], b)).abs() <= eps(radius(&items[i], a)),
                _ => true,
            },
            Con::TanLine(i, (la, lb)) if i == k => match asg[i] {
                Some(ctr) => {
                    let (u, v) = ((s.points[lb].x - s.points[la].x), (s.points[lb].y - s.points[la].y));
                    let len = u.hypot(v).max(1e-30);
                    let off = ((s.points[ctr].x - s.points[la].x) * v - (s.points[ctr].y - s.points[la].y) * u).abs() / len;
                    (off - radius(&items[i], ctr)).abs() <= eps(off)
                }
                None => true,
            },
            Con::TanArc(i, j) if i == k || j == k => match (asg[i], asg[j]) {
                (Some(a), Some(b)) => {
                    let (r1, r2, dc) = (radius(&items[i], a), radius(&items[j], b), d(a, b));
                    (dc - (r1 + r2)).abs() <= eps(dc) || (dc - (r1 - r2).abs()).abs() <= eps(dc)
                }
                _ => true,
            },
            _ => true,
        })
    };
    // depth-first search over the items that have candidates, fewest first
    let order: Vec<usize> = {
        let mut o: Vec<usize> = (0..items.len()).filter(|&i| !items[i].cands.is_empty()).collect();
        o.sort_by_key(|&i| items[i].cands.len());
        o
    };
    fn dfs(order: &[usize], at: usize, items: &[Item], asg: &mut Vec<Option<usize>>, ok: &dyn Fn(&[Option<usize>], usize) -> bool, sols: &mut Vec<Vec<Option<usize>>>, nodes: &mut usize) {
        if sols.len() >= 2 || *nodes > 50_000 {
            return;
        }
        *nodes += 1;
        let Some(&i) = order.get(at) else {
            sols.push(asg.clone());
            return;
        };
        for &c in &items[i].cands {
            asg[i] = Some(c);
            if ok(asg, i) {
                dfs(order, at + 1, items, asg, ok, sols, nodes);
            }
        }
        asg[i] = None;
    }
    let mut sols: Vec<Vec<Option<usize>>> = vec![];
    let (mut asg, mut nodes) = (vec![None; items.len()], 0usize);
    dfs(&order, 0, &items, &mut asg, &ok, &mut sols, &mut nodes);
    let unique = sols.len() == 1 && nodes <= 50_000;
    let choice: Vec<Option<usize>> = match sols.first() {
        Some(sol) if unique || sols.len() > 1 => sol.clone(),
        _ => items.iter().map(|i| i.prefer.or(i.cands.first().copied())).collect(),
    };
    let arcs = items.iter().filter(|i| i.q.is_some()).count();
    let circles = items.len() - arcs;
    for (k, it) in items.iter().enumerate() {
        let (centre, mut certain) = (choice[k], false);
        if let Some(_c) = centre {
            certain = if it.q.is_some() {
                unique || it.cands.len() == 1 && sols.len() <= 1
            } else {
                unique && it.cands.len() == 1 && circles == 1 && arcs == 0 && free.len() == 1
            };
        }
        match &mut s.curves[it.curve] {
            SkCurve::Arc { centre: c, centre_certain, .. } | SkCurve::Circle { centre: c, centre_certain, .. } => {
                *c = centre;
                *centre_certain = certain;
            }
            _ => {}
        }
    }
}

/// Marks the centres of circles as certain where a parameter of the sketch equals their diameter or radius (both stored in
/// metres; a diameter dimension `D` stores the diameter). Returns how many were confirmed.
pub fn confirm_radius(s: &mut Sketch) -> usize {
    let mut n = 0;
    let vals: Vec<f64> = s.params.iter().map(|p| p.value).collect();
    for k in 0..s.curves.len() {
        let SkCurve::Circle { rim, centre: Some(c), centre_certain: false, .. } = s.curves[k].clone() else { continue };
        let r = dist(&s.points, rim, c);
        if vals.iter().any(|v| (v - r).abs() < 1e-9 || (v - 2.0 * r).abs() < 1e-9) {
            if let SkCurve::Circle { centre_certain, .. } = &mut s.curves[k] {
                *centre_certain = true;
                n += 1;
            }
        }
    }
    n
}

/// The `KeyWords` tree lists lengths in the document's display unit: how many of them are a metre, for the units the tree is
/// compared in.
pub fn tree_scale(unit: Option<&str>) -> Option<f64> {
    match unit? {
        "mm" => Some(1e3),
        "cm" => Some(1e2),
        "m" => Some(1.0),
        "in" => Some(1.0 / 0.0254),
        "ft" => Some(1.0 / 0.3048),
        _ => None,
    }
}

/// The `Contents/Config-<n>-ResolvedFeatures` stream of an opened container (the lowest-numbered configuration).
pub fn read(c: &Container, limits: &Limits) -> R<SketchSet> {
    let base = c.streams.iter().map(|s| s.name.as_str()).filter(|n| n.contains("Config-") && n.contains("ResolvedFeatures")).min_by_key(|n| (!n.contains("Config-0-"), n.len()));
    let Some(b) = base else { return Err(SldprtError::new("sketch.no_stream", "the file has no ResolvedFeatures stream")) };
    let d = c.data(b.strip_suffix("__ZLB").unwrap_or(b), limits).ok_or_else(|| SldprtError::new("sketch.no_stream", "the ResolvedFeatures stream cannot be unpacked"))?;
    let mut set = parse(&d, limits)?;
    let meta = crate::meta::read(c, limits);
    let scale = tree_scale(meta.units.as_ref().and_then(|u| u.length.as_deref()));
    let planes: Vec<u32> = meta.features.iter().filter(|f| f.kind == "Plane" || f.kind == "RefPlane").filter_map(|f| u32::try_from(f.id).ok()).collect();
    let all: Vec<u32> = meta.features.iter().filter_map(|f| u32::try_from(f.id).ok()).collect();
    let origin: Vec<u32> = set.features.iter().filter(|f| f.classes.iter().any(|c| c == "moOriginProfileFeature_c")).map(|f| f.id as u32).collect();
    for s in &mut set.sketches {
        confirm_radius(s);
        // (a circle's centre may have changed: the dimensions are read again against the settled geometry)
        // the file's own tree lists each sketch's dimensions by name and value (millimetres, degrees): of the parameter records
        // that carry a dimension's name (a display record has another number), the one with the listed value is the driving one
        let tree: Vec<(String, f64)> = if scale.is_some() { meta.features.iter().filter(|kf| kf.name == s.name).flat_map(|kf| kf.dims.iter()).filter_map(|d| Some((d.name.clone(), d.value?))).collect() } else { vec![] };
        let keep = |p: &Param| {
            let listed: Vec<f64> = tree.iter().filter(|(n, _)| *n == p.name).map(|(_, v)| *v).collect();
            // (a parameter the tree does not list is not contradicted by it; one it lists must have a listed value, as millimetres or degrees)
            listed.is_empty() || listed.iter().any(|v| [p.value * scale.unwrap_or(1e3), p.value.to_degrees()].iter().any(|x| (x - v).abs() <= 1e-3 * (1.0 + v.abs())))
        };
        let (params, raw) = (s.params.clone(), s.raw_dims.clone());
        s.dims = classify_dims(s, &params, &raw, &keep);
        s.resolve_plane(&planes, &all, &origin);
    }
    Ok(set)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn le32(v: u32) -> [u8; 4] {
        v.to_le_bytes()
    }
    fn f(v: f64) -> [u8; 8] {
        v.to_le_bytes()
    }
    fn decl(name: &str) -> Vec<u8> {
        let mut o = vec![0xff, 0xff, 1, 0];
        o.extend((name.len() as u16).to_le_bytes());
        o.extend(name.as_bytes());
        o
    }
    fn text(s: &str) -> Vec<u8> {
        let mut o = vec![0xff, 0xfe, 0xff, s.len() as u8];
        for u in s.encode_utf16() {
            o.extend(u.to_le_bytes());
        }
        o
    }
    fn feature(tag: u16, name: &str, id: u16) -> Vec<u8> {
        let mut o = tag.to_le_bytes().to_vec();
        o.extend(text(name));
        o.extend([0u8; 7]);
        o.push(0x40);
        o.extend(id.to_le_bytes());
        o.extend([0, 0]);
        o
    }
    fn header(id: u32, modern: bool) -> Vec<u8> {
        let mut o = le32(id).to_vec();
        o.extend([0xff, 0xff, 0x1f, 0, 3]);
        o.extend([0xff; 8]);
        o.extend([0, 0, 0x80, 0xbf, 1, 0, 0, 0, 0, 0, 4, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0x80, 0xbf, 0, 0, 4, 0, 0, 0]);
        o.extend([0; 17]);
        o.extend([0xf0, 0x3f]);
        if modern {
            o.extend([0; 8]);
        }
        o
    }
    fn point(id: u32, x: f64, y: f64, c: (u16, u16), handles: &[(u16, u16)]) -> Vec<u8> {
        let mut o = header(id, true);
        o.extend([0x1e, 0]);
        o.extend(f(x));
        o.extend(f(y));
        o.extend(c.0.to_le_bytes());
        o.extend(c.1.to_le_bytes());
        for (t, i) in handles {
            o.extend(t.to_le_bytes());
            o.extend(i.to_le_bytes());
            o.extend([0xff; 4]);
            o.extend([0; 4]);
        }
        o.extend([0u8; 30]);
        o
    }
    fn seg(id: u32, a: u16, c: u16, flag: u16) -> Vec<u8> {
        let mut o = header(id, true);
        o.extend(a.to_le_bytes());
        o.extend(c.to_le_bytes());
        o.extend(flag.to_le_bytes());
        o.extend([0u8; 30]);
        o
    }
    fn param(tag: u16, name: &str, v: f64) -> Vec<u8> {
        let mut o = tag.to_le_bytes().to_vec();
        o.extend(text(name));
        o.extend([0u8; 7]);
        o.push(0x40);
        o.extend([0xff; 4]);
        o.extend([0; 4]);
        o.extend([0xff, 0xfe, 0xff, 0]);
        o.extend([0, 0]);
        o.extend(f(v));
        o
    }

    /// A modern stream with a 10 x 6 mm rectangle sketch (id 26), a circle of radius 2 mm about (5, 3) in it, and a
    /// feature with a depth parameter. Class tags follow the CArchive numbering: start 109, so `moProfileFeature_c` is 109,
    /// its object 110, the base-class tag 111, `sgPointHandle` 112 (0x8070), `sgLineHandle` 114 (0x8072), `sgArcHandle` 116 (0x8074).
    fn stream() -> Vec<u8> {
        let mut d = le32(109).to_vec();
        d.extend([0x13, 0]);
        d.extend(decl("moProfileFeature_c"));
        d.extend(feature(0x8004, "Sketch1", 26));
        d.extend(decl("sgPointHandle"));
        d.extend(decl("sgLineHandle"));
        d.extend(decl("sgArcHandle"));
        d.extend(point(1, 0.0, 0.0, (0, 2), &[(0x8072, 1), (0x8072, 2)]));
        d.extend(point(2, 0.010, 0.006, (0, 2), &[(0x8072, 3), (0x8072, 4)]));
        d.extend(point(3, 0.0, 0.006, (0, 2), &[(0x8072, 2), (0x8072, 3)]));
        d.extend(point(4, 0.010, 0.0, (0, 2), &[(0x8072, 1), (0x8072, 4)]));
        d.extend(point(6, 0.005, 0.003, (0, 3), &[]));
        d.extend(point(5, 0.007, 0.003, (2, 2), &[(0x8074, 0)]));
        // a second handle-less point: the circle's centre is no longer the sketch's only possible one
        d.extend(point(7, 0.001, 0.001, (0, 1), &[]));
        d.extend(seg(1, 0, 3, 1));
        d.extend(seg(2, 0, 2, 1));
        d.extend(seg(3, 2, 1, 1));
        d.extend(seg(4, 3, 1, 1));
        d.extend(seg(1, 5, 5, 1));
        d.extend(param(0x8004, "D1", 0.004));
        d.extend(decl("moExtrusion_c"));
        d.extend(feature(0x80c2, "Boss-Extrude1", 32));
        d.extend(param(0x8004, "D1", 0.025));
        d
    }

    #[test]
    fn reads_a_rectangle_and_a_circle() {
        let set = parse(&stream(), &Limits::default()).unwrap();
        assert!(set.modern);
        assert_eq!(set.features.len(), 2);
        let s = set.sketch(26).expect("sketch 26");
        assert_eq!(s.name, "Sketch1");
        assert_eq!(s.points.len(), 7);
        assert_eq!(s.points[1].x, 0.010);
        let lines = s.curves.iter().filter(|c| matches!(c, SkCurve::Line { .. })).count();
        assert_eq!(lines, 4);
        let circ = s.curves.iter().find_map(|c| if let SkCurve::Circle { centre, rim, .. } = c { Some((*centre, *rim)) } else { None }).unwrap();
        assert_eq!(circ, (Some(4), 5));
        assert!((s.radius(s.curves.last().unwrap()).unwrap() - 0.002).abs() < 1e-12);
        // the parameter D1 = 4 mm is the circle's diameter: confirms the centre
        let mut s2 = s.clone();
        assert!(!s2.exact());
        assert_eq!(confirm_radius(&mut s2), 1);
        assert!(s2.exact());
        // the extrude feature's own parameter
        let e = set.feature("Boss-Extrude1").unwrap();
        assert_eq!(e.params.len(), 1);
        assert_eq!(e.params[0].value, 0.025);
        assert_eq!(set.sketch_before(1).unwrap().feature_id, 26);
    }

    #[test]
    fn never_panics_on_truncation_or_noise() {
        let d = stream();
        for n in 0..d.len() {
            let _ = parse(&d[..n], &Limits::default());
        }
        let mut x = d.clone();
        for i in 0..x.len() {
            let keep = x[i];
            x[i] = x[i].wrapping_add(0x55);
            let _ = parse(&x, &Limits::default());
            x[i] = keep;
        }
    }

    #[test]
    fn limits_and_errors_have_codes() {
        let l = Limits { max_stream: 10, ..Limits::default() };
        assert_eq!(parse(&stream(), &l).unwrap_err().code, "limit.stream");
        assert_eq!(parse(&[0; 4], &Limits::default()).unwrap_err().code, "sketch.malformed");
    }

    #[test]
    fn an_arc_gets_its_centre_from_the_record_before_its_last_end() {
        // points: centre (0,0), start (0.01,0), end (0,0.01); arc handles on both ends
        let mut d = le32(109).to_vec();
        d.extend([0x13, 0]);
        d.extend(decl("moProfileFeature_c"));
        d.extend(feature(0x8004, "Sketch1", 26));
        d.extend(decl("sgPointHandle"));
        d.extend(decl("sgLineHandle"));
        d.extend(decl("sgArcHandle"));
        d.extend(point(2, 0.0, 0.0, (0, 1), &[]));
        d.extend(point(1, 0.010, 0.0, (0, 1), &[(0x8074, 0)]));
        d.extend(point(3, 0.0, 0.010, (0, 1), &[(0x8074, 0)]));
        d.extend(seg(1, 1, 2, 1));
        let set = parse(&d, &Limits::default()).unwrap();
        let s = &set.sketches[0];
        match &s.curves[0] {
            SkCurve::Arc { centre, centre_certain, .. } => {
                assert_eq!(*centre, Some(0));
                assert!(*centre_certain);
            }
            c => panic!("{c:?}"),
        }
    }

    fn seg_full(id: u32, a: u16, c: u16, flag: u16, sense: i32, tail: [u16; 6]) -> Vec<u8> {
        let mut o = header(id, true);
        o.extend(a.to_le_bytes());
        o.extend(c.to_le_bytes());
        o.extend(flag.to_le_bytes());
        o.extend([0, 0]);
        o.extend(f(-1.0));
        o.extend(sense.to_le_bytes());
        for t in tail {
            o.extend(t.to_le_bytes());
        }
        o.extend([0u8; 20]);
        o
    }
    fn handle(tag: u16, id: u16) -> Vec<u8> {
        let mut o = tag.to_le_bytes().to_vec();
        o.extend(id.to_le_bytes());
        o.extend([0xff; 4]);
        o.extend([0; 4]);
        o
    }

    /// A circle of radius 2 mm about (5, 3) whose diameter dimension (D1 = 4 mm) names the circle's handle, and a distance
    /// between two points named by two point handles.
    fn dimensioned() -> Vec<u8> {
        let mut d = le32(109).to_vec();
        d.extend([0x13, 0]);
        d.extend(decl("moProfileFeature_c"));
        d.extend(feature(0x8004, "Sketch1", 26));
        d.extend(decl("sgPointHandle"));
        d.extend(decl("sgLineHandle"));
        d.extend(decl("sgArcHandle"));
        d.extend(point(1, 0.0, 0.0, (0, 1), &[(0x8072, 1)]));
        d.extend(point(2, 0.010, 0.0, (0, 1), &[(0x8072, 1)]));
        d.extend(point(6, 0.005, 0.003, (0, 3), &[]));
        d.extend(point(5, 0.007, 0.003, (2, 2), &[(0x8074, 0)]));
        d.extend(seg(1, 0, 1, 1));
        d.extend(seg(1, 3, 3, 1));
        // the circle's diameter: the value, then the handle of the circle
        d.extend(param(0x8004, "D1", 0.004));
        d.extend([0u8; 30]);
        d.extend(handle(0x8074, 0));
        // the length of the line: two point handles
        d.extend(param(0x8004, "D2", 0.010));
        d.extend([0u8; 30]);
        d.extend(handle(0x8070, 0));
        d.extend(handle(0x8070, 1));
        // a value that no measure of the sketch equals: not kept
        d.extend(param(0x8004, "D3", 0.0123));
        d.extend([0u8; 30]);
        d.extend(handle(0x8074, 0));
        d.extend(decl("moExtrusion_c"));
        d.extend(feature(0x80c2, "Boss-Extrude1", 32));
        d
    }

    #[test]
    fn a_dimension_keeps_its_operands_when_the_value_is_their_measure() {
        let set = parse(&dimensioned(), &Limits::default()).unwrap();
        let s = set.sketch(26).expect("sketch 26");
        assert_eq!(s.params.len(), 3);
        let names: Vec<_> = s.dims.iter().map(|d| (d.name.as_str(), d.kind)).collect();
        assert!(names.contains(&("D1", DimKind::Diameter)), "{names:?} {:?}", s.dims);
        assert!(!s.dims.iter().any(|d| d.name == "D3"), "{names:?}");
        let d1 = s.dims.iter().find(|d| d.name == "D1").unwrap();
        assert_eq!(d1.targets, vec![RelTarget::Arc(0)]);
    }

    #[test]
    fn an_ellipse_record_is_read_when_its_points_lie_on_it() {
        // centre (0, 0); axis ends (+-6, 0) and (0, +-3); the rim point (3, 3*sqrt(3)/2) lies on x^2/36 + y^2/9 = 1
        let rim = (0.003, 0.003 * 3f64.sqrt() / 2.0);
        let mut d = le32(109).to_vec();
        d.extend([0x13, 0]);
        d.extend(decl("moProfileFeature_c"));
        d.extend(feature(0x8004, "Sketch1", 26));
        d.extend(decl("sgPointHandle"));
        d.extend(point(1, rim.0, rim.1, (2, 2), &[]));
        d.extend(point(2, 0.0, 0.0, (0, 1), &[]));
        d.extend(point(3, 0.006, 0.0, (0, 1), &[]));
        d.extend(point(4, 0.0, 0.003, (0, 1), &[]));
        d.extend(point(5, -0.006, 0.0, (0, 1), &[]));
        d.extend(point(6, 0.0, -0.003, (0, 1), &[]));
        d.extend(seg_full(1, 0, 0, 1, 1, [1, 2, 3, 4, 5, 0]));
        let set = parse(&d, &Limits::default()).unwrap();
        let s = &set.sketches[0];
        match &s.curves[0] {
            SkCurve::Ellipse { centre, major, minor, .. } => {
                assert_eq!((*centre, *major), (1, 2));
                assert!((minor - 0.003).abs() < 1e-12);
            }
            c => panic!("{c:?}"),
        }
        assert!(s.exact());
        // a rim point off the ellipse: refused, the sketch is not exact
        let mut e = d.clone();
        let at = e.windows(8).position(|w| w == f(rim.1)).unwrap();
        e[at..at + 8].copy_from_slice(&f(rim.1 + 0.001));
        let s = &parse(&e, &Limits::default()).unwrap().sketches[0];
        assert!(!s.exact());
    }
}
