// SPDX-License-Identifier: MIT
//! Assembly relationships (mitcad#155): the constraints a relationship
//! item holds between geometry of two occurrences, and tangent
//! relationships, a contact between two bodies *(read from the corpus'
//! relationship items: the occurrence placements the file stores after
//! them satisfy the decoded frames, senses and offsets; SCHEMA.md §5.3)*.
//!
//! A relationship item ([`GEOMETRIC_RELATIONSHIP`]) refers to its rows,
//! one per constraint, in order: objects of [`RELATION_ROW`] or a class
//! derived from it ([`ALIGN_ROW`], [`CENTER_ROW`], [`ANGLE_ROW`]). A row
//! refers to the two sides' placements ([`PLACEMENT`]: the first two it
//! names; side one first), the parameter holder of its offset or angle
//! (`Offset`, `Angle`), its faces and edges (two [`BODY_INPUT`] lists, `u32
//! n | n refs` each, side one's first: one input per side, two for a
//! centring), the key points picked on them ([`KEY_POINT`]) and its
//! constraint ([`GEOM_CONSTRAINT`]).
//!
//! The constraint holds the geometry. Class version 7 refers to a data
//! record (`ref data | ref relationship | str16`), whose class says what it
//! is: [`ALIGN_DATA`] (an alignment of points, axes, planes or circles),
//! [`CENTER_DATA`] (side one's two faces centred on side two's) or
//! [`ANGLE_DATA`]. After the record's root part:
//!
//! ```text
//! ALIGN_DATA:  u32 type | value part | data part
//! CENTER_DATA: u32 type | data part
//! ANGLE_DATA:  41 bytes (version 2; not read) | value part | data part
//! value part:  3 flags (VALUE_DATA version 0) or 4 (version 1)
//!              | f64 maximum | f64 minimum | f64 value
//! data part:   u32 n, n × f64[3] points | u32 sense | u32 n, n × u32 geometry
//!              | u32 n, n × frame | str16 name | occurrence paths
//!              | ref constraint | u8 | u32 n, n × str16 | u32 n, n × u8
//! frame:       u8 0 + f64[16] (row-major, cm), or u8 1: the identity
//! ```
//!
//! Versions 3 to 6 hold the data part themselves: after the root part 2
//! bytes (version 3) or 3, `str16 name`, a `u8` (versions 3 and 5) or a
//! `u32` (6), the points, a `u32`, the geometry and the frames, then `f64
//! maximum | f64 minimum`, the occurrence paths and references, and at
//! the end `u32 n, n × str16 | u32 sense | u32 type | u32 n, n × u8`.
//!
//! - **Frames.** One per face or edge, side one's first, in the component
//!   the side's occurrence path ends in: a plane's origin on it and its z
//!   axis its normal, an axis's origin on it and z along it, a
//!   point's origin, a circle's centre and z its normal. The geometry
//!   codes: 0 a point, 1 an axis (of a cylindrical face), 2
//!   a plane, 3 a circle (a circular edge) *(the selection names agree:
//!   `Point`, `Face`, `Edge`)*.
//! - **Sense.** 1: the z axes the same way, 2: opposed, 0: either.
//! - **Offset.** The row's parameter (the value part's value is the same
//!   number): for two planes side two's plane that far from side one's
//!   along side one's normal; for a point or axis and a plane, side one's
//!   point (origin) that far from side two's plane along its normal.
//! - **Type.** The data record's code names the geometry: 2 a point and a
//!   plane, 3 two axes, 4 an axis and a plane, 5 two planes, 6 two
//!   circles; versions 3 to 6 count one more (3 a point and a plane, 4
//!   axes, 6 planes, 7 circles), 9 a centring and 0 an angle.
//! - **Limits.** The maximum and minimum are stored whether used or not;
//!   no flag is set in the corpus but one (the third flag of three
//!   alignments, whose values are all 0), so they are left out.
//!
//! A tangent relationship item ([`TANGENT_RELATIONSHIP`]): after its root
//! part `ref key point × 2 | ref body input × 2 | u32 n, n × ref placement
//! | ref relationship ([`TANGENT`]) | ...`: each side a point on a face of
//! its body (in the side's component) and the body (a [`BODY_REF`])
//! *(the corpus' three: a plane and a cylinder, which touch where the file
//! places the occurrences right after the item in one design; the points
//! themselves do not meet)*.

use serde_json::{Map, Value, json};

use super::super::classes::*;
use super::super::decode::TimelineEntry;
use super::super::ir::*;
use super::super::stream::{Segment, f64_at, str16_at, u32_at};
use super::{Builder, edge_input, face_input, inputs_of, json_value, paramref};

/// A byte cursor over an object's data.
struct Cur<'a> {
    d: &'a [u8],
    p: usize,
}

impl Cur<'_> {
    fn skip(&mut self, n: usize) -> Option<()> {
        (self.p + n <= self.d.len()).then(|| self.p += n)
    }

    fn u8(&mut self) -> Option<u8> {
        let v = *self.d.get(self.p)?;
        self.p += 1;
        Some(v)
    }

    fn u32(&mut self) -> Option<u32> {
        let v = u32_at(self.d, self.p)?;
        self.p += 4;
        Some(v)
    }

    fn f64(&mut self) -> Option<f64> {
        let v = f64_at(self.d, self.p)?;
        self.p += 8;
        v.is_finite().then_some(v)
    }

    fn str16(&mut self) -> Option<String> {
        let (s, e) = str16_at(self.d, self.p)?;
        self.p = e;
        Some(s)
    }

    /// `u32 n`, then `n` items (at most [`MAX_ITEMS`]).
    fn vec<T>(&mut self, mut f: impl FnMut(&mut Self) -> Option<T>) -> Option<Vec<T>> {
        let n = self.u32()?;
        if n > MAX_ITEMS {
            return None;
        }
        (0..n).map(|_| f(self)).collect()
    }

    fn point(&mut self) -> Option<Vec3> {
        Some([self.f64()?, self.f64()?, self.f64()?])
    }

    /// `u8 0` + a rigid 4x4 matrix, or `u8 1` for none.
    fn frame(&mut self) -> Option<Option<Mat4>> {
        match self.u8()? {
            0 => {
                let m = super::rigid_matrix(self.d.get(self.p..self.p + 128)?)?;
                self.p += 128;
                Some(Some(m))
            }
            1 => Some(None),
            _ => None,
        }
    }
}

/// The most points, frames or names of one constraint.
const MAX_ITEMS: u32 = 16;

/// What a constraint holds (its data record, or its own data in versions
/// 3 to 6).
#[derive(Debug, Default, PartialEq)]
struct Data {
    /// `align`, `center` or `angle`.
    kind: Option<&'static str>,
    type_code: Option<u32>,
    sense: Option<u32>,
    points: Vec<Vec3>,
    geometry: Vec<u32>,
    frames: Vec<Option<Mat4>>,
    selections: Vec<String>,
    /// The value part's flags and maximum, minimum (and value).
    flags: Vec<u8>,
    values: Vec<f64>,
}

/// The data part's points, sense, geometry and frames; the three lists
/// have one entry per face or edge.
fn data_part(c: &mut Cur, out: &mut Data) -> Option<()> {
    out.points = c.vec(Cur::point)?;
    out.sense = Some(c.u32()?);
    out.geometry = c.vec(Cur::u32)?;
    out.frames = c.vec(Cur::frame)?;
    let n = out.frames.len();
    (n >= 2 && out.points.len() == n && out.geometry.len() == n).then_some(())
}

/// The value part: flags, then maximum, minimum and value.
fn value_part(seg: &Segment, c: &mut Cur, out: &mut Data) -> Option<()> {
    let flags = match seg.class_by_guid(VALUE_DATA)?.version {
        0 => 3,
        1 => 4,
        _ => return None,
    };
    for _ in 0..flags {
        out.flags.push(c.u8()?);
    }
    for _ in 0..3 {
        out.values.push(c.f64()?);
    }
    Some(())
}

/// A data record ([`CONSTRAINT_DATA`]'s classes) of constraint `gc`.
fn data_record(seg: &Segment, id: u64, gc: u64) -> Option<Data> {
    let o = seg.object(id)?;
    let class = seg.guid(o)?;
    let d = seg.data(o);
    let mut c = Cur {
        d,
        p: seg.root_part(d)?.end,
    };
    let mut out = Data::default();
    match class {
        ALIGN_DATA => {
            out.kind = Some("align");
            out.type_code = Some(c.u32()?);
            value_part(seg, &mut c, &mut out)?;
        }
        CENTER_DATA => {
            out.kind = Some("center");
            out.type_code = Some(c.u32()?);
        }
        ANGLE_DATA if seg.version(o) == Some(2) => {
            out.kind = Some("angle");
            c.skip(41)?;
            value_part(seg, &mut c, &mut out)?;
        }
        _ => return None,
    }
    data_part(&mut c, &mut out)?;
    out.sense.filter(|&s| s <= 2)?;
    c.str16()?;
    // The occurrence paths (not read: the rows' placements name the
    // occurrences), up to the reference back to the constraint.
    let (_, r) = seg
        .refs_in(d, c.p, d.len())
        .into_iter()
        .find(|(_, r)| r.id == gc)?;
    c.p = r.end;
    c.u8()?;
    out.selections = c.vec(Cur::str16)?;
    let used = c.vec(Cur::u8)?;
    (used.len() == out.frames.len() && c.p == d.len()).then_some(out)
}

/// The data a constraint of class version 3 to 6 holds itself.
fn own_data(seg: &Segment, gc: u64) -> Option<Data> {
    let o = seg.object(gc)?;
    let version = seg.version(o)?;
    let d = seg.data(o);
    let mut c = Cur {
        d,
        p: seg.root_part(d)?.end,
    };
    c.skip(match version {
        3 => 2,
        5 | 6 => 3,
        _ => return None,
    })?;
    c.str16()?;
    c.skip(if version == 6 { 4 } else { 1 })?;
    let mut out = Data::default();
    data_part(&mut c, &mut out)?;
    out.values = vec![c.f64()?, c.f64()?];
    // The end: names, sense, type and one flag per frame.
    let n = out.frames.len();
    let end = d.len().checked_sub(n + 4)?;
    if u32_at(d, end)? as usize != n {
        return None;
    }
    out.type_code = Some(u32_at(d, end - 4)?);
    out.sense = Some(u32_at(d, end - 8)?).filter(|&s| s <= 2);
    out.sense?;
    // The names' list ends where the sense starts.
    out.selections = (c.p..end - 8).rev().find_map(|q| {
        let mut names = Cur { d, p: q };
        let list = names.vec(Cur::str16)?;
        (names.p == end - 8 && list.len() == n).then_some(list)
    })?;
    out.kind = match (out.type_code, n) {
        (Some(0), 2) => Some("angle"),
        (Some(3..=7), 2) => Some("align"),
        (Some(9), 4) => Some("center"),
        _ => None,
    };
    Some(out)
}

/// The data of a constraint ([`GEOM_CONSTRAINT`]).
fn constraint_data(seg: &Segment, gc: u64) -> Option<Data> {
    let o = seg.object(gc)?;
    match seg.version(o)? {
        7 => {
            let id = seg
                .ref_ids(seg.data(o))
                .into_iter()
                .find(|&r| seg.is_kind_of(r, CONSTRAINT_DATA))?;
            data_record(seg, id, gc)
        }
        _ => own_data(seg, gc),
    }
}

/// The geometry code's name.
fn geometry_name(code: u32) -> &'static str {
    match code {
        0 => "point",
        1 => "axis",
        2 => "plane",
        3 => "circle",
        _ => "other",
    }
}

/// The body inputs of each side: the two lists `u32 n | n refs` that
/// start before the first body input the row names.
fn side_inputs(seg: &Segment, row: u64) -> Option<[Vec<u64>; 2]> {
    let d = seg.data_of(row);
    let (first, _) = seg
        .refs_in(d, 0, d.len())
        .into_iter()
        .find(|(_, r)| seg.guid_of(r.id) == Some(BODY_INPUT))?;
    let mut p = first.checked_sub(4)?;
    let mut list = || -> Option<Vec<u64>> {
        let n = u32_at(d, p)?;
        if n == 0 || n > MAX_ITEMS {
            return None;
        }
        p += 4;
        let mut out = Vec::new();
        for _ in 0..n {
            let r = seg.ref_at(d, p)?;
            out.push(r.id);
            p = r.end;
        }
        Some(out)
    };
    Some([list()?, list()?])
}

impl Builder<'_> {
    /// The occurrence a side's placement names: its occurrence path, else
    /// (an empty path) the occurrence whose placements the placement
    /// refers to *(some designs name their occurrences only so)*.
    fn side_occurrence(&self, placement: u64) -> Option<Value> {
        let seg = self.seg;
        let mut v = self.placement_occurrence(placement)?;
        let empty = v
            .pointer("/_f3d/path")
            .and_then(Value::as_array)
            .is_none_or(|p| p.is_empty());
        if empty
            && let Some(h) = inputs_of(seg, placement, OCCURRENCE_PLACEMENTS)
                .first()
                .and_then(|&h| super::placements::history(seg, h))
        {
            let path: Vec<u64> = if h.path.is_empty() {
                h.steps
                    .first()
                    .and_then(|(item, _)| super::item_occurrence(seg, *item))
                    .into_iter()
                    .collect()
            } else {
                h.path
            };
            if let Some(name) = path
                .last()
                .and_then(|&o| super::occurrence_info(seg, o))
                .filter(|i| i.context.is_none())
                .and_then(|i| self.component_names.get(&i.component))
            {
                v["component"] = json!(name);
            }
            v["_f3d"]["path"] = json!(path);
        }
        Some(v)
    }

    /// A face or edge input ([`BODY_INPUT`]) as a reference.
    fn row_entity(&self, input: u64) -> Option<Value> {
        let seg = self.seg;
        let f = *inputs_of(seg, input, FACE_REF).first()?;
        edge_input(seg, f)
            .or_else(|| face_input(seg, f))
            .map(|r| json_value(&r))
    }

    /// A relationship item's detail (SCHEMA.md §5.3,
    /// `GeometricRelationship`): its constraints as rows.
    pub(super) fn geometric_relationship(&self, it: &TimelineEntry) -> Detail {
        let seg = self.seg;
        let mut rows = Vec::new();
        let mut seen = Vec::new();
        for r in seg.ref_ids(seg.data_of(it.id)) {
            if !seen.contains(&r) && seg.is_kind_of(r, RELATION_ROW) {
                seen.push(r);
                rows.push(self.relation_row(r));
            }
        }
        let mut map = Map::new();
        map.insert("rows".into(), Value::Array(rows));
        Detail::Other(map)
    }

    /// One constraint of a relationship item.
    fn relation_row(&self, row: u64) -> Value {
        let seg = self.seg;
        let mut v = Map::new();
        let gc = inputs_of(seg, row, GEOM_CONSTRAINT).first().copied();
        let data = gc.and_then(|gc| constraint_data(seg, gc));
        if let Some(kind) = data.as_ref().and_then(|d| d.kind) {
            v.insert("type".into(), json!(kind));
        }
        let placements = inputs_of(seg, row, PLACEMENT);
        for (k, key) in ["occurrenceOne", "occurrenceTwo"].into_iter().enumerate() {
            if let Some(o) = placements.get(k).and_then(|&p| self.side_occurrence(p)) {
                v.insert(key.into(), o);
            }
        }
        let sides = side_inputs(seg, row);
        if let Some(sides) = &sides {
            for (key, side) in [("entitiesOne", &sides[0]), ("entitiesTwo", &sides[1])] {
                let refs: Vec<Value> = side
                    .iter()
                    .map(|&i| self.row_entity(i).unwrap_or(Value::Null))
                    .collect();
                v.insert(key.into(), json!(refs));
            }
        }
        if let Some(h) = inputs_of(seg, row, PARAMETER_HOLDER).first() {
            for p in self.dec.parameters.iter().filter(|p| p.holder == Some(*h)) {
                match p.role.as_str() {
                    "Offset" => v.insert("offset".into(), json_value(&paramref(p))),
                    "Angle" => v.insert("angle".into(), json_value(&paramref(p))),
                    _ => None,
                };
            }
        }
        let mut f3d = json!({"record": row, "constraint": gc});
        if let Some(d) = &data {
            v.insert(
                "geometry".into(),
                json!(
                    d.geometry
                        .iter()
                        .map(|&g| geometry_name(g))
                        .collect::<Vec<_>>()
                ),
            );
            if let Some(s) = d.sense {
                let sense = match s {
                    1 => "aligned",
                    2 => "opposed",
                    _ => "either",
                };
                v.insert("sense".into(), json!(sense));
            }
            f3d["type_code"] = json!(d.type_code);
            f3d["frames"] = json!(
                d.frames
                    .iter()
                    .map(|m| json_value(&m.unwrap_or(IDENTITY)))
                    .collect::<Vec<_>>()
            );
            f3d["points"] = json!(d.points);
            f3d["selections"] = json!(d.selections);
            f3d["values"] = json!(d.values);
            f3d["flags"] = json!(d.flags);
        }
        if let Some(sides) = &sides {
            f3d["inputs"] = json!(sides);
        }
        let key_points: Vec<Value> = inputs_of(seg, row, KEY_POINT)
            .into_iter()
            .filter_map(|k| super::joints::key_point(seg, k))
            .map(|(p, code)| json!({"point": p, "code": code}))
            .collect();
        f3d["key_points"] = json!(key_points);
        v.insert("_f3d".into(), f3d);
        Value::Object(v)
    }

    /// A tangent relationship item's detail (SCHEMA.md §5.3,
    /// `TangentRelationship`): each side's occurrence, body and point.
    pub(super) fn tangent_relationship(&self, it: &TimelineEntry) -> Detail {
        let seg = self.seg;
        let mut map = Map::new();
        let placements = inputs_of(seg, it.id, PLACEMENT);
        let key_points = inputs_of(seg, it.id, KEY_POINT);
        let bodies = inputs_of(seg, it.id, BODY_INPUT);
        let sides = [
            ("occurrenceOne", "bodyOne", "pointOne"),
            ("occurrenceTwo", "bodyTwo", "pointTwo"),
        ];
        for (k, (occurrence, body, point)) in sides.into_iter().enumerate() {
            if let Some(o) = placements.get(k).and_then(|&p| self.side_occurrence(p)) {
                map.insert(occurrence.into(), o);
            }
            if let Some(b) = bodies
                .get(k)
                .and_then(|&b| inputs_of(seg, b, BODY_REF).first().copied())
                .and_then(|b| self.body_input(b))
            {
                map.insert(body.into(), json_value(&b));
            }
            if let Some((p, _)) = key_points
                .get(k)
                .and_then(|&kp| super::joints::key_point(seg, kp))
            {
                map.insert(point.into(), json!(p));
            }
        }
        if let Some(face) = key_points.first().and_then(|&kp| {
            inputs_of(seg, kp, FACE_REF)
                .first()
                .and_then(|&f| face_input(seg, f))
        }) {
            map.insert("faceOne".into(), json_value(&face));
        }
        if let Some(face) = key_points.get(1).and_then(|&kp| {
            inputs_of(seg, kp, FACE_REF)
                .first()
                .and_then(|&f| face_input(seg, f))
        }) {
            map.insert("faceTwo".into(), json_value(&face));
        }
        Detail::Other(map)
    }
}
