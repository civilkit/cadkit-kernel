//! 3D sketches (`sketch3d` features): points with `x`/`y`/`z`, entities
//! `line`/`arc`/`spline`/`cvspline`/`point`/`ref` and the 3D constraint
//! vocabulary (see `solver.rs`).
//!
//! Wire shape (the client `sr()` constructor + the samples): the same shape
//! as a 2D sketch without `plane`. The regen result echoes the sketch back
//! into `sketches[i][1].sketch` (oracle probes):
//! * missing `points`/`entities`/`constraints` arrays are appended;
//! * a point keeps its key order; `x`/`y`/`z` are coerced in place and
//!   appended (0) when missing; `fixed` is coerced to bool only if present;
//!   a non-object point becomes `{"id":"","x":0,"y":0,"z":0}`;
//! * entities of an unknown type are DROPPED — and so are `circle`/`ellipse`
//!   (not part of the 3D vocabulary); `line`/`arc` ensure their string
//!   fields, `spline`/`cvspline` ensure `pts` (`cvspline` also `degree`,
//!   default 1), `point` ensures `p`, `ref` ensures `pts` (any `curve`
//!   object is echoed verbatim);
//! * constraints are echoed verbatim except a numeric-string scalar `value`
//!   (`"7"` -> 7; array values like the `fix` point are NOT coerced) and an
//!   `expr`-driven value is rewritten.

pub mod solver;

use ck_json::Json;

use crate::sketch::{jnum, jnum_opt, jstr, jstr_list, jtruthy, obj_set};

/// A 3D point in a sketch: `{"id":"p1","x":0,"y":0,"z":0,"fixed":true}`.
#[derive(Debug, Clone, Default)]
pub struct Point3 {
    pub id: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Omitted unless true.
    pub fixed: bool,
}

/// Sketch3 curve entities (client `ue()` vocabulary for 3D sketches).
#[derive(Debug, Clone)]
pub enum Entity3 {
    Line { id: String, p1: String, p2: String },
    Arc { id: String, c: String, p1: String, p2: String },
    Spline { id: String, pts: Vec<String>, closed: bool },
    CvSpline { id: String, pts: Vec<String>, degree: u32, knots: Option<Vec<f64>>, closed: bool },
    Point { id: String, p: String },
    /// Reference to another sketch's curve, carrying its own (rational)
    /// B-spline definition `{P, p, knots, w?}` (client `as()`).
    Ref { id: String, pts: Vec<String>, closed: bool, curve: Option<Json> },
}

impl Entity3 {
    pub fn id(&self) -> &str {
        match self {
            Entity3::Line { id, .. }
            | Entity3::Arc { id, .. }
            | Entity3::Spline { id, .. }
            | Entity3::CvSpline { id, .. }
            | Entity3::Point { id, .. }
            | Entity3::Ref { id, .. } => id,
        }
    }

    pub fn type_name(&self) -> &'static str {
        match self {
            Entity3::Line { .. } => "line",
            Entity3::Arc { .. } => "arc",
            Entity3::Spline { .. } => "spline",
            Entity3::CvSpline { .. } => "cvspline",
            Entity3::Point { .. } => "point",
            Entity3::Ref { .. } => "ref",
        }
    }

    /// Point ids the entity is built on (client `ue()`).
    pub fn point_ids(&self) -> Vec<&str> {
        match self {
            Entity3::Line { p1, p2, .. } => vec![p1, p2],
            Entity3::Arc { c, p1, p2, .. } => vec![c, p1, p2],
            Entity3::Point { p, .. } => vec![p],
            Entity3::Spline { pts, .. } | Entity3::CvSpline { pts, .. } | Entity3::Ref { pts, .. } => {
                pts.iter().map(|s| s.as_str()).collect()
            }
        }
    }

    pub fn from_json(j: &Json) -> Option<Entity3> {
        let id = jstr(j, "id");
        let s = |k: &str| jstr(j, k);
        Some(match j.get("type")?.as_str()? {
            "line" => Entity3::Line { id, p1: s("p1"), p2: s("p2") },
            "arc" => Entity3::Arc { id, c: s("c"), p1: s("p1"), p2: s("p2") },
            "spline" => Entity3::Spline {
                id,
                pts: jstr_list(j.get("pts")),
                closed: jtruthy(j.get("closed")),
            },
            "cvspline" => Entity3::CvSpline {
                id,
                pts: jstr_list(j.get("pts")),
                degree: jnum_opt(j.get("degree")).map(|d| d.max(1.0) as u32).unwrap_or(1),
                knots: j.get("knots").and_then(|k| k.as_arr()).map(|a| a.iter().map(|x| jnum(Some(x))).collect()),
                closed: jtruthy(j.get("closed")),
            },
            "point" => Entity3::Point { id, p: s("p") },
            "ref" => Entity3::Ref {
                id,
                pts: jstr_list(j.get("pts")),
                closed: jtruthy(j.get("closed")),
                curve: j.get("curve").cloned(),
            },
            _ => return None,
        })
    }

    /// Normalised echo of an entity DOM (see module doc).
    fn normalize_json(&self, raw: &Json) -> Json {
        let mut o = if let Json::Obj(_) = raw { raw.clone() } else { Json::Obj(vec![]) };
        let ensure_str = |o: &mut Json, k: &str| {
            if o.get(k).and_then(|v| v.as_str()).is_none() {
                obj_set(o, k, Json::Str(String::new()));
            }
        };
        ensure_str(&mut o, "id");
        match self {
            Entity3::Line { .. } => {
                ensure_str(&mut o, "p1");
                ensure_str(&mut o, "p2");
            }
            Entity3::Arc { .. } => {
                ensure_str(&mut o, "c");
                ensure_str(&mut o, "p1");
                ensure_str(&mut o, "p2");
            }
            Entity3::Spline { pts, .. } | Entity3::Ref { pts, .. } => {
                obj_set(&mut o, "pts", Json::Arr(pts.iter().map(|p| Json::Str(p.clone())).collect()));
            }
            Entity3::CvSpline { pts, degree, .. } => {
                obj_set(&mut o, "pts", Json::Arr(pts.iter().map(|p| Json::Str(p.clone())).collect()));
                obj_set(&mut o, "degree", Json::Num(*degree as f64));
            }
            Entity3::Point { .. } => ensure_str(&mut o, "p"),
        }
        o
    }
}

/// Constraint on 3D sketch geometry. `value` is a scalar (distance / radius
/// / angle / onCurve parameter / tension) or the `fix` point `[x, y, z]`;
/// `plane` is the raw `onPlane` plane (a std plane id or `{o, n}`).
#[derive(Debug, Clone, Default)]
pub struct Constraint3 {
    pub id: String,
    pub kind: String,
    pub refs: Vec<String>,
    pub value: Option<f64>,
    pub point: Option<[f64; 3]>,
    pub at: Option<String>,
    /// The raw `plane` field (`onPlane`).
    pub plane: Option<Json>,
    /// Angle direction signs (default 1).
    pub s1: f64,
    pub s2: f64,
    pub expr: Option<String>,
    /// False when the constraint failed typed parsing.
    pub valid: bool,
}

/// A 3D sketch: points + entities + constraints (no plane).
#[derive(Debug, Clone)]
pub struct Sketch3 {
    pub id: String,
    pub name: String,
    pub points: Vec<Point3>,
    pub entities: Vec<Entity3>,
    pub constraints: Vec<Constraint3>,
    pub seq: u32,
    pub visible: bool,
    /// The feature object as received (echoed back by regen).
    pub raw: Json,
}

fn jstr_list3(j: Option<&Json>) -> Vec<String> {
    j.and_then(|v| v.as_arr())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(|s| s.to_string())).collect())
        .unwrap_or_default()
}

impl Point3 {
    pub fn from_json(j: &Json) -> Point3 {
        Point3 {
            id: jstr(j, "id"),
            x: jnum(j.get("x")),
            y: jnum(j.get("y")),
            z: jnum(j.get("z")),
            fixed: jtruthy(j.get("fixed")),
        }
    }
}

impl Constraint3 {
    pub fn from_json(j: &Json) -> Constraint3 {
        let refs_ok = j.get("refs").and_then(|r| r.as_arr()).map_or(false, |a| a.iter().all(|x| x.as_str().is_some()));
        let kind = jstr(j, "type");
        let (value, point) = match j.get("value") {
            Some(Json::Arr(a)) if a.len() >= 3 => {
                (None, Some([jnum(a.get(0)), jnum(a.get(1)), jnum(a.get(2))]))
            }
            v => (jnum_opt(v), None),
        };
        Constraint3 {
            id: jstr(j, "id"),
            valid: refs_ok && !kind.is_empty() && j.get("id").and_then(|v| v.as_str()).is_some(),
            kind,
            refs: jstr_list3(j.get("refs")),
            value,
            point,
            at: j.get("at").and_then(|v| v.as_str()).map(|s| s.to_string()),
            plane: j.get("plane").cloned(),
            s1: jnum_opt(j.get("s1")).unwrap_or(1.0),
            s2: jnum_opt(j.get("s2")).unwrap_or(1.0),
            expr: match j.get("expr") {
                Some(Json::Str(s)) => Some(s.clone()),
                Some(Json::Num(n)) => Some(ck_json::fmt_f64(*n)),
                _ => None,
            },
        }
    }
}

impl Sketch3 {
    /// Parse a sketch3d feature object. Never fails (lenient like the kernel).
    pub fn from_json(j: &Json) -> Sketch3 {
        let arr = |k: &str| j.get(k).and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
        Sketch3 {
            id: jstr(j, "id"),
            name: jstr(j, "name"),
            points: arr("points").iter().map(Point3::from_json).collect(),
            entities: arr("entities").iter().filter_map(Entity3::from_json).collect(),
            constraints: arr("constraints").iter().map(Constraint3::from_json).collect(),
            seq: jnum_opt(j.get("seq")).unwrap_or(0.0).max(0.0) as u32,
            visible: j.get("visible").map_or(true, |v| jtruthy(Some(v))),
            raw: j.clone(),
        }
    }

    pub fn point(&self, id: &str) -> Option<&Point3> {
        // last duplicate wins, like `new Map(points)`
        self.points.iter().rev().find(|p| p.id == id)
    }

    pub fn entity(&self, id: &str) -> Option<&Entity3> {
        self.entities.iter().rev().find(|e| e.id() == id)
    }

    /// Point position by id, also accepting point entities and arc ids (the
    /// centre) — client `Tl()`/`ir()`.
    pub fn point_xyz(&self, id: &str) -> Option<[f64; 3]> {
        if let Some(p) = self.point(id) {
            return Some([p.x, p.y, p.z]);
        }
        match self.entity(id)? {
            Entity3::Point { p, .. } => self.point(p).map(|p| [p.x, p.y, p.z]),
            Entity3::Arc { c, .. } => self.point(c).map(|p| [p.x, p.y, p.z]),
            _ => None,
        }
    }

    /// Serialise for the regen echo: the original DOM with solved/normalised
    /// points, entities and constraint values (see module doc).
    pub fn to_json(&self) -> Json {
        let mut o = if let Json::Obj(_) = self.raw { self.raw.clone() } else { Json::Obj(vec![]) };
        // points: normalise each raw point, then write the (solved) coordinates.
        let raw_pts: Vec<Json> = self.raw.get("points").and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
        let mut pts = Vec::with_capacity(self.points.len());
        for (i, p) in self.points.iter().enumerate() {
            let mut q = match raw_pts.get(i) {
                Some(r @ Json::Obj(_)) => r.clone(),
                _ => Json::Obj(vec![]),
            };
            obj_set(&mut q, "id", Json::Str(p.id.clone()));
            obj_set(&mut q, "x", Json::Num(p.x));
            obj_set(&mut q, "y", Json::Num(p.y));
            obj_set(&mut q, "z", Json::Num(p.z));
            if q.get("fixed").is_some() {
                obj_set(&mut q, "fixed", Json::Bool(p.fixed));
            }
            pts.push(q);
        }
        obj_set(&mut o, "points", Json::Arr(pts));
        // entities: raw entities of a known 3D type, in order.
        let raw_ents: Vec<Json> = self.raw.get("entities").and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
        let mut ents = Vec::new();
        let mut it = self.entities.iter();
        for r in &raw_ents {
            if Entity3::from_json(r).is_some() {
                if let Some(e) = it.next() {
                    ents.push(e.normalize_json(r));
                }
            }
        }
        obj_set(&mut o, "entities", Json::Arr(ents));
        // constraints: verbatim except the scalar value coercion / expr values.
        let raw_cons: Vec<Json> = self.raw.get("constraints").and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
        let mut cons = Vec::new();
        for (i, r) in raw_cons.iter().enumerate() {
            let mut q = r.clone();
            if let (Some(c), Json::Obj(_)) = (self.constraints.get(i), &q) {
                if c.valid {
                    match q.get("value") {
                        Some(Json::Str(s)) => {
                            if let Ok(v) = s.trim().parse::<f64>() {
                                obj_set(&mut q, "value", Json::Num(v));
                            }
                        }
                        Some(_) | None => {
                            if let Some(v) = c.value {
                                if q.get("value").map_or(true, |v| !matches!(v, Json::Arr(_))) {
                                    obj_set(&mut q, "value", Json::Num(v));
                                }
                            }
                        }
                    }
                }
            }
            cons.push(q);
        }
        obj_set(&mut o, "constraints", Json::Arr(cons));
        o
    }
}
