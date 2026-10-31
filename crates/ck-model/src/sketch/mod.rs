//! Sketch documents: plane, points, entities, constraints, solver submodules.
//!
//! Wire shape (all 4 sample .ckpart files + client `Wi()` constructor):
//! `{"type":"sketch","id":"F1","name":..,"plane":{"kind":"std","id":"front"},
//!   "points":[{"id":"O","x":0,"y":0,"fixed":true},..],"entities":[..],
//!   "constraints":[..],"seq":22,"visible":false}`.
//!
//! The regen result echoes the sketch back (`sketches[i][1].sketch`) as the
//! feature DOM with points/entities normalised (oracle probes):
//! * missing `points`/`entities`/`constraints` arrays are appended in that
//!   order; a non-array becomes `[]` in place;
//! * a point keeps its key order and unknown keys; `x`/`y` are coerced to
//!   numbers in place (`"3"` -> 3, `null` -> 0), missing `id`/`x`/`y` are
//!   appended (`""`, 0, 0); `fixed` is coerced to bool only if present;
//!   a non-object point becomes `{"id":"","x":0,"y":0}`;
//! * entities without a known `type` (incl. `ref`) are dropped; missing
//!   string fields default to `""`, `r` to 0, cvspline `degree` to 1;
//! * constraints are echoed verbatim except a numeric-string `value` is
//!   coerced (`"7"` -> 7, `[1,"2"]` -> `[1,2]`; `"=2+3"` stays a string) and
//!   an `expr`-driven value is rewritten.

pub mod analyze;
pub mod guard;
pub mod offset;
pub mod profile;
pub mod smooth;
pub mod solver;
pub mod status;
pub mod spline;
pub mod text;

use crate::{GeomRef, Plane};
use ck_json::Json;

/// Id of the fixed origin point every client-created sketch carries
/// (`Ze = "O"` in app.pretty.js; referenced as `"O"` by constraints).
pub const ORIGIN: &str = "O";

/// A 2D sketch: plane + points + entities + constraints.
#[derive(Debug, Clone)]
pub struct Sketch {
    pub id: String,
    pub name: String,
    /// `None` when the `plane` field is missing/unparseable (regen error
    /// `missingReference`).
    pub plane: Option<Plane>,
    pub points: Vec<SketchPoint>,
    pub entities: Vec<SketchEntity>,
    pub constraints: Vec<Constraint>,
    pub seq: u32,
    pub visible: bool,
    /// The feature object as received (echoed back by regen, see module doc).
    pub raw: Json,
}

/// A 2D point in a sketch. JSON: `{"id":"p1","x":0,"y":10,"fixed":true}`
#[derive(Debug, Clone, Default)]
pub struct SketchPoint {
    pub id: String,
    pub x: f64,
    pub y: f64,
    /// Omitted unless true.
    pub fixed: bool,
}

/// Sketch curve entities. JSON: `{"type":"line","id":"e7","p1":"p1","p2":"p2"}`.
/// Entity vocabulary from the client `$s()`/`ue()` helpers and the samples.
#[derive(Debug, Clone)]
pub enum SketchEntity {
    Line { id: String, p1: String, p2: String, construction: bool },
    /// `ext`/`from` circles have a driven (non-variable) radius.
    Circle { id: String, c: String, r: f64, construction: bool, driven_r: bool },
    /// Counter-clockwise from `p1` to `p2` around `c` (client `tn()`).
    Arc { id: String, c: String, p1: String, p2: String, construction: bool },
    /// `p` is a point on the major axis; `r` the minor radius.
    Ellipse { id: String, c: String, p: String, r: f64, construction: bool },
    Point { id: String, p: String },
    /// Interpolating spline through `pts`.
    Spline { id: String, pts: Vec<String>, closed: bool, construction: bool },
    /// Control-point spline.
    CvSpline { id: String, pts: Vec<String>, degree: u32, knots: Option<Vec<f64>>, closed: bool, construction: bool },
    /// Lettering (fixed geometry, see `text`).
    Text { id: String, text: Box<text::TextEntity>, construction: bool },
}

impl SketchEntity {
    pub fn id(&self) -> &str {
        match self {
            SketchEntity::Line { id, .. }
            | SketchEntity::Circle { id, .. }
            | SketchEntity::Arc { id, .. }
            | SketchEntity::Ellipse { id, .. }
            | SketchEntity::Point { id, .. }
            | SketchEntity::Spline { id, .. }
            | SketchEntity::CvSpline { id, .. }
            | SketchEntity::Text { id, .. } => id,
        }
    }

    pub fn type_name(&self) -> &'static str {
        match self {
            SketchEntity::Line { .. } => "line",
            SketchEntity::Circle { .. } => "circle",
            SketchEntity::Arc { .. } => "arc",
            SketchEntity::Ellipse { .. } => "ellipse",
            SketchEntity::Point { .. } => "point",
            SketchEntity::Spline { .. } => "spline",
            SketchEntity::CvSpline { .. } => "cvspline",
            SketchEntity::Text { .. } => "text",
        }
    }

    pub fn construction(&self) -> bool {
        match self {
            SketchEntity::Line { construction, .. }
            | SketchEntity::Circle { construction, .. }
            | SketchEntity::Arc { construction, .. }
            | SketchEntity::Ellipse { construction, .. }
            | SketchEntity::Spline { construction, .. }
            | SketchEntity::CvSpline { construction, .. }
            | SketchEntity::Text { construction, .. } => *construction,
            SketchEntity::Point { .. } => false,
        }
    }

    /// Point ids the entity is built on (client `ue()`).
    pub fn point_ids(&self) -> Vec<&str> {
        match self {
            SketchEntity::Line { p1, p2, .. } => vec![p1, p2],
            SketchEntity::Circle { c, .. } => vec![c],
            SketchEntity::Ellipse { c, p, .. } => vec![c, p],
            SketchEntity::Arc { c, p1, p2, .. } => vec![c, p1, p2],
            SketchEntity::Point { p, .. } => vec![p],
            SketchEntity::Spline { pts, .. } | SketchEntity::CvSpline { pts, .. } => {
                pts.iter().map(|s| s.as_str()).collect()
            }
            SketchEntity::Text { text, .. } => text.p.iter().chain(text.arc_c.iter()).map(|s| s.as_str()).collect(),
        }
    }
}

/// Constraint on sketch geometry. Wire: `{"type":"horizontal","id":"k1","refs":[..]}`;
/// dimensional ones add `"value":20,"offset":[dx,dy]|null` and optionally
/// `"expr":"w/2"` (a global-variable expression that drives `value`).
///
/// `kind` is the wire `type`: coincident, concentric, pointOnLine,
/// pointOnCircle, midpoint, horizontal, vertical, parallel, perpendicular,
/// collinear, equal, tangent, fix, pierce, symmetric, g1/g2/g3, onCurve,
/// tension, distance, hdist, vdist, pldist, radius, diameter, raddiff, angle.
#[derive(Debug, Clone, Default)]
pub struct Constraint {
    pub id: String,
    pub kind: String,
    pub refs: Vec<String>,
    /// Scalar `value` (dimensions; tension/onCurve parameters).
    pub value: Option<f64>,
    /// Point `value` (`fix`/`pierce`: `[x, y]`).
    pub point: Option<[f64; 2]>,
    pub expr: Option<String>,
    /// Tangent: which shared endpoint (`at`); `internal` circle tangency.
    pub at: Option<String>,
    pub internal: bool,
    /// Angle direction signs (default 1), raddiff `sign`.
    pub s1: f64,
    pub s2: f64,
    pub sign: f64,
    /// False when the constraint failed typed parsing (echoed verbatim and
    /// ignored by the solver build, which then fails as a whole).
    pub valid: bool,
    /// The agent's `suppressed` flag: the solver ignores this constraint
    /// everywhere (build, solve, status, analysis). The document still
    /// carries it verbatim and `status.suppressed` lists it.
    pub suppressed: bool,
}

/// Dimensional constraint types (client `gn` set).
pub const DIMENSIONS: [&str; 8] = ["distance", "hdist", "vdist", "pldist", "radius", "diameter", "raddiff", "angle"];

impl Constraint {
    pub fn is_dimension(&self) -> bool {
        DIMENSIONS.contains(&self.kind.as_str())
    }
}

// ---------------------------------------------------------------------------
// JSON helpers

pub(crate) fn jstr(j: &Json, k: &str) -> String {
    j.get(k).and_then(|v| v.as_str()).unwrap_or("").to_string()
}

/// Truthiness like the kernel's bool coercion (`fixed: 1` -> true).
pub(crate) fn jtruthy(j: Option<&Json>) -> bool {
    match j {
        Some(Json::Bool(b)) => *b,
        Some(Json::Num(n)) => *n != 0.0 && !n.is_nan(),
        Some(Json::Str(s)) => !s.is_empty(),
        Some(Json::Arr(_)) | Some(Json::Obj(_)) => true,
        _ => false,
    }
}

/// Numeric coercion for coordinates: number, numeric string, else 0.
pub(crate) fn jnum(j: Option<&Json>) -> f64 {
    match j {
        Some(Json::Num(n)) => *n,
        // GUESS: "=expr" coordinates are not evaluated (only `"3"` -> 3 was probed).
        Some(Json::Str(s)) => s.trim().parse::<f64>().ok().filter(|v| v.is_finite()).unwrap_or(0.0),
        _ => 0.0,
    }
}

pub(crate) fn jnum_opt(j: Option<&Json>) -> Option<f64> {
    match j {
        Some(Json::Num(n)) => Some(*n),
        Some(Json::Str(s)) => s.trim().parse::<f64>().ok(),
        _ => None,
    }
}

pub(crate) fn jstr_list(j: Option<&Json>) -> Vec<String> {
    j.and_then(|v| v.as_arr())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(|s| s.to_string())).collect())
        .unwrap_or_default()
}

pub(crate) fn obj_set(j: &mut Json, k: &str, v: Json) {
    if let Json::Obj(m) = j {
        if let Some(e) = m.iter_mut().find(|(kk, _)| kk == k) {
            e.1 = v;
        } else {
            m.push((k.to_string(), v));
        }
    }
}

// ---------------------------------------------------------------------------
// Parsing

impl SketchPoint {
    pub fn from_json(j: &Json) -> SketchPoint {
        SketchPoint { id: jstr(j, "id"), x: jnum(j.get("x")), y: jnum(j.get("y")), fixed: jtruthy(j.get("fixed")) }
    }
}

impl SketchEntity {
    /// `None` for an unknown/missing `type` (the kernel drops those).
    pub fn from_json(j: &Json) -> Option<SketchEntity> {
        let id = jstr(j, "id");
        let cons = jtruthy(j.get("construction"));
        let s = |k: &str| jstr(j, k);
        Some(match j.get("type")?.as_str()? {
            "line" => SketchEntity::Line { id, p1: s("p1"), p2: s("p2"), construction: cons },
            "circle" => SketchEntity::Circle {
                id,
                c: s("c"),
                r: jnum(j.get("r")),
                construction: cons,
                driven_r: jtruthy(j.get("ext")) || jtruthy(j.get("from")),
            },
            "arc" => SketchEntity::Arc { id, c: s("c"), p1: s("p1"), p2: s("p2"), construction: cons },
            "ellipse" => SketchEntity::Ellipse { id, c: s("c"), p: s("p"), r: jnum(j.get("r")), construction: cons },
            "point" => SketchEntity::Point { id, p: s("p") },
            "spline" => SketchEntity::Spline {
                id,
                pts: jstr_list(j.get("pts")),
                closed: jtruthy(j.get("closed")),
                construction: cons,
            },
            "cvspline" => SketchEntity::CvSpline {
                id,
                pts: jstr_list(j.get("pts")),
                // oracle: a cvspline without degree echoes `"degree":1`
                degree: jnum_opt(j.get("degree")).map(|d| d.max(1.0) as u32).unwrap_or(1),
                knots: j.get("knots").and_then(|k| k.as_arr()).map(|a| a.iter().map(|x| jnum(Some(x))).collect()),
                closed: jtruthy(j.get("closed")),
                construction: cons,
            },
            "text" => SketchEntity::Text { id, text: Box::new(text::TextEntity::from_json(j)), construction: cons },
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
            SketchEntity::Line { .. } => {
                ensure_str(&mut o, "p1");
                ensure_str(&mut o, "p2");
            }
            SketchEntity::Circle { r, .. } => {
                ensure_str(&mut o, "c");
                obj_set(&mut o, "r", Json::Num(*r));
            }
            SketchEntity::Arc { .. } => {
                ensure_str(&mut o, "c");
                ensure_str(&mut o, "p1");
                ensure_str(&mut o, "p2");
            }
            SketchEntity::Ellipse { r, .. } => {
                ensure_str(&mut o, "c");
                ensure_str(&mut o, "p");
                obj_set(&mut o, "r", Json::Num(*r));
            }
            SketchEntity::Point { .. } => ensure_str(&mut o, "p"),
            SketchEntity::Spline { pts, .. } => {
                obj_set(&mut o, "pts", Json::Arr(pts.iter().map(|p| Json::Str(p.clone())).collect()));
            }
            SketchEntity::CvSpline { pts, degree, .. } => {
                obj_set(&mut o, "pts", Json::Arr(pts.iter().map(|p| Json::Str(p.clone())).collect()));
                obj_set(&mut o, "degree", Json::Num(*degree as f64));
            }
            SketchEntity::Text { .. } => ensure_str(&mut o, "string"),
        }
        o
    }
}

impl Constraint {
    pub fn from_json(j: &Json) -> Constraint {
        let refs_ok = j.get("refs").and_then(|r| r.as_arr()).map_or(false, |a| a.iter().all(|x| x.as_str().is_some()));
        let kind = jstr(j, "type");
        let num_or = |k: &str, d: f64| jnum_opt(j.get(k)).unwrap_or(d);
        let (value, point) = match j.get("value") {
            Some(Json::Arr(a)) if a.len() >= 2 => (None, Some([jnum(a.get(0)), jnum(a.get(1))])),
            v => (jnum_opt(v), None),
        };
        Constraint {
            id: jstr(j, "id"),
            valid: refs_ok && !kind.is_empty() && j.get("id").and_then(|v| v.as_str()).is_some(),
            kind,
            refs: jstr_list(j.get("refs")),
            value,
            point,
            expr: match j.get("expr") {
                Some(Json::Str(s)) => Some(s.clone()),
                Some(Json::Num(n)) => Some(ck_json::fmt_f64(*n)),
                _ => None,
            },
            at: j.get("at").and_then(|v| v.as_str()).map(|s| s.to_string()),
            internal: jtruthy(j.get("internal")),
            s1: num_or("s1", 1.0),
            s2: num_or("s2", 1.0),
            sign: num_or("sign", 1.0),
            suppressed: jtruthy(j.get("suppressed")),
        }
    }
}

impl Sketch {
    /// Parse a sketch feature object. Never fails: the kernel is lenient
    /// (missing pieces surface later as regen errors).
    pub fn from_json(j: &Json) -> Sketch {
        let arr = |k: &str| j.get(k).and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
        Sketch {
            id: jstr(j, "id"),
            name: jstr(j, "name"),
            plane: j.get("plane").and_then(GeomRef::from_json),
            points: arr("points").iter().map(SketchPoint::from_json).collect(),
            entities: arr("entities").iter().filter_map(SketchEntity::from_json).collect(),
            constraints: arr("constraints").iter().map(Constraint::from_json).collect(),
            seq: jnum_opt(j.get("seq")).unwrap_or(0.0).max(0.0) as u32,
            visible: j.get("visible").map_or(true, |v| jtruthy(Some(v))),
            raw: j.clone(),
        }
    }

    /// Replace the echoed DOM (and the two display fields parsed from it) by `raw`, a document of the same sketch that
    /// differs from the current one only in cosmetic keys (see the regen cache's `COSMETIC_KEYS`).
    pub fn set_cosmetic_raw(&mut self, raw: Json) {
        self.name = jstr(&raw, "name");
        self.visible = raw.get("visible").map_or(true, |v| jtruthy(Some(v)));
        self.raw = raw;
    }

    pub fn point(&self, id: &str) -> Option<&SketchPoint> {
        self.points.iter().find(|p| p.id == id)
    }

    pub fn point_mut(&mut self, id: &str) -> Option<&mut SketchPoint> {
        self.points.iter_mut().find(|p| p.id == id)
    }

    pub fn entity(&self, id: &str) -> Option<&SketchEntity> {
        self.entities.iter().find(|e| e.id() == id)
    }

    /// Point position by id, also accepting point entities and circle/arc/
    /// ellipse ids (their centre) — client `Tl()`.
    pub fn point_xy(&self, id: &str) -> Option<[f64; 2]> {
        if let Some(p) = self.point(id) {
            return Some([p.x, p.y]);
        }
        match self.entity(id)? {
            SketchEntity::Point { p, .. } => self.point(p).map(|p| [p.x, p.y]),
            SketchEntity::Circle { c, .. } | SketchEntity::Arc { c, .. } | SketchEntity::Ellipse { c, .. } => {
                self.point(c).map(|p| [p.x, p.y])
            }
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
            if q.get("fixed").is_some() {
                obj_set(&mut q, "fixed", Json::Bool(p.fixed));
            }
            pts.push(q);
        }
        obj_set(&mut o, "points", Json::Arr(pts));
        // entities: raw entities with a known type, in order.
        let raw_ents: Vec<Json> =
            self.raw.get("entities").and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
        let mut ents = Vec::new();
        let mut it = self.entities.iter();
        for r in &raw_ents {
            if SketchEntity::from_json(r).is_some() {
                if let Some(e) = it.next() {
                    ents.push(e.normalize_json(r));
                }
            }
        }
        obj_set(&mut o, "entities", Json::Arr(ents));
        // constraints: verbatim except value coercion / expr-driven values.
        let raw_cons: Vec<Json> =
            self.raw.get("constraints").and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
        let mut cons = Vec::new();
        for (i, r) in raw_cons.iter().enumerate() {
            let mut q = r.clone();
            if let (Some(c), Json::Obj(_)) = (self.constraints.get(i), &q) {
                if c.valid {
                    match q.get("value") {
                        Some(Json::Arr(a)) => {
                            let a2 = a
                                .iter()
                                .map(|x| match x {
                                    Json::Str(s) => s.trim().parse::<f64>().map(Json::Num).unwrap_or(x.clone()),
                                    _ => x.clone(),
                                })
                                .collect();
                            obj_set(&mut q, "value", Json::Arr(a2));
                        }
                        Some(Json::Str(s)) if s.trim().parse::<f64>().is_err() && c.value.is_none() => {}
                        Some(_) => {
                            if let Some(v) = c.value {
                                obj_set(&mut q, "value", Json::Num(v));
                            }
                        }
                        None => {
                            if let Some(v) = c.value {
                                obj_set(&mut q, "value", Json::Num(v));
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

#[cfg(test)]
mod tests {
    use super::*;

    fn sk(src: &str) -> Sketch {
        Sketch::from_json(&ck_json::parse(src).unwrap())
    }

    /// Oracle: `"points":[{"id":"p1"},5,{"id":"p2","x":null,"y":true,"fixed":1}]`
    #[test]
    fn echo_points_normalised() {
        let s = sk(r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1"},5,{"id":"p2","x":null,"y":true,"fixed":1}]}"#);
        assert_eq!(
            ck_json::to_string(&s.to_json()),
            r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"","x":0,"y":0},{"id":"p2","x":0,"y":0,"fixed":true}],"entities":[],"constraints":[]}"#
        );
    }

    /// Oracle: key order and unknown keys survive, `"y":1.50` -> 1.5.
    #[test]
    fn echo_preserves_order() {
        let s = sk(r#"{"zz":5,"points":[{"id":"p1","y":0,"x":0,"foo":1},{"id":"p2","x":10,"y":1.50,"fixed":false}],"type":"sketch","id":"F1","plane":{"kind":"std","id":"top","extra":1},"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2","construction":false}],"constraints":[{"type":"horizontal","id":"c1","refs":["e1"],"bar":2}]}"#);
        assert_eq!(
            ck_json::to_string(&s.to_json()),
            r#"{"zz":5,"points":[{"id":"p1","y":0,"x":0,"foo":1},{"id":"p2","x":10,"y":1.5,"fixed":false}],"type":"sketch","id":"F1","plane":{"kind":"std","id":"top","extra":1},"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2","construction":false}],"constraints":[{"type":"horizontal","id":"c1","refs":["e1"],"bar":2}]}"#
        );
    }

    /// Oracle: entity normalisation (unknown/missing types dropped).
    #[test]
    fn echo_entities_normalised() {
        let s = sk(r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"entities":[{"type":"line"},{"id":"e2"},{"type":"circle","id":"e3","c":"p1","r":"4"},{"type":"arc","id":"e4"},{"type":"spline","id":"e5"},{"type":"cvspline","id":"e6"},{"type":"ellipse","id":"e7"},{"type":"point","id":"e8"},{"type":"ref","id":"e9"},5]}"#);
        assert_eq!(
            ck_json::to_string(&s.to_json()),
            r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"entities":[{"type":"line","id":"","p1":"","p2":""},{"type":"circle","id":"e3","c":"p1","r":4},{"type":"arc","id":"e4","c":"","p1":"","p2":""},{"type":"spline","id":"e5","pts":[]},{"type":"cvspline","id":"e6","pts":[],"degree":1},{"type":"ellipse","id":"e7","c":"","p":"","r":0},{"type":"point","id":"e8","p":""}],"points":[],"constraints":[]}"#
        );
    }

    /// Oracle: constraint value coercion.
    #[test]
    fn echo_constraints() {
        let s = sk(r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"constraints":[{"type":"radius","id":"k3","refs":["e1"],"value":"=2+3"},{"type":"radius","id":"k4","refs":["e1"]},{"type":"distance","refs":["a","b"]},{"type":"horizontal","id":"k6","refs":["e1"],"value":"9"},{"type":"distance","id":"k8","refs":["a",5],"value":5},{"type":"fix","id":"k9","refs":["a"],"value":[1,"2"]},{"type":"angle","id":"k10","refs":["a","b"],"value":5,"offset":"x","s1":"-1"}]}"#);
        assert_eq!(
            ck_json::to_string(&s.to_json()),
            r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"constraints":[{"type":"radius","id":"k3","refs":["e1"],"value":"=2+3"},{"type":"radius","id":"k4","refs":["e1"]},{"type":"distance","refs":["a","b"]},{"type":"horizontal","id":"k6","refs":["e1"],"value":9},{"type":"distance","id":"k8","refs":["a",5],"value":5},{"type":"fix","id":"k9","refs":["a"],"value":[1,2]},{"type":"angle","id":"k10","refs":["a","b"],"value":5,"offset":"x","s1":"-1"}],"points":[],"entities":[]}"#
        );
    }
}
