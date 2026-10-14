//! Data model: Part documents, features, sketches, constraints, references.
//!
//! Everything here is the wire format of `.ckpart` files and kernel messages.
//! Field names MUST match the JSON exactly — the schema is pinned by the
//! .ckpart samples and oracle goldens. JSON mapping lives in `ck-json`
//! (the original kernel has no serde; it uses a hand-rolled DOM).
//!
//! Parsing is deliberately lenient, like the original: the only hard part
//! error is `"Format"` (reply `invalid part: Format`, kernel.strings), raised
//! when the part is not an object, `format != "ck-part"`, or `features`
//! is not an array. Everything else (bad feature types, missing fields,
//! wrong JSON types) parses with defaults and surfaces as per-feature regen
//! errors (`unknownFeature`, `missingReference`, `missingSketch`, ...).
//! Every feature keeps its raw DOM in `Feature::raw` for fields not typed here.

pub mod expr;
pub mod features;
pub mod interference;
pub mod planes;
pub mod refs;
pub mod regen;
pub mod sketch;
pub mod sketch3;
pub mod threads;

pub use sketch::{Constraint, Sketch, SketchEntity, SketchPoint};

use ck_json::Json;

/// The part-level parse error text (`invalid part: Format`).
pub const PART_FORMAT_ERROR: &str = "Format";
/// The only accepted `format` value.
pub const PART_FORMAT: &str = "ck-part";

/// Standard sketch planes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StdPlane {
    Front,
    Top,
    Right,
}

impl StdPlane {
    pub fn from_wire(s: &str) -> Option<StdPlane> {
        match s {
            "front" => Some(StdPlane::Front),
            "top" => Some(StdPlane::Top),
            "right" => Some(StdPlane::Right),
            _ => None,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            StdPlane::Front => "front",
            StdPlane::Top => "top",
            StdPlane::Right => "right",
        }
    }
    /// The plane frame (ck-core `std_plane`, = the client's `Ot` table).
    pub fn frame(self) -> ck_core::linalg::Frame {
        ck_core::linalg::std_plane(self.as_str())
    }
}

/// A geometric reference object, `{"kind": ..}` on the wire, or a bare
/// string (sketch entity / feature id). Kinds seen in the samples:
/// `{"kind":"std","id":"front"}`, `{"kind":"face","id":"F2.C1"}`,
/// `{"kind":"feature","id":"F12"}` (a plane feature), `{"kind":"curve","id":"F11"}`
/// (sweep path), `{"kind":"axis","axis":"x"}` (pattern/insert axes);
/// revolve/surfaceRevolve `axis` is a bare sketch-entity id (`"e14"`).
#[derive(Debug, Clone, PartialEq)]
pub enum GeomRef {
    Std(StdPlane),
    Face(String),
    Feature(String),
    Curve(String),
    /// Principal axis name (`"x"`, `"y"`, `"z"`).
    Axis(String),
    /// A bare string reference.
    Name(String),
    /// Any other `{kind}` object, kept raw.
    Other(Json),
}

/// Sketch/assembly plane reference (same wire shape as [`GeomRef`]).
pub type Plane = GeomRef;

impl GeomRef {
    pub fn from_json(j: &Json) -> Option<GeomRef> {
        match j {
            Json::Str(s) => Some(GeomRef::Name(s.clone())),
            Json::Obj(_) => {
                let id = || j.get("id").and_then(|v| v.as_str()).map(|s| s.to_string());
                Some(match j.get("kind").and_then(|k| k.as_str()) {
                    Some("std") => match id().as_deref().and_then(StdPlane::from_wire) {
                        Some(p) => GeomRef::Std(p),
                        None => GeomRef::Other(j.clone()),
                    },
                    Some("face") => id().map(GeomRef::Face).unwrap_or(GeomRef::Other(j.clone())),
                    Some("feature") => id().map(GeomRef::Feature).unwrap_or(GeomRef::Other(j.clone())),
                    Some("curve") => id().map(GeomRef::Curve).unwrap_or(GeomRef::Other(j.clone())),
                    Some("axis") => match j.get("axis").and_then(|v| v.as_str()) {
                        Some(a) => GeomRef::Axis(a.to_string()),
                        None => GeomRef::Other(j.clone()),
                    },
                    _ => GeomRef::Other(j.clone()),
                })
            }
            _ => None,
        }
    }

    pub fn to_json(&self) -> Json {
        let kv = |k: &str, id: &str| {
            Json::Obj(vec![("kind".into(), Json::Str(k.into())), ("id".into(), Json::Str(id.into()))])
        };
        match self {
            GeomRef::Std(p) => kv("std", p.as_str()),
            GeomRef::Face(s) => kv("face", s),
            GeomRef::Feature(s) => kv("feature", s),
            GeomRef::Curve(s) => kv("curve", s),
            GeomRef::Axis(a) => {
                Json::Obj(vec![("kind".into(), Json::Str("axis".into())), ("axis".into(), Json::Str(a.clone()))])
            }
            GeomRef::Name(s) => Json::Str(s.clone()),
            GeomRef::Other(j) => j.clone(),
        }
    }
}

/// Reference selector string: `"F3"` (feature), `"e2"` (sketch entity),
/// `"F4.C0@F1.1"` (derived face), `"F8.B0"` (blend ball), etc.
/// Kept as an opaque string on the wire; resolved by `ck-model::refs`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Ref(pub String);

impl Ref {
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for Ref {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl From<&str> for Ref {
    fn from(s: &str) -> Self {
        Ref(s.to_string())
    }
}

/// An edge selection: `{"faces":["F2.C0","F2.S.e10"],"at":[-26,58,2]}` —
/// the faces meeting at the edge plus a point on it (fillet/chamfer/fill/lip).
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeRef {
    pub faces: Vec<String>,
    pub at: Option<[f64; 3]>,
}

impl EdgeRef {
    pub fn from_json(j: &Json) -> Option<EdgeRef> {
        let faces = j.get("faces")?.as_arr()?.iter().filter_map(|f| f.as_str().map(|s| s.to_string())).collect();
        let at = j.get("at").and_then(|a| a.as_arr()).and_then(|a| {
            if a.len() == 3 {
                Some([a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?])
            } else {
                None
            }
        });
        Some(EdgeRef { faces, at })
    }
}

/// A numeric field value: a JSON number, or a string (`"=d/2"` expression,
/// or a plain number string). Evaluate with [`Expr::eval`].
#[derive(Debug, Clone, PartialEq)]
pub enum Expr {
    Num(f64),
    Text(String),
}

impl Expr {
    pub fn from_json(j: &Json) -> Option<Expr> {
        match j {
            Json::Num(n) => Some(Expr::Num(*n)),
            Json::Str(s) => Some(Expr::Text(s.clone())),
            _ => None,
        }
    }
    /// Evaluate per [`expr::eval_field`] semantics (error `badExpression`).
    pub fn eval(&self, vars: &expr::Vars) -> Result<f64, &'static str> {
        match self {
            Expr::Num(n) => Ok(*n),
            Expr::Text(s) => expr::eval_field(Some(&Json::Str(s.clone())), vars, 0.0),
        }
    }
    pub fn as_num(&self) -> Option<f64> {
        match self {
            Expr::Num(n) => Some(*n),
            _ => None,
        }
    }
    pub fn to_json(&self) -> Json {
        match self {
            Expr::Num(n) => Json::Num(*n),
            Expr::Text(s) => Json::Str(s.clone()),
        }
    }
}

impl From<&str> for Expr {
    fn from(s: &str) -> Self {
        Expr::Text(s.to_string())
    }
}

impl From<f64> for Expr {
    fn from(v: f64) -> Self {
        Expr::Num(v)
    }
}

/// Part document — the `.ckpart` file format and kernel input.
/// JSON: `{"format":"ck-part","version":1,"name":...,"features":[...],
/// "seq":13,"rollback":null,"material":"steel","counters":{"sketch":4,..},
/// "variables":[{"name":"D1","expr":"10"}], "kind":"assembly",
/// "exploded":{..}, "appearance":{..}}`
#[derive(Debug, Clone)]
pub struct Part {
    /// Always `"ck-part"`.
    pub format: String,
    /// 1 in every sample; not validated by the kernel (version 2 regens fine).
    pub version: u32,
    pub name: String,
    /// The regen program, in order.
    pub features: Vec<Feature>,
    /// Monotonic counter used to mint feature ids (`"F" + seq`).
    pub seq: u32,
    /// Numeric rollback = regenerate only the first N features (oracle:
    /// `rollback:1` keeps only F1; a string rollback is ignored).
    pub rollback: Option<usize>,
    pub material: String,
    /// Per-command name counters (`{"sketch":4,"cutExtrude":2,..}`), used by
    /// the client to number feature names.
    pub counters: Vec<(String, u32)>,
    /// Global variables (equations dialog).
    pub variables: Vec<expr::Variable>,
    /// `"assembly"` for assembly documents, else empty.
    pub kind: String,
    /// The part object as received.
    pub raw: Json,
}

/// Feature envelope shared by all kinds.
#[derive(Debug, Clone)]
pub struct Feature {
    /// `""` when missing (oracle: errors are then keyed by `""`).
    pub id: String,
    /// Display ordinal.
    pub num: u32,
    pub name: String,
    /// Wire `type` (`""` when missing).
    pub type_name: String,
    /// `suppressed: true` features are skipped by regen (oracle).
    pub suppressed: bool,
    pub kind: features::FeatureKind,
    /// The feature object as received.
    pub raw: Json,
}

impl Part {
    /// Parse a part document. Errors only with `"Format"` (see crate doc).
    pub fn from_json(j: &Json) -> Result<Part, String> {
        if !matches!(j, Json::Obj(_)) || j.get("format").and_then(|f| f.as_str()) != Some(PART_FORMAT) {
            return Err(PART_FORMAT_ERROR.to_string());
        }
        let feats = j.get("features").and_then(|f| f.as_arr()).ok_or_else(|| PART_FORMAT_ERROR.to_string())?;
        let num = |k: &str| j.get(k).and_then(|v| v.as_f64());
        Ok(Part {
            format: PART_FORMAT.to_string(),
            version: num("version").unwrap_or(1.0) as u32,
            name: j.get("name").and_then(|v| v.as_str()).unwrap_or("").to_string(),
            features: feats.iter().map(Feature::from_json).collect(),
            seq: num("seq").unwrap_or(0.0).max(0.0) as u32,
            rollback: num("rollback").map(|r| r.max(0.0) as usize),
            material: j.get("material").and_then(|v| v.as_str()).unwrap_or("").to_string(),
            counters: match j.get("counters") {
                Some(Json::Obj(m)) => m.iter().filter_map(|(k, v)| v.as_f64().map(|n| (k.clone(), n as u32))).collect(),
                _ => Vec::new(),
            },
            variables: expr::parse_variables(j.get("variables")),
            kind: j.get("kind").and_then(|v| v.as_str()).unwrap_or("").to_string(),
            raw: j.clone(),
        })
    }

    /// Parse from JSON text (convenience; JSON syntax errors also map to "Format").
    pub fn parse(src: &str) -> Result<Part, String> {
        let j = ck_json::parse(src).map_err(|_| PART_FORMAT_ERROR.to_string())?;
        Part::from_json(&j)
    }

    /// Serialise back to the DOM (the raw document with sketches re-echoed).
    pub fn to_json(&self) -> Json {
        let mut o = self.raw.clone();
        sketch::obj_set(&mut o, "features", Json::Arr(self.features.iter().map(|f| f.to_json()).collect()));
        o
    }

    /// Find a feature by id (`"F4"`).
    pub fn feature(&self, id: &str) -> Option<&Feature> {
        self.features.iter().find(|f| f.id == id)
    }

    /// Index of a feature by id.
    pub fn feature_index(&self, id: &str) -> Option<usize> {
        self.features.iter().position(|f| f.id == id)
    }

    pub fn is_assembly(&self) -> bool {
        self.kind == "assembly"
    }
}

impl Feature {
    pub fn from_json(j: &Json) -> Feature {
        let s = |k: &str| j.get(k).and_then(|v| v.as_str()).unwrap_or("").to_string();
        let type_name = s("type");
        Feature {
            id: s("id"),
            num: j.get("num").and_then(|v| v.as_f64()).unwrap_or(0.0).max(0.0) as u32,
            name: s("name"),
            suppressed: sketch::jtruthy(j.get("suppressed")),
            kind: features::FeatureKind::from_json(&type_name, j),
            type_name,
            raw: j.clone(),
        }
    }

    pub fn to_json(&self) -> Json {
        match &self.kind {
            features::FeatureKind::Sketch(s) => s.to_json(),
            _ => self.raw.clone(),
        }
    }

    pub fn as_sketch(&self) -> Option<&Sketch> {
        match &self.kind {
            features::FeatureKind::Sketch(s) => Some(s),
            _ => None,
        }
    }
}
