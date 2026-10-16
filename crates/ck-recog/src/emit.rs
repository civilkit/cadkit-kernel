//! Feature JSON builders (the shapes the app and the scripts use).

use crate::model::PlaneSpec;
use ck_json::{Json, Obj};

pub fn n(v: f64) -> Json {
    let r = (v * 1e9).round() / 1e9;
    Json::Num(if r == 0.0 { 0.0 } else { r })
}

pub fn s(v: &str) -> Json {
    Json::Str(v.to_string())
}

pub fn arr(v: Vec<Json>) -> Json {
    Json::Arr(v)
}

pub fn vec3(p: ck_geom::Vec3) -> Json {
    arr(vec![n(p.x), n(p.y), n(p.z)])
}

/// Feature id allocator (`F1`, `F2`, ...).
#[derive(Default)]
pub struct Ids(pub usize);

impl Ids {
    pub fn next(&mut self) -> String {
        self.0 += 1;
        format!("F{}", self.0)
    }
}

/// The feature list under construction.
#[derive(Default)]
pub struct Tree {
    pub ids: Ids,
    pub feats: Vec<Json>,
}

impl Tree {
    pub fn push(&mut self, f: Json) {
        self.feats.push(f);
    }

    /// A sketch plane reference at `level` along the plane normal (adds an offset plane feature when needed).
    pub fn plane(&mut self, spec: &PlaneSpec, level: f64) -> Json {
        let std = Obj::new().put("kind", "std").put("id", spec.std).build();
        if level.abs() < 1e-9 {
            return std;
        }
        let id = self.ids.next();
        self.push(
            Obj::new()
                .put("type", "plane")
                .put("id", id.as_str())
                .put("mode", "offset")
                .put("base", std)
                .put("offset", n(level))
                .build(),
        );
        Obj::new().put("kind", "feature").put("id", id.as_str()).build()
    }
}

/// A sketch under construction.
pub struct Sk {
    pub id: String,
    pub plane: Json,
    pub points: Vec<Json>,
    pub entities: Vec<Json>,
    np: usize,
    ne: usize,
}

impl Sk {
    pub fn new(id: String, plane: Json) -> Sk {
        Sk { id, plane, points: vec![], entities: vec![], np: 0, ne: 0 }
    }
    pub fn point(&mut self, x: f64, y: f64) -> String {
        self.np += 1;
        let id = format!("p{}", self.np);
        self.points.push(Obj::new().put("id", id.as_str()).put("x", n(x)).put("y", n(y)).build());
        id
    }
    fn eid(&mut self) -> String {
        self.ne += 1;
        format!("e{}", self.ne)
    }
    pub fn line(&mut self, a: &str, b: &str, construction: bool) -> String {
        let id = self.eid();
        let mut o = Obj::new().put("type", "line").put("id", id.as_str()).put("p1", a).put("p2", b);
        if construction {
            o = o.put("construction", true);
        }
        self.entities.push(o.build());
        id
    }
    /// Counter-clockwise arc about `c` from `p1` to `p2`.
    pub fn arc(&mut self, c: &str, p1: &str, p2: &str) -> String {
        let id = self.eid();
        self.entities.push(Obj::new().put("type", "arc").put("id", id.as_str()).put("c", c).put("p1", p1).put("p2", p2).build());
        id
    }
    pub fn circle(&mut self, c: &str, r: f64) -> String {
        let id = self.eid();
        self.entities.push(Obj::new().put("type", "circle").put("id", id.as_str()).put("c", c).put("r", n(r)).build());
        id
    }
    pub fn point_entity(&mut self, p: &str) -> String {
        let id = self.eid();
        self.entities.push(Obj::new().put("type", "point").put("id", id.as_str()).put("p", p).build());
        id
    }
    pub fn json(self) -> Json {
        Obj::new()
            .put("type", "sketch")
            .put("id", self.id.as_str())
            .put("plane", self.plane)
            .put("points", arr(self.points))
            .put("entities", arr(self.entities))
            .put("constraints", arr(vec![]))
            .build()
    }
}

pub fn extrude(id: &str, sketch: &str, op: &str, depth: f64, flip: bool, through: bool) -> Json {
    let mut o = Obj::new().put("type", "extrude").put("id", id).put("op", op).put("sketch", sketch);
    if through {
        o = o.put("end", "through");
    } else {
        o = o.put("end", "blind").put("depth", n(depth));
    }
    if flip {
        o = o.put("flip", true);
    }
    o.build()
}

pub fn revolve(id: &str, sketch: &str, op: &str, axis: &str) -> Json {
    Obj::new().put("type", "revolve").put("id", id).put("op", op).put("sketch", sketch).put("axis", axis).put("angle", 360.0).build()
}
