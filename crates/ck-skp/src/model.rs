//! The parsed SketchUp model: definitions (vertices, edges, faces, instances), materials, layers. Units are SketchUp's
//! internal inches; [`crate::scene`] converts to millimetres.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! The shape follows OpenSKP's parsed model (`defs_dict` / `_GeometryBuilder` in the Python package, `model.hpp` in the
//! C++ one).

use std::collections::HashMap;

/// Insertion-ordered map from an id to a value (OpenSKP relies on dict insertion order for a stable output order).
#[derive(Debug, Clone)]
pub struct OrderedMap<V> {
    items: Vec<(u64, V)>,
    index: HashMap<u64, usize>,
}

impl<V> Default for OrderedMap<V> {
    fn default() -> Self {
        OrderedMap { items: Vec::new(), index: HashMap::new() }
    }
}

impl<V> OrderedMap<V> {
    pub fn new() -> Self {
        Self::default()
    }
    /// Insert or replace (a replaced key keeps its original position, like a Python dict).
    pub fn insert(&mut self, k: u64, v: V) {
        match self.index.get(&k) {
            Some(&i) => self.items[i].1 = v,
            None => {
                self.index.insert(k, self.items.len());
                self.items.push((k, v));
            }
        }
    }
    pub fn get(&self, k: u64) -> Option<&V> {
        self.index.get(&k).map(|&i| &self.items[i].1)
    }
    pub fn contains(&self, k: u64) -> bool {
        self.index.contains_key(&k)
    }
    pub fn len(&self) -> usize {
        self.items.len()
    }
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    pub fn iter(&self) -> impl Iterator<Item = (u64, &V)> {
        self.items.iter().map(|(k, v)| (*k, v))
    }
    pub fn values(&self) -> impl Iterator<Item = &V> {
        self.items.iter().map(|(_, v)| v)
    }
}

/// Sentinel for "no vertex" in edge ends and loop vertex lists.
pub const NO_ID: u64 = u64::MAX;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge {
    /// Start / end vertex ids ([`NO_ID`] when the file did not name one).
    pub v1: u64,
    pub v2: u64,
    /// Display flags: 0x01 hidden, 0x08 soft, 0x10 smooth.
    pub flags: u8,
    /// Layer (tag) id, if the edge carries one.
    pub layer: Option<u64>,
    /// Parent curve id (`Edge#curve`), if the edge is part of a curve.
    pub curve: Option<u64>,
}

/// One coedge: edge id and `+1` (same direction as the edge) or `-1` (reversed).
pub type CoEdge = (u64, i8);

#[derive(Debug, Clone, PartialEq)]
pub struct Face {
    /// Outer loop first, then holes.
    pub loops: Vec<Vec<CoEdge>>,
    pub normal: [f64; 3],
    pub material_id: Option<u64>,
    pub back_material_id: Option<u64>,
    pub layer: Option<u64>,
    /// 3x3 row-major texture-space -> face-plane matrices (inches), when the face has a positioned texture.
    pub uv_front: Option<[f64; 9]>,
    pub uv_back: Option<[f64; 9]>,
    pub hidden: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Instance {
    pub name: String,
    /// Definition id this instance places ([`Model::definitions`] key).
    pub def: Option<u64>,
    pub guid: String,
    /// 3x4 affine (row-major rotation `r00..r22`, translation `tx ty tz`) plus uniform scale: 13 numbers.
    pub matrix: Option<[f64; 13]>,
    pub material_id: Option<u64>,
    pub hidden: bool,
    pub layer: Option<u64>,
}

#[derive(Debug, Clone, Default)]
pub struct Definition {
    /// Entity id / archive slot (`None` for the root).
    pub id: Option<u64>,
    pub guid: String,
    pub name: String,
    pub always_faces_camera: bool,
    pub shadows_face_sun: bool,
    pub is_image: bool,
    pub vertices: OrderedMap<[f64; 3]>,
    pub edges: OrderedMap<Edge>,
    pub faces: OrderedMap<Face>,
    pub instances: Vec<Instance>,
}

#[derive(Debug, Clone)]
pub struct Texture {
    pub filename: String,
    /// Tile size in inches (0 when absent).
    pub x_scale: f64,
    pub y_scale: f64,
    /// The image file exactly as stored (PNG / JPEG / ...).
    pub data: Option<Vec<u8>>,
}

#[derive(Debug, Clone)]
pub struct Material {
    pub name: String,
    pub rgba: [u8; 4],
    /// 1 = opaque, 0 = invisible.
    pub opacity: f64,
    pub colorized: bool,
    pub colorize_type: i32,
    pub texture: Option<Texture>,
}

#[derive(Debug, Clone)]
pub struct Layer {
    pub name: String,
    pub rgb: [u8; 3],
    pub hidden: bool,
}

#[derive(Debug, Clone)]
pub struct Style {
    pub name: String,
    pub front_color: Option<[u8; 3]>,
    pub back_color: Option<[u8; 3]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Container {
    /// SketchUp 2021+: binary header plus ZIP with a TLV `model.dat`.
    Vff,
    /// SketchUp 3 to 2020: MFC `CArchive` object stream.
    Legacy,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Warning {
    pub code: &'static str,
    pub message: String,
    pub offset: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct Model {
    pub container: Container,
    /// Version string from the header, e.g. `{25.0.575}` (VFF) or `{17.0.18}` style (legacy), or `unknown`.
    pub version: String,
    /// Major version (SketchUp 3 .. 25) when it could be read.
    pub version_major: Option<u32>,
    /// Display unit string from `meta/meta.dat` (VFF only), e.g. `Millimeter`. Geometry is always stored in inches.
    pub units: Option<String>,
    /// Materials keyed by name in `material_by_name` (later duplicates replace earlier ones, like OpenSKP).
    pub materials: Vec<Material>,
    pub material_by_name: HashMap<String, usize>,
    /// VFF: material folder name -> material (fallback join).
    pub material_by_folder: HashMap<String, usize>,
    /// Face / instance `material_id` -> material name.
    pub material_id_to_name: HashMap<u64, String>,
    pub layers: Vec<Layer>,
    pub layer_id_to_name: HashMap<u64, String>,
    pub styles: Vec<Style>,
    /// Component / group definitions keyed by id, in file order.
    pub definitions: OrderedMap<Definition>,
    /// The model root (loose geometry and top-level instances).
    pub root: Definition,
    pub warnings: Vec<Warning>,
}

impl Model {
    pub fn new(container: Container) -> Model {
        Model {
            container,
            version: "unknown".into(),
            version_major: None,
            units: None,
            materials: vec![],
            material_by_name: HashMap::new(),
            material_by_folder: HashMap::new(),
            material_id_to_name: HashMap::new(),
            layers: vec![],
            layer_id_to_name: HashMap::new(),
            styles: vec![],
            definitions: OrderedMap::new(),
            root: Definition::default(),
            warnings: vec![],
        }
    }

    pub fn warn(&mut self, code: &'static str, message: impl Into<String>, offset: Option<usize>) {
        if self.warnings.len() < 200 {
            self.warnings.push(Warning { code, message: message.into(), offset });
        }
    }

    /// Resolve a face / instance material id: id -> name -> material by name, falling back to the folder join.
    pub fn resolve_material(&self, id: Option<u64>) -> Option<&Material> {
        let name = self.material_id_to_name.get(&id?)?;
        let i = self.material_by_name.get(name).or_else(|| self.material_by_folder.get(name))?;
        self.materials.get(*i)
    }

    pub fn layer_by_name(&self, name: &str) -> Option<&Layer> {
        self.layers.iter().find(|l| l.name == name)
    }

    /// Totals over all definitions and the root: (vertices, edges, faces, instances).
    pub fn totals(&self) -> (usize, usize, usize, usize) {
        let mut t = (self.root.vertices.len(), self.root.edges.len(), self.root.faces.len(), self.root.instances.len());
        for d in self.definitions.values() {
            t.0 += d.vertices.len();
            t.1 += d.edges.len();
            t.2 += d.faces.len();
            t.3 += d.instances.len();
        }
        t
    }
}
