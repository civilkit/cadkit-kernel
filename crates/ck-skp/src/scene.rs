//! Scene baking: flatten the placed component / group instances of a [`Model`] into world-space, triangulated meshes
//! grouped by resolved material, in millimetres (SketchUp's Z-up frame is kept; inches become millimetres).
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! Follows OpenSKP's `build_scene` / `build_local_face_groups` (`scene.py`, `_face_groups.py`, `scene.cpp`): faces are
//! grouped by (colour, double sidedness, texture, opacity); a face whose front and back colours differ is emitted twice
//! (front, and reversed back). Differences: triangle winding is made to agree with the face normal (also under mirrored
//! instance transforms), nesting depth, instance count and triangle count are limited with structured errors, and
//! recursive definitions are skipped with a warning instead of raising.

use crate::error::{Limits, SkpError, R};
use crate::model::*;
use crate::tri::triangulate_loops;
use std::collections::{HashMap, HashSet};

pub const INCHES_TO_MM: f64 = 25.4;

#[derive(Debug, Clone)]
pub struct BakeOptions {
    /// Bake hidden faces, instances and layers too (OpenSKP does; a CAD import should not).
    pub include_hidden: bool,
    /// Deepest instance nesting baked (root = 0). Deeper instances are skipped with a warning.
    pub max_depth: usize,
    /// Most instance placements visited.
    pub max_instances: usize,
    /// Most triangles produced.
    pub max_triangles: usize,
    /// Also keep the world-space polygons (loops with holes) of every baked face, for [`crate::body`].
    pub keep_polygons: bool,
    /// Reproduce OpenSKP exactly where it is lossy: it ignores the layer and paint of instances of classic (pre-2021)
    /// files when baking. Used by the parity tests; off by default (the paint is applied).
    pub openskp_compat: bool,
    /// Which way is up in the baked scene; see [`Up`]. Default [`Up::Z`] (as in the file).
    pub up: Up,
}

/// Orientation of the baked scene. SketchUp is Z up (right-handed); the kit and kernel convention is Y up. The change is
/// a proper rotation (determinant +1) applied to the finished scene (after instance flattening and the inch to mm
/// scale), so winding, face order, areas and volumes are unchanged and normals rotate with the geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Up {
    /// As in the file: (x, y, z) -> (x, y, z).
    #[default]
    Z,
    /// SketchUp Z becomes our Y: (x, y, z) -> (x, z, -y), a -90 degree rotation about X. SketchUp's front (-Y) faces +Z.
    Y,
    /// `Y` followed by 180 degrees about Z, for models that arrive upside down: (x, y, z) -> (-x, -z, -y).
    YFlip,
}

impl Up {
    pub fn parse(s: &str) -> Option<Up> {
        match s {
            "z" => Some(Up::Z),
            "y" => Some(Up::Y),
            "yFlip" => Some(Up::YFlip),
            _ => None,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Up::Z => "z",
            Up::Y => "y",
            Up::YFlip => "yFlip",
        }
    }
    /// Row-major 3x3 rotation applied to column vectors.
    pub fn matrix(self) -> [[f64; 3]; 3] {
        match self {
            Up::Z => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            Up::Y => [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
            Up::YFlip => [[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]],
        }
    }
    pub fn apply(self, v: [f64; 3]) -> [f64; 3] {
        let m = self.matrix();
        [0, 1, 2].map(|r| m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2])
    }
}

impl Default for BakeOptions {
    fn default() -> Self {
        BakeOptions { include_hidden: false, max_depth: 32, max_instances: 2_000_000, max_triangles: 30_000_000, keep_polygons: false, openskp_compat: false, up: Up::Z }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SceneMaterial {
    pub rgb: [u8; 3],
    /// 1 = opaque.
    pub opacity: f32,
    pub double_sided: bool,
    /// Index into [`Scene::textures`].
    pub texture: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct SceneTexture {
    pub data: Vec<u8>,
    pub mime: &'static str,
    pub filename: String,
}

/// One triangle mesh with one material, in world space (mm, Z up). Vertices are split where UVs or materials differ.
#[derive(Debug, Clone, Default)]
pub struct Primitive {
    pub material: usize,
    pub positions: Vec<[f64; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<u32>,
    /// Instance path (`ROOT / a / b`), tag name and definition name of the geometry's owner.
    pub path: String,
    pub layer: String,
    pub definition: String,
}

/// A planar face in world space (mm): loops with the outer one first. Kept when [`BakeOptions::keep_polygons`].
#[derive(Debug, Clone)]
pub struct PolyFace {
    pub loops: Vec<Vec<[f64; 3]>>,
    /// Unit normal of the front side in world space.
    pub normal: [f64; 3],
    /// Index into [`Scene::materials`] of the front side.
    pub material: usize,
    pub layer: String,
    pub path: String,
}

#[derive(Debug, Clone, Default)]
pub struct SceneStats {
    pub instances: usize,
    pub faces: usize,
    pub triangles: usize,
    pub skipped_hidden: usize,
    pub skipped_missing_def: usize,
    pub skipped_recursive: usize,
    pub skipped_too_deep: usize,
    pub untriangulated_faces: usize,
}

#[derive(Debug, Clone, Default)]
pub struct Scene {
    pub materials: Vec<SceneMaterial>,
    pub textures: Vec<SceneTexture>,
    pub primitives: Vec<Primitive>,
    pub polygons: Vec<PolyFace>,
    pub stats: SceneStats,
    pub warnings: Vec<String>,
}

impl Scene {
    /// Axis-aligned box of all vertices (mm).
    pub fn bbox(&self) -> Option<([f64; 3], [f64; 3])> {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut any = false;
        for p in &self.primitives {
            for v in &p.positions {
                any = true;
                for k in 0..3 {
                    lo[k] = lo[k].min(v[k]);
                    hi[k] = hi[k].max(v[k]);
                }
            }
        }
        any.then_some((lo, hi))
    }

    pub fn triangle_count(&self) -> usize {
        self.primitives.iter().map(|p| p.indices.len() / 3).sum()
    }

    /// Total surface area of all triangles (mm^2).
    pub fn area(&self) -> f64 {
        let mut a = 0.0;
        for p in &self.primitives {
            for t in p.indices.chunks_exact(3) {
                let (v0, v1, v2) = (p.positions[t[0] as usize], p.positions[t[1] as usize], p.positions[t[2] as usize]);
                let (e1, e2) = ([v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]], [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]]);
                let c = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
                a += 0.5 * (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
            }
        }
        a
    }

    /// One primitive per material (vertices concatenated), in material order.
    pub fn merged_by_material(&self) -> Vec<Primitive> {
        let mut out: Vec<Primitive> = (0..self.materials.len()).map(|m| Primitive { material: m, ..Default::default() }).collect();
        for p in &self.primitives {
            let o = &mut out[p.material];
            let base = o.positions.len() as u32;
            o.positions.extend_from_slice(&p.positions);
            o.normals.extend_from_slice(&p.normals);
            o.uvs.extend_from_slice(&p.uvs);
            o.indices.extend(p.indices.iter().map(|i| i + base));
        }
        out.retain(|p| !p.indices.is_empty());
        out
    }
}

// ---- matrices (13 numbers: 3x3 rows, translation, scale)

const IDENT: [f64; 13] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

pub fn transform_point(m: &[f64; 13], p: [f64; 3]) -> [f64; 3] {
    [m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[9], m[3] * p[0] + m[4] * p[1] + m[5] * p[2] + m[10], m[6] * p[0] + m[7] * p[1] + m[8] * p[2] + m[11]]
}

pub fn rotate_vec(m: &[f64; 13], p: [f64; 3]) -> [f64; 3] {
    [m[0] * p[0] + m[1] * p[1] + m[2] * p[2], m[3] * p[0] + m[4] * p[1] + m[5] * p[2], m[6] * p[0] + m[7] * p[1] + m[8] * p[2]]
}

/// A surface normal under the affine map `m`: the inverse transpose applied to `n` (as the cofactor matrix times the sign
/// of the determinant, so a mirror keeps the side: the same as `rotate_vec` for rotations, mirrors and uniform scales). `rotate_vec` is only right for rotations and uniform scales;
/// under a non-uniform scale it tilts the normal off the transformed face plane.
pub fn transform_normal(m: &[f64; 13], n: [f64; 3]) -> [f64; 3] {
    let (r0, r1, r2) = ([m[0], m[1], m[2]], [m[3], m[4], m[5]], [m[6], m[7], m[8]]);
    let cr = |a: [f64; 3], b: [f64; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    // rows of the cofactor matrix are the cross products of the other two rows
    let (c0, c1, c2) = (cr(r1, r2), cr(r2, r0), cr(r0, r1));
    let d = |c: [f64; 3]| c[0] * n[0] + c[1] * n[1] + c[2] * n[2];
    let sg = if det3(m) < 0.0 { -1.0 } else { 1.0 };
    [sg * d(c0), sg * d(c1), sg * d(c2)]
}

/// `parent * child` (the child is applied first).
pub fn multiply(parent: &[f64; 13], child: &[f64; 13]) -> [f64; 13] {
    let (pr0, pr1, pr2) = ([parent[0], parent[1], parent[2], parent[9]], [parent[3], parent[4], parent[5], parent[10]], [parent[6], parent[7], parent[8], parent[11]]);
    let (c0, c1, c2, c3) = ([child[0], child[3], child[6], 0.0], [child[1], child[4], child[7], 0.0], [child[2], child[5], child[8], 0.0], [child[9], child[10], child[11], 1.0]);
    let d = |r: [f64; 4], c: [f64; 4]| r[0] * c[0] + r[1] * c[1] + r[2] * c[2] + r[3] * c[3];
    [d(pr0, c0), d(pr0, c1), d(pr0, c2), d(pr1, c0), d(pr1, c1), d(pr1, c2), d(pr2, c0), d(pr2, c1), d(pr2, c2), d(pr0, c3), d(pr1, c3), d(pr2, c3), parent[12] * child[12]]
}

fn det3(m: &[f64; 13]) -> f64 {
    m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6])
}

// ---- UV (OpenSKP `face_uv_basis` / `compute_face_uv`)

const VERTICAL_TOLERANCE: f64 = 1e-3;

fn face_uv_basis(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let (cx, cy) = (-n[1], n[0]);
    let clen = (cx * cx + cy * cy).sqrt();
    if clen < VERTICAL_TOLERANCE {
        let xr = if n[2] >= 0.0 { [1.0, 0.0, 0.0] } else { [-1.0, 0.0, 0.0] };
        (xr, [0.0, 1.0, 0.0])
    } else {
        let xr = [cx / clen, cy / clen, 0.0];
        let yr = [n[1] * xr[2] - n[2] * xr[1], n[2] * xr[0] - n[0] * xr[2], n[0] * xr[1] - n[1] * xr[0]];
        (xr, yr)
    }
}

fn invert3(m: &[f64; 9]) -> [f64; 9] {
    let [a, b, c, d, e, f, g, h, i] = *m;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if !(det.abs() >= 1e-12) {
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    }
    let k = 1.0 / det;
    [(e * i - f * h) * k, (c * h - b * i) * k, (b * f - c * e) * k, (f * g - d * i) * k, (a * i - c * g) * k, (c * d - a * f) * k, (d * h - e * g) * k, (b * g - a * h) * k, (a * e - b * d) * k]
}

fn face_uv(p: [f64; 3], xr: [f64; 3], yr: [f64; 3], uvt: Option<&[f64; 9]>, tw: f64, th: f64) -> (f64, f64) {
    let px = p[0] * xr[0] + p[1] * xr[1] + p[2] * xr[2];
    let py = p[0] * yr[0] + p[1] * yr[1] + p[2] * yr[2];
    match uvt {
        None => (px / tw, py / th),
        Some(t) => {
            let inv = invert3(t);
            let u = px * inv[0] + py * inv[3] + inv[6];
            let v = px * inv[1] + py * inv[4] + inv[7];
            let mut q = px * inv[2] + py * inv[5] + inv[8];
            if q.abs() < 1e-12 {
                q = 1.0;
            }
            ((u / q) / tw, (v / q) / th)
        }
    }
}

fn sniff_mime(d: &[u8]) -> Option<&'static str> {
    if d.len() >= 3 && d[0] == 0xFF && d[1] == 0xD8 && d[2] == 0xFF {
        Some("image/jpeg")
    } else if d.len() >= 8 && d[..8] == [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A] {
        Some("image/png")
    } else {
        None
    }
}

// ---- baking

struct CachedFace {
    loops: Vec<Vec<u64>>,
    tris: Vec<[u64; 3]>,
    normal: [f64; 3],
    front: Option<u64>,
    back: Option<u64>,
    uv_front: Option<[f64; 9]>,
    uv_back: Option<[f64; 9]>,
    layer: Option<u64>,
    hidden: bool,
}

type MatKey = ([u8; 3], bool, Option<usize>, u32);

#[derive(Default)]
struct Group {
    mat: usize,
    prim: Primitive,
    vmap: HashMap<(u64, u64, u64), u32>,
    normals_acc: Vec<[f64; 3]>,
}

struct Baker<'m> {
    m: &'m Model,
    lim: &'m Limits,
    opt: &'m BakeOptions,
    cache: HashMap<u64, Vec<CachedFace>>,
    active: HashSet<u64>,
    mat_index: HashMap<MatKey, usize>,
    tex_index: HashMap<(usize, [u8; 16]), usize>,
    scene: Scene,
}

fn layer_color(m: &Model, name: &str) -> [u8; 3] {
    m.layer_by_name(name).map(|l| l.rgb).unwrap_or([136, 136, 136])
}

fn is_generic_name(n: &str) -> bool {
    // ^(?:Group|Component)\d*#\d+$
    let rest = n.strip_prefix("Group").or_else(|| n.strip_prefix("Component"));
    let Some(rest) = rest else { return false };
    let Some((a, b)) = rest.split_once('#') else { return false };
    a.chars().all(|c| c.is_ascii_digit()) && !b.is_empty() && b.chars().all(|c| c.is_ascii_digit())
}

impl<'m> Baker<'m> {
    fn def(&self, id: u64) -> Option<&'m Definition> {
        self.m.definitions.get(id)
    }

    fn texture_index(&mut self, mat: Option<&Material>) -> Option<usize> {
        let t = mat?.texture.as_ref()?;
        let data = t.data.as_ref()?;
        if data.is_empty() {
            return None;
        }
        let mime = sniff_mime(data)?;
        let mut head = [0u8; 16];
        let n = data.len().min(16);
        head[..n].copy_from_slice(&data[..n]);
        let key = (data.len(), head);
        if let Some(&i) = self.tex_index.get(&key) {
            return Some(i);
        }
        let i = self.scene.textures.len();
        self.scene.textures.push(SceneTexture { data: data.clone(), mime, filename: t.filename.clone() });
        self.tex_index.insert(key, i);
        Some(i)
    }

    fn scene_material(&mut self, rgb: [u8; 3], double_sided: bool, tex: Option<usize>, opacity: f32) -> usize {
        let key: MatKey = (rgb, double_sided, tex, opacity.to_bits());
        if let Some(&i) = self.mat_index.get(&key) {
            return i;
        }
        let i = self.scene.materials.len();
        self.scene.materials.push(SceneMaterial { rgb, opacity, double_sided, texture: tex });
        self.mat_index.insert(key, i);
        i
    }

    fn cached(&mut self, id: u64, d: &Definition) -> R<()> {
        if self.cache.contains_key(&id) {
            return Ok(());
        }
        let mut out = Vec::with_capacity(d.faces.len());
        for (_, f) in d.faces.iter() {
            let mut loops: Vec<Vec<u64>> = vec![];
            for lp in &f.loops {
                let mut vs: Vec<u64> = vec![];
                for &(eid, orient) in lp {
                    if let Some(e) = d.edges.get(eid) {
                        let start = if orient == 1 { e.v1 } else { e.v2 };
                        if vs.last() != Some(&start) {
                            vs.push(start);
                        }
                    }
                }
                if vs.len() > 1 && vs[0] == vs[vs.len() - 1] {
                    vs.pop();
                }
                if !vs.is_empty() {
                    loops.push(vs);
                }
            }
            let tris = if loops.is_empty() { vec![] } else { triangulate_loops(&|v| d.vertices.get(v).copied(), &loops, f.normal, self.lim.max_face_verts) };
            if !loops.is_empty() && tris.is_empty() {
                self.scene.stats.untriangulated_faces += 1;
            }
            out.push(CachedFace { loops, tris, normal: f.normal, front: f.material_id, back: f.back_material_id, uv_front: f.uv_front, uv_back: f.uv_back, layer: f.layer, hidden: f.hidden });
        }
        self.cache.insert(id, out);
        Ok(())
    }

    fn layer_hidden(&self, name: &str) -> bool {
        self.m.layer_by_name(name).map_or(false, |l| l.hidden)
    }

    #[allow(clippy::too_many_arguments)]
    fn instantiate(&mut self, key: Option<u64>, d: &'m Definition, matrix: &[f64; 13], parent_layer: &str, path: &str, inherited: Option<[u8; 3]>, depth: usize) -> R<()> {
        let mut parent_layer = parent_layer.to_string();
        if !d.faces.is_empty() {
            let ck = key.unwrap_or(NO_ID - 1);
            self.cached(ck, d)?;
            let cache = self.cache.remove(&ck).unwrap_or_default();
            let r = self.emit_faces(d, &cache, matrix, &mut parent_layer, path, inherited);
            self.cache.insert(ck, cache);
            r?;
        }
        for inst in &d.instances {
            self.scene.stats.instances += 1;
            if self.scene.stats.instances > self.opt.max_instances {
                return Err(SkpError::new("limit.instances", format!("more than {} instances placed", self.opt.max_instances)));
            }
            let Some(rid) = inst.def else {
                self.scene.stats.skipped_missing_def += 1;
                continue;
            };
            let Some(cd) = self.def(rid) else {
                self.scene.stats.skipped_missing_def += 1;
                continue;
            };
            let mut l_name = parent_layer.clone();
            let mut inst_color = inherited;
            let apply_overrides = !(self.opt.openskp_compat && self.m.container == Container::Legacy);
            if let Some(l) = inst.layer.filter(|_| apply_overrides) {
                if let Some(n) = self.m.layer_id_to_name.get(&l) {
                    l_name = n.clone();
                }
            }
            if let Some(mat) = self.m.resolve_material(inst.material_id).filter(|_| apply_overrides) {
                inst_color = Some([mat.rgba[0], mat.rgba[1], mat.rgba[2]]);
            }
            if !self.opt.include_hidden && (inst.hidden || self.layer_hidden(&l_name)) {
                self.scene.stats.skipped_hidden += 1;
                continue;
            }
            if depth + 1 > self.opt.max_depth {
                self.scene.stats.skipped_too_deep += 1;
                continue;
            }
            if self.active.contains(&rid) {
                self.scene.stats.skipped_recursive += 1;
                if self.scene.warnings.len() < 50 {
                    self.scene.warnings.push(format!("recursive component definition {rid} skipped at {path}"));
                }
                continue;
            }
            let new_matrix = match &inst.matrix {
                Some(c) => multiply(matrix, c),
                None => *matrix,
            };
            let inst_name = if !inst.name.is_empty() {
                inst.name.clone()
            } else if !cd.name.is_empty() && !is_generic_name(&cd.name) {
                cd.name.clone()
            } else {
                format!("Component_{rid}")
            };
            let full = if path.len() < 400 { format!("{path} / {inst_name}") } else { path.to_string() };
            self.active.insert(rid);
            let r = self.instantiate(Some(rid), cd, &new_matrix, &l_name, &full, inst_color, depth + 1);
            self.active.remove(&rid);
            r?;
        }
        Ok(())
    }

    fn emit_faces(&mut self, d: &Definition, faces: &[CachedFace], matrix: &[f64; 13], parent_layer: &mut String, path: &str, inherited: Option<[u8; 3]>) -> R<()> {
        // a definition's faces carry their own layer: the most used one names the owner
        let mut counts: Vec<(u64, usize)> = vec![];
        for f in faces {
            if let Some(l) = f.layer.filter(|&l| l != 0) {
                match counts.iter_mut().find(|c| c.0 == l) {
                    Some(c) => c.1 += 1,
                    None => counts.push((l, 1)),
                }
            }
        }
        let best = counts.iter().map(|c| c.1).max().unwrap_or(0);
        if let Some(first) = counts.iter().find(|c| c.1 == best).map(|c| c.0) {
            if let Some(n) = self.m.layer_id_to_name.get(&first) {
                *parent_layer = n.clone();
            }
        }
        let fallback = inherited.unwrap_or_else(|| layer_color(self.m, parent_layer));
        let flip = det3(matrix) < 0.0;
        let model: &'m Model = self.m;
        let mut groups: HashMap<MatKey, usize> = HashMap::new();
        let mut order: Vec<Group> = vec![];
        for f in faces {
            if f.tris.is_empty() {
                continue;
            }
            if !self.opt.include_hidden {
                let lh = f.layer.and_then(|l| self.m.layer_id_to_name.get(&l)).map_or(false, |n| self.layer_hidden(n));
                if f.hidden || lh {
                    self.scene.stats.skipped_hidden += 1;
                    continue;
                }
            }
            let front_mat = model.resolve_material(f.front);
            let back_mat = model.resolve_material(f.back);
            let front_color = front_mat.map(|m| [m.rgba[0], m.rgba[1], m.rgba[2]]).unwrap_or(fallback);
            let back_color = back_mat.map(|m| [m.rgba[0], m.rgba[1], m.rgba[2]]).unwrap_or(fallback);
            self.scene.stats.faces += 1;
            let sides: Vec<(bool, [u8; 3], Option<&Material>, bool)> = if front_color == back_color { vec![(true, front_color, front_mat, false)] } else { vec![(false, front_color, front_mat, false), (false, back_color, back_mat, true)] };
            let mut front_scene_mat = None;
            for (double_sided, color, mat, reverse) in sides {
                let tex = self.texture_index(mat);
                let opacity = mat.map_or(1.0, |m| (m.rgba[3] as f64 / 255.0 * m.opacity) as f32);
                let mi = self.scene_material(color, double_sided, tex, opacity);
                if !reverse {
                    front_scene_mat = Some(mi);
                }
                let key: MatKey = (color, double_sided, tex, opacity.to_bits());
                let gi = *groups.entry(key).or_insert_with(|| {
                    order.push(Group { mat: mi, ..Default::default() });
                    order.len() - 1
                });
                let g = &mut order[gi];
                let nrm = f.normal;
                let side_n = if reverse { [-nrm[0], -nrm[1], -nrm[2]] } else { nrm };
                let (xr, yr) = face_uv_basis(nrm);
                let (tw, th) = match mat.and_then(|m| m.texture.as_ref()) {
                    Some(t) => (if t.x_scale > 1e-9 { t.x_scale } else { 1.0 }, if t.y_scale > 1e-9 { t.y_scale } else { 1.0 }),
                    None => (1.0, 1.0),
                };
                let uvt = if reverse { f.uv_back.as_ref() } else { f.uv_front.as_ref() };
                let mut face_map: HashMap<u64, u32> = HashMap::new();
                for tri in &f.tris {
                    let mut ids = *tri;
                    // winding agrees with the face normal on the side that is emitted
                    if let (Some(a), Some(b), Some(c)) = (d.vertices.get(ids[0]), d.vertices.get(ids[1]), d.vertices.get(ids[2])) {
                        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                        let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                        let cr = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
                        if cr[0] * nrm[0] + cr[1] * nrm[1] + cr[2] * nrm[2] < 0.0 {
                            ids.swap(1, 2);
                        }
                    }
                    if reverse != flip {
                        ids.swap(1, 2);
                    }
                    let mut idx = [0u32; 3];
                    let mut ok = true;
                    for (k, &vid) in ids.iter().enumerate() {
                        let Some(p) = d.vertices.get(vid).copied() else {
                            ok = false;
                            continue;
                        };
                        let gi2 = match face_map.get(&vid) {
                            Some(&i) => i,
                            None => {
                                let (u, v) = face_uv(p, xr, yr, uvt, tw, th);
                                let vk = (vid, u.to_bits(), v.to_bits());
                                let i = match g.vmap.get(&vk) {
                                    Some(&i) => {
                                        let a = &mut g.normals_acc[i as usize];
                                        a[0] += side_n[0];
                                        a[1] += side_n[1];
                                        a[2] += side_n[2];
                                        i
                                    }
                                    None => {
                                        let w = transform_point(matrix, p);
                                        g.prim.positions.push([w[0] * INCHES_TO_MM, w[1] * INCHES_TO_MM, w[2] * INCHES_TO_MM]);
                                        g.prim.uvs.push([u as f32, v as f32]);
                                        g.normals_acc.push(side_n);
                                        let i = (g.prim.positions.len() - 1) as u32;
                                        g.vmap.insert(vk, i);
                                        i
                                    }
                                };
                                face_map.insert(vid, i);
                                i
                            }
                        };
                        idx[k] = gi2;
                    }
                    if ok && idx[0] != idx[1] && idx[1] != idx[2] && idx[0] != idx[2] {
                        g.prim.indices.extend_from_slice(&idx);
                        self.scene.stats.triangles += 1;
                        if self.scene.stats.triangles > self.opt.max_triangles {
                            return Err(SkpError::new("limit.triangles", format!("more than {} triangles baked", self.opt.max_triangles)));
                        }
                    }
                }
            }
            if self.opt.keep_polygons {
                let mut loops: Vec<Vec<[f64; 3]>> = vec![];
                let mut okp = true;
                for lp in &f.loops {
                    let mut pts = Vec::with_capacity(lp.len());
                    for &vid in lp {
                        match d.vertices.get(vid) {
                            Some(&p) => {
                                let w = transform_point(matrix, p);
                                pts.push([w[0] * INCHES_TO_MM, w[1] * INCHES_TO_MM, w[2] * INCHES_TO_MM]);
                            }
                            None => okp = false,
                        }
                    }
                    loops.push(pts);
                }
                if okp && !loops.is_empty() {
                    let mut n = transform_normal(matrix, f.normal);
                    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                    if l > 1e-12 {
                        n = [n[0] / l, n[1] / l, n[2] / l];
                    }
                    self.scene.polygons.push(PolyFace { loops, normal: n, material: front_scene_mat.unwrap_or(0), layer: parent_layer.clone(), path: path.to_string() });
                }
            }
        }
        for g in order {
            let mut prim = g.prim;
            if prim.indices.is_empty() {
                continue;
            }
            prim.normals = g
                .normals_acc
                .iter()
                .map(|n| {
                    let r = rotate_vec(matrix, *n);
                    let l = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
                    if l > 1e-6 {
                        [(r[0] / l) as f32, (r[1] / l) as f32, (r[2] / l) as f32]
                    } else {
                        [0.0, 0.0, 1.0]
                    }
                })
                .collect();
            prim.material = g.mat;
            prim.path = path.to_string();
            prim.layer = parent_layer.clone();
            prim.definition = d.name.clone();
            self.scene.primitives.push(prim);
        }
        Ok(())
    }
}

/// Bake the model into a world-space scene.
pub fn bake(m: &Model, lim: &Limits, opt: &BakeOptions) -> R<Scene> {
    let mut b = Baker { m, lim, opt, cache: HashMap::new(), active: HashSet::new(), mat_index: HashMap::new(), tex_index: HashMap::new(), scene: Scene::default() };
    let root: &Definition = &m.root;
    b.instantiate(None, root, &IDENT, "Layer0", "ROOT", None, 0)?;
    let mut scene = b.scene;
    if opt.up != Up::Z {
        let up = opt.up;
        for p in &mut scene.primitives {
            for v in &mut p.positions {
                *v = up.apply(*v);
            }
            for n in &mut p.normals {
                let r = up.apply([n[0] as f64, n[1] as f64, n[2] as f64]);
                *n = [r[0] as f32, r[1] as f32, r[2] as f32];
            }
        }
        for pf in &mut scene.polygons {
            for l in &mut pf.loops {
                for v in l.iter_mut() {
                    *v = up.apply(*v);
                }
            }
            pf.normal = up.apply(pf.normal);
        }
    }
    Ok(scene)
}
