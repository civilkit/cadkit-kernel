//! Cache of the regen reply's view snapshot (the `body` and `surfaces` JSON with their display meshes), keyed by the
//! regen cache's final chain key. A request for a document the kernel has shown before (the same document again, an
//! undo or redo back to an earlier state) replays the snapshot instead of re-tessellating.
//!
//! The key covers the whole document up to the last feature that ran, the document-level settings and the options that
//! change regen results (regen_cache::chain_keys), so a hit is byte-identical to a fresh snapshot. Buffers are not
//! stored as `$buf` descriptors (those hold offsets into one reply): the typed arrays are kept and attached to the new
//! reply's buffer area in the pinned order.

use crate::regen_cache::Key;
use ck_json::Json;
use ck_tess::mesh::Mesh;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

/// Default cap of cached snapshot bytes.
pub const DEFAULT_CAP: usize = 160 << 20;

/// One body of a snapshot: its wire JSON without the `silhouette` / `mesh` buffers, and the mesh arrays.
pub struct BodyView {
    pub json: Json,
    pub mesh: Mesh,
}

pub struct View {
    pub body: Option<BodyView>,
    pub surfaces: Vec<(String, BodyView)>,
}

fn json_bytes(j: &Json) -> usize {
    match j {
        Json::Arr(a) => 24 + a.iter().map(json_bytes).sum::<usize>(),
        Json::Obj(m) => 24 + m.iter().map(|(k, v)| 24 + k.len() + json_bytes(v)).sum::<usize>(),
        Json::Str(s) => 32 + s.len(),
        _ => 16,
    }
}

fn mesh_bytes(m: &Mesh) -> usize {
    4 * (m.positions.len() + m.normals.len() + m.mids.len() + m.tri_face.len() + m.silhouette.len())
}

impl BodyView {
    fn bytes(&self) -> usize {
        json_bytes(&self.json) + mesh_bytes(&self.mesh)
    }
}

impl View {
    fn bytes(&self) -> usize {
        256 + self.body.as_ref().map_or(0, |b| b.bytes()) + self.surfaces.iter().map(|(id, b)| id.len() + b.bytes()).sum::<usize>()
    }
}

#[derive(Default, Clone, Copy)]
pub struct Stats {
    pub lookups: u64,
    pub hits: u64,
    pub stored: u64,
    pub evicted: u64,
}

pub struct ViewCache {
    entries: HashMap<Key, (Rc<View>, u64, usize)>,
    bytes: usize,
    cap: usize,
    tick: u64,
    pub stats: Stats,
}

impl Default for ViewCache {
    fn default() -> Self {
        ViewCache { entries: HashMap::new(), bytes: 0, cap: DEFAULT_CAP, tick: 0, stats: Stats::default() }
    }
}

impl ViewCache {
    pub fn get(&mut self, k: &Key) -> Option<Rc<View>> {
        self.stats.lookups += 1;
        self.tick += 1;
        let t = self.tick;
        let e = self.entries.get_mut(k)?;
        e.1 = t;
        self.stats.hits += 1;
        Some(e.0.clone())
    }

    pub fn put(&mut self, k: Key, v: View) -> Rc<View> {
        let v = Rc::new(v);
        if self.entries.contains_key(&k) {
            return v;
        }
        self.tick += 1;
        let b = v.bytes();
        self.bytes += b;
        self.entries.insert(k, (v.clone(), self.tick, b));
        self.stats.stored += 1;
        while self.bytes > self.cap && self.entries.len() > 1 {
            let victim = self.entries.iter().filter(|(kk, _)| **kk != k).min_by_key(|(_, e)| e.1).map(|(kk, _)| *kk);
            let Some(victim) = victim else { break };
            if let Some(e) = self.entries.remove(&victim) {
                self.bytes -= e.2;
                self.stats.evicted += 1;
            }
        }
        v
    }

    pub fn set_cap(&mut self, cap: usize) {
        self.cap = cap;
    }
    pub fn len(&self) -> usize {
        self.entries.len()
    }
    pub fn bytes(&self) -> usize {
        self.bytes
    }
    pub fn clear(&mut self) {
        self.entries.clear();
        self.bytes = 0;
    }
}

thread_local! {
    static CACHE: RefCell<ViewCache> = RefCell::new(ViewCache::default());
}

pub fn with<T>(f: impl FnOnce(&mut ViewCache) -> T) -> T {
    CACHE.with(|c| f(&mut c.borrow_mut()))
}
