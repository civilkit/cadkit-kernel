//! Incremental regen: a bounded cache of the complete regen state after each
//! feature, keyed by a hash of the document prefix. See docs/regen-cache.md.
//!
//! Correctness rule: an entry is only ever found through a key that covers
//! everything the state after feature `i` depends on (document-level
//! settings, the full JSON of features `0..=i`, regen options that change
//! results, kernel version). A hit restores that state and the regen
//! continues with the first feature that differs, so the result is
//! byte-identical to a cold regen.

use ck_json::Json;
use ck_model::features::BrepSnap;
use ck_model::regen::{Checkpoint, FeatureExec, RegenResult, RegenState};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

/// Bump when the meaning of a cached state changes without the version.
const FORMAT: u32 = 1;

/// Default cap of estimated cached state, bytes. The large sample part (50
/// features, ~2.8 MB estimated per distinct body state) needs ~60 MB for a
/// full chain; 256 MB keeps a few documents' worth of history while the
/// wasm linear memory stays far below the 4 GB limit (docs/regen-cache.md).
pub const DEFAULT_CAP: usize = 256 << 20;

pub type Key = [u8; 32];

use ck_text::sha256;

// -------------------------------------------------------------------- keys

/// What decides whether a cached state may be reused, besides the features.
pub struct KeyOpts {
    pub sketch_status: bool,
    pub solve_sketches: bool,
    pub auto_heal: Option<f64>,
}

/// Fingerprints (`name=<sha256 | missing>[^pin]`) of the fonts used by `text` entities anywhere in `j`.
fn text_fonts(j: &Json, out: &mut Vec<String>) {
    match j {
        Json::Obj(m) => {
            if j.get_str("type") == Some("text") {
                let name = j.get_str("font").unwrap_or("gauge");
                out.push(ck_text::font_fingerprint(name, j.get_str("fontHash")));
            }
            for (_, v) in m {
                text_fonts(v, out);
            }
        }
        Json::Arr(a) => a.iter().for_each(|v| text_fonts(v, out)),
        _ => {}
    }
}

/// Feature keys that no regen code path reads: they only label a feature in the tree (docs/regen-cache.md,
/// "Cosmetic fields"). They are left out of the hash of a feature, so editing one (grouping into a folder, a rename,
/// hiding a plane) keeps every key and the cached states valid. The only place they reach a reply is the echo of a
/// sketch / sketch3d feature, which [`refresh_cosmetic`] rewrites from the current document on every hit.
pub const COSMETIC_KEYS: [&str; 5] = ["group", "name", "autoName", "visible", "userHidden"];

fn is_cosmetic(k: &str) -> bool {
    COSMETIC_KEYS.contains(&k)
}

/// The feature as hashed: the object without its cosmetic keys (the same bytes as before when it has none).
fn hashed_form(f: &Json) -> String {
    match f {
        Json::Obj(m) if m.iter().any(|(k, _)| is_cosmetic(k)) => {
            ck_json::to_string(&Json::Obj(m.iter().filter(|(k, _)| !is_cosmetic(k)).cloned().collect()))
        }
        _ => ck_json::to_string(f),
    }
}

/// `stored` is a cached echo of a feature (its object as received, normalised by appending missing arrays); `current`
/// is the same feature in the document now, equal in every non-cosmetic key. The echo of the current document:
/// the keys of `current` in its order (cosmetic ones from `current`, the rest from `stored`), then whatever the
/// normalisation appended. `None` when `stored` already is that.
fn refresh_echo(stored: &Json, current: &Json) -> Option<Json> {
    let (Json::Obj(sm), Json::Obj(cm)) = (stored, current) else { return None };
    let mut out: Vec<(String, Json)> = Vec::with_capacity(sm.len() + 1);
    for (k, v) in cm {
        if is_cosmetic(k) {
            out.push((k.clone(), v.clone()));
        } else if let Some(sv) = stored.get(k) {
            out.push((k.clone(), sv.clone()));
        }
    }
    for (k, v) in sm {
        if !is_cosmetic(k) && !cm.iter().any(|(ck, _)| ck == k) {
            out.push((k.clone(), v.clone()));
        }
    }
    let same = out.len() == sm.len() && out.iter().zip(sm).all(|(a, b)| a.0 == b.0 && a.1 == b.1);
    if same {
        None
    } else {
        Some(Json::Obj(out))
    }
}

/// A restored state echoes the cosmetic keys of the document that built it (inside the `sketch` of every sketch and
/// sketch3d entry). Make the echoes those of `feats`, so a hit replies exactly as a cold run of this document.
pub fn refresh_cosmetic(r: &mut RegenResult, feats: &[ck_model::Feature]) {
    let cur = |id: &str| feats.iter().rev().find(|f| f.id == id).map(|f| &f.raw);
    for s in r.sketches.iter_mut() {
        if let Some(c) = cur(&s.id) {
            if let Some(j) = refresh_echo(&s.sketch.raw, c) {
                s.sketch.set_cosmetic_raw(j);
            }
        }
    }
    for (id, j) in r.sketches3.iter_mut() {
        if let Some(n) = cur(id).and_then(|c| refresh_echo(j, c)) {
            *j = n;
        }
    }
}

/// `keys[i]` = key of the state after `features[..i]` (`keys[0]` is the
/// document key; `keys.len() == n + 1`). Each key chains the previous one
/// with the full JSON of one more feature, so equal keys mean equal
/// documents up to that feature (SHA-256, collisions are not a concern).
///
/// Document level: only `format`, `version`, `kind`, `material` and
/// `variables` (the part fields the engine reads besides `features`);
/// `name`, `seq`, `counters`, `rollback` and unknown keys do not affect
/// geometry and are deliberately left out so renames and feature-counter
/// bumps keep the cache warm.
pub fn chain_keys(part: &Json, o: &KeyOpts, n: usize) -> Vec<Key> {
    let doc: Vec<(String, Json)> = ["format", "version", "kind", "material", "variables"]
        .iter()
        .map(|k| (k.to_string(), part.get(k).cloned().unwrap_or(Json::Null)))
        .collect();
    let heal = o.auto_heal.map_or(String::from("-"), |t| format!("{:016x}", t.to_bits()));
    let head = format!(
        "ck-regen-cache/{FORMAT}/{}/status={}/solve={}/heal={}/{}",
        env!("CARGO_PKG_VERSION"),
        o.sketch_status,
        o.solve_sketches,
        heal,
        ck_json::to_string(&Json::Obj(doc))
    );
    let mut keys = Vec::with_capacity(n + 1);
    keys.push(sha256(&[head.as_bytes()]));
    let feats = part.get("features").and_then(|f| f.as_arr()).unwrap_or(&[]);
    for f in feats.iter().take(n) {
        let mut s = hashed_form(f);
        // lettering: the content hash of every font the feature names (a loaded font can change under its name)
        let mut fonts = vec![];
        text_fonts(f, &mut fonts);
        if !fonts.is_empty() {
            fonts.sort();
            fonts.dedup();
            s.push_str("\u{0}fonts:");
            s.push_str(&fonts.join(","));
        }
        let prev = *keys.last().unwrap();
        keys.push(sha256(&[&prev, s.as_bytes()]));
    }
    keys
}

// ------------------------------------------------------------------- cache

pub struct Entry {
    result: Rc<RegenResult>,
    axes: Vec<(String, (ck_core::linalg::Vec3, ck_core::linalg::Vec3))>,
    snap: BrepSnap,
    last_used: u64,
    /// Shared pieces this entry holds (pointer, bytes) and its own bytes.
    pieces: Vec<(usize, usize)>,
}

#[derive(Default, Clone, Copy)]
pub struct Stats {
    pub lookups: u64,
    pub hits: u64,
    pub stored: u64,
    pub evicted: u64,
}

pub struct RegenCache {
    entries: HashMap<Key, Entry>,
    /// piece pointer -> (bytes, number of entries holding it)
    pieces: HashMap<usize, (usize, u32)>,
    bytes: usize,
    cap: usize,
    tick: u64,
    pub stats: Stats,
}

impl Default for RegenCache {
    fn default() -> Self {
        RegenCache { entries: HashMap::new(), pieces: HashMap::new(), bytes: 0, cap: DEFAULT_CAP, tick: 0, stats: Stats::default() }
    }
}

/// What a lookup hands back: the state after `next` features.
pub struct Hit {
    pub next: usize,
    pub result: RegenResult,
    pub axes: Vec<(String, (ck_core::linalg::Vec3, ck_core::linalg::Vec3))>,
    pub snap: BrepSnap,
}

fn ptr<T: ?Sized>(r: &Rc<T>) -> usize {
    Rc::as_ptr(r) as *const () as usize
}

fn json_bytes(j: &Json) -> usize {
    match j {
        Json::Arr(a) => 24 + a.iter().map(json_bytes).sum::<usize>(),
        Json::Obj(m) => 24 + m.iter().map(|(k, v)| 24 + k.len() + json_bytes(v)).sum::<usize>(),
        Json::Str(s) => 32 + s.len(),
        _ => 16,
    }
}

fn result_bytes(r: &RegenResult) -> usize {
    use std::mem::size_of_val;
    let sk: usize = r
        .sketches
        .iter()
        .map(|s| {
            s.sketch.points.len() * 64
                + s.sketch.entities.len() * 128
                + s.sketch.constraints.len() * 160
                + json_bytes(&s.sketch.raw)
                + s.profiles.regions.len() * 512
                + 512
        })
        .sum();
    sk + r.errors.len() * 200
        + r.planes.len() * 160
        + r.components.iter().chain(&r.markers).chain(&r.curves).chain(&r.surfaces).map(json_bytes).sum::<usize>()
        + r.sketches3.iter().map(|(_, j)| 64 + json_bytes(j)).sum::<usize>()
        + r.body.as_ref().map_or(0, json_bytes)
        + size_of_val(r)
}

fn snap_pieces(s: &BrepSnap) -> Vec<(usize, usize)> {
    let mut v = Vec::with_capacity(4 + s.tools.len());
    v.push((ptr(&s.body), std::mem::size_of::<Option<ck_topo::body::Body>>() + s.body.as_ref().as_ref().map_or(0, |b| b.est_bytes())));
    v.push((ptr(&s.asm), 256 + s.asm.cache.iter().map(|(k, n)| k.len() + n.body.as_ref().map_or(0, |b| b.est_bytes())).sum::<usize>() + s.asm.components.len() * 256));
    v.push((ptr(&s.helices), 64 + s.helices.len() * 128));
    v.push((ptr(&s.surfs), 64 + s.surfs.iter().map(|(_, b)| b.est_bytes()).sum::<usize>()));
    for (_, t) in &s.tools {
        v.push((ptr(t), 1024 + t.body.est_bytes()));
    }
    v
}

impl RegenCache {
    pub fn cap(&self) -> usize {
        self.cap
    }
    pub fn bytes(&self) -> usize {
        self.bytes
    }
    pub fn len(&self) -> usize {
        self.entries.len()
    }
    pub fn set_cap(&mut self, cap: usize) {
        self.cap = cap;
        self.evict_to_cap(None);
    }
    pub fn contains(&self, k: &Key) -> bool {
        self.entries.contains_key(k)
    }
    pub fn clear(&mut self) {
        self.entries.clear();
        self.pieces.clear();
        self.bytes = 0;
    }

    /// Longest cached prefix `i` (1..keys.len()), restored as a [`Hit`].
    pub fn lookup(&mut self, keys: &[Key]) -> Option<Hit> {
        self.stats.lookups += 1;
        self.tick += 1;
        let tick = self.tick;
        for i in (1..keys.len()).rev() {
            if let Some(e) = self.entries.get_mut(&keys[i]) {
                e.last_used = tick;
                self.stats.hits += 1;
                return Some(Hit { next: i, result: (*e.result).clone(), axes: e.axes.clone(), snap: e.snap.clone() });
            }
        }
        None
    }

    /// The shared state of an entry (used to chain snapshots), touching it.
    fn touch(&mut self, k: &Key) -> Option<(Rc<RegenResult>, BrepSnap)> {
        self.tick += 1;
        let t = self.tick;
        self.entries.get_mut(k).map(|e| {
            e.last_used = t;
            (e.result.clone(), e.snap.clone())
        })
    }

    fn insert(&mut self, key: Key, result: Rc<RegenResult>, axes: Vec<(String, (ck_core::linalg::Vec3, ck_core::linalg::Vec3))>, snap: BrepSnap) {
        if self.entries.contains_key(&key) {
            return;
        }
        self.tick += 1;
        let mut pieces = snap_pieces(&snap);
        pieces.push((ptr(&result), result_bytes(&result)));
        for &(p, b) in &pieces {
            let slot = self.pieces.entry(p).or_insert((b, 0));
            if slot.1 == 0 {
                self.bytes += b;
            }
            slot.1 += 1;
        }
        self.entries.insert(key, Entry { result, axes, snap, last_used: self.tick, pieces });
        self.stats.stored += 1;
        self.evict_to_cap(Some(key));
    }

    /// Drop least-recently-used entries until under the cap (never `keep`).
    fn evict_to_cap(&mut self, keep: Option<Key>) {
        while self.bytes > self.cap {
            let victim = self.entries.iter().filter(|(k, _)| Some(**k) != keep).min_by_key(|(_, e)| e.last_used).map(|(k, _)| *k);
            let Some(v) = victim else {
                // only `keep` is left and it alone exceeds the cap: drop it too
                if let Some(k) = keep {
                    self.remove(&k);
                }
                break;
            };
            self.remove(&v);
        }
    }

    fn remove(&mut self, k: &Key) {
        if let Some(e) = self.entries.remove(k) {
            self.stats.evicted += 1;
            for (p, _) in e.pieces {
                if let Some(slot) = self.pieces.get_mut(&p) {
                    slot.1 -= 1;
                    if slot.1 == 0 {
                        self.bytes -= slot.0;
                        self.pieces.remove(&p);
                    }
                }
            }
        }
    }
}

thread_local! {
    static CACHE: RefCell<RegenCache> = RefCell::new(RegenCache::default());
}

pub fn with<T>(f: impl FnOnce(&mut RegenCache) -> T) -> T {
    CACHE.with(|c| f(&mut c.borrow_mut()))
}

/// Stores the state after every feature of a run under the prefix keys.
pub struct Recorder {
    keys: Vec<Key>,
    prev: Option<(Rc<RegenResult>, BrepSnap)>,
}

impl Recorder {
    /// `resumed`: the hit the run continues from, if any (its state chains the
    /// first new snapshot).
    pub fn new(keys: Vec<Key>, resumed: Option<(RegenResult, BrepSnap)>) -> Recorder {
        Recorder { keys, prev: resumed.map(|(r, s)| (Rc::new(r), s)) }
    }
}

impl Checkpoint for Recorder {
    fn after(&mut self, i: usize, ran: bool, state: &RegenState, exec: &mut dyn FeatureExec) {
        let key = self.keys[i + 1];
        // an identical prefix is already stored: just continue from it
        if let Some(p) = with(|c| c.touch(&key)) {
            self.prev = Some(p);
            return;
        }
        let (result, snap) = if !ran && self.prev.is_some() {
            // suppressed feature: nothing changes
            self.prev.clone().unwrap()
        } else {
            let Some(snap) = exec.save(self.prev.as_ref().map(|p| &p.1)) else { return };
            (Rc::new(state.result.clone()), snap)
        };
        with(|c| c.insert(key, result.clone(), state.axes.clone(), snap.clone()));
        self.prev = Some((result, snap));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(k: &Key) -> String {
        k.iter().map(|b| format!("{b:02x}")).collect()
    }

    #[test]
    fn sha256_vectors() {
        assert_eq!(hex(&sha256(&[b""])), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
        assert_eq!(hex(&sha256(&[b"abc"])), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
        // multi-block, split across parts
        let long = vec![b'a'; 1000];
        assert_eq!(sha256(&[&long]), sha256(&[&long[..37], &long[37..500], &long[500..]]));
        assert_eq!(hex(&sha256(&[b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"])), "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    }
}
