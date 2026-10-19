//! Content-keyed memos of pure geometric functions (edge boxes, pcurves, surface sample grids).
//!
//! A boolean chain (a pattern of ten unions, a fillet followed by cuts) hands most faces and edges of one
//! result unchanged to the next operation, which used to recompute everything about them: the box of every
//! edge, the pcurve of every coedge, the sample grid of every curved face. Keyed by the exact bits of the
//! inputs (`Curve::push_key`, `Surface::push_key` in ck-geom), a repeat costs one
//! lookup and returns the very value the computation would produce, so results stay bit-identical.

use std::cell::RefCell;
use std::hash::{Hash, Hasher};
use std::thread::LocalKey;

/// Exact key: a word list (a newtype so that the hasher sees whole words, not bytes).
#[derive(PartialEq, Eq)]
pub struct Key(pub Box<[u64]>);

impl Hash for Key {
    fn hash<H: Hasher>(&self, h: &mut H) {
        h.write_usize(self.0.len());
        for &w in self.0.iter() {
            h.write_u64(w);
        }
    }
}

/// One memo table; cleared whole when its keys pass `cap_words` words (a bound on memory, not a policy).
pub struct Memo<V> {
    map: crate::fxhash::FxHashMap<Key, V>,
    words: usize,
    cap_words: usize,
}

impl<V> Memo<V> {
    pub fn new(cap_words: usize) -> Memo<V> {
        Memo { map: Default::default(), words: 0, cap_words }
    }
}

thread_local! {
    static COUNTS: std::cell::Cell<[u64; 2]> = const { std::cell::Cell::new([0; 2]) };
}

/// `[hits, misses]` of all memo tables on this thread (for work-counter tests and profiling).
pub fn counts() -> [u64; 2] {
    COUNTS.with(|c| c.get())
}

/// `f()` memoised in `table` under `key`. The computation runs outside the table's borrow (it may use other
/// memos, or this one).
pub fn cached<V: Clone + 'static>(table: &'static LocalKey<RefCell<Memo<V>>>, key: Vec<u64>, f: impl FnOnce() -> V) -> V {
    let key = Key(key.into_boxed_slice());
    if let Some(v) = table.with(|t| t.borrow().map.get(&key).cloned()) {
        COUNTS.with(|c| c.set([c.get()[0] + 1, c.get()[1]]));
        return v;
    }
    COUNTS.with(|c| c.set([c.get()[0], c.get()[1] + 1]));
    let v = f();
    table.with(|t| {
        let mut t = t.borrow_mut();
        if t.words + key.0.len() > t.cap_words {
            t.map.clear();
            t.words = 0;
        }
        t.words += key.0.len();
        t.map.insert(key, v.clone());
    });
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    thread_local! {
        static T: RefCell<Memo<u32>> = RefCell::new(Memo::new(8));
    }

    #[test]
    fn hits_misses_and_cap() {
        let mut calls = 0;
        let mut get = |k: Vec<u64>| cached(&T, k, || {
            calls += 1;
            7
        });
        assert_eq!(get(vec![1, 2, 3]), 7);
        assert_eq!(get(vec![1, 2, 3]), 7);
        assert_eq!(get(vec![1, 2]), 7);
        // over the cap: the table is cleared, the next lookup recomputes
        assert_eq!(get(vec![4, 5, 6, 7]), 7);
        assert_eq!(get(vec![1, 2, 3]), 7);
        assert_eq!(calls, 4);
    }
}
