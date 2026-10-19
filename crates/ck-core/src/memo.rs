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

/// One memo table in two generations: entries go into the current one; when its keys pass half of `cap_words`
/// words it becomes the old one (the previous old one is dropped), and a hit in the old generation moves the
/// entry back into the current one. Entries in use survive a turnover, so a long chain of operations does not
/// lose its working set all at once (a bound on memory, never on correctness).
pub struct Memo<V> {
    cur: crate::fxhash::FxHashMap<Key, V>,
    old: crate::fxhash::FxHashMap<Key, V>,
    words: usize,
    cap_words: usize,
}

impl<V> Memo<V> {
    pub fn new(cap_words: usize) -> Memo<V> {
        Memo { cur: Default::default(), old: Default::default(), words: 0, cap_words }
    }

    fn insert(&mut self, key: Key, v: V) {
        if self.words + key.0.len() > self.cap_words / 2 {
            self.old = std::mem::take(&mut self.cur);
            self.words = 0;
        }
        self.words += key.0.len();
        self.cur.insert(key, v);
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
    let hit = table.with(|t| {
        let mut t = t.borrow_mut();
        if let Some(v) = t.cur.get(&key) {
            return Some(v.clone());
        }
        let (k, v) = t.old.remove_entry(&key)?;
        t.insert(k, v.clone());
        Some(v)
    });
    if let Some(v) = hit {
        COUNTS.with(|c| c.set([c.get()[0] + 1, c.get()[1]]));
        return v;
    }
    COUNTS.with(|c| c.set([c.get()[0], c.get()[1] + 1]));
    let v = f();
    table.with(|t| t.borrow_mut().insert(key, v.clone()));
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    thread_local! {
        static T: RefCell<Memo<u32>> = RefCell::new(Memo::new(8));
    }

    #[test]
    fn hits_misses_and_generations() {
        let mut calls = 0;
        let mut get = |k: Vec<u64>| {
            cached(&T, k, || {
                calls += 1;
                7
            })
        };
        // cap 8 words: a generation holds 4
        assert_eq!(get(vec![1, 2, 3]), 7);
        assert_eq!(get(vec![1, 2, 3]), 7);
        // [9, 9] does not fit: the current generation becomes the old one
        assert_eq!(get(vec![9, 9]), 7);
        // a hit in the old generation (moved back: [9, 9] turns old)
        assert_eq!(get(vec![1, 2, 3]), 7);
        assert_eq!(get(vec![9, 9]), 7);
        // two turnovers later an entry not used meanwhile is gone
        assert_eq!(get(vec![5, 5, 5, 5]), 7);
        assert_eq!(get(vec![6, 6, 6, 6]), 7);
        assert_eq!(get(vec![1, 2, 3]), 7);
        assert_eq!(calls, 5);
    }
}
