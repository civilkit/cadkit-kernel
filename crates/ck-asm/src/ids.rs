// SPDX-License-Identifier: MIT
//! Maps and sets keyed by record and entity indices, with a quick hash.
//!
//! Reading a large design looks up every entity it walks in the history
//! state's view and in sets of entities seen (mitcad#119: with the
//! standard library's keyed hash, hashing took about a third of the time
//! of finding the inputs and converting the bodies of a design of bodies
//! of 15 000 and 23 000 faces). The keys are indices into the file's own
//! records, so a keyed hash protects against nothing here. Nothing may
//! depend on the order of the entries (it differs from the standard
//! library's, which differs from run to run).

use std::collections::{HashMap, HashSet};
use std::hash::{BuildHasherDefault, Hasher};

/// A multiplicative hash of integers (the one Rust's own compiler uses for
/// its maps): one rotation, one exclusive or and one multiplication a word.
#[derive(Default, Clone, Copy)]
pub struct IdHasher(u64);

const SEED: u64 = 0x51_7c_c1_b7_27_22_0a_95;

impl IdHasher {
    fn add(&mut self, word: u64) {
        self.0 = (self.0.rotate_left(5) ^ word).wrapping_mul(SEED);
    }
}

impl Hasher for IdHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        let (words, rest) = bytes.as_chunks::<8>();
        for word in words {
            self.add(u64::from_le_bytes(*word));
        }
        if !rest.is_empty() {
            let mut word = [0u8; 8];
            word[..rest.len()].copy_from_slice(rest);
            self.add(u64::from_le_bytes(word));
        }
    }

    fn write_u8(&mut self, i: u8) {
        self.add(u64::from(i));
    }

    fn write_u32(&mut self, i: u32) {
        self.add(u64::from(i));
    }

    fn write_u64(&mut self, i: u64) {
        self.add(i);
    }

    fn write_usize(&mut self, i: usize) {
        self.add(i as u64);
    }
}

/// The hasher's builder.
pub type IdBuild = BuildHasherDefault<IdHasher>;

/// A map keyed by indices.
pub type IdMap<V> = HashMap<usize, V, IdBuild>;

/// A set of indices.
pub type IdSet = HashSet<usize, IdBuild>;

/// A history state's view of a blob: entity -> the record holding its data
/// at the state (`None`: the entity does not exist then); entities not in it
/// are as stored ([`crate::asm::history::History::view`]).
pub type View = IdMap<Option<usize>>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_and_sets_of_indices() {
        let mut map: IdMap<usize> = IdMap::default();
        let mut set = IdSet::default();
        for i in 0..10_000 {
            map.insert(i * 7, i);
            set.insert(i * 3);
        }
        assert_eq!(map.len(), 10_000);
        assert_eq!(map.get(&(7 * 1234)), Some(&1234));
        assert!(!map.contains_key(&5));
        assert!(set.contains(&9) && !set.contains(&10));
    }

    #[test]
    fn neighbouring_indices_spread() {
        // Consecutive keys differ in the low bits a table indexes by.
        let low = |i: usize| {
            let mut h = IdHasher::default();
            h.write_usize(i);
            h.finish() & 0xff
        };
        let distinct: HashSet<u64> = (0..64).map(low).collect();
        assert!(distinct.len() > 48);
    }
}
