// SPDX-License-Identifier: MIT
//! The parts of a timeline item in other components (mitcad#169).
//!
//! A feature that changes the bodies of several components (a cut through
//! a part and the part under it) is one timeline item, owned by one
//! component, and one feature more in the feature list
//! ([`FEATURE_LIST`]) of each other component it changes: a feature of the
//! item's class, with its own inputs (the same profile, the bodies of its
//! component), its own item tail and its own operation in that
//! component's history, which no timeline item names. The design's table
//! [`ITEM_PARTS`] lists each such item with its features. The decoder
//! gives every part with a result of its own as an item of its own, right
//! after the item, so that the import replays it in its component
//! (`_f3d.part_of` names the item).

use std::collections::HashMap;

use super::super::classes::*;
use super::super::decode::{self, TimelineEntry};
use super::super::stream::{Segment, str16_at, u32_at};

/// How far into an [`ITEM_PARTS`] object its first list may start.
const MAX_START: usize = 64;
/// The most entries read per list.
const MAX_ENTRIES: u32 = 100_000;

/// Each item's features in [`ITEM_PARTS`] other than the item itself, by
/// the item's object id.
pub fn item_parts(seg: &Segment) -> HashMap<u64, Vec<u64>> {
    let mut out: HashMap<u64, Vec<u64>> = HashMap::new();
    for o in seg.objects_of(ITEM_PARTS) {
        let d = seg.data(o);
        let Some(lists) = (0..MAX_START.min(d.len())).find_map(|p| parse(seg, d, p)) else {
            continue;
        };
        for (item, features) in lists {
            let parts = out.entry(item).or_default();
            parts.extend(features.into_iter().filter(|&f| f != item));
        }
    }
    out
}

/// The two lists of an [`ITEM_PARTS`] object starting at `p`, joined by
/// their keys: (item, its features). `None` unless both lists read.
fn parse(seg: &Segment, d: &[u8], p: usize) -> Option<Vec<(u64, Vec<u64>)>> {
    let n = u32_at(d, p).filter(|&n| n > 0 && n <= MAX_ENTRIES)?;
    let mut q = p + 4;
    let mut items: HashMap<String, u64> = HashMap::new();
    for _ in 0..n {
        let (key, k) = str16_at(d, q).filter(|(k, _)| !k.is_empty())?;
        let r = seg.ref_at(d, k)?;
        items.insert(key, r.id);
        q = r.end;
    }
    let m = u32_at(d, q).filter(|&m| m <= MAX_ENTRIES)?;
    q += 4;
    let mut out = Vec::new();
    for _ in 0..m {
        let (key, k) = str16_at(d, q)?;
        let (features, end) = seg.ref_list_at(d, k)?;
        q = end;
        if let Some(&item) = items.get(&key) {
            out.push((item, features));
        }
    }
    (q <= d.len()).then_some(out)
}

/// The parts of the timeline item `it` given as items of their own:
/// features of the item's class in another component's feature list
/// (`owners`, by object id) that are not timeline items themselves
/// (`in_timeline`) and made an operation of their own (a result number).
/// Each comes as a timeline entry at the item's position, named after it.
pub(super) fn part_entries(
    seg: &Segment,
    it: &TimelineEntry,
    parts: &HashMap<u64, Vec<u64>>,
    in_timeline: &HashMap<u64, usize>,
    owners: &HashMap<u64, u64>,
) -> Vec<TimelineEntry> {
    let Some(features) = parts.get(&it.id) else {
        return Vec::new();
    };
    let owner = owners.get(&it.id);
    features
        .iter()
        .filter_map(|&f| {
            if in_timeline.contains_key(&f) {
                return None;
            }
            let o = seg.object(f)?;
            if seg.guid(o) != Some(it.class.as_str()) {
                return None;
            }
            let own = owners.get(&f)?;
            if Some(own) == owner {
                return None;
            }
            let tail = decode::feature_tail(seg, o).filter(|t| t.result_no > 0)?;
            Some(TimelineEntry {
                pos: it.pos,
                id: f,
                class: it.class.clone(),
                class_version: seg.version(o).unwrap_or(0),
                type_name: it.type_name,
                name: it.name.clone(),
                extrude: if it.class == EXTRUDE {
                    decode::extrude_fields(seg, o)
                } else {
                    None
                },
                tail: Some(tail),
            })
        })
        .collect()
}
