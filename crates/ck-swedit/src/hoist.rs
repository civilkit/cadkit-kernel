//! Features a read-back step wants run earlier than the step itself (`"swHoistAfter": <SolidWorks feature id>`): the
//! filler of a round the kernel cannot run as a fillet is a revolve that must be added before the holes and cuts that
//! interrupt the round in the file, so it goes right after the step that made the second of its supports.

use crate::Step;
use ck_json::Json;

/// Move every feature carrying `swHoistAfter` behind the last feature of the step with that id (the key is dropped);
/// `ranges` follow. Features whose target step is not among `steps` stay where they are.
pub(crate) fn apply(feats: &mut Vec<Json>, ranges: &mut Vec<(usize, usize)>, steps: &[Step]) {
    let key = |f: &Json| f.get("swHoistAfter").and_then(|v| v.as_f64()).map(|v| v as i64);
    let target = |id: i64| steps.iter().position(|s| s.sw_id == id);
    let hoisted: Vec<Option<usize>> = feats.iter().map(|f| key(f).and_then(target)).collect();
    if hoisted.iter().all(|h| h.is_none()) || ranges.len() != steps.len() {
        strip(feats);
        return;
    }
    // insertion point (old index to insert after; None = before everything) per target step
    let anchor = |j: usize| -> usize {
        let (a, b) = ranges[j];
        if b > a { b - 1 } else { a.saturating_sub(1) }
    };
    let n = feats.len();
    let mut order: Vec<usize> = vec![];
    for i in 0..n {
        if hoisted[i].is_some() {
            continue;
        }
        order.push(i);
        for (h, t) in hoisted.iter().enumerate() {
            if let Some(j) = t {
                if anchor(*j) == i && (ranges[*j].1 > ranges[*j].0 || (ranges[*j].0 > 0 && i + 1 == ranges[*j].0)) {
                    order.push(h);
                }
            }
        }
    }
    // a target with no feature before it (anchor of an empty range at 0): hoist to the front
    for (h, t) in hoisted.iter().enumerate() {
        if t.is_some() && !order.contains(&h) {
            order.insert(0, h);
        }
    }
    let mut newpos = vec![0usize; n];
    for (p, &o) in order.iter().enumerate() {
        newpos[o] = p;
    }
    let mut new_ranges: Vec<(usize, usize)> = vec![];
    for (j, &(a, b)) in ranges.iter().enumerate() {
        let own: Vec<usize> = (a..b).filter(|&i| hoisted[i].is_none()).collect();
        let mut r = match (own.first(), own.last()) {
            (Some(&f), Some(&l)) => (newpos[f], newpos[l] + 1),
            _ => {
                let p = (a..n).find(|&i| hoisted[i].is_none()).map_or(order.len(), |i| newpos[i]);
                (p, p)
            }
        };
        // the hoisted features sit right behind the target step's own
        let mine: Vec<usize> = (0..n).filter(|&h| hoisted[h] == Some(j)).collect();
        if let (Some(first), Some(last)) = (mine.iter().map(|&h| newpos[h]).min(), mine.iter().map(|&h| newpos[h]).max()) {
            if own.is_empty() {
                r.0 = first;
            }
            r.1 = r.1.max(last + 1);
        }
        new_ranges.push(r);
    }
    let old = std::mem::take(feats);
    *feats = order.iter().map(|&i| old[i].clone()).collect();
    *ranges = new_ranges;
    strip(feats);
}

fn strip(feats: &mut [Json]) {
    for f in feats.iter_mut() {
        if let Json::Obj(kv) = f {
            kv.retain(|(k, _)| k != "swHoistAfter");
        }
    }
}
