//! Two consecutive fillets read back in the other order when the first one's corner patches rest on the second's
//! strips.
//!
//! A ring of concave rounds along the foot of a wall (fillet A) whose corner patches are tori about the axes of the
//! vertical rounds of the wall's corners (fillet B, next in the tree) is the foot ring of the already rounded corners:
//! the tori have major radius `R_B + r_A` and rest on B's cylinders. Read in the file's order, A runs into sharp
//! corners (mitred strips) and B then cuts its rounds through them, which is a different body. Read with B first, A's
//! tori are ordinary edge picks (the arcs where B's cylinders meet the floor) and the ring runs round the corners.
//!
//! The reordered history is kept only when the whole body verifies against the file; the two steps say so in their
//! notes.

use crate::{classify, rebuild_ordered, Class, Outcome, Tagged};
use ck_recog::model::{Kind, Model};
use ck_sldprt::meta::SwFeature;
use std::cell::Cell;

thread_local! {
    /// Set while a reordered history is read back (no second reordering inside it).
    static ACTIVE: Cell<bool> = const { Cell::new(false) };
}

/// Index pairs `(a, b)` into `t.features`: fillet `a` has a torus face next to a cylinder of fillet `b`, the next
/// feature with geometry after `a`.
fn candidates(t: &Tagged, m: &Model) -> Vec<(usize, usize)> {
    let geo: Vec<usize> = (0..t.features.len()).filter(|&i| classify(&t.features[i]) != Class::Skip).collect();
    let has = |f: ck_topo::body::FaceId, id: i64| t.tags.get(f.idx()).map_or(false, |v| v.iter().any(|x| x.0 == id));
    let mut out = vec![];
    for w in geo.windows(2) {
        let (fa, fb) = (&t.features[w[0]], &t.features[w[1]]);
        if classify(fa) != Class::Fillet || classify(fb) != Class::Fillet {
            continue;
        }
        let hit = m.faces.iter().any(|&g| {
            has(g, fa.id)
                && matches!(m.kind(g), Kind::Torus { .. })
                && m.edges(g).into_iter().any(|e| m.across(e, g).map_or(false, |h| has(h, fb.id) && !has(h, fa.id) && matches!(m.kind(h), Kind::Cyl { .. })))
        });
        if hit {
            out.push((w[0], w[1]));
        }
    }
    out
}

/// The history with the fillet pairs of [`candidates`] swapped, when it verifies (exact or close).
pub(crate) fn reorder(t: &Tagged) -> Option<Outcome> {
    if ACTIVE.with(|c| c.get()) {
        return None;
    }
    let m = Model::new(t.body);
    let pairs = candidates(t, &m);
    if pairs.is_empty() {
        return None;
    }
    let mut feats: Vec<SwFeature> = t.features.to_vec();
    for &(a, b) in pairs.iter().rev() {
        let f = feats.remove(b);
        feats.insert(a, f);
    }
    let t2 = Tagged { body: t.body, tags: t.tags.clone(), features: &feats, unit_mm: t.unit_mm, start: t.start.clone() };
    ACTIVE.with(|c| c.set(true));
    let out = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| rebuild_ordered(&t2)));
    ACTIVE.with(|c| c.set(false));
    let mut out = out.ok()?;
    if !(out.exact() || out.close()) {
        return None;
    }
    let moved: Vec<i64> = pairs.iter().flat_map(|&(a, b)| [t.features[a].id, t.features[b].id]).collect();
    for s in out.steps.iter_mut().filter(|s| moved.contains(&s.sw_id)) {
        s.note = format!("reordered (geometry-equivalent, not the file's order: the corner rounds before the foot ring that wraps them): {}", s.note);
    }
    // the steps are listed in file order (their features stay in the order they regenerate in)
    let order: std::collections::HashMap<i64, usize> = t.features.iter().enumerate().map(|(i, f)| (f.id, i)).collect();
    let mut idx: Vec<usize> = (0..out.steps.len()).collect();
    idx.sort_by_key(|&i| order.get(&out.steps[i].sw_id).copied().unwrap_or(usize::MAX));
    let (st, rg) = idx.into_iter().map(|i| (out.steps[i].clone(), out.ranges[i])).unzip();
    out.steps = st;
    out.ranges = rg;
    out.notes.push("fillets read back in a geometry-equivalent order: corner rounds before the foot ring whose tori wrap them (the whole body verifies)".into());
    Some(out)
}
