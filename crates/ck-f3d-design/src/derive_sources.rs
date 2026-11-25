// SPDX-License-Identifier: MIT
//! Where a derive's placements are when they keep no matrix (mitcad#324).
//!
//! A derive item's placements (the deriving design in the source design,
//! each entity's occurrence there; `build/derive.rs`) name an occurrence
//! path of the source design by the GUIDs its occurrences carry, and keep a
//! matrix only where one was captured *(one design of the corpora; the
//! other derives' placements are paths alone)*. Without it the placement is
//! where the source design puts that occurrence: the path's levels found in
//! the source design's occurrence tree, from its root down, their
//! placements composed. The source design's file is one of the files of
//! the documents the design refers to ([`ExternalFile`], its key the
//! source's); its design streams are decoded for the tree, nothing else.
//! The placement found goes into `transform` with `_f3d.from_source`; one
//! whose path is not in the tree keeps none, with `_f3d.why`.

use std::collections::HashMap;

use serde_json::{Value, json};

use super::inputs::ExternalFile;
use super::ir::{Detail, Dump, Mat4, OccurrenceNode};

/// A rigid 4 x 4 product `a · b`.
fn mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [[0.0; 4]; 4];
    for (r, row) in out.iter_mut().enumerate() {
        for (c, x) in row.iter_mut().enumerate() {
            *x = (0..4).map(|k| a[r][k] * b[k][c]).sum();
        }
    }
    out
}

const IDENTITY: Mat4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// The placement of the occurrence path `guids` in a source design's
/// occurrence tree (cm): each level an occurrence among the previous
/// level's children (the root's for the first) carrying that GUID.
pub fn path_placement(occurrences: &[OccurrenceNode], guids: &[String]) -> Option<Mat4> {
    let mut level: Vec<&OccurrenceNode> = occurrences.iter().collect();
    let mut at = IDENTITY;
    for (depth, guid) in guids.iter().enumerate() {
        let node = level.iter().find(|n| {
            n.f3d
                .as_ref()
                .and_then(|f| f.guids.as_ref())
                .is_some_and(|g| g.iter().any(|x| x.eq_ignore_ascii_case(guid)))
        })?;
        // Top-level occurrences carry their placement in the root, nested
        // ones in their parent component.
        let local = if depth == 0 {
            node.transform?
        } else {
            node.f3d.as_ref().and_then(|f| f.local_transform)?
        };
        at = mul(&at, &local);
        level = node.children.iter().flatten().collect();
    }
    (!guids.is_empty()).then_some(at)
}

/// The occurrence path GUIDs of a placement value.
fn path_of(placement: &Value) -> Vec<String> {
    placement
        .pointer("/occurrence/_f3d/path_guids")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|g| g.as_str().map(str::to_owned))
        .collect()
}

/// Fills a placement without a matrix from the source design's tree.
fn fill(placement: &mut Value, tree: Option<&[OccurrenceNode]>) -> bool {
    if placement.get("transform").is_some() {
        return false;
    }
    let path = path_of(placement);
    if path.is_empty() {
        return false;
    }
    let Some(tree) = tree else {
        placement["_f3d"]["why"] = json!("the source design's file was not found");
        return false;
    };
    match path_placement(tree, &path) {
        Some(m) => {
            placement["transform"] = json!(m);
            placement["_f3d"]["from_source"] = json!(true);
            true
        }
        None => {
            placement["_f3d"]["why"] = json!("its occurrence path is not in the source design");
            false
        }
    }
}

/// Fills the derives' placements that keep no matrix from their source
/// designs' occurrence trees (the files among `documents` by the source's
/// key); how many were filled.
pub fn resolve(dump: &mut Dump, documents: &[ExternalFile<'_>]) -> usize {
    let Some(items) = dump.timeline.as_mut().and_then(|t| t.items.as_mut()) else {
        return 0;
    };
    let mut trees: HashMap<String, Option<Vec<OccurrenceNode>>> = HashMap::new();
    let mut filled = 0;
    for item in items.iter_mut() {
        if item.object_type() != Some("DerivedContext") {
            continue;
        }
        let Some(Detail::Other(map)) = item.detail.as_mut() else {
            continue;
        };
        let Some(key) = map
            .get("source")
            .and_then(|s| s.pointer("/_f3d/key"))
            .and_then(Value::as_str)
            .map(str::to_owned)
        else {
            continue;
        };
        let guid = map
            .get("source")
            .and_then(|s| s.pointer("/_f3d/document_guid"))
            .and_then(Value::as_str)
            .map(str::to_lowercase);
        let tree = trees.entry(key.clone()).or_insert_with(|| {
            // By its key, else by its GUID (the files are listed once per
            // document, under the key of its first reference).
            let doc = documents
                .iter()
                .find(|d| d.key == key)
                .or_else(|| documents.iter().find(|d| Some(&d.guid) == guid.as_ref()))?;
            let decoded = super::decode_streams(doc.file, "source").ok()?;
            decoded.dump?.occurrences.map(|o| o.into_iter().collect())
        });
        let tree = tree.as_deref();
        if let Some(Value::Array(placements)) = map.get_mut("placements")
            && let Some(first) = placements.first_mut()
        {
            filled += usize::from(fill(first, tree));
        }
        if let Some(Value::Array(entities)) = map.get_mut("entities") {
            for e in entities {
                if let Some(p) = e.get_mut("placement") {
                    filled += usize::from(fill(p, tree));
                }
            }
        }
    }
    filled
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(guid: &str, transform: Mat4, nested: bool, children: Vec<Value>) -> Value {
        let mut v = json!({"children": children, "_f3d": {"guids": [guid]}});
        if nested {
            v["_f3d"]["local_transform"] = json!(transform);
        } else {
            v["transform"] = json!(transform);
        }
        v
    }

    fn shift(x: f64, y: f64, z: f64) -> Mat4 {
        [
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }

    /// A path of two levels: the top-level occurrence's placement in the
    /// root, then the nested one's in its parent; a level not there.
    #[test]
    fn a_path_is_placed_by_its_levels_in_the_source_tree() {
        let tree: Vec<OccurrenceNode> = serde_json::from_value(json!([
            node(
                "aaaaaaaa-0000-0000-0000-000000000001",
                shift(1.0, 0.0, 0.0),
                false,
                vec![node(
                    "bbbbbbbb-0000-0000-0000-000000000002",
                    shift(0.0, 2.0, 0.0),
                    true,
                    vec![]
                )]
            ),
            node(
                "cccccccc-0000-0000-0000-000000000003",
                shift(0.0, 0.0, 5.0),
                false,
                vec![]
            )
        ]))
        .unwrap();
        let path = |g: &[&str]| g.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>();
        assert_eq!(
            path_placement(
                &tree,
                &path(&[
                    "AAAAAAAA-0000-0000-0000-000000000001",
                    "bbbbbbbb-0000-0000-0000-000000000002"
                ])
            ),
            Some(shift(1.0, 2.0, 0.0))
        );
        assert_eq!(
            path_placement(&tree, &path(&["cccccccc-0000-0000-0000-000000000003"])),
            Some(shift(0.0, 0.0, 5.0))
        );
        assert_eq!(
            path_placement(&tree, &path(&["bbbbbbbb-0000-0000-0000-000000000002"])),
            None
        );
        assert_eq!(path_placement(&tree, &[]), None);
    }
}
